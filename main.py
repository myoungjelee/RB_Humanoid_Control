#!/usr/bin/env python3
"""Sim2Real unified orchestrator entrypoint.

학습/분석 우선 모드:
- 자동 run_dir/summary 생성 없음
- 자동 topic capture 없음
"""

from __future__ import annotations

import argparse
import sys
import traceback

from isaaclab.app import AppLauncher

from scripts.sim2real.app.config import (
    apply_m1_cli_overrides,
    get_phase_config,
    load_sim2real_config,
)
from scripts.sim2real.app.phase_registry import SUPPORTED_PHASES, run_phase
from scripts.sim2real.app.phase_runtime import prepare_phase_runtime
from scripts.sim2real.app.telemetry import log_config, log_exception, log_result


def _resolve_runtime_device() -> str:
    """런타임 기본 디바이스 선택.

    규칙:
    - Sim2Real 제어 트랙은 CPU를 기본값으로 사용
    - GPU를 쓰려면 --device cuda:0 를 명시
    """
    return "cpu"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RB Sim2Real Orchestrator")
    parser.add_argument("--config", type=str, default=None, help="config yaml path")
    parser.add_argument("--phase", type=str, default=None, help="phase override")

    # M1 override options
    parser.add_argument("--task", choices=["flat", "rough", "stand"], default=None)
    parser.add_argument("--num_envs", type=int, default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--disable_fabric", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--reset_on_done", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--sim_dt", type=float, default=None)
    parser.add_argument("--decimation", type=int, default=None)
    parser.add_argument("--substeps", type=int, default=None)
    parser.add_argument("--robot_usd_path", type=str, default=None)
    parser.add_argument("--enable_ros2_bridge", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--bridge_extension", type=str, default=None)

    AppLauncher.add_app_launcher_args(parser)
    return parser


def run(argv: list[str] | None = None, forced_phase: str | None = None) -> int:
    parser = _build_parser()
    args_cli = parser.parse_args(argv)
    raw_argv = list(argv) if argv is not None else sys.argv[1:]
    device_from_cli = "--device" in raw_argv
    disable_from_cli = ("--disable_fabric" in raw_argv) or ("--no-disable_fabric" in raw_argv)

    if device_from_cli:
        selected_device = str(args_cli.device)
    else:
        selected_device = _resolve_runtime_device()

    if disable_from_cli:
        selected_disable_fabric = bool(args_cli.disable_fabric)
    else:
        selected_disable_fabric = selected_device.startswith("cpu")

    if disable_from_cli and selected_disable_fabric and selected_device != "cpu":
        log_config("overriding --device to cpu because --disable_fabric was explicitly set")
        selected_device = "cpu"

    args_cli.device = selected_device
    args_cli.disable_fabric = selected_disable_fabric
    log_config(
        f"device={args_cli.device} disable_fabric={args_cli.disable_fabric}"
        f" (device_cli={device_from_cli}, disable_cli={disable_from_cli})"
    )

    config = load_sim2real_config(args_cli.config)
    default_phase = str(config.get("sim2real", {}).get("phase", "m1_sensor"))
    phase = forced_phase or args_cli.phase or default_phase

    phase_cfg = get_phase_config(config, phase)
    if phase not in SUPPORTED_PHASES:
        raise ValueError(f"Unsupported phase: {phase}")
    phase_cfg = apply_m1_cli_overrides(phase_cfg, args_cli)

    prepare_phase_runtime(args_cli, phase_cfg)
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    exit_code = 1
    try:
        result = run_phase(
            phase,
            args_cli=args_cli,
            phase_cfg=phase_cfg,
            simulation_app=simulation_app,
        )
        for key in ("task_id", "status", "executed_steps", "elapsed_sec", "graph_built", "graph_note"):
            log_result(key, result.get(key))
        status = str(result.get("status", "failed"))
        exit_code = 0 if status in (
            "completed_target_steps",
            "stopped_early",
            "stopped_no_step_limit",
            "stopped_on_fall_event",
        ) else 1
    except Exception as exc:
        log_exception(exc, traceback.format_exc())
        exit_code = 1
    finally:
        simulation_app.close()

    return exit_code


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
