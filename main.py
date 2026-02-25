#!/usr/bin/env python3
"""Sim2Real unified orchestrator entrypoint.

학습/분석 우선 모드:
- 자동 run_dir/summary 생성 없음
- 자동 topic capture 없음
"""

from __future__ import annotations

import argparse
import traceback

from isaaclab.app import AppLauncher

from scripts.sim2real.app.config import (
    apply_m1_cli_overrides,
    get_phase_config,
    load_sim2real_config,
)
from scripts.sim2real.app.phases import prepare_m1_runtime, run_m1_sensor_phase


def _resolve_runtime_device() -> str:
    """런타임 디바이스 자동 선택.

    규칙:
    - CUDA 사용 가능하면 cuda:0
    - 아니면 cpu
    """
    try:
        import torch

        return "cuda:0" if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RB Sim2Real Orchestrator")
    parser.add_argument("--config", type=str, default=None, help="config yaml path")
    parser.add_argument("--phase", type=str, default=None, help="phase override")

    # M1 override options
    parser.add_argument("--task", choices=["flat", "rough"], default=None)
    parser.add_argument("--num_envs", type=int, default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--disable_fabric", action="store_true", default=None)
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
    # 디바이스 자동 선택: GPU 있으면 cuda:0, 없으면 cpu
    resolved_device = _resolve_runtime_device()
    if getattr(args_cli, "device", None) != resolved_device:
        print(
            f"[CONFIG] overriding --device to auto-selected value: {resolved_device}",
            flush=True,
        )
    args_cli.device = resolved_device

    config = load_sim2real_config(args_cli.config)
    default_phase = str(config.get("sim2real", {}).get("phase", "m1_sensor"))
    phase = forced_phase or args_cli.phase or default_phase

    phase_cfg = get_phase_config(config, phase)
    if phase == "m1_sensor":
        phase_cfg = apply_m1_cli_overrides(phase_cfg, args_cli)
    else:
        raise ValueError(f"Unsupported phase: {phase}")

    prepare_m1_runtime(args_cli, phase_cfg)
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    exit_code = 1
    try:
        if phase == "m1_sensor":
            result = run_m1_sensor_phase(args_cli, phase_cfg, simulation_app)
            print(f"[RESULT] task_id={result.get('task_id')}", flush=True)
            print(f"[RESULT] status={result.get('status')}", flush=True)
            print(f"[RESULT] executed_steps={result.get('executed_steps')}", flush=True)
            print(f"[RESULT] elapsed_sec={result.get('elapsed_sec')}", flush=True)
            print(f"[RESULT] graph_built={result.get('graph_built')}", flush=True)
            print(f"[RESULT] graph_note={result.get('graph_note')}", flush=True)
            status = str(result.get("status", "failed"))
            exit_code = 0 if status in ("completed_target_steps", "stopped_early", "stopped_no_step_limit") else 1
    except Exception as exc:
        print(f"[ERROR] exception: {type(exc).__name__}: {exc}", flush=True)
        print(traceback.format_exc(), flush=True)
        exit_code = 1
    finally:
        simulation_app.close()

    return exit_code


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
