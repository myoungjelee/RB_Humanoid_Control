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
from scripts.sim2real.app.phases import (
    prepare_phase_runtime,
    run_m1_sensor_phase,
    run_m3_command_phase,
    run_m5_stand_phase,
)


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
    parser.add_argument("--task", choices=["flat", "rough"], default=None)
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

    # 1) device 결정
    if device_from_cli:
        selected_device = str(args_cli.device)
    else:
        selected_device = _resolve_runtime_device()

    # 2) disable_fabric 결정
    # - CLI로 명시했으면 그 값을 최우선
    # - 아니면 디바이스 기반 자동 선택(cpu면 true, cuda면 false)
    if disable_from_cli:
        selected_disable_fabric = bool(args_cli.disable_fabric)
    else:
        selected_disable_fabric = selected_device.startswith("cpu")

    # --disable_fabric 명시 시 안정성을 위해 device를 cpu로 강제
    if disable_from_cli and selected_disable_fabric and selected_device != "cpu":
        print(
            "[CONFIG] overriding --device to cpu because --disable_fabric was explicitly set",
            flush=True,
        )
        selected_device = "cpu"

    args_cli.device = selected_device
    args_cli.disable_fabric = selected_disable_fabric
    print(
        f"[CONFIG] device={args_cli.device} disable_fabric={args_cli.disable_fabric}"
        f" (device_cli={device_from_cli}, disable_cli={disable_from_cli})",
        flush=True,
    )

    config = load_sim2real_config(args_cli.config)
    default_phase = str(config.get("sim2real", {}).get("phase", "m1_sensor"))
    phase = forced_phase or args_cli.phase or default_phase

    phase_cfg = get_phase_config(config, phase)
    if phase in ("m1_sensor", "m3_command", "m5_stand"):
        phase_cfg = apply_m1_cli_overrides(phase_cfg, args_cli)
    else:
        raise ValueError(f"Unsupported phase: {phase}")

    prepare_phase_runtime(args_cli, phase_cfg)
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    exit_code = 1
    try:
        if phase == "m1_sensor":
            result = run_m1_sensor_phase(args_cli, phase_cfg, simulation_app)
        elif phase == "m3_command":
            result = run_m3_command_phase(args_cli, phase_cfg, simulation_app)
        elif phase == "m5_stand":
            result = run_m5_stand_phase(args_cli, phase_cfg, simulation_app)
        else:
            raise ValueError(f"Unsupported phase: {phase}")
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
