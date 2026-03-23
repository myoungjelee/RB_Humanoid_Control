"""Phase-specific Isaac rollout handlers.

Each handler keeps the existing `(args_cli, phase_cfg, simulation_app)` contract,
so parameter injection via `phase_cfg` stays unchanged.
"""

from __future__ import annotations

from typing import Any

from .graph_builder import build_m1_sensor_graph, build_m3_command_graph
from .phase_runtime import run_rollout_phase


def run_m1_sensor_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M1 phase 실행(/clock, /rb/joint_states, /rb/imu publish 확인)."""
    return run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=build_m1_sensor_graph,
    )


def run_m3_command_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M3 phase 실행(/rb/command_raw -> articulation 적용 검증)."""
    return run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=build_m3_command_graph,
    )


def run_m5_stand_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M5 phase 실행(/rb/command_safe -> articulation 적용 확인)."""
    return run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=build_m3_command_graph,
    )


def run_m8_disturb_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M8 phase 실행(standing + single disturbance 적용 비교)."""
    return run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=build_m3_command_graph,
    )


def run_m5_pose_audit_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M5 pose audit 실행(paused GUI에서 nominal pose를 확인)."""
    return run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=None,
    )
