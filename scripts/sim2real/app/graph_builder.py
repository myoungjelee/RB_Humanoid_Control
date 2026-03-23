"""Facade for Isaac ROS bridge graph builders."""

from __future__ import annotations

from typing import Any

from .sensor_graph_builder import build_sensor_graph


def build_m1_sensor_graph(env: Any, phase_cfg: dict[str, Any]) -> tuple[bool, str]:
    """M1 최소 센서 그래프 생성."""
    return build_sensor_graph(env=env, phase_cfg=phase_cfg, include_command_apply=False)


def build_m3_command_graph(env: Any, phase_cfg: dict[str, Any]) -> tuple[bool, str]:
    """M3 명령 적용 그래프 생성(M1 센서 + command apply)."""
    return build_sensor_graph(env=env, phase_cfg=phase_cfg, include_command_apply=True)
