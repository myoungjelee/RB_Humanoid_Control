"""Sim2Real 설정 로딩/병합 유틸.

- YAML 기본 설정 로드
- phase별 설정 추출
- CLI override 병합
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "scripts" / "sim2real" / "config" / "default.yaml"

M1_OVERRIDE_KEYS = (
    "task",
    "num_envs",
    "steps",
    "disable_fabric",
    "reset_on_done",
    "sim_dt",
    "decimation",
    "substeps",
    "robot_usd_path",
    "enable_ros2_bridge",
    "bridge_extension",
)


def resolve_config_path(config_path: str | None) -> Path:
    """설정 파일 경로를 절대 경로로 정규화."""
    if not config_path:
        return DEFAULT_CONFIG_PATH
    path = Path(config_path)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def load_sim2real_config(config_path: str | None) -> dict[str, Any]:
    """설정 파일을 로드하고 최소 스키마(sim2real 루트)만 검증."""
    path = resolve_config_path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or "sim2real" not in data:
        raise ValueError(f"Invalid config format: {path}")
    return data


def get_phase_config(config: dict[str, Any], phase: str) -> dict[str, Any]:
    """전체 설정에서 특정 phase 블록만 분리한다."""
    sim2real = config.get("sim2real", {})
    phase_cfg = sim2real.get(phase)
    if not isinstance(phase_cfg, dict):
        raise ValueError(f"Phase config not found: {phase}")
    return deepcopy(phase_cfg)


def apply_m1_cli_overrides(m1_cfg: dict[str, Any], args: Any) -> dict[str, Any]:
    """CLI에 값이 들어온 항목만 YAML 설정을 덮어쓴다."""
    cfg = deepcopy(m1_cfg)
    for key in M1_OVERRIDE_KEYS:
        value = getattr(args, key, None)
        if value is not None:
            cfg[key] = value
    return cfg
