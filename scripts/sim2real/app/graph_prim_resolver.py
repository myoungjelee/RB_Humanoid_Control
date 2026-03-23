"""USD prim resolution helpers for Isaac ROS bridge graphs."""

from __future__ import annotations

import re
from typing import Any


def normalize_prim_path(path: str) -> str:
    """env regex prim path를 단일 env 기준(env_0) concrete path로 변환."""
    out = path
    out = out.replace("env_.*/", "env_0/")
    out = out.replace(".*", "0")
    out = re.sub(r"/+", "/", out)
    if not out.startswith("/"):
        out = "/" + out
    return out


def stage_has_prim(path: str) -> bool:
    """현재 USD stage에 prim이 존재하는지 확인."""
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False
        prim = stage.GetPrimAtPath(path)
        return bool(prim and prim.IsValid())
    except Exception:
        return False


def find_descendant_prim(root_path: str, prim_name: str) -> str | None:
    """root_path 하위에서 basename이 prim_name인 prim을 찾는다."""
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None

        root_prefix = root_path.rstrip("/") + "/"
        for prim in stage.TraverseAll():
            path = str(prim.GetPath())
            if not path.startswith(root_prefix):
                continue
            if path.rsplit("/", 1)[-1] == prim_name:
                return path
    except Exception:
        return None
    return None


def resolve_robot_prim(env: Any, override: str | None) -> str:
    """robot prim 경로를 자동 탐색하거나 override 값을 사용한다."""
    normalized_override = normalize_prim_path(override) if override else None
    if normalized_override and stage_has_prim(normalized_override):
        return normalized_override

    scene = getattr(getattr(env, "unwrapped", env), "scene", None)
    if scene is None:
        if normalized_override:
            return normalized_override
        raise RuntimeError("Failed to resolve robot prim: env.scene missing")

    entity = None
    for key in ("robot", "humanoid", "g1"):
        try:
            entity = scene[key]
            break
        except Exception:
            candidate = getattr(scene, key, None)
            if candidate is not None:
                entity = candidate
                break

    if entity is None:
        raise RuntimeError("Failed to resolve robot prim: robot entity not found")

    candidates: list[str] = []
    for attr in ("prim_path", "_prim_path"):
        value = getattr(entity, attr, None)
        if isinstance(value, str) and value:
            candidates.append(value)

    cfg = getattr(entity, "cfg", None)
    cfg_prim = getattr(cfg, "prim_path", None)
    if isinstance(cfg_prim, str) and cfg_prim:
        candidates.append(cfg_prim)

    prim_paths = getattr(entity, "prim_paths", None)
    if isinstance(prim_paths, (list, tuple)) and prim_paths:
        first = prim_paths[0]
        if isinstance(first, str) and first:
            candidates.append(first)

    if not candidates:
        raise RuntimeError("Failed to resolve robot prim: no prim path candidate")

    normalized = [normalize_prim_path(path) for path in candidates]
    if normalized_override and normalized_override not in normalized:
        normalized.insert(0, normalized_override)

    expanded: list[str] = []
    for path in normalized:
        if path.endswith("/Robot"):
            for suffix in ("/pelvis", "/base_link"):
                alt = f"{path}{suffix}"
                if alt not in expanded:
                    expanded.append(alt)
        if path not in expanded:
            expanded.append(path)

    for path in expanded:
        if stage_has_prim(path):
            return path
    return expanded[0]


def resolve_imu_prim(robot_prim: str, override: str | None) -> tuple[str, str]:
    """IMU source prim을 찾고, frame_id 기본값도 함께 반환한다."""
    normalized_override = normalize_prim_path(override) if override else None
    if normalized_override and stage_has_prim(normalized_override):
        return normalized_override, normalized_override.rsplit("/", 1)[-1]

    search_root = robot_prim.rsplit("/", 1)[0] if "/" in robot_prim.rstrip("/") else robot_prim
    for prim_name in ("imu_link", "torso_link", robot_prim.rsplit("/", 1)[-1]):
        candidate = find_descendant_prim(search_root, prim_name)
        if candidate:
            return candidate, prim_name

    return robot_prim, robot_prim.rsplit("/", 1)[-1]
