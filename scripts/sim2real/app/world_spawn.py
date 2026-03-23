"""Standalone Isaac world spawn/config helpers."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

from .telemetry import log_config

PROJECT_ROOT = Path(__file__).resolve().parents[3]


def resolve_robot_usd_path(phase_cfg: dict[str, Any]) -> str | None:
    """standalone direct spawn에 쓸 로컬 robot USD override를 정규화한다."""
    robot_usd_path = phase_cfg.get("robot_usd_path")
    if not robot_usd_path:
        return None
    usd_path = Path(str(robot_usd_path)).expanduser()
    if not usd_path.is_absolute():
        usd_path = PROJECT_ROOT / usd_path
    usd_path = usd_path.resolve()
    if not usd_path.exists():
        raise FileNotFoundError(f"robot_usd_path not found: {usd_path}")
    return str(usd_path)


def resolve_spawn_joint_seed_path(phase_cfg: dict[str, Any]) -> str | None:
    """standalone direct spawn에 쓸 spawn joint seed YAML 경로를 정규화한다."""
    seed_path = phase_cfg.get("spawn_joint_seed_path")
    if not seed_path:
        return None
    path = Path(str(seed_path)).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    path = path.resolve()
    if not path.exists():
        raise FileNotFoundError(f"spawn_joint_seed_path not found: {path}")
    return str(path)


def resolve_spawn_root_height_z(phase_cfg: dict[str, Any]) -> float | None:
    """standalone direct spawn 시작 root z override를 읽는다."""
    value = phase_cfg.get("spawn_root_height_z")
    if value is None:
        return None
    return float(value)


def resolve_pose_audit_relax_arms(phase_cfg: dict[str, Any]) -> bool:
    """pose audit에서 arm actuator stiffness/damping을 완화할지 반환."""
    return bool(phase_cfg.get("pose_audit_relax_arms", False))


def resolve_pose_audit_arm_stiffness(phase_cfg: dict[str, Any]) -> float:
    """pose audit arm actuator stiffness override를 읽는다."""
    return float(phase_cfg.get("pose_audit_arm_stiffness", 8.0))


def resolve_pose_audit_arm_damping(phase_cfg: dict[str, Any]) -> float:
    """pose audit arm actuator damping override를 읽는다."""
    return float(phase_cfg.get("pose_audit_arm_damping", 1.5))


def resolve_ankle_actuator_override(phase_cfg: dict[str, Any]) -> dict[str, float] | None:
    """stand 실험에서 ankle actuator override 설정을 읽는다."""
    raw_cfg = phase_cfg.get("ankle_actuator_override")
    if not isinstance(raw_cfg, dict) or not bool(raw_cfg.get("enabled", False)):
        return None
    return {
        "stiffness": float(raw_cfg.get("stiffness", 40.0)),
        "damping": float(raw_cfg.get("damping", 4.0)),
        "effort_limit_sim": float(raw_cfg.get("effort_limit_sim", 40.0)),
    }


def load_named_joint_seed_values(seed_path: str) -> dict[str, float]:
    """joint-name -> q_ref 형식의 YAML에서 spawn용 exact joint seed map을 읽는다."""
    with Path(seed_path).open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    named = data.get("named_seed_values")
    if not isinstance(named, dict) or not named:
        raise RuntimeError(f"named_seed_values not found in spawn joint seed YAML: {seed_path}")
    return {str(name): float(value) for name, value in named.items()}


def resolve_startup_joint_dump_path(phase_cfg: dict[str, Any]) -> Path | None:
    """startup joint pose dump 경로를 정규화한다."""
    dump_path = phase_cfg.get("startup_joint_dump_path")
    if not dump_path:
        return None
    path = Path(str(dump_path)).expanduser()
    if path.is_absolute():
        return path
    base = os.environ.get("BASE")
    if base:
        return (Path(base).expanduser() / path).resolve()
    return (PROJECT_ROOT / path).resolve()


def dump_startup_joint_state(
    *,
    robot: Any,
    default_root_state: Any,
    default_joint_pos: Any,
    default_joint_vel: Any,
    dump_path: Path,
) -> None:
    """env_0 startup pose를 YAML로 저장한다."""
    joint_names = list(getattr(robot, "joint_names"))
    joint_pos_env0 = default_joint_pos[0].detach().cpu().tolist()
    joint_vel_env0 = default_joint_vel[0].detach().cpu().tolist()
    root_state_env0 = default_root_state[0].detach().cpu().tolist()

    payload = {
        "joint_count": len(joint_names),
        "joint_order": joint_names,
        "startup_joint_pos": [float(v) for v in joint_pos_env0],
        "startup_joint_vel": [float(v) for v in joint_vel_env0],
        "named_joint_pos": {name: float(val) for name, val in zip(joint_names, joint_pos_env0)},
        "named_joint_vel": {name: float(val) for name, val in zip(joint_names, joint_vel_env0)},
        "startup_root_pos_xyz": [float(v) for v in root_state_env0[0:3]],
        "startup_root_quat_wxyz": [float(v) for v in root_state_env0[3:7]],
        "startup_root_linvel_xyz": [float(v) for v in root_state_env0[7:10]],
        "startup_root_angvel_xyz": [float(v) for v in root_state_env0[10:13]],
        "note": "Captured from robot.data.default_* immediately after standalone direct-spawn reset.",
    }
    dump_path.parent.mkdir(parents=True, exist_ok=True)
    dump_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    log_config(f"wrote startup_joint_dump_path={dump_path}")


def build_direct_g1_scene_cfg(phase_cfg: dict[str, Any]) -> tuple[Any, str]:
    """direct spawn용 G1 scene cfg를 구성한다."""
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_assets.robots.unitree import G1_CFG

    robot_cfg = G1_CFG.copy()
    robot_cfg.prim_path = "{ENV_REGEX_NS}/Robot"
    if resolve_pose_audit_relax_arms(phase_cfg):
        arms_actuator = robot_cfg.actuators.get("arms")
        if arms_actuator is not None:
            arms_actuator.stiffness = resolve_pose_audit_arm_stiffness(phase_cfg)
            arms_actuator.damping = resolve_pose_audit_arm_damping(phase_cfg)
            log_config(
                "pose_audit_relax_arms=true "
                f"stiffness={arms_actuator.stiffness} damping={arms_actuator.damping}"
            )
    ankle_override = resolve_ankle_actuator_override(phase_cfg)
    if ankle_override is not None:
        feet_actuator = robot_cfg.actuators.get("feet")
        if feet_actuator is not None:
            feet_actuator.stiffness = ankle_override["stiffness"]
            feet_actuator.damping = ankle_override["damping"]
            feet_actuator.effort_limit_sim = ankle_override["effort_limit_sim"]
            log_config(
                "ankle_actuator_override enabled "
                f"stiffness={feet_actuator.stiffness} "
                f"damping={feet_actuator.damping} "
                f"effort_limit_sim={feet_actuator.effort_limit_sim}"
            )
    spawn_seed_path = resolve_spawn_joint_seed_path(phase_cfg)
    if spawn_seed_path:
        robot_cfg.init_state.joint_pos = load_named_joint_seed_values(spawn_seed_path)

    robot_usd_path = resolve_robot_usd_path(phase_cfg)
    if robot_usd_path:
        robot_cfg.spawn.usd_path = robot_usd_path

    @configclass
    class StandaloneG1SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(
            prim_path="/World/ground",
            spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
        )
        robot = robot_cfg
        light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2000.0),
        )

    scene_cfg = StandaloneG1SceneCfg(
        num_envs=int(phase_cfg["num_envs"]),
        env_spacing=float(phase_cfg.get("env_spacing", 2.0)),
    )
    if spawn_seed_path:
        log_config(
            f"applying spawn_joint_seed_path={spawn_seed_path}"
            f" joint_count={len(robot_cfg.init_state.joint_pos)}"
        )
    return scene_cfg, str(robot_cfg.spawn.usd_path)
