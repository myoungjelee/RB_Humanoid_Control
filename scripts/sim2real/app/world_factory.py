"""World/env factory functions for Sim2Real phases."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .telemetry import log_config
from .world_runtime import StandaloneDirectWorld
from .world_spawn import (
    build_direct_g1_scene_cfg,
    dump_startup_joint_state,
    resolve_spawn_root_height_z,
    resolve_startup_joint_dump_path,
)

TASK_ID_BY_SHORT = {
    "stand": "RB-Stage1-G1-Flat-Stand-v0",
    "flat": "RB-Stage1-G1-Flat-Stand-v0",
    "rough": "RB-Stage1-G1-Rough-Stand-v0",
}


def resolve_task_id(task_short: str) -> str:
    """사용자 친화 키(flat/rough/stand)를 실제 gym task id로 변환."""
    if task_short not in TASK_ID_BY_SHORT:
        raise ValueError(f"Unsupported task key: {task_short}")
    return TASK_ID_BY_SHORT[task_short]


def create_env(args_cli: Any, phase_cfg: dict[str, Any]):
    """M1 실행용 IsaacLab env를 생성해서 반환한다."""
    import gymnasium as gym
    import RB_Humanoid_Control.tasks  # noqa: F401
    from isaaclab_tasks.utils import parse_env_cfg

    task_id = resolve_task_id(str(phase_cfg["task"]))
    env_cfg = parse_env_cfg(
        task_id,
        device=args_cli.device,
        num_envs=int(phase_cfg["num_envs"]),
        use_fabric=not bool(phase_cfg.get("disable_fabric", False)),
    )

    scene_cfg = getattr(env_cfg, "scene", None)
    robot_cfg = getattr(scene_cfg, "robot", None)
    robot_spawn_cfg = getattr(robot_cfg, "spawn", None)
    sim_cfg = getattr(env_cfg, "sim", None)

    robot_usd_path = phase_cfg.get("robot_usd_path")
    if robot_usd_path:
        usd_path = str(Path(str(robot_usd_path)).expanduser().resolve())
        if not Path(usd_path).exists():
            raise FileNotFoundError(f"robot_usd_path not found: {usd_path}")
        if robot_cfg is None:
            raise RuntimeError("env_cfg.scene.robot not found, cannot apply robot_usd_path")
        if robot_spawn_cfg is None or not hasattr(robot_spawn_cfg, "usd_path"):
            raise RuntimeError("env_cfg.scene.robot.spawn.usd_path not found, cannot apply robot_usd_path")
        setattr(robot_spawn_cfg, "usd_path", usd_path)

    if hasattr(env_cfg, "decimation"):
        env_cfg.decimation = int(phase_cfg["decimation"])
    if sim_cfg is not None and hasattr(sim_cfg, "dt"):
        setattr(sim_cfg, "dt", float(phase_cfg["sim_dt"]))
    if sim_cfg is not None and hasattr(sim_cfg, "substeps"):
        setattr(sim_cfg, "substeps", int(phase_cfg["substeps"]))

    env = gym.make(task_id, cfg=env_cfg)
    resolved_usd_path = None
    if robot_spawn_cfg is not None and hasattr(robot_spawn_cfg, "usd_path"):
        resolved_usd_path = str(getattr(robot_spawn_cfg, "usd_path"))
    return task_id, env, resolved_usd_path


def create_standalone_world(args_cli: Any, phase_cfg: dict[str, Any]):
    """ROS2 메인 트랙용 standalone direct-spawn world를 생성해서 반환한다."""
    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene

    task_id = resolve_task_id(str(phase_cfg["task"]))
    legacy_stage_usd_path = phase_cfg.get("stage_usd_path")
    if legacy_stage_usd_path:
        log_config(
            f"ignoring legacy stage_usd_path={legacy_stage_usd_path}"
            " because standalone backend now uses direct G1 spawn"
        )

    sim_cfg = sim_utils.SimulationCfg(
        device=str(args_cli.device),
        dt=float(phase_cfg["sim_dt"]),
        render_interval=1,
        use_fabric=not bool(phase_cfg.get("disable_fabric", False)),
    )
    sim = sim_utils.SimulationContext(sim_cfg)
    scene_cfg, asset_source = build_direct_g1_scene_cfg(phase_cfg)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    robot = scene["robot"]
    default_root_state = robot.data.default_root_state.clone()
    spawn_root_height_z = resolve_spawn_root_height_z(phase_cfg)
    if spawn_root_height_z is not None:
        default_root_state[:, 2] = spawn_root_height_z
        log_config(f"applying spawn_root_height_z={spawn_root_height_z:.3f}")
    default_root_state[:, 0:3] += scene.env_origins
    robot.write_root_pose_to_sim(default_root_state[:, :7])
    robot.write_root_velocity_to_sim(default_root_state[:, 7:])
    default_joint_pos = robot.data.default_joint_pos.clone()
    default_joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(default_joint_pos, default_joint_vel)
    robot.set_joint_position_target(default_joint_pos)
    robot.set_joint_velocity_target(default_joint_vel)
    scene.write_data_to_sim()

    startup_dump_path = resolve_startup_joint_dump_path(phase_cfg)
    if startup_dump_path is not None:
        dump_startup_joint_state(
            robot=robot,
            default_root_state=default_root_state,
            default_joint_pos=default_joint_pos,
            default_joint_vel=default_joint_vel,
            dump_path=startup_dump_path,
        )

    return (
        task_id,
        StandaloneDirectWorld(
            sim=sim,
            scene=scene,
            robot=robot,
            asset_source=asset_source,
            sim_dt=float(phase_cfg["sim_dt"]),
            fall_watch_cfg=phase_cfg.get("fall_watch"),
            disturbance_cfg=phase_cfg.get("disturbance"),
        ),
        asset_source,
    )
