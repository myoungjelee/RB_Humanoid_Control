"""IsaacLab 환경(Env) 생성 유틸.

역할:
- task short name(flat/rough) -> Isaac task id 변환
- parse_env_cfg로 EnvCfg 생성
- dt/decimation/substeps 등 실행 파라미터 주입
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

TASK_ID_BY_SHORT = {
    "flat": "RB-Stage1-G1-Flat-Stand-v0",
    "rough": "RB-Stage1-G1-Rough-Stand-v0",
}


def resolve_task_id(task_short: str) -> str:
    """사용자 친화 키(flat/rough)를 실제 gym task id로 변환."""
    if task_short not in TASK_ID_BY_SHORT:
        raise ValueError(f"Unsupported task key: {task_short}")
    return TASK_ID_BY_SHORT[task_short]


def create_env(args_cli: Any, phase_cfg: dict[str, Any]):
    """M1 실행용 IsaacLab env를 생성해서 반환한다."""
    import gymnasium as gym

    # task 등록 side-effect import (gym.make에서 task id 인식용)
    import RB_Humanoid_Control.tasks  # noqa: F401
    from isaaclab_tasks.utils import parse_env_cfg

    task_id = resolve_task_id(str(phase_cfg["task"]))
    env_cfg = parse_env_cfg(
        task_id,
        device=args_cli.device,
        num_envs=int(phase_cfg["num_envs"]),
        use_fabric=not bool(phase_cfg.get("disable_fabric", False)),
    )

    # 로컬 USD 경로를 지정하면 기본 Nucleus 경로 대신 강제 사용한다.
    # 예: /home/leemou/Projects/RB_Humanoid_Control/sim/isaac_scenes/g1_local.usd
    robot_usd_path = phase_cfg.get("robot_usd_path")
    if robot_usd_path:
        usd_path = str(Path(str(robot_usd_path)).expanduser().resolve())
        if not Path(usd_path).exists():
            raise FileNotFoundError(f"robot_usd_path not found: {usd_path}")
        if not hasattr(env_cfg, "scene") or not hasattr(env_cfg.scene, "robot"):
            raise RuntimeError("env_cfg.scene.robot not found, cannot apply robot_usd_path")
        if not hasattr(env_cfg.scene.robot, "spawn") or not hasattr(env_cfg.scene.robot.spawn, "usd_path"):
            raise RuntimeError("env_cfg.scene.robot.spawn.usd_path not found, cannot apply robot_usd_path")
        env_cfg.scene.robot.spawn.usd_path = usd_path

    # Sim2Real에서 고정하기로 한 타이밍 파라미터를 cfg에 반영
    if hasattr(env_cfg, "decimation"):
        env_cfg.decimation = int(phase_cfg["decimation"])
    if hasattr(env_cfg, "sim") and hasattr(env_cfg.sim, "dt"):
        env_cfg.sim.dt = float(phase_cfg["sim_dt"])
    if hasattr(env_cfg, "sim") and hasattr(env_cfg.sim, "substeps"):
        env_cfg.sim.substeps = int(phase_cfg["substeps"])

    env = gym.make(task_id, cfg=env_cfg)
    resolved_usd_path = None
    if (
        hasattr(env_cfg, "scene")
        and hasattr(env_cfg.scene, "robot")
        and hasattr(env_cfg.scene.robot, "spawn")
        and hasattr(env_cfg.scene.robot.spawn, "usd_path")
    ):
        resolved_usd_path = str(env_cfg.scene.robot.spawn.usd_path)
    return task_id, env, resolved_usd_path
