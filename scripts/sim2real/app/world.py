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
    import gymnasium as gym  # noqa: I001  # Isaac runtime order/lazy import

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

    scene_cfg = getattr(env_cfg, "scene", None)
    robot_cfg = getattr(scene_cfg, "robot", None)
    robot_spawn_cfg = getattr(robot_cfg, "spawn", None)
    sim_cfg = getattr(env_cfg, "sim", None)

    # 로컬 USD 경로를 지정하면 기본 Nucleus 경로 대신 강제 사용한다.
    # 예: /home/leemou/Projects/RB_Humanoid_Control/sim/isaac_scenes/g1_local.usd
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

    # Sim2Real에서 고정하기로 한 타이밍 파라미터를 cfg에 반영
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
