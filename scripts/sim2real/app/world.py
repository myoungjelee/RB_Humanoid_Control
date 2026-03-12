"""Isaac 환경 생성 유틸.

역할:
- task short name(flat/rough/stand) -> Isaac task id 변환
- Gym/IsaacLab env 생성(베이스라인/보존 경로)
- standalone direct-spawn world 생성(ROS2 메인 트랙)
- dt/decimation/substeps 등 실행 파라미터 주입
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

TASK_ID_BY_SHORT = {
    # stand 키는 M5에서 의도를 명확히 보이기 위한 alias다.
    # 실제 task id는 Stage1 Flat Stand env를 사용한다.
    "stand": "RB-Stage1-G1-Flat-Stand-v0",
    "flat": "RB-Stage1-G1-Flat-Stand-v0",
    "rough": "RB-Stage1-G1-Rough-Stand-v0",
}

PROJECT_ROOT = Path(__file__).resolve().parents[3]


def resolve_task_id(task_short: str) -> str:
    """사용자 친화 키(flat/rough/stand)를 실제 gym task id로 변환."""
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


class StandaloneDirectWorld:
    """IsaacLab direct spawn 기반 standalone simulation 래퍼."""

    def __init__(self, sim: Any, scene: Any, asset_source: str):
        self.world = sim
        self.scene = scene
        self.asset_source = asset_source

    def step(self, *, render: bool = True) -> None:
        """physics/render step을 1회 진행한다."""
        self.world.step(render=render)

    def close(self) -> None:
        """명시적으로 정리할 리소스가 있으면 정리한다."""
        stop = getattr(self.world, "stop", None)
        if callable(stop):
            stop()


def _resolve_robot_usd_path(phase_cfg: dict[str, Any]) -> str | None:
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


def _build_direct_g1_scene_cfg(phase_cfg: dict[str, Any]) -> tuple[Any, str]:
    """direct spawn용 G1 scene cfg를 구성한다."""
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_assets.robots.unitree import G1_CFG

    robot_cfg = G1_CFG.copy()
    robot_cfg.prim_path = "{ENV_REGEX_NS}/Robot"

    robot_usd_path = _resolve_robot_usd_path(phase_cfg)
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
    return scene_cfg, str(robot_cfg.spawn.usd_path)


def create_standalone_world(args_cli: Any, phase_cfg: dict[str, Any]):
    """ROS2 메인 트랙용 standalone direct-spawn world를 생성해서 반환한다."""
    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene

    task_id = resolve_task_id(str(phase_cfg["task"]))
    legacy_stage_usd_path = phase_cfg.get("stage_usd_path")
    if legacy_stage_usd_path:
        print(
            f"[CONFIG] ignoring legacy stage_usd_path={legacy_stage_usd_path}"
            " because standalone backend now uses direct G1 spawn",
            flush=True,
        )

    sim_cfg = sim_utils.SimulationCfg(
        device=str(args_cli.device),
        dt=float(phase_cfg["sim_dt"]),
        render_interval=1,
        use_fabric=not bool(phase_cfg.get("disable_fabric", False)),
    )
    sim = sim_utils.SimulationContext(sim_cfg)
    scene_cfg, asset_source = _build_direct_g1_scene_cfg(phase_cfg)
    scene = InteractiveScene(scene_cfg)
    sim.reset()

    return task_id, StandaloneDirectWorld(sim=sim, scene=scene, asset_source=asset_source), asset_source
