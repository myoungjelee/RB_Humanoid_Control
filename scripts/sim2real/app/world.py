"""Isaac 환경 생성 유틸.

역할:
- task short name(flat/rough/stand) -> Isaac task id 변환
- Gym/IsaacLab env 생성(베이스라인/보존 경로)
- standalone direct-spawn world 생성(ROS2 메인 트랙)
- dt/decimation/substeps 등 실행 파라미터 주입
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import yaml

TASK_ID_BY_SHORT = {
    # stand 키는 M5에서 의도를 명확히 보이기 위한 alias다.
    # 실제 task id는 Stage1 Flat Stand env를 사용한다.
    "stand": "RB-Stage1-G1-Flat-Stand-v0",
    "flat": "RB-Stage1-G1-Flat-Stand-v0",
    "rough": "RB-Stage1-G1-Rough-Stand-v0",
}

PROJECT_ROOT = Path(__file__).resolve().parents[3]


def _canonicalize_disturbance_frame_mode(mode: str) -> str:
    """외란 입력 force/torque를 어떤 기준축으로 해석할지 정규화한다."""
    value = str(mode).strip().lower()
    if value in {"identity", "body_local", "torso_local"}:
        return "identity"
    if value in {"g1_imu_link", "control_frame"}:
        return "g1_imu_link"
    return "identity"


def _remap_disturbance_wrench(
    force_xyz: tuple[float, float, float],
    torque_xyz: tuple[float, float, float],
    frame_mode: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """설정된 control frame wrench를 실제 torso local frame wrench로 변환한다.

    `g1_imu_link`는 현재 observer가 사용하는 control frame 정의와 맞춘다.
    경험적으로 torso local X 입력은 control roll로 들어가고,
    control pitch(앞뒤) 입력은 torso local Y에서 더 직접적으로 관측됐다.
    그래서 control-force [fx, fy, fz]를 torso-local [fy, fx, fz]로 치환한다.
    """
    if frame_mode != "g1_imu_link":
        return force_xyz, torque_xyz

    fx, fy, fz = force_xyz
    tx, ty, tz = torque_xyz
    mapped_force = (fy, fx, fz)
    mapped_torque = (ty, tx, tz)
    return mapped_force, mapped_torque


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

    def __init__(
        self,
        sim: Any,
        scene: Any,
        robot: Any,
        asset_source: str,
        sim_dt: float,
        fall_watch_cfg: dict[str, Any] | None = None,
        disturbance_cfg: dict[str, Any] | None = None,
    ):
        self.world = sim
        self.scene = scene
        self.robot = robot
        self.asset_source = asset_source
        self.sim_dt = float(sim_dt)
        self._step_count = 0
        self._fall_watch_cfg = dict(fall_watch_cfg or {})
        self._fall_watch_enabled = bool(self._fall_watch_cfg.get("enabled", False))
        self._fall_event_emitted = False
        self._fall_watch_min_elapsed_sec = float(self._fall_watch_cfg.get("min_elapsed_sec", 0.5))
        self._fall_watch_pelvis_z_threshold = float(self._fall_watch_cfg.get("pelvis_z_threshold", 0.45))
        self._fall_watch_torso_z_threshold = float(self._fall_watch_cfg.get("torso_z_threshold", 0.58))
        torso_name_candidates = self._fall_watch_cfg.get(
            "torso_body_name_candidates", ["torso_link", "torso", "base_link"]
        )
        self._torso_body_index, self._torso_body_name = self._resolve_body_candidate(torso_name_candidates)
        self._disturbance_cfg = dict(disturbance_cfg or {})
        self._disturbance_enabled = bool(self._disturbance_cfg.get("enabled", False))
        self._disturbance_active = False
        self._disturbance_completed = False
        self._disturbance_start_elapsed_sec = float(self._disturbance_cfg.get("start_elapsed_sec", 5.0))
        self._disturbance_start_after_control_active_sec = float(
            self._disturbance_cfg.get("start_after_control_active_sec", -1.0)
        )
        self._disturbance_duration_sec = float(self._disturbance_cfg.get("duration_sec", 0.2))
        self._disturbance_end_elapsed_sec = self._disturbance_start_elapsed_sec + self._disturbance_duration_sec
        self._disturbance_sync_marker_path = str(os.environ.get("DISTURBANCE_SYNC_MARKER", "")).strip()
        self._disturbance_control_active_elapsed_sec: float | None = None
        self._disturbance_is_global = bool(self._disturbance_cfg.get("is_global", True))
        self._disturbance_frame_mode = _canonicalize_disturbance_frame_mode(
            str(self._disturbance_cfg.get("frame_mode", "identity"))
        )
        force_xyz = self._disturbance_cfg.get("force_xyz", [0.0, 0.0, 0.0])
        torque_xyz = self._disturbance_cfg.get("torque_xyz", [0.0, 0.0, 0.0])
        self._disturbance_force_xyz = tuple(float(v) for v in force_xyz)
        self._disturbance_torque_xyz = tuple(float(v) for v in torque_xyz)
        (
            self._disturbance_applied_force_xyz,
            self._disturbance_applied_torque_xyz,
        ) = _remap_disturbance_wrench(
            self._disturbance_force_xyz,
            self._disturbance_torque_xyz,
            self._disturbance_frame_mode,
        )
        disturbance_body_candidates = self._disturbance_cfg.get(
            "body_name_candidates", ["torso_link", "torso", "base_link"]
        )
        self._disturbance_body_index, self._disturbance_body_name = self._resolve_body_candidate(
            disturbance_body_candidates
        )
        if self._fall_watch_enabled:
            torso_label = self._torso_body_name if self._torso_body_name is not None else "unresolved"
            print(
                "[CONFIG] fall_watch enabled "
                f"min_elapsed_sec={self._fall_watch_min_elapsed_sec:.2f} "
                f"pelvis_z_threshold={self._fall_watch_pelvis_z_threshold:.3f} "
                f"torso_z_threshold={self._fall_watch_torso_z_threshold:.3f} "
                f"torso_body={torso_label}",
                flush=True,
            )
        if self._disturbance_enabled:
            body_label = self._disturbance_body_name if self._disturbance_body_name is not None else "unresolved"
            print(
                "[CONFIG] disturbance enabled "
                f"start_elapsed_sec={self._disturbance_start_elapsed_sec:.2f} "
                f"start_after_control_active_sec={self._disturbance_start_after_control_active_sec:.2f} "
                f"duration_sec={self._disturbance_duration_sec:.2f} "
                f"body={body_label} "
                f"frame_mode={self._disturbance_frame_mode} "
                f"configured_force_xyz={self._disturbance_force_xyz} "
                f"configured_torque_xyz={self._disturbance_torque_xyz} "
                f"applied_force_xyz={self._disturbance_applied_force_xyz} "
                f"applied_torque_xyz={self._disturbance_applied_torque_xyz} "
                f"is_global={self._disturbance_is_global} "
                f"sync_marker={self._disturbance_sync_marker_path or 'disabled'}",
                flush=True,
            )

    def step(self, *, render: bool = True) -> None:
        """physics/render step을 1회 진행한다."""
        self._maybe_apply_disturbance()
        self.world.step(render=render)
        self.scene.update(self.sim_dt)
        self._step_count += 1

    def _resolve_body_candidate(self, name_candidates: Any) -> tuple[int | None, str | None]:
        if not name_candidates:
            return None, None
        if isinstance(name_candidates, str):
            candidate_list = [name_candidates]
        else:
            candidate_list = [str(name) for name in name_candidates]
        for candidate in candidate_list:
            body_ids, body_names = self.robot.find_bodies(candidate, preserve_order=True)
            if body_ids:
                return int(body_ids[0]), str(body_names[0])
        print(
            f"[WARN] fall_watch body candidates not found: {candidate_list}",
            flush=True,
        )
        return None, None

    def _apply_disturbance_wrench(self, *, active: bool) -> None:
        """disturbance wrench를 직접 PhysX view에 적용한다."""
        if self._disturbance_body_index is None:
            return
        body_count = len(getattr(self.robot, "body_names"))
        forces = self.robot.data.root_pos_w.new_zeros((1, body_count, 3))
        torques = self.robot.data.root_pos_w.new_zeros((1, body_count, 3))
        if active:
            fx, fy, fz = self._disturbance_applied_force_xyz
            tx, ty, tz = self._disturbance_applied_torque_xyz
            forces[:, self._disturbance_body_index, 0] = fx
            forces[:, self._disturbance_body_index, 1] = fy
            forces[:, self._disturbance_body_index, 2] = fz
            torques[:, self._disturbance_body_index, 0] = tx
            torques[:, self._disturbance_body_index, 1] = ty
            torques[:, self._disturbance_body_index, 2] = tz
        self.robot.root_physx_view.apply_forces_and_torques_at_position(
            force_data=forces.view(-1, 3),
            torque_data=torques.view(-1, 3),
            position_data=None,
            indices=self.robot._ALL_INDICES,
            is_global=self._disturbance_is_global,
        )

    def _maybe_apply_disturbance(self) -> None:
        """설정된 외란을 일정 시간 창에서만 적용한다."""
        if not self._disturbance_enabled or self._disturbance_body_index is None or self._disturbance_completed:
            return
        elapsed_sec = self._step_count * self.sim_dt
        if (
            self._disturbance_control_active_elapsed_sec is None
            and self._disturbance_sync_marker_path
            and Path(self._disturbance_sync_marker_path).exists()
        ):
            self._disturbance_control_active_elapsed_sec = elapsed_sec
            print(
                "[DISTURBANCE_SYNC] "
                f"control_active_elapsed_sec={elapsed_sec:.3f} "
                f"marker={self._disturbance_sync_marker_path}",
                flush=True,
            )

        active_window_start = self._disturbance_start_elapsed_sec
        if (
            self._disturbance_start_after_control_active_sec >= 0.0
            and self._disturbance_control_active_elapsed_sec is not None
        ):
            active_window_start = (
                self._disturbance_control_active_elapsed_sec
                + self._disturbance_start_after_control_active_sec
            )
        active_window_end = active_window_start + self._disturbance_duration_sec

        if active_window_start <= elapsed_sec < active_window_end:
            if not self._disturbance_active:
                print(
                    "[DISTURBANCE_START] "
                    f"elapsed_sec={elapsed_sec:.3f} "
                    f"window_start_sec={active_window_start:.3f} "
                    f"body={self._disturbance_body_name} "
                    f"frame_mode={self._disturbance_frame_mode} "
                    f"configured_force_xyz={self._disturbance_force_xyz} "
                    f"configured_torque_xyz={self._disturbance_torque_xyz} "
                    f"applied_force_xyz={self._disturbance_applied_force_xyz} "
                    f"applied_torque_xyz={self._disturbance_applied_torque_xyz} "
                    f"is_global={self._disturbance_is_global}",
                    flush=True,
                )
                self._disturbance_active = True
            self._apply_disturbance_wrench(active=True)
            return
        if self._disturbance_active:
            self._apply_disturbance_wrench(active=False)
            print(
                "[DISTURBANCE_END] "
                f"elapsed_sec={elapsed_sec:.3f} "
                f"window_end_sec={active_window_end:.3f} "
                f"body={self._disturbance_body_name}",
                flush=True,
            )
            self._disturbance_active = False
            self._disturbance_completed = True

    def maybe_emit_fall_event(self) -> bool:
        """pelvis/torso 높이 하강을 fall event로 간주하고 한 번만 로그를 남긴다."""
        if not self._fall_watch_enabled or self._fall_event_emitted:
            return False
        elapsed_sec = self._step_count * self.sim_dt
        if elapsed_sec < self._fall_watch_min_elapsed_sec:
            return False

        env_origin_z = 0.0
        env_origins = getattr(self.scene, "env_origins", None)
        if env_origins is not None:
            env_origin_z = float(env_origins[0, 2].item())

        pelvis_z = float(self.robot.data.root_pos_w[0, 2].item()) - env_origin_z
        torso_z = math.nan
        if self._torso_body_index is not None:
            torso_z = float(self.robot.data.body_pos_w[0, self._torso_body_index, 2].item()) - env_origin_z

        pelvis_trigger = pelvis_z <= self._fall_watch_pelvis_z_threshold
        torso_trigger = not math.isnan(torso_z) and torso_z <= self._fall_watch_torso_z_threshold
        if not pelvis_trigger and not torso_trigger:
            return False

        trigger_parts: list[str] = []
        if pelvis_trigger:
            trigger_parts.append("pelvis")
        if torso_trigger:
            torso_name = self._torso_body_name or "torso"
            trigger_parts.append(f"torso:{torso_name}")

        torso_z_text = "nan" if math.isnan(torso_z) else f"{torso_z:.3f}"
        print(
            "[FALL_EVENT] "
            f"elapsed_sec={elapsed_sec:.3f} "
            f"trigger={'+'.join(trigger_parts)} "
            f"pelvis_z={pelvis_z:.3f}/{self._fall_watch_pelvis_z_threshold:.3f} "
            f"torso_z={torso_z_text}/{self._fall_watch_torso_z_threshold:.3f}",
            flush=True,
        )
        self._fall_event_emitted = True
        return True

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


def _resolve_spawn_joint_seed_path(phase_cfg: dict[str, Any]) -> str | None:
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


def _resolve_spawn_root_height_z(phase_cfg: dict[str, Any]) -> float | None:
    """standalone direct spawn 시작 root z override를 읽는다."""
    value = phase_cfg.get("spawn_root_height_z")
    if value is None:
        return None
    return float(value)


def _resolve_pose_audit_relax_arms(phase_cfg: dict[str, Any]) -> bool:
    """pose audit에서 arm actuator stiffness/damping을 완화할지 반환."""
    return bool(phase_cfg.get("pose_audit_relax_arms", False))


def _resolve_pose_audit_arm_stiffness(phase_cfg: dict[str, Any]) -> float:
    """pose audit arm actuator stiffness override를 읽는다."""
    return float(phase_cfg.get("pose_audit_arm_stiffness", 8.0))


def _resolve_pose_audit_arm_damping(phase_cfg: dict[str, Any]) -> float:
    """pose audit arm actuator damping override를 읽는다."""
    return float(phase_cfg.get("pose_audit_arm_damping", 1.5))


def _resolve_ankle_actuator_override(phase_cfg: dict[str, Any]) -> dict[str, float] | None:
    """stand 실험에서 ankle actuator override 설정을 읽는다."""
    raw_cfg = phase_cfg.get("ankle_actuator_override")
    if not isinstance(raw_cfg, dict) or not bool(raw_cfg.get("enabled", False)):
        return None
    return {
        "stiffness": float(raw_cfg.get("stiffness", 40.0)),
        "damping": float(raw_cfg.get("damping", 4.0)),
        "effort_limit_sim": float(raw_cfg.get("effort_limit_sim", 40.0)),
    }


def _load_named_joint_seed_values(seed_path: str) -> dict[str, float]:
    """joint-name -> q_ref 형식의 YAML에서 spawn용 exact joint seed map을 읽는다."""
    with Path(seed_path).open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    named = data.get("named_seed_values")
    if not isinstance(named, dict) or not named:
        raise RuntimeError(f"named_seed_values not found in spawn joint seed YAML: {seed_path}")
    return {str(name): float(value) for name, value in named.items()}


def _resolve_startup_joint_dump_path(phase_cfg: dict[str, Any]) -> Path | None:
    """startup joint pose dump 경로를 정규화한다.

    규칙:
    - 절대경로면 그대로 사용
    - 상대경로면 `BASE` 환경변수가 있으면 그 아래로 쓴다
    - 없으면 프로젝트 루트 기준으로 쓴다
    """
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


def _dump_startup_joint_state(
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
    print(f"[CONFIG] wrote startup_joint_dump_path={dump_path}", flush=True)


def _build_direct_g1_scene_cfg(phase_cfg: dict[str, Any]) -> tuple[Any, str]:
    """direct spawn용 G1 scene cfg를 구성한다."""
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_assets.robots.unitree import G1_CFG

    robot_cfg = G1_CFG.copy()
    robot_cfg.prim_path = "{ENV_REGEX_NS}/Robot"
    if _resolve_pose_audit_relax_arms(phase_cfg):
        arms_actuator = robot_cfg.actuators.get("arms")
        if arms_actuator is not None:
            arms_actuator.stiffness = _resolve_pose_audit_arm_stiffness(phase_cfg)
            arms_actuator.damping = _resolve_pose_audit_arm_damping(phase_cfg)
            print(
                "[CONFIG] pose_audit_relax_arms=true "
                f"stiffness={arms_actuator.stiffness} damping={arms_actuator.damping}",
                flush=True,
            )
    ankle_override = _resolve_ankle_actuator_override(phase_cfg)
    if ankle_override is not None:
        feet_actuator = robot_cfg.actuators.get("feet")
        if feet_actuator is not None:
            feet_actuator.stiffness = ankle_override["stiffness"]
            feet_actuator.damping = ankle_override["damping"]
            feet_actuator.effort_limit_sim = ankle_override["effort_limit_sim"]
            print(
                "[CONFIG] ankle_actuator_override enabled "
                f"stiffness={feet_actuator.stiffness} "
                f"damping={feet_actuator.damping} "
                f"effort_limit_sim={feet_actuator.effort_limit_sim}",
                flush=True,
            )
    spawn_seed_path = _resolve_spawn_joint_seed_path(phase_cfg)
    if spawn_seed_path:
        robot_cfg.init_state.joint_pos = _load_named_joint_seed_values(spawn_seed_path)

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
    if spawn_seed_path:
        print(
            f"[CONFIG] applying spawn_joint_seed_path={spawn_seed_path}"
            f" joint_count={len(robot_cfg.init_state.joint_pos)}",
            flush=True,
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
    robot = scene["robot"]
    default_root_state = robot.data.default_root_state.clone()
    spawn_root_height_z = _resolve_spawn_root_height_z(phase_cfg)
    if spawn_root_height_z is not None:
        default_root_state[:, 2] = spawn_root_height_z
        print(f"[CONFIG] applying spawn_root_height_z={spawn_root_height_z:.3f}", flush=True)
    default_root_state[:, 0:3] += scene.env_origins
    robot.write_root_pose_to_sim(default_root_state[:, :7])
    robot.write_root_velocity_to_sim(default_root_state[:, 7:])
    default_joint_pos = robot.data.default_joint_pos.clone()
    default_joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(default_joint_pos, default_joint_vel)
    # implicit actuator targets도 동일 pose로 맞춰 spawn 직후 재수렴을 줄인다.
    robot.set_joint_position_target(default_joint_pos)
    robot.set_joint_velocity_target(default_joint_vel)
    scene.write_data_to_sim()

    startup_dump_path = _resolve_startup_joint_dump_path(phase_cfg)
    if startup_dump_path is not None:
        _dump_startup_joint_state(
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
