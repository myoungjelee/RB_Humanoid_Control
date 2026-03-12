"""
Stage 1: G1 Stand-Only Validation Env (Manager-Based)

- Base: IsaacLab G1 Velocity (Rough / Flat)
- Modification:
    * base_velocity command = 0 (stand-only)
    * stand 검증에 방해되는 외란/노이즈 비활성
    * reset 시 base pose 랜덤화 제거(원점/정면 시작)
    * RL training 금지 (rollout only)
    * Stage 1 validation 전용
"""

# ===== 원본 G1 velocity env import =====
from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.flat_env_cfg import (
    G1FlatEnvCfg,
)
from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.rough_env_cfg import (
    G1RoughEnvCfg,
)


def _apply_stand_validation_overrides(env_cfg):
    """Stage1 stand 검증 공통 오버라이드.

    의도:
    - 보행 command를 완전히 0으로 고정
    - 외란(force/push)과 관측 corruption을 꺼서 제어기 자체를 먼저 본다
    - reset 시작 자세를 고정해서 반복 실험 편차를 줄인다
    """

    # --- stand-only command 강제 ---
    env_cfg.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
    env_cfg.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
    env_cfg.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
    if hasattr(env_cfg.commands.base_velocity.ranges, "heading"):
        env_cfg.commands.base_velocity.ranges.heading = (0.0, 0.0)

    # --- 검증 방해 외란 비활성 ---
    events_cfg = getattr(env_cfg, "events", None)
    if events_cfg is not None:
        if hasattr(events_cfg, "base_external_force_torque"):
            events_cfg.base_external_force_torque = None
        if hasattr(events_cfg, "push_robot"):
            events_cfg.push_robot = None

        # reset 시작 자세를 원점/정면으로 고정해 반복성 향상
        reset_base_cfg = getattr(events_cfg, "reset_base", None)
        if reset_base_cfg is not None and hasattr(reset_base_cfg, "params") and isinstance(reset_base_cfg.params, dict):
            reset_base_cfg.params = {
                "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)},
                "velocity_range": {
                    "x": (0.0, 0.0),
                    "y": (0.0, 0.0),
                    "z": (0.0, 0.0),
                    "roll": (0.0, 0.0),
                    "pitch": (0.0, 0.0),
                    "yaw": (0.0, 0.0),
                },
            }

    # --- 관측 corruption 비활성 ---
    policy_obs_cfg = getattr(getattr(env_cfg, "observations", None), "policy", None)
    if policy_obs_cfg is not None and hasattr(policy_obs_cfg, "enable_corruption"):
        policy_obs_cfg.enable_corruption = False

# ============================================================
# Stage 1 - Rough Terrain (Stand Only)
# ============================================================


class RBStage1G1RoughStandEnvCfg(G1RoughEnvCfg):
    """
    Stage 1 전용 G1 Rough Env

    - 모든 velocity command를 0으로 고정
    - stand 안정성 검증 목적
    """

    def __post_init__(self):
        super().__post_init__()
        _apply_stand_validation_overrides(self)


# ============================================================
# Stage 1 - Flat Terrain (Stand Only)
# ============================================================


class RBStage1G1FlatStandEnvCfg(G1FlatEnvCfg):
    """
    Stage 1 전용 G1 Flat Env

    - 모든 velocity command를 0으로 고정
    - stand 안정성 검증 목적
    """

    def __post_init__(self):
        super().__post_init__()
        _apply_stand_validation_overrides(self)
