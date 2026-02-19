"""
Stage 1: G1 Stand-Only Validation Env (Manager-Based)

- Base: IsaacLab G1 Velocity (Rough / Flat)
- Modification:
    * base_velocity command = 0 (stand-only)
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

        # --- stand-only 강제 ---
        # lin_vel_x 고정
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)

        # lin_vel_y 고정
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)

        # yaw rate 고정
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        # heading command가 존재하면 고정
        if hasattr(self.commands.base_velocity.ranges, "heading"):
            self.commands.base_velocity.ranges.heading = (0.0, 0.0)


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

        # --- stand-only 강제 ---
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        if hasattr(self.commands.base_velocity.ranges, "heading"):
            self.commands.base_velocity.ranges.heading = (0.0, 0.0)
