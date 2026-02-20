"""실험 액션 생성 유틸리티.

- 공통 루프와 분리해 stage별 액션 전략을 교체하기 쉽게 유지
- `pose` 모드는 먼저 로봇 현재 joint_pos를 사용하고, 불가하면 obs 슬라이스로 fallback
"""

from __future__ import annotations

from typing import Any


def extract_policy_obs(obs: Any) -> Any:
    """관측이 dict면 `policy` 키를 우선 사용하고, 아니면 원본 관측을 반환한다."""
    if isinstance(obs, dict) and "policy" in obs:
        return obs["policy"]
    return obs


def _get_robot_from_scene(scene: Any) -> Any | None:
    for key in ("robot", "humanoid", "g1"):
        try:
            return scene[key]
        except Exception:
            candidate = getattr(scene, key, None)
            if candidate is not None:
                return candidate
    return None


def _pose_actions_from_joint_pos(env: Any, device: Any, action_shape: tuple[int, ...]) -> Any | None:
    if env is None:
        return None

    unwrapped = getattr(env, "unwrapped", env)
    scene = getattr(unwrapped, "scene", None)
    if scene is None:
        return None

    robot = _get_robot_from_scene(scene)
    if robot is None:
        return None

    data = getattr(robot, "data", None)
    if data is None:
        return None

    joint_pos = getattr(data, "joint_pos", None)
    if joint_pos is None or not hasattr(joint_pos, "shape"):
        return None

    action_dim = action_shape[-1]
    if joint_pos.shape[-1] < action_dim:
        raise RuntimeError(
            f"Pose mode needs joint_pos dim >= {action_dim}, got {joint_pos.shape[-1]}."
        )
    return joint_pos[:, :action_dim].to(device).clone()


def make_actions(
    mode: str,
    obs: Any,
    device: Any,
    action_shape: tuple[int, ...],
    env: Any | None = None,
):
    """실험 모드에 맞는 action 텐서를 만든다.

    - `zero`: 모든 action을 0으로 설정
    - `pose`: `joint_pos` 기반 액션(우선) 또는 legacy obs 슬라이스(fallback)
    """
    import torch

    if mode == "zero":
        return torch.zeros(action_shape, device=device)

    if mode == "pose":
        pose_actions = _pose_actions_from_joint_pos(env, device, action_shape)
        if pose_actions is not None:
            return pose_actions

        policy_obs = extract_policy_obs(obs)
        if not hasattr(policy_obs, "shape"):
            raise RuntimeError("Pose mode expects tensor-like policy observation.")

        action_dim = action_shape[-1]
        start = 12
        end = start + action_dim
        if policy_obs.shape[-1] < end:
            raise RuntimeError(
                f"Pose mode fallback needs policy obs dim >= {end}, got {policy_obs.shape[-1]}."
            )

        print(
            "[WARN] pose action uses legacy obs slice [12:12+action_dim]; "
            "joint_pos path unavailable.",
            flush=True,
        )
        return policy_obs[:, start:end].to(device).clone()

    raise ValueError(f"Unsupported mode: {mode}")

