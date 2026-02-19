"""Rollout loop helpers (env shape/device access + mode-based action generation)."""

from __future__ import annotations

import time
from typing import Any, cast

from rb_utils.termination_utils import as_any_bool, infer_done_reason, reward_mean_to_float


def get_unwrapped_num_envs(env: Any) -> int:
    num_envs = getattr(getattr(env, "unwrapped", env), "num_envs", None)
    if not isinstance(num_envs, int):
        raise RuntimeError("Environment does not expose integer 'num_envs'.")
    return num_envs


def get_unwrapped_device(env: Any) -> Any:
    device = getattr(getattr(env, "unwrapped", env), "device", None)
    if device is None:
        raise RuntimeError("Environment does not expose 'device'.")
    return device


def get_action_shape(env: Any) -> tuple[int, ...]:
    shape = getattr(getattr(env, "action_space", None), "shape", None)
    if shape is None or not isinstance(shape, tuple):
        raise RuntimeError("Environment action_space.shape is missing or invalid.")
    return cast(tuple[int, ...], shape)


def extract_policy_obs(obs: Any) -> Any:
    if isinstance(obs, dict) and "policy" in obs:
        return obs["policy"]
    return obs


def make_actions(mode: str, obs: Any, device: Any, action_shape: tuple[int, ...]):
    import torch

    if mode == "zero":
        return torch.zeros(action_shape, device=device)

    if mode == "pose":
        policy_obs = extract_policy_obs(obs)
        if not hasattr(policy_obs, "shape"):
            raise RuntimeError("Pose mode expects tensor-like policy observation.")
        action_dim = action_shape[-1]
        start = 12
        end = start + action_dim
        if policy_obs.shape[-1] < end:
            raise RuntimeError(
                f"Pose mode needs policy obs dim >= {end}, got {policy_obs.shape[-1]}."
            )
        return policy_obs[:, start:end].to(device).clone()

    raise ValueError(f"Unsupported mode: {mode}")


def run_rollout(
    env: Any,
    simulation_app: Any,
    steps: int,
    mode: str,
    reset_on_done: bool,
    initial_obs: Any | None = None,
    initial_info: Any | None = None,
) -> dict[str, Any]:
    if initial_obs is None:
        obs, info = env.reset()
    else:
        obs, info = initial_obs, initial_info
    device = get_unwrapped_device(env)
    action_shape = get_action_shape(env)
    actions = make_actions(mode, obs, device, action_shape)

    i = 0
    first_done_step = "none"
    first_done_reason = "none"
    episode_end_count = 0
    reward_mean_sum = 0.0

    start_t = time.time()
    while simulation_app.is_running() and i < steps:
        import torch

        with torch.inference_mode():
            obs, rew, terminated, truncated, info = env.step(actions)

        i += 1
        rew_mean = reward_mean_to_float(rew)
        reward_mean_sum += rew_mean

        if i % 50 == 0 or i == 1:
            print(f"[STEP] {i}/{steps} reward_mean={rew_mean:.4f}", flush=True)

        if as_any_bool(terminated) or as_any_bool(truncated):
            episode_end_count += 1
            if first_done_step == "none":
                first_done_step = str(i)
                first_done_reason = infer_done_reason(info)

            if reset_on_done:
                print(f"[DONE] episode ended at step={i}; resetting", flush=True)
                obs, info = env.reset()
                actions = make_actions(mode, obs, device, action_shape)
            else:
                print(f"[DONE] episode ended at step={i}; stopping rollout", flush=True)
                break

    elapsed = time.time() - start_t
    avg_reward_per_step = reward_mean_sum / i if i > 0 else 0.0
    return {
        "executed_steps": i,
        "first_done_step": first_done_step,
        "first_done_reason": first_done_reason,
        "episode_end_count": episode_end_count,
        "avg_reward_per_step": avg_reward_per_step,
        "elapsed_s": elapsed,
    }
