"""실험 롤아웃 실행 유틸리티.

- env에서 공통 속성(num_envs/device/action shape) 추출
- step 루프 실행 및 done 진단/통계 수집
- 액션 생성 로직은 외부 함수로 주입 가능
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable, cast

from rb_utils.action_generators import make_actions as make_actions_default
from rb_utils.termination_utils import (
    as_any_bool,
    infer_done_reason,
    reward_mean_to_float,
    summarize_done_info,
)


def get_unwrapped_num_envs(env: Any) -> int:
    """`env.unwrapped.num_envs`를 안전하게 읽어 정수로 반환한다."""
    num_envs = getattr(getattr(env, "unwrapped", env), "num_envs", None)
    if not isinstance(num_envs, int):
        raise RuntimeError("Environment does not expose integer 'num_envs'.")
    return num_envs


def get_unwrapped_device(env: Any) -> Any:
    """`env.unwrapped.device`를 반환한다."""
    device = getattr(getattr(env, "unwrapped", env), "device", None)
    if device is None:
        raise RuntimeError("Environment does not expose 'device'.")
    return device


def get_action_shape(env: Any) -> tuple[int, ...]:
    """`env.action_space.shape`를 튜플로 반환한다."""
    shape = getattr(getattr(env, "action_space", None), "shape", None)
    if shape is None or not isinstance(shape, tuple):
        raise RuntimeError("Environment action_space.shape is missing or invalid.")
    return cast(tuple[int, ...], shape)


def _first_env_vector_to_list(value: Any) -> list[float] | None:
    """벡터형 값을 첫 번째 env 기준 `list[float]`로 변환한다."""
    try:
        first = value[0]
    except Exception:
        return None

    try:
        if hasattr(first, "detach"):
            first = first.detach()
        if hasattr(first, "cpu"):
            first = first.cpu()
        if hasattr(first, "tolist"):
            raw = first.tolist()
        else:
            raw = first
    except Exception:
        return None

    if isinstance(raw, (list, tuple)):
        result: list[float] = []
        for item in raw:
            try:
                result.append(float(item))
            except Exception:
                return None
        return result
    return None


def _quat_wxyz_to_roll_pitch(quat_wxyz: list[float]) -> tuple[float, float] | None:
    """wxyz 쿼터니언을 roll/pitch(rad)로 변환한다."""
    if len(quat_wxyz) != 4:
        return None
    w, x, y, z = quat_wxyz

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


def _get_first_done_state(env: Any) -> str:
    """첫 done 시점의 로봇 상태(z/roll/pitch/속도)를 문자열로 요약한다."""
    unwrapped = getattr(env, "unwrapped", env)
    scene = getattr(unwrapped, "scene", None)
    if scene is None:
        return "scene=missing"

    robot = None
    for key in ("robot", "humanoid", "g1"):
        try:
            candidate = scene[key]
            robot = candidate
            break
        except Exception:
            candidate = getattr(scene, key, None)
            if candidate is not None:
                robot = candidate
                break
    if robot is None:
        return "robot=missing"

    data = getattr(robot, "data", None)
    if data is None:
        return "robot.data=missing"

    pos = _first_env_vector_to_list(getattr(data, "root_pos_w", None))
    quat = _first_env_vector_to_list(getattr(data, "root_quat_w", None))
    lin = _first_env_vector_to_list(getattr(data, "root_lin_vel_w", None))
    ang = _first_env_vector_to_list(getattr(data, "root_ang_vel_w", None))
    rp = _quat_wxyz_to_roll_pitch(quat) if quat is not None else None

    parts: list[str] = []
    if pos is not None and len(pos) >= 3:
        parts.append(f"z={pos[2]:.3f}")
    if rp is not None:
        parts.append(f"roll={rp[0]:.3f}")
        parts.append(f"pitch={rp[1]:.3f}")
    if lin is not None and len(lin) >= 3:
        parts.append(f"lin=({lin[0]:.3f},{lin[1]:.3f},{lin[2]:.3f})")
    if ang is not None and len(ang) >= 3:
        parts.append(f"ang=({ang[0]:.3f},{ang[1]:.3f},{ang[2]:.3f})")

    if parts:
        return " ".join(parts)
    return "done_state=unavailable"


def _get_first_done_terms(env: Any) -> list[str]:
    """IsaacLab termination_manager에서 첫 done에 해당하는 term 이름들을 수집한다."""
    unwrapped = getattr(env, "unwrapped", env)
    term_manager = getattr(unwrapped, "termination_manager", None)
    if term_manager is None:
        return []

    active_terms = getattr(term_manager, "active_terms", None)
    if not isinstance(active_terms, list):
        return []

    hit_terms: list[str] = []
    for name in active_terms:
        try:
            term_value = term_manager.get_term(name)
            if as_any_bool(term_value[0]):
                hit_terms.append(str(name))
        except Exception:
            continue
    return hit_terms


def run_rollout(
    env: Any,
    simulation_app: Any,
    steps: int,
    mode: str,
    reset_on_done: bool,
    initial_obs: Any | None = None,
    initial_info: Any | None = None,
    action_fn: Callable[[str, Any, Any, tuple[int, ...], Any | None], Any] | None = None,
) -> dict[str, Any]:
    """롤아웃 루프를 실행하고 done/보상/시간 통계를 반환한다.

    `action_fn`을 넘기면 stage별 액션 생성기를 주입할 수 있다.
    """
    if initial_obs is None:
        obs, info = env.reset()
    else:
        obs, info = initial_obs, initial_info
    device = get_unwrapped_device(env)
    action_shape = get_action_shape(env)
    if action_fn is None:
        action_fn = make_actions_default
    actions = action_fn(mode, obs, device, action_shape, env)

    i = 0
    first_done_step = "none"
    first_done_reason = "none"
    first_done_flags = "none"
    first_done_terms = "none"
    first_done_state = "none"
    first_done_info = "none"
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

        term_flag = as_any_bool(terminated)
        trunc_flag = as_any_bool(truncated)
        if term_flag or trunc_flag:
            episode_end_count += 1
            if first_done_step == "none":
                first_done_step = str(i)
                first_done_reason = infer_done_reason(info)
                if first_done_reason == "unknown":
                    if term_flag and trunc_flag:
                        first_done_reason = "terminated+truncated"
                    elif term_flag:
                        first_done_reason = "terminated"
                    elif trunc_flag:
                        first_done_reason = "truncated"
                first_done_flags = f"terminated={term_flag},truncated={trunc_flag}"
                term_hits = _get_first_done_terms(env)
                if term_hits:
                    first_done_terms = ",".join(term_hits)
                    first_done_reason = term_hits[0]
                first_done_state = _get_first_done_state(env)
                first_done_info = summarize_done_info(info)
                print(
                    f"[DONE] step={i} {first_done_flags} reason={first_done_reason}",
                    flush=True,
                )
                if first_done_terms != "none":
                    print(f"[DONE_TERMS] {first_done_terms}", flush=True)
                print(f"[DONE_STATE] {first_done_state}", flush=True)
                print(f"[DONE_INFO] {first_done_info}", flush=True)

            if reset_on_done:
                print(f"[DONE] episode ended at step={i}; resetting", flush=True)
                obs, info = env.reset()
                actions = action_fn(mode, obs, device, action_shape, env)
            else:
                print(f"[DONE] episode ended at step={i}; stopping rollout", flush=True)
                break

    elapsed = time.time() - start_t
    avg_reward_per_step = reward_mean_sum / i if i > 0 else 0.0
    return {
        "executed_steps": i,
        "first_done_step": first_done_step,
        "first_done_reason": first_done_reason,
        "first_done_flags": first_done_flags,
        "first_done_terms": first_done_terms,
        "first_done_state": first_done_state,
        "first_done_info": first_done_info,
        "episode_end_count": episode_end_count,
        "avg_reward_per_step": avg_reward_per_step,
        "elapsed_s": elapsed,
    }
