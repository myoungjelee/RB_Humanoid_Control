"""종료 상태 판별 및 보상 수치 변환 유틸리티."""

from __future__ import annotations

from typing import Any, SupportsFloat, cast


def as_any_bool(value: Any) -> bool:
    """bool/tensor/array 등 다양한 타입을 불리언 하나로 안전 변환한다."""
    if hasattr(value, "any"):
        any_value = value.any()
        if hasattr(any_value, "item"):
            return bool(any_value.item())
        return bool(any_value)
    return bool(value)


def infer_done_reason(info: Any) -> str:
    """서로 다른 env info 스키마에서 done 원인을 최대한 추론해 반환한다."""
    if not isinstance(info, dict):
        return "unknown"

    for key in ("time_out", "timeouts", "truncations"):
        if key in info:
            try:
                if as_any_bool(info[key]):
                    return "time_out"
            except Exception:
                pass

    for key in ("termination", "termination_reason", "terminated_reason", "done_reason"):
        if key in info:
            try:
                value = info[key]
                if isinstance(value, str):
                    return value
                if isinstance(value, dict):
                    for reason, is_active in value.items():
                        try:
                            if as_any_bool(is_active):
                                return str(reason)
                        except Exception:
                            continue
            except Exception:
                pass

    return "unknown"


def summarize_done_info(info: Any) -> str:
    """info dict에서 종료 관련 신호를 사람이 읽기 좋은 한 줄로 요약한다."""
    if not isinstance(info, dict):
        return "info:not_dict"

    parts: list[str] = []
    for key in ("time_out", "timeouts", "truncations"):
        if key in info:
            try:
                if as_any_bool(info[key]):
                    parts.append(f"{key}=True")
            except Exception:
                parts.append(f"{key}=parse_error")

    for key in ("termination", "termination_reason", "terminated_reason", "done_reason"):
        if key not in info:
            continue
        value = info[key]
        if isinstance(value, str):
            parts.append(f"{key}={value}")
            continue
        if isinstance(value, dict):
            active: list[str] = []
            for reason, is_active in value.items():
                try:
                    if as_any_bool(is_active):
                        active.append(str(reason))
                except Exception:
                    continue
            if active:
                parts.append(f"{key}={','.join(active)}")
            else:
                parts.append(f"{key}=none_active")
            continue
        try:
            parts.append(f"{key}={as_any_bool(value)}")
        except Exception:
            parts.append(f"{key}=parse_error")

    if parts:
        return "; ".join(parts)
    return "info:no_termination_keys"


def reward_mean_to_float(reward: Any) -> float:
    """보상 값(스칼라/텐서)의 평균을 float으로 변환한다."""
    mean_fn = getattr(reward, "mean", None)
    if callable(mean_fn):
        mean_val = mean_fn()
        if hasattr(mean_val, "item"):
            return float(cast(Any, mean_val).item())
        return float(cast(SupportsFloat, mean_val))
    return float(cast(SupportsFloat, reward))
