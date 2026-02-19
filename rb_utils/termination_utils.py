"""Termination and reward conversion helpers."""

from __future__ import annotations

from typing import Any, SupportsFloat, cast


def as_any_bool(value: Any) -> bool:
    if hasattr(value, "any"):
        any_value = value.any()
        if hasattr(any_value, "item"):
            return bool(any_value.item())
        return bool(any_value)
    return bool(value)


def infer_done_reason(info: Any) -> str:
    """Best-effort done reason extraction across different env info schemas."""
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


def reward_mean_to_float(reward: Any) -> float:
    mean_fn = getattr(reward, "mean", None)
    if callable(mean_fn):
        mean_val = mean_fn()
        if hasattr(mean_val, "item"):
            return float(cast(Any, mean_val).item())
        return float(cast(SupportsFloat, mean_val))
    return float(cast(SupportsFloat, reward))

