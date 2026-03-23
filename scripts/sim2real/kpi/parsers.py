"""Log parsers for M8/M9 KPI extraction."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any


def read_text(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def parse_triplet(text: str) -> list[float] | None:
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3:
        return None
    try:
        return [float(part) for part in parts]
    except ValueError:
        return None


def round_or_none(value: float | None, digits: int = 6) -> float | None:
    if value is None:
        return None
    return round(value, digits)


def parse_fall_event(path: Path) -> dict[str, Any]:
    text = read_text(path).strip()
    result: dict[str, Any] = {
        "survived": None,
        "fall_elapsed_sec": None,
        "post_disturbance_capture_sec": None,
        "fall_event_raw": text,
    }
    if not text:
        return result

    no_fall_match = re.search(r"\[NO_FALL_EVENT\]\s+post_disturbance_capture_sec=([0-9.]+)", text)
    if no_fall_match:
        result["survived"] = True
        result["post_disturbance_capture_sec"] = float(no_fall_match.group(1))
        return result

    fall_match = re.search(r"\[FALL_EVENT\]\s+elapsed_sec=([0-9.]+)", text)
    if fall_match:
        result["survived"] = False
        result["fall_elapsed_sec"] = float(fall_match.group(1))

    return result


def parse_key_float_file(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    for line in read_text(path).splitlines():
        match = re.match(r"([A-Za-z0-9_]+)=(-?[0-9.]+)", line.strip())
        if match:
            values[match.group(1)] = float(match.group(2))
    return values


def parse_reason_count(path: Path) -> dict[str, Any]:
    text = read_text(path).strip()
    result: dict[str, Any] = {
        "first_safety_reason": None,
        "reason_counts": {},
        "reason_count_raw": text,
    }
    if not text:
        return result
    if "[NO_SAFETY_REASON]" in text:
        return result

    counts: dict[str, int] = {}
    for line in text.splitlines():
        match = re.match(r"\s*(\d+)\s+(\S+)\s*$", line)
        if match:
            reason = match.group(2)
            if reason == r"":
                reason = "UNKNOWN"
            counts[reason] = int(match.group(1))

    if counts:
        result["reason_counts"] = counts
        result["first_safety_reason"] = next(iter(counts))
    else:
        result["first_safety_reason"] = "UNKNOWN"
    return result


def parse_sync_markers(path: Path) -> dict[str, Any]:
    text = read_text(path)
    result: dict[str, Any] = {
        "first_sim_step_sec": None,
        "control_active_wall_sec": None,
        "disturb_elapsed_sec": None,
        "disturb_window_start_sec": None,
        "disturb_window_end_sec": None,
        "disturb_target_body": None,
        "disturb_frame_mode": None,
        "disturb_configured_force_xyz": None,
        "disturb_applied_force_xyz": None,
        "disturb_is_global": None,
    }

    first_step_match = re.search(r"\[SYNC\]\s+FIRST_SIM_STEP\s+sim_elapsed_sec=([0-9.]+)", text)
    if first_step_match:
        result["first_sim_step_sec"] = float(first_step_match.group(1))

    control_active_match = re.search(r"\[([0-9.]+)\].*\[SYNC\]\s+CONTROL_ACTIVE", text)
    if control_active_match:
        result["control_active_wall_sec"] = float(control_active_match.group(1))

    disturb_start_match = re.search(
        r"\[DISTURBANCE_START\]\s+elapsed_sec=([0-9.]+)\s+window_start_sec=([0-9.]+)\s+"
        r"body=([^\s]+)\s+frame_mode=([^\s]+).*configured_force_xyz=\(([^)]+)\).*"
        r"applied_force_xyz=\(([^)]+)\).*is_global=(True|False)",
        text,
    )
    if disturb_start_match:
        result["disturb_elapsed_sec"] = float(disturb_start_match.group(1))
        result["disturb_window_start_sec"] = float(disturb_start_match.group(2))
        result["disturb_target_body"] = disturb_start_match.group(3)
        result["disturb_frame_mode"] = disturb_start_match.group(4)
        result["disturb_configured_force_xyz"] = parse_triplet(disturb_start_match.group(5))
        result["disturb_applied_force_xyz"] = parse_triplet(disturb_start_match.group(6))
        result["disturb_is_global"] = disturb_start_match.group(7) == "True"

    disturb_end_match = re.search(r"\[DISTURBANCE_END\]\s+elapsed_sec=([0-9.]+)\s+window_end_sec=([0-9.]+)", text)
    if disturb_end_match:
        result["disturb_window_end_sec"] = float(disturb_end_match.group(2))

    return result


def parse_start(path: Path) -> dict[str, Any]:
    text = read_text(path)
    result: dict[str, Any] = {
        "enable_tilt_feedback": None,
        "stand_kp": None,
        "stand_kd": None,
        "stand_limit": None,
        "tilt_apply_mode": None,
        "imu_frame_mode": None,
        "tilt_kp_roll": None,
        "tilt_kd_roll": None,
        "tilt_kp_pitch": None,
        "tilt_kd_pitch": None,
        "tilt_qref_bias_abs_max": None,
        "tilt_weight_roll_raw": None,
        "tilt_weight_pitch_raw": None,
    }

    controller_match = re.search(r"stand_kp=([0-9.]+)\s+stand_kd=([0-9.]+)\s+stand_limit=([0-9.]+)", text)
    if controller_match:
        result["stand_kp"] = float(controller_match.group(1))
        result["stand_kd"] = float(controller_match.group(2))
        result["stand_limit"] = float(controller_match.group(3))

    tilt_match = re.search(
        r"tilt_feedback=(on|off)\s+mode=([^\s]+)\s+imu_frame_mode=([^\s]+).*"
        r"kp_roll=([0-9.]+)\s+kd_roll=([0-9.]+)\s+kp_pitch=([0-9.]+)\s+kd_pitch=([0-9.]+).*"
        r"qref_bias_max=([0-9.]+).*weights roll\(h/a/t\)=([0-9./-]+)\s+pitch\(h/a/k/t\)=([0-9./-]+)",
        text,
    )
    if tilt_match:
        result["enable_tilt_feedback"] = tilt_match.group(1) == "on"
        result["tilt_apply_mode"] = tilt_match.group(2)
        result["imu_frame_mode"] = tilt_match.group(3)
        result["tilt_kp_roll"] = float(tilt_match.group(4))
        result["tilt_kd_roll"] = float(tilt_match.group(5))
        result["tilt_kp_pitch"] = float(tilt_match.group(6))
        result["tilt_kd_pitch"] = float(tilt_match.group(7))
        result["tilt_qref_bias_abs_max"] = float(tilt_match.group(8))
        result["tilt_weight_roll_raw"] = tilt_match.group(9)
        result["tilt_weight_pitch_raw"] = tilt_match.group(10)

    return result


def parse_loop_stats(path: Path) -> dict[str, Any]:
    text = read_text(path)
    first_line = text.splitlines()[0] if text.splitlines() else ""
    result: dict[str, Any] = {
        "dt_mean": None,
        "dt_max": None,
        "dt_p95": None,
        "miss_total": None,
    }
    if not first_line:
        return result

    for key in ("dt_mean", "dt_max", "dt_p95"):
        match = re.search(rf"{key}=([0-9.]+)", first_line)
        if match:
            result[key] = float(match.group(1))
    miss_match = re.search(r"miss_total=([0-9]+)", first_line)
    if miss_match:
        result["miss_total"] = int(miss_match.group(1))
    return result


def load_trim_hint() -> dict[str, Any]:
    scenario_path = (
        Path(__file__).resolve().parents[3]
        / "ros2_ws"
        / "src"
        / "rb_controller"
        / "config"
        / "scenarios"
        / "stand_pd_balance_base.yaml"
    )
    text = read_text(scenario_path)
    match = re.search(r"stand_q_ref_trim:\s*\[(.*?)\]", text, re.DOTALL)
    if not match:
        return {"stand_q_ref_trim_hint": None}

    raw_values = [chunk.strip() for chunk in match.group(1).replace("\n", " ").split(",") if chunk.strip()]
    try:
        values = [float(value) for value in raw_values]
    except ValueError:
        return {"stand_q_ref_trim_hint": None}

    hint = {
        "left_hip_pitch_joint": values[0] if len(values) > 0 else None,
        "right_hip_pitch_joint": values[1] if len(values) > 1 else None,
        "left_ankle_pitch_joint": values[15] if len(values) > 15 else None,
        "right_ankle_pitch_joint": values[16] if len(values) > 16 else None,
    }
    return {"stand_q_ref_trim_hint": hint}
