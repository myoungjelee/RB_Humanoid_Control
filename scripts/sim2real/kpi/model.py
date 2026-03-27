"""KPI aggregation/comparison helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .parsers import (
    load_trim_hint,
    parse_fall_event,
    parse_key_float_file,
    parse_loop_stats,
    parse_reason_count,
    parse_start,
    parse_sync_markers,
    round_or_none,
)


def build_label_kpi(run_dir: Path, label: str) -> dict[str, Any]:
    label_dir = run_dir / label
    fall = parse_fall_event(label_dir / "fall_event.txt")
    disturb = parse_key_float_file(label_dir / "disturb_kpi.txt")
    reasons = parse_reason_count(label_dir / "reason_count.txt")
    sync = parse_sync_markers(label_dir / "sync_markers.txt")
    start_source = label_dir / "start.txt"
    if not start_source.exists():
        start_source = label_dir / "ros2_control.log"
    start = parse_start(start_source)
    loop_stats = parse_loop_stats(label_dir / "loop_post_sync.txt")

    survival_after_disturb_sec = None
    if fall["survived"] is True:
        survival_after_disturb_sec = fall["post_disturbance_capture_sec"]
    elif fall["fall_elapsed_sec"] is not None and sync["disturb_elapsed_sec"] is not None:
        survival_after_disturb_sec = max(0.0, fall["fall_elapsed_sec"] - sync["disturb_elapsed_sec"])

    kpi: dict[str, Any] = {
        "run_id": run_dir.name,
        "milestone": run_dir.parent.name,
        "label": label,
        "survived": fall["survived"],
        "fall_elapsed_sec": round_or_none(fall["fall_elapsed_sec"], 3),
        "post_disturbance_capture_sec": round_or_none(fall["post_disturbance_capture_sec"], 3),
        "survival_after_disturb_sec": round_or_none(survival_after_disturb_sec, 3),
        "peak_abs_tilt_r_after_disturb": round_or_none(disturb.get("peak_abs_tilt_r_after_disturb"), 3),
        "peak_abs_tilt_p_after_disturb": round_or_none(disturb.get("peak_abs_tilt_p_after_disturb"), 3),
        "first_safety_reason": reasons["first_safety_reason"],
        "reason_counts": reasons["reason_counts"],
        "disturb_elapsed_sec": round_or_none(sync["disturb_elapsed_sec"], 3),
        "disturb_window_start_sec": round_or_none(sync["disturb_window_start_sec"], 3),
        "disturb_window_end_sec": round_or_none(sync["disturb_window_end_sec"], 3),
        "disturb_target_body": sync["disturb_target_body"],
        "disturb_frame_mode": sync["disturb_frame_mode"],
        "disturb_configured_force_xyz": sync["disturb_configured_force_xyz"],
        "disturb_applied_force_xyz": sync["disturb_applied_force_xyz"],
        "disturb_is_global": sync["disturb_is_global"],
        "enable_tilt_feedback": start["enable_tilt_feedback"],
        "stand_kp": start["stand_kp"],
        "stand_kd": start["stand_kd"],
        "stand_limit": start["stand_limit"],
        "tilt_apply_mode": start["tilt_apply_mode"],
        "imu_frame_mode": start["imu_frame_mode"],
        "tilt_kp_roll": start["tilt_kp_roll"],
        "tilt_kd_roll": start["tilt_kd_roll"],
        "tilt_kp_pitch": start["tilt_kp_pitch"],
        "tilt_kd_pitch": start["tilt_kd_pitch"],
        "tilt_qref_bias_abs_max": start["tilt_qref_bias_abs_max"],
        "tilt_weight_roll_raw": start["tilt_weight_roll_raw"],
        "tilt_weight_pitch_raw": start["tilt_weight_pitch_raw"],
        "dt_mean": round_or_none(loop_stats["dt_mean"], 6),
        "dt_max": round_or_none(loop_stats["dt_max"], 6),
        "dt_p95": round_or_none(loop_stats["dt_p95"], 6),
        "miss_total": loop_stats["miss_total"],
        "sources": {
            "fall_event": str(label_dir / "fall_event.txt"),
            "disturb_kpi": str(label_dir / "disturb_kpi.txt"),
            "reason_count": str(label_dir / "reason_count.txt"),
            "sync_markers": str(label_dir / "sync_markers.txt"),
            "start": str(start_source),
        },
    }
    kpi.update(load_trim_hint())
    return kpi


def build_comparison(off: dict[str, Any], on: dict[str, Any]) -> dict[str, Any]:
    delta_roll = None
    delta_pitch = None
    if off["peak_abs_tilt_r_after_disturb"] is not None and on["peak_abs_tilt_r_after_disturb"] is not None:
        delta_roll = on["peak_abs_tilt_r_after_disturb"] - off["peak_abs_tilt_r_after_disturb"]
    if off["peak_abs_tilt_p_after_disturb"] is not None and on["peak_abs_tilt_p_after_disturb"] is not None:
        delta_pitch = on["peak_abs_tilt_p_after_disturb"] - off["peak_abs_tilt_p_after_disturb"]

    if off["survived"] is False and on["survived"] is True:
        result_tag = "pass"
    elif off["survived"] is True and on["survived"] is False:
        result_tag = "fail"
    elif off["survived"] is True and on["survived"] is True:
        improved = False
        if delta_roll is not None and delta_roll < 0:
            improved = True
        if delta_pitch is not None and delta_pitch < 0:
            improved = True
        result_tag = "partial" if improved else "fail"
    else:
        improved_count = 0
        if (
            off["fall_elapsed_sec"] is not None
            and on["fall_elapsed_sec"] is not None
            and on["fall_elapsed_sec"] > off["fall_elapsed_sec"]
        ):
            improved_count += 1
        if delta_roll is not None and delta_roll < 0:
            improved_count += 1
        if delta_pitch is not None and delta_pitch < 0:
            improved_count += 1
        result_tag = "partial" if improved_count >= 2 else "fail"

    return {
        "run_id": off["run_id"],
        "milestone": off["milestone"],
        "off_survived": off["survived"],
        "on_survived": on["survived"],
        "off_fall_elapsed_sec": off["fall_elapsed_sec"],
        "on_fall_elapsed_sec": on["fall_elapsed_sec"],
        "off_survival_after_disturb_sec": off["survival_after_disturb_sec"],
        "on_survival_after_disturb_sec": on["survival_after_disturb_sec"],
        "off_peak_abs_tilt_r_after_disturb": off["peak_abs_tilt_r_after_disturb"],
        "on_peak_abs_tilt_r_after_disturb": on["peak_abs_tilt_r_after_disturb"],
        "off_peak_abs_tilt_p_after_disturb": off["peak_abs_tilt_p_after_disturb"],
        "on_peak_abs_tilt_p_after_disturb": on["peak_abs_tilt_p_after_disturb"],
        "delta_peak_abs_tilt_r_after_disturb": round_or_none(delta_roll, 3),
        "delta_peak_abs_tilt_p_after_disturb": round_or_none(delta_pitch, 3),
        "disturb_configured_force_xyz": off["disturb_configured_force_xyz"] or on["disturb_configured_force_xyz"],
        "disturb_duration_sec": round_or_none(
            (off["disturb_window_end_sec"] - off["disturb_window_start_sec"])
            if off["disturb_window_end_sec"] is not None and off["disturb_window_start_sec"] is not None
            else None,
            3,
        ),
        "result_tag": result_tag,
    }


def build_interpretation(off: dict[str, Any], on: dict[str, Any], comparison: dict[str, Any]) -> str:
    result_tag = comparison["result_tag"]
    if result_tag == "pass":
        return "ON survived while OFF fell."
    if result_tag == "partial":
        if off["survived"] is True and on["survived"] is True:
            return "Both survived, but ON reduced the disturbance response."
        if off["survived"] is False and on["survived"] is False:
            return "Both fell, but ON delayed failure or reduced peak tilt."
    return "ON was not better than OFF in this run."
