#!/usr/bin/env python3
"""M8 run artifacts -> KPI/summary extractor.

Reads existing raw logs under:
  logs/sim2real/m8/<run_id>/balance_off
  logs/sim2real/m8/<run_id>/balance_on

Writes:
  logs/sim2real/m9/<run_id>/balance_off_kpi.json
  logs/sim2real/m9/<run_id>/balance_on_kpi.json
  logs/sim2real/m9/<run_id>/comparison.json
  logs/sim2real/m9/<run_id>/summary.md
  logs/sim2real/m9/index.csv
"""

from __future__ import annotations

import argparse
import csv
import json
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

    no_fall_match = re.search(
        r"\[NO_FALL_EVENT\]\s+post_disturbance_capture_sec=([0-9.]+)", text
    )
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
            if reason == r"\1":
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

    disturb_end_match = re.search(
        r"\[DISTURBANCE_END\]\s+elapsed_sec=([0-9.]+)\s+window_end_sec=([0-9.]+)",
        text,
    )
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

    controller_match = re.search(
        r"stand_kp=([0-9.]+)\s+stand_kd=([0-9.]+)\s+stand_limit=([0-9.]+)", text
    )
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
        Path(__file__).resolve().parents[2]
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


def build_label_kpi(run_dir: Path, label: str) -> dict[str, Any]:
    label_dir = run_dir / label
    fall = parse_fall_event(label_dir / "fall_event.txt")
    disturb = parse_key_float_file(label_dir / "disturb_kpi.txt")
    reasons = parse_reason_count(label_dir / "reason_count.txt")
    sync = parse_sync_markers(label_dir / "sync_markers.txt")
    start = parse_start(label_dir / "start.txt")
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
            "start": str(label_dir / "start.txt"),
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


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def format_result_line(label: str, survived: bool | None, fall_elapsed_sec: float | None) -> str:
    if survived is True:
        return f"- {label}: NO_FALL_EVENT"
    if survived is False and fall_elapsed_sec is not None:
        return f"- {label}: FALL at {fall_elapsed_sec:.3f}s"
    return f"- {label}: UNKNOWN"


def format_trim_hint(trim_hint: dict[str, Any] | None) -> str:
    if not trim_hint:
        return "none"

    hip = trim_hint.get("left_hip_pitch_joint")
    ankle = trim_hint.get("left_ankle_pitch_joint")
    if hip is None and ankle is None:
        return "none"
    return f"hip_pitch {hip:+.3f}, ankle_pitch {ankle:+.3f}"


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


def format_fall_or_no_fall(survived: bool | None, fall_elapsed_sec: float | None) -> str:
    if survived is True:
        return "NO_FALL_EVENT"
    if survived is False and fall_elapsed_sec is not None:
        return f"FALL at {fall_elapsed_sec:.3f}s"
    return "UNKNOWN"


def write_summary_md(path: Path, off: dict[str, Any], on: dict[str, Any], comparison: dict[str, Any]) -> None:
    force = comparison["disturb_configured_force_xyz"]
    duration = comparison["disturb_duration_sec"]
    force_summary = "torso impulse unknown"
    if force and force[0] is not None:
        force_summary = f"torso impulse {force[0]:.0f}N x {duration:.2f}s"

    trim_hint = on.get("stand_q_ref_trim_hint") or {}
    hip_trim = trim_hint.get("left_hip_pitch_joint")
    ankle_trim = trim_hint.get("left_ankle_pitch_joint")

    lines = [
        "# M8 Result",
        "",
        f"- run_id: {comparison['run_id']}",
        f"- verdict: {comparison['result_tag'].upper()}",
        f"- disturbance: {force_summary}",
        "",
        "## Outcome",
        "| label | result |",
        "|---|---|",
        f"| balance_off | {format_fall_or_no_fall(off['survived'], off['fall_elapsed_sec'])} |",
        f"| balance_on | {format_fall_or_no_fall(on['survived'], on['fall_elapsed_sec'])} |",
        "",
        f"Interpretation: {build_interpretation(off, on, comparison)}",
        "",
        "## Key KPI",
        "| metric | off | on |",
        "|---|---:|---:|",
        (
            f"| peak_abs_tilt_r_after_disturb | "
            f"{off['peak_abs_tilt_r_after_disturb']:.3f} | {on['peak_abs_tilt_r_after_disturb']:.3f} |"
        ),
        (
            f"| peak_abs_tilt_p_after_disturb | "
            f"{off['peak_abs_tilt_p_after_disturb']:.3f} | {on['peak_abs_tilt_p_after_disturb']:.3f} |"
        ),
        "",
        "## Config",
        "| field | value |",
        "|---|---|",
        f"| tilt_qref_bias_abs_max | {on['tilt_qref_bias_abs_max']:.2f} |",
        f"| hip_pitch_joint_trim | {hip_trim:+.3f} |" if hip_trim is not None else "| hip_pitch_joint_trim | none |",
        f"| ankle_pitch_joint_trim | {ankle_trim:+.3f} |" if ankle_trim is not None else "| ankle_pitch_joint_trim | none |",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def upsert_index_csv(index_path: Path, comparison: dict[str, Any]) -> None:
    fieldnames = [
        "run_id",
        "force_x",
        "force_y",
        "force_z",
        "duration_sec",
        "off_survived",
        "on_survived",
        "off_fall_elapsed_sec",
        "on_fall_elapsed_sec",
        "off_peak_tilt_r",
        "on_peak_tilt_r",
        "off_peak_tilt_p",
        "on_peak_tilt_p",
        "result_tag",
    ]

    force = comparison.get("disturb_configured_force_xyz") or [None, None, None]
    row = {
        "run_id": comparison["run_id"],
        "force_x": force[0],
        "force_y": force[1],
        "force_z": force[2],
        "duration_sec": comparison["disturb_duration_sec"],
        "off_survived": comparison["off_survived"],
        "on_survived": comparison["on_survived"],
        "off_fall_elapsed_sec": comparison["off_fall_elapsed_sec"],
        "on_fall_elapsed_sec": comparison["on_fall_elapsed_sec"],
        "off_peak_tilt_r": comparison["off_peak_abs_tilt_r_after_disturb"],
        "on_peak_tilt_r": comparison["on_peak_abs_tilt_r_after_disturb"],
        "off_peak_tilt_p": comparison["off_peak_abs_tilt_p_after_disturb"],
        "on_peak_tilt_p": comparison["on_peak_abs_tilt_p_after_disturb"],
        "result_tag": comparison["result_tag"],
    }

    rows: list[dict[str, Any]] = []
    if index_path.exists():
        with index_path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            rows = list(reader)

    updated = False
    for idx, existing in enumerate(rows):
        if existing.get("run_id") == comparison["run_id"]:
            rows[idx] = row
            updated = True
            break
    if not updated:
        rows.append(row)

    index_path.parent.mkdir(parents=True, exist_ok=True)
    with index_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract M8 KPI from an existing run directory.")
    parser.add_argument("run_dir", type=Path, help="Path like logs/sim2real/m8/<run_id>")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    m9_root = run_dir.parent.parent / "m9"
    output_dir = m9_root / run_dir.name
    output_dir.mkdir(parents=True, exist_ok=True)

    off = build_label_kpi(run_dir, "balance_off")
    on = build_label_kpi(run_dir, "balance_on")
    comparison = build_comparison(off, on)

    write_json(output_dir / "balance_off_kpi.json", off)
    write_json(output_dir / "balance_on_kpi.json", on)
    write_json(output_dir / "comparison.json", comparison)
    write_summary_md(output_dir / "summary.md", off, on, comparison)
    upsert_index_csv(m9_root / "index.csv", comparison)

    print(f"[M9] wrote {output_dir / 'balance_off_kpi.json'}")
    print(f"[M9] wrote {output_dir / 'balance_on_kpi.json'}")
    print(f"[M9] wrote {output_dir / 'comparison.json'}")
    print(f"[M9] wrote {output_dir / 'summary.md'}")
    print(f"[M9] updated {m9_root / 'index.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
