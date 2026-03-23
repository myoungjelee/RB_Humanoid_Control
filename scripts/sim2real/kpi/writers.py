"""Writers for KPI JSON/markdown/index artifacts."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from .model import build_interpretation


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
