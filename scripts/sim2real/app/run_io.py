"""런 아티팩트 I/O 유틸.

역할:
- timestamp 런 폴더 생성
- params.yaml 저장
- summary.md 저장
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any


def make_run_dir(project_root: Path) -> Path:
    """`logs/sim2real/YYYYMMDD-HHMMSS` 런 폴더 생성."""
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = project_root / "logs" / "sim2real" / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def write_params_yaml(path: Path, params: dict[str, Any]) -> None:
    """실행 파라미터를 단순 key:value YAML 형태로 저장."""
    lines = []
    for key, value in params.items():
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        else:
            rendered = str(value)
        lines.append(f"{key}: {rendered}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_summary_md(path: Path, summary: dict[str, Any]) -> None:
    """실행 결과를 사람이 바로 읽을 수 있는 markdown으로 저장."""
    graph_built = summary.get("graph_built")
    graph_note = summary.get("graph_note")
    capture_started = summary.get("capture_started")
    capture_finished = summary.get("capture_finished")
    capture_ok = summary.get("capture_ok")
    capture_note = summary.get("capture_note")

    lines = [
        "# Sim2Real Phase Summary",
        "",
        "## Run",
        f"- phase: {summary['phase']}",
        f"- run_dir: {summary['run_dir']}",
        f"- task_id: {summary['task_id']}",
        f"- status: {summary['status']}",
        f"- executed_steps: {summary['executed_steps']}",
        f"- elapsed_sec: {summary['elapsed_sec']}",
    ]

    if graph_built is not None:
        lines.append(f"- graph_built: {graph_built}")
    if graph_note is not None:
        lines.append(f"- graph_note: {graph_note}")
    if capture_started is not None:
        lines.append(f"- capture_started: {capture_started}")
    if capture_finished is not None:
        lines.append(f"- capture_finished: {capture_finished}")
    if capture_ok is not None:
        lines.append(f"- capture_ok: {capture_ok}")
    if capture_note is not None:
        lines.append(f"- capture_note: {capture_note}")

    lines.extend(
        [
            "",
            "## Evidence Files",
            f"- {summary['run_dir']}/topic_list.txt",
            f"- {summary['run_dir']}/topic_hz_joint_states.txt",
            f"- {summary['run_dir']}/topic_echo_imu.txt",
            f"- {summary['run_dir']}/topic_echo_clock.txt",
            f"- {summary['run_dir']}/topic_echo_joint_states_once.txt",
            f"- {summary['run_dir']}/topic_capture.log",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
