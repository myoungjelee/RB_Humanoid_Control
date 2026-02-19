"""Experiment-level utilities (paths, run-id, git, kit logging args)."""

from __future__ import annotations

import os
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any


def ensure_stage_dirs(project_root: Path, stage_name: str = "stage1") -> tuple[Path, Path]:
    stage_root = project_root / "logs" / stage_name
    raw_dir = stage_root / "raw"
    summary_dir = stage_root / "summary"
    raw_dir.mkdir(parents=True, exist_ok=True)
    summary_dir.mkdir(parents=True, exist_ok=True)
    return raw_dir, summary_dir


def set_kit_log_dir_env(raw_log_dir: Path) -> None:
    # Kit can use this as a log directory hint.
    os.environ["OMNI_KIT_LOG_DIR"] = str(raw_log_dir)


def safe_git_commit(project_root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=project_root,
            text=True,
        ).strip()
    except Exception:
        return "unknown"


def make_run_id(task_short: str, mode: str, steps: int, timestamp: str | None = None) -> str:
    ts = timestamp or datetime.now().strftime("%Y%m%d-%H%M%S")
    return f"{task_short}_{mode}_s{steps}_{ts}"


def attach_kit_log_file(args_cli: Any, raw_log_file: Path) -> None:
    forced = f"--/log/file={raw_log_file} --/log/fileAppend=false"
    args_cli.kit_args = f"{args_cli.kit_args} {forced}".strip()

