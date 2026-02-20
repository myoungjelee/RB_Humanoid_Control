"""실험 단위 유틸리티 모음 (경로, run-id, kit 로그 인자)."""

from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path
from typing import Any


def ensure_stage_dirs(project_root: Path, stage_name: str = "stage1") -> tuple[Path, Path]:
    """스테이지 실행용 raw/summary 로그 디렉터리를 생성하고 반환한다."""
    stage_root = project_root / "logs" / stage_name
    raw_dir = stage_root / "raw"
    summary_dir = stage_root / "summary"
    raw_dir.mkdir(parents=True, exist_ok=True)
    summary_dir.mkdir(parents=True, exist_ok=True)
    return raw_dir, summary_dir


def set_kit_log_dir_env(raw_log_dir: Path) -> None:
    """Kit 로그 디렉터리 환경 변수에 raw 로그 경로를 설정한다."""
    # Kit가 로그 디렉터리 힌트로 사용할 수 있다.
    os.environ["OMNI_KIT_LOG_DIR"] = str(raw_log_dir)


def make_run_id(task_short: str, mode: str, steps: int, timestamp: str | None = None) -> str:
    """태스크 메타데이터와 타임스탬프로 일관된 run-id를 만든다."""
    ts = timestamp or datetime.now().strftime("%Y%m%d-%H%M%S")
    return f"{task_short}_{mode}_s{steps}_{ts}"


def attach_kit_log_file(args_cli: Any, raw_log_file: Path) -> None:
    """Kit 로그 파일을 ``raw_log_file``로 고정하고 append 모드를 끈다."""
    forced = f"--/log/file={raw_log_file} --/log/fileAppend=false"
    args_cli.kit_args = f"{args_cli.kit_args} {forced}".strip()
