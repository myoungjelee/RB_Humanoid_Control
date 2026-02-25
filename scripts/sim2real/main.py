#!/usr/bin/env python3
"""Compatibility wrapper for project-root main.py."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROOT_MAIN_PATH = PROJECT_ROOT / "main.py"


def _load_root_main():
    spec = importlib.util.spec_from_file_location("rb_root_main", ROOT_MAIN_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load root main module: {ROOT_MAIN_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(argv: list[str] | None = None, forced_phase: str | None = None) -> int:
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))
    module = _load_root_main()
    return int(module.run(argv=argv, forced_phase=forced_phase))


if __name__ == "__main__":
    raise SystemExit(run())
