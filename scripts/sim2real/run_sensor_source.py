#!/usr/bin/env python3
"""Backward-compatible wrapper for running Sim2Real M1 sensor phase."""

from __future__ import annotations

from scripts.sim2real.main import run


if __name__ == "__main__":
    raise SystemExit(run(forced_phase="m1_sensor"))
