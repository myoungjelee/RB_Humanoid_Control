#!/usr/bin/env python3
"""Compute a single-joint delta from two `ros2 topic echo` JointState dumps."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def parse_joint_state_dump(path: Path) -> tuple[list[str], list[float]]:
    names: list[str] = []
    positions: list[float] = []
    section: str | None = None

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if line == "name:":
            section = "name"
            continue
        if line == "position:":
            section = "position"
            continue
        if line.endswith(":") and line not in {"name:", "position:"}:
            section = None
            continue
        if not section or not line.startswith("- "):
            continue

        value = line[2:].strip()
        if section == "name":
            names.append(value)
        elif section == "position":
            positions.append(float(value))

    if not names:
        raise ValueError(f"{path} does not contain a JointState name list")
    if len(names) != len(positions):
        raise ValueError(
            f"{path} has mismatched name/position lengths: {len(names)} vs {len(positions)}"
        )
    return names, positions


def sign_label(value: float, eps: float) -> str:
    if math.isclose(value, 0.0, abs_tol=eps):
        return "zero"
    return "positive" if value > 0.0 else "negative"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--joint", required=True, type=str)
    parser.add_argument(
        "--eps",
        type=float,
        default=1e-6,
        help="Absolute tolerance used when classifying zero delta.",
    )
    args = parser.parse_args()

    names_before, pos_before = parse_joint_state_dump(args.before)
    names_after, pos_after = parse_joint_state_dump(args.after)

    if names_before != names_after:
        raise ValueError("JointState name ordering changed between before/after captures")

    try:
        idx = names_before.index(args.joint)
    except ValueError as exc:
        raise ValueError(f"joint '{args.joint}' not found in capture") from exc

    before = pos_before[idx]
    after = pos_after[idx]
    delta = after - before

    print(f"joint={args.joint}")
    print(f"index={idx}")
    print(f"before={before:.6f}")
    print(f"after={after:.6f}")
    print(f"delta={delta:.6f}")
    print(f"observed_sign={sign_label(delta, args.eps)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
