#!/usr/bin/env python3
"""Generate a full stand_q_ref array from IsaacLab G1 init_state and joint_order_g1.yaml."""

from __future__ import annotations

import argparse
import re
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_JOINT_ORDER = PROJECT_ROOT / "ros2_ws" / "src" / "rb_bringup" / "config" / "joint_order_g1.yaml"
DEFAULT_OUTPUT = PROJECT_ROOT / "ros2_ws" / "src" / "rb_bringup" / "config" / "stand_qref_g1_seed.yaml"


SEED_RULES: list[tuple[str, float]] = [
    (".*_hip_pitch_joint", -0.20),
    (".*_knee_joint", 0.42),
    (".*_ankle_pitch_joint", -0.23),
    (".*_elbow_pitch_joint", 0.87),
    ("left_shoulder_roll_joint", 0.16),
    ("left_shoulder_pitch_joint", 0.35),
    ("right_shoulder_roll_joint", -0.16),
    ("right_shoulder_pitch_joint", 0.35),
    ("left_one_joint", 1.0),
    ("right_one_joint", -1.0),
    ("left_two_joint", 0.52),
    ("right_two_joint", -0.52),
]


def load_joint_order(path: Path) -> list[str]:
    names: list[str] = []
    in_joint_order = False
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped == "joint_order:":
            in_joint_order = True
            continue
        if not in_joint_order:
            continue
        if not stripped.startswith("- "):
            break
        names.append(stripped[2:].strip())
    if not names:
        raise RuntimeError(f"Failed to load joint_order from {path}")
    return names


def resolve_seed_value(joint_name: str) -> float:
    for pattern, value in SEED_RULES:
        if re.fullmatch(pattern, joint_name):
            return value
    return 0.0


def build_qref(joint_order: list[str]) -> list[float]:
    return [resolve_seed_value(name) for name in joint_order]


def write_output(path: Path, joint_order_path: Path, joint_order: list[str], qref: list[float]) -> None:
    lines = [
        "# Seed standing q_ref built from IsaacLab G1_CFG.init_state.joint_pos",
        f"generated_at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"source_joint_order: {joint_order_path}",
        "seed_source: IsaacLab G1_CFG.init_state.joint_pos",
        f"joint_count: {len(joint_order)}",
        "joint_order:",
    ]
    for name in joint_order:
        lines.append(f"  - {name}")
    lines.append("stand_q_ref:")
    for value in qref:
        lines.append(f"  - {value:.6f}")
    lines.append("named_seed_values:")
    for name, value in zip(joint_order, qref):
        lines.append(f"  {name}: {value:.6f}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint-order", type=Path, default=DEFAULT_JOINT_ORDER)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    joint_order = load_joint_order(args.joint_order)
    qref = build_qref(joint_order)
    write_output(args.output, args.joint_order, joint_order, qref)

    print(f"[OK] wrote stand_q_ref seed: {args.output}")
    print(f"[OK] joint_count: {len(joint_order)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
