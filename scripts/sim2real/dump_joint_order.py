#!/usr/bin/env python3
"""Capture one JointState name[] and dump it to joint_order_g1.yaml."""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PROJECT_ROOT = Path(__file__).resolve().parents[2]


class JointOrderCapture(Node):
    def __init__(self, topic: str):
        super().__init__("joint_order_capture")
        self._topic = topic
        self.joint_names: list[str] | None = None
        self.create_subscription(JointState, topic, self._on_joint_state, 10)
        self.get_logger().info(f"Waiting for joint names from topic: {topic}")

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name:
            self.get_logger().warn("Received JointState with empty name[]; waiting.")
            return
        cleaned_names = [str(name).strip() for name in msg.name]
        if any(not name for name in cleaned_names):
            self.get_logger().warn("Received JointState with blank joint names; waiting for valid sample.")
            return
        self.joint_names = cleaned_names
        self.get_logger().info(f"Captured {len(self.joint_names)} joints with valid names.")


def _write_joint_order_yaml(path: Path, topic: str, joint_names: list[str]) -> None:
    lines = [
        "# Source of truth for /rb/joint_states.name[] ordering",
        f"generated_at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"source_topic: {topic}",
        f"joint_count: {len(joint_names)}",
        "joint_order:",
    ]
    for name in joint_names:
        lines.append(f"  - {name}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Dump joint ordering from JointState.")
    parser.add_argument("--topic", type=str, default="/rb/joint_states")
    parser.add_argument(
        "--timeout_sec",
        type=float,
        default=10.0,
        help="Timeout waiting for the first non-empty JointState.name[].",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "ros2_ws" / "src" / "rb_bringup" / "config" / "joint_order_g1.yaml",
    )
    args = parser.parse_args()

    rclpy.init()
    node = JointOrderCapture(args.topic)
    deadline = time.monotonic() + args.timeout_sec

    interrupted = False
    try:
        while rclpy.ok() and node.joint_names is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        node.destroy_node()
        # Ctrl+C나 외부 signal로 이미 shutdown 된 경우를 안전하게 처리
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    if interrupted:
        print("[INFO] Interrupted by user.", file=sys.stderr)
        return 130

    if node.joint_names is None:
        print(
            (
                f"[ERROR] Timed out after {args.timeout_sec:.1f}s waiting for "
                f"{args.topic}. Check ROS2 Bridge and whether JointState.name[] is populated."
            ),
            file=sys.stderr,
        )
        return 1

    _write_joint_order_yaml(args.output, args.topic, node.joint_names)
    print(f"[OK] wrote joint ordering: {args.output}")
    print(f"[OK] joint_count: {len(node.joint_names)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
