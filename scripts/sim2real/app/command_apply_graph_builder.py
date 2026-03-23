"""Command-apply graph augmentation helpers."""

from __future__ import annotations

from typing import Any


def extend_with_command_apply(
    *,
    create_nodes: list[tuple[str, str]],
    connect_edges: list[tuple[str, str]],
    set_values: list[tuple[str, Any]],
    tick_output_port: str,
    topic_command_raw: str,
    robot_prim: str,
) -> None:
    """센서 그래프에 command_raw -> articulation apply 노드를 추가한다."""
    create_nodes.extend(
        [
            ("JointStateSub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ]
    )
    connect_edges.extend(
        [
            (f"Tick.{tick_output_port}", "JointStateSub.inputs:execIn"),
            (f"Tick.{tick_output_port}", "ArticulationController.inputs:execIn"),
            ("Context.outputs:context", "JointStateSub.inputs:context"),
            ("JointStateSub.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("JointStateSub.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ]
    )
    set_values.extend(
        [
            ("JointStateSub.inputs:topicName", topic_command_raw),
            ("ArticulationController.inputs:robotPath", robot_prim),
        ]
    )
