"""Configuration normalization for Isaac bridge graphs."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class GraphRuntimeConfig:
    enabled: bool
    strict: bool
    graph_path: str
    frame_imu: str
    topic_joint: str
    topic_imu: str
    topic_command_raw: str
    tick_node_type: str
    tick_output_port: str
    evaluator_name: str
    use_on_demand_pipeline: bool
    tick_only_playback: bool
    tick_frame_period: int
    bridge_extension: str


def resolve_graph_runtime_config(phase_cfg: dict[str, Any]) -> GraphRuntimeConfig:
    """그래프 빌더가 사용할 런타임 설정을 정규화한다."""
    graph_cfg = phase_cfg.get("graph_builder", {})
    topics = phase_cfg.get("topics", {})
    frame_cfg = graph_cfg.get("frame_ids", {})

    tick_source = str(graph_cfg.get("tick_source", "on_physics_step")).lower()
    if tick_source == "on_physics_step":
        tick_node_type = "isaacsim.core.nodes.OnPhysicsStep"
        tick_output_port = "outputs:step"
        evaluator_name = "push"
        use_on_demand_pipeline = True
    elif tick_source == "on_tick":
        tick_node_type = "omni.graph.action.OnTick"
        tick_output_port = "outputs:tick"
        evaluator_name = "execution"
        use_on_demand_pipeline = False
    else:
        tick_node_type = "omni.graph.action.OnPlaybackTick"
        tick_output_port = "outputs:tick"
        evaluator_name = "execution"
        use_on_demand_pipeline = False

    return GraphRuntimeConfig(
        enabled=bool(graph_cfg.get("enabled", True)),
        strict=bool(graph_cfg.get("strict", False)),
        graph_path=str(graph_cfg.get("graph_path", "/RBM1Graph")),
        frame_imu=str(frame_cfg.get("imu", "auto")),
        topic_joint=str(topics.get("joint_states", "/rb/joint_states")),
        topic_imu=str(topics.get("imu", "/rb/imu")),
        topic_command_raw=str(topics.get("command_raw", "/rb/command_raw")),
        tick_node_type=tick_node_type,
        tick_output_port=tick_output_port,
        evaluator_name=evaluator_name,
        use_on_demand_pipeline=use_on_demand_pipeline,
        tick_only_playback=bool(graph_cfg.get("tick_only_playback", False)),
        tick_frame_period=int(graph_cfg.get("tick_frame_period", 0)),
        bridge_extension=str(phase_cfg.get("bridge_extension", "isaacsim.ros2.bridge")),
    )
