"""OmniGraph assembly helpers for sensor/control bridge graphs."""

from __future__ import annotations

from typing import Any

from .command_apply_graph_builder import extend_with_command_apply
from .graph_config import resolve_graph_runtime_config
from .graph_prim_resolver import resolve_imu_prim, resolve_robot_prim


def build_sensor_graph(
    env: Any,
    phase_cfg: dict[str, Any],
    *,
    include_command_apply: bool,
) -> tuple[bool, str]:
    """센서 그래프를 생성하고, 옵션으로 command apply 노드를 추가한다."""
    cfg = resolve_graph_runtime_config(phase_cfg)
    if not cfg.enabled:
        return False, "graph_builder_disabled"

    frame_imu = cfg.frame_imu
    try:
        try:
            from isaacsim.core.utils.extensions import enable_extension
        except ModuleNotFoundError:
            from omni.isaac.core.utils.extensions import enable_extension

        enable_extension(cfg.bridge_extension)

        import omni.graph.core as og

        robot_prim = resolve_robot_prim(env, phase_cfg.get("graph_builder", {}).get("robot_prim"))
        imu_prim, imu_frame_default = resolve_imu_prim(robot_prim, phase_cfg.get("graph_builder", {}).get("imu_prim"))
        if frame_imu.lower() == "auto":
            frame_imu = imu_frame_default
        keys = og.Controller.Keys

        graph_spec: dict[str, Any] = {
            "graph_path": cfg.graph_path,
            "evaluator_name": cfg.evaluator_name,
        }
        if cfg.use_on_demand_pipeline:
            graph_spec["pipeline_stage"] = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND

        create_nodes: list[tuple[str, str]] = [
            ("Tick", cfg.tick_node_type),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ClockPub", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("JointStatePub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("OdomCompute", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("ImuPub", "isaacsim.ros2.bridge.ROS2PublishImu"),
        ]
        connect_edges: list[tuple[str, str]] = [
            (f"Tick.{cfg.tick_output_port}", "ClockPub.inputs:execIn"),
            (f"Tick.{cfg.tick_output_port}", "JointStatePub.inputs:execIn"),
            (f"Tick.{cfg.tick_output_port}", "OdomCompute.inputs:execIn"),
            (f"Tick.{cfg.tick_output_port}", "ImuPub.inputs:execIn"),
            ("Context.outputs:context", "ClockPub.inputs:context"),
            ("Context.outputs:context", "JointStatePub.inputs:context"),
            ("Context.outputs:context", "ImuPub.inputs:context"),
            ("ReadTime.outputs:simulationTime", "JointStatePub.inputs:timeStamp"),
            ("ReadTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
            ("ReadTime.outputs:simulationTime", "ImuPub.inputs:timeStamp"),
            ("OdomCompute.outputs:orientation", "ImuPub.inputs:orientation"),
            ("OdomCompute.outputs:angularVelocity", "ImuPub.inputs:angularVelocity"),
            ("OdomCompute.outputs:linearAcceleration", "ImuPub.inputs:linearAcceleration"),
        ]
        set_values: list[tuple[str, Any]] = [
            ("JointStatePub.inputs:targetPrim", robot_prim),
            ("JointStatePub.inputs:topicName", cfg.topic_joint),
            ("OdomCompute.inputs:chassisPrim", imu_prim),
            ("ImuPub.inputs:topicName", cfg.topic_imu),
            ("ImuPub.inputs:frameId", frame_imu),
            ("ImuPub.inputs:publishOrientation", True),
            ("ImuPub.inputs:publishAngularVelocity", True),
            ("ImuPub.inputs:publishLinearAcceleration", True),
        ]

        if include_command_apply:
            extend_with_command_apply(
                create_nodes=create_nodes,
                connect_edges=connect_edges,
                set_values=set_values,
                tick_output_port=cfg.tick_output_port,
                topic_command_raw=cfg.topic_command_raw,
                robot_prim=robot_prim,
            )

        (graph, _, _, _) = og.Controller.edit(
            graph_spec,
            {
                keys.CREATE_NODES: create_nodes,
                keys.CONNECT: connect_edges,
                keys.SET_VALUES: set_values,
            },
        )
        if cfg.tick_node_type == "omni.graph.action.OnTick":
            og.Controller.set(f"{cfg.graph_path}/Tick.inputs:onlyPlayback", cfg.tick_only_playback)
            og.Controller.set(f"{cfg.graph_path}/Tick.inputs:framePeriod", cfg.tick_frame_period)
        og.Controller.evaluate_sync(graph)
        return (
            True,
            (
                f"{cfg.graph_path} robot_prim={robot_prim}"
                f" imu_prim={imu_prim} frame_imu={frame_imu}"
                f" tick_node={cfg.tick_node_type} tick_output={cfg.tick_output_port}"
                f" only_playback={cfg.tick_only_playback} frame_period={cfg.tick_frame_period}"
                f" evaluator={cfg.evaluator_name} command_apply={include_command_apply}"
                f" command_topic={cfg.topic_command_raw}"
            ),
        )
    except Exception as exc:
        note = f"graph_build_failed: {type(exc).__name__}: {exc}"
        if cfg.strict:
            raise RuntimeError(note) from exc
        return False, note
