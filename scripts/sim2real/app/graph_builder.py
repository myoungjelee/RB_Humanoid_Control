"""Isaac ROS bridge graph builders.

This module keeps graph config normalization, prim resolution,
sensor graph assembly, and command-apply augmentation together so the
bridge path is readable as one subsystem.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any


# ---------------------------------------------------------------------------
# prim resolution 관련 helper
# ---------------------------------------------------------------------------

def normalize_prim_path(path: str) -> str:
    """env regex prim path를 단일 env 기준(env_0) concrete path로 변환."""
    out = path
    out = out.replace("env_.*/", "env_0/")
    out = out.replace(".*", "0")
    out = re.sub(r"/+", "/", out)
    if not out.startswith("/"):
        out = "/" + out
    return out


def stage_has_prim(path: str) -> bool:
    """현재 USD stage에 prim이 존재하는지 확인."""
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False
        prim = stage.GetPrimAtPath(path)
        return bool(prim and prim.IsValid())
    except Exception:
        return False


def find_descendant_prim(root_path: str, prim_name: str) -> str | None:
    """root_path 하위에서 basename이 prim_name인 prim을 찾는다."""
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None

        root_prefix = root_path.rstrip("/") + "/"
        for prim in stage.TraverseAll():
            path = str(prim.GetPath())
            if not path.startswith(root_prefix):
                continue
            if path.rsplit("/", 1)[-1] == prim_name:
                return path
    except Exception:
        return None
    return None


def resolve_robot_prim(env: Any, override: str | None) -> str:
    """robot prim 경로를 자동 탐색하거나 override 값을 사용한다."""
    normalized_override = normalize_prim_path(override) if override else None
    if normalized_override and stage_has_prim(normalized_override):
        return normalized_override

    scene = getattr(getattr(env, "unwrapped", env), "scene", None)
    if scene is None:
        if normalized_override:
            return normalized_override
        raise RuntimeError("Failed to resolve robot prim: env.scene missing")

    entity = None
    for key in ("robot", "humanoid", "g1"):
        try:
            entity = scene[key]
            break
        except Exception:
            candidate = getattr(scene, key, None)
            if candidate is not None:
                entity = candidate
                break

    if entity is None:
        raise RuntimeError("Failed to resolve robot prim: robot entity not found")

    candidates: list[str] = []
    for attr in ("prim_path", "_prim_path"):
        value = getattr(entity, attr, None)
        if isinstance(value, str) and value:
            candidates.append(value)

    cfg = getattr(entity, "cfg", None)
    cfg_prim = getattr(cfg, "prim_path", None)
    if isinstance(cfg_prim, str) and cfg_prim:
        candidates.append(cfg_prim)

    prim_paths = getattr(entity, "prim_paths", None)
    if isinstance(prim_paths, (list, tuple)) and prim_paths:
        first = prim_paths[0]
        if isinstance(first, str) and first:
            candidates.append(first)

    if not candidates:
        raise RuntimeError("Failed to resolve robot prim: no prim path candidate")

    normalized = [normalize_prim_path(path) for path in candidates]
    if normalized_override and normalized_override not in normalized:
        normalized.insert(0, normalized_override)

    expanded: list[str] = []
    for path in normalized:
        if path.endswith("/Robot"):
            for suffix in ("/pelvis", "/base_link"):
                alt = f"{path}{suffix}"
                if alt not in expanded:
                    expanded.append(alt)
        if path not in expanded:
            expanded.append(path)

    for path in expanded:
        if stage_has_prim(path):
            return path
    return expanded[0]


def resolve_imu_prim(robot_prim: str, override: str | None) -> tuple[str, str]:
    """IMU source prim을 찾고, frame_id 기본값도 함께 반환한다."""
    normalized_override = normalize_prim_path(override) if override else None
    if normalized_override and stage_has_prim(normalized_override):
        return normalized_override, normalized_override.rsplit("/", 1)[-1]

    search_root = robot_prim.rsplit("/", 1)[0] if "/" in robot_prim.rstrip("/") else robot_prim
    for prim_name in ("imu_link", "torso_link", robot_prim.rsplit("/", 1)[-1]):
        candidate = find_descendant_prim(search_root, prim_name)
        if candidate:
            return candidate, prim_name

    return robot_prim, robot_prim.rsplit("/", 1)[-1]


# ---------------------------------------------------------------------------
# graph runtime config 정리
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# command apply augmentation
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# sensor graph assembly
# ---------------------------------------------------------------------------

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

        # 먼저 concrete stage prim을 확정한 뒤 graph를 만들면
        # 마지막에 남기는 graph note도 실제 배선 결과와 맞춰진다.
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


# ---------------------------------------------------------------------------
# public phase helper
# ---------------------------------------------------------------------------

def build_m1_sensor_graph(env: Any, phase_cfg: dict[str, Any]) -> tuple[bool, str]:
    """M1 최소 센서 그래프 생성."""
    return build_sensor_graph(env=env, phase_cfg=phase_cfg, include_command_apply=False)


def build_m3_command_graph(env: Any, phase_cfg: dict[str, Any]) -> tuple[bool, str]:
    """M3 명령 적용 그래프 생성(M1 센서 + command apply)."""
    return build_sensor_graph(env=env, phase_cfg=phase_cfg, include_command_apply=True)


__all__ = [
    "GraphRuntimeConfig",
    "build_m1_sensor_graph",
    "build_m3_command_graph",
    "build_sensor_graph",
    "normalize_prim_path",
    "resolve_graph_runtime_config",
    "resolve_imu_prim",
    "resolve_robot_prim",
]
