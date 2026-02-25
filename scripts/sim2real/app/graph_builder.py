"""M1 센서 파이프라인용 OmniGraph 생성기.

목표 토픽:
- /clock
- /rb/joint_states
- /rb/imu
"""

from __future__ import annotations

import re
from typing import Any


def _normalize_prim_path(path: str) -> str:
    """env regex prim path를 단일 env 기준(env_0) concrete path로 변환."""
    out = path
    out = out.replace("env_.*/", "env_0/")
    out = out.replace(".*", "0")
    out = re.sub(r"/+", "/", out)
    if not out.startswith("/"):
        out = "/" + out
    return out


def _stage_has_prim(path: str) -> bool:
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


def _resolve_robot_prim(env: Any, override: str | None) -> str:
    """robot prim 경로를 자동 탐색하거나 override 값을 사용한다."""
    if override:
        return _normalize_prim_path(override)

    scene = getattr(getattr(env, "unwrapped", env), "scene", None)
    if scene is None:
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

    normalized = [_normalize_prim_path(path) for path in candidates]

    # IsaacLab 자산에서 /Robot는 컨테이너이고 실제 articulation root가 /pelvis인 경우가 많다.
    # 따라서 /Robot 경로가 있으면 자식(root 후보)을 먼저 우선시한다.
    expanded: list[str] = []
    for path in normalized:
        if path.endswith("/Robot"):
            for suffix in ("/pelvis", "/base_link"):
                alt = f"{path}{suffix}"
                if alt not in expanded:
                    expanded.append(alt)
        if path not in expanded:
            expanded.append(path)

    # stage에 실제 존재하는 경로를 우선 사용한다.
    for path in expanded:
        if _stage_has_prim(path):
            return path

    # stage 조회 실패/미초기화 시 첫 후보를 fallback으로 사용.
    return expanded[0]


def build_m1_sensor_graph(env: Any, phase_cfg: dict[str, Any]) -> tuple[bool, str]:
    """M1 최소 센서 그래프 생성.

    반환:
    - bool: 생성 성공 여부
    - str : 디버깅용 상태 노트(prim/tick node 정보 또는 에러)
    """
    graph_cfg = phase_cfg.get("graph_builder", {})
    enabled = bool(graph_cfg.get("enabled", True))
    if not enabled:
        return False, "graph_builder_disabled"

    strict = bool(graph_cfg.get("strict", False))
    graph_path = str(graph_cfg.get("graph_path", "/RBM1Graph"))
    frame_cfg = graph_cfg.get("frame_ids", {})
    topics = phase_cfg.get("topics", {})

    frame_imu = str(frame_cfg.get("imu", "imu_link"))
    topic_joint = str(topics.get("joint_states", "/rb/joint_states"))
    topic_imu = str(topics.get("imu", "/rb/imu"))
    tick_source = str(graph_cfg.get("tick_source", "on_physics_step")).lower()
    if tick_source == "on_physics_step":
        tick_node_type = "isaacsim.core.nodes.OnPhysicsStep"
        tick_output_port = "outputs:step"
        evaluator_name = "push"
    elif tick_source == "on_tick":
        tick_node_type = "omni.graph.action.OnTick"
        tick_output_port = "outputs:tick"
        evaluator_name = "execution"
    else:
        tick_node_type = "omni.graph.action.OnPlaybackTick"
        tick_output_port = "outputs:tick"
        evaluator_name = "execution"
    tick_only_playback = bool(graph_cfg.get("tick_only_playback", False))
    tick_frame_period = int(graph_cfg.get("tick_frame_period", 0))

    try:
        # Isaac Sim 5.x는 `isaacsim.core.*` 경로를 사용.
        # 4.x 호환을 위해 기존 omni.isaac.core 경로를 fallback으로 둔다.
        try:
            from isaacsim.core.utils.extensions import enable_extension
        except ModuleNotFoundError:
            from omni.isaac.core.utils.extensions import enable_extension

        bridge_extension = str(
            phase_cfg.get("bridge_extension", "isaacsim.ros2.bridge")
        )
        enable_extension(bridge_extension)

        import omni.graph.core as og

        robot_prim = _resolve_robot_prim(env, graph_cfg.get("robot_prim"))
        keys = og.Controller.Keys

        # 그래프 생성 + 노드 연결 + 기본 값 세팅
        (graph, _, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": evaluator_name},
            {
                keys.CREATE_NODES: [
                    ("Tick", tick_node_type),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ClockPub", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("JointStatePub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("OdomCompute", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("ImuPub", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ],
                keys.CONNECT: [
                    (f"Tick.{tick_output_port}", "ClockPub.inputs:execIn"),
                    (f"Tick.{tick_output_port}", "JointStatePub.inputs:execIn"),
                    (f"Tick.{tick_output_port}", "OdomCompute.inputs:execIn"),
                    (f"Tick.{tick_output_port}", "ImuPub.inputs:execIn"),
                    ("Context.outputs:context", "ClockPub.inputs:context"),
                    ("Context.outputs:context", "JointStatePub.inputs:context"),
                    ("Context.outputs:context", "ImuPub.inputs:context"),
                    ("ReadTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                    ("ReadTime.outputs:simulationTime", "ImuPub.inputs:timeStamp"),
                    ("OdomCompute.outputs:orientation", "ImuPub.inputs:orientation"),
                    (
                        "OdomCompute.outputs:angularVelocity",
                        "ImuPub.inputs:angularVelocity",
                    ),
                    (
                        "OdomCompute.outputs:linearAcceleration",
                        "ImuPub.inputs:linearAcceleration",
                    ),
                ],
                keys.SET_VALUES: [
                    ("JointStatePub.inputs:targetPrim", robot_prim),
                    ("JointStatePub.inputs:topicName", topic_joint),
                    ("OdomCompute.inputs:chassisPrim", robot_prim),
                    ("ImuPub.inputs:topicName", topic_imu),
                    ("ImuPub.inputs:frameId", frame_imu),
                    ("ImuPub.inputs:publishOrientation", True),
                    ("ImuPub.inputs:publishAngularVelocity", True),
                    ("ImuPub.inputs:publishLinearAcceleration", True),
                ],
            },
        )
        # OnTick일 때만 onlyPlayback/framePeriod 입력 포트가 존재
        if tick_node_type == "omni.graph.action.OnTick":
            og.Controller.set(
                f"{graph_path}/Tick.inputs:onlyPlayback", tick_only_playback
            )
            og.Controller.set(
                f"{graph_path}/Tick.inputs:framePeriod", tick_frame_period
            )
        og.Controller.evaluate_sync(graph)
        return (
            True,
            (
                f"{graph_path} robot_prim={robot_prim}"
                f" tick_node={tick_node_type} tick_output={tick_output_port} only_playback={tick_only_playback}"
                f" frame_period={tick_frame_period} evaluator={evaluator_name}"
            ),
        )
    except Exception as exc:
        note = f"graph_build_failed: {type(exc).__name__}: {exc}"
        if strict:
            raise RuntimeError(note) from exc
        return False, note
