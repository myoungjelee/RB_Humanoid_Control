"""Sim2Real phase 실행 로직.

현재 구현 phase:
- M1(sensor publish 확인)
- M3(command apply 확인)
- M5(stand: /rb/command_safe 적용 확인)
메인 ROS2 트랙은 standalone `World.step()` backend를 사용한다.
"""

from __future__ import annotations

import time
from typing import Any

from .graph_builder import build_m1_sensor_graph, build_m3_command_graph
from .world import create_standalone_world


def prepare_phase_runtime(args_cli: Any, phase_cfg: dict[str, Any]) -> None:
    """phase 실행 직전 런타임 세팅.

    현재는 ROS2 bridge extension 활성화 인자만 주입한다.
    """
    if bool(phase_cfg.get("enable_ros2_bridge", False)):
        ext = str(phase_cfg.get("bridge_extension", "isaacsim.ros2.bridge"))
        args_cli.kit_args = f"{args_cli.kit_args} --enable {ext}".strip()


def _run_rollout_phase(
    args_cli: Any,
    phase_cfg: dict[str, Any],
    simulation_app: Any,
    *,
    graph_builder: Any | None,
) -> dict[str, Any]:
    """공통 rollout phase 실행기.

    공통 흐름:
    1) standalone direct spawn scene 구성
    2) OmniGraph 구성
    3) `World.step()` 진행
    """
    task_id, world, asset_source = create_standalone_world(args_cli, phase_cfg)

    status = "failed_before_run"
    executed_steps = 0
    elapsed_sec = 0.0
    graph_built = False
    graph_note = "not_attempted"

    try:
        if asset_source:
            print(f"[ASSET] asset_source={asset_source}", flush=True)

        start_paused = bool(phase_cfg.get("start_paused", False))
        graph_cfg = phase_cfg.get("graph_builder", {})
        graph_enabled = bool(graph_cfg.get("enabled", True))

        # SpotATS 패턴: 타임라인 재생을 명시적으로 켠다.
        try:
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            if start_paused:
                if timeline.is_playing():
                    timeline.pause()
            elif not timeline.is_playing():
                timeline.play()
            print(
                f"[TIMELINE] is_playing={timeline.is_playing()} start_paused={start_paused}",
                flush=True,
            )
        except Exception as exc:
            print(f"[TIMELINE] warning: failed to control timeline: {type(exc).__name__}: {exc}", flush=True)

        if graph_enabled and callable(graph_builder):
            graph_built, graph_note = graph_builder(env=world, phase_cfg=phase_cfg)
        else:
            graph_built, graph_note = False, "graph_builder_disabled"
        print(f"[GRAPH] built={graph_built} note={graph_note}", flush=True)

        t0 = time.time()
        target_steps = int(phase_cfg.get("steps", -1))
        infinite_steps = target_steps <= 0
        stop_on_fall_event = bool(phase_cfg.get("stop_on_fall_event", False))
        fall_event_triggered = False
        progress_log_interval_sec = float(phase_cfg.get("progress_log_interval_sec", 10.0))
        if progress_log_interval_sec < 0.0:
            progress_log_interval_sec = 10.0
        target_text = "INF" if infinite_steps else str(target_steps)
        print(
            f"[RUNNING] phase_steps={target_text} progress_log_interval_sec={progress_log_interval_sec}",
            flush=True,
        )
        last_progress_log_sec = t0

        if start_paused:
            print("[POSE_AUDIT] render_only_idle_loop=true", flush=True)
            update_app = getattr(simulation_app, "update", None)
            while simulation_app.is_running():
                if callable(update_app):
                    update_app()
                else:
                    time.sleep(1.0 / 30.0)
                now_sec = time.time()
                if progress_log_interval_sec > 0.0 and (now_sec - last_progress_log_sec) >= progress_log_interval_sec:
                    print(f"[HEARTBEAT] paused elapsed_sec={now_sec - t0:.2f}", flush=True)
                    last_progress_log_sec = now_sec
        else:
            while simulation_app.is_running() and (infinite_steps or executed_steps < target_steps):
                world.step(render=True)
                executed_steps += 1
                if executed_steps == 1:
                    print(
                        f"[SYNC] FIRST_SIM_STEP sim_elapsed_sec={executed_steps * world.sim_dt:.3f}",
                        flush=True,
                    )
                maybe_emit_fall_event = getattr(world, "maybe_emit_fall_event", None)
                if callable(maybe_emit_fall_event):
                    fall_event_triggered = bool(maybe_emit_fall_event()) or fall_event_triggered
                    if fall_event_triggered and stop_on_fall_event:
                        print("[RUNNING] stop_on_fall_event=true -> stopping rollout", flush=True)
                        break
                now_sec = time.time()
                if progress_log_interval_sec > 0.0 and (now_sec - last_progress_log_sec) >= progress_log_interval_sec:
                    print(
                        f"[HEARTBEAT] steps={executed_steps}/{target_text} elapsed_sec={now_sec - t0:.2f}",
                        flush=True,
                    )
                    last_progress_log_sec = now_sec

        elapsed_sec = time.time() - t0
        if start_paused:
            status = "stopped_gui_closed"
        elif fall_event_triggered and stop_on_fall_event:
            status = "stopped_on_fall_event"
        elif infinite_steps:
            status = "stopped_no_step_limit"
        else:
            status = "completed_target_steps" if executed_steps >= target_steps else "stopped_early"
    finally:
        world.close()

    return {
        "task_id": task_id,
        "backend": "standalone_world",
        "status": status,
        "executed_steps": executed_steps,
        "elapsed_sec": elapsed_sec,
        "graph_built": graph_built,
        "graph_note": graph_note,
    }


def run_m1_sensor_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M1 phase 실행(/clock, /rb/joint_states, /rb/imu publish 확인)."""
    return _run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=build_m1_sensor_graph,
    )


def run_m3_command_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M3 phase 실행(/rb/command_raw -> articulation 적용 검증)."""
    return _run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=build_m3_command_graph,
    )


def run_m5_stand_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M5 phase 실행(/rb/command_safe -> articulation 적용 확인)."""
    return _run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=build_m3_command_graph,
    )


def run_m5_pose_audit_phase(args_cli: Any, phase_cfg: dict[str, Any], simulation_app: Any) -> dict[str, Any]:
    """M5 pose audit 실행(paused GUI에서 nominal pose를 확인)."""
    return _run_rollout_phase(
        args_cli=args_cli,
        phase_cfg=phase_cfg,
        simulation_app=simulation_app,
        graph_builder=None,
    )
