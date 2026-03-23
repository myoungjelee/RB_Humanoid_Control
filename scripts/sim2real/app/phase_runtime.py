"""Common Isaac phase runtime utilities.

This module keeps the existing parameter ingress unchanged:
`main.py` resolves `phase_cfg` once, then passes it through the phase runtime.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from .telemetry import (
    log_asset_source,
    log_first_sim_step,
    log_graph_result,
    log_paused_heartbeat,
    log_pose_audit_idle_loop,
    log_rollout_heartbeat,
    log_rollout_start,
    log_stop_on_fall_event,
    log_timeline_state,
    log_timeline_warning,
)
from .world import create_standalone_world

GraphBuilder = Callable[[Any, dict[str, Any]], tuple[bool, str]]


def prepare_phase_runtime(args_cli: Any, phase_cfg: dict[str, Any]) -> None:
    """Inject runtime options right before phase execution."""
    if bool(phase_cfg.get("enable_ros2_bridge", False)):
        ext = str(phase_cfg.get("bridge_extension", "isaacsim.ros2.bridge"))
        args_cli.kit_args = f"{args_cli.kit_args} --enable {ext}".strip()


def _configure_timeline(*, start_paused: bool) -> None:
    """Match Isaac timeline state to the current phase mode."""
    try:
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        if start_paused:
            if timeline.is_playing():
                timeline.pause()
        elif not timeline.is_playing():
            timeline.play()
        log_timeline_state(is_playing=timeline.is_playing(), start_paused=start_paused)
    except Exception as exc:
        log_timeline_warning(exc)


def _run_paused_loop(
    simulation_app: Any,
    *,
    t0: float,
    progress_log_interval_sec: float,
) -> None:
    """Run a render-only idle loop for paused pose-audit phases."""
    log_pose_audit_idle_loop()
    update_app = getattr(simulation_app, "update", None)
    last_progress_log_sec = t0
    while simulation_app.is_running():
        if callable(update_app):
            update_app()
        else:
            time.sleep(1.0 / 30.0)
        now_sec = time.time()
        if progress_log_interval_sec > 0.0 and (now_sec - last_progress_log_sec) >= progress_log_interval_sec:
            log_paused_heartbeat(elapsed_sec=now_sec - t0)
            last_progress_log_sec = now_sec


def _run_active_rollout(
    simulation_app: Any,
    world: Any,
    *,
    target_steps: int,
    stop_on_fall_event: bool,
    progress_log_interval_sec: float,
) -> tuple[int, bool]:
    """Run the common stepped rollout loop."""
    infinite_steps = target_steps <= 0
    executed_steps = 0
    fall_event_triggered = False
    t0 = time.time()
    last_progress_log_sec = t0
    target_text = "INF" if infinite_steps else str(target_steps)
    log_rollout_start(target_text=target_text, progress_log_interval_sec=progress_log_interval_sec)

    while simulation_app.is_running() and (infinite_steps or executed_steps < target_steps):
        world.step(render=True)
        executed_steps += 1
        if executed_steps == 1:
            log_first_sim_step(sim_elapsed_sec=executed_steps * world.sim_dt)

        maybe_emit_fall_event = getattr(world, "maybe_emit_fall_event", None)
        if callable(maybe_emit_fall_event):
            fall_event_triggered = bool(maybe_emit_fall_event()) or fall_event_triggered
            if fall_event_triggered and stop_on_fall_event:
                log_stop_on_fall_event()
                break

        now_sec = time.time()
        if progress_log_interval_sec > 0.0 and (now_sec - last_progress_log_sec) >= progress_log_interval_sec:
            log_rollout_heartbeat(steps=executed_steps, target_text=target_text, elapsed_sec=now_sec - t0)
            last_progress_log_sec = now_sec

    return executed_steps, fall_event_triggered


def run_rollout_phase(
    args_cli: Any,
    phase_cfg: dict[str, Any],
    simulation_app: Any,
    *,
    graph_builder: GraphBuilder | None,
) -> dict[str, Any]:
    """Execute the common standalone Isaac rollout flow."""
    task_id, world, asset_source = create_standalone_world(args_cli, phase_cfg)

    status = "failed_before_run"
    executed_steps = 0
    elapsed_sec = 0.0
    graph_built = False
    graph_note = "not_attempted"

    try:
        if asset_source:
            log_asset_source(asset_source)

        start_paused = bool(phase_cfg.get("start_paused", False))
        graph_cfg = phase_cfg.get("graph_builder", {})
        graph_enabled = bool(graph_cfg.get("enabled", True))
        _configure_timeline(start_paused=start_paused)

        if graph_enabled and callable(graph_builder):
            graph_built, graph_note = graph_builder(env=world, phase_cfg=phase_cfg)
        else:
            graph_built, graph_note = False, "graph_builder_disabled"
        log_graph_result(built=graph_built, note=graph_note)

        t0 = time.time()
        target_steps = int(phase_cfg.get("steps", -1))
        infinite_steps = target_steps <= 0
        stop_on_fall_event = bool(phase_cfg.get("stop_on_fall_event", False))
        progress_log_interval_sec = float(phase_cfg.get("progress_log_interval_sec", 10.0))
        if progress_log_interval_sec < 0.0:
            progress_log_interval_sec = 10.0

        if start_paused:
            _run_paused_loop(
                simulation_app,
                t0=t0,
                progress_log_interval_sec=progress_log_interval_sec,
            )
            fall_event_triggered = False
        else:
            executed_steps, fall_event_triggered = _run_active_rollout(
                simulation_app,
                world,
                target_steps=target_steps,
                stop_on_fall_event=stop_on_fall_event,
                progress_log_interval_sec=progress_log_interval_sec,
            )

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
