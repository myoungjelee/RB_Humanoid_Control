"""Shared Isaac-side log marker helpers.

Keep structured log markers stable so existing tmux capture and KPI parsing
continue to work while the logging responsibility is centralized.
"""

from __future__ import annotations


def _emit(line: str) -> None:
    print(line, flush=True)


def log_config(message: str) -> None:
    _emit(f"[CONFIG] {message}")


def log_warning(message: str) -> None:
    _emit(f"[WARN] {message}")


def log_result(key: str, value: object) -> None:
    _emit(f"[RESULT] {key}={value}")


def log_exception(exc: BaseException, traceback_text: str) -> None:
    _emit(f"[ERROR] exception: {type(exc).__name__}: {exc}")
    _emit(traceback_text.rstrip())


def log_asset_source(asset_source: str) -> None:
    _emit(f"[ASSET] asset_source={asset_source}")


def log_timeline_state(*, is_playing: bool, start_paused: bool) -> None:
    _emit(f"[TIMELINE] is_playing={is_playing} start_paused={start_paused}")


def log_timeline_warning(exc: BaseException) -> None:
    _emit(f"[TIMELINE] warning: failed to control timeline: {type(exc).__name__}: {exc}")


def log_graph_result(*, built: bool, note: str) -> None:
    _emit(f"[GRAPH] built={built} note={note}")


def log_pose_audit_idle_loop() -> None:
    _emit("[POSE_AUDIT] render_only_idle_loop=true")


def log_rollout_start(*, target_text: str, progress_log_interval_sec: float) -> None:
    _emit(f"[RUNNING] phase_steps={target_text} progress_log_interval_sec={progress_log_interval_sec}")


def log_first_sim_step(*, sim_elapsed_sec: float) -> None:
    _emit(f"[SYNC] FIRST_SIM_STEP sim_elapsed_sec={sim_elapsed_sec:.3f}")


def log_stop_on_fall_event() -> None:
    _emit("[RUNNING] stop_on_fall_event=true -> stopping rollout")


def log_paused_heartbeat(*, elapsed_sec: float) -> None:
    _emit(f"[HEARTBEAT] paused elapsed_sec={elapsed_sec:.2f}")


def log_rollout_heartbeat(*, steps: int, target_text: str, elapsed_sec: float) -> None:
    _emit(f"[HEARTBEAT] steps={steps}/{target_text} elapsed_sec={elapsed_sec:.2f}")


def log_disturbance_sync(*, elapsed_sec: float, marker: str) -> None:
    _emit(
        "[DISTURBANCE_SYNC] "
        f"control_active_elapsed_sec={elapsed_sec:.3f} "
        f"marker={marker}"
    )


def log_disturbance_start(
    *,
    elapsed_sec: float,
    window_start_sec: float,
    body: str | None,
    frame_mode: str,
    configured_force_xyz: tuple[float, float, float],
    configured_torque_xyz: tuple[float, float, float],
    applied_force_xyz: tuple[float, float, float],
    applied_torque_xyz: tuple[float, float, float],
    is_global: bool,
) -> None:
    _emit(
        "[DISTURBANCE_START] "
        f"elapsed_sec={elapsed_sec:.3f} "
        f"window_start_sec={window_start_sec:.3f} "
        f"body={body} "
        f"frame_mode={frame_mode} "
        f"configured_force_xyz={configured_force_xyz} "
        f"configured_torque_xyz={configured_torque_xyz} "
        f"applied_force_xyz={applied_force_xyz} "
        f"applied_torque_xyz={applied_torque_xyz} "
        f"is_global={is_global}"
    )


def log_disturbance_end(*, elapsed_sec: float, window_end_sec: float, body: str | None) -> None:
    _emit(
        "[DISTURBANCE_END] "
        f"elapsed_sec={elapsed_sec:.3f} "
        f"window_end_sec={window_end_sec:.3f} "
        f"body={body}"
    )


def log_fall_event(
    *,
    elapsed_sec: float,
    trigger: str,
    pelvis_z: float,
    pelvis_threshold: float,
    torso_z_text: str,
    torso_threshold: float,
) -> None:
    _emit(
        "[FALL_EVENT] "
        f"elapsed_sec={elapsed_sec:.3f} "
        f"trigger={trigger} "
        f"pelvis_z={pelvis_z:.3f}/{pelvis_threshold:.3f} "
        f"torso_z={torso_z_text}/{torso_threshold:.3f}"
    )
