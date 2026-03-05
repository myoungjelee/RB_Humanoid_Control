"""Sim2Real phase 실행 로직.

현재 구현 phase:
- M1(sensor publish 확인)
- M3(command apply 확인)
학습/분석 우선: 자동 로그/자동 캡처는 포함하지 않는다.
"""

from __future__ import annotations

import time
from typing import Any

from rb_utils.termination_utils import as_any_bool

from .graph_builder import build_m1_sensor_graph, build_m3_command_graph
from .world import create_env


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
    graph_builder: Any,
) -> dict[str, Any]:
    """공통 rollout phase 실행기.

    공통 흐름:
    1) env reset
    2) OmniGraph 구성
    3) rollout step 진행
    """
    import torch

    task_id, env, robot_usd_path = create_env(args_cli, phase_cfg)

    status = "failed_before_run"
    executed_steps = 0
    elapsed_sec = 0.0
    graph_built = False
    graph_note = "not_attempted"

    try:
        if robot_usd_path:
            print(f"[ASSET] robot_usd_path={robot_usd_path}", flush=True)

        # SpotATS 패턴: 타임라인 재생을 명시적으로 켠다.
        try:
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            if not timeline.is_playing():
                timeline.play()
            print(f"[TIMELINE] is_playing={timeline.is_playing()}", flush=True)
        except Exception as exc:
            print(f"[TIMELINE] warning: failed to control timeline: {type(exc).__name__}: {exc}", flush=True)

        _, _ = env.reset()
        graph_built, graph_note = graph_builder(env=env, phase_cfg=phase_cfg)
        print(f"[GRAPH] built={graph_built} note={graph_note}", flush=True)

        unwrapped = getattr(env, "unwrapped", None)
        device = getattr(unwrapped, "device", "cpu")
        action_shape = env.action_space.shape
        if not isinstance(action_shape, tuple):
            raise RuntimeError("Invalid env.action_space.shape")

        actions = torch.zeros(action_shape, device=device)
        t0 = time.time()
        target_steps = int(phase_cfg.get("steps", -1))
        infinite_steps = target_steps <= 0
        progress_log_interval_sec = float(phase_cfg.get("progress_log_interval_sec", 10.0))
        if progress_log_interval_sec < 0.0:
            progress_log_interval_sec = 10.0
        target_text = "INF" if infinite_steps else str(target_steps)
        print(
            f"[RUNNING] phase_steps={target_text} progress_log_interval_sec={progress_log_interval_sec}",
            flush=True,
        )
        last_progress_log_sec = t0
        sim = getattr(env.unwrapped, "sim", None)

        while simulation_app.is_running() and (infinite_steps or executed_steps < target_steps):
            with torch.no_grad():
                _, _, terminated, truncated, _ = env.step(actions)

            # SpotATS의 world.step(render=True)와 같은 효과를 주기 위해 렌더를 강제한다.
            # OnTick/OnPlaybackTick 기반 ROS2 graph가 헤드리스에서도 tick되도록 보조한다.
            if sim is not None and hasattr(sim, "render"):
                sim.render()

            executed_steps += 1
            now_sec = time.time()
            if progress_log_interval_sec > 0.0 and (now_sec - last_progress_log_sec) >= progress_log_interval_sec:
                print(
                    f"[HEARTBEAT] steps={executed_steps}/{target_text} elapsed_sec={now_sec - t0:.2f}",
                    flush=True,
                )
                last_progress_log_sec = now_sec

            done = as_any_bool(terminated) or as_any_bool(truncated)
            if done and bool(phase_cfg.get("reset_on_done", True)):
                _, _ = env.reset()
                actions = torch.zeros(action_shape, device=device)
            elif done:
                print(f"[DONE] episode ended at step={executed_steps}", flush=True)
                break

        elapsed_sec = time.time() - t0
        if infinite_steps:
            status = "stopped_no_step_limit"
        else:
            status = "completed_target_steps" if executed_steps >= target_steps else "stopped_early"
    finally:
        env.close()

    return {
        "task_id": task_id,
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
