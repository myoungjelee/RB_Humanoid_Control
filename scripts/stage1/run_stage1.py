#!/usr/bin/env python3
"""Stage1 실행 스크립트 (zero/pose 모드)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from isaaclab.app import AppLauncher

from rb_utils.action_generators import extract_policy_obs, make_actions
from rb_utils.experiment import (
    attach_kit_log_file,
    ensure_stage_dirs,
    make_run_id,
    set_kit_log_dir_env,
)
from rb_utils.experiment_runner import get_unwrapped_num_envs, run_rollout
from rb_utils.summary_writer import print_run_meta, write_stage_summary

RAW_LOG_DIR, SUMMARY_DIR = ensure_stage_dirs(PROJECT_ROOT, "stage1")
set_kit_log_dir_env(RAW_LOG_DIR)

parser = argparse.ArgumentParser(description="Stage1 롤아웃 실행 스크립트.")
parser.add_argument("--task", choices=["flat", "rough"], default="flat")
parser.add_argument("--mode", choices=["zero", "pose"], default="zero")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=200)
parser.add_argument("--disable_fabric", action="store_true", default=False)
parser.add_argument(
    "--reset_on_done",
    action="store_true",
    default=False,
    help="설정 시 done 이후 reset하고 계속 실행. 기본값은 첫 done에서 종료.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


def _task_id(short_name: str) -> str:
    """짧은 태스크 이름(flat/rough)을 IsaacLab env id로 변환한다."""
    if short_name == "flat":
        return "RB-Stage1-G1-Flat-Stand-v0"
    return "RB-Stage1-G1-Rough-Stand-v0"


def _center_view_once(env) -> None:
    """reset 직후 카메라를 env0 로봇 근처로 1회 이동한다.

    viewer 설정을 고정하지 않고도 rough 지형에서 초기 가시성을 확보하기 위한 보조 함수다.
    """
    unwrapped = getattr(env, "unwrapped", env)
    scene = getattr(unwrapped, "scene", None)
    sim = getattr(unwrapped, "sim", None)
    if scene is None or sim is None:
        return

    robot = None
    for key in ("robot", "humanoid", "g1"):
        try:
            robot = scene[key]
            break
        except Exception:
            candidate = getattr(scene, key, None)
            if candidate is not None:
                robot = candidate
                break
    if robot is None:
        return

    root_pos = getattr(getattr(robot, "data", None), "root_pos_w", None)
    if root_pos is None:
        return

    try:
        p0 = root_pos[0]
        if hasattr(p0, "detach"):
            p0 = p0.detach()
        if hasattr(p0, "cpu"):
            p0 = p0.cpu()
        if hasattr(p0, "tolist"):
            p0 = p0.tolist()
        x, y, z = float(p0[0]), float(p0[1]), float(p0[2])
        # 3인칭 시점: 로봇 중심을 보되 좌측 상단 사선에서 내려다보는 구도
        sim.set_camera_view(eye=(x + 5, y + 5, z + 5), target=(x, y, z + 0.3))
        print(
            f"[CAM] centered once to env0 robot at ({x:.2f}, {y:.2f}, {z:.2f})"
            " with wider view",
            flush=True,
        )
    except Exception as exc:
        print("[CAM] center failed:", repr(exc), flush=True)


def main():
    """Stage1 실험 1회를 실행하고 요약 파일을 생성한다."""
    import gymnasium as gym

    env_id = _task_id(args_cli.task)
    run_id = make_run_id(args_cli.task, args_cli.mode, args_cli.steps)
    raw_log_file = RAW_LOG_DIR / f"{run_id}.log"
    summary_file = SUMMARY_DIR / f"{run_id}_summary.md"

    attach_kit_log_file(args_cli, raw_log_file)
    print(f"[LOG] kit log file -> {raw_log_file}", flush=True)

    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    run_meta: dict[str, str] = {
        "task": env_id,
        "mode": args_cli.mode,
        "num_envs": str(args_cli.num_envs),
        "steps": str(args_cli.steps),
        "device": str(getattr(args_cli, "device", "unknown")),
        "fabric": str(not args_cli.disable_fabric),
        "reset_on_done": str(args_cli.reset_on_done),
        "isaaclab_version": "unknown",
    }
    run_stats: dict[str, str] = {
        "status": "failed_before_run",
        "executed_steps": "0",
        "first_done_step": "none",
        "episode_end_count": "0",
        "avg_reward_per_step": "0.000000",
        "elapsed_s": "0.000",
        "first_done_reason": "unknown",
        "interpretation": "run did not reach rollout loop.",
    }
    summary_written = False

    try:
        # 중요: Isaac/Omniverse 모듈은 SimulationApp 실행 이후에 import 해야 한다.
        import RB_Humanoid_Control.tasks  # noqa: F401

        import isaaclab

        import isaaclab_tasks  # noqa: F401
        from isaaclab_tasks.utils import parse_env_cfg

        env_cfg = parse_env_cfg(
            env_id,
            device=args_cli.device,
            num_envs=args_cli.num_envs,
            use_fabric=not args_cli.disable_fabric,
        )
        env_cfg.sim.log_dir = str(RAW_LOG_DIR)
        env_cfg.sim.save_logs_to_file = False

        env = gym.make(env_id, cfg=env_cfg)
        obs, info = env.reset()
        print(
            f"[OK] reset: {env_id} num_envs={get_unwrapped_num_envs(env)}", flush=True
        )
        if not getattr(args_cli, "headless", False):
            _center_view_once(env)

        run_meta = {
            "task": env_id,
            "mode": args_cli.mode,
            "num_envs": str(args_cli.num_envs),
            "steps": str(args_cli.steps),
            "device": str(getattr(env.unwrapped, "device", "unknown")),
            "fabric": str(not args_cli.disable_fabric),
            "reset_on_done": str(args_cli.reset_on_done),
            "isaaclab_version": getattr(isaaclab, "__version__", "unknown"),
        }
        print_run_meta(run_meta)

        try:
            policy_obs = extract_policy_obs(obs)
            vc = policy_obs[0, 9:12].tolist()
            print(f"[CHK] velocity_commands[0] = {vc}", flush=True)
        except Exception as exc:
            print("[CHK] velocity_commands read failed:", repr(exc), flush=True)

        stats_raw = run_rollout(
            env=env,
            simulation_app=simulation_app,
            steps=args_cli.steps,
            mode=args_cli.mode,
            reset_on_done=args_cli.reset_on_done,
            initial_obs=obs,
            initial_info=info,
            action_fn=make_actions,
        )
        env.close()
        print("[OK] finish", flush=True)

        if stats_raw["executed_steps"] >= args_cli.steps:
            status = "completed_target_steps"
        elif stats_raw["first_done_step"] != "none" and not args_cli.reset_on_done:
            status = "stopped_on_first_done"
        else:
            status = "stopped_before_target_steps"

        if stats_raw["first_done_step"] != "none":
            interpretation = (
                f"{args_cli.mode} baseline terminated first at global step "
                f"{stats_raw['first_done_step']} (reason={stats_raw['first_done_reason']})."
            )
        elif stats_raw["executed_steps"] >= args_cli.steps:
            interpretation = f"{args_cli.mode} baseline sustained for all {stats_raw['executed_steps']} steps."
        else:
            interpretation = f"run stopped early at {stats_raw['executed_steps']} steps (app not running)."

        run_stats = {
            "status": status,
            "executed_steps": str(stats_raw["executed_steps"]),
            "first_done_step": str(stats_raw["first_done_step"]),
            "episode_end_count": str(stats_raw["episode_end_count"]),
            "avg_reward_per_step": f"{stats_raw['avg_reward_per_step']:.6f}",
            "elapsed_s": f"{stats_raw['elapsed_s']:.3f}",
            "first_done_reason": str(stats_raw["first_done_reason"]),
            "first_done_flags": str(stats_raw["first_done_flags"]),
            "first_done_terms": str(stats_raw["first_done_terms"]),
            "first_done_state": str(stats_raw["first_done_state"]),
            "first_done_info": str(stats_raw["first_done_info"]),
            "interpretation": interpretation,
        }
        write_stage_summary(summary_file, run_id, raw_log_file, run_meta, run_stats)
        summary_written = True

    except KeyboardInterrupt:
        run_stats["status"] = "interrupted"
        run_stats["interpretation"] = "run interrupted by user."
        if not summary_written:
            write_stage_summary(summary_file, run_id, raw_log_file, run_meta, run_stats)
            summary_written = True
        raise

    except Exception as exc:
        run_stats["status"] = "error"
        run_stats["first_done_reason"] = type(exc).__name__
        run_stats["interpretation"] = f"run failed with {type(exc).__name__}: {exc}"
        if not summary_written:
            write_stage_summary(summary_file, run_id, raw_log_file, run_meta, run_stats)
            summary_written = True
        raise

    finally:
        if not summary_written:
            write_stage_summary(summary_file, run_id, raw_log_file, run_meta, run_stats)
        simulation_app.close()


if __name__ == "__main__":
    main()
