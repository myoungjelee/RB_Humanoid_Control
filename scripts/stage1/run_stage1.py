#!/usr/bin/env python3
"""Stage1 runner (zero/pose modes)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from isaaclab.app import AppLauncher

from rb_utils.experiment import (
    attach_kit_log_file,
    ensure_stage_dirs,
    make_run_id,
    safe_git_commit,
    set_kit_log_dir_env,
)
from rb_utils.rollout_runner import extract_policy_obs, get_unwrapped_num_envs, run_rollout
from rb_utils.summary_writer import print_run_meta, write_stage_summary

RAW_LOG_DIR, SUMMARY_DIR = ensure_stage_dirs(PROJECT_ROOT, "stage1")
set_kit_log_dir_env(RAW_LOG_DIR)

parser = argparse.ArgumentParser(description="Stage1 rollout script.")
parser.add_argument("--task", choices=["flat", "rough"], default="flat")
parser.add_argument("--mode", choices=["zero", "pose"], default="zero")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=200)
parser.add_argument("--disable_fabric", action="store_true", default=False)
parser.add_argument(
    "--reset_on_done",
    action="store_true",
    default=False,
    help="If set, keep running by resetting after done. Default is to stop on first done.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


def _task_id(short_name: str) -> str:
    if short_name == "flat":
        return "RB-Stage1-G1-Flat-Stand-v0"
    return "RB-Stage1-G1-Rough-Stand-v0"


def main():
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
        "git_commit": safe_git_commit(PROJECT_ROOT),
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
        # IMPORTANT: Isaac/Omniverse modules must be imported after SimulationApp launch.
        import isaaclab
        import isaaclab_tasks  # noqa: F401
        from isaaclab_tasks.utils import parse_env_cfg

        import RB_Humanoid_Control.tasks  # noqa: F401

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
        print(f"[OK] reset: {env_id} num_envs={get_unwrapped_num_envs(env)}", flush=True)

        run_meta = {
            "task": env_id,
            "mode": args_cli.mode,
            "num_envs": str(args_cli.num_envs),
            "steps": str(args_cli.steps),
            "device": str(getattr(env.unwrapped, "device", "unknown")),
            "fabric": str(not args_cli.disable_fabric),
            "reset_on_done": str(args_cli.reset_on_done),
            "isaaclab_version": getattr(isaaclab, "__version__", "unknown"),
            "git_commit": safe_git_commit(PROJECT_ROOT),
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

