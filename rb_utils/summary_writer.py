"""Summary/meta writers for stage runs."""

from __future__ import annotations

from pathlib import Path


def print_run_meta(meta: dict[str, str]) -> None:
    print("===== STAGE1 RUN META =====", flush=True)
    print(f"task               : {meta['task']}", flush=True)
    print(f"mode               : {meta['mode']}", flush=True)
    print(f"num_envs           : {meta['num_envs']}", flush=True)
    print(f"steps              : {meta['steps']}", flush=True)
    print(f"device             : {meta['device']}", flush=True)
    print(f"fabric             : {meta['fabric']}", flush=True)
    print(f"reset_on_done      : {meta['reset_on_done']}", flush=True)
    print(f"isaaclab_version   : {meta['isaaclab_version']}", flush=True)
    print(f"git_commit         : {meta['git_commit']}", flush=True)
    print("===========================", flush=True)


def write_stage_summary(
    summary_file: Path,
    run_id: str,
    raw_log_file: Path,
    meta: dict[str, str],
    stats: dict[str, str],
) -> None:
    lines = [
        "# Stage1 Rollout Summary",
        "",
        "## Run ID",
        f"- {run_id}",
        "",
        "## Metadata",
        f"- task: {meta['task']}",
        f"- mode: {meta['mode']}",
        f"- num_envs: {meta['num_envs']}",
        f"- steps: {meta['steps']}",
        f"- device: {meta['device']}",
        f"- fabric: {meta['fabric']}",
        f"- reset_on_done: {meta['reset_on_done']}",
        f"- isaaclab_version: {meta['isaaclab_version']}",
        f"- git_commit: {meta['git_commit']}",
        "",
        "## Outcome",
        f"- status: {stats['status']}",
        f"- executed_steps: {stats['executed_steps']}",
        f"- first_done_step: {stats['first_done_step']}",
        f"- episode_end_count: {stats['episode_end_count']}",
        f"- avg_reward_per_step: {stats['avg_reward_per_step']}",
        f"- elapsed_s: {stats['elapsed_s']}",
        f"- first_done_reason: {stats['first_done_reason']}",
        "",
        "## Artifacts",
        f"- raw_log: {raw_log_file}",
        "",
        "## Interpretation",
        f"- {stats['interpretation']}",
    ]
    summary_file.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[LOG] summary file -> {summary_file}", flush=True)

