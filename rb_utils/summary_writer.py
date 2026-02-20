"""스테이지 실행 메타데이터/요약 리포트 작성 유틸리티."""

from __future__ import annotations

from pathlib import Path


def print_run_meta(meta: dict[str, str]) -> None:
    """실행 시작 시 핵심 메타데이터를 콘솔에 정해진 포맷으로 출력한다."""
    print("===== STAGE1 RUN META =====", flush=True)
    print(f"task               : {meta['task']}", flush=True)
    print(f"mode               : {meta['mode']}", flush=True)
    print(f"num_envs           : {meta['num_envs']}", flush=True)
    print(f"steps              : {meta['steps']}", flush=True)
    print(f"device             : {meta['device']}", flush=True)
    print(f"fabric             : {meta['fabric']}", flush=True)
    print(f"reset_on_done      : {meta['reset_on_done']}", flush=True)
    print(f"isaaclab_version   : {meta['isaaclab_version']}", flush=True)
    print("===========================", flush=True)


def write_stage_summary(
    summary_file: Path,
    run_id: str,
    raw_log_file: Path,
    meta: dict[str, str],
    stats: dict[str, str],
) -> None:
    """실행 결과를 마크다운 요약 파일로 저장한다.

    필수 통계 외에 first-done 상세 키가 있으면 선택적으로 추가 기록한다.
    """
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
        "",
        "## Outcome",
        f"- status: {stats['status']}",
        f"- executed_steps: {stats['executed_steps']}",
        f"- first_done_step: {stats['first_done_step']}",
        f"- episode_end_count: {stats['episode_end_count']}",
        f"- avg_reward_per_step: {stats['avg_reward_per_step']}",
        f"- elapsed_s: {stats['elapsed_s']}",
        f"- first_done_reason: {stats['first_done_reason']}",
    ]
    if "first_done_flags" in stats:
        lines.append(f"- first_done_flags: {stats['first_done_flags']}")
    if "first_done_terms" in stats:
        lines.append(f"- first_done_terms: {stats['first_done_terms']}")
    if "first_done_state" in stats:
        lines.append(f"- first_done_state: {stats['first_done_state']}")
    if "first_done_info" in stats:
        lines.append(f"- first_done_info: {stats['first_done_info']}")
    lines.extend(
        [
            "",
            "## Artifacts",
            f"- raw_log: {raw_log_file}",
            "",
            "## Interpretation",
            f"- {stats['interpretation']}",
        ]
    )
    summary_file.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[LOG] summary file -> {summary_file}", flush=True)
