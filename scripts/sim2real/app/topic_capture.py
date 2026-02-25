"""M1 토픽 증빙 캡처 유틸.

런 도중에 ros2 CLI를 호출해 다음 파일을 자동 생성한다:
- topic_list.txt
- topic_hz_joint_states.txt
- topic_echo_imu.txt
- topic_echo_clock.txt
- topic_echo_joint_states_once.txt
- topic_capture.log
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path


def _to_text(data: object) -> str:
    """subprocess 결과(stdout/stderr)가 bytes여도 안전하게 문자열로 변환."""
    if data is None:
        return ""
    if isinstance(data, bytes):
        return data.decode("utf-8", errors="replace")
    if isinstance(data, str):
        return data
    return str(data)


def _run_command(cmd: list[str], timeout_sec: float) -> tuple[int, str]:
    """외부 명령 실행 래퍼.

    timeout 발생 시 예외를 밖으로 던지지 않고 rc=124와 로그 문자열로 반환한다.
    """
    try:
        completed = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
        return int(completed.returncode), _to_text(completed.stdout)
    except subprocess.TimeoutExpired as exc:
        output = _to_text(exc.stdout) + _to_text(exc.stderr)
        output += f"\n[TIMEOUT] command timed out after {timeout_sec:.1f}s\n"
        return 124, output
    except Exception as exc:  # pragma: no cover - defensive path for tooling/env issues
        return 1, f"[ERROR] {type(exc).__name__}: {exc}\n"


def _write_text(path: Path, text: str) -> None:
    """텍스트 파일 저장 헬퍼."""
    path.write_text(text, encoding="utf-8")


def capture_m1_topics(
    run_dir: Path,
    joint_topic: str,
    imu_topic: str,
    clock_topic: str,
    wait_timeout_sec: float = 20.0,
    poll_interval_sec: float = 1.0,
    hz_timeout_sec: float = 5.0,
    echo_timeout_sec: float = 5.0,
) -> dict[str, object]:
    """M1 증빙 파일을 run_dir에 저장한다.

    순서:
    1) topic list 폴링으로 토픽 생성 대기
    2) hz/echo 측정
    3) 결과 로그(topic_capture.log) 작성
    """
    run_dir.mkdir(parents=True, exist_ok=True)

    topic_list_out = run_dir / "topic_list.txt"
    hz_out = run_dir / "topic_hz_joint_states.txt"
    imu_echo_out = run_dir / "topic_echo_imu.txt"
    clock_echo_out = run_dir / "topic_echo_clock.txt"
    joint_echo_out = run_dir / "topic_echo_joint_states_once.txt"
    capture_log_out = run_dir / "topic_capture.log"

    t0 = time.time()
    latest_list_output = ""
    expected = {joint_topic, imu_topic, clock_topic}
    found = set()

    while time.time() - t0 < wait_timeout_sec:
        rc, output = _run_command(["ros2", "topic", "list"], timeout_sec=3.0)
        latest_list_output = output
        if rc == 0:
            found = {line.strip() for line in output.splitlines() if line.strip()}
            if expected.issubset(found):
                break
        time.sleep(max(poll_interval_sec, 0.1))

    if latest_list_output:
        listed = sorted(line.strip() for line in latest_list_output.splitlines() if line.strip())
        _write_text(topic_list_out, "\n".join(listed) + ("\n" if listed else ""))
    else:
        _write_text(topic_list_out, "")

    hz_rc, hz_text = _run_command(["ros2", "topic", "hz", joint_topic], timeout_sec=hz_timeout_sec)
    _write_text(hz_out, hz_text)

    imu_rc, imu_text = _run_command(["ros2", "topic", "echo", "--once", imu_topic], timeout_sec=echo_timeout_sec)
    _write_text(imu_echo_out, imu_text)

    clock_rc, clock_text = _run_command(
        ["ros2", "topic", "echo", "--once", clock_topic], timeout_sec=echo_timeout_sec
    )
    _write_text(clock_echo_out, clock_text)

    joint_rc, joint_text = _run_command(
        ["ros2", "topic", "echo", "--once", joint_topic], timeout_sec=echo_timeout_sec
    )
    _write_text(joint_echo_out, joint_text)

    # 토픽 존재 + 각 측정 명령 성공 시 capture_ok=True
    all_ok = expected.issubset(found) and hz_rc == 0 and imu_rc == 0 and clock_rc == 0 and joint_rc == 0
    note = "ok" if all_ok else "partial_or_failed"

    capture_log = [
        f"joint_topic: {joint_topic}",
        f"imu_topic: {imu_topic}",
        f"clock_topic: {clock_topic}",
        f"wait_timeout_sec: {wait_timeout_sec}",
        f"found_topics: {sorted(found)}",
        f"topic_ready: {expected.issubset(found)}",
        f"hz_rc: {hz_rc}",
        f"imu_echo_rc: {imu_rc}",
        f"clock_echo_rc: {clock_rc}",
        f"joint_echo_rc: {joint_rc}",
        f"status: {note}",
    ]
    _write_text(capture_log_out, "\n".join(capture_log) + "\n")

    return {
        "capture_ok": all_ok,
        "capture_note": note,
        "capture_outputs": [
            str(topic_list_out),
            str(hz_out),
            str(imu_echo_out),
            str(clock_echo_out),
            str(joint_echo_out),
            str(capture_log_out),
        ],
    }
