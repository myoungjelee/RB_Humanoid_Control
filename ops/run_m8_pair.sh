#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION_NAME="${SESSION_NAME:-rb_m8_disturb}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d-%H%M%S)_m8_disturb}"
CAPTURE_MAX_SEC="${CAPTURE_MAX_SEC:-120}"
POST_DIST_CAPTURE_SEC="${POST_DIST_CAPTURE_SEC:-15}"
AUTO_KILL_AFTER_CAPTURE="${AUTO_KILL_AFTER_CAPTURE:-1}"
SESSION_START_TIMEOUT_SEC="${SESSION_START_TIMEOUT_SEC:-15}"

BALANCE_BASE_SCENARIO="${BALANCE_BASE_SCENARIO:-$ROOT_DIR/ros2_ws/src/rb_controller/config/scenarios/stand_pd_balance_base.yaml}"

wait_for_session_start() {
  local waited=0
  until tmux has-session -t "$SESSION_NAME" 2>/dev/null; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge "$SESSION_START_TIMEOUT_SEC" ]; then
      echo "[ERROR] tmux session '$SESSION_NAME' did not start within ${SESSION_START_TIMEOUT_SEC}s" >&2
      return 1
    fi
  done
}

wait_for_session_end() {
  while tmux has-session -t "$SESSION_NAME" 2>/dev/null; do
    sleep 1
  done
}

run_once() {
  local label="$1"
  local enable_flag="$2"

  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "[ERROR] tmux session '$SESSION_NAME' is already running" >&2
    return 1
  fi

  echo "[RUN] M8 ${label} (RUN_ID=${RUN_ID})"
  (
    cd "$ROOT_DIR"
    RUN_ID="$RUN_ID" \
    RESULT_LABEL="$label" \
    SCENARIO_PATH="$BALANCE_BASE_SCENARIO" \
    ENABLE_TILT_FEEDBACK="$enable_flag" \
    CAPTURE_MAX_SEC="$CAPTURE_MAX_SEC" \
    POST_DIST_CAPTURE_SEC="$POST_DIST_CAPTURE_SEC" \
    AUTO_KILL_AFTER_CAPTURE="$AUTO_KILL_AFTER_CAPTURE" \
    tmuxp load -d -y ops/tmuxp/m8_disturb.yaml
  )
  wait_for_session_start
  wait_for_session_end
}

run_once "balance_off" "false"
run_once "balance_on" "true"

echo "[DONE] M8 pair complete"
echo "[INFO] logs: $ROOT_DIR/logs/sim2real/m8/$RUN_ID"
