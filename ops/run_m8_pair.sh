#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE_PATH="${PROFILE_PATH:-$ROOT_DIR/ops/profiles/m8_disturb.env}"
set -a
source "$PROFILE_PATH"
set +a

SESSION_NAME="${SESSION_NAME:-$SESSION_NAME_DEFAULT}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d-%H%M%S)_m8_disturb}"
CAPTURE_MAX_SEC="${CAPTURE_MAX_SEC:-$CAPTURE_MAX_SEC_DEFAULT}"
POST_DIST_CAPTURE_SEC="${POST_DIST_CAPTURE_SEC:-$POST_DIST_CAPTURE_SEC_DEFAULT}"
POST_FALL_CAPTURE_SEC="${POST_FALL_CAPTURE_SEC:-$POST_FALL_CAPTURE_SEC_DEFAULT}"
AUTO_KILL_AFTER_CAPTURE="${AUTO_KILL_AFTER_CAPTURE:-$AUTO_KILL_AFTER_CAPTURE_DEFAULT}"
SESSION_START_TIMEOUT_SEC="${SESSION_START_TIMEOUT_SEC:-15}"
STACK_PARAMS_PATH="${STACK_PARAMS_PATH:-$STACK_PARAMS_PATH_DEFAULT}"
CONTROLLERS_PATH="${CONTROLLERS_PATH:-$CONTROLLERS_PATH_DEFAULT}"

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
    ENABLE_TILT_FEEDBACK="$enable_flag" \
    CONTROLLERS_PATH="$CONTROLLERS_PATH" \
    STACK_PARAMS_PATH="$STACK_PARAMS_PATH" \
    CAPTURE_MAX_SEC="$CAPTURE_MAX_SEC" \
    POST_DIST_CAPTURE_SEC="$POST_DIST_CAPTURE_SEC" \
    POST_FALL_CAPTURE_SEC="$POST_FALL_CAPTURE_SEC" \
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
