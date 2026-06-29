#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${PYTHON_BIN:-}" ]]; then
  if [[ -x "/home/leemou/IsaacLab/isaaclab.sh" ]]; then
    PYTHON_BIN="/home/leemou/IsaacLab/isaaclab.sh -p"
  else
    PYTHON_BIN="$(command -v python3 || command -v python || true)"
  fi
fi
if [[ -z "${PYTHON_BIN}" ]]; then
  echo "[doctor] ERROR: python3 or python not found" >&2
  exit 1
fi

echo "[doctor] python = $(${PYTHON_BIN} -c 'import sys; print(sys.executable)')"
${PYTHON_BIN} -c "import isaacsim" 2>/dev/null || {
  echo "[doctor] ERROR: isaacsim import failed with ${PYTHON_BIN}" >&2
  echo "[doctor] hint: activate the Isaac Sim Python environment or set PYTHON_BIN=/path/to/isaac/python" >&2
  exit 1
}
echo "[doctor] isaacsim import OK"

${PYTHON_BIN} -c "from pxr import Usd" 2>/dev/null || {
  echo "[doctor] ERROR: pxr.Usd import failed with ${PYTHON_BIN}" >&2
  echo "[doctor] hint: activate the Isaac Sim Python environment or set PYTHON_BIN=/path/to/isaac/python" >&2
  exit 1
}
echo "[doctor] pxr OK"

for cmd in tmux tmuxp; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "[doctor] ERROR: required command not found: $cmd" >&2
    if [[ "$cmd" == "tmux" ]]; then
      echo "[doctor] hint: sudo apt install tmux" >&2
    else
      echo "[doctor] hint: python3 -m pip install --user tmuxp" >&2
    fi
    exit 1
  fi
  echo "[doctor] $cmd = $(command -v "$cmd")"
done

echo "[doctor] isaacsim CLI = $(command -v isaacsim || true)"
