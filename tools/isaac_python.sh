#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${ISAAC_PYTHON_CMD:-}" ]]; then
  # Allow commands such as: ISAAC_PYTHON_CMD="/path/to/isaaclab.sh -p"
  exec ${ISAAC_PYTHON_CMD} "$@"
fi

if [[ -x "/home/leemou/IsaacLab/isaaclab.sh" ]]; then
  exec /home/leemou/IsaacLab/isaaclab.sh -p "$@"
fi

exec python3 "$@"
