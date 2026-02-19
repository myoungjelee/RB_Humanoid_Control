#!/usr/bin/env bash
set -euo pipefail

echo "[doctor] python = $(python -c 'import sys; print(sys.executable)')"
python -c "import isaacsim; print('[doctor] isaacsim import OK')"
python -c "from pxr import Usd; print('[doctor] pxr OK')"

echo "[doctor] isaacsim CLI = $(command -v isaacsim || true)"
