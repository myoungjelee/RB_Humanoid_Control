#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${RUN_ID:-$(date +%Y%m%d-%H%M%S)}"

RUN_ID="$RUN_ID" bash "$ROOT_DIR/ops/run_m8_pair.sh"
bash "$ROOT_DIR/ops/run_m9_kpi.sh" "$RUN_ID"

echo "[DONE] M8+M9 wrapper complete"
echo "[INFO] m8 logs: $ROOT_DIR/logs/sim2real/m8/$RUN_ID"
echo "[INFO] m9 logs: $ROOT_DIR/logs/sim2real/m9/$RUN_ID"
