#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:?usage: run_m9_kpi.sh <run_id>}"

cd "$ROOT_DIR"
python3 -m scripts.sim2real.extract_m8_kpi "$ROOT_DIR/logs/sim2real/m8/$RUN_ID"
