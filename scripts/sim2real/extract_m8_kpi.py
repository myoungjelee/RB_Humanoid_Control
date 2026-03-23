#!/usr/bin/env python3
"""M8 run artifacts -> KPI/summary extractor."""

from __future__ import annotations

import argparse
from pathlib import Path

from scripts.sim2real.kpi.model import build_comparison, build_label_kpi
from scripts.sim2real.kpi.writers import upsert_index_csv, write_json, write_summary_md


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract M8 KPI from an existing run directory.")
    parser.add_argument("run_dir", type=Path, help="Path like logs/sim2real/m8/<run_id>")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    m9_root = run_dir.parent.parent / "m9"
    output_dir = m9_root / run_dir.name
    output_dir.mkdir(parents=True, exist_ok=True)

    off = build_label_kpi(run_dir, "balance_off")
    on = build_label_kpi(run_dir, "balance_on")
    comparison = build_comparison(off, on)

    write_json(output_dir / "balance_off_kpi.json", off)
    write_json(output_dir / "balance_on_kpi.json", on)
    write_json(output_dir / "comparison.json", comparison)
    write_summary_md(output_dir / "summary.md", off, on, comparison)
    upsert_index_csv(m9_root / "index.csv", comparison)

    print(f"[M9] wrote {output_dir / 'balance_off_kpi.json'}")
    print(f"[M9] wrote {output_dir / 'balance_on_kpi.json'}")
    print(f"[M9] wrote {output_dir / 'comparison.json'}")
    print(f"[M9] wrote {output_dir / 'summary.md'}")
    print(f"[M9] updated {m9_root / 'index.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
