#!/usr/bin/env python3
"""Valida que existan artefactos minimos para tablas/figuras/conclusiones TFM."""

from __future__ import annotations

import argparse
from pathlib import Path


REQUIRED = [
    "reports/tables/summary_results.csv",
    "reports/tables/table_metrics_final.csv",
    "reports/tables/table_validation_aggregated.csv",
    "reports/figures/loss_val_by_epoch.png",
    "reports/figures/val_success_by_epoch.png",
    "reports/figures/bar_val_success_final.png",
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--strict", action="store_true")
    args = ap.parse_args()

    missing = [p for p in REQUIRED if not Path(p).exists()]
    if missing:
        print("[WARN] Faltan artefactos:")
        for m in missing:
            print(" -", m)
        if args.strict:
            return 1
    else:
        print("[OK] Artefactos minimos presentes")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
