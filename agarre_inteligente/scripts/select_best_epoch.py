#!/usr/bin/env python3
"""Selecciona best_epoch por seed en base a max(val_success)."""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiment-dir", required=True)
    args = ap.parse_args()

    exp_dir = Path(args.experiment_dir)
    rows = []
    for metrics_csv in sorted(exp_dir.glob("seed_*/metrics.csv")):
        df = pd.read_csv(metrics_csv)
        if df.empty:
            continue
        best_idx = int(df["val_success"].idxmax())
        best = df.iloc[best_idx].to_dict()
        seed = metrics_csv.parent.name.replace("seed_", "")
        best["seed"] = int(seed)
        best["best_epoch"] = int(best["epoch"])
        rows.append(best)

    out = exp_dir / "best_epoch_summary.csv"
    pd.DataFrame(rows).sort_values("seed").to_csv(out, index=False)
    print(f"[OK] {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
