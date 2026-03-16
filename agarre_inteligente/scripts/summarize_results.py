#!/usr/bin/env python3
"""Agrega resultados por experimento y genera resumen global."""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def _agg(df: pd.DataFrame, exp_name: str) -> dict:
    return {
        "experiment": exp_name,
        "n_seeds": int(len(df)),
        "val_success_mean": float(df["val_success"].mean()),
        "val_success_std": float(df["val_success"].std(ddof=0)),
        "val_iou_mean": float(df["val_iou"].mean()),
        "val_iou_std": float(df["val_iou"].std(ddof=0)),
        "val_angle_deg_mean": float(df["val_angle_deg"].mean()),
        "val_angle_deg_std": float(df["val_angle_deg"].std(ddof=0)),
        "val_loss_mean": float(df["val_loss"].mean()),
        "val_loss_std": float(df["val_loss"].std(ddof=0)),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiments-root", default="experiments")
    ap.add_argument("--output", default="reports/tables/summary_results.csv")
    args = ap.parse_args()

    root = Path(args.experiments_root)
    rows = []
    seed_rows = []
    for exp_dir in sorted(root.glob("EXP*")):
        best_csv = exp_dir / "best_epoch_summary.csv"
        if not best_csv.exists():
            continue
        df = pd.read_csv(best_csv)
        rows.append(_agg(df, exp_dir.name))
        seed_df = df.copy()
        seed_df.insert(0, "experiment", exp_dir.name)
        seed_rows.append(seed_df)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(out, index=False)

    if seed_rows:
        pd.concat(seed_rows, ignore_index=True).to_csv(out.parent / "results_by_seed.csv", index=False)

    print(f"[OK] resumen agregado -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
