#!/usr/bin/env python3
"""Selecciona best_epoch por seed en base a max(val_success).

Si se activa el fallback por `val_loss`, cuando una seed no discrimina por
`val_success` (por ejemplo, todo a 0.0) se elige la epoca con menor
`val_loss` para evitar fijar `best_epoch` arbitrariamente en la primera fila.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiment-dir", required=True)
    ap.add_argument(
        "--fallback-val-loss-on-flat-success",
        action="store_true",
        help="Si val_success no discrimina en una seed, usa la epoca con menor val_loss.",
    )
    args = ap.parse_args()

    exp_dir = Path(args.experiment_dir)
    rows = []
    for metrics_csv in sorted(exp_dir.glob("seed_*/metrics.csv")):
        df = pd.read_csv(metrics_csv)
        if df.empty:
            continue
        best_idx = int(df["val_success"].idxmax())
        if args.fallback_val_loss_on_flat_success and "val_loss" in df.columns:
            success_series = pd.to_numeric(df["val_success"], errors="coerce")
            finite_success = success_series.dropna()
            if not finite_success.empty and float(finite_success.max()) <= 0.0 and float(finite_success.min()) == float(finite_success.max()):
                val_loss_series = pd.to_numeric(df["val_loss"], errors="coerce")
                finite_loss = val_loss_series.dropna()
                if not finite_loss.empty:
                    best_idx = int(val_loss_series.idxmin())
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
