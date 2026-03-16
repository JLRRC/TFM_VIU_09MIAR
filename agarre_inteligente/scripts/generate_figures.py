#!/usr/bin/env python3
"""Genera figuras de resultados (curvas, barras, comparativas por seed)."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def _plot_metric_by_epoch(experiments_root: Path, metric: str, output_path: Path) -> None:
    plt.figure(figsize=(10, 6))
    for metrics_csv in sorted(experiments_root.glob("EXP*/seed_*/metrics.csv")):
        df = pd.read_csv(metrics_csv)
        if metric not in df.columns:
            continue
        exp = metrics_csv.parents[1].name
        seed = metrics_csv.parent.name
        plt.plot(df["epoch"], df[metric], alpha=0.7, label=f"{exp}-{seed}")
    plt.xlabel("epoch")
    plt.ylabel(metric)
    plt.title(f"{metric} por epoca")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=7, ncol=2)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()


def _bar_from_summary(summary: pd.DataFrame, col_mean: str, title: str, out: Path) -> None:
    plt.figure(figsize=(8, 5))
    plt.bar(summary["experiment"], summary[col_mean])
    plt.title(title)
    plt.xticks(rotation=25, ha="right")
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, dpi=160)
    plt.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiments-root", default="experiments")
    ap.add_argument("--summary", default="reports/tables/summary_results.csv")
    ap.add_argument("--out-dir", default="reports/figures")
    args = ap.parse_args()

    exp_root = Path(args.experiments_root)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    _plot_metric_by_epoch(exp_root, "train_loss", out_dir / "loss_train_by_epoch.png")
    _plot_metric_by_epoch(exp_root, "val_loss", out_dir / "loss_val_by_epoch.png")
    _plot_metric_by_epoch(exp_root, "val_success", out_dir / "val_success_by_epoch.png")
    _plot_metric_by_epoch(exp_root, "val_iou", out_dir / "val_iou_by_epoch.png")
    _plot_metric_by_epoch(exp_root, "val_angle_deg", out_dir / "val_angle_deg_by_epoch.png")

    summary = pd.read_csv(args.summary)
    _bar_from_summary(summary, "val_success_mean", "Exito final agregado", out_dir / "bar_val_success_final.png")
    _bar_from_summary(summary, "val_iou_mean", "IoU final agregado", out_dir / "bar_val_iou_final.png")
    _bar_from_summary(summary, "val_angle_deg_mean", "Angulo final agregado", out_dir / "bar_val_angle_final.png")

    print(f"[OK] Figuras generadas en {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
