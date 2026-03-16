#!/usr/bin/env python3
"""Genera tablas TFM (CSV+Markdown) desde summary_results.csv y metadatos."""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

MODEL_MAP = {
    "EXP1_SIMPLE_RGB": "SimpleGraspCNN",
    "EXP2_SIMPLE_RGBD": "SimpleGraspCNN",
    "EXP3_RESNET18_RGB_AUGMENT": "ResNet18Grasp",
    "EXP4_RESNET18_RGBD": "ResNet18Grasp",
}
MODALITY_MAP = {
    "EXP1_SIMPLE_RGB": "RGB",
    "EXP2_SIMPLE_RGBD": "RGB-D",
    "EXP3_RESNET18_RGB_AUGMENT": "RGB",
    "EXP4_RESNET18_RGBD": "RGB-D",
}


def _fmt(mean: float, std: float) -> str:
    return f"{mean:.4f} +- {std:.4f}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", default="reports/tables/summary_results.csv")
    ap.add_argument("--latency", default="reports/bench/latency_results.csv")
    ap.add_argument("--out-dir", default="reports/tables")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summary = pd.read_csv(args.summary)

    t_val = summary.copy()
    t_val["model"] = t_val["experiment"].map(MODEL_MAP)
    t_val["modality"] = t_val["experiment"].map(MODALITY_MAP)
    t_val.to_csv(out_dir / "table_validation_aggregated.csv", index=False)

    compact = pd.DataFrame(
        {
            "experiment": t_val["experiment"],
            "model": t_val["model"],
            "modality": t_val["modality"],
            "val_success": [
                _fmt(a, b) for a, b in zip(t_val["val_success_mean"], t_val["val_success_std"])
            ],
            "val_iou": [_fmt(a, b) for a, b in zip(t_val["val_iou_mean"], t_val["val_iou_std"])],
            "val_angle_deg": [
                _fmt(a, b) for a, b in zip(t_val["val_angle_deg_mean"], t_val["val_angle_deg_std"])
            ],
            "val_loss": [_fmt(a, b) for a, b in zip(t_val["val_loss_mean"], t_val["val_loss_std"])],
        }
    )
    compact.to_csv(out_dir / "table_metrics_final.csv", index=False)

    # Comparativa A/B por modalidad
    ab_rows = []
    for modality in ["RGB", "RGB-D"]:
        a = t_val[(t_val["model"] == "ResNet18Grasp") & (t_val["modality"] == modality)]
        b = t_val[(t_val["model"] == "SimpleGraspCNN") & (t_val["modality"] == modality)]
        if a.empty or b.empty:
            continue
        a_s = float(a.iloc[0]["val_success_mean"])
        b_s = float(b.iloc[0]["val_success_mean"])
        ab_rows.append(
            {
                "modality": modality,
                "resnet18_val_success": a_s,
                "simplecnn_val_success": b_s,
                "delta_abs_points": a_s - b_s,
            }
        )
    if ab_rows:
        pd.DataFrame(ab_rows).to_csv(out_dir / "table_ab_comparison_by_modality.csv", index=False)
    else:
        pd.DataFrame(
            columns=["modality", "resnet18_val_success", "simplecnn_val_success", "delta_abs_points"]
        ).to_csv(out_dir / "table_ab_comparison_by_modality.csv", index=False)

    if Path(args.latency).exists():
        pd.read_csv(args.latency).to_csv(out_dir / "table_latency.csv", index=False)

    # Export markdown rapido
    md = ["# Tablas TFM regeneradas", ""]
    for name in [
        "table_validation_aggregated.csv",
        "table_metrics_final.csv",
        "table_ab_comparison_by_modality.csv",
        "table_latency.csv",
    ]:
        p = out_dir / name
        if not p.exists():
            continue
        md.append(f"## {name}")
        md.append("")
        head_df = pd.read_csv(p).head(20)
        try:
            md.append(head_df.to_markdown(index=False))
        except Exception:
            md.append(head_df.to_csv(index=False))
        md.append("")
    (out_dir / "TABLES_SUMMARY.md").write_text("\n".join(md), encoding="utf-8")

    print(f"[OK] Tablas generadas en {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
