#!/usr/bin/env python3
"""Genera overlays GT vs Pred para analisis cualitativo (si hay imagenes)."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image


def draw_rect(ax, cx, cy, w, h, color, label, angle_deg=0.0):
    """Dibuja rectángulo de agarre rotado (convención Cornell)."""
    import numpy as np
    from matplotlib.patches import Polygon
    hw, hh = w / 2.0, h / 2.0
    ang = np.radians(angle_deg)
    cs, sn = np.cos(ang), np.sin(ang)
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    pts = [(cx + dx * cs - dy * sn, cy + dx * sn + dy * cs) for dx, dy in corners]
    poly = Polygon(pts, fill=False, edgecolor=color, linewidth=2, label=label)
    ax.add_patch(poly)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--predictions", required=True, help="CSV con pred y gt")
    ap.add_argument("--data-root", default="data/processed")
    ap.add_argument("--out-dir", default="reports/figures/qualitative")
    ap.add_argument("--max-samples", type=int, default=24)
    args = ap.parse_args()

    pred_csv = Path(args.predictions)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(pred_csv).head(args.max_samples)
    generated = 0
    skipped = 0
    for i, row in df.iterrows():
        img_path = Path(args.data_root) / str(row["image_path"])
        if not img_path.exists():
            skipped += 1
            continue
        img = Image.open(img_path).convert("RGB")
        fig, ax = plt.subplots(figsize=(5, 5))
        ax.imshow(img)
        gt_angle = float(row.get("gt_angle_deg", 0.0)) if "gt_angle_deg" in row else 0.0
        pred_angle = float(row.get("pred_angle_deg", 0.0)) if "pred_angle_deg" in row else 0.0
        draw_rect(ax, row["gt_cx"], row["gt_cy"], row["gt_w"], row["gt_h"], "green", "GT", gt_angle)
        draw_rect(ax, row["pred_cx"], row["pred_cy"], row["pred_w"], row["pred_h"], "red", "Pred", pred_angle)
        ax.set_title(f"sample_{i}")
        ax.axis("off")
        fig.tight_layout()
        fig.savefig(out_dir / f"overlay_{i:03d}.png", dpi=150)
        plt.close(fig)
        generated += 1

    (out_dir / "overlay_summary.txt").write_text(
        f"generated={generated}\nskipped_missing_images={skipped}\n", encoding="utf-8"
    )
    print(f"[OK] overlays generados={generated}, omitidos={skipped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
