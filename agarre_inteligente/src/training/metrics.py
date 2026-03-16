"""Metricas tipo Cornell para deteccion 2D/2.5D de agarres."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def angle_error_deg(pred_deg: np.ndarray, gt_deg: np.ndarray) -> np.ndarray:
    # Simetria de 180 grados para pinza paralela.
    diff = np.abs(pred_deg - gt_deg) % 180.0
    return np.minimum(diff, 180.0 - diff)


def iou_axis_aligned_boxes(pred: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """IoU aproximado usando cajas axis-aligned derivadas de (cx, cy, w, h)."""
    px1 = pred[:, 0] - pred[:, 2] / 2
    py1 = pred[:, 1] - pred[:, 3] / 2
    px2 = pred[:, 0] + pred[:, 2] / 2
    py2 = pred[:, 1] + pred[:, 3] / 2

    gx1 = gt[:, 0] - gt[:, 2] / 2
    gy1 = gt[:, 1] - gt[:, 3] / 2
    gx2 = gt[:, 0] + gt[:, 2] / 2
    gy2 = gt[:, 1] + gt[:, 3] / 2

    ix1 = np.maximum(px1, gx1)
    iy1 = np.maximum(py1, gy1)
    ix2 = np.minimum(px2, gx2)
    iy2 = np.minimum(py2, gy2)

    inter_w = np.maximum(0.0, ix2 - ix1)
    inter_h = np.maximum(0.0, iy2 - iy1)
    inter = inter_w * inter_h

    p_area = np.maximum(1e-9, (px2 - px1) * (py2 - py1))
    g_area = np.maximum(1e-9, (gx2 - gx1) * (gy2 - gy1))
    union = p_area + g_area - inter + 1e-9
    return inter / union


def cornell_success(pred: np.ndarray, gt: np.ndarray, iou_thr: float = 0.25, angle_thr: float = 30.0) -> np.ndarray:
    iou = iou_axis_aligned_boxes(pred, gt)
    ang = angle_error_deg(pred[:, 4], gt[:, 4])
    return (iou >= iou_thr) & (ang <= angle_thr)


def summarize_batch(pred: np.ndarray, gt: np.ndarray) -> dict[str, float]:
    iou = iou_axis_aligned_boxes(pred, gt)
    ang = angle_error_deg(pred[:, 4], gt[:, 4])
    succ = cornell_success(pred, gt)
    return {
        "val_iou": float(np.mean(iou)),
        "val_angle_deg": float(np.mean(ang)),
        "val_success": float(np.mean(succ.astype(np.float32))),
    }


def mean_dict(rows: Iterable[dict[str, float]]) -> dict[str, float]:
    rows = list(rows)
    if not rows:
        return {}
    keys = rows[0].keys()
    return {k: float(np.mean([r[k] for r in rows])) for k in keys}


def log_metrics(metrics: dict[str, float]) -> str:
    ordered = [f"{k}={v:.4f}" for k, v in sorted(metrics.items())]
    return " | ".join(ordered)
