"""Thin wrappers around src.training.metrics for Cornell evaluation.

The ROS 2 panel calls these with single-sample arguments;
the underlying implementation operates on NumPy arrays.
"""
from __future__ import annotations

import numpy as np

from src.training.metrics import (
    angle_error_deg as _angle_error_batch,
    cornell_success as _cornell_success_batch,
    iou_axis_aligned_boxes as _iou_batch,
)


def grasp_iou(pred, ref) -> float:
    """IoU between two grasp rectangles (cx, cy, w, h, angle_deg)."""
    p = np.asarray(pred, dtype=np.float64).reshape(1, -1)[:, :5]
    g = np.asarray(ref, dtype=np.float64).reshape(1, -1)[:, :5]
    return float(_iou_batch(p, g)[0])


def angle_diff_deg(pred_angle: float, ref_angle: float) -> float:
    """Symmetric angle difference (deg) between two angles."""
    p = np.asarray([pred_angle], dtype=np.float64)
    g = np.asarray([ref_angle], dtype=np.float64)
    return float(_angle_error_batch(p, g)[0])


def compute_grasp_success(
    pred, ref, *, iou_thresh: float = 0.25, angle_thresh: float = 30.0
) -> bool:
    """Cornell success criterion for a single prediction."""
    p = np.asarray(pred, dtype=np.float64).reshape(1, -1)[:, :5]
    g = np.asarray(ref, dtype=np.float64).reshape(1, -1)[:, :5]
    return bool(_cornell_success_batch(p, g, iou_thr=iou_thresh, angle_thr=angle_thresh)[0])
