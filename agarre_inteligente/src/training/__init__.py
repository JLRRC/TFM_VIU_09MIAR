"""Modulo de entrenamiento para experimentos de agarre."""

from .trainer import Trainer
from .metrics import cornell_success, iou_axis_aligned_boxes, mean_dict, summarize_batch, log_metrics

__all__ = [
    "Trainer",
    "cornell_success",
    "iou_axis_aligned_boxes",
    "summarize_batch",
    "mean_dict",
    "log_metrics",
]
