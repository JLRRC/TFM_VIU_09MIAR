#!/usr/bin/env python3
"""Reconstruye inventario cualitativo reproducible para la revision visual del TFM."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
import pandas as pd
import torch
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.data.transforms import get_val_transforms
from src.models.resnet_variants import ResNetGrasp
from src.models.simple_cnn import SimpleCNN
from src.training.metrics import angle_error_deg, cornell_success, iou_axis_aligned_boxes
from src.utils.config_loader import load_config


OUTPUT_ROOT = PROJECT_ROOT / "reports" / "tfm_visual_revision"
INTERMEDIATE_ROOT = OUTPUT_ROOT / "intermediate"
INVENTORY_ROOT = OUTPUT_ROOT / "inventory"

EXPERIMENT_RUNS = [
    {
        "experiment": "EXP3_RESNET18_RGB_AUGMENT",
        "seed": 0,
        "config": PROJECT_ROOT / "config" / "exp3_resnet18_rgb_augment.yaml",
        "checkpoint": PROJECT_ROOT / "experiments" / "EXP3_RESNET18_RGB_AUGMENT" / "seed_0" / "checkpoints" / "best.pth",
        "role": "success_primary",
    },
    {
        "experiment": "EXP1_SIMPLE_RGB",
        "seed": 0,
        "config": PROJECT_ROOT / "config" / "exp1_simple_rgb.yaml",
        "checkpoint": PROJECT_ROOT / "experiments" / "EXP1_SIMPLE_RGB" / "seed_0" / "checkpoints" / "best.pth",
        "role": "failure_primary",
    },
]


@dataclass
class MatchMetrics:
    gt_idx: int
    success: bool
    iou: float
    angle_deg: float
    center_distance: float
    center_distance_norm: float
    area_ratio: float
    pred_area: float
    gt_area: float
    candidate_error: str


def build_model(model_cfg: dict) -> torch.nn.Module:
    if model_cfg["name"] == "SimpleGraspCNN":
        return SimpleCNN(
            input_channels=int(model_cfg["input_channels"]),
            dropout=float(model_cfg.get("dropout", 0.2)),
        )
    return ResNetGrasp(
        input_channels=int(model_cfg["input_channels"]),
        pretrained=False,
        freeze_backbone=bool(model_cfg.get("freeze_backbone", False)),
        dropout=float(model_cfg.get("dropout", 0.2)),
    )


def draw_rotated_rect(
    ax,
    cx: float,
    cy: float,
    width: float,
    height: float,
    angle_deg: float,
    color: str,
    linestyle: str,
    linewidth: float,
) -> None:
    half_w = width / 2.0
    half_h = height / 2.0
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    corners = [(-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)]
    pts = [
        (
            cx + dx * cos_a - dy * sin_a,
            cy + dx * sin_a + dy * cos_a,
        )
        for dx, dy in corners
    ]
    ax.add_patch(
        Polygon(
            pts,
            closed=True,
            fill=False,
            edgecolor=color,
            linestyle=linestyle,
            linewidth=linewidth,
        )
    )


def infer_single_image(model: torch.nn.Module, image_tensor: torch.Tensor, device: str) -> np.ndarray:
    with torch.no_grad():
        pred = model(image_tensor.unsqueeze(0).to(device)).cpu().numpy()[0]
    pred = np.nan_to_num(pred, nan=0.0, posinf=1.0, neginf=-1.0)
    _, h, w = image_tensor.shape
    return np.array(
        [
            pred[0] * float(w),
            pred[1] * float(h),
            abs(pred[2] * float(w)),
            abs(pred[3] * float(h)),
            pred[4] * 90.0,
        ],
        dtype=np.float64,
    )


def scale_gt(group: pd.DataFrame, final_w: int, final_h: int) -> np.ndarray:
    gt = group[["cx", "cy", "w", "h", "angle_deg"]].to_numpy(dtype=np.float64).copy()
    gt[:, 0] = gt[:, 0] * (final_w / 640.0)
    gt[:, 1] = gt[:, 1] * (final_h / 480.0)
    gt[:, 2] = gt[:, 2] * (final_w / 640.0)
    gt[:, 3] = gt[:, 3] * (final_h / 480.0)
    return gt


def classify_candidate_error(
    pred: np.ndarray,
    gt: np.ndarray,
    ious: np.ndarray,
    angles: np.ndarray,
    best_idx: int,
) -> str:
    best_iou = float(ious[best_idx])
    best_ang = float(angles[best_idx])
    pred_area = float(max(pred[2] * pred[3], 1e-9))
    gt_area = float(max(gt[best_idx, 2] * gt[best_idx, 3], 1e-9))
    area_ratio = pred_area / gt_area
    center_dist = float(np.linalg.norm(pred[:2] - gt[best_idx, :2]))
    diag = float(math.hypot(gt[best_idx, 2], gt[best_idx, 3]))
    center_dist_norm = center_dist / max(diag, 1e-9)

    pred_box = np.array(
        [
            pred[0] - pred[2] / 2.0,
            pred[1] - pred[3] / 2.0,
            pred[0] + pred[2] / 2.0,
            pred[1] + pred[3] / 2.0,
        ]
    )
    gt_box = np.array(
        [
            gt[best_idx, 0] - gt[best_idx, 2] / 2.0,
            gt[best_idx, 1] - gt[best_idx, 3] / 2.0,
            gt[best_idx, 0] + gt[best_idx, 2] / 2.0,
            gt[best_idx, 1] + gt[best_idx, 3] / 2.0,
        ]
    )
    expanded_gt = np.array(
        [
            gt_box[0] - 0.35 * gt[best_idx, 2],
            gt_box[1] - 0.35 * gt[best_idx, 3],
            gt_box[2] + 0.35 * gt[best_idx, 2],
            gt_box[3] + 0.35 * gt[best_idx, 3],
        ]
    )
    pred_center = pred[:2]
    center_outside = (
        pred_center[0] < expanded_gt[0]
        or pred_center[0] > expanded_gt[2]
        or pred_center[1] < expanded_gt[1]
        or pred_center[1] > expanded_gt[3]
    )

    if best_iou >= 0.25 and best_ang > 30.0:
        return "E1"
    if center_outside and center_dist_norm > 0.8:
        return "E4"
    if area_ratio < 0.55 or area_ratio > 1.8:
        return "E3"
    return "E2"


def choose_match(pred: np.ndarray, gt: np.ndarray) -> MatchMetrics:
    pred_batch = pred.reshape(1, 5)
    ious = iou_axis_aligned_boxes(pred_batch, gt)
    angles = angle_error_deg(np.repeat(pred[4], len(gt)), gt[:, 4])
    successes = cornell_success(
        np.repeat(pred_batch, len(gt), axis=0),
        gt,
    )

    if np.any(successes):
        success_indices = np.where(successes)[0]
        best_idx = int(success_indices[np.argmax(ious[success_indices])])
        candidate_error = "OK"
        is_success = True
    else:
        score = ious - 0.002 * angles
        best_idx = int(np.argmax(score))
        candidate_error = classify_candidate_error(pred, gt, ious, angles, best_idx)
        is_success = False

    center_distance = float(np.linalg.norm(pred[:2] - gt[best_idx, :2]))
    diag = float(math.hypot(gt[best_idx, 2], gt[best_idx, 3]))
    pred_area = float(max(pred[2] * pred[3], 1e-9))
    gt_area = float(max(gt[best_idx, 2] * gt[best_idx, 3], 1e-9))

    return MatchMetrics(
        gt_idx=best_idx,
        success=is_success,
        iou=float(ious[best_idx]),
        angle_deg=float(angles[best_idx]),
        center_distance=center_distance,
        center_distance_norm=center_distance / max(diag, 1e-9),
        area_ratio=pred_area / gt_area,
        pred_area=pred_area,
        gt_area=gt_area,
        candidate_error=candidate_error,
    )


def save_overlay(
    image_path: Path,
    pred: np.ndarray,
    gt: np.ndarray,
    metrics: MatchMetrics,
    output_path: Path,
    experiment: str,
    role: str,
) -> None:
    image = Image.open(image_path).convert("RGB").resize((224, 224))
    fig, ax = plt.subplots(figsize=(5.2, 5.2))
    ax.imshow(image)
    draw_rotated_rect(ax, gt[metrics.gt_idx, 0], gt[metrics.gt_idx, 1], gt[metrics.gt_idx, 2], gt[metrics.gt_idx, 3], gt[metrics.gt_idx, 4], "#15b24a", "--", 2.6)
    draw_rotated_rect(ax, pred[0], pred[1], pred[2], pred[3], pred[4], "#d62828", "-", 2.6)
    ax.axis("off")
    title_left = f"{experiment} | {role}"
    title_right = f"IoU={metrics.iou:.2f} | dtheta={metrics.angle_deg:.1f} deg"
    subtitle = "OK" if metrics.success else metrics.candidate_error
    fig.text(0.02, 0.98, title_left, ha="left", va="top", fontsize=10, fontweight="bold")
    fig.text(0.98, 0.98, title_right, ha="right", va="top", fontsize=10)
    fig.text(0.5, 0.02, subtitle, ha="center", va="bottom", fontsize=11, fontweight="bold")
    fig.tight_layout(rect=[0.0, 0.04, 1.0, 0.95])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)


def run_inventory(run_cfg: dict[str, object]) -> pd.DataFrame:
    config_path = Path(run_cfg["config"])
    checkpoint_path = Path(run_cfg["checkpoint"])
    experiment = str(run_cfg["experiment"])
    seed = int(run_cfg["seed"])
    role = str(run_cfg["role"])

    cfg = load_config(config_path)
    model = build_model(cfg["model"])
    state = torch.load(checkpoint_path, map_location="cpu")
    model.load_state_dict(state["model_state_dict"])
    device = "cpu"
    model = model.to(device)
    model.eval()

    val_csv = PROJECT_ROOT / cfg["data"]["val_csv"]
    data_root = PROJECT_ROOT / cfg["data"]["data_root"]
    val_df = pd.read_csv(val_csv)
    grouped = val_df.groupby("image_path", sort=True)
    transform = get_val_transforms(image_size=int(cfg["data"].get("image_size", 224)))

    overlay_root = INTERMEDIATE_ROOT / "overlays" / f"{experiment}_seed_{seed}"
    rows: list[dict[str, object]] = []

    for image_idx, (image_rel_path, group) in enumerate(grouped):
        image_path = data_root / image_rel_path
        image = Image.open(image_path).convert("RGB")
        image_tensor = transform(image)
        _, final_h, final_w = image_tensor.shape
        pred = infer_single_image(model, image_tensor, device=device)
        gt = scale_gt(group, final_w=final_w, final_h=final_h)
        metrics = choose_match(pred, gt)

        overlay_name = image_rel_path.replace("/", "__").replace(".png", ".png")
        overlay_path = overlay_root / overlay_name
        save_overlay(image_path, pred, gt, metrics, overlay_path, experiment, role)

        rows.append(
            {
                "experiment": experiment,
                "seed": seed,
                "role": role,
                "image_idx": image_idx,
                "image_path": image_rel_path,
                "overlay_path": str(overlay_path.relative_to(PROJECT_ROOT)),
                "success": metrics.success,
                "candidate_error": metrics.candidate_error,
                "display_gt_idx": metrics.gt_idx,
                "iou_display_gt": metrics.iou,
                "angle_display_gt_deg": metrics.angle_deg,
                "center_distance_px": metrics.center_distance,
                "center_distance_norm": metrics.center_distance_norm,
                "area_ratio_pred_gt": metrics.area_ratio,
                "pred_cx": pred[0],
                "pred_cy": pred[1],
                "pred_w": pred[2],
                "pred_h": pred[3],
                "pred_angle_deg": pred[4],
                "gt_cx": gt[metrics.gt_idx, 0],
                "gt_cy": gt[metrics.gt_idx, 1],
                "gt_w": gt[metrics.gt_idx, 2],
                "gt_h": gt[metrics.gt_idx, 3],
                "gt_angle_deg": gt[metrics.gt_idx, 4],
                "gt_count_for_image": len(group),
                "orig_image_path": str(image_path),
            }
        )

    df = pd.DataFrame(rows)
    csv_path = INVENTORY_ROOT / f"qualitative_inventory_{experiment}_seed_{seed}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(csv_path, index=False)
    return df


def main() -> int:
    all_frames = [run_inventory(run_cfg) for run_cfg in EXPERIMENT_RUNS]
    all_df = pd.concat(all_frames, ignore_index=True)
    combined_path = INVENTORY_ROOT / "qualitative_inventory_combined.csv"
    all_df.to_csv(combined_path, index=False)

    summary = (
        all_df.groupby(["experiment", "success", "candidate_error"])
        .size()
        .reset_index(name="n_cases")
        .sort_values(["experiment", "success", "candidate_error"])
    )
    summary_path = INVENTORY_ROOT / "qualitative_inventory_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"[OK] Inventario combinado guardado en {combined_path}")
    print(f"[OK] Resumen guardado en {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
