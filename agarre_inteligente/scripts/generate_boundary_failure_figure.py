#!/usr/bin/env python3
"""Selecciona un falso negativo limítrofe y genera una figura académica para el TFM."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn.functional as F
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from PIL import Image, ImageFile
from torch.utils.data import DataLoader, Dataset

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.data.transforms import get_val_transforms
from src.models.factory import build_model
from src.training.metrics import angle_error_deg, cornell_success, iou_axis_aligned_boxes
from src.utils.config_loader import load_config

ImageFile.LOAD_TRUNCATED_IMAGES = True

CONFIG_BY_EXPERIMENT = {
    "EXP1_SIMPLE_RGB": PROJECT_ROOT / "config" / "exp1_simple_rgb.yaml",
    "EXP2_SIMPLE_RGBD": PROJECT_ROOT / "config" / "exp2_simple_rgbd.yaml",
    "EXP3_RESNET18_RGB_AUGMENT": PROJECT_ROOT / "config" / "exp3_resnet18_rgb_augment.yaml",
    "EXP4_RESNET18_RGBD": PROJECT_ROOT / "config" / "exp4_resnet18_rgbd.yaml",
    "EXP1.1_SIMPLEGRASP_RGB": PROJECT_ROOT / "config" / "exp1_1_simplegrasp_rgb.yaml",
    "EXP1.2_SIMPLEGRASP_RGBD": PROJECT_ROOT / "config" / "exp1_2_simplegrasp_rgbd.yaml",
}

DEFAULT_OUTPUT_DIR = WORKSPACE_ROOT / "reports" / "figures" / "chapter5"
DEFAULT_BASE_NAME = "fig_5_15_false_negative_plausible"


@dataclass(frozen=True)
class SelectedCheckpoint:
    experiment: str
    seed: int
    config_path: Path
    checkpoint_path: Path
    val_success: float
    model_name: str
    modality: str


class UniqueImageDataset(Dataset):
    def __init__(self, unique_df: pd.DataFrame, data_root: Path, modality: str, image_size: int):
        self.unique_df = unique_df.reset_index(drop=True)
        self.data_root = data_root
        self.modality = modality
        self.transform = get_val_transforms(image_size=image_size)

    def __len__(self) -> int:
        return len(self.unique_df)

    def __getitem__(self, idx: int):
        row = self.unique_df.iloc[idx]
        rgb_path = self.data_root / str(row["image_path"])
        rgb = Image.open(rgb_path).convert("RGB")
        orig_w, orig_h = rgb.size
        rgb_tensor = self.transform(rgb)

        if self.modality == "rgbd":
            depth_rel = row.get("depth_path", None)
            if depth_rel is None or pd.isna(depth_rel):
                depth = torch.zeros((1, rgb_tensor.shape[1], rgb_tensor.shape[2]), dtype=torch.float32)
            else:
                d_img = Image.open(self.data_root / str(depth_rel))
                d_np = np.asarray(d_img).astype(np.float32)
                if d_np.ndim == 3:
                    d_np = d_np[..., 0]
                d_np = d_np / (np.max(d_np) + 1e-6)
                depth = torch.from_numpy(d_np).unsqueeze(0)
                if depth.shape[1:] != rgb_tensor.shape[1:]:
                    depth = F.interpolate(
                        depth.unsqueeze(0),
                        size=rgb_tensor.shape[1:],
                        mode="bilinear",
                        align_corners=False,
                    ).squeeze(0)
            image = torch.cat([rgb_tensor, depth], dim=0)
        else:
            image = rgb_tensor

        return image, str(row["image_path"]), int(orig_w), int(orig_h)


def rotated_box_to_points(cx: float, cy: float, w: float, h: float, angle_deg: float) -> np.ndarray:
    hw, hh = w / 2.0, h / 2.0
    ang = math.radians(angle_deg)
    cs, sn = math.cos(ang), math.sin(ang)
    corners = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=np.float32)
    rot = np.array([[cs, -sn], [sn, cs]], dtype=np.float32)
    return corners @ rot.T + np.array([cx, cy], dtype=np.float32)


def draw_rect(ax, box: np.ndarray, color: str, label: str, linestyle: str = "-", linewidth: float = 2.5) -> None:
    pts = rotated_box_to_points(*box)
    poly = Polygon(pts, closed=True, fill=False, edgecolor=color, linewidth=linewidth, linestyle=linestyle, label=label)
    ax.add_patch(poly)


def scale_box(box: np.ndarray, from_w: float, from_h: float, to_w: float, to_h: float) -> np.ndarray:
    scaled = box.copy().astype(np.float32)
    scaled[0] = scaled[0] * (to_w / from_w)
    scaled[1] = scaled[1] * (to_h / from_h)
    scaled[2] = scaled[2] * (to_w / from_w)
    scaled[3] = scaled[3] * (to_h / from_h)
    return scaled


def primary_failure_cause(iou: float, angle_deg: float, iou_thr: float, angle_thr: float) -> str:
    iou_fail = iou < iou_thr
    angle_fail = angle_deg > angle_thr
    if iou_fail and angle_fail:
        iou_gap = (iou_thr - iou) / max(iou_thr, 1e-6)
        angle_gap = (angle_deg - angle_thr) / max(angle_thr, 1e-6)
        return "IoU + Δθ" if abs(iou_gap - angle_gap) < 0.15 else ("solapamiento insuficiente" if iou_gap > angle_gap else "desviación angular")
    if iou_fail:
        return "solapamiento insuficiente"
    if angle_fail:
        return "desviación angular"
    return "éxito"


def select_reference_gt(ious: np.ndarray, angles: np.ndarray, iou_thr: float, angle_thr: float) -> int:
    iou_gap = np.maximum(iou_thr - ious, 0.0) / iou_thr
    angle_gap = np.maximum(angles - angle_thr, 0.0) / angle_thr
    boundary = np.hypot(iou_gap, angle_gap)
    ranking = boundary - (0.01 * ious) + (0.0001 * angles)
    return int(np.argmin(ranking))


def candidate_score(row: pd.Series, iou_thr: float, angle_thr: float) -> float:
    iou = float(row["iou"])
    angle = float(row["angle_deg"])
    area = float(row["gt_area_frac"])
    iou_gap = max(iou_thr - iou, 0.0) / iou_thr
    angle_gap = max(angle - angle_thr, 0.0) / angle_thr
    boundary = math.hypot(iou_gap, angle_gap)
    center_dist = float(row["center_dist_px"]) / max(float(row["image_diag_px"]), 1.0)
    # Penaliza objetos muy pequeños para evitar ejemplos poco legibles en la memoria.
    size_penalty = (max(0.018 - area, 0.0) * 10.0) + (max(0.012 - area, 0.0) * 8.0)
    dual_fail_penalty = 0.10 if (iou < iou_thr and angle > angle_thr) else 0.0
    implausible_penalty = 0.0
    if iou < 0.15:
        implausible_penalty += 0.35
    if angle > 45.0:
        implausible_penalty += 0.35
    near_bonus = 0.0
    if (iou_thr - 0.03) <= iou < iou_thr:
        near_bonus -= 0.22
    elif 0.20 <= iou < iou_thr:
        near_bonus -= 0.12
    if angle <= angle_thr:
        near_bonus -= 0.06
    elif angle_thr < angle <= 35.0:
        near_bonus -= 0.08
    return boundary + (0.45 * center_dist) + size_penalty + dual_fail_penalty + implausible_penalty + near_bonus


def choose_checkpoints(experiments_root: Path, experiments: list[str]) -> list[SelectedCheckpoint]:
    selected: list[SelectedCheckpoint] = []
    for experiment in experiments:
        config_path = CONFIG_BY_EXPERIMENT[experiment]
        cfg = load_config(config_path)
        summary_path = experiments_root / experiment / "best_epoch_summary.csv"
        summary_df = pd.read_csv(summary_path)
        best_row = summary_df.sort_values("val_success", ascending=False).iloc[0]
        seed = int(best_row["seed"])
        checkpoint_path = experiments_root / experiment / f"seed_{seed}" / "checkpoints" / "best.pth"
        selected.append(
            SelectedCheckpoint(
                experiment=experiment,
                seed=seed,
                config_path=config_path,
                checkpoint_path=checkpoint_path,
                val_success=float(best_row["val_success"]),
                model_name=str(cfg["model"]["name"]),
                modality=str(cfg["data"]["modality"]),
            )
        )
    return selected


def infer_candidates_for_checkpoint(checkpoint_info: SelectedCheckpoint, batch_size: int) -> pd.DataFrame:
    cfg = load_config(checkpoint_info.config_path)
    val_csv = PROJECT_ROOT / cfg["data"]["val_csv"]
    full_val_df = pd.read_csv(val_csv)
    unique_df = full_val_df.groupby("image_path", as_index=False).first()[["image_path", "depth_path"]]
    gt_by_image = {
        image_path: group[["cx", "cy", "w", "h", "angle_deg"]].to_numpy(dtype=np.float32)
        for image_path, group in full_val_df.groupby("image_path")
    }

    model = build_model(cfg["model"])
    state = torch.load(checkpoint_info.checkpoint_path, map_location="cpu")
    model.load_state_dict(state["model_state_dict"])
    model.eval()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = model.to(device)

    dataset = UniqueImageDataset(
        unique_df=unique_df,
        data_root=PROJECT_ROOT,
        modality=str(cfg["data"]["modality"]),
        image_size=int(cfg["data"].get("image_size", 224)),
    )
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=0)

    iou_thr = float(cfg["metrics"].get("iou_threshold", 0.25))
    angle_thr = float(cfg["metrics"].get("angle_threshold_deg", 30.0))

    rows: list[dict[str, object]] = []
    print(f"[RUN] {checkpoint_info.experiment} seed={checkpoint_info.seed} checkpoint={checkpoint_info.checkpoint_path.name}")

    with torch.no_grad():
        for images, image_paths, orig_ws, orig_hs in loader:
            images = images.to(device)
            pred = model(images).cpu().numpy()
            pred = np.nan_to_num(pred, nan=0.0, posinf=1.0, neginf=-1.0)

            final_h = float(images.shape[2])
            final_w = float(images.shape[3])
            pred[:, 0] = pred[:, 0] * final_w
            pred[:, 1] = pred[:, 1] * final_h
            pred[:, 2] = np.abs(pred[:, 2] * final_w)
            pred[:, 3] = np.abs(pred[:, 3] * final_h)
            pred[:, 4] = pred[:, 4] * 90.0

            for idx, image_path in enumerate(image_paths):
                orig_w = float(orig_ws[idx])
                orig_h = float(orig_hs[idx])
                pred_eval = pred[idx].astype(np.float32)
                pred_orig = scale_box(pred_eval, final_w, final_h, orig_w, orig_h)
                gt_orig = gt_by_image[image_path].copy()
                gt_eval = gt_orig.copy()
                gt_eval[:, 0] = gt_eval[:, 0] * (final_w / orig_w)
                gt_eval[:, 1] = gt_eval[:, 1] * (final_h / orig_h)
                gt_eval[:, 2] = gt_eval[:, 2] * (final_w / orig_w)
                gt_eval[:, 3] = gt_eval[:, 3] * (final_h / orig_h)

                pred_single = pred_eval.reshape(1, 5)
                ious = iou_axis_aligned_boxes(pred_single, gt_eval)
                angles = angle_error_deg(pred_single[:, 4], gt_eval[:, 4])
                successes = cornell_success(pred_single, gt_eval, iou_thr=iou_thr, angle_thr=angle_thr)
                chosen_idx = select_reference_gt(ious, angles, iou_thr, angle_thr)
                gt_match_orig = gt_orig[chosen_idx].astype(np.float32)
                gt_match_eval = gt_eval[chosen_idx].astype(np.float32)
                center_dist_px = float(np.hypot(pred_orig[0] - gt_match_orig[0], pred_orig[1] - gt_match_orig[1]))
                image_diag_px = float(np.hypot(orig_w, orig_h))
                area_frac = float((gt_match_orig[2] * gt_match_orig[3]) / max(orig_w * orig_h, 1.0))
                row = {
                    "experiment": checkpoint_info.experiment,
                    "seed": checkpoint_info.seed,
                    "model": checkpoint_info.model_name,
                    "modality": checkpoint_info.modality,
                    "checkpoint_path": str(checkpoint_info.checkpoint_path),
                    "image_path": image_path,
                    "orig_w": orig_w,
                    "orig_h": orig_h,
                    "pred_cx": float(pred_orig[0]),
                    "pred_cy": float(pred_orig[1]),
                    "pred_w": float(pred_orig[2]),
                    "pred_h": float(pred_orig[3]),
                    "pred_angle_deg": float(pred_orig[4]),
                    "gt_cx": float(gt_match_orig[0]),
                    "gt_cy": float(gt_match_orig[1]),
                    "gt_w": float(gt_match_orig[2]),
                    "gt_h": float(gt_match_orig[3]),
                    "gt_angle_deg": float(gt_match_orig[4]),
                    "iou": float(ious[chosen_idx]),
                    "angle_deg": float(angles[chosen_idx]),
                    "success": bool(successes[chosen_idx]),
                    "cause": primary_failure_cause(float(ious[chosen_idx]), float(angles[chosen_idx]), iou_thr, angle_thr),
                    "center_dist_px": center_dist_px,
                    "image_diag_px": image_diag_px,
                    "gt_area_frac": area_frac,
                    "all_gt_count": int(len(gt_orig)),
                    "iou_thr": iou_thr,
                    "angle_thr": angle_thr,
                    "pred_eval_cx": float(pred_eval[0]),
                    "pred_eval_cy": float(pred_eval[1]),
                    "pred_eval_w": float(pred_eval[2]),
                    "pred_eval_h": float(pred_eval[3]),
                    "pred_eval_angle_deg": float(pred_eval[4]),
                    "gt_eval_cx": float(gt_match_eval[0]),
                    "gt_eval_cy": float(gt_match_eval[1]),
                    "gt_eval_w": float(gt_match_eval[2]),
                    "gt_eval_h": float(gt_match_eval[3]),
                    "gt_eval_angle_deg": float(gt_match_eval[4]),
                }
                row["selection_score"] = candidate_score(pd.Series(row), iou_thr, angle_thr)
                rows.append(row)

    df = pd.DataFrame(rows)
    return df.sort_values(["success", "selection_score", "iou"], ascending=[True, True, False]).reset_index(drop=True)


def filter_top_candidates(all_rows: pd.DataFrame, top_k: int) -> pd.DataFrame:
    failures = all_rows.loc[~all_rows["success"]].copy()
    # Prioriza falsos negativos más "pedagógicos":
    # 1) fallo por IoU muy cercano al umbral con ángulo válido y objeto suficientemente visible.
    preferred = failures.loc[
        (failures["iou"] < failures["iou_thr"])
        & (failures["iou"] >= (failures["iou_thr"] - 0.03))
        & (failures["angle_deg"] <= failures["angle_thr"])
        & (failures["gt_area_frac"] >= 0.015)
    ].copy()

    if preferred.empty:
        near_single = failures.loc[
            (
                (failures["iou"] < failures["iou_thr"])
                & (failures["iou"] >= (failures["iou_thr"] - 0.05))
                & (failures["angle_deg"] <= failures["angle_thr"])
            )
            | (
                (failures["iou"] >= failures["iou_thr"])
                & (failures["angle_deg"] > failures["angle_thr"])
                & (failures["angle_deg"] <= (failures["angle_thr"] + 4.0))
            )
        ].copy()
        if not near_single.empty:
            preferred = near_single

    focused = preferred if not preferred.empty else failures.copy()
    focused = focused.sort_values(
        ["selection_score", "iou", "gt_area_frac", "center_dist_px", "angle_deg"],
        ascending=[True, False, False, True, True],
    )
    return focused.head(top_k).reset_index(drop=True)


def select_candidate(
    merged: pd.DataFrame,
    candidates: pd.DataFrame,
    selected_experiment: str | None,
    selected_image_path: str | None,
) -> pd.Series:
    if not selected_image_path:
        return candidates.iloc[0]

    pool = merged.loc[~merged["success"]].copy()
    if selected_experiment:
        pool = pool.loc[pool["experiment"] == selected_experiment]
    pool = pool.loc[pool["image_path"] == selected_image_path]
    if pool.empty:
        raise ValueError(
            f"No se encontró el candidato solicitado: experiment={selected_experiment!r}, image_path={selected_image_path!r}"
        )
    return pool.sort_values(["selection_score", "iou"], ascending=[True, False]).iloc[0]


def render_selected_figure(selected: pd.Series, output_png: Path, output_pdf: Path) -> None:
    image_path = PROJECT_ROOT / str(selected["image_path"])
    image = Image.open(image_path).convert("RGB")
    image_np = np.asarray(image)

    pred_box = np.array(
        [
            selected["pred_cx"],
            selected["pred_cy"],
            selected["pred_w"],
            selected["pred_h"],
            selected["pred_angle_deg"],
        ],
        dtype=np.float32,
    )
    gt_box = np.array(
        [
            selected["gt_cx"],
            selected["gt_cy"],
            selected["gt_w"],
            selected["gt_h"],
            selected["gt_angle_deg"],
        ],
        dtype=np.float32,
    )

    pred_pts = rotated_box_to_points(*pred_box)
    gt_pts = rotated_box_to_points(*gt_box)
    all_pts = np.vstack([pred_pts, gt_pts])
    min_x, min_y = np.min(all_pts, axis=0)
    max_x, max_y = np.max(all_pts, axis=0)
    pad_x = max((max_x - min_x) * 0.55, 40.0)
    pad_y = max((max_y - min_y) * 0.55, 40.0)
    crop_x0 = max(int(min_x - pad_x), 0)
    crop_y0 = max(int(min_y - pad_y), 0)
    crop_x1 = min(int(max_x + pad_x), image_np.shape[1])
    crop_y1 = min(int(max_y + pad_y), image_np.shape[0])

    cause = str(selected["cause"])
    cause_text = {
        "solapamiento insuficiente": "fallo por IoU insuficiente",
        "desviación angular": "fallo por Δθ excesivo",
        "IoU + Δθ": "fallo por IoU y Δθ",
    }.get(cause, "fallo geométrico")

    fig, (ax_full, ax_zoom) = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={"width_ratios": [1.2, 1.0]})
    fig.suptitle(f"Caso limítrofe: {cause_text}", fontsize=16, fontweight="bold", y=0.98)

    for ax in (ax_full, ax_zoom):
        ax.imshow(image_np)
        draw_rect(ax, gt_box, color="#1b9e77", label="GT", linestyle="--")
        draw_rect(ax, pred_box, color="#d62728", label="Pred", linestyle="-")
        ax.axis("off")

    ax_full.set_title("Imagen completa", fontsize=12, fontweight="bold")
    ax_zoom.set_title("Zoom de la región de agarre", fontsize=12, fontweight="bold")
    ax_zoom.set_xlim(crop_x0, crop_x1)
    ax_zoom.set_ylim(crop_y1, crop_y0)

    legend_handles = [
        Line2D([0], [0], color="#1b9e77", lw=2.5, linestyle="--", label="GT"),
        Line2D([0], [0], color="#d62728", lw=2.5, linestyle="-", label="Pred"),
    ]
    ax_full.legend(handles=legend_handles, loc="upper right", frameon=True)

    footer = (
        f"Experimento: {selected['experiment']}  |  Seed: {int(selected['seed'])}  |  "
        f"Modelo: {selected['model']} ({selected['modality']})\n"
        f"Imagen: {selected['image_path']}  |  IoU = {selected['iou']:.3f}  |  "
        f"Δθ = {selected['angle_deg']:.2f}°  |  Resultado = FALLO  |  "
        f"Causa principal = {cause}"
    )
    fig.text(0.5, 0.04, footer, ha="center", va="bottom", fontsize=11)
    fig.tight_layout(rect=[0.02, 0.09, 0.98, 0.95])

    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=300, bbox_inches="tight")
    fig.savefig(output_pdf, bbox_inches="tight")
    plt.close(fig)


def write_traceability(trace_path: Path, selected: pd.Series, checkpoints: list[SelectedCheckpoint], candidates: pd.DataFrame, script_path: Path, output_png: Path, output_pdf: Path) -> None:
    lines = [
        "# Trazabilidad figura 5-15 alternativa",
        "",
        f"- Script utilizado: `{script_path}`",
        f"- Figura PNG: `{output_png}`",
        f"- Figura PDF: `{output_pdf}`",
        f"- Muestra seleccionada: `{selected['image_path']}`",
        f"- Experimento: `{selected['experiment']}`",
        f"- Seed: `{int(selected['seed'])}`",
        f"- Modelo: `{selected['model']}`",
        f"- Modalidad: `{selected['modality']}`",
        f"- Checkpoint: `{selected['checkpoint_path']}`",
        f"- IoU: `{selected['iou']:.6f}`",
        f"- Δθ: `{selected['angle_deg']:.6f}`",
        f"- Resultado: `fail`",
        f"- Causa principal: `{selected['cause']}`",
        "",
        "## Checkpoints evaluados",
        "",
    ]

    for info in checkpoints:
        lines.append(
            f"- `{info.experiment}` | seed `{info.seed}` | `{info.model_name}` | `{info.modality}` | val_success `{info.val_success:.4f}` | `{info.checkpoint_path}`"
        )

    lines.extend(["", "## Top candidatos", ""])
    preview_cols = ["experiment", "seed", "model", "modality", "image_path", "iou", "angle_deg", "cause", "selection_score"]
    lines.append("```csv")
    lines.append(candidates[preview_cols].to_csv(index=False).strip())
    lines.append("```")
    lines.append("")
    trace_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiments-root", type=Path, default=PROJECT_ROOT / "experiments")
    ap.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    ap.add_argument("--top-k", type=int, default=5)
    ap.add_argument("--batch-size", type=int, default=32)
    ap.add_argument("--selected-experiment")
    ap.add_argument("--selected-image-path")
    ap.add_argument(
        "--experiments",
        nargs="*",
        default=list(CONFIG_BY_EXPERIMENT.keys()),
        choices=sorted(CONFIG_BY_EXPERIMENT.keys()),
    )
    ap.add_argument("--base-name", default=DEFAULT_BASE_NAME)
    args = ap.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    checkpoints = choose_checkpoints(args.experiments_root, args.experiments)
    all_rows = [infer_candidates_for_checkpoint(info, batch_size=args.batch_size) for info in checkpoints]
    merged = pd.concat(all_rows, ignore_index=True)
    candidates = filter_top_candidates(merged, top_k=args.top_k)
    if candidates.empty:
        raise RuntimeError("No se encontraron candidatos de falso negativo.")

    selected = select_candidate(
        merged=merged,
        candidates=candidates,
        selected_experiment=args.selected_experiment,
        selected_image_path=args.selected_image_path,
    )
    output_png = output_dir / f"{args.base_name}.png"
    output_pdf = output_dir / f"{args.base_name}.pdf"
    output_csv = output_dir / f"{args.base_name}_candidates.csv"
    output_trace = output_dir / f"{args.base_name}_trace.md"

    candidates.to_csv(output_csv, index=False)
    render_selected_figure(selected, output_png=output_png, output_pdf=output_pdf)
    write_traceability(
        trace_path=output_trace,
        selected=selected,
        checkpoints=checkpoints,
        candidates=candidates,
        script_path=Path(__file__).resolve(),
        output_png=output_png,
        output_pdf=output_pdf,
    )

    print(f"[OK] Figura final: {output_png}")
    print(f"[OK] Candidatos: {output_csv}")
    print(f"[OK] Trazabilidad: {output_trace}")
    print(
        f"[BEST] {selected['experiment']} seed={int(selected['seed'])} "
        f"image={selected['image_path']} IoU={selected['iou']:.3f} "
        f"angle={selected['angle_deg']:.2f} cause={selected['cause']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
