#!/usr/bin/env python3
"""Compone las figuras finales para responder a los comentarios 2, 3 y 4."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import pandas as pd
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[3]
OUTPUT_ROOT = PROJECT_ROOT / "reports" / "tfm_visual_revision"
FINAL_ROOT = OUTPUT_ROOT / "final"
MASTER_FIGURES_ROOT = PROJECT_ROOT / "reports" / "figures"
INVENTORY_CSV = OUTPUT_ROOT / "inventory" / "qualitative_inventory_combined.csv"

SUCCESS_CASES = [
    {
        "experiment": "EXP3_RESNET18_RGB_AUGMENT",
        "image_path": "data/raw/cornell/02/pcd0254r.png",
        "panel_label": "a",
    },
    {
        "experiment": "EXP3_RESNET18_RGB_AUGMENT",
        "image_path": "data/raw/cornell/02/pcd0284r.png",
        "panel_label": "b",
    },
    {
        "experiment": "EXP3_RESNET18_RGB_AUGMENT",
        "image_path": "data/raw/cornell/06/pcd0611r.png",
        "panel_label": "c",
    },
    {
        "experiment": "EXP3_RESNET18_RGB_AUGMENT",
        "image_path": "data/raw/cornell/06/pcd0691r.png",
        "panel_label": "d",
    },
]

FAILURE_CASES = [
    {
        "experiment": "EXP1_SIMPLE_RGB",
        "image_path": "data/raw/cornell/02/pcd0287r.png",
        "error_label": "E1",
    },
    {
        "experiment": "EXP1_SIMPLE_RGB",
        "image_path": "data/raw/cornell/02/pcd0268r.png",
        "error_label": "E2",
    },
    {
        "experiment": "EXP3_RESNET18_RGB_AUGMENT",
        "image_path": "data/raw/cornell/06/pcd0633r.png",
        "error_label": "E3",
    },
    {
        "experiment": "EXP1_SIMPLE_RGB",
        "image_path": "data/raw/cornell/02/pcd0250r.png",
        "error_label": "E4",
    },
]

ILLUSTRATIVE_CASE = {
    "experiment": "EXP3_RESNET18_RGB_AUGMENT",
    "image_path": "data/raw/cornell/09/pcd0914r.png",
    "crop_margin_px": 18,
}


def draw_rotated_rect(ax, cx, cy, width, height, angle_deg, color, linestyle, linewidth):
    half_w = width / 2.0
    half_h = height / 2.0
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    corners = [(-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)]
    pts = [(cx + dx * cos_a - dy * sin_a, cy + dx * sin_a + dy * cos_a) for dx, dy in corners]
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


def load_inventory() -> pd.DataFrame:
    df = pd.read_csv(INVENTORY_CSV)
    return df


def get_case_row(df: pd.DataFrame, experiment: str, image_path: str) -> pd.Series:
    rows = df[(df["experiment"] == experiment) & (df["image_path"] == image_path)]
    if rows.empty:
        raise FileNotFoundError(f"No se encontro el caso {experiment} | {image_path}")
    return rows.iloc[0]


def load_image(case_row: pd.Series) -> Image.Image:
    return Image.open(case_row["orig_image_path"]).convert("RGB").resize((224, 224))


def render_case(ax, case_row: pd.Series, title_prefix: str | None = None, add_footer: bool = False) -> None:
    image = load_image(case_row)
    ax.imshow(image)
    draw_rotated_rect(
        ax,
        case_row["gt_cx"],
        case_row["gt_cy"],
        case_row["gt_w"],
        case_row["gt_h"],
        case_row["gt_angle_deg"],
        "#13b24a",
        "--",
        2.8,
    )
    draw_rotated_rect(
        ax,
        case_row["pred_cx"],
        case_row["pred_cy"],
        case_row["pred_w"],
        case_row["pred_h"],
        case_row["pred_angle_deg"],
        "#d62828",
        "-",
        2.8,
    )
    ax.set_xticks([])
    ax.set_yticks([])
    metric_text = f"IoU={case_row['iou_display_gt']:.2f} | dtheta={case_row['angle_display_gt_deg']:.1f} deg"
    if title_prefix:
        metric_text = f"{title_prefix} | {metric_text}"
    ax.set_title(metric_text, fontsize=12, pad=10)
    if add_footer:
        ax.text(
            0.5,
            -0.08,
            Path(case_row["image_path"]).name,
            transform=ax.transAxes,
            ha="center",
            va="top",
            fontsize=9,
            color="#333333",
        )


def compose_success_figure(df: pd.DataFrame) -> Path:
    fig, axes = plt.subplots(2, 2, figsize=(11.8, 9.6))
    axes = axes.flatten()
    for ax, case in zip(axes, SUCCESS_CASES):
        row = get_case_row(df, case["experiment"], case["image_path"])
        render_case(ax, row, title_prefix=f"({case['panel_label']})", add_footer=False)
    fig.suptitle("Aciertos representativos en validacion", fontsize=16, fontweight="bold", y=0.98)
    fig.text(0.5, 0.02, "GT: verde discontinuo | Pred: rojo continuo", ha="center", fontsize=11)
    fig.tight_layout(rect=[0.02, 0.05, 0.98, 0.95])
    return save_dual(fig, "cap5_aciertos_cualitativos_v1")


def compose_failure_figure(df: pd.DataFrame) -> Path:
    fig, axes = plt.subplots(2, 2, figsize=(11.8, 9.6))
    axes = axes.flatten()
    for ax, case in zip(axes, FAILURE_CASES):
        row = get_case_row(df, case["experiment"], case["image_path"])
        render_case(ax, row, title_prefix=case["error_label"], add_footer=False)
        ax.text(
            0.03,
            0.93,
            case["error_label"],
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=14,
            fontweight="bold",
            color="white",
            bbox={"boxstyle": "round,pad=0.28", "facecolor": "#202020", "edgecolor": "none"},
        )
    fig.suptitle("Fallos representativos tipificados como E1-E4", fontsize=16, fontweight="bold", y=0.98)
    fig.text(0.5, 0.02, "GT: verde discontinuo | Pred: rojo continuo", ha="center", fontsize=11)
    fig.tight_layout(rect=[0.02, 0.05, 0.98, 0.95])
    return save_dual(fig, "cap5_fallos_cualitativos_E1_E4_v1")


def _crop_bounds(case_row: pd.Series, margin_px: int) -> tuple[int, int, int, int]:
    x_min = min(
        case_row["pred_cx"] - case_row["pred_w"] / 2.0,
        case_row["gt_cx"] - case_row["gt_w"] / 2.0,
    )
    y_min = min(
        case_row["pred_cy"] - case_row["pred_h"] / 2.0,
        case_row["gt_cy"] - case_row["gt_h"] / 2.0,
    )
    x_max = max(
        case_row["pred_cx"] + case_row["pred_w"] / 2.0,
        case_row["gt_cx"] + case_row["gt_w"] / 2.0,
    )
    y_max = max(
        case_row["pred_cy"] + case_row["pred_h"] / 2.0,
        case_row["gt_cy"] + case_row["gt_h"] / 2.0,
    )
    x0 = max(int(math.floor(x_min - margin_px)), 0)
    y0 = max(int(math.floor(y_min - margin_px)), 0)
    x1 = min(int(math.ceil(x_max + margin_px)), 224)
    y1 = min(int(math.ceil(y_max + margin_px)), 224)
    return x0, y0, x1, y1


def compose_illustrative_case(df: pd.DataFrame) -> Path:
    row = get_case_row(df, ILLUSTRATIVE_CASE["experiment"], ILLUSTRATIVE_CASE["image_path"])
    image = load_image(row)
    crop_bounds = _crop_bounds(row, margin_px=int(ILLUSTRATIVE_CASE["crop_margin_px"]))
    x0, y0, x1, y1 = crop_bounds
    crop = image.crop(crop_bounds)

    fig, axes = plt.subplots(1, 2, figsize=(11.2, 5.6), width_ratios=[1.0, 1.0])
    ax_full, ax_zoom = axes

    ax_full.imshow(image)
    draw_rotated_rect(ax_full, row["gt_cx"], row["gt_cy"], row["gt_w"], row["gt_h"], row["gt_angle_deg"], "#13b24a", "--", 2.6)
    draw_rotated_rect(ax_full, row["pred_cx"], row["pred_cy"], row["pred_w"], row["pred_h"], row["pred_angle_deg"], "#d62828", "-", 2.6)
    ax_full.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor="#222222", linewidth=2.2))
    ax_full.set_title("Imagen completa", fontsize=13, pad=8)
    ax_full.set_xticks([])
    ax_full.set_yticks([])

    ax_zoom.imshow(crop.resize((224, 224)))
    scale_x = 224.0 / max(x1 - x0, 1)
    scale_y = 224.0 / max(y1 - y0, 1)
    draw_rotated_rect(
        ax_zoom,
        (row["gt_cx"] - x0) * scale_x,
        (row["gt_cy"] - y0) * scale_y,
        row["gt_w"] * scale_x,
        row["gt_h"] * scale_y,
        row["gt_angle_deg"],
        "#13b24a",
        "--",
        3.0,
    )
    draw_rotated_rect(
        ax_zoom,
        (row["pred_cx"] - x0) * scale_x,
        (row["pred_cy"] - y0) * scale_y,
        row["pred_w"] * scale_x,
        row["pred_h"] * scale_y,
        row["pred_angle_deg"],
        "#d62828",
        "-",
        3.0,
    )
    ax_zoom.set_title("Recorte ampliado del area de agarre", fontsize=13, pad=8)
    ax_zoom.set_xticks([])
    ax_zoom.set_yticks([])

    fig.suptitle(
        "Caso ilustrativo: objeto pequeno en la escena y detalle ampliado",
        fontsize=16,
        fontweight="bold",
        y=0.98,
    )
    fig.text(
        0.5,
        0.04,
        f"IoU={row['iou_display_gt']:.2f} | dtheta={row['angle_display_gt_deg']:.1f} deg | GT: verde discontinuo | Pred: rojo continuo",
        ha="center",
        fontsize=11,
    )
    fig.tight_layout(rect=[0.02, 0.08, 0.98, 0.93])
    return save_dual(fig, "cap5_caso_ilustrativo_v1")


def save_dual(fig: plt.Figure, stem: str) -> Path:
    FINAL_ROOT.mkdir(parents=True, exist_ok=True)
    MASTER_FIGURES_ROOT.mkdir(parents=True, exist_ok=True)
    png_final = FINAL_ROOT / f"{stem}.png"
    pdf_final = FINAL_ROOT / f"{stem}.pdf"
    png_master = MASTER_FIGURES_ROOT / f"{stem}.png"
    pdf_master = MASTER_FIGURES_ROOT / f"{stem}.pdf"

    fig.savefig(png_final, dpi=320, bbox_inches="tight")
    fig.savefig(pdf_final, dpi=320, bbox_inches="tight")
    fig.savefig(png_master, dpi=320, bbox_inches="tight")
    fig.savefig(pdf_master, dpi=320, bbox_inches="tight")
    plt.close(fig)
    return png_final


def main() -> int:
    df = load_inventory()
    compose_success_figure(df)
    compose_failure_figure(df)
    compose_illustrative_case(df)
    print("[OK] Figuras finales generadas en final/ y reports/figures/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
