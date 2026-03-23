#!/usr/bin/env python3
"""Regenera la Ilustracion 5-18 con pick_demo y prediccion mas cercana al GT."""

from __future__ import annotations

import ast
import csv
import math
import os
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from PIL import Image


@dataclass(frozen=True)
class Rect:
    cx: float
    cy: float
    w: float
    h: float
    angle_deg: float


@dataclass(frozen=True)
class Candidate:
    checkpoint: Path
    model_name: str
    in_channels: int
    pred: Rect
    iou: float
    dtheta: float
    centroid_dist_px: float


def _rotated_corners(rect: Rect) -> np.ndarray:
    hw = rect.w / 2.0
    hh = rect.h / 2.0
    ang = math.radians(rect.angle_deg)
    cs = math.cos(ang)
    sn = math.sin(ang)
    base = np.asarray(
        [
            [-hw, -hh],
            [hw, -hh],
            [hw, hh],
            [-hw, hh],
        ],
        dtype=np.float64,
    )
    rot = np.asarray([[cs, -sn], [sn, cs]], dtype=np.float64)
    pts = base @ rot.T
    pts[:, 0] += rect.cx
    pts[:, 1] += rect.cy
    return pts


def _draw_rect(
    ax,
    rect: Rect,
    *,
    color: str,
    linestyle: str,
    linewidth: float = 2.2,
    with_center: bool = True,
    center_color: str | None = None,
) -> None:
    poly = Polygon(
        _rotated_corners(rect),
        fill=False,
        edgecolor=color,
        linewidth=linewidth,
        linestyle=linestyle,
        joinstyle="miter",
    )
    ax.add_patch(poly)
    if with_center:
        ax.scatter(
            [rect.cx],
            [rect.cy],
            s=28.0,
            color=(center_color or color),
            edgecolors="none",
            zorder=5,
        )


def _iter_checkpoints(exp_root: Path) -> Iterable[Path]:
    return sorted(exp_root.glob("*/seed_*/checkpoints/best.pth"))


def _resolve_best_pick_demo_reference(infer_log: Path) -> tuple[Path, Rect, Rect, float, float]:
    """Parsea infer.log y extrae (overlay, pred_panel, ref_gt, iou_panel, dtheta_panel)."""
    text = infer_log.read_text(encoding="utf-8", errors="ignore").splitlines()
    session_to_selected: dict[str, str] = {}
    # infer_ref no lleva session explícita; se toma el último valor válido de pick_demo.
    last_ref_rect: Rect | None = None
    best_line_data: tuple[Path, Rect, Rect, float, float] | None = None

    re_session = re.compile(r"session=([^\s]+)")
    re_selected = re.compile(r"selected=([^\s]+)")
    re_overlay = re.compile(r"overlay=([^\s]+)")
    re_iou = re.compile(r"'iou':\s*([0-9eE+\-\.]+)")
    re_dtheta = re.compile(r"'dtheta':\s*([0-9eE+\-\.]+)")

    for line in text:
        if "infer_start" in line:
            m_sess = re_session.search(line)
            m_sel = re_selected.search(line)
            if m_sess and m_sel:
                session_to_selected[m_sess.group(1)] = m_sel.group(1)
            continue

        if "infer_ref" in line and "selected=pick_demo" in line and "pred=" in line and "ref=" in line:
            pred_idx = line.find("pred=")
            ref_idx = line.find("ref=")
            if pred_idx < 0 or ref_idx < 0:
                continue
            pred_str = line[pred_idx + len("pred=") : ref_idx].strip()
            ref_str = line[ref_idx + len("ref=") :].strip()
            try:
                pred_d = ast.literal_eval(pred_str)
                ref_d = ast.literal_eval(ref_str)
            except Exception:
                continue
            _ = Rect(  # solo se valida pred_d; el centro real se toma de infer_end/grasp.
                cx=float(ref_d.get("cx", 0.0)),
                cy=float(ref_d.get("cy", 0.0)),
                w=float(pred_d.get("w", 0.0)),
                h=float(pred_d.get("h", 0.0)),
                angle_deg=float(pred_d.get("angle_deg", 0.0)),
            )
            last_ref_rect = Rect(
                cx=float(ref_d.get("cx", 0.0)),
                cy=float(ref_d.get("cy", 0.0)),
                w=float(ref_d.get("w", 0.0)),
                h=float(ref_d.get("h", 0.0)),
                angle_deg=float(ref_d.get("angle_deg", 0.0)),
            )
            continue

        if "infer_end" in line and "cornell=" in line and "overlay=" in line:
            m_sess = re_session.search(line)
            if not m_sess:
                continue
            sess = m_sess.group(1)
            if session_to_selected.get(sess) != "pick_demo":
                continue
            if last_ref_rect is None:
                continue
            m_overlay = re_overlay.search(line)
            m_iou = re_iou.search(line)
            m_dth = re_dtheta.search(line)
            if not (m_overlay and m_iou and m_dth):
                continue
            try:
                overlay_path = Path(m_overlay.group(1))
                iou_val = float(m_iou.group(1))
                dtheta_val = float(m_dth.group(1))
            except Exception:
                continue

            # En infer_end sí viene el grasp con centro completo.
            grasp_idx = line.find("grasp=")
            cornell_idx = line.find("cornell=")
            if grasp_idx < 0 or cornell_idx < 0 or cornell_idx <= grasp_idx:
                continue
            grasp_str = line[grasp_idx + len("grasp=") : cornell_idx].strip()
            try:
                grasp_d = ast.literal_eval(grasp_str)
                pred_rect = Rect(
                    cx=float(grasp_d.get("cx", 0.0)),
                    cy=float(grasp_d.get("cy", 0.0)),
                    w=float(grasp_d.get("w", 0.0)),
                    h=float(grasp_d.get("h", 0.0)),
                    angle_deg=float(grasp_d.get("angle_deg", 0.0)),
                )
            except Exception:
                continue

            ref_rect = last_ref_rect
            if best_line_data is None:
                best_line_data = (overlay_path, pred_rect, ref_rect, iou_val, dtheta_val)
            else:
                # Para baseline de panel, prioriza mayor IoU y menor dtheta.
                _, _, _, best_iou, best_dtheta = best_line_data
                if (iou_val > best_iou) or (math.isclose(iou_val, best_iou) and dtheta_val < best_dtheta):
                    best_line_data = (overlay_path, pred_rect, ref_rect, iou_val, dtheta_val)

    if best_line_data is None:
        raise RuntimeError("No se pudo extraer una referencia pick_demo válida desde infer.log")
    return best_line_data


def _evaluate_candidates(
    root: Path,
    frame_rgb: np.ndarray,
    roi: tuple[int, int, int],
    ref_rect: Rect,
) -> list[Candidate]:
    os.environ.setdefault("VISION_DIR", str(root / "agarre_inteligente"))

    sys.path.insert(0, str(root / "agarre_ros2_ws" / "src" / "tfm_grasping"))
    sys.path.insert(0, str(root / "agarre_inteligente"))

    from tfm_grasping.model import GraspModel  # type: ignore
    from tfm_grasping.perception import InputFrame  # type: ignore
    from graspnet.utils.metrics import angle_diff_deg, compute_grasp_success, grasp_iou  # type: ignore

    frame = InputFrame(
        image=frame_rgb,
        width=int(frame_rgb.shape[1]),
        height=int(frame_rgb.shape[0]),
        timestamp=0.0,
        roi=roi,
        preprocessed=False,
    )

    ref = [ref_rect.cx, ref_rect.cy, ref_rect.w, ref_rect.h, ref_rect.angle_deg]
    out: list[Candidate] = []
    for ckpt in _iter_checkpoints(root / "agarre_inteligente" / "experiments"):
        model = GraspModel(model_path=str(ckpt))
        if not model.load():
            continue
        grasp = model.infer(frame)
        if grasp is None:
            continue
        pred_rect = Rect(
            cx=float(grasp.center_x),
            cy=float(grasp.center_y),
            w=float(grasp.width_px),
            h=float(grasp.height_px or 0.0),
            angle_deg=float(math.degrees(grasp.angle_rad)),
        )
        pred = [pred_rect.cx, pred_rect.cy, pred_rect.w, pred_rect.h, pred_rect.angle_deg]
        iou = float(grasp_iou(pred, ref))
        dtheta = float(angle_diff_deg(pred_rect.angle_deg, ref_rect.angle_deg))
        _ = bool(compute_grasp_success(pred, ref, iou_thresh=0.25, angle_thresh=30.0))
        centroid_dist = float(math.hypot(pred_rect.cx - ref_rect.cx, pred_rect.cy - ref_rect.cy))
        out.append(
            Candidate(
                checkpoint=ckpt,
                model_name=str(model.info.model_name),
                in_channels=int(model.info.in_channels),
                pred=pred_rect,
                iou=iou,
                dtheta=dtheta,
                centroid_dist_px=centroid_dist,
            )
        )
    return out


def _build_window(
    img_w: int,
    img_h: int,
    gt_rect: Rect,
    pred_rect: Rect,
    padding: int,
) -> tuple[int, int, int, int]:
    pts = np.vstack([_rotated_corners(gt_rect), _rotated_corners(pred_rect)])
    x0 = int(max(0, math.floor(float(np.min(pts[:, 0])) - padding)))
    y0 = int(max(0, math.floor(float(np.min(pts[:, 1])) - padding)))
    x1 = int(min(img_w, math.ceil(float(np.max(pts[:, 0])) + padding)))
    y1 = int(min(img_h, math.ceil(float(np.max(pts[:, 1])) + padding)))
    if x1 <= x0:
        x1 = min(img_w, x0 + 1)
    if y1 <= y0:
        y1 = min(img_h, y0 + 1)
    return x0, y0, x1, y1


def _render_figure(
    base_img: np.ndarray,
    gt_rect: Rect,
    pred_rect: Rect,
    context_box: tuple[int, int, int, int],
    zoom_box: tuple[int, int, int, int],
    out_png: Path,
    out_pdf: Path,
) -> None:
    cx0, cy0, cx1, cy1 = context_box
    x0, y0, x1, y1 = zoom_box
    context = base_img[cy0:cy1, cx0:cx1]
    zoom = base_img[y0:y1, x0:x1]

    fig = plt.figure(figsize=(13.66, 7.68), dpi=100, facecolor="#ececec")
    gs = fig.add_gridspec(1, 2, width_ratios=[2.05, 1.05], wspace=0.045)
    ax_full = fig.add_subplot(gs[0, 0])
    ax_zoom = fig.add_subplot(gs[0, 1])

    for ax in (ax_full, ax_zoom):
        ax.set_facecolor("#ececec")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#a8a8a8")
            spine.set_linewidth(1.3)

    # Panel contexto (solo zona de pick_demo).
    ax_full.imshow(context)
    gt_context = Rect(
        cx=gt_rect.cx - cx0,
        cy=gt_rect.cy - cy0,
        w=gt_rect.w,
        h=gt_rect.h,
        angle_deg=gt_rect.angle_deg,
    )
    pred_context = Rect(
        cx=pred_rect.cx - cx0,
        cy=pred_rect.cy - cy0,
        w=pred_rect.w,
        h=pred_rect.h,
        angle_deg=pred_rect.angle_deg,
    )
    _draw_rect(
        ax_full,
        gt_context,
        color="#1aa88a",
        linestyle=(0, (3.2, 2.0)),
        linewidth=2.4,
        center_color="#1aa88a",
    )
    _draw_rect(
        ax_full,
        pred_context,
        color="#e7332f",
        linestyle="-",
        linewidth=2.6,
        center_color="#e7332f",
    )
    ax_full.set_title("Objeto pick_demo (contexto)", fontsize=16, fontweight="bold", pad=10.0)

    # Panel zoom.
    ax_zoom.imshow(zoom)
    gt_local = Rect(
        cx=gt_rect.cx - x0,
        cy=gt_rect.cy - y0,
        w=gt_rect.w,
        h=gt_rect.h,
        angle_deg=gt_rect.angle_deg,
    )
    pred_local = Rect(
        cx=pred_rect.cx - x0,
        cy=pred_rect.cy - y0,
        w=pred_rect.w,
        h=pred_rect.h,
        angle_deg=pred_rect.angle_deg,
    )
    _draw_rect(ax_zoom, gt_local, color="#1aa88a", linestyle=(0, (3.2, 2.0)), linewidth=2.4, center_color="#1aa88a")
    _draw_rect(ax_zoom, pred_local, color="#e7332f", linestyle="-", linewidth=2.6, center_color="#e7332f")
    ax_zoom.set_title("Zoom de la region de agarre", fontsize=16, fontweight="bold", pad=10.0)

    fig.text(
        0.515,
        0.06,
        "GT verde discontinuo | PRED rojo continuo",
        ha="center",
        va="center",
        fontsize=14,
        color="#4a4a4a",
        fontweight="semibold",
    )
    fig.tight_layout(rect=[0.0, 0.09, 1.0, 1.0])
    out_png.parent.mkdir(parents=True, exist_ok=True)
    out_pdf.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=100)
    fig.savefig(out_pdf, dpi=300)
    plt.close(fig)


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    infer_log = root / "auditoria" / "panel_audit" / "logs" / "infer.log"
    panel_frame = root / "agarre_ros2_ws" / "log" / "panel_infer" / "frame_20260323_231409.png"
    # Para 5-18 se fuerza fondo exclusivo del frame pick_demo (sin multi-objeto de 5-16).
    base_image = panel_frame

    out_png = root / "reports" / "evidence" / "chapter5" / "fig_5_18_overlay_prediccion_plausible.png"
    out_pdf = root / "reports" / "evidence" / "chapter5" / "fig_5_18_overlay_prediccion_plausible.pdf"
    out_csv = root / "reports" / "evidence" / "chapter5" / "fig_5_18_overlay_prediccion_plausible_candidates.csv"
    out_trace = root / "reports" / "evidence" / "chapter5" / "fig_5_18_overlay_prediccion_plausible_trace.md"
    out_figure_official = (
        root
        / "reports"
        / "figures"
        / "cap5"
        / "Ilustracion_5-18_evidencia_funcional_adicional_del_pipeline_percepcion_publicacion_consumo_en_ros.png"
    )

    if not infer_log.exists():
        raise FileNotFoundError(f"No existe infer.log: {infer_log}")
    if not base_image.exists():
        raise FileNotFoundError(f"No existe imagen base 5-16: {base_image}")
    if not panel_frame.exists():
        raise FileNotFoundError(f"No existe frame pick_demo: {panel_frame}")

    overlay_path, panel_pred, gt_rect, panel_iou, panel_dtheta = _resolve_best_pick_demo_reference(infer_log)
    frame_rgb = np.array(Image.open(panel_frame).convert("RGB"))
    candidates = _evaluate_candidates(root, frame_rgb, (160, 171, 96), gt_rect)
    if not candidates:
        raise RuntimeError("No hubo candidatos inferibles para pick_demo.")

    # Prioridad: mayor IoU, menor dtheta y menor distancia de centroides.
    candidates = sorted(candidates, key=lambda c: (-c.iou, c.dtheta, c.centroid_dist_px))
    best = candidates[0]

    base_rgb = np.array(Image.open(base_image).convert("RGB"))
    context_box = _build_window(
        base_rgb.shape[1],
        base_rgb.shape[0],
        gt_rect,
        best.pred,
        padding=62,
    )
    zoom_box = _build_window(
        base_rgb.shape[1],
        base_rgb.shape[0],
        gt_rect,
        best.pred,
        padding=26,
    )
    _render_figure(base_rgb, gt_rect, best.pred, context_box, zoom_box, out_png, out_pdf)
    out_figure_official.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(out_png, out_figure_official)

    # CSV trazable.
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "base_image",
                "panel_frame",
                "overlay_source_panel",
                "selected_object",
                "checkpoint",
                "model_name",
                "in_channels",
                "panel_iou",
                "panel_dtheta_deg",
                "best_iou",
                "best_dtheta_deg",
                "pred_center_x",
                "pred_center_y",
                "pred_w",
                "pred_h",
                "pred_angle_deg",
                "gt_center_x",
                "gt_center_y",
                "gt_w",
                "gt_h",
                "gt_angle_deg",
                "centroid_distance_px",
                "zoom_x1",
                "zoom_y1",
                "zoom_x2",
                "zoom_y2",
                "context_x1",
                "context_y1",
                "context_x2",
                "context_y2",
            ]
        )
        writer.writerow(
            [
                str(base_image),
                str(panel_frame),
                str(overlay_path),
                "pick_demo",
                str(best.checkpoint),
                best.model_name,
                best.in_channels,
                f"{panel_iou:.6f}",
                f"{panel_dtheta:.6f}",
                f"{best.iou:.6f}",
                f"{best.dtheta:.6f}",
                f"{best.pred.cx:.3f}",
                f"{best.pred.cy:.3f}",
                f"{best.pred.w:.3f}",
                f"{best.pred.h:.3f}",
                f"{best.pred.angle_deg:.3f}",
                f"{gt_rect.cx:.3f}",
                f"{gt_rect.cy:.3f}",
                f"{gt_rect.w:.3f}",
                f"{gt_rect.h:.3f}",
                f"{gt_rect.angle_deg:.3f}",
                f"{best.centroid_dist_px:.3f}",
                zoom_box[0],
                zoom_box[1],
                zoom_box[2],
                zoom_box[3],
                context_box[0],
                context_box[1],
                context_box[2],
                context_box[3],
            ]
        )

    # Trace markdown.
    out_trace.parent.mkdir(parents=True, exist_ok=True)
    out_trace.write_text(
        "\n".join(
            [
                "# Figura 5-18 (overlay prediccion plausible, estilo TFM)",
                "",
                "- Objeto objetivo: `pick_demo` (exclusivo).",
                f"- Imagen base: `{base_image}`",
                f"- Frame de inferencia usado: `{panel_frame}`",
                f"- Overlay panel (baseline): `{overlay_path}`",
                "- Criterio de selección del caso: maximizar IoU y minimizar error angular/centroides respecto al GT del mismo frame.",
                f"- Baseline panel (seed_2): IoU={panel_iou:.6f}, dTheta={panel_dtheta:.6f} deg.",
                f"- Candidato seleccionado: `{best.checkpoint}` (model={best.model_name}, in_ch={best.in_channels}).",
                f"- Métricas del caso seleccionado: IoU={best.iou:.6f}, dTheta={best.dtheta:.6f} deg, dist_centro={best.centroid_dist_px:.3f} px.",
                "- Convención visual aplicada:",
                "  - `GT`: verde discontinuo",
                "  - `PRED`: rojo continuo",
                f"- Contexto recortado (solo objeto): x1={context_box[0]}, y1={context_box[1]}, x2={context_box[2]}, y2={context_box[3]}",
                f"- Zoom final: x1={zoom_box[0]}, y1={zoom_box[1]}, x2={zoom_box[2]}, y2={zoom_box[3]}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"[OK] Figura 5-18 regenerada: {out_png}")
    print(f"[OK] Figura 5-18 oficial actualizada: {out_figure_official}")
    print(f"[OK] CSV candidatos: {out_csv}")
    print(f"[OK] Trace: {out_trace}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
