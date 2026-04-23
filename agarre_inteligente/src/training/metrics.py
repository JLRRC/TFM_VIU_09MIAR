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


# ---------------------------------------------------------------------------
# IoU con rectángulos orientados — alineación metodológica posterior al TFM
# ---------------------------------------------------------------------------
# Las funciones originales (iou_axis_aligned_boxes, cornell_success,
# summarize_batch) se conservan intactas para retrocompatibilidad con EXP1..EXP4.
# Las variantes siguientes implementan el criterio Cornell canónico con
# rectángulos orientados (Jiang et al. 2011 / Redmon & Angelova 2015).

def _rect_vertices(cx: float, cy: float, w: float, h: float, angle_deg: float) -> np.ndarray:
    """Devuelve los 4 vértices de un rectángulo orientado como array (4, 2).

    Parametrización: (cx, cy) centro, w ancho (eje largo), h alto (eje corto),
    angle_deg ángulo del eje largo respecto al eje X positivo, en [-90, 90].
    La rotación se aplica en coordenadas de imagen (Y hacia abajo).
    """
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    hw, hh = w / 2.0, h / 2.0
    local = np.array(
        [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=np.float64
    )
    # R.T de la matriz de rotación (cos, -sin / sin, cos)
    Rt = np.array([[cos_t, sin_t], [-sin_t, cos_t]])
    return local @ Rt + np.array([cx, cy])


def _is_inside_halfplane(point, edge_a, edge_b) -> bool:
    """True si point está en el semiplano interior de la arista edge_a→edge_b.

    El semiplano interior es el definido por los rectángulos generados con
    _rect_vertices (vértices en orden: UL, UR, LR, LL en coordenadas de imagen).
    El test cross >= 0 es consistente para ese orden de vértices.
    """
    return (
        (edge_b[0] - edge_a[0]) * (point[1] - edge_a[1])
        - (edge_b[1] - edge_a[1]) * (point[0] - edge_a[0])
    ) >= 0.0


def _line_intersect(p1, p2, q1, q2):
    """Intersección de las líneas p1-p2 y q1-q2. Asume que no son paralelas."""
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    x3, y3 = float(q1[0]), float(q1[1])
    x4, y4 = float(q2[0]), float(q2[1])
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-12:
        return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def _sutherland_hodgman(subject: list, clip: list) -> list:
    """Recorta el polígono convexo subject contra el polígono convexo clip.

    Ambos son listas de pares (x, y). Retorna la lista de vértices del
    polígono intersección (puede ser vacía).
    """
    output = list(subject)
    n = len(clip)
    for i in range(n):
        if not output:
            return []
        ea, eb = clip[i], clip[(i + 1) % n]
        inp = output
        output = []
        for j in range(len(inp)):
            curr = inp[j]
            prev = inp[j - 1]
            curr_in = _is_inside_halfplane(curr, ea, eb)
            prev_in = _is_inside_halfplane(prev, ea, eb)
            if curr_in:
                if not prev_in:
                    output.append(_line_intersect(prev, curr, ea, eb))
                output.append(curr)
            elif prev_in:
                output.append(_line_intersect(prev, curr, ea, eb))
    return output


def _polygon_area(vertices: list) -> float:
    """Área del polígono mediante la fórmula del shoelace."""
    n = len(vertices)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i][0] * vertices[j][1] - vertices[j][0] * vertices[i][1]
    return abs(area) * 0.5


def _iou_single_oriented(pred5: np.ndarray, gt5: np.ndarray) -> float:
    """IoU entre un par de rectángulos orientados (arrays de longitud 5)."""
    area_p = float(abs(pred5[2])) * float(abs(pred5[3]))
    area_g = float(abs(gt5[2])) * float(abs(gt5[3]))
    if area_p <= 0.0 or area_g <= 0.0:
        return 0.0
    pv = _rect_vertices(float(pred5[0]), float(pred5[1]), float(abs(pred5[2])), float(abs(pred5[3])), float(pred5[4]))
    gv = _rect_vertices(float(gt5[0]), float(gt5[1]), float(abs(gt5[2])), float(abs(gt5[3])), float(gt5[4]))
    inter_verts = _sutherland_hodgman(pv.tolist(), gv.tolist())
    inter_area = _polygon_area(inter_verts)
    union_area = area_p + area_g - inter_area
    if union_area <= 0.0:
        return 0.0
    return float(np.clip(inter_area / union_area, 0.0, 1.0))


def iou_oriented_boxes(pred: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """IoU entre rectángulos orientados.

    pred, gt: arrays (N, 5) con columnas [cx, cy, w, h, angle_deg].
    Soporta broadcasting: si uno tiene shape (1, 5) se empareja con cada fila del otro.
    Retorna array (N,) con valores en [0, 1].

    Implementación: intersección de polígonos convexos mediante Sutherland-Hodgman.
    """
    pred = np.atleast_2d(np.asarray(pred, dtype=np.float64))
    gt = np.atleast_2d(np.asarray(gt, dtype=np.float64))
    n = max(len(pred), len(gt))
    if len(pred) == 1 and len(gt) > 1:
        pred = np.tile(pred, (n, 1))
    elif len(gt) == 1 and len(pred) > 1:
        gt = np.tile(gt, (n, 1))
    result = np.empty(n, dtype=np.float64)
    for i in range(n):
        result[i] = _iou_single_oriented(pred[i], gt[i])
    return result


def cornell_success_oriented(
    pred: np.ndarray,
    gt: np.ndarray,
    iou_thr: float = 0.25,
    angle_thr: float = 30.0,
) -> np.ndarray:
    """Criterio Cornell con IoU de rectángulos orientados.

    Reemplaza iou_axis_aligned_boxes con iou_oriented_boxes. El umbral de IoU
    (0.25) y ángulo (30°) siguen siendo los del protocolo Cornell estándar.
    """
    iou = iou_oriented_boxes(pred, gt)
    ang = angle_error_deg(pred[:, 4], gt[:, 4])
    return (iou >= iou_thr) & (ang <= angle_thr)


def summarize_batch_oriented(pred: np.ndarray, gt: np.ndarray) -> dict[str, float]:
    """Como summarize_batch pero con IoU orientada.

    Devuelve las mismas claves para ser intercambiable con summarize_batch.
    """
    iou = iou_oriented_boxes(pred, gt)
    ang = angle_error_deg(pred[:, 4], gt[:, 4])
    succ = cornell_success_oriented(pred, gt)
    return {
        "val_iou": float(np.mean(iou)),
        "val_angle_deg": float(np.mean(ang)),
        "val_success": float(np.mean(succ.astype(np.float32))),
    }
