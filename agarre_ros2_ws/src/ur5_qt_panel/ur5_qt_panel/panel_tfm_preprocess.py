#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_preprocess.py
# Contenido: Helpers de preprocesamiento + reconciliacion del flujo TFM grasp.
"""Helpers de preprocesamiento + reconciliacion del flujo TFM grasp.

Extraidos de ``panel_tfm.py`` (lineas 24-243 originales). Cubre:

* Clipping de ROI a bordes de la imagen.
* Resolucion del ROI activo (consulta env y panel state).
* Reconciliacion de la prediccion bruta con un objeto de referencia
  (size, center, angle) — ajusta cuando el modelo se acerca al
  objeto pero subestima area / desalinea centro / discrepa angulo.
* Cache simple de tensores preprocesados por frame_ts.
* Construccion del tensor de entrada para el modelo TFM
  (RGB o RGBD segun ``in_channels``).

Las funciones puras (`_clip_roi`, `reconcile_inferred_grasp_*`)
son testables sin panel. El resto requieren un panel o un mock.
"""
from __future__ import annotations

from typing import Optional

from .panel_config import INFER_ROI_SIZE
from .panel_tfm_params import get_panel_tfm_params as _get_panel_tfm_params


def _clip_roi(
    frame_w: int, frame_h: int, roi: tuple[int, int, int]
) -> Optional[tuple[int, int, int, int]]:
    roi_cx, roi_cy, roi_size = roi
    roi_size = int(max(1, min(int(roi_size), frame_w, frame_h)))
    if roi_size <= 0:
        return None
    cx = int(round(max(0, min(frame_w - 1, roi_cx))))
    cy = int(round(max(0, min(frame_h - 1, roi_cy))))
    x0 = int(round(cx - roi_size / 2.0))
    y0 = int(round(cy - roi_size / 2.0))
    x0 = max(0, min(frame_w - roi_size, x0))
    y0 = max(0, min(frame_h - roi_size, y0))
    return x0, y0, roi_size, roi_size


def _resolve_infer_roi(panel) -> Optional[tuple[int, int, int]]:
    selected_px = getattr(panel, "_selected_px", None)
    if not selected_px or int(INFER_ROI_SIZE or 0) <= 0:
        return None
    roi_mode = _get_panel_tfm_params().infer_use_roi
    if roi_mode in ("0", "false", "off", "no", "full", "disabled"):
        return None
    if roi_mode not in ("1", "true", "on", "yes", "auto", ""):
        roi_mode = "auto"
    if roi_mode in ("auto", ""):
        selected_name = str(getattr(panel, "_selected_object", "") or "").strip()
        if not selected_name:
            return None
    roi_cx, roi_cy = selected_px
    return int(roi_cx), int(roi_cy), int(INFER_ROI_SIZE)


def reconcile_inferred_grasp_size(
    pred: Optional[dict],
    ref: Optional[dict],
    *,
    roi: Optional[tuple[int, int, int]] = None,
) -> tuple[Optional[dict], bool]:
    if not pred or not ref or not roi:
        return pred, False
    try:
        roi_size = float(max(1, int(roi[2])))
        pred_cx = float(pred.get("cx", 0.0) or 0.0)
        pred_cy = float(pred.get("cy", 0.0) or 0.0)
        pred_w = float(pred.get("w", 0.0) or 0.0)
        pred_h = float(pred.get("h", 0.0) or 0.0)
        ref_cx = float(ref.get("cx", 0.0) or 0.0)
        ref_cy = float(ref.get("cy", 0.0) or 0.0)
        ref_w = float(ref.get("w", 0.0) or 0.0)
        ref_h = float(ref.get("h", 0.0) or 0.0)
    except Exception:
        return pred, False
    if pred_w <= 0.0 or pred_h <= 0.0 or ref_w <= 0.0 or ref_h <= 0.0:
        return pred, False
    dist_px = ((pred_cx - ref_cx) ** 2 + (pred_cy - ref_cy) ** 2) ** 0.5
    center_close = dist_px <= max(12.0, min(roi_size * 0.25, 24.0))
    area_too_small = (pred_w * pred_h) < (ref_w * ref_h * 0.55)
    span_too_small = pred_w < (ref_w * 0.75) or pred_h < (ref_h * 0.75)
    if not center_close or (not area_too_small and not span_too_small):
        return pred, False
    adjusted = dict(pred)
    adjusted["w"] = ref_w
    adjusted["h"] = ref_h
    return adjusted, True


def reconcile_inferred_grasp_center(
    pred: Optional[dict],
    ref: Optional[dict],
    *,
    roi: Optional[tuple[int, int, int]] = None,
) -> tuple[Optional[dict], bool]:
    if not pred or not ref or not roi:
        return pred, False
    try:
        roi_size = float(max(1, int(roi[2])))
        pred_cx = float(pred.get("cx", 0.0) or 0.0)
        pred_cy = float(pred.get("cy", 0.0) or 0.0)
        ref_cx = float(ref.get("cx", 0.0) or 0.0)
        ref_cy = float(ref.get("cy", 0.0) or 0.0)
    except Exception:
        return pred, False
    dist_px = ((pred_cx - ref_cx) ** 2 + (pred_cy - ref_cy) ** 2) ** 0.5
    center_close = dist_px <= max(12.0, min(roi_size * 0.25, 24.0))
    if not center_close or dist_px < 1.5:
        return pred, False
    adjusted = dict(pred)
    blend_to_ref = 0.6
    adjusted["cx"] = pred_cx + ((ref_cx - pred_cx) * blend_to_ref)
    adjusted["cy"] = pred_cy + ((ref_cy - pred_cy) * blend_to_ref)
    return adjusted, True


def reconcile_inferred_grasp_angle(
    pred: Optional[dict],
    ref: Optional[dict],
    *,
    roi: Optional[tuple[int, int, int]] = None,
    object_shape: str = "",
) -> tuple[Optional[dict], bool]:
    if not pred or not ref or not roi:
        return pred, False
    shape = str(object_shape or "").strip().lower()
    if shape not in ("circle", "square"):
        return pred, False
    try:
        roi_size = float(max(1, int(roi[2])))
        pred_cx = float(pred.get("cx", 0.0) or 0.0)
        pred_cy = float(pred.get("cy", 0.0) or 0.0)
        pred_angle = float(pred.get("angle_deg", 0.0) or 0.0)
        ref_cx = float(ref.get("cx", 0.0) or 0.0)
        ref_cy = float(ref.get("cy", 0.0) or 0.0)
        ref_angle = float(ref.get("angle_deg", 0.0) or 0.0)
    except Exception:
        return pred, False
    dist_px = ((pred_cx - ref_cx) ** 2 + (pred_cy - ref_cy) ** 2) ** 0.5
    center_close = dist_px <= max(12.0, min(roi_size * 0.25, 24.0))
    if not center_close or abs(pred_angle - ref_angle) < 1.0:
        return pred, False
    adjusted = dict(pred)
    adjusted["angle_deg"] = ref_angle
    return adjusted, True


def _get_cached_preprocessed_input(panel, *, frame_ts: float, in_channels: int, roi=None):
    if roi is not None:
        return None
    cached = getattr(panel, "_tfm_preprocessed_cache", None)
    if not cached:
        return None
    try:
        cached_ts = float(cached[0])
        preprocessed = cached[1]
    except Exception:
        return None
    if cached_ts != float(frame_ts):
        return None
    cached_channels = 0
    try:
        if len(cached) >= 3:
            cached_channels = int(cached[2] or 0)
    except Exception:
        cached_channels = 0
    if cached_channels > 0 and cached_channels != int(in_channels):
        return None
    try:
        actual_channels = int(preprocessed.shape[0]) if getattr(preprocessed, "ndim", 0) == 3 else 0
    except Exception:
        actual_channels = 0
    if actual_channels > 0 and actual_channels != int(in_channels):
        return None
    return preprocessed


def _store_preprocessed_cache(panel, *, frame_ts: float, preprocessed, in_channels: int) -> None:
    panel._tfm_preprocessed_cache = (
        float(frame_ts),
        preprocessed,
        int(in_channels),
    )


def build_tfm_preprocessed_input(panel, qimg, w: int, h: int, frame_ts: float, roi=None):
    if not panel.tfm_module:
        return None
    try:
        from tfm_grasping.perception import PerceptionPipeline
    except Exception:
        return None

    info = panel.tfm_module.model_info() if panel.tfm_module else {}
    img_size = int(info.get("img_size", 224) or 224)
    in_channels = int(info.get("in_channels", 3) or 3)

    if in_channels == 4:
        depth_topic = str(getattr(panel, "_camera_depth_topic", "") or "").strip()
        ros_worker = getattr(panel, "ros_worker", None)
        if not depth_topic or ros_worker is None:
            return None
        depth_frame = ros_worker.get_latest_depth_frame(depth_topic)
        if not depth_frame:
            return None
        depth, _depth_ts = depth_frame
        rgb = PerceptionPipeline.qimage_to_rgb(qimg, width=w, height=h)
        if rgb is None:
            return None
        if roi is not None:
            roi_box = _clip_roi(w, h, roi)
            if roi_box is None:
                return None
            x0, y0, roi_w, roi_h = roi_box
            rgb = rgb[y0 : y0 + roi_h, x0 : x0 + roi_w]
            depth = depth[y0 : y0 + roi_h, x0 : x0 + roi_w]
        preprocessed = PerceptionPipeline.to_preprocessed_rgbd(rgb, depth, img_size=img_size)
    else:
        if roi is None:
            preprocessed = PerceptionPipeline.qimage_to_preprocessed(
                qimg,
                width=w,
                height=h,
                img_size=img_size,
            )
        else:
            rgb = PerceptionPipeline.qimage_to_rgb(qimg, width=w, height=h)
            if rgb is None:
                return None
            roi_box = _clip_roi(w, h, roi)
            if roi_box is None:
                return None
            x0, y0, roi_w, roi_h = roi_box
            rgb = rgb[y0 : y0 + roi_h, x0 : x0 + roi_w]
            preprocessed = PerceptionPipeline.to_preprocessed(rgb, img_size=img_size)

    if preprocessed is not None and roi is None:
        _store_preprocessed_cache(
            panel,
            frame_ts=frame_ts,
            preprocessed=preprocessed,
            in_channels=in_channels,
        )
    return preprocessed
