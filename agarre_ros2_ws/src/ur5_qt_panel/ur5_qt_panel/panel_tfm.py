#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""TFM inference helpers for the panel."""
from __future__ import annotations

import json
import os
import shlex
import time
from pathlib import Path
from typing import Optional

from .panel_config import INFER_CKPT, INFER_ROI_SIZE, INFER_SCRIPT, LOG_DIR, VISION_DIR
from .panel_utils import ensure_dir, run_cmd


def _clip_roi(frame_w: int, frame_h: int, roi: tuple[int, int, int]) -> Optional[tuple[int, int, int, int]]:
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
    roi_mode = str(os.environ.get("PANEL_TFM_INFER_USE_ROI", "auto") or "auto").strip().lower()
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


def _prepare_module_artifacts(qimg) -> tuple[str, str]:
    out_dir = os.path.join(LOG_DIR, "panel_infer")
    ensure_dir(out_dir)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    image_path = os.path.join(out_dir, f"frame_{stamp}.png")
    out_path = os.path.join(out_dir, f"grasp_{stamp}.json")
    try:
        if not qimg.save(image_path):
            image_path = ""
    except Exception:
        image_path = ""
    return image_path, out_path


def _selection_snapshot(panel) -> dict:
    selected_px = getattr(panel, "_selected_px", None)
    selected_world = getattr(panel, "_selected_world", None)
    selected_base = getattr(panel, "_selected_base", None)
    return {
        "name": str(getattr(panel, "_selected_object", "") or ""),
        "px": tuple(selected_px) if selected_px is not None else None,
        "world": tuple(selected_world) if selected_world is not None else None,
        "base": tuple(selected_base) if selected_base is not None else None,
        "base_frame": str(getattr(panel, "_selected_base_frame", "") or ""),
        "timestamp": float(getattr(panel, "_selection_timestamp", 0.0) or 0.0),
    }


def tfm_infer(panel) -> tuple[bool, str]:
    if panel._tfm_infer_inflight:
        panel._set_status("TFM: inferencia en curso", error=False)
        return False, "inferencia en curso"
    experiment_ready, experiment_reason = panel._tfm_experiment_ready_status()
    if not experiment_ready:
        panel._set_status(f"TFM bloqueado: {experiment_reason}", error=True)
        panel._emit_log(f"[SAFETY] TFM infer bloqueado: {experiment_reason}")
        panel._audit_append("logs/infer.log", f"[TFM] infer_blocked reason={experiment_reason}")
        return False, experiment_reason
    infer_ready, infer_reason = panel._tfm_infer_ready_status()
    if not infer_ready:
        panel._set_status(f"TFM en espera: {infer_reason}", error=True)
        panel._emit_log(f"[SAFETY] TFM infer bloqueado: {infer_reason}")
        panel._audit_append("logs/infer.log", f"[TFM] infer_blocked reason={infer_reason}")
        return False, infer_reason
    frame_snapshot = panel._latest_camera_frame_snapshot() if hasattr(panel, "_latest_camera_frame_snapshot") else panel._last_camera_frame
    if not frame_snapshot:
        panel._set_status("TFM en espera: sin frame de cámara", error=True)
        panel._emit_log("[SAFETY] TFM infer bloqueado: sin frame de cámara")
        panel._audit_append("logs/infer.log", "[TFM] infer_blocked reason=sin frame de cámara")
        return False, "sin frame de cámara"
    qimg, w, h, frame_ts = frame_snapshot
    roi = _resolve_infer_roi(panel)
    selection_snapshot = _selection_snapshot(panel)

    ckpt_path = panel._tfm_get_ckpt_path() or INFER_CKPT
    if panel.tfm_module:
        if ckpt_path and os.path.isfile(ckpt_path):
            if (not panel.tfm_module.is_model_loaded()) or (ckpt_path != panel._tfm_ckpt_selected):
                panel.tfm_module.load_model(ckpt_path)
        if panel.tfm_module.is_model_loaded():
            image_path, out_path = _prepare_module_artifacts(qimg)
            model_info = panel.tfm_module.model_info()
            in_channels = int((model_info or {}).get("in_channels", 0) or 0)
            preprocessed = None
            preprocessed = _get_cached_preprocessed_input(
                panel,
                frame_ts=frame_ts,
                in_channels=in_channels,
                roi=roi,
            )
            if preprocessed is None:
                preprocessed = build_tfm_preprocessed_input(panel, qimg, w, h, frame_ts, roi=roi)
            if in_channels >= 4 and preprocessed is None:
                depth_required, depth_topic = panel._camera_depth_expectation()
                reason = (
                    f"depth no disponible para inferencia RGB-D ({depth_topic})"
                    if depth_required
                    else "entrada RGB-D no disponible"
                )
                panel._set_status(f"TFM en espera: {reason}", error=True)
                panel._emit_log(f"[SAFETY] TFM infer bloqueado: {reason}")
                panel._audit_append("logs/infer.log", f"[TFM] infer_blocked reason={reason}")
                return False, reason
            if preprocessed is not None:
                panel.tfm_module.set_input_image(
                    preprocessed,
                    width=w,
                    height=h,
                    roi=roi,
                    preprocessed=True,
                )
            else:
                panel.tfm_module.set_input_image(qimg, width=w, height=h, roi=roi)
            panel._audit_append(
                "logs/infer.log",
                f"[TFM] infer_start session={panel._infer_session_id} "
                f"mode=module ckpt={ckpt_path} camera={panel.camera_topic} roi={roi} "
                f"selected={selection_snapshot.get('name') or 'none'} image={image_path or 'none'}",
            )
            start_ts = time.monotonic()
            panel._tfm_infer_inflight = True
            panel._set_status("TFM: inferencia en curso…", error=False)

            def worker():
                pred = panel.tfm_module.infer_grasp_params() if panel.tfm_module else None
                infer_ms = (time.monotonic() - start_ts) * 1000.0
                total_ms = max(0.0, (time.monotonic() - float(frame_ts)) * 1000.0)
                err = ""
                if not pred and panel.tfm_module:
                    err = panel.tfm_module.last_error()
                if pred and out_path:
                    try:
                        Path(out_path).write_text(json.dumps(pred, ensure_ascii=True), encoding="utf-8")
                    except Exception:
                        pass
                result = {
                    "ok": bool(pred),
                    "pred": pred,
                    "roi": tuple(roi) if roi is not None else None,
                    "infer_ms": infer_ms,
                    "total_ms": total_ms,
                    "frame_w": w,
                    "frame_h": h,
                    "frame_ts": frame_ts,
                    "image_path": image_path,
                    "out_path": out_path,
                    "selection_snapshot": selection_snapshot,
                    "error": err or "infer_grasp sin salida válida",
                }
                panel.signal_run_ui.emit(lambda: panel._handle_infer_result(result))

            panel._run_async(worker, name="infer_grasp")
            return True, "inferencia iniciada"
        else:
            panel._log_warning("[TFM] Modelo no cargado; usando inferencia por script.")

    infer_script = INFER_SCRIPT
    if not infer_script or not os.path.isfile(infer_script):
        candidate = os.path.join(VISION_DIR, "scripts", "predict.py")
        if os.path.isfile(candidate):
            infer_script = candidate
    if not infer_script or not os.path.isfile(infer_script):
        panel._set_status("TFM: script de inferencia no disponible", error=True)
        return False, "script de inferencia no disponible"
    if not ckpt_path or not os.path.isfile(ckpt_path):
        panel._set_status("TFM: checkpoint no disponible", error=True)
        return False, "checkpoint no disponible"
    out_dir = os.path.join(LOG_DIR, "panel_infer")
    ensure_dir(out_dir)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    image_path = os.path.join(out_dir, f"frame_{stamp}.png")
    out_path = os.path.join(out_dir, f"grasp_{stamp}.json")
    if not qimg.save(image_path):
        panel._set_status("TFM: no se pudo guardar imagen", error=True)
        return False, "no se pudo guardar imagen"
    roi_args = ""
    if roi:
        roi_cx, roi_cy, roi_size = roi
        roi_args = f" --roi-cx {roi_cx} --roi-cy {roi_cy} --roi-size {roi_size}"
    script_name = os.path.basename(infer_script)
    if script_name == "predict.py":
        cfg_path = str(getattr(panel, "_exp_info", {}).get("config_path", "") or "")
        if not cfg_path or not os.path.isfile(cfg_path):
            panel._load_experiment_info()
            cfg_path = str(getattr(panel, "_exp_info", {}).get("config_path", "") or "")
        if not cfg_path or not os.path.isfile(cfg_path):
            panel._set_status("TFM: config del experimento no disponible", error=True)
            return False, "config del experimento no disponible"
        cmd = (
            f"python3 {shlex.quote(infer_script)}"
            f" --config {shlex.quote(cfg_path)}"
            f" --checkpoint {shlex.quote(ckpt_path)}"
            f" --image {shlex.quote(image_path)}"
        )
    else:
        cmd = (
            f"python3 {shlex.quote(infer_script)}"
            f" --image {shlex.quote(image_path)}"
            f" --ckpt {shlex.quote(ckpt_path)}"
            f" --out {shlex.quote(out_path)}"
            f"{roi_args}"
        )
    start_ts = time.time()
    panel._tfm_infer_inflight = True
    panel._set_status("TFM: inferencia en curso…", error=False)
    panel._audit_append(
        "logs/infer.log",
        f"[TFM] infer_start session={panel._infer_session_id} "
        f"mode=script ckpt={ckpt_path} camera={panel.camera_topic} roi={roi} "
        f"selected={selection_snapshot.get('name') or 'none'} image={image_path}",
    )

    def worker():
        res = run_cmd(cmd, timeout=30.0, capture_output=True)
        infer_ms = (time.time() - start_ts) * 1000.0
        total_ms = (time.time() - frame_ts) * 1000.0
        pred = None
        err = ""
        if os.path.isfile(out_path):
            try:
                pred = json.loads(Path(out_path).read_text(encoding="utf-8"))
            except Exception as exc:
                err = f"{exc}"
        if not pred:
            raw = (res.stdout or "").strip()
            if raw:
                try:
                    pred = json.loads(raw)
                except Exception as exc:
                    err = f"{exc}"
        # Fallback for scripts/predict.py CSV-like output:
        #   pred_cx,pred_cy,pred_w,pred_h,pred_angle_deg\n<vals>
        if not pred:
            raw = (res.stdout or "").strip()
            lines = [ln.strip() for ln in raw.splitlines() if ln.strip()]
            if len(lines) >= 2 and "pred_cx" in lines[0]:
                try:
                    vals = [float(v.strip()) for v in lines[1].split(",")]
                    if len(vals) >= 5:
                        pred = {
                            "cx": vals[0],
                            "cy": vals[1],
                            "w": vals[2],
                            "h": vals[3],
                            "angle_deg": vals[4],
                        }
                except Exception as exc:
                    err = f"{exc}"
        if res.returncode != 0 and not err:
            err = (res.stderr or res.stdout or "error en inferencia").strip()
        result = {
            "ok": bool(pred) and res.returncode == 0,
            "pred": pred,
            "infer_ms": infer_ms,
            "total_ms": total_ms,
            "frame_w": w,
            "frame_h": h,
            "frame_ts": frame_ts,
            "image_path": image_path,
            "out_path": out_path,
            "selection_snapshot": selection_snapshot,
            "error": err,
        }
        panel.signal_run_ui.emit(lambda: panel._handle_infer_result(result))

    panel._run_async(worker, name="infer_grasp")
    return True, "inferencia iniciada"
