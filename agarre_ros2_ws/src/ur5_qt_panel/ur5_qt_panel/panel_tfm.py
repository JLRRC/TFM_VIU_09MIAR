#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""TFM inference helpers for the panel."""
from __future__ import annotations

import datetime
import json
import os
import re
import shlex
import time
from pathlib import Path
from typing import Optional

from .panel_config import INFER_CKPT, INFER_ROI_SIZE, INFER_SCRIPT, LOG_DIR, VISION_DIR
from .panel_utils import ensure_dir, run_cmd
from .panel_objects import get_object_position, get_object_state, get_object_states, is_on_table, update_object_state
from .panel_pick_object import run_pick_object
from .panel_utils import pixel_to_table_xy


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

# ── Additional imports needed by extended TFM block ──────────────────────────
import json
import math
import os
import threading
import time
import uuid
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .panel_config import (
    BASE_FRAME,
    GRIPPER_TCP_Z_OFFSET,
    OBJECT_SHAPES,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
)
from .panel_objects import ObjectLogicalState
from .panel_robot_presets import JOINT_TABLE_POSE_RAD
from .panel_camera import _runtime_time
from .panel_robot_presets import _make_pose_data
from .panel_objects import ObjectLogicalState

MOVEIT_POSE_TOPIC = "/desired_grasp"
MOVEIT_CARTESIAN_POSE_TOPIC = "/desired_grasp_cartesian"
def tfm_infer_grasp(panel):
    panel._log_button("TFM Inferir agarre")
    panel._emit_log(
        "[TFM][BUTTON] action=infer "
        f"applied={str(bool(getattr(panel, '_tfm_experiment_applied', False))).lower()} "
        f"inflight={str(bool(getattr(panel, '_tfm_infer_inflight', False))).lower()} "
        f"selected={str(getattr(panel, '_selected_object', '') or 'none')}"
    )
    return tfm_infer(panel)

def tfm_canonical_use_pick_object(panel) -> bool:
    return str(
        os.environ.get("PANEL_TFM_CANONICAL_USE_PICK_OBJECT", "1") or "1"
    ).strip().lower() not in ("0", "false", "no", "off")

def complete_pending_tfm_infer_request(panel, success: bool, message: str) -> None:
    request_id = str(getattr(panel, "_tfm_infer_pending_request_id", "") or "").strip()
    if not request_id:
        return
    panel._tfm_infer_pending_request_id = ""
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            panel.ros_worker.complete_tfm_infer_request(request_id, success, message)
        except Exception:
            pass

def complete_pending_tfm_execute_request(panel, success: bool, message: str) -> None:
    request_id = str(getattr(panel, "_tfm_execute_pending_request_id", "") or "").strip()
    if not request_id:
        return
    panel._tfm_execute_pending_request_id = ""
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            panel.ros_worker.complete_tfm_execute_request(request_id, success, message)
        except Exception:
            pass

def complete_pending_pick_demo_request(panel, success: bool, message: str) -> None:
    request_id = str(getattr(panel, "_pick_demo_pending_request_id", "") or "").strip()
    if not request_id:
        return
    panel._pick_demo_pending_request_id = ""
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            panel.ros_worker.complete_pick_demo_request(request_id, success, message)
        except Exception:
            pass

def build_tfm_pick_object_override(panel,
    *,
    grasp_base: Dict[str, float],
    selected_object: str,
    source: str,
) -> Dict[str, object]:
    return {
        "enabled": True,
        "mode": "tfm_moveit",
        "selected_object": str(selected_object or "").strip(),
        "frame": panel._business_base_frame(),
        "x": float(grasp_base.get("x", 0.0) or 0.0),
        "y": float(grasp_base.get("y", 0.0) or 0.0),
        "z": float(grasp_base.get("z", 0.0) or 0.0),
        "yaw_deg": float(grasp_base.get("yaw_deg", 0.0) or 0.0),
        "source": str(source or "unknown"),
        "created_ts": time.time(),
    }

def tfm_canonical_state_reset(panel,
    *,
    selected_object: str,
    grasp_base: Dict[str, float],
    source: str,
) -> None:
    session = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    panel._tfm_canonical_ctx = {
        "route": "TFM->MoveIt",
        "session": session,
        "selected_object": str(selected_object or "").strip(),
        "source": str(source or "unknown"),
        "grasp_base": dict(grasp_base),
        "state": "READY",
        "events": [],
        "start_ts": time.time(),
        "success": None,
        "message": "",
    }
    panel._audit_append(
        "logs/tfm_moveit_canonical.log",
        "[TFM][CANON] start "
        f"session={session} selected={selected_object or 'none'} "
        f"grasp=({float(grasp_base.get('x', 0.0) or 0.0):.3f},"
        f"{float(grasp_base.get('y', 0.0) or 0.0):.3f},"
        f"{float(grasp_base.get('z', 0.0) or 0.0):.3f}) "
        f"yaw={float(grasp_base.get('yaw_deg', 0.0) or 0.0):.1f} "
        f"source={source or 'unknown'}",
    )
    panel._audit_write_json(
        "artifacts/tfm_moveit_canonical_last.json",
        dict(panel._tfm_canonical_ctx),
    )

def tfm_canonical_phase_update(panel, state: str, *, detail: str = "") -> None:
    ctx = getattr(panel, "_tfm_canonical_ctx", None)
    if not isinstance(ctx, dict):
        return
    phase = str(state or "").strip() or "UNKNOWN"
    event = {
        "ts": time.time(),
        "state": phase,
        "detail": str(detail or ""),
    }
    events = ctx.setdefault("events", [])
    if isinstance(events, list):
        events.append(event)
    ctx["state"] = phase
    panel._audit_append(
        "logs/tfm_moveit_canonical.log",
        f"[TFM][CANON] state={phase} detail={detail or 'n/a'}",
    )
    panel._audit_write_json(
        "artifacts/tfm_moveit_canonical_last.json",
        dict(ctx),
    )

def tfm_canonical_finish(panel, success: bool, message: str, *, final_state: str) -> None:
    ctx = getattr(panel, "_tfm_canonical_ctx", None)
    end_ts = time.time()
    if isinstance(ctx, dict):
        ctx["success"] = bool(success)
        ctx["message"] = str(message or "")
        ctx["final_state"] = str(final_state or ("HOME_DONE" if success else "FAIL_TERMINAL"))
        ctx["state"] = ctx["final_state"]
        ctx["end_ts"] = end_ts
        ctx["duration_sec"] = max(0.0, end_ts - float(ctx.get("start_ts", end_ts) or end_ts))
        events = ctx.setdefault("events", [])
        if isinstance(events, list):
            events.append(
                {
                    "ts": end_ts,
                    "state": ctx["final_state"],
                    "detail": str(message or ""),
                }
            )
        panel._audit_append(
            "logs/tfm_moveit_canonical.log",
            "[TFM][CANON] finish "
            f"success={str(bool(success)).lower()} "
            f"final_state={ctx['final_state']} "
            f"duration={float(ctx.get('duration_sec', 0.0) or 0.0):.2f}s "
            f"message={message or 'n/a'}",
        )
        panel._audit_write_json(
            "artifacts/tfm_moveit_canonical_last.json",
            dict(ctx),
        )
    panel._pick_object_grasp_override = None
    panel._tfm_canonical_ctx = None
    panel._tfm_execute_inflight = False
    complete_pending_tfm_execute_request(panel, bool(success), str(message or ""))

def restore_infer_selection_snapshot(panel, snapshot: object) -> None:
    if not isinstance(snapshot, dict):
        return
    name = str(snapshot.get("name") or "").strip()
    if not name:
        return
    current_selected = str(getattr(panel, "_selected_object", "") or "").strip()
    if current_selected and current_selected != name:
        return
    selected_px = snapshot.get("px")
    selected_world = snapshot.get("world")
    selected_base = snapshot.get("base")
    if not current_selected:
        panel._selected_object = name
        panel._selected_px = tuple(selected_px) if selected_px is not None else None
        panel._selected_world = tuple(selected_world) if selected_world is not None else None
        panel._selected_base = tuple(selected_base) if selected_base is not None else None
        panel._selected_base_frame = str(snapshot.get("base_frame") or panel._business_base_frame())
    snap_ts = float(snapshot.get("timestamp", 0.0) or 0.0)
    if snap_ts > 0.0:
        if float(getattr(panel, "_selection_timestamp", 0.0) or 0.0) <= 0.0:
            panel._selection_timestamp = snap_ts
        if not panel._selection_last_user_name:
            panel._selection_last_user_name = name
        if not panel._selection_last_user_ts:
            panel._selection_last_user_ts = snap_ts
    store_ok = ensure_selected_object_in_store(panel, 
        name,
        reason="infer_snapshot_restore",
    )
    panel._emit_log(
        "[TFM][INFER] restored_selection_from_snapshot "
        f"name={name} px={panel._selected_px if panel._selected_px is not None else 'n/a'} "
        f"store_ok={str(bool(store_ok)).lower()}"
    )

def latest_camera_frame_snapshot(panel) -> Optional[Tuple[object, int, int, float]]:
    latest = panel._last_camera_frame
    with panel._camera_frame_lock:
        pending = panel._camera_pending_frame
    if pending:
        topic, qimg, w, h, _fps, ts = pending
        if topic == panel.camera_topic and int(w) > 2 and int(h) > 2:
            latest_ts = float(latest[3]) if latest is not None else 0.0
            if latest is None or float(ts) >= latest_ts:
                latest = (qimg, int(w), int(h), float(ts))
                panel._last_camera_frame = latest
    return latest

def ensure_selected_object_in_store(panel, name: str, *, reason: str) -> bool:
    target = str(name or "").strip()
    if not target:
        return False
    current = get_object_state(target)
    if current and current.logical_state == ObjectLogicalState.SELECTED:
        return True
    pos = get_object_position(target)
    if pos is None and current is not None:
        pos = tuple(current.position)
    if pos is None or not is_on_table(pos):
        panel._emit_log(
            "[PICK][SELECT_STORE] restore_skip "
            f"name={target} reason={reason} "
            f"pos={'n/a' if pos is None else f'({float(pos[0]):.3f},{float(pos[1]):.3f},{float(pos[2]):.3f})'}"
        )
        return False
    for other_name, other_state in get_object_states().items():
        if other_name == target:
            continue
        if other_state.logical_state == ObjectLogicalState.SELECTED:
            update_object_state(
                other_name,
                logical_state=ObjectLogicalState.ON_TABLE,
                reason=f"{reason}_clear_other",
            )
    ok = update_object_state(
        target,
        logical_state=ObjectLogicalState.SELECTED,
        reason=reason,
    )
    panel._emit_log(
        "[PICK][SELECT_STORE] restore "
        f"name={target} reason={reason} ok={str(bool(ok)).lower()}"
    )
    return bool(ok)

def handle_infer_result(panel, result: Dict[str, object]) -> None:
    panel._tfm_infer_inflight = False
    if not result.get("ok"):
        err = result.get("error") or "infer_grasp sin salida válida"
        panel._set_status(f"TFM: inferencia fallida ({err})", error=True)
        infer_ms = float(result.get("infer_ms", 0.0) or 0.0)
        total_ms = float(result.get("total_ms", 0.0) or 0.0)
        panel._audit_append(
            "logs/infer.log",
            f"[TFM] infer_end session={panel._infer_session_id} status=FAIL "
            f"infer_ms={infer_ms:.2f} total_ms={total_ms:.2f} err={err}",
        )
        if hasattr(panel, "btn_tfm_grasp_object"):
            panel.btn_tfm_grasp_object.setEnabled(False)
        complete_pending_tfm_infer_request(panel, False, f"inferencia fallida ({err})")
        return
    pred = result.get("pred") if isinstance(result.get("pred"), dict) else None
    if not pred:
        panel._set_status("TFM: salida inválida", error=True)
        panel._audit_append("logs/infer.log", "[TFM] infer_end status=FAIL err=salida_invalida")
        if hasattr(panel, "btn_tfm_grasp_object"):
            panel.btn_tfm_grasp_object.setEnabled(False)
        complete_pending_tfm_infer_request(panel, False, "salida inválida")
        return
    raw_pred = dict(pred)
    selection_snapshot = result.get("selection_snapshot")
    panel._last_infer_selection_snapshot = dict(selection_snapshot) if isinstance(selection_snapshot, dict) else {}
    restore_infer_selection_snapshot(panel, selection_snapshot)
    snapshot_name = ""
    if isinstance(selection_snapshot, dict):
        snapshot_name = str(selection_snapshot.get("name") or "").strip()
    panel._last_grasp_selection_name = str(
        getattr(panel, "_selected_object", "") or snapshot_name or ""
    ).strip()
    panel._last_infer_image_path = str(result.get("image_path") or "")
    panel._last_infer_output_path = str(result.get("out_path") or "")
    infer_ckpt_path = str(result.get("ckpt_path") or "")
    infer_selection_policy = str(result.get("selection_policy") or "")
    infer_postprocess_policy = str(result.get("postprocess_policy") or "")
    infer_model_info = result.get("model_info") if isinstance(result.get("model_info"), dict) else {}
    infer_experiment = str(panel._exp_info.get("experiment", "--"))
    infer_seed: object = panel._exp_info.get("seed", "--")
    infer_meta = getattr(panel, "_tfm_ckpt_meta", {}).get(infer_ckpt_path, {}) if infer_ckpt_path else {}
    if isinstance(infer_meta, dict):
        exp_meta = str(infer_meta.get("experiment") or "").strip()
        if exp_meta:
            infer_experiment = exp_meta
        seed_meta = infer_meta.get("seed")
        if seed_meta is not None:
            infer_seed = seed_meta
    if infer_ckpt_path:
        infer_path = Path(infer_ckpt_path).expanduser()
        if infer_path.parent.name == "checkpoints":
            seed_dir = infer_path.parent.parent
            exp_dir = seed_dir.parent
        else:
            seed_dir = infer_path.parent
            exp_dir = seed_dir.parent if seed_dir.name.startswith("seed_") else None
        if exp_dir and exp_dir.name:
            infer_experiment = exp_dir.name
        seed_match = re.match(r"seed_(\d+)", seed_dir.name) if infer_ckpt_path else None
        if seed_match:
            infer_seed = seed_match.group(1)
    try:
        panel._last_infer_frame_ts = float(result.get("frame_ts", 0.0) or 0.0)
    except Exception:
        panel._last_infer_frame_ts = 0.0
    panel._perf_infer_ms = float(result.get("infer_ms", 0.0))
    panel._perf_total_ms = float(result.get("total_ms", 0.0))
    panel._push_history(panel._perf_infer_hist, panel._perf_infer_ms, max_len=20)
    panel._push_history(panel._perf_total_hist, panel._perf_total_ms, max_len=20)
    frame_w = int(result.get("frame_w", 0))
    frame_h = int(result.get("frame_h", 0))
    roi = result.get("roi") if isinstance(result.get("roi"), (tuple, list)) else None
    postprocess_enabled = panel._tfm_postprocess_enabled()
    ref_for_size = None
    angle_adjusted = False
    center_adjusted = False
    size_adjusted = False
    if postprocess_enabled and frame_w > 0 and frame_h > 0 and roi is not None:
        ref_for_size = panel._build_reference_grasp(frame_w, frame_h)
        object_shape = str(OBJECT_SHAPES.get(panel._selected_object or "", "") or "")
        pred, angle_adjusted = reconcile_inferred_grasp_angle(
            pred,
            ref_for_size,
            roi=tuple(roi),
            object_shape=object_shape,
        )
        pred, center_adjusted = reconcile_inferred_grasp_center(pred, ref_for_size, roi=tuple(roi))
        pred, size_adjusted = reconcile_inferred_grasp_size(pred, ref_for_size, roi=tuple(roi))
    if angle_adjusted and ref_for_size:
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_angle_adjust "
            f"selected={panel._selected_object or 'none'} roi={tuple(roi)} shape={OBJECT_SHAPES.get(panel._selected_object or '', 'unknown')} "
            f"pred_angle_raw={float(raw_pred.get('angle_deg', 0.0) or 0.0):.2f} "
            f"pred_angle_adj={float(pred.get('angle_deg', 0.0) or 0.0):.2f} "
            f"ref_angle={float(ref_for_size.get('angle_deg', 0.0) or 0.0):.2f}",
        )
    if center_adjusted and ref_for_size:
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_center_adjust "
            f"selected={panel._selected_object or 'none'} roi={tuple(roi)} "
            f"pred_center_raw=({float(raw_pred.get('cx', 0.0) or 0.0):.2f},{float(raw_pred.get('cy', 0.0) or 0.0):.2f}) "
            f"pred_center_adj=({float(pred.get('cx', 0.0) or 0.0):.2f},{float(pred.get('cy', 0.0) or 0.0):.2f}) "
            f"ref_center=({float(ref_for_size.get('cx', 0.0) or 0.0):.2f},{float(ref_for_size.get('cy', 0.0) or 0.0):.2f})",
        )
    panel._last_grasp_px = {
        "cx": float(pred.get("cx", 0.0)),
        "cy": float(pred.get("cy", 0.0)),
        "w": float(pred.get("w", 0.0)),
        "h": float(pred.get("h", 0.0)),
        "angle_deg": float(pred.get("angle_deg", 0.0)),
    }
    if size_adjusted and ref_for_size:
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_size_adjust "
            f"selected={panel._selected_object or 'none'} roi={tuple(roi)} "
            f"pred_size_raw=({float(raw_pred.get('w', 0.0) or 0.0):.2f},{float(raw_pred.get('h', 0.0) or 0.0):.2f}) "
            f"ref_size=({float(ref_for_size.get('w', 0.0) or 0.0):.2f},{float(ref_for_size.get('h', 0.0) or 0.0):.2f})",
        )
    adjustments: List[str] = []
    if angle_adjusted:
        adjustments.append("angle")
    if center_adjusted:
        adjustments.append("center")
    if size_adjusted:
        adjustments.append("size")
    if adjustments:
        panel._last_tfm_postprocess_note = f"ajustes panel: {', '.join(adjustments)}"
        panel._emit_log(
            "[TFM] postprocess "
            f"adjustments={','.join(adjustments)} "
            f"selected={panel._selected_object or 'none'}"
        )
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_postprocess "
            f"adjustments={','.join(adjustments)} "
            f"selected={panel._selected_object or 'none'}",
        )
    else:
        if postprocess_enabled:
            panel._last_tfm_postprocess_note = "sin ajustes panel"
        else:
            panel._last_tfm_postprocess_note = "postproceso desactivado (predicción raw)"
            panel._emit_log(
                "[TFM] postprocess disabled "
                f"selected={panel._selected_object or 'none'} mode=raw"
            )
            panel._audit_append(
                "logs/infer.log",
                "[TFM] infer_postprocess "
                f"adjustments=none selected={panel._selected_object or 'none'} mode=raw_disabled",
            )
    panel._last_grasp_source = "infer_model"
    panel._last_grasp_frame = panel.camera_topic or "image"
    panel._last_grasp_update_ts = _runtime_time()
    panel._last_grasp_world = panel._compute_world_grasp(frame_w, frame_h)
    panel._last_grasp_base = panel._world_grasp_to_base(panel._last_grasp_world)
    panel._refresh_cornell_metrics(frame_w, frame_h)
    sync_tfm_module_grasp_state(panel)
    grasp_rect_publish_ok = panel._publish_current_grasp_rect()
    overlay_refresh_ok = panel._refresh_grasp_overlay_now()
    overlay_name = (
        f"overlay_infer_{int(panel._last_infer_frame_ts * 1000)}.png"
        if panel._last_infer_frame_ts > 0.0
        else f"overlay_infer_{panel._infer_session_id}.png"
    )
    panel._last_infer_overlay_path = panel._save_grasp_overlay(overlay_name) if overlay_refresh_ok else ""
    panel._audit_append(
        "logs/visualize.log",
        "[TFM] overlay_sync "
        f"session={panel._infer_session_id} frame_ts={panel._last_infer_frame_ts:.6f} "
        f"refresh_ok={str(bool(overlay_refresh_ok)).lower()} "
        f"overlay={panel._last_infer_overlay_path or 'none'}",
    )
    panel._refresh_science_ui()
    if grasp_rect_publish_ok:
        panel._set_status("TFM: grasp inferido y publicado", error=False)
        infer_message = "grasp inferido y publicado"
    else:
        panel._set_status("TFM: grasp inferido", error=False)
        infer_message = "grasp inferido"
    alignment_2d = None
    if panel._last_grasp_px and panel._last_cornell_ref:
        pred_cx = float(panel._last_grasp_px.get("cx", 0.0) or 0.0)
        pred_cy = float(panel._last_grasp_px.get("cy", 0.0) or 0.0)
        ref_cx = float(panel._last_cornell_ref.get("cx", 0.0) or 0.0)
        ref_cy = float(panel._last_cornell_ref.get("cy", 0.0) or 0.0)
        delta_x_px = pred_cx - ref_cx
        delta_y_px = pred_cy - ref_cy
        dist_px = math.hypot(delta_x_px, delta_y_px)
        alignment_2d = {
            "selected": str(panel._selected_object or ""),
            "pred_cx": pred_cx,
            "pred_cy": pred_cy,
            "ref_cx": ref_cx,
            "ref_cy": ref_cy,
            "delta_x_px": delta_x_px,
            "delta_y_px": delta_y_px,
            "dist_px": dist_px,
            "pred_w": float(panel._last_grasp_px.get("w", 0.0) or 0.0),
            "pred_h": float(panel._last_grasp_px.get("h", 0.0) or 0.0),
            "ref_w": float(panel._last_cornell_ref.get("w", 0.0) or 0.0),
            "ref_h": float(panel._last_cornell_ref.get("h", 0.0) or 0.0),
        }
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_align_2d "
            f"selected={panel._selected_object or 'none'} "
            f"pred=({pred_cx:.2f},{pred_cy:.2f}) "
            f"ref=({ref_cx:.2f},{ref_cy:.2f}) "
            f"delta=({delta_x_px:.2f},{delta_y_px:.2f}) dist_px={dist_px:.2f} "
            f"size_pred=({float(panel._last_grasp_px.get('w', 0.0) or 0.0):.2f},{float(panel._last_grasp_px.get('h', 0.0) or 0.0):.2f}) "
            f"size_ref=({float(panel._last_cornell_ref.get('w', 0.0) or 0.0):.2f},{float(panel._last_cornell_ref.get('h', 0.0) or 0.0):.2f})",
        )
    audit_payload = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "session": panel._infer_session_id,
        "status": "OK",
        "source": panel._last_grasp_source,
        "experiment": {
            "selection_policy": infer_selection_policy,
            "postprocess_policy": infer_postprocess_policy,
            "checkpoint_path": infer_ckpt_path,
            "experiment": infer_experiment,
            "seed": infer_seed,
            "model": panel._exp_info.get("model", "--"),
            "modality": panel._exp_info.get("modality", "--"),
            "model_info": infer_model_info,
        },
        "visual_grasp": {
            "topic": panel._grasp_rect_topic,
            "msg_type": "std_msgs/msg/Float32MultiArray",
            "publish_ok": bool(grasp_rect_publish_ok),
        },
        "executable_grasp": {
            "pose_topic": MOVEIT_POSE_TOPIC,
            "cartesian_topic": MOVEIT_CARTESIAN_POSE_TOPIC,
            "result_topic": "/desired_grasp/result",
        },
        "grasp": panel._last_grasp_px,
        "grasp_base": panel._last_grasp_base,
        "grasp_rect_publish_ok": bool(grasp_rect_publish_ok),
        "grasp_rect_topic": panel._grasp_rect_topic,
        "frame": panel._last_grasp_frame,
        "cornell": panel._last_cornell,
        "alignment_2d": alignment_2d,
        "cornell_reason": panel._last_cornell_reason,
        "perf": {
            "infer_ms": panel._perf_infer_ms,
            "total_ms": panel._perf_total_ms,
        },
        "frame_info": {
            "w": int(result.get("frame_w", 0) or 0),
            "h": int(result.get("frame_h", 0) or 0),
            "ts": panel._last_infer_frame_ts or result.get("frame_ts"),
        },
        "artifacts": {
            "image_path": panel._last_infer_image_path or None,
            "grasp_path": panel._last_infer_output_path or None,
            "overlay_path": panel._last_infer_overlay_path or None,
        },
    }
    panel._audit_write_json("artifacts/grasp_last.json", audit_payload)
    panel._audit_append(
        "logs/infer.log",
        f"[TFM] infer_end session={panel._infer_session_id} status=OK "
        f"infer_ms={panel._perf_infer_ms:.2f} total_ms={panel._perf_total_ms:.2f} "
        f"visual_topic={panel._grasp_rect_topic} executable_topic={MOVEIT_POSE_TOPIC} "
        f"frame_ts={panel._last_infer_frame_ts:.6f} "
        f"grasp_rect_publish_ok={str(bool(grasp_rect_publish_ok)).lower()} "
        f"overlay_refresh_ok={str(bool(overlay_refresh_ok)).lower()} "
        f"overlay={panel._last_infer_overlay_path or 'none'} "
        f"grasp={panel._last_grasp_px} cornell={panel._last_cornell} "
        f"cornell_reason={panel._last_cornell_reason!r}",
    )
    complete_pending_tfm_infer_request(panel, True, infer_message)
    if hasattr(panel, "btn_tfm_grasp_object"):
        panel.btn_tfm_grasp_object.setEnabled(True)

def sync_tfm_module_grasp_state(panel) -> None:
    if not panel.tfm_module or not panel._last_grasp_px:
        return
    try:
        from tfm_grasping.geometry import Grasp2D

        grasp = Grasp2D(
            center_x=panel._last_grasp_px["cx"],
            center_y=panel._last_grasp_px["cy"],
            angle_rad=math.radians(panel._last_grasp_px["angle_deg"]),
            width_px=panel._last_grasp_px["w"],
            quality=0.0,
            depth_m=None,
            frame_id=panel._last_grasp_frame,
        )
        if hasattr(panel.tfm_module, "set_last_grasp"):
            panel.tfm_module.set_last_grasp(grasp)
        else:
            panel.tfm_module._last_grasp = grasp  # type: ignore[attr-defined]
    except Exception as exc:
        panel._log_warning(f"[TFM] set_last_grasp error: {exc}")

def tfm_visualize_grasp(panel):
    panel._log_button("TFM Comparar grasp/ref")
    panel._emit_log(
        "[TFM][BUTTON] action=visualize "
        f"applied={str(bool(getattr(panel, '_tfm_experiment_applied', False))).lower()} "
        f"has_grasp={str(bool(getattr(panel, '_last_grasp_px', None))).lower()} "
        f"selected={str(getattr(panel, '_selected_object', '') or 'none')}"
    )
    experiment_ready, experiment_reason = panel._tfm_experiment_ready_status()
    if not experiment_ready:
        panel._set_status(f"TFM bloqueado: {experiment_reason}", error=True)
        panel._audit_append("logs/visualize.log", f"[TFM] visualize FAIL reason={experiment_reason}")
        return False
    if not panel.tfm_module and not panel._last_grasp_px:
        panel._set_status("TFM no disponible", error=True)
        return False
    rep = None
    if panel.tfm_module:
        try:
            rep = panel.tfm_module.get_grasp_representation()
        except Exception as exc:
            panel._log_warning(f"[TFM] get_grasp_representation error: {exc}")
    if not rep and panel._last_grasp_px:
        rep = dict(panel._last_grasp_px)
    if not rep:
        panel._set_status("TFM: sin grasp para visualizar", error=True)
        return False
    ref = None
    if panel._last_camera_frame:
        _qimg, w, h, _ts = panel._last_camera_frame
        ref = panel._build_reference_grasp(w, h)
    if ref:
        panel._tfm_visual_compare_enabled = not bool(panel._tfm_visual_compare_enabled)
        mode_txt = "compare_on" if panel._tfm_visual_compare_enabled else "compare_off"
    else:
        panel._tfm_visual_compare_enabled = False
        mode_txt = "pred_only"
    panel._log(f"[TFM] Grasp actual: {rep} ref={ref} mode={mode_txt}")
    panel._refresh_grasp_overlay_now()
    overlay_path = panel._save_grasp_overlay()
    if overlay_path:
        panel._audit_append(
            "logs/visualize.log",
            f"[TFM] visualize OK mode={mode_txt} overlay={overlay_path}",
        )
    else:
        panel._audit_append("logs/visualize.log", f"[TFM] visualize FAIL mode={mode_txt} overlay=none")
    if panel._tfm_visual_compare_enabled:
        panel._set_status("TFM: comparación grasp/ref activada", error=False)
    elif ref:
        panel._set_status("TFM: comparación grasp/ref desactivada", error=False)
    else:
        panel._set_status("TFM: grasp visualizado (sin referencia)", error=False)
    return True

def wait_tfm_moveit_result(panel,
    label: str,
    *,
    since_wall: float,
    since_seq: int,
    timeout_sec: float,
    expected_request_id: Optional[int] = None,
    expected_request_uuid: str = "",
) -> Tuple[bool, str]:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        panel._motion_in_progress = False
        return False, "ros_worker_not_ready"
    started = time.time()
    deadline = started + max(0.2, float(timeout_sec))
    cursor_wall = float(since_wall)
    cursor_seq = int(since_seq)
    expected_uuid = str(expected_request_uuid or "").strip()
    result_topic = "/desired_grasp/result"
    grace_applied = False
    try:
        active_request_grace_sec = float(
            os.environ.get(
                "PANEL_TFM_MOVEIT_ACTIVE_REQUEST_GRACE_SEC",
                "90.0",
            )
        )
    except Exception:
        active_request_grace_sec = 90.0
    active_request_grace_sec = max(10.0, active_request_grace_sec)
    try:
        hb_recent_window_sec = float(
            os.environ.get(
                "PANEL_TFM_MOVEIT_ACTIVE_REQUEST_HB_SEC",
                "2.5",
            )
        )
    except Exception:
        hb_recent_window_sec = 2.5
    hb_recent_window_sec = max(1.2, hb_recent_window_sec)
    while time.time() < deadline:
        chunk = min(0.8, max(0.1, deadline - time.time()))
        ok, raw, wall, seq = panel.ros_worker.wait_for_moveit_result(
            since_wall=cursor_wall,
            since_seq=cursor_seq,
            timeout_sec=chunk,
        )
        if not ok:
            now = time.time()
            if now >= deadline:
                result_pubs = int(panel.ros_worker.topic_publisher_count(result_topic))
                result_subs = int(panel.ros_worker.topic_subscriber_count(result_topic))
                bridge_alive = bool(panel._proc_alive(getattr(panel, "moveit_bridge_proc", None)))
                hb_age = panel.ros_worker.moveit_bridge_heartbeat_age()
                hb_recent = panel.ros_worker.has_recent_moveit_bridge_heartbeat(hb_recent_window_sec)
                hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
                if (
                    not grace_applied
                    and result_pubs > 0
                    and result_subs > 0
                    and bridge_alive
                    and (
                        bool(hb_recent)
                        or (not math.isinf(hb_age) and hb_age <= max(5.0, hb_recent_window_sec * 2.0))
                    )
                ):
                    old_timeout = max(0.0, deadline - started)
                    deadline = now + active_request_grace_sec
                    grace_applied = True
                    panel._emit_log(
                        f"[TFM][MOVEIT][WAIT] {label} extend_wait old_timeout={old_timeout:.1f}s "
                        f"new_timeout={max(0.0, deadline - started):.1f}s grace={active_request_grace_sec:.1f}s "
                        f"bridge_alive={str(bridge_alive).lower()} "
                        f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
                    )
                    continue
            continue
        cursor_wall = max(cursor_wall, float(wall))
        cursor_seq = max(cursor_seq, int(seq))
        text = str(raw or "").strip()
        if not text:
            continue
        try:
            data = json.loads(text)
        except Exception:
            low = text.lower()
            success = ("success" in low) or ("ok" in low)
            # FASE 3: Liberar motion_in_progress.
            panel._motion_in_progress = False
            return success, text
        if expected_request_id is not None:
            got_id = int(data.get("request_id", -1) or -1)
            if got_id != int(expected_request_id):
                continue
        if expected_uuid:
            got_uid = str(data.get("request_uuid", "") or "")
            if got_uid != expected_uuid:
                continue
        success = bool(data.get("success", False))
        if not success and ("plan_ok" in data or "exec_ok" in data):
            success = bool(data.get("plan_ok", False) and data.get("exec_ok", False))
        if not success and "exec_ok" in data:
            success = bool(data.get("exec_ok", False))
        msg = str(data.get("message") or data.get("status") or "")
        if not msg:
            msg = text
        # FASE 3: Liberar motion_in_progress al obtener resultado.
        panel._motion_in_progress = False
        return success, msg
    # FASE 3: Liberar motion_in_progress al salir por timeout.
    panel._motion_in_progress = False
    return False, f"timeout_active_request>{max(0.0, deadline - started):.1f}s"

def execute_tfm_world_grasp(panel) -> bool:
    if not panel._last_grasp_px:
        return False
    if not panel._last_grasp_world and panel._last_camera_frame:
        _qimg, frame_w, frame_h, _ts = panel._last_camera_frame
        if frame_w > 0 and frame_h > 0:
            panel._last_grasp_world = panel._compute_world_grasp(frame_w, frame_h)
    if panel._last_grasp_base is None and panel._last_grasp_world is not None:
        panel._last_grasp_base = panel._world_grasp_to_base(panel._last_grasp_world)
    if not panel._last_grasp_base:
        panel._set_status("TFM: grasp base_link no disponible (cámara/calibración)", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM] execute FAIL reason=base_grasp_missing source={panel._last_grasp_source or 'unknown'}",
        )
        return False
    if panel._tfm_execute_inflight:
        panel._set_status("TFM: ejecución en curso", error=False)
        return False
    if not panel._moveit_required:
        panel._set_status("TFM: MoveIt no habilitado", error=True)
        panel._audit_append("logs/execute.log", "[TFM] execute FAIL reason=moveit_disabled")
        return False

    grasp_base = dict(panel._last_grasp_base)
    source = panel._last_grasp_source or "unknown"
    if tfm_canonical_use_pick_object(panel):
        selected_name = str(
            getattr(panel, "_selected_object", "") or getattr(panel, "_last_grasp_selection_name", "") or ""
        ).strip()
        grasp_selection = str(getattr(panel, "_last_grasp_selection_name", "") or "").strip()
        if not selected_name:
            panel._set_status("TFM: selección de objeto no disponible", error=True)
            panel._audit_append(
                "logs/execute.log",
                "[TFM] execute FAIL reason=selected_object_missing_for_canonical_route",
            )
            return False
        if grasp_selection and selected_name != grasp_selection:
            panel._set_status(
                f"TFM: grasp no corresponde a la selección actual ({grasp_selection} -> {selected_name})",
                error=True,
            )
            panel._audit_append(
                "logs/execute.log",
                "[TFM] execute FAIL reason=selection_grasp_mismatch "
                f"selected={selected_name} grasp_selection={grasp_selection}",
            )
            return False
        panel._tfm_execute_inflight = True
        panel._pick_object_grasp_override = build_tfm_pick_object_override(panel, 
            grasp_base=grasp_base,
            selected_object=selected_name,
            source=source,
        )
        tfm_canonical_state_reset(panel, 
            selected_object=selected_name,
            grasp_base=grasp_base,
            source=source,
        )
        tfm_canonical_phase_update(panel, "READY", detail="preconditions_ok")
        tfm_canonical_phase_update(panel, "OBJECT_SELECTED", detail=f"name={selected_name}")
        tfm_canonical_phase_update(panel, 
            "GRASP_FRESH",
            detail=f"source={source} age_sec={max(0.0, _runtime_time() - float(panel._last_grasp_update_ts or 0.0)):.2f}",
        )
        tfm_canonical_phase_update(panel, 
            "VISUAL_GRASP_OK",
            detail=f"topic={panel._grasp_rect_topic or '/grasp_rect'}",
        )
        tfm_canonical_phase_update(panel, 
            "EXECUTABLE_GRASP_OK",
            detail=f"pose_topic={MOVEIT_POSE_TOPIC} cartesian_topic={MOVEIT_CARTESIAN_POSE_TOPIC}",
        )
        setattr(panel, "_pick_object_worker_started", False)
        run_pick_object(panel)
        if not bool(getattr(panel, "_pick_object_worker_started", False)):
            tfm_canonical_finish(panel, 
                False,
                "tfm_canonical_pick_object_not_started",
                final_state="FAIL_TERMINAL",
            )
            return False
        return True

    panel._tfm_execute_inflight = True
    _minor_yaw_override = getattr(panel, "_tfm_grasp_minor_yaw_deg", None)
    _preopen_rad_override = getattr(panel, "_tfm_grasp_preopen_rad", None)
    _grasp_orientation_tag = "minor_axis" if _minor_yaw_override is not None else "direct_angle"
    panel._tfm_grasp_minor_yaw_deg = None
    panel._tfm_grasp_preopen_rad = None

    def _env_float(name: str, default: float) -> float:
        try:
            return float(os.environ.get(name, str(default)) or default)
        except Exception:
            return default

    def _next_tfm_request_id() -> int:
        seq = int(getattr(panel, "_tfm_moveit_request_id", 0) or 0) + 1
        setattr(panel, "_tfm_moveit_request_id", seq)
        return seq

    def _encode_request_frame(frame: str, request_id: int, request_uuid: str) -> str:
        base = str(frame or BASE_FRAME or "base_link").strip() or "base_link"
        uid = str(request_uuid or "").strip()
        if uid:
            return f"{base}|rid={int(request_id)}|uid={uid}"
        return f"{base}|rid={int(request_id)}"

    def _ros_clock_now_ns() -> int:
        try:
            if panel._ros_worker_started and panel.ros_worker.node_ready():
                with panel.ros_worker._lock:
                    node = getattr(panel.ros_worker, "_node", None)
                if node is not None:
                    now_ns = int(node.get_clock().now().nanoseconds)
                    if now_ns > 0:
                        return now_ns
        except Exception:
            pass
        return 0

    def _ensure_moveit_bridge_path(timeout_sec: float = 6.0) -> bool:
        if not panel._ros_worker_started:
            panel._ensure_ros_worker_started()
        if not panel.ros_worker.node_ready():
            raise RuntimeError("ros_worker_not_ready_for_moveit_bridge")

        pose_topic = MOVEIT_POSE_TOPIC
        result_topic = "/desired_grasp/result"

        def _path_ready() -> bool:
            pose_subs = int(panel.ros_worker.topic_subscriber_count(pose_topic))
            result_pubs = int(panel.ros_worker.topic_publisher_count(result_topic))
            return pose_subs > 0 and result_pubs > 0

        if _path_ready():
            return False

        panel._emit_log(
            "[TFM][MOVEIT] bridge_path_missing "
            f"pose_topic={pose_topic} result_topic={result_topic}; attempting_recover=true"
        )

        done = threading.Event()

        def _launch_bridge() -> None:
            try:
                panel._start_moveit_bridge()
            finally:
                done.set()

        panel.signal_run_ui.emit(_launch_bridge)
        done.wait(timeout=3.0)

        deadline = time.time() + max(1.0, float(timeout_sec))
        while time.time() < deadline:
            pose_subs = int(panel.ros_worker.topic_subscriber_count(pose_topic))
            result_pubs = int(panel.ros_worker.topic_publisher_count(result_topic))
            panel._emit_log(
                "[TFM][MOVEIT] bridge_recover_check "
                f"pose_subs={pose_subs} result_pubs={result_pubs}"
            )
            if pose_subs > 0 and result_pubs > 0:
                return True
            time.sleep(0.4)

        raise RuntimeError(
            "moveit_bridge_not_ready_after_recover "
            f"pose_topic={pose_topic} result_topic={result_topic}"
        )

    def worker() -> None:
        try:
            x = float(grasp_base.get("x", 0.0))
            y = float(grasp_base.get("y", 0.0))
            z_raw = float(grasp_base.get("z", 0.0))
            yaw_deg = (
                float(_minor_yaw_override) if _minor_yaw_override is not None
                else float(grasp_base.get("yaw_deg", 0.0) or 0.0)
            )
            proj_z_source = str(grasp_base.get("proj_z_source", "unknown") or "unknown")
            grasp_semantics = str(
                grasp_base.get("grasp_semantics", "projection_surface") or "projection_surface"
            )
            pretable_enabled = str(
                os.environ.get("PANEL_TFM_EXECUTE_PRETABLE", "1")
            ).strip().lower() not in ("0", "false", "no", "off")
            if pretable_enabled:
                move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
                panel._emit_log(
                    "[TFM][PRETABLE] yendo a MESA antes de ejecutar grasp"
                )
                ok_table, info_table = panel._publish_joint_trajectory(JOINT_TABLE_POSE_RAD, move_sec)
                if not ok_table:
                    raise RuntimeError(f"pretable_publish_failed:{info_table}")
                table_reached = panel._wait_for_joint_target(
                    JOINT_TABLE_POSE_RAD,
                    timeout_sec=max(6.0, move_sec + 4.0),
                    tol_rad=0.08,
                )
                if not table_reached:
                    raise RuntimeError("pretable_target_not_reached")
                panel._emit_log(
                    "[TFM][PRETABLE] MESA alcanzada; continuando con ejecución MoveIt"
                )
            table_top_world = float(panel._resolve_table_top_z())
            table_top_base = z_raw
            table_base = panel._ensure_base_coords(
                (float(TABLE_CENTER_X), float(TABLE_CENTER_Y), table_top_world),
                panel._world_frame_config_first(),
                timeout_sec=0.35,
            )
            if table_base is not None:
                table_top_base = float(table_base[2])
            min_margin = max(0.0, _env_float("PANEL_TFM_MIN_TABLE_MARGIN_M", 0.01))
            z_approach = max(0.12, min(0.20, _env_float("PANEL_PICK_Z_APPROACH_M", 0.14)))
            z_grasp_offset = _env_float("PANEL_PICK_Z_GRASP_OFFSET_M", 0.02)
            grasp_contact_z_offset = float(GRIPPER_TCP_Z_OFFSET)
            speed_scale = max(0.01, min(1.0, _env_float("PANEL_PICK_SPEED_SCALE", 0.25)))
            accel_scale = max(0.01, min(1.0, _env_float("PANEL_PICK_ACCEL_SCALE", 0.25)))
            bridge_request_timeout = max(
                2.0,
                _env_float("PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC", 35.0),
            )
            result_timeout = max(
                    10.0,
                    _env_float(
                        "PANEL_TFM_MOVEIT_RESULT_TIMEOUT_SEC",
                        bridge_request_timeout,
                    ),
                    bridge_request_timeout,
            )
            z_grasp = max(table_top_base + min_margin, z_raw + z_grasp_offset + grasp_contact_z_offset)
            z_pre = max(z_grasp + z_approach, table_top_base + min_margin + 0.02)
            z_retreat = z_pre
            panel._emit_log(
                "[TFM][GRASP_SEMANTICS] "
                f"source={source} visual_topic={panel._grasp_rect_topic or '/grasp_rect'} "
                f"semantics={grasp_semantics} proj_z_source={proj_z_source} "
                f"exec_seed_z={z_raw:.3f} table_top_base={table_top_base:.3f} exec_z={z_grasp:.3f}"
            )
            yaw_rad = math.radians(yaw_deg)
            orientation = (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))
            frame = panel._business_base_frame()
            tfm_grasp_cartesian = str(
                os.environ.get("PANEL_TFM_GRASP_CARTESIAN", "0")
            ).strip().lower() not in ("0", "false", "no", "off")
            grasp_mode = "cartesian" if tfm_grasp_cartesian else "pose"
            pre_pose = _make_pose_data((x, y, z_pre), orientation=orientation, frame=frame)
            grasp_pose = _make_pose_data((x, y, z_grasp), orientation=orientation, frame=frame)
            retreat_pose = _make_pose_data((x, y, z_retreat), orientation=orientation, frame=frame)

            for pd in (pre_pose, grasp_pose, retreat_pose):
                pd["speed_scale"] = float(speed_scale)
                pd["accel_scale"] = float(accel_scale)

            has_results = False
            if panel._ros_worker_started and panel.ros_worker.node_ready():
                try:
                    has_results = bool(panel.ros_worker.subscribe_moveit_result("/desired_grasp/result"))
                except Exception:
                    has_results = False
            if has_results:
                _ensure_moveit_bridge_path()

            panel._audit_append(
                "logs/execute.log",
                f"[TFM] execute START source={source} base=({x:.3f},{y:.3f},{z_grasp:.3f}) "
                f"pre_z={z_pre:.3f} yaw={yaw_deg:.1f} frame={frame} wait_result={str(has_results).lower()} "
                f"semantics={grasp_semantics} proj_z_source={proj_z_source} z_seed={z_raw:.3f} "
                f"table_top_base={table_top_base:.3f} "
                f"z_approach={z_approach:.3f} z_grasp_offset={z_grasp_offset:+.3f} "
                f"speed_scale={speed_scale:.2f} accel_scale={accel_scale:.2f} "
                f"result_timeout={result_timeout:.1f} grasp_mode={grasp_mode}",
            )

            if _preopen_rad_override is not None:
                _pr = float(_preopen_rad_override)
                panel.signal_run_ui.emit(
                    lambda _r=_pr: panel._command_gripper_preopen(_r, log_action="PICK")
                )
            else:
                panel.signal_run_ui.emit(lambda: panel._command_gripper(False, log_action="PICK", force=True))
            panel._emit_log(
                "[PICK][MOVEIT] sequence=HOME->PREGRASP->DESCENT->GRASP->RETREAT "
                f"frame={frame} z_approach={z_approach:.3f} z_grasp_offset={z_grasp_offset:+.3f} "
                f"speed_scale={speed_scale:.2f} accel_scale={accel_scale:.2f} "
                f"grasp_mode={grasp_mode}"
            )
            time.sleep(0.35)

            since_wall = 0.0
            since_seq = -1
            if has_results:
                _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
            pre_request_id = _next_tfm_request_id()
            pre_request_uuid = uuid.uuid4().hex
            pre_pose_send = dict(pre_pose)
            pre_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
            pre_pose_send["frame"] = _encode_request_frame(frame, pre_request_id, pre_request_uuid)
            pose_subs_now = int(panel.ros_worker.topic_subscriber_count(MOVEIT_POSE_TOPIC)) if has_results else 1
            if pose_subs_now <= 0:
                raise RuntimeError(f"no_pose_subscribers_before_pregrasp topic={MOVEIT_POSE_TOPIC}")
            if not panel._publish_moveit_pose("TFM_PRE_GRASP", pre_pose_send, cartesian=False):
                raise RuntimeError("publish_pregrasp_failed")
            if has_results:
                ok_pre, msg_pre = wait_tfm_moveit_result(panel, 
                    "TFM_PRE_GRASP",
                    since_wall=since_wall,
                    since_seq=since_seq,
                    timeout_sec=result_timeout,
                    expected_request_id=pre_request_id,
                    expected_request_uuid=pre_request_uuid,
            )
                if not ok_pre:
                    raise RuntimeError(f"pregrasp_result_failed:{msg_pre}")
            else:
                time.sleep(0.8)

            since_wall = 0.0
            since_seq = -1
            if has_results:
                _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
            grasp_request_id = _next_tfm_request_id()
            grasp_request_uuid = uuid.uuid4().hex
            grasp_pose_send = dict(grasp_pose)
            grasp_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
            grasp_pose_send["frame"] = _encode_request_frame(frame, grasp_request_id, grasp_request_uuid)
            pose_subs_now = int(panel.ros_worker.topic_subscriber_count(MOVEIT_POSE_TOPIC)) if has_results else 1
            if pose_subs_now <= 0:
                raise RuntimeError(f"no_pose_subscribers_before_grasp topic={MOVEIT_POSE_TOPIC}")
            if _grasp_orientation_tag == "minor_axis":
                panel._emit_log(
                    "[TFM_GRASP][MOVEIT] publish topic=/desired_grasp/request "
                    "mode=moveit_sequence source=infer_model step=grasp_descent"
                )
            if not panel._publish_moveit_pose("TFM_GRASP", grasp_pose_send, cartesian=tfm_grasp_cartesian):
                raise RuntimeError("publish_grasp_failed")
            if has_results:
                if _grasp_orientation_tag == "minor_axis":
                    panel._emit_log(
                        "[TFM_GRASP][MOVEIT] waiting topic=/desired_grasp/result "
                        f"step=grasp_descent timeout_sec={result_timeout:.1f}"
                    )
                ok_grasp, msg_grasp = wait_tfm_moveit_result(panel, 
                    "TFM_GRASP",
                    since_wall=since_wall,
                    since_seq=since_seq,
                    timeout_sec=result_timeout,
                    expected_request_id=grasp_request_id,
                    expected_request_uuid=grasp_request_uuid,
                )
                if _grasp_orientation_tag == "minor_axis":
                    panel._emit_log(
                        f"[TFM_GRASP][MOVEIT] result={'OK' if ok_grasp else 'FAIL'} "
                        f"step=grasp_descent reason={msg_grasp or 'ok'}"
                    )
                if not ok_grasp:
                    if not tfm_grasp_cartesian:
                        raise RuntimeError(f"grasp_result_failed:{msg_grasp}")
                    msg_low = str(msg_grasp or "").lower()
                    reason = "unknown"
                    if "collision" in msg_low:
                        reason = "collision"
                    elif "ik" in msg_low:
                        reason = "ik"
                    elif "frame" in msg_low or "tf" in msg_low:
                        reason = "frame_mismatch"
                    elif "plan" in msg_low:
                        reason = "planning"
                    panel._emit_log(
                        "[PICK][MOVEIT] cartesian_descent_failed "
                        f"reason={reason} msg={msg_grasp}; fallback=planner"
                    )
                    panel._audit_append(
                        "logs/execute.log",
                        "[TFM] grasp cartesian_failed "
                        f"reason={reason} msg={msg_grasp}",
                    )
                    since_wall = 0.0
                    since_seq = -1
                    _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
                    fallback_request_id = _next_tfm_request_id()
                    fallback_request_uuid = uuid.uuid4().hex
                    fallback_pose_send = dict(grasp_pose)
                    fallback_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
                    fallback_pose_send["frame"] = _encode_request_frame(frame, fallback_request_id, fallback_request_uuid)
                    if not panel._publish_moveit_pose("TFM_GRASP_FALLBACK", fallback_pose_send, cartesian=False):
                        raise RuntimeError("publish_grasp_fallback_failed")
                    ok_fb, msg_fb = wait_tfm_moveit_result(panel, 
                        "TFM_GRASP_FALLBACK",
                        since_wall=since_wall,
                        since_seq=since_seq,
                        timeout_sec=result_timeout,
                        expected_request_id=fallback_request_id,
                        expected_request_uuid=fallback_request_uuid,
                    )
                    if not ok_fb:
                        raise RuntimeError(f"grasp_fallback_result_failed:{msg_fb}")
            else:
                time.sleep(0.8)

            panel.signal_run_ui.emit(lambda: panel._command_gripper(True, log_action="PICK", force=True))
            time.sleep(0.35)

            since_wall = 0.0
            since_seq = -1
            if has_results:
                _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
            retreat_request_id = _next_tfm_request_id()
            retreat_request_uuid = uuid.uuid4().hex
            retreat_pose_send = dict(retreat_pose)
            retreat_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
            retreat_pose_send["frame"] = _encode_request_frame(frame, retreat_request_id, retreat_request_uuid)
            if not panel._publish_moveit_pose("TFM_RETREAT", retreat_pose_send, cartesian=False):
                raise RuntimeError("publish_retreat_failed")
            if has_results:
                ok_ret, msg_ret = wait_tfm_moveit_result(panel, 
                    "TFM_RETREAT",
                    since_wall=since_wall,
                    since_seq=since_seq,
                    timeout_sec=result_timeout,
                    expected_request_id=retreat_request_id,
                    expected_request_uuid=retreat_request_uuid,
            )
                if not ok_ret:
                    raise RuntimeError(f"retreat_result_failed:{msg_ret}")
            else:
                time.sleep(0.6)

            panel._audit_append(
                "logs/execute.log",
                f"[TFM] execute OK mode=moveit_sequence source={source} "
                f"target=({x:.3f},{y:.3f},{z_grasp:.3f}) pre=({x:.3f},{y:.3f},{z_pre:.3f}) "
                f"retreat=({x:.3f},{y:.3f},{z_retreat:.3f}) yaw={yaw_deg:.1f}",
            )
            if _grasp_orientation_tag == "minor_axis":
                panel._emit_log(
                    "[TFM_GRASP][EXECUTE] status=OK mode=moveit_sequence "
                    "source=infer_model grasp_orientation=minor_axis"
                )
                panel._ui_set_status(
                    "TFM: agarre completado (MoveIt + inferencia)", error=False
                )
            else:
                panel._ui_set_status("TFM: PREGRASP + DESCENT + GRASP + RETREAT ejecutados", error=False)
            complete_pending_tfm_execute_request(panel, 
                True,
                "PREGRASP + DESCENT + GRASP + RETREAT ejecutados",
            )
        except Exception as exc:
            panel._audit_append("logs/execute.log", f"[TFM] execute FAIL mode=moveit_sequence err={exc}")
            if _grasp_orientation_tag == "minor_axis":
                panel._emit_log(
                    f"[TFM_GRASP][EXECUTE] status=FAIL mode=moveit_sequence "
                    f"source=infer_model grasp_orientation=minor_axis reason={exc}"
                )
            panel._ui_set_status(f"TFM: ejecución fallida ({exc})", error=True)
            complete_pending_tfm_execute_request(panel, False, f"ejecución fallida ({exc})")
        finally:
            panel._tfm_execute_inflight = False

    panel._run_async(worker)
    return True

def on_tfm_grasp_object_clicked(panel) -> None:
    """Botón 'Agarre objeto': ejecuta agarre MoveIt usando inferencia del rectángulo rojo.

    Flujo trazable:
      experiment_check → inference_check → rect_validate → minor_axis →
      preopen_compute → base_link_pose → moveit_execute
      mode=moveit_sequence  source=infer_model  grasp_source=red_inference_rect
      tcp=rg2_pinch_center  frame=base_link
    """
    panel._log_button("TFM Agarre objeto")
    source = str(getattr(panel, "_last_grasp_source", "") or "unknown")
    experiment_applied = bool(getattr(panel, "_tfm_experiment_applied", False))
    has_grasp = bool(getattr(panel, "_last_grasp_px", None))
    selected = str(
        getattr(panel, "_selected_object", "")
        or getattr(panel, "_last_grasp_selection_name", "")
        or "none"
    )

    panel._emit_log(
        "[TFM_GRASP][BUTTON] clicked button=agarre_objeto mode=moveit_sequence "
        "source=infer_model grasp_source=red_inference_rect "
        f"experiment_applied={str(experiment_applied).lower()} "
        f"has_grasp={str(has_grasp).lower()} selected={selected}"
    )

    # CHECK 1: experimento aplicado
    experiment_ready, experiment_reason = panel._tfm_experiment_ready_status()
    panel._emit_log(
        f"[TFM_GRASP][CHECK] experiment_applied={str(experiment_ready).lower()} "
        f"reason={experiment_reason or 'ok'}"
    )
    if not experiment_ready:
        if "aplica" in experiment_reason.lower() or "experiment" in experiment_reason.lower():
            msg_ui = "No hay experimento aplicado. Aplica primero un experimento."
        else:
            msg_ui = f"TFM bloqueado: {experiment_reason}"
        panel._set_status(f"TFM: {msg_ui}", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source=infer_model "
            f"reason={experiment_reason}",
        )
        return

    # CHECK 2: inferencia válida y no expirada
    grasp_ts = float(getattr(panel, "_last_grasp_update_ts", 0.0) or 0.0)
    age_sec = max(0.0, _runtime_time() - grasp_ts) if grasp_ts > 0.0 else -1.0
    grasp_ok, grasp_reason = panel._current_grasp_status()
    panel._emit_log(
        f"[TFM_GRASP][CHECK] inference_valid={str(grasp_ok).lower()} "
        f"source={source} age_sec={age_sec:.1f} reason={grasp_reason or 'ok'}"
    )
    if not grasp_ok:
        if grasp_reason == "sin grasp":
            msg_ui = "No hay inferencia válida. Ejecuta primero la inferencia."
        elif "expirado" in grasp_reason or "stale" in grasp_reason.lower():
            msg_ui = "La inferencia no es reciente. Ejecuta de nuevo la inferencia."
        else:
            msg_ui = f"No hay inferencia válida ({grasp_reason})."
        panel._set_status(f"TFM: {msg_ui}", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} reason={grasp_reason}",
        )
        return

    # CHECK 3: conversión a base_link disponible
    grasp_base = getattr(panel, "_last_grasp_base", None)
    if not grasp_base:
        panel._emit_log("[TFM_GRASP][CHECK] base_link=unavailable")
        panel._set_status(
            "TFM: No se pudo convertir la hipótesis de agarre a objetivo MoveIt.", error=True
        )
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} "
            "reason=base_link_unavailable",
        )
        return

    # RECT: leer y validar rectángulo de inferencia (Cx, Cy, w, h, θ)
    grasp_px = dict(panel._last_grasp_px or {})
    cx_px = float(grasp_px.get("cx", 0.0))
    cy_px = float(grasp_px.get("cy", 0.0))
    w_px = float(grasp_px.get("w", 0.0))
    h_px = float(grasp_px.get("h", 0.0))
    angle_deg = float(grasp_px.get("angle_deg", 0.0))
    theta_img = math.radians(angle_deg)

    panel._emit_log(
        f"[TFM_GRASP][RECT] cx={cx_px:.1f} cy={cy_px:.1f} "
        f"w={w_px:.1f} h={h_px:.1f} theta={angle_deg:.2f}deg"
    )

    if not (
        math.isfinite(cx_px) and math.isfinite(cy_px)
        and w_px > 0 and h_px > 0 and math.isfinite(theta_img)
    ):
        panel._set_status("TFM: rectángulo de inferencia inválido.", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} reason=rect_invalid",
        )
        return

    # AXIS: calcular eje fino del rectángulo rojo
    minor_px, opening_axis_theta_img = _compute_minor_axis_from_grasp_rect(
        w_px, h_px, theta_img
    )
    panel._emit_log(
        f"[TFM_GRASP][AXIS] minor_px={minor_px:.1f} "
        f"opening_axis_theta_img={math.degrees(opening_axis_theta_img):.2f}deg"
    )

    if minor_px <= 0:
        panel._set_status("TFM: eje fino del rectángulo inválido.", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} "
            "reason=minor_axis_invalid",
        )
        return

    # Obtener dimensiones del frame para conversión pixel → metro
    fw = fh = 0
    frame_snap = getattr(panel, "_last_camera_frame", None)
    if frame_snap:
        try:
            _qimg, fw, fh, _fts = frame_snap
        except Exception:
            fw = fh = 0
    proj_z = float((panel._last_grasp_world or {}).get("proj_z_target", 0.0))

    # Convertir minor_px a metros usando pixel_to_table_xy
    minor_width_m = 0.04  # fallback 40mm
    width_conversion_ok = False
    if fw > 0 and fh > 0:
        try:
            wx_c, wy_c = pixel_to_table_xy(
                int(round(cx_px)), int(round(cy_px)), fw, fh, z_target=proj_z
            )
            wx_e, wy_e = pixel_to_table_xy(
                int(round(cx_px + minor_px)), int(round(cy_px)), fw, fh, z_target=proj_z
            )
            if wx_c is not None and wx_e is not None:
                minor_width_m = math.hypot(
                    float(wx_e) - float(wx_c), float(wy_e) - float(wy_c)
                )
                width_conversion_ok = True
        except Exception as _exc:
            panel._emit_log(f"[TFM_GRASP] width_conversion_err={_exc}")

    # Calcular apertura previa RG2
    pre_open_width_m, finger_cmd_rad = _compute_rg2_preopen_from_minor_width(minor_width_m)

    # WIDTH log con fuente explícita (para trazabilidad en la memoria del TFM)
    _width_source = "calibrated" if width_conversion_ok else "fallback"
    panel._emit_log(
        f"[TFM_GRASP][WIDTH] source={_width_source} "
        f"minor_width_m={minor_width_m:.4f} "
        f"pre_open_width_m={pre_open_width_m:.4f}"
    )

    # Validar apertura RG2
    if not (0.015 <= pre_open_width_m <= 0.110):
        panel._set_status("TFM: apertura RG2 fuera de rango válido.", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} "
            f"reason=preopen_out_of_range pre_open_width_m={pre_open_width_m:.4f}",
        )
        return

    if not (0.0 <= finger_cmd_rad <= 1.18):
        panel._set_status("TFM: ángulo RG2 fuera de rango.", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} "
            f"reason=finger_rad_out_of_range finger_cmd_rad={finger_cmd_rad:.4f}",
        )
        return

    # GRIPPER log
    panel._emit_log(
        f"[TFM_GRASP][GRIPPER] pre_open_cmd_rad={finger_cmd_rad:.4f}"
    )

    # Calcular yaw_base desde el eje fino (opening_axis_theta_img → base_link)
    grasp_base_d = dict(grasp_base)
    minor_axis_yaw_deg = float(grasp_base_d.get("yaw_deg", 0.0))  # fallback
    yaw_conversion_ok = False
    _yaw_method = "fallback_grasp_base"
    _p0_px = (cx_px, cy_px)
    _p1_px = (cx_px, cy_px)
    _p0_base = (0.0, 0.0)
    _p1_base = (0.0, 0.0)
    if fw > 0 and fh > 0:
        try:
            step = 10.0
            dx = math.cos(opening_axis_theta_img) * step
            dy = math.sin(opening_axis_theta_img) * step
            wx_c2, wy_c2 = pixel_to_table_xy(
                int(round(cx_px)), int(round(cy_px)), fw, fh, z_target=proj_z
            )
            wx2, wy2 = pixel_to_table_xy(
                int(round(cx_px + dx)), int(round(cy_px + dy)), fw, fh, z_target=proj_z
            )
            if wx_c2 is not None and wx2 is not None:
                _p1_px = (cx_px + dx, cy_px + dy)
                _p0_base = (float(wx_c2), float(wy_c2))
                _p1_base = (float(wx2), float(wy2))
                minor_axis_yaw_deg = math.degrees(
                    math.atan2(float(wy2) - float(wy_c2), float(wx2) - float(wx_c2))
                )
                yaw_conversion_ok = True
                _yaw_method = "minor_axis_projection"
        except Exception as _exc:
            panel._emit_log(f"[TFM_GRASP] yaw_conversion_err={_exc}")
    else:
        # Sin frame disponible: usar el yaw ya calculado en _last_grasp_base (fallback)
        yaw_conversion_ok = grasp_base_d.get("yaw_deg") is not None

    panel._emit_log(
        f"[TFM_GRASP][YAW_SOURCE] method={_yaw_method} "
        f"p0_px=({_p0_px[0]:.1f},{_p0_px[1]:.1f}) "
        f"p1_px=({_p1_px[0]:.1f},{_p1_px[1]:.1f}) "
        f"p0_base=({_p0_base[0]:.3f},{_p0_base[1]:.3f}) "
        f"p1_base=({_p1_base[0]:.3f},{_p1_base[1]:.3f}) "
        f"yaw_base_deg={minor_axis_yaw_deg:.2f}"
    )

    if not yaw_conversion_ok:
        panel._set_status(
            "TFM: No se pudo convertir la inferencia a pose base_link.", error=True
        )
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} "
            "reason=yaw_base_link_conversion_failed",
        )
        return

    # POSE log
    panel._emit_log(
        f"[TFM_GRASP][POSE] frame=base_link tcp=rg2_pinch_center "
        f"x={grasp_base_d.get('x', 0.0):.3f} y={grasp_base_d.get('y', 0.0):.3f} "
        f"z={grasp_base_d.get('z', 0.0):.3f} yaw_base={minor_axis_yaw_deg:.2f}deg"
    )

    # Almacenar overrides para _execute_tfm_world_grasp
    panel._tfm_grasp_minor_yaw_deg = minor_axis_yaw_deg
    panel._tfm_grasp_preopen_rad = finger_cmd_rad

    # TARGET: registra el objeto y la hipótesis de agarre
    object_id = str(
        getattr(panel, "_selected_object", "")
        or getattr(panel, "_last_grasp_selection_name", "")
        or "unknown"
    ).strip()
    panel._emit_log(
        "[TFM_GRASP][TARGET] "
        f"object_id={object_id} "
        f"grasp_rect=cx={cx_px:.1f},"
        f"cy={cy_px:.1f},"
        f"w={w_px:.1f},"
        f"h={h_px:.1f},"
        f"angle={angle_deg:.1f}deg "
        f"base=({grasp_base_d.get('x', 0.0):.3f},"
        f"{grasp_base_d.get('y', 0.0):.3f},"
        f"{grasp_base_d.get('z', 0.0):.3f}) "
        f"source=infer_model mode=moveit_sequence"
    )
    panel._audit_append(
        "logs/execute.log",
        "[TFM_GRASP] execute TARGET "
        f"object_id={object_id} source=infer_model mode=moveit_sequence "
        f"grasp_px=({cx_px:.1f},{cy_px:.1f},"
        f"w={w_px:.1f},h={h_px:.1f},"
        f"angle={angle_deg:.1f}deg) "
        f"minor_px={minor_px:.1f} "
        f"opening_axis_theta_img={math.degrees(opening_axis_theta_img):.2f}deg "
        f"minor_width_m={minor_width_m:.4f} "
        f"pre_open_width_m={pre_open_width_m:.4f} "
        f"finger_cmd_rad={finger_cmd_rad:.4f} "
        f"base=({grasp_base_d.get('x', 0.0):.3f},{grasp_base_d.get('y', 0.0):.3f},"
        f"{grasp_base_d.get('z', 0.0):.3f}) yaw_base={minor_axis_yaw_deg:.1f}",
    )

    # MOVEIT: publicar petición
    panel._emit_log(
        "[TFM_GRASP][MOVEIT] publish topic=/desired_grasp/request "
        f"mode=moveit_sequence source=infer_model object_id={object_id} "
        f"frame=base_link tcp=rg2_pinch_center "
        f"yaw_base={minor_axis_yaw_deg:.2f}deg"
    )
    panel._audit_append(
        "logs/execute.log",
        "[TFM_GRASP] execute REQUEST "
        f"object_id={object_id} source=infer_model mode=moveit_sequence "
        f"pose_topic={MOVEIT_POSE_TOPIC} result_topic=/desired_grasp/result",
    )

    handled = execute_tfm_world_grasp(panel)
    if not handled:
        panel._emit_log(
            "[TFM_GRASP][MOVEIT] result=FAIL mode=moveit_sequence source=infer_model "
            f"object_id={object_id} reason=execute_not_started"
        )
        panel._emit_log(
            "[TFM_GRASP][EXECUTE] status=FAIL mode=moveit_sequence source=infer_model "
            "grasp_orientation=minor_axis reason=execute_not_started"
        )
        panel._audit_append(
            "logs/execute.log",
            "[TFM_GRASP] execute FAIL mode=moveit_sequence source=infer_model "
            f"object_id={object_id} reason=execute_not_started",
        )
    # El resultado final (OK/FAIL) lo registra _execute_tfm_world_grasp vía [TFM_GRASP][EXECUTE]

def tfm_publish_grasp(panel):
    on_tfm_grasp_object_clicked(panel)
    if getattr(panel, "_tfm_execute_inflight", False):
        return True, "ejecucion iniciada"
    return False, "agarre no iniciado"

# ── Grasp geometry helpers (originally in panel_v2.py) ────────────────────
def _tfm_clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _tfm_normalize_angle(angle: float) -> float:
    import math as _math
    while angle > _math.pi:
        angle -= 2.0 * _math.pi
    while angle < -_math.pi:
        angle += 2.0 * _math.pi
    return angle


def _compute_minor_axis_from_grasp_rect(w_px: float, h_px: float, theta_img: float):
    if w_px <= h_px:
        minor_px = w_px
        opening_axis_theta_img = _tfm_normalize_angle(theta_img + 3.14159265358979 / 2.0)
    else:
        minor_px = h_px
        opening_axis_theta_img = _tfm_normalize_angle(theta_img)
    return minor_px, opening_axis_theta_img


def _compute_rg2_preopen_from_minor_width(
    minor_width_m: float,
    safety_margin_m: float = 0.015,
    min_open_m: float = 0.015,
    max_open_m: float = 0.110,
    max_finger_rad: float = 1.18,
):
    pre_open_width_m = _tfm_clamp(minor_width_m + safety_margin_m, min_open_m, max_open_m)
    finger_cmd_rad = _tfm_clamp((pre_open_width_m / max_open_m) * max_finger_rad, 0.0, max_finger_rad)
    return pre_open_width_m, finger_cmd_rad
