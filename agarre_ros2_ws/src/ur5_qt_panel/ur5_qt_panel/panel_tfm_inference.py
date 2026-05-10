#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_inference.py
# Contenido: F3 — TFM inference flow + handlers (689 LOC).
"""TFM inference flow extraído de panel_tfm.py (F3).

Cubre:

* ``tfm_infer(panel)`` — kickoff del proceso de inferencia.
* ``tfm_infer_grasp(panel)`` — wrapper de UI.
* ``handle_infer_result(panel, result)`` — postprocesa el resultado.
* ``restore_infer_selection_snapshot(panel, snapshot)``.
* ``latest_camera_frame_snapshot(panel)`` — snapshot de la cámara.
* ``ensure_selected_object_in_store(panel, name, *, reason)``.
* ``sync_tfm_module_grasp_state(panel)`` — propaga grasp al módulo
  tfm_grasping cuando la inferencia da resultado.
* helpers privados ``_prepare_module_artifacts``, ``_selection_snapshot``.

Re-exportado desde panel_tfm para preservar la API pública.
"""

from __future__ import annotations

import datetime
import json
import math
import os
import re
import shlex
import time
from pathlib import Path
from typing import Optional

from .panel_config import INFER_CKPT, INFER_SCRIPT, LOG_DIR, VISION_DIR
from .panel_objects import (
    get_object_position,
    get_object_state,
    get_object_states,
    is_on_table,
    update_object_state,
)
from .panel_tfm_canonical import (
    complete_pending_tfm_infer_request,
)
from .panel_tfm_preprocess import (
    _get_cached_preprocessed_input,  # iter4-bis (2026-05-11): fix bug preexistente NameError en linea 250.
    _resolve_infer_roi,
    build_tfm_preprocessed_input,
    reconcile_inferred_grasp_angle,
    reconcile_inferred_grasp_center,
    reconcile_inferred_grasp_size,
)
from .panel_utils import ensure_dir, run_cmd





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


def _tfm_infer_run_script_mode(
    panel,
    *,
    qimg,
    w: int,
    h: int,
    frame_ts: float,
    roi,
    selection_snapshot: dict,
    ckpt_path: str,
) -> tuple[bool, str]:
    """F3-step18a: branch script-mode de tfm_infer (panel.tfm_module no cargado).

    Resuelve INFER_SCRIPT (predict.py preferido), guarda imagen, construye
    cmd con --config/--checkpoint/--image (predict.py) o --image/--ckpt/--out
    (legacy), spawn subprocess async + parsing JSON o CSV-like fallback.
    Devuelve (started, message).
    """
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

    return _tfm_infer_run_script_mode(
        panel,
        qimg=qimg,
        w=w,
        h=h,
        frame_ts=frame_ts,
        roi=roi,
        selection_snapshot=selection_snapshot,
        ckpt_path=ckpt_path,
    )

# ── Additional imports needed by extended TFM block ──────────────────────────
from typing import Dict, List, Tuple

from .panel_config import (
    OBJECT_SHAPES,
)
from .panel_objects import ObjectLogicalState
from .panel_camera import _runtime_time

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

# Canonical state machine + completion callbacks → panel_tfm_canonical.
from .panel_tfm_canonical import (  # noqa: F401
    build_tfm_pick_object_override,
    complete_pending_pick_demo_request,
    complete_pending_tfm_execute_request,
    tfm_canonical_finish,
    tfm_canonical_phase_update,
    tfm_canonical_state_reset,
    tfm_canonical_use_pick_object,
)

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

def _handle_infer_compute_alignment_2d(panel) -> Optional[Dict[str, object]]:
    """F3-step15a: calcula y registra alignment 2D entre grasp inferido y Cornell ref.

    Si hay panel._last_grasp_px y panel._last_cornell_ref, calcula deltas px,
    distancia, tamaños, y emite log [TFM] infer_align_2d. Devuelve dict con
    pred/ref/delta/dist o None si no hay datos suficientes.
    """
    if not (panel._last_grasp_px and panel._last_cornell_ref):
        return None
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
    return alignment_2d


def _handle_infer_write_audit(
    panel,
    *,
    result: Dict[str, object],
    infer_selection_policy: str,
    infer_postprocess_policy: str,
    infer_ckpt_path: str,
    infer_experiment: str,
    infer_seed: object,
    infer_model_info: dict,
    grasp_rect_publish_ok: bool,
    overlay_refresh_ok: bool,
    alignment_2d: Optional[Dict[str, object]],
) -> None:
    """F3-step15b: serializa audit_payload completo a artifacts/grasp_last.json
    y emite [TFM] infer_end OK con todas las métricas (perf, topics, frame_info,
    artifacts, alignment_2d).
    """
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


def _handle_infer_log_postprocess_adjustments(
    panel,
    *,
    angle_adjusted: bool,
    center_adjusted: bool,
    size_adjusted: bool,
    postprocess_enabled: bool,
    raw_pred: dict,
    pred: dict,
    ref_for_size: Optional[dict],
    roi: Optional[tuple],
) -> None:
    """F3-step15c: emite logs de ajustes postprocess (angle/center/size) +
    construye panel._last_tfm_postprocess_note + log [TFM] postprocess(_disabled).
    Aplica ajuste por ajuste con su [TFM] infer_*_adjust line correspondiente.
    """
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
    panel._last_grasp_px = {
        "cx": float(pred.get("cx", 0.0)),
        "cy": float(pred.get("cy", 0.0)),
        "w": float(pred.get("w", 0.0)),
        "h": float(pred.get("h", 0.0)),
        "angle_deg": float(pred.get("angle_deg", 0.0)),
    }
    _handle_infer_log_postprocess_adjustments(
        panel,
        angle_adjusted=angle_adjusted,
        center_adjusted=center_adjusted,
        size_adjusted=size_adjusted,
        postprocess_enabled=postprocess_enabled,
        raw_pred=raw_pred,
        pred=pred,
        ref_for_size=ref_for_size,
        roi=roi,
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
    alignment_2d = _handle_infer_compute_alignment_2d(panel)
    _handle_infer_write_audit(
        panel,
        result=result,
        infer_selection_policy=infer_selection_policy,
        infer_postprocess_policy=infer_postprocess_policy,
        infer_ckpt_path=infer_ckpt_path,
        infer_experiment=infer_experiment,
        infer_seed=infer_seed,
        infer_model_info=infer_model_info,
        grasp_rect_publish_ok=grasp_rect_publish_ok,
        overlay_refresh_ok=overlay_refresh_ok,
        alignment_2d=alignment_2d,
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

