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

from .panel_config import INFER_CKPT, INFER_SCRIPT, LOG_DIR, VISION_DIR
from .panel_tfm_params import get_panel_tfm_params as _get_panel_tfm_params
from .panel_utils import ensure_dir, run_cmd
from .panel_objects import get_object_position, get_object_state, get_object_states, is_on_table, update_object_state
from .panel_pick_object import run_pick_object
from .panel_utils import pixel_to_table_xy
from .panel_tfm_preprocess import (  # noqa: F401
    _clip_roi,
    _get_cached_preprocessed_input,
    _resolve_infer_roi,
    _store_preprocessed_cache,
    build_tfm_preprocessed_input,
    reconcile_inferred_grasp_angle,
    reconcile_inferred_grasp_center,
    reconcile_inferred_grasp_size,
)
# F3: TFM inference flow extraído a panel_tfm_inference.py.
# Re-exportado para preservar API pública.
from .panel_tfm_inference import (  # noqa: F401
    _prepare_module_artifacts,
    _selection_snapshot,
    ensure_selected_object_in_store,
    handle_infer_result,
    latest_camera_frame_snapshot,
    restore_infer_selection_snapshot,
    sync_tfm_module_grasp_state,
    tfm_infer,
    tfm_infer_grasp,
)

# F3: TFM execute flow extraído a panel_tfm_execute.py.
from .panel_tfm_execute import (  # noqa: F401
    execute_tfm_world_grasp,
    tfm_visualize_grasp,
    wait_tfm_moveit_result,
)

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

# Grasp geometry helpers (compartidos con panel_v2 via panel_tfm_geometry).
from .panel_tfm_geometry import (  # noqa: E402,F401
    _compute_minor_axis_from_grasp_rect,
    _compute_rg2_preopen_from_minor_width,
    _tfm_clamp,
    _tfm_normalize_angle,
)
