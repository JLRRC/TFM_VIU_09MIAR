#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_calibration.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Calibration helpers for the panel."""
from __future__ import annotations

import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication

from .calibration_service import CalibrationMode
from .panel_config import AUTO_CALIB_FROM_CAMERA, CALIBRATION_CAMERA_TOPIC
from .panel_objects import save_object_positions
from .panel_readiness import tf_ready_status
from .panel_utils import load_table_calib


def start_calibration(panel) -> None:
    panel._log_button("Calibrar")
    if getattr(panel, "_pick_target_lock_active", False):
        lock_name = str(getattr(panel, "_pick_target_lock_name", "") or "none")
        lock_id = str(getattr(panel, "_pick_target_lock_id", "") or "n/a")
        lock_reason = str(getattr(panel, "_pick_target_lock_reason", "") or "n/a")
        lock_source = str(getattr(panel, "_pick_target_lock_source", "") or "n/a")
        lock_ts = float(getattr(panel, "_pick_target_lock_ts", 0.0) or 0.0)
        lock_age = max(0.0, time.time() - lock_ts) if lock_ts > 0.0 else 0.0
        panel._emit_log(
            "[CALIB] Bloqueada por PICK activo "
            f"lock_id={lock_id} lock_name={lock_name} lock_source={lock_source} "
            f"lock_reason={lock_reason} lock_age={lock_age:.2f}s"
        )
        panel._set_status("Calibración bloqueada: PICK Objeto en curso", error=False)
        return

    if not panel._require_ready_basic("Calibración"):
        return
    tf_ok, tf_reason = tf_ready_status(panel)
    if not tf_ok:
        panel._set_status(f"TF no listo; esperando calibración ({tf_reason})", error=True)
        panel._emit_log(f"[CALIB] Bloqueada: {panel._tf_not_ready_reason()}")
        return

    if not panel._calibration_topic_allowed():
        panel._set_status(
            f"Calibración solo disponible en {CALIBRATION_CAMERA_TOPIC}",
            error=True,
        )
        return

    panel._capture_calibration_frame()
    if AUTO_CALIB_FROM_CAMERA and not (QApplication.keyboardModifiers() & Qt.ShiftModifier):
        if panel._auto_calibrate_from_camera():
            return

    if not panel._objects_settled:
        panel._request_settle_snapshot("calibrar")
        panel._log_calib_blocked("esperando caída/estabilidad de objetos")
        panel._set_status("Esperando caída/estabilización de objetos", error=False)
        return
    if not panel._pose_info_ok:
        panel._log_calib_blocked("pose/info no disponible")
        panel._set_status("Esperando pose/info", error=False)
        return
    if panel._camera_required and not panel._camera_stream_ok:
        panel._log_calib_blocked("cámara no publica")
        panel._set_status("Esperando cámara", error=False)
        return

    if not (QApplication.keyboardModifiers() & Qt.ShiftModifier):
        try:
            calib = load_table_calib()
        except Exception as exc:
            calib = None
            panel._log(f"[CALIB] Aviso: no se pudo leer calibración ({exc})")
        if calib:
            panel._calibrating = False
            panel._calib_points = []
            panel._calib_grid_until = time.time() + 1.0
            panel._load_table_calibration()
            panel._set_calibrate_button_text("Calibrar")
            panel._set_status("✅ Calibración aplicada (archivo)", error=False)
            panel._log("[CALIB] Calibración aplicada desde archivo (sin clicks)")
            panel._publish_calib_grid_marker()
            panel._post_calibration_pipeline()
            panel._refresh_objects_from_gz()
            save_object_positions()
            panel._emit_log("[CALIB] Objetos sincronizados y guardados tras calibración")
            return

    if panel._calibrating:
        panel._calibrating = False
        panel._calib_points = []
        panel._calib_grid_until = 0.0
        panel._set_calibrate_button_text("Calibrar")
        panel._set_status("Calibración desactivada", error=False)
        panel._log("[CALIB] Calibración desactivada")
        return

    try:
        calib = load_table_calib()
        if calib:
            panel._log("[CALIB] Calibración existente; iniciando modo manual")
            panel._set_status("Calibración existente; iniciando modo manual", error=False)
    except Exception as exc:
        panel._log(f"[CALIB] Aviso: no se pudo leer calibración guardada ({exc}), se ofrece modo manual")

    if not panel._camera_subscribed:
        panel._set_status("Conecta la cámara antes de calibrar", error=True)
        return

    panel._calibrating = True
    panel._calib_points = []
    panel._selected_object = None
    panel._selected_px = None
    panel._selected_world = None
    panel._calib_grid_until = time.time() + 1.0
    panel.calib_service.start_calibration(panel.camera_topic, CalibrationMode.LINEAR_2PT)
    panel._set_calibrate_button_text("✓ Calibrar (activo - click para desactivar)")
    panel._set_status(
        "CALIBRACIÓN: Click en 4 esquinas de la mesa (arriba-izq, arriba-der, abajo-der, abajo-izq)",
        error=False,
    )
    panel._log("[CALIB] Calibración manual 4 puntos (grid activo)")
