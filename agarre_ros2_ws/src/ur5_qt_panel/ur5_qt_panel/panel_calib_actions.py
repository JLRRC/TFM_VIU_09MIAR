#!/usr/bin/env python3
"""Calibration, pick launch, and slider methods for ControlPanelV2."""
from __future__ import annotations

import json
import math
import os
import time
from typing import List, Optional

try:
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point
    from builtin_interfaces.msg import Duration
    from rclpy.time import Time
except Exception:
    Marker = None  # type: ignore
    Point = None   # type: ignore
    Duration = None  # type: ignore
    Time = None    # type: ignore

from .panel_config import (
    CALIB_GRID_STEP,
    JOINT_SLIDER_SCALE,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    TABLE_SIZE_X,
    TABLE_SIZE_Y,
    WORLD_FRAME,
)
from .panel_calibration import start_calibration
from .panel_objects import get_object_positions
from .panel_pick_demo import run_pick_demo
from .panel_pick_object import run_pick_object


def _log_exception(label: str, exc: Exception) -> None:
    try:
        import traceback
        print(f"[EXC][{label}] {exc}\n{traceback.format_exc()}")
    except Exception:
        pass


def _publish_calib_grid_marker(panel) -> None:
    if Marker is None or Point is None or Duration is None:
        panel._log("[CALIB] Marker no disponible")
        return
    panel._ensure_moveit_node()
    if panel._moveit_node is None:
        panel._log("[CALIB] ROS node no disponible para Marker")
        return
    if panel._marker_pub is None:
        try:
            panel._marker_pub = panel._moveit_node.create_publisher(Marker, "/calibration_grid", 1)
        except Exception as exc:
            panel._log(f"[CALIB] Error creando publisher Marker: {exc}")
            panel._marker_pub = None
    if panel._marker_pub is None:
        return

    table_top = panel._resolve_table_top_z()
    x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
    x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
    y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
    y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)
    step = max(0.02, CALIB_GRID_STEP)
    marker = Marker()
    marker.header.frame_id = WORLD_FRAME or "world"
    try:
        marker.header.stamp = Time().to_msg()
    except Exception as exc:
        _log_exception("calib marker stamp", exc)
    marker.ns = "calib_grid"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.004
    marker.color.r = 0.12
    marker.color.g = 0.40
    marker.color.b = 0.96
    marker.color.a = 0.8
    marker.pose.orientation.w = 1.0
    marker.lifetime = Duration(sec=1, nanosec=0)

    points: List[Point] = []
    x = x_min
    while x <= (x_max + 1e-6):
        p0 = Point(x=x, y=y_min, z=table_top)
        p1 = Point(x=x, y=y_max, z=table_top)
        points.extend([p0, p1])
        x += step
    y = y_min
    while y <= (y_max + 1e-6):
        p0 = Point(x=x_min, y=y, z=table_top)
        p1 = Point(x=x_max, y=y, z=table_top)
        points.extend([p0, p1])
        y += step
    marker.points = points
    panel._marker_pub.publish(marker)
    panel._set_status("Malla de calibración publicada en /calibration_grid", error=False)
    panel._log("[CALIB] Malla publicada en /calibration_grid")

def _start_calibration(panel):
    """Iniciar/Desactivar calibración manual mostrando la grilla."""
    start_calibration(panel)

def _set_calibrate_button_text(panel, text: str) -> None:
    if panel.btn_calibrate is not None:
        panel.btn_calibrate.setText(text)

def _capture_calibration_frame(panel) -> None:
    """Captura la última imagen disponible para trazabilidad."""
    if not panel._last_camera_frame:
        panel._log("[CALIB] Imagen no disponible para captura")
        return
    _qimg, w, h, ts = panel._last_camera_frame
    panel._log(f"[CALIB] Imagen capturada {w}x{h} age={time.time() - ts:.2f}s")

def _auto_calibrate_from_camera(panel) -> bool:
    """Auto-calibrar usando la cámara overhead y la geometría de la mesa."""
    from .panel_utils import (
        load_table_calib,
        TABLE_CALIB_PATH,
        TABLE_CAM_INFO,
        pixel_to_norm,
    )

    calib = load_table_calib()
    if not calib or not TABLE_CAM_INFO:
        return False
    w = getattr(panel.camera_view, "_img_width", 0) if hasattr(panel, "camera_view") else 0
    h = getattr(panel.camera_view, "_img_height", 0) if hasattr(panel, "camera_view") else 0
    if w <= 0 or h <= 0:
        try:
            w = int(TABLE_CAM_INFO.get("width") or 0)
            h = int(TABLE_CAM_INFO.get("height") or 0)
        except Exception as exc:
            _log_exception("auto_calib image size", exc)
            w = 0
            h = 0
    if w <= 0 or h <= 0:
        panel._log("[CALIB] Auto: tamaño de imagen desconocido")
        return False

    table_z = 0.850
    corners = [
        (TABLE_CENTER_X - TABLE_SIZE_X / 2.0, TABLE_CENTER_Y + TABLE_SIZE_Y / 2.0, table_z),
        (TABLE_CENTER_X + TABLE_SIZE_X / 2.0, TABLE_CENTER_Y + TABLE_SIZE_Y / 2.0, table_z),
        (TABLE_CENTER_X + TABLE_SIZE_X / 2.0, TABLE_CENTER_Y - TABLE_SIZE_Y / 2.0, table_z),
        (TABLE_CENTER_X - TABLE_SIZE_X / 2.0, TABLE_CENTER_Y - TABLE_SIZE_Y / 2.0, table_z),
    ]
    pixel_points = []
    world_points = []
    for x, y, z in corners:
        pix = world_xyz_to_pixel(x, y, z, w, h)
        if not pix:
            panel._log("[CALIB] Auto: no se pudo proyectar esquina de mesa")
            return False
        nx, ny = pixel_to_norm(pix[0], pix[1], w, h)
        pixel_points.append((nx, ny))
        world_points.append((x, y))

    try:
        hom = panel._compute_homography(pixel_points, world_points)
    except Exception as exc:
        panel._log(f"[CALIB] Auto: error calculando homografía: {exc}")
        return False
    if hom is None:
        panel._log("[CALIB] Auto: homografía inválida")
        return False

    payload = {
        "mode": "homography",
        "h": hom,
        "camera": TABLE_CAM_INFO,
    }
    try:
        with open(TABLE_CALIB_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
    except Exception as exc:
        panel._log(f"[CALIB] Auto: no se pudo guardar {TABLE_CALIB_PATH}: {exc}")
        return False

    panel._load_table_calibration()
    panel._calib_grid_until = time.time() + 1.0
    panel._calibrating = False
    panel._calib_points = []
    panel._set_status("✅ Calibración auto aplicada", error=False)
    panel._log("[CALIB] Auto: calibración guardada y cargada")
    panel._publish_calib_grid_marker()
    panel._post_calibration_pipeline()
    panel._refresh_objects_from_gz_async()
    return True
def _pick_confirm_dialog(parent, btn_label: str, frame: str, pose_str: str) -> bool:
    """Muestra diálogo de confirmación con pose actual de la pinza.
    Devuelve True si el usuario pulsa Iniciar, False si cancela.
    Se ejecuta en el GUI thread antes de que el worker arranque.
    """
    # Auto-accept in offscreen/automated modes — no one can click the dialog.
    if os.environ.get("PANEL_FORCE_OFFSCREEN", "0") == "1" or \
            os.environ.get("PANEL_AUTO_RUN_PICK_DEMO", "0") in ("1", "true", "yes", "on"):
        return True
    from PyQt5.QtWidgets import QMessageBox, QPushButton
    dlg = QMessageBox(parent)
    dlg.setWindowTitle(f"Confirmar: {btn_label}")
    dlg.setText(
        f"<b>Botón pulsado:</b> {btn_label}<br><br>"
        f"<b>Pose actual de la pinza</b><br>"
        f"&nbsp;&nbsp;frame: <tt>{frame}</tt><br>"
        f"&nbsp;&nbsp;XYZ&nbsp;&nbsp;: <tt>{pose_str}</tt><br><br>"
        f"Pulse <b>Iniciar</b> para cargar y arrancar la secuencia,<br>"
        f"o <b>Cancelar</b> para abortar."
    )
    btn_sig = dlg.addButton("Iniciar", QMessageBox.AcceptRole)
    btn_can = dlg.addButton("Cancelar",  QMessageBox.RejectRole)
    dlg.setDefaultButton(btn_sig)
    dlg.exec_()
    return dlg.clickedButton() is btn_sig

def _run_pick_demo(panel):
    """Ejecuta la ruta directa del demo sobre el objeto seleccionado."""
    if panel._direct_waiting_for_approach_confirmation():
        if panel._step_mode == "STEP_BY_STEP":
            panel._log_button("Agarre Objeto (Directo) [ignorado: authority=step_panel]")
            panel._emit_log(
                "[PICK][DIRECT][AUTH] "
                "action=ignore_main_panel_approach "
                "authority=step_panel "
                "reason=step_by_step_single_source_of_truth"
            )
            return
        panel._log_button("Agarre Objeto (Directo) [confirmar APPROACH_COARSE]")
        if panel._direct_release_waiting_for_approach_confirmation():
            panel._emit_log(
                "[BOTON] Confirmado: continuar desde MESA hacia APPROACH_COARSE"
            )
        return
    panel._log_button("Agarre Objeto (Directo)")
    panel._step_capture_start_pose("Agarre Objeto (Directo)")
    _op_frame = panel._step_operational_frame_name()
    _fresh_xyz = panel._step_fetch_live_pose(_op_frame)
    if _fresh_xyz is not None:
        _px, _py, _pz = _fresh_xyz
        _pose_str = f"({_px:.4f}, {_py:.4f}, {_pz:.4f})"
    else:
        _pose_str = "no disponible (TF no listo)"
    panel._emit_log(
        f"[BOTON] Pulsado: Agarre Objeto (Directo) | "
        f"frame={_op_frame} | Pose pinza ahora: {_pose_str}"
    )
    if panel._step_mode == "STEP_BY_STEP":
        panel._step_prepare_pipeline_view("DIRECT")
    if not panel._pick_confirm_dialog(panel, "Agarre Objeto (Directo)", _op_frame, _pose_str):
        panel._emit_log("[BOTON] Inicio cancelado por el usuario")
        if panel._step_mode == "STEP_BY_STEP":
            panel._step_reset_sequence_view(clear_history=True)
            panel._step_window_refresh()
        return
    run_pick_demo(panel)


def _run_pick_object(panel):
    """Ejecuta pick & place del objeto seleccionado hacia la cesta."""
    panel._log_button("Agarre Objeto (MoveIT)")
    panel._step_capture_start_pose("Agarre Objeto (MoveIT)")
    _op_frame = panel._step_operational_frame_name()
    _fresh_xyz = panel._step_fetch_live_pose(_op_frame)
    if _fresh_xyz is not None:
        _px, _py, _pz = _fresh_xyz
        _pose_str = f"({_px:.4f}, {_py:.4f}, {_pz:.4f})"
    else:
        _pose_str = "no disponible (TF no listo)"
    panel._emit_log(
        f"[BOTON] Pulsado: Agarre Objeto (MoveIT) | "
        f"frame={_op_frame} | Pose pinza ahora: {_pose_str}"
    )
    if panel._step_mode == "STEP_BY_STEP":
        panel._step_prepare_pipeline_view("PICK_OBJECT")
        panel._step_pre_insert_inicio_row(
            _fresh_xyz,
            target_pos=None,
            obj_pos=None,
            flow_name="PICK_OBJECT",
        )
    if not panel._pick_confirm_dialog(panel, "Agarre Objeto (MoveIT)", _op_frame, _pose_str):
        panel._emit_log("[BOTON] Inicio cancelado por el usuario")
        if panel._step_mode == "STEP_BY_STEP":
            panel._step_reset_sequence_view(clear_history=True)
            panel._step_window_refresh()
        return
    run_pick_object(panel)

def _get_object_world_position(panel, obj_name: str) -> Optional[tuple]:
    """Obtener posición mundial del objeto desde poses actuales."""
    try:
        positions = get_object_positions()
        if positions and obj_name in positions:
            pos = positions[obj_name]
            # pos es una tupla (x, y, z)
            x, y, z = pos
            return (float(x), float(y), float(z))
    except Exception as e:
        panel._log(f"[PICK] Error obteniendo posición: {e}")
    return None

def _on_slider_change(panel, idx: int, value: int):
    # Ignorar cambios si vienen de _on_joint_state (flag)
    if panel._updating_sliders_from_joint_state:
        return
    
    deg = float(value) / JOINT_SLIDER_SCALE
    rad = math.radians(deg)
    panel.joint_value_labels[idx].setText(f"{deg:.1f} deg / {rad:.3f} rad")
    
    # Bloquear actualizaciones de sliders por 2 segundos después de movimiento manual
    panel._slider_update_blocked_until = time.time() + 2.0
    
    # Debug: mostrar cambio incremental cuando se mueve un slider
    if panel._debug_logs_enabled:
        joint_name = UR5_JOINT_NAMES[idx]
        old_value = panel._last_slider_values.get(idx, value)
        delta_deg = deg - (float(old_value) / JOINT_SLIDER_SCALE)
        direction = "↑" if delta_deg > 0 else "↓" if delta_deg < 0 else "→"
        if abs(delta_deg) > 0.5:  # Solo mostrar cambios significativos
            panel._log(f"[SLIDER] {joint_name:25s} {direction} {deg:+7.1f}° (delta: {delta_deg:+.1f}°)")
        panel._last_slider_values[idx] = value

def _slider_to_deg(panel, value: int) -> float:
    return float(value) / JOINT_SLIDER_SCALE

def _current_joint_positions_rad(panel):
    positions = [math.radians(panel._slider_to_deg(s.value())) for s in panel.joint_sliders]
    
    # Debug: mostrar valores de sliders
    if panel._debug_logs_enabled:
        slider_values = [s.value() for s in panel.joint_sliders]
        deg_values = [panel._slider_to_deg(v) for v in slider_values]
        panel._log(f"[SLIDER_READ] slider_raw={slider_values}")
        panel._log(f"[SLIDER_READ] slider_deg={[f'{d:.1f}' for d in deg_values]}")
    
    return positions
