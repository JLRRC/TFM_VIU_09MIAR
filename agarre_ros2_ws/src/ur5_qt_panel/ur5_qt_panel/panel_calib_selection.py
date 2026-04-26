#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_calib_selection.py
# Contenido: Calibration and object selection callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Calibration and object selection callbacks for ControlPanelV2."""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

from .panel_config import (
    PICK_TF_TIMEOUT_SEC,
    SELECTION_SNAP_DIST,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    WORLD_FRAME,
)
from .panel_objects import (
    ObjectLogicalState,
    get_object_positions,
    get_object_state,
    is_on_table,
    update_object_state,
)
from .panel_utils import world_xyz_to_pixel


def _log_exception(context: str, exc: Exception) -> None:
    print(f"[CALIB_SEL][ERROR][{context}] {exc}")


def _handle_calibration_click(panel, px: int, py: int):
    """Manejar click durante calibración (IGUAL A PANEL ONLY - SIN DIÁLOGOS)."""
    if not panel._calibrating:
        return
    
    # ✅ Simplemente guardar el píxel clickeado
    panel._calib_points.append((px, py))
    panel._log(f"[CALIB] Punto {len(panel._calib_points)}: píxel ({px}, {py})")
    
    # Si tenemos 4 puntos, calcular calibración automáticamente
    if len(panel._calib_points) >= 4:
        panel._finish_calibration()
    else:
        # Mostrar progreso
        needed = 4 - len(panel._calib_points)
        panel._set_status(f"CALIBRACIÓN: {len(panel._calib_points)}/4 puntos - Click {needed} más", error=False)

def _finish_calibration(panel):
    """Finalizar calibración calculando homografía automáticamente (IGUAL A PANEL ONLY)."""
    if len(panel._calib_points) < 4:
        panel._log("[CALIB] Calibración incompleta")
        return
    
    # ✅ Extraer píxeles clickeados
    p1_px, p1_py = panel._calib_points[0]
    p2_px, p2_py = panel._calib_points[1]
    p3_px, p3_py = panel._calib_points[2]
    p4_px, p4_py = panel._calib_points[3]
    
    # ✅ Calcular coordenadas del mundo basadas en la tabla
    # Asumimos que los 4 puntos son las esquinas de la mesa
    cx = TABLE_CENTER_X
    cy = TABLE_CENTER_Y
    sx = TABLE_SIZE_X / 2.0
    sy = TABLE_SIZE_Y / 2.0
    
    # Esquinas en coordenadas del mundo (arriba-izq, arriba-der, abajo-der, abajo-izq)
    w1 = (cx - sx, cy + sy)   # Arriba-izq
    w2 = (cx + sx, cy + sy)   # Arriba-der
    w3 = (cx + sx, cy - sy)   # Abajo-der
    w4 = (cx - sx, cy - sy)   # Abajo-izq
    
    # ✅ Validación mejorada: que los 4 puntos formen un cuadrilátero razonable
    # Chequea distancias entre puntos consecutivos y diagonales
    def dist(p1, p2):
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    
    points = [(p1_px, p1_py), (p2_px, p2_py), (p3_px, p3_py), (p4_px, p4_py)]
    
    # Verificar que cada par de puntos adyacentes estén separados >= 20px
    min_distance = 20
    dist_12 = dist(points[0], points[1])
    dist_23 = dist(points[1], points[2])
    dist_34 = dist(points[2], points[3])
    dist_41 = dist(points[3], points[0])
    
    panel._log(f"[CALIB] Distancias: P1-P2={dist_12:.1f}px, P2-P3={dist_23:.1f}px, P3-P4={dist_34:.1f}px, P4-P1={dist_41:.1f}px")
    
    if min(dist_12, dist_23, dist_34, dist_41) < min_distance:
        panel._log(f"[CALIB] ERROR: puntos muy cercanos (mínimo {min_distance}px). Repite calibración.")
        panel._set_status(f"ERROR: puntos muy cercanos ({min_distance}px mín)", error=True)
        panel._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR
        panel._set_status("Calibración reiniciada - click en 4 esquinas", error=False)
        return
    
    # ✅ Calcular homografía
    try:
        pixel_points = [(float(p1_px), float(p1_py)), (float(p2_px), float(p2_py)),
                       (float(p3_px), float(p3_py)), (float(p4_px), float(p4_py))]
        world_points = [(float(w1[0]), float(w1[1])), (float(w2[0]), float(w2[1])),
                       (float(w3[0]), float(w3[1])), (float(w4[0]), float(w4[1]))]

        hom = None
        # Intentar usar graspnet si está instalado
        try:
            from graspnet.utils.homography import compute_homography as g_compute
            hom = g_compute(pixel_points, world_points)
        except Exception as e:
            panel._log(f"[CALIB] Aviso: graspnet no disponible ({e}), usando OpenCV")
            try:
                import numpy as np
                import cv2
                src = np.array(pixel_points, dtype=np.float32)
                dst = np.array(world_points, dtype=np.float32)
                hom, _ = cv2.findHomography(src, dst, method=0)
                if hom is not None:
                    hom = hom.tolist()
            except Exception as e_cv:
                raise RuntimeError(f"No se pudo calcular homografía: {e_cv}")

        if not hom:
            panel._log("[CALIB] ERROR: homografía inválida. Repite calibración.")
            panel._set_status("ERROR: homografía inválida", error=True)
            panel._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR
            return
        
        # ✅ Enviar calibración al servicio
        panel.calib_service.set_homography(hom)
        panel._calibrating = False
        panel._calib_points = []  # ✅ LIMPIAR PUNTOS DESPUÉS DE ÉXITO
        panel._set_calibrate_button_text("Calibrar")
        panel._set_status(f"✅ Calibración completada ({dist_12:.0f}-{dist_34:.0f}px)", error=False)
        panel._log(f"[CALIB] ✅ Homografía completada y almacenada en servicio")
        panel._publish_calib_grid_marker()
        panel._post_calibration_pipeline()
        panel._refresh_objects_from_gz()
        save_object_positions()
        panel._emit_log("[CALIB] Objetos sincronizados y guardados tras calibración")
        
    except Exception as e:
        panel._log(f"[CALIB] ERROR calculando homografía: {e}")
        panel._set_status(f"ERROR: {e}", error=True)
        panel._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR


def _handle_object_selection_click(panel, px: int, py: int):
    """Manejar click en cámara para seleccionar objeto (igual a Panel Only)."""
    ok, reason = pick_ui_status(panel)
    if not ok and "pose/info" not in str(reason):
        panel._emit_log(f"[PICK] Bloqueado: {reason}")
        return
    if not ok:
        # La selección queda marcada; solo bloqueamos la ejecución posterior de PICK.
        panel._emit_log(f"[PICK] Selección permitida sin pose/info: {reason}")
    panel._emit_log(f"[PICK][SELECT_ATTEMPT] px=({px},{py})")
    if getattr(panel, "_pick_target_lock_active", False):
        lock_name = str(getattr(panel, "_pick_target_lock_name", "") or "none")
        lock_id = str(getattr(panel, "_pick_target_lock_id", "") or "n/a")
        lock_source = str(getattr(panel, "_pick_target_lock_source", "") or "n/a")
        lock_reason = str(getattr(panel, "_pick_target_lock_reason", "") or "n/a")
        lock_ts = float(getattr(panel, "_pick_target_lock_ts", 0.0) or 0.0)
        lock_age = max(0.0, time.time() - lock_ts) if lock_ts > 0.0 else 0.0
        panel._emit_log(
            "[PICK][SELECT] ignored_selection_update_due_to_pick_active=true "
            f"source=camera_click lock_name={lock_name} lock_id={lock_id} "
            f"lock_source={lock_source} lock_reason={lock_reason} lock_age={lock_age:.2f}s"
        )
        panel._set_status("Selección bloqueada: PICK Objeto en curso", error=False)
        return
    # Usar homografía global/table map para convertir a mundo
    w = getattr(panel.camera_view, "_img_width", 0)
    h = getattr(panel.camera_view, "_img_height", 0)
    world_x, world_y = pixel_to_table_xy(px, py, w, h)
    if world_x is None or world_y is None:
        panel._log("[PICK] No hay calibración válida (pixel_to_table_xy) - carga table_pixel_map.json o calibra")
        panel._set_status("⚠️ Calibra la cámara o carga table_pixel_map.json", error=True)
        return

    obj_name = nearest_table_object(world_x, world_y)
    obj_pos = get_object_position(obj_name) if obj_name else None
    world_z = obj_pos[2] if obj_pos else 0.0
    base_event = panel._ensure_base_coords(
        (float(world_x), float(world_y), float(world_z)),
        panel._world_frame_last_first(),
        timeout_sec=0.35,
    )
    base_txt = "base=(n/a)"
    if base_event is not None:
        base_txt = (
            f"base=({float(base_event[0]):.3f},{float(base_event[1]):.3f},{float(base_event[2]):.3f})"
        )
    panel._emit_log(
        "[PICK][SELECT_EVT] "
        f"source=camera_click px=({px},{py}) candidate={obj_name or 'none'} "
        f"{base_txt}"
    )

    if obj_pos:
        # Recalcular con la altura del objeto para ajustar la proyección
        wx_adj, wy_adj = pixel_to_table_xy(px, py, w, h, z_target=obj_pos[2]) or (world_x, world_y)
        world_x, world_y = wx_adj, wy_adj
        dx = world_x - obj_pos[0]
        dy = world_y - obj_pos[1]
        snapped = math.hypot(dx, dy) <= SELECTION_SNAP_DIST
        if snapped:
            world_x, world_y = obj_pos[0], obj_pos[1]
            pix = world_xyz_to_pixel(world_x, world_y, obj_pos[2], w, h)
            if not pix:
                pix = table_xy_to_pixel(world_x, world_y, w, h)
            if pix:
                px, py = pix
        state = get_object_state(obj_name) if obj_name else None
        geom_on_table = is_on_table(obj_pos) if obj_pos else False
        state_ok = bool(
            state
            and (
                state.logical_state == ObjectLogicalState.ON_TABLE
                or (state.logical_state == ObjectLogicalState.SELECTED and geom_on_table)
            )
        )
        if not state_ok:
            details = f"state={state.logical_state.value}" if state else "state=unknown"
            pos_txt = f"pos=({obj_pos[0]:.3f},{obj_pos[1]:.3f},{obj_pos[2]:.3f})" if obj_pos else "pos=n/a"
            from .panel_objects import table_geom_debug
            geom = table_geom_debug(obj_pos) if obj_pos else ""
            panel._emit_log(f"[PICK] Bloqueado: {obj_name} no está ON_TABLE ({details} {pos_txt} {geom})")
            panel._set_status("Objeto no está sobre la mesa", error=True)
            return
        panel._select_object(
            obj_name,
            px,
            py,
            world_x,
            world_y,
            world_z,
            source="camera_click",
        )
        return

    # Click en vacío (no objetos cerca)
    base_empty = panel._ensure_base_coords(
        (float(world_x), float(world_y), 0.0),
        panel._world_frame_last_first(),
        timeout_sec=0.35,
    )
    if base_empty is not None:
        panel._log(
            "[PICK] Click en vacío: "
            f"px=({px},{py}) → base=({float(base_empty[0]):.2f},{float(base_empty[1]):.2f})"
        )
    else:
        panel._log(f"[PICK] Click en vacío: px=({px},{py}) → base=(n/a)")
    if panel._selected_object:
        prev_state = get_object_state(panel._selected_object)
        if prev_state and prev_state.logical_state == ObjectLogicalState.SELECTED:
            update_object_state(panel._selected_object, logical_state=ObjectLogicalState.ON_TABLE, reason="deselect_empty")
    panel._selected_object = None
    panel._selected_px = (px, py)
    panel._selected_world = (world_x, world_y, 0.0)
    panel._selected_base = base_empty
    panel._selection_last_user_name = ""
    panel._selection_last_user_ts = 0.0
    panel._last_cornell = None
    panel._refresh_science_ui()
    panel._update_objects()
    panel._log_selection_tf(panel._selected_world)

def _select_object(panel,
    name: str,
    px: int,
    py: int,
    wx: float,
    wy: float,
    wz: float,
    *,
    source: str = "unknown",
):
    """Seleccionar un objeto (desde click o desde lista)."""
    if bool(getattr(panel, "_calibrating", False)):
        panel._emit_log(
            "[PICK][SELECT] blocked_calibration_active=true "
            f"source={source} name={name}"
        )
        panel._set_status("Selección bloqueada: calibración activa", error=False)
        return
    if getattr(panel, "_pick_target_lock_active", False):
        lock_name = str(getattr(panel, "_pick_target_lock_name", "") or "none")
        lock_id = str(getattr(panel, "_pick_target_lock_id", "") or "n/a")
        lock_source = str(getattr(panel, "_pick_target_lock_source", "") or "n/a")
        lock_reason = str(getattr(panel, "_pick_target_lock_reason", "") or "n/a")
        lock_ts = float(getattr(panel, "_pick_target_lock_ts", 0.0) or 0.0)
        lock_age = max(0.0, time.time() - lock_ts) if lock_ts > 0.0 else 0.0
        panel._emit_log(
            "[PICK][SELECT] ignored_selection_update_due_to_pick_active=true "
            f"source={source} name={name} lock_name={lock_name} lock_id={lock_id} "
            f"lock_source={lock_source} lock_reason={lock_reason} lock_age={lock_age:.2f}s"
        )
        return
    prev_name = panel._selected_object
    selected_ts = time.time()
    state_updated = update_object_state(name, logical_state=ObjectLogicalState.SELECTED, reason="select")
    panel._emit_log(
        "[PICK][SELECT_STORE] "
        f"source={source} name={name} ts={selected_ts:.3f} update_ok={str(state_updated).lower()}"
    )
    if not state_updated:
        panel._set_status(f"Selección rechazada: {name}", error=True)
        panel._emit_log(f"[PICK][SELECT] selección rechazada por store para {name}")
        return
    if prev_name and prev_name != name:
        prev_state = get_object_state(prev_name)
        if prev_state and prev_state.logical_state == ObjectLogicalState.SELECTED:
            update_object_state(prev_name, logical_state=ObjectLogicalState.ON_TABLE, reason="deselect")
    panel._selected_object = name
    panel._selected_px = (px, py)
    panel._selected_world = (wx, wy, wz)
    selected_base = panel._ensure_base_coords(
        (float(wx), float(wy), float(wz)),
        panel._world_frame_last_first(),
        timeout_sec=0.35,
    )
    panel._selected_base = selected_base
    panel._selected_base_frame = panel._business_base_frame()
    panel._selection_timestamp = selected_ts
    panel._selection_last_user_name = name
    panel._selection_last_user_ts = selected_ts
    px_label = f"px=({px},{py})" if px >= 0 and py >= 0 else "px=n/a"
    base_label = "base=(n/a)"
    if selected_base is not None:
        base_label = (
            f"base=({float(selected_base[0]):.2f},{float(selected_base[1]):.2f},{float(selected_base[2]):.2f})"
        )
    panel._log(
        f"[PICK][SELECT] source={source} name={name} ts={selected_ts:.3f} "
        f"{px_label} {base_label}"
    )
    state_after = get_object_state(name)
    panel._emit_log(
        "[PICK][SELECT_STATE] "
        f"name={name} state={state_after.logical_state.value if state_after else 'none'} "
        f"state_ts={float(getattr(state_after, 'last_update_ts', 0.0) or 0.0):.3f} "
        f"user_ts={selected_ts:.3f}"
    )
    if selected_base is not None:
        panel._set_status(
            f"✓ Seleccionado: {name} ({float(selected_base[0]):.2f},{float(selected_base[1]):.2f})",
            error=False,
        )
    else:
        panel._set_status(f"✓ Seleccionado: {name}", error=False)
    if panel._last_grasp_px and panel._last_camera_frame:
        _qimg, w, h, _ts = panel._last_camera_frame
        panel._refresh_cornell_metrics(w, h)
        panel._refresh_science_ui()
    panel._update_objects()
    panel._log_selection_tf(panel._selected_world)

def _log_selection_tf(panel, world_pose: Tuple[float, float, float], frame: str = WORLD_FRAME or "world"):
    """Store and log selection in business frame (base_link)."""
    if not world_pose:
        return
    world_frame = frame or panel._expected_world_frame()
    panel._last_selection_frame = world_frame
    panel._last_selected_world_pose = (world_pose[0], world_pose[1], world_pose[2], world_frame)
    base_info = panel._selection_to_base(world_pose, world_frame)
    if base_info:
        bx, by, bz = base_info["coords"]
        bf = str(base_info["frame"] or panel._business_base_frame())
        panel._selected_base = (float(bx), float(by), float(bz))
        panel._selected_base_frame = bf
        panel._last_selected_base_pose = (float(bx), float(by), float(bz), bf)
        panel._emit_log(
            "[PICK] selected_base="
            f"({float(bx):.3f},{float(by):.3f},{float(bz):.3f}) frame={bf}"
        )
    else:
        panel._emit_log("[PICK] selected_base unavailable (tf_pending)")
    panel._start_tf_diagnose_async(world_pose, world_frame)
    panel._start_pick_tf_resolve(world_pose, world_frame)

def _start_tf_diagnose_async(panel, world_pose: Tuple[float, float, float], world_frame: str) -> None:
    panel._run_async(lambda: run_tf_diagnose(panel, world_pose, world_frame))

def _start_pick_tf_resolve(panel, world_pose: Tuple[float, float, float], world_frame: str) -> None:
    if panel._pick_tf_inflight:
        return
    if not panel._moveit_required:
        panel._emit_log("[PICK] Bloqueado: MoveIt deshabilitado")
        panel._ui_set_status("PICK en espera: MoveIt no activo", error=False)
        return
    if panel._managed_mode:
        if panel._moveit_state != MoveItState.READY:
            reason = panel._set_moveit_wait_status("pick")
            panel._emit_log(f"[PICK] Bloqueado: MoveIt ({reason})")
            return
        if not panel._state_ready_basic():
            reason = panel._system_state_reason or panel._system_state.value
            panel._emit_log(f"[PICK] Bloqueado: estado={panel._system_state.value}")
            panel._ui_set_status(f"PICK en espera: {reason}", error=False)
            return
    elif not panel._state_ready_moveit():
        reason = panel._system_state_reason or panel._system_state.value
        panel._emit_log(f"[PICK] Bloqueado: estado={panel._system_state.value}")
        panel._ui_set_status(f"PICK en espera: {reason}", error=False)
        return
    if not panel._pose_info_ok:
        panel._emit_log("[PICK] Bloqueado: pose/info no disponible")
        panel._ui_set_status("PICK en espera: pose/info no disponible", error=False)
        return
    tf_ok, tf_reason = tf_ready_status(panel)
    if not tf_ok:
        panel._emit_log(f"[PICK] Bloqueado: {tf_reason}")
        panel._ui_set_status(f"PICK en espera: {tf_reason}", error=False)
        return
    panel._pick_tf_inflight = True
    candidates = panel._base_frame_candidates()
    base_frame = candidates[0]

    def worker():
        try:
            coords = None
            start = time.time()
            while (time.time() - start) < PICK_TF_TIMEOUT_SEC:
                coords, _ = transform_point_to_frame(
                    world_pose,
                    base_frame,
                    source_frame=world_frame,
                    timeout_sec=PICK_TF_RETRY_SEC,
            )
                if coords:
                    break
                panel._wait_for_state_change(PICK_TF_RETRY_SEC)
            if not coords:
                reason = panel._tf_not_ready_reason()
                panel._log(f"[PICK] Bloqueado: {reason}")
                panel._ui_set_status(f"PICK en espera: {reason}", error=False)
                return
            panel._last_selected_base_pose = (coords[0], coords[1], coords[2], base_frame)
            panel._selected_base = (float(coords[0]), float(coords[1]), float(coords[2]))
            panel._selected_base_frame = base_frame
            panel._motion_in_progress = False
            panel._ui_set_status("Objeto seleccionado; listo para PICK", error=False)
        finally:
            panel._pick_tf_inflight = False

    panel._run_async(worker)

def _selection_candidate(panel) -> Optional[Dict[str, object]]:
    """Return the last selected object/base tuple if still recent."""
    if not panel._selected_object:
        return None
    if panel._selected_base is None and panel._selected_world is not None:
        source_frame = panel._world_frame_last_first()
        panel._selected_base = panel._ensure_base_coords(
            tuple(panel._selected_world), source_frame, timeout_sec=0.35
        )
        panel._selected_base_frame = panel._business_base_frame()
    if panel._selected_base is None:
        return None
    age = time.time() - panel._selection_timestamp
    if age > SELECTION_TIMEOUT_SEC:
        if getattr(panel, "_pick_target_lock_active", False):
            lock_name = str(getattr(panel, "_pick_target_lock_name", "") or "none")
            lock_id = str(getattr(panel, "_pick_target_lock_id", "") or "n/a")
            panel._emit_log_throttled(
                "PICK:selection_timeout:lock_active",
                "[PICK][SELECT] selection_timeout_ignored_due_to_pick_active=true "
                f"name={panel._selected_object} age={age:.1f}s lock_name={lock_name} lock_id={lock_id}",
            )
            frame = panel._selected_base_frame or panel._business_base_frame()
            return {
                "name": panel._selected_object,
                "base": tuple(panel._selected_base),
                "frame": frame,
                "age": age,
            }
        panel._log(f"[PICK] Selección expirada (age={age:.1f}s) -> clearing selection")
        prev_state = get_object_state(panel._selected_object)
        if prev_state and prev_state.logical_state == ObjectLogicalState.SELECTED:
            update_object_state(panel._selected_object, logical_state=ObjectLogicalState.ON_TABLE, reason="select_timeout")
        panel._selected_object = None
        panel._selected_world = None
        panel._selected_base = None
        return None
    frame = panel._selected_base_frame or panel._business_base_frame()
    return {
        "name": panel._selected_object,
        "base": tuple(panel._selected_base),
        "frame": frame,
        "age": age,
    }

def _selection_to_base(panel, world_pos: Tuple[float, float, float], source_frame: str) -> Optional[Dict[str, object]]:
    """Try transforming the selected point into a base frame (prefers effective frames)."""
    candidates = panel._base_frame_candidates()
    for base_frame in candidates:
        coords, transform = transform_point_to_frame(world_pos, base_frame, source_frame)
        if coords:
            tx = ty = tz = 0.0
            yaw = 0.0
            if transform is not None:
                tx = float(transform.transform.translation.x)
                ty = float(transform.transform.translation.y)
                tz = float(transform.transform.translation.z)
                yaw = yaw_from_quaternion(transform.transform.rotation)
            return {
                "coords": coords,
                "frame": base_frame,
                "transform": transform,
                "translation": (tx, ty, tz),
                "yaw": yaw,
                "via_tf": bool(transform is not None),
            }
    return None

def _on_object_clicked(panel, name: str):
    """Manejar click en objeto de la lista (IGUAL A PANEL ONLY)."""
    if hasattr(panel, "obj_panel") and not panel.obj_panel.isEnabled():
        return
    panel._emit_log(f"[PICK][LIST_CLICK] name={name}")
    ok, reason = pick_ui_status(panel)
    if not ok:
        panel._emit_log(
            f"[PICK][LIST_CLICK] not_ready_but_continuing name={name} reason={reason}"
        )
    if getattr(panel, "_pick_target_lock_active", False):
        lock_name = str(getattr(panel, "_pick_target_lock_name", "") or "none")
        lock_id = str(getattr(panel, "_pick_target_lock_id", "") or "n/a")
        lock_source = str(getattr(panel, "_pick_target_lock_source", "") or "n/a")
        lock_reason = str(getattr(panel, "_pick_target_lock_reason", "") or "n/a")
        lock_ts = float(getattr(panel, "_pick_target_lock_ts", 0.0) or 0.0)
        lock_age = max(0.0, time.time() - lock_ts) if lock_ts > 0.0 else 0.0
        panel._emit_log(
            "[PICK][SELECT] ignored_selection_update_due_to_pick_active=true "
            f"source=object_list name={name} lock_name={lock_name} lock_id={lock_id} "
            f"lock_source={lock_source} lock_reason={lock_reason} lock_age={lock_age:.2f}s"
        )
        return
    # Obtener posición del objeto
    objects = get_object_positions()
    if name not in objects:
        panel._log(f"[PICK] Objeto {name} no encontrado en Gazebo")
        return
    
    x, y, z = objects[name]
    base_pose = panel.get_pose_in_base(name, timeout_sec=0.35)
    base_obj = None
    if base_pose is not None:
        base_obj = (
            float(base_pose.pose.position.x),
            float(base_pose.pose.position.y),
            float(base_pose.pose.position.z),
        )
    if base_obj is not None:
        panel._log(
            f"[PICK] Click en objeto: {name} @ base ({float(base_obj[0]):.3f}, {float(base_obj[1]):.3f}, {float(base_obj[2]):.3f})"
        )
    else:
        panel._log(f"[PICK] Click en objeto: {name} @ base (n/a)")
    panel._emit_log(
        "[PICK][SELECT_EVT] "
        f"source=object_list name={name} "
        + (
            f"base=({float(base_obj[0]):.3f},{float(base_obj[1]):.3f},{float(base_obj[2]):.3f})"
            if base_obj is not None
            else "base=(n/a)"
        )
    )
    
    # ✅ Convertir a píxel usando table_xy_to_pixel (IGUAL A PANEL ONLY)
    px, py = -1, -1
    # Usar las dimensiones de la última imagen recibida del CameraView
    w = getattr(panel.camera_view, "_img_width", 0) if hasattr(panel, "camera_view") else 0
    h = getattr(panel.camera_view, "_img_height", 0) if hasattr(panel, "camera_view") else 0
    
    if not (panel._camera_stream_ok and panel._pose_info_ok and w > 0 and h > 0):
        panel._log("[PICK] Sin cámara/pose_info lista; omitiendo conversión a píxel")
    else:
        # Intentar convertir con coordenadas XYZ primero
        pix = world_xyz_to_pixel(x, y, z, w, h)
        if not pix:
            # Fallback a XY (igual a Panel Only)
            pix = table_xy_to_pixel(x, y, w, h)
        if pix:
            px, py = pix
            panel._log(f"[PICK] Conversión XYZ→píxel OK: ({px}, {py})")
        else:
            panel._log(f"[PICK] No se pudo convertir XYZ/XY→píxel, usando fallback (0,0)")
    
    state = get_object_state(name)
    geom_on_table = is_on_table((x, y, z))
    state_ok = bool(
        state
        and (
            state.logical_state == ObjectLogicalState.ON_TABLE
            or (state.logical_state == ObjectLogicalState.SELECTED and geom_on_table)
        )
    )
    if not state_ok:
        pos_txt = f"pos=({x:.3f},{y:.3f},{z:.3f})"
        details = f"state={state.logical_state.value}" if state else "state=unknown"
        from .panel_objects import table_geom_debug
        geom = table_geom_debug((x, y, z))
        panel._emit_log(f"[PICK] Bloqueado: {name} no está ON_TABLE ({details} {pos_txt} {geom})")
        panel._set_status("Objeto no está sobre la mesa", error=True)
        return
    panel._select_object(name, px, py, x, y, z, source="object_list")

def _update_objects(panel):
    """Actualizar lista de objetos usando ObjectListPanel (mismo estilo que panel principal)."""
    if not hasattr(panel, "obj_panel"):
        return

    objects = get_object_positions() if panel._gz_running else {}
    pickable_map = {}
    moveit_ready = panel._moveit_ready() and panel._controllers_ok and panel._tf_ready_state
    if objects:
        use_cache = panel._pickable_map_cache if panel._objects_settled else None
        if use_cache:
            for name, (x, y, _z) in objects.items():
                state = get_object_state(name)
                on_table_state = bool(
                    state
                    and state.logical_state in (ObjectLogicalState.ON_TABLE, ObjectLogicalState.SELECTED)
            )
                pickable_map[name] = bool(use_cache.get(name, False)) and on_table_state
        else:
            for name, (x, y, _z) in objects.items():
                state = get_object_state(name)
                on_table_state = bool(
                    state
                    and state.logical_state in (ObjectLogicalState.ON_TABLE, ObjectLogicalState.SELECTED)
            )
                pickable_map[name] = not object_out_of_reach(x, y) and on_table_state
    panel.obj_panel.update_objects(objects, pickable=pickable_map)

    sel_text = "Sel: -"
    if panel._selected_object and panel._selected_base:
        sel_text = f"Sel: {panel._selected_object}"
        if not moveit_ready:
            sel_text = f"{sel_text} · MoveIt no"
    panel.obj_panel.set_selected(panel._selected_object, sel_text)

def _push_history(panel, values: List[float], value: Optional[float], max_len: int = 30) -> None:
    if value is None or not math.isfinite(float(value)):
        return
    values.append(float(value))
    if len(values) > max_len:
        del values[0]

def _mean_history(panel, values: List[float]) -> Optional[float]:
    if not values:
        return None
    return float(sum(values) / max(len(values), 1))

def _update_fps_stats(panel, fps: float) -> None:
    if fps <= 0.0 or not math.isfinite(float(fps)):
        return
