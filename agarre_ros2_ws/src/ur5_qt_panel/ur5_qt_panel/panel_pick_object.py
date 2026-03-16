#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Pick object sequence helper for the panel."""
from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from typing import List, Optional

try:
    from trajectory_msgs.msg import JointTrajectory
except Exception:  # pragma: no cover - ROS not available in unit contexts
    JointTrajectory = None

from .panel_config import (
    BASKET_DROP,
    BASE_FRAME,
    GRIPPER_JOINT_NAMES,
    GRIPPER_TCP_Z_OFFSET,
    PICK_DEMO_DROP_Z_OFFSET,
    PICK_DEMO_GRASP_Z_OFFSET,
    WORLD_FRAME,
)
from .panel_robot_presets import (
    JOINT_BASKET_POSE_RAD,
    JOINT_GRASP_DOWN_POSE_RAD,
    JOINT_TABLE_POSE_RAD,
    JOINT_PICK_IMAGE_POSE_RAD,
    _make_pose_data,
)
from .panel_utils import get_object_positions, transform_point_to_frame, get_tf_helper
from .panel_objects import (
    ObjectLogicalState,
    get_object_state,
    get_object_states,
    mark_object_attached,
    mark_object_grasped,
    mark_object_released,
)
from .panel_readiness import pick_ui_status
from .panel_state import MoveItState


def run_pick_object(panel) -> None:
    """Ejecuta pick & place del objeto seleccionado hacia la cesta."""
    def _dbg(msg: str) -> None:
        if panel._debug_logs_enabled:
            panel._emit_log(msg)

    def _block(reason: str, *, status_text: Optional[str] = None, error: bool = False) -> None:
        state = getattr(getattr(panel, "_system_state", None), "value", "n/a")
        moveit_state = getattr(getattr(panel, "_moveit_state", None), "value", "n/a")
        moveit_reason = getattr(panel, "_moveit_state_reason", "") or "n/a"
        ctrl_ok = str(bool(getattr(panel, "_controllers_ok", False))).lower()
        ctrl_reason = getattr(panel, "_controllers_reason", "") or "n/a"
        safety_reason = getattr(panel, "_robot_test_last_failure", "") or "none"
        lock_active = str(bool(getattr(panel, "_pick_target_lock_active", False))).lower()
        lock_name = str(getattr(panel, "_pick_target_lock_name", "") or "none")
        lock_id = str(getattr(panel, "_pick_target_lock_id", "") or "n/a")
        panel._emit_log(
            f"[PICK_OBJ][BLOCK] reason={reason} state={state} gazebo={panel._gazebo_state()} "
            f"controllers={ctrl_ok}:{ctrl_reason} moveit={moveit_state}:{moveit_reason} "
            f"safety={safety_reason} lock_active={lock_active} lock_name={lock_name} lock_id={lock_id}"
        )
        if status_text:
            panel._ui_set_status(status_text, error=error)

    def _clear_stale_ui_selection(reason: str) -> None:
        """Clear stale UI/local selection when store has no SELECTED object."""
        prev_ui = str(getattr(panel, "_selected_object", "") or "").strip()
        prev_user = str(getattr(panel, "_selection_last_user_name", "") or "").strip()
        prev_ts = float(getattr(panel, "_selection_last_user_ts", 0.0) or 0.0)
        panel._selected_object = None
        panel._selected_px = None
        panel._selected_world = None
        panel._selection_timestamp = 0.0
        panel._selection_last_user_name = ""
        panel._selection_last_user_ts = 0.0
        try:
            panel.obj_panel.set_selected(None, "")
        except Exception:
            pass
        panel._emit_log(
            "[PICK][SELECT_RESET] "
            f"reason={reason} prev_ui={prev_ui or 'none'} prev_user={prev_user or 'none'} "
            f"prev_user_ts={prev_ts:.3f}"
        )

    panel._log_button("PICK Objeto")
    if not panel._require_ready_basic("PICK Objeto"):
        _block("ready_basic=false")
        return
    ok, reason = pick_ui_status(panel)
    if not ok:
        _block(reason, status_text=f"PICK en espera: {reason}", error=False)
        panel._emit_log(f"[PICK] Bloqueado: {reason}")
        return
    if not panel._moveit_required:
        _block(
            "moveit_disabled",
            status_text="MoveIt deshabilitado; bloqueando pick",
            error=True,
        )
        panel._emit_log("[PICK] Bloqueado: MoveIt deshabilitado")
        return
    if panel._moveit_state != MoveItState.READY:
        reason = panel._set_moveit_wait_status("pick")
        _block(f"moveit_not_ready:{reason}")
        panel._emit_log(f"[PICK] Bloqueado: MoveIt ({reason})")
        return
    ready, reason = panel._wait_for_controllers_ready(5.0)
    if not ready:
        _block(
            f"controllers_not_ready:{reason}",
            status_text="Controladores no listos; esperando",
            error=False,
        )
        panel._emit_log(f"[PICK] controladores no listos ({reason})")
        return

    ui_selected = str(getattr(panel, "_selected_object", "") or "").strip()
    ui_selected_ts = float(getattr(panel, "_selection_timestamp", 0.0) or 0.0)
    user_selected = str(getattr(panel, "_selection_last_user_name", "") or "").strip()
    user_selected_ts = float(getattr(panel, "_selection_last_user_ts", 0.0) or 0.0)
    selected_states = [
        (name, st)
        for name, st in get_object_states().items()
        if st.logical_state == ObjectLogicalState.SELECTED
    ]
    selected_states.sort(key=lambda item: float(item[1].last_update_ts), reverse=True)
    latest_store_name = selected_states[0][0] if selected_states else ""
    latest_store_ts = float(getattr(selected_states[0][1], "last_update_ts", 0.0) or 0.0) if selected_states else 0.0
    if len(selected_states) > 1:
        panel._emit_log(
            "[PICK_OBJ][TARGET] WARN multiple SELECTED states="
            + ",".join([f"{name}@{st.last_update_ts:.3f}" for name, st in selected_states[:4]])
        )
    panel._emit_log(
        "[PICK][SELECT_EFFECTIVE] "
        f"ui_selected={ui_selected or 'none'} ui_ts={ui_selected_ts:.3f} "
        f"store_selected={latest_store_name or 'none'} store_ts={latest_store_ts:.3f} "
        f"user_selected={user_selected or 'none'} user_ts={user_selected_ts:.3f}"
    )
    if ui_selected and not latest_store_name:
        _clear_stale_ui_selection("store_selected_none")
        _block(
            "selection_missing",
            status_text="PICK Objeto: selección no disponible en store, vuelve a seleccionar",
            error=True,
        )
        panel._emit_log(
            "[PICK_OBJ][ABORT] selection_missing_after_reset "
            f"ui_selected={ui_selected}"
        )
        return
    selection_max_age_sec = max(
        5.0,
        float(os.environ.get("PANEL_PICK_OBJECT_SELECTION_MAX_AGE_SEC", "600.0") or 600.0),
    )
    if not user_selected or user_selected_ts <= 0.0:
        _block(
            "no_user_selection",
            status_text="PICK Objeto: selecciona un objeto antes de ejecutar",
            error=True,
        )
        panel._emit_log("[PICK_OBJ][ABORT] no_user_selection")
        return
    selection_age = max(0.0, time.time() - user_selected_ts)
    if selection_age > selection_max_age_sec:
        _block(
            "no_user_selection_expired",
            status_text="PICK Objeto: selección expirada, vuelve a seleccionar",
            error=True,
        )
        panel._emit_log(
            f"[PICK_OBJ][ABORT] no_user_selection_expired name={user_selected} "
            f"age={selection_age:.1f}s max_age={selection_max_age_sec:.1f}s"
        )
        return
    if not ui_selected or not latest_store_name:
        _block(
            "selection_missing",
            status_text="PICK Objeto: selección no disponible en UI/store",
            error=True,
        )
        panel._emit_log(
            "[PICK_OBJ][ABORT] selection_missing "
            f"ui_selected={ui_selected or 'none'} store_selected={latest_store_name or 'none'}"
        )
        return
    if not (ui_selected == latest_store_name == user_selected):
        _block(
            "selection_mismatch",
            status_text="PICK Objeto: selección inconsistente (UI/store)",
            error=True,
        )
        panel._emit_log(
            "[PICK_OBJ][ABORT] selection_mismatch "
            f"ui_selected={ui_selected or 'none'} store_selected={latest_store_name or 'none'} "
            f"user_selected={user_selected or 'none'}"
        )
        return
    snapshot_name = user_selected
    selected_state = get_object_state(snapshot_name)
    if not selected_state:
        msg = f"Selección inválida en store: {snapshot_name} (state=none)"
        _block("selection_missing_in_store", status_text=msg, error=True)
        panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
        return
    if selected_state.logical_state != ObjectLogicalState.SELECTED:
        _block(
            "selection_not_selected",
            status_text=f"PICK Objeto: {snapshot_name} no está SELECTED",
            error=True,
        )
        panel._emit_log(
            "[PICK_OBJ][ABORT] selection_not_selected "
            f"name={snapshot_name} store_state={selected_state.logical_state.value}"
        )
        return
    panel._emit_log(
        "[PICK_OBJ][TARGET] "
        f"snapshot={snapshot_name or 'none'} store_selected={latest_store_name or 'none'} "
        f"ui_selected={ui_selected or 'none'} ui_ts={ui_selected_ts:.3f} store_ts={latest_store_ts:.3f}"
    )
    obj_name = snapshot_name
    selected_ts = max(
        ui_selected_ts,
        latest_store_ts,
        user_selected_ts,
        float(getattr(selected_state, "last_update_ts", 0.0) or 0.0),
    )
    panel._emit_log(f"[PICK_OBJ][OBJECT] using={obj_name} for planning+attach")
    _dbg(f"[PICK_OBJ_DEBUG] Inicio: obj_name={obj_name} selected_ts={selected_ts:.3f}")
    positions = get_object_positions()
    _dbg("[PICK_OBJ_DEBUG] get_object_positions OK")
    if not positions or obj_name not in positions:
        _block("object_not_found_in_pose_info", status_text="PICK Objeto: objeto no disponible", error=True)
        panel._emit_log(f"[PICK] Objeto no encontrado: {obj_name}")
        return
    # Fuente de verdad para planificacion MoveIt:
    # usar SIEMPRE pose_info live del objeto seleccionado.
    px, py, pz = positions[obj_name]
    obj_x, obj_y, obj_z = float(px), float(py), float(pz)
    pose_source = "pose_info/live"
    if selected_state is not None and getattr(selected_state, "position", None):
        sx, sy, sz = selected_state.position
        sx, sy, sz = float(sx), float(sy), float(sz)
        if abs(obj_x - sx) > 0.02 or abs(obj_y - sy) > 0.02 or abs(obj_z - sz) > 0.03:
            _block(
                "selection_pose_mismatch",
                status_text="PICK Objeto: pose seleccionada desfasada",
                error=True,
            )
            panel._emit_log(
                "[PICK_OBJ][ABORT] selection_pose_mismatch "
                f"name={obj_name} state=({sx:.3f},{sy:.3f},{sz:.3f}) "
                f"pose_info=({obj_x:.3f},{obj_y:.3f},{obj_z:.3f})"
            )
            return
    _dbg("[PICK_OBJ_DEBUG] Objeto en positions OK")
    state = get_object_state(obj_name)
    _dbg(f"[PICK_OBJ_DEBUG] get_object_state OK: state={state}")
    if state and state.logical_state not in (
        ObjectLogicalState.ON_TABLE,
        ObjectLogicalState.SELECTED,
    ):
        _block(
            f"object_state_invalid:{state.logical_state}",
            status_text="PICK Objeto: objeto no está en mesa",
            error=True,
        )
        panel._emit_log(
            f"[PICK_OBJ] PRE-CHECK FALLÓ: {obj_name} en estado {state.logical_state} (requiere ON_TABLE/SELECTED)"
        )
        return

    _dbg("[PICK_OBJ_DEBUG] Pre-check passed")
    # FASE 3: Pre-check - Validar estado inicial
    panel._emit_log(
        f"[PICK_OBJ] PRE-CHECK: {obj_name} estado = {state.logical_state if state else 'DESCONOCIDO'} ✓"
    )
    if panel._robot_test_last_failure:
        panel._emit_log(
            f"[PICK_OBJ] INFO: TEST_FAILED previo='{panel._robot_test_last_failure}' no bloquea contexto PICK_OBJECT"
        )

    table_top = panel._resolve_table_top_z()
    _dbg(f"[PICK_OBJ_DEBUG] table_top={table_top:.3f}")
    panel._emit_log(
        f"[PICK_OBJ][TARGET] name={obj_name} selected_ts={selected_ts:.3f} "
        f"pose_source={pose_source}"
    )
    world_frame = WORLD_FRAME or "world"
    if os.environ.get("DEBUG_PICK_OBJ", "0") == "1":
        pose_topic = "/desired_grasp"
        result_topic = "/desired_grasp/result"
        pose_subs = panel.ros_worker.topic_subscriber_count(pose_topic)
        result_pubs = panel.ros_worker.topic_publisher_count(result_topic)
        result_subs = panel.ros_worker.topic_subscriber_count(result_topic)
        result_nodes = ",".join(panel.ros_worker.publisher_nodes_by_topic(result_topic)) or "n/a"
        panel._emit_log(
            "[PICK_OBJ][DIAG] "
            f"selected={obj_name} selected_ts={selected_ts:.3f} source={pose_source} "
            f"group=manipulator ee_tf={panel._ee_frame_effective or 'rg2_tcp'} "
            f"base_hint={panel._base_frame_effective or BASE_FRAME or 'base_link'} "
            f"pose_subs={pose_subs} result_pubs={result_pubs} result_subs={result_subs} "
            f"result_nodes={result_nodes}"
        )
    _dbg(f"[PICK_OBJ_DEBUG] obj pos: ({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})")
    sdf = panel._sdf_model_cache.get(obj_name, {})
    geom_type = sdf.get("type") or "desconocido"
    size = sdf.get("size")
    radius = sdf.get("radius")
    length = sdf.get("length")
    height = None
    if geom_type == "box" and size:
        height = float(size[2])
    elif geom_type == "cylinder" and length:
        height = float(length)
    elif geom_type == "sphere" and radius:
        height = float(radius) * 2.0
    if height:
        obj_z = table_top + (height / 2.0)
    pickable, why = panel._is_pickable(obj_name, obj_x, obj_y, obj_z, table_top, positions)
    if not pickable:
        motivo = f" ({why})" if why else ""
        _block(
            f"object_not_pickable:{why or 'unknown'}",
            status_text=f"PICK Objeto: no pickable{motivo}",
            error=True,
        )
        panel._emit_log(f"[PICK] Bloqueado: {obj_name} no pickable{motivo}")
        return

    obj_info = panel._selection_to_base((obj_x, obj_y, obj_z), world_frame)
    if not obj_info:
        _block("tf_object_to_base_missing", status_text="PICK Objeto: TF no disponible", error=True)
        panel._emit_log("[PICK] Bloqueado: no se pudo transformar objeto a base")
        return
    base_frame = obj_info["frame"]
    bx, by, bz = obj_info["coords"]
    obj_base_stamp_ns = 0
    try:
        _obj_tf_stamp = obj_info["transform"].header.stamp  # type: ignore[index]
        obj_base_stamp_ns = int(_obj_tf_stamp.sec) * 1_000_000_000 + int(_obj_tf_stamp.nanosec)
    except Exception:
        obj_base_stamp_ns = 0
    basket_coords, _ = transform_point_to_frame(
        BASKET_DROP,
        base_frame,
        source_frame=world_frame,
    )
    if not basket_coords:
        _block("tf_basket_to_base_missing", status_text="PICK Objeto: cesta sin TF", error=True)
        panel._emit_log("[PICK] Bloqueado: no se pudo transformar cesta a base")
        return
    basket_x, basket_y, basket_z = basket_coords

    _dbg(
        f"[PICK_OBJ_DEBUG] Basket coords: bx={bx:.3f}, by={by:.3f}, bz={bz:.3f} basket=({basket_x:.3f}, {basket_y:.3f}, {basket_z:.3f})"
    )

    # Alturas del pick en base_link para evitar empuje con la pinza abierta.
    # z_top se calcula sobre la geometria del objeto actual, y los margenes se
    # ajustan via entorno sin tocar logica de PICK_MESA.
    obj_height = float(height or 0.0)
    obj_top_z_base = bz + (obj_height * 0.5 if obj_height > 0.0 else 0.0)
    table_top_base = None
    table_base_coords, _ = transform_point_to_frame(
        (obj_x, obj_y, table_top),
        base_frame,
        source_frame=world_frame,
    )
    if table_base_coords:
        table_top_base = float(table_base_coords[2])
    else:
        table_top_base = bz - (obj_height * 0.5 if obj_height > 0.0 else 0.0)
    try:
        z_clear = float(os.environ.get("PANEL_PICK_OBJECT_APPROACH_CLEARANCE_M", "0.20"))
    except Exception:
        z_clear = 0.20
    try:
        z_min_approach_clearance = float(
            os.environ.get("PANEL_PICK_OBJECT_MIN_APPROACH_CLEARANCE_M", "0.30")
        )
    except Exception:
        z_min_approach_clearance = 0.30
    z_min_approach_clearance = max(0.0, z_min_approach_clearance)
    try:
        z_pre_margin = float(os.environ.get("PANEL_PICK_OBJECT_PRE_MARGIN_M", "0.04"))
    except Exception:
        z_pre_margin = 0.04
    try:
        z_min_pre_clearance = float(
            os.environ.get("PANEL_PICK_OBJECT_MIN_PRE_CLEARANCE_M", "0.15")
        )
    except Exception:
        z_min_pre_clearance = 0.15
    z_min_pre_clearance = max(0.0, z_min_pre_clearance)
    # Legacy offset kept for compatibility; new contact target uses contact_down_z.
    try:
        z_insert = float(os.environ.get("PANEL_PICK_OBJECT_INSERT_M", "0.015"))
    except Exception:
        z_insert = 0.015
    try:
        contact_down_z = float(
            os.environ.get(
                "PANEL_PICK_OBJECT_CONTACT_DOWN_Z_M",
                os.environ.get(
                    "PANEL_PICK_OBJECT_EXTRA_DOWN_Z",
                    str(max(0.060, float(PICK_DEMO_GRASP_Z_OFFSET))),
                ),
            )
        )
    except Exception:
        contact_down_z = 0.060
    if contact_down_z < 0.0:
        contact_down_z = 0.0
    try:
        z_table_margin = float(os.environ.get("PANEL_PICK_OBJECT_TABLE_MARGIN_M", "0.015"))
    except Exception:
        z_table_margin = 0.015
    try:
        z_lift_clearance = float(os.environ.get("PANEL_PICK_OBJECT_LIFT_CLEARANCE_M", "0.20"))
    except Exception:
        z_lift_clearance = 0.20
    try:
        z_transport_clearance = float(
            os.environ.get("PANEL_PICK_OBJECT_TRANSPORT_CLEARANCE_M", "0.20")
        )
    except Exception:
        z_transport_clearance = 0.20

    approach_clearance = max(z_clear, z_min_approach_clearance)
    z_approach = obj_top_z_base + approach_clearance
    pre_clearance = max(z_pre_margin, z_min_pre_clearance)
    z_pre = obj_top_z_base + pre_clearance
    legacy_grasp_z = max(table_top_base + z_table_margin, obj_top_z_base - z_insert)
    contact_target_z = obj_top_z_base - float(GRIPPER_TCP_Z_OFFSET) - contact_down_z
    z_grasp = max(table_top_base + z_table_margin, contact_target_z)
    z_lift = max(z_grasp + z_lift_clearance, z_approach)
    z_drop = basket_z + PICK_DEMO_DROP_Z_OFFSET
    z_transport = max(z_drop + z_transport_clearance, z_lift)

    pose_frame = base_frame
    approach_pose = (bx, by, z_approach)
    pre_grasp_pose = (bx, by, z_pre)
    grasp_down_pose = (bx, by, z_grasp)
    lift_pose = (bx, by, z_lift)
    transport_pose = (basket_x, basket_y, z_transport)
    drop_pose = (basket_x, basket_y, z_drop)

    _dbg(
        f"[PICK_OBJ_DEBUG] BASE: APPROACH=({approach_pose[0]:.3f}, {approach_pose[1]:.3f}, {approach_pose[2]:.3f}), "
        f"PRE_GRASP=({pre_grasp_pose[0]:.3f}, {pre_grasp_pose[1]:.3f}, {pre_grasp_pose[2]:.3f}), "
        f"GRASP_DOWN=({grasp_down_pose[0]:.3f}, {grasp_down_pose[1]:.3f}, {grasp_down_pose[2]:.3f}) "
        f"LIFT=({lift_pose[0]:.3f}, {lift_pose[1]:.3f}, {lift_pose[2]:.3f}) "
        f"TRANSPORT=({transport_pose[0]:.3f}, {transport_pose[1]:.3f}, {transport_pose[2]:.3f}) "
        f"DROP=({drop_pose[0]:.3f}, {drop_pose[1]:.3f}, {drop_pose[2]:.3f}) "
        f"(obj_height={obj_height:.3f} table_top_base={table_top_base:.3f})"
    )
    _dbg(
        f"[PICK_OBJ_DEBUG] height={height} table_top_world={table_top:.3f}"
    )

    # Optional micro-descent sequence before GRASP_DOWN final.
    try:
        grasp_micro_step_m = float(os.environ.get("PANEL_PICK_OBJECT_GRASP_MICRO_STEP_M", "0.012"))
    except Exception:
        grasp_micro_step_m = 0.012
    try:
        grasp_micro_steps_max = int(os.environ.get("PANEL_PICK_OBJECT_GRASP_MICRO_STEPS_MAX", "4"))
    except Exception:
        grasp_micro_steps_max = 4
    grasp_micro_steps_max = max(0, min(grasp_micro_steps_max, 12))
    grasp_micro_poses: List[tuple] = []
    if grasp_micro_steps_max > 0 and grasp_down_pose[2] < pre_grasp_pose[2]:
        z_cur = float(pre_grasp_pose[2])
        idx = 0
        while z_cur - grasp_micro_step_m > float(grasp_down_pose[2]) and idx < grasp_micro_steps_max:
            z_cur -= grasp_micro_step_m
            idx += 1
            grasp_micro_poses.append((bx, by, z_cur))

    sequence = [
        ("APPROACH", _make_pose_data(approach_pose, frame=pose_frame), 0.4),
        ("PRE_GRASP", _make_pose_data(pre_grasp_pose, frame=pose_frame), 0.5),
        ("GRASP_DOWN", _make_pose_data(grasp_down_pose, frame=pose_frame), 0.7),
        ("LIFT", _make_pose_data(lift_pose, frame=pose_frame), 0.5),
        ("TRANSPORT", _make_pose_data(transport_pose, frame=pose_frame), 0.6),
        ("DROP", _make_pose_data(drop_pose, frame=pose_frame), 0.8),
    ]

    _dbg("[PICK_OBJ_DEBUG] Sequence construida correctamente")
    target_lock_id = uuid.uuid4().hex[:12]
    setattr(panel, "_pick_target_lock_active", True)
    setattr(panel, "_pick_target_lock_name", obj_name)
    setattr(panel, "_pick_target_lock_ts", selected_ts)
    setattr(panel, "_pick_target_lock_id", target_lock_id)
    setattr(panel, "_pick_target_lock_source", pose_source)
    setattr(panel, "_pick_target_lock_reason", "pick_object_sequence")
    setattr(panel, "_pick_target_lock_world", (float(obj_x), float(obj_y), float(obj_z)))
    setattr(panel, "_pick_target_lock_base", (float(bx), float(by), float(bz)))
    panel._emit_log(
        f"[PICK_OBJ][TARGET_LOCK] activate lock_id={target_lock_id} "
        f"name={obj_name} ts={selected_ts:.3f} source={pose_source} "
        f"base=({bx:.3f},{by:.3f},{bz:.3f}) frame={base_frame}"
    )

    panel._emit_log("[PICK_OBJ] === SECUENCIA INICIADA ===")
    panel._emit_log(
        "[PICK_OBJ] Objeto seleccionado: "
        f"{obj_name} geom_type={geom_type} height={height}"
    )
    panel._emit_log(
        f"[PICK_OBJ][MOVEIT][PLAN] object_pose_source=pose_info frame_id={base_frame} selected={obj_name}"
    )
    panel._emit_log(f"[PICK_OBJ] Posición BASE_LINK: ({bx:.3f}, {by:.3f}, {bz:.3f})")
    panel._emit_log(
        f"[PICK_OBJ][Z] top={obj_top_z_base:.3f} approach={z_approach:.3f} "
        f"pre={z_pre:.3f} grasp={z_grasp:.3f} lift={z_lift:.3f} "
        f"transport={z_transport:.3f} drop={z_drop:.3f} table_top={table_top_base:.3f} "
        f"legacy_grasp={legacy_grasp_z:.3f} contact_target={contact_target_z:.3f}"
    )
    panel._emit_log(
        f"[PICK_OBJ][GRASP_CFG] contact_down_z={contact_down_z:.3f} "
        f"tcp_z_offset={float(GRIPPER_TCP_Z_OFFSET):.3f} "
        f"table_margin={z_table_margin:.3f} "
        f"micro_step={grasp_micro_step_m:.3f} micro_steps={len(grasp_micro_poses)}"
    )
    panel._emit_log(
        f"[PICK_OBJ] Alturas de poses (FRAME={pose_frame}): "
        f"APPROACH={approach_pose[2]:.3f}m, PRE_GRASP={pre_grasp_pose[2]:.3f}m, "
        f"GRASP={grasp_down_pose[2]:.3f}m, LIFT={lift_pose[2]:.3f}m, "
        f"TRANSPORT={transport_pose[2]:.3f}m, DROP={drop_pose[2]:.3f}m"
    )
    if not (approach_pose[2] > pre_grasp_pose[2] > grasp_down_pose[2]):
        _block(
            "approach_not_descending",
            status_text="Pick objeto: altura incorrecta (no desciende)",
            error=True,
        )
        panel._emit_log(
            "[PICK_OBJ] ✗ Pre-grasp y grasp_down incoherentes (no hay descenso). Abortando."
        )
        return
    panel._emit_log(
        f"[PICK_OBJ][TF][CHECK] approach_z approach={approach_pose[2]:.3f} "
        f"pre={pre_grasp_pose[2]:.3f} grasp={grasp_down_pose[2]:.3f} descending=true"
    )
    panel._set_status("Pick objeto: ejecutando secuencia…")
    panel._set_motion_lock(True)

    def worker():
        moveit_result_topic = "/desired_grasp/result"
        moveit_pose_topic = "/desired_grasp"
        moveit_hb_topic = "/ur5_moveit_bridge/heartbeat"
        measured_ee_frame = panel._ee_frame_effective or "rg2_tcp"
        target_snapshot_name = obj_name
        target_snapshot_ts = selected_ts
        target_lock_id_local = str(getattr(panel, "_pick_target_lock_id", "") or target_lock_id)
        selection_change_logged = False
        moveit_exec_window = {"active": False}
        gripper_open_required = {"active": True}
        moveit_monitor_topic = "/joint_trajectory_controller/joint_trajectory"
        moveit_monitor_sub = None
        moveit_monitor_total = 0
        moveit_monitor_manual = 0
        moveit_monitor_conflict_ts = 0.0
        live_tf_buffer = None
        live_tf_listener = None

        def _norm_frame(frame_name: str) -> str:
            return (frame_name or "").strip().lstrip("/")

        def _log_selection_changed_during_pick() -> None:
            nonlocal selection_change_logged
            if selection_change_logged:
                return
            current_name = str(getattr(panel, "_selected_object", "") or "").strip()
            current_ts = float(getattr(panel, "_selection_timestamp", 0.0) or 0.0)
            if current_name and current_name != target_snapshot_name and current_ts > target_snapshot_ts:
                panel._emit_log(
                    "[PICK_OBJ][TARGET] selection_changed_during_pick "
                    f"lock_id={target_lock_id_local} "
                    f"snapshot={target_snapshot_name}@{target_snapshot_ts:.3f} "
                    f"current={current_name}@{current_ts:.3f} "
                    f"store_selected={str(getattr(panel, '_selected_object', '') or 'none')}"
                )
                selection_change_logged = True

        def _wait_manual_idle_before_moveit(label: str) -> None:
            deadline = time.monotonic() + 2.5
            while bool(getattr(panel, "_manual_inflight", False)) and time.monotonic() < deadline:
                time.sleep(0.05)
            if bool(getattr(panel, "_manual_inflight", False)):
                raise RuntimeError(f"manual_inflight_timeout_before_{label}")
            time.sleep(0.5)

        def _on_moveit_exec_traj(_msg) -> None:
            nonlocal moveit_monitor_total, moveit_monitor_manual, moveit_monitor_conflict_ts
            if not moveit_exec_window["active"]:
                return
            moveit_monitor_total += 1
            if bool(getattr(panel, "_manual_inflight", False)):
                moveit_monitor_manual += 1
                now = time.time()
                if (now - moveit_monitor_conflict_ts) >= 0.3:
                    panel._emit_log(
                        "[PICK_OBJ][MOVEIT][EXEC] manual_conflict: published while moveit_executing"
                    )
                    moveit_monitor_conflict_ts = now

        def _ensure_moveit_exec_monitor() -> None:
            nonlocal moveit_monitor_sub
            if moveit_monitor_sub is not None or JointTrajectory is None:
                return
            try:
                with panel.ros_worker._lock:
                    node = getattr(panel.ros_worker, "_node", None)
                    if node is None:
                        return
                    moveit_monitor_sub = node.create_subscription(
                        JointTrajectory,
                        moveit_monitor_topic,
                        _on_moveit_exec_traj,
                        10,
                    )
            except Exception as exc:
                panel._emit_log(
                    f"[PICK_OBJ][MOVEIT][EXEC] monitor_unavailable topic={moveit_monitor_topic} err={exc}"
                )

        def _clear_moveit_exec_monitor() -> None:
            nonlocal moveit_monitor_sub
            if moveit_monitor_sub is None:
                return
            try:
                with panel.ros_worker._lock:
                    node = getattr(panel.ros_worker, "_node", None)
                    if node is not None:
                        node.destroy_subscription(moveit_monitor_sub)
            except Exception:
                pass
            finally:
                moveit_monitor_sub = None

        def _read_tcp_in_frame(
            target_frame: str,
            ee_frame: str,
            *,
            timeout_sec: float = 0.3,
        ) -> tuple[Optional[tuple], Optional[object]]:
            nonlocal live_tf_buffer, live_tf_listener
            # Prefer a live TF buffer bound to ros_worker node. The global helper
            # can lag several seconds under heavy panel load and cause false
            # negatives in PRE_GRASP/GRASP TF checks.
            try:
                if panel._ros_worker_started and panel.ros_worker.node_ready():
                    with panel.ros_worker._lock:
                        ros_node = getattr(panel.ros_worker, "_node", None)
                    if ros_node is not None:
                        if live_tf_buffer is None:
                            from rclpy.duration import Duration as _RosDuration
                            from rclpy.time import Time as _RosTime
                            from tf2_ros import Buffer as _TfBuffer, TransformListener as _TfListener
                            live_tf_buffer = _TfBuffer(cache_time=_RosDuration(seconds=15.0))
                            live_tf_listener = _TfListener(live_tf_buffer, ros_node, spin_thread=False)
                        from rclpy.duration import Duration as _RosDuration
                        from rclpy.time import Time as _RosTime
                        tf_msg = live_tf_buffer.lookup_transform(
                            target_frame,
                            ee_frame,
                            _RosTime(),
                            timeout=_RosDuration(seconds=max(0.05, float(timeout_sec))),
                        )
                        tr = tf_msg.transform.translation
                        return (float(tr.x), float(tr.y), float(tr.z)), tf_msg
            except Exception:
                pass
            # Disabled by default: fallback helper can return stale/mismatched TF
            # during high-load phases and trigger false mismatch aborts.
            allow_helper_fallback = str(
                os.environ.get("PANEL_PICK_OBJECT_ALLOW_TF_HELPER_FALLBACK", "0")
            ).strip().lower() in ("1", "true", "yes", "on")
            if not allow_helper_fallback:
                return None, None
            return transform_point_to_frame(
                (0.0, 0.0, 0.0),
                target_frame,
                source_frame=ee_frame,
                timeout_sec=timeout_sec,
            )

        def _tf_stamp_ns(tf_msg: Optional[object]) -> int:
            try:
                stamp = tf_msg.header.stamp  # type: ignore[attr-defined]
                return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            except Exception:
                return 0

        def _ros_clock_now_ns() -> int:
            # Use live ROS clock first; cached /clock is only a fallback.
            # This avoids stale cache values inflating TF age diagnostics.
            def _sanitize_ros_ns(raw: object) -> int:
                try:
                    value = int(raw or 0)
                except Exception:
                    return 0
                if value <= 0:
                    return 0
                # Filter out wall-time epoch values in sim-time checks.
                if value > 1_000_000_000_000_000:
                    return 0
                return value

            try:
                if panel._ros_worker_started and panel.ros_worker.node_ready():
                    with panel.ros_worker._lock:
                        ros_node = getattr(panel.ros_worker, "_node", None)
                        cached_clock_ns = _sanitize_ros_ns(
                            getattr(panel.ros_worker, "_last_clock_stamp_ns", 0)
                        )
                    if ros_node is not None:
                        now_ns = _sanitize_ros_ns(ros_node.get_clock().now().nanoseconds)
                        if now_ns > 0:
                            return now_ns
                    if cached_clock_ns > 0:
                        return cached_clock_ns
            except Exception:
                pass
            try:
                helper = get_tf_helper()
                node = getattr(helper, "_node", None) if helper is not None else None
                if node is not None:
                    now_ns = _sanitize_ros_ns(node.get_clock().now().nanoseconds)
                    if now_ns > 0:
                        return now_ns
            except Exception:
                pass
            return 0

        def _ensure_moveit_bridge_path(topic: str) -> bool:
            bridge_recovered = False
            def _run_ui_sync(fn, timeout_sec: float = 2.5) -> bool:
                done = threading.Event()
                def _wrapped() -> None:
                    try:
                        fn()
                    finally:
                        done.set()
                panel.signal_run_ui.emit(_wrapped)
                return done.wait(timeout=max(0.2, timeout_sec))

            def _restart_moveit_bridge_and_wait() -> None:
                panel._emit_log(
                    "[PICK_OBJ][MOVEIT][PLAN] bridge_recover restarting "
                    "node=ur5_moveit_bridge pose_topic=/desired_grasp result_topic=/desired_grasp/result"
                )
                _run_ui_sync(lambda: panel._stop_moveit_bridge(), timeout_sec=2.5)
                time.sleep(0.2)
                _run_ui_sync(lambda: panel._start_moveit_bridge(), timeout_sec=3.0)
                deadline = time.time() + 6.0
                while time.time() < deadline:
                    if not panel._ros_worker_started:
                        panel._ensure_ros_worker_started()
                    if panel._ros_worker_started and panel.ros_worker.node_ready():
                        pose_subs = panel.ros_worker.topic_subscriber_count(topic)
                        result_pubs = panel.ros_worker.topic_publisher_count(moveit_result_topic)
                        panel._emit_log(
                            "[PICK_OBJ][MOVEIT][PLAN] bridge_recover_check "
                            f"pose_subs={pose_subs} result_pubs={result_pubs}"
                        )
                        if pose_subs > 0 and result_pubs > 0:
                            return
                    time.sleep(0.4)
                raise RuntimeError("MoveItBridge no recuperado tras reinicio (pose_subs/result_pubs)")

            if not panel._ros_worker_started:
                panel._ensure_ros_worker_started()
            if not panel.ros_worker.node_ready():
                raise RuntimeError("ROS node no listo para MoveIt bridge")
            panel.ros_worker.subscribe_moveit_bridge_heartbeat(moveit_hb_topic)
            subs = panel.ros_worker.topic_subscriber_count(topic)
            result_pubs = panel.ros_worker.topic_publisher_count(moveit_result_topic)
            result_subs = panel.ros_worker.topic_subscriber_count(moveit_result_topic)
            hb_age = panel.ros_worker.moveit_bridge_heartbeat_age()
            hb_recent = panel.ros_worker.has_recent_moveit_bridge_heartbeat(1.2)
            hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
            panel._emit_log(
                f"[PICK_OBJ][MOVEIT][PLAN] topics pose_subs={subs} result_pubs={result_pubs} result_subs={result_subs} "
                f"pose_topic={topic} result_topic={moveit_result_topic} "
                f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
            )
            if subs <= 0 or result_pubs <= 0:
                panel._emit_log(
                    "[PICK_OBJ][MOVEIT][PLAN] bridge_path_missing "
                    f"pose_subs={subs} result_pubs={result_pubs}; attempting_recover=true"
                )
                _restart_moveit_bridge_and_wait()
                bridge_recovered = True
                subs = panel.ros_worker.topic_subscriber_count(topic)
                result_pubs = panel.ros_worker.topic_publisher_count(moveit_result_topic)
                result_subs = panel.ros_worker.topic_subscriber_count(moveit_result_topic)
                hb_age = panel.ros_worker.moveit_bridge_heartbeat_age()
                hb_recent = panel.ros_worker.has_recent_moveit_bridge_heartbeat(1.2)
                hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
                panel._emit_log(
                    f"[PICK_OBJ][MOVEIT][PLAN] topics_after_recover pose_subs={subs} result_pubs={result_pubs} "
                    f"result_subs={result_subs} pose_topic={topic} result_topic={moveit_result_topic} "
                    f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
                )
                if subs <= 0:
                    msg = f"MoveItBridge NO conectado: {topic} sin subscriptores"
                    panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
                    raise RuntimeError(msg)
            if not panel.ros_worker.subscribe_moveit_result(moveit_result_topic):
                msg = f"No se pudo suscribir a {moveit_result_topic}"
                panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
                raise RuntimeError(msg)
            result_pubs = panel.ros_worker.topic_publisher_count(moveit_result_topic)
            result_subs = panel.ros_worker.topic_subscriber_count(moveit_result_topic)
            panel._emit_log(
                f"[PICK_OBJ][MOVEIT][PLAN] result_path result_pubs={result_pubs} result_subs={result_subs} "
                f"topic={moveit_result_topic}"
            )
            if result_pubs <= 0:
                msg = f"MoveIt bridge sin publisher en {moveit_result_topic}"
                panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
                raise RuntimeError(msg)
            return bridge_recovered

        def _wait_moveit_result(label: str, since_wall: float, *, timeout_total: float = 12.0) -> dict:
            timeout_total = max(0.5, float(timeout_total))
            started = time.time()
            raw = ""
            ok = False
            last_diag = 0.0
            cursor_wall = since_wall
            cursor_seq = int(getattr(_wait_moveit_result, "_since_seq", -1) or -1)
            lost_pub_since = None
            # Contract matching: accept ONLY the expected request_id for this publish.
            expected_request_id = int(getattr(_wait_moveit_result, "_expected_request_id", -1) or -1)
            expected_stamp_ns = int(getattr(_wait_moveit_result, "_expected_stamp_ns", 0) or 0)
            expected_request_uuid = str(getattr(_wait_moveit_result, "_expected_request_uuid", "") or "")
            panel_request_id = int(getattr(_wait_moveit_result, "_panel_request_id", -1) or -1)
            while (time.time() - started) < timeout_total:
                wait_chunk = min(1.0, max(0.2, timeout_total - (time.time() - started)))
                ok, raw, _wall, _seq = panel.ros_worker.wait_for_moveit_result(
                    since_wall=cursor_wall,
                    since_seq=cursor_seq,
                    timeout_sec=wait_chunk,
                )
                if ok:
                    try:
                        data = json.loads(raw)
                    except Exception:
                        cursor_wall = max(cursor_wall, _wall)
                        cursor_seq = max(cursor_seq, int(_seq))
                        continue
                    req_id = int(data.get("request_id", -1) or -1)
                    got_stamp_ns = int(data.get("target_stamp_ns", 0) or 0)
                    got_uuid = str(data.get("request_uuid", "") or "")
                    panel._emit_log(
                        f"[PANEL][RESULT_RX] label={label} got_request_id={req_id} "
                        f"expected_request_id={expected_request_id} got_stamp={got_stamp_ns} "
                        f"expected_stamp={expected_stamp_ns} got_uuid={got_uuid or 'n/a'} "
                        f"expected_uuid={expected_request_uuid or 'n/a'}"
                    )
                    if expected_request_id >= 0 and req_id != expected_request_id:
                        panel._emit_log(
                            f"[PICK_OBJ][MOVEIT][RESULT] {label} ignore_unmatched request_id={req_id} "
                            f"expected_request_id={expected_request_id} target_stamp_ns={got_stamp_ns}"
                        )
                        cursor_wall = max(cursor_wall, _wall)
                        cursor_seq = max(cursor_seq, int(_seq))
                        continue
                    if expected_request_uuid and got_uuid != expected_request_uuid:
                        panel._emit_log(
                            f"[PICK_OBJ][RESULT_STALE] {label} ignore_unmatched_uuid got_uuid={got_uuid or 'n/a'} "
                            f"expected_uuid={expected_request_uuid} request_id={req_id}"
                        )
                        cursor_wall = max(cursor_wall, _wall)
                        cursor_seq = max(cursor_seq, int(_seq))
                        continue
                    raw = json.dumps(data, ensure_ascii=True)
                    break
                now = time.time()
                if (now - last_diag) >= 2.0:
                    elapsed = now - started
                    result_pubs = panel.ros_worker.topic_publisher_count(moveit_result_topic)
                    result_subs = panel.ros_worker.topic_subscriber_count(moveit_result_topic)
                    bridge_alive = bool(panel._proc_alive(getattr(panel, "moveit_bridge_proc", None)))
                    hb_age = panel.ros_worker.moveit_bridge_heartbeat_age()
                    hb_recent = panel.ros_worker.has_recent_moveit_bridge_heartbeat(1.2)
                    hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
                    panel._emit_log(
                        f"[PICK_OBJ][MOVEIT][EXEC] {label} still_waiting elapsed={elapsed:.1f}s "
                        f"timeout={timeout_total:.1f}s result_pubs={result_pubs} result_subs={result_subs} "
                        f"bridge_alive={str(bridge_alive).lower()} "
                        f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
                    )
                    last_diag = now
                    if result_pubs <= 0:
                        if lost_pub_since is None:
                            lost_pub_since = now
                        lost_age = now - float(lost_pub_since)
                        # Heartbeat can be absent/stale in some sessions; do not hard-fail
                        # solely on hb_age. Require bridge process down before early abort.
                        if (not bridge_alive) and lost_age >= 1.5:
                            raise RuntimeError(
                                f"lost_result_publisher:{moveit_result_topic}:{label}:"
                                f"lost_age={lost_age:.1f}s bridge_alive={str(bridge_alive).lower()} "
                                f"hb_recent={str(bool(hb_recent)).lower()}"
                            )
                    else:
                        lost_pub_since = None
            if not ok:
                elapsed = time.time() - started
                result_pubs = panel.ros_worker.topic_publisher_count(moveit_result_topic)
                result_subs = panel.ros_worker.topic_subscriber_count(moveit_result_topic)
                lost_age_txt = (
                    "n/a"
                    if lost_pub_since is None
                    else f"{max(0.0, time.time() - float(lost_pub_since)):.1f}s"
                )
                raise RuntimeError(
                    f"MoveIt bridge sin resultado en {moveit_result_topic} para {label} "
                        f"(elapsed={elapsed:.1f}s pubs={result_pubs} subs={result_subs} "
                        f"lost_pub_age={lost_age_txt} since_seq={cursor_seq} "
                        f"panel_request_id={panel_request_id} expected_request_id={expected_request_id} "
                        f"expected_stamp_ns={expected_stamp_ns} expected_uuid={expected_request_uuid or 'n/a'})"
                    )
            try:
                data = json.loads(raw)
            except Exception as exc:
                raise RuntimeError(f"resultado MoveIt inválido ({exc})") from exc
            plan_ok = bool(data.get("plan_ok", False))
            exec_ok = bool(data.get("exec_ok", False))
            success = bool(data.get("success", False))
            message = str(data.get("message") or "")
            ee_link_moveit = str(data.get("ee_link") or "")
            req_id = int(data.get("request_id", -1) or -1)
            stamp_ns = int(data.get("target_stamp_ns", 0) or 0)
            panel._emit_log(
                f"[PICK_OBJ][MOVEIT][PLAN] {label} plan_ok={str(plan_ok).lower()} "
                f"ee_link={ee_link_moveit or 'n/a'} msg={message or 'n/a'}"
            )
            panel._emit_log(
                f"[PICK_OBJ][MOVEIT][EXEC] {label} exec_ok={str(exec_ok).lower()} "
                f"success={str(success).lower()} result_topic={moveit_result_topic}"
            )
            panel._emit_log(
                f"[PICK_OBJ][MOVEIT][RESULT] {label} request_id={req_id} target_stamp_ns={stamp_ns} "
                f"success={str(success).lower()} plan_ok={str(plan_ok).lower()} "
                f"exec_ok={str(exec_ok).lower()} msg={message or 'n/a'}"
            )
            return data

        def _tf_distance_check(
            *,
            label: str,
            pose_data: dict,
            tol_m: float,
            ee_frame: str,
        ) -> dict:
            frame_id = str(pose_data.get("frame", BASE_FRAME or "base_link"))
            position = pose_data.get("position", (0.0, 0.0, 0.0))
            target = (float(position[0]), float(position[1]), float(position[2]))
            timeout_pos_reach = max(
                2.0, float(getattr(panel, "_pick_tf_reach_timeout_sec", 8.0) or 8.0)
            )
            timeout_tf_fresh = 2.0
            max_tf_age_sec = 0.80
            deadline = time.time() + timeout_pos_reach
            fresh_deadline = time.time() + timeout_tf_fresh
            wait_log_ts = 0.0
            last_diag: Optional[dict] = None
            best_dist = float("inf")
            best_dist_ts = 0.0
            saw_fresh = False
            while time.time() <= deadline:
                tcp, tf_msg = _read_tcp_in_frame(frame_id, ee_frame)
                if tcp is None:
                    time.sleep(0.08)
                    continue
                dx = target[0] - tcp[0]
                dy = target[1] - tcp[1]
                dz = target[2] - tcp[2]
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                tf_stamp = "n/a"
                tf_stamp_ns = 0
                try:
                    stamp = tf_msg.header.stamp  # type: ignore[attr-defined]
                    tf_stamp = f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"
                    tf_stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                except Exception:
                    tf_stamp = "n/a"
                ros_now_ns = 0
                ros_now_txt = "n/a"
                ros_now_ns = _ros_clock_now_ns()
                if ros_now_ns > 0:
                    ros_now_txt = f"{ros_now_ns // 1_000_000_000}.{ros_now_ns % 1_000_000_000:09d}"
                tf_age_sec = None
                if ros_now_ns > 0 and tf_stamp_ns > 0:
                    tf_age_sec = (ros_now_ns - tf_stamp_ns) / 1_000_000_000.0
                tf_age_txt = "n/a" if tf_age_sec is None else f"{tf_age_sec:.3f}"
                tf_fresh = tf_age_sec is None or (-0.10 <= float(tf_age_sec) <= max_tf_age_sec)
                if tf_fresh:
                    saw_fresh = True
                if dist < best_dist:
                    best_dist = dist
                    best_dist_ts = time.time()
                # Distancia manda; freshness solo diagnostica para evitar falsos abortos.
                tf_ok = bool(dist <= tol_m)
                last_diag = {
                    "dist": dist,
                    "best_dist": best_dist,
                    "best_dist_age_sec": max(0.0, time.time() - best_dist_ts) if best_dist_ts > 0 else None,
                    "target": target,
                    "tcp": tcp,
                    "tol": float(tol_m),
                    "frame": frame_id,
                    "ee_frame": ee_frame,
                    "tf_stamp": tf_stamp,
                    "tf_stamp_ns": tf_stamp_ns,
                    "ros_now_ns": ros_now_ns,
                    "tf_age_sec": tf_age_sec,
                    "tf_fresh": tf_fresh,
                    "saw_fresh": saw_fresh,
                    "ok": tf_ok,
                }
                now_txt = ros_now_txt
                panel._emit_log(
                    f"[TF_CHECK] label={label} frame={frame_id} ee_link={ee_frame} "
                    f"dist={dist:.3f} best={best_dist:.3f} tol={tol_m:.3f} "
                    f"tf_age={tf_age_txt}s ros_now={now_txt} tf_stamp={tf_stamp} "
                    f"fresh={str(tf_fresh).lower()}"
                )
                if tf_ok:
                    panel._emit_log(
                        f"[PICK_OBJ][TF][CHECK] {label} target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) "
                        f"tcp=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) dist={dist:.3f} "
                        f"tol={tol_m:.3f} frame={frame_id} ee={ee_frame} "
                        f"tf_stamp={tf_stamp} ros_now={ros_now_txt} tf_age={tf_age_txt}s "
                        f"fresh={str(tf_fresh).lower()} ok=true"
                    )
                    return last_diag
                # If TF is stale, give it a short dedicated grace window before
                # deciding based on distance timeout.
                if (not tf_fresh) and (time.time() <= fresh_deadline):
                    time.sleep(0.08)
                    continue
                now = time.time()
                if (now - wait_log_ts) >= 0.6:
                    panel._emit_log(
                        f"[PICK_OBJ][TF][WAIT] {label} dist={dist:.3f} tol={tol_m:.3f} "
                        f"best={best_dist:.3f} tf_age={tf_age_txt}s fresh={str(tf_fresh).lower()} "
                        f"frame={frame_id} ee={ee_frame}"
                    )
                    wait_log_ts = now
                time.sleep(0.10)
            if last_diag is None:
                raise RuntimeError(f"TF sin datos para {frame_id}->{ee_frame}")
            last_tcp = last_diag["tcp"]
            tf_age_sec = last_diag.get("tf_age_sec")
            tf_age_txt = "n/a" if tf_age_sec is None else f"{float(tf_age_sec):.3f}"
            fail_reason = "pos_not_reached"
            last_diag["fail_reason"] = fail_reason
            panel._emit_log(
                f"[PICK_OBJ][TF][CHECK] {label} target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) "
                f"tcp=({last_tcp[0]:.3f},{last_tcp[1]:.3f},{last_tcp[2]:.3f}) dist={float(last_diag['dist']):.3f} "
                f"best={float(last_diag.get('best_dist', float(last_diag['dist']))):.3f} "
                f"tol={tol_m:.3f} frame={frame_id} ee={ee_frame} "
                f"tf_stamp={last_diag.get('tf_stamp', 'n/a')} tf_age={tf_age_txt}s "
                f"fresh={str(bool(last_diag.get('tf_fresh', False))).lower()} "
                f"reason={fail_reason} ok=false"
            )
            return last_diag

        try:
            panel._emit_log("[PICK_OBJ] === WORKER INICIADO ===")
            move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
            home_pose = panel._get_home_joint_pose()
            _dbg(f"[PICK_OBJ] HOME pose: {[f'{math.degrees(j):.1f}°' for j in home_pose]}")
            transport_fallback_enabled = str(
                os.environ.get("PANEL_PICK_OBJECT_TRANSPORT_JOINT_FALLBACK", "1")
            ).strip().lower() not in ("0", "false", "no", "off")
            approach_fallback_enabled = str(
                os.environ.get("PANEL_PICK_OBJECT_APPROACH_JOINT_FALLBACK", "1")
            ).strip().lower() not in ("0", "false", "no", "off")

            def _is_step_tf_mismatch(msg: str, label: str) -> bool:
                return "exec_succeeded_but_tf_mismatch" in str(msg) and f"label={label}" in str(msg)

            def _is_step_exec_failed(msg: str, label: str) -> bool:
                txt = str(msg)
                return (
                    f"execute failed ({label})" in txt
                    or (f"label={label}" in txt and "exec_failed:" in txt)
                    or (f"label={label}" in txt and "fjt_result_timeout" in txt)
                )

            def _run_joint_step(
                label: str,
                joints: List[float],
                timeout_sec: Optional[float] = None,
                tol_rad: float = 0.02,
            ) -> None:
                _log_selection_changed_during_pick()
                panel._emit_log(
                    f"[PICK_OBJ] {label}: enviando joints {[f'{math.degrees(j):.1f}°' for j in joints]}"
                )
                ok, info = panel._publish_joint_trajectory(joints, move_sec)
                if not ok:
                    raise RuntimeError(f"{label} fallo: {info}")
                wait_timeout = move_sec + 2.0 if timeout_sec is None else timeout_sec
                if not panel._wait_for_joint_target(
                    joints,
                    wait_timeout,
                    tol_rad=tol_rad,
                ):
                    panel._emit_log(f"[PICK_OBJ] WARN: {label} no alcanzado exactamente")

            def _ensure_gripper_open_for_moveit(*, reason: str = "MOVEIT") -> None:
                try:
                    min_opening_m = float(
                        os.environ.get("PANEL_PICK_OBJECT_MIN_OPENING_M", "0.020")
                    )
                except Exception:
                    min_opening_m = 0.020
                reason_upper = str(reason or "").strip().upper()
                if reason_upper in ("PRE_GRASP", "GRASP_DOWN"):
                    try:
                        min_opening_before_descent_m = float(
                            os.environ.get(
                                "PANEL_PICK_OBJECT_MIN_OPENING_BEFORE_DESCENT_M", "0.030"
                            )
                        )
                    except Exception:
                        min_opening_before_descent_m = 0.030
                    min_opening_m = max(min_opening_m, min_opening_before_descent_m)
                min_opening_m = max(0.0, min_opening_m)
                try:
                    open_wait_sec = float(
                        os.environ.get("PANEL_PICK_OBJECT_OPEN_WAIT_SEC", "2.2")
                    )
                except Exception:
                    open_wait_sec = 2.2
                open_wait_sec = max(0.1, open_wait_sec)

                joint_names = [str(j).strip() for j in (GRIPPER_JOINT_NAMES or []) if str(j).strip()]
                if not joint_names:
                    joint_names = ["rg2_finger_joint1", "rg2_finger_joint2"]

                def _current_opening_m() -> tuple[Optional[float], bool]:
                    if panel.ros_worker is None:
                        return None, False
                    payload, payload_wall = panel.ros_worker.get_last_joint_state()
                    if not payload:
                        return None, False
                    if payload_wall <= 0.0 or (time.time() - float(payload_wall)) > 1.0:
                        return None, False
                    try:
                        names = list(payload.get("name", []))
                        pos = list(payload.get("position", []))
                    except Exception:
                        return None, False
                    if not names or not pos:
                        return None, False
                    pos_map = {str(n): float(p) for n, p in zip(names, pos)}
                    vals = [abs(float(pos_map[j])) for j in joint_names if j in pos_map]
                    if len(vals) != len(joint_names):
                        return None, True
                    return float(sum(vals)), True

                opening_ok = False
                saw_joint_state = False
                last_opening_m: Optional[float] = None
                for attempt in (1, 2):
                    state = {"done": False, "ok": False}

                    def _open_cmd() -> None:
                        state["ok"] = bool(
                            panel._command_gripper(False, log_action="PICK", force=True)
                        )
                        state["done"] = True

                    panel.signal_run_ui.emit(_open_cmd)
                    deadline = time.time() + 1.5
                    while time.time() < deadline:
                        if state["done"]:
                            break
                        time.sleep(0.03)
                    if not state["done"]:
                        raise RuntimeError("gripper_open_timeout_before_moveit")
                    if not state["ok"]:
                        raise RuntimeError("gripper_open_command_failed_before_moveit")
                    if bool(getattr(panel, "_gripper_closed", False)):
                        raise RuntimeError("gripper_state_closed_before_moveit")

                    wait_deadline = time.time() + open_wait_sec
                    while time.time() < wait_deadline:
                        opening_m, has_js = _current_opening_m()
                        saw_joint_state = saw_joint_state or has_js
                        if opening_m is not None:
                            last_opening_m = float(opening_m)
                            if last_opening_m >= min_opening_m:
                                opening_ok = True
                                break
                        time.sleep(0.05)
                    if opening_ok:
                        break
                    if attempt == 1:
                        panel._emit_log(
                            f"[PICK_OBJ][GRIPPER] apertura insuficiente antes de {reason}; "
                            "reintentando abrir"
                        )

                if saw_joint_state and not opening_ok:
                    raise RuntimeError(
                        f"gripper_opening_below_threshold_before_{str(reason).lower()} "
                        f"opening_m={0.0 if last_opening_m is None else float(last_opening_m):.4f} "
                        f"min_opening_m={float(min_opening_m):.4f}"
                    )
                if saw_joint_state and opening_ok:
                    panel._emit_log(
                        f"[PICK_OBJ][GRIPPER] apertura_ok before={reason} "
                        f"opening_m={float(last_opening_m or 0.0):.4f} "
                        f"min_opening_m={float(min_opening_m):.4f}"
                    )
                panel._emit_log(
                    f"[PICK_OBJ][GRIPPER] open_guard before={reason} "
                    f"min_opening_m={float(min_opening_m):.4f} open_wait_sec={float(open_wait_sec):.2f}"
                )
                try:
                    gripper_settle_sec = float(
                        os.environ.get("PANEL_PICK_OBJECT_GRIPPER_OPEN_SETTLE_SEC", "0.70")
                    )
                except Exception:
                    gripper_settle_sec = 0.70
                if gripper_settle_sec > 0.0:
                    time.sleep(gripper_settle_sec)
                panel._emit_log(f"[PICK_OBJ] Gripper opened BEFORE {reason}")

            def _run_moveit_step(label: str, pose_data: dict, delay: float) -> None:
                nonlocal moveit_monitor_total, moveit_monitor_manual
                label_up = str(label).upper()
                _log_selection_changed_during_pick()
                _wait_manual_idle_before_moveit(label)
                if gripper_open_required["active"] and label_up in ("APPROACH", "PRE_GRASP", "GRASP_DOWN"):
                    panel._emit_log(f"[PICK_OBJ][GRIPPER] enforce_open_before_{label_up}")
                    _ensure_gripper_open_for_moveit(reason=label_up)
                if gripper_open_required["active"] and bool(getattr(panel, "_gripper_closed", False)):
                    raise RuntimeError(
                        f"gripper_open_required_before_{str(label).lower()}"
                    )
                if panel._moveit_state != MoveItState.READY:
                    raise RuntimeError("MoveIt no listo durante la secuencia")
                _ensure_moveit_exec_monitor()
                moveit_monitor_total = 0
                moveit_monitor_manual = 0
                moveit_exec_window["active"] = True
                setattr(panel, "_pick_moveit_phase_active", True)
                position = pose_data.get("position", (0.0, 0.0, 0.0))
                frame_id = pose_data.get("frame", BASE_FRAME or "base_link")
                try:
                    panel._emit_log(
                        f"[PICK_OBJ][MOVEIT][PLAN] Pose {label} (frame={frame_id}): {position}"
                    )
                    bridge_recovered = _ensure_moveit_bridge_path(moveit_pose_topic)
                    try:
                        moveit_wait_sec = float(
                            os.environ.get("PANEL_PICK_OBJECT_MOVEIT_WAIT_SEC", "35.0")
                        )
                    except Exception:
                        moveit_wait_sec = 35.0
                    moveit_wait_sec = max(10.0, moveit_wait_sec)
                    try:
                        moveit_wait_recovered_sec = float(
                            os.environ.get(
                                "PANEL_PICK_OBJECT_MOVEIT_WAIT_RECOVERED_SEC",
                                str(moveit_wait_sec),
                            )
                        )
                    except Exception:
                        moveit_wait_recovered_sec = moveit_wait_sec
                    moveit_wait_recovered_sec = max(10.0, moveit_wait_recovered_sec)
                    if bridge_recovered:
                        panel._emit_log(
                            f"[PICK_OBJ][MOVEIT][PLAN] {label} bridge_recovered=true reset_request_id_gate"
                        )

                    def _next_panel_request_id() -> int:
                        seq = int(getattr(panel, "_pick_obj_moveit_request_id", 0) or 0) + 1
                        setattr(panel, "_pick_obj_moveit_request_id", seq)
                        return seq

                    def _next_panel_request_uuid() -> str:
                        return uuid.uuid4().hex

                    def _encode_request_frame(frame: str, request_id: int, request_uuid: str) -> str:
                        base = str(frame or BASE_FRAME or "base_link").strip() or "base_link"
                        rid = int(request_id)
                        uid = str(request_uuid or "").strip()
                        if uid:
                            return f"{base}|rid={rid}|uid={uid}"
                        return f"{base}|rid={rid}"

                    use_cartesian = False
                    if label_up == "GRASP_DOWN":
                        grasp_cartesian_raw = str(
                            os.environ.get("PANEL_PICK_OBJECT_GRASP_CARTESIAN", "0")
                        ).strip().lower()
                        use_cartesian = grasp_cartesian_raw not in ("0", "false", "no", "off")
                        panel._emit_log(
                            "[PICK_OBJ][MOVEIT][PLAN] GRASP_DOWN "
                            f"cartesian={str(use_cartesian).lower()} "
                            f"env=PANEL_PICK_OBJECT_GRASP_CARTESIAN:{grasp_cartesian_raw or 'default'}"
                        )

                    def _publish_and_wait_once(
                        *,
                        timeout_sec: float,
                        clear_gate: bool,
                        cartesian_mode: bool,
                    ) -> tuple[dict, int]:
                        drained = int(panel.ros_worker.drain_moveit_results(duration_sec=0.20))
                        if drained > 0:
                            panel._emit_log(
                                f"[PICK_OBJ][MOVEIT][PLAN] drain_stale_results drained={drained} "
                                f"topic={moveit_result_topic}"
                            )
                        _baseline_raw, baseline_wall, baseline_seq = panel.ros_worker.moveit_result_snapshot()
                        if clear_gate:
                            baseline_seq = 0
                        panel_request_id = _next_panel_request_id()
                        panel_request_uuid = _next_panel_request_uuid()
                        request_stamp_ns = _ros_clock_now_ns()
                        if label_up == "APPROACH":
                            tol_step = 0.04
                        elif label_up == "PRE_GRASP":
                            tol_step = 0.03
                        elif label_up in ("LIFT", "TRANSPORT", "DROP"):
                            tol_step = 0.03
                        else:
                            tol_step = 0.02
                        pose_to_send = dict(pose_data)
                        pose_to_send["stamp_ns"] = int(request_stamp_ns)
                        pose_to_send["frame"] = _encode_request_frame(
                            frame_id, panel_request_id, panel_request_uuid
                        )
                        request_wall = max(time.time() - 0.05, float(baseline_wall) - 0.001)
                        pose_subs_now = panel.ros_worker.topic_subscriber_count(moveit_pose_topic)
                        if pose_subs_now <= 0:
                            raise RuntimeError(
                                f"no_pose_subscribers_before_publish topic={moveit_pose_topic}"
                            )
                        panel._emit_log(
                            f"[PICK_OBJ][STEP] label={label} request_id={panel_request_id} "
                            f"request_uuid={panel_request_uuid} "
                            f"lock_id={target_lock_id_local} target={target_snapshot_name} "
                            f"pose=({float(position[0]):.3f},{float(position[1]):.3f},{float(position[2]):.3f}) "
                            f"frame={frame_id} tol={tol_step:.3f}"
                        )
                        panel._emit_log(
                            f"[PICK_OBJ][MOVEIT][PUB] {label} request_id={panel_request_id} "
                            f"request_uuid={panel_request_uuid} topic={moveit_pose_topic} "
                            f"subs_count={pose_subs_now} stamp_ns={request_stamp_ns} "
                            f"lock_id={target_lock_id_local} target={target_snapshot_name} "
                            f"cartesian={str(bool(cartesian_mode)).lower()}"
                        )
                        panel._emit_log(
                            f"[PANEL][PUB] {label} request_id={panel_request_id} "
                            f"request_uuid={panel_request_uuid} stamp_ns={request_stamp_ns} "
                            f"topic={moveit_pose_topic} subs_count={pose_subs_now} "
                            f"lock_id={target_lock_id_local}"
                        )
                        published = panel._publish_moveit_pose(
                            label,
                            pose_to_send,
                            cartesian=bool(cartesian_mode),
                        )
                        if not published:
                            raise RuntimeError(
                                f"publicación MoveIt rechazada para {label} request_id={panel_request_id}"
                            )
                        expected_req_id = int(panel_request_id)
                        setattr(_wait_moveit_result, "_expected_request_id", expected_req_id)
                        setattr(_wait_moveit_result, "_expected_stamp_ns", int(request_stamp_ns))
                        setattr(_wait_moveit_result, "_expected_request_uuid", panel_request_uuid)
                        setattr(_wait_moveit_result, "_since_seq", int(baseline_seq))
                        setattr(_wait_moveit_result, "_panel_request_id", int(panel_request_id))
                        try:
                            result_data = _wait_moveit_result(label, request_wall, timeout_total=timeout_sec)
                        finally:
                            panel._motion_in_progress = False
                        return result_data, panel_request_id

                    try:
                        result, panel_request_id = _publish_and_wait_once(
                            timeout_sec=moveit_wait_recovered_sec if bridge_recovered else moveit_wait_sec,
                            clear_gate=bridge_recovered,
                            cartesian_mode=use_cartesian,
                        )
                    except RuntimeError as first_wait_exc:
                        first_err = str(first_wait_exc)
                        retry_reasons = (
                            "sin resultado",
                            "lost_result_publisher",
                            "sin publisher",
                            "no_pose_subscribers_before_publish",
                        )
                        if any(r in first_err for r in retry_reasons):
                            panel._emit_log(
                                f"[PICK_OBJ][MOVEIT][RECOVERY] reason=lost_result_publisher label={label} "
                                f"detail={first_err}"
                            )
                            _ensure_moveit_bridge_path(moveit_pose_topic)
                            panel._emit_log(
                                f"[PICK_OBJ][MOVEIT][PUB] {label} no_result_after_publish retry_once=true "
                                f"reason={first_err}"
                            )
                            try:
                                result, panel_request_id = _publish_and_wait_once(
                                    timeout_sec=moveit_wait_sec,
                                    clear_gate=True,
                                    cartesian_mode=use_cartesian,
                                )
                            except RuntimeError as second_wait_exc:
                                raise RuntimeError(
                                    f"no_result_after_publish request_id={getattr(_wait_moveit_result, '_panel_request_id', -1)} "
                                    f"label={label} reason={second_wait_exc}"
                                ) from second_wait_exc
                        else:
                            raise
                    ee_link_moveit = str(result.get("ee_link") or "")
                    if ee_link_moveit and _norm_frame(ee_link_moveit) != _norm_frame(measured_ee_frame):
                        panel._emit_log(
                            f"[PICK_OBJ][ABORT] ee_link mismatch moveit={ee_link_moveit} tf={measured_ee_frame}"
                        )
                        raise RuntimeError(
                            f"ee_link mismatch (moveit={ee_link_moveit} tf={measured_ee_frame})"
                        )
                    if not bool(result.get("success", False)):
                        message = str(result.get("message") or "execute failed")
                        if label_up == "GRASP_DOWN" and use_cartesian and "cartesian_" in message:
                            panel._emit_log(
                                "[PICK_OBJ][MOVEIT][RECOVERY] GRASP_DOWN cartesian fallo; "
                                "reintentando con pose planner"
                            )
                            result, panel_request_id = _publish_and_wait_once(
                                timeout_sec=moveit_wait_sec,
                                clear_gate=True,
                                cartesian_mode=False,
                            )
                            message = str(result.get("message") or "")
                        if not bool(result.get("success", False)):
                            panel._emit_log(
                                f"[PICK_OBJ][ABORT] execute failed label={label} reason={message}"
                            )
                            raise RuntimeError(f"execute failed ({label}): {message}")
                    if label == "PRE_GRASP":
                        try:
                            tol = float(os.environ.get("PANEL_PICK_OBJECT_PRE_GRASP_TOL_M", "0.08"))
                        except Exception:
                            tol = 0.08
                    elif label in ("LIFT", "TRANSPORT", "DROP"):
                        try:
                            tol = float(os.environ.get("PANEL_PICK_OBJECT_PATH_TOL_M", "0.03"))
                        except Exception:
                            tol = 0.03
                    else:
                        try:
                            tol = float(os.environ.get("PANEL_PICK_OBJECT_STEP_TOL_M", "0.02"))
                        except Exception:
                            tol = 0.02
                    panel._emit_log(
                        f"[PICK_OBJ][MOVEIT][EXEC] {label} ee_link_moveit={ee_link_moveit or 'n/a'} "
                        f"ee_link_tf_check={measured_ee_frame} target_frame={frame_id} "
                        f"tf_lookup={frame_id}->{measured_ee_frame}"
                    )
                    tf_diag = _tf_distance_check(
                        label=label,
                        pose_data=pose_data,
                        tol_m=tol,
                        ee_frame=measured_ee_frame,
                    )
                    dist = float(tf_diag.get("dist", 0.0))
                    if not bool(tf_diag.get("ok", False)):
                        tf_age_sec = tf_diag.get("tf_age_sec")
                        tf_stamp = str(tf_diag.get("tf_stamp", "n/a"))
                        tf_age_txt = "n/a" if tf_age_sec is None else f"{float(tf_age_sec):.3f}s"
                        fresh = str(bool(tf_diag.get("tf_fresh", False))).lower()
                        fail_reason = str(tf_diag.get("fail_reason") or "tf_mismatch")
                        msg = (
                            "exec_succeeded_but_tf_mismatch "
                            f"label={label} dist={dist:.3f} tol={tol:.3f} "
                            f"ee_link={measured_ee_frame} frame={frame_id} "
                            f"tf_stamp={tf_stamp} tf_age={tf_age_txt} fresh={fresh} "
                            f"reason={fail_reason}"
                        )
                        panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
                        raise RuntimeError(msg)
                    panel._emit_log(
                        f"[PICK_OBJ][MOVEIT][EXEC] {label} traj_msgs={moveit_monitor_total} "
                        f"manual_conflicts={moveit_monitor_manual} topic={moveit_monitor_topic}"
                    )
                    if moveit_monitor_manual > 0:
                        raise RuntimeError(
                            f"manual_conflict: published while moveit_executing ({label})"
                        )
                    time.sleep(delay)
                finally:
                    moveit_exec_window["active"] = False
                    setattr(panel, "_pick_moveit_phase_active", False)

            # By default start from MESA for a stable and reproducible approach.
            # Legacy modes are kept via env overrides.
            preflight_mode = str(
                os.environ.get("PANEL_PICK_OBJECT_PREFLIGHT_MODE", "mesa")
            ).strip().lower()
            if preflight_mode in ("fixed", "legacy", "table", "pick_image", "1", "true", "on"):
                panel._emit_log("[PICK_OBJ] FASE 1: Acercamiento a mesa (preflight=fixed)")
                _run_joint_step("HOME", home_pose)
                _run_joint_step("MESA", JOINT_TABLE_POSE_RAD)
                _run_joint_step("PICK_IMAGE", JOINT_PICK_IMAGE_POSE_RAD)
            elif preflight_mode in ("mesa", "mesa_only", "table_only", "default"):
                panel._emit_log("[PICK_OBJ] FASE 1: Ir a MESA antes del pick (preflight=mesa)")
                _run_joint_step(
                    "MESA_PREGRASP",
                    JOINT_TABLE_POSE_RAD,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=0.06,
                )
            else:
                panel._emit_log(
                    "[PICK_OBJ] FASE 1: Preflight fijo omitido (preflight=direct); "
                    "se usa objetivo live seleccionado"
                )

            panel._emit_log("[PICK_OBJ] FASE 2: Abrir gripper antes de MoveIt")
            _ensure_gripper_open_for_moveit()

            panel._emit_log("[PICK_OBJ] FASE 3: APPROACH sobre objeto (pinza abierta)")
            try:
                _run_moveit_step(*sequence[0])
            except RuntimeError as exc:
                msg = str(exc)
                approach_tf_mismatch = _is_step_tf_mismatch(msg, "APPROACH")
                approach_exec_failed = _is_step_exec_failed(msg, "APPROACH")
                if not (approach_fallback_enabled and (approach_tf_mismatch or approach_exec_failed)):
                    raise
                reason = "tf_mismatch" if approach_tf_mismatch else "exec_failed"
                panel._emit_log(
                    f"[PICK_OBJ][RECOVERY] APPROACH {reason} detectado; "
                    "fallback articular a PICK_IMAGE y reintento APPROACH"
                )
                _run_joint_step(
                    "APPROACH_FALLBACK_JOINT",
                    JOINT_PICK_IMAGE_POSE_RAD,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=0.08,
                )
                _ensure_gripper_open_for_moveit(reason="APPROACH_RETRY")
                _run_moveit_step(*sequence[0])

            panel._emit_log("[PICK_OBJ] FASE 4: PRE_GRASP (pinza abierta)")
            _run_moveit_step(*sequence[1])

            panel._emit_log("[PICK_OBJ] FASE 5: GRASP_DOWN (pinza abierta)")
            grasp_label, grasp_pose_data, grasp_delay = sequence[2]
            grasp_pose_send = dict(grasp_pose_data)
            grasp_orig_pos = grasp_pose_send.get("position", (bx, by, bz))
            panel._emit_log(
                f"[PICK_OBJ][GRASP_PLAN] obj_base_z={bz:.3f} obj_top_z={obj_top_z_base:.3f} "
                f"tcp_target_z={float(grasp_orig_pos[2]):.3f} "
                f"contact_down_z={contact_down_z:.3f} tcp_z_offset={float(GRIPPER_TCP_Z_OFFSET):.3f} "
                f"table_top_base={table_top_base:.3f} table_margin={z_table_margin:.3f}"
            )
            grasp_pose_adapted = False
            adapt_grasp_xy = str(
                os.environ.get("PANEL_PICK_OBJECT_ADAPT_GRASP_XY", "1")
            ).strip().lower() not in ("0", "false", "no", "off")
            if adapt_grasp_xy:
                try:
                    tcp_pre, _ = _read_tcp_in_frame(base_frame, measured_ee_frame, timeout_sec=0.25)
                    if tcp_pre is not None:
                        try:
                            adapt_max = float(
                                os.environ.get("PANEL_PICK_OBJECT_ADAPT_GRASP_XY_MAX_DELTA_M", "0.015")
                            )
                        except Exception:
                            adapt_max = 0.015
                        dx_pre = float(tcp_pre[0]) - float(bx)
                        dy_pre = float(tcp_pre[1]) - float(by)
                        dxy_pre = math.hypot(dx_pre, dy_pre)
                        if dxy_pre <= adapt_max:
                            grasp_pose_send["position"] = (
                                float(tcp_pre[0]),
                                float(tcp_pre[1]),
                                float(grasp_orig_pos[2]),
                            )
                            grasp_pose_adapted = True
                            panel._emit_log(
                                "[PICK_OBJ][ADAPT] GRASP_DOWN xy_from_pregrasp "
                                f"orig=({float(grasp_orig_pos[0]):.3f},{float(grasp_orig_pos[1]):.3f}) "
                                f"new=({float(tcp_pre[0]):.3f},{float(tcp_pre[1]):.3f}) "
                                f"obj=({bx:.3f},{by:.3f}) dxy={dxy_pre:.3f} max={adapt_max:.3f}"
                            )
                        else:
                            panel._emit_log(
                                "[PICK_OBJ][ADAPT] GRASP_DOWN xy adapt skipped "
                                f"dx_pre={dx_pre:.3f} dy_pre={dy_pre:.3f} dxy={dxy_pre:.3f} "
                                f"max={adapt_max:.3f}"
                            )
                except Exception as exc:
                    panel._emit_log(f"[PICK_OBJ][ADAPT] GRASP_DOWN xy adapt error: {exc}")
            grasp_joint_fallback_enabled = str(
                os.environ.get("PANEL_PICK_OBJECT_GRASP_JOINT_FALLBACK", "0")
            ).strip().lower() not in ("0", "false", "no", "off")
            grasp_moveit_retry_enabled = str(
                os.environ.get("PANEL_PICK_OBJECT_GRASP_MOVEIT_RETRY", "1")
            ).strip().lower() not in ("0", "false", "no", "off")
            if grasp_micro_poses:
                gx, gy, _ = grasp_pose_send.get("position", (bx, by, bz))
                panel._emit_log(
                    f"[PICK_OBJ][GRASP_MICRO] steps={len(grasp_micro_poses)} "
                    f"step_m={grasp_micro_step_m:.3f} start_z={pre_grasp_pose[2]:.3f} "
                    f"target_z={float(grasp_orig_pos[2]):.3f}"
                )
                for idx, _micro_pose in enumerate(grasp_micro_poses, start=1):
                    micro_data = dict(grasp_pose_send)
                    micro_data["position"] = (float(gx), float(gy), float(_micro_pose[2]))
                    micro_label = f"GRASP_DOWN_MICRO_{idx}"
                    try:
                        _run_moveit_step(micro_label, micro_data, min(0.25, grasp_delay))
                    except RuntimeError as exc:
                        msg = str(exc)
                        is_micro_step_issue = (
                            "label=GRASP_DOWN_MICRO_" in msg
                            or micro_label in msg
                        )
                        if not is_micro_step_issue:
                            raise
                        panel._emit_log(
                            "[PICK_OBJ][RECOVERY] micro-step descent failed; "
                            f"skipping remaining micro-steps and falling back to {grasp_label} recovery path "
                            f"reason={msg}"
                        )
                        break
            try:
                _run_moveit_step(grasp_label, grasp_pose_send, grasp_delay)
            except RuntimeError as exc:
                msg = str(exc)
                grasp_tf_mismatch = (
                    "exec_succeeded_but_tf_mismatch" in msg and "label=GRASP_DOWN" in msg
                )
                grasp_cartesian_enabled = str(
                    os.environ.get("PANEL_PICK_OBJECT_GRASP_CARTESIAN", "0")
                ).strip().lower() not in ("0", "false", "no", "off")
                if grasp_tf_mismatch and grasp_cartesian_enabled:
                    panel._emit_log(
                        "[PICK_OBJ][RECOVERY] GRASP_DOWN tf_mismatch en cartesian; "
                        "reintentando con pose planner (cartesian=off)"
                    )
                    prev_cartesian_env = os.environ.get("PANEL_PICK_OBJECT_GRASP_CARTESIAN")
                    os.environ["PANEL_PICK_OBJECT_GRASP_CARTESIAN"] = "0"
                    try:
                        _run_moveit_step(grasp_label, grasp_pose_send, grasp_delay)
                        panel._emit_log(
                            "[PICK_OBJ][RECOVERY] GRASP_DOWN retry pose planner ok"
                        )
                        grasp_tf_mismatch = False
                    except RuntimeError as retry_exc:
                        msg = str(retry_exc)
                        grasp_tf_mismatch = (
                            "exec_succeeded_but_tf_mismatch" in msg and "label=GRASP_DOWN" in msg
                        )
                        if not grasp_tf_mismatch:
                            raise
                    finally:
                        if prev_cartesian_env is None:
                            os.environ.pop("PANEL_PICK_OBJECT_GRASP_CARTESIAN", None)
                        else:
                            os.environ["PANEL_PICK_OBJECT_GRASP_CARTESIAN"] = prev_cartesian_env
                if grasp_tf_mismatch and grasp_pose_adapted:
                    panel._emit_log(
                        "[PICK_OBJ][RECOVERY] GRASP_DOWN tf_mismatch con xy adaptado; "
                        "reintentando con xy exacto del objeto"
                    )
                    try:
                        _ensure_gripper_open_for_moveit(reason="GRASP_DOWN_RETRY_OBJECT_XY")
                        retry_pose = dict(grasp_pose_data)
                        retry_pos = retry_pose.get("position", (bx, by, bz))
                        retry_pose["position"] = (float(bx), float(by), float(retry_pos[2]))
                        _run_moveit_step(grasp_label, retry_pose, grasp_delay)
                        panel._emit_log(
                            "[PICK_OBJ][RECOVERY] GRASP_DOWN retry object_xy ok"
                        )
                        grasp_tf_mismatch = False
                    except RuntimeError as retry_exc:
                        msg = str(retry_exc)
                        grasp_tf_mismatch = (
                            "exec_succeeded_but_tf_mismatch" in msg and "label=GRASP_DOWN" in msg
                        )
                        if not grasp_tf_mismatch:
                            raise
                if grasp_tf_mismatch and grasp_moveit_retry_enabled:
                    try:
                        retry_count = int(
                            os.environ.get("PANEL_PICK_OBJECT_GRASP_MOVEIT_RETRY_COUNT", "2")
                        )
                    except Exception:
                        retry_count = 2
                    retry_count = max(1, min(retry_count, 5))
                    retry_ok = False
                    panel._emit_log(
                        "[PICK_OBJ][RECOVERY] GRASP_DOWN tf_mismatch persistente; "
                        f"replan APPROACH+PRE_GRASP con {retry_count} reintento(s)"
                    )
                    for retry_idx in range(1, retry_count + 1):
                        panel._emit_log(
                            f"[PICK_OBJ][RECOVERY] GRASP_DOWN replan intento {retry_idx}/{retry_count}"
                        )
                        try:
                            _ensure_gripper_open_for_moveit(
                                reason=f"GRASP_DOWN_RETRY_REPLAN_{retry_idx}"
                            )
                            _run_moveit_step(*sequence[0])  # APPROACH
                            _run_moveit_step(*sequence[1])  # PRE_GRASP
                            retry_pose = dict(grasp_pose_data)
                            retry_pos = retry_pose.get("position", (bx, by, bz))
                            retry_pose["position"] = (float(bx), float(by), float(retry_pos[2]))
                            _run_moveit_step(grasp_label, retry_pose, grasp_delay)
                            panel._emit_log(
                                f"[PICK_OBJ][RECOVERY] GRASP_DOWN retry replan ok intento={retry_idx}"
                            )
                            grasp_tf_mismatch = False
                            retry_ok = True
                            break
                        except RuntimeError as retry_exc:
                            msg = str(retry_exc)
                            grasp_tf_mismatch = (
                                ("exec_succeeded_but_tf_mismatch" in msg and "label=GRASP_DOWN" in msg)
                                or ("label=APPROACH" in msg)
                                or ("label=PRE_GRASP" in msg)
                            )
                            if not grasp_tf_mismatch:
                                raise
                    if not retry_ok and grasp_tf_mismatch:
                        panel._emit_log(
                            "[PICK_OBJ][RECOVERY] GRASP_DOWN replan agotado sin converger"
                        )
                if not (grasp_joint_fallback_enabled and grasp_tf_mismatch):
                    raise
                panel._emit_log(
                    "[PICK_OBJ][RECOVERY] GRASP_DOWN tf_mismatch detectado; "
                    "fallback a descenso articular"
                )
                _run_joint_step(
                    "GRASP_DOWN_FALLBACK_JOINT",
                    JOINT_GRASP_DOWN_POSE_RAD,
                    timeout_sec=move_sec + 2.5,
                    tol_rad=0.08,
                )
            time.sleep(0.3)  # Permitir que brazo se estabilice en posición

            tcp_base, _tcp_tf = _read_tcp_in_frame(base_frame, measured_ee_frame, timeout_sec=0.3)
            if tcp_base is None:
                raise RuntimeError(f"no se pudo leer TCP en {base_frame}")
            target_tcp_z = float(grasp_pose_send.get("position", (bx, by, bz))[2])
            try:
                grasp_z_reach_tol = float(os.environ.get("PANEL_PICK_OBJECT_GRASP_Z_REACH_TOL_M", "0.015"))
            except Exception:
                grasp_z_reach_tol = 0.015
            z_reach_err = abs(float(tcp_base[2]) - target_tcp_z)
            panel._emit_log(
                f"[PICK_OBJ][GRASP_VALIDATE] target_tcp_z={target_tcp_z:.3f} "
                f"tcp_z={float(tcp_base[2]):.3f} z_err={z_reach_err:.3f} "
                f"z_tol={grasp_z_reach_tol:.3f}"
            )
            if z_reach_err > grasp_z_reach_tol:
                raise RuntimeError(
                    f"altura_insuficiente_z target={target_tcp_z:.3f} tcp={float(tcp_base[2]):.3f} "
                    f"err={z_reach_err:.3f} tol={grasp_z_reach_tol:.3f}"
                )
            try:
                attach_z_ref_mode = str(os.environ.get("PANEL_PICK_OBJECT_ATTACH_Z_REF_MODE", "center")).strip().lower()
            except Exception:
                attach_z_ref_mode = "center"
            if attach_z_ref_mode not in ("center", "top"):
                attach_z_ref_mode = "center"
            try:
                attach_z_clearance = float(os.environ.get("PANEL_PICK_OBJECT_ATTACH_Z_CLEARANCE_M", "0.0"))
            except Exception:
                attach_z_clearance = 0.0
            obj_center_z = float(bz)
            obj_top_z = obj_center_z + (float(height or 0.0) * 0.5 if float(height or 0.0) > 0.0 else 0.0)
            obj_ref_z = (obj_center_z if attach_z_ref_mode == "center" else obj_top_z) + float(attach_z_clearance)
            dx_obj = bx - tcp_base[0]
            dy_obj = by - tcp_base[1]
            dz_obj = tcp_base[2] - obj_ref_z
            try:
                max_tcp_above_obj_z = float(os.environ.get("PANEL_PICK_OBJECT_MAX_TCP_ABOVE_OBJ_Z", "0.045"))
            except Exception:
                max_tcp_above_obj_z = 0.045
            try:
                object_xy_gate_tol = float(os.environ.get("PANEL_PICK_OBJECT_OBJECT_XY_GATE_TOL_M", "0.030"))
            except Exception:
                object_xy_gate_tol = 0.030
            try:
                tcp_obj_dist_max = float(os.environ.get("PANEL_PICK_OBJECT_TCP_OBJ_DIST_MAX_M", "0.055"))
            except Exception:
                tcp_obj_dist_max = 0.055
            tcp_obj_dist = math.sqrt((dx_obj * dx_obj) + (dy_obj * dy_obj) + (dz_obj * dz_obj))
            panel._emit_log(
                f"[PICK_OBJ][TF][CHECK] object_xy_error dx={dx_obj:.3f} dy={dy_obj:.3f} tol={object_xy_gate_tol:.3f}"
            )
            panel._emit_log(
                f"[PICK_OBJ][TF][CHECK] object_z_gap frame={base_frame} tcp_z={tcp_base[2]:.3f} "
                f"obj_center_z={obj_center_z:.3f} obj_top_z={obj_top_z:.3f} "
                f"obj_ref_z={obj_ref_z:.3f} z_ref_mode={attach_z_ref_mode} "
                f"z_clearance={attach_z_clearance:.3f} dz={dz_obj:.3f} max={max_tcp_above_obj_z:.3f} "
                f"tcp_stamp_ns={_tf_stamp_ns(_tcp_tf)} obj_stamp_ns={obj_base_stamp_ns}"
            )
            panel._emit_log(
                f"[PICK_OBJ][GRASP_VALIDATE] tcp_obj_dist={tcp_obj_dist:.3f} "
                f"tcp_obj_dist_max={tcp_obj_dist_max:.3f} "
                f"expected_obj_base=({bx:.3f},{by:.3f},{bz:.3f}) "
                f"tcp_base=({float(tcp_base[0]):.3f},{float(tcp_base[1]):.3f},{float(tcp_base[2]):.3f})"
            )
            if abs(dx_obj) > object_xy_gate_tol or abs(dy_obj) > object_xy_gate_tol:
                panel._emit_log(
                    f"[PICK_OBJ][ABORT] TCP lejos del objeto en XY dx={dx_obj:.3f} dy={dy_obj:.3f}"
                )
                raise RuntimeError(
                    f"TCP lejos del objeto; no se puede agarrar (dx={dx_obj:.3f},dy={dy_obj:.3f})"
                )
            if dz_obj > max_tcp_above_obj_z:
                panel._emit_log(
                    f"[PICK_OBJ][ABORT] TCP alto sobre objeto dz={dz_obj:.3f} max={max_tcp_above_obj_z:.3f}"
                )
                raise RuntimeError(
                    f"TCP demasiado alto para agarre (dz={dz_obj:.3f},max={max_tcp_above_obj_z:.3f})"
                )
            if tcp_obj_dist > tcp_obj_dist_max:
                panel._emit_log(
                    f"[PICK_OBJ][ABORT] TCP lejos del objeto (dist) dist={tcp_obj_dist:.3f} max={tcp_obj_dist_max:.3f}"
                )
                raise RuntimeError(
                    f"altura_insuficiente_z dist={tcp_obj_dist:.3f} max={tcp_obj_dist_max:.3f}"
                )

            # Anti-empuje: si el objeto se desplazo antes de cerrar, abortar.
            try:
                max_obj_move_before_close = float(
                    os.environ.get("PANEL_PICK_OBJECT_MAX_OBJ_MOVE_BEFORE_CLOSE_M", "0.025")
                )
            except Exception:
                max_obj_move_before_close = 0.025
            current_positions = get_object_positions() or {}
            cur_world = current_positions.get(obj_name)
            if cur_world is None:
                cur_state = get_object_state(obj_name)
                cur_world = (
                    tuple(cur_state.position) if cur_state and getattr(cur_state, "position", None) else None
                )
            if cur_world is not None:
                cwx, cwy, cwz = float(cur_world[0]), float(cur_world[1]), float(cur_world[2])
                ddx = cwx - float(obj_x)
                ddy = cwy - float(obj_y)
                ddz = cwz - float(obj_z)
                dpos = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
                panel._emit_log(
                    f"[PICK_OBJ][TF][CHECK] object_delta_before_close "
                    f"dx={ddx:.3f} dy={ddy:.3f} dz={ddz:.3f} dist={dpos:.3f} "
                    f"max={max_obj_move_before_close:.3f}"
                )
                if dpos > max_obj_move_before_close:
                    panel._emit_log(
                        f"[PICK_OBJ] ABORT object_moved_before_close delta={dpos:.3f}"
                    )
                    raise RuntimeError(
                        f"object_moved_before_close delta={dpos:.3f} max={max_obj_move_before_close:.3f}"
                    )

            panel._emit_log("[PICK_OBJ] Cerrando gripper en GRASP_DOWN")
            gripper_open_required["active"] = False
            # Seguir patrón de PICK_DEMO: cierre + grasp + attach en MISMO contexto UI thread
            attach_result = {"ok": False}
            try:
                attach_xy_tol = float(os.environ.get("PANEL_ATTACH_XY_TOL_M", "0.02"))
            except Exception:
                attach_xy_tol = 0.02
            try:
                attach_z_tol = float(os.environ.get("PANEL_ATTACH_Z_TOL_M", "0.03"))
            except Exception:
                attach_z_tol = 0.03

            def _close_grasp_attach():
                panel._command_gripper(True, log_action="PICK", force=True)
                tcp_attach_base, tcp_attach_tf = _read_tcp_in_frame(
                    base_frame,
                    measured_ee_frame,
                    timeout_sec=0.25,
                )
                if tcp_attach_base is None:
                    tcp_attach_base = tcp_base
                tcp_attach_stamp_ns = _tf_stamp_ns(tcp_attach_tf)
                panel._emit_log(
                    f"[ATTACH] selected={obj_name} attach_name={obj_name} "
                    f"resolved_entity={obj_name} frame={base_frame} "
                    f"lock_id={target_lock_id_local}"
                )
                attach_ok = panel._attempt_attach(
                    "pick_object",
                    selected_name=obj_name,
                    tcp_base=tcp_attach_base,
                    object_base=(bx, by, bz),
                    object_height=height if height else 0.0,
                    base_frame=base_frame,
                    xy_tol_m=attach_xy_tol,
                    z_tol_m=attach_z_tol,
                    z_ref_mode=attach_z_ref_mode,
                    z_clearance_m=attach_z_clearance,
                    tcp_stamp_ns=tcp_attach_stamp_ns,
                    obj_stamp_ns=obj_base_stamp_ns,
                )
                attach_result["ok"] = bool(attach_ok)
                if attach_ok:
                    mark_object_grasped(obj_name, reason="pick_object")
                    mark_object_attached(obj_name, reason="pick_object")
                else:
                    panel._emit_log("[PICK_OBJ] ✗ Attach fallido (sin objeto en rango)")
            
            panel.signal_run_ui.emit(_close_grasp_attach)
            time.sleep(1.0)  # Esperar más (como PICK_DEMO) para que attach se procese en Gazebo
            
            # Verificar que el attach fue exitoso
            if not attach_result["ok"]:
                raise RuntimeError("fallo al fijar objeto (attach no publicado)")
            obj_state = get_object_state(obj_name)
            if obj_state is None or obj_state.logical_state != ObjectLogicalState.CARRIED:
                panel._emit_log(f"[PICK_OBJ] ✗ Error: objeto no fue vinculado (state={obj_state})")
                raise RuntimeError("fallo al fijar objeto (attach no completado)")
            # Nota: NO iniciamos el thread de actualización de pose aquí.
            # Gazebo sincroniza automáticamente la posición del objeto en cada pose_info.
            # El thread causaba condición de carrera con bulk_update_object_positions().
            panel._selected_object = None
            panel._selected_px = None
            panel._selected_world = None
            panel.obj_panel.set_selected(None, "")
            panel._emit_log("[PICK_OBJ] FASE 6 COMPLETADA: Objeto agarrado")

            panel._emit_log("[PICK_OBJ] FASE 7: Levantar objeto 20cm (MoveIt)")
            _run_moveit_step(*sequence[3])  # LIFT

            return_to_table_after_grasp = str(
                os.environ.get("PANEL_PICK_OBJECT_RETURN_TO_MESA", "1")
            ).strip().lower() not in ("0", "false", "no", "off")
            if return_to_table_after_grasp:
                panel._emit_log("[PICK_OBJ] FASE 8: Volver a MESA con objeto agarrado (joint)")
                _run_joint_step(
                    "MESA_WITH_OBJECT",
                    JOINT_TABLE_POSE_RAD,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=0.06,
                )

            home_before_basket = str(
                os.environ.get("PANEL_PICK_OBJECT_HOME_BEFORE_CESTA", "1")
            ).strip().lower() not in ("0", "false", "no", "off")
            if home_before_basket:
                panel._emit_log("[PICK_OBJ] FASE 8.5: Volver a HOME con objeto (joint)")
                _run_joint_step(
                    "HOME_WITH_OBJECT",
                    home_pose,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=0.06,
                )

            basket_joint_only_raw = os.environ.get(
                "PANEL_PICK_OBJECT_TRANSPORT_JOINT_ONLY",
                os.environ.get("PANEL_PICK_OBJECT_BASKET_JOINT_ONLY", "1"),
            )
            transport_joint_only = str(basket_joint_only_raw).strip().lower() not in (
                "0",
                "false",
                "no",
                "off",
            )
            panel._emit_log(
                "[PICK_OBJ] FASE 9: Ir a CESTA con objeto "
                + ("(joint)" if transport_joint_only else "(MoveIt)")
            )
            transport_fallback_used = False
            if transport_joint_only:
                _run_joint_step(
                    "CESTA_WITH_OBJECT",
                    JOINT_BASKET_POSE_RAD,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=0.06,
                )
                transport_fallback_used = True
            else:
                try:
                    _run_moveit_step(*sequence[4])  # TRANSPORT
                except RuntimeError as exc:
                    msg = str(exc)
                    transport_tf_mismatch = _is_step_tf_mismatch(msg, "TRANSPORT")
                    transport_exec_failed = _is_step_exec_failed(msg, "TRANSPORT")
                    if not (transport_fallback_enabled and (transport_tf_mismatch or transport_exec_failed)):
                        raise
                    reason = "tf_mismatch" if transport_tf_mismatch else "exec_failed"
                    panel._emit_log(
                        f"[PICK_OBJ][RECOVERY] TRANSPORT {reason} detectado; "
                        "fallback a cesta articular"
                    )
                    _run_joint_step(
                        "CESTA_WITH_OBJECT_FALLBACK",
                        JOINT_BASKET_POSE_RAD,
                        timeout_sec=move_sec + 3.0,
                        tol_rad=0.06,
                    )
                    transport_fallback_used = True

            if transport_fallback_used:
                panel._emit_log(
                    "[PICK_OBJ] FASE 10: Drop MoveIt omitido (trayectoria articular a cesta)"
                )
            else:
                panel._emit_log("[PICK_OBJ] FASE 10: Descenso de drop en cesta (MoveIt)")
                _run_moveit_step(*sequence[5])  # DROP

            panel._emit_log("[PICK_OBJ] FASE 11: Soltar objeto en cesta")

            # Post-check - Validar posición antes de soltar
            basket_z_pos = drop_pose[2]
            panel._emit_log(
                f"[PICK_OBJ] POST-CHECK: Validando posición para soltar (cesta z={basket_z_pos:.3f}m)"
            )

            # Verificar que objeto está en estado CARRIED
            current_state = get_object_state(obj_name)
            if current_state:
                panel._emit_log(
                    f"[PICK_OBJ] POST-CHECK: Estado actual = {current_state.logical_state}"
                )
                if current_state.logical_state not in (
                    ObjectLogicalState.CARRIED,
                    ObjectLogicalState.GRASPED,
                ):
                    panel._emit_log(
                        f"[PICK_OBJ] WARN: Objeto {obj_name} no está en estado CARRIED/GRASPED"
                    )

            def _open_and_release() -> None:
                panel._command_gripper(False, log_action="DROP", force=True)
                panel._emit_log("[PICK_OBJ] Pinza abierta - soltando objeto")
                panel._emit_log(
                    f"[PICK_OBJ] POST-CHECK: Transición {current_state.logical_state if current_state else '?'} → RELEASED"
                )
                mark_object_released(obj_name, reason="pick_object")
                panel._emit_log(
                    f"[PICK_OBJ] POST-CHECK: {obj_name} estado = RELEASED ✓"
                )

            panel.signal_run_ui.emit(_open_and_release)
            time.sleep(1.0)
            panel._emit_log("[PICK_OBJ] Cerrando gripper (limpieza)")
            panel.signal_run_ui.emit(
                lambda: panel._command_gripper(True, log_action="DROP", force=True)
            )
            time.sleep(0.5)

            panel._emit_log("[PICK_OBJ] FASE 12: Regresar a HOME")
            _run_joint_step("HOME_FINAL", home_pose)

            panel._ui_set_status("Pick objeto: COMPLETADO ✓")
            panel._emit_log("[PICK_OBJ] === SECUENCIA COMPLETADA EXITOSAMENTE ===")
        except Exception as exc:
            import traceback

            err_txt = str(exc or "")
            err_low = err_txt.lower()
            fail_kind = "unknown"
            if "camera" in err_low:
                fail_kind = "camera_not_ready"
            elif "tf" in err_low or "ee_link" in err_low:
                fail_kind = "tf_inconsistent"
            elif "plan" in err_low or "publicación moveit" in err_low or "moveit" in err_low:
                fail_kind = "planning"
            elif "execute failed" in err_low or "exec_failed" in err_low:
                fail_kind = "execution"
            if "altura_insuficiente_z" in err_low or "tcp demasiado alto" in err_low:
                fail_kind = "height_insufficient_z"
            panel._emit_log(
                f"[PICK_OBJ][FAIL_CLASS] type={fail_kind} detail={err_txt}"
            )
            try:
                panel._emit_log("[PICK_OBJ][RECOVERY] Error detectado; intentando HOME_SAFE")
                _run_joint_step("HOME_SAFE", home_pose, timeout_sec=move_sec + 3.0, tol_rad=0.08)
            except Exception as home_exc:
                panel._emit_log(f"[PICK_OBJ][RECOVERY] HOME_SAFE falló: {home_exc}")
            _block(f"worker_exception:{exc}")
            panel._ui_set_status(f"Error en pick objeto: {exc}", error=True)
            panel._emit_log(f"[PICK_OBJ] ✗ ERROR: {exc}")
            panel._emit_log(f"[PICK_OBJ] Traceback: {traceback.format_exc()}")
        finally:
            try:
                _clear_moveit_exec_monitor()
            except Exception:
                pass
            
            # CRÍTICA: Liberar objeto si quedó en estado CARRIED/GRASPED por error
            try:
                obj_state = get_object_state(obj_name)
                if obj_state and obj_state.logical_state in (
                    ObjectLogicalState.CARRIED,
                    ObjectLogicalState.GRASPED,
                ):
                    panel._emit_log(
                        f"[PICK_OBJ][CLEANUP] Objeto {obj_name} en estado {obj_state.logical_state.value}; "
                        f"liberando (conexión fantasma prevention)"
                    )
                    mark_object_released(obj_name, reason="pick_object_error_cleanup")
                    panel._emit_log(
                        f"[PICK_OBJ][CLEANUP] {obj_name} liberado → ON_TABLE"
                    )
            except Exception as cleanup_exc:
                panel._emit_log(
                    f"[PICK_OBJ][CLEANUP] Error liberando objeto: {cleanup_exc}"
                )
            
            prev_lock_id = str(getattr(panel, "_pick_target_lock_id", "") or target_lock_id)
            prev_lock_name = str(getattr(panel, "_pick_target_lock_name", "") or obj_name)
            panel._emit_log(
                f"[PICK_OBJ][TARGET_LOCK] release lock_id={prev_lock_id} name={prev_lock_name}"
            )
            setattr(panel, "_pick_target_lock_active", False)
            setattr(panel, "_pick_target_lock_name", "")
            setattr(panel, "_pick_target_lock_ts", 0.0)
            setattr(panel, "_pick_target_lock_id", "")
            setattr(panel, "_pick_target_lock_source", "")
            setattr(panel, "_pick_target_lock_reason", "")
            setattr(panel, "_pick_target_lock_world", None)
            setattr(panel, "_pick_target_lock_base", None)
            setattr(panel, "_pick_moveit_phase_active", False)
            panel._set_motion_lock(False)

    panel._run_async(worker)
