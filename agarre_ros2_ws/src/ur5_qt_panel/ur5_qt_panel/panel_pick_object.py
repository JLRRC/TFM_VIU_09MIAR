#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Pick object sequence helper for the panel."""
from __future__ import annotations

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

try:
    from std_msgs.msg import Empty
    from std_msgs.msg import Float64MultiArray
except Exception:  # pragma: no cover - ROS not available in unit contexts
    Empty = None
    Float64MultiArray = None

from ur5_tools.gripper_geometry import (
    RG2_PINCH_CENTER_FRAME,
    RG2_TCP_FRAME,
    contact_z_correction_for_frame,
    load_gripper_geometry,
)

from .panel_config import (
    BASKET_DROP,
    BASE_FRAME,
    DROP_NAME_SET,
    PICK_DEMO_NAME_SET,
    GRIPPER_ATTACH_PREFIX,
    GRIPPER_JOINT_NAMES,
    GRIPPER_TCP_Z_OFFSET,
    PICK_DEMO_DROP_Z_OFFSET,
    PICK_DEMO_GRASP_Z_OFFSET,
    WORLD_FRAME,
)
from .panel_pick_geometry import compute_pick_height_profile
from .panel_tfm_params import get_panel_tfm_params as _get_panel_tfm_params
from .panel_pick_object_params import (
    get_pick_object_params as _get_pick_object_params,
)
from ur5_tools.moveit_bridge.params import (
    get_moveit_bridge_params as _get_moveit_bridge_params,
)
from .panel_robot_presets import (
    JOINT_BASKET_POSE_RAD,
    JOINT_GRASP_DOWN_POSE_RAD,
    JOINT_TABLE_POSE_RAD,
    JOINT_PICK_IMAGE_POSE_RAD,
    _make_pose_data,
)
from .panel_utils import get_object_positions, transform_point_to_frame, get_tf_helper, nearest_table_object, fmt_vec3, fmt_pose_dict, fmt_age_sec
from .panel_objects import (
    ObjectLogicalState,
    ObjectOwner,
    get_object_state,
    get_object_states,
    is_on_table,
    mark_object_attached,
    mark_object_grasped,
    mark_object_released,
    update_object_state,
)
from .panel_readiness import pick_ui_status, tf_ready_status
from .panel_state import MoveItState
from .pick_object.parsing_helpers import (
    is_step_exec_failed as _is_step_exec_failed_pure,
    is_step_tf_mismatch as _is_step_tf_mismatch_pure,
    max_joint_error as _max_joint_error_pure,
    norm_frame as _norm_frame_pure,
    sanitize_ros_ns as _sanitize_ros_ns_pure,
    tf_stamp_ns as _tf_stamp_ns_pure,
)
from .pick_object.wait_moveit_result import (
    WaitMoveItResultContext,
    wait_moveit_result as _wait_moveit_result_pure,
)
from .pick_object.tf_distance_check import (
    TfDistanceCheckContext,
    tf_distance_check as _tf_distance_check_pure,
)
from .pick_object.ensure_gripper_open import (
    EnsureGripperOpenContext,
    ensure_gripper_open_for_moveit as _ensure_gripper_open_for_moveit_pure,
)
from .pick_object.carry_coherence import (
    CarryCoherenceContext,
    assert_carry_coherence_after_lift as _assert_carry_coherence_after_lift_pure,
)
from .pick_object.strict_pre_lift_contact import (
    StrictPreLiftContactContext,
    StrictPreLiftContactState,
    ensure_strict_pre_lift_contact as _ensure_strict_pre_lift_contact_pure,
)
from .pick_object.moveit_bridge_path import (
    MoveItBridgePathContext,
    ensure_moveit_bridge_path_main_body as _ensure_moveit_bridge_path_main_body_pure,
)


_PICK_OBJECT_GRIPPER_GEOMETRY = load_gripper_geometry()


def run_pick_object(panel) -> None:
    """Ejecuta pick & place del objeto seleccionado hacia la cesta."""
    setattr(panel, "_pick_object_worker_started", False)

    def _dbg(msg: str) -> None:
        if panel._debug_logs_enabled:
            panel._emit_log(msg)

    def _moveit2_log(scope: str, msg: str) -> None:
        panel._emit_log(f"[MOVEIT2][{scope}] {msg}")

    _step_last_phase = {"value": ""}

    def _step_phase_gate(phase: str, *, position=None, decision: str = "", object_position=None) -> None:
        gate_fn = getattr(panel, "_step_wait_for_phase", None)
        if not callable(gate_fn):
            return
        label = str(phase or "").strip().upper()
        if not label:
            return
        if label.startswith("GRASP_DOWN_MICRO_"):
            label = "GRASP_DOWN"
        elif label.startswith("STRICT_LIFT_STAGE_"):
            label = "LIFT"
        elif label.startswith("TRANSPORT_STAGE_"):
            label = "TRANSPORT"
        elif label == "PRE_GRASP_RECENTER":
            label = "PRE_GRASP"
        if _step_last_phase["value"] == label:
            panel._emit_log(
                f"[PICK][MOVEIT][STEP] gate_reuse phase={label} decision={decision or 'n/a'}"
            )
            return
        _step_last_phase["value"] = label
        panel._emit_log(
            "[PICK][MOVEIT][STEP] "
            f"phase={label} decision={decision or 'n/a'} "
            f"step_mode={str(getattr(panel, '_step_mode', 'AUTO'))}"
        )
        gate_fn(
            f"PICK_OBJ.{label}",
            flow="PICK_OBJECT",
            position=position,
            decision=decision,
            object_position=object_position,
        )

    def _pick_orientation(yaw_deg: Optional[float] = None) -> tuple[float, float, float, float]:
        # F3: usa pick_object/orientation_helpers para la lógica pura.
        from .pick_object.orientation_helpers import (
            compose_grasp_orientation,
            parse_orientation_xyzw,
        )
        base_q = parse_orientation_xyzw(_get_pick_object_params().orientation_xyzw)
        use_yaw = _get_panel_tfm_params().canonical_use_grasp_yaw
        return compose_grasp_orientation(base_q, yaw_deg, use_yaw=use_yaw)

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
        _moveit2_log(
            "BLOCK",
            f"reason={reason} state={state} moveit={moveit_state}:{moveit_reason} "
            f"controllers={ctrl_ok}:{ctrl_reason} lock={lock_active}:{lock_name}:{lock_id}",
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

    canonical_override = getattr(panel, "_pick_object_grasp_override", None)
    canonical_active = isinstance(canonical_override, dict) and bool(canonical_override.get("enabled", False))

    def _canonical_phase(state: str, detail: str = "") -> None:
        if not canonical_active:
            return
        try:
            panel._tfm_canonical_phase_update(state, detail=detail)
        except Exception:
            pass

    def _canonical_finish(success: bool, message: str, final_state: str) -> None:
        if not canonical_active:
            return
        try:
            panel._tfm_canonical_finish(success, message, final_state=final_state)
        except Exception:
            pass

    panel._log_button("PICK Objeto")
    panel._emit_log(
        "[PICK][MOVEIT][BUTTON] "
        f"ts={time.time():.6f} "
        f"selected_ui={str(getattr(panel, '_selected_object', '') or 'none')} "
        f"selection_ts={float(getattr(panel, '_selection_timestamp', 0.0) or 0.0):.3f} "
        "grasp_mode=moveit_pick_object"
    )
    _moveit2_log(
        "BUTTON",
        f"trigger=pick_object selected_ui={str(getattr(panel, '_selected_object', '') or 'none')} "
        f"selection_ts={float(getattr(panel, '_selection_timestamp', 0.0) or 0.0):.3f}",
    )
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
        panel._emit_log(
            "[PICK][MOVEIT][INIT] controllers=FAIL "
            f"reason={reason or 'n/a'}"
        )
        return

    # -- [PICK][MOVEIT][INIT] BLOQUE TF EXPLÍCITO --
    # Equivalente al sync_gate stage=tf_and_ee de DIRECTO.
    # Valida que TF world→base_link y base_link→ee estén disponibles antes de
    # continuar, evitando fallos silenciosos en _selection_to_base() más adelante.
    _moveit_init_tf_ok, _moveit_init_tf_reason = tf_ready_status(panel)
    _moveit_init_ee_frame = str(getattr(panel, "_ee_frame_effective", None) or "").strip()
    _moveit_init_base_frame = str(BASE_FRAME or "base_link")
    _moveit_init_sim_time = bool(getattr(panel, "_use_sim_time", True))
    panel._emit_log(
        "[PICK][MOVEIT][INIT] "
        f"ts={time.time():.6f} mode=moveit "
        f"controllers=OK "
        f"moveit_state={getattr(getattr(panel, '_moveit_state', None), 'value', str(getattr(panel, '_moveit_state', 'n/a')))} "
        f"tf_ok={str(bool(_moveit_init_tf_ok)).lower()} "
        f"tf_reason={_moveit_init_tf_reason or 'ok'} "
        f"ee_frame={_moveit_init_ee_frame or 'none'} "
        f"base_frame={_moveit_init_base_frame} "
        f"use_sim_time={str(_moveit_init_sim_time).lower()}"
    )
    if not _moveit_init_tf_ok or not _moveit_init_ee_frame:
        _block(
            f"tf_or_ee_not_ready:{_moveit_init_tf_reason or 'unknown'}",
            status_text=f"MOVEIT: TF no listo ({_moveit_init_tf_reason or 'ee_frame_missing'})",
            error=True,
        )
        panel._emit_log(
            "[PICK][MOVEIT][INIT] tf_and_ee=FAIL "
            f"tf_ok={str(bool(_moveit_init_tf_ok)).lower()} "
            f"tf_reason={_moveit_init_tf_reason or 'n/a'} "
            f"ee_frame={_moveit_init_ee_frame or 'none'}"
        )
        return
    panel._emit_log(
        "[PICK][MOVEIT][INIT] tf_and_ee=OK "
        f"ee_frame={_moveit_init_ee_frame} base_frame={_moveit_init_base_frame}"
    )
    # -- FIN BLOQUE TF EXPLÍCITO --

    def _world_ready_scope() -> str:
        # V1.1 audit-v4: delega al helper puro pick_object.world_ready_helpers.
        from .pick_object.world_ready_helpers import normalize_world_ready_scope
        raw = _get_pick_object_params().world_ready_scope
        return normalize_world_ready_scope(raw)

    def _world_ready_tracked_names() -> list[str]:
        scope = _world_ready_scope()
        if scope == "all":
            return sorted(set(DROP_NAME_SET) | set(PICK_DEMO_NAME_SET))
        tracked: list[str] = []
        for candidate in (
            str(getattr(panel, "_selected_object", "") or "").strip(),
            str(getattr(panel, "_selection_last_user_name", "") or "").strip(),
        ):
            if candidate and candidate not in tracked:
                tracked.append(candidate)
        if tracked:
            return tracked
        return ["pick_demo"]

    def _world_ready_pending() -> list[str]:
        pending: list[str] = []
        if not bool(getattr(panel, "_pose_info_ok", False)):
            pending.append("pose_info:not_ready")
            return pending
        positions = get_object_positions() or {}
        tracked_names = _world_ready_tracked_names()
        for name in tracked_names:
            pose = positions.get(name)
            if pose is None or len(pose) < 3:
                pending.append(f"{name}:missing")
                continue
            try:
                xyz = (float(pose[0]), float(pose[1]), float(pose[2]))
            except Exception:
                pending.append(f"{name}:invalid")
                continue
            if not is_on_table(xyz):
                pending.append(f"{name}:z={xyz[2]:.3f}")
        return pending

    def _wait_for_world_ready() -> bool:
        try:
            wait_sec = _get_pick_object_params().world_ready_wait_sec
        except Exception:
            wait_sec = 20.0
        try:
            poll_sec = _get_pick_object_params().world_ready_poll_sec
        except Exception:
            poll_sec = 0.25
        wait_sec = max(1.0, wait_sec)
        poll_sec = max(0.1, poll_sec)
        deadline = time.time() + wait_sec
        last_log_ts = 0.0
        last_summary = ""
        while time.time() < deadline:
            pending = _world_ready_pending()
            if not pending:
                tracked_names = _world_ready_tracked_names()
                panel._emit_log(
                    "[PICK_OBJ][WORLD_READY] ready "
                    f"scope={_world_ready_scope()} tracked={','.join(tracked_names)}"
                )
                return True
            now = time.time()
            summary = ",".join(pending[:4])
            if summary != last_summary or (now - last_log_ts) >= 1.5:
                panel._emit_log(
                    "[PICK_OBJ][WORLD_READY] waiting "
                    f"pending={summary} total_pending={len(pending)}"
                )
                last_summary = summary
                last_log_ts = now
            panel._wait_for_state_change(poll_sec)
        pending = _world_ready_pending()
        panel._emit_log(
            "[PICK_OBJ][WORLD_READY] timeout "
            f"pending={','.join(pending[:6]) or 'unknown'} total_pending={len(pending)}"
        )
        return False

    if not _wait_for_world_ready():
        _block(
            "world_not_ready",
            status_text="PICK Objeto: escena Gazebo no lista (objetos fuera de mesa)",
            error=True,
        )
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
        restored_missing_store = False
        restore_candidate = ui_selected if ui_selected == user_selected else ""
        if restore_candidate and hasattr(panel, "_ensure_selected_object_in_store"):
            try:
                restored_missing_store = bool(
                    panel._ensure_selected_object_in_store(
                        restore_candidate,
                        reason="pick_object_restore_missing_store",
                    )
                )
            except Exception as exc:
                panel._emit_log(
                    "[PICK_OBJ][SELECT_RESTORE] error "
                    f"name={restore_candidate} err={type(exc).__name__}:{exc}"
                )
            if restored_missing_store:
                selected_states = [
                    (name, st)
                    for name, st in get_object_states().items()
                    if st.logical_state == ObjectLogicalState.SELECTED
                ]
                selected_states.sort(key=lambda item: float(item[1].last_update_ts), reverse=True)
                latest_store_name = selected_states[0][0] if selected_states else ""
                latest_store_ts = (
                    float(getattr(selected_states[0][1], "last_update_ts", 0.0) or 0.0)
                    if selected_states
                    else 0.0
                )
                panel._emit_log(
                    "[PICK_OBJ][SELECT_RESTORE] restored_missing_store "
                    f"name={restore_candidate} store_selected={latest_store_name or 'none'} "
                    f"store_ts={latest_store_ts:.3f}"
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
        _get_pick_object_params().selection_max_age_sec,
    )
    if (not user_selected or user_selected_ts <= 0.0) and ui_selected and latest_store_name:
        if ui_selected == latest_store_name:
            recovered_ts = max(ui_selected_ts, latest_store_ts)
            if recovered_ts <= 0.0:
                recovered_ts = time.time()
            panel._selection_last_user_name = ui_selected
            panel._selection_last_user_ts = recovered_ts
            user_selected = ui_selected
            user_selected_ts = recovered_ts
            panel._emit_log(
                "[PICK][SELECT_RESTORE] restored_user_from_effective_selection "
                f"name={user_selected} ts={user_selected_ts:.3f}"
            )
    if not user_selected or user_selected_ts <= 0.0:
        grasp_world = getattr(panel, "_last_grasp_world", None)
        grasp_x = grasp_y = None
        if isinstance(grasp_world, dict):
            try:
                grasp_x = float(grasp_world.get("x", 0.0))
                grasp_y = float(grasp_world.get("y", 0.0))
            except Exception:
                grasp_x = grasp_y = None
        if grasp_x is not None and grasp_y is not None:
            candidate = str(nearest_table_object(grasp_x, grasp_y) or "").strip()
            positions = get_object_positions() or {}
            pose = positions.get(candidate)
            if candidate and pose is not None and len(pose) >= 3:
                try:
                    px, py, pz = float(pose[0]), float(pose[1]), float(pose[2])
                except Exception:
                    px = py = pz = 0.0
                if is_on_table((px, py, pz)) and hasattr(panel, "_select_object"):
                    try:
                        panel._select_object(
                            candidate,
                            -1,
                            -1,
                            px,
                            py,
                            pz,
                            source="pick_object_restore_from_last_grasp",
                        )
                    except Exception as exc:
                        panel._emit_log(
                            "[PICK_OBJ][SELECT_RESTORE] error "
                            f"source=last_grasp name={candidate} err={type(exc).__name__}:{exc}"
                        )
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
                    latest_store_ts = (
                        float(getattr(selected_states[0][1], "last_update_ts", 0.0) or 0.0)
                        if selected_states
                        else 0.0
                    )
                    if user_selected and user_selected_ts > 0.0:
                        panel._emit_log(
                            "[PICK][SELECT_RESTORE] restored_from_last_grasp "
                            f"name={user_selected} ui_selected={ui_selected or 'none'} "
                            f"store_selected={latest_store_name or 'none'}"
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
    if snapshot_name not in DROP_NAME_SET and snapshot_name not in PICK_DEMO_NAME_SET:
        _block(
            "selection_not_drop_object",
            status_text="PICK Objeto: seleccion actual no corresponde a un objeto pickable",
            error=True,
        )
        panel._emit_log(
            "[PICK_OBJ][ABORT] selection_not_drop_object "
            f"name={snapshot_name or 'none'} allowed=DROP|PICK_DEMO"
        )
        return
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
    panel._emit_log(
        "[PICK][MOVEIT][SELECT] "
        f"selected_object={snapshot_name or 'none'} "
        f"ui_selected={ui_selected or 'none'} store_selected={latest_store_name or 'none'} "
        f"user_selected={user_selected or 'none'} "
        f"ui_ts={ui_selected_ts:.3f} store_ts={latest_store_ts:.3f} user_ts={user_selected_ts:.3f}"
    )
    _moveit2_log(
        "SELECT",
        f"object={snapshot_name or 'none'} ui={ui_selected or 'none'} store={latest_store_name or 'none'} "
        f"user={user_selected or 'none'} ui_ts={ui_selected_ts:.3f} store_ts={latest_store_ts:.3f}",
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
    from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params
    if _get_panel_ui_params().debug_pick_obj:
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
    panel._emit_log(
        "[PICK][MOVEIT][LIVE_OBJECT] "
        f"name={obj_name} source={pose_source} "
        f"world_frame={world_frame} world_pose={fmt_vec3((obj_x, obj_y, obj_z))} "
        f"base_frame={base_frame} base_pose={fmt_vec3((bx, by, bz))} "
        f"selected_ts={selected_ts:.3f} obj_base_stamp_ns={obj_base_stamp_ns}"
    )
    grasp_yaw_deg: Optional[float] = None
    if canonical_active:
        override_selected = str(canonical_override.get("selected_object", "") or "").strip()
        override_frame = str(canonical_override.get("frame", "") or base_frame).strip() or base_frame
        try:
            yaw_raw = canonical_override.get("yaw_deg", None)
            if yaw_raw is not None and str(yaw_raw).strip() != "":
                grasp_yaw_deg = float(yaw_raw)
        except Exception:
            grasp_yaw_deg = None
        if override_selected and override_selected != obj_name:
            _block(
                "canonical_selection_mismatch",
                status_text="TFM->MoveIt: selección/grasp inconsistentes",
                error=True,
            )
            panel._emit_log(
                "[PICK_OBJ][TFM_CANON] selection_mismatch "
                f"selected={obj_name} override_selected={override_selected}"
            )
            _canonical_finish(False, "canonical_selection_mismatch", "FAIL_TERMINAL")
            return
        if override_frame != base_frame:
            _block(
                "canonical_frame_mismatch",
                status_text="TFM->MoveIt: frame de grasp inconsistente",
                error=True,
            )
            panel._emit_log(
                "[PICK_OBJ][TFM_CANON] frame_mismatch "
                f"override_frame={override_frame} base_frame={base_frame}"
            )
            _canonical_finish(False, "canonical_frame_mismatch", "FAIL_TERMINAL")
            return
        try:
            max_override_delta = _get_panel_tfm_params().canonical_grasp_xy_max_delta_m
        except Exception:
            max_override_delta = 0.080
        snap_selected_xy = _get_panel_tfm_params().canonical_snap_selected_xy
        override_x_raw = float(canonical_override.get("x", bx) or bx)
        override_y_raw = float(canonical_override.get("y", by) or by)
        override_x = float(override_x_raw)
        override_y = float(override_y_raw)
        override_dxy = math.hypot(override_x - float(bx), override_y - float(by))
        if override_dxy > max_override_delta:
            if not snap_selected_xy:
                _block(
                    "canonical_grasp_xy_out_of_gate",
                    status_text="TFM->MoveIt: grasp fuera del objeto seleccionado",
                    error=True,
                )
                panel._emit_log(
                    "[PICK_OBJ][TFM_CANON] grasp_xy_out_of_gate "
                    f"selected_xy=({float(bx):.3f},{float(by):.3f}) "
                    f"grasp_xy=({override_x:.3f},{override_y:.3f}) "
                    f"dxy={override_dxy:.3f} max={max_override_delta:.3f}"
                )
                _canonical_finish(False, "canonical_grasp_xy_out_of_gate", "FAIL_TERMINAL")
                return
            panel._emit_log(
                "[PICK_OBJ][TFM_CANON] grasp_xy_out_of_gate_snap "
                f"selected_xy=({float(bx):.3f},{float(by):.3f}) "
                f"grasp_xy_raw=({override_x_raw:.3f},{override_y_raw:.3f}) "
                f"dxy={override_dxy:.3f} max={max_override_delta:.3f} "
                "action=use_selected_object_xy"
            )
            override_x = float(bx)
            override_y = float(by)
        yaw_txt = "n/a" if grasp_yaw_deg is None else f"{float(grasp_yaw_deg):.1f}"
        panel._emit_log(
            "[PICK_OBJ][TFM_CANON] override_xy "
            f"selected_xy=({float(bx):.3f},{float(by):.3f}) "
            f"grasp_xy_raw=({override_x_raw:.3f},{override_y_raw:.3f}) "
            f"grasp_xy_exec=({override_x:.3f},{override_y:.3f}) "
            f"dxy={override_dxy:.3f} yaw={yaw_txt}"
        )
        bx = override_x
        by = override_y
        pose_source = "tfm_grasp_override"
    # Referencia congelada de ciclo MoveIt2: equivalente explícito al patrón CYCLE_REF de Directo.
    # bx/by ya incluyen el canonical override si estaba activo. Se asigna UNA SOLA VEZ.
    _moveit_cycle_object_world = (float(obj_x), float(obj_y), float(obj_z))
    _moveit_cycle_object_base = (float(bx), float(by), float(bz))
    _moveit_cycle_object_source = str(pose_source)
    panel._emit_log(
        "[MOVEIT2][CYCLE_REF][SELECT] "
        f"phase=BUTTON_PRESS object={obj_name} "
        f"source={_moveit_cycle_object_source} "
        f"world=({obj_x:.3f},{obj_y:.3f},{obj_z:.3f}) "
        f"base=({bx:.3f},{by:.3f},{bz:.3f}) "
        f"canonical_override={str(canonical_active).lower()}"
    )
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
        z_clear = _get_pick_object_params().approach_clearance_m
    except Exception:
        z_clear = 0.20
    try:
        z_min_approach_clearance = _get_pick_object_params().min_approach_clearance_m
    except Exception:
        z_min_approach_clearance = 0.30
    z_min_approach_clearance = max(0.0, z_min_approach_clearance)
    try:
        z_pre_margin = _get_pick_object_params().pre_margin_m
    except Exception:
        z_pre_margin = 0.04
    try:
        z_min_pre_clearance = _get_pick_object_params().min_pre_clearance_m
    except Exception:
        z_min_pre_clearance = 0.15
    z_min_pre_clearance = max(0.0, z_min_pre_clearance)
    # Legacy offset kept for compatibility; new contact target uses contact_down_z.
    try:
        z_insert = _get_pick_object_params().insert_m
    except Exception:
        z_insert = 0.015
    # F2-step1: contact_down_z con doble alias (CONTACT_DOWN_Z_M canonical >
    # EXTRA_DOWN_Z legacy > default = max(0.020, PICK_DEMO_GRASP_Z_OFFSET)).
    _po_params = _get_pick_object_params()
    try:
        if _po_params.contact_down_z_m is not None:
            contact_down_z = float(_po_params.contact_down_z_m)
        elif _po_params.extra_down_z_m is not None:
            contact_down_z = float(_po_params.extra_down_z_m)
        else:
            contact_down_z = max(0.020, float(PICK_DEMO_GRASP_Z_OFFSET))
    except Exception:
        contact_down_z = 0.020
    if contact_down_z < 0.0:
        contact_down_z = 0.0
    try:
        z_table_margin = _get_pick_object_params().table_margin_m
    except Exception:
        z_table_margin = 0.015
    try:
        z_lift_clearance = _get_pick_object_params().lift_clearance_m
    except Exception:
        z_lift_clearance = 0.20
    try:
        z_transport_clearance = _get_pick_object_params().transport_clearance_m
    except Exception:
        z_transport_clearance = 0.20
    pick_contact_z_offset = contact_z_correction_for_frame(
        RG2_PINCH_CENTER_FRAME,
        geometry=_PICK_OBJECT_GRIPPER_GEOMETRY,
    )

    height_profile = compute_pick_height_profile(
        object_center_z=bz,
        object_height=obj_height,
        table_top_z=table_top_base,
        basket_z=basket_z,
        tcp_z_offset=float(pick_contact_z_offset),
        pick_demo_grasp_z_offset=float(PICK_DEMO_GRASP_Z_OFFSET),
        pick_demo_drop_z_offset=float(PICK_DEMO_DROP_Z_OFFSET),
        approach_clearance=z_clear,
        min_approach_clearance=z_min_approach_clearance,
        pre_margin=z_pre_margin,
        min_pre_clearance=z_min_pre_clearance,
        insert_z=z_insert,
        contact_down_z=contact_down_z,
        table_margin=z_table_margin,
        lift_clearance=z_lift_clearance,
        transport_clearance=z_transport_clearance,
    )
    z_approach = height_profile.approach_z
    z_pre = height_profile.pre_grasp_z
    legacy_grasp_z = height_profile.legacy_grasp_z
    safe_tcp_floor_z = height_profile.safe_tcp_floor_z
    contact_target_z = height_profile.contact_target_z
    z_grasp = height_profile.grasp_z
    z_lift = height_profile.lift_z
    z_drop = basket_z + PICK_DEMO_DROP_Z_OFFSET
    z_transport = height_profile.transport_z

    pose_frame = base_frame
    approach_pose = (bx, by, z_approach)
    pre_grasp_pose = (bx, by, z_pre)
    grasp_down_pose = (bx, by, z_grasp)
    lift_pose = (bx, by, z_lift)
    transport_pose = (basket_x, basket_y, z_transport)
    drop_pose = (basket_x, basket_y, z_drop)
    try:
        transport_split_stages = _get_pick_object_params().transport_split_stages
    except Exception:
        transport_split_stages = 3
    transport_split_stages = max(2, min(transport_split_stages, 6))
    transport_stage_poses: List[tuple] = []
    for stage_idx in range(1, transport_split_stages + 1):
        frac = float(stage_idx) / float(transport_split_stages)
        transport_stage_poses.append(
            (
                float(lift_pose[0]) + (float(transport_pose[0]) - float(lift_pose[0])) * frac,
                float(lift_pose[1]) + (float(transport_pose[1]) - float(lift_pose[1])) * frac,
                float(lift_pose[2]) + (float(transport_pose[2]) - float(lift_pose[2])) * frac,
            )
        )
    pick_orientation = _pick_orientation(grasp_yaw_deg if canonical_active else None)

    _dbg(
        f"[PICK_OBJ_DEBUG] BASE: APPROACH=({approach_pose[0]:.3f}, {approach_pose[1]:.3f}, {approach_pose[2]:.3f}), "
        f"PRE_GRASP=({pre_grasp_pose[0]:.3f}, {pre_grasp_pose[1]:.3f}, {pre_grasp_pose[2]:.3f}), "
        f"GRASP_DOWN=({grasp_down_pose[0]:.3f}, {grasp_down_pose[1]:.3f}, {grasp_down_pose[2]:.3f}) "
        f"LIFT=({lift_pose[0]:.3f}, {lift_pose[1]:.3f}, {lift_pose[2]:.3f}) "
        f"TRANSPORT_SPLIT_STAGES={transport_split_stages} "
        f"TRANSPORT=({transport_pose[0]:.3f}, {transport_pose[1]:.3f}, {transport_pose[2]:.3f}) "
        f"DROP=({drop_pose[0]:.3f}, {drop_pose[1]:.3f}, {drop_pose[2]:.3f}) "
        f"(obj_height={obj_height:.3f} table_top_base={table_top_base:.3f})"
    )
    _dbg(
        f"[PICK_OBJ_DEBUG] height={height} table_top_world={table_top:.3f}"
    )
    panel._emit_log(
        "[PICK_OBJ][ORIENTATION] "
        f"xyzw=({pick_orientation[0]:.4f},{pick_orientation[1]:.4f},"
        f"{pick_orientation[2]:.4f},{pick_orientation[3]:.4f}) "
        "source=env_or_default"
    )

    # Optional micro-descent sequence before GRASP_DOWN final.
    try:
        grasp_micro_step_m = _get_pick_object_params().grasp_micro_step_m
    except Exception:
        grasp_micro_step_m = 0.012
    try:
        grasp_micro_steps_max = _get_pick_object_params().grasp_micro_steps_max
    except Exception:
        grasp_micro_steps_max = 0
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
        ("APPROACH", _make_pose_data(approach_pose, orientation=pick_orientation, frame=pose_frame), 0.4),
        ("PRE_GRASP", _make_pose_data(pre_grasp_pose, orientation=pick_orientation, frame=pose_frame), 0.5),
        ("GRASP_DOWN", _make_pose_data(grasp_down_pose, orientation=pick_orientation, frame=pose_frame), 0.7),
        ("LIFT", _make_pose_data(lift_pose, orientation=pick_orientation, frame=pose_frame), 0.5),
        ("TRANSPORT", _make_pose_data(transport_pose, orientation=pick_orientation, frame=pose_frame), 0.6),
        ("DROP", _make_pose_data(drop_pose, orientation=pick_orientation, frame=pose_frame), 0.8),
    ]
    transport_stage_data = [
        _make_pose_data(stage_pose, orientation=pick_orientation, frame=pose_frame)
        for stage_pose in transport_stage_poses
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
    _moveit2_log(
        "TARGET_LOCK",
        f"activate lock_id={target_lock_id} name={obj_name} source={pose_source} "
        f"base=({bx:.3f},{by:.3f},{bz:.3f}) frame={base_frame}",
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
        f"tcp_z_offset={float(pick_contact_z_offset):.3f} "
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
        # El bridge publica ee_link="rg2_tcp" en todos sus resultados.
        # _ee_frame_effective es "rg2_pinch_center" en runtime (orden de preferencia TF).
        # Ambos frames son coubicados (Z=0.175, distance=0.0); usar RG2_TCP_FRAME
        # en modo MOVEIT garantiza que el check de mismatch (línea ~3486) pase.
        _raw_ee = str(panel._ee_frame_effective or RG2_TCP_FRAME).strip()
        measured_ee_frame = (
            RG2_TCP_FRAME
            if _raw_ee in (RG2_TCP_FRAME, RG2_PINCH_CENTER_FRAME)
            else _raw_ee
        )
        panel._emit_log(
            "[PICK][MOVEIT][LIFECYCLE] stage=worker_start "
            f"obj={obj_name} ee_frame={measured_ee_frame} "
            f"base_frame={base_frame} "
            f"world=({float(obj_x):.3f},{float(obj_y):.3f},{float(obj_z):.3f}) "
            f"base=({float(bx):.3f},{float(by):.3f},{float(bz):.3f}) "
            f"moveit_result_topic={moveit_result_topic}"
        )
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
            return _norm_frame_pure(frame_name)

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
            allow_helper_fallback = _get_pick_object_params().allow_tf_helper_fallback
            if not allow_helper_fallback:
                return None, None
            return transform_point_to_frame(
                (0.0, 0.0, 0.0),
                target_frame,
                source_frame=ee_frame,
                timeout_sec=timeout_sec,
            )

        def _log_moveit_panel_trace(label: str, *, current_obj_base: Optional[tuple] = None) -> None:
            panel_fk_tcp = getattr(panel, "_last_tcp_base", None)
            panel_fk_rpy = getattr(panel, "_last_tcp_rpy_deg", None)
            live_tcp = getattr(panel, "_last_trace_tcp_base", None)
            live_rpy = getattr(panel, "_last_trace_tcp_rpy_deg", None)
            object_age = getattr(panel, "_last_trace_object_age_sec", None)
            panel._emit_log(
                "[PICK][MOVEIT][PANEL_TRACE] "
                f"label={label} "
                f"selected_object={str(getattr(panel, '_selected_object', '') or 'none')} "
                f"object_pose_base={fmt_vec3(current_obj_base)} "
                f"tcp_panel_fk_base={fmt_vec3(panel_fk_tcp)} "
                f"tcp_panel_fk_rpy_deg={fmt_vec3(panel_fk_rpy)} "
                f"tcp_live_base={fmt_vec3(live_tcp)} "
                f"tcp_live_rpy_deg={fmt_vec3(live_rpy)} "
                f"panel_fk_age_sec={fmt_age_sec(max(0.0, time.monotonic() - float(getattr(panel, '_last_tcp_fk_ts', time.monotonic()) or time.monotonic())))} "
                f"tcp_live_age_sec={fmt_age_sec(max(0.0, time.monotonic() - float(getattr(panel, '_last_trace_tcp_ts', time.monotonic()) or time.monotonic())))} "
                f"object_age_sec={fmt_age_sec(object_age)}"
            )
            if (
                isinstance(panel_fk_tcp, (list, tuple))
                and len(panel_fk_tcp) >= 3
                and isinstance(live_tcp, (list, tuple))
                and len(live_tcp) >= 3
            ):
                ddx = float(panel_fk_tcp[0]) - float(live_tcp[0])
                ddy = float(panel_fk_tcp[1]) - float(live_tcp[1])
                ddz = float(panel_fk_tcp[2]) - float(live_tcp[2])
                dist = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
                if dist > 0.02:
                    panel._emit_log(
                        "[PICK][MOVEIT][DIVERGENCE] "
                        "kind=panel_fk_vs_tf_live "
                        f"label={label} delta_m={dist:.3f} "
                        f"delta_vec=({ddx:.3f},{ddy:.3f},{ddz:.3f}) "
                        "note=panel_fk_model_pose_not_same_as_live_rg2_tcp"
                    )

        def _resolve_live_object_pose_for_audit(name: str) -> tuple[Optional[tuple], Optional[tuple], str]:
            try:
                live_positions = get_object_positions() or {}
                live_world = live_positions.get(name)
            except Exception:
                live_world = None
            if live_world is None or len(live_world) < 3:
                return None, None, "pose_info/live_missing"
            try:
                live_world_xyz = (
                    float(live_world[0]),
                    float(live_world[1]),
                    float(live_world[2]),
                )
            except Exception:
                return None, None, "pose_info/live_invalid"
            live_base_info = panel._selection_to_base(live_world_xyz, world_frame)
            if not live_base_info:
                return live_world_xyz, None, "pose_info/live_tf_failed"
            try:
                live_base_xyz = tuple(float(v) for v in live_base_info["coords"])
            except Exception:
                live_base_xyz = None
            return live_world_xyz, live_base_xyz, "pose_info/live"

        def _tf_stamp_ns(tf_msg: Optional[object]) -> int:
            return _tf_stamp_ns_pure(tf_msg)

        def _ros_clock_now_ns() -> int:
            # Use live ROS clock first; cached /clock is only a fallback.
            # This avoids stale cache values inflating TF age diagnostics.
            _sanitize_ros_ns = _sanitize_ros_ns_pure

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

            def _read_bridge_topics() -> tuple[int, int, int, float, bool]:
                pose_subs = panel.ros_worker.topic_subscriber_count(topic)
                res_pubs = panel.ros_worker.topic_publisher_count(moveit_result_topic)
                res_subs = panel.ros_worker.topic_subscriber_count(moveit_result_topic)
                hb_age_local = panel.ros_worker.moveit_bridge_heartbeat_age()
                hb_recent_local = panel.ros_worker.has_recent_moveit_bridge_heartbeat(1.2)
                return pose_subs, res_pubs, res_subs, hb_age_local, hb_recent_local

            def _wait_bridge_match(
                *,
                timeout_sec: float,
                log_tag: str,
                require_heartbeat: bool = False,
            ) -> tuple[bool, int, int, int, float, bool]:
                deadline = time.time() + max(0.5, float(timeout_sec))
                while time.time() < deadline:
                    if not panel._ros_worker_started:
                        panel._ensure_ros_worker_started()
                    if panel._ros_worker_started and panel.ros_worker.node_ready():
                        pose_subs, result_pubs, result_subs_local, hb_age_local, hb_recent_local = _read_bridge_topics()
                        hb_age_local_txt = "inf" if math.isinf(hb_age_local) else f"{hb_age_local:.2f}s"
                        panel._emit_log(
                            f"[PICK_OBJ][MOVEIT][PLAN] {log_tag} "
                            f"pose_subs={pose_subs} result_pubs={result_pubs} result_subs={result_subs_local} "
                            f"hb_recent={str(bool(hb_recent_local)).lower()} hb_age={hb_age_local_txt} "
                            f"require_heartbeat={str(bool(require_heartbeat)).lower()}"
                        )
                        if pose_subs > 0 and result_pubs > 0 and (
                            not require_heartbeat or bool(hb_recent_local)
                        ):
                            return True, pose_subs, result_pubs, result_subs_local, hb_age_local, hb_recent_local
                    time.sleep(0.4)
                pose_subs = panel.ros_worker.topic_subscriber_count(topic)
                result_pubs = panel.ros_worker.topic_publisher_count(moveit_result_topic)
                result_subs_local = panel.ros_worker.topic_subscriber_count(moveit_result_topic)
                hb_age_local = panel.ros_worker.moveit_bridge_heartbeat_age()
                hb_recent_local = panel.ros_worker.has_recent_moveit_bridge_heartbeat(1.2)
                return False, pose_subs, result_pubs, result_subs_local, hb_age_local, hb_recent_local

            def _restart_moveit_bridge_and_wait(*, cold_start: bool) -> None:
                if cold_start:
                    panel._emit_log(
                        "[PICK_OBJ][MOVEIT][PLAN] bridge_cold_start "
                        "node=ur5_moveit_bridge pose_topic=/desired_grasp result_topic=/desired_grasp/result"
                    )
                else:
                    panel._emit_log(
                        "[PICK_OBJ][MOVEIT][PLAN] bridge_recover restarting "
                        "node=ur5_moveit_bridge pose_topic=/desired_grasp result_topic=/desired_grasp/result"
                    )
                    _run_ui_sync(lambda: panel._stop_moveit_bridge(), timeout_sec=2.5)
                    time.sleep(0.2)
                _run_ui_sync(lambda: panel._start_moveit_bridge(), timeout_sec=3.0)
                try:
                    recover_timeout_sec = _get_pick_object_params().moveit_bridge_recover_timeout_sec
                except Exception:
                    recover_timeout_sec = 10.0
                matched, *_diag = _wait_bridge_match(
                    timeout_sec=max(6.0, float(recover_timeout_sec)),
                    log_tag="bridge_recover_check",
                    require_heartbeat=True,
                )
                if matched:
                    return
                raise RuntimeError("MoveItBridge no recuperado tras reinicio (pose_subs/result_pubs)")

            # F3-step5bis-c: cuerpo principal (~100 LOC) extraído a
            # pick_object/moveit_bridge_path.py. Las 4 nested defs
            # permanecen aquí porque sus kwargs homónimos colisionarían
            # con free vars del closure padre. Se inyectan al ctx como
            # callables.
            _mbp_ctx = MoveItBridgePathContext(
                panel=panel,
                moveit_hb_topic=moveit_hb_topic,
                moveit_result_topic=moveit_result_topic,
                read_bridge_topics_fn=_read_bridge_topics,
                wait_bridge_match_fn=_wait_bridge_match,
                restart_moveit_bridge_and_wait_fn=_restart_moveit_bridge_and_wait,
            )
            return _ensure_moveit_bridge_path_main_body_pure(_mbp_ctx, topic)

        # F3-step4a: cuerpo de _wait_moveit_result extraído a
        # pick_object/wait_moveit_result.py. El wrapper construye
        # WaitMoveItResultContext y delega. Los 14 callsites legacy NO
        # cambian (firma idéntica).
        def _wait_moveit_result(label: str, since_wall: float, *, timeout_total: float = 12.0) -> dict:
            _wmr_ctx = WaitMoveItResultContext(
                panel=panel,
                moveit_result_topic=moveit_result_topic,
            )
            return _wait_moveit_result_pure(
                _wmr_ctx, label, since_wall, timeout_total=timeout_total,
            )


        # F3-step4b: cuerpo de _tf_distance_check extraído a
        # pick_object/tf_distance_check.py. Wrapper de 11 LOC.
        def _tf_distance_check(*, label, pose_data, tol_m, ee_frame):
            _tdc_ctx = TfDistanceCheckContext(
                panel=panel,
                base_frame=BASE_FRAME or "base_link",
                world_frame=WORLD_FRAME or "world",
            )
            return _tf_distance_check_pure(
                _tdc_ctx, label=label, pose_data=pose_data,
                tol_m=tol_m, ee_frame=ee_frame,
            )


        try:
            panel._emit_log("[PICK_OBJ] === WORKER INICIADO ===")
            move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
            home_pose = panel._get_home_joint_pose()
            _dbg(f"[PICK_OBJ] HOME pose: {[f'{math.degrees(j):.1f}°' for j in home_pose]}")
            moveit_exclusive = _get_pick_object_params().moveit_exclusive
            allow_joint_recovery_in_moveit = _get_pick_object_params().allow_joint_recovery_in_moveit
            allow_joint_preflight_in_moveit = _get_pick_object_params().allow_joint_preflight_in_moveit
            panel._emit_log(
                f"[PICK_OBJ][MODE] moveit_exclusive={str(moveit_exclusive).lower()}"
            )
            if allow_joint_preflight_in_moveit:
                panel._emit_log(
                    "[PICK_OBJ][MODE] allow_joint_preflight_in_moveit=true"
                )
            if allow_joint_recovery_in_moveit:
                panel._emit_log(
                    "[PICK_OBJ][MODE] allow_joint_recovery_in_moveit=true"
                )
            transport_fallback_enabled = str(
                os.environ.get(
                    "PANEL_PICK_OBJECT_TRANSPORT_JOINT_FALLBACK",
                    "0" if moveit_exclusive else "1",
                )
            ).strip().lower() not in ("0", "false", "no", "off")
            transport_split_moveit = _get_pick_object_params().transport_split_moveit
            approach_fallback_enabled = str(
                os.environ.get(
                    "PANEL_PICK_OBJECT_APPROACH_JOINT_FALLBACK",
                    "0" if moveit_exclusive else "1",
                )
            ).strip().lower() not in ("0", "false", "no", "off")
            pre_grasp_fallback_enabled = str(
                os.environ.get(
                    "PANEL_PICK_OBJECT_PRE_GRASP_JOINT_FALLBACK",
                    "1" if moveit_exclusive else "1",
                )
            ).strip().lower() not in ("0", "false", "no", "off")
            if moveit_exclusive and not allow_joint_recovery_in_moveit:
                if approach_fallback_enabled:
                    panel._emit_log(
                        "[PICK_OBJ][MODE] moveit_exclusive=true; desactivando APPROACH_JOINT_FALLBACK"
                    )
                if pre_grasp_fallback_enabled:
                    panel._emit_log(
                        "[PICK_OBJ][MODE] moveit_exclusive=true; desactivando PRE_GRASP_JOINT_FALLBACK"
                    )
                approach_fallback_enabled = False
                pre_grasp_fallback_enabled = False
            elif moveit_exclusive and (approach_fallback_enabled or pre_grasp_fallback_enabled):
                panel._emit_log(
                    "[PICK_OBJ][MODE] moveit_exclusive=true; recovery_fallbacks_permitidos "
                    f"approach={str(bool(approach_fallback_enabled)).lower()} "
                    f"pre_grasp={str(bool(pre_grasp_fallback_enabled)).lower()}"
                )
            deterministic_joint_after_approach = _get_pick_object_params().deterministic_joint_after_approach
            try:
                deterministic_carry_gate_max_m = _get_pick_object_params().deterministic_carry_gate_max_m
            except Exception:
                deterministic_carry_gate_max_m = 0.80
            try:
                deterministic_carry_gate_min_consecutive = _get_pick_object_params().deterministic_carry_gate_min_consecutive
            except Exception:
                deterministic_carry_gate_min_consecutive = 1
            panel._emit_log(
                "[PICK_OBJ][MODE] deterministic_joint_after_approach="
                f"{str(deterministic_joint_after_approach).lower()}"
            )
            # F2-step5 (audit-v4): env read movida a panel_env.is_strict_physics_mode.
            from .panel_env import is_strict_physics_mode as _is_strict_physics_mode
            strict_physics_mode = _is_strict_physics_mode()
            try:
                carry_joint_move_sec = _get_pick_object_params().carry_joint_time_sec
            except Exception:
                carry_joint_move_sec = 8.0
            carry_joint_move_sec = max(move_sec, carry_joint_move_sec)

            _is_step_tf_mismatch = _is_step_tf_mismatch_pure
            _is_step_exec_failed = _is_step_exec_failed_pure

            def _run_joint_step(
                label: str,
                joints: List[float],
                timeout_sec: Optional[float] = None,
                tol_rad: float = 0.02,
                duration_sec: Optional[float] = None,
                require_reached: bool = False,
            ) -> None:
                _log_selection_changed_during_pick()
                motion_sec = max(0.5, float(duration_sec or move_sec))
                panel._emit_log(
                    f"[PICK_OBJ] {label}: enviando joints {[f'{math.degrees(j):.1f}°' for j in joints]} "
                    f"dur={motion_sec:.2f}s"
                )
                ok, info = panel._publish_joint_trajectory(joints, motion_sec)
                if not ok:
                    raise RuntimeError(f"{label} fallo: {info}")
                wait_timeout = motion_sec + 2.0 if timeout_sec is None else timeout_sec
                reached = panel._wait_for_joint_target(
                    joints,
                    wait_timeout,
                    tol_rad=tol_rad,
                )
                if not reached and len(joints) == 6:
                    current_joints = _read_current_arm_joint_positions()
                    if current_joints is not None:
                        settle_err = _max_joint_error(current_joints, joints)
                        if settle_err <= tol_rad:
                            panel._emit_log(
                                f"[PICK_OBJ][JOINT] {label} accept_after_final_read "
                                f"max_err={settle_err:.4f}rad tol={tol_rad:.4f}rad"
                            )
                            reached = True
                    if not reached:
                        try:
                            settle_extra_sec = _get_pick_object_params().joint_settle_extra_sec
                        except Exception:
                            settle_extra_sec = 2.5
                        settle_extra_sec = max(0.0, settle_extra_sec)
                        if settle_extra_sec > 0.0:
                            time.sleep(settle_extra_sec)
                            current_joints = _read_current_arm_joint_positions()
                            if current_joints is not None:
                                settle_err = _max_joint_error(current_joints, joints)
                                if settle_err <= tol_rad:
                                    panel._emit_log(
                                        f"[PICK_OBJ][JOINT] {label} accept_after_settle "
                                        f"extra={settle_extra_sec:.2f}s "
                                        f"max_err={settle_err:.4f}rad tol={tol_rad:.4f}rad"
                                    )
                                    reached = True
                if not reached:
                    msg = f"{label} no alcanzado exactamente"
                    if require_reached:
                        raise RuntimeError(msg)
                    panel._emit_log(f"[PICK_OBJ] WARN: {msg}")

            def _read_current_arm_joint_positions() -> Optional[List[float]]:
                joint_map = {}
                if panel.ros_worker is not None:
                    try:
                        payload, payload_wall = panel.ros_worker.get_last_joint_state()
                    except Exception:
                        payload, payload_wall = None, 0.0
                    if payload and payload_wall > 0.0 and (time.time() - float(payload_wall)) <= 1.0:
                        try:
                            names = list(payload.get("name", []))
                            positions = list(payload.get("position", []))
                        except Exception:
                            names, positions = [], []
                        for name, pos in zip(names, positions):
                            try:
                                joint_map[str(name).strip()] = float(pos)
                            except Exception:
                                continue
                if not joint_map:
                    last_ts = float(getattr(panel, "_last_joint_time", 0.0) or 0.0)
                    if last_ts > 0.0 and (time.time() - last_ts) <= 1.0:
                        try:
                            joint_map = {
                                str(name).strip(): float(pos)
                                for name, pos in dict(getattr(panel, "_last_joint_positions", {})).items()
                            }
                        except Exception:
                            joint_map = {}
                if not joint_map:
                    return None
                arm_joint_names = (
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                )
                out: List[float] = []
                for joint_name in arm_joint_names:
                    if joint_name not in joint_map:
                        return None
                    out.append(float(joint_map[joint_name]))
                return out

            _max_joint_error = _max_joint_error_pure

            def _ensure_home_start() -> None:
                force_home_start = _get_pick_object_params().force_home_start
                preflight_mode_now = str(
                    _get_pick_object_params().preflight_mode
                ).strip().lower()
                home_like_preflight = preflight_mode_now in (
                    "home",
                    "home_only",
                    "direct",
                    "default",
                    "none",
                    "off",
                    "0",
                    "false",
                )
                conditional_home_if_far = _get_pick_object_params().home_start_if_far
                if not force_home_start:
                    panel._emit_log(
                        "[PICK_OBJ] FASE 1: HOME inicial desactivado por entorno "
                        "(PANEL_PICK_OBJECT_FORCE_HOME_START=0)"
                    )
                try:
                    home_tol_rad = _get_pick_object_params().home_tol_rad
                except Exception:
                    home_tol_rad = 0.08
                try:
                    force_if_far_max_err_rad = _get_pick_object_params().home_start_if_far_max_err_rad
                except Exception:
                    force_if_far_max_err_rad = 2.0
                current_joints = _read_current_arm_joint_positions()
                need_home_start = force_home_start
                if current_joints is not None:
                    home_err = _max_joint_error(current_joints, home_pose)
                    if home_err <= home_tol_rad:
                        panel._emit_log(
                            f"[PICK_OBJ] FASE 1: robot ya está en HOME "
                            f"(max_err={home_err:.3f}rad tol={home_tol_rad:.3f}rad)"
                        )
                        return
                    if (
                        (not need_home_start)
                        and conditional_home_if_far
                        and home_like_preflight
                        and home_err > force_if_far_max_err_rad
                    ):
                        need_home_start = True
                        panel._emit_log(
                            "[PICK_OBJ] FASE 1: HOME inicial reactivado por arranque frío "
                            f"(preflight={preflight_mode_now} max_err={home_err:.3f}rad "
                            f"gate={force_if_far_max_err_rad:.3f}rad)"
                        )
                    if not need_home_start:
                        panel._emit_log(
                            "[PICK_OBJ] FASE 1: HOME inicial omitido "
                            f"(max_err={home_err:.3f}rad tol={home_tol_rad:.3f}rad "
                            f"gate={force_if_far_max_err_rad:.3f}rad preflight={preflight_mode_now})"
                        )
                        return
                    panel._emit_log(
                        f"[PICK_OBJ] FASE 1: Ir a HOME antes del pick "
                        f"(max_err={home_err:.3f}rad tol={home_tol_rad:.3f}rad)"
                    )
                else:
                    if not need_home_start:
                        panel._emit_log(
                            "[PICK_OBJ] FASE 1: HOME inicial omitido "
                            "(joint_state actual no disponible y HOME no forzado)"
                        )
                        return
                    panel._emit_log(
                        "[PICK_OBJ] FASE 1: Ir a HOME antes del pick "
                        "(joint_state actual no disponible)"
                    )
                try:
                    home_ready_wait_sec = _get_pick_object_params().home_start_ready_wait_sec
                except Exception:
                    home_ready_wait_sec = 45.0
                home_ready_wait_sec = max(5.0, home_ready_wait_sec)
                panel._emit_log(
                    "[PICK_OBJ] FASE 1A: esperar controladores antes de HOME "
                    f"(timeout={home_ready_wait_sec:.1f}s)"
                )
                ready, reason = panel._wait_for_controllers_ready(home_ready_wait_sec)
                if not ready:
                    raise RuntimeError(
                        "HOME_START controladores no listos tras espera: "
                        f"{reason}"
                    )
                try:
                    home_start_dur_sec = _get_pick_object_params().home_start_dur_sec
                except Exception:
                    home_start_dur_sec = 8.0
                home_start_dur_sec = max(move_sec, home_start_dur_sec)
                home_start_require_reached = _get_pick_object_params().home_start_require_reached
                _run_joint_step(
                    "HOME_START",
                    home_pose,
                    timeout_sec=home_start_dur_sec + 3.0,
                    tol_rad=home_tol_rad,
                    duration_sec=home_start_dur_sec,
                    require_reached=home_start_require_reached,
                )

            # F3-step5bis-a: cuerpo de _assert_carry_coherence_after_lift
            # extraído a pick_object/carry_coherence.py.
            def _assert_carry_coherence_after_lift(
                require_state_override=None,
                timeout_override=None,
                max_dist_override=None,
                min_consecutive_override=None,
                gate_label: str = "carry_gate",
            ) -> None:
                _cc_ctx = CarryCoherenceContext(
                    panel=panel,
                    read_tcp_in_frame=_read_tcp_in_frame,
                    base_frame=base_frame,
                    world_frame=world_frame,
                    measured_ee_frame=measured_ee_frame,
                    obj_name=obj_name,
                    get_pick_object_params=_get_pick_object_params,
                    log_moveit_panel_trace=_log_moveit_panel_trace,
                )
                _assert_carry_coherence_after_lift_pure(
                    _cc_ctx,
                    require_state_override=require_state_override,
                    timeout_override=timeout_override,
                    max_dist_override=max_dist_override,
                    min_consecutive_override=min_consecutive_override,
                    gate_label=gate_label,
                )


            # F3-step4c: cuerpo de _ensure_gripper_open_for_moveit extraído
            # a pick_object/ensure_gripper_open.py. Wrapper de 11 LOC.
            def _ensure_gripper_open_for_moveit(*, reason: str = "MOVEIT") -> None:
                _ego_ctx = EnsureGripperOpenContext(
                    panel=panel,
                    get_pick_object_params=_get_pick_object_params,
                    norm_frame=_norm_frame,
                    log_moveit_panel_trace=_log_moveit_panel_trace,
                )
                _ensure_gripper_open_for_moveit_pure(_ego_ctx, reason=reason)


            def _read_gripper_contact_metrics() -> dict:
                metrics = {
                    "has_joint_state": False,
                    "age_sec": None,
                    "opening_m": None,
                    "effort_abs_max": None,
                }
                if panel.ros_worker is None:
                    return metrics
                payload, payload_wall = panel.ros_worker.get_last_joint_state()
                if not payload:
                    return metrics
                metrics["has_joint_state"] = True
                if payload_wall > 0.0:
                    metrics["age_sec"] = max(0.0, time.time() - float(payload_wall))
                try:
                    names = list(payload.get("name", []))
                    positions = list(payload.get("position", []))
                    efforts = list(payload.get("effort", []))
                except Exception:
                    return metrics
                if not names:
                    return metrics
                pos_map = {}
                for name, pos in zip(names, positions):
                    try:
                        pos_val = float(pos)
                        if not math.isfinite(pos_val):
                            continue
                        pos_map[str(name).strip()] = pos_val
                    except Exception:
                        continue
                eff_map = {}
                for name, effort in zip(names, efforts):
                    try:
                        effort_val = abs(float(effort))
                        if not math.isfinite(effort_val):
                            continue
                        eff_map[str(name).strip()] = effort_val
                    except Exception:
                        continue
                joint_names = [str(j).strip() for j in (GRIPPER_JOINT_NAMES or []) if str(j).strip()]
                if not joint_names:
                    joint_names = ["rg2_finger_joint1", "rg2_finger_joint2"]
                opening_vals = [abs(float(pos_map[j])) for j in joint_names if j in pos_map]
                if opening_vals:
                    metrics["opening_m"] = float(max(opening_vals))
                effort_vals = [float(eff_map[j]) for j in joint_names if j in eff_map]
                if effort_vals:
                    metrics["effort_abs_max"] = float(max(effort_vals))
                return metrics

            def _close_gripper_sync(*, reason: str = "PICK") -> None:
                state = {"done": False, "ok": False}

                def _close_cmd() -> None:
                    state["ok"] = bool(panel._command_gripper(True, log_action=reason, force=True))
                    state["done"] = True

                panel.signal_run_ui.emit(_close_cmd)
                deadline = time.time() + 1.5
                while time.time() < deadline:
                    if state["done"]:
                        break
                    time.sleep(0.03)
                if not state["done"]:
                    raise RuntimeError(f"gripper_close_timeout_{str(reason).lower()}")
                if not state["ok"]:
                    raise RuntimeError(f"gripper_close_command_failed_{str(reason).lower()}")

            # F3-step5bis-b: cuerpo de _ensure_strict_pre_lift_contact extraído
            # a pick_object/strict_pre_lift_contact.py. Wrapper preserva la
            # nonlocal var ``last_metrics`` vía StrictPreLiftContactState
            # mutable construido fresh por call (state inicial None).
            def _ensure_strict_pre_lift_contact(grasp_pose_live: dict, grasp_delay_live: float) -> None:
                _splc_ctx = StrictPreLiftContactContext(
                    panel=panel,
                    close_gripper_sync=_close_gripper_sync,
                    ensure_gripper_open_for_moveit=_ensure_gripper_open_for_moveit,
                    read_gripper_contact_metrics=_read_gripper_contact_metrics,
                    run_moveit_step=_run_moveit_step,
                    strict_physics_mode=strict_physics_mode,
                    table_top_base=table_top_base,
                    get_pick_object_params=_get_pick_object_params,
                )
                _splc_state = StrictPreLiftContactState()
                _ensure_strict_pre_lift_contact_pure(
                    _splc_ctx, _splc_state, grasp_pose_live, grasp_delay_live,
                )


            def _ensure_strict_probe_carry(grasp_pose_live: dict, grasp_delay_live: float) -> None:
                if not strict_physics_mode:
                    return
                try:
                    probe_lift_m = _get_pick_object_params().strict_probe_lift_m
                except Exception:
                    probe_lift_m = 0.050
                try:
                    probe_retries = _get_pick_object_params().strict_probe_retries
                except Exception:
                    probe_retries = 2
                try:
                    probe_timeout_sec = _get_pick_object_params().strict_probe_timeout_sec
                except Exception:
                    probe_timeout_sec = 0.90
                try:
                    probe_max_dist_m = _get_pick_object_params().strict_probe_max_dist_m
                except Exception:
                    probe_max_dist_m = 0.160
                try:
                    probe_regrasp_step_m = _get_pick_object_params().strict_probe_regrasp_step_m
                except Exception:
                    probe_regrasp_step_m = 0.006
                try:
                    regrasp_floor_margin_m = _get_pick_object_params().strict_regrasp_floor_margin_m
                except Exception:
                    regrasp_floor_margin_m = 0.008

                probe_lift_m = max(0.015, probe_lift_m)
                probe_retries = max(0, min(probe_retries, 3))
                probe_timeout_sec = max(0.3, probe_timeout_sec)
                probe_regrasp_step_m = max(0.001, probe_regrasp_step_m)
                min_z = float(table_top_base) + max(0.004, regrasp_floor_margin_m)

                for attempt_idx in range(0, probe_retries + 1):
                    cur_pos = grasp_pose_live.get("position", (bx, by, bz))
                    cur_x = float(cur_pos[0])
                    cur_y = float(cur_pos[1])
                    cur_z = float(cur_pos[2])
                    probe_target_z = min(float(lift_pose[2]), cur_z + probe_lift_m)
                    probe_pose = _make_pose_data((cur_x, cur_y, probe_target_z), frame=pose_frame)
                    probe_label = f"STRICT_PROBE_LIFT_{attempt_idx + 1}"
                    panel._emit_log(
                        f"[PICK_OBJ][STRICT_PROBE] attempt={attempt_idx + 1}/{probe_retries + 1} "
                        f"grasp_z={cur_z:.3f} probe_z={probe_target_z:.3f}"
                    )
                    _run_moveit_step(probe_label, probe_pose, min(0.35, float(sequence[3][2])))
                    try:
                        _assert_carry_coherence_after_lift(
                            require_state_override=False,
                            timeout_override=probe_timeout_sec,
                            max_dist_override=probe_max_dist_m,
                            min_consecutive_override=1,
                            gate_label=f"strict_probe_{attempt_idx + 1}",
                        )
                        grasp_pose_live["position"] = (cur_x, cur_y, probe_target_z)
                        panel._emit_log(
                            f"[PICK_OBJ][STRICT_PROBE] ok attempt={attempt_idx + 1} probe_z={probe_target_z:.3f}"
                        )
                        return
                    except RuntimeError as exc:
                        if attempt_idx >= probe_retries:
                            raise
                        panel._emit_log(
                            f"[PICK_OBJ][STRICT_PROBE] retrying after={exc}"
                        )
                    recover_z = max(min_z, cur_z - probe_regrasp_step_m)
                    panel._emit_log(
                        f"[PICK_OBJ][STRICT_PROBE] regrasp attempt={attempt_idx + 1}/{probe_retries} "
                        f"recover_z={recover_z:.3f} prev_z={cur_z:.3f}"
                    )
                    _ensure_gripper_open_for_moveit(reason=f"STRICT_PROBE_REOPEN_{attempt_idx + 1}")
                    recover_pose = _make_pose_data((cur_x, cur_y, recover_z), frame=pose_frame)
                    _run_moveit_step(
                        f"STRICT_PROBE_REGRASP_{attempt_idx + 1}",
                        recover_pose,
                        min(0.25, float(grasp_delay_live)),
                    )
                    grasp_pose_live["position"] = (cur_x, cur_y, recover_z)
                    _close_gripper_sync(reason=f"STRICT_PROBE_REGRASP_{attempt_idx + 1}")
                    time.sleep(0.35)
                    _ensure_strict_pre_lift_contact(grasp_pose_live, grasp_delay_live)
                raise RuntimeError("strict_probe_unreachable")

            def _run_strict_staged_lift(grasp_pose_live: dict) -> None:
                staged_lift_force = _get_pick_object_params().staged_lift_always
                if not (strict_physics_mode or staged_lift_force):
                    _run_moveit_step(*sequence[3])
                    _assert_carry_coherence_after_lift()
                    return
                try:
                    stage_step_m = _get_pick_object_params().strict_lift_stage_step_m
                except Exception:
                    stage_step_m = 0.050
                try:
                    stage_timeout_sec = _get_pick_object_params().strict_lift_stage_timeout_sec
                except Exception:
                    stage_timeout_sec = 0.90
                try:
                    stage_max_dist_m = _get_pick_object_params().strict_lift_stage_max_dist_m
                except Exception:
                    stage_max_dist_m = 0.245
                try:
                    stage_min_consecutive = _get_pick_object_params().strict_lift_stage_min_consecutive
                except Exception:
                    stage_min_consecutive = 1
                try:
                    stage_settle_sec = _get_pick_object_params().strict_lift_settle_sec
                except Exception:
                    stage_settle_sec = 0.20

                # Allow very fine lift staging during pickup; 1 cm still caused
                # late-stage TF drift once the object was already detached.
                stage_step_m = max(0.005, stage_step_m)
                stage_timeout_sec = max(0.3, stage_timeout_sec)
                stage_max_dist_m = max(0.10, stage_max_dist_m)
                stage_min_consecutive = max(1, stage_min_consecutive)
                stage_settle_sec = max(0.0, stage_settle_sec)
                cur_pos = grasp_pose_live.get("position", (bx, by, grasp_down_pose[2]))
                cur_x = float(cur_pos[0])
                cur_y = float(cur_pos[1])
                cur_z = float(cur_pos[2])
                final_z = float(lift_pose[2])
                stage_idx = 0

                while cur_z + 1e-6 < final_z:
                    next_z = min(final_z, cur_z + stage_step_m)
                    is_final = abs(next_z - final_z) <= 1e-6
                    label = "LIFT" if is_final else f"STRICT_LIFT_STAGE_{stage_idx + 1}"
                    gate_label = "after_lift" if is_final else f"strict_lift_stage_{stage_idx + 1}"
                    panel._emit_log(
                        f"[PICK_OBJ][STRICT_LIFT] stage={stage_idx + 1} current_z={cur_z:.3f} next_z={next_z:.3f} final={str(is_final).lower()}"
                    )
                    stage_orientation = grasp_pose_live.get("orientation", pick_orientation)
                    stage_pose = _make_pose_data(
                        (cur_x, cur_y, next_z),
                        orientation=stage_orientation,
                        frame=pose_frame,
                    )
                    _run_moveit_step(label, stage_pose, min(0.35, float(sequence[3][2])))
                    if stage_settle_sec > 0.0:
                        time.sleep(stage_settle_sec)
                    _assert_carry_coherence_after_lift(
                        require_state_override=False,
                        timeout_override=stage_timeout_sec,
                        max_dist_override=stage_max_dist_m,
                        min_consecutive_override=stage_min_consecutive,
                        gate_label=gate_label,
                    )
                    measured_tcp, _measured_tcp_tf = _read_tcp_in_frame(
                        base_frame,
                        measured_ee_frame,
                        timeout_sec=0.25,
                    )
                    if measured_tcp is not None:
                        next_x = float(measured_tcp[0])
                        next_y = float(measured_tcp[1])
                        if abs(next_x - cur_x) > 1e-4 or abs(next_y - cur_y) > 1e-4:
                            panel._emit_log(
                                f"[PICK_OBJ][STRICT_LIFT] live_xy_update stage={stage_idx + 1} "
                                f"prev=({cur_x:.3f},{cur_y:.3f}) "
                                f"tcp=({next_x:.3f},{next_y:.3f})"
                            )
                        cur_x = next_x
                        cur_y = next_y
                    else:
                        panel._emit_log(
                            f"[PICK_OBJ][STRICT_LIFT] live_xy_update_unavailable stage={stage_idx + 1}; "
                            "keeping_previous_xy"
                        )
                    grasp_pose_live["position"] = (cur_x, cur_y, next_z)
                    cur_z = next_z
                    stage_idx += 1
                try:
                    gripper_settle_sec = _get_pick_object_params().gripper_open_settle_sec
                except Exception:
                    gripper_settle_sec = 0.70
                if gripper_settle_sec > 0.0:
                    time.sleep(gripper_settle_sec)
                panel._emit_log(f"[PICK_OBJ] Gripper opened BEFORE {reason}")

            def _run_moveit_step(label: str, pose_data: dict, delay: float) -> None:
                nonlocal moveit_monitor_total, moveit_monitor_manual
                label_up = str(label).upper()
                _step_phase_gate(
                    label_up,
                    position=pose_data.get("position"),
                    decision=f"moveit_step:{label_up}",
                    object_position=(bx, by, bz) if 'bx' in locals() else None,
                )
                _set_exec_fn = getattr(panel, "_step_set_exec_target", None)
                if callable(_set_exec_fn):
                    _set_exec_fn(pose_data.get("position"))
                _moveit2_log("STEP", f"label={label_up} state=start")
                if deterministic_joint_after_approach and label_up != "APPROACH":
                    panel._emit_log(
                        f"[PICK_OBJ][MODE] deterministic_joint_path active; skip_moveit label={label_up}"
                    )
                    raise RuntimeError(f"deterministic_joint_mode label={label}")
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
                        bridge_request_timeout = _get_moveit_bridge_params().request_timeout_sec
                    except Exception:
                        bridge_request_timeout = 60.0
                    bridge_request_timeout = max(2.0, bridge_request_timeout)
                    wait_default_sec = max(45.0, bridge_request_timeout + 10.0)
                    try:
                        moveit_wait_sec = float(
                            os.environ.get(
                                "PANEL_PICK_OBJECT_MOVEIT_WAIT_SEC",
                                str(wait_default_sec),
                            )
                        )
                    except Exception:
                        moveit_wait_sec = wait_default_sec
                    moveit_wait_sec = max(10.0, moveit_wait_sec, bridge_request_timeout)
                    try:
                        moveit_wait_recovered_sec = float(
                            os.environ.get(
                                "PANEL_PICK_OBJECT_MOVEIT_WAIT_RECOVERED_SEC",
                                str(moveit_wait_sec),
                            )
                        )
                    except Exception:
                        moveit_wait_recovered_sec = moveit_wait_sec
                    moveit_wait_recovered_sec = max(
                        10.0,
                        moveit_wait_recovered_sec,
                        bridge_request_timeout,
                    )
                    if label_up.startswith("TRANSPORT"):
                        try:
                            transport_wait_sec = _get_pick_object_params().transport_moveit_wait_sec
                        except Exception:
                            transport_wait_sec = 460.0
                        transport_wait_sec = max(
                            moveit_wait_sec,
                            moveit_wait_recovered_sec,
                            bridge_request_timeout,
                            transport_wait_sec,
                        )
                        moveit_wait_sec = transport_wait_sec
                        moveit_wait_recovered_sec = max(
                            moveit_wait_recovered_sec,
                            transport_wait_sec,
                        )
                        panel._emit_log(
                            "[PICK_OBJ][MOVEIT][PLAN] "
                            f"{label} transport_wait_override={transport_wait_sec:.1f}s"
                        )
                    if bridge_recovered:
                        panel._emit_log(
                            f"[PICK_OBJ][MOVEIT][PLAN] {label} bridge_recovered=true reset_request_id_gate"
                        )

                    def _next_panel_request_id() -> int:
                        # Use the panel-global MoveIt request counter so pick_object
                        # does not collide with concurrent/previous MoveIt flows.
                        seq = int(getattr(panel, "_panel_moveit_request_id", 0) or 0) + 1
                        setattr(panel, "_panel_moveit_request_id", seq)
                        return seq

                    def _next_panel_request_uuid(label_key: str = "") -> str:
                        base_uuid = uuid.uuid4().hex
                        label_norm = str(label_key or "").strip().upper()
                        if label_norm.startswith("TRANSPORT"):
                            if _get_pick_object_params().transport_skip_constraints:
                                return f"skip_constraints:{base_uuid}"
                        return base_uuid

                    def _phase_tf_tol_m(label_key: str) -> float:
                        if label_key == "PRE_GRASP":
                            try:
                                return _get_pick_object_params().pre_grasp_tol_m
                            except Exception:
                                return 0.10
                        if label_key == "PRE_GRASP_RECENTER":
                            # F2-step1: override Optional ⇒ pre_grasp_tol_m fallback.
                            try:
                                _p = _get_pick_object_params()
                                return float(
                                    _p.pre_grasp_recenter_tol_m_override
                                    if _p.pre_grasp_recenter_tol_m_override is not None
                                    else _p.pre_grasp_tol_m
                                )
                            except Exception:
                                return 0.10
                        if str(label_key).startswith("GRASP_DOWN"):
                            # F2-step1: canónico unificado a 0.040 (era 0.04 legacy).
                            try:
                                return float(_get_pick_object_params().grasp_step_tol_m)
                            except Exception:
                                return 0.04
                        if label_key in ("LIFT", "DROP") or str(label_key).startswith("TRANSPORT"):
                            try:
                                return _get_pick_object_params().path_tol_m
                            except Exception:
                                return 0.03
                        if label_key == "APPROACH":
                            try:
                                return _get_pick_object_params().approach_tol_m
                            except Exception:
                                return 0.10
                        try:
                            return _get_pick_object_params().step_tol_m
                        except Exception:
                            return 0.02

                    def _encode_request_frame(
                        frame: str,
                        request_id: int,
                        request_uuid: str,
                        *,
                        tol_m: float | None = None,
                        phase_label: str | None = None,
                    ) -> str:
                        base = str(frame or BASE_FRAME or "base_link").strip() or "base_link"
                        rid = int(request_id)
                        uid = str(request_uuid or "").strip()
                        parts = [base, f"rid={rid}"]
                        if uid:
                            parts.append(f"uid={uid}")
                        if tol_m is not None:
                            try:
                                parts.append(f"tol={float(tol_m):.3f}")
                            except Exception:
                                pass
                        phase_txt = str(phase_label or "").strip()
                        if phase_txt:
                            phase_txt = phase_txt.replace("|", "_")
                            parts.append(f"phase={phase_txt}")
                        return "|".join(parts)

                    use_cartesian = False
                    cart_env_name = ""
                    if label_up == "APPROACH":
                        cart_env_name = "PANEL_PICK_OBJECT_APPROACH_CARTESIAN"
                    elif label_up == "GRASP_DOWN" or label_up.startswith("GRASP_DOWN_MICRO_"):
                        cart_env_name = "PANEL_PICK_OBJECT_GRASP_CARTESIAN"
                    elif label_up == "LIFT" or label_up.startswith("STRICT_LIFT_STAGE_"):
                        cart_env_name = "PANEL_PICK_OBJECT_LIFT_CARTESIAN"
                    elif label_up.startswith("TRANSPORT"):
                        cart_env_name = "PANEL_PICK_OBJECT_TRANSPORT_CARTESIAN"
                    if cart_env_name:
                        default_cartesian = "0" if label_up == "APPROACH" else "1"
                        grasp_cartesian_raw = str(
                            os.environ.get(
                                cart_env_name,
                                default_cartesian,
                            )
                        ).strip().lower()
                        use_cartesian = grasp_cartesian_raw not in ("0", "false", "no", "off")
                        panel._emit_log(
                            f"[PICK_OBJ][MOVEIT][PLAN] {label_up} "
                            f"cartesian={str(use_cartesian).lower()} "
                            f"env={cart_env_name}:{grasp_cartesian_raw or 'default'}"
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
                        panel_request_uuid = _next_panel_request_uuid(label_up)
                        request_stamp_ns = _ros_clock_now_ns()
                        tol_step = _phase_tf_tol_m(label)
                        pose_to_send = dict(pose_data)
                        pose_to_send["stamp_ns"] = int(request_stamp_ns)
                        pose_to_send["frame"] = _encode_request_frame(
                            frame_id,
                            panel_request_id,
                            panel_request_uuid,
                            tol_m=tol_step,
                            phase_label=label_up,
                        )
                        # wait_for_moveit_result() gates on RosWorker._moveit_result_wall,
                        # which is recorded with the monotonic clock. Using time.time()
                        # here makes every incoming result look stale and the panel keeps
                        # waiting even after the bridge has published a terminal outcome.
                        request_wall = max(0.0, float(baseline_wall) - 0.001)
                        pose_subs_now = panel.ros_worker.topic_subscriber_count(moveit_pose_topic)
                        if pose_subs_now <= 0:
                            raise RuntimeError(
                                f"no_pose_subscribers_before_publish topic={moveit_pose_topic}"
                            )
                        live_world_xyz, live_base_xyz, live_source = _resolve_live_object_pose_for_audit(
                            target_snapshot_name
                        )
                        tcp_before, _tcp_before_tf = _read_tcp_in_frame(frame_id, measured_ee_frame)
                        _log_moveit_panel_trace(label, current_obj_base=live_base_xyz)
                        if live_base_xyz is not None and tcp_before is not None:
                            dx_bt = float(live_base_xyz[0]) - float(tcp_before[0])
                            dy_bt = float(live_base_xyz[1]) - float(tcp_before[1])
                            dz_bt = float(live_base_xyz[2]) - float(tcp_before[2])
                            dist_bt = math.sqrt(dx_bt * dx_bt + dy_bt * dy_bt + dz_bt * dz_bt)
                        else:
                            dx_bt = dy_bt = dz_bt = dist_bt = float("nan")
                        panel._emit_log(
                            "[PICK][MOVEIT][TCP_BEFORE] "
                            f"label={label} ee_frame={measured_ee_frame} target_frame={frame_id} "
                            f"tcp_live_base={fmt_vec3(tcp_before)} "
                            f"object_live_base={fmt_vec3(live_base_xyz)} "
                            f"delta_object_tcp_base={fmt_vec3((dx_bt, dy_bt, dz_bt)) if tcp_before is not None and live_base_xyz is not None else '(--,--,--)'} "
                            f"dist_object_tcp_base_m={dist_bt:.3f}" if tcp_before is not None and live_base_xyz is not None else
                            f"[PICK][MOVEIT][TCP_BEFORE] label={label} ee_frame={measured_ee_frame} target_frame={frame_id} tcp_live_base={fmt_vec3(tcp_before)} object_live_base={fmt_vec3(live_base_xyz)} delta_object_tcp_base=(--,--,--) dist_object_tcp_base_m=n/a"
                        )
                        panel._emit_log(
                            "[PICK][MOVEIT][TARGET] "
                            f"label={label} target_source={live_source} "
                            f"target_original={fmt_pose_dict(pose_data)} "
                            f"target_command={fmt_pose_dict(pose_to_send)} "
                            f"object_live_world={fmt_vec3(live_world_xyz)} "
                            f"object_live_base={fmt_vec3(live_base_xyz)}"
                        )
                        panel._emit_log(
                            "[PICK][MOVEIT][FRAME_AUDIT] "
                            f"label={label} object_world_frame={world_frame} "
                            f"object_base_frame={base_frame} target_frame_original={frame_id} "
                            f"ee_frame={measured_ee_frame} "
                            f"delta_object_tcp_base={fmt_vec3((dx_bt, dy_bt, dz_bt)) if tcp_before is not None and live_base_xyz is not None else '(--,--,--)'} "
                            f"dist_object_tcp_base_m={dist_bt:.3f}" if tcp_before is not None and live_base_xyz is not None else
                            f"[PICK][MOVEIT][FRAME_AUDIT] label={label} object_world_frame={world_frame} object_base_frame={base_frame} target_frame_original={frame_id} ee_frame={measured_ee_frame} delta_object_tcp_base=(--,--,--) dist_object_tcp_base_m=n/a"
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
                            "[PICK][MOVEIT][REQUEST] "
                            f"label={label} request_id={panel_request_id} request_uuid={panel_request_uuid} "
                            f"phase={label_up} cartesian={str(bool(cartesian_mode)).lower()} "
                            f"timeout_sec={float(timeout_sec):.2f} "
                            f"command_frame={frame_id} command_pose={fmt_vec3(position)} "
                            f"tol_m={tol_step:.3f}"
                        )
                        panel._emit_log(
                            f"[PICK_OBJ][MOVEIT][REQUEST_GATE] {label} "
                            f"baseline_wall_steady={float(baseline_wall):.6f} "
                            f"request_wall_steady={request_wall:.6f} "
                            f"baseline_seq={int(baseline_seq)}"
                        )
                        panel._emit_log(
                            f"[PANEL][PUB] {label} request_id={panel_request_id} "
                            f"request_uuid={panel_request_uuid} stamp_ns={request_stamp_ns} "
                            f"topic={moveit_pose_topic} subs_count={pose_subs_now} "
                            f"lock_id={target_lock_id_local}"
                        )
                        pub_wall_us = int(time.time() * 1_000_000)
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
                        setattr(_wait_moveit_result, "_pub_wall_us", pub_wall_us)
                        panel._emit_log(
                            f"[PICK_OBJ][APPROACH_DIAGNOSTIC] label={label} "
                            f"request_id={panel_request_id} request_uuid={panel_request_uuid} "
                            f"pub_wall_us={pub_wall_us} pub_ok=true"
                        )
                        if label_up == "APPROACH":
                            panel._emit_log(
                                f"[PICK_OBJ][TX][APPROACH] ts_us={pub_wall_us} req_id={panel_request_id} "
                                f"req_uuid={panel_request_uuid or 'n/a'} topic={moveit_pose_topic} "
                                f"frame={frame_id} pose=({float(position[0]):.3f},{float(position[1]):.3f},{float(position[2]):.3f}) "
                                f"label={label}"
                            )
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
                        _moveit2_log(
                            "STEP",
                            f"label={label_up} state=abort reason=ee_link_mismatch moveit={ee_link_moveit} tf={measured_ee_frame}",
                        )
                        raise RuntimeError(
                            f"ee_link mismatch (moveit={ee_link_moveit} tf={measured_ee_frame})"
                        )
                    if not bool(result.get("success", False)):
                        message = str(result.get("message") or "execute failed")
                        if use_cartesian and "cartesian_" in message:
                            if label_up == "APPROACH":
                                panel._emit_log(
                                    "[PICK_OBJ][MOVEIT][RECOVERY] APPROACH cartesian fallo; "
                                    "delegando fallback articular inmediato"
                                )
                                raise RuntimeError(f"execute failed ({label}): {message}")
                            panel._emit_log(
                                f"[PICK_OBJ][MOVEIT][RECOVERY] {label_up} cartesian fallo; "
                                "reintentando con pose planner"
                            )
                            result, panel_request_id = _publish_and_wait_once(
                                timeout_sec=moveit_wait_sec,
                                clear_gate=True,
                                cartesian_mode=False,
                            )
                            message = str(result.get("message") or "")
                        if not bool(result.get("success", False)):
                            recovered_from_abort = False
                            msg_l = str(message).lower()
                            pre_grasp_abort = label in ("PRE_GRASP", "PRE_GRASP_RECENTER")
                            path_tol_abort = (
                                "path tolerance violation" in msg_l
                                or "error_code=-4" in msg_l
                                or "fjt_aborted" in msg_l
                            )
                            if pre_grasp_abort and path_tol_abort:
                                # F2-step1: override Optional ⇒ pre_grasp_tol_m fallback.
                                try:
                                    _p = _get_pick_object_params()
                                    recover_tol = float(
                                        _p.pre_grasp_abort_recovery_tol_m_override
                                        if _p.pre_grasp_abort_recovery_tol_m_override is not None
                                        else _p.pre_grasp_tol_m
                                    )
                                except Exception:
                                    recover_tol = 0.10
                                panel._emit_log(
                                    f"[PICK_OBJ][RECOVERY] {label_up} abort por path tolerance; "
                                    f"verificando TF con tol={recover_tol:.3f}"
                                )
                                try:
                                    tf_recover_diag = _tf_distance_check(
                                        label=f"{label}_ABORT_RECOVERY",
                                        pose_data=pose_data,
                                        tol_m=recover_tol,
                                        ee_frame=measured_ee_frame,
                                    )
                                    if bool(tf_recover_diag.get("ok", False)):
                                        recovered_from_abort = True
                                        panel._emit_log(
                                            f"[PICK_OBJ][RECOVERY] {label_up} abort aceptado por TF "
                                            f"dist={float(tf_recover_diag.get('dist', 0.0)):.3f} "
                                            f"tol={recover_tol:.3f}"
                                        )
                                except Exception as tf_recover_exc:
                                    panel._emit_log(
                                        f"[PICK_OBJ][RECOVERY] {label_up} abort recovery TF falló: {tf_recover_exc}"
                                    )
                            if not recovered_from_abort:
                                panel._emit_log(
                                    f"[PICK_OBJ][ABORT] execute failed label={label} reason={message}"
                                )
                                _moveit2_log(
                                    "STEP",
                                    f"label={label_up} state=abort reason=execute_failed detail={message}",
                                )
                                raise RuntimeError(f"execute failed ({label}): {message}")
                    tol = _phase_tf_tol_m(label)
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
                    tcp_after = tf_diag.get("tcp")
                    target_after = tf_diag.get("target")
                    dist = float(tf_diag.get("dist", 0.0))
                    panel._emit_log(
                        "[PICK][MOVEIT][TCP_AFTER] "
                        f"label={label} ee_frame={measured_ee_frame} frame={frame_id} "
                        f"tcp_live={fmt_vec3(tcp_after)} target={fmt_vec3(target_after)} "
                        f"delta_target_tcp={fmt_vec3((float(target_after[0]) - float(tcp_after[0]), float(target_after[1]) - float(tcp_after[1]), float(target_after[2]) - float(tcp_after[2]))) if isinstance(target_after, (list, tuple)) and isinstance(tcp_after, (list, tuple)) else '(--,--,--)'} "
                        f"dist_m={dist:.3f} tol_m={tol:.3f}"
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
                        if label_up == "PRE_GRASP" and fail_reason == "pos_not_reached":
                            panel._emit_log(
                                "[PICK_OBJ][RECOVERY] PRE_GRASP tf_mismatch diferido al gate de alineacion "
                                f"dist={dist:.3f} tol={tol:.3f}"
                            )
                        else:
                            panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
                            panel._emit_log(
                                "[PICK][MOVEIT][DIVERGENCE] "
                                f"label={label} kind=bridge_success_vs_tf_reach "
                                f"dist_m={dist:.3f} tol_m={tol:.3f} "
                                f"reason={fail_reason}"
                            )
                            _moveit2_log(
                                "STEP",
                                f"label={label_up} state=abort reason=tf_mismatch dist={dist:.3f} tol={tol:.3f}",
                            )
                            raise RuntimeError(msg)
                    panel._emit_log(
                        "[PICK][MOVEIT][EXEC_RESULT] "
                        f"label={label} request_id={panel_request_id} "
                        f"success=true tf_reached={str(bool(tf_diag.get('ok', False))).lower()} "
                        f"dist_m={dist:.3f} tol_m={tol:.3f} "
                        f"ee_link_moveit={ee_link_moveit or 'n/a'} ee_link_tf={measured_ee_frame}"
                    )
                    _moveit2_log(
                        "STEP",
                        f"label={label_up} state=ok request_id={panel_request_id} dist={dist:.3f} tol={tol:.3f}",
                    )
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

            def _ensure_pre_grasp_alignment(pre_grasp_data: dict, pre_grasp_delay: float) -> None:
                try:
                    xy_tol = _get_pick_object_params().pre_grasp_align_xy_tol_m
                except Exception:
                    xy_tol = 0.045
                try:
                    z_tol = _get_pick_object_params().pre_grasp_align_z_tol_m
                except Exception:
                    z_tol = 0.050
                recenter_enabled = _get_pick_object_params().pre_grasp_recenter_enable

                tcp_pre, _tcp_tf = _read_tcp_in_frame(base_frame, measured_ee_frame, timeout_sec=0.30)
                if tcp_pre is None:
                    panel._emit_log("[PICK_OBJ][PRE_GRASP_ALIGN] tcp_unavailable skip_check=true")
                    return

                pre_pos = pre_grasp_data.get("position", (bx, by, pre_grasp_pose[2]))
                dx = float(tcp_pre[0]) - float(bx)
                dy = float(tcp_pre[1]) - float(by)
                dxy = math.hypot(dx, dy)
                z_err = abs(float(tcp_pre[2]) - float(pre_pos[2]))
                panel._emit_log(
                    f"[PICK_OBJ][PRE_GRASP_ALIGN] tcp=({float(tcp_pre[0]):.3f},{float(tcp_pre[1]):.3f},{float(tcp_pre[2]):.3f}) "
                    f"obj=({bx:.3f},{by:.3f},{bz:.3f}) target_z={float(pre_pos[2]):.3f} "
                    f"dx={dx:.3f} dy={dy:.3f} dxy={dxy:.3f} z_err={z_err:.3f} "
                    f"xy_tol={xy_tol:.3f} z_tol={z_tol:.3f}"
                )
                if dxy <= xy_tol and z_err <= z_tol:
                    return

                if not recenter_enabled:
                    panel._emit_log(
                        "[PICK_OBJ][RECOVERY] PRE_GRASP desalineado pero recenter deshabilitado; "
                        "continuando con tolerancia actual"
                    )
                    return

                panel._emit_log(
                    "[PICK_OBJ][RECOVERY] PRE_GRASP desalineado; reintentando recenter sobre xy del objeto"
                )
                recenter_pose = dict(pre_grasp_data)
                recenter_pose["position"] = (float(bx), float(by), float(pre_pos[2]))
                _run_moveit_step("PRE_GRASP_RECENTER", recenter_pose, min(0.20, float(pre_grasp_delay)))

                tcp_after, _tcp_after_tf = _read_tcp_in_frame(base_frame, measured_ee_frame, timeout_sec=0.30)
                if tcp_after is None:
                    raise RuntimeError("pre_grasp_recenter_tcp_unavailable")

                dx_after = float(tcp_after[0]) - float(bx)
                dy_after = float(tcp_after[1]) - float(by)
                dxy_after = math.hypot(dx_after, dy_after)
                z_err_after = abs(float(tcp_after[2]) - float(pre_pos[2]))
                panel._emit_log(
                    f"[PICK_OBJ][PRE_GRASP_ALIGN] recenter_result tcp=({float(tcp_after[0]):.3f},{float(tcp_after[1]):.3f},{float(tcp_after[2]):.3f}) "
                    f"dx={dx_after:.3f} dy={dy_after:.3f} dxy={dxy_after:.3f} z_err={z_err_after:.3f}"
                )
                if dxy_after > xy_tol or z_err_after > z_tol:
                    raise RuntimeError(
                        "pre_grasp_alignment_failed "
                        f"dxy={dxy_after:.3f} z_err={z_err_after:.3f} "
                        f"xy_tol={xy_tol:.3f} z_tol={z_tol:.3f}"
                    )

            # SECUENCIA GLOBAL: ir a MESA antes de HOME_START y del preflight
            # Regla de negocio: todos los modos arrancan siempre desde MESA.
            # HOME_START sigue ejecutándose después para garantizar planificación MoveIt.
            # En STEP_BY_STEP, la fase INICIO se prepara aquí mostrando TARGET=MESA;
            # al pulsar Iniciar en esa fila el robot se desplaza a MESA y luego
            # aparece el siguiente punto de control.
            # Para fases de movimiento articular no usamos target Cartesiano porque
            # fk_ur5 computa tool0 mientras TF-live mide rg2_pinch_center: frames
            # distintos → delta siempre grande → reached=NO falso. position=None
            # hace que reached quede como PEND (neutro) en vez de NO.
            _step_phase_gate("INICIO", position=None, decision="Pulse Iniciar en la fila INICIO → robot irá a pose MESA (joint), punto de partida del agarre")
            _run_joint_step(
                "MESA_GLOBAL",
                JOINT_TABLE_POSE_RAD,
                timeout_sec=move_sec + 5.0,
                tol_rad=0.06,
            )
            panel._emit_log(
                "[PICK_OBJ] FASE 0: MESA_GLOBAL alcanzada — secuencia MESA→pick→MESA→CESTA activa"
            )

            # Capturar pose EE en MESA antes del HOME silencioso, para que el
            # gate de APPROACH muestre MESA como ORG aunque el robot esté en HOME.
            _op_frame_pre_home = panel._step_operational_frame_name() if panel._step_mode == "STEP_BY_STEP" else ""
            if _op_frame_pre_home:
                _mesa_ee_pos = None
                for _ovr_attempt in range(5):
                    _mesa_ee_pos = panel._step_fetch_live_pose(_op_frame_pre_home)
                    if _mesa_ee_pos is not None:
                        break
                    import time as _time_mod
                    _time_mod.sleep(0.10)
                if _mesa_ee_pos is not None:
                    setattr(panel, "_step_origin_override_for_next_gate", _mesa_ee_pos)
                    panel._emit_log(
                        f"[PICK_OBJ][STEP] origin_override_captured frame={_op_frame_pre_home} "
                        f"xyz=({_mesa_ee_pos[0]:.3f},{_mesa_ee_pos[1]:.3f},{_mesa_ee_pos[2]:.3f})"
                    )
                else:
                    panel._emit_log(
                        f"[PICK_OBJ][STEP] origin_override_FAILED — TF timeout tras 5 intentos, APPROACH ORG usará live TF"
                    )

            _ensure_home_start()

            # Start from MESA by default so the approach always begins from
            # a known, repeatable posture close to the work area.
            preflight_mode = str(
                _get_pick_object_params().preflight_mode
            ).strip().lower()
            pick_image_preflight_modes = (
                "home_pick_image",
                "pick_image_home",
                "pick_image_only",
                "home_then_pick_image",
            )
            mesa_preflight_modes = (
                "mesa",
                "mesa_only",
                "table_only",
            )
            if moveit_exclusive and preflight_mode in (
                *pick_image_preflight_modes,
                "fixed",
                "legacy",
                "table",
                "pick_image",
                *mesa_preflight_modes,
                "1",
                "true",
                "on",
            ):
                if allow_joint_preflight_in_moveit and preflight_mode in (
                    *pick_image_preflight_modes,
                    *mesa_preflight_modes,
                ):
                    panel._emit_log(
                        "[PICK_OBJ][MODE] moveit_exclusive=true; permitiendo preflight articular controlado "
                        f"({preflight_mode})"
                    )
                else:
                    panel._emit_log(
                        "[PICK_OBJ][MODE] moveit_exclusive=true; ignorando preflight articular "
                        f"({preflight_mode}) y forzando preflight=home"
                    )
                    preflight_mode = "home"
            if preflight_mode in pick_image_preflight_modes:
                try:
                    pick_image_pregrasp_tol_rad = _get_pick_object_params().pick_image_preflight_tol_rad
                except Exception:
                    pick_image_pregrasp_tol_rad = 0.05
                pick_image_pregrasp_require_reached = _get_pick_object_params().pick_image_preflight_require_reached
                panel._emit_log(
                    "[PICK_OBJ] FASE 1B: Ir a PICK_IMAGE desde HOME "
                    f"(preflight={preflight_mode})"
                )
                _run_joint_step(
                    "PICK_IMAGE_PREGRASP",
                    JOINT_PICK_IMAGE_POSE_RAD,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=pick_image_pregrasp_tol_rad,
                    require_reached=pick_image_pregrasp_require_reached,
                )
            elif preflight_mode in ("fixed", "legacy", "table", "pick_image", "1", "true", "on"):
                panel._emit_log("[PICK_OBJ] FASE 1B: Acercamiento legacy vía mesa (preflight=fixed)")
                _run_joint_step("MESA", JOINT_TABLE_POSE_RAD)
                _run_joint_step("PICK_IMAGE", JOINT_PICK_IMAGE_POSE_RAD)
            elif preflight_mode in ("mesa", "mesa_only", "table_only"):
                panel._emit_log("[PICK_OBJ] FASE 1B: Ir a MESA antes del pick (preflight=mesa)")
                _run_joint_step(
                    "MESA_PREGRASP",
                    JOINT_TABLE_POSE_RAD,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=0.06,
                )
            elif preflight_mode in ("home", "home_only", "direct", "default", "none", "off", "0", "false"):
                panel._emit_log(
                    "[PICK_OBJ] FASE 1B: MoveIt arranca desde HOME "
                    f"(preflight={preflight_mode})"
                )
            else:
                panel._emit_log(
                    "[PICK_OBJ] FASE 1B: Preflight fijo omitido; "
                    f"MoveIt arranca desde HOME (preflight={preflight_mode})"
                )

            panel._emit_log("[PICK_OBJ] FASE 2: Abrir gripper antes de MoveIt")
            _ensure_gripper_open_for_moveit()

            panel._emit_log("[PICK_OBJ] FASE 3: APPROACH sobre objeto (pinza abierta)")
            _moveit2_log("PHASE", "APPROACH start")
            panel._emit_log(
                "[MOVEIT2][CYCLE_REF][USE] "
                f"phase=APPROACH consumer=_run_moveit_step "
                f"source={_moveit_cycle_object_source} "
                f"world=({_moveit_cycle_object_world[0]:.3f},{_moveit_cycle_object_world[1]:.3f},{_moveit_cycle_object_world[2]:.3f}) "
                f"base=({_moveit_cycle_object_base[0]:.3f},{_moveit_cycle_object_base[1]:.3f},{_moveit_cycle_object_base[2]:.3f}) "
                f"approach_target=({approach_pose[0]:.3f},{approach_pose[1]:.3f},{approach_pose[2]:.3f})"
            )
            try:
                _run_moveit_step(*sequence[0])
            except RuntimeError as exc:
                msg = str(exc)
                approach_tf_mismatch = _is_step_tf_mismatch(msg, "APPROACH")
                approach_exec_failed = _is_step_exec_failed(msg, "APPROACH")
                if not (approach_fallback_enabled and (approach_tf_mismatch or approach_exec_failed)):
                    raise
                skip_retry_after_fallback = _get_pick_object_params().approach_skip_retry_on_fallback
                reason = "tf_mismatch" if approach_tf_mismatch else "exec_failed"
                panel._emit_log(
                    f"[PICK_OBJ][RECOVERY] APPROACH {reason} detectado; "
                    "fallback articular a PICK_IMAGE"
                )
                _run_joint_step(
                    "APPROACH_FALLBACK_JOINT",
                    JOINT_PICK_IMAGE_POSE_RAD,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=0.08,
                )
                _ensure_gripper_open_for_moveit(reason="APPROACH_RETRY")
                if skip_retry_after_fallback:
                    panel._emit_log(
                        "[PICK_OBJ][RECOVERY] APPROACH fallback aplicado; "
                        "continuando a PRE_GRASP sin reintentar APPROACH"
                    )
                else:
                    panel._emit_log(
                        "[PICK_OBJ][RECOVERY] APPROACH fallback aplicado; "
                        "reintentando APPROACH"
                    )
                    _run_moveit_step(*sequence[0])
            _canonical_phase(
                "APPROACH_REACHED",
                detail=f"target=({approach_pose[0]:.3f},{approach_pose[1]:.3f},{approach_pose[2]:.3f})",
            )

            panel._emit_log("[PICK_OBJ] FASE 4: PRE_GRASP (pinza abierta)")
            _moveit2_log("PHASE", "PRE_GRASP start")
            panel._emit_log(
                "[MOVEIT2][CYCLE_REF][USE] "
                f"phase=PRE_GRASP consumer=_run_moveit_step "
                f"source={_moveit_cycle_object_source} "
                f"world=({_moveit_cycle_object_world[0]:.3f},{_moveit_cycle_object_world[1]:.3f},{_moveit_cycle_object_world[2]:.3f}) "
                f"base=({_moveit_cycle_object_base[0]:.3f},{_moveit_cycle_object_base[1]:.3f},{_moveit_cycle_object_base[2]:.3f}) "
                f"pre_grasp_target=({pre_grasp_pose[0]:.3f},{pre_grasp_pose[1]:.3f},{pre_grasp_pose[2]:.3f})"
            )
            try:
                _run_moveit_step(*sequence[1])
            except RuntimeError as exc:
                msg = str(exc)
                pre_grasp_tf_mismatch = _is_step_tf_mismatch(msg, "PRE_GRASP")
                pre_grasp_exec_failed = _is_step_exec_failed(msg, "PRE_GRASP")
                if not (pre_grasp_fallback_enabled and (pre_grasp_tf_mismatch or pre_grasp_exec_failed)):
                    raise
                reason = "tf_mismatch" if pre_grasp_tf_mismatch else "exec_failed"
                panel._emit_log(
                    f"[PICK_OBJ][RECOVERY] PRE_GRASP {reason} detectado; "
                    + (
                        "fallback articular a PICK_IMAGE"
                        if deterministic_joint_after_approach
                        else "fallback articular a GRASP_DOWN"
                    )
                )
                _run_joint_step(
                    "PRE_GRASP_FALLBACK_JOINT",
                    JOINT_PICK_IMAGE_POSE_RAD if deterministic_joint_after_approach else JOINT_GRASP_DOWN_POSE_RAD,
                    timeout_sec=move_sec + 3.0,
                    tol_rad=0.10,
                )
                _ensure_gripper_open_for_moveit(reason="PRE_GRASP_RETRY")
            _ensure_pre_grasp_alignment(sequence[1][1], sequence[1][2])
            _canonical_phase(
                "PREGRASP_REACHED",
                detail=f"target=({pre_grasp_pose[0]:.3f},{pre_grasp_pose[1]:.3f},{pre_grasp_pose[2]:.3f})",
            )

            panel._emit_log("[PICK_OBJ] FASE 5: GRASP_DOWN (pinza abierta)")
            _moveit2_log("PHASE", "GRASP_DOWN start")
            panel._emit_log(
                "[MOVEIT2][CYCLE_REF][USE] "
                f"phase=GRASP_DOWN consumer=_run_moveit_step "
                f"source={_moveit_cycle_object_source} "
                f"world=({_moveit_cycle_object_world[0]:.3f},{_moveit_cycle_object_world[1]:.3f},{_moveit_cycle_object_world[2]:.3f}) "
                f"base=({_moveit_cycle_object_base[0]:.3f},{_moveit_cycle_object_base[1]:.3f},{_moveit_cycle_object_base[2]:.3f}) "
                f"grasp_target=({grasp_down_pose[0]:.3f},{grasp_down_pose[1]:.3f},{grasp_down_pose[2]:.3f})"
            )
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
            adapt_grasp_xy = _get_pick_object_params().adapt_grasp_xy
            if adapt_grasp_xy:
                try:
                    tcp_pre, _ = _read_tcp_in_frame(base_frame, measured_ee_frame, timeout_sec=0.25)
                    if tcp_pre is not None:
                        try:
                            adapt_max = _get_pick_object_params().adapt_grasp_xy_max_delta_m
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
            grasp_joint_fallback_enabled = _get_pick_object_params().grasp_joint_fallback
            grasp_joint_fallback_applied = False
            grasp_moveit_retry_enabled = _get_pick_object_params().grasp_moveit_retry
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
                grasp_exec_failed = _is_step_exec_failed(msg, "GRASP_DOWN")
                # F2-step1: lectura inicial via dataclass; el bloque de retry de
                # más abajo sigue mutando os.environ para que _run_moveit_step
                # (que lee env directamente) use el nuevo valor en el reintento.
                grasp_cartesian_enabled = bool(_get_pick_object_params().grasp_cartesian)
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
                        retry_count = _get_pick_object_params().grasp_moveit_retry_count
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
                if not (grasp_joint_fallback_enabled and (grasp_tf_mismatch or grasp_exec_failed)):
                    raise
                fallback_reason = "tf_mismatch" if grasp_tf_mismatch else "exec_failed"
                panel._emit_log(
                    f"[PICK_OBJ][RECOVERY] GRASP_DOWN {fallback_reason} detectado; "
                    "fallback a descenso articular"
                )
                _run_joint_step(
                    "GRASP_DOWN_FALLBACK_JOINT",
                    JOINT_GRASP_DOWN_POSE_RAD,
                    timeout_sec=move_sec + 2.5,
                    tol_rad=0.08,
                )
                grasp_joint_fallback_applied = True
            time.sleep(0.3)  # Permitir que brazo se estabilice en posición

            # ── GRASP_DOWN confirmación de estabilidad TF ─────────────────────
            # Lee TF N veces y verifica que el TCP se mantenga dentro de la
            # tolerancia antes de aprobar la fase. Evita falsos positivos cuando
            # el resultado de bridge=OK llega justo en el límite de tolerancia.
            _grasp_target_pos = grasp_pose_send.get("position", (bx, by, bz))
            # F2-step1: tolerancia explícita (default histórico 0.045).
            try:
                _grasp_stable_tol = float(_get_pick_object_params().grasp_tf_stable_tol_m)
            except Exception:
                _grasp_stable_tol = 0.045
            try:
                _grasp_stable_n = _get_pick_object_params().grasp_tf_stable_samples
                _grasp_stable_n = max(2, min(_grasp_stable_n, 10))
            except Exception:
                _grasp_stable_n = 5
            try:
                _grasp_stable_min_ok = _get_pick_object_params().grasp_tf_stable_min_ok
                _grasp_stable_min_ok = max(1, min(_grasp_stable_min_ok, _grasp_stable_n))
            except Exception:
                _grasp_stable_min_ok = 4

            _stable_dists = []
            _stable_ok_count = 0
            for _s_idx in range(_grasp_stable_n):
                _s_tcp, _ = _read_tcp_in_frame(base_frame, measured_ee_frame, timeout_sec=0.15)
                if _s_tcp is not None:
                    _s_d = math.sqrt(sum(
                        (float(_s_tcp[i]) - float(_grasp_target_pos[i])) ** 2
                        for i in range(3)
                    ))
                    _stable_dists.append(_s_d)
                    if _s_d <= _grasp_stable_tol:
                        _stable_ok_count += 1
                else:
                    _stable_dists.append(None)
                if _s_idx < _grasp_stable_n - 1:
                    time.sleep(0.06)

            _grasp_stable_pass = _stable_ok_count >= _grasp_stable_min_ok
            _dists_txt = "[" + ", ".join(
                f"{d:.3f}" if d is not None else "none" for d in _stable_dists
            ) + "]"
            panel._emit_log(
                f"[GRASP_DOWN][STABLE_CONFIRM] "
                f"frame={base_frame} ee={measured_ee_frame} "
                f"target=({float(_grasp_target_pos[0]):.3f},{float(_grasp_target_pos[1]):.3f},{float(_grasp_target_pos[2]):.3f}) "
                f"samples={_grasp_stable_n} ok_count={_stable_ok_count} "
                f"min_ok={_grasp_stable_min_ok} tol={_grasp_stable_tol:.3f} "
                f"dists={_dists_txt} stable={str(_grasp_stable_pass).lower()}"
            )
            if not _grasp_stable_pass:
                raise RuntimeError(
                    f"grasp_down_unstable_tf "
                    f"ok_count={_stable_ok_count} min_ok={_grasp_stable_min_ok} "
                    f"samples={_grasp_stable_n} tol={_grasp_stable_tol:.3f} "
                    f"dists={_dists_txt}"
                )

            tcp_base, _tcp_tf = _read_tcp_in_frame(base_frame, measured_ee_frame, timeout_sec=0.3)
            if tcp_base is None:
                raise RuntimeError(f"no se pudo leer TCP en {base_frame}")
            target_tcp_z = float(grasp_pose_send.get("position", (bx, by, bz))[2])
            # F2-step1: tolerancias canónicas (defaults históricos 0.040).
            try:
                _po_p = _get_pick_object_params()
                grasp_z_reach_tol = float(
                    _po_p.grasp_z_reach_tol_fallback_m
                    if grasp_joint_fallback_applied
                    else _po_p.grasp_z_reach_tol_m
                )
            except Exception:
                grasp_z_reach_tol = 0.040
            z_reach_err = abs(float(tcp_base[2]) - target_tcp_z)
            panel._emit_log(
                f"[PICK_OBJ][GRASP_VALIDATE] target_tcp_z={target_tcp_z:.3f} "
                f"tcp_z={float(tcp_base[2]):.3f} z_err={z_reach_err:.3f} "
                f"z_tol={grasp_z_reach_tol:.3f} "
                f"tol_source={'fallback' if grasp_joint_fallback_applied else 'primary'}"
            )
            if z_reach_err > grasp_z_reach_tol:
                raise RuntimeError(
                    f"altura_insuficiente_z target={target_tcp_z:.3f} tcp={float(tcp_base[2]):.3f} "
                    f"err={z_reach_err:.3f} tol={grasp_z_reach_tol:.3f}"
                )
            try:
                attach_z_ref_mode = str(_get_pick_object_params().attach_z_ref_mode).strip().lower()
            except Exception:
                attach_z_ref_mode = "top"
            if attach_z_ref_mode not in ("center", "top"):
                attach_z_ref_mode = "top"
            try:
                attach_z_clearance = _get_pick_object_params().attach_z_clearance_m
            except Exception:
                attach_z_clearance = 0.0
            obj_center_z = float(bz)
            obj_top_z = obj_center_z + (float(height or 0.0) * 0.5 if float(height or 0.0) > 0.0 else 0.0)
            obj_ref_z = (obj_center_z if attach_z_ref_mode == "center" else obj_top_z) + float(attach_z_clearance)
            measured_ee_frame = str(panel._ee_frame_effective or RG2_TCP_FRAME).strip() or RG2_TCP_FRAME
            tcp_z_correction = contact_z_correction_for_frame(
                measured_ee_frame,
                geometry=_PICK_OBJECT_GRIPPER_GEOMETRY,
            )
            tcp_contact_z = float(tcp_base[2]) - float(tcp_z_correction)
            dx_obj = bx - tcp_base[0]
            dy_obj = by - tcp_base[1]
            dz_obj = tcp_contact_z - obj_ref_z
            try:
                max_tcp_above_obj_z = _get_pick_object_params().max_tcp_above_obj_z
            except Exception:
                max_tcp_above_obj_z = 0.045
            try:
                object_xy_gate_tol = _get_pick_object_params().object_xy_gate_tol_m
            except Exception:
                object_xy_gate_tol = 0.030
            try:
                tcp_obj_dist_max = _get_pick_object_params().tcp_obj_dist_max_m
            except Exception:
                tcp_obj_dist_max = 0.055
            tcp_obj_dist = math.sqrt((dx_obj * dx_obj) + (dy_obj * dy_obj) + (dz_obj * dz_obj))
            panel._emit_log(
                f"[PICK_OBJ][TF][CHECK] object_xy_error dx={dx_obj:.3f} dy={dy_obj:.3f} tol={object_xy_gate_tol:.3f}"
            )
            panel._emit_log(
                f"[PICK_OBJ][TF][CHECK] object_z_gap frame={base_frame} tcp_z={tcp_base[2]:.3f} "
                f"tcp_contact_z={tcp_contact_z:.3f} tcp_z_offset={float(tcp_z_correction):.3f} "
                f"obj_center_z={obj_center_z:.3f} obj_top_z={obj_top_z:.3f} "
                f"obj_ref_z={obj_ref_z:.3f} z_ref_mode={attach_z_ref_mode} "
                f"z_clearance={attach_z_clearance:.3f} dz={dz_obj:.3f} max={max_tcp_above_obj_z:.3f} "
                f"tcp_stamp_ns={_tf_stamp_ns(_tcp_tf)} obj_stamp_ns={obj_base_stamp_ns}"
            )
            panel._emit_log(
                f"[PICK_OBJ][GRASP_VALIDATE] tcp_obj_dist={tcp_obj_dist:.3f} "
                f"tcp_obj_dist_max={tcp_obj_dist_max:.3f} "
                f"expected_obj_base=({bx:.3f},{by:.3f},{bz:.3f}) "
                f"tcp_base=({float(tcp_base[0]):.3f},{float(tcp_base[1]):.3f},{float(tcp_base[2]):.3f}) "
                f"tcp_contact_z={tcp_contact_z:.3f}"
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
                max_obj_move_before_close = _get_pick_object_params().max_obj_move_before_close_m
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
            attach_result = {"ok": False, "strict_physical_pending": False}
            # F2-step1: PANEL_ATTACH_XY_TOL_M / PANEL_ATTACH_Z_TOL_M centralizadas.
            try:
                _po_p = _get_pick_object_params()
                attach_xy_tol = float(_po_p.attach_xy_tol_m)
                attach_z_tol = float(_po_p.attach_z_tol_m)
            except Exception:
                attach_xy_tol = 0.02
                attach_z_tol = 0.04
            if deterministic_joint_after_approach:
                attach_xy_tol = max(attach_xy_tol, 0.22)
                attach_z_tol = max(attach_z_tol, 0.13)
                panel._emit_log(
                    f"[PICK_OBJ][MODE] deterministic attach_tol xy={attach_xy_tol:.3f} z={attach_z_tol:.3f}"
                )

            def _close_grasp_attach():
                panel._command_gripper(True, log_action="PICK", force=True)
                if strict_physics_mode:
                    attach_result["ok"] = True
                    attach_result["strict_physical_pending"] = True
                    panel._emit_log(
                        "[PICK_OBJ][STRICT] attach lógico diferido; esperando prueba física en lift"
                    )
                    return
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
                    tcp_z_correction_m=0.0,  # rg2_pinch_center ya es el frame de contacto
                )
                attach_result["ok"] = bool(attach_ok)
                if attach_ok:
                    update_object_state(
                        obj_name,
                        logical_state=ObjectLogicalState.GRASPED,
                        owner=ObjectOwner.ROBOT,
                        attached=True,
                        reason="pick_object_attach_tracking",
                    )
                    panel._emit_log(
                        "[PICK_OBJ][STATE] attach lógico publicado; "
                        "tracking interno marcado attached=true y CARRIED diferido hasta validar transporte post-lift"
                    )
                else:
                    panel._emit_log("[PICK_OBJ] ✗ Attach fallido (sin objeto en rango)")
            
            _step_phase_gate(
                "GRASP_ATTACH",
                position=grasp_pose_send.get("position"),
                decision="close_and_attach",
                object_position=(bx, by, bz),
            )
            panel.signal_run_ui.emit(_close_grasp_attach)
            time.sleep(1.0)  # Esperar más (como PICK_DEMO) para que attach se procese en Gazebo
            
            # Verificar que el attach fue exitoso
            if not attach_result["ok"]:
                raise RuntimeError("fallo al fijar objeto (attach no publicado)")
            if attach_result.get("strict_physical_pending"):
                _ensure_strict_pre_lift_contact(grasp_pose_send, grasp_delay)
                _ensure_strict_probe_carry(grasp_pose_send, grasp_delay)
            if not attach_result.get("strict_physical_pending"):
                obj_state = get_object_state(obj_name)
                if obj_state is None or obj_state.logical_state not in (
                    ObjectLogicalState.GRASPED,
                    ObjectLogicalState.CARRIED,
                ):
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
            else:
                panel._emit_log(
                    "[PICK_OBJ][STRICT] FASE 6: cierre completado; attach lógico pendiente de prueba física"
                )
            _canonical_phase(
                "GRASP_ACQUIRED",
                detail=f"attach_ok={str(bool(attach_result.get('ok'))).lower()} object={obj_name}",
            )

            panel._emit_log("[PICK_OBJ] FASE 7: Levantar objeto 20cm (MoveIt)")
            _step_phase_gate("LIFT", position=lift_pose, decision="lift_object", object_position=(bx, by, bz))
            _moveit2_log("PHASE", "LIFT start")
            if attach_result.get("strict_physical_pending"):
                _run_strict_staged_lift(grasp_pose_send)
                mark_object_grasped(obj_name, reason="pick_object_strict_probe")
                if not mark_object_attached(obj_name, reason="pick_object_strict_probe"):
                    raise RuntimeError("fallo al comprometer estado CARRIED tras prueba física estricta")
                panel._selected_object = None
                panel._selected_px = None
                panel._selected_world = None
                panel.obj_panel.set_selected(None, "")
                panel._emit_log(
                    "[PICK_OBJ][STRICT] prueba física superada; attach lógico comprometido tras lift"
                )
                panel._emit_log("[PICK_OBJ] FASE 6 COMPLETADA: Objeto agarrado")
            else:
                staged_lift_force = _get_pick_object_params().staged_lift_always
                if deterministic_joint_after_approach:
                    panel._emit_log(
                        "[PICK_OBJ][MODE] deterministic_joint_path active; LIFT por ruta articular"
                    )
                    _run_joint_step(
                        "LIFT_WITH_OBJECT_JOINT",
                        JOINT_PICK_IMAGE_POSE_RAD,
                        timeout_sec=carry_joint_move_sec + 3.0,
                        tol_rad=0.10,
                        duration_sec=carry_joint_move_sec,
                    )
                elif staged_lift_force:
                    panel._emit_log(
                        "[PICK_OBJ][MODE] staged_lift_always=true; LIFT por etapas"
                    )
                    _run_strict_staged_lift(grasp_pose_send)
                else:
                    _run_moveit_step(*sequence[3])  # LIFT
                try:
                    post_lift_max_dist_m = _get_pick_object_params().post_lift_max_dist_m
                except Exception:
                    post_lift_max_dist_m = 0.18
                if deterministic_joint_after_approach:
                    post_lift_max_dist_m = max(post_lift_max_dist_m, 0.22)
                try:
                    post_lift_min_consecutive = _get_pick_object_params().post_lift_min_consecutive
                except Exception:
                    post_lift_min_consecutive = 2
                _assert_carry_coherence_after_lift(
                    max_dist_override=max(
                        0.10,
                        deterministic_carry_gate_max_m
                        if deterministic_joint_after_approach
                        else post_lift_max_dist_m,
                    ),
                    min_consecutive_override=max(
                        1,
                        deterministic_carry_gate_min_consecutive
                        if deterministic_joint_after_approach
                        else post_lift_min_consecutive,
                    ),
                    gate_label="after_lift",
                )
                if not mark_object_attached(obj_name, reason="pick_object_post_lift_gate"):
                    raise RuntimeError("fallo al comprometer estado CARRIED tras carry gate")
                panel._emit_log(
                    "[PICK_OBJ][STATE] transporte validado; estado lógico promovido a CARRIED"
                )
            _canonical_phase(
                "LIFT_SUCCEEDED",
                detail=f"target_z={lift_pose[2]:.3f} object={obj_name}",
            )

            return_to_table_after_grasp = str(
                os.environ.get(
                    "PANEL_PICK_OBJECT_RETURN_TO_MESA",
                    "0" if moveit_exclusive else "1",
                )
            ).strip().lower() not in ("0", "false", "no", "off")
            if return_to_table_after_grasp:
                panel._emit_log("[PICK_OBJ] FASE 8: Volver a MESA con objeto agarrado (joint)")
                _step_phase_gate("MESA_WITH_OBJECT", position=None, decision="joint_return_to_table", object_position=(bx, by, bz))
                _run_joint_step(
                    "MESA_WITH_OBJECT",
                    JOINT_TABLE_POSE_RAD,
                    timeout_sec=carry_joint_move_sec + 3.0,
                    tol_rad=0.06,
                    duration_sec=carry_joint_move_sec,
                )
                _assert_carry_coherence_after_lift(
                    max_dist_override=(deterministic_carry_gate_max_m if deterministic_joint_after_approach else None),
                    min_consecutive_override=(deterministic_carry_gate_min_consecutive if deterministic_joint_after_approach else None),
                    gate_label="mesa_with_object",
                )

            home_before_basket = str(
                os.environ.get(
                    "PANEL_PICK_OBJECT_HOME_BEFORE_CESTA",
                    "0" if moveit_exclusive else "1",
                )
            ).strip().lower() not in ("0", "false", "no", "off")
            if home_before_basket:
                panel._emit_log("[PICK_OBJ] FASE 8.5: Volver a HOME con objeto (joint)")
                _step_phase_gate("HOME_WITH_OBJECT", position=None, decision="joint_return_home", object_position=(bx, by, bz))
                _run_joint_step(
                    "HOME_WITH_OBJECT",
                    home_pose,
                    timeout_sec=carry_joint_move_sec + 3.0,
                    tol_rad=0.06,
                    duration_sec=carry_joint_move_sec,
                )
                _assert_carry_coherence_after_lift(
                    max_dist_override=(deterministic_carry_gate_max_m if deterministic_joint_after_approach else None),
                    min_consecutive_override=(deterministic_carry_gate_min_consecutive if deterministic_joint_after_approach else None),
                    gate_label="home_with_object",
                )

            basket_joint_only_raw = os.environ.get(
                "PANEL_PICK_OBJECT_TRANSPORT_JOINT_ONLY",
                os.environ.get(
                    "PANEL_PICK_OBJECT_BASKET_JOINT_ONLY",
                    "0" if moveit_exclusive else "1",
                ),
            )
            transport_joint_only = str(basket_joint_only_raw).strip().lower() not in (
                "0",
                "false",
                "no",
                "off",
            )
            if deterministic_joint_after_approach:
                transport_joint_only = True
            panel._emit_log(
                "[PICK_OBJ] FASE 9: Ir a CESTA con objeto "
                + ("(joint)" if transport_joint_only else "(MoveIt)")
            )
            _step_phase_gate("TRANSPORT", position=transport_pose, decision="transport_to_basket", object_position=(bx, by, bz))
            _moveit2_log(
                "PHASE",
                "TRANSPORT start mode=" + ("joint" if transport_joint_only else "moveit"),
            )
            transport_fallback_used = False
            if transport_joint_only:
                _run_joint_step(
                    "CESTA_WITH_OBJECT",
                    JOINT_BASKET_POSE_RAD,
                    timeout_sec=carry_joint_move_sec + 3.0,
                    tol_rad=0.06,
                    duration_sec=carry_joint_move_sec,
                )
                _assert_carry_coherence_after_lift(
                    max_dist_override=(deterministic_carry_gate_max_m if deterministic_joint_after_approach else None),
                    min_consecutive_override=(deterministic_carry_gate_min_consecutive if deterministic_joint_after_approach else None),
                    gate_label="cesta_with_object",
                )
                transport_fallback_used = True
            else:
                try:
                    if transport_split_moveit:
                        transport_stage_labels = [
                            f"TRANSPORT_STAGE_{stage_idx}"
                            for stage_idx in range(1, len(transport_stage_data) + 1)
                        ]
                        via_summary = ", ".join(
                            f"{stage_label}=({stage_pose[0]:.3f}, {stage_pose[1]:.3f}, {stage_pose[2]:.3f})"
                            for stage_label, stage_pose in zip(transport_stage_labels, transport_stage_poses)
                        )
                        panel._emit_log(
                            "[PICK_OBJ][MOVEIT][PLAN] TRANSPORT split_moveit=true "
                            f"stages={len(transport_stage_data)} "
                            f"{via_summary} "
                            f"final=({transport_pose[0]:.3f}, {transport_pose[1]:.3f}, {transport_pose[2]:.3f})"
                        )
                        for stage_idx, stage_data in enumerate(transport_stage_data, start=1):
                            _run_moveit_step(
                                f"TRANSPORT_STAGE_{stage_idx}",
                                stage_data,
                                float(sequence[4][2]),
                            )
                    else:
                        _run_moveit_step(*sequence[4])  # TRANSPORT
                except RuntimeError as exc:
                    msg = str(exc)
                    if transport_split_moveit:
                        transport_labels = tuple(
                            ["TRANSPORT"]
                            + [
                                f"TRANSPORT_STAGE_{stage_idx}"
                                for stage_idx in range(1, len(transport_stage_data) + 1)
                            ]
                        )
                    else:
                        transport_labels = ("TRANSPORT",)
                    transport_tf_mismatch = any(
                        _is_step_tf_mismatch(msg, stage_label) for stage_label in transport_labels
                    )
                    transport_exec_failed = any(
                        _is_step_exec_failed(msg, stage_label) for stage_label in transport_labels
                    )
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
                        timeout_sec=carry_joint_move_sec + 3.0,
                        tol_rad=0.06,
                        duration_sec=carry_joint_move_sec,
                    )
                    _assert_carry_coherence_after_lift(
                        max_dist_override=(deterministic_carry_gate_max_m if deterministic_joint_after_approach else None),
                        min_consecutive_override=(deterministic_carry_gate_min_consecutive if deterministic_joint_after_approach else None),
                        gate_label="cesta_with_object_fallback"
                    )
                    transport_fallback_used = True
            _canonical_phase(
                "TRANSPORT_SUCCEEDED",
                detail=f"fallback={str(bool(transport_fallback_used)).lower()} target=({transport_pose[0]:.3f},{transport_pose[1]:.3f},{transport_pose[2]:.3f})",
            )

            if transport_fallback_used:
                panel._emit_log(
                    "[PICK_OBJ] FASE 10: Drop MoveIt omitido (trayectoria articular a cesta)"
                )
                _step_phase_gate("DROP", position=drop_pose, decision="drop_phase", object_position=(bx, by, bz))
            else:
                panel._emit_log("[PICK_OBJ] FASE 10: Descenso de drop en cesta (MoveIt)")
                _step_phase_gate("DROP", position=drop_pose, decision="drop_phase", object_position=(bx, by, bz))
                _moveit2_log("PHASE", "DROP start")
                _run_moveit_step(*sequence[5])  # DROP

            panel._emit_log("[PICK_OBJ] FASE 11: Soltar objeto en cesta")
            _step_phase_gate("RELEASE", position=drop_pose, decision="release_object", object_position=(bx, by, bz))
            _moveit2_log("PHASE", "RELEASE start")

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
                detach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj_name}/detach"
                detach_sent = False
                try:
                    if Empty is None:
                        panel._log_warning(
                            f"[PICK_OBJ][DETACH] skipped topic={detach_topic} reason=empty_msg_unavailable"
                        )
                    else:
                        detach_pub = panel._get_attach_publisher(detach_topic)
                        detach_subs = -1
                        if detach_pub is not None and panel._ros_worker_started and panel.ros_worker.node_ready():
                            detach_subs = int(panel.ros_worker.topic_subscriber_count(detach_topic))
                        if detach_pub is None:
                            panel._log_warning(
                                f"[PICK_OBJ][DETACH] skipped topic={detach_topic} reason=publisher_unavailable"
                            )
                        elif detach_subs == 0:
                            panel._log_warning(
                                f"[PICK_OBJ][DETACH] skipped topic={detach_topic} reason=no_subscribers"
                            )
                        else:
                            detach_pub.publish(Empty())
                            detach_sent = True
                            panel._emit_log(
                                f"[PICK_OBJ][DETACH] publish object={obj_name} topic={detach_topic}"
                            )
                except Exception as exc:
                    panel._log_warning(
                        f"[PICK_OBJ][DETACH] error topic={detach_topic} exc={exc}"
                    )
                panel._emit_log(
                    f"[PICK_OBJ] POST-CHECK: Transición {current_state.logical_state if current_state else '?'} → RELEASED"
                )
                mark_object_released(obj_name, reason="pick_object")
                panel._emit_log(
                    f"[PICK_OBJ] POST-CHECK: {obj_name} estado = RELEASED ✓"
                )
                if not detach_sent:
                    panel._emit_log(
                        f"[PICK_OBJ][DETACH] pending_manual_check object={obj_name} topic={detach_topic}"
                    )

            panel.signal_run_ui.emit(_open_and_release)
            time.sleep(1.0)
            _canonical_phase(
                "RELEASE_SUCCEEDED",
                detail=f"object={obj_name} detach_topic={GRIPPER_ATTACH_PREFIX}/{obj_name}/detach",
            )
            panel._emit_log("[PICK_OBJ] Cerrando gripper (limpieza)")
            panel.signal_run_ui.emit(
                lambda: panel._command_gripper(True, log_action="DROP", force=True)
            )
            time.sleep(0.5)

            panel._emit_log("[PICK_OBJ] FASE 12: Regresar a HOME")
            _step_phase_gate("HOME_FINAL", position=None, decision="return_home", object_position=(bx, by, bz))
            _moveit2_log("PHASE", "HOME_FINAL start")
            _run_joint_step("HOME_FINAL", home_pose)

            panel._ui_set_status("Pick objeto: COMPLETADO ✓")
            panel._emit_log("[PICK_OBJ] === SECUENCIA COMPLETADA EXITOSAMENTE ===")
            _moveit2_log("VERDICT", "completed=true")
            _canonical_phase("HOME_DONE", detail="pick_object_sequence_completed")
            _canonical_finish(True, "tfm_moveit_sequence_completed", "HOME_DONE")
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
            panel._emit_log(
                f"[PICK][MOVEIT][ERROR] type={fail_kind} detail={err_txt}"
            )
            _moveit2_log("FAIL", f"type={fail_kind} detail={err_txt}")
            try:
                panel._emit_log("[PICK_OBJ][RECOVERY] Error detectado; intentando HOME_SAFE")
                _run_joint_step("HOME_SAFE", home_pose, timeout_sec=move_sec + 3.0, tol_rad=0.08)
            except Exception as home_exc:
                panel._emit_log(f"[PICK_OBJ][RECOVERY] HOME_SAFE falló: {home_exc}")
            _block(f"worker_exception:{exc}")
            panel._ui_set_status(f"Error en pick objeto: {exc}", error=True)
            panel._emit_log(f"[PICK_OBJ] ✗ ERROR: {exc}")
            panel._emit_log(f"[PICK_OBJ] Traceback: {traceback.format_exc()}")
            _canonical_finish(False, f"{fail_kind}:{err_txt}", "FAIL_TERMINAL")
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
            setattr(panel, "_pick_object_worker_started", False)
            panel._set_motion_lock(False)

    setattr(panel, "_pick_object_worker_started", True)
    panel._run_async(worker)
