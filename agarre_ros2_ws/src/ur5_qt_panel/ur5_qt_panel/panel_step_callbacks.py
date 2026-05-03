#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_step_callbacks.py
# Contenido: Step-by-step pipeline callbacks extracted from ControlPanelV2 in panel_v2.py.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Step pipeline callbacks for ControlPanelV2."""
from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QPushButton, QTableWidgetItem

from .panel_config import (
    GRIPPER_CLOSED_RAD,
    GRIPPER_JOINT_NAMES,
    GRIPPER_OPEN_RAD,
    UR5_JOINT_NAMES,
)
from .panel_utils import (
    get_object_positions,
    world_to_base,
)
from .panel_runtime_pose_auditor import (
    build_runtime_audit_snapshot,
    compute_step_history_metrics,
    runtime_status_style,
)
from .step_pipeline_helpers import (
    step_phase_action_text,
    step_phase_gripper_state,
    step_phase_intent,
    step_phase_sequence,
    step_predict_next_phase,
)
from .tf_pose_utils import (
    get_transform as tf_get_transform,
    get_tcp_in_base as tf_get_tcp_in_base,
)
from .panel_objects import get_object_states
from .panel_step_ui import build_step_cart_debug_window
from .panel_utils import base_to_world
from .step_pipeline_helpers import step_present_flow_name
from .ur5_kinematics import fk_ur5, ik_ur5
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME
from .panel_objects import ObjectLogicalState
from .logging_utils import emit_log_line


def _normalize_joint_name(name) -> str:
    text = str(name).strip()
    if "::" in text:
        text = text.split("::")[-1]
    if "/" in text:
        text = text.split("/")[-1]
    return text.strip()


def _log_exception(context: str, exc: Exception) -> None:
    emit_log_line(f"[STEP_CB][ERROR][{context}] {exc}")


def _step_cartesian_move_runtime_target(panel,
    target_tcp_runtime: Tuple[float, float, float],
    timeout_sec: float,
    *,
    axis_tol_m: float,
) -> Tuple[bool, str, Optional[Tuple[float, float, float]], Optional[Tuple[float, float, float]]]:
    base_frame = panel._business_base_frame()
    joint_snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
    seed: List[float] = []
    missing = []
    for name in UR5_JOINT_NAMES:
        if name in joint_snapshot:
            seed.append(float(joint_snapshot[name]))
        else:
            missing.append(name)
    if len(seed) != len(UR5_JOINT_NAMES):
        seed = [float(v) for v in panel._get_home_joint_pose()]
        seed_source = "home_fallback"
    else:
        seed_source = "joint_state_snapshot"

    try:
        _seed_pos, target_rot = fk_ur5(seed)
    except Exception as exc:
        return False, f"fk_seed_failed:{exc}", None, None

    local_offset = None
    tf_tool0_ee, tf_reason = tf_get_transform("tool0", "rg2_pinch_center", timeout=0.10, logger=None)
    if tf_tool0_ee is not None:
        tr = tf_tool0_ee.transform.translation
        local_offset = (float(tr.x), float(tr.y), float(tr.z))
        offset_source = "tf:tool0<-rg2_pinch_center"
    else:
        local_offset = _canonical_tool0_to_semantic_frame("rg2_pinch_center")
        if local_offset is None:
            return False, "canonical_tool0_offset_unavailable", None, None
        offset_source = f"fallback:urdf_canonical:{tf_reason or 'tf_unavailable'}"

    target_model = (
        -float(target_tcp_runtime[0]),
        -float(target_tcp_runtime[1]),
        float(target_tcp_runtime[2]),
    )
    offset_vector = (
        float(target_rot[0, 0]) * float(local_offset[0])
        + float(target_rot[0, 1]) * float(local_offset[1])
        + float(target_rot[0, 2]) * float(local_offset[2]),
        float(target_rot[1, 0]) * float(local_offset[0])
        + float(target_rot[1, 1]) * float(local_offset[1])
        + float(target_rot[1, 2]) * float(local_offset[2]),
        float(target_rot[2, 0]) * float(local_offset[0])
        + float(target_rot[2, 1]) * float(local_offset[1])
        + float(target_rot[2, 2]) * float(local_offset[2]),
    )
    target_ik = (
        float(target_model[0]) - float(offset_vector[0]),
        float(target_model[1]) - float(offset_vector[1]),
        float(target_model[2]) - float(offset_vector[2]),
    )

    solved_q, err_norm, ik_ok = ik_ur5(
        target_ik,
        target_rot,
        seed,
        max_iter=240,
        pos_weight=1.0,
        rot_weight=1.20,
        joint_weight=0.02,
    )
    if (not ik_ok) or float(err_norm) > 0.030:
        return False, f"ik_failed err_norm={float(err_norm):.4f}", None, None

    two_pi = 2.0 * math.pi
    solved_q_list = [
        float(q) + two_pi * round((float(s) - float(q)) / two_pi)
        for q, s in zip([float(v) for v in solved_q.tolist()], seed)
    ]

    dist_m = math.sqrt(
        float(target_tcp_runtime[0]) ** 2 + float(target_tcp_runtime[1]) ** 2 + float(target_tcp_runtime[2]) ** 2
    )
    move_sec = max(0.8, min(2.5, 0.8 + dist_m * 0.0))
    ok, info = panel._publish_joint_trajectory(solved_q_list, move_sec)
    if not ok:
        return False, f"joint_publish_failed:{info}", None, None

    deadline = time.time() + max(0.5, float(timeout_sec))
    final_pos: Optional[Tuple[float, float, float]] = None
    err_xyz: Optional[Tuple[float, float, float]] = None
    while time.time() < deadline:
        final_pos = panel._step_fetch_live_pose("rg2_pinch_center")
        if final_pos is None:
            time.sleep(0.05)
            continue
        err_xyz = (
            float(target_tcp_runtime[0]) - float(final_pos[0]),
            float(target_tcp_runtime[1]) - float(final_pos[1]),
            float(target_tcp_runtime[2]) - float(final_pos[2]),
        )
        if (
            abs(float(err_xyz[0])) <= float(axis_tol_m)
            and abs(float(err_xyz[1])) <= float(axis_tol_m)
            and abs(float(err_xyz[2])) <= float(axis_tol_m)
        ):
            return True, f"ok seed={seed_source} offset={offset_source}", final_pos, err_xyz
        time.sleep(0.05)

    if final_pos is not None and err_xyz is not None:
        return (
            False,
            (
                f"target_not_reached frame={base_frame} final={panel._step_format_inline_xyz(final_pos)} "
                f"err_xyz=({err_xyz[0]:+.4f},{err_xyz[1]:+.4f},{err_xyz[2]:+.4f}) "
                f"tol_axis={axis_tol_m:.4f} seed={seed_source} offset={offset_source}"
            ),
            final_pos,
            err_xyz,
        )
    return False, "target_not_reached_no_tf", final_pos, err_xyz

# F3: callbacks del cart_debug window extraídas a step_cart_debug.py.
# Re-exportadas aquí para preservar la API pública (panel_v2 y otros
# importan estos nombres directamente desde panel_step_callbacks).
from .step_cart_debug import (  # noqa: F401
    _ensure_step_cart_debug_window,
    _show_step_cart_debug_window,
    _step_cart_debug_handle_axis,
    _step_cart_debug_move_delta,
    _step_cart_debug_run_validation_xyz,
)

# F3: pipeline view + direct waiting helpers extraídos a step_pipeline_view.py.
# Re-exportadas para preservar API pública.
from .step_pipeline_view import (  # noqa: F401
    _direct_clear_waiting_for_approach_confirmation,
    _direct_enter_waiting_for_approach_confirmation,
    _direct_release_waiting_for_approach_confirmation,
    _direct_waiting_for_approach_confirmation,
    _step_effective_flow,
    _step_find_history_row,
    _step_phase_action_text,
    _step_phase_completed,
    _step_phase_gate_already_owned,
    _step_phase_gripper_state,
    _step_phase_intent,
    _step_phase_sequence,
    _step_pipeline_phase_state,
    _step_pipeline_rebuild,
    _step_predict_next_phase,
    _step_prepare_pipeline_view,
    _step_present_flow_name,
    _step_refresh_pipeline_table,
    _step_reset_sequence_view,
    _step_window_maybe_refresh,
)

def _step_live_gripper_state(panel) -> str:
    state = panel._read_gripper_feedback_state()
    command_closed = bool(state.get("command_closed"))
    command_txt = "Cerrada" if command_closed else "Abierta"
    inferred_state = str(state.get("inferred_state") or "unknown")
    age = state.get("joint_state_age_sec")
    age_ok = age is None or float(age) <= 0.5
    if not state.get("joint_positions"):
        return f"Sin medida | comando: {command_txt}"
    if not age_ok:
        age_txt = f"{float(age):.2f}s" if age is not None else "--"
        return f"Medida obsoleta | comando: {command_txt} | age: {age_txt}"
    if inferred_state == "open":
        measured_txt = "Abierta"
    elif inferred_state == "closed":
        measured_txt = "Cerrada"
    else:
        measured_txt = "Indeterminada"
    if measured_txt == command_txt:
        return f"{measured_txt} (medida)"
    return f"{measured_txt} (medida) | comando: {command_txt}"

def _read_gripper_feedback_state(panel) -> Dict[str, object]:
    positions: Dict[str, float] = {}
    joint_state_age_sec = None
    if panel._ros_worker_started and panel.ros_worker is not None:
        try:
            payload, wall_ts = panel.ros_worker.get_last_joint_state()
        except Exception:
            payload, wall_ts = None, 0.0
        if payload:
            try:
                names = [
                    _normalize_joint_name(str(name))
                    for name in (payload.get("name") or [])
                ]
                pos_list = payload.get("position") or []
            except Exception:
                names, pos_list = [], []
            for joint_name, pos in zip(names, pos_list):
                if joint_name not in GRIPPER_JOINT_NAMES:
                    continue
                try:
                    positions[joint_name] = float(pos)
                except Exception:
                    continue
            if wall_ts:
                try:
                    joint_state_age_sec = max(0.0, time.time() - float(wall_ts))
                except Exception:
                    joint_state_age_sec = None
    if not positions:
        joint_snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
        for joint_name in GRIPPER_JOINT_NAMES:
            if joint_name in joint_snapshot:
                positions[joint_name] = float(joint_snapshot[joint_name])
    open_target = abs(float(GRIPPER_OPEN_RAD))
    closed_target = abs(float(GRIPPER_CLOSED_RAD))
    open_err = None
    closed_err = None
    inferred_state = "unknown"
    opening_sum = None
    if positions:
        magnitudes = [abs(float(pos)) for pos in positions.values()]
        opening_sum = float(sum(magnitudes))
        open_err = float(max(abs(mag - open_target) for mag in magnitudes))
        closed_err = float(max(abs(mag - closed_target) for mag in magnitudes))
        if open_err < closed_err:
            inferred_state = "open"
        elif closed_err < open_err:
            inferred_state = "closed"
    return {
        "joint_positions": positions,
        "joint_state_age_sec": joint_state_age_sec,
        "opening_sum": opening_sum,
        "open_err": open_err,
        "closed_err": closed_err,
        "inferred_state": inferred_state,
        "command_closed": bool(getattr(panel, "_gripper_is_closed", getattr(panel, "_gripper_closed", False))),
    }

# F3: pose formatters y fetch helpers extraídos a step_pose_format.py.
# Re-exportados aquí para preservar la API pública.
from .step_pose_format import (  # noqa: F401
    _step_display_position,
    _step_fetch_live_pose,
    _step_format_inline_rpy,
    _step_format_inline_xyz,
    _step_format_xyz,
    _step_live_pose_text,
    _step_operational_frame_name,
)

def _step_assess_target_reached(panel,
    target: Optional[Tuple[float, float, float]],
    actual: Optional[Tuple[float, float, float]],
) -> Optional[bool]:
    if target is None or actual is None:
        return None
    try:
        dx = abs(float(actual[0]) - float(target[0]))
        dy = abs(float(actual[1]) - float(target[1]))
        dz = abs(float(actual[2]) - float(target[2]))
    except Exception:
        return None
    return (
        dx <= float(panel._step_history_target_xy_tol_m)
        and dy <= float(panel._step_history_target_xy_tol_m)
        and dz <= float(panel._step_history_target_z_tol_m)
    )

def _step_status_item(panel,
    reached: Optional[bool],
    *,
    row_state: str = "",
    row_kind: str = "",
) -> QTableWidgetItem:
    state_name = str(row_state or "").strip().upper()
    kind_name = str(row_kind or "").strip().upper()
    if kind_name == "EVENT" or state_name == "EVENT_SNAPSHOT":
        item = QTableWidgetItem("EVENT")
        item.setBackground(QColor(168, 85, 247))
    elif state_name == "PHASE_READY":
        item = QTableWidgetItem("READY")
        item.setBackground(QColor(148, 163, 184))
    elif state_name == "PHASE_RUNNING":
        item = QTableWidgetItem("RUN")
        item.setBackground(QColor(59, 130, 246))
    elif state_name == "PHASE_DONE":
        item = QTableWidgetItem("DONE")
        item.setBackground(QColor(34, 197, 94))
    elif state_name == "PHASE_BLOCKED":
        item = QTableWidgetItem("BLOCK")
        item.setBackground(QColor(245, 158, 11))
    elif state_name == "PHASE_ABORTED":
        item = QTableWidgetItem("ABORT")
        item.setBackground(QColor(239, 68, 68))
    elif reached is True:
        item = QTableWidgetItem("OK")
        item.setBackground(QColor(34, 197, 94))
    elif reached is False:
        item = QTableWidgetItem("NO")
        item.setBackground(QColor(239, 68, 68))
    else:
        item = QTableWidgetItem("PEND")
        item.setBackground(QColor(245, 158, 11))
    item.setTextAlignment(Qt.AlignCenter)
    return item

# F3: history recorder extraído a step_history_recorder.py.
# Re-exportadas para preservar API pública.
from .step_history_recorder import (  # noqa: F401
    _step_capture_start_pose,
    _step_pre_insert_inicio_row,
    _step_record_direct_event_snapshot,
    _step_record_direct_home_initial,
    _step_record_direct_initial_snapshot,
    _step_record_direct_mesa_ready,
    _step_record_history,
    _step_upsert_history_row_ordered,
)

# F3: step_window_refresh + step_runtime_refresh extraídos.
from .step_window_refresh import (  # noqa: F401
    _step_runtime_refresh,
    _step_window_refresh,
)

def _step_update_phase_result(panel,
    *,
    phase: str,
    tcp_tf_actual: Optional[Tuple[float, float, float]],
    object_world: Optional[Tuple[float, float, float]],
    dist_tcp_obj: Optional[float],
    check_reason: str,
    ok: Optional[bool],
    tf_visual_gap: Optional[float] = None,
) -> None:
    """Actualiza la fila de una fase con datos de un evaluador externo.

    Se llama desde el hilo UI (via signal_run_ui.emit).  Busca la fila de la
    fase indicada (de atrás hacia delante) y sobreescribe los campos de resultado,
    asegurando que el invariante Cierre[N]=Org[N+1] en _step_wait_for_phase
    no vuelva a pisarlos (porque 'actual' ya no será None).
    """
    target_phase = str(phase).strip().upper()
    for row in reversed(panel._step_history_rows):
        if str(row.get("phase", "")).strip().upper() == target_phase:
            if tcp_tf_actual is not None:
                row["actual"] = tuple(float(v) for v in tcp_tf_actual)
            if object_world is not None:
                row["object_world_snapshot"] = tuple(float(v) for v in object_world)
            if dist_tcp_obj is not None:
                row["dist_tcp_obj_snapshot"] = float(dist_tcp_obj)
            row["check_reason"] = str(check_reason)
            row["reached"] = ok
            if str(row.get("row_kind") or "").strip().upper() == "PHASE":
                if ok is True:
                    row["row_state"] = "PHASE_DONE"
                elif ok is False:
                    row["row_state"] = "PHASE_BLOCKED"
            if tf_visual_gap is not None:
                row["tf_visual_gap"] = float(tf_visual_gap)
            if target_phase == str(panel._step_running_phase or "").strip().upper():
                panel._step_running_phase = ""
            break
    if panel._step_history_table is not None:
        panel._step_window_refresh()

def _step_selected_object_name(panel) -> str:
    positions = get_object_positions() or {}
    for candidate in (
        str(getattr(panel, "_selected_object", "") or "").strip(),
        str(getattr(panel, "_selection_last_user_name", "") or "").strip(),
        str(getattr(panel, "_last_grasp_selection_name", "") or "").strip(),
        PICK_DEMO_OBJECT_NAME,
    ):
        if candidate and candidate in positions:
            return candidate
    try:
        for name, state in get_object_states().items():
            if getattr(state, "logical_state", None) == ObjectLogicalState.SELECTED and name in positions:
                return str(name)
    except Exception:
        pass
    if len(positions) == 1:
        try:
            return str(next(iter(positions.keys())))
        except Exception:
            return ""
    return ""

def _step_fetch_object_world(panel) -> Optional[Tuple[float, float, float]]:
    """Lee la pose del objeto activo en world desde el estado global."""
    try:
        positions = get_object_positions() or {}
        selected_name = panel._step_selected_object_name()
        if selected_name:
            selected_pos = positions.get(selected_name)
            if isinstance(selected_pos, (list, tuple)) and len(selected_pos) >= 3:
                return (
                    float(selected_pos[0]),
                    float(selected_pos[1]),
                    float(selected_pos[2]),
                )
        object_hint = getattr(panel, "_step_object_position", None)
        if isinstance(object_hint, (list, tuple)) and len(object_hint) >= 3:
            hint_world = panel._step_display_position(
                (float(object_hint[0]), float(object_hint[1]), float(object_hint[2]))
            )
            if hint_world is not None:
                return hint_world
        if len(positions) == 1:
            only_pos = next(iter(positions.values()))
            if isinstance(only_pos, (list, tuple)) and len(only_pos) >= 3:
                return (float(only_pos[0]), float(only_pos[1]), float(only_pos[2]))
    except Exception:
        pass
    return None

def _step_update_row_object_metrics(panel,
    row: Dict[str, object],
    actual_pose: Optional[Tuple[float, float, float]],
) -> None:
    obj_world = panel._step_fetch_object_world()
    row["object_world_snapshot"] = obj_world
    if actual_pose is None or obj_world is None:
        row["dist_tcp_obj_snapshot"] = None
        return
    try:
        obj_base_for_dist = world_to_base(
            float(obj_world[0]), float(obj_world[1]), float(obj_world[2])
        )
        import math as _math

        row["dist_tcp_obj_snapshot"] = _math.sqrt(
            sum((float(actual_pose[i]) - float(obj_base_for_dist[i])) ** 2 for i in range(3))
        )
    except Exception:
        row["dist_tcp_obj_snapshot"] = None

def _step_record_current_phase_actual(panel) -> None:
    if not panel._step_history_rows:
        return
    last_row = panel._step_history_rows[-1]
    if last_row.get("actual") is not None:
        return
    _op_frame = panel._step_operational_frame_name()
    actual_pose = panel._step_fetch_live_pose(_op_frame)
    _pose_src = "tf_live" if actual_pose is not None else "unavailable"
    _phase_tag = str(last_row.get("phase") or "").strip()
    _xyz_log = (
        f"({actual_pose[0]:.3f},{actual_pose[1]:.3f},{actual_pose[2]:.3f})"
        if actual_pose is not None else "none"
    )
    panel._emit_log(
        f"[STEP][TF_LIVE] available={str(actual_pose is not None).lower()} "
        f"frame={_op_frame} xyz={_xyz_log}"
    )
    panel._emit_log(
        f"[STEP][POSE_SOURCE] phase={_phase_tag} source={_pose_src} "
        f"frame={_op_frame} xyz={_xyz_log} "
        f"same_frame_as_backend=true same_source_as_backend=true"
    )
    # Sin fallback a caché stale: si TF no disponible, actual queda None → PEND
    last_row["actual"] = actual_pose
    reached = panel._step_assess_target_reached(last_row.get("target"), actual_pose)
    last_row["reached"] = reached
    if str(last_row.get("row_kind") or "").strip().upper() == "PHASE":
        if reached is True:
            last_row["row_state"] = "PHASE_DONE"
        elif reached is False:
            last_row["row_state"] = "PHASE_BLOCKED"

    # Capturar pose del objeto y distancia TCP↔objeto al cerrar la fase.
    # Esto debe congelarse incluso cuando la fase se cierra por handoff
    # implícito hacia la siguiente fila STEP.
    panel._step_update_row_object_metrics(last_row, actual_pose)

    # Razón legible del check
    if actual_pose is None:
        last_row["check_reason"] = "tcp_tf_unavailable"
    elif reached is True:
        last_row["check_reason"] = "ok"
    elif reached is False:
        tgt = last_row.get("target")
        if tgt is not None and actual_pose is not None:
            import math as _math
            _err = _math.sqrt(sum((actual_pose[i] - tgt[i]) ** 2 for i in range(3)))
            last_row["check_reason"] = f"pos_not_reached err={_err:.3f}m"
        else:
            last_row["check_reason"] = "pos_not_reached"
    else:
        last_row["check_reason"] = "pending"

    if _phase_tag.strip().upper() == str(panel._step_running_phase or "").strip().upper():
        panel._step_running_phase = ""

    if panel._step_history_table is not None:
        panel._step_window_refresh()

def _step_set_exec_target(panel, target_base) -> None:
    """Registra el target de ejecución real en la última fila de la tabla STEP.

    Se llama desde _phase_begin (hilo worker) inmediatamente después de que el
    gate se libera, para fijar en la columna Exec el target efectivo que el IK /
    sistema de movimiento va a usar.  Puede diferir del campo 'target' si hubo
    ajuste XY u otro override en la lógica previa al movimiento.

    - Modifica solo 'exec_target_snapshot' de la última fila.
    - No toca 'target', 'actual' ni 'origin_snapshot'.
    - No bloquea, no lanza excepciones.
    """
    if not panel._step_history_rows:
        return
    pos3 = None
    if isinstance(target_base, (list, tuple)) and len(target_base) >= 3:
        try:
            pos3 = (float(target_base[0]), float(target_base[1]), float(target_base[2]))
        except Exception:
            pos3 = None
    last_row = panel._step_history_rows[-1]
    last_row["exec_target_snapshot"] = pos3
    panel._emit_log(
        f"[STEP][EXEC_TARGET] "
        f"phase={str(last_row.get('phase') or '').strip().upper()} "
        f"exec={panel._step_format_inline_xyz(pos3)}"
    )

def _step_window_set_waiting(panel) -> None:
    panel._ensure_step_window()
    panel._step_window_refresh()
    if panel._step_window is not None:
        panel._step_window.show()

def _step_window_hide(panel) -> None:
    if panel._step_window is None:
        return
    panel._step_window.hide()
    if panel._step_cart_debug_window is not None:
        panel._step_cart_debug_window.hide()

def _on_step_window_finished(panel, _result: int) -> None:
    # If user closes the step window manually, return to AUTO to avoid deadlocks.
    if panel._closing:
        return
    if panel._step_mode == "STEP_BY_STEP":
        panel._set_step_mode("AUTO", emit_log=True)

def _on_step_phase_start_clicked(panel, phase: str) -> None:
    phase_name = str(phase or "").strip().upper()
    with panel._step_gate_lock:
        pending = str(panel._step_current_phase or "").strip().upper()
        active = bool(panel._step_wait_active)
        allowed = active and bool(phase_name) and pending == phase_name
        if allowed:
            panel._step_wait_active = False
            panel._step_running_phase = phase_name
            for row in reversed(panel._step_history_rows):
                if str(row.get("phase") or "").strip().upper() == phase_name:
                    if str(row.get("row_kind") or "").strip().upper() == "PHASE":
                        row["row_state"] = "PHASE_RUNNING"
                    break
            panel._step_wait_event.set()
    if allowed:
        panel._emit_log(f"[STEP] phase_start_requested phase={phase_name}")
        panel._step_window_refresh()
    else:
        panel._emit_log(
            f"[STEP] phase_start_ignored requested={phase_name or 'unknown'} "
            f"enabled_phase={pending or 'none'} active={str(active).lower()}"
        )

def _on_step_continue_clicked(panel) -> None:
    current_phase = str(panel._step_current_phase or "").strip().upper()
    if current_phase:
        panel._on_step_phase_start_clicked(current_phase)

def _step_wait_for_phase(panel, phase: str, *, flow: str = "", position=None, decision: str = "", object_position=None) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        return
    phase_name = str(phase or "").strip() or "UNKNOWN_PHASE"
    flow_name = str(flow or "general").strip() or "general"
    display_phase = phase_name.split(".", 1)[-1].strip().upper()
    next_phase = panel._step_predict_next_phase(flow_name, display_phase)

    with panel._step_gate_lock:
        panel._step_pending_flow = flow_name
        panel._step_pipeline_flow = flow_name
        panel._step_pending_phase = phase_name
        panel._step_current_phase = display_phase
        panel._step_next_phase = next_phase
        panel._step_running_phase = ""
        panel._step_decision = str(decision or "").strip()
        phase_pos = None
        if isinstance(position, (list, tuple)) and len(position) >= 3:
            try:
                phase_pos = (float(position[0]), float(position[1]), float(position[2]))
            except Exception:
                phase_pos = None
        panel._step_phase_position = phase_pos
        obj_pos = None
        if isinstance(object_position, (list, tuple)) and len(object_position) >= 3:
            try:
                obj_pos = (float(object_position[0]), float(object_position[1]), float(object_position[2]))
            except Exception:
                obj_pos = None
        panel._step_object_position = obj_pos
        panel._step_record_history(flow_name, display_phase, position)
        # Invariante semántico: Cierre[N] == Origen[N+1].
        # _step_record_history acaba de capturar origin_snapshot para la fase actual
        # con hasta 3 reintentos (3×80 ms). Usamos ese mismo valor como actual de la
        # fila anterior para garantizar que ambas columnas muestren exactamente la
        # misma pose (capturada con la mejor calidad disponible), sin depender de una
        # captura TF independiente que puede devolver datos stale del movimiento previo.
        if len(panel._step_history_rows) >= 2:
            _prev_row = panel._step_history_rows[-2]
            _curr_snap = panel._step_history_rows[-1].get("origin_snapshot")
            if _curr_snap is not None and _prev_row.get("actual") is None:
                # Invariante Cierre[N] == Origen[N+1].
                # Solo aplica si la fila anterior NO fue ya rellenada por un evaluador
                # (e.g. AttachGateEvaluator via _step_update_phase_result).
                _prev_row["actual"] = _curr_snap
                panel._step_update_row_object_metrics(_prev_row, _curr_snap)
                _prev_row["reached"] = panel._step_assess_target_reached(
                    _prev_row.get("target"), _curr_snap
                )
                if str(_prev_row.get("row_kind") or "").strip().upper() == "PHASE":
                    _prev_row["row_state"] = (
                        "PHASE_DONE"
                        if _prev_row.get("reached") is True
                        else "PHASE_BLOCKED"
                    )
                if not _prev_row.get("check_reason"):
                    _rv = _prev_row.get("reached")
                    import math as _math
                    _tgt = _prev_row.get("target")
                    if _rv is True:
                        _prev_row["check_reason"] = "ok"
                    elif _rv is False and _tgt is not None:
                        _e = _math.sqrt(sum(
                            (_curr_snap[i] - _tgt[i]) ** 2 for i in range(3)
                        ))
                        _prev_row["check_reason"] = f"pos_not_reached err={_e:.3f}m"
                    else:
                        _prev_row["check_reason"] = "pos_not_reached"
        panel._step_wait_active = True
        panel._step_wait_event.clear()

    panel._emit_log(f"[STEP] pending flow={flow_name} phase={phase_name}")
    panel.signal_run_ui.emit(panel._step_window_set_waiting)
    with panel._step_gate_lock:
        row_actual = None
        row_target = None
        if panel._step_history_rows:
            _row = panel._step_history_rows[-1]
            row_actual = _row.get("actual")
            row_target = _row.get("target")
        panel._emit_log(
            "[STEP][ROW_VISIBLE_BEFORE_CONTINUE] "
            f"flow={flow_name} phase={display_phase} "
            f"actual={panel._step_format_inline_xyz(row_actual)} "
            f"target={panel._step_format_inline_xyz(row_target)}"
        )

    reason = "button"
    while True:
        if panel._closing:
            reason = "closing"
            break
        if panel._step_mode != "STEP_BY_STEP":
            reason = "mode_auto"
            break
        if panel._step_wait_event.wait(timeout=0.1):
            reason = "button"
            break

    with panel._step_gate_lock:
        panel._step_wait_active = False
        panel._step_pending_phase = ""
        panel._step_wait_event.clear()
        if reason != "button":
            panel._step_running_phase = ""

    if panel._step_mode == "STEP_BY_STEP":
        panel.signal_run_ui.emit(panel._step_window_set_waiting)
    else:
        panel.signal_run_ui.emit(panel._step_window_hide)
    panel._emit_log(f"[STEP] release flow={flow_name} phase={phase_name} reason={reason}")
    if reason == "button":
        panel._emit_log(f"[STEP][PHASE_START] phase={display_phase} flow={flow_name}")

def _canonical_tool0_to_semantic_frame(frame_name: str):
    """Return (dx, dy, dz) offset from tool0 to the named semantic frame, or None."""
    try:
        from ur5_tools.gripper_geometry import tool0_offset_for_frame
        frame = str(frame_name or "").strip()
        if frame not in {"rg2_pinch_center", "rg2_tcp"}:
            return None
        return tool0_offset_for_frame(frame)
    except Exception:
        return None
