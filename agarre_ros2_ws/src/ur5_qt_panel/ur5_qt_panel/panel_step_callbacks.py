#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_step_callbacks.py
# Contenido: Step-by-step pipeline callbacks extracted from ControlPanelV2 in panel_v2.py.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Step pipeline callbacks for ControlPanelV2."""
from __future__ import annotations

import math
import os
import time
import threading
from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor, QFont
from PyQt5.QtWidgets import QPushButton, QTableWidgetItem

from .panel_config import (
    GRIPPER_CLOSED_RAD,
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


def _normalize_joint_name(name) -> str:
    text = str(name).strip()
    if "::" in text:
        text = text.split("::")[-1]
    if "/" in text:
        text = text.split("/")[-1]
    return text.strip()


def _log_exception(context: str, exc: Exception) -> None:
    print(f"[STEP_CB][ERROR][{context}] {exc}")


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

def _step_cart_debug_move_delta(panel, dx_m: float, dy_m: float, dz_m: float) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        panel._step_cart_debug_set_status("Solo disponible en STEP_BY_STEP", error=True)
        return
    if panel._step_cart_debug_move_inflight:
        panel._step_cart_debug_set_status("Movimiento en curso...", error=False)
        return
    before = panel._step_fetch_live_pose("rg2_pinch_center")
    if before is None:
        panel._step_cart_debug_set_status("TF rg2_pinch_center no disponible", error=True)
        panel._step_cart_debug_log_event(
            "move_rejected",
            reason="pinch_pose_unavailable",
            delta_m=[float(dx_m), float(dy_m), float(dz_m)],
        )
        return
    target = (
        float(before[0]) + float(dx_m),
        float(before[1]) + float(dy_m),
        float(before[2]) + float(dz_m),
    )
    panel._step_cart_debug_set_status("Ejecutando nudge cartesiano...", error=False)
    panel._step_cart_debug_log_event(
        "move_request",
        delta_m=[float(dx_m), float(dy_m), float(dz_m)],
        before=before,
        target=target,
    )

    def worker() -> None:
        panel._step_cart_debug_move_inflight = True
        ok = False
        info = ""
        after = None
        axis_err = None
        try:
            axis_tol_m = max(0.0010, panel._step_cart_debug_step_m() * 0.30)
            max_attempts = 3
            for attempt in range(1, max_attempts + 1):
                ok, info, after, axis_err = panel._step_cartesian_move_runtime_target(
                    target,
                    timeout_sec=4.0,
                    axis_tol_m=axis_tol_m,
                )
                panel._step_cart_debug_log_event(
                    "move_attempt",
                    attempt=int(attempt),
                    max_attempts=int(max_attempts),
                    target=target,
                    after=after,
                    axis_error_to_target=axis_err,
                    success=bool(ok),
                    result=str(info),
                    axis_tol_m=float(axis_tol_m),
                )
                if ok:
                    break
        except Exception as exc:
            ok = False
            info = f"exception:{exc}"
        finally:
            panel._step_cart_debug_move_inflight = False

        delta_after = None
        if before is not None and after is not None:
            delta_after = (
                float(after[0]) - float(before[0]),
                float(after[1]) - float(before[1]),
                float(after[2]) - float(before[2]),
            )

        def _ui_done() -> None:
            if ok:
                panel._step_cart_debug_set_status("Nudge ejecutado", error=False)
            else:
                panel._step_cart_debug_set_status(f"Error: {info}", error=True)
            panel._step_cart_debug_refresh()

        panel.signal_run_ui.emit(_ui_done)
        panel._step_cart_debug_log_event(
            "move_result",
            success=bool(ok),
            result=str(info),
            delta_m=[float(dx_m), float(dy_m), float(dz_m)],
            before=before,
            target=target,
            after=after,
            delta_after=delta_after,
            axis_error_to_target=axis_err,
        )

    panel._run_async(worker, name="step_cartesian_debug_move")

def _step_cart_debug_handle_axis(panel, axis: str, sign: int) -> None:
    step = panel._step_cart_debug_step_m()
    axis_n = str(axis or "").strip().upper()
    s = 1.0 if int(sign) >= 0 else -1.0
    dx = s * step if axis_n == "X" else 0.0
    dy = s * step if axis_n == "Y" else 0.0
    dz = s * step if axis_n == "Z" else 0.0
    panel._step_cart_debug_move_delta(dx, dy, dz)

def _step_cart_debug_run_validation_xyz(panel) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        panel._step_cart_debug_set_status("Validación disponible solo en STEP_BY_STEP", error=True)
        return
    if panel._step_cart_debug_move_inflight:
        panel._step_cart_debug_set_status("Movimiento en curso...", error=False)
        return

    def worker() -> None:
        panel._step_cart_debug_move_inflight = True
        try:
            step = 0.005
            axis_tol_m = max(0.0010, step * 0.30)
            tests = [
                ("X+", (step, 0.0, 0.0)),
                ("Y+", (0.0, step, 0.0)),
                ("Z+", (0.0, 0.0, step)),
            ]
            panel._step_cart_debug_log_event(
                "validation_xyz_start",
                step_m=float(step),
                axis_tol_m=float(axis_tol_m),
            )

            for name, delta in tests:
                before = panel._step_fetch_live_pose("rg2_pinch_center")
                if before is None:
                    panel._step_cart_debug_log_event(
                        "validation_xyz_case",
                        case=name,
                        status="failed",
                        reason="before_pose_unavailable",
                    )
                    continue
                target = (
                    float(before[0]) + float(delta[0]),
                    float(before[1]) + float(delta[1]),
                    float(before[2]) + float(delta[2]),
                )
                ok, info, after, err_xyz = panel._step_cartesian_move_runtime_target(
                    target,
                    timeout_sec=4.0,
                    axis_tol_m=axis_tol_m,
                )
                delta_after = None
                if after is not None:
                    delta_after = (
                        float(after[0]) - float(before[0]),
                        float(after[1]) - float(before[1]),
                        float(after[2]) - float(before[2]),
                    )
                panel._step_cart_debug_log_event(
                    "validation_xyz_case",
                    case=name,
                    status="ok" if ok else "failed",
                    before=before,
                    target=target,
                    after=after,
                    delta_after=delta_after,
                    axis_error_to_target=err_xyz,
                    result=str(info),
                )

            panel._step_cart_debug_log_event("validation_xyz_end")

            def _ui_ok() -> None:
                panel._step_cart_debug_set_status("Validación XYZ registrada en historico", error=False)
                panel._step_cart_debug_refresh()

            panel.signal_run_ui.emit(_ui_ok)
        except Exception as exc:
            panel._step_cart_debug_log_event("validation_xyz_error", error=str(exc))

            def _ui_fail() -> None:
                panel._step_cart_debug_set_status(f"Error validación: {exc}", error=True)
                panel._step_cart_debug_refresh()

            panel.signal_run_ui.emit(_ui_fail)
        finally:
            panel._step_cart_debug_move_inflight = False

    panel._run_async(worker, name="step_cartesian_debug_validation")

def _ensure_step_cart_debug_window(panel) -> None:
    if panel._step_cart_debug_window is not None:
        return
    build_step_cart_debug_window(panel)

def _show_step_cart_debug_window(panel) -> None:
    panel._ensure_step_cart_debug_window()
    if panel._step_cart_debug_window is None:
        return
    if panel._step_window is not None:
        geo = panel._step_window.geometry()
        panel._step_cart_debug_window.move(geo.x() + geo.width() + 12, geo.y())
    panel._step_cart_debug_window.show()
    panel._step_cart_debug_refresh()

def _step_window_maybe_refresh(panel) -> None:
    """Refresca el header del step window cada 500ms, solo si hay gate activo.

    Garantiza que los labels live del TCP sean realmente actuales mientras el
    usuario espera para pulsar el botón Iniciar de la fase habilitada.
    Sin este timer, los labels solo se actualizaban en la transición de gate.
    """
    if panel._step_mode != "STEP_BY_STEP":
        return
    if not bool(getattr(panel, "_step_wait_active", False)):
        return
    if panel._step_window is None or not panel._step_window.isVisible():
        return
    panel._step_window_refresh()

def _step_present_flow_name(panel, flow: str) -> str:
    return step_present_flow_name(flow)

def _step_effective_flow(panel) -> str:
    for candidate in (
        panel._step_pending_flow,
        panel._step_history_flow,
        panel._step_pipeline_flow,
    ):
        flow_name = str(candidate or "").strip().upper()
        if flow_name:
            return flow_name
    return ""

def _step_reset_sequence_view(panel, *, clear_history: bool = True) -> None:
    panel._step_pending_phase = ""
    panel._step_pending_flow = ""
    panel._step_current_phase = ""
    panel._step_next_phase = ""
    panel._step_running_phase = ""
    panel._step_decision = ""
    panel._step_phase_position = None
    panel._step_object_position = None
    panel._step_pipeline_flow = ""
    if clear_history:
        panel._step_history_flow = ""
        panel._step_history_rows = []

def _step_prepare_pipeline_view(panel, flow: str) -> None:
    flow_name = str(flow or "").strip().upper()
    if not flow_name:
        return
    panel._step_history_flow = flow_name
    panel._step_history_rows = []
    panel._step_pipeline_flow = flow_name
    panel._step_pending_flow = flow_name
    panel._step_pending_phase = ""
    panel._step_current_phase = ""
    sequence = panel._step_phase_sequence(flow_name)
    panel._step_next_phase = sequence[0] if sequence else "--"
    panel._step_running_phase = ""
    panel._step_decision = ""
    panel._step_phase_position = None
    panel._step_object_position = None
    panel._ensure_step_window()
    panel._step_window_refresh()
    if panel._step_window is not None:
        panel._step_window.show()

def _step_phase_sequence(panel, flow: str) -> List[str]:
    return step_phase_sequence(flow)

def _step_predict_next_phase(panel, flow: str, phase: str) -> str:
    return step_predict_next_phase(flow, phase)

def _step_phase_intent(panel, flow: str, phase: str) -> str:
    return step_phase_intent(flow, phase)

def _step_phase_action_text(panel, flow: str, phase: str, decision: str) -> str:
    return step_phase_action_text(flow, phase, decision)

def _step_phase_gripper_state(panel, flow: str, phase: str) -> str:
    return step_phase_gripper_state(flow, phase)

def _step_find_history_row(panel, phase: str) -> Optional[Dict[str, object]]:
    phase_name = str(phase or "").strip().upper()
    for row in reversed(panel._step_history_rows):
        if str(row.get("phase", "")).strip().upper() == phase_name:
            return row
    return None

def _step_phase_completed(panel, phase: str) -> bool:
    row = panel._step_find_history_row(phase)
    if row is None:
        return False
    row_kind = str(row.get("row_kind") or "").strip().upper()
    row_state = str(row.get("row_state") or "").strip().upper()
    if row_kind == "EVENT":
        return False
    if row_state in {"PHASE_DONE", "PHASE_BLOCKED", "PHASE_ABORTED"}:
        return True
    if row_state in {"PHASE_READY", "PHASE_RUNNING", "PHASE_WAITING_CONFIRMATION"}:
        return False
    if row.get("actual") is not None:
        return True
    if row.get("reached") is not None:
        return True
    reason = str(row.get("check_reason") or "").strip().lower()
    return reason not in {"", "pending"}

def _step_pipeline_phase_state(panel,
    flow: str,
    phase: str,
) -> Tuple[str, QColor, bool, str]:
    flow_name = str(flow or "").strip().upper()
    phase_name = str(phase or "").strip().upper()
    sequence = panel._step_phase_sequence(flow_name)
    current_phase = str(panel._step_current_phase or "").strip().upper()
    running_phase = str(panel._step_running_phase or "").strip().upper()
    waiting_active = bool(panel._step_wait_active)
    row = panel._step_find_history_row(phase_name)
    row_kind = str((row or {}).get("row_kind") or "").strip().upper()
    row_state = str((row or {}).get("row_state") or "").strip().upper()

    if row_kind == "EVENT" or row_state == "EVENT_SNAPSHOT":
        return (
            "Evento capturado",
            QColor(168, 85, 247),
            False,
            "Entrada de trazabilidad: snapshot registrado, no una fase ejecutada.",
        )

    if panel._step_phase_completed(phase_name):
        reached = None if row is None else row.get("reached")
        if reached is False:
            return (
                "Completada con desajuste",
                QColor(251, 191, 36),
                False,
                "La fase ya terminó, pero el cierre registró una desviación respecto al target.",
            )
        return (
            "Completada",
            QColor(34, 197, 94),
            False,
            "La fase ya se ejecutó.",
        )
    if running_phase == phase_name:
        return (
            "En ejecución",
            QColor(59, 130, 246),
            False,
            "Esta es la fase que está ejecutando ahora mismo el robot.",
        )
    if waiting_active and current_phase == phase_name:
        return (
            "Lista para iniciar",
            QColor(14, 165, 233),
            True,
            "Pulsa Iniciar para ejecutar exactamente esta fase.",
        )
    if phase_name == current_phase and not waiting_active:
        return (
            "Esperando punto de control",
            QColor(148, 163, 184),
            False,
            "La fase está cargada, pero todavía no ha llegado el momento de habilitarla.",
        )
    if not current_phase:
        first_phase = sequence[0] if sequence else ""
        if phase_name == first_phase:
            return (
                "Pendiente de arranque",
                QColor(148, 163, 184),
                False,
                "El flujo está cargado; la primera fase se habilitará cuando la secuencia llegue a su punto de control.",
            )
        return (
            "Bloqueada",
            QColor(203, 213, 225),
            False,
            "Esta fase se habilitará más adelante, cuando terminen las anteriores.",
        )
    if phase_name in sequence and current_phase in sequence:
        if sequence.index(phase_name) < sequence.index(current_phase):
            return (
                "Completada",
                QColor(34, 197, 94),
                False,
                "La fase ya quedó atrás en la secuencia.",
            )
    return (
        "Bloqueada",
        QColor(203, 213, 225),
        False,
        "Solo se desbloquea la siguiente fase cuando termina la fase activa.",
    )

def _direct_waiting_for_approach_confirmation(panel) -> bool:
    return (
        str(getattr(panel, "_direct_flow_state", "") or "").strip().upper()
        == "WAITING_FOR_APPROACH_CONFIRMATION"
    )

def _direct_enter_waiting_for_approach_confirmation(panel,
    *,
    request_id: str = "",
    tcp_base=None,
    object_base=None,
) -> None:
    panel._direct_flow_state = "WAITING_FOR_APPROACH_CONFIRMATION"
    panel._direct_flow_request_id = str(request_id or "").strip()
    panel._direct_wait_for_approach_event.clear()
    panel._ui_set_status(
        "Directo: robot en MESA; pulsa de nuevo para iniciar APPROACH_COARSE",
        error=False,
    )
    panel._emit_log(
        "[PICK][DIRECT][STATE] "
        "state=WAITING_FOR_APPROACH_CONFIRMATION "
        f"request_id={panel._direct_flow_request_id or 'none'} "
        f"tcp_base={panel._step_format_inline_xyz(tcp_base)} "
        f"object_base={panel._step_format_inline_xyz(object_base)}"
    )
    panel._refresh_controls()

def _direct_release_waiting_for_approach_confirmation(panel) -> bool:
    if not panel._direct_waiting_for_approach_confirmation():
        return False
    panel._direct_flow_state = "RUNNING_APPROACH"
    panel._emit_log(
        "[PICK][DIRECT][STATE] "
        "state=RUNNING_APPROACH "
        f"request_id={panel._direct_flow_request_id or 'none'}"
    )
    panel._ui_set_status("Directo: reanudando desde MESA hacia APPROACH_COARSE", error=False)
    panel._direct_wait_for_approach_event.set()
    panel._refresh_controls()
    return True

def _step_phase_gate_already_owned(panel, *, flow: str = "", phase: str = "") -> bool:
    flow_name = str(flow or "").strip().upper()
    phase_name = str(phase or "").strip().upper().split(".")[-1]
    if panel._step_mode != "STEP_BY_STEP":
        return False
    with panel._step_gate_lock:
        if bool(panel._step_wait_active):
            return False
        pipeline_flow = str(panel._step_pipeline_flow or panel._step_pending_flow or "").strip().upper()
        current_phase = str(panel._step_current_phase or "").strip().upper()
        running_phase = str(panel._step_running_phase or "").strip().upper()
        if flow_name and pipeline_flow and flow_name != pipeline_flow:
            return False
        if not phase_name or current_phase != phase_name or running_phase != phase_name:
            return False
        row = panel._step_find_history_row(phase_name)
        if row is None:
            return False
        row_kind = str(row.get("row_kind") or "").strip().upper()
        row_state = str(row.get("row_state") or "").strip().upper()
        return row_kind == "PHASE" and row_state == "PHASE_RUNNING"

def _direct_clear_waiting_for_approach_confirmation(panel, *, reason: str = "") -> None:
    previous = str(getattr(panel, "_direct_flow_state", "") or "").strip()
    request_id = str(getattr(panel, "_direct_flow_request_id", "") or "").strip()
    panel._direct_flow_state = ""
    panel._direct_flow_request_id = ""
    panel._direct_wait_for_approach_event.clear()
    if previous:
        panel._emit_log(
            "[PICK][DIRECT][STATE] "
            f"state=CLEARED previous={previous} "
            f"request_id={request_id or 'none'} "
            f"reason={reason or 'none'}"
        )
    panel._refresh_controls()

def _step_pipeline_rebuild(panel, flow: str) -> None:
    table = panel._step_pipeline_table
    if table is None:
        return
    flow_name = str(flow or "").strip().upper()
    sequence = panel._step_phase_sequence(flow_name)
    panel._step_pipeline_flow = flow_name
    panel._step_pipeline_row_index = {}
    panel._step_pipeline_buttons = {}
    table.clearContents()
    table.setRowCount(len(sequence))
    for row_idx, phase_name in enumerate(sequence):
        panel._step_pipeline_row_index[phase_name] = row_idx
        btn = QPushButton("Iniciar")
        btn.clicked.connect(
            lambda _checked=False, phase=phase_name: panel._on_step_phase_start_clicked(phase)
        )
        btn.setEnabled(False)
        table.setCellWidget(row_idx, 0, btn)
        panel._step_pipeline_buttons[phase_name] = btn
        for col_idx in range(1, 5):
            table.setItem(row_idx, col_idx, QTableWidgetItem(""))

def _step_refresh_pipeline_table(panel) -> None:
    table = panel._step_pipeline_table
    help_label = panel._step_pipeline_help_label
    if table is None or help_label is None:
        return
    flow_name = panel._step_effective_flow()
    if not flow_name:
        panel._step_pipeline_flow = ""
        panel._step_pipeline_row_index = {}
        panel._step_pipeline_buttons = {}
        table.clearContents()
        table.setRowCount(0)
        help_label.setText(
            "Todavía no hay un flujo cargado. Pulsa un botón de operación para cargar aquí "
            "todas las fases y ver cuál será la siguiente en habilitarse."
        )
        return

    sequence = panel._step_phase_sequence(flow_name)
    if (
        flow_name != panel._step_pipeline_flow
        or table.rowCount() != len(sequence)
        or set(panel._step_pipeline_row_index.keys()) != set(sequence)
    ):
        panel._step_pipeline_rebuild(flow_name)

    help_label.setText(
        "Solo se habilita un único botón Iniciar cada vez: el de la fase lista para ejecutar. "
        "Las demás filas permanecen bloqueadas hasta que termina la fase anterior."
    )
    for row_idx, phase_name in enumerate(sequence):
        button = panel._step_pipeline_buttons.get(phase_name)
        action_text = panel._step_phase_action_text(
            flow_name,
            phase_name,
            panel._step_decision if str(panel._step_current_phase or "").strip().upper() == phase_name else "",
        )
        gripper_text = panel._step_phase_gripper_state(flow_name, phase_name)
        state_text, state_color, button_enabled, tooltip = panel._step_pipeline_phase_state(
            flow_name,
            phase_name,
        )
        if button is not None:
            button.setEnabled(button_enabled)
            button.setToolTip(tooltip)
        phase_item = QTableWidgetItem(phase_name)
        phase_item.setTextAlignment(Qt.AlignCenter)
        action_item = QTableWidgetItem(action_text)
        action_item.setToolTip(action_text)
        gripper_item = QTableWidgetItem(gripper_text)
        gripper_item.setTextAlignment(Qt.AlignCenter)
        state_item = QTableWidgetItem(state_text)
        state_item.setTextAlignment(Qt.AlignCenter)
        state_item.setBackground(state_color)
        table.setItem(row_idx, 1, phase_item)
        table.setItem(row_idx, 2, action_item)
        table.setItem(row_idx, 3, gripper_item)
        table.setItem(row_idx, 4, state_item)

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

def _step_format_xyz(panel, position: Optional[Tuple[float, float, float]]) -> Tuple[str, str, str]:
    if position is None:
        return ("--", "--", "--")
    return (f"{float(position[0]):.3f}", f"{float(position[1]):.3f}", f"{float(position[2]):.3f}")

def _step_display_position(panel, position: Optional[Tuple[float, float, float]]) -> Optional[Tuple[float, float, float]]:
    if position is None:
        return None
    try:
        pos3 = (float(position[0]), float(position[1]), float(position[2]))
    except Exception:
        return None
    try:
        wx, wy, wz = base_to_world(pos3[0], pos3[1], pos3[2])
        return (float(wx), float(wy), float(wz))
    except Exception:
        return pos3

def _step_format_inline_xyz(panel, position: Optional[Tuple[float, float, float]]) -> str:
    x, y, z = panel._step_format_xyz(position)
    if x == "--":
        return "--"
    return f"({x}, {y}, {z})"

def _step_format_inline_rpy(panel, rpy_deg: Optional[Tuple[float, float, float]]) -> str:
    if rpy_deg is None:
        return "--"
    try:
        return f"({float(rpy_deg[0]):.1f}, {float(rpy_deg[1]):.1f}, {float(rpy_deg[2]):.1f})"
    except Exception:
        return "--"

def _step_fetch_live_pose(panel, ee_frame: str) -> Optional[Tuple[float, float, float]]:
    """Consulta TF en tiempo real. Devuelve None si TF no disponible; nunca usa caché stale."""
    frame_name = str(ee_frame or "").strip()
    if not frame_name:
        return None
    base_frame = panel._business_base_frame()
    tcp_pose_base, _tcp_rpy_deg, _tcp_reason = tf_get_tcp_in_base(
        base_frame=base_frame,
        ee_frame=frame_name,
        timeout=0.20,
        logger=None,
    )
    if tcp_pose_base is None:
        return None
    return (
        float(tcp_pose_base.pose.position.x),
        float(tcp_pose_base.pose.position.y),
        float(tcp_pose_base.pose.position.z),
    )

def _step_operational_frame_name(panel) -> str:
    return str(
        getattr(panel, "_step_target_frame", "")
        or getattr(panel, "_ee_frame_effective", "")
        or panel._required_ee_frame
        or "rg2_pinch_center"
    ).strip() or "rg2_pinch_center"

def _step_live_pose_text(panel, label: str, ee_frame: str, position: Optional[Tuple[float, float, float]]) -> str:
    frame_name = str(ee_frame or "").strip() or "--"
    xyz_txt = panel._step_format_inline_xyz(position)
    return f"{label}: {xyz_txt} | frame: {frame_name}"

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

def _step_capture_start_pose(panel, trigger: str) -> None:
    base_frame = panel._business_base_frame()
    ee_frame = panel._step_operational_frame_name()
    start_xyz = None
    start_rpy = None
    tcp_pose_base, tcp_rpy_deg, _tcp_reason = tf_get_tcp_in_base(
        base_frame=base_frame,
        ee_frame=ee_frame,
        timeout=0.12,
        logger=None,
    )
    if tcp_pose_base is not None:
        start_xyz = (
            float(tcp_pose_base.pose.position.x),
            float(tcp_pose_base.pose.position.y),
            float(tcp_pose_base.pose.position.z),
        )
        if tcp_rpy_deg is not None and len(tcp_rpy_deg) >= 3:
            start_rpy = (
                float(tcp_rpy_deg[0]),
                float(tcp_rpy_deg[1]),
                float(tcp_rpy_deg[2]),
            )
    if start_xyz is None:
        start_xyz = panel._last_trace_tcp_base or panel._last_tcp_base
    if start_rpy is None:
        start_rpy = panel._last_trace_tcp_rpy_deg or panel._last_tcp_rpy_deg
    panel._step_start_pose_base = start_xyz
    panel._step_start_pose_rpy_deg = start_rpy
    panel._step_start_trigger = str(trigger or "").strip()

def _step_record_history(panel, flow: str, phase: str, position=None) -> None:
    phase_name = str(phase or "").strip().upper()
    flow_name = str(flow or "").strip().upper()
    pos3 = None
    if isinstance(position, (list, tuple)) and len(position) >= 3:
        try:
            pos3 = (float(position[0]), float(position[1]), float(position[2]))
        except Exception:
            pos3 = None
    sequence = panel._step_phase_sequence(flow_name)
    first_phase = sequence[0] if sequence else ""
    # No resetear si la fila INICIO ya fue pre-insertada desde el hilo principal.
    # El guard se activa si existe una fila con la fase inicial (independientemente
    # de si actual está fijado o no — la fila es válida en cualquier caso).
    _inicio_pre_frozen = bool(
        first_phase
        and panel._step_history_rows
        and str(panel._step_history_rows[0].get("phase", "")).strip().upper() == first_phase
    )
    if flow_name != panel._step_history_flow:
        panel._step_history_flow = flow_name
        panel._step_history_rows = []
    elif first_phase and phase_name == first_phase and not _inicio_pre_frozen:
        panel._step_history_rows = []
    if panel._step_history_rows:
        last_row = panel._step_history_rows[-1]
        last_phase = str(last_row.get("phase") or "").strip().upper()
        if last_phase != phase_name and last_row.get("actual") is None:
            _op_frame = panel._step_operational_frame_name()
            actual_pose = panel._step_fetch_live_pose(_op_frame)
            _pose_src = "tf_live" if actual_pose is not None else "unavailable"
            _xyz_log = (
                f"({actual_pose[0]:.3f},{actual_pose[1]:.3f},{actual_pose[2]:.3f})"
                if actual_pose is not None else "none"
            )
            panel._emit_log(
                f"[STEP][TF_LIVE] available={str(actual_pose is not None).lower()} "
                f"frame={_op_frame} xyz={_xyz_log}"
            )
            panel._emit_log(
                f"[STEP][POSE_SOURCE] phase={last_phase} source={_pose_src} "
                f"frame={_op_frame} xyz={_xyz_log} "
                f"same_frame_as_backend=true same_source_as_backend=true"
            )
            # Sin fallback a caché stale: si TF no devuelve pose, actual queda None
            # y la tabla muestra PEND en vez de datos inventados.
            last_row["actual"] = actual_pose
            panel._step_update_row_object_metrics(last_row, actual_pose)
            last_row["reached"] = panel._step_assess_target_reached(last_row.get("target"), actual_pose)
            if str(last_row.get("row_kind") or "").strip().upper() == "PHASE":
                last_row["row_state"] = (
                    "PHASE_DONE"
                    if last_row.get("reached") is True
                    else "PHASE_BLOCKED"
                )
        elif last_phase != phase_name and last_row.get("actual") is not None:
            panel._emit_log(
                "[STEP][ROW_FROZEN] "
                f"phase={last_phase} "
                f"actual={panel._step_format_inline_xyz(last_row.get('actual'))} "
                f"target={panel._step_format_inline_xyz(last_row.get('target'))}"
            )
    _op_frame_snap = panel._step_operational_frame_name()
    if panel._step_history_rows and str(panel._step_history_rows[-1].get("phase") or "").strip().upper() == phase_name:
        # Mismo fase: actualizar target.
        # origin_snapshot se actualiza si actual=None (gate aún no cerrado): esto ocurre
        # cuando _step_pre_insert_inicio_row lo capturó antes de que el robot llegara
        # a MESA. El worker llama de nuevo con la pose real post-MESA para corregirlo.
        _existing_snap = panel._step_history_rows[-1].get("origin_snapshot")
        _row_actual = panel._step_history_rows[-1].get("actual")
        if _row_actual is None:
            # Gate no cerrado aún: actualizar origin_snapshot con TF live actual.
            _current_snap = panel._step_fetch_live_pose(_op_frame_snap)
            if _current_snap is not None:
                if _existing_snap is not None:
                    _mut_delta = max(abs(_current_snap[i] - _existing_snap[i]) for i in range(3))
                    panel._emit_log(
                        f"[STEP][ORIGIN_UPDATE] phase={phase_name} "
                        f"old={panel._step_format_inline_xyz(_existing_snap)} "
                        f"new={panel._step_format_inline_xyz(_current_snap)} "
                        f"delta_m={_mut_delta:.4f}"
                    )
                panel._step_history_rows[-1]["origin_snapshot"] = _current_snap
        elif _existing_snap is not None:
            _current_snap = panel._step_fetch_live_pose(_op_frame_snap)
            if _current_snap is not None:
                _mut_delta = max(abs(_current_snap[i] - _existing_snap[i]) for i in range(3))
                if _mut_delta > 0.02:
                    panel._emit_log(
                        f"[STEP][ORIGIN_MUTATION_ERROR] phase={phase_name} "
                        f"old={panel._step_format_inline_xyz(_existing_snap)} "
                        f"new={panel._step_format_inline_xyz(_current_snap)} "
                        f"delta_m={_mut_delta:.4f}"
                    )
        panel._step_history_rows[-1]["target"] = pos3
    else:
        # Nueva fase: capturar pose actual como origin_snapshot (congelado).
        # Retry hasta 3 veces con 80ms de espera si TF devuelve None.
        # Siempre se usa TF live para reflejar la posición real del robot
        # cuando abre el gate, independientemente de la fase anterior.
        # Override: si existe _step_origin_override_for_next_gate (capturado antes
        # de un HOME silencioso), usarlo como ORG para reflejar la pose real pre-HOME.
        _origin_snap = None
        _snap_source = "unavailable_after_3_retries"
        _override_snap = getattr(panel, "_step_origin_override_for_next_gate", None)
        if _override_snap is not None:
            _origin_snap = _override_snap
            _snap_source = "origin_override_pre_home"
            try:
                delattr(panel, "_step_origin_override_for_next_gate")
            except AttributeError:
                pass
            panel._emit_log(
                f"[STEP][ORG_OVERRIDE] phase={phase_name} "
                f"xyz={panel._step_format_inline_xyz(_origin_snap)} "
                f"source={_snap_source}"
            )
        else:
            for _snap_attempt in range(3):
                _origin_snap = panel._step_fetch_live_pose(_op_frame_snap)
                if _origin_snap is not None:
                    _snap_source = f"tf_live_attempt_{_snap_attempt + 1}"
                    break
                time.sleep(0.08)
        # Fallback si TF falló: usar el actual de la fila anterior.
        if _origin_snap is None and panel._step_history_rows:
            _prev_actual = panel._step_history_rows[-1].get("actual")
            if _prev_actual is not None:
                _origin_snap = _prev_actual
                _snap_source = "prev_row_actual_fallback"
        panel._emit_log(
            f"[STEP][ORG_CAPTURE] phase={phase_name} "
            f"frame={_op_frame_snap} "
            f"xyz={panel._step_format_inline_xyz(_origin_snap)} "
            f"source={_snap_source}"
        )
        panel._emit_log(
            f"[STEP][ORG_CAPTURE_FRAME] phase={phase_name} "
            f"op_frame={_op_frame_snap} "
            f"capture_time_mono={time.monotonic():.3f}"
        )
        panel._step_history_rows.append(
            {
                "phase": phase_name,
                "target": pos3,
                "actual": None,            # TCP TF live al cerrar la fase (columna TCP-TF)
                "origin_snapshot": _origin_snap,  # pose robot al abrir la gate (congelada)
                "exec_target_snapshot": None,     # fijado por _step_set_exec_target
                "reached": None,
                "row_kind": "PHASE",
                "row_state": "PHASE_READY",
                "object_world_snapshot": None,    # pose objeto world al cerrar la fase
                "dist_tcp_obj_snapshot": None,    # distancia TCP↔objeto al cerrar la fase
                "check_reason": None,             # texto corto del motivo de check
            }
        )
        panel._emit_log(f"[STEP][PREPARED_PHASE] phase={phase_name} started=false")
        panel._emit_log(
            f"[STEP][ORIGIN_SNAPSHOT] phase={phase_name} "
            f"pose={panel._step_format_inline_xyz(_origin_snap)} frame={_op_frame_snap}"
        )
        panel._emit_log(
            f"[STEP][TARGET_SNAPSHOT] phase={phase_name} "
            f"pose={panel._step_format_inline_xyz(pos3)}"
        )
        # actual queda None hasta que la fase SIGUIENTE empiece: en ese momento
        # la lógica de transición (líneas ~5224-5246) captura la pose real del
        # robot, que YA habrá llegado al destino de esta fase. Esto garantiza que
        # la columna "actual" muestre dónde llegó el robot (pose de llegada),
        # no dónde estaba antes de empezar a moverse (pose de salida).

def _step_pre_insert_inicio_row(panel,
    pos_actual,
    target_pos,
    obj_pos=None,
    flow_name: str = "DIRECT",
) -> None:
    """Pre-inserta la fila INICIO en la tabla STEP_BY_STEP desde el hilo principal,
    ANTES del diálogo de confirmación, para que sea visible antes de pulsar Iniciar.

    - No bloquea (no espera evento de continue).
    - 'actual' se deja None: _step_window_refresh muestra la pose TF2 en vivo mientras
      el usuario espera.  Cuando el worker pase a APPROACH_COARSE, la lógica de transición
      en _step_record_history capturará la pose real (robot aún en HOME) y marcará
      reached=True (INICIO es fase de verificación, no de movimiento).
    - 'target' = pos3_target (destino APPROACH_COARSE): la columna "X/Y/Z Obj" del
      panel muestra adónde irá el robot cuando el usuario pulse Sigue.
    - Deshabilita el botón "Sigue" hasta que el worker esté listo y llame a
      _step_wait_for_phase, que lo habilitará via _step_window_set_waiting.
    - Solo actúa en modo STEP_BY_STEP; no hace nada en AUTO.
    - obj_pos: pose del objeto en frame base_link (para el label "Objeto XYZ").
    """
    if panel._step_mode != "STEP_BY_STEP":
        return
    first_phase = "INICIO"

    # Reset del flow para este nuevo ciclo
    panel._step_history_flow = flow_name
    panel._step_history_rows = []

    pos3_actual = None
    if isinstance(pos_actual, (list, tuple)) and len(pos_actual) >= 3:
        try:
            pos3_actual = (float(pos_actual[0]), float(pos_actual[1]), float(pos_actual[2]))
        except Exception:
            pos3_actual = None

    pos3_target = None
    if isinstance(target_pos, (list, tuple)) and len(target_pos) >= 3:
        try:
            pos3_target = (float(target_pos[0]), float(target_pos[1]), float(target_pos[2]))
        except Exception:
            pos3_target = None

    # Insertar fila INICIO:
    #   origin_snapshot = pos3_actual → pose congelada del robot al abrir la gate (MESA)
    #   actual = None  → PEND hasta que el worker capture el Cierre tras el gate
    #   target = pos3_target → pose MESA (= pos3_actual, donde ya está el robot)
    #   reached = None → PEND hasta captura
    panel._step_history_rows.append({
        "phase": first_phase,
        "target": pos3_target,           # ← pose MESA (= pos3_actual)
        "actual": None,                  # ← capturado en worker tras pulsar Sigue
        "origin_snapshot": pos3_actual,  # ← congelado: pose robot al abrir la gate
        "exec_target_snapshot": pos3_target,
        "reached": None,                 # ← PEND hasta captura
        "object_world_snapshot": None,
        "dist_tcp_obj_snapshot": None,
        "check_reason": None,
    })
    panel._emit_log(f"[STEP][PREPARED_PHASE] phase={first_phase} started=false")
    panel._emit_log(
        f"[STEP][ORIGIN_SNAPSHOT] phase={first_phase} "
        f"pose={panel._step_format_inline_xyz(pos3_actual)} frame={panel._step_operational_frame_name()}"
    )
    panel._emit_log(
        f"[STEP][TARGET_SNAPSHOT] phase={first_phase} "
        f"pose={panel._step_format_inline_xyz(pos3_target)}"
    )

    # Pose del objeto físico en frame base_link (label "Objeto XYZ")
    pos3_obj = None
    if isinstance(obj_pos, (list, tuple)) and len(obj_pos) >= 3:
        try:
            pos3_obj = (float(obj_pos[0]), float(obj_pos[1]), float(obj_pos[2]))
        except Exception:
            pos3_obj = None
    panel._step_object_position = pos3_obj

    # Actualizar estado visible de la ventana
    panel._step_pending_flow = flow_name.lower()
    panel._step_pending_phase = f"{flow_name}.{first_phase}"
    panel._step_current_phase = first_phase
    panel._step_next_phase = panel._step_predict_next_phase(flow_name, first_phase)
    panel._step_decision = "INICIO - Ir a MESA y abrir la pinza antes de continuar"
    panel._step_phase_position = pos3_target

    # Abrir ventana y mostrar la secuencia; el botón de la fase aún queda bloqueado
    # hasta que el worker alcance el punto de control correspondiente.
    panel._ensure_step_window()
    panel._step_window_refresh()
    if panel._step_window is not None:
        panel._step_window.show()

    panel._emit_log(
        "[STEP][PRE_INSERT_INICIO] "
        f"actual={panel._step_format_inline_xyz(pos3_actual)} "
        f"target={panel._step_format_inline_xyz(pos3_target)} "
        "btn_iniciar=disabled pending_worker=true"
    )

def _step_record_direct_initial_snapshot(panel,
    *,
    request_id: str,
    tcp_base=None,
    object_base=None,
    dx=None,
    dy=None,
    dz=None,
    dist3d=None,
    joints=None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
) -> None:
    panel._step_record_direct_event_snapshot(
        phase_name="INITIAL_SNAPSHOT",
        request_id=request_id,
        tcp_base=tcp_base,
        object_base=object_base,
        dx=dx,
        dy=dy,
        dz=dz,
        dist3d=dist3d,
        joints=joints,
        pose_source=pose_source,
        flow_name=flow_name,
        prepend=True,
        decision_text="INITIAL_SNAPSHOT - Snapshot inicial previo al movimiento hacia MESA",
    )

def _step_upsert_history_row_ordered(panel,
    *,
    flow_name: str,
    row: Dict[str, object],
    prepend: bool = False,
) -> None:
    flow_name = str(flow_name or "DIRECT").strip().upper() or "DIRECT"
    phase_name = str(row.get("phase") or "").strip().upper()
    if flow_name != panel._step_history_flow:
        panel._step_history_flow = flow_name
        panel._step_history_rows = []

    for idx, existing_row in enumerate(panel._step_history_rows):
        if str(existing_row.get("phase") or "").strip().upper() == phase_name:
            panel._step_history_rows[idx] = row
            return

    sequence = panel._step_phase_sequence(flow_name)
    phase_index = sequence.index(phase_name) if phase_name in sequence else None
    if phase_index is None:
        if prepend:
            panel._step_history_rows.insert(0, row)
        else:
            panel._step_history_rows.append(row)
        return

    insert_at = len(panel._step_history_rows)
    for idx, existing_row in enumerate(panel._step_history_rows):
        existing_phase = str(existing_row.get("phase") or "").strip().upper()
        if existing_phase not in sequence:
            continue
        if sequence.index(existing_phase) > phase_index:
            insert_at = idx
            break
    panel._step_history_rows.insert(insert_at, row)

def _step_record_direct_home_initial(panel,
    *,
    request_id: str,
    tcp_base=None,
    object_base=None,
    dx=None,
    dy=None,
    dz=None,
    dist3d=None,
    joints=None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        return

    flow_name = str(flow_name or "DIRECT").strip().upper() or "DIRECT"
    phase_name = "HOME_INITIAL"

    tcp_pos3 = None
    if isinstance(tcp_base, (list, tuple)) and len(tcp_base) >= 3:
        try:
            tcp_pos3 = (float(tcp_base[0]), float(tcp_base[1]), float(tcp_base[2]))
        except Exception:
            tcp_pos3 = None

    obj_pos3 = None
    if isinstance(object_base, (list, tuple)) and len(object_base) >= 3:
        try:
            obj_pos3 = (float(object_base[0]), float(object_base[1]), float(object_base[2]))
        except Exception:
            obj_pos3 = None

    dist_val = None
    if dist3d is not None:
        try:
            dist_val = float(dist3d)
        except Exception:
            dist_val = None

    def _fmt_scalar_local(value) -> str:
        if value is None:
            return "--"
        try:
            return f"{float(value):.3f}"
        except Exception:
            return "--"

    joint_values = []
    if isinstance(joints, (list, tuple)):
        for value in joints:
            try:
                joint_values.append(float(value))
            except Exception:
                joint_values.append(None)
    joints_txt = json.dumps(joint_values, ensure_ascii=True)
    reason = (
        f"request_id={str(request_id or '').strip() or 'none'} "
        f"source={str(pose_source or '').strip() or 'none'} "
        f"dx={_fmt_scalar_local(dx)} dy={_fmt_scalar_local(dy)} "
        f"dz={_fmt_scalar_local(dz)} dist3d={_fmt_scalar_local(dist_val)} "
        f"joints={joints_txt}"
    )
    row = {
        "phase": phase_name,
        "target": tcp_pos3,
        "actual": tcp_pos3,
        "origin_snapshot": tcp_pos3,
        "exec_target_snapshot": tcp_pos3,
        "reached": True,
        "row_kind": "PHASE",
        "row_state": "PHASE_DONE",
        "object_world_snapshot": panel._step_display_position(obj_pos3),
        "dist_tcp_obj_snapshot": dist_val,
        "check_reason": reason,
        "request_id": str(request_id or "").strip(),
        "pose_source": str(pose_source or "").strip(),
        "joint_snapshot": joint_values,
        "object_base_snapshot": obj_pos3,
        "delta_snapshot": {
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "dist3d": dist_val,
        },
    }

    panel._step_upsert_history_row_ordered(
        flow_name=flow_name,
        row=row,
        prepend=False,
    )

    preserve_active_gate = bool(panel._step_wait_active) or bool(
        str(panel._step_running_phase or "").strip()
    )
    panel._step_pipeline_flow = flow_name
    if not preserve_active_gate:
        panel._step_pending_flow = flow_name
        panel._step_pending_phase = f"{flow_name}.{phase_name}"
        panel._step_current_phase = phase_name
        panel._step_next_phase = panel._step_predict_next_phase(flow_name, phase_name)
        panel._step_running_phase = ""
        panel._step_decision = (
            "HOME_INITIAL - Robot movido a MESA y detenido; esperando confirmación para APPROACH_COARSE"
        )
        panel._step_phase_position = tcp_pos3
        panel._step_object_position = obj_pos3

    panel._emit_log(
        f"[STEP][{phase_name}] "
        f"request_id={str(request_id or '').strip() or 'none'} "
        f"tcp={panel._step_format_inline_xyz(tcp_pos3)} "
        f"object={panel._step_format_inline_xyz(obj_pos3)} "
        f"source={str(pose_source or '').strip() or 'none'} "
        f"dist3d={_fmt_scalar_local(dist_val)} "
        f"row_state=PHASE_DONE preserve_active_gate={str(preserve_active_gate).lower()}"
    )
    panel._step_window_refresh()
    if panel._step_window is not None:
        panel._step_window.show()

def _step_record_direct_mesa_ready(panel,
    *,
    request_id: str,
    tcp_base=None,
    object_base=None,
    dx=None,
    dy=None,
    dz=None,
    dist3d=None,
    joints=None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
) -> None:
    panel._step_record_direct_home_initial(
        request_id=request_id,
        tcp_base=tcp_base,
        object_base=object_base,
        dx=dx,
        dy=dy,
        dz=dz,
        dist3d=dist3d,
        joints=joints,
        pose_source=pose_source,
        flow_name=flow_name,
    )

def _step_record_direct_event_snapshot(panel,
    *,
    phase_name: str,
    request_id: str,
    tcp_base=None,
    object_base=None,
    dx=None,
    dy=None,
    dz=None,
    dist3d=None,
    joints=None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
    prepend: bool = False,
    decision_text: str = "",
) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        return

    phase_name = str(phase_name or "").strip().upper() or "DIRECT_EVENT"
    flow_name = str(flow_name or "DIRECT").strip().upper() or "DIRECT"

    tcp_pos3 = None
    if isinstance(tcp_base, (list, tuple)) and len(tcp_base) >= 3:
        try:
            tcp_pos3 = (float(tcp_base[0]), float(tcp_base[1]), float(tcp_base[2]))
        except Exception:
            tcp_pos3 = None

    obj_pos3 = None
    if isinstance(object_base, (list, tuple)) and len(object_base) >= 3:
        try:
            obj_pos3 = (float(object_base[0]), float(object_base[1]), float(object_base[2]))
        except Exception:
            obj_pos3 = None

    dist_val = None
    if dist3d is not None:
        try:
            dist_val = float(dist3d)
        except Exception:
            dist_val = None

    def _fmt_scalar_local(value) -> str:
        if value is None:
            return "--"
        try:
            return f"{float(value):.3f}"
        except Exception:
            return "--"

    joint_values = []
    if isinstance(joints, (list, tuple)):
        for value in joints:
            try:
                joint_values.append(float(value))
            except Exception:
                joint_values.append(None)
    joints_txt = json.dumps(joint_values, ensure_ascii=True)
    reason = (
        f"request_id={str(request_id or '').strip() or 'none'} "
        f"source={str(pose_source or '').strip() or 'none'} "
        f"dx={_fmt_scalar_local(dx)} dy={_fmt_scalar_local(dy)} "
        f"dz={_fmt_scalar_local(dz)} dist3d={_fmt_scalar_local(dist_val)} "
        f"joints={joints_txt}"
    )
    row = {
        "phase": phase_name,
        "target": tcp_pos3,
        "actual": tcp_pos3,
        "origin_snapshot": tcp_pos3,
        "exec_target_snapshot": tcp_pos3,
        "reached": None,
        "row_kind": "EVENT",
        "row_state": "EVENT_SNAPSHOT",
        "object_world_snapshot": panel._step_display_position(obj_pos3),
        "dist_tcp_obj_snapshot": dist_val,
        "check_reason": reason,
        "request_id": str(request_id or "").strip(),
        "pose_source": str(pose_source or "").strip(),
        "joint_snapshot": joint_values,
        "object_base_snapshot": obj_pos3,
        "delta_snapshot": {
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "dist3d": dist_val,
        },
    }

    panel._step_upsert_history_row_ordered(
        flow_name=flow_name,
        row=row,
        prepend=prepend,
    )

    preserve_active_gate = bool(panel._step_wait_active) or bool(
        str(panel._step_running_phase or "").strip()
    )
    panel._step_pipeline_flow = flow_name
    if not preserve_active_gate:
        panel._step_pending_flow = flow_name
        panel._step_pending_phase = f"{flow_name}.{phase_name}"
        panel._step_current_phase = phase_name
        panel._step_next_phase = panel._step_predict_next_phase(flow_name, phase_name)
        panel._step_running_phase = ""
        panel._step_decision = str(decision_text or "").strip()
        panel._step_phase_position = tcp_pos3
        panel._step_object_position = obj_pos3

    panel._emit_log(
        f"[STEP][{phase_name}] "
        f"request_id={str(request_id or '').strip() or 'none'} "
        f"tcp={panel._step_format_inline_xyz(tcp_pos3)} "
        f"object={panel._step_format_inline_xyz(obj_pos3)} "
        f"source={str(pose_source or '').strip() or 'none'} "
        f"dist3d={_fmt_scalar_local(dist_val)} "
        f"preserve_active_gate={str(preserve_active_gate).lower()}"
    )
    panel._step_window_refresh()
    if panel._step_window is not None:
        panel._step_window.show()

def _step_runtime_refresh(panel) -> None:
    if not getattr(panel, "_step_runtime_block_labels", None):
        return
    try:
        snapshot = build_runtime_audit_snapshot(panel)
    except Exception as exc:
        panel._log_warning(f"[STEP][RUNTIME] {exc}")
        for labels in panel._step_runtime_block_labels.values():
            labels["status"].setText("STALE")
            labels["status"].setStyleSheet(runtime_status_style("STALE"))
            labels["summary"].setText("sin dato")
            labels["planned"].setText("sin dato")
            labels["runtime"].setText("sin dato")
        return
    for block_key, block in snapshot.blocks.items():
        labels = panel._step_runtime_block_labels.get(block_key)
        if not labels:
            continue
        labels["status"].setText(str(block.status or "STALE"))
        labels["status"].setStyleSheet(runtime_status_style(block.status))
        labels["summary"].setText(str(block.summary or "sin dato"))
        labels["planned"].setText("\n".join(block.planned_lines) or "sin dato")
        labels["runtime"].setText("\n".join(block.runtime_lines) or "sin dato")

def _step_window_refresh(panel) -> None:
    panel._ensure_step_window()
    current_flow = panel._step_effective_flow()
    current_phase = str(panel._step_current_phase or "").strip()
    running_phase = str(panel._step_running_phase or "").strip()
    next_phase = str(panel._step_next_phase or "").strip() or "--"
    if panel._step_mode_label is not None:
        panel._step_mode_label.setText(
            f"Flujo cargado: {panel._step_present_flow_name(current_flow)}"
        )
    if panel._step_phase_label is not None:
        if panel._step_wait_active and current_phase:
            phase_text = current_phase
        elif current_flow:
            phase_text = "esperando siguiente punto de control"
        else:
            phase_text = "--"
        panel._step_phase_label.setText(f"Fase lista para iniciar: {phase_text}")
    if panel._step_current_label is not None:
        current = running_phase or "--"
        current_gripper = panel._step_phase_gripper_state(
            current_flow,
            current,
        )
        panel._step_current_label.setText(
            f"Fase en ejecución: {current} | pinza esperada: {current_gripper}"
        )
    if panel._step_next_label is not None:
        next_gripper = panel._step_phase_gripper_state(
            current_flow,
            next_phase,
        )
        panel._step_next_label.setText(
            f"Próxima fase bloqueada: {next_phase} | pinza esperada: {next_gripper}"
        )
    if panel._step_intent_label is not None:
        intent = panel._step_phase_intent(current_flow, current_phase)
        panel._step_intent_label.setText(f"Objetivo de la fase: {intent}")
    if panel._step_decision_label is not None:
        action_text = panel._step_phase_action_text(
            current_flow,
            current_phase,
            panel._step_decision,
        ) if current_phase else "--"
        panel._step_decision_label.setText(
            f"Acción exacta al pulsar Iniciar: {action_text}"
        )
    if panel._step_target_label is not None:
        world_frame = panel._world_frame_last_first()
        panel._step_target_label.setText(
            panel._step_live_pose_text(
                "XYZ objetivo de la fase (world)",
                world_frame,
                panel._step_display_position(panel._step_phase_position),
            )
        )
    operational_frame = panel._step_operational_frame_name()
    world_frame = panel._world_frame_last_first()
    operational_live = panel._step_fetch_live_pose(operational_frame)
    operational_live_display = panel._step_display_position(operational_live)
    # Sin fallback a caché stale: si TF no disponible, el label muestra "--"
    if operational_live is not None:
        panel._emit_log(
            f"[STEP][LIVE_POSE_HEADER] "
            f"frame={operational_frame} "
            f"xyz={panel._step_format_inline_xyz(operational_live)}"
        )
    if panel._step_live_operational_label is not None:
        panel._step_live_operational_label.setText(
            panel._step_live_pose_text("XYZ actual del TCP (world)", world_frame, operational_live_display)
        )
    # FIX: visual_frame usaba "tool0" (base del gripper) en vez del TCP real.
    # rg2_pinch_center está desplazado respecto a tool0 por la geometría canónica
    # del URDF, así que mezclar ambos frames introduce un desfase sistemático.
    # Ahora ambos labels usan el mismo frame operacional.
    visual_frame = operational_frame
    visual_live = panel._step_fetch_live_pose(visual_frame)
    visual_live_display = panel._step_display_position(visual_live)
    # Log [PINZA_ALIGN]: diferencia entre frame visual y frame actual para validación
    if operational_live is not None and visual_live is not None:
        _dz = float(visual_live[2]) - float(operational_live[2]) if (
            len(visual_live) > 2 and len(operational_live) > 2
        ) else None
        panel._emit_log(
            f"[PINZA_ALIGN] "
            f"actual_frame={operational_frame} "
            f"visual_frame={visual_frame} "
            f"actual_xyz={panel._step_format_inline_xyz(operational_live)} "
            f"visual_xyz={panel._step_format_inline_xyz(visual_live)} "
            f"dz={f'{_dz:.4f}' if _dz is not None else '--'}"
        )
    # [MESH_ALIGN] Log de diagnóstico comparando todos los frames relevantes
    # en el MISMO ciclo de refresco para detectar desfases entre UI y geometría.
    try:
        _ma_tool0_base = panel._step_fetch_live_pose("tool0")
        _ma_pinch_base = operational_live  # ya calculado arriba (base_link)
        _ma_obj_world = panel._step_fetch_object_world()  # world directo de pose/info
        # Convertir todo a world para comparación unificada
        _ma_tool0_world = panel._step_display_position(_ma_tool0_base)
        _ma_pinch_world = operational_live_display  # ya es world
        # Org de la tabla STEP (primera fila de la fase activa, en base_link)
        _ma_step_org_world = None
        _ma_step_org_raw = None
        if panel._step_history_rows:
            _active_row = next(
                (r for r in reversed(panel._step_history_rows)
                 if str(r.get("phase", "")).strip().upper()
                 == str(panel._step_current_phase or "").strip().upper()),
                None,
            )
            if _active_row is not None:
                _ma_step_org_raw = _active_row.get("origin_snapshot")
                _ma_step_org_world = panel._step_display_position(_ma_step_org_raw)
        def _maz(a, b):
            """Z diff (a - b), both world tuples."""
            if a is None or b is None:
                return "--"
            try:
                return f"{float(a[2]) - float(b[2]):.4f}"
            except Exception:
                return "--"
        panel._emit_log(
            f"[MESH_ALIGN] "
            f"phase={panel._step_current_phase or '--'} "
            f"tool0_world={panel._step_format_inline_xyz(_ma_tool0_world)} "
            f"rg2_pinch_center_world={panel._step_format_inline_xyz(_ma_pinch_world)} "
            f"obj_root_world={panel._step_format_inline_xyz(_ma_obj_world)} "
            f"step_org_world={panel._step_format_inline_xyz(_ma_step_org_world)} "
            f"header_label_world={panel._step_format_inline_xyz(_ma_pinch_world)} "
            f"dz_tool0_pinch={_maz(_ma_tool0_world, _ma_pinch_world)} "
            f"dz_pinch_obj={_maz(_ma_pinch_world, _ma_obj_world)} "
            f"dz_tool0_obj={_maz(_ma_tool0_world, _ma_obj_world)} "
            f"dz_org_header={_maz(_ma_step_org_world, _ma_pinch_world)} "
            f"dz_org_obj={_maz(_ma_step_org_world, _ma_obj_world)}"
        )
    except Exception as _ma_exc:
        panel._emit_log(f"[MESH_ALIGN] exception={_ma_exc}")
    if panel._step_live_visual_label is not None:
        panel._step_live_visual_label.setText(
            panel._step_live_pose_text("XYZ de referencia visual (world)", world_frame, visual_live_display)
        )
    if panel._step_gripper_expected_label is not None:
        expected_gripper = panel._step_phase_gripper_state(current_flow, current_phase)
        panel._step_gripper_expected_label.setText(
            f"Pinza esperada en la fase seleccionada: {expected_gripper}"
        )
    if panel._step_live_gripper_label is not None:
        panel._step_live_gripper_label.setText(
            f"Pinza live: {panel._step_live_gripper_state()}"
        )
    if panel._step_object_label is not None:
        panel._step_object_label.setText(
            f"Objeto XYZ (world): {panel._step_format_inline_xyz(panel._step_display_position(panel._step_object_position))}"
        )
    if panel._step_start_pose_label is not None:
        trigger = str(panel._step_start_trigger or "").strip() or "--"
        xyz_txt = panel._step_format_inline_xyz(panel._step_display_position(panel._step_start_pose_base))
        rpy_txt = panel._step_format_inline_rpy(panel._step_start_pose_rpy_deg)
        panel._step_start_pose_label.setText(
            f"Pose inicial del robot al lanzar la secuencia (world): {xyz_txt} | "
            f"frame: {world_frame} | RPY: {rpy_txt} | botón: {trigger}"
        )
    panel._step_runtime_refresh()
    panel._step_refresh_pipeline_table()
    if getattr(panel, "_step_history_frame_help_label", None) is not None:
        panel._step_history_frame_help_label.setText(
            f"Tabla STEP (frame operacional: {world_frame} | interno: {operational_frame}@{panel._business_base_frame()}). "
            "Org=pose robot al abrir la fase | TCP-TF=TCP real por TF al cerrar | Obj World=pose objeto en world | "
            "Target/Exec separados | D TCP-Obj / D Target-Obj / Err TCP-Exec en metros | "
            "Tipo Target=OBJETO_EXACTO / OBJETO_MAS_CLEARANCE / CONTACTO_GRASP / EXEC_REAL / CACHE."
        )
    if panel._step_history_table is not None:
        def _fmt_metric(value) -> str:
            try:
                return f"{float(value):.3f}"
            except Exception:
                return "--"

        panel._step_history_table.setRowCount(len(panel._step_history_rows))
        for row_idx, row_data in enumerate(panel._step_history_rows):
            phase_name = str(row_data.get("phase") or "").strip().upper()
            pos3 = row_data.get("target")
            reached = row_data.get("reached")
            # Org: origin_snapshot congelado al abrir el gate (donde estaba el robot).
            # Fallback a actual si no hay snapshot; TF en vivo solo para la fase activa PEND.
            _origin_snap = row_data.get("origin_snapshot")
            if _origin_snap is not None:
                org3 = _origin_snap
            else:
                org3 = row_data.get("actual")
                if org3 is None and phase_name == str(panel._step_current_phase or "").strip().upper():
                    org3 = operational_live  # fallback: TF en vivo si no hay snapshot aún
            # TCP-TF (columna Cierre renombrada): pose real del TCP por TF al cerrar la fase.
            # Es None (PEND) mientras la fase no haya concluido.
            cierre3 = row_data.get("actual")
            # Target: destino teórico (calculado al planificar).
            # Exec: target efectivo enviado al IK (fijado por _step_set_exec_target).
            exec3 = row_data.get("exec_target_snapshot")
            # Objeto world y distancia TCP↔objeto al cerrar la fase.
            obj_world3 = row_data.get("object_world_snapshot")
            history_metrics = compute_step_history_metrics(panel, row_data)
            dist_tcp_obj = history_metrics.get("dist_tcp_obj")
            dist_target_obj = history_metrics.get("dist_target_obj")
            err_tcp_exec = history_metrics.get("err_tcp_exec")
            target_kind = str(history_metrics.get("target_kind") or "CACHE")
            target_note = str(history_metrics.get("target_note") or "")
            check_reason = row_data.get("check_reason") or ""
            org3_display = panel._step_display_position(org3)
            cierre3_display = panel._step_display_position(cierre3)
            pos3_display = panel._step_display_position(pos3)
            exec3_display = panel._step_display_position(exec3)
            # obj_world3 ya es world (guardado por _step_fetch_object_world que devuelve
            # pose/info en world frame). No pasar por _step_display_position (base→world)
            # porque produciría una doble conversión y valores erróneos en la tabla.
            obj_world3_display = obj_world3
            display_phase = f"{phase_name} - {panel._step_phase_intent(panel._step_history_flow, phase_name)}"
            expected_gripper = panel._step_phase_gripper_state(panel._step_history_flow, phase_name)
            values = (
                (display_phase, expected_gripper)
                + panel._step_format_xyz(org3_display)
                + panel._step_format_xyz(cierre3_display)
                + panel._step_format_xyz(obj_world3_display)
                + panel._step_format_xyz(pos3_display)
                + panel._step_format_xyz(exec3_display)
                + (_fmt_metric(dist_tcp_obj),)
                + (_fmt_metric(dist_target_obj),)
                + (_fmt_metric(err_tcp_exec),)
                + (target_kind,)
                + (check_reason,)
            )
            for col_idx, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col_idx not in (0, 20, len(values) - 1):
                    item.setTextAlignment(Qt.AlignCenter)
                if col_idx == 20 and target_note:
                    item.setToolTip(target_note)
                if col_idx == len(values) - 1 and check_reason:
                    item.setToolTip(str(check_reason))
                panel._step_history_table.setItem(row_idx, col_idx, item)
            # Estado (columna final: índice 22)
            panel._step_history_table.setItem(
                row_idx,
                len(values),
                panel._step_status_item(
                    reached,
                    row_state=str(row_data.get("row_state") or ""),
                    row_kind=str(row_data.get("row_kind") or ""),
                ),
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
