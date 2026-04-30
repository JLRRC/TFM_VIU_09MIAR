#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/step_pipeline_view.py
# Contenido: F3 — pipeline view + flow helpers + direct waiting (354 LOC).
"""Helpers del pipeline view del step pipeline.

Extraído de ``panel_step_callbacks.py`` para reducir el god-file. 19
funciones cohesivas relacionadas con:

* Flow effective / present flow name.
* Pipeline view: reset, prepare, rebuild, refresh, phase_completed.
* Direct waiting for approach confirmation (4 funciones).
* Phase gate ownership.
* Phase state computation (1-line wrappers a step_pipeline_helpers).

Cada función toma ``panel`` como primer arg. Se re-exportan desde
``panel_step_callbacks`` para preservar la API pública.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QPushButton, QTableWidgetItem

from .step_pipeline_helpers import (
    step_phase_action_text,
    step_phase_gripper_state,
    step_phase_intent,
    step_phase_sequence,
    step_predict_next_phase,
    step_present_flow_name,
)



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

