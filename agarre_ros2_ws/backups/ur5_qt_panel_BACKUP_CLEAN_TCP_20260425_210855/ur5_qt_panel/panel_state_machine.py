#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_state_machine.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Explicit state machine for panel system state resolution."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .panel_state import SystemState


@dataclass(frozen=True)
class StateDecision:
    state: SystemState
    reason: str
    fatal_reason: Optional[str] = None


class PanelStateMachine:
    """Resolve panel system state from external state or local snapshot."""

    def decide_external(self, panel) -> StateDecision:
        if not panel._external_state_active():
            return StateDecision(SystemState.WAITING_GAZEBO, "Esperando /system_state")
        target, reason = panel._resolve_external_state()
        if target is None:
            return StateDecision(SystemState.WAITING_GAZEBO, "Esperando /system_state")
        if target == SystemState.ERROR_FATAL:
            return StateDecision(target, reason or "ERROR_FATAL", fatal_reason=reason or "ERROR_FATAL")
        return StateDecision(target, reason or target.value)
