#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_safety.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Safety gating helpers for panel actions."""
from __future__ import annotations

import time
from typing import Dict


class PanelSafety:
    """Centralized safety gating + throttled logging for the panel."""

    def __init__(self, panel) -> None:
        self._panel = panel
        self._log_last_ts: Dict[str, float] = {}

    def emit_log_throttled(self, key: str, msg: str, min_interval: float = 1.0) -> None:
        now = time.time()
        last = self._log_last_ts.get(key, 0.0)
        if (now - last) < min_interval:
            return
        self._log_last_ts[key] = now
        self._panel._emit_log(msg)

    def require_ready_basic(self, action: str) -> bool:
        ok, reason = self._panel._basic_ready_status()
        if ok:
            return True
        self._panel._set_status(f"{action} en espera: {reason}", error=False)
        self.emit_log_throttled(f"SAFETY:{action}:{reason}", f"[SAFETY] {action} bloqueado: {reason}")
        return False

    def require_ready_vision(self, action: str) -> bool:
        if self._panel._state_ready_vision():
            return True
        reason = self._panel._system_state_reason or self._panel._system_state.value
        self._panel._set_status(f"{action} en espera: {reason}", error=True)
        self.emit_log_throttled(f"SAFETY:{action}:{reason}", f"[SAFETY] {action} bloqueado: {reason}")
        return False

    def require_manual_ready(self, action: str) -> bool:
        ok, reason = self._panel._manual_control_status()
        if ok:
            return True
        self._panel._set_status(f"{action} en espera: {reason}", error=False)
        self.emit_log_throttled(f"SAFETY:{action}:{reason}", f"[SAFETY] {action} bloqueado: {reason}")
        return False
