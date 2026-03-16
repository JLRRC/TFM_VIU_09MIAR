#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_watchdog.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Explicit watchdog for critical system dependencies."""
from __future__ import annotations

import time
from typing import Optional

from .panel_utils import effective_base_frame

class PanelWatchdog:
    """Evaluate critical dependency timeouts and report fatal reasons."""

    def __init__(self, panel) -> None:
        self._panel = panel

    def check(self) -> Optional[str]:
        if self._panel._fatal_latched or self._panel._closing:
            return None
        now = time.monotonic()
        if self._panel._managed_mode and not self._panel._external_state_active():
            if self._panel._system_state_deadline and now > self._panel._system_state_deadline:
                return "system_state no disponible"
        clock_ok, _clock_reason = self._panel._clock_status()
        if clock_ok:
            self._panel._clock_ever_ok = True
        return self._resolve_critical_fault(now, clock_ok)

    def _resolve_critical_fault(self, now: float, clock_ok: bool) -> Optional[str]:
        p = self._panel
        release_inflight = bool(getattr(p, "_detach_inflight", False))
        base_frame = effective_base_frame(p)
        world_base_label = f"TF world->{base_frame}"
        gz_state = p._gazebo_state()
        if p._critical_clock_deadline and now > p._critical_clock_deadline:
            if gz_state == "GAZEBO_READY" and not clock_ok:
                return "Gazebo/clock no listo"
        if p._critical_pose_deadline and now > p._critical_pose_deadline:
            if gz_state == "GAZEBO_READY" and not p._pose_info_ok and not release_inflight:
                # pose/info can flap transiently while Gazebo recovers entities.
                # Keep panel operational and let readiness gates block only actions that need it.
                return None
        if p._critical_tf_deadline and now > p._critical_tf_deadline:
            if gz_state == "GAZEBO_READY" and not p._tf_ready_state:
                return f"{world_base_label} no disponible"
        if p._state_ready_basic():
            if p._clock_ever_ok and gz_state == "GAZEBO_READY" and not clock_ok:
                return "Gazebo/clock perdido"
            if p._pose_info_ever_ok and gz_state == "GAZEBO_READY" and not p._pose_info_ok and not release_inflight:
                return None
            if p._tf_ever_ok and gz_state == "GAZEBO_READY" and not p._tf_ready_state:
                return f"{world_base_label} perdido"
        return None
