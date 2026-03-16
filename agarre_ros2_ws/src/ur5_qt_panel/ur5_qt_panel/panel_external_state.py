#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_external_state.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""External /system_state helpers for the panel."""
from __future__ import annotations

import time
from typing import Optional, Tuple

from .panel_state import SystemState, MoveItState, EXTERNAL_STATE_MAP


def external_state_active(panel) -> bool:
    if not panel._external_state:
        return False
    return (time.time() - panel._external_state_last) < 2.0


def resolve_external_state(panel) -> Tuple[Optional[SystemState], str]:
    state = (panel._external_state or "").upper()
    reason = panel._external_state_reason or ""
    target = EXTERNAL_STATE_MAP.get(state)
    if target is None:
        return None, ""
    if target == SystemState.READY_VISION and panel._moveit_required:
        if panel._moveit_state != MoveItState.READY:
            reason = panel._moveit_not_ready_reason()
            return SystemState.READY_VISION, reason
        return SystemState.READY_MOVEIT, reason
    return target, (reason or f"Estado externo: {state}")


def apply_external_system_state(panel) -> None:
    target, reason = resolve_external_state(panel)
    if target is None:
        return
    if target == SystemState.ERROR_FATAL:
        panel._trigger_fatal(reason or "ERROR_FATAL")
        return
    if target == SystemState.ERROR:
        panel._system_error_reason = reason or panel._system_error_reason
    else:
        panel._system_error_reason = ""
    panel._set_system_state(target, reason)
    if target in (SystemState.READY_BASIC, SystemState.READY_VISION, SystemState.READY_MOVEIT):
        panel._ensure_pose_subscription()
        panel._start_pose_info_watch()
        panel._start_tf_ready_timer()
