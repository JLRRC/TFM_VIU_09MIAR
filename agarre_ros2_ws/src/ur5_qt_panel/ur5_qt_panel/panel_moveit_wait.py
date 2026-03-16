#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_moveit_wait.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""MoveIt wait helper for the panel."""
from __future__ import annotations

import time

from .panel_config import MOVEIT_READY_TIMEOUT_SEC
from .panel_state import SystemState


def wait_for_moveit_ready(panel) -> None:
    """Esperar a que move_group esté listo (action server + topics)."""
    deadline = time.monotonic() + max(2.0, MOVEIT_READY_TIMEOUT_SEC)
    while time.monotonic() < deadline:
        if panel._closing:
            return
        if panel._system_state == SystemState.ERROR_FATAL:
            panel._emit_log_throttled(
                "SAFETY:moveit_wait:ERROR_FATAL",
                "[SAFETY] MoveIt wait abortado: ERROR_FATAL",
            )
            return
        if panel.moveit_proc is None or panel.moveit_proc.poll() is not None:
            try:
                panel.signal_moveit_state.emit("ERROR", "move_group terminó")
            except RuntimeError:
                return
            panel._moveit_launching = False
            panel._moveit_launch_start = 0.0
            panel.signal_refresh_controls.emit()
            return
        if not panel._ros_worker_started:
            panel._ensure_ros_worker_started()
        if panel._move_group_ready():
            try:
                panel.signal_moveit_state.emit("READY", "move_group activo")
            except RuntimeError:
                return
            panel._moveit_launching = False
            panel._moveit_launch_start = 0.0
            panel.signal_refresh_controls.emit()
            return
        panel._wait_for_state_change(0.5)
    try:
        action_ready = panel._moveit_action_ready()
        status_ready = panel._moveit_status_ready()
        traj_ready = panel._follow_joint_traj_ready()
        panel._emit_log(
            f"[MOVEIT] READY checks: action={action_ready} status={status_ready} traj={traj_ready}"
        )
        panel.signal_moveit_state.emit("WAITING_MOVEIT_READY", "timeout esperando move_group")
    except RuntimeError:
        return
    panel.signal_refresh_controls.emit()
