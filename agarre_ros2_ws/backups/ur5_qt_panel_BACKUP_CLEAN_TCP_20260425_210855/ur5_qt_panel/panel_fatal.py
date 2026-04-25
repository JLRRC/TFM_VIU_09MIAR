#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_fatal.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Fatal shutdown helpers for the panel."""
from __future__ import annotations


def abort_local_stack(panel) -> None:
    """Abort local stack processes (non-managed mode)."""
    try:
        panel._stop_bag()
    except Exception:
        pass
    try:
        panel._stop_moveit_bridge()
    except Exception:
        pass
    try:
        panel._stop_moveit()
    except Exception:
        pass
    try:
        panel._stop_bridge()
    except Exception:
        pass
    try:
        panel._stop_world_tf_publisher()
    except Exception:
        pass
    try:
        panel._stop_gazebo()
    except Exception:
        pass
