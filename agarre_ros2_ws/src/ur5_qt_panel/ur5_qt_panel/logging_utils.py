# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/logging_utils.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Utilities for timestamped logging output in the panel."""

from __future__ import annotations

from datetime import datetime


def timestamped_line(message: str) -> str:
    """Return *message* prefixed with the current local timestamp."""

    timestamp = datetime.now().isoformat(timespec="seconds")
    return f"[{timestamp}] {message}"


class _PanelLogger:
    """Logger façade so ControlPanelV2 can call get_logger().info(...)."""

    def __init__(self, panel) -> None:
        self._panel = panel

    def info(self, msg: str) -> None:
        self._panel._log(msg)
