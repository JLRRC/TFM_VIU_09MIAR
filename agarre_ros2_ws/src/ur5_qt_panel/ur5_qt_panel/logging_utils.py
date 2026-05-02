# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/logging_utils.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Utilities for timestamped logging output in the panel."""

from __future__ import annotations

import sys
from datetime import datetime
from typing import IO, Optional


def timestamped_line(message: str) -> str:
    """Return *message* prefixed with the current local timestamp."""

    timestamp = datetime.now().isoformat(timespec="seconds")
    return f"[{timestamp}] {message}"


def emit_log_line(
    line: str,
    *,
    stream: Optional[IO[str]] = None,
    flush: bool = True,
) -> None:
    """Write *line* to *stream* (default ``sys.stdout``) appending a newline.

    Single point of control for the panel's diagnostic output. Replaces the
    duplicated module-level ``print()`` calls inside the legacy
    ``_log_exception`` helpers so that future migration to ``logging`` stdlib
    only needs to be done here.
    """

    target = stream if stream is not None else sys.stdout
    target.write(line if line.endswith("\n") else f"{line}\n")
    if flush:
        target.flush()


class _PanelLogger:
    """Logger façade so ControlPanelV2 can call get_logger().info(...)."""

    def __init__(self, panel) -> None:
        self._panel = panel

    def info(self, msg: str) -> None:
        self._panel._log(msg)
