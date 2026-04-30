#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ros_emitters.py
# Contenido: F3 — helper para emisores Qt resilientes a RuntimeError.
"""Helper minimal para emisores de signals Qt en RosWorker.

Patrón presente en panel_ros.py (8 emisores ``_emit_*_request`` con
cuerpo idéntico):

    def _emit_X_request(self, source: str) -> None:
        try:
            self.X_request.emit(source)
        except RuntimeError:
            pass

Después de ROS shutdown / ventana cerrada, los signals Qt pueden estar
desconectados y ``emit`` levanta ``RuntimeError: wrapped C/C++ object …
has been deleted``. Este helper centraliza el guard.
"""

from __future__ import annotations

from typing import Any


def safe_signal_emit(signal: Any, *args: Any) -> None:
    """Emit ``signal`` con ``args``; ignora RuntimeError post-shutdown.

    El único error que silenciamos es ``RuntimeError`` (objeto Qt
    destruido). Cualquier otro error propaga.
    """
    try:
        signal.emit(*args)
    except RuntimeError:
        pass
