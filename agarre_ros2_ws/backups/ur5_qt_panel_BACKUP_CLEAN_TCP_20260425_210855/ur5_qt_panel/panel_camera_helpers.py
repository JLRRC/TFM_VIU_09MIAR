#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_camera_helpers.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Camera helpers for the UR5 panel."""
from __future__ import annotations

CAMERA_TOPIC_PREFIX = "/camera"


def is_camera_topic(topic: str) -> bool:
    """Return True for the candidate camera topics we care about."""
    if not topic:
        return False
    normalized = topic.strip()
    if not normalized.startswith(CAMERA_TOPIC_PREFIX):
        return False
    leaf = normalized.split("/")[-1].lower()
    return "image" in leaf or "depth" in leaf
