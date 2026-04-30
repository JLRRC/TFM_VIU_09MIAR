#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ros_msg_helpers.py
# Contenido: F3 — helpers de parseo de mensajes ROS (puros, sin self).
"""Helpers de parseo de mensajes ROS extraídos de RosWorker.

Funciones puras (sin estado, sin ROS spin) que toman un mensaje ya
recibido y devuelven datos. Extraídas de panel_ros.py para reducir
el tamaño del god-file (2.2k LOC) y permitir testeo aislado.

Públicas:

* ``resolve_ros_message_class(type_name)`` — convierte un nombre de
  tipo (con ``/msg/`` o ``.msg.``) en la clase Python del mensaje, o
  None si no se puede importar.
* ``extract_grasp_rect(msg)`` — extrae ``{cx, cy, w, h, theta_rad,
  angle_deg}`` de un mensaje compatible con uno de tres contratos:
  - vision_msgs/BoundingBox2D (center.position + size_x/size_y +
    center.theta).
  - Mensaje custom con campos ``c_x, c_y, w, h, theta``.
  - std_msgs/Float32MultiArray con ``data=[cx, cy, w, h, theta]``.

Ambas funciones devuelven None ante mensajes inválidos.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

try:
    from rosidl_runtime_py.utilities import get_message as _ros_get_message
except Exception:  # pragma: no cover - rosidl_runtime_py optional in tests
    _ros_get_message = None  # type: ignore


def resolve_ros_message_class(type_name: str):
    """Resolver el nombre de un tipo ROS a su clase Python.

    Acepta los dos estilos comunes (``pkg/msg/Type`` y ``pkg.msg.Type``)
    y prueba ambos. Devuelve None si rosidl_runtime_py no está
    disponible o el tipo no se puede importar.
    """
    normalized = (type_name or "").strip()
    if not normalized or _ros_get_message is None:
        return None
    candidates = [normalized]
    if "/msg/" in normalized:
        candidates.append(normalized.replace("/msg/", ".msg."))
    elif ".msg." in normalized:
        candidates.append(normalized.replace(".msg.", "/msg/"))
    for candidate in candidates:
        try:
            return _ros_get_message(candidate)
        except Exception:
            continue
    return None


def extract_grasp_rect(msg: Any) -> Optional[Dict[str, float]]:
    """Extraer rect de grasp ``{cx, cy, w, h, theta_rad, angle_deg}``.

    Tres contratos de mensaje aceptados (probados en orden):

    1. vision_msgs/BoundingBox2D: ``msg.center.position.{x,y}`` +
       ``msg.size_x`` + ``msg.size_y`` + ``msg.center.theta``.
    2. Mensaje custom con campos directos ``c_x, c_y, w, h, theta``.
    3. std_msgs/Float32MultiArray con ``msg.data = [cx, cy, w, h,
       theta]``.

    Devuelve None si el mensaje no encaja en ningún contrato o si
    cualquier conversión falla.
    """
    # 1. vision_msgs/BoundingBox2D
    try:
        center = getattr(msg, "center", None)
        position = getattr(center, "position", None) if center is not None else None
        if position is not None and hasattr(msg, "size_x") and hasattr(msg, "size_y"):
            cx = float(getattr(position, "x"))
            cy = float(getattr(position, "y"))
            width = float(getattr(msg, "size_x"))
            height = float(getattr(msg, "size_y"))
            theta_rad = float(getattr(center, "theta", 0.0))
            return {
                "cx": cx,
                "cy": cy,
                "w": width,
                "h": height,
                "theta_rad": theta_rad,
                "angle_deg": math.degrees(theta_rad),
            }
    except Exception:
        pass

    # 2. Mensaje custom (c_x, c_y, w, h, theta)
    try:
        if all(hasattr(msg, attr) for attr in ("c_x", "c_y", "w", "h", "theta")):
            theta_rad = float(getattr(msg, "theta", 0.0))
            return {
                "cx": float(getattr(msg, "c_x")),
                "cy": float(getattr(msg, "c_y")),
                "w": float(getattr(msg, "w")),
                "h": float(getattr(msg, "h")),
                "theta_rad": theta_rad,
                "angle_deg": math.degrees(theta_rad),
            }
    except Exception:
        pass

    # 3. std_msgs/Float32MultiArray fallback
    try:
        data = list(getattr(msg, "data", []))
        if len(data) >= 5:
            theta_rad = float(data[4])
            return {
                "cx": float(data[0]),
                "cy": float(data[1]),
                "w": float(data[2]),
                "h": float(data[3]),
                "theta_rad": theta_rad,
                "angle_deg": math.degrees(theta_rad),
            }
    except Exception:
        pass

    return None
