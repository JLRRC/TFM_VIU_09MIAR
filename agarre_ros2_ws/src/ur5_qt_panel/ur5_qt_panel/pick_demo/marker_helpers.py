#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/marker_helpers.py
# Contenido: F3 — helpers puros de creación de markers RViz para el pick_demo.
"""Helpers puros para construir Marker (RViz) del pick_demo.

Extraído de ``panel_pick_demo.run_pick_demo`` (closures
``_direct_make_sphere_marker`` y ``_direct_make_arrow_marker``).
Son funciones puras: dado xyz/rgba devuelven un ``Marker`` (o None
si los msgs ROS no están disponibles).

Las firmas exigen ``stamp`` como argumento explícito (en el closure
original venía del cierre via ``_direct_debug_stamp``). Esto rompe
la dependencia con el panel y permite testeo aislado.
"""

from __future__ import annotations

from typing import Optional, Tuple

try:
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker
except Exception:  # pragma: no cover - ROS not available in unit contexts
    Point = None  # type: ignore
    Marker = None  # type: ignore


def make_sphere_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    xyz: Tuple[float, float, float],
    rgba: Tuple[float, float, float, float],
    stamp=None,
    diameter_m: float = 0.018,
):
    """Construye un Marker SPHERE en ``frame_id`` con xyz/color rgba.

    Devuelve None si Marker no es importable o xyz inválido.
    """
    if Marker is None or xyz is None:
        return None
    try:
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    except (TypeError, ValueError, IndexError):
        return None
    marker = Marker()
    marker.header.frame_id = str(frame_id or "world")
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = str(ns or "")
    marker.id = int(marker_id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    d = float(diameter_m)
    marker.scale.x = d
    marker.scale.y = d
    marker.scale.z = d
    marker.color.r = float(rgba[0])
    marker.color.g = float(rgba[1])
    marker.color.b = float(rgba[2])
    marker.color.a = float(rgba[3])
    return marker


def make_arrow_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    start_xyz: Tuple[float, float, float],
    end_xyz: Tuple[float, float, float],
    rgba: Tuple[float, float, float, float],
    stamp=None,
    shaft_diameter_m: float = 0.004,
    head_diameter_m: float = 0.008,
    head_length_m: float = 0.012,
):
    """Construye un Marker ARROW desde ``start_xyz`` hasta ``end_xyz``.

    Devuelve None si Marker o Point no son importables.
    """
    if Marker is None or Point is None or start_xyz is None or end_xyz is None:
        return None
    try:
        sx, sy, sz = float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2])
        ex, ey, ez = float(end_xyz[0]), float(end_xyz[1]), float(end_xyz[2])
    except (TypeError, ValueError, IndexError):
        return None
    marker = Marker()
    marker.header.frame_id = str(frame_id or "world")
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = str(ns or "")
    marker.id = int(marker_id)
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = float(shaft_diameter_m)
    marker.scale.y = float(head_diameter_m)
    marker.scale.z = float(head_length_m)
    marker.color.r = float(rgba[0])
    marker.color.g = float(rgba[1])
    marker.color.b = float(rgba[2])
    marker.color.a = float(rgba[3])
    marker.points = [
        Point(x=sx, y=sy, z=sz),
        Point(x=ex, y=ey, z=ez),
    ]
    return marker
