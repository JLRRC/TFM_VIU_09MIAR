#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/planning_scene_sync_helpers.py
# Contenido: F3-step6 (audit-v4 2026-05-08) — helpers puros del planning_scene_sync.
"""Pure helpers extraídos de ``planning_scene_sync.py``.

El nodo ``PlanningSceneSync`` requiere `rclpy` para arrancar. Estos helpers,
en cambio, son matemática pura (cuaterniones, transformaciones de poses) y
pueden testearse sin ROS sourced.

Se re-exportan desde ``planning_scene_sync`` con su nombre original (con
underscore prefix) para preservar la API interna existente.

Funciones:

* ``strip_ns(tag)`` — elimina namespace de un tag XML (Gazebo SDF).
* ``quat_from_rpy(r, p, y)`` — construye quaternion (xyzw) desde Euler RPY.
* ``quat_multiply(lhs, rhs)`` — Hamilton product de dos quaterniones xyzw.
* ``quat_conjugate(q)`` — conjugado de un quaternion.
* ``rotate_vector(quat, v)`` — rota un vector 3D por un quaternion.
* ``compose_pose(parent, child)`` — composición de poses (parent ∘ child).
* ``parse_pose_text(text)`` — parsea formato SDF "x y z roll pitch yaw" a tuple7.
"""
from __future__ import annotations

import math
from typing import Tuple


def strip_ns(tag: str) -> str:
    """Elimina prefijo de namespace XML del tag, si existe."""
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def quat_from_rpy(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """Construye quaternion (xyzw) desde Euler intrínsecos RPY (orden ZYX)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_multiply(
    lhs: Tuple[float, float, float, float],
    rhs: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Hamilton product de dos quaterniones xyzw: ``lhs ⊗ rhs``."""
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
        (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
        (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
        (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
    )


def quat_conjugate(
    quat: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Conjugado de un quaternion xyzw: ``(-x, -y, -z, w)``."""
    qx, qy, qz, qw = quat
    return (-qx, -qy, -qz, qw)


def rotate_vector(
    quat: Tuple[float, float, float, float],
    vector: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Rota un vector 3D por un quaternion xyzw via ``q ⊗ v ⊗ q*``."""
    vx, vy, vz = vector
    vec_quat = (vx, vy, vz, 0.0)
    rotated = quat_multiply(quat_multiply(quat, vec_quat), quat_conjugate(quat))
    return (rotated[0], rotated[1], rotated[2])


def compose_pose(
    parent: Tuple[float, float, float, float, float, float, float],
    child: Tuple[float, float, float, float, float, float, float],
) -> Tuple[float, float, float, float, float, float, float]:
    """Compone dos poses tuple7 (xyz + xyzw): ``parent ∘ child``."""
    px, py, pz, pqx, pqy, pqz, pqw = parent
    cx, cy, cz, cqx, cqy, cqz, cqw = child
    ox, oy, oz = rotate_vector((pqx, pqy, pqz, pqw), (cx, cy, cz))
    qx, qy, qz, qw = quat_multiply((pqx, pqy, pqz, pqw), (cqx, cqy, cqz, cqw))
    return (px + ox, py + oy, pz + oz, qx, qy, qz, qw)


def parse_pose_text(
    text: str,
) -> Tuple[float, float, float, float, float, float, float]:
    """Parsea un string SDF ``"x y z roll pitch yaw"`` a tuple7 (xyz+xyzw).

    Si el texto está vacío, malformado, o tiene <6 valores, devuelve
    pose neutra (0,0,0,0,0,0,1). Fail-soft.
    """
    values = [part for part in (text or "").split() if part]
    if len(values) < 6:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    try:
        x, y, z, rr, pp, yy = (float(values[idx]) for idx in range(6))
    except Exception:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    qx, qy, qz, qw = quat_from_rpy(rr, pp, yy)
    return (x, y, z, qx, qy, qz, qw)
