#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/tf_geometry_logic.py
# Contenido: F16 (2026-05-01) — lógica pura del tf_geometry_service.
"""Lógica pura del ``tf_geometry_service`` (F16).

F16 (2026-05-01) crea un nodo ROS dedicado que aloja los servicios
``WorldToBase`` y ``ComputeApproachPose`` (ya definidos en
``ur5_panel_interfaces``). Este módulo contiene los helpers
matemáticos puros consumidos por el nodo, testeables sin ROS.

Funciones públicas:

* ``compute_approach_pose(object_pose_base, z_clearance_m)`` —
  devuelve la pose de approach desplazada ``z_clearance_m`` arriba
  en Z respecto del objeto, conservando la orientación. Tipos
  domésticos (tuples) para no depender de ``geometry_msgs``.
* ``apply_world_to_base_transform(world_xyz, t_world_base,
  q_world_base)`` — aplica la transformación rígida
  ``world → base_link`` a un punto.
* ``rotate_vector_by_quat(v, q)`` — rota un vector 3D por un
  quaternión (x, y, z, w).
* ``invert_transform(t, q)`` — invierte una transformación rígida
  (rotación + traslación).
"""

from __future__ import annotations

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (x, y, z, w)


def rotate_vector_by_quat(v: Vec3, q: Quat) -> Vec3:
    """Rota el vector ``v`` por el quaternión ``q`` (x, y, z, w).

    Implementa ``v' = q ⊗ v ⊗ q⁻¹`` directamente (sin matrices
    intermedias) para minimizar acumulación de error.
    """
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    # v' = v + qw*t + cross(q.xyz, t)
    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


def invert_transform(t: Vec3, q: Quat) -> Tuple[Vec3, Quat]:
    """Invierte la transformación rígida ``(t, q)``.

    Para un quaternión unitario, ``q⁻¹ = (-x, -y, -z, w)``. La
    traslación inversa es ``-q⁻¹ ⊗ t``.
    """
    qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    qi = (-qx, -qy, -qz, qw)
    neg_t = (-float(t[0]), -float(t[1]), -float(t[2]))
    ti = rotate_vector_by_quat(neg_t, qi)
    return ti, qi


def apply_world_to_base_transform(
    world_xyz: Vec3,
    t_world_base: Vec3,
    q_world_base: Quat,
) -> Vec3:
    """Convierte un punto en frame ``world`` a frame ``base_link``.

    ``t_world_base`` y ``q_world_base`` representan la transformación
    de ``world`` a ``base_link`` (es decir, el origen y rotación de
    ``base_link`` expresados en ``world``). Para llevar un punto ``p``
    expresado en ``world`` a ``base_link`` aplicamos la inversa.
    """
    t_inv, q_inv = invert_transform(t_world_base, q_world_base)
    rotated = rotate_vector_by_quat(
        (
            float(world_xyz[0]),
            float(world_xyz[1]),
            float(world_xyz[2]),
        ),
        q_inv,
    )
    return (
        rotated[0] + t_inv[0],
        rotated[1] + t_inv[1],
        rotated[2] + t_inv[2],
    )


def compute_approach_pose(
    object_pose_base: Tuple[Vec3, Quat],
    z_clearance_m: float,
) -> Tuple[Vec3, Quat]:
    """Calcula la pose de approach desplazada en Z sobre el objeto.

    Recibe ``object_pose_base`` como tupla ``(xyz, quat)`` con el
    objeto expresado en ``base_link``. Devuelve la pose de approach
    desplazada ``z_clearance_m`` hacia arriba (Z+ en base_link),
    manteniendo la orientación.

    Si ``z_clearance_m <= 0``, no aplica desplazamiento (la pose de
    approach coincide con el objeto, lo cual es semánticamente
    inválido pero matemáticamente válido — el servicio debe rechazar
    esta entrada al nivel de validación).
    """
    pos, quat = object_pose_base
    clearance = max(0.0, float(z_clearance_m))
    return (
        (
            float(pos[0]),
            float(pos[1]),
            float(pos[2]) + clearance,
        ),
        (
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3]),
        ),
    )


def quat_norm(q: Quat) -> float:
    """Norma euclídea del quaternión."""
    return math.sqrt(
        float(q[0]) ** 2
        + float(q[1]) ** 2
        + float(q[2]) ** 2
        + float(q[3]) ** 2
    )


def is_unit_quat(q: Quat, *, tol: float = 1e-3) -> bool:
    """True si ``q`` es unitario dentro de una tolerancia ``tol``."""
    return abs(quat_norm(q) - 1.0) <= tol


def normalize_quat(q: Quat) -> Quat:
    """Devuelve el quaternión normalizado. Si la norma es 0, devuelve
    la identidad ROS ``(0, 0, 0, 1)``.
    """
    n = quat_norm(q)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (
        float(q[0]) / n,
        float(q[1]) / n,
        float(q[2]) / n,
        float(q[3]) / n,
    )
