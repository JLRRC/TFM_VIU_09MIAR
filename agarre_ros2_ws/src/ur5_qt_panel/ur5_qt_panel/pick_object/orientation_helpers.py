#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/orientation_helpers.py
# Contenido: F3 — helpers puros de orientación quaternion para pick_object.
"""Helpers puros de orientación / quaternion para el pick_object flow.

Extraídos de las closures internas ``_quat_multiply`` y
``_pick_orientation`` de ``panel_pick_object.run_pick_object``.

Diseño:

* ``quat_multiply(q1, q2)``: convención (x, y, z, w) — la misma que
  usa attach_math y ROS geometry_msgs.
* ``parse_orientation_xyzw(raw, default)``: parsea string CSV "x,y,z,w"
  con fallback al default. Útil para leer del param/env.
* ``compose_grasp_orientation(base_q, yaw_deg)``: aplica un yaw
  alrededor de Z al cuaternion base. Si yaw_deg es None o no
  finito, devuelve base_q sin cambio.
* ``normalize_quat(q)``: norma 1, fallback a (0,0,0,1) si zero.

Cero dependencias ROS — testeable en pytest puro.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple


_DEFAULT_GRASP_BASE_QUAT: Tuple[float, float, float, float] = (
    0.70710678, 0.0, 0.70710678, 0.0,
)


def normalize_quat(
    q: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Normaliza ``q`` (x,y,z,w) a norma 1. Fallback (0,0,0,1) si norma ~0."""
    try:
        n = math.sqrt(sum(c * c for c in q))
    except (TypeError, ValueError):
        return (0.0, 0.0, 0.0, 1.0)
    if n <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(c / n for c in q)


def quat_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Multiplicación de cuaterniones convención (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        (w1 * x2) + (x1 * w2) + (y1 * z2) - (z1 * y2),
        (w1 * y2) - (x1 * z2) + (y1 * w2) + (z1 * x2),
        (w1 * z2) + (x1 * y2) - (y1 * x2) + (z1 * w2),
        (w1 * w2) - (x1 * x2) - (y1 * y2) - (z1 * z2),
    )


def parse_orientation_xyzw(
    raw: str,
    *,
    default: Tuple[float, float, float, float] = _DEFAULT_GRASP_BASE_QUAT,
) -> Tuple[float, float, float, float]:
    """Parsea ``raw`` CSV "x,y,z,w" a tuple normalizada.

    Si ``raw`` es vacío, no parseable, no son 4 floats, o algún
    componente no es finito, devuelve ``default`` normalizado.
    """
    s = str(raw or "").strip()
    if not s:
        return normalize_quat(default)
    try:
        parts = [float(p.strip()) for p in s.split(",")]
    except (ValueError, AttributeError):
        return normalize_quat(default)
    if len(parts) != 4 or not all(math.isfinite(v) for v in parts):
        return normalize_quat(default)
    n = math.sqrt(sum(v * v for v in parts))
    if n <= 1e-9:
        return normalize_quat(default)
    return tuple(v / n for v in parts)


def compose_grasp_orientation(
    base_q: Tuple[float, float, float, float],
    yaw_deg: Optional[float],
    *,
    use_yaw: bool = True,
) -> Tuple[float, float, float, float]:
    """Aplica un yaw (grados) alrededor de Z al cuaternion base.

    Si ``yaw_deg`` es None, ``use_yaw`` es False, o el yaw no es
    finito, devuelve ``base_q`` (normalizado) sin modificar.

    El cuaternion resultante se normaliza a norma 1.
    """
    base = normalize_quat(base_q)
    if not use_yaw or yaw_deg is None:
        return base
    try:
        yd = float(yaw_deg)
    except (TypeError, ValueError):
        return base
    if not math.isfinite(yd):
        return base
    yaw_rad = math.radians(yd)
    yaw_q = (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))
    out = quat_multiply(yaw_q, base)
    return normalize_quat(out)
