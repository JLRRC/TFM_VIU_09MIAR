#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/seed_metrics.py
"""F3-step40 (2026-05-08) — IK seed deviation metrics (puras).

Extraídas de panel_pick_demo.py (líneas 386-407). Detectan soluciones IK
en una rama distinta a la del seed (>90° de deviación angular).

API:
- ``seed_devs(q_arr, s_arr)`` — lista de desviaciones por joint.
- ``seed_max_dev(q_arr, s_arr)`` — desviación máxima.
- ``seed_sum_dev(q_arr, s_arr)`` — suma de desviaciones.

Sin estado. Sin ROS. Tests offline en ``test/test_pick_demo_seed_metrics.py``.
"""

from __future__ import annotations

import math
from typing import Iterable, List

_TWO_PI_R = 2.0 * math.pi


def seed_devs(q_arr: Iterable[float], s_arr: Iterable[float]) -> List[float]:
    """Devuelve |q + 2π·round((s-q)/2π) - s| por joint.

    El término ``2π·round((s-q)/2π)`` es la corrección por wrap angular:
    si la solución IK ``q`` está en una rama distinta del seed ``s``, el
    término ajusta el wrap antes de medir la diferencia.

    Returns:
        Lista de desviaciones angulares (rad) en el mismo orden que
        ``q_arr`` y ``s_arr``.
    """
    return [
        abs(float(q) + _TWO_PI_R * round((float(s) - float(q)) / _TWO_PI_R) - float(s))
        for q, s in zip(q_arr, s_arr)
    ]


def seed_max_dev(q_arr: Iterable[float], s_arr: Iterable[float]) -> float:
    """Desviación máxima entre IK solution y seed (rad)."""
    return max(seed_devs(q_arr, s_arr))


def seed_sum_dev(q_arr: Iterable[float], s_arr: Iterable[float]) -> float:
    """Suma de desviaciones entre IK solution y seed (rad)."""
    return sum(seed_devs(q_arr, s_arr))
