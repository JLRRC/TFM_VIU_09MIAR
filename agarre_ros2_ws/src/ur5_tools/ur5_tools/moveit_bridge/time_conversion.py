#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/moveit_bridge/time_conversion.py
"""F3-step41b (2026-05-08) — Conversiones tiempo ROS puras.

Extraído de executor.py:445-453 donde se convierte ``goal_time_tol_sec``
(float seconds) a ``(sec: int, nsec: int)`` para asignar a
``goal.goal_time_tolerance.sec`` y ``.nanosec``.

La conversión naive `int((seconds - sec) * 1e9)` puede producir
``nsec >= 1_000_000_000`` por errores de redondeo (e.g. 0.999999999s
→ sec=0, nsec=999999999.5 → 1_000_000_000). El helper maneja el
overflow al sec siguiente.

Sin estado, sin ROS al importar. Tests offline.
"""

from __future__ import annotations

from typing import Tuple

NANOS_PER_SEC = 1_000_000_000


def seconds_to_sec_nsec(seconds: float) -> Tuple[int, int]:
    """Convierte ``seconds`` (float) a ``(sec, nsec)`` para builtin_interfaces/Duration.

    Maneja correctamente:
    - seconds < 0 → clamps a (0, 0).
    - seconds = 0.0 → (0, 0).
    - Redondeo de nanos que produce overflow → suma 1 al sec.
    - Float64 precision: usa round() en el componente nsec.

    Returns:
        Tuple[int, int]: (sec, nsec) con 0 <= nsec < 1_000_000_000.

    Examples:
        >>> seconds_to_sec_nsec(0.0)
        (0, 0)
        >>> seconds_to_sec_nsec(1.5)
        (1, 500000000)
        >>> seconds_to_sec_nsec(-0.5)  # negative clamps to zero
        (0, 0)
        >>> seconds_to_sec_nsec(0.9999999995)  # near-1s edge
        (1, 0)
    """
    total = max(0.0, float(seconds))
    sec = int(total)
    nsec = int(round((total - sec) * NANOS_PER_SEC))
    if nsec >= NANOS_PER_SEC:
        sec += 1
        nsec -= NANOS_PER_SEC
    return sec, nsec


def is_negative_or_zero(seconds: float) -> bool:
    """True si ``seconds`` <= 0.0 (sentinel para "no aplicar tolerancia").

    En el contrato del FJT goal, valores negativos se interpretan como
    "no setear goal_time_tolerance" — MoveIt aplica su default. Este
    helper centraliza ese chequeo.
    """
    try:
        return float(seconds) <= 0.0
    except (TypeError, ValueError):
        return True
