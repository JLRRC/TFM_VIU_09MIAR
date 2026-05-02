#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_clock_helpers.py
# Contenido: F15 (2026-05-01) — helpers de clock/freshness compartidos.
"""Helpers de clock / freshness compartidos por el panel.

F15 (2026-05-01) consolida los helpers de timestamp monotónico y
checks de freshness que estaban duplicados en ``panel_v2._runtime_time``
y ``panel_ros._steady_time`` a un único módulo puro. Sin Qt, sin ROS.

Funciones públicas:

* ``steady_time()`` — wallclock monotónico local del proceso.
* ``clock_age_threshold_exceeded(msg_mono_ts, threshold_sec, *,
  now=None)`` — True si la edad supera el umbral.
* ``format_clock_age(age_sec)`` — render canónico ``'X.YY s'`` /
  ``'stale'`` / ``'fresh'``.
"""

from __future__ import annotations

import time
from typing import Optional


def steady_time() -> float:
    """Timestamp monotónico local. Sustituye los helpers privados
    duplicados ``_runtime_time`` (panel_v2) y ``_steady_time``
    (panel_ros).
    """
    return time.monotonic()


def clock_age_threshold_exceeded(
    msg_mono_ts: Optional[float],
    threshold_sec: float,
    *,
    now: Optional[float] = None,
) -> bool:
    """Devuelve True si la edad ``now - msg_mono_ts`` supera ``threshold_sec``.

    Si ``msg_mono_ts`` es None / inválido, devuelve True (datos
    inservibles equivalen a stale).

    Si ``threshold_sec <= 0``, devuelve siempre False (umbral
    desactivado).
    """
    try:
        threshold = float(threshold_sec)
    except (TypeError, ValueError):
        return True
    if threshold <= 0.0:
        return False
    if msg_mono_ts is None:
        return True
    try:
        ts = float(msg_mono_ts)
    except (TypeError, ValueError):
        return True
    if ts <= 0.0:
        return True
    n = float(now) if now is not None else steady_time()
    age = n - ts
    return age > threshold


def format_clock_age(age_sec: Optional[float]) -> str:
    """Render canónico de la edad en segundos.

    * ``None``                       → ``'fresh'``
    * ``> 999``                      → ``'stale'``
    * ``< 0``                        → ``'fresh'`` (clock desfasado)
    * resto                          → ``'X.YY s'``
    """
    if age_sec is None:
        return "fresh"
    try:
        v = float(age_sec)
    except (TypeError, ValueError):
        return "fresh"
    if v < 0.0:
        return "fresh"
    if v > 999.0:
        return "stale"
    return f"{v:.2f} s"
