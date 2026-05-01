#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/wait_helpers.py
# Contenido: F3 — helpers puros de wait/timeout/log para pick_object.
"""Helpers puros de wait / timeout / log para pick_object flow.

Extraídos de las mega-closures de ``run_pick_object`` (especialmente
``_wait_moveit_result`` 248 LOC y ``_tf_distance_check`` 199 LOC).

Funciones públicas:

* ``compute_wait_chunk_sec(deadline, now, *, min_chunk, max_chunk)``:
  calcula el chunk de espera entre ``[min_chunk, max_chunk]`` clamped
  al tiempo restante hasta ``deadline``.
* ``clamp_grace_window(value, *, floor)``: clamp con default si
  ``value`` no es float válido.
* ``format_wait_state_log(state, label, elapsed_sec, expected_id,
  expected_uuid, last_seen_id, last_seen_uuid)``: builder del log
  ``[PICK_OBJ][WAIT_RESULT]``.

Cero dependencia ROS / threading.
"""

from __future__ import annotations

from typing import Optional


def compute_wait_chunk_sec(
    deadline: float,
    now: float,
    *,
    min_chunk: float = 0.2,
    max_chunk: float = 1.0,
) -> float:
    """Chunk de espera entre [min_chunk, max_chunk] clamped a tiempo restante.

    Si ``deadline <= now``, devuelve ``min_chunk`` (caller debe
    chequear deadline antes para evitar entrar al loop).
    """
    remaining = float(deadline) - float(now)
    if remaining <= 0.0:
        return float(min_chunk)
    return min(float(max_chunk), max(float(min_chunk), remaining))


def clamp_grace_window(
    value: object,
    *,
    floor: float,
    default: Optional[float] = None,
) -> float:
    """Clamp a ``value`` (float-able) a ``>= floor``. Si no parseable, usa default.

    Si ``default`` es None y ``value`` no es válido, usa ``floor`` como default.
    """
    try:
        v = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        v = float(default) if default is not None else float(floor)
    return max(float(floor), v)


def format_wait_state_log(
    *,
    state: str,
    label: str,
    elapsed_sec: float,
    expected_id: int,
    expected_uuid: str,
    last_seen_id: int,
    last_seen_uuid: str,
) -> str:
    """Builder del log ``[PICK_OBJ][WAIT_RESULT]``."""
    return (
        f"[PICK_OBJ][WAIT_RESULT] state={state} label={label} "
        f"elapsed={float(elapsed_sec):.1f}s "
        f"expected_id={int(expected_id)} expected_uuid={expected_uuid or 'n/a'} "
        f"last_seen_id={int(last_seen_id)} last_seen_uuid={last_seen_uuid or 'n/a'}"
    )
