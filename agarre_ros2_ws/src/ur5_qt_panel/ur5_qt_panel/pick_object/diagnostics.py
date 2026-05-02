#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/diagnostics.py
# Contenido: F15 (2026-05-01) — helpers puros de diagnóstico de fases pick_object.
"""Helpers puros de logs estructurados de pick_object (F15).

F15 (2026-05-01) extrae los formateadores de logs estructurados de
``run_pick_object`` (panel_pick_object.py, 4 623 LOC) a este módulo
puro. Sin Qt, sin ROS — testeable offline.

Funciones públicas:

* ``format_phase_marker(phase, *, status='ok', detail='')`` — render
  canónico ``"[PICK_OBJECT][PHASE_X] status=ok detail=..."``.
* ``format_phase_metrics(phase, metrics)`` — pares clave=valor
  ordenados, valores formateados consistentemente (str, int, float).
* ``format_target_pose(xyz_world, *, frame='world')`` — pose
  estructurada legible.
* ``classify_phase_outcome(success, reason)`` — clasifica el outcome
  ``ok|failed|timeout|unknown`` a partir del reason del bridge.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, Tuple


def format_phase_marker(
    phase: str,
    *,
    status: str = "ok",
    detail: str = "",
) -> str:
    """Render canónico ``[PICK_OBJECT][PHASE] status=ok detail=...``.

    Maneja phase/status/detail vacíos sin crashear.
    """
    p = str(phase or "UNKNOWN").strip().upper()
    s = str(status or "ok").strip().lower()
    d = str(detail or "").strip()
    if d:
        return f"[PICK_OBJECT][{p}] status={s} detail={d}"
    return f"[PICK_OBJECT][{p}] status={s}"


def format_phase_metrics(
    phase: str,
    metrics: Dict[str, Any],
) -> str:
    """Render de pares clave=valor ordenados.

    Floats se formatean con 4 decimales máx, ints como enteros, otros
    via ``str()``. Si ``metrics`` no es dict o está vacío, devuelve
    sólo el marcador de fase.

    Ejemplo::

        format_phase_metrics("APPROACH", {"x": 0.5, "y": 0.0, "z": 0.3})
        → "[PICK_OBJECT][APPROACH] x=0.5000 y=0.0000 z=0.3000"
    """
    p = str(phase or "UNKNOWN").strip().upper()
    if not isinstance(metrics, dict) or not metrics:
        return f"[PICK_OBJECT][{p}]"
    parts = []
    for key in sorted(metrics.keys()):
        val = metrics[key]
        if isinstance(val, bool):
            parts.append(f"{key}={'true' if val else 'false'}")
        elif isinstance(val, int):
            parts.append(f"{key}={val}")
        elif isinstance(val, float):
            parts.append(f"{key}={val:.4f}")
        else:
            parts.append(f"{key}={val}")
    return f"[PICK_OBJECT][{p}] " + " ".join(parts)


def format_target_pose(
    xyz_world: Tuple[float, float, float],
    *,
    frame: str = "world",
) -> str:
    """Render legible de una pose objetivo en un frame dado.

    Tolerante a tuples / lists / arrays de 3 elementos. Si la entrada
    no es válida, devuelve ``"target=invalid"``.
    """
    try:
        coords = tuple(float(c) for c in xyz_world[:3])
        if len(coords) != 3:
            raise ValueError
    except (TypeError, ValueError, IndexError):
        return "target=invalid"
    f = str(frame or "world").strip()
    return f"target=({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})@{f}"


def classify_phase_outcome(
    success: Any,
    reason: str,
) -> str:
    """Clasifica el outcome de una fase: ``ok``, ``failed``, ``timeout``, ``unknown``.

    Reglas:
      * ``success is True`` → ``"ok"``.
      * ``success is False`` y reason contiene ``"timeout"`` →
        ``"timeout"``.
      * ``success is False`` con cualquier otra reason → ``"failed"``.
      * Cualquier otro caso → ``"unknown"``.
    """
    if success is True:
        return "ok"
    if success is False:
        r = str(reason or "").lower()
        if "timeout" in r or "timed_out" in r:
            return "timeout"
        return "failed"
    return "unknown"


def filter_known_metrics(
    metrics: Dict[str, Any],
    allowed_keys: Iterable[str],
) -> Dict[str, Any]:
    """Filtra ``metrics`` dejando solo las claves en ``allowed_keys``.

    Útil para sanear payload antes de un log estructurado: descarta
    claves transitorias / debug.
    """
    if not isinstance(metrics, dict):
        return {}
    allowed = set(allowed_keys)
    return {k: v for k, v in metrics.items() if k in allowed}
