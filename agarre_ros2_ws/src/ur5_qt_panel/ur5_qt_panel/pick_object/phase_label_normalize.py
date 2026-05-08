#!/usr/bin/env python3
"""V1.1 audit-v4.1 / F3-iter3 (2026-05-09) — phase label normalization.

Lógica pura extraída de :func:`panel_pick_object._step_phase_gate`. Mapea
las sublabels emitidas por el closure de pick (e.g. ``GRASP_DOWN_MICRO_3``,
``TRANSPORT_STAGE_2``) a su label canónica usada por el step gate del panel
(``GRASP_DOWN``, ``TRANSPORT``).

Sin dependencia de panel/ROS — testeable offline.
"""
from __future__ import annotations


_GRASP_DOWN_PREFIX = "GRASP_DOWN_MICRO_"
_LIFT_PREFIX = "STRICT_LIFT_STAGE_"
_TRANSPORT_PREFIX = "TRANSPORT_STAGE_"
_PRE_GRASP_RECENTER = "PRE_GRASP_RECENTER"


def normalize_phase_label(phase: str | None) -> str:
    """Devuelve la label canónica usada por el step gate.

    Reglas (idénticas al monolito histórico):
      * cadena vacía / None → ``""``
      * upper-case + strip
      * ``GRASP_DOWN_MICRO_*`` → ``GRASP_DOWN``
      * ``STRICT_LIFT_STAGE_*`` → ``LIFT``
      * ``TRANSPORT_STAGE_*`` → ``TRANSPORT``
      * ``PRE_GRASP_RECENTER`` → ``PRE_GRASP``
      * cualquier otra → la propia label en upper-case
    """
    label = str(phase or "").strip().upper()
    if not label:
        return ""
    if label.startswith(_GRASP_DOWN_PREFIX):
        return "GRASP_DOWN"
    if label.startswith(_LIFT_PREFIX):
        return "LIFT"
    if label.startswith(_TRANSPORT_PREFIX):
        return "TRANSPORT"
    if label == _PRE_GRASP_RECENTER:
        return "PRE_GRASP"
    return label
