#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_place_client_logic.py
# Contenido: F6.4 — lógica pura del cliente PickPlace (sin ROS).
"""Lógica pura del cliente de la action ``PickPlace``.

Diseño:

* La parte ROS (ActionClient, send_goal, callbacks) vive en
  ``pick_place_client.py``.
* Aquí están los **conversores y validators** puros:
  - ``build_goal_request(object_name, drop_xyz_world)``: produce el
    request listo para serializar.
  - ``feedback_to_panel_event(feedback)``: transforma el Feedback de
    ROS en un dict simple consumible por el panel Qt sin importar
    ros msgs.
  - ``result_to_panel_event(result)``: idem para el Result.
* No toca ROS, no levanta excepciones.

El propósito de F6.4 es ofrecer un cliente thin del orchestrator
desde el panel Qt — opt-in via env var. El run_pick_demo legacy
(10.7k LOC) permanece intacto: si el flag no está activo, sigue el
flujo embebido. Esto permite migración progresiva sin riesgo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


@dataclass(frozen=True)
class PickPlaceGoalRequest:
    """Goal del action PickPlace, ya validado y normalizado."""

    object_name: str
    drop_xyz_world: Tuple[float, float, float]


def build_goal_request(
    object_name: str,
    drop_xyz_world: Tuple[float, float, float],
) -> Tuple[Optional[PickPlaceGoalRequest], str]:
    """Construye y valida un PickPlaceGoalRequest.

    Devuelve ``(request, "")`` si OK, o ``(None, reason)`` si inválido.

    Reglas:
      - object_name no vacío tras strip.
      - drop_xyz_world debe ser tuple/list de 3 elementos finitos.
    """
    name = str(object_name or "").strip()
    if not name:
        return None, "object_name_empty"

    try:
        if drop_xyz_world is None:
            raise ValueError("None")
        coords = tuple(float(c) for c in drop_xyz_world[:3])
        if len(coords) != 3:
            raise ValueError(f"len={len(coords)}")
        for c in coords:
            if not math.isfinite(c):
                raise ValueError("non_finite")
    except (TypeError, ValueError, IndexError) as exc:
        return None, f"drop_xyz_world_invalid:{exc}"

    return PickPlaceGoalRequest(
        object_name=name,
        drop_xyz_world=coords,
    ), ""


def feedback_to_panel_event(feedback: Any) -> Dict[str, Any]:
    """Convierte el Feedback de PickPlace.action a dict simple.

    Acepta cualquier objeto con los atributos ``current_phase``,
    ``progress``, ``phase_index``, ``detail`` (es decir, una instancia
    de PickPlace.Feedback o un mock equivalente).

    Tolerante: campos faltantes se reemplazan por defaults razonables.
    """
    if feedback is None:
        return {
            "current_phase": "",
            "progress": 0.0,
            "phase_index": -1,
            "detail": "",
        }
    return {
        "current_phase": str(getattr(feedback, "current_phase", "") or ""),
        "progress": float(getattr(feedback, "progress", 0.0) or 0.0),
        "phase_index": int(getattr(feedback, "phase_index", -1) or -1),
        "detail": str(getattr(feedback, "detail", "") or ""),
    }


def result_to_panel_event(result: Any) -> Dict[str, Any]:
    """Convierte el Result de PickPlace.action a dict simple."""
    if result is None:
        return {
            "success": False,
            "reason": "no_result",
            "duration_sec": 0.0,
            "cycles_completed": 0,
        }
    return {
        "success": bool(getattr(result, "success", False)),
        "reason": str(getattr(result, "reason", "") or ""),
        "duration_sec": float(getattr(result, "duration_sec", 0.0) or 0.0),
        "cycles_completed": int(getattr(result, "cycles_completed", 0) or 0),
    }


def should_use_orchestrator(env_value: Optional[str]) -> bool:
    """True si el panel debe usar el cliente del orchestrator.

    Activado por ``PANEL_PICK_DEMO_USE_ORCHESTRATOR=1|true|yes|on``.
    Default: False (sigue el legacy run_pick_demo).
    """
    if env_value is None:
        return False
    return str(env_value).strip().lower() in ("1", "true", "yes", "on")
