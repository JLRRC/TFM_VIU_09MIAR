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
    """Goal del action PickPlace, ya validado y normalizado.

    F5-step2: ``object_pose_world_hint`` opcional. Tuple de 7 floats
    ``(x,y,z,qx,qy,qz,qw)`` o None. Si None, el orchestrator usará
    placeholder (modo F5-step1).
    """

    object_name: str
    drop_xyz_world: Tuple[float, float, float]
    object_pose_world_hint: Optional[Tuple[float, ...]] = None


def _validate_pose_hint(
    pose_hint: Optional[Tuple[float, ...]],
) -> Tuple[Optional[Tuple[float, ...]], str]:
    """Valida y normaliza un pose hint para el goal.

    Acepta:
      * None → (None, "").
      * Tuple/list de 3 (xyz) → completa con identity quat → tuple de 7.
      * Tuple/list de 7 (xyz + quat) → cast a floats.

    Devuelve ``(norm, "")`` si OK; ``(None, reason)`` si inválido.
    """
    if pose_hint is None:
        return None, ""
    try:
        coords = tuple(float(c) for c in pose_hint)
    except (TypeError, ValueError) as exc:
        return None, f"object_pose_world_hint_invalid:{exc}"
    for c in coords:
        if not math.isfinite(c):
            return None, "object_pose_world_hint_invalid:non_finite"
    if len(coords) == 3:
        return coords + (0.0, 0.0, 0.0, 1.0), ""
    if len(coords) == 7:
        return coords, ""
    return None, f"object_pose_world_hint_invalid:len={len(coords)}"


def build_goal_request(
    object_name: str,
    drop_xyz_world: Tuple[float, float, float],
    *,
    object_pose_world_hint: Optional[Tuple[float, ...]] = None,
) -> Tuple[Optional[PickPlaceGoalRequest], str]:
    """Construye y valida un PickPlaceGoalRequest.

    Devuelve ``(request, "")`` si OK, o ``(None, reason)`` si inválido.

    Reglas:
      - object_name no vacío tras strip.
      - drop_xyz_world debe ser tuple/list de 3 elementos finitos.
      - object_pose_world_hint (opcional) debe ser None, tuple/list de
        3 (xyz, identity quat asumido) o de 7 (xyz + quat).
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

    hint_norm, hint_reason = _validate_pose_hint(object_pose_world_hint)
    if hint_reason:
        return None, hint_reason

    return PickPlaceGoalRequest(
        object_name=name,
        drop_xyz_world=coords,
        object_pose_world_hint=hint_norm,
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


def should_use_orchestrator(
    env_value: Optional[str],
    *,
    legacy_env_value: Optional[str] = None,
) -> bool:
    """True si el panel debe usar el cliente del orchestrator.

    F12 (2026-05-01): default = orchestrator (True). Legacy en deprecación.
    F5-legacy-removed (2026-05-08): legacy borrado físicamente
    (run_pick_demo eliminado de panel_pick_demo.py). El orchestrator es
    ahora el ÚNICO path. Para recuperar legacy:
        git checkout audit-pre-borrar-legacy-20260508

    Reglas actuales:
      - Si ``legacy_env_value`` (USE_LEGACY_PICK_DEMO) es truthy → False
        (sentinel "legacy_removed" — dispatcher emite log y no ejecuta nada).
      - En cualquier otro caso (incluido None) → True (orchestrator).
      - ``env_value`` mantenido por compat con tests; valor falsy fuerza
        el sentinel legacy_removed.
    """
    truthy = ("1", "true", "yes", "on")
    falsy = ("0", "false", "no", "off")
    if legacy_env_value is not None:
        legacy_norm = str(legacy_env_value).strip().lower()
        if legacy_norm in truthy:
            return False
    if env_value is None:
        # F5-legacy-removed (2026-05-08): default = orchestrator
        # (era legacy en F1.7, revertido tras borrado físico del legacy).
        return True
    norm = str(env_value).strip().lower()
    if norm in falsy:
        return False
    return True
