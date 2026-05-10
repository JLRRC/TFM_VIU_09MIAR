#!/usr/bin/env python3
"""F13 (auditoría 2026-05-10): MoveIt readiness pre-check.

Helpers puros para que el orchestrator pueda decidir RÁPIDAMENTE si
MoveIt está listo antes de despachar APPROACH (que actualmente
puede esperar 60s antes de fallar si move_group está caído).

El nodo orchestrator usa estos helpers junto con su lista de topics/
services/actions activos (obtenida vía rclpy.Node.get_*) y decide:
  * Si OK → despacha la fase normalmente.
  * Si NO OK → falla rápido con razón clara, evitando 60s de timeout.

Cero dependencias ROS — el caller pasa las listas como inputs.
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

#: Action que MoveIt publica cuando ``move_group`` está listo.
MOVE_GROUP_ACTION_DEFAULT = "/move_action"

#: Services que MoveIt publica al arrancar.
MOVEIT_REQUIRED_SERVICES_DEFAULT = (
    "/get_planning_scene",
    "/compute_ik",
)


def moveit_ready_from_graph(
    *,
    action_names: Sequence[str],
    service_names: Sequence[str],
    move_group_action: str = MOVE_GROUP_ACTION_DEFAULT,
    required_services: Sequence[str] = MOVEIT_REQUIRED_SERVICES_DEFAULT,
) -> Tuple[bool, str]:
    """Decide si MoveIt está listo dado el grafo ROS observable.

    Args:
        action_names: lista de actions visibles (vía
            ``node.get_action_names_and_types()``).
        service_names: lista de services visibles (vía
            ``node.get_service_names_and_types()``).
        move_group_action: action de move_group esperado.
        required_services: services adicionales que también deben estar
            (compute_ik / get_planning_scene son los más críticos).

    Returns:
        ``(ready, reason)``. ``ready=True`` si action y todos los
        services requeridos están presentes. Si NO, ``reason`` indica
        qué falta primero (action_missing o service_missing:NAME).
    """
    actions_set = set(action_names)
    services_set = set(service_names)
    if move_group_action not in actions_set:
        return False, f"action_missing:{move_group_action}"
    for svc in required_services:
        if svc not in services_set:
            return False, f"service_missing:{svc}"
    return True, "ok"


def select_precheck_timeout(
    *,
    base_timeout_sec: float,
    moveit_known_running: bool,
) -> float:
    """Selecciona timeout corto si NO se sabe si MoveIt corre.

    Política:
      * ``moveit_known_running=True``: usa timeout completo (caller
        confía en que MoveIt está arriba; le damos margen).
      * ``moveit_known_running=False``: timeout reducido a ``min(5.0,
        base_timeout_sec)`` para evitar esperas largas si está caído.

    Args:
        base_timeout_sec: timeout que el caller usaría sin pre-check.
        moveit_known_running: True si el orchestrator ya verificó
            move_group activo (e.g. via system_state_manager).

    Returns:
        Timeout efectivo en segundos.
    """
    if moveit_known_running:
        return float(base_timeout_sec)
    return min(5.0, float(base_timeout_sec))


def diagnose_missing_moveit(
    *,
    action_names: Sequence[str],
    service_names: Sequence[str],
    move_group_action: str = MOVE_GROUP_ACTION_DEFAULT,
    required_services: Sequence[str] = MOVEIT_REQUIRED_SERVICES_DEFAULT,
) -> List[str]:
    """Devuelve lista detallada de qué falta de MoveIt.

    Útil para logs estructurados. A diferencia de
    ``moveit_ready_from_graph`` que devuelve el primer fallo, este
    enumera TODOS los components ausentes.
    """
    missing: List[str] = []
    actions_set = set(action_names)
    services_set = set(service_names)
    if move_group_action not in actions_set:
        missing.append(f"action:{move_group_action}")
    for svc in required_services:
        if svc not in services_set:
            missing.append(f"service:{svc}")
    return missing


__all__ = [
    "MOVE_GROUP_ACTION_DEFAULT",
    "MOVEIT_REQUIRED_SERVICES_DEFAULT",
    "moveit_ready_from_graph",
    "select_precheck_timeout",
    "diagnose_missing_moveit",
]
