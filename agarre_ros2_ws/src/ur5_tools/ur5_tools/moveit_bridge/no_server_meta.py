#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/moveit_bridge/no_server_meta.py
"""F3-step41a (2026-05-08) — Helpers puros de meta NO_SERVER del executor.

Extrae la construcción del meta dict y reason string que el executor
emite cuando no encuentra el action server del controller. Función pura,
testeable offline.

Esta es la primera extracción de F3-step41 (partir
_execute_joint_trajectory_action 1.343 LOC). El objetivo final es que
todo el cuerpo de _execute_jt_action sea una secuencia de llamadas a
helpers puros + bloques cortos de orquestación.
"""

from __future__ import annotations

from typing import Dict, Sequence


def build_no_server_meta(
    *,
    expected_action: str,
    available_actions: Sequence[str],
    candidates: Sequence[str],
    status_text: str = "NO_SERVER",
) -> Dict[str, str]:
    """Construye el meta dict para el caso 'controller action no disponible'.

    Args:
        expected_action: nombre del action que el bridge esperaba
            (e.g. ``/joint_trajectory_controller/follow_joint_trajectory``).
        available_actions: actions detectados en el entorno (probablemente
            con el mismo namespace pero distinto nombre).
        candidates: candidatos que se probaron antes de rendirse.
        status_text: texto canónico ("NO_SERVER" por default).

    Returns:
        Dict ``{"action", "status_text", "error_string"}`` listo para
        meter en el ``meta`` del result.
    """
    available = ",".join(sorted(str(a) for a in available_actions)) if available_actions else "none"
    checked = ",".join(str(c) for c in candidates) if candidates else "none"
    return {
        "action": str(expected_action),
        "status_text": str(status_text),
        "error_string": f"checked={checked} available={available}",
    }


def build_no_server_reason(
    *,
    expected_action: str,
    available_actions: Sequence[str],
    candidates: Sequence[str],
) -> str:
    """Construye la reason string canónica de NO_SERVER.

    Format: ``fjt_no_action_server:expected_action=X;checked_candidates=Y;available_actions=Z``.
    El orchestrator y evidence_logger parsean este formato — no cambiar
    sin actualizar consumers.
    """
    available = ",".join(sorted(str(a) for a in available_actions)) if available_actions else "none"
    checked = ",".join(str(c) for c in candidates) if candidates else "none"
    return (
        f"fjt_no_action_server:"
        f"expected_action={expected_action};"
        f"checked_candidates={checked};"
        f"available_actions={available}"
    )
