#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/system_health_helpers.py
"""T33 (2026-05-08) — Helpers puros de salud del sistema (offline-friendly).

Función ``count_processes_with_name(name, process_list_provider)`` cuenta
procesos cuyo cmdline contenga ``name``. El provider es inyectable para
poder testear sin fork de subprocess.

Función ``zombie_move_group_check(process_list)`` devuelve un veredicto
estructurado: ``ZombieCheckResult(zombie_count, healthy)`` donde
healthy=True si zombie_count == 0.

Sin estado, sin ROS. Tests offline en ``test/test_system_health_helpers.py``.
"""

from __future__ import annotations

from typing import Iterable, List, NamedTuple


class ZombieCheckResult(NamedTuple):
    """Resultado del chequeo de zombies de un proceso."""

    process_name: str
    process_count: int  # No usar 'count' (colisión con tuple.count).
    healthy: bool  # True si process_count == 0 (no zombies; el primer
                  # proceso real es el "vivo" — el chequeo se hace
                  # post-cleanup, así que la expectativa es 0 antes
                  # de relanzar el stack).


def count_processes_with_name(
    name: str,
    process_cmdlines: Iterable[str],
) -> int:
    """Cuenta cmdlines que contengan ``name`` como substring.

    Args:
        name: nombre del proceso a buscar (e.g. "move_group").
        process_cmdlines: iterable de cmdlines (e.g. salida de ``ps``).

    Returns:
        Número de cmdlines que contienen ``name``.
    """
    target = str(name).strip()
    if not target:
        return 0
    return sum(1 for cmd in process_cmdlines if target in str(cmd))


def zombie_move_group_check(
    process_cmdlines: Iterable[str],
    *,
    name: str = "move_group",
) -> ZombieCheckResult:
    """Verifica si hay zombies de ``move_group`` antes de relanzar el stack.

    Contexto bug F1.16 (memoria 2026-05-08): tras varios cycles E2E,
    quedaban 3 instancias zombi de ``/move_group``. Causa raíz de los
    APPROACH timeouts del cycle 2/3.

    Returns:
        ZombieCheckResult(name, count, healthy=count==0).
    """
    n = count_processes_with_name(name, process_cmdlines)
    return ZombieCheckResult(process_name=name, process_count=n, healthy=n == 0)


def zombie_controller_bootstrap_check(
    process_cmdlines: Iterable[str],
    *,
    name: str = "controller_bootstrap",
    max_allowed: int = 1,
) -> ZombieCheckResult:
    """Audit-v4 (2026-05-08): controller_bootstrap zombie guardrail.

    Observado en stack live: 2× /controller_bootstrap activos
    simultáneamente cuando el factory bootstrap se relanza sin cleanup
    previo. El segundo intenta cargar controllers ya activos y produce
    error spam.

    El nodo es transient (one-shot), así que ``max_allowed=1`` significa
    "máximo 1 ejecutándose ahora". 0 también es healthy (post-bootstrap).

    Returns:
        ZombieCheckResult con healthy = (count <= max_allowed).
    """
    n = count_processes_with_name(name, process_cmdlines)
    return ZombieCheckResult(
        process_name=name,
        process_count=n,
        healthy=n <= int(max_allowed),
    )


def list_known_move_group_killers() -> List[str]:
    """Devuelve la lista de scripts canónicos que deben matar move_group.

    Si añades un nuevo cleanup script al stack, añádelo aquí también — y
    actualiza ``test_system_health_helpers.py::test_canonical_scripts_kill_move_group``.
    """
    return [
        "scripts/start_panel_v2.sh",
        "scripts/stop_panel_v2.sh",
        "scripts/limpia_stack.sh",
        "scripts/fresh_build.sh",
        "scripts/debug_bridge_isolated.sh",
        "scripts/run_directo_validation.sh",
        "scripts/run_directo2_validation.sh",
    ]
