#!/usr/bin/env python3
"""Audit-v4.1 (2026-05-08) FASE D.3: gating de lecturas ``os.environ`` en producción.

Snapshot del conjunto de archivos que **leen** ``os.environ`` en código de
producción (excluye ``test/``, ``cli_*``, ``build/``, ``install/``). El test
falla si:

1. **Aparece un archivo nuevo** con ``os.environ.*`` que no está en la
   allowlist. Esto fuerza que cualquier nueva lectura de env pase
   primero por ``panel_env``, ``*_params.py``, o ``moveit_bridge_utils``.

2. **La allowlist contiene un archivo que ya no tiene env reads** —
   señal de que la migración avanzó y la entrada debe eliminarse.

Documentación: :doc:`../../docs/ENV_INVENTORY_V4_1.md`.

**No bloquea** la deuda existente (no requiere migración inmediata).
**Bloquea** la regresión (que crezca). Esto permite cerrar la deuda en
sesiones posteriores sin tener que parar el sprint.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

WORKSPACE_SRC = Path(__file__).resolve().parents[2]
assert WORKSPACE_SRC.name == "src", f"Unexpected layout: {WORKSPACE_SRC}"

# Allowlist generada del HEAD audit-v4.1 FASE A+B (commit e9cdceb).
# Bajar es bueno (sácalo de aquí cuando migres). Crecer requiere justificación
# en el commit message + PR review (typically: nueva integración con sistema
# externo donde panel_env/bridge_env_* no aplica).
ENV_READS_ALLOWLIST: frozenset[str] = frozenset({
    # Legítimos — patrón central (panel_env, *_params, bridge_env_*)
    "src/ur5_qt_panel/ur5_qt_panel/panel_env.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_launchers_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_pick_object_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_ros_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_tfm_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_ui_params.py",
    "src/ur5_tools/ur5_tools/moveit_bridge/params.py",
    "src/ur5_tools/ur5_tools/moveit_bridge_utils.py",
    # Legítimos — launch (parámetros de arranque)
    "src/ur5_bringup/launch/launch_helpers.py",
    "src/ur5_bringup/launch/stack_factories.py",
    "src/ur5_bringup/launch/ur5_stack.launch.py",
    "src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py",
    # DEUDA — pendiente de migración (FASE D.2 — sesión dedicada)
    "src/tfm_grasping/tfm_grasping/model.py",
    "src/ur5_qt_panel/ur5_qt_panel/directo_gate_evaluator.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_startup.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_system_status.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_tfm_execute.py",
    "src/ur5_tools/ur5_tools/gripper_attach_backend.py",
    "src/ur5_tools/ur5_tools/release_objects_service.py",
    "src/ur5_tools/ur5_tools/system_state_manager.py",
    "src/ur5_tools/ur5_tools/world_tf_publisher.py",
})


# audit-v4.1 FASE D.2 (2026-05-08): regex refinada — sólo READS reales.
# Excluye writes ``os.environ["FOO"] = value`` y ``os.environ.setdefault(...)``,
# que no son la deuda perseguida (escribir env vars hacia subprocess es
# patrón legítimo). Captura:
#   * ``os.environ.get(...)``
#   * ``os.environ["FOO"]`` cuando NO va seguido de ``=`` (read no-write).
_ENV_READ_PATTERN = re.compile(
    r"os\.environ\.get\(|os\.environ\[[^\]]+\](?!\s*=)"
)


def _scan_production_env_reads() -> set[str]:
    """Devuelve los paths relativos (a src/) con env reads en producción."""
    found: set[str] = set()
    for path in WORKSPACE_SRC.rglob("*.py"):
        s = str(path)
        # Excluir tests, CLIs, build, install.
        if "/test/" in s or "/test_" in s.split("/")[-1]:
            continue
        if "/cli_" in s or path.name.startswith("cli_"):
            continue
        if "/build/" in s or "/install/" in s:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        if _ENV_READ_PATTERN.search(text):
            rel = path.relative_to(WORKSPACE_SRC.parent)
            found.add(str(rel))
    return found


def test_no_new_env_reads_outside_allowlist() -> None:
    """Cualquier archivo nuevo con env reads debe pasar por panel_env/params/launch."""
    actual = _scan_production_env_reads()
    extras = actual - ENV_READS_ALLOWLIST
    assert not extras, (
        f"{len(extras)} archivo(s) NUEVO(s) con `os.environ` fuera del patrón. "
        f"Encapsula vía panel_env / *_params.py / moveit_bridge_utils, o si "
        f"realmente es deuda nueva, añade a ENV_READS_ALLOWLIST con justificación.\n"
        f"Nuevos: {sorted(extras)}"
    )


def test_allowlist_entries_still_have_env_reads() -> None:
    """Si una entrada ya no contiene env reads, debe salir de la allowlist."""
    actual = _scan_production_env_reads()
    stale = ENV_READS_ALLOWLIST - actual
    assert not stale, (
        f"{len(stale)} entrada(s) en allowlist ya no tienen env reads. "
        f"Migrate exitoso — sácalas de ENV_READS_ALLOWLIST:\n{sorted(stale)}"
    )


def test_allowlist_count_baseline() -> None:
    """Sanity: la allowlist tiene un tamaño razonable (no se vació accidentalmente)."""
    assert len(ENV_READS_ALLOWLIST) >= 18, (
        "Allowlist sospechosamente pequeña — ¿se borró por accidente?"
    )
    # Cuando bajemos de 15, eliminar este test (la deuda estará casi cerrada).
    assert len(ENV_READS_ALLOWLIST) <= 28, (
        f"Allowlist creciendo ({len(ENV_READS_ALLOWLIST)} > 28). "
        "Estás añadiendo deuda en lugar de cerrarla."
    )
