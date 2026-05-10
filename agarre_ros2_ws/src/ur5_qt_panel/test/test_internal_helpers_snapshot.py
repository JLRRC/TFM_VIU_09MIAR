#!/usr/bin/env python3
"""Audit-v4 (2026-05-08): snapshot del contrato público de internal_helpers.

Congela las signaturas de funciones top-level para que el split iter2
(deferred v1.1) no rompa consumers accidentalmente.

NOTA (F6 audit 2026-05-10): el módulo ``pick_demo/internal_helpers.py`` fue
borrado en B4 cleanup junto con todo el directorio ``pick_demo/`` (ver
git log b4-lot1/lot2). Este snapshot ya no aplica — se salta a nivel
de módulo si el archivo no existe. Se mantiene por si la auditoría de
re-introducción del split lo recupera.
"""
from __future__ import annotations

import inspect
from pathlib import Path
from typing import Set

import pytest

PKG = Path(__file__).resolve().parent.parent / "ur5_qt_panel"
MODULE_FILE = PKG / "pick_demo" / "internal_helpers.py"

pytestmark = pytest.mark.skipif(
    not MODULE_FILE.is_file(),
    reason="pick_demo/internal_helpers.py fue borrado en B4 cleanup",
)


# Funciones públicas (que pueden ser importadas desde fuera) — congeladas
# 2026-05-08. iter2 puede MOVER funciones a sub-módulos, pero las que sigan
# siendo públicas en pick_demo.internal_helpers deben aparecer aquí.
EXPECTED_PUBLIC_FUNCTIONS: Set[str] = {
    # Si alguna pasa a privada (_) post-split, eliminar de aquí explícitamente.
}


# Funciones privadas (con _) que el panel usa internamente. Documentadas
# para asegurar que no se borran por accidente durante el split.
EXPECTED_PRIVATE_FUNCTIONS: Set[str] = {
    "_live_joint_seed_or_none",
    "_resolve_live_object_world_snapshot",
    "_resolve_live_object_world_stable",
    "_resolve_live_object_world",
    "_resolve_live_object_base",
    "_select_compute_stable_promotion_status",
    "_select_pick_demo_cycle_object_reference",
    "_demo_object_in_basket",
    "_validate_demo_transport_follow",
    "_wait_for_demo_runtime_target_progress",
}


def test_module_file_exists() -> None:
    assert MODULE_FILE.is_file(), f"missing {MODULE_FILE}"


def test_loc_decreases_or_stays() -> None:
    """Baseline 1.262 LOC. iter2 debe REDUCIR — no permitido crecer."""
    n_lines = sum(1 for _ in MODULE_FILE.read_text(encoding="utf-8").splitlines())
    BASELINE = 1262
    assert n_lines <= BASELINE, (
        f"internal_helpers.py grew from baseline {BASELINE} to {n_lines}"
    )


def _module_top_level_names() -> Set[str]:
    """Devuelve los nombres de funciones top-level definidas en el archivo."""
    text = MODULE_FILE.read_text(encoding="utf-8")
    import re
    names: Set[str] = set()
    for m in re.finditer(r"^def\s+(\w+)\s*\(", text, flags=re.MULTILINE):
        names.add(m.group(1))
    return names


def test_expected_private_functions_present() -> None:
    """Las funciones privadas que el panel usa siguen presentes."""
    names = _module_top_level_names()
    missing = EXPECTED_PRIVATE_FUNCTIONS - names
    assert not missing, f"missing private helpers: {missing}"


def test_no_top_level_class_definitions() -> None:
    """internal_helpers.py es procedural — no debe ganar clases sin
    decisión explícita."""
    text = MODULE_FILE.read_text(encoding="utf-8")
    import re
    class_defs = re.findall(r"^class\s+\w+", text, flags=re.MULTILINE)
    assert not class_defs, (
        f"internal_helpers.py ganó class definitions inesperadas: {class_defs}"
    )
