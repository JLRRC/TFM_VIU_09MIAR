#!/usr/bin/env python3
"""Audit-v4.1 (2026-05-08): LOC baseline para los ficheros backend > 600 LOC.

Espejo de :mod:`ur5_qt_panel.test.test_ui_files_loc_baseline` pero para los
módulos de ``ur5_tools`` (bridge MoveIt, lifecycle nodes, services).

Cada entrada es deuda registrada en :doc:`../docs/AUDIT_20260508_v4_1.md`.
El test fuerza **monotonicidad**: sólo permite REDUCIR LOC, nunca crecer.

Si necesitas crecer un fichero (caso raro), súbelo aquí explícitamente con
justificación en el commit message. La excepción habitual es F5-iter*: si
estás extrayendo helpers in-place que ampliarán LOC temporalmente, **el
plan correcto es moverlos a un módulo aparte** (ver
``moveit_bridge.fjt_lifecycle_mixin`` como referencia).
"""
from __future__ import annotations

from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent / "ur5_tools"


# Baseline congelado 2026-05-08 (post FASE A audit-v4.1).
# Bajar es bueno; subir requiere actualizar este dict + justificar.
LOC_BASELINE = {
    # 2026-05-09: ur5_moveit_bridge.py + moveit_bridge/* BORRADOS
    # (path MoveIt-classic eliminado).
    "plan_to_pose_server.py": 1476,  # F1.24-refactor T15 (2026-05-09): _execute_fjt_direct + _execute_moveit_direct splits (10 sub-helpers, +154 LOC). Fix bug latente seed_positions. T15 cumplido.
    "gripper_attach_backend.py": 1220,  # F3.3 audit (2026-05-10): NOTE bloque de plan refactor F9 (+13 LOC).
    "release_objects_service.py": 1178,
    "system_state_manager.py": 953,  # F2 audit (2026-05-10): preexistente +1 LOC.
    "world_tf_publisher.py": 599,
}


@pytest.mark.parametrize("filename,baseline", sorted(LOC_BASELINE.items()))
def test_backend_file_loc_does_not_grow(filename: str, baseline: int) -> None:
    """Cada fichero backend listado debe permanecer ≤ baseline."""
    path = PKG / filename
    assert path.is_file(), f"missing {filename}"
    n = sum(1 for _ in path.read_text(encoding="utf-8").splitlines())
    assert n <= baseline, (
        f"{filename} grew from baseline {baseline} to {n} LOC. "
        "Bajar LOC es ok; crecer requiere subir el baseline aquí o "
        "extraer helpers a módulos aparte (ver fjt_lifecycle_mixin)."
    )


def test_baseline_count_reasonable() -> None:
    """Sanity: el dict no se vacía accidentalmente."""
    # 2026-05-09: 7 → 5 tras borrar ur5_moveit_bridge.py + moveit_bridge/executor.py.
    assert len(LOC_BASELINE) >= 5


def test_no_backend_file_below_400_loc_in_baseline() -> None:
    """Si un fichero baja de 400, debe salir del baseline (era >600)."""
    too_small = {k: v for k, v in LOC_BASELINE.items() if v < 400}
    assert not too_small, (
        f"Estos ya bajaron de 400 LOC — sácalos del baseline: {too_small}"
    )


# 2026-05-09: test_executor_below_v4_baseline ELIMINADO — moveit_bridge/executor.py
# fue borrado con el path MoveIt-classic.
