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
    "ur5_moveit_bridge.py": 1849,
    "moveit_bridge/executor.py": 1491,
    "plan_to_pose_server.py": 1322,
    "gripper_attach_backend.py": 1207,
    "release_objects_service.py": 1178,
    "system_state_manager.py": 952,
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
    assert len(LOC_BASELINE) >= 7


def test_no_backend_file_below_400_loc_in_baseline() -> None:
    """Si un fichero baja de 400, debe salir del baseline (era >600)."""
    too_small = {k: v for k, v in LOC_BASELINE.items() if v < 400}
    assert not too_small, (
        f"Estos ya bajaron de 400 LOC — sácalos del baseline: {too_small}"
    )


def test_executor_below_v4_baseline() -> None:
    """Audit-v4.1 FASE A: executor.py debe estar por debajo del baseline v4 (1.546).

    Esto bloquea cualquier intento futuro de re-introducir helpers in-place
    que vuelvan a inflar el archivo (la regresión histórica fue 1546→1663
    en commits 3b31f45/f645c44/a006ded/b9fbd7b antes de F5-iter4).
    """
    path = PKG / "moveit_bridge/executor.py"
    n = sum(1 for _ in path.read_text(encoding="utf-8").splitlines())
    assert n <= 1546, (
        f"executor.py creció a {n} LOC. El baseline v4 era 1.546 — "
        f"si extraes helpers, ponlos en un módulo aparte (ver "
        f"moveit_bridge/fjt_lifecycle_mixin.py)."
    )
