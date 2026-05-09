#!/usr/bin/env python3
"""Audit-v4 (2026-05-08): LOC baseline para los 8 ficheros UI > 800 LOC.

Cada uno se considera deuda v1.1 (split por subdominio). El test
fuerza monotonicidad — sólo permite REDUCIR LOC, nunca crecer.

Si necesitas crecer un fichero (caso raro), súbelo aquí explícitamente
con justificación en el commit message.
"""
from __future__ import annotations

from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent / "ur5_qt_panel"


# Baseline congelado 2026-05-08 (HEAD audit-v4 cierre).
# Bajar es bueno; subir requiere actualizar este dict + justificar.
LOC_BASELINE = {
    "cameras_tab.py": 1134,
    "panel_status_mgmt.py": 1049,
    "panel_trace_callbacks.py": 1033,
    "panel_utils.py": 1034,  # audit-v4.1/D.2 (2026-05-08): +1 LOC migración panel_env.
    "panel_tfm_science.py": 1031,
    "panel_launchers.py": 1033,  # audit-v4.1/D.2 (2026-05-08): +2 LOC migración panel_env.
    "panel_gz_objects.py": 1015,
    "panel_camera_controllers.py": 883,
    # Ya snapshot'd en archivos dedicados pero también incluyen baseline.
    "panel_helpers.py": 1441,
    "panel_ros.py": 2158,
    # 2026-05-09: panel_pick_object.py BORRADO (path MoveIt-classic eliminado).
    "panel_v2.py": 1252,
}


@pytest.mark.parametrize("filename,baseline", sorted(LOC_BASELINE.items()))
def test_ui_file_loc_does_not_grow(filename: str, baseline: int) -> None:
    """Cada fichero UI listado debe permanecer ≤ baseline."""
    path = PKG / filename
    assert path.is_file(), f"missing {filename}"
    n = sum(1 for _ in path.read_text(encoding="utf-8").splitlines())
    assert n <= baseline, (
        f"{filename} grew from baseline {baseline} to {n} LOC. "
        "Bajar LOC es ok; crecer requiere subir el baseline aquí."
    )


def test_baseline_count_reasonable() -> None:
    """Sanity: el dict no se vacía accidentalmente."""
    assert len(LOC_BASELINE) >= 8


def test_no_ui_file_below_400_loc_in_baseline() -> None:
    """Si un fichero baja de 400, debe salir del baseline (era >800)."""
    too_small = {k: v for k, v in LOC_BASELINE.items() if v < 400}
    assert not too_small, (
        f"Estos ya bajaron de 400 LOC — sácalos del baseline: {too_small}"
    )
