#!/usr/bin/env python3
"""F6 audit (2026-05-10): bloquea crecimiento de los launchers en panel.

`panel_launchers.py` (1 033 LOC) y `panel_gz_startup.py` (826 LOC)
contienen lógica de arranque del stack que **debería estar en
ur5_bringup/launch**, no en el panel Qt. F6 documenta el plan de
migración y este test fija el límite superior: cualquier nueva línea
hace fallar el lint y obliga a justificar (vía allowlist o migración
real al bringup).

Cuando F6b se ejecute (migración real con live-test), el baseline
debe BAJAR — el test debería actualizarse en consecuencia.
"""
from __future__ import annotations

from pathlib import Path

import pytest

WS = Path(__file__).resolve().parents[3]

# Baselines fijados 2026-05-10 (F6 docs commit). Si bajan, actualiza
# este dict tras el commit del refactor para que el lint refleje el
# nuevo techo. Si suben sin justificación, falla.
BASELINES = {
    "src/ur5_qt_panel/ur5_qt_panel/panel_launchers.py": 1033,
    "src/ur5_qt_panel/ur5_qt_panel/panel_gz_startup.py": 826,
}


@pytest.mark.parametrize("rel,baseline", BASELINES.items())
def test_panel_launcher_does_not_grow(rel: str, baseline: int) -> None:
    path = WS / rel
    assert path.is_file(), f"Archivo desaparecido: {rel}"
    actual = sum(1 for _ in path.open("r", encoding="utf-8"))
    assert actual <= baseline, (
        f"{rel}: {actual} LOC > baseline {baseline}. F6 audit bloquea "
        f"crecimiento de launchers en el panel; mueve al bringup o "
        f"actualiza BASELINES si la justificación es legítima."
    )
