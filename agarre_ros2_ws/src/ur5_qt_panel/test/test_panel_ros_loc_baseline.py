#!/usr/bin/env python3
"""F5 audit (2026-05-10): bloquea crecimiento de panel_ros.py.

`panel_ros.py` (2 126 LOC) es el god-file del panel: 8 services + 5
subs + 4 pubs + RosWorker core. F5 documenta el plan de split y este
test fija el baseline. Cualquier crecimiento sin migración real al
split obliga a actualizar el baseline o, mejor, a ejecutar F5b.

Cuando F5b se complete, el baseline DEBE bajar drásticamente
(target < 800 LOC, los 1300 LOC restantes en módulos extraídos).
"""
from __future__ import annotations

from pathlib import Path

WS = Path(__file__).resolve().parents[3]
PANEL_ROS = WS / "src" / "ur5_qt_panel" / "ur5_qt_panel" / "panel_ros.py"

# Baseline 2026-05-10. Bajar tras F5b.
BASELINE_LOC = 2126


def test_panel_ros_does_not_grow() -> None:
    assert PANEL_ROS.is_file()
    actual = sum(1 for _ in PANEL_ROS.open("r", encoding="utf-8"))
    assert actual <= BASELINE_LOC, (
        f"panel_ros.py: {actual} LOC > baseline {BASELINE_LOC}. F5 audit "
        f"bloquea crecimiento. Si vas a hacer F5b (split) actualiza el "
        f"baseline a la baja en este test tras los commits del split."
    )
