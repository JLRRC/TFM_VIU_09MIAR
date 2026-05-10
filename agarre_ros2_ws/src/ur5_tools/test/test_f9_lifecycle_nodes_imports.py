#!/usr/bin/env python3
# Ruta/archivo: src/ur5_tools/test/test_f9_lifecycle_nodes_imports.py
# Contenido: F9 audit (2026-05-10) — smoke imports de nodos lifecycle nuevos.
"""F9 audit (2026-05-10): smoke tests de los 2 LifecycleNodes nuevos."""
from __future__ import annotations

from pathlib import Path

import pytest


PKG_DIR = Path(__file__).resolve().parents[1] / "ur5_tools"


@pytest.mark.parametrize(
    "module_name",
    ["controller_health_monitor_node", "simulation_reset_service"],
)
def test_module_file_exists_and_main(module_name: str) -> None:
    src = PKG_DIR / f"{module_name}.py"
    assert src.is_file(), f"missing {src}"
    text = src.read_text(encoding="utf-8")
    assert "def main(" in text
    assert "LifecycleNode" in text
    # Anti-Qt: nodos backend nunca deben importar PyQt5.
    assert "PyQt5" not in text


def test_controller_health_monitor_publishes_to_canonical_topic() -> None:
    src = PKG_DIR / "controller_health_monitor_node.py"
    text = src.read_text(encoding="utf-8")
    assert '"/controller_health"' in text or "'/controller_health'" in text


def test_simulation_reset_service_exposes_full_reset() -> None:
    src = PKG_DIR / "simulation_reset_service.py"
    text = src.read_text(encoding="utf-8")
    assert "~/full_reset" in text
    assert "~/reset_world" in text
    assert "~/reset_robot" in text


def test_both_nodes_have_lifecycle_transitions() -> None:
    for name in ("controller_health_monitor_node", "simulation_reset_service"):
        src = PKG_DIR / f"{name}.py"
        text = src.read_text(encoding="utf-8")
        for transition in ("on_configure", "on_activate", "on_deactivate", "on_cleanup"):
            assert transition in text, f"{name}: falta transition {transition}"


def test_entry_points_declared_in_setup_py() -> None:
    setup_py = PKG_DIR.parent / "setup.py"
    text = setup_py.read_text(encoding="utf-8")
    assert "controller_health_monitor_node = ur5_tools.controller_health_monitor_node:main" in text
    assert "simulation_reset_service = ur5_tools.simulation_reset_service:main" in text
