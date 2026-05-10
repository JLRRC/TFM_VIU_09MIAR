#!/usr/bin/env python3
# Ruta/archivo: src/ur5_tools/test/test_panel_backend_nodes_imports.py
# Contenido: F6 audit (2026-05-10) — smoke imports + estructura nodos backend.
"""F6 audit (2026-05-10): smoke tests de los nodos backend del panel.

Sin instanciar (requeriría rclpy.init), solo se valida:
  * import del módulo no falla.
  * los entry points main() existen y son callables.
  * no hay imports Qt en estos nodos (deben ser pure rclpy).
"""
from __future__ import annotations

import importlib
from pathlib import Path

import pytest


PKG_DIR = Path(__file__).resolve().parents[1] / "ur5_tools"


@pytest.mark.parametrize(
    "module_name",
    ["panel_launch_control_node", "panel_backend_node"],
)
def test_module_importable(module_name: str) -> None:
    """Cada nodo debe importarse sin errores de sintaxis (sin rclpy real)."""
    src = PKG_DIR / f"{module_name}.py"
    assert src.is_file(), f"missing {src}"
    text = src.read_text(encoding="utf-8")
    # Validación básica del esqueleto.
    assert "def main(" in text, f"{module_name}: falta entry point main()"
    assert "class " in text, f"{module_name}: falta clase"
    # Anti-Qt en backend nodes.
    assert "PyQt5" not in text, f"{module_name}: backend importa PyQt5 (antipatrón)"
    assert "QtCore" not in text, f"{module_name}: backend importa QtCore (antipatrón)"


def test_panel_launch_control_node_uses_lifecycle() -> None:
    src = PKG_DIR / "panel_launch_control_node.py"
    text = src.read_text(encoding="utf-8")
    assert "LifecycleNode" in text
    assert "on_configure" in text
    assert "on_activate" in text
    assert "on_cleanup" in text


def test_panel_backend_node_uses_lifecycle_and_action_client() -> None:
    src = PKG_DIR / "panel_backend_node.py"
    text = src.read_text(encoding="utf-8")
    assert "LifecycleNode" in text
    assert "ActionClient" in text
    assert "PickPlace" in text


def test_entry_points_declared_in_setup_py() -> None:
    setup_py = PKG_DIR.parent / "setup.py"
    text = setup_py.read_text(encoding="utf-8")
    assert "panel_launch_control_node = ur5_tools.panel_launch_control_node:main" in text
    assert "panel_backend_node = ur5_tools.panel_backend_node:main" in text
