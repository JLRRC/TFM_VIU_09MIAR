#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_publisher_mixin.py
# Contenido: F14-step2 (2026-05-01) — verifica el contrato del mixin de publishers.
"""Tests offline del refactor F14-step2 — PanelV2PublisherMixin.

F14-step2 extrae los métodos de publishers MoveIt y wrappers
auto-bridge de ``ControlPanelV2`` a ``PanelV2PublisherMixin``.

Estos tests son **estructurales** (AST + grep) para correr offline
sin Qt/ROS. Verifican:

1. El mixin define los 11 métodos esperados.
2. ``ControlPanelV2`` hereda del mixin antes de ``QMainWindow``.
3. ``ControlPanelV2`` ya no define localmente los métodos extraídos.
4. ``ControlPanelV2.__init__`` propaga el flag ``_use_sim_time_flag``
   que el mixin consume.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

EXPECTED_MIXIN_METHODS = (
    # 3 con lógica real
    "_init_moveit_publisher",
    "_ensure_moveit_node",
    "_publish_current_grasp_rect",
    # 8 wrappers thin a panel_state_methods
    "_moveit_publish_context",
    "_request_auto_bridge_start",
    "_auto_bridge_tick",
    "_get_traj_publisher",
    "_get_gripper_publisher",
    "_get_attach_publisher",
    "_publish_joint_trajectory",
    "_publish_moveit_pose",
)


def _read(name: str) -> str:
    path = _SRC_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_SRC_DIR}"
    return path.read_text(encoding="utf-8")


def test_mixin_module_exists():
    """panel_v2_publisher_mixin.py debe existir y exportar el mixin."""
    src = _read("panel_v2_publisher_mixin.py")
    assert "class PanelV2PublisherMixin" in src, (
        "panel_v2_publisher_mixin.py debe exportar la clase "
        "PanelV2PublisherMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_mixin_defines_method(method):
    """Cada método del contrato debe estar definido en el mixin."""
    src = _read("panel_v2_publisher_mixin.py")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), (
        f"panel_v2_publisher_mixin.py: falta método '{method}'"
    )


def test_panel_v2_inherits_from_mixin():
    """ControlPanelV2 debe heredar del mixin."""
    src = _read("panel_v2.py")
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "ControlPanelV2":
            base_names = []
            for b in node.bases:
                if isinstance(b, ast.Name):
                    base_names.append(b.id)
                elif isinstance(b, ast.Attribute):
                    base_names.append(b.attr)
            assert "PanelV2PublisherMixin" in base_names, (
                f"ControlPanelV2 debe heredar de PanelV2PublisherMixin "
                f"(bases: {base_names})"
            )
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    """panel_v2 debe importar el mixin."""
    src = _read("panel_v2.py")
    pat = r"from\s+\.panel_v2_publisher_mixin\s+import\s+PanelV2PublisherMixin"
    assert re.search(pat, src), (
        "panel_v2.py debe importar PanelV2PublisherMixin de "
        ".panel_v2_publisher_mixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    """ControlPanelV2 no debe redefinir métodos del mixin (causaría override silencioso).

    Verifica que el método sólo aparece en el mixin, no como
    ``def method(self, ...)`` indentado dentro de panel_v2.py.
    """
    src = _read("panel_v2.py")
    # Buscamos `    def <method>(` (4 espacios + def + nombre).
    pat = rf"^\s{{4}}def\s+{re.escape(method)}\s*\("
    matches = re.findall(pat, src, flags=re.MULTILINE)
    assert not matches, (
        f"panel_v2.py todavía define localmente '{method}'; debería "
        "estar solo en panel_v2_publisher_mixin.py"
    )


def test_panel_v2_init_sets_use_sim_time_flag():
    """ControlPanelV2.__init__ debe propagar _use_sim_time_flag para el mixin."""
    src = _read("panel_v2.py")
    assert "_use_sim_time_flag" in src, (
        "ControlPanelV2.__init__ debe asignar self._use_sim_time_flag "
        "(consumido por PanelV2PublisherMixin._init_moveit_publisher)"
    )
