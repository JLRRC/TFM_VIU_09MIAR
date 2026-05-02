#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_base_pose_mixin.py
# Contenido: F14-step3 (2026-05-01) — verifica el contrato del mixin de base pose.
"""Tests offline del refactor F14-step3 — PanelV2BasePoseMixin.

F14-step3 extrae los wrappers thin de helpers TCP / base pose desde
``ControlPanelV2`` a ``PanelV2BasePoseMixin``. Todos delegan al
módulo ``panel_state_methods`` (alias ``_stm``).

Tests estructurales (AST + grep), correr offline sin Qt/ROS.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

EXPECTED_MIXIN_METHODS = (
    "_expected_world_frame",
    "_business_base_frame",
    "_base_frame_candidates",
    "ensure_base_pose",
    "_ensure_base_coords",
    "get_tcp_base",
    "get_tcp_pose_base",
    "transform_pose_to_base",
    "log_pose_base",
    "log_pose",
    "get_pose_in_base",
)


def _read(name: str) -> str:
    path = _SRC_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_SRC_DIR}"
    return path.read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_base_pose_mixin.py")
    assert "class PanelV2BasePoseMixin" in src, (
        "panel_v2_base_pose_mixin.py debe exportar PanelV2BasePoseMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_base_pose_mixin.py")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), (
        f"panel_v2_base_pose_mixin.py: falta método '{method}'"
    )


def test_panel_v2_inherits_from_mixin():
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
            assert "PanelV2BasePoseMixin" in base_names, (
                f"ControlPanelV2 debe heredar de PanelV2BasePoseMixin "
                f"(bases: {base_names})"
            )
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    pat = (
        r"from\s+\.panel_v2_base_pose_mixin\s+import\s+PanelV2BasePoseMixin"
    )
    assert re.search(pat, src), (
        "panel_v2.py debe importar PanelV2BasePoseMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    """ControlPanelV2 no debe redefinir métodos del mixin."""
    src = _read("panel_v2.py")
    pat = rf"^\s{{4}}def\s+{re.escape(method)}\s*\("
    matches = re.findall(pat, src, flags=re.MULTILINE)
    assert not matches, (
        f"panel_v2.py todavía define localmente '{method}'; debería "
        "estar solo en panel_v2_base_pose_mixin.py"
    )


def test_mixin_delegates_to_panel_state_methods():
    """Los wrappers deben delegar a panel_state_methods (_stm)."""
    src = _read("panel_v2_base_pose_mixin.py")
    assert "import panel_state_methods as _stm" in src, (
        "panel_v2_base_pose_mixin.py debe importar panel_state_methods "
        "como alias _stm"
    )
    # Cada método debería contener la llamada `return _stm.<method>(...)`.
    for method in EXPECTED_MIXIN_METHODS:
        pat = rf"return\s+_stm\.{re.escape(method)}\s*\("
        assert re.search(pat, src), (
            f"panel_v2_base_pose_mixin.{method} debe delegar a "
            f"_stm.{method}(...)"
        )
