#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_gripper_attach_mixin.py
# Contenido: F14-step4 (2026-05-01) — verifica el contrato del mixin gripper/attach.
"""Tests offline del refactor F14-step4 — PanelV2GripperAttachMixin.

F14-step4 extrae los wrappers de comandos gripper + lógica de attach
desde ``ControlPanelV2`` a ``PanelV2GripperAttachMixin``. Todos
delegan en ``panel_state_methods`` (alias ``_stm``), salvo
``_normalize_attach_name`` que es helper estático sin self.

Tests estructurales (AST + grep), correr offline sin Qt/ROS.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

EXPECTED_MIXIN_METHODS = (
    "_normalize_attach_name",
    "_find_attach_candidate",
    "_attempt_attach",
    "_schedule_attach_attempt",
    "_command_gripper",
    "_command_gripper_preopen",
)


def _read(name: str) -> str:
    path = _SRC_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_SRC_DIR}"
    return path.read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_gripper_attach_mixin.py")
    assert "class PanelV2GripperAttachMixin" in src, (
        "panel_v2_gripper_attach_mixin.py debe exportar "
        "PanelV2GripperAttachMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_gripper_attach_mixin.py")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), (
        f"panel_v2_gripper_attach_mixin.py: falta método '{method}'"
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
            assert "PanelV2GripperAttachMixin" in base_names, (
                f"ControlPanelV2 debe heredar de PanelV2GripperAttachMixin "
                f"(bases: {base_names})"
            )
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    pat = (
        r"from\s+\.panel_v2_gripper_attach_mixin\s+import\s+"
        r"PanelV2GripperAttachMixin"
    )
    assert re.search(pat, src), (
        "panel_v2.py debe importar PanelV2GripperAttachMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    """ControlPanelV2 no debe redefinir métodos del mixin."""
    src = _read("panel_v2.py")
    pat = rf"^\s{{4}}def\s+{re.escape(method)}\s*\("
    matches = re.findall(pat, src, flags=re.MULTILINE)
    assert not matches, (
        f"panel_v2.py todavía define localmente '{method}'; debería "
        "estar solo en panel_v2_gripper_attach_mixin.py"
    )


def test_normalize_attach_name_is_stateless_passthrough():
    """_normalize_attach_name no debe pasar `self` a _stm (es helper estático)."""
    src = _read("panel_v2_gripper_attach_mixin.py")
    # Buscamos la línea de retorno que NO incluye `self`:
    pat = r"return\s+_stm\._normalize_attach_name\(\*args,\s*\*\*kwargs\)"
    assert re.search(pat, src), (
        "_normalize_attach_name debe llamar a _stm._normalize_attach_name "
        "SIN pasar self (helper estático)"
    )


def test_other_methods_pass_self_to_stm():
    """Resto de métodos del mixin pasan self como primer arg a _stm.<m>."""
    src = _read("panel_v2_gripper_attach_mixin.py")
    for method in EXPECTED_MIXIN_METHODS:
        if method == "_normalize_attach_name":
            continue  # excepción documentada arriba
        pat = (
            rf"return\s+_stm\.{re.escape(method)}\(self,\s*\*args,\s*"
            r"\*\*kwargs\)"
        )
        assert re.search(pat, src), (
            f"panel_v2_gripper_attach_mixin.{method} debe delegar a "
            f"_stm.{method}(self, ...)"
        )
