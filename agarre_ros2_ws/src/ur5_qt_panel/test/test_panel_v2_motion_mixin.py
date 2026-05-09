#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_motion_mixin.py
# Contenido: F14-step5 (2026-05-01) — verifica el contrato del mixin de motion.
"""Tests offline del refactor F14-step5 — PanelV2MotionMixin.

F14-step5 extrae los wrappers de cliente FollowJointTrajectory +
helpers de espera de joint/TCP desde ``ControlPanelV2`` a
``PanelV2MotionMixin``. Todos delegan en ``panel_state_methods``
(_stm), salvo ``_wait_action_server`` que es helper estático.

Tests estructurales (AST + grep) — correr offline sin Qt/ROS.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

EXPECTED_MIXIN_METHODS = (
    "_traj_action_target",
    "_resolve_traj_action_name",
    "_get_action_client",
    "_wait_action_server",
    "_format_action_error",
    "_joint_motion_since",
    "_wait_for_joint_target",
    "_wait_for_tcp_base_z",
    "_wait_for_tcp_base_target",
)


def _read(name: str) -> str:
    path = _SRC_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_SRC_DIR}"
    return path.read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_motion_mixin.py")
    assert "class PanelV2MotionMixin" in src, (
        "panel_v2_motion_mixin.py debe exportar PanelV2MotionMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_motion_mixin.py")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), (
        f"panel_v2_motion_mixin.py: falta método '{method}'"
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
            assert "PanelV2MotionMixin" in base_names, (
                f"ControlPanelV2 debe heredar de PanelV2MotionMixin "
                f"(bases: {base_names})"
            )
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    pat = r"from\s+\.panel_v2_motion_mixin\s+import\s+PanelV2MotionMixin"
    assert re.search(pat, src), (
        "panel_v2.py debe importar PanelV2MotionMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    pat = rf"^\s{{4}}def\s+{re.escape(method)}\s*\("
    matches = re.findall(pat, src, flags=re.MULTILINE)
    assert not matches, (
        f"panel_v2.py todavía define localmente '{method}'; debería "
        "estar solo en panel_v2_motion_mixin.py"
    )


def test_wait_action_server_is_stateless():
    """_wait_action_server es helper estático en _stm — NO debe pasar self."""
    src = _read("panel_v2_motion_mixin.py")
    pat = r"return\s+_stm\._wait_action_server\(\*args,\s*\*\*kwargs\)"
    assert re.search(pat, src), (
        "_wait_action_server debe llamar a _stm._wait_action_server "
        "SIN pasar self (helper estático)"
    )


def test_other_methods_pass_self():
    """Resto de métodos del mixin pasan self como primer arg a _stm."""
    src = _read("panel_v2_motion_mixin.py")
    for method in EXPECTED_MIXIN_METHODS:
        if method == "_wait_action_server":
            continue
        pat = (
            rf"return\s+_stm\.{re.escape(method)}\(self,\s*\*args,\s*"
            r"\*\*kwargs\)"
        )
        assert re.search(pat, src), (
            f"panel_v2_motion_mixin.{method} debe delegar a "
            f"_stm.{method}(self, ...)"
        )
