#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_traj_settle_mixin.py
# Contenido: F14-step6 (2026-05-01) — verifica el contrato del mixin traj+settle.
"""Tests offline del refactor F14-step6 — PanelV2TrajSettleMixin.

F14-step6 extrae los wrappers thin de cliente FollowJointTrajectory
+ objects settle desde ``ControlPanelV2``. Todos delegan en
``panel_state_methods`` (alias ``_stm``).

Tests estructurales (AST + grep) — correr offline sin Qt/ROS.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

EXPECTED_MIXIN_METHODS = (
    "_send_joint_trajectory_action",
    "_schedule_traj_action_fallback",
    "_clamp_joint_positions",
    "_log_traj_action_fallback",
    "_start_objects_settle_watch",
    "_invalidate_settle",
    "_run_fall_test_async",
    "_objects_settle_worker",
    "_handle_objects_settled",
    "_log_calib_blocked",
    "_log_settle_snapshot",
    "_request_settle_snapshot",
    "wait_for_objects_to_settle",
)


def _read(name: str) -> str:
    path = _SRC_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_SRC_DIR}"
    return path.read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_traj_settle_mixin.py")
    assert "class PanelV2TrajSettleMixin" in src, (
        "panel_v2_traj_settle_mixin.py debe exportar PanelV2TrajSettleMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_traj_settle_mixin.py")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), (
        f"panel_v2_traj_settle_mixin.py: falta método '{method}'"
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
            assert "PanelV2TrajSettleMixin" in base_names, (
                f"ControlPanelV2 debe heredar de PanelV2TrajSettleMixin "
                f"(bases: {base_names})"
            )
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    pat = (
        r"from\s+\.panel_v2_traj_settle_mixin\s+import\s+"
        r"PanelV2TrajSettleMixin"
    )
    assert re.search(pat, src), (
        "panel_v2.py debe importar PanelV2TrajSettleMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    pat = rf"^\s{{4}}def\s+{re.escape(method)}\s*\("
    matches = re.findall(pat, src, flags=re.MULTILINE)
    assert not matches, (
        f"panel_v2.py todavía define localmente '{method}'; debería "
        "estar solo en panel_v2_traj_settle_mixin.py"
    )


def test_all_methods_pass_self():
    """Todos los métodos del mixin pasan self como primer arg a _stm."""
    src = _read("panel_v2_traj_settle_mixin.py")
    for method in EXPECTED_MIXIN_METHODS:
        pat = (
            rf"return\s+_stm\.{re.escape(method)}\(self,\s*\*args,\s*"
            r"\*\*kwargs\)"
        )
        assert re.search(pat, src), (
            f"panel_v2_traj_settle_mixin.{method} debe delegar a "
            f"_stm.{method}(self, ...)"
        )
