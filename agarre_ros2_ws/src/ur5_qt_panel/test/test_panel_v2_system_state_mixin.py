#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_system_state_mixin.py
# Contenido: F14-step7 (2026-05-01) — verifica el contrato del mixin signals/system_state.
"""Tests offline del refactor F14-step7 — PanelV2SystemStateMixin.

F14-step7 extrae el grupo más grande de wrappers (29 métodos) de la
clase ``ControlPanelV2`` a ``PanelV2SystemStateMixin``: UI build,
debouncing, signals Qt, status/LED y system_state machine. Todos
delegan en ``panel_state_methods`` (alias ``_stm``).

Tests estructurales (AST + grep) — correr offline sin Qt/ROS.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

EXPECTED_MIXIN_METHODS = (
    # UI / event
    "_build_ui",
    "_debounced_btn_action",
    "showEvent",
    # Status / LED / signals
    "_set_status",
    "_set_status_async",
    "_set_led_async",
    "_on_tf_ready_signal",
    "_on_calib_ready_signal",
    "_run_startup_tf_sanity_check_once",
    "_on_controllers_ready_signal",
    "_on_error_signal",
    "_on_moveit_state_signal",
    "_on_trace_ready",
    "_on_calibration_check",
    # System state
    "_set_system_state",
    "_effective_system_state",
    "_trigger_fatal",
    "_resolve_system_state",
    "_build_state_snapshot",
    "_evaluate_system_state",
    "_update_system_state",
    "_check_critical_timeouts",
    "_resolve_critical_fault",
    "_state_ready_basic",
    "_state_ready_vision",
    "_state_ready_moveit",
    "_state_ready_level",
    "_manual_control_ready",
    "_calibration_topic_allowed",
    "_overhead_camera_active",
)


def _read(name: str) -> str:
    path = _SRC_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_SRC_DIR}"
    return path.read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_system_state_mixin.py")
    assert "class PanelV2SystemStateMixin" in src


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_system_state_mixin.py")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), (
        f"panel_v2_system_state_mixin.py: falta método '{method}'"
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
            assert "PanelV2SystemStateMixin" in base_names, (
                f"ControlPanelV2 debe heredar de PanelV2SystemStateMixin "
                f"(bases: {base_names})"
            )
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    pat = (
        r"from\s+\.panel_v2_system_state_mixin\s+import\s+"
        r"PanelV2SystemStateMixin"
    )
    assert re.search(pat, src), (
        "panel_v2.py debe importar PanelV2SystemStateMixin"
    )


@pytest.mark.parametrize("method", EXPECTED_MIXIN_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    pat = rf"^\s{{4}}def\s+{re.escape(method)}\s*\("
    matches = re.findall(pat, src, flags=re.MULTILINE)
    assert not matches, (
        f"panel_v2.py todavía define localmente '{method}'; debería "
        "estar solo en panel_v2_system_state_mixin.py"
    )


def test_all_methods_pass_self():
    """Todos los métodos del mixin pasan self como primer arg a _stm."""
    src = _read("panel_v2_system_state_mixin.py")
    for method in EXPECTED_MIXIN_METHODS:
        pat = (
            rf"return\s+_stm\.{re.escape(method)}\(self,\s*\*args,\s*"
            r"\*\*kwargs\)"
        )
        assert re.search(pat, src), (
            f"panel_v2_system_state_mixin.{method} debe delegar a "
            f"_stm.{method}(self, ...)"
        )
