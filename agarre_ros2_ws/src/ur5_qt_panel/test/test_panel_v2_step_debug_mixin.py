#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_step_debug_mixin.py
# Contenido: F14-step8 (2026-05-01) — verifica el contrato del mixin step/direct/cart_debug.
"""Tests offline del refactor F14-step8 — PanelV2StepDebugMixin.

Mixin gigante (72 wrappers) extraído de ``ControlPanelV2``: step UI,
direct flow, cart debug, debug motion. La mayoría delega a
``panel_helpers`` (alias ``_ph``); ``_step_joint`` delega a
``panel_motion_control`` (alias ``_mc``).
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

# Subset representativo (no exhaustivo — el mixin tiene 72 métodos y
# verificarlos todos individualmente sería redundante con los tests
# offline ya existentes para los demás mixins).
SAMPLE_METHODS = (
    "_set_debug_motion_button_waiting",
    "_on_step_continue_clicked",
    "_step_window_refresh",
    "_step_phase_completed",
    "_step_record_history",
    "_direct_waiting_for_approach_confirmation",
    "_step_cart_debug_log_event",
    "_step_cartesian_move_runtime_target",
    "_read_gripper_feedback_state",
    "_on_debug_motion_button",
    "_debug_motion_wait_for_continue",
    "_step_joint",
)


def _read(name: str) -> str:
    path = _SRC_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_SRC_DIR}"
    return path.read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_step_debug_mixin.py")
    assert "class PanelV2StepDebugMixin" in src


def test_mixin_imports_panel_helpers_and_motion_control():
    src = _read("panel_v2_step_debug_mixin.py")
    assert "import panel_helpers as _ph" in src
    assert "import panel_motion_control as _mc" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_step_debug_mixin.py")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), (
        f"panel_v2_step_debug_mixin.py: falta método '{method}'"
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
            assert "PanelV2StepDebugMixin" in base_names, (
                f"ControlPanelV2 debe heredar de PanelV2StepDebugMixin "
                f"(bases: {base_names})"
            )
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    pat = (
        r"from\s+\.panel_v2_step_debug_mixin\s+import\s+"
        r"PanelV2StepDebugMixin"
    )
    assert re.search(pat, src)


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    pat = rf"^\s{{4}}def\s+{re.escape(method)}\s*\("
    matches = re.findall(pat, src, flags=re.MULTILINE)
    assert not matches, (
        f"panel_v2.py todavía define localmente '{method}'"
    )


def test_step_joint_delegates_to_motion_control():
    """_step_joint debe delegar a _mc (no _ph)."""
    src = _read("panel_v2_step_debug_mixin.py")
    pat = r"return\s+_mc\._step_joint\(self,\s*\*args,\s*\*\*kwargs\)"
    assert re.search(pat, src)


def test_on_debug_motion_button_legacy_signature_preserved():
    """_on_debug_motion_button preserva el comportamiento legacy: NO
    propaga *args/**kwargs (el método original llamaba
    `_ph._on_debug_motion_button(self)` sin propagar).
    """
    src = _read("panel_v2_step_debug_mixin.py")
    # El método debe NOT propagar args/kwargs en su return.
    # Buscamos la línea exacta que tiene `_on_debug_motion_button(self)`
    # SIN args.
    pat = r"return\s+_ph\._on_debug_motion_button\(self\)\s*$"
    assert re.search(pat, src, flags=re.MULTILINE), (
        "_on_debug_motion_button debe llamar _ph._on_debug_motion_button(self) "
        "SIN propagar args (preservar comportamiento legacy)"
    )


def test_panel_v2_under_2400_lines():
    """Guardrail F14-step8: panel_v2.py <= 2400 LOC tras la extracción."""
    src = _read("panel_v2.py")
    line_count = src.count("\n") + (0 if src.endswith("\n") else 1)
    assert line_count <= 2400, (
        f"panel_v2.py creció a {line_count} LOC; el umbral F14-step8 es "
        "<=2400. Si el aumento es legítimo, considera extraer otro grupo "
        "de wrappers a un mixin adicional."
    )
