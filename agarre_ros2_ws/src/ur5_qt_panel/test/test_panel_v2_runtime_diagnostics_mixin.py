#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_runtime_diagnostics_mixin.py
# Contenido: F14-step9 (2026-05-01) — verifica el contrato del mixin runtime/diagnostics.
"""Tests offline del refactor F14-step9 — PanelV2RuntimeDiagnosticsMixin.

El mixin más grande hasta la fecha: 97 wrappers thin que cubren
logs, async, external state, status, pose info, gazebo, controllers,
camera, bridge presets, start/stop runtime.

Tests estructurales (AST + grep) — correr offline sin Qt/ROS.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

SAMPLE_METHODS = (
    "_log_error",
    "_log_warning",
    "_on_async_error",
    "_run_ui_callable",
    "_run_async",
    "_log_camera_diagnostics",
    "_clock_status",
    "_pose_info_topic",
    "_gazebo_process_signal",
    "_gazebo_state",
    "_ros2_control_available",
    "_controllers_ready",
    "_list_controllers",
    "_subscribe_camera",
    "_unsubscribe_camera",
    "_start_all",
    "_stop_all",
    "_start_gazebo",
    "_recover_runtime",
    "_toggle_debug_poses",
    "check_physics_runtime",
)

LEGACY_NO_PROPAGATE = (
    "_recover_runtime",
    "_start_gazebo",
    "_toggle_debug_poses",
)


def _read(name: str) -> str:
    path = _SRC_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_SRC_DIR}"
    return path.read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_runtime_diagnostics_mixin.py")
    assert "class PanelV2RuntimeDiagnosticsMixin" in src


def test_mixin_imports_both_aliases():
    src = _read("panel_v2_runtime_diagnostics_mixin.py")
    assert "import panel_helpers as _ph" in src
    assert "import panel_gz_startup as _gs" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_runtime_diagnostics_mixin.py")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), (
        f"panel_v2_runtime_diagnostics_mixin.py: falta método '{method}'"
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
            assert "PanelV2RuntimeDiagnosticsMixin" in base_names, (
                f"ControlPanelV2 debe heredar de "
                f"PanelV2RuntimeDiagnosticsMixin (bases: {base_names})"
            )
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2RuntimeDiagnosticsMixin" in src
    assert "panel_v2_runtime_diagnostics_mixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    pat = rf"^\s{{4}}def\s+{re.escape(method)}\s*\("
    matches = re.findall(pat, src, flags=re.MULTILINE)
    assert not matches, (
        f"panel_v2.py todavía define localmente '{method}'"
    )


@pytest.mark.parametrize("method", LEGACY_NO_PROPAGATE)
def test_legacy_signature_preserved(method):
    """Métodos especiales que llamaban con `(self)` sin propagar args."""
    src = _read("panel_v2_runtime_diagnostics_mixin.py")
    pat = rf"return\s+_gs\.{re.escape(method)}\(self\)\s*$"
    assert re.search(pat, src, flags=re.MULTILINE), (
        f"{method} debe llamar _gs.{method}(self) SIN *args/**kwargs"
    )


def test_panel_v2_under_2100_lines():
    """Guardrail F14-step9: panel_v2.py <= 2100 LOC tras la extracción."""
    src = _read("panel_v2.py")
    line_count = src.count("\n") + (0 if src.endswith("\n") else 1)
    assert line_count <= 2100, (
        f"panel_v2.py creció a {line_count} LOC; el umbral F14-step9 es "
        "<=2100. Si el aumento es legítimo, considera extraer otro grupo."
    )
