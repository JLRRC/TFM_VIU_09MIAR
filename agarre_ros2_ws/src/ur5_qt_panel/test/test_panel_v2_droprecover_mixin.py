#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_droprecover_mixin.py
# Contenido: F14-step (2026-05-02) — verifica el contrato del mixin PanelV2DropRecoverMixin.
"""Tests offline F14 — PanelV2DropRecoverMixin (estructural)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

SAMPLE_METHODS = (
    '_log_trace',
    '_apply_debug_button_style',
    '_print_pose_snapshot',
    '_drop_detach_supported',
    '_release_objects',
    '_schedule_release_retry',
    '_attach_drop_objects',
    '_resolve_set_pose_service',
    '_resolve_world_sdf_path',
    '_run_gz_service_cli',
    '_recover_pick_demo_to_table',
    '_hold_drop_objects',
    '_drop_hold_tick',
    '_maybe_recover_pick_demo',
)


def _read(name: str) -> str:
    return (_SRC_DIR / name).read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_droprecover_mixin.py")
    assert "class PanelV2DropRecoverMixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_droprecover_mixin.py")
    assert re.search(rf"def\s+{re.escape(method)}\s*\(", src)


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
            assert "PanelV2DropRecoverMixin" in base_names
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2DropRecoverMixin" in src
    assert "panel_v2_droprecover_mixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    matches = re.findall(rf"^\s{{4}}def\s+{re.escape(method)}\s*\(", src, flags=re.MULTILINE)
    assert not matches, f"panel_v2.py todavía define '{method}'"
