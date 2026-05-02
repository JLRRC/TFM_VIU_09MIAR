#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_calibpick_mixin.py
# Contenido: F14-step (2026-05-02) — verifica el contrato del mixin PanelV2CalibPickMixin.
"""Tests offline F14 — PanelV2CalibPickMixin (estructural)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

SAMPLE_METHODS = (
    '_publish_calib_grid_marker',
    '_start_calibration',
    '_capture_calibration_frame',
    '_run_pick_demo',
    '_run_pick_object',
    '_get_object_world_position',
    '_on_slider_change',
    '_resolve_table_top_z',
    '_check_robot_above_table',
    '_load_joint_limits',
    '_build_object_report',
    '_is_pickable',
    '_go_table',
    '_go_basket',
    '_toggle_gripper_button',
    '_on_camera_click',
    '_refresh_objects_from_gz',
)


def _read(name: str) -> str:
    return (_SRC_DIR / name).read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_calibpick_mixin.py")
    assert "class PanelV2CalibPickMixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_calibpick_mixin.py")
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
            assert "PanelV2CalibPickMixin" in base_names
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2CalibPickMixin" in src
    assert "panel_v2_calibpick_mixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    matches = re.findall(rf"^\s{{4}}def\s+{re.escape(method)}\s*\(", src, flags=re.MULTILINE)
    assert not matches, f"panel_v2.py todavía define '{method}'"
