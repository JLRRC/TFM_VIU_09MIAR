#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_overlaysselection_mixin.py
# Contenido: F14-step (2026-05-02) — verifica el contrato del mixin PanelV2OverlaysSelectionMixin.
"""Tests offline F14 — PanelV2OverlaysSelectionMixin (estructural)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

SAMPLE_METHODS = (
    '_compute_homography',
    '_draw_calib_overlay',
    '_draw_selection_overlay',
    '_draw_grasp_overlay',
    '_should_draw_reach_overlay',
    '_should_draw_selection_overlay',
    '_compute_test_corner_base_points',
    '_draw_tcp_pose_overlay',
    '_save_overhead_frame_with_overlays',
    '_handle_calibration_click',
    '_handle_object_selection_click',
    '_select_object',
    '_log_selection_tf',
    '_on_object_clicked',
    '_update_objects',
    '_push_history',
    '_update_fps_stats',
)


def _read(name: str) -> str:
    return (_SRC_DIR / name).read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_overlaysselection_mixin.py")
    assert "class PanelV2OverlaysSelectionMixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_overlaysselection_mixin.py")
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
            assert "PanelV2OverlaysSelectionMixin" in base_names
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2OverlaysSelectionMixin" in src
    assert "panel_v2_overlaysselection_mixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    matches = re.findall(rf"^\s{{4}}def\s+{re.escape(method)}\s*\(", src, flags=re.MULTILINE)
    assert not matches, f"panel_v2.py todavía define '{method}'"
