#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_ready_reasons_mixin.py
# Contenido: F14-step (2026-05-02) — verifica el contrato del mixin PanelV2ReadyReasonsMixin.
"""Tests offline F14 — PanelV2ReadyReasonsMixin (estructural)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

SAMPLE_METHODS = (
    '_moveit_not_ready_reason',
    '_set_moveit_wait_status',
    '_controllers_not_ready_reason',
    '_ros_node_not_ready_reason',
    '_camera_not_ready_reason',
    '_tfm_experiment_ready_status',
    '_tfm_infer_ready_status',
    '_current_grasp_status',
    '_calibration_action_status',
    '_pose_info_not_ready_reason',
    '_tf_not_ready_reason',
)


def _read(name: str) -> str:
    return (_SRC_DIR / name).read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_ready_reasons_mixin.py")
    assert "class PanelV2ReadyReasonsMixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_ready_reasons_mixin.py")
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
            assert "PanelV2ReadyReasonsMixin" in base_names
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2ReadyReasonsMixin" in src
    assert "panel_v2_ready_reasons_mixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    matches = re.findall(rf"^\s{{4}}def\s+{re.escape(method)}\s*\(", src, flags=re.MULTILINE)
    assert not matches, f"panel_v2.py todavía define '{method}'"
