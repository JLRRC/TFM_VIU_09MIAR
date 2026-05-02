#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_tfmsciencetrace_mixin.py
# Contenido: F14-step (2026-05-02) — verifica el contrato del mixin PanelV2TfmScienceTraceMixin.
"""Tests offline F14 — PanelV2TfmScienceTraceMixin (estructural)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

SAMPLE_METHODS = (
    '_tfm_repro_profile',
    '_tfm_postprocess_enabled',
    '_on_tfm_repro_mode_changed',
    '_discover_tfm_checkpoints',
    '_pick_default_tfm_checkpoint',
    '_tfm_get_ckpt_path',
    '_tfm_apply_experiment',
    '_tfm_reset_grasp',
    '_load_experiment_info',
    '_refresh_science_ui',
    '_world_to_pixel',
    '_compute_world_grasp',
    '_world_grasp_to_base',
    '_save_episode',
    '_start_trace_timer',
    '_refresh_trace_data',
    '_tf_sanity_check',
    '_run_self_check_once',
    '_try_mark_tf_ready',
    '_log_tf_chain_once',
)


def _read(name: str) -> str:
    return (_SRC_DIR / name).read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_tfmsciencetrace_mixin.py")
    assert "class PanelV2TfmScienceTraceMixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_tfmsciencetrace_mixin.py")
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
            assert "PanelV2TfmScienceTraceMixin" in base_names
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2TfmScienceTraceMixin" in src
    assert "panel_v2_tfmsciencetrace_mixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    matches = re.findall(rf"^\s{{4}}def\s+{re.escape(method)}\s*\(", src, flags=re.MULTILINE)
    assert not matches, f"panel_v2.py todavía define '{method}'"
