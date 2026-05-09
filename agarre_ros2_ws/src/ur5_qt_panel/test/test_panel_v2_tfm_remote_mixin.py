#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_tfm_remote_mixin.py
# Contenido: F14-step12 (2026-05-01) — verifica el contrato del mixin TFM+remote.
"""Tests offline F14-step12 — PanelV2TfmRemoteMixin (27 wrappers)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

TFM_METHODS = (
    "_tfm_infer_grasp", "_tfm_canonical_use_pick_object",
    "_complete_pending_tfm_infer_request",
    "_complete_pending_tfm_execute_request",
    "_complete_pending_pick_demo_request",
    "_build_tfm_pick_object_override",
    "_tfm_canonical_state_reset", "_tfm_canonical_phase_update",
    "_tfm_canonical_finish", "_restore_infer_selection_snapshot",
    "_latest_camera_frame_snapshot", "_ensure_selected_object_in_store",
    "_handle_infer_result", "_sync_tfm_module_grasp_state",
    "_tfm_visualize_grasp", "_wait_tfm_moveit_result",
    "_execute_tfm_world_grasp", "_on_tfm_grasp_object_clicked",
    "_tfm_publish_grasp",
)

REMOTE_METHODS = (
    "_on_remote_camera_connect_request",
    "_on_remote_camera_disconnect_request",
    "_on_remote_recover_request", "_on_remote_tfm_infer_request",
    "_on_remote_tfm_execute_request", "_on_remote_pick_demo_request",
    "_on_remote_pick_object_request",
    "_on_remote_object_select_request",
)

LEGACY_NO_PROPAGATE = (
    "_tfm_infer_grasp", "_tfm_visualize_grasp",
    "_on_tfm_grasp_object_clicked",
)


def _read(name: str) -> str:
    return (_SRC_DIR / name).read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_tfm_remote_mixin.py")
    assert "class PanelV2TfmRemoteMixin" in src


def test_mixin_imports():
    src = _read("panel_v2_tfm_remote_mixin.py")
    assert "from .panel_tfm import" in src
    assert "import panel_remote_callbacks as _rc" in src


@pytest.mark.parametrize("method", TFM_METHODS + REMOTE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_tfm_remote_mixin.py")
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
            assert "PanelV2TfmRemoteMixin" in base_names
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2TfmRemoteMixin" in src
    assert "panel_v2_tfm_remote_mixin" in src


@pytest.mark.parametrize("method", TFM_METHODS + REMOTE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    matches = re.findall(
        rf"^\s{{4}}def\s+{re.escape(method)}\s*\(", src, flags=re.MULTILINE
    )
    assert not matches, f"panel_v2.py todavía define '{method}'"


@pytest.mark.parametrize("method", LEGACY_NO_PROPAGATE)
def test_legacy_signature_preserved(method):
    """3 firmas legacy: NO propagan *args/**kwargs."""
    src = _read("panel_v2_tfm_remote_mixin.py")
    pat = rf"return\s+{re.escape(method[1:])}\(self\)\s*$"
    assert re.search(pat, src, flags=re.MULTILINE), (
        f"{method} debe llamar al backend con (self) sin propagar"
    )


def test_remote_callbacks_preserve_typed_signature():
    """Remote callbacks preservan los tipos en la firma."""
    src = _read("panel_v2_tfm_remote_mixin.py")
    for method in REMOTE_METHODS:
        if method == "_on_remote_object_select_request":
            pat = rf"def\s+{re.escape(method)}\(self, name: str, source: str\)"
        else:
            pat = rf"def\s+{re.escape(method)}\(self, source: str\)"
        assert re.search(pat, src), f"{method} debe preservar firma typed"


def test_panel_v2_under_1700_lines():
    src = _read("panel_v2.py")
    line_count = src.count("\n") + (0 if src.endswith("\n") else 1)
    assert line_count <= 1700, (
        f"panel_v2.py creció a {line_count} LOC; umbral F14-step12 <=1700"
    )
