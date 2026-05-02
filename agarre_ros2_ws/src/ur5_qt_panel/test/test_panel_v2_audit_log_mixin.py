#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_audit_log_mixin.py
# Contenido: F14-step11 (2026-05-01) — verifica el contrato del mixin audit/log/ready.
"""Tests offline F14-step11 — PanelV2AuditLogMixin (29 wrappers)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

SAMPLE_METHODS = (
    "_moveit_control_status", "_manual_control_status",
    "_external_publishers_for_topic", "_bridge_publishers_only",
    "_emit_log", "_metric_mark", "_audit_root", "_audit_append",
    "_audit_write_json", "_sha256_file", "_should_emit_log",
    "_set_motion_lock", "_set_btn_state", "_set_launching_style",
    "_require_ready_basic", "_basic_ready_status",
    "_pick_demo_remote_ready_status", "_log", "_emit_log_throttled",
    "_block_if_managed",
)


def _read(name: str) -> str:
    return (_SRC_DIR / name).read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_audit_log_mixin.py")
    assert "class PanelV2AuditLogMixin" in src


def test_mixin_imports_panel_helpers():
    src = _read("panel_v2_audit_log_mixin.py")
    assert "import panel_helpers as _ph" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_audit_log_mixin.py")
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
            assert "PanelV2AuditLogMixin" in base_names
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2AuditLogMixin" in src
    assert "panel_v2_audit_log_mixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    matches = re.findall(
        rf"^\s{{4}}def\s+{re.escape(method)}\s*\(", src, flags=re.MULTILINE
    )
    assert not matches, f"panel_v2.py todavía define '{method}'"


def test_panel_v2_under_1800_lines():
    src = _read("panel_v2.py")
    line_count = src.count("\n") + (0 if src.endswith("\n") else 1)
    assert line_count <= 1800, (
        f"panel_v2.py creció a {line_count} LOC; umbral F14-step11 <=1800"
    )
