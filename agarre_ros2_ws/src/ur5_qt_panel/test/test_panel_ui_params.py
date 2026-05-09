#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_ui_params.py
"""Tests para panel_ui_params (F2 bucket C)."""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from ur5_qt_panel.panel_ui_params import (
    ENV_VAR_BY_FIELD,
    PanelUiParams,
    load_panel_ui_params,
    reset_panel_ui_params_cache,
)


@pytest.fixture(autouse=True)
def _isolate_env(monkeypatch):
    for env_name in ENV_VAR_BY_FIELD.values():
        monkeypatch.delenv(env_name, raising=False)
    reset_panel_ui_params_cache()
    yield
    reset_panel_ui_params_cache()


def test_defaults_match_dataclass():
    p = load_panel_ui_params(yaml_path=Path("/nonexistent.yaml"))
    assert p == PanelUiParams()


def test_known_defaults():
    p = PanelUiParams()
    assert p.debug_exceptions is False
    assert p.tcp_pose_overlay is True
    assert p.required_ee_frame == "rg2_pinch_center"
    assert p.tf_drop_grace_sec == 4.0
    assert p.auto_run_pick_demo_attempts == 1
    # F2 final wave: debug extras
    assert p.debug_pick_obj is False
    assert p.direct_debug_root == ""
    assert p.strict_joint_identity is True


def test_env_flag_truthy(monkeypatch):
    monkeypatch.setenv("PANEL_DEBUG_EXCEPTIONS", "1")
    p = load_panel_ui_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.debug_exceptions is True


def test_env_flag_falsy(monkeypatch):
    monkeypatch.setenv("PANEL_TCP_POSE_OVERLAY", "0")
    p = load_panel_ui_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.tcp_pose_overlay is False


def test_env_str_override(monkeypatch):
    monkeypatch.setenv("PANEL_REQUIRED_EE_FRAME", "tool0")
    p = load_panel_ui_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.required_ee_frame == "tool0"


def test_env_float_override(monkeypatch):
    monkeypatch.setenv("PANEL_TF_DROP_GRACE_SEC", "1.5")
    p = load_panel_ui_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.tf_drop_grace_sec == 1.5


def test_yaml_overrides(tmp_path):
    yml = tmp_path / "panel_ui_runtime.yaml"
    yml.write_text(textwrap.dedent("""
        debug_exceptions: true
        required_ee_frame: tool0
        tf_drop_grace_sec: 7.5
    """).strip())
    p = load_panel_ui_params(yaml_path=yml)
    assert p.debug_exceptions is True
    assert p.required_ee_frame == "tool0"
    assert p.tf_drop_grace_sec == 7.5


def test_env_beats_yaml(tmp_path, monkeypatch):
    yml = tmp_path / "panel_ui_runtime.yaml"
    yml.write_text("debug_exceptions: true\n")
    monkeypatch.setenv("PANEL_DEBUG_EXCEPTIONS", "0")
    p = load_panel_ui_params(yaml_path=yml)
    assert p.debug_exceptions is False


def test_dataclass_is_frozen():
    p = PanelUiParams()
    with pytest.raises(Exception):
        p.debug_exceptions = True  # type: ignore[misc]


def test_env_var_mapping_covers_all_fields():
    base = PanelUiParams()
    field_names = {f.name for f in base.__dataclass_fields__.values()}
    mapped = set(ENV_VAR_BY_FIELD.keys())
    assert field_names == mapped, f"missing: {field_names - mapped}, extra: {mapped - field_names}"
