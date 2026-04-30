#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_moveit_bridge_params.py
"""Tests para moveit_bridge.params (F2 bucket D)."""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from ur5_tools.moveit_bridge.params import (
    ENV_VAR_BY_FIELD,
    MoveItBridgeParams,
    load_moveit_bridge_params,
    reset_moveit_bridge_params_cache,
)


@pytest.fixture(autouse=True)
def _isolate_env(monkeypatch):
    for env_name in ENV_VAR_BY_FIELD.values():
        monkeypatch.delenv(env_name, raising=False)
    reset_moveit_bridge_params_cache()
    yield
    reset_moveit_bridge_params_cache()


def test_defaults():
    p = load_moveit_bridge_params(yaml_path=Path("/nonexistent.yaml"))
    assert p == MoveItBridgeParams()


def test_known_defaults():
    p = MoveItBridgeParams()
    assert p.allow_feedback_early_success is False
    assert p.approach_internal_replan is True
    assert p.approach_relaxed_constraint_retry is True
    assert p.request_timeout_sec == 60.0
    # F2 final wave: motion gating extensions
    assert p.tf_gate_timeout_sec == 1.2
    assert p.startup_timeout_sec == 40.0
    assert p.wait_joint_target_max_age_sec == 0.35
    assert p.wait_joint_target_max_vel_rad_s == 0.05
    assert p.wait_joint_target_stable_samples == 3
    assert p.disable_joint_wrap_align is False


def test_env_flag_truthy(monkeypatch):
    monkeypatch.setenv("PANEL_MOVEIT_BRIDGE_ALLOW_FEEDBACK_EARLY_SUCCESS", "1")
    p = load_moveit_bridge_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.allow_feedback_early_success is True


def test_env_flag_falsy(monkeypatch):
    monkeypatch.setenv("PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN", "0")
    p = load_moveit_bridge_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.approach_internal_replan is False


def test_env_float(monkeypatch):
    monkeypatch.setenv("PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC", "30.0")
    p = load_moveit_bridge_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.request_timeout_sec == 30.0


def test_yaml_overrides(tmp_path):
    yml = tmp_path / "moveit_bridge_runtime.yaml"
    yml.write_text(textwrap.dedent("""
        allow_ee_early_success: true
        request_timeout_sec: 120.0
    """).strip())
    p = load_moveit_bridge_params(yaml_path=yml)
    assert p.allow_ee_early_success is True
    assert p.request_timeout_sec == 120.0


def test_env_beats_yaml(tmp_path, monkeypatch):
    yml = tmp_path / "moveit_bridge_runtime.yaml"
    yml.write_text("approach_internal_replan: false\n")
    monkeypatch.setenv("PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN", "1")
    p = load_moveit_bridge_params(yaml_path=yml)
    assert p.approach_internal_replan is True


def test_dataclass_is_frozen():
    p = MoveItBridgeParams()
    with pytest.raises(Exception):
        p.request_timeout_sec = 999  # type: ignore[misc]


def test_env_var_mapping_covers_all_fields():
    base = MoveItBridgeParams()
    field_names = {f.name for f in base.__dataclass_fields__.values()}
    mapped = set(ENV_VAR_BY_FIELD.keys())
    assert field_names == mapped, f"missing: {field_names - mapped}, extra: {mapped - field_names}"
