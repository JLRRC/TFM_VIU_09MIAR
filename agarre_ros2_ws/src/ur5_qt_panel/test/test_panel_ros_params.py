#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_ros_params.py
"""Tests para panel_ros_params (F2 bucket B)."""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from ur5_qt_panel.panel_ros_params import (
    ENV_VAR_BY_FIELD,
    PanelRosParams,
    load_panel_ros_params,
    reset_panel_ros_params_cache,
)


@pytest.fixture(autouse=True)
def _isolate_env(monkeypatch):
    for env_name in ENV_VAR_BY_FIELD.values():
        monkeypatch.delenv(env_name, raising=False)
    reset_panel_ros_params_cache()
    yield
    reset_panel_ros_params_cache()


def test_defaults():
    p = load_panel_ros_params(yaml_path=Path("/nonexistent.yaml"))
    base = PanelRosParams()
    assert p == base


def test_str_default_camera_connect():
    p = PanelRosParams()
    assert p.camera_connect_trigger_service == "/panel/camera_connect"
    assert p.tfm_execute_service_timeout_sec == 480.0
    assert p.ros_executor_threads == 3


def test_env_overrides_str(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_TRIGGER_SERVICE", "/foo/pd")
    p = load_panel_ros_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.pick_demo_trigger_service == "/foo/pd"


def test_env_overrides_float(monkeypatch):
    monkeypatch.setenv("PANEL_TFM_INFER_SERVICE_TIMEOUT_SEC", "12.5")
    p = load_panel_ros_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.tfm_infer_service_timeout_sec == 12.5


def test_env_overrides_int(monkeypatch):
    monkeypatch.setenv("PANEL_ROS_EXECUTOR_THREADS", "8")
    p = load_panel_ros_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.ros_executor_threads == 8


def test_yaml_overrides_default(tmp_path):
    yml = tmp_path / "panel_ros_runtime.yaml"
    yml.write_text(textwrap.dedent("""
        ros_executor_threads: 16
        pick_demo_service_timeout_sec: 240.0
    """).strip())
    p = load_panel_ros_params(yaml_path=yml)
    assert p.ros_executor_threads == 16
    assert p.pick_demo_service_timeout_sec == 240.0


def test_env_beats_yaml(tmp_path, monkeypatch):
    yml = tmp_path / "panel_ros_runtime.yaml"
    yml.write_text("ros_executor_threads: 99\n")
    monkeypatch.setenv("PANEL_ROS_EXECUTOR_THREADS", "5")
    p = load_panel_ros_params(yaml_path=yml)
    assert p.ros_executor_threads == 5


def test_invalid_int_falls_back(monkeypatch):
    monkeypatch.setenv("PANEL_ROS_EXECUTOR_THREADS", "not-int")
    p = load_panel_ros_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.ros_executor_threads == 3


def test_dataclass_is_frozen():
    p = PanelRosParams()
    with pytest.raises(Exception):
        p.ros_executor_threads = 999  # type: ignore[misc]


def test_env_var_mapping_covers_all_fields():
    base = PanelRosParams()
    field_names = {f.name for f in base.__dataclass_fields__.values()}
    mapped = set(ENV_VAR_BY_FIELD.keys())
    assert field_names == mapped, f"missing: {field_names - mapped}, extra: {mapped - field_names}"
