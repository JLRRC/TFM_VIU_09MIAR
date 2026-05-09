#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_tfm_params.py
"""Tests para panel_tfm_params (F2 bucket E)."""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from ur5_qt_panel.panel_tfm_params import (
    ENV_VAR_BY_FIELD,
    PanelTfmParams,
    load_panel_tfm_params,
    reset_panel_tfm_params_cache,
)


@pytest.fixture(autouse=True)
def _isolate_env(monkeypatch):
    for env_name in ENV_VAR_BY_FIELD.values():
        monkeypatch.delenv(env_name, raising=False)
    reset_panel_tfm_params_cache()
    yield
    reset_panel_tfm_params_cache()


def test_defaults():
    p = load_panel_tfm_params(yaml_path=Path("/nonexistent.yaml"))
    assert p == PanelTfmParams()


def test_known_defaults():
    p = PanelTfmParams()
    assert p.execute_pretable is True
    assert p.grasp_cartesian is False
    assert p.infer_use_roi == "auto"
    assert p.canonical_grasp_xy_max_delta_m == 0.080
    assert p.remote_execute_ready_wait_sec == 90.0


def test_env_flag_truthy(monkeypatch):
    monkeypatch.setenv("PANEL_TFM_GRASP_CARTESIAN", "1")
    p = load_panel_tfm_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.grasp_cartesian is True


def test_env_float(monkeypatch):
    monkeypatch.setenv("PANEL_TFM_INFER_FRAME_MAX_AGE_SEC", "2.5")
    p = load_panel_tfm_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.infer_frame_max_age_sec == 2.5


def test_env_str(monkeypatch):
    monkeypatch.setenv("PANEL_TFM_INFER_USE_ROI", "off")
    p = load_panel_tfm_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.infer_use_roi == "off"


def test_yaml_overrides(tmp_path):
    yml = tmp_path / "panel_tfm_runtime.yaml"
    yml.write_text(textwrap.dedent("""
        execute_pretable: false
        canonical_grasp_xy_max_delta_m: 0.05
    """).strip())
    p = load_panel_tfm_params(yaml_path=yml)
    assert p.execute_pretable is False
    assert p.canonical_grasp_xy_max_delta_m == 0.05


def test_env_beats_yaml(tmp_path, monkeypatch):
    yml = tmp_path / "panel_tfm_runtime.yaml"
    yml.write_text("execute_pretable: false\n")
    monkeypatch.setenv("PANEL_TFM_EXECUTE_PRETABLE", "1")
    p = load_panel_tfm_params(yaml_path=yml)
    assert p.execute_pretable is True


def test_dataclass_is_frozen():
    p = PanelTfmParams()
    with pytest.raises(Exception):
        p.execute_pretable = False  # type: ignore[misc]


def test_env_var_mapping_covers_all_fields():
    base = PanelTfmParams()
    field_names = {f.name for f in base.__dataclass_fields__.values()}
    mapped = set(ENV_VAR_BY_FIELD.keys())
    assert field_names == mapped, f"missing: {field_names - mapped}, extra: {mapped - field_names}"
