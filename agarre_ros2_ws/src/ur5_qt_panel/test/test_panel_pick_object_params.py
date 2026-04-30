#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_pick_object_params.py
# Contenido: Tests del loader de parámetros runtime de pick_object (F2).
"""Tests unitarios para panel_pick_object_params."""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from ur5_qt_panel.panel_pick_object_params import (
    ENV_VAR_BY_FIELD,
    PickObjectParams,
    load_pick_object_params,
    reset_pick_object_params_cache,
)


@pytest.fixture(autouse=True)
def _isolate_env(monkeypatch):
    """Ningún env de la familia PANEL_PICK_OBJECT_ debe filtrarse al test."""
    for env_name in ENV_VAR_BY_FIELD.values():
        monkeypatch.delenv(env_name, raising=False)
    reset_pick_object_params_cache()
    yield
    reset_pick_object_params_cache()


def test_defaults_match_dataclass():
    p = load_pick_object_params(yaml_path=Path("/nonexistent.yaml"))
    base = PickObjectParams()
    assert p == base


def test_env_overrides_default(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_OBJECT_APPROACH_TOL_M", "0.05")
    p = load_pick_object_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.approach_tol_m == 0.05


def test_yaml_overrides_default(tmp_path):
    yml = tmp_path / "pick_object_runtime.yaml"
    yml.write_text(textwrap.dedent("""
        approach_tol_m: 0.07
        carry_gate_enable: false
    """).strip())
    p = load_pick_object_params(yaml_path=yml)
    assert p.approach_tol_m == 0.07
    assert p.carry_gate_enable is False


def test_env_beats_yaml(tmp_path, monkeypatch):
    yml = tmp_path / "pick_object_runtime.yaml"
    yml.write_text("approach_tol_m: 0.999\n")
    monkeypatch.setenv("PANEL_PICK_OBJECT_APPROACH_TOL_M", "0.01")
    p = load_pick_object_params(yaml_path=yml)
    assert p.approach_tol_m == 0.01


def test_invalid_value_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_OBJECT_APPROACH_TOL_M", "not-a-number")
    p = load_pick_object_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.approach_tol_m == 0.10


def test_dataclass_is_frozen():
    p = PickObjectParams()
    with pytest.raises(Exception):
        p.approach_tol_m = 0.999  # type: ignore[misc]


def test_env_var_mapping_covers_all_fields():
    """Cada campo del dataclass debe tener una env var asociada."""
    base = PickObjectParams()
    field_names = {f.name for f in base.__dataclass_fields__.values()}
    mapped = set(ENV_VAR_BY_FIELD.keys())
    assert not (field_names - mapped), f"campos sin env var: {field_names - mapped}"
    assert not (mapped - field_names), f"env vars sin campo: {mapped - field_names}"


def test_flag_coercion_truthy(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_OBJECT_FORCE_HOME_START", "1")
    p = load_pick_object_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.force_home_start is True


def test_flag_coercion_falsy(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_OBJECT_MOVEIT_EXCLUSIVE", "0")
    p = load_pick_object_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.moveit_exclusive is False


def test_int_coercion(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_OBJECT_GRASP_TF_STABLE_SAMPLES", "10")
    p = load_pick_object_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.grasp_tf_stable_samples == 10


def test_str_field(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_OBJECT_PREFLIGHT_MODE", "cesta")
    p = load_pick_object_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.preflight_mode == "cesta"
