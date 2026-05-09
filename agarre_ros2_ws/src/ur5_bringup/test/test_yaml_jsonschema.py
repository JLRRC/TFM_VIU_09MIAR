#!/usr/bin/env python3
"""F2 audit-v4 (2026-05-08): jsonschema validation for YAML configs.

Validates that ``runtime_defaults.yaml`` matches its JSON schema. New
keys must be added to the schema; type errors fail the test.

Skip if ``jsonschema`` not installed (offline-only safety).
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

WS_DIR = Path(__file__).resolve().parents[3]


def _load_yaml(rel: str) -> dict:
    path = WS_DIR / rel
    assert path.is_file(), f"YAML missing: {path}"
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return data


def _load_schema(rel: str) -> dict:
    path = WS_DIR / rel
    assert path.is_file(), f"Schema missing: {path}"
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _require_jsonschema():
    try:
        import jsonschema  # noqa: F401
    except ImportError:
        pytest.skip("jsonschema not installed (offline-only safety)")


def test_runtime_defaults_yaml_valid_against_schema() -> None:
    _require_jsonschema()
    import jsonschema
    data = _load_yaml("src/ur5_bringup/config/runtime_defaults.yaml")
    schema = _load_schema("src/ur5_bringup/schemas/runtime_defaults.schema.json")
    try:
        jsonschema.validate(data, schema)
    except jsonschema.ValidationError as exc:
        pytest.fail(f"runtime_defaults.yaml failed schema validation: {exc.message}")


def test_runtime_defaults_yaml_no_undocumented_keys() -> None:
    """All YAML keys must be declared in schema (additionalProperties=false)."""
    _require_jsonschema()
    import jsonschema
    data = _load_yaml("src/ur5_bringup/config/runtime_defaults.yaml")
    schema = _load_schema("src/ur5_bringup/schemas/runtime_defaults.schema.json")
    declared = set(schema.get("properties", {}).keys())
    actual = set(data.keys())
    undocumented = actual - declared
    assert not undocumented, (
        f"YAML keys missing from schema (add them or use additionalProperties=true):"
        f"\n  {sorted(undocumented)}"
    )
    # Sanity: validate with strict additionalProperties.
    jsonschema.validate(data, schema)


# Audit-v4.1 F2 (2026-05-09): nuevos schemas para system_state_manager y
# panel_settings. Patrón espejo del de runtime_defaults.

def test_system_state_manager_yaml_valid_against_schema() -> None:
    _require_jsonschema()
    import jsonschema
    data = _load_yaml("src/ur5_bringup/config/system_state_manager.yaml")
    schema = _load_schema("src/ur5_bringup/schemas/system_state_manager.schema.json")
    try:
        jsonschema.validate(data, schema)
    except jsonschema.ValidationError as exc:
        pytest.fail(
            f"system_state_manager.yaml falló schema validation: {exc.message}"
        )


def test_system_state_manager_yaml_no_undocumented_keys() -> None:
    """Sanity: las keys de ros__parameters están declaradas en el schema."""
    _require_jsonschema()
    data = _load_yaml("src/ur5_bringup/config/system_state_manager.yaml")
    schema = _load_schema("src/ur5_bringup/schemas/system_state_manager.schema.json")
    declared = set(
        schema["properties"]["system_state_manager"]["properties"][
            "ros__parameters"
        ]["properties"].keys()
    )
    actual = set(data["system_state_manager"]["ros__parameters"].keys())
    undocumented = actual - declared
    assert not undocumented, (
        f"system_state_manager YAML keys no declaradas en schema: "
        f"{sorted(undocumented)}"
    )


def test_panel_settings_yaml_valid_against_schema() -> None:
    _require_jsonschema()
    import jsonschema
    data = _load_yaml("src/ur5_qt_panel/config/panel_settings.yaml")
    schema = _load_schema("src/ur5_qt_panel/schemas/panel_settings.schema.json")
    try:
        jsonschema.validate(data, schema)
    except jsonschema.ValidationError as exc:
        pytest.fail(
            f"panel_settings.yaml falló schema validation: {exc.message}"
        )


def test_panel_settings_yaml_no_undocumented_keys() -> None:
    """Sanity: campos de panel_settings declarados en el schema."""
    _require_jsonschema()
    data = _load_yaml("src/ur5_qt_panel/config/panel_settings.yaml")
    schema = _load_schema("src/ur5_qt_panel/schemas/panel_settings.schema.json")
    declared = set(
        schema["properties"]["panel_settings"]["properties"].keys()
    )
    actual = set(data["panel_settings"].keys())
    undocumented = actual - declared
    assert not undocumented, (
        f"panel_settings YAML keys no declaradas en schema: "
        f"{sorted(undocumented)}"
    )
