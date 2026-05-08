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
