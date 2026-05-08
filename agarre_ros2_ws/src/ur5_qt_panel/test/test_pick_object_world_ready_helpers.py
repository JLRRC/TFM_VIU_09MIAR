#!/usr/bin/env python3
"""V1.1 audit-v4: tests offline para pick_object.world_ready_helpers."""
from __future__ import annotations

import pytest

from ur5_qt_panel.pick_object.world_ready_helpers import (
    check_object_on_table,
    compute_default_tracked_names,
    derive_tracked_names_target_scope,
    normalize_world_ready_scope,
)


# ---- normalize_world_ready_scope -------------------------------------------


@pytest.mark.parametrize("raw", ["all", "strict"])
def test_normalize_scope_all(raw):
    assert normalize_world_ready_scope(raw) == "all"


@pytest.mark.parametrize("raw", ["target", "single", "", None, 42, "ALL"])
def test_normalize_scope_default_target(raw):
    """Cualquier otra cosa → "target" (case-sensitive: ALL no matchea)."""
    assert normalize_world_ready_scope(raw) == "target"


# ---- compute_default_tracked_names -----------------------------------------


def test_default_tracked_names_dedup_and_sort():
    out = compute_default_tracked_names(
        drop_names={"box", "drop_anchor"},
        pick_demo_names={"pick_demo", "box"},  # box duplicado
    )
    assert out == ["box", "drop_anchor", "pick_demo"]


def test_default_tracked_names_empty():
    assert compute_default_tracked_names([], []) == []


def test_default_tracked_names_only_drop():
    out = compute_default_tracked_names({"a", "b"}, [])
    assert out == ["a", "b"]


# ---- derive_tracked_names_target_scope -------------------------------------


def test_target_scope_first_valid_returned():
    out = derive_tracked_names_target_scope(
        ["box_red", "box_blue"], fallback_name="pick_demo"
    )
    assert out == ["box_red", "box_blue"]


def test_target_scope_skips_empty_and_none():
    out = derive_tracked_names_target_scope(
        ["", None, "  ", "valid"], fallback_name="pick_demo"
    )
    assert out == ["valid"]


def test_target_scope_dedupes_preserving_order():
    out = derive_tracked_names_target_scope(
        ["a", "b", "a", "c"], fallback_name="pd"
    )
    assert out == ["a", "b", "c"]


def test_target_scope_strips_whitespace():
    out = derive_tracked_names_target_scope(
        ["  spaced  "], fallback_name="pd"
    )
    assert out == ["spaced"]


def test_target_scope_fallback_when_empty():
    assert derive_tracked_names_target_scope([]) == ["pick_demo"]


def test_target_scope_fallback_when_all_invalid():
    out = derive_tracked_names_target_scope(["", None, "  "])
    assert out == ["pick_demo"]


def test_target_scope_custom_fallback():
    out = derive_tracked_names_target_scope([], fallback_name="custom")
    assert out == ["custom"]


# ---- check_object_on_table -------------------------------------------------


@pytest.mark.parametrize("z,expected", [
    (0.05, True),    # in range
    (0.0, True),     # min boundary
    (0.10, True),    # max boundary
    (-0.001, False), # below
    (0.101, False),  # above
])
def test_check_object_on_table_z_only(z, expected):
    out = check_object_on_table((1.0, 2.0, z), table_z_min=0.0, table_z_max=0.10)
    assert out is expected


def test_check_object_on_table_inverse_range():
    """Si min > max no hay ningún z válido."""
    assert check_object_on_table((0, 0, 0.05), table_z_min=0.10, table_z_max=0.05) is False
