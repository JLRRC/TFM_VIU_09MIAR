#!/usr/bin/env python3
"""F5 audit-v4: tests offline de release_objects_logic puro."""
from __future__ import annotations

from ur5_tools.release_objects_logic import (
    compute_missing_required,
    find_drop_anchor_duplicates,
    parse_world_name_from_sdf,
)


# ---- compute_missing_required -----------------------------------------------


def test_missing_required_all_present() -> None:
    out = compute_missing_required(
        pose_names=["drop_anchor", "box_red", "box_blue"],
        expected_names=["box_red", "box_blue"],
        anchor_name="drop_anchor",
    )
    assert out == []


def test_missing_required_anchor_missing() -> None:
    out = compute_missing_required(
        pose_names=["box_red"],
        expected_names=["box_red"],
        anchor_name="drop_anchor",
    )
    assert out == ["drop_anchor"]


def test_missing_required_object_missing() -> None:
    out = compute_missing_required(
        pose_names=["drop_anchor", "box_red"],
        expected_names=["box_red", "box_blue"],
        anchor_name="drop_anchor",
    )
    assert out == ["box_blue"]


def test_missing_required_preserves_order() -> None:
    out = compute_missing_required(
        pose_names=[],
        expected_names=["c", "a", "b"],
        anchor_name="anchor",
    )
    assert out == ["anchor", "c", "a", "b"]


def test_missing_required_empty_inputs() -> None:
    out = compute_missing_required(
        pose_names=[],
        expected_names=[],
        anchor_name="x",
    )
    assert out == ["x"]


# ---- find_drop_anchor_duplicates -------------------------------------------


def test_find_duplicates_none_when_clean() -> None:
    out = find_drop_anchor_duplicates(["drop_anchor", "box"], "drop_anchor")
    assert out == []


def test_find_duplicates_excludes_base_name() -> None:
    """drop_anchor (sin paréntesis) NO es duplicado."""
    out = find_drop_anchor_duplicates(["drop_anchor"], "drop_anchor")
    assert out == []


def test_find_duplicates_detects_indexed() -> None:
    out = find_drop_anchor_duplicates(
        ["drop_anchor", "drop_anchor(2)", "drop_anchor(3)"], "drop_anchor"
    )
    assert out == ["drop_anchor(2)", "drop_anchor(3)"]


def test_find_duplicates_with_other_anchor() -> None:
    out = find_drop_anchor_duplicates(
        ["release(1)", "drop_anchor(2)"], "release"
    )
    assert out == ["release(1)"]


# ---- parse_world_name_from_sdf ---------------------------------------------


def test_parse_world_name_basic() -> None:
    sdf = '<sdf><world name="ur5_mesa_objetos"></world></sdf>'
    assert parse_world_name_from_sdf(sdf) == "ur5_mesa_objetos"


def test_parse_world_name_root_world() -> None:
    sdf = '<world name="empty_world"></world>'
    assert parse_world_name_from_sdf(sdf) == "empty_world"


def test_parse_world_name_missing_attr() -> None:
    sdf = '<sdf><world></world></sdf>'
    assert parse_world_name_from_sdf(sdf) == ""


def test_parse_world_name_no_world_elem() -> None:
    sdf = '<sdf><other_thing/></sdf>'
    assert parse_world_name_from_sdf(sdf) == ""


def test_parse_world_name_invalid_xml() -> None:
    assert parse_world_name_from_sdf("<not valid>") == ""


def test_parse_world_name_empty() -> None:
    assert parse_world_name_from_sdf("") == ""
    assert parse_world_name_from_sdf("   \n  ") == ""
