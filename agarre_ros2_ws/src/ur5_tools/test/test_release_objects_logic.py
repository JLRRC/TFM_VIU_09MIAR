#!/usr/bin/env python3
"""F5 audit-v4: tests offline de release_objects_logic puro."""
from __future__ import annotations

from ur5_tools.release_objects_logic import (
    compute_missing_required,
    drop_anchor_cleanup_targets,
    find_drop_anchor_duplicates,
    parse_world_name_from_sdf,
    pick_gz_service,
)


# ---- drop_anchor_cleanup_targets (F8 audit 2026-05-10) -----------------------


def test_cleanup_targets_with_primary() -> None:
    out = drop_anchor_cleanup_targets("drop_anchor", include_primary=True)
    assert len(out) == 9  # 3 model_names * 3 entries
    assert ("drop_anchor", "MODEL") in out
    assert ("drop_anchor::link", "LINK") in out
    assert ("drop_anchor/link", "LINK") in out
    assert ("drop_anchor(1)", "MODEL") in out
    assert ("drop_anchor(2)", "MODEL") in out


def test_cleanup_targets_without_primary() -> None:
    out = drop_anchor_cleanup_targets("drop_anchor", include_primary=False)
    assert len(out) == 6  # 2 model_names (1)+(2) * 3 entries
    assert ("drop_anchor", "MODEL") not in out
    assert ("drop_anchor(1)", "MODEL") in out


def test_cleanup_targets_custom_anchor() -> None:
    out = drop_anchor_cleanup_targets("foo_anchor", include_primary=True)
    assert ("foo_anchor", "MODEL") in out
    assert ("foo_anchor(2)::link", "LINK") in out


# ---- pick_gz_service (F8 audit 2026-05-10) -----------------------------------


def test_pick_gz_service_exact_suffix_match() -> None:
    services = [
        "/world/empty/clock",
        "/world/myworld/create/blocking",
        "/world/myworld/spawn",
    ]
    out = pick_gz_service(services, "myworld", ("create/blocking", "spawn"))
    assert out == "/world/myworld/create/blocking"


def test_pick_gz_service_falls_back_to_no_slash_suffix() -> None:
    services = ["/world/myworld/foo_create"]
    out = pick_gz_service(services, "myworld", ("create",))
    assert out == "/world/myworld/foo_create"


def test_pick_gz_service_returns_none_if_no_scoped() -> None:
    services = ["/world/otra/spawn"]
    out = pick_gz_service(services, "myworld", ("spawn",))
    assert out is None


def test_pick_gz_service_empty_services() -> None:
    assert pick_gz_service([], "myworld", ("spawn",)) is None


def test_pick_gz_service_respects_suffix_priority() -> None:
    services = [
        "/world/myworld/spawn",
        "/world/myworld/create/blocking",
    ]
    out = pick_gz_service(services, "myworld", ("create/blocking", "spawn"))
    assert out == "/world/myworld/create/blocking"


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
