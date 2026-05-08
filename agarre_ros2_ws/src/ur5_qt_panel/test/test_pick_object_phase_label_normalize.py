#!/usr/bin/env python3
"""Tests offline para pick_object.phase_label_normalize (F3-iter3)."""
from __future__ import annotations

import pytest

from ur5_qt_panel.pick_object.phase_label_normalize import normalize_phase_label


def test_empty_returns_empty() -> None:
    assert normalize_phase_label("") == ""
    assert normalize_phase_label(None) == ""
    assert normalize_phase_label("   ") == ""


def test_upper_strip() -> None:
    assert normalize_phase_label("  pre_grasp  ") == "PRE_GRASP"
    assert normalize_phase_label("approach") == "APPROACH"


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("GRASP_DOWN_MICRO_0", "GRASP_DOWN"),
        ("GRASP_DOWN_MICRO_1", "GRASP_DOWN"),
        ("GRASP_DOWN_MICRO_42", "GRASP_DOWN"),
        ("grasp_down_micro_3", "GRASP_DOWN"),  # case-insensitive via upper
    ],
)
def test_grasp_down_micro_collapses(raw: str, expected: str) -> None:
    assert normalize_phase_label(raw) == expected


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("STRICT_LIFT_STAGE_0", "LIFT"),
        ("STRICT_LIFT_STAGE_1", "LIFT"),
        ("strict_lift_stage_2", "LIFT"),
    ],
)
def test_strict_lift_stage_collapses(raw: str, expected: str) -> None:
    assert normalize_phase_label(raw) == expected


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("TRANSPORT_STAGE_0", "TRANSPORT"),
        ("TRANSPORT_STAGE_3", "TRANSPORT"),
        ("transport_stage_4", "TRANSPORT"),
    ],
)
def test_transport_stage_collapses(raw: str, expected: str) -> None:
    assert normalize_phase_label(raw) == expected


def test_pre_grasp_recenter_aliases_pre_grasp() -> None:
    assert normalize_phase_label("PRE_GRASP_RECENTER") == "PRE_GRASP"
    assert normalize_phase_label("pre_grasp_recenter") == "PRE_GRASP"


@pytest.mark.parametrize(
    "raw",
    ["APPROACH", "PRE_GRASP", "GRASP_DOWN", "LIFT", "TRANSPORT", "RELEASE", "DONE"],
)
def test_canonical_labels_passthrough(raw: str) -> None:
    assert normalize_phase_label(raw) == raw


def test_unknown_label_passthrough() -> None:
    assert normalize_phase_label("CUSTOM_PHASE") == "CUSTOM_PHASE"
    assert normalize_phase_label("foobar") == "FOOBAR"


def test_grasp_down_exact_not_collapsed_by_micro() -> None:
    """GRASP_DOWN literal no debe colapsar como GRASP_DOWN_MICRO_*."""
    assert normalize_phase_label("GRASP_DOWN") == "GRASP_DOWN"


def test_lift_exact_not_collapsed_by_strict() -> None:
    assert normalize_phase_label("LIFT") == "LIFT"


def test_transport_exact_not_collapsed_by_stage() -> None:
    assert normalize_phase_label("TRANSPORT") == "TRANSPORT"
