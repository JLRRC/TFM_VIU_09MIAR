"""F3-step5bis-a: tests smoke del carry_coherence extraído."""

from __future__ import annotations

import inspect
from unittest.mock import MagicMock

import pytest

from ur5_qt_panel.pick_object.carry_coherence import (
    CarryCoherenceContext,
    assert_carry_coherence_after_lift,
)


def test_carry_coherence_context_constructs():
    panel = MagicMock()
    ctx = CarryCoherenceContext(
        panel=panel,
        read_tcp_in_frame=MagicMock(),
        base_frame="base_link",
        world_frame="world",
        measured_ee_frame="rg2_pinch_center",
        obj_name="box_red",
        get_pick_object_params=MagicMock(),
        log_moveit_panel_trace=MagicMock(),
    )
    assert ctx.panel is panel
    assert ctx.obj_name == "box_red"


def test_assert_carry_coherence_after_lift_is_callable():
    assert callable(assert_carry_coherence_after_lift)


def test_assert_carry_coherence_signature_matches_legacy():
    sig = inspect.signature(assert_carry_coherence_after_lift)
    expected = [
        "ctx", "require_state_override", "timeout_override",
        "max_dist_override", "min_consecutive_override", "gate_label",
    ]
    assert list(sig.parameters.keys()) == expected
