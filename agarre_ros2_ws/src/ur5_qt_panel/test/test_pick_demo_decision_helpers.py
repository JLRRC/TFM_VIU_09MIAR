#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_demo_decision_helpers.py
"""F3-step40 (2026-05-08) — Tests offline para pick_demo.decision_helpers."""

from __future__ import annotations

import pytest

from ur5_qt_panel.pick_demo.decision_helpers import execution_type_from_decision


@pytest.mark.parametrize(
    "decision, expected",
    [
        ("fallback_joint_preset", "preset"),
        ("FALLBACK_JOINT_PRESET", "preset"),
        ("target_unavailable", "preset"),
        ("preset_with_fallback_joint_preset_inside", "preset"),
        ("direct_ik_move", "geometrico"),
        ("DIRECT_IK_MOVE", "geometrico"),
        ("direct_ik_move_refresh", "geometrico"),
        ("hybrid_strategy_x", "hibrido"),
        ("anything_else", "hibrido"),
        ("", "hibrido"),
        (None, "hibrido"),
        ("   ", "hibrido"),
    ],
)
def test_execution_type_from_decision(decision, expected):
    assert execution_type_from_decision(decision) == expected
