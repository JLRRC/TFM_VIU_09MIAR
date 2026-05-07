"""F3-step5bis-b: tests smoke del strict_pre_lift_contact extraído."""

from __future__ import annotations

import inspect
from unittest.mock import MagicMock


from ur5_qt_panel.pick_object.strict_pre_lift_contact import (
    StrictPreLiftContactContext,
    StrictPreLiftContactState,
    ensure_strict_pre_lift_contact,
)


def test_strict_pre_lift_state_default_is_none():
    state = StrictPreLiftContactState()
    assert state.last_metrics is None


def test_strict_pre_lift_state_is_mutable():
    state = StrictPreLiftContactState()
    state.last_metrics = {"opening_m": 0.005}
    assert state.last_metrics["opening_m"] == 0.005


def test_strict_pre_lift_context_constructs():
    panel = MagicMock()
    ctx = StrictPreLiftContactContext(
        panel=panel,
        close_gripper_sync=MagicMock(),
        ensure_gripper_open_for_moveit=MagicMock(),
        read_gripper_contact_metrics=MagicMock(return_value={}),
        run_moveit_step=MagicMock(),
        strict_physics_mode=True,
        table_top_base=(0.0, 0.0, 0.05),
        get_pick_object_params=MagicMock(),
    )
    assert ctx.panel is panel
    assert ctx.strict_physics_mode is True


def test_ensure_strict_pre_lift_is_callable_and_signature_matches():
    assert callable(ensure_strict_pre_lift_contact)
    sig = inspect.signature(ensure_strict_pre_lift_contact)
    assert list(sig.parameters.keys()) == [
        "ctx", "state", "grasp_pose_live", "grasp_delay_live",
    ]
