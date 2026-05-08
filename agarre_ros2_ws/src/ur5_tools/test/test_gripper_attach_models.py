#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_gripper_attach_models.py
"""F#13-step (2026-05-08) — Tests offline para gripper_attach_models."""

from __future__ import annotations

import pytest

from ur5_tools.gripper_attach_models import (
    AttachedTarget,
    DemoTransportState,
    PoseSample,
    coherence_breach_exceeded,
    increment_breach,
    reset_breach,
)


# ---------------------------------------------------------------------------
# PoseSample
# ---------------------------------------------------------------------------


def test_pose_sample_construction():
    p = PoseSample(
        x=0.5, y=0.1, z=0.3,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        stamp_ns=1234567890,
    )
    assert p.x == 0.5
    assert p.qw == 1.0
    assert p.stamp_ns == 1234567890


# ---------------------------------------------------------------------------
# AttachedTarget
# ---------------------------------------------------------------------------


def test_attached_target_default_breach_count_zero():
    target = AttachedTarget(
        name="pick_demo",
        offset_x=0.0, offset_y=0.0, offset_z=0.05,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        attach_stamp_ns=1000,
    )
    assert target.coherence_breach_count == 0


def test_attached_target_with_explicit_breach():
    target = AttachedTarget(
        name="box_red",
        offset_x=0.0, offset_y=0.0, offset_z=0.05,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        attach_stamp_ns=1000,
        coherence_breach_count=3,
    )
    assert target.coherence_breach_count == 3


# ---------------------------------------------------------------------------
# DemoTransportState
# ---------------------------------------------------------------------------


def test_demo_transport_state_defaults():
    s = DemoTransportState(name="pick_demo")
    assert s.last_pose is None
    assert s.last_spawn_ts == 0.0
    assert s.world_offset_z == pytest.approx(-0.1)
    assert s.use_world_locked_pose is True


def test_demo_transport_state_with_pose():
    pose = PoseSample(0.5, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0, 100)
    s = DemoTransportState(
        name="box_red",
        last_pose=pose,
        last_spawn_ts=10.5,
        use_world_locked_pose=False,
    )
    assert s.last_pose is pose
    assert s.last_spawn_ts == pytest.approx(10.5)
    assert s.use_world_locked_pose is False


# ---------------------------------------------------------------------------
# coherence_breach_exceeded
# ---------------------------------------------------------------------------


def _make_target(breach_count: int = 0) -> AttachedTarget:
    return AttachedTarget(
        name="pick_demo",
        offset_x=0.0, offset_y=0.0, offset_z=0.05,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        attach_stamp_ns=1000,
        coherence_breach_count=breach_count,
    )


def test_breach_exceeded_zero_below_threshold():
    target = _make_target(breach_count=0)
    assert coherence_breach_exceeded(target, breach_threshold=3) is False


def test_breach_exceeded_at_threshold():
    target = _make_target(breach_count=3)
    assert coherence_breach_exceeded(target, breach_threshold=3) is True


def test_breach_exceeded_above_threshold():
    target = _make_target(breach_count=10)
    assert coherence_breach_exceeded(target, breach_threshold=3) is True


def test_breach_threshold_zero_clamps_to_one():
    """threshold=0 se trata como 1 (cualquier breach dispara)."""
    target = _make_target(breach_count=1)
    assert coherence_breach_exceeded(target, breach_threshold=0) is True
    target2 = _make_target(breach_count=0)
    assert coherence_breach_exceeded(target2, breach_threshold=0) is False


# ---------------------------------------------------------------------------
# increment_breach / reset_breach (immutable)
# ---------------------------------------------------------------------------


def test_increment_breach_returns_new_target():
    target = _make_target(breach_count=0)
    incremented = increment_breach(target)
    assert incremented is not target
    assert target.coherence_breach_count == 0  # original no se mutó
    assert incremented.coherence_breach_count == 1


def test_increment_breach_preserves_other_fields():
    target = _make_target(breach_count=5)
    incremented = increment_breach(target)
    assert incremented.name == target.name
    assert incremented.offset_z == target.offset_z
    assert incremented.attach_stamp_ns == target.attach_stamp_ns
    assert incremented.coherence_breach_count == 6


def test_reset_breach_returns_zero():
    target = _make_target(breach_count=10)
    cleared = reset_breach(target)
    assert cleared is not target
    assert cleared.coherence_breach_count == 0
    # Original sigue inalterado
    assert target.coherence_breach_count == 10


def test_breach_workflow():
    """Simula 3 breaches consecutivos + reset."""
    target = _make_target(breach_count=0)
    target = increment_breach(target)
    target = increment_breach(target)
    target = increment_breach(target)
    assert target.coherence_breach_count == 3
    assert coherence_breach_exceeded(target, breach_threshold=3) is True
    target = reset_breach(target)
    assert target.coherence_breach_count == 0
    assert coherence_breach_exceeded(target, breach_threshold=3) is False
