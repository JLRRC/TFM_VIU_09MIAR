#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_initial_snapshot.py
# Contenido: B-iter5 (2026-05-03) — tests del helper INITIAL_SNAPSHOT real.
"""Tests para tfm_orchestrator.initial_snapshot.

100% offline: mocks de TF lookup, JointState msg, resolver service.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from tfm_orchestrator.initial_snapshot import (
    InitialSnapshotResult,
    capture_initial_snapshot,
    capture_tcp_pose_base,
    extract_joint_positions,
)


# ---------------------------------------------------------------------------
# Helpers de mock
# ---------------------------------------------------------------------------


def _make_transform_stamped(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """Mock mínimo de TransformStamped con .transform.{translation,rotation}."""
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z),
            rotation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
        ),
    )


def _make_joint_state(
    names=("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"),
    positions=(0.0, -1.57, 0.0, -1.57, 0.0, 0.0),
):
    return SimpleNamespace(name=list(names), position=list(positions))


def _make_resolve_response(success=True, x=0.5, y=0.0, z=0.5, qw=1.0, detail=""):
    return SimpleNamespace(
        success=success,
        detail=detail,
        pose_world=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=qw),
        ),
    )


# ---------------------------------------------------------------------------
# capture_tcp_pose_base
# ---------------------------------------------------------------------------


def test_capture_tcp_pose_returns_tuple7_on_success():
    def lookup(target, source, time_=None, timeout=None):
        assert target == "base_link"
        assert source == "rg2_tcp"
        return _make_transform_stamped(0.1, 0.2, 0.3, qx=0.0, qy=0.0, qz=0.0, qw=1.0)

    pose, reason = capture_tcp_pose_base(lookup)
    assert pose == (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
    assert reason == "ok"


def test_capture_tcp_pose_returns_none_on_lookup_exception():
    def lookup(*a, **kw):
        raise RuntimeError("tf_extrapolation")

    pose, reason = capture_tcp_pose_base(lookup)
    assert pose is None
    assert "tf_lookup_exception" in reason
    assert "tf_extrapolation" in reason


def test_capture_tcp_pose_returns_none_on_unparseable_transform():
    def lookup(*a, **kw):
        return SimpleNamespace()  # sin .transform

    pose, reason = capture_tcp_pose_base(lookup)
    assert pose is None
    assert reason == "tf_transform_unparseable"


# ---------------------------------------------------------------------------
# extract_joint_positions
# ---------------------------------------------------------------------------


def test_extract_joints_returns_tuple6_in_canonical_order():
    msg = _make_joint_state(positions=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    joints, reason = extract_joint_positions(msg)
    assert joints == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert reason == "ok"


def test_extract_joints_reorders_when_msg_in_different_order():
    msg = SimpleNamespace(
        name=["wrist_3_joint", "shoulder_pan_joint", "elbow_joint",
              "shoulder_lift_joint", "wrist_2_joint", "wrist_1_joint"],
        position=[6.0, 1.0, 3.0, 2.0, 5.0, 4.0],
    )
    joints, reason = extract_joint_positions(msg)
    assert joints == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert reason == "ok"


def test_extract_joints_returns_none_when_msg_none():
    joints, reason = extract_joint_positions(None)
    assert joints is None
    assert reason == "joint_state_msg_none"


def test_extract_joints_returns_none_when_missing_joint():
    msg = SimpleNamespace(
        name=["shoulder_pan_joint", "shoulder_lift_joint"],
        position=[0.0, -1.57],
    )
    joints, reason = extract_joint_positions(msg)
    assert joints is None
    assert "joint_state_missing" in reason
    assert "elbow_joint" in reason


def test_extract_joints_returns_none_on_length_mismatch():
    msg = SimpleNamespace(
        name=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"],
        position=[0.0, -1.57],  # mismatch
    )
    joints, reason = extract_joint_positions(msg)
    assert joints is None
    assert "joint_state_len_mismatch" in reason


# ---------------------------------------------------------------------------
# capture_initial_snapshot (integración)
# ---------------------------------------------------------------------------


def test_capture_initial_snapshot_full_success():
    def lookup(*a, **kw):
        return _make_transform_stamped(0.0, 0.366, 1.001, qx=-0.707, qw=0.707)

    def resolve(name):
        assert name == "pick_demo"
        return _make_resolve_response(x=-0.41, y=0.0, z=0.875)

    result = capture_initial_snapshot(
        object_name="pick_demo",
        tf_lookup=lookup,
        joint_state_msg=_make_joint_state(),
        resolve_object_pose=resolve,
    )
    assert result.success is True
    assert result.reason == "snapshot_ok"
    assert result.tcp_pose_base == (0.0, 0.366, 1.001, -0.707, 0.0, 0.0, 0.707)
    assert result.joint_positions == (0.0, -1.57, 0.0, -1.57, 0.0, 0.0)
    assert result.object_pose_world == (-0.41, 0.0, 0.875, 0.0, 0.0, 0.0, 1.0)


def test_capture_initial_snapshot_partial_when_tf_fails():
    def lookup(*a, **kw):
        raise RuntimeError("tf_unavailable")

    def resolve(name):
        return _make_resolve_response()

    result = capture_initial_snapshot(
        object_name="pick_demo",
        tf_lookup=lookup,
        joint_state_msg=_make_joint_state(),
        resolve_object_pose=resolve,
    )
    assert result.success is False
    assert "tcp:tf_lookup_exception" in result.reason
    assert result.tcp_pose_base is None
    assert result.joint_positions is not None  # joints sí ok
    assert result.object_pose_world is not None  # objeto sí ok


def test_capture_initial_snapshot_partial_when_resolver_fails():
    def lookup(*a, **kw):
        return _make_transform_stamped(0.0, 0.366, 1.001)

    def resolve(name):
        return _make_resolve_response(success=False, detail="pose_stale:age=10s")

    result = capture_initial_snapshot(
        object_name="pick_demo",
        tf_lookup=lookup,
        joint_state_msg=_make_joint_state(),
        resolve_object_pose=resolve,
    )
    assert result.success is False
    assert "object:resolver_failed:pose_stale:age=10s" in result.reason
    assert result.tcp_pose_base is not None
    assert result.joint_positions is not None
    assert result.object_pose_world is None


def test_capture_initial_snapshot_handles_resolve_exception():
    def lookup(*a, **kw):
        return _make_transform_stamped(0.0, 0.366, 1.001)

    def resolve(name):
        raise RuntimeError("service_unavailable")

    result = capture_initial_snapshot(
        object_name="pick_demo",
        tf_lookup=lookup,
        joint_state_msg=_make_joint_state(),
        resolve_object_pose=resolve,
    )
    assert result.success is False
    assert "object:resolve_exception:RuntimeError" in result.reason
    assert result.object_pose_world is None


def test_capture_initial_snapshot_skips_object_when_require_false():
    """Modo non-strict: success=True aunque no haya pose objeto."""
    def lookup(*a, **kw):
        return _make_transform_stamped(0.0, 0.366, 1.001)

    result = capture_initial_snapshot(
        object_name="",  # vacío → no resolver
        tf_lookup=lookup,
        joint_state_msg=_make_joint_state(),
        resolve_object_pose=None,
        require_object_pose=False,
    )
    assert result.success is True
    assert result.reason == "snapshot_ok"
    assert result.object_pose_world is None


def test_capture_initial_snapshot_no_inputs_provided():
    result = capture_initial_snapshot(
        object_name="x",
        tf_lookup=None,
        joint_state_msg=None,
        resolve_object_pose=None,
    )
    assert result.success is False
    assert "tcp:no_tf_lookup_provided" in result.reason
    assert "joints:joint_state_msg_none" in result.reason
    assert "object:no_resolver_provided" in result.reason
