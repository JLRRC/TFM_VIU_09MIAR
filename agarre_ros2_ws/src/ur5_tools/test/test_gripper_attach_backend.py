#!/usr/bin/env python3
"""Tests unitarios de seleccion de fuente TCP en el backend de attach."""

from __future__ import annotations

from ur5_tools.gripper_attach_backend import GripperAttachBackend, PoseSample


def _pose(x: float, y: float, z: float, *, stamp_ns: int = 1) -> PoseSample:
    return PoseSample(
        x=x,
        y=y,
        z=z,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        stamp_ns=stamp_ns,
    )


def _make_backend() -> GripperAttachBackend:
    backend = object.__new__(GripperAttachBackend)
    backend._world_frame = "world"
    backend._base_frame = "base_link"
    backend._tcp_frame = "rg2_pinch_center"
    backend._max_pose_age_sec = 1.5
    backend._stable_world_base_pose = None
    backend._joint_state_positions = None
    backend._joint_state_stamp_ns = 0
    backend._last_tcp_pose_diag = {}
    backend._last_tcp_pose_source = "none"
    return backend


def test_lookup_tcp_pose_prefers_base_chain_when_all_candidates_are_stale() -> None:
    backend = _make_backend()
    backend._stable_world_base_pose = _pose(0.0, 0.0, 0.0, stamp_ns=5)

    world_tcp = _pose(1.0, 0.0, 0.0, stamp_ns=10)
    base_tcp = _pose(2.0, 0.0, 0.0, stamp_ns=20)

    def fake_lookup_tf_pose(parent: str, child: str):
        lookup = {
            ("world", "rg2_pinch_center"): world_tcp,
            ("world", "base_link"): None,
            ("base_link", "rg2_pinch_center"): base_tcp,
        }
        return lookup.get((parent, child))

    backend._lookup_tf_pose = fake_lookup_tf_pose
    backend._fallback_tcp_pose = lambda: None
    backend._pose_age_sec = lambda pose: 3.6
    backend._pose_age_ok = lambda pose: False

    selected = backend._lookup_tcp_pose()

    assert selected is not None
    assert selected.x == 2.0
    assert selected.y == 0.0
    assert selected.z == 0.0
    assert backend._last_tcp_pose_source == "base_chain"


def test_lookup_tcp_pose_prefers_joint_state_chain_when_fresh() -> None:
    backend = _make_backend()
    world_base = _pose(10.0, 0.0, 0.0, stamp_ns=50)
    joint_state_base = _pose(3.0, 0.0, 0.0, stamp_ns=60)
    world_tcp = _pose(99.0, 0.0, 0.0, stamp_ns=10)

    def fake_lookup_tf_pose(parent: str, child: str):
        lookup = {
            ("world", "rg2_pinch_center"): world_tcp,
            ("world", "base_link"): world_base,
            ("base_link", "rg2_pinch_center"): None,
        }
        return lookup.get((parent, child))

    def fake_pose_age(pose: PoseSample) -> float:
        if pose.stamp_ns >= 50:
            return 0.05
        return 3.6

    backend._lookup_tf_pose = fake_lookup_tf_pose
    backend._joint_state_tcp_base_pose = lambda: joint_state_base
    backend._fallback_tcp_pose = lambda: None
    backend._pose_age_sec = fake_pose_age
    backend._pose_age_ok = lambda pose: fake_pose_age(pose) <= 1.5

    selected = backend._lookup_tcp_pose()

    assert selected is not None
    assert selected.x == 13.0
    assert selected.y == 0.0
    assert selected.z == 0.0
    assert backend._last_tcp_pose_source == "joint_state_chain"
