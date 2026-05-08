#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/test/test_grasp_selector_node_logic.py
"""F17-step2 (2026-05-08) — Tests offline para grasp_selector_node.

Cubre la conversión PoseArray → GraspCandidate (función pura del nodo).
El nodo ROS completo no se testea unitariamente — eso requiere
launch_testing y ROS vivo.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest


@dataclass
class _FakePoint:
    x: float
    y: float
    z: float


@dataclass
class _FakeQuaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass
class _FakePose:
    position: _FakePoint
    orientation: _FakeQuaternion


@dataclass
class _FakePoseArray:
    poses: list


# ---------------------------------------------------------------------------
# pose_array_to_candidates
# ---------------------------------------------------------------------------


def _make_pose(xyz, quat):
    return _FakePose(
        position=_FakePoint(*xyz),
        orientation=_FakeQuaternion(*quat),
    )


def test_pose_array_to_candidates_empty():
    pytest.importorskip("rclpy")
    from tfm_grasping.grasp_selector_node import pose_array_to_candidates
    msg = _FakePoseArray(poses=[])
    cands = pose_array_to_candidates(msg)
    assert cands == []


def test_pose_array_to_candidates_no_scores():
    pytest.importorskip("rclpy")
    from tfm_grasping.grasp_selector_node import pose_array_to_candidates
    msg = _FakePoseArray(poses=[
        _make_pose((0.5, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0)),
        _make_pose((0.4, 0.1, 0.3), (1.0, 0.0, 0.0, 0.0)),
    ])
    cands = pose_array_to_candidates(msg)
    assert len(cands) == 2
    assert cands[0].score == pytest.approx(0.0)
    assert cands[1].score == pytest.approx(0.0)
    assert cands[0].label == "cand_000"
    assert cands[1].label == "cand_001"


def test_pose_array_to_candidates_with_scores():
    pytest.importorskip("rclpy")
    from tfm_grasping.grasp_selector_node import pose_array_to_candidates
    msg = _FakePoseArray(poses=[
        _make_pose((0.5, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0)),
        _make_pose((0.4, 0.1, 0.3), (1.0, 0.0, 0.0, 0.0)),
    ])
    cands = pose_array_to_candidates(msg, scores=[0.7, 0.9])
    assert cands[0].score == pytest.approx(0.7)
    assert cands[1].score == pytest.approx(0.9)


def test_pose_array_to_candidates_xyz_quat_preserved():
    pytest.importorskip("rclpy")
    from tfm_grasping.grasp_selector_node import pose_array_to_candidates
    msg = _FakePoseArray(poses=[
        _make_pose((0.123, 0.456, 0.789), (0.1, 0.2, 0.3, 0.4)),
    ])
    cands = pose_array_to_candidates(msg)
    assert cands[0].xyz == pytest.approx((0.123, 0.456, 0.789))
    assert cands[0].quat_xyzw == pytest.approx((0.1, 0.2, 0.3, 0.4))


def test_pose_array_to_candidates_score_length_mismatch_falls_back_to_zero():
    """Si len(scores) != len(poses), usa 0.0 para todos."""
    pytest.importorskip("rclpy")
    from tfm_grasping.grasp_selector_node import pose_array_to_candidates
    msg = _FakePoseArray(poses=[
        _make_pose((0.5, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0)),
        _make_pose((0.4, 0.1, 0.3), (1.0, 0.0, 0.0, 0.0)),
    ])
    cands = pose_array_to_candidates(msg, scores=[0.7])  # mismatch
    assert all(c.score == 0.0 for c in cands)


def test_pose_array_to_candidates_label_format():
    """Labels siempre con formato cand_NNN (3 dígitos zero-padded)."""
    pytest.importorskip("rclpy")
    from tfm_grasping.grasp_selector_node import pose_array_to_candidates
    msg = _FakePoseArray(poses=[
        _make_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)) for _ in range(5)
    ])
    cands = pose_array_to_candidates(msg)
    labels = [c.label for c in cands]
    assert labels == ["cand_000", "cand_001", "cand_002", "cand_003", "cand_004"]
