#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_fjt_direct_helpers.py
"""F1.24 / H9 LIVE (2026-05-08) — Tests offline para fjt_direct_helpers."""

from __future__ import annotations

import pytest

from ur5_tools.fjt_direct_helpers import (
    build_fjt_trajectory_two_point,
    build_ik_request,
    parse_ik_result,
)


# ---------------------------------------------------------------------------
# build_ik_request
# ---------------------------------------------------------------------------


def test_build_ik_request_basic_structure():
    req = build_ik_request(
        target_xyz=(0.5, 0.0, 0.3),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="rg2_pinch_center",
        base_frame="base_link",
        group_name="manipulator",
        current_joints=[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        joint_names=[
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ],
    )
    ik = req.ik_request
    assert ik.group_name == "manipulator"
    assert ik.ik_link_name == "rg2_pinch_center"
    assert ik.pose_stamped.header.frame_id == "base_link"
    assert ik.pose_stamped.pose.position.x == pytest.approx(0.5)
    assert ik.pose_stamped.pose.orientation.w == pytest.approx(1.0)
    assert ik.avoid_collisions is False  # default


def test_build_ik_request_seed_state():
    req = build_ik_request(
        target_xyz=(0.5, 0.0, 0.3),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="ee",
        base_frame="base",
        group_name="g",
        current_joints=[0.1, 0.2, 0.3],
        joint_names=["j1", "j2", "j3"],
    )
    js = req.ik_request.robot_state.joint_state
    assert list(js.name) == ["j1", "j2", "j3"]
    assert list(js.position) == pytest.approx([0.1, 0.2, 0.3])


def test_build_ik_request_avoid_collisions_opt_in():
    req = build_ik_request(
        target_xyz=(0.5, 0.0, 0.3),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="ee",
        base_frame="base",
        group_name="g",
        current_joints=[0.0],
        joint_names=["j1"],
        avoid_collisions=True,
    )
    assert req.ik_request.avoid_collisions is True


def test_build_ik_request_timeout_in_duration():
    req = build_ik_request(
        target_xyz=(0.5, 0.0, 0.3),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="ee",
        base_frame="base",
        group_name="g",
        current_joints=[0.0],
        joint_names=["j1"],
        timeout_sec=2.5,
    )
    assert req.ik_request.timeout.sec == 2
    assert req.ik_request.timeout.nanosec == pytest.approx(500_000_000, abs=10)


# ---------------------------------------------------------------------------
# build_fjt_trajectory_two_point
# ---------------------------------------------------------------------------


def test_build_fjt_trajectory_two_points():
    jt = build_fjt_trajectory_two_point(
        joint_names=["j1", "j2"],
        start_positions=[0.0, 0.5],
        target_positions=[1.0, 1.5],
        duration_sec=3.0,
    )
    assert list(jt.joint_names) == ["j1", "j2"]
    assert len(jt.points) == 2
    assert list(jt.points[0].positions) == pytest.approx([0.0, 0.5])
    assert jt.points[0].time_from_start.sec == 0
    assert list(jt.points[1].positions) == pytest.approx([1.0, 1.5])
    assert jt.points[1].time_from_start.sec == 3


def test_build_fjt_trajectory_zero_velocities():
    jt = build_fjt_trajectory_two_point(
        joint_names=["j1"],
        start_positions=[0.0],
        target_positions=[1.0],
    )
    # Both points should have zero velocities (start + end at rest)
    assert all(v == 0.0 for v in jt.points[0].velocities)
    assert all(v == 0.0 for v in jt.points[1].velocities)


def test_build_fjt_trajectory_length_mismatch_raises():
    with pytest.raises(ValueError):
        build_fjt_trajectory_two_point(
            joint_names=["j1", "j2"],
            start_positions=[0.0],
            target_positions=[1.0, 2.0],
        )


def test_build_fjt_trajectory_fractional_duration():
    jt = build_fjt_trajectory_two_point(
        joint_names=["j1"],
        start_positions=[0.0],
        target_positions=[1.0],
        duration_sec=1.5,
    )
    assert jt.points[1].time_from_start.sec == 1
    assert jt.points[1].time_from_start.nanosec == pytest.approx(500_000_000, abs=10)


def test_build_fjt_trajectory_minimum_duration():
    """Duración 0 o negativa se clamp a 0.1s para evitar punto degenerado."""
    jt = build_fjt_trajectory_two_point(
        joint_names=["j1"],
        start_positions=[0.0],
        target_positions=[1.0],
        duration_sec=0.0,
    )
    assert jt.points[1].time_from_start.sec == 0
    assert jt.points[1].time_from_start.nanosec >= 100_000_000


# ---------------------------------------------------------------------------
# parse_ik_result
# ---------------------------------------------------------------------------


class _FakeErrCode:
    def __init__(self, val: int):
        self.val = val


class _FakeJointState:
    def __init__(self, names, positions):
        self.name = list(names)
        self.position = list(positions)


class _FakeRobotState:
    def __init__(self, joint_state):
        self.joint_state = joint_state


class _FakeResponse:
    def __init__(self, val: int, names=None, positions=None):
        self.error_code = _FakeErrCode(val)
        if names is not None and positions is not None:
            self.solution = _FakeRobotState(_FakeJointState(names, positions))
        else:
            self.solution = None


def test_parse_ik_result_success():
    resp = _FakeResponse(
        val=1,
        names=["j1", "j2", "j3"],
        positions=[0.5, -0.3, 0.7],
    )
    ok, reason, positions = parse_ik_result(resp, ["j1", "j2", "j3"])
    assert ok is True
    assert reason == "ik:SUCCESS"
    assert positions == pytest.approx([0.5, -0.3, 0.7])


def test_parse_ik_result_no_ik_solution():
    resp = _FakeResponse(val=-31)
    ok, reason, positions = parse_ik_result(resp, ["j1"])
    assert ok is False
    assert "NO_IK_SOLUTION" in reason
    assert positions is None


def test_parse_ik_result_planning_failed():
    resp = _FakeResponse(val=-1)
    ok, reason, positions = parse_ik_result(resp, ["j1"])
    assert ok is False
    assert "PLANNING_FAILED" in reason


def test_parse_ik_result_unknown_error():
    resp = _FakeResponse(val=-9999)
    ok, reason, positions = parse_ik_result(resp, ["j1"])
    assert ok is False
    assert "err_val=-9999" in reason


def test_parse_ik_result_none_response():
    ok, reason, positions = parse_ik_result(None, ["j1"])
    assert ok is False
    assert reason == "ik_no_response"


def test_parse_ik_result_reorders_joints():
    """IK puede devolver joints en orden distinto — debe reordenar al orden esperado."""
    resp = _FakeResponse(
        val=1,
        names=["j3", "j1", "j2"],
        positions=[0.7, 0.5, -0.3],
    )
    ok, reason, positions = parse_ik_result(resp, ["j1", "j2", "j3"])
    assert ok is True
    assert positions == pytest.approx([0.5, -0.3, 0.7])


def test_parse_ik_result_missing_joint():
    resp = _FakeResponse(
        val=1,
        names=["j1", "j2"],  # falta j3
        positions=[0.5, -0.3],
    )
    ok, reason, positions = parse_ik_result(resp, ["j1", "j2", "j3"])
    assert ok is False
    assert "ik_joint_missing:j3" in reason
