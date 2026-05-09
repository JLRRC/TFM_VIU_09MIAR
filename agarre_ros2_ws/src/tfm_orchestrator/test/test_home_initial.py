#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_home_initial.py
# Contenido: B-iter6 (2026-05-03) — tests del helper HOME_INITIAL real.
"""Tests para tfm_orchestrator.home_initial.

100% offline: build/parse de mensajes ROS sin levantar nodes.
"""

from __future__ import annotations

import math

import pytest

from tfm_orchestrator.home_initial import (
    UR5_HOME_POSITIONS_RAD,
    UR5_JOINT_NAMES_CANONICAL,
    build_follow_joint_trajectory_goal,
    build_home_joint_trajectory,
    parse_fjt_result,
)


# ---------------------------------------------------------------------------
# build_home_joint_trajectory
# ---------------------------------------------------------------------------


def test_build_home_jt_uses_default_home_positions():
    jt = build_home_joint_trajectory()
    assert list(jt.joint_names) == list(UR5_JOINT_NAMES_CANONICAL)
    assert len(jt.points) == 1
    pt = jt.points[0]
    assert list(pt.positions) == list(UR5_HOME_POSITIONS_RAD)
    assert list(pt.positions) == [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
    assert list(pt.velocities) == [0.0] * 6
    assert list(pt.accelerations) == [0.0] * 6


def test_build_home_jt_default_duration_is_5s():
    jt = build_home_joint_trajectory()
    pt = jt.points[0]
    assert pt.time_from_start.sec == 5
    assert pt.time_from_start.nanosec == 0


def test_build_home_jt_custom_duration_fractional():
    jt = build_home_joint_trajectory(duration_sec=2.5)
    pt = jt.points[0]
    assert pt.time_from_start.sec == 2
    assert pt.time_from_start.nanosec == 500_000_000


def test_build_home_jt_min_duration_clamped_to_05s():
    jt = build_home_joint_trajectory(duration_sec=0.1)
    pt = jt.points[0]
    assert pt.time_from_start.sec == 0
    assert pt.time_from_start.nanosec == 500_000_000


def test_build_home_jt_custom_positions():
    custom = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    jt = build_home_joint_trajectory(home_positions=custom)
    assert list(jt.points[0].positions) == list(custom)


def test_build_home_jt_raises_on_length_mismatch():
    with pytest.raises(ValueError, match="len mismatch"):
        build_home_joint_trajectory(home_positions=(0.0, 0.0))


# ---------------------------------------------------------------------------
# build_follow_joint_trajectory_goal
# ---------------------------------------------------------------------------


def test_build_fjt_goal_basic_structure():
    jt = build_home_joint_trajectory()
    goal = build_follow_joint_trajectory_goal(jt)
    assert goal.trajectory is jt
    assert len(goal.path_tolerance) == 6
    assert len(goal.goal_tolerance) == 6
    # F1.7 (2026-05-08): path_tolerance default 10.0 rad (effectively
    # disabled) para que gz_ros2_control en sim_time no aborte con
    # PATH_TOLERANCE_VIOLATED. goal_tolerance sigue 0.10 (estricto).
    assert goal.path_tolerance[0].position == pytest.approx(10.0)
    assert goal.goal_tolerance[0].position == pytest.approx(0.10)
    assert goal.goal_time_tolerance.sec == 5
    assert goal.goal_time_tolerance.nanosec == 0


def test_build_fjt_goal_custom_position_tol():
    jt = build_home_joint_trajectory()
    goal = build_follow_joint_trajectory_goal(jt, position_tol_rad=0.05)
    # F1.7: position_tol_rad sólo afecta a goal_tolerance.
    # path_tolerance default 10.0 rad salvo path_tol_rad explícito.
    assert all(t.position == pytest.approx(10.0) for t in goal.path_tolerance)
    assert all(t.position == pytest.approx(0.05) for t in goal.goal_tolerance)


def test_build_fjt_goal_position_tol_clamped_min():
    jt = build_home_joint_trajectory()
    goal = build_follow_joint_trajectory_goal(jt, position_tol_rad=0.001)
    # F1.7: position_tol_rad clamped a min 0.01 sólo afecta goal_tolerance.
    assert all(t.position == pytest.approx(0.01) for t in goal.goal_tolerance)
    # path_tolerance default 10.0 sin tocar.
    assert all(t.position == pytest.approx(10.0) for t in goal.path_tolerance)


def test_build_fjt_goal_custom_goal_time_tol():
    jt = build_home_joint_trajectory()
    goal = build_follow_joint_trajectory_goal(jt, goal_time_tol_sec=2.5)
    assert goal.goal_time_tolerance.sec == 2
    assert goal.goal_time_tolerance.nanosec == 500_000_000


def test_build_fjt_goal_tolerance_names_match_joints():
    jt = build_home_joint_trajectory()
    goal = build_follow_joint_trajectory_goal(jt)
    pt_names = [t.name for t in goal.path_tolerance]
    gt_names = [t.name for t in goal.goal_tolerance]
    assert pt_names == list(UR5_JOINT_NAMES_CANONICAL)
    assert gt_names == list(UR5_JOINT_NAMES_CANONICAL)


# ---------------------------------------------------------------------------
# parse_fjt_result
# ---------------------------------------------------------------------------


class _FakeFjtResult:
    def __init__(self, error_code: int, error_string: str = ""):
        self.error_code = error_code
        self.error_string = error_string


class _FakeWrapper:
    def __init__(self, error_code: int, error_string: str = ""):
        self.result = _FakeFjtResult(error_code, error_string)


def test_parse_fjt_success():
    ok, reason = parse_fjt_result(_FakeWrapper(0))
    assert ok is True
    assert reason == "fjt:SUCCESSFUL"


def test_parse_fjt_path_tolerance_violated():
    ok, reason = parse_fjt_result(_FakeWrapper(-4, "joint_X exceeded tolerance"))
    assert ok is False
    assert "PATH_TOLERANCE_VIOLATED" in reason
    assert "joint_X exceeded tolerance" in reason


def test_parse_fjt_goal_tolerance_violated_no_string():
    ok, reason = parse_fjt_result(_FakeWrapper(-5))
    assert ok is False
    assert reason == "fjt_err:GOAL_TOLERANCE_VIOLATED"


def test_parse_fjt_unknown_error_code():
    ok, reason = parse_fjt_result(_FakeWrapper(-99))
    assert ok is False
    assert "unknown:val=-99" in reason


def test_parse_fjt_none_wrapper():
    ok, reason = parse_fjt_result(None)
    assert ok is False
    assert reason == "no_result"


def test_parse_fjt_no_payload():
    class _NoPayload:
        result = None
    ok, reason = parse_fjt_result(_NoPayload())
    assert ok is False
    assert reason == "no_payload"
