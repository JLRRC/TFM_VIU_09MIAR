#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_plan_to_pose_moveit_direct.py
# Contenido: B-iter3 (2026-05-03) — tests del helper MOVEIT_DIRECT.
"""Tests para plan_to_pose_moveit_direct.

Cubre:
  - build_move_group_goal: estructura del MotionPlanRequest construido
  - parse_move_group_result: decoding de MoveItErrorCodes
"""

from __future__ import annotations

import pytest


# ---------------------------------------------------------------------------
# build_move_group_goal
# ---------------------------------------------------------------------------


def test_build_goal_basic_structure():
    """Goal contiene MotionPlanRequest con goal_constraints + PlanningOptions."""
    from ur5_tools.plan_to_pose_moveit_direct import build_move_group_goal
    g = build_move_group_goal(
        target_xyz=(0.5, 0.1, 0.6),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="rg2_pinch_center",
        base_frame="base_link",
    )
    assert g.request.group_name == "manipulator"  # default (UR5 SRDF)
    assert g.request.allowed_planning_time == pytest.approx(5.0)
    assert g.planning_options.plan_only is False
    assert len(g.request.goal_constraints) == 1
    cs = g.request.goal_constraints[0]
    assert len(cs.position_constraints) == 1
    assert len(cs.orientation_constraints) == 1


def test_build_goal_position_constraint_uses_target_and_ee_frame():
    from ur5_tools.plan_to_pose_moveit_direct import build_move_group_goal
    g = build_move_group_goal(
        target_xyz=(0.123, 0.456, 0.789),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="custom_ee",
        base_frame="custom_base",
        position_tol_m=0.010,
    )
    pc = g.request.goal_constraints[0].position_constraints[0]
    assert pc.link_name == "custom_ee"
    assert pc.header.frame_id == "custom_base"
    sphere = pc.constraint_region.primitives[0]
    assert sphere.dimensions[0] == pytest.approx(0.010)
    pose = pc.constraint_region.primitive_poses[0]
    assert pose.position.x == pytest.approx(0.123)
    assert pose.position.y == pytest.approx(0.456)
    assert pose.position.z == pytest.approx(0.789)


def test_build_goal_orientation_constraint_uses_target_quat():
    from ur5_tools.plan_to_pose_moveit_direct import build_move_group_goal
    g = build_move_group_goal(
        target_xyz=(0.0, 0.0, 0.0),
        target_quat_xyzw=(0.1, 0.2, 0.3, 0.4),
        ee_frame="ee",
        base_frame="base",
        orientation_tol_rad=0.123,
    )
    oc = g.request.goal_constraints[0].orientation_constraints[0]
    assert oc.link_name == "ee"
    assert oc.header.frame_id == "base"
    assert oc.orientation.x == pytest.approx(0.1)
    assert oc.orientation.y == pytest.approx(0.2)
    assert oc.orientation.z == pytest.approx(0.3)
    assert oc.orientation.w == pytest.approx(0.4)
    assert oc.absolute_x_axis_tolerance == pytest.approx(0.123)
    assert oc.absolute_y_axis_tolerance == pytest.approx(0.123)
    assert oc.absolute_z_axis_tolerance == pytest.approx(0.123)


def test_build_goal_custom_group_and_planner():
    from ur5_tools.plan_to_pose_moveit_direct import build_move_group_goal
    g = build_move_group_goal(
        target_xyz=(0.0, 0.0, 0.0),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="ee",
        base_frame="base",
        group_name="custom_group",
        planner_id="RRTConnect",
        planning_time_sec=10.0,
    )
    assert g.request.group_name == "custom_group"
    assert g.request.planner_id == "RRTConnect"
    assert g.request.allowed_planning_time == pytest.approx(10.0)


# ---------------------------------------------------------------------------
# parse_move_group_result
# ---------------------------------------------------------------------------


class _FakeErrorCode:
    def __init__(self, val: int):
        self.val = val


class _FakeResult:
    def __init__(self, val: int):
        self.error_code = _FakeErrorCode(val)


class _FakeWrapper:
    def __init__(self, val: int):
        self.result = _FakeResult(val)


def test_parse_result_success():
    from ur5_tools.plan_to_pose_moveit_direct import parse_move_group_result
    ok, reason = parse_move_group_result(_FakeWrapper(1))
    assert ok is True
    assert reason == "moveit:SUCCESS"


def test_parse_result_planning_failed():
    from ur5_tools.plan_to_pose_moveit_direct import parse_move_group_result
    ok, reason = parse_move_group_result(_FakeWrapper(-1))
    assert ok is False
    assert reason == "moveit_err:PLANNING_FAILED"


def test_parse_result_goal_in_collision():
    from ur5_tools.plan_to_pose_moveit_direct import parse_move_group_result
    ok, reason = parse_move_group_result(_FakeWrapper(-12))
    assert ok is False
    assert reason == "moveit_err:GOAL_IN_COLLISION"


def test_parse_result_unknown_code():
    from ur5_tools.plan_to_pose_moveit_direct import parse_move_group_result
    ok, reason = parse_move_group_result(_FakeWrapper(-9999))
    assert ok is False
    assert "unknown:val=-9999" in reason


def test_parse_result_none_wrapper():
    from ur5_tools.plan_to_pose_moveit_direct import parse_move_group_result
    ok, reason = parse_move_group_result(None)
    assert ok is False
    assert reason == "no_result"


def test_parse_result_no_payload():
    from ur5_tools.plan_to_pose_moveit_direct import parse_move_group_result

    class _NoPayload:
        result = None

    ok, reason = parse_move_group_result(_NoPayload())
    assert ok is False
    assert reason == "no_payload"


def test_parse_result_no_error_code():
    from ur5_tools.plan_to_pose_moveit_direct import parse_move_group_result

    class _NoErr:
        pass

    class _Wrapper:
        result = _NoErr()

    ok, reason = parse_move_group_result(_Wrapper())
    assert ok is False
    assert reason == "no_error_code"
