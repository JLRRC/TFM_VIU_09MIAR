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
# build_move_group_goal — F1.18 scaling per-fase
# ---------------------------------------------------------------------------


def test_build_goal_default_scaling_is_0_25():
    """F1.18 default: cuando no se pasa scaling, usa 0.25 (fases cortas)."""
    from ur5_tools.plan_to_pose_moveit_direct import build_move_group_goal
    g = build_move_group_goal(
        target_xyz=(0.5, 0.1, 0.6),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="rg2_pinch_center",
        base_frame="base_link",
    )
    assert g.request.max_velocity_scaling_factor == pytest.approx(0.25)
    assert g.request.max_acceleration_scaling_factor == pytest.approx(0.25)


def test_build_goal_transport_scaling_0_5():
    """F1.18: cuando se pasa scaling=0.5 (TRANSPORT), se aplica."""
    from ur5_tools.plan_to_pose_moveit_direct import build_move_group_goal
    g = build_move_group_goal(
        target_xyz=(0.5, 0.0, 0.0),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="rg2_pinch_center",
        base_frame="base_link",
        velocity_scaling_factor=0.5,
        acceleration_scaling_factor=0.5,
    )
    assert g.request.max_velocity_scaling_factor == pytest.approx(0.5)
    assert g.request.max_acceleration_scaling_factor == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# classify_phase_by_target_z — F1.18 heurística per-fase
# ---------------------------------------------------------------------------


def test_classify_phase_transport_drop_pose():
    """TRANSPORT: target Z < 0.05 base_link (drop a Z_world≈0.85)."""
    from ur5_tools.plan_to_pose_moveit_direct import classify_phase_by_target_z
    pt = classify_phase_by_target_z((0.5, 0.0, 0.0))
    assert pt.phase_label == "TRANSPORT"
    assert pt.velocity_scaling == pytest.approx(0.5)
    assert pt.acceleration_scaling == pytest.approx(0.5)
    assert pt.first_attempt_timeout_sec == pytest.approx(240.0)


def test_classify_phase_other_approach():
    """APPROACH: target Z >> 0.05 (objeto sobre mesa, base_link Z>0.1)."""
    from ur5_tools.plan_to_pose_moveit_direct import classify_phase_by_target_z
    pt = classify_phase_by_target_z((0.5, 0.0, 0.30))
    assert pt.phase_label == "OTHER"
    assert pt.velocity_scaling == pytest.approx(0.25)
    assert pt.acceleration_scaling == pytest.approx(0.25)
    assert pt.first_attempt_timeout_sec == pytest.approx(120.0)


def test_classify_phase_boundary_at_threshold():
    """Z = 0.05 exacto cae en OTHER (umbral es <, no <=)."""
    from ur5_tools.plan_to_pose_moveit_direct import classify_phase_by_target_z
    pt = classify_phase_by_target_z((0.5, 0.0, 0.05))
    assert pt.phase_label == "OTHER"


def test_classify_phase_boundary_below_threshold():
    """Z = 0.049 cae en TRANSPORT."""
    from ur5_tools.plan_to_pose_moveit_direct import classify_phase_by_target_z
    pt = classify_phase_by_target_z((0.5, 0.0, 0.049))
    assert pt.phase_label == "TRANSPORT"


def test_classify_phase_custom_threshold():
    """El umbral es configurable (knob de tuning para regresiones)."""
    from ur5_tools.plan_to_pose_moveit_direct import classify_phase_by_target_z
    # Con threshold 0.10, Z=0.08 ahora cae en TRANSPORT
    pt = classify_phase_by_target_z((0.5, 0.0, 0.08), transport_z_threshold_m=0.10)
    assert pt.phase_label == "TRANSPORT"
    pt2 = classify_phase_by_target_z((0.5, 0.0, 0.08), transport_z_threshold_m=0.05)
    assert pt2.phase_label == "OTHER"


def test_classify_phase_negative_z():
    """Z negativo (poco realista pero válido) cae en TRANSPORT."""
    from ur5_tools.plan_to_pose_moveit_direct import classify_phase_by_target_z
    pt = classify_phase_by_target_z((0.5, 0.0, -0.10))
    assert pt.phase_label == "TRANSPORT"


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
