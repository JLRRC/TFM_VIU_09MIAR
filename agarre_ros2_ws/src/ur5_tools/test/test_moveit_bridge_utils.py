#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_moveit_bridge_utils.py
# Contenido: Unit tests for moveit_bridge_utils — pure helpers, no ROS needed.
"""Unit tests for ur5_tools.moveit_bridge_utils — all run without rclpy."""

import math
import os

import pytest

from ur5_tools.moveit_bridge_utils import (
    bridge_env_float,
    describe_execute_result,
    diag_to_message,
    goal_status_text,
    is_stamp_fresh,
    joint_trajectory_duration_sec,
    joint_trajectory_initial_segment_max_delta,
    matrix_to_pose,
    normalize_action_name,
    parse_request_meta,
    plan_error_code_val,
    plan_success_code,
    plan_success_ok,
    pose_to_matrix,
    result_meta_to_message,
    scale_joint_trajectory_timing,
    stamp_age_sec,
    wait_future_done,
)


# ---------------------------------------------------------------------------
# bridge_env_float
# ---------------------------------------------------------------------------

def test_bridge_env_float_default(monkeypatch):
    monkeypatch.delenv("TEST_BEF_VAR", raising=False)
    assert bridge_env_float("TEST_BEF_VAR", 3.14) == pytest.approx(3.14)


def test_bridge_env_float_reads_env(monkeypatch):
    monkeypatch.setenv("TEST_BEF_VAR", "2.5")
    assert bridge_env_float("TEST_BEF_VAR", 0.0) == pytest.approx(2.5)


def test_bridge_env_float_bad_value_returns_default(monkeypatch):
    monkeypatch.setenv("TEST_BEF_VAR", "not_a_float")
    assert bridge_env_float("TEST_BEF_VAR", 9.9) == pytest.approx(9.9)


# ---------------------------------------------------------------------------
# normalize_action_name
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(("raw", "expected"), [
    ("follow_joint_trajectory", "/follow_joint_trajectory"),
    ("/follow_joint_trajectory", "/follow_joint_trajectory"),
    ("//double_slash", "/double_slash"),
    ("", ""),
    (None, ""),
])
def test_normalize_action_name(raw, expected):
    assert normalize_action_name(raw) == expected


# ---------------------------------------------------------------------------
# parse_request_meta
# ---------------------------------------------------------------------------

def test_parse_request_meta_empty():
    frame, rid, uuid, meta = parse_request_meta("")
    assert frame == "" and rid is None and uuid == "" and meta == {}


def test_parse_request_meta_frame_only():
    frame, rid, uuid, meta = parse_request_meta("world")
    assert frame == "world" and rid is None


def test_parse_request_meta_full():
    frame, rid, uuid, meta = parse_request_meta("world|rid=42|uid=abc|tol=0.01|phase=GRASP")
    assert frame == "world"
    assert rid == 42
    assert uuid == "abc"
    assert meta["tol_m"] == pytest.approx(0.01)
    assert meta["phase_label"] == "GRASP"


def test_parse_request_meta_bad_rid():
    _, rid, _, _ = parse_request_meta("f|rid=bad")
    assert rid is None


# ---------------------------------------------------------------------------
# plan_success_code
# ---------------------------------------------------------------------------

class _CodeObj:
    def __init__(self, val):
        self.val = val


@pytest.mark.parametrize(("raw", "expected"), [
    (True, 1),
    (False, 0),
    (1, 1),
    (0, 0),
    (_CodeObj(1), 1),
    (_CodeObj(-1), -1),
    (None, None),
])
def test_plan_success_code(raw, expected):
    assert plan_success_code(raw) == expected


# ---------------------------------------------------------------------------
# plan_success_ok
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(("raw", "expected"), [
    (True, True),
    (False, False),
    (1, True),
    (0, False),
    (_CodeObj(1), True),
    (_CodeObj(-1), False),
    ("SUCCESS", True),
    ("ABORTED", False),
    (None, True),
])
def test_plan_success_ok(raw, expected):
    assert plan_success_ok(raw) is expected


# ---------------------------------------------------------------------------
# plan_error_code_val
# ---------------------------------------------------------------------------

def test_plan_error_code_val_none():
    assert plan_error_code_val(object()) is None


def test_plan_error_code_val_with_val():
    class _Plan:
        class error_code:
            val = 99
    assert plan_error_code_val(_Plan()) == 99


def test_plan_error_code_val_int_ec():
    class _Plan:
        error_code = 5
    assert plan_error_code_val(_Plan()) == 5


# ---------------------------------------------------------------------------
# describe_execute_result
# ---------------------------------------------------------------------------

def test_describe_execute_result_none():
    assert describe_execute_result(None) == {"type": "NoneType"}


def test_describe_execute_result_has_success():
    class _Result:
        success = True
        status = None
        message = None
        error_code = None
        val = None
    meta = describe_execute_result(_Result())
    assert meta["type"] == "_Result"
    assert meta["success"] is True


# ---------------------------------------------------------------------------
# result_meta_to_message
# ---------------------------------------------------------------------------

def test_result_meta_to_message_empty():
    assert result_meta_to_message({}) == "none"


def test_result_meta_to_message_known_keys():
    msg = result_meta_to_message({"type": "Foo", "success": True})
    assert "type=Foo" in msg
    assert "success=True" in msg


# ---------------------------------------------------------------------------
# diag_to_message
# ---------------------------------------------------------------------------

def test_diag_to_message_contains_reason():
    diag = {
        "reason": "timeout",
        "backend": "moveit_py",
        "use_sim_time": True,
        "controller": "fjt",
        "controller_action": "/fjt",
        "controller_action_matched": None,
        "controller_action_available": False,
        "controller_manager": "/controller_manager",
        "active_controllers": [],
        "joint_state_age_sec": 0.123,
        "trajectory_topic_pubs": 1,
        "trajectory_topic_subs": 0,
    }
    msg = diag_to_message(diag)
    assert "timeout" in msg
    assert "joint_state_age_sec=0.123" in msg


# ---------------------------------------------------------------------------
# goal_status_text
# ---------------------------------------------------------------------------

def test_goal_status_text_known():
    assert goal_status_text(4) == "SUCCEEDED"
    assert goal_status_text(6) == "ABORTED"


def test_goal_status_text_unknown():
    assert goal_status_text(99) == "STATUS_99"


# ---------------------------------------------------------------------------
# wait_future_done
# ---------------------------------------------------------------------------

class _FutureDone:
    def done(self): return True


class _FutureNotDone:
    def done(self): return False


def test_wait_future_done_immediate():
    assert wait_future_done(_FutureDone(), 0.5) is True


def test_wait_future_done_timeout():
    assert wait_future_done(_FutureNotDone(), 0.05) is False


# ---------------------------------------------------------------------------
# JointTrajectory helpers (duck-typed stubs)
# ---------------------------------------------------------------------------

class _TFS:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _Point:
    def __init__(self, positions, tfs=None):
        self.positions = positions
        self.time_from_start = tfs


class _JT:
    def __init__(self, points):
        self.points = points


def test_joint_trajectory_duration_sec_none():
    assert joint_trajectory_duration_sec(None) == 0.0


def test_joint_trajectory_duration_sec_empty():
    assert joint_trajectory_duration_sec(_JT([])) == 0.0


def test_joint_trajectory_duration_sec_value():
    jt = _JT([_Point([0.0], _TFS(1, 500_000_000)), _Point([1.0], _TFS(2, 0))])
    assert joint_trajectory_duration_sec(jt) == pytest.approx(2.0)


def test_joint_trajectory_initial_segment_max_delta_few_points():
    assert joint_trajectory_initial_segment_max_delta(_JT([_Point([0.0])])) == -1.0


def test_joint_trajectory_initial_segment_max_delta_value():
    jt = _JT([_Point([0.0, 0.0]), _Point([0.3, 0.1])])
    assert joint_trajectory_initial_segment_max_delta(jt) == pytest.approx(0.3)


def test_scale_joint_trajectory_timing_doubles():
    jt = _JT([_Point([0.0], _TFS(1, 0)), _Point([1.0], _TFS(2, 0))])
    scaled = scale_joint_trajectory_timing(jt, scale=2.0)
    assert scaled.points[0].time_from_start.sec == 2
    assert scaled.points[1].time_from_start.sec == 4


def test_scale_joint_trajectory_timing_min_scale_1():
    jt = _JT([_Point([0.0], _TFS(1, 0))])
    scaled = scale_joint_trajectory_timing(jt, scale=0.1)
    assert scaled.points[0].time_from_start.sec == 1


# ---------------------------------------------------------------------------
# pose_to_matrix / matrix_to_pose (round-trip)
# ---------------------------------------------------------------------------

import numpy as np


class _Vec3:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Quat:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Pose:
    def __init__(self, pos, ori):
        self.position = pos
        self.orientation = ori


def _make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return _Pose(_Vec3(x, y, z), _Quat(qx, qy, qz, qw))


def test_pose_to_matrix_identity():
    T = pose_to_matrix(_make_pose(0, 0, 0))
    assert np.allclose(T, np.eye(4))


def test_pose_to_matrix_translation():
    T = pose_to_matrix(_make_pose(1.0, 2.0, 3.0))
    assert T[0, 3] == pytest.approx(1.0)
    assert T[1, 3] == pytest.approx(2.0)
    assert T[2, 3] == pytest.approx(3.0)


def test_pose_to_matrix_rotation_90z():
    # 90° around Z: qw=cos(45°), qz=sin(45°)
    s = math.sqrt(2) / 2
    T = pose_to_matrix(_make_pose(0, 0, 0, qx=0, qy=0, qz=s, qw=s))
    # X-column of R should become (0,1,0)
    assert np.allclose(T[:3, 0], [0, 1, 0], atol=1e-9)


def test_matrix_to_pose_identity():
    pytest.importorskip("geometry_msgs")
    T = np.eye(4)
    pose = matrix_to_pose(T)
    assert pose.position.x == pytest.approx(0.0)
    assert pose.orientation.w == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# stamp_age_sec / is_stamp_fresh
# ---------------------------------------------------------------------------

def test_stamp_age_fresh():
    assert stamp_age_sec(100.0, 100.5) == pytest.approx(0.5)


def test_stamp_age_zero_when_equal():
    assert stamp_age_sec(50.0, 50.0) == pytest.approx(0.0)


def test_stamp_age_clamps_negative():
    # clock skew / future stamp must not produce negative age
    assert stamp_age_sec(100.5, 100.0) == pytest.approx(0.0)


def test_is_stamp_fresh_within_limit():
    assert is_stamp_fresh(100.0, 100.3, max_age_sec=0.5) is True


def test_is_stamp_fresh_exactly_at_limit():
    assert is_stamp_fresh(100.0, 100.5, max_age_sec=0.5) is True


def test_is_stamp_fresh_just_over_limit():
    assert is_stamp_fresh(100.0, 100.51, max_age_sec=0.5) is False


def test_is_stamp_fresh_negative_max_age_disabled():
    # max_age_sec < 0 means freshness checking is disabled
    assert is_stamp_fresh(0.0, 9999.0, max_age_sec=-1.0) is True
