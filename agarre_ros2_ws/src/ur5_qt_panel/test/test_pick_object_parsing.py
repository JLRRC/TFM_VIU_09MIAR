#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_object_parsing.py
# Contenido: F3 — tests offline de pick_object/parsing_helpers.py.
"""Tests offline de los parsing helpers de pick_object."""

from __future__ import annotations

import math

import pytest

from ur5_qt_panel.pick_object.parsing_helpers import (
    is_step_exec_failed,
    is_step_tf_mismatch,
    max_joint_error,
    norm_frame,
    sanitize_ros_ns,
    tf_stamp_ns,
)


# ---------------------------------------------------------------------------
# norm_frame
# ---------------------------------------------------------------------------


def test_norm_frame_strips_slash_prefix():
    assert norm_frame("/base_link") == "base_link"


def test_norm_frame_strips_whitespace():
    assert norm_frame("  base_link  ") == "base_link"


def test_norm_frame_combined_whitespace_and_slash():
    assert norm_frame("  /world  ") == "world"


def test_norm_frame_empty():
    assert norm_frame("") == ""


def test_norm_frame_none():
    assert norm_frame(None) == ""


def test_norm_frame_no_changes_needed():
    assert norm_frame("base_link") == "base_link"


# ---------------------------------------------------------------------------
# tf_stamp_ns
# ---------------------------------------------------------------------------


class _FakeStamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _FakeHeader:
    def __init__(self, stamp):
        self.stamp = stamp


class _FakeTfMsg:
    def __init__(self, sec, nanosec):
        self.header = _FakeHeader(_FakeStamp(sec, nanosec))


def test_tf_stamp_ns_from_header():
    msg = _FakeTfMsg(sec=10, nanosec=500_000_000)
    assert tf_stamp_ns(msg) == 10 * 1_000_000_000 + 500_000_000


def test_tf_stamp_ns_zero_stamp():
    msg = _FakeTfMsg(sec=0, nanosec=0)
    assert tf_stamp_ns(msg) == 0


def test_tf_stamp_ns_none_msg():
    assert tf_stamp_ns(None) == 0


def test_tf_stamp_ns_missing_header():
    class Empty:
        pass
    assert tf_stamp_ns(Empty()) == 0


# ---------------------------------------------------------------------------
# sanitize_ros_ns
# ---------------------------------------------------------------------------


def test_sanitize_ros_ns_valid():
    assert sanitize_ros_ns(123_456_789) == 123_456_789


def test_sanitize_ros_ns_zero_returns_zero():
    assert sanitize_ros_ns(0) == 0


def test_sanitize_ros_ns_negative_returns_zero():
    assert sanitize_ros_ns(-100) == 0


def test_sanitize_ros_ns_walltime_epoch_returns_zero():
    """Valores >1e15 son walltime epoch (sec*1e9), no sim-time."""
    walltime_now_ns = 1_700_000_000 * 1_000_000_000  # ~2023 wall clock
    assert sanitize_ros_ns(walltime_now_ns) == 0


def test_sanitize_ros_ns_just_under_threshold():
    """1e15 - 1 should pass."""
    val = 999_999_999_999_999
    assert sanitize_ros_ns(val) == val


def test_sanitize_ros_ns_invalid_string_returns_zero():
    assert sanitize_ros_ns("not_a_number") == 0


def test_sanitize_ros_ns_none_returns_zero():
    assert sanitize_ros_ns(None) == 0


# ---------------------------------------------------------------------------
# is_step_tf_mismatch
# ---------------------------------------------------------------------------


def test_step_tf_mismatch_positive():
    msg = "exec_succeeded_but_tf_mismatch label=APPROACH_COARSE detail=..."
    assert is_step_tf_mismatch(msg, "APPROACH_COARSE") is True


def test_step_tf_mismatch_wrong_label():
    msg = "exec_succeeded_but_tf_mismatch label=FOO"
    assert is_step_tf_mismatch(msg, "APPROACH_COARSE") is False


def test_step_tf_mismatch_no_keyword():
    msg = "label=APPROACH_COARSE everything fine"
    assert is_step_tf_mismatch(msg, "APPROACH_COARSE") is False


# ---------------------------------------------------------------------------
# is_step_exec_failed
# ---------------------------------------------------------------------------


def test_step_exec_failed_format_a():
    assert is_step_exec_failed("execute failed (LIFT) detail=...", "LIFT") is True


def test_step_exec_failed_format_b_exec_failed():
    msg = "label=GRASP exec_failed: timeout"
    assert is_step_exec_failed(msg, "GRASP") is True


def test_step_exec_failed_format_c_fjt_timeout():
    msg = "label=APPROACH fjt_result_timeout reason=..."
    assert is_step_exec_failed(msg, "APPROACH") is True


def test_step_exec_failed_format_d_deterministic_joint():
    msg = "label=HOME deterministic_joint_mode bypassed"
    assert is_step_exec_failed(msg, "HOME") is True


def test_step_exec_failed_no_label():
    msg = "execute failed (LIFT)"
    # Sin label match en formatos b/c/d, pero formato a permite
    assert is_step_exec_failed(msg, "LIFT") is True


def test_step_exec_failed_wrong_label():
    msg = "label=WRONG exec_failed: timeout"
    assert is_step_exec_failed(msg, "GRASP") is False


def test_step_exec_failed_unrelated_message():
    assert is_step_exec_failed("everything fine label=GRASP", "GRASP") is False


# ---------------------------------------------------------------------------
# max_joint_error
# ---------------------------------------------------------------------------


def test_max_joint_error_empty_current():
    assert math.isinf(max_joint_error([], [0.0, 1.0]))


def test_max_joint_error_empty_target():
    assert math.isinf(max_joint_error([0.0, 1.0], []))


def test_max_joint_error_none_current():
    assert math.isinf(max_joint_error(None, [0.0]))


def test_max_joint_error_zero_when_identical():
    assert max_joint_error([0.1, 0.2, 0.3], [0.1, 0.2, 0.3]) == pytest.approx(0.0)


def test_max_joint_error_max_diff():
    out = max_joint_error([0.1, 0.5, 0.3], [0.2, 0.0, 0.4])
    # diffs: 0.1, 0.5, 0.1 → max 0.5
    assert out == pytest.approx(0.5)


def test_max_joint_error_zips_to_shorter():
    """zip stops at shorter — only first 2 compared."""
    out = max_joint_error([0.1, 0.2], [0.5, 0.0, 99.0])
    # diffs: 0.4, 0.2 → max 0.4
    assert out == pytest.approx(0.4)
