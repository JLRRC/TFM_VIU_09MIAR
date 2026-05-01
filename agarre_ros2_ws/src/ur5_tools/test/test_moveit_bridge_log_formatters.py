#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_moveit_bridge_log_formatters.py
# Contenido: F3 — tests offline de log_formatters + queue_helpers.
"""Tests offline de moveit_bridge.log_formatters + queue_helpers."""

from __future__ import annotations

import pytest

from ur5_tools.moveit_bridge.log_formatters import (
    format_busy_message,
    format_exec_start_log,
    format_pick_request_log,
    format_pose_xyz,
    format_recv_log,
    format_rx_log,
    format_target_log,
)
from ur5_tools.moveit_bridge.queue_helpers import (
    compute_request_stamp_ns,
    compute_settle_params,
    is_invalid_business_frame,
    is_stale_request,
    stale_request_age,
)


# ---------------------------------------------------------------------------
# format_pose_xyz
# ---------------------------------------------------------------------------


class _Vec:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def test_format_pose_from_object():
    assert format_pose_xyz(_Vec(0.5, 0.4, 0.3)) == "(0.500,0.400,0.300)"


def test_format_pose_from_dict():
    assert format_pose_xyz({"x": 1.234, "y": -2.0, "z": 0.0}) == "(1.234,-2.000,0.000)"


def test_format_pose_from_tuple():
    assert format_pose_xyz((0.1, 0.2, 0.3)) == "(0.100,0.200,0.300)"


def test_format_pose_none():
    assert format_pose_xyz(None) == "(0.000,0.000,0.000)"


# ---------------------------------------------------------------------------
# format_rx_log
# ---------------------------------------------------------------------------


def test_rx_log_accepted():
    s = format_rx_log(
        ts_us=123,
        request_id=7,
        request_uuid="abc",
        frame_id="base_link",
        pose=(0.5, 0.5, 0.1),
        accepted=True,
    )
    assert "[MOVEIT_BRIDGE][RX]" in s
    assert "ts_us=123" in s
    assert "req_id=7" in s
    assert "req_uuid=abc" in s
    assert "frame=base_link" in s
    assert "pose=(0.500,0.500,0.100)" in s
    assert "accepted=true" in s
    assert "reason=" not in s


def test_rx_log_rejected_with_reason():
    s = format_rx_log(
        ts_us=0,
        request_id=1,
        request_uuid="",
        frame_id="",
        pose=None,
        accepted=False,
        reason="bridge_busy",
    )
    assert "accepted=false" in s
    assert "reason=bridge_busy" in s
    assert "req_uuid=n/a" in s
    assert "frame=n/a" in s


def test_rx_log_rejected_no_reason():
    s = format_rx_log(
        ts_us=0,
        request_id=1,
        request_uuid="",
        frame_id="",
        pose=None,
        accepted=False,
    )
    assert "accepted=false" in s
    assert "reason=" not in s


# ---------------------------------------------------------------------------
# format_recv_log
# ---------------------------------------------------------------------------


def test_recv_log_pose():
    s = format_recv_log(
        label="POSE",
        topic_name="/desired_grasp",
        request_id=5,
        frame_id="base_link",
        pose=(0.5, 0.4, 0.0),
        stamp_sec=10,
        stamp_nanosec=500_000_000,
        frame_raw="base_link;req=5",
        request_uuid="uu",
        ee_target_tol_m=0.025,
        phase_label="APPROACH",
    )
    assert "[BRIDGE][RECV]" in s
    assert "label=POSE" in s
    assert "topic=/desired_grasp" in s
    assert "stamp=10.500000000" in s
    assert "ee_target_tol_m=0.025" in s
    assert "phase=APPROACH" in s


def test_recv_log_cartesian_no_tol():
    s = format_recv_log(
        label="CARTESIAN",
        topic_name="",
        request_id=1,
        frame_id="base_link",
        pose=(0, 0, 0),
        stamp_sec=0,
        stamp_nanosec=0,
        frame_raw="",
        request_uuid="",
        ee_target_tol_m=None,
        phase_label=None,
    )
    assert "label=CARTESIAN" in s
    assert "topic=n/a" in s
    assert "ee_target_tol_m=n/a" in s
    assert "phase=n/a" in s


# ---------------------------------------------------------------------------
# format_pick_request_log
# ---------------------------------------------------------------------------


def test_pick_request_log_full():
    s = format_pick_request_log(
        ts_us=999,
        request_id=42,
        request_uuid="abc",
        frame_id="base_link",
        pose=(0.5, 0.5, 0.1),
        cartesian=True,
        phase_label="LIFT",
        ee_target_tol_m=0.030,
    )
    assert "[PICK][MOVEIT][REQUEST]" in s
    assert "cartesian=true" in s
    assert "phase=LIFT" in s
    assert "ee_target_tol_m=0.03" in s


# ---------------------------------------------------------------------------
# format_exec_start_log
# ---------------------------------------------------------------------------


def test_exec_start_log_pose():
    s = format_exec_start_log(
        request_id=3,
        cartesian=False,
        frame_id="base_link",
        ee_frame="rg2_pinch_center",
        base_frame="base_link",
        pose=(0.5, 0.5, 0.05),
    )
    assert "[BRIDGE][EXEC_START]" in s
    assert "label=POSE" in s
    assert "ee_link=rg2_pinch_center" in s


def test_exec_start_log_cartesian():
    s = format_exec_start_log(
        request_id=3,
        cartesian=True,
        frame_id="base_link",
        ee_frame="rg2_pinch_center",
        base_frame="base_link",
        pose=(0.5, 0.5, 0.05),
    )
    assert "label=CARTESIAN" in s


# ---------------------------------------------------------------------------
# format_target_log
# ---------------------------------------------------------------------------


def test_target_log():
    s = format_target_log(
        request_id=7,
        request_uuid="uu",
        phase_label="APPROACH",
        frame_id="base_link",
        pose=(0.1, 0.2, 0.3),
        cartesian=False,
        ee_frame="rg2_pinch_center",
    )
    assert "[PICK][MOVEIT][TARGET]" in s
    assert "request_id=7" in s
    assert "phase=APPROACH" in s
    assert "cartesian=false" in s


# ---------------------------------------------------------------------------
# format_busy_message
# ---------------------------------------------------------------------------


def test_busy_message():
    s = format_busy_message(
        active_request_id=42,
        active_request_uuid="abc",
        active_age_sec=3.456,
    )
    assert s == "bridge_busy:active_request_id=42;active_request_uuid=abc;active_age=3.46s"


def test_busy_message_no_uuid():
    s = format_busy_message(
        active_request_id=42,
        active_request_uuid="",
        active_age_sec=0.0,
    )
    assert "active_request_uuid=n/a" in s


# ---------------------------------------------------------------------------
# compute_request_stamp_ns
# ---------------------------------------------------------------------------


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


def test_compute_request_stamp_ns_basic():
    assert compute_request_stamp_ns(_Stamp(10, 500_000_000)) == 10_500_000_000


def test_compute_request_stamp_ns_none():
    assert compute_request_stamp_ns(None) == 0


def test_compute_request_stamp_ns_zero():
    assert compute_request_stamp_ns(_Stamp(0, 0)) == 0


# ---------------------------------------------------------------------------
# is_invalid_business_frame
# ---------------------------------------------------------------------------


def test_invalid_business_frame_base_with_base_link():
    assert is_invalid_business_frame("base", "base_link") is True
    assert is_invalid_business_frame("/base", "base_link") is True


def test_valid_business_frame():
    assert is_invalid_business_frame("base_link", "base_link") is False
    assert is_invalid_business_frame("world", "base_link") is False


def test_invalid_business_frame_no_base_link():
    """Si bridge usa otro base, no se aplica la regla."""
    assert is_invalid_business_frame("base", "other") is False


# ---------------------------------------------------------------------------
# compute_settle_params
# ---------------------------------------------------------------------------


def _env_default(name, default):
    return float(default)


def test_settle_params_defaults():
    timeout, stable, tol = compute_settle_params(_env_default, 1.0)
    # default_timeout = min(1.5, max(0.4, 1.0)) = 1.0; max(0.4, 1.0) = 1.0
    assert timeout == pytest.approx(1.0)
    assert stable == pytest.approx(0.25)
    assert tol == pytest.approx(0.02)


def test_settle_params_clamp_low_timeout():
    """joint_state_valid_timeout < 0.4 → clamped a 0.4."""
    timeout, _, _ = compute_settle_params(_env_default, 0.1)
    assert timeout == pytest.approx(0.4)


def test_settle_params_clamp_high_timeout():
    """joint_state_valid_timeout > 1.5 → clamped a 1.5."""
    timeout, _, _ = compute_settle_params(_env_default, 5.0)
    assert timeout == pytest.approx(1.5)


def test_settle_params_env_overrides():
    def env(name, default):
        if "TIMEOUT" in name:
            return 2.0
        if "STABLE" in name:
            return 0.5
        if "TOL_RAD" in name:
            return 0.1
        return default
    timeout, stable, tol = compute_settle_params(env, 1.0)
    assert timeout == pytest.approx(2.0)
    assert stable == pytest.approx(0.5)
    assert tol == pytest.approx(0.1)


def test_settle_params_env_clamps_floor():
    """env values below floor → clamped to floor."""
    def env(name, default):
        return 0.0
    timeout, stable, tol = compute_settle_params(env, 1.0)
    assert timeout == pytest.approx(0.4)
    assert stable == pytest.approx(0.05)
    assert tol == pytest.approx(0.005)


# ---------------------------------------------------------------------------
# stale_request_age + is_stale_request
# ---------------------------------------------------------------------------


def test_stale_age_basic():
    assert stale_request_age(10.0, 15.5) == pytest.approx(5.5)


def test_stale_age_clamped_negative():
    """Si now < queued (clock skew) → 0."""
    assert stale_request_age(15.0, 10.0) == 0.0


def test_is_stale_no_ttl_disables():
    """ttl_sec <= 0 desactiva el check."""
    is_stale, age = is_stale_request(0.0, 100.0, 0.0)
    assert is_stale is False
    assert age == 100.0


def test_is_stale_above_threshold():
    is_stale, age = is_stale_request(0.0, 5.0, 2.0)
    assert is_stale is True
    assert age == pytest.approx(5.0)


def test_is_stale_below_threshold():
    is_stale, age = is_stale_request(0.0, 1.0, 2.0)
    assert is_stale is False
    assert age == pytest.approx(1.0)
