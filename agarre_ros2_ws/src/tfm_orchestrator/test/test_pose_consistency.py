#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_pose_consistency.py
# Contenido: B-iter10 (2026-05-03) — tests gates pose consistency.
"""Tests para tfm_orchestrator.pose_consistency."""

from __future__ import annotations

import math

import pytest

from tfm_orchestrator.pose_consistency import (
    DEFAULT_MAX_AGE_SEC,
    DEFAULT_MAX_NORM_M,
    DEFAULT_MAX_ORIENT_RAD,
    compute_orientation_angle_diff,
    compute_position_norm,
    evaluate_pose_divergence_gate,
    evaluate_pose_freshness_gate,
)


# ---------------------------------------------------------------------------
# evaluate_pose_freshness_gate
# ---------------------------------------------------------------------------


def test_freshness_ok_when_both_fresh():
    ok, reason = evaluate_pose_freshness_gate(0.05, 0.10)
    assert ok is True
    assert "ok" in reason


def test_freshness_ok_at_exact_limit():
    ok, reason = evaluate_pose_freshness_gate(
        DEFAULT_MAX_AGE_SEC, DEFAULT_MAX_AGE_SEC,
    )
    assert ok is True


def test_freshness_fail_when_tcp_tf_stale():
    ok, reason = evaluate_pose_freshness_gate(0.50, 0.05)
    assert ok is False
    assert "tcp_tf:stale" in reason


def test_freshness_fail_when_joint_state_stale():
    ok, reason = evaluate_pose_freshness_gate(0.05, 0.50)
    assert ok is False
    assert "joint_state:stale" in reason


def test_freshness_fail_when_both_stale():
    ok, reason = evaluate_pose_freshness_gate(1.0, 2.0)
    assert ok is False
    assert "tcp_tf:stale" in reason
    assert "joint_state:stale" in reason


def test_freshness_fail_when_tcp_tf_none():
    ok, reason = evaluate_pose_freshness_gate(None, 0.05)
    assert ok is False
    assert "tcp_tf:unmeasured" in reason


def test_freshness_fail_when_joint_state_none():
    ok, reason = evaluate_pose_freshness_gate(0.05, None)
    assert ok is False
    assert "joint_state:unmeasured" in reason


def test_freshness_custom_max_age():
    # default 0.20s → 0.30s sería stale; custom 0.5s → ok.
    ok, reason = evaluate_pose_freshness_gate(0.30, 0.30, max_age_sec=0.5)
    assert ok is True


def test_freshness_invalid_value_treated_as_fail():
    ok, reason = evaluate_pose_freshness_gate("bad", 0.05)  # type: ignore[arg-type]
    assert ok is False
    assert "tcp_tf:invalid" in reason


# ---------------------------------------------------------------------------
# compute_position_norm
# ---------------------------------------------------------------------------


def test_compute_position_norm_zero_same_pose():
    p = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    assert compute_position_norm(p, p) == pytest.approx(0.0)


def test_compute_position_norm_simple_case():
    p1 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    p2 = (3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert compute_position_norm(p1, p2) == pytest.approx(5.0)


def test_compute_position_norm_returns_none_for_none():
    p = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert compute_position_norm(None, p) is None
    assert compute_position_norm(p, None) is None


# ---------------------------------------------------------------------------
# compute_orientation_angle_diff
# ---------------------------------------------------------------------------


def test_orient_diff_zero_for_identical_quat():
    q = (0.0, 0.0, 0.0, 1.0)
    assert compute_orientation_angle_diff(q, q) == pytest.approx(0.0)


def test_orient_diff_zero_for_negated_quat_same_rotation():
    """q y -q representan la misma rotación → angle 0."""
    q1 = (0.0, 0.0, 0.0, 1.0)
    q2 = (0.0, 0.0, 0.0, -1.0)
    assert compute_orientation_angle_diff(q1, q2) == pytest.approx(0.0)


def test_orient_diff_pi_for_opposite_rotation():
    """Rotación 180° alrededor de Z: q = (0,0,1,0)."""
    q1 = (0.0, 0.0, 0.0, 1.0)  # identity
    q2 = (0.0, 0.0, 1.0, 0.0)  # 180° rot around Z
    angle = compute_orientation_angle_diff(q1, q2)
    assert angle == pytest.approx(math.pi)


def test_orient_diff_returns_none_for_none():
    q = (0.0, 0.0, 0.0, 1.0)
    assert compute_orientation_angle_diff(None, q) is None
    assert compute_orientation_angle_diff(q, None) is None


# ---------------------------------------------------------------------------
# evaluate_pose_divergence_gate
# ---------------------------------------------------------------------------


def test_divergence_gate_ok_for_same_pose():
    p = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    ok, reason = evaluate_pose_divergence_gate(p, p)
    assert ok is True
    assert "ok" in reason


def test_divergence_gate_ok_within_position_tolerance():
    p1 = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    # 0.005m posición, mismo orient
    p2 = (1.005, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    ok, reason = evaluate_pose_divergence_gate(p1, p2)
    assert ok is True


def test_divergence_gate_fail_when_position_too_far():
    p1 = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    p2 = (1.5, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    ok, reason = evaluate_pose_divergence_gate(p1, p2)
    assert ok is False
    assert "fail" in reason
    assert "pos=" in reason


def test_divergence_gate_fail_when_orientation_too_far():
    p1 = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    # 90° rot around Z, mismo position
    p2 = (1.0, 2.0, 3.0, 0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    ok, reason = evaluate_pose_divergence_gate(p1, p2)
    assert ok is False
    assert "fail" in reason
    assert "orient=" in reason


def test_divergence_gate_returns_fail_for_none():
    p = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    ok, reason = evaluate_pose_divergence_gate(None, p)
    assert ok is False
    assert "none" in reason


def test_divergence_gate_custom_tolerances():
    p1 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    p2 = (0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    # Default max_norm 0.006 → fail
    ok, _ = evaluate_pose_divergence_gate(p1, p2)
    assert ok is False
    # Custom max 0.10 → ok
    ok, _ = evaluate_pose_divergence_gate(p1, p2, max_norm_m=0.10)
    assert ok is True


def test_divergence_gate_position_only_zero_orient_diff():
    """Verifica formato del reason cuando solo posición falla."""
    p1 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    p2 = (0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    ok, reason = evaluate_pose_divergence_gate(p1, p2)
    assert ok is False
    assert "pos=0.5000m>" in reason
    assert "orient=0.0000rad<=" in reason
