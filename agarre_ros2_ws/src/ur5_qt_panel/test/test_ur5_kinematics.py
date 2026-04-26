#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_ur5_kinematics.py
"""Unit tests for ur5_kinematics — pure numpy/scipy, no ROS needed."""
from __future__ import annotations

import math

import numpy as np
import pytest

from ur5_qt_panel.ur5_kinematics import (
    fk_ur5,
    ik_ur5,
    rot_x,
    rot_z,
)


# ---------------------------------------------------------------------------
# rot_z / rot_x
# ---------------------------------------------------------------------------

def test_rot_z_identity_at_zero():
    R = rot_z(0.0)
    assert np.allclose(R, np.eye(3))


def test_rot_z_quarter_turn():
    R = rot_z(math.pi / 2)
    expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    assert np.allclose(R, expected, atol=1e-9)


def test_rot_z_half_turn():
    R = rot_z(math.pi)
    expected = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=float)
    assert np.allclose(R, expected, atol=1e-9)


def test_rot_z_is_orthogonal():
    R = rot_z(1.23)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)


def test_rot_z_determinant_is_one():
    R = rot_z(0.77)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-12)


def test_rot_x_identity_at_zero():
    R = rot_x(0.0)
    assert np.allclose(R, np.eye(3))


def test_rot_x_quarter_turn():
    R = rot_x(math.pi / 2)
    expected = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
    assert np.allclose(R, expected, atol=1e-9)


def test_rot_x_is_orthogonal():
    R = rot_x(0.55)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)


# ---------------------------------------------------------------------------
# fk_ur5
# ---------------------------------------------------------------------------

def test_fk_ur5_home_pose_returns_arrays():
    q = [0.0] * 6
    pos, rot = fk_ur5(q)
    assert pos.shape == (3,)
    assert rot.shape == (3, 3)


def test_fk_ur5_rotation_is_orthogonal():
    q = [0.1, -0.5, 0.8, -1.2, 0.3, 0.0]
    _, rot = fk_ur5(q)
    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-10)


def test_fk_ur5_rotation_determinant_is_one():
    q = [0.1, -0.5, 0.8, -1.2, 0.3, 0.0]
    _, rot = fk_ur5(q)
    assert np.linalg.det(rot) == pytest.approx(1.0, abs=1e-9)


def test_fk_ur5_raises_on_wrong_joint_count():
    with pytest.raises(ValueError, match="6"):
        fk_ur5([0.0, 0.0, 0.0])


def test_fk_ur5_home_position_plausible():
    # At q=0 the DH chain places the EE roughly in the XY plane of the base.
    # Accept anything within arm length and not absurdly below the base.
    pos, _ = fk_ur5([0.0] * 6)
    reach = float(np.linalg.norm(pos))
    assert 0.05 < reach < 1.2, f"Reach {reach:.4f} out of expected (0.05, 1.2)"
    assert float(pos[2]) > -0.10, f"EE Z={pos[2]:.4f} implausibly low"


def test_fk_ur5_elbow_up_reachable():
    # A known canonical UR5 pose (elbow up): EE roughly at (0.4, 0, 0.4)
    q = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
    pos, _ = fk_ur5(q)
    # EE should have non-trivial reach and be within robot arm length
    reach = float(np.linalg.norm(pos))
    assert 0.2 < reach < 1.2, f"Reach {reach:.4f} out of expected [0.2, 1.2]"


def test_fk_ur5_different_configs_give_different_positions():
    pos1, _ = fk_ur5([0.0] * 6)
    pos2, _ = fk_ur5([0.1, -0.5, 0.8, -1.2, 0.3, 0.0])
    assert not np.allclose(pos1, pos2)


# ---------------------------------------------------------------------------
# ik_ur5 — FK/IK round-trip
# ---------------------------------------------------------------------------

def test_ik_ur5_roundtrip_position():
    """IK should recover a position reachable by FK to within 5mm."""
    q_ref = [0.1, -1.2, 0.9, -0.8, 0.5, 0.3]
    pos_ref, rot_ref = fk_ur5(q_ref)
    q_init = [0.0, -1.0, 0.8, -0.5, 0.3, 0.0]
    q_sol, err, ok = ik_ur5(pos_ref, rot_ref, q_init, max_iter=500)
    pos_sol, _ = fk_ur5(q_sol)
    pos_err = float(np.linalg.norm(pos_sol - pos_ref))
    assert pos_err < 0.005, f"IK position error {pos_err*1000:.2f}mm > 5mm"


def test_ik_ur5_returns_six_joints():
    q_ref = [0.0, -1.0, 0.8, -0.5, 0.3, 0.0]
    pos, rot = fk_ur5(q_ref)
    q_sol, _, _ = ik_ur5(pos, rot, q_ref, max_iter=100)
    assert q_sol.shape == (6,)


def test_ik_ur5_raises_on_wrong_rot_shape():
    pos = np.array([0.3, 0.1, 0.4])
    with pytest.raises(ValueError, match="3x3"):
        ik_ur5(pos, np.eye(4), [0.0] * 6)


def test_ik_ur5_raises_on_wrong_q0_size():
    pos = np.array([0.3, 0.1, 0.4])
    with pytest.raises(ValueError, match="6"):
        ik_ur5(pos, np.eye(3), [0.0, 0.0, 0.0])


def test_ik_ur5_err_norm_is_non_negative():
    q_ref = [0.0, -1.0, 0.8, -0.5, 0.3, 0.0]
    pos, rot = fk_ur5(q_ref)
    _, err, _ = ik_ur5(pos, rot, q_ref, max_iter=100)
    assert err >= 0.0


def test_ik_ur5_joint_weight_activates_wrap_to_pi_branch():
    # joint_weight > 0.0 activates the _wrap_to_pi path (lines 70, 78-79)
    q_ref = [0.0, -1.0, 0.8, -0.5, 0.3, 0.0]
    pos, rot = fk_ur5(q_ref)
    q_sol, err, _ = ik_ur5(pos, rot, q_ref, max_iter=200, joint_weight=0.1)
    assert err >= 0.0
    assert len(q_sol) == 6
