#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_tf_geometry_logic.py
# Contenido: F16 (2026-05-01) — tests offline de tf_geometry_logic.
"""Tests offline de ``tf_geometry_logic``.

F16 separa la lógica matemática del nodo ROS para que sea
verificable sin rclpy/tf2.
"""

from __future__ import annotations

import math

import pytest

from ur5_tools.tf_geometry_logic import (
    apply_world_to_base_transform,
    compute_approach_pose,
    invert_transform,
    is_unit_quat,
    normalize_quat,
    quat_norm,
    rotate_vector_by_quat,
)


# ---------------------------------------------------------------------------
# rotate_vector_by_quat
# ---------------------------------------------------------------------------


def test_rotate_by_identity_returns_same():
    v = (1.0, 2.0, 3.0)
    out = rotate_vector_by_quat(v, (0.0, 0.0, 0.0, 1.0))
    assert out[0] == pytest.approx(1.0)
    assert out[1] == pytest.approx(2.0)
    assert out[2] == pytest.approx(3.0)


def test_rotate_90_z_x_to_y():
    """Rotación 90° alrededor de Z lleva (1,0,0) a (0,1,0)."""
    s = math.sin(math.pi / 4)
    out = rotate_vector_by_quat((1.0, 0.0, 0.0), (0.0, 0.0, s, s))
    assert out[0] == pytest.approx(0.0, abs=1e-6)
    assert out[1] == pytest.approx(1.0, abs=1e-6)
    assert out[2] == pytest.approx(0.0, abs=1e-6)


def test_rotate_180_z_negates_xy():
    """Rotación 180° alrededor de Z niega X e Y."""
    out = rotate_vector_by_quat((1.0, 2.0, 3.0), (0.0, 0.0, 1.0, 0.0))
    assert out[0] == pytest.approx(-1.0)
    assert out[1] == pytest.approx(-2.0)
    assert out[2] == pytest.approx(3.0)


def test_rotate_90_x_y_to_z():
    """Rotación 90° alrededor de X lleva (0,1,0) a (0,0,1)."""
    s = math.sin(math.pi / 4)
    out = rotate_vector_by_quat((0.0, 1.0, 0.0), (s, 0.0, 0.0, s))
    assert out[0] == pytest.approx(0.0, abs=1e-6)
    assert out[1] == pytest.approx(0.0, abs=1e-6)
    assert out[2] == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------------------
# invert_transform
# ---------------------------------------------------------------------------


def test_invert_identity_is_identity():
    t = (0.0, 0.0, 0.0)
    q = (0.0, 0.0, 0.0, 1.0)
    ti, qi = invert_transform(t, q)
    assert ti == pytest.approx((0.0, 0.0, 0.0))
    assert qi == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_invert_pure_translation():
    """Inversa de pura traslación niega la traslación."""
    t = (1.0, 2.0, 3.0)
    q = (0.0, 0.0, 0.0, 1.0)
    ti, qi = invert_transform(t, q)
    assert ti == pytest.approx((-1.0, -2.0, -3.0))
    assert qi == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_invert_pure_rotation_negates_xyz_keeps_w():
    s = math.sin(math.pi / 4)
    q = (0.0, 0.0, s, s)  # 90° Z
    _, qi = invert_transform((0.0, 0.0, 0.0), q)
    assert qi[0] == pytest.approx(0.0)
    assert qi[1] == pytest.approx(0.0)
    assert qi[2] == pytest.approx(-s)
    assert qi[3] == pytest.approx(s)


# ---------------------------------------------------------------------------
# apply_world_to_base_transform
# ---------------------------------------------------------------------------


def test_world_to_base_identity_returns_negated_t():
    """Si la TF es identidad pura, world→base = -t (origen base en world)."""
    out = apply_world_to_base_transform(
        (1.0, 2.0, 3.0),
        t_world_base=(0.5, 0.0, 0.0),
        q_world_base=(0.0, 0.0, 0.0, 1.0),
    )
    # base_xyz = inv_q * world_xyz + inv_t
    # con q identidad: inv_q*v = v; inv_t = -t = (-0.5, 0, 0)
    assert out[0] == pytest.approx(0.5)
    assert out[1] == pytest.approx(2.0)
    assert out[2] == pytest.approx(3.0)


def test_world_to_base_origin_returns_neg_t():
    """El origen world (0,0,0) en base = -inv_t."""
    out = apply_world_to_base_transform(
        (0.0, 0.0, 0.0),
        t_world_base=(1.0, 2.0, 3.0),
        q_world_base=(0.0, 0.0, 0.0, 1.0),
    )
    assert out[0] == pytest.approx(-1.0)
    assert out[1] == pytest.approx(-2.0)
    assert out[2] == pytest.approx(-3.0)


# ---------------------------------------------------------------------------
# compute_approach_pose
# ---------------------------------------------------------------------------


def test_approach_pose_adds_clearance_in_z():
    obj = ((0.5, 0.0, 0.05), (0.0, 0.0, 0.0, 1.0))
    pos, quat = compute_approach_pose(obj, 0.10)
    assert pos == pytest.approx((0.5, 0.0, 0.15))
    assert quat == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_approach_pose_preserves_orientation():
    obj = ((1.0, 2.0, 3.0), (0.1, 0.2, 0.3, 0.9))
    _, quat = compute_approach_pose(obj, 0.05)
    assert quat == pytest.approx((0.1, 0.2, 0.3, 0.9))


def test_approach_pose_negative_clearance_clamped_to_zero():
    obj = ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))
    pos, _ = compute_approach_pose(obj, -0.5)
    assert pos == pytest.approx((0.0, 0.0, 1.0))


# ---------------------------------------------------------------------------
# quat helpers
# ---------------------------------------------------------------------------


def test_quat_norm_identity_is_one():
    assert quat_norm((0.0, 0.0, 0.0, 1.0)) == pytest.approx(1.0)


def test_is_unit_quat_with_tolerance():
    assert is_unit_quat((0.0, 0.0, 0.0, 1.0))
    assert is_unit_quat((0.0, 0.0, 0.0, 1.0001))
    assert not is_unit_quat((0.0, 0.0, 0.0, 1.5))


def test_normalize_quat_basic():
    out = normalize_quat((0.0, 0.0, 0.0, 2.0))
    assert out == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_normalize_quat_zero_returns_identity():
    out = normalize_quat((0.0, 0.0, 0.0, 0.0))
    assert out == (0.0, 0.0, 0.0, 1.0)
