#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_demo_geometry.py
# Contenido: F3 — tests offline de pick_demo/geometry.py.
"""Tests offline de funciones geométricas puras pick_demo."""

from __future__ import annotations

import math

import pytest

from ur5_qt_panel.pick_demo.geometry import (
    apply_local_offset_to_fk,
    dynamic_pre_close_reference,
    object_top_pose,
    vec_dist3,
)


# ---------------------------------------------------------------------------
# vec_dist3
# ---------------------------------------------------------------------------


def test_vec_dist3_zero():
    assert vec_dist3((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)) == 0.0


def test_vec_dist3_axis_aligned():
    assert vec_dist3((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == pytest.approx(5.0)


def test_vec_dist3_arbitrary():
    assert vec_dist3((1.0, 1.0, 1.0), (2.0, 3.0, 5.0)) == pytest.approx(
        math.sqrt(1 + 4 + 16)
    )


# ---------------------------------------------------------------------------
# object_top_pose
# ---------------------------------------------------------------------------


def test_object_top_pose_none():
    assert object_top_pose(None, 0.05) is None


def test_object_top_pose_invalid():
    assert object_top_pose(("a", "b", "c"), 0.05) is None


def test_object_top_pose_eleva_z_por_media_altura():
    out = object_top_pose((0.5, 0.4, 0.0), 0.05)
    assert out == pytest.approx((0.5, 0.4, 0.025))


def test_object_top_pose_grande():
    out = object_top_pose((0.0, 0.0, 1.0), 0.20)
    assert out[2] == pytest.approx(1.10)


# ---------------------------------------------------------------------------
# dynamic_pre_close_reference
# ---------------------------------------------------------------------------


def test_dynamic_pre_close_none_obj():
    assert dynamic_pre_close_reference(None, grasp_z=0.020, extra_z_m=0.10) is None


def test_dynamic_pre_close_basic():
    out = dynamic_pre_close_reference((0.5, 0.5, 0.0), grasp_z=0.020, extra_z_m=0.10)
    assert out == pytest.approx((0.5, 0.5, 0.120))


def test_dynamic_pre_close_clamps_negative_extra():
    out = dynamic_pre_close_reference((0.5, 0.5, 0.0), grasp_z=0.020, extra_z_m=-0.5)
    assert out == pytest.approx((0.5, 0.5, 0.020))


def test_dynamic_pre_close_keeps_xy():
    out = dynamic_pre_close_reference((0.123, -0.456, 0.789), grasp_z=0.0, extra_z_m=0.0)
    assert out == pytest.approx((0.123, -0.456, 0.789))


# ---------------------------------------------------------------------------
# apply_local_offset_to_fk
# ---------------------------------------------------------------------------


_IDENTITY_ROT = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
]


def test_offset_zero_with_flip():
    """fk=(1,2,3), rot=I, offset=(0,0,0), flip=True → (-1, -2, 3)."""
    out = apply_local_offset_to_fk((1.0, 2.0, 3.0), _IDENTITY_ROT, (0.0, 0.0, 0.0))
    assert out == pytest.approx((-1.0, -2.0, 3.0))


def test_offset_zero_no_flip():
    out = apply_local_offset_to_fk(
        (1.0, 2.0, 3.0), _IDENTITY_ROT, (0.0, 0.0, 0.0), flip_xy=False
    )
    assert out == pytest.approx((1.0, 2.0, 3.0))


def test_offset_identity_rot_with_offset_z():
    """fk=(0,0,0), rot=I, offset=(0,0,0.1), flip=True → (0, 0, 0.1)."""
    out = apply_local_offset_to_fk(
        (0.0, 0.0, 0.0), _IDENTITY_ROT, (0.0, 0.0, 0.1)
    )
    assert out == pytest.approx((0.0, 0.0, 0.1))


def test_offset_z_axis_rot_90():
    """rot 90° alrededor de Z: x_local → -y_world."""
    rot_z_90 = [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    # offset local (0.1, 0, 0) tras rot Z90 → (0, 0.1, 0) en frame fk
    # luego flip xy → (0, -0.1, 0)
    out = apply_local_offset_to_fk(
        (0.0, 0.0, 0.0), rot_z_90, (0.1, 0.0, 0.0)
    )
    assert out == pytest.approx((0.0, -0.1, 0.0))


def test_offset_with_translation_and_offset_combined():
    out = apply_local_offset_to_fk(
        (1.0, 0.5, 0.3), _IDENTITY_ROT, (0.0, 0.0, 0.05), flip_xy=False
    )
    assert out == pytest.approx((1.0, 0.5, 0.35))
