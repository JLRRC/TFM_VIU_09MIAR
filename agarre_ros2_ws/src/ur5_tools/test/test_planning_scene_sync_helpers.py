#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_planning_scene_sync_helpers.py
# Contenido: F4 T36 — tests de helpers puros del planning_scene_sync.
"""F4 T36 — Tests de planning_scene_sync_helpers.

Cubre los helpers matemáticos extraídos del nodo PlanningSceneSync:
  * strip_ns (XML namespace)
  * quat_from_rpy / quat_multiply / quat_conjugate / rotate_vector
  * compose_pose (parent ∘ child)
  * parse_pose_text (formato SDF)

Tests offline puros — no requieren ROS sourced ni geometry_msgs.
"""
from __future__ import annotations

import math

import pytest

from ur5_tools.planning_scene_sync_helpers import (
    compose_pose,
    parse_pose_text,
    quat_conjugate,
    quat_from_rpy,
    quat_multiply,
    rotate_vector,
    strip_ns,
)


# ---------------------------------------------------------------------------
# strip_ns
# ---------------------------------------------------------------------------


def test_strip_ns_removes_namespace() -> None:
    assert strip_ns("{http://sdf}model") == "model"


def test_strip_ns_passthrough_no_namespace() -> None:
    assert strip_ns("model") == "model"


def test_strip_ns_empty() -> None:
    assert strip_ns("") == ""


def test_strip_ns_only_namespace() -> None:
    assert strip_ns("{ns}") == ""


# ---------------------------------------------------------------------------
# quat_from_rpy
# ---------------------------------------------------------------------------


def test_quat_from_rpy_identity() -> None:
    """RPY=(0,0,0) → identity quaternion (0,0,0,1)."""
    q = quat_from_rpy(0.0, 0.0, 0.0)
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-9)


def test_quat_from_rpy_yaw_only() -> None:
    """yaw=π → 180° around Z: (0, 0, 1, 0)."""
    q = quat_from_rpy(0.0, 0.0, math.pi)
    assert q == pytest.approx((0.0, 0.0, 1.0, 0.0), abs=1e-9)


def test_quat_from_rpy_roll_only() -> None:
    """roll=π → 180° around X: (1, 0, 0, 0)."""
    q = quat_from_rpy(math.pi, 0.0, 0.0)
    assert q == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)


def test_quat_from_rpy_unit_norm() -> None:
    """Para cualquier RPY, |q|=1."""
    for rpy in [(0.1, 0.2, 0.3), (1.0, -0.5, 2.7), (math.pi / 4, math.pi / 3, -math.pi)]:
        q = quat_from_rpy(*rpy)
        norm = math.sqrt(sum(c * c for c in q))
        assert norm == pytest.approx(1.0, abs=1e-9), f"rpy={rpy} q={q}"


# ---------------------------------------------------------------------------
# quat_multiply
# ---------------------------------------------------------------------------


def test_quat_multiply_identity_is_neutral() -> None:
    identity = (0.0, 0.0, 0.0, 1.0)
    q = (0.5, 0.3, 0.1, math.sqrt(0.65))
    assert quat_multiply(identity, q) == pytest.approx(q, abs=1e-9)
    assert quat_multiply(q, identity) == pytest.approx(q, abs=1e-9)


def test_quat_multiply_90x_then_90x_equals_180x() -> None:
    """90° around X then 90° around X == 180° around X."""
    q90x = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
    result = quat_multiply(q90x, q90x)
    expected = (1.0, 0.0, 0.0, 0.0)
    assert result == pytest.approx(expected, abs=1e-9)


# ---------------------------------------------------------------------------
# quat_conjugate
# ---------------------------------------------------------------------------


def test_quat_conjugate_negates_xyz() -> None:
    q = (0.5, -0.3, 0.7, 0.41)
    assert quat_conjugate(q) == (-0.5, 0.3, -0.7, 0.41)


def test_quat_conjugate_of_identity_is_identity() -> None:
    assert quat_conjugate((0.0, 0.0, 0.0, 1.0)) == (0.0, 0.0, 0.0, 1.0)


def test_quat_times_conjugate_is_identity() -> None:
    """q ⊗ q* = identity (para q unit)."""
    q = quat_from_rpy(0.5, -0.3, 1.2)
    result = quat_multiply(q, quat_conjugate(q))
    assert result == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-9)


# ---------------------------------------------------------------------------
# rotate_vector
# ---------------------------------------------------------------------------


def test_rotate_vector_identity_passthrough() -> None:
    """Identity quaternion no rota el vector."""
    v = (1.0, 2.0, 3.0)
    assert rotate_vector((0.0, 0.0, 0.0, 1.0), v) == pytest.approx(v, abs=1e-9)


def test_rotate_vector_90z_rotates_x_to_y() -> None:
    """90° around Z: X-world (1,0,0) → Y-world (0,1,0)."""
    q = quat_from_rpy(0.0, 0.0, math.pi / 2)
    rotated = rotate_vector(q, (1.0, 0.0, 0.0))
    assert rotated == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


def test_rotate_vector_180x_flips_yz() -> None:
    """180° around X: Y → -Y, Z → -Z, X stays."""
    q = (1.0, 0.0, 0.0, 0.0)
    assert rotate_vector(q, (1.0, 2.0, 3.0)) == pytest.approx(
        (1.0, -2.0, -3.0), abs=1e-9
    )


# ---------------------------------------------------------------------------
# compose_pose
# ---------------------------------------------------------------------------


def test_compose_pose_identity_is_neutral() -> None:
    identity = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    pose = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    assert compose_pose(identity, pose) == pytest.approx(pose, abs=1e-9)
    assert compose_pose(pose, identity) == pytest.approx(pose, abs=1e-9)


def test_compose_pose_translates_child_by_parent() -> None:
    """Parent en (5, 0, 0) sin rotación + child (1, 2, 3) → (6, 2, 3)."""
    parent = (5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    child = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    composed = compose_pose(parent, child)
    assert composed[:3] == pytest.approx((6.0, 2.0, 3.0), abs=1e-9)


def test_compose_pose_with_parent_rotation_rotates_child() -> None:
    """Parent rotado 90°Z + child en (1,0,0) → child en (0,1,0) en parent frame."""
    parent_q = quat_from_rpy(0.0, 0.0, math.pi / 2)
    parent = (0.0, 0.0, 0.0) + parent_q
    child = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    composed = compose_pose(parent, child)
    assert composed[:3] == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


# ---------------------------------------------------------------------------
# parse_pose_text
# ---------------------------------------------------------------------------


def test_parse_pose_text_empty_returns_neutral() -> None:
    assert parse_pose_text("") == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_parse_pose_text_short_returns_neutral() -> None:
    """Si tiene < 6 valores, fail-soft → neutra."""
    assert parse_pose_text("1.0 2.0 3.0") == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_parse_pose_text_xyz_zero_rpy() -> None:
    """xyz only with rpy=0 → identity orientation."""
    result = parse_pose_text("1.5 -2.5 3.5 0 0 0")
    assert result[:3] == pytest.approx((1.5, -2.5, 3.5), abs=1e-9)
    assert result[3:] == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-9)


def test_parse_pose_text_with_yaw() -> None:
    """xyz + yaw=π → quat (0, 0, 1, 0)."""
    result = parse_pose_text(f"1.0 2.0 3.0 0 0 {math.pi}")
    assert result[:3] == pytest.approx((1.0, 2.0, 3.0), abs=1e-9)
    assert result[3:] == pytest.approx((0.0, 0.0, 1.0, 0.0), abs=1e-9)


def test_parse_pose_text_invalid_returns_neutral() -> None:
    """Valores no numéricos → neutra."""
    assert parse_pose_text("a b c d e f") == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_parse_pose_text_extra_values_ignored() -> None:
    """Si hay > 6 valores, sólo se usan los 6 primeros."""
    result = parse_pose_text("1 2 3 0 0 0 EXTRA EXTRA")
    assert result == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
