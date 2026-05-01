#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_orientation_helpers.py
# Contenido: F3 — tests offline de pick_object/orientation_helpers.
"""Tests offline de los helpers de orientación quaternion."""

from __future__ import annotations

import math

import pytest

from ur5_qt_panel.pick_object.orientation_helpers import (
    compose_grasp_orientation,
    normalize_quat,
    parse_orientation_xyzw,
    quat_multiply,
)


# ---------------------------------------------------------------------------
# normalize_quat
# ---------------------------------------------------------------------------


def test_normalize_quat_already_unit():
    q = normalize_quat((0.0, 0.0, 0.0, 1.0))
    assert q == (0.0, 0.0, 0.0, 1.0)


def test_normalize_quat_scales_to_unit():
    q = normalize_quat((2.0, 0.0, 0.0, 0.0))
    n = math.sqrt(sum(c * c for c in q))
    assert n == pytest.approx(1.0)


def test_normalize_quat_zero_returns_identity():
    assert normalize_quat((0.0, 0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0, 1.0)


def test_normalize_quat_invalid_input_returns_identity():
    assert normalize_quat(("a", "b", "c", "d")) == (0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# quat_multiply
# ---------------------------------------------------------------------------


def test_quat_multiply_identity_left():
    """(0,0,0,1) * q = q."""
    q = (0.5, 0.5, 0.5, 0.5)
    out = quat_multiply((0.0, 0.0, 0.0, 1.0), q)
    assert out == pytest.approx(q)


def test_quat_multiply_identity_right():
    """q * (0,0,0,1) = q."""
    q = (0.1, 0.2, 0.3, 0.4)
    out = quat_multiply(q, (0.0, 0.0, 0.0, 1.0))
    assert out == pytest.approx(q)


def test_quat_multiply_z_yaw_90_composition():
    """Aplicar yaw=π/2 alrededor de Z al identity da el yaw quat."""
    yaw_q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    out = quat_multiply(yaw_q, (0.0, 0.0, 0.0, 1.0))
    assert out == pytest.approx(yaw_q)


# ---------------------------------------------------------------------------
# parse_orientation_xyzw
# ---------------------------------------------------------------------------


def test_parse_orientation_default_when_empty():
    q = parse_orientation_xyzw("")
    n = math.sqrt(sum(c * c for c in q))
    assert n == pytest.approx(1.0)
    assert q[0] == pytest.approx(0.70710678, abs=1e-6)


def test_parse_orientation_csv_normalizes():
    q = parse_orientation_xyzw("2,0,0,0")
    n = math.sqrt(sum(c * c for c in q))
    assert n == pytest.approx(1.0)


def test_parse_orientation_invalid_uses_default():
    q = parse_orientation_xyzw("foo,bar,baz,qux")
    # Default es (0.70710678,0,0.70710678,0) normalizado.
    assert q[0] == pytest.approx(0.70710678, abs=1e-6)


def test_parse_orientation_wrong_arity_uses_default():
    q = parse_orientation_xyzw("1,0,0")
    assert q[0] == pytest.approx(0.70710678, abs=1e-6)


def test_parse_orientation_nan_uses_default():
    q = parse_orientation_xyzw("nan,0,0,1")
    assert q[0] == pytest.approx(0.70710678, abs=1e-6)


def test_parse_orientation_zero_norm_uses_default():
    q = parse_orientation_xyzw("0,0,0,0")
    assert q[0] == pytest.approx(0.70710678, abs=1e-6)


def test_parse_orientation_custom_default():
    q = parse_orientation_xyzw("", default=(0.0, 0.0, 0.0, 1.0))
    assert q == (0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# compose_grasp_orientation
# ---------------------------------------------------------------------------


def test_compose_no_yaw_returns_base():
    base = (0.0, 0.0, 0.0, 1.0)
    out = compose_grasp_orientation(base, None)
    assert out == base


def test_compose_use_yaw_false_returns_base():
    base = (0.0, 0.0, 0.0, 1.0)
    out = compose_grasp_orientation(base, 90.0, use_yaw=False)
    assert out == base


def test_compose_yaw_nan_returns_base():
    base = (0.0, 0.0, 0.0, 1.0)
    out = compose_grasp_orientation(base, float("nan"))
    assert out == base


def test_compose_yaw_inf_returns_base():
    base = (0.0, 0.0, 0.0, 1.0)
    out = compose_grasp_orientation(base, float("inf"))
    assert out == base


def test_compose_yaw_invalid_string_returns_base():
    base = (0.0, 0.0, 0.0, 1.0)
    out = compose_grasp_orientation(base, "not_a_number")  # type: ignore[arg-type]
    assert out == base


def test_compose_yaw_90_produces_unit_quat():
    base = (0.0, 0.0, 0.0, 1.0)
    out = compose_grasp_orientation(base, 90.0)
    n = math.sqrt(sum(c * c for c in out))
    assert n == pytest.approx(1.0)


def test_compose_yaw_180_changes_quat():
    base = (0.0, 0.0, 0.0, 1.0)
    out = compose_grasp_orientation(base, 180.0)
    assert out != base


def test_compose_normalizes_unnormalized_base():
    base = (2.0, 0.0, 0.0, 0.0)  # norma = 2
    out = compose_grasp_orientation(base, None)
    n = math.sqrt(sum(c * c for c in out))
    assert n == pytest.approx(1.0)
