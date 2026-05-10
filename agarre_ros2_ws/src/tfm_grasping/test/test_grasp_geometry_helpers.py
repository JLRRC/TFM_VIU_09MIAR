#!/usr/bin/env python3
"""F8 audit (2026-05-10): tests de grasp_geometry_helpers."""
from __future__ import annotations

import math

import pytest

from tfm_grasping.grasp_geometry_helpers import (
    clamp,
    compute_minor_axis_from_grasp_rect,
    compute_rg2_preopen_from_minor_width,
    normalize_angle,
)


def test_clamp_within_range() -> None:
    assert clamp(0.5, 0.0, 1.0) == 0.5


def test_clamp_below_lo() -> None:
    assert clamp(-1.0, 0.0, 1.0) == 0.0


def test_clamp_above_hi() -> None:
    assert clamp(2.0, 0.0, 1.0) == 1.0


@pytest.mark.parametrize("a,expected", [
    (0.0, 0.0),
    (math.pi, math.pi),
    (-math.pi, -math.pi),
    (3 * math.pi, math.pi),
    (-3 * math.pi, -math.pi),
    (math.pi + 0.1, -math.pi + 0.1),
])
def test_normalize_angle(a: float, expected: float) -> None:
    assert math.isclose(normalize_angle(a), expected, abs_tol=1e-9)


def test_minor_axis_w_smaller_uses_w_and_rotates() -> None:
    minor, theta = compute_minor_axis_from_grasp_rect(20.0, 50.0, theta_img=0.0)
    assert minor == 20.0
    # Cuando w < h, el eje de apertura es perpendicular al theta original.
    assert math.isclose(theta, math.pi / 2, abs_tol=1e-9)


def test_minor_axis_h_smaller_keeps_theta() -> None:
    minor, theta = compute_minor_axis_from_grasp_rect(50.0, 20.0, theta_img=0.5)
    assert minor == 20.0
    assert math.isclose(theta, 0.5, abs_tol=1e-9)


def test_rg2_preopen_clamped_to_min() -> None:
    pre, finger = compute_rg2_preopen_from_minor_width(0.0)
    assert pre == 0.015  # min_open_m default
    assert finger > 0.0


def test_rg2_preopen_clamped_to_max() -> None:
    pre, finger = compute_rg2_preopen_from_minor_width(0.5)  # > max
    assert pre == 0.110  # max_open_m default
    assert math.isclose(finger, 1.18, abs_tol=1e-3)


def test_rg2_preopen_linear_finger_command() -> None:
    minor = 0.05  # 50 mm
    pre, finger = compute_rg2_preopen_from_minor_width(minor, safety_margin_m=0.0)
    expected_finger = (minor / 0.110) * 1.18
    assert math.isclose(finger, expected_finger, abs_tol=1e-9)


def test_rg2_preopen_safety_margin_added() -> None:
    pre, _ = compute_rg2_preopen_from_minor_width(0.04, safety_margin_m=0.02)
    assert pre == 0.06
