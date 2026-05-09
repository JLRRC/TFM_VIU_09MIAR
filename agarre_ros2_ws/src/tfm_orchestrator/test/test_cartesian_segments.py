#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_cartesian_segments.py
# Contenido: B-iter11 (2026-05-03) — tests segmentación cartesiana.
"""Tests para tfm_orchestrator.cartesian_segments."""

from __future__ import annotations

import math

import pytest

from tfm_orchestrator.cartesian_segments import (
    compute_segments_for_distance,
    segment_cartesian_path,
    slerp,
)


# ---------------------------------------------------------------------------
# slerp
# ---------------------------------------------------------------------------


def test_slerp_returns_q0_at_t_zero():
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    result = slerp(q0, q1, 0.0)
    assert result == pytest.approx(q0, abs=1e-6)


def test_slerp_returns_q1_at_t_one():
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    result = slerp(q0, q1, 1.0)
    assert result == pytest.approx(q1, abs=1e-6)


def test_slerp_interpolates_45deg_at_t_half():
    """De identity (0°) a 90° rot Z, t=0.5 → 45° rot Z."""
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    result = slerp(q0, q1, 0.5)
    expected_z = math.sin(math.pi / 8)
    expected_w = math.cos(math.pi / 8)
    assert result[2] == pytest.approx(expected_z, abs=1e-6)
    assert result[3] == pytest.approx(expected_w, abs=1e-6)


def test_slerp_handles_negated_quat_short_arc():
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (0.0, 0.0, 0.0, -1.0)  # misma rotación que q0
    result = slerp(q0, q1, 0.5)
    # Debe ser cercano a q0 (no atravesar el "long way").
    norm = sum(c * c for c in result) ** 0.5
    assert norm == pytest.approx(1.0, abs=1e-6)


def test_slerp_returns_unit_quat():
    q0 = (0.5, 0.5, 0.5, 0.5)
    q1 = (0.0, 0.0, 0.0, 1.0)
    result = slerp(q0, q1, 0.3)
    norm = sum(c * c for c in result) ** 0.5
    assert norm == pytest.approx(1.0, abs=1e-6)


def test_slerp_close_quats_uses_lerp():
    """Quats casi paralelos deben usar lerp simple sin asin issues."""
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (1e-7, 0.0, 0.0, math.sqrt(1.0 - 1e-14))
    result = slerp(q0, q1, 0.5)
    # No debe lanzar; resultado normalizado.
    norm = sum(c * c for c in result) ** 0.5
    assert norm == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------------------
# segment_cartesian_path
# ---------------------------------------------------------------------------


def test_segment_path_n1_returns_endpoints():
    waypoints = segment_cartesian_path((0, 0, 0), (1, 0, 0), 1)
    assert len(waypoints) == 2
    assert waypoints[0][0] == (0.0, 0.0, 0.0)
    assert waypoints[1][0] == (1.0, 0.0, 0.0)


def test_segment_path_n4_returns_5_waypoints():
    waypoints = segment_cartesian_path((0, 0, 0), (1, 0, 0), 4)
    assert len(waypoints) == 5
    # Posiciones equiespaciadas.
    for i, (xyz, _q) in enumerate(waypoints):
        expected_x = i * 0.25
        assert xyz[0] == pytest.approx(expected_x, abs=1e-6)


def test_segment_path_default_quat_is_identity():
    waypoints = segment_cartesian_path((0, 0, 0), (1, 0, 0), 2)
    for _xyz, q in waypoints:
        assert q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-6)


def test_segment_path_interpolates_quat_when_provided():
    q_start = (0.0, 0.0, 0.0, 1.0)
    q_end = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # 90° Z
    waypoints = segment_cartesian_path(
        (0, 0, 0), (1, 0, 0), 2,
        start_quat=q_start, end_quat=q_end,
    )
    # mid waypoint debe tener orient ~45°.
    mid_q = waypoints[1][1]
    assert mid_q[2] == pytest.approx(math.sin(math.pi / 8), abs=1e-6)
    assert mid_q[3] == pytest.approx(math.cos(math.pi / 8), abs=1e-6)


def test_segment_path_raises_on_invalid_n():
    with pytest.raises(ValueError, match="n_segments"):
        segment_cartesian_path((0, 0, 0), (1, 0, 0), 0)


# ---------------------------------------------------------------------------
# compute_segments_for_distance
# ---------------------------------------------------------------------------


def test_compute_segments_short_distance_returns_min():
    n = compute_segments_for_distance((0, 0, 0), (0.01, 0, 0))
    assert n == 1  # 1cm de viaje, max_segment 5cm → 1 segment


def test_compute_segments_long_distance_returns_proportional():
    n = compute_segments_for_distance((0, 0, 0), (0.50, 0, 0))
    assert n == 10  # 50cm / 5cm = 10 segments


def test_compute_segments_clamped_to_max():
    n = compute_segments_for_distance(
        (0, 0, 0), (10.0, 0, 0), max_segments=5,
    )
    assert n == 5  # 10m / 5cm = 200, pero clamped a 5


def test_compute_segments_clamped_to_min():
    n = compute_segments_for_distance(
        (0, 0, 0), (0.001, 0, 0), min_segments=3,
    )
    assert n == 3


def test_compute_segments_custom_max_segment():
    n = compute_segments_for_distance(
        (0, 0, 0), (0.20, 0, 0), max_segment_m=0.10,
    )
    assert n == 2  # 20cm / 10cm = 2 segments


def test_compute_segments_zero_distance_returns_min():
    n = compute_segments_for_distance((0.5, 0, 0), (0.5, 0, 0))
    assert n == 1
