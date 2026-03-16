#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_utils.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Unit tests for panel_utils helpers (no ROS required)."""

from ur5_qt_panel.panel_config import UR5_BASE_X, UR5_BASE_Y, UR5_BASE_Z
from ur5_qt_panel.panel_utils import (
    _apply_homography,
    _invert_3x3,
    _solve_linear_system,
    base_frame_candidates,
    compute_homography,
    world_to_base,
)


def test_base_frame_candidates_order_and_dedup():
    out = base_frame_candidates("base_link", "base_link", fallbacks=("base_link", "base"))
    assert out == ["base_link", "base"]
    out = base_frame_candidates("base_link", "base", fallbacks=("base_link", "base"))
    assert out == ["base_link", "base"]
    out = base_frame_candidates(None, "base", fallbacks=("base_link", "base"))
    assert out == ["base", "base_link"]


def test_world_to_base_uses_config_offsets():
    x, y, z = 1.23, -4.56, 7.89
    bx, by, bz = world_to_base(x, y, z)
    assert bx == x - UR5_BASE_X
    assert by == y - UR5_BASE_Y
    assert bz == z - UR5_BASE_Z


def test_compute_homography_identity_mapping():
    pixels = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    worlds = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    mat = compute_homography(pixels, worlds)
    assert mat is not None
    for u, v in pixels:
        out = _apply_homography(mat, u, v)
        assert out is not None
        x, y = out
        assert abs(x - u) < 1e-6
        assert abs(y - v) < 1e-6


def test_invert_3x3_round_trip():
    mat = [
        [1.0, 2.0, 3.0],
        [0.0, 1.0, 4.0],
        [5.0, 6.0, 0.0],
    ]
    inv = _invert_3x3(mat)
    assert inv is not None
    out = _apply_homography(inv, *_apply_homography(mat, 0.2, -0.4))
    assert out is not None
    x, y = out
    assert abs(x - 0.2) < 1e-6
    assert abs(y + 0.4) < 1e-6


def test_invert_3x3_singular_returns_none():
    mat = [
        [1.0, 2.0, 3.0],
        [2.0, 4.0, 6.0],
        [0.0, 0.0, 1.0],
    ]
    assert _invert_3x3(mat) is None


def test_apply_homography_denominator_zero():
    mat = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
    ]
    assert _apply_homography(mat, 1.0, 1.0) is None


def test_solve_linear_system_basic():
    a = [
        [2.0, 1.0],
        [1.0, 3.0],
    ]
    b = [5.0, 6.0]
    sol = _solve_linear_system(a, b)
    assert sol is not None
    x, y = sol
    assert abs((2.0 * x + 1.0 * y) - 5.0) < 1e-6
    assert abs((1.0 * x + 3.0 * y) - 6.0) < 1e-6
