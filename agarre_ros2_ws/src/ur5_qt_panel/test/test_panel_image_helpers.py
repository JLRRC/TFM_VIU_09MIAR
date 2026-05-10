#!/usr/bin/env python3
"""F9 (auditoría 2026-05-10): tests offline de panel_image_helpers.

Cobertura de los 4 helpers puros extraídos del worker ROS:
percentile/min-max ranges, normalización uint8, colorización TURBO,
resize con aspect.
"""
from __future__ import annotations

import numpy as np
import pytest

from ur5_qt_panel.panel_image_helpers import (
    colorize_depth_bgr,
    compute_depth_normalization_range,
    normalize_depth_uint8,
    resize_image_max_dim,
)


# ----------------- compute_depth_normalization_range -----------------


def test_range_percentile_default():
    """Para array uniforme [1..100], percentil 1 ≈ 2.0, percentil 99 ≈ 99.0."""
    arr = np.arange(1, 101, dtype="float32").reshape(10, 10)
    lo, hi = compute_depth_normalization_range(arr)
    assert lo == pytest.approx(1.99, abs=0.5)
    assert hi == pytest.approx(99.01, abs=0.5)


def test_range_fast_uses_min_max():
    arr = np.arange(1, 101, dtype="float32").reshape(10, 10)
    lo, hi = compute_depth_normalization_range(arr, fast=True)
    assert lo == 1.0
    assert hi == 100.0


def test_range_excludes_non_positive():
    """Pixels <= 0 se descartan (típico para 'no medición')."""
    arr = np.array([[0.0, -1.0, 5.0], [10.0, 0.0, 20.0]], dtype="float32")
    lo, hi = compute_depth_normalization_range(arr, fast=True)
    assert lo == 5.0
    assert hi == 20.0


def test_range_returns_none_when_no_valid_pixels():
    arr = np.zeros((5, 5), dtype="float32")
    assert compute_depth_normalization_range(arr) is None


def test_range_returns_none_when_all_negative():
    arr = -np.ones((5, 5), dtype="float32")
    assert compute_depth_normalization_range(arr) is None


def test_range_stride_subsamples():
    """Con stride alto, computamos sobre menos pixels (subsample del array)."""
    arr = np.arange(1, 401, dtype="float32").reshape(20, 20)
    # stride=2 reduce muestra a 10x10 (índices pares 0,2,...,18 en cada dim)
    # Min: arr[0,0] = 1.0; Max: arr[18,18] = 1 + 18*20 + 18 = 379.0
    lo_s, hi_s = compute_depth_normalization_range(arr, fast=True, stride=2)
    assert lo_s == 1.0
    assert hi_s == 379.0


def test_range_none_input():
    assert compute_depth_normalization_range(None) is None


# ------------------------ normalize_depth_uint8 ----------------------


def test_normalize_basic():
    arr = np.array([[0.0, 5.0, 10.0]], dtype="float32")
    out = normalize_depth_uint8(arr, 0.0, 10.0)
    assert out.dtype.name == "uint8"
    assert out[0, 0] == 0
    assert out[0, 2] == 255


def test_normalize_clamps_below_lo():
    arr = np.array([[-5.0, 0.0, 5.0]], dtype="float32")
    out = normalize_depth_uint8(arr, 0.0, 5.0)
    assert out[0, 0] == 0  # negativo → clamp inferior


def test_normalize_clamps_above_hi():
    arr = np.array([[0.0, 5.0, 100.0]], dtype="float32")
    out = normalize_depth_uint8(arr, 0.0, 5.0)
    assert out[0, 2] == 255  # supera hi → clamp superior


def test_normalize_handles_degenerate_range():
    """hi <= lo → debe usar lo + 1e-3 sin crashear."""
    arr = np.array([[1.0, 2.0]], dtype="float32")
    out = normalize_depth_uint8(arr, 5.0, 5.0)  # rango cero
    assert out.dtype.name == "uint8"
    assert out.shape == (1, 2)


def test_normalize_preserves_shape():
    arr = np.random.RandomState(42).rand(7, 11).astype("float32")
    out = normalize_depth_uint8(arr, 0.0, 1.0)
    assert out.shape == (7, 11)


# ------------------------- colorize_depth_bgr ------------------------


def test_colorize_returns_bgr_3channel():
    arr = np.linspace(0.0, 10.0, 64).reshape(8, 8).astype("float32")
    bgr = colorize_depth_bgr(arr, 0.0, 10.0)
    assert bgr.shape == (8, 8, 3)
    assert bgr.dtype.name == "uint8"


def test_colorize_extreme_values_distinct_colors():
    """El min y el max del rango deben dar colores distintos en TURBO."""
    arr = np.array([[0.0, 10.0]], dtype="float32")
    bgr = colorize_depth_bgr(arr, 0.0, 10.0)
    assert not np.array_equal(bgr[0, 0], bgr[0, 1])


# ------------------------ resize_image_max_dim -----------------------


def test_resize_no_op_when_within_max():
    bgr = np.zeros((100, 200, 3), dtype="uint8")
    out = resize_image_max_dim(bgr, max_size=300)
    assert out.shape == bgr.shape
    # Mismo objeto (no copia)
    assert out is bgr


def test_resize_scales_down_keeping_aspect():
    bgr = np.zeros((400, 800, 3), dtype="uint8")
    out = resize_image_max_dim(bgr, max_size=200)
    # max dim debería ser 200 ahora
    assert max(out.shape[:2]) == 200
    # Aspect preservado: 400/800 = 0.5; out ratio 100/200 = 0.5
    assert out.shape[:2] == (100, 200)


def test_resize_zero_max_size_no_op():
    bgr = np.zeros((100, 100, 3), dtype="uint8")
    assert resize_image_max_dim(bgr, max_size=0) is bgr


def test_resize_negative_max_size_no_op():
    bgr = np.zeros((100, 100, 3), dtype="uint8")
    assert resize_image_max_dim(bgr, max_size=-1) is bgr


def test_resize_none_input_returns_none():
    assert resize_image_max_dim(None, max_size=100) is None


def test_resize_handles_extreme_aspect():
    """Imagen muy alargada: preservar aspect sin colapsar a 0."""
    bgr = np.zeros((10, 1000, 3), dtype="uint8")
    out = resize_image_max_dim(bgr, max_size=100)
    h, w = out.shape[:2]
    assert w == 100
    assert h >= 1  # nunca cero
