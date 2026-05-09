"""Tests de la IoU con rectángulos orientados.

Casos de referencia con valores exactos verificables analíticamente:
- Rectángulo idéntico a sí mismo → IoU = 1.0
- Sin solapamiento → IoU = 0.0
- Dos cuadrados concéntricos de igual tamaño a 45° → IoU calculable exactamente
- Degenerado (dimensión 0) → IoU = 0.0
- Comparación con IoU axis-aligned a θ=0 (deben coincidir)
"""

from __future__ import annotations

import sys
from pathlib import Path
import math
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.training.metrics import (
    iou_oriented_boxes,
    iou_axis_aligned_boxes,
    cornell_success_oriented,
    _rect_vertices,
    _polygon_area,
    _sutherland_hodgman,
)


def _box(cx, cy, w, h, a=0.0):
    return np.array([[cx, cy, w, h, a]], dtype=np.float64)


class TestRectVertices:
    def test_zero_angle_corners(self):
        verts = _rect_vertices(10.0, 10.0, 4.0, 2.0, 0.0)
        assert verts.shape == (4, 2)
        xs = sorted(verts[:, 0])
        ys = sorted(verts[:, 1])
        assert abs(xs[0] - 8.0) < 1e-9
        assert abs(xs[-1] - 12.0) < 1e-9
        assert abs(ys[0] - 9.0) < 1e-9
        assert abs(ys[-1] - 11.0) < 1e-9

    def test_90deg_swaps_axes(self):
        # Rotar 90° un rectángulo w=4, h=2: los ejes se intercambian
        verts = _rect_vertices(0.0, 0.0, 4.0, 2.0, 90.0)
        xs_range = verts[:, 0].max() - verts[:, 0].min()
        ys_range = verts[:, 1].max() - verts[:, 1].min()
        assert abs(xs_range - 2.0) < 1e-9, f"expected x-range 2, got {xs_range}"
        assert abs(ys_range - 4.0) < 1e-9, f"expected y-range 4, got {ys_range}"


class TestPolygonArea:
    def test_unit_square(self):
        verts = [[0, 0], [1, 0], [1, 1], [0, 1]]
        assert abs(_polygon_area(verts) - 1.0) < 1e-9

    def test_triangle(self):
        verts = [[0, 0], [4, 0], [0, 3]]
        assert abs(_polygon_area(verts) - 6.0) < 1e-9

    def test_degenerate_line(self):
        verts = [[0, 0], [1, 0], [2, 0]]
        assert _polygon_area(verts) == 0.0

    def test_empty(self):
        assert _polygon_area([]) == 0.0
        assert _polygon_area([[0, 0]]) == 0.0


class TestOrientedIoU:
    def test_identical_rectangle_iou_one(self):
        box = _box(50, 50, 40, 20, 30.0)
        iou = iou_oriented_boxes(box, box)
        assert abs(iou[0] - 1.0) < 1e-6, f"IoU idéntico debe ser 1.0, got {iou[0]}"

    def test_identical_angle_zero(self):
        box = _box(0, 0, 10, 5, 0.0)
        iou = iou_oriented_boxes(box, box)
        assert abs(iou[0] - 1.0) < 1e-6

    def test_no_overlap_returns_zero(self):
        a = _box(0, 0, 10, 5, 0.0)
        b = _box(100, 100, 10, 5, 0.0)
        iou = iou_oriented_boxes(a, b)
        assert iou[0] == pytest.approx(0.0, abs=1e-9)

    def test_touching_edge_zero_overlap(self):
        # Dos rectángulos que se tocan por un lado — intersección vacía
        a = _box(0, 0, 10, 4, 0.0)   # x ∈ [-5, 5]
        b = _box(10, 0, 10, 4, 0.0)  # x ∈ [5, 15]
        iou = iou_oriented_boxes(a, b)
        assert iou[0] == pytest.approx(0.0, abs=1e-6)

    def test_axis_aligned_angle_zero_matches_aa_iou(self):
        # Con θ=0, la IoU orientada debe coincidir con la axis-aligned.
        for _ in range(10):
            rng = np.random.default_rng(42)
            cx1, cy1 = rng.uniform(20, 80, 2)
            cx2, cy2 = rng.uniform(20, 80, 2)
            w1, h1 = rng.uniform(10, 50, 2)
            w2, h2 = rng.uniform(10, 50, 2)
            pred = np.array([[cx1, cy1, w1, h1, 0.0]])
            gt = np.array([[cx2, cy2, w2, h2, 0.0]])
            iou_o = iou_oriented_boxes(pred, gt)[0]
            iou_a = iou_axis_aligned_boxes(pred, gt)[0]
            assert abs(iou_o - iou_a) < 1e-4, (
                f"IoU orientada {iou_o:.6f} ≠ axis-aligned {iou_a:.6f}"
            )

    def test_90deg_rotation_symmetric_rectangle(self):
        # Cuadrado concéntrico: la IoU a 90° debe ser 1.0 (cuadrado)
        box0 = _box(50, 50, 20, 20, 0.0)
        box90 = _box(50, 50, 20, 20, 90.0)
        iou = iou_oriented_boxes(box0, box90)
        assert abs(iou[0] - 1.0) < 1e-6, f"Cuadrado rotado 90° debe ser IoU=1, got {iou[0]}"

    def test_45deg_rotation_reduces_iou(self):
        # Rectángulo no cuadrado a 45° respecto al original: IoU < 1
        box0 = _box(50, 50, 40, 20, 0.0)
        box45 = _box(50, 50, 40, 20, 45.0)
        iou_o = iou_oriented_boxes(box0, box45)[0]
        iou_a = iou_axis_aligned_boxes(box0, box45)[0]
        assert iou_o < 1.0, "IoU orientada debe ser < 1.0 cuando hay rotación"
        assert iou_a >= iou_o, (
            "IoU axis-aligned sobreestima respecto a la orientada cuando hay rotación"
        )

    def test_partial_overlap_known_value(self):
        # Dos cuadrados de lado 10 desplazados 5 unidades en x (50% de overlap)
        # Área de cada uno: 100; Intersección: 5×10=50; Unión: 150; IoU=1/3
        a = _box(5, 5, 10, 10, 0.0)
        b = _box(10, 5, 10, 10, 0.0)
        iou = iou_oriented_boxes(a, b)[0]
        assert abs(iou - 1.0 / 3.0) < 1e-4, f"Expected IoU≈0.333, got {iou}"

    def test_zero_area_degenerate(self):
        a = _box(0, 0, 0, 0, 0.0)
        b = _box(0, 0, 10, 10, 0.0)
        iou = iou_oriented_boxes(a, b)[0]
        assert iou == 0.0

    def test_broadcasting_one_vs_many(self):
        pred = _box(50, 50, 20, 10, 0.0)
        gts = np.array([
            [50, 50, 20, 10, 0.0],    # idéntico → 1.0
            [100, 100, 20, 10, 0.0],  # sin overlap → 0.0
        ])
        ious = iou_oriented_boxes(pred, gts)
        assert ious.shape == (2,)
        assert abs(ious[0] - 1.0) < 1e-6
        assert ious[1] == pytest.approx(0.0, abs=1e-9)

    def test_iou_in_range_0_1(self):
        rng = np.random.default_rng(7)
        preds = rng.uniform(-5, 5, (20, 2))
        for i in range(20):
            cx, cy = float(preds[i, 0]), float(preds[i, 1])
            a = _box(cx, cy, rng.uniform(1, 30), rng.uniform(1, 30), rng.uniform(-90, 90))
            b = _box(cx + rng.uniform(-10, 10), cy + rng.uniform(-10, 10),
                     rng.uniform(1, 30), rng.uniform(1, 30), rng.uniform(-90, 90))
            iou = iou_oriented_boxes(a, b)[0]
            assert 0.0 <= iou <= 1.0 + 1e-9, f"IoU fuera de rango [0,1]: {iou}"


class TestCornellSuccessOriented:
    def test_identical_box_is_success(self):
        box = _box(50, 50, 40, 20, 30.0)
        result = cornell_success_oriented(box, box)
        assert bool(result[0])

    def test_large_angle_error_is_failure(self):
        pred = _box(50, 50, 40, 20, 0.0)
        gt = _box(50, 50, 40, 20, 45.0)
        result = cornell_success_oriented(pred, gt, angle_thr=30.0)
        assert not bool(result[0])

    def test_non_overlapping_is_failure(self):
        pred = _box(0, 0, 10, 5, 0.0)
        gt = _box(100, 100, 10, 5, 0.0)
        result = cornell_success_oriented(pred, gt)
        assert not bool(result[0])
