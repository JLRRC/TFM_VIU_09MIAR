"""Tests del criterio grasp success por imagen con IoU orientada.

Verifica que cornell_success_oriented implementa el criterio Cornell canónico:
una predicción se considera correcta si existe AL MENOS UNA anotación GT de
esa imagen para la que simultáneamente IoU >= 0.25 y |Δθ| <= 30°.

También valida que la diferencia cualitativa con axis-aligned es la esperada:
la IoU orientada es más estricta cuando hay diferencia de ángulo.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.training.metrics import (
    cornell_success_oriented,
    cornell_success,
    iou_oriented_boxes,
    iou_axis_aligned_boxes,
    angle_error_deg,
)


def _box(cx, cy, w, h, a):
    return np.array([[cx, cy, w, h, a]], dtype=np.float64)


class TestSuccessPerImage:
    """Simula la evaluación por imagen: predicción única vs múltiples GT."""

    def test_success_if_any_gt_matches(self):
        pred = _box(50, 50, 30, 15, 10.0)
        # GT1: sin solapamiento (fracaso), GT2: idéntico a pred (éxito)
        gt_bad = _box(200, 200, 30, 15, 10.0)
        gt_good = _box(50, 50, 30, 15, 10.0)
        all_gts = np.vstack([gt_bad, gt_good])

        successes = cornell_success_oriented(pred, all_gts)
        # El criterio por imagen: np.any → True si al menos una GT da éxito
        assert bool(np.any(successes)), "Debe ser éxito si al menos un GT coincide"

    def test_failure_if_no_gt_matches(self):
        pred = _box(50, 50, 30, 15, 0.0)
        # Todos los GT tienen ángulo muy diferente o posición lejana
        gt1 = _box(200, 200, 30, 15, 0.0)
        gt2 = _box(50, 50, 30, 15, 80.0)  # ángulo muy diferente
        all_gts = np.vstack([gt1, gt2])

        successes = cornell_success_oriented(pred, all_gts)
        assert not bool(np.any(successes)), "Debe ser fracaso si ningún GT coincide"

    def test_oriented_stricter_than_axis_aligned_with_rotation(self):
        # Predicción correcta en posición/tamaño pero rotada 35° respecto al GT
        pred = _box(50, 50, 40, 10, 35.0)
        gt = _box(50, 50, 40, 10, 0.0)

        iou_o = iou_oriented_boxes(pred, gt)[0]
        iou_a = iou_axis_aligned_boxes(pred, gt)[0]
        ang_err = angle_error_deg(pred[:, 4], gt[:, 4])[0]

        # La IoU orientada debe ser más pequeña (más estricta)
        assert iou_o <= iou_a + 1e-6, (
            f"IoU orientada {iou_o:.4f} debería ser <= axis-aligned {iou_a:.4f} "
            "cuando hay diferencia de ángulo significativa"
        )
        # El éxito orientado debe exigir más que el axis-aligned
        succ_o = cornell_success_oriented(pred, gt, iou_thr=0.25, angle_thr=30.0)[0]
        succ_a = cornell_success(pred, gt, iou_thr=0.25, angle_thr=30.0)[0]
        # Si la axis-aligned falla, la orientada también debe fallar
        if not succ_a:
            assert not succ_o

    def test_angle_threshold_respected(self):
        # Predicción con 29° de diferencia: debe ser éxito (< 30°)
        pred = _box(50, 50, 30, 15, 29.0)
        gt = _box(50, 50, 30, 15, 0.0)
        result = cornell_success_oriented(pred, gt, angle_thr=30.0)
        assert bool(result[0]), "Diferencia de 29° < umbral 30° debe ser éxito"

    def test_angle_threshold_boundary(self):
        # Predicción con exactamente 30° de diferencia: debe ser éxito (<=)
        pred = _box(50, 50, 30, 15, 30.0)
        gt = _box(50, 50, 30, 15, 0.0)
        result = cornell_success_oriented(pred, gt, angle_thr=30.0)
        assert bool(result[0]), "Diferencia de exactamente 30° (≤ umbral) debe ser éxito"

    def test_iou_threshold_respected(self):
        # Sin solapamiento: IoU = 0 < 0.25 → fracaso
        pred = _box(0, 0, 10, 5, 0.0)
        gt = _box(100, 100, 10, 5, 0.0)
        result = cornell_success_oriented(pred, gt)
        assert not bool(result[0])

    def test_180deg_symmetry_success(self):
        # Ángulos equivalentes por simetría de pinza: 89° ≡ -91° ≡ -91 + 180 = 89°
        # Diferencia real = 0° mod 180° = 0° → éxito
        pred = _box(50, 50, 30, 15, 89.0)
        gt = _box(50, 50, 30, 15, -89.0)
        ang_err = angle_error_deg(pred[:, 4], gt[:, 4])[0]
        assert ang_err == pytest.approx(2.0, abs=0.1), (
            f"Diferencia circular 89° - (-89°) mod 180° debería ser ≈2°, got {ang_err}"
        )

    def test_multiple_gt_per_image_semantics(self):
        # Simula imagen con 5 anotaciones Cornell — solo una "buena" para la pred
        pred = _box(50, 50, 30, 15, 5.0)
        gts = np.array([
            [10,  10, 30, 15, 5.0],   # lejos
            [90,  50, 30, 15, 5.0],   # lejos
            [50,  50, 30, 15, 5.0],   # idéntico → éxito
            [50,  50, 30, 15, 60.0],  # ángulo malo
            [200, 50, 30, 15, 5.0],   # lejos
        ], dtype=np.float64)
        successes = cornell_success_oriented(pred, gts)
        assert bool(np.any(successes))
        # Solo el índice 2 debe ser éxito
        assert bool(successes[2])
        assert not bool(successes[0])
        assert not bool(successes[1])
