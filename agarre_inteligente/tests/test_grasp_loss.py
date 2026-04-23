"""Tests de GraspLoss — función de pérdida metodológicamente alineada.

Propiedades verificadas:
- Pérdida cero para predicción idéntica al target
- Simetría de 180°: ángulos equivalentes (diff = 2 en espacio normalizado) dan pérdida cero
- Pérdida máxima del término de ángulo cuando la diferencia es 90° (norm diff = 1)
- La pérdida geométrica se comporta como SmoothL1 en los 4 primeros outputs
- build_criterion retorna los tipos correctos
"""

from __future__ import annotations

import sys
import math
from pathlib import Path

import torch
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.training.losses import GraspLoss, build_criterion


class TestGraspLossAngularSymmetry:
    def test_identical_prediction_zero_loss(self):
        loss_fn = GraspLoss()
        t = torch.tensor([[0.5, 0.5, 0.3, 0.2, 0.5]])
        assert loss_fn(t, t).item() == pytest.approx(0.0, abs=1e-6)

    def test_180deg_equivalent_angles_zero_angle_loss(self):
        # angle_norm = 1.0 ≡ 90° y angle_norm = -1.0 ≡ -90°: diferencia 180° ≡ 0°
        # El término de ángulo debe ser: 1 - cos(pi * 2.0) = 1 - 1 = 0
        pred = torch.tensor([[0.5, 0.5, 0.3, 0.2, 1.0]])
        gt = torch.tensor([[0.5, 0.5, 0.3, 0.2, -1.0]])
        loss_fn = GraspLoss(angle_weight=1.0, geom_weight=0.0)
        angle_loss = loss_fn(pred, gt).item()
        assert angle_loss == pytest.approx(0.0, abs=1e-5), (
            f"Ángulos equivalentes (diff=180°) deben dar pérdida≈0, got {angle_loss}"
        )

    def test_90deg_difference_maximum_angle_loss(self):
        # diff normalizado = 1.0 ≡ 90°: L = 1 - cos(pi * 1.0) = 1 - (-1) = 2
        pred = torch.tensor([[0.5, 0.5, 0.3, 0.2, 0.5]])
        gt = torch.tensor([[0.5, 0.5, 0.3, 0.2, -0.5]])
        loss_fn = GraspLoss(angle_weight=1.0, geom_weight=0.0)
        angle_loss = loss_fn(pred, gt).item()
        assert angle_loss == pytest.approx(2.0, abs=1e-5), (
            f"diff=90° debe dar pérdida=2.0, got {angle_loss}"
        )

    def test_30deg_difference(self):
        # diff normalizado = 30/90 = 1/3: L = 1 - cos(pi/3) = 1 - 0.5 = 0.5
        pred = torch.tensor([[0.0, 0.0, 0.0, 0.0, 1 / 3]])
        gt = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0]])
        loss_fn = GraspLoss(angle_weight=1.0, geom_weight=0.0)
        angle_loss = loss_fn(pred, gt).item()
        assert angle_loss == pytest.approx(0.5, abs=1e-5)

    def test_geom_loss_only_matches_smooth_l1(self):
        import torch.nn as nn
        rng = torch.Generator()
        rng.manual_seed(0)
        pred = torch.rand(8, 5, generator=rng)
        gt = torch.rand(8, 5, generator=rng)
        loss_fn = GraspLoss(angle_weight=0.0, geom_weight=1.0)
        smooth_l1 = nn.SmoothL1Loss()
        assert loss_fn(pred, gt).item() == pytest.approx(
            smooth_l1(pred[:, :4], gt[:, :4]).item(), abs=1e-5
        )

    def test_weights_scale_loss(self):
        pred = torch.tensor([[0.5, 0.5, 0.3, 0.2, 0.5]])
        gt = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0]])
        loss1 = GraspLoss(angle_weight=1.0, geom_weight=1.0)(pred, gt).item()
        loss2 = GraspLoss(angle_weight=2.0, geom_weight=1.0)(pred, gt).item()
        assert loss2 > loss1

    def test_batch_consistent(self):
        rng = torch.Generator()
        rng.manual_seed(1)
        pred = torch.rand(16, 5, generator=rng)
        gt = torch.rand(16, 5, generator=rng)
        loss_fn = GraspLoss()
        loss = loss_fn(pred, gt)
        assert loss.item() >= 0.0
        assert torch.isfinite(loss)


class TestBuildCriterion:
    def test_smooth_l1(self):
        import torch.nn as nn
        c = build_criterion("smooth_l1")
        assert isinstance(c, nn.SmoothL1Loss)

    def test_mse(self):
        import torch.nn as nn
        c = build_criterion("mse")
        assert isinstance(c, nn.MSELoss)

    def test_grasp_loss(self):
        c = build_criterion("grasp_loss")
        assert isinstance(c, GraspLoss)

    def test_grasp_loss_with_kwargs(self):
        c = build_criterion("grasp_loss", angle_weight=2.0, geom_weight=0.5)
        assert isinstance(c, GraspLoss)
        assert c.angle_weight == pytest.approx(2.0)
        assert c.geom_weight == pytest.approx(0.5)

    def test_unknown_raises(self):
        with pytest.raises(ValueError):
            build_criterion("unknown_loss")
