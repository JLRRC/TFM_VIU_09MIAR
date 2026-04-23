"""Funciones de pérdida para entrenamiento de modelos de agarre.

OFICIAL TFM (EXP1..EXP4): nn.SmoothL1Loss() — hardcoded en train.py.

ALINEACIÓN METODOLÓGICA POSTERIOR: GraspLoss — trata el ángulo con su
simetría de 180° (loss coseno periódico) en lugar de aplicar SmoothL1
directo sobre el valor normalizado, lo que puede penalizar ángulos
equivalentes como si fueran muy distintos.
"""

from __future__ import annotations

import math

import torch
import torch.nn as nn


class GraspLoss(nn.Module):
    """Pérdida de agarre metodológicamente alineada con la formulación del TFM.

    Para los componentes geométricos (cx, cy, w, h) se usa SmoothL1Loss.
    Para el ángulo (normalizado en [-1, 1] como angle_deg / 90) se usa una
    pérdida coseno periódica que respeta la simetría de 180° de la pinza:

        L_angle = mean(1 - cos(π · (pred_norm - gt_norm)))

    Esta función tiene periodo 2 en el espacio normalizado (equivalente a 180°
    en grados), por lo que pred_norm=1.0 y gt_norm=-1.0 dan pérdida=0, al ser
    ángulos equivalentes módulo 180° para una pinza paralela.

    Parámetros
    ----------
    angle_weight : float
        Peso del término de ángulo en la pérdida total. Por defecto 1.0.
    geom_weight : float
        Peso de la pérdida geométrica (cx, cy, w, h). Por defecto 1.0.
    beta : float
        Parámetro beta de SmoothL1Loss (transición entre L1 y L2). Defecto 1.0.
    """

    def __init__(
        self,
        angle_weight: float = 1.0,
        geom_weight: float = 1.0,
        beta: float = 1.0,
    ) -> None:
        super().__init__()
        self.smooth_l1 = nn.SmoothL1Loss(beta=beta)
        self.angle_weight = float(angle_weight)
        self.geom_weight = float(geom_weight)

    def forward(self, pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        """
        pred, target: tensores (B, 5) con columnas (cx, cy, w, h, angle_norm).
        angle_norm = angle_deg / 90, rango [-1, 1].
        """
        geom_loss = self.smooth_l1(pred[:, :4], target[:, :4])
        angle_diff = pred[:, 4] - target[:, 4]
        # Periodo 2 ≡ simetría de 180° — no penaliza diferencias de 180°
        angle_loss = (1.0 - torch.cos(math.pi * angle_diff)).mean()
        return self.geom_weight * geom_loss + self.angle_weight * angle_loss


def build_criterion(name: str, **kwargs) -> nn.Module:
    """Construye la función de pérdida a partir del nombre en el YAML de config.

    Nombres válidos
    ---------------
    "smooth_l1" : nn.SmoothL1Loss() — función oficial de EXP1..EXP4.
    "mse"       : nn.MSELoss() — alternativa estándar.
    "grasp_loss": GraspLoss — variante metodológicamente alineada (posterior TFM).

    Cualquier kwarg adicional se pasa al constructor si el loss lo acepta.
    """
    name = name.strip().lower()
    if name in ("smooth_l1", "smoothl1"):
        return nn.SmoothL1Loss()
    if name == "mse":
        return nn.MSELoss()
    if name == "grasp_loss":
        return GraspLoss(**kwargs)
    raise ValueError(f"Criterio desconocido: {name!r}. Opciones: smooth_l1, mse, grasp_loss")
