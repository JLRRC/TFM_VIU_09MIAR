"""Modelo de referencia ResNet18Grasp para regresion de agarre."""

from __future__ import annotations

import torch
import torch.nn as nn
import torchvision.models as tvm


class ResNetGrasp(nn.Module):
    def __init__(
        self,
        input_channels: int = 3,
        pretrained: bool = True,
        freeze_backbone: bool = False,
        dropout: float = 0.2,
    ):
        super().__init__()
        weights = tvm.ResNet18_Weights.DEFAULT if pretrained else None
        backbone = tvm.resnet18(weights=weights)

        if input_channels != 3:
            old = backbone.conv1
            backbone.conv1 = nn.Conv2d(
                input_channels,
                old.out_channels,
                kernel_size=old.kernel_size,
                stride=old.stride,
                padding=old.padding,
                bias=old.bias,
            )
            with torch.no_grad():
                backbone.conv1.weight[:, :3] = old.weight
                if input_channels > 3:
                    for i in range(3, input_channels):
                        backbone.conv1.weight[:, i : i + 1] = old.weight[:, :1]

        in_features = backbone.fc.in_features
        backbone.fc = nn.Sequential(
            nn.Dropout(dropout),
            nn.Linear(in_features, 5),
        )

        if freeze_backbone:
            for n, p in backbone.named_parameters():
                if not n.startswith("fc"):
                    p.requires_grad = False

        self.model = backbone

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.model(x)
