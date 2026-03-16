"""Modelo ligero SimpleGraspCNN para regresion de rectangulo de agarre."""

from __future__ import annotations

import torch
import torch.nn as nn


class SimpleCNN(nn.Module):
    def __init__(self, input_channels: int = 3, dropout: float = 0.2):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(input_channels, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(128, 5),  # cx, cy, w, h, angle_deg
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.head(self.features(x))
