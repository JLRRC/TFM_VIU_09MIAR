"""Factoría única para construir modelos de agarre desde configuración."""

from __future__ import annotations

from .resnet_variants import ResNetGrasp
from .simple_cnn import SimpleCNN
from .simple_grasp import SimpleGrasp


def build_model(model_cfg: dict):
    name = model_cfg["name"]
    input_channels = int(model_cfg["input_channels"])

    if name == "SimpleGraspCNN":
        return SimpleCNN(
            input_channels=input_channels,
            dropout=float(model_cfg.get("dropout", 0.2)),
        )
    if name == "SimpleGrasp":
        return SimpleGrasp(input_channels=input_channels)
    if name == "ResNet18Grasp":
        return ResNetGrasp(
            input_channels=input_channels,
            pretrained=bool(model_cfg.get("pretrained", False)),
            freeze_backbone=bool(model_cfg.get("freeze_backbone", False)),
            dropout=float(model_cfg.get("dropout", 0.2)),
        )
    raise ValueError(f"Modelo no soportado: {name}")
