#!/usr/bin/env python3
"""Evalua un checkpoint sobre split de validacion y exporta predictions.csv."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import pandas as pd
import torch
from torch import nn
from torch.utils.data import DataLoader

from src.data.grasp_dataset import GraspDataset
from src.data.transforms import get_val_transforms
from src.evaluation.evaluator import Evaluator
from src.models.resnet_variants import ResNetGrasp
from src.models.simple_cnn import SimpleCNN
from src.utils.config_loader import load_config


def build_model(model_cfg: dict):
    if model_cfg["name"] == "SimpleGraspCNN":
        return SimpleCNN(input_channels=int(model_cfg["input_channels"]), dropout=float(model_cfg.get("dropout", 0.2)))
    return ResNetGrasp(
        input_channels=int(model_cfg["input_channels"]),
        pretrained=False,
        freeze_backbone=bool(model_cfg.get("freeze_backbone", False)),
        dropout=float(model_cfg.get("dropout", 0.2)),
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--allow-synthetic", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    model = build_model(cfg["model"])

    state = torch.load(args.checkpoint, map_location="cpu")
    model.load_state_dict(state["model_state_dict"])

    modality = cfg["data"]["modality"]
    val_ds = GraspDataset(
        cfg["data"]["val_csv"],
        cfg["data"]["data_root"],
        modality=modality,
        transform=get_val_transforms(image_size=int(cfg["data"].get("image_size", 224))),
        allow_synthetic=args.allow_synthetic,
        seed=1234,
    )
    val_loader = DataLoader(val_ds, batch_size=int(cfg["training"].get("batch_size", 32)), shuffle=False)

    device = cfg["training"].get("device", "cpu")
    if device == "cuda" and not torch.cuda.is_available():
        device = "cpu"

    criterion = nn.SmoothL1Loss()
    evaluator = Evaluator(model.to(device), val_loader, device=device)
    result = evaluator.evaluate(criterion)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame([result.__dict__]).to_csv(out, index=False)
    evaluator.save_predictions(out.parent / "predictions.csv")

    print(f"[OK] Evaluacion guardada en {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
