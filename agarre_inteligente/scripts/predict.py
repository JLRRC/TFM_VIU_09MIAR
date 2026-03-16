#!/usr/bin/env python3
"""Inferencia simple sobre una imagen para obtener rectangulo de agarre."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import torch
from PIL import Image

from src.data.transforms import get_val_transforms
from src.models.resnet_variants import ResNetGrasp
from src.models.simple_cnn import SimpleCNN
from src.utils.config_loader import load_config


def build_model(model_cfg: dict):
    if model_cfg["name"] == "SimpleGraspCNN":
        return SimpleCNN(input_channels=int(model_cfg["input_channels"]))
    return ResNetGrasp(input_channels=int(model_cfg["input_channels"]), pretrained=False)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--image", required=True)
    args = ap.parse_args()

    cfg = load_config(args.config)
    model = build_model(cfg["model"])
    state = torch.load(args.checkpoint, map_location="cpu")
    model.load_state_dict(state["model_state_dict"])
    model.eval()

    img = Image.open(Path(args.image)).convert("RGB")
    tfm = get_val_transforms(image_size=int(cfg["data"].get("image_size", 224)))
    x = tfm(img).unsqueeze(0)

    with torch.no_grad():
        y = model(x).squeeze(0).tolist()

    print("pred_cx,pred_cy,pred_w,pred_h,pred_angle_deg")
    print(",".join(f"{v:.6f}" for v in y))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
