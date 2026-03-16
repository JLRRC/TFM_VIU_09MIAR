#!/usr/bin/env python3
"""Entrena un experimento/seed y genera metrics.csv + checkpoints."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import torch
from torch import nn
from torch.utils.data import DataLoader

from src.data.grasp_dataset import GraspDataset
from src.data.transforms import get_train_transforms, get_val_transforms
from src.models.resnet_variants import ResNetGrasp
from src.models.simple_cnn import SimpleCNN
from src.training.trainer import Trainer
from src.utils.config_loader import load_config, save_config_snapshot


def build_model(model_cfg: dict):
    if model_cfg["name"] == "SimpleGraspCNN":
        return SimpleCNN(input_channels=int(model_cfg["input_channels"]), dropout=float(model_cfg.get("dropout", 0.2)))
    if model_cfg["name"] == "ResNet18Grasp":
        return ResNetGrasp(
            input_channels=int(model_cfg["input_channels"]),
            pretrained=bool(model_cfg.get("pretrained", False)),
            freeze_backbone=bool(model_cfg.get("freeze_backbone", False)),
            dropout=float(model_cfg.get("dropout", 0.2)),
        )
    raise ValueError(f"Modelo no soportado: {model_cfg['name']}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--allow-synthetic", action="store_true", help="Permite entrenar sin dataset real")
    args = ap.parse_args()

    cfg = load_config(args.config)
    exp_name = cfg["experiment"]["name"]
    run_dir = Path(cfg["outputs"]["experiments_root"]) / exp_name / f"seed_{args.seed}"
    run_dir.mkdir(parents=True, exist_ok=True)

    torch.manual_seed(args.seed)
    device = cfg["training"].get("device", "cpu")
    if device == "cuda" and not torch.cuda.is_available():
        device = "cpu"

    modality = cfg["data"]["modality"]
    image_size = int(cfg["data"].get("image_size", 224))
    train_csv = cfg["data"]["train_csv"]
    val_csv = cfg["data"]["val_csv"]
    data_root = cfg["data"]["data_root"]

    train_ds = GraspDataset(
        train_csv,
        data_root,
        modality=modality,
        transform=get_train_transforms(image_size=image_size, augmentation=bool(cfg["data"].get("augmentation", False))),
        allow_synthetic=args.allow_synthetic,
        seed=args.seed,
    )
    val_ds = GraspDataset(
        val_csv,
        data_root,
        modality=modality,
        transform=get_val_transforms(image_size=image_size),
        allow_synthetic=args.allow_synthetic,
        seed=args.seed + 999,
    )

    batch_size = int(cfg["training"].get("batch_size", 32))
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False)

    model = build_model(cfg["model"])

    lr = float(cfg["training"]["optimizer"]["lr"])
    wd = float(cfg["training"]["optimizer"].get("weight_decay", 0.0))
    if cfg["training"]["optimizer"]["name"].lower() == "sgd":
        momentum = float(cfg["training"]["optimizer"].get("momentum", 0.9))
        optimizer = torch.optim.SGD(model.parameters(), lr=lr, weight_decay=wd, momentum=momentum)
    else:
        optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=wd)

    criterion = nn.SmoothL1Loss()

    trainer = Trainer(
        model=model,
        train_loader=train_loader,
        val_loader=val_loader,
        optimizer=optimizer,
        criterion=criterion,
        device=device,
        run_dir=run_dir,
        val_csv_path=val_csv,
    )

    epochs = int(cfg["training"]["epochs"])
    _, best_epoch = trainer.train(epochs=epochs)

    save_config_snapshot(cfg, run_dir / "config_snapshot.yaml")
    (run_dir / "best_epoch.txt").write_text(str(best_epoch), encoding="utf-8")

    print(f"[OK] Entrenamiento finalizado: {run_dir}")
    print(f"[OK] best_epoch={best_epoch}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
