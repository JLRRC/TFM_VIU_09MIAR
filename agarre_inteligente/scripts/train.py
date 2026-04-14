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
from src.models.factory import build_model
from src.training.trainer import Trainer
from src.utils.config_loader import load_config, save_config_snapshot


def _validate_split_size(
    *,
    split_name: str,
    dataset,
    csv_path: str,
    expected_size: object,
) -> None:
    """Fail fast when the configured split metadata does not match the loaded dataset."""
    if expected_size in (None, ""):
        return
    try:
        expected = int(expected_size)
    except Exception:
        raise ValueError(
            f"Tamaño esperado invalido para {split_name}: {expected_size!r}"
        ) from None
    actual = len(dataset)
    if actual != expected:
        raise ValueError(
            f"Split {split_name} inconsistente: expected={expected} actual={actual} csv={csv_path}"
        )


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
    exp_cfg = cfg.get("experiment", {}) if isinstance(cfg.get("experiment"), dict) else {}
    _validate_split_size(
        split_name="train",
        dataset=train_ds,
        csv_path=train_csv,
        expected_size=exp_cfg.get("train_size_expected"),
    )
    _validate_split_size(
        split_name="val",
        dataset=val_ds,
        csv_path=val_csv,
        expected_size=exp_cfg.get("val_size_expected"),
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
