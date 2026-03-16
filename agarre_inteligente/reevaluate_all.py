#!/usr/bin/env python3
"""Re-evaluar todos los experimentos con el fix del evaluator."""

import sys
from pathlib import Path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

import torch
import pandas as pd
from torch.utils.data import DataLoader
from src.data.grasp_dataset import GraspDataset
from src.data.transforms import get_val_transforms
from src.models.resnet_variants import ResNetGrasp
from src.models.simple_cnn import SimpleCNN
from src.evaluation.evaluator import Evaluator
from src.utils.config_loader import load_config

experiments = [
    ("EXP1_SIMPLE_RGB", "config/exp1_simple_rgb.yaml"),
    ("EXP2_SIMPLE_RGBD", "config/exp2_simple_rgbd.yaml"),
    ("EXP3_RESNET18_RGB_AUGMENT", "config/exp3_resnet18_rgb_augment.yaml"),
    ("EXP4_RESNET18_RGBD", "config/exp4_resnet18_rgbd.yaml"),
]

def build_model_from_config(cfg):
    model_cfg = cfg["model"]
    if model_cfg["name"] == "SimpleGraspCNN":
        return SimpleCNN(input_channels=int(model_cfg["input_channels"]), dropout=float(model_cfg.get("dropout", 0.2)))
    elif model_cfg["name"] == "ResNet18Grasp":
        return ResNetGrasp(
            input_channels=int(model_cfg["input_channels"]),
            pretrained=False,  # No usamos pretrained para cargar checkpoint
            freeze_backbone=bool(model_cfg.get("freeze_backbone", False)),
            dropout=float(model_cfg.get("dropout", 0.2)),
        )
    raise ValueError(f"Modelo no soportado: {model_cfg['name']}")

def reevaluate_experiment(exp_name, config_path):
    print(f"\n{'='*60}")
    print(f"Experimento: {exp_name}")
    print(f"{'='*60}")
    
    cfg = load_config(config_path)
    seeds = cfg["experiment"]["seeds"]
    modality = cfg["data"]["modality"]
    val_csv = cfg["data"]["val_csv"]
    
    # Crear val_loader
    val_ds = GraspDataset(
        csv_path=val_csv,
        data_root=cfg["data"]["data_root"],
        modality=modality,
        transform=get_val_transforms(image_size=224),
        allow_synthetic=False,
    )
    val_loader = DataLoader(val_ds, batch_size=32, shuffle=False)
    
    for seed in seeds:
        seed_dir = Path("experiments") / exp_name / f"seed_{seed}"
        ckpt_path = seed_dir / "checkpoints" / "best.pth"
        
        if not ckpt_path.exists():
            print(f"  [SKIP] seed_{seed}: no checkpoint")
            continue
        
        # Cargar modelo
        model = build_model_from_config(cfg)
        ckpt = torch.load(ckpt_path, map_location="cpu")
        model.load_state_dict(ckpt["model_state_dict"])
        
        # Evaluar con el fix
        evaluator = Evaluator(model, val_loader, device="cpu", csv_path=val_csv)
        criterion = torch.nn.SmoothL1Loss()
        result = evaluator.evaluate(criterion)
        
        print(f"  seed_{seed}:")
        print(f"    Success: {result.val_success:.1%}")
        print(f"    IoU:     {result.val_iou:.4f}")
        print(f"    Angle:   {result.val_angle_deg:.2f}°")
        
        # Actualizar el metrics.csv con las métricas corregidas
        metrics_path = seed_dir / "metrics.csv"
        if metrics_path.exists():
            df = pd.read_csv(metrics_path)
            # Actualizar la última época (o la mejor) con las métricas correctas
            df.iloc[-1, df.columns.get_loc("val_success")] = result.val_success
            df.iloc[-1, df.columns.get_loc("val_iou")] = result.val_iou  
            df.iloc[-1, df.columns.get_loc("val_angle_deg")] = result.val_angle_deg
            df.to_csv(metrics_path, index=False)
            print(f"    ✅ Actualizado {metrics_path.name}")

print("\n" + "="*60)
print("RE-EVALUACIÓN CON FIX DEL EVALUATOR")
print("="*60)

for exp_name, config_path in experiments:
    reevaluate_experiment(exp_name, config_path)

print("\n" + "="*60)
print("✅ RE-EVALUACIÓN COMPLETADA")
print("="*60)
print("\nAhora regenera el resumen y figuras con:")
print("  python scripts/summarize_results.py --experiments-root experiments --output reports/tables/summary_results.csv")
print("  python scripts/generate_figures.py --experiments-root experiments --summary reports/tables/summary_results.csv --out-dir reports/figures")
