#!/usr/bin/env python3
"""Test del fix del evaluator con múltiples GT por imagen."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import torch
from torch.utils.data import DataLoader
from src.data.grasp_dataset import GraspDataset
from src.data.transforms import get_val_transforms
from src.models.resnet_variants import ResNetGrasp
from src.evaluation.evaluator import Evaluator

print("=== TEST DEL FIX DEL EVALUATOR ===\n")

# Cargar un modelo entrenado
ckpt_path = "experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth"
if not Path(ckpt_path).exists():
    print(f"ERROR: No existe {ckpt_path}")
    sys.exit(1)

print(f"Cargando checkpoint: {ckpt_path}")
model = ResNetGrasp(input_channels=3, pretrained=False)
ckpt = torch.load(ckpt_path, map_location="cpu")
model.load_state_dict(ckpt["model_state_dict"])

# Crear val_loader
val_ds = GraspDataset(
    csv_path="data/processed/cornell/splits/object_wise/val.csv",
    data_root=".",
    modality="rgb",
    transform=get_val_transforms(image_size=224),
    allow_synthetic=False,
)
val_loader = DataLoader(val_ds, batch_size=32, shuffle=False)

print(f"Val dataset: {len(val_ds)} samples\n")

# Evaluar SIN el fix (modo antiguo: sin csv_path)
print("1. Evaluación SIN fix (1 GT por imagen):")
evaluator_old = Evaluator(model, val_loader, device="cpu", csv_path=None)
criterion = torch.nn.SmoothL1Loss()
result_old = evaluator_old.evaluate(criterion)
print(f"   Success: {result_old.val_success:.1%}")
print(f"   IoU:     {result_old.val_iou:.4f}")
print(f"   Angle:   {result_old.val_angle_deg:.2f}°\n")

# Evaluar CON el fix (con csv_path para cargar todos los GT)
print("2. Evaluación CON fix (todos los GT por imagen):")
evaluator_new = Evaluator(model, val_loader, device="cpu", csv_path="data/processed/cornell/splits/object_wise/val.csv")
result_new = evaluator_new.evaluate(criterion)
print(f"   Success: {result_new.val_success:.1%}")
print(f"   IoU:     {result_new.val_iou:.4f}")
print(f"   Angle:   {result_new.val_angle_deg:.2f}°\n")

# Comparar
mejora_absoluta = result_new.val_success - result_old.val_success
mejora_relativa = (result_new.val_success / max(result_old.val_success, 0.001) - 1) * 100

print("="*50)
print(f"MEJORA: {mejora_absoluta:.1%} absoluto ({mejora_relativa:+.1f}% relativo)")
if result_new.val_success > 0.30:
    print("✅ FIX EXITOSO: Métricas recuperadas cerca del nivel esperado (>30%)")
elif result_new.val_success > result_old.val_success * 1.5:
    print("✅ FIX FUNCIONA: Mejora significativa detectada")
else:
    print("⚠️ WARNING: Mejora menor de lo esperado, revisar lógica")
print("="*50)
