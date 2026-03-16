"""Test rápido para verificar que métricas no son NaN."""

import sys
from pathlib import Path
PROJECT_ROOT = Path(__file__).resolve().parents[0]
sys.path.insert(0, str(PROJECT_ROOT))

import torch
import yaml
import numpy as np
from torch.utils.data import DataLoader, Subset
from src.data.grasp_dataset import GraspDataset
from src.data.transforms import get_train_transforms, get_val_transforms
from src.models.simple_cnn import SimpleCNN
from src.training.metrics import summarize_batch

# Cargar config
with open("config/exp1_simple_rgb.yaml") as f:
    cfg = yaml.safe_load(f)

# Datasets pequeños (solo 64 samples)
train_ds = GraspDataset(
    cfg["data"]["train_csv"],
    cfg["data"]["data_root"],
    modality="rgb",
    transform=get_train_transforms(224, augmentation=False),
    seed=0
)
val_ds = GraspDataset(
    cfg["data"]["val_csv"],
    cfg["data"]["data_root"],
    modality="rgb",
    transform=get_val_transforms(224),
    seed=999
)

# Subset pequeño
train_subset = Subset(train_ds, list(range(64)))
val_subset = Subset(val_ds, list(range(64)))

train_loader = DataLoader(train_subset, batch_size=32, shuffle=True)
val_loader = DataLoader(val_subset, batch_size=32, shuffle=False)

# Modelo
model = SimpleCNN(input_channels=3, dropout=0.2)
optimizer = torch.optim.Adam(model.parameters(), lr=0.0005)
criterion = torch.nn.SmoothL1Loss()

print("=" * 60)
print("TEST RÁPIDO: 3 ÉPOCAS CON 64 SAMPLES")
print("=" * 60)

device = "cpu"
model.to(device)

for epoch in range(1, 4):
    print(f"\n--- Época {epoch} ---")
    
    # Training
    model.train()
    batch_losses = []
    for x, y, _ in train_loader:
        x = x.to(device)
        y = y.to(device)
        optimizer.zero_grad(set_to_none=True)
        pred = model(x)
        loss = criterion(pred, y)
        loss.backward()
        optimizer.step()
        batch_losses.append(float(loss.item()))
    
    train_loss = float(sum(batch_losses) / max(len(batch_losses), 1))
    
    # Validation
    model.eval()
    val_losses = []
    metric_rows = []
    with torch.no_grad():
        for x, y, _ in val_loader:
            x = x.to(device)
            y = y.to(device)
            pred = model(x)
            loss = criterion(pred, y)
            val_losses.append(float(loss.item()))
            m = summarize_batch(pred.cpu().numpy(), y.cpu().numpy())
            metric_rows.append(m)
    
    val_loss = float(np.mean(val_losses)) if val_losses else float("nan")
    val_success = float(np.mean([r["val_success"] for r in metric_rows]))
    val_iou = float(np.mean([r["val_iou"] for r in metric_rows]))
    val_angle_deg = float(np.mean([r["val_angle_deg"] for r in metric_rows]))
    
    print(f"  train_loss: {train_loss:.4f}")
    print(f"  val_loss: {val_loss:.4f}")
    print(f"  val_success: {val_success:.4f}")
    print(f"  val_iou: {val_iou:.4f}")
    print(f"  val_angle_deg: {val_angle_deg:.4f}")
    
    # Verificar NaN
    has_nan = (
        np.isnan(train_loss) or 
        np.isnan(val_loss) or 
        np.isnan(val_success) or 
        np.isnan(val_iou) or 
        np.isnan(val_angle_deg)
    )
    
    if has_nan:
        print("  ❌ ERROR: Hay métricas con NaN")
        break
    else:
        print("  ✅ Todas las métricas válidas")

print("\n" + "=" * 60)
print("TEST COMPLETADO")
print("=" * 60)
