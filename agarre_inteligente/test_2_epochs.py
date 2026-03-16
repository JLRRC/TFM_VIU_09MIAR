"""Test de 2 épocas completas para verificar que las métricas se calculan bien."""

import sys
from pathlib import Path
PROJECT_ROOT = Path(__file__).resolve().parents[0]
sys.path.insert(0, str(PROJECT_ROOT))

import torch
import yaml
from torch.utils.data import DataLoader
from src.data.grasp_dataset import GraspDataset
from src.data.transforms import get_train_transforms, get_val_transforms
from src.models.simple_cnn import SimpleCNN
from src.training.trainer import Trainer

# Cargar config
with open("config/exp1_simple_rgb.yaml") as f:
    cfg = yaml.safe_load(f)

# Datasets
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

train_loader = DataLoader(train_ds, batch_size=32, shuffle=True)
val_loader = DataLoader(val_ds, batch_size=32, shuffle=False)

# Modelo
model = SimpleCNN(input_channels=3, dropout=0.2)
optimizer = torch.optim.Adam(model.parameters(), lr=0.0005)
criterion = torch.nn.SmoothL1Loss()

# Entrenar 2 épocas
print("=" * 60)
print("ENTRENAMIENTO DE 2 ÉPOCAS")
print("=" * 60)

trainer = Trainer(
    model=model,
    train_loader=train_loader,
    val_loader=val_loader,
    optimizer=optimizer,
    criterion=criterion,
    device="cpu",
    run_dir="/tmp/test_trainer"
)

metrics_df, best_epoch = trainer.train(epochs=2)

print("\n" + "=" * 60)
print("MÉTRICAS FINALES")
print("=" * 60)
print(metrics_df)
print(f"\nBest epoch: {best_epoch}")

# Verificar que no hay NaN
print("\n" + "=" * 60)
print("VERIFICACIÓN DE NaN")
print("=" * 60)
print(f"train_loss tiene NaN: {metrics_df['train_loss'].isna().any()}")
print(f"val_loss tiene NaN: {metrics_df['val_loss'].isna().any()}")
print(f"val_iou tiene NaN: {metrics_df['val_iou'].isna().any()}")
print(f"val_angle_deg tiene NaN: {metrics_df['val_angle_deg'].isna().any()}")
print(f"val_success tiene NaN: {metrics_df['val_success'].isna().any()}")

if not metrics_df.isna().any().any():
    print("\n✅ ¡ÉXITO! Todas las métricas tienen valores válidos.")
else:
    print("\n❌ ERROR: Hay métricas con NaN.")
