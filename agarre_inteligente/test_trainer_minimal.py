"""Test minimal del Trainer para ver por qué produce NaN."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import torch
import yaml
from torch.utils.data import DataLoader, Subset
from src.data.grasp_dataset import GraspDataset  
from src.data.transforms import get_train_transforms, get_val_transforms
from src.models.simple_cnn import SimpleCNN
from src.training.trainer import Trainer

# Config
with open("config/exp1_simple_rgb.yaml") as f:
    cfg = yaml.safe_load(f)

# Datasets MUY PEQUEÑOS (32 samples cada uno)
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

# Subsets de solo 32 muestras
train_sub = Subset(train_ds, list(range(32)))
val_sub = Subset(val_ds, list(range(32)))

train_loader = DataLoader(train_sub, batch_size=16, shuffle=True)
val_loader = DataLoader(val_sub, batch_size=16, shuffle=False)

print(f"Train subset: {len(train_sub)} samples")
print(f"Val subset: {len(val_sub)} samples")

# Modelo
model = SimpleCNN(input_channels=3, dropout=0.2)
optimizer = torch.optim.Adam(model.parameters(), lr=0.0005)
criterion = torch.nn.SmoothL1Loss()

# Trainer
print("\n" + "=" * 60)
print("ENTRENANDO 2 ÉPOCAS CON LOGGING ACTIVADO")
print("=" * 60)

trainer = Trainer(
    model=model,
    train_loader=train_loader,
    val_loader=val_loader,
    optimizer=optimizer,
    criterion=criterion,
    device="cuda" if torch.cuda.is_available() else "cpu",
    run_dir="/tmp/test_trainer_debug"
)

print(f"Using device: {'cuda' if torch.cuda.is_available() else 'cpu'}")

metrics_df, best_epoch = trainer.train(epochs=2)

print("\n" + "=" * 60)
print("RESULTADOS")
print("=" * 60)
print(metrics_df)
print(f"\nBest epoch: {best_epoch}")

# Verificar NaN
has_nan = metrics_df.isna().any().any()
print(f"\n¿Hay NaN? {has_nan}")

if has_nan:
    print("\n❌ PROBLEMA REPRODUCIDO - hay métricas NaN")
    print("\nColumnas con NaN:")
    for col in metrics_df.columns:
        if metrics_df[col].isna().any():
            print(f"  - {col}")
else:
    print("\n✅ NO hay NaN - el problema no se reprodujo")
