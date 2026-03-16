# Models Directory

Carpeta destinada a almacenar modelos finales exportados o seleccionados.

## Relacion con `experiments/`

- `experiments/<EXP>/seed_<N>/checkpoints/`: checkpoints de entrenamiento por seed (`best.pth`, `last.pth`).
- `models/`: opcion para copiar un modelo final consolidado para inferencia o despliegue.

## Formato de checkpoint esperado

Los checkpoints del proyecto guardan, como minimo:

```python
{
    "epoch": int,
    "model_state_dict": ...,
    "optimizer_state_dict": ...,
    "metrics": {
        "epoch": int,
        "train_loss": float,
        "val_loss": float,
        "val_success": float,
        "val_iou": float,
        "val_angle_deg": float,
    },
}
```

## Carga basica de modelo

```python
import torch
from src.models.simple_cnn import SimpleCNN

ckpt = torch.load("experiments/EXP1_SIMPLE_RGB/seed_0/checkpoints/best.pth", map_location="cpu")
model = SimpleCNN(input_channels=3)
model.load_state_dict(ckpt["model_state_dict"])
model.eval()
```

Para ResNet:

```python
from src.models.resnet_variants import ResNetGrasp

model = ResNetGrasp(input_channels=3, pretrained=False)
```

## Recomendacion de gestion

- Mantener checkpoints originales en `experiments/`.
- Copiar a `models/` solo artefactos finales estables para inferencia.
- Documentar en `docs/EXPERIMENTS.md` cualquier modelo promovido como final.

**Ultima actualizacion:** 2026-03-07
