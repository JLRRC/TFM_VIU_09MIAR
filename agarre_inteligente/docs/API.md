# API Documentation

Referencia tecnica de los modulos principales en `src/`.

## `src.models`

### `SimpleCNN` (`src/models/simple_cnn.py`)

Modelo CNN ligero para regresion de agarres.

- Entrada: tensor `N x C x H x W` (`C=3` RGB o `C=4` RGB-D)
- Salida: tensor `N x 5` con `[cx, cy, w, h, angle_deg]` normalizados

Firma:

```python
SimpleCNN(input_channels: int = 3, dropout: float = 0.2)
```

### `ResNetGrasp` (`src/models/resnet_variants.py`)

ResNet18 adaptada para regresion de agarres.

Firma:

```python
ResNetGrasp(
    input_channels: int = 3,
    pretrained: bool = True,
    freeze_backbone: bool = False,
    dropout: float = 0.2,
)
```

Notas:

- Cuando `input_channels != 3`, se adapta `conv1`.
- La cabeza final proyecta a 5 valores de regresion.

## `src.data`

### `GraspDataset` (`src/data/grasp_dataset.py`)

Dataset Cornell con soporte `rgb` y `rgbd`.

Firma:

```python
GraspDataset(
    csv_path,
    data_root,
    modality: str = "rgb",
    transform=None,
    allow_synthetic: bool = False,
    synthetic_size: int = 256,
    seed: int = 0,
)
```

CSV esperado:

- `image_path, depth_path, cx, cy, w, h, angle_deg`

Salida de `__getitem__`:

- `image`: tensor `C x 224 x 224`
- `target`: tensor de 5 elementos normalizados
- `image_path`: string

### Transforms (`src/data/transforms.py`)

- `get_train_transform(input_size=224, augmentation=False)`
- `get_val_transform(input_size=224)`

## `src.training`

### `Trainer` (`src/training/trainer.py`)

Entrenamiento por epoca con checkpointing y evaluacion de validacion.

Firma:

```python
Trainer(
    model,
    train_loader,
    val_loader,
    optimizer,
    criterion,
    device,
    run_dir,
    val_csv_path=None,
)
```

Metodo principal:

- `train(epochs: int) -> tuple[pd.DataFrame, int]`

`metrics.csv` generado por seed incluye:

- `epoch, train_loss, val_loss, val_success, val_iou, val_angle_deg`

### Metricas (`src/training/metrics.py`)

Funciones principales:

- `iou_axis_aligned_boxes(pred, gt)`
- `angle_error_deg(pred_angle, gt_angle)`
- `cornell_success(pred, gt, iou_thr=0.25, angle_thr=30.0)`

## `src.evaluation`

### `Evaluator` (`src/evaluation/evaluator.py`)

Evaluacion estilo Cornell y export de predicciones.

Firma:

```python
Evaluator(model, dataloader, device="cpu", csv_path=None)
```

Metodos:

- `evaluate(criterion) -> EvalResult`
- `save_predictions(output_csv)`

`EvalResult` contiene:

- `val_success`
- `val_iou`
- `val_angle_deg`
- `val_loss`

Nota: la evaluacion vigente compara cada prediccion contra todos los GT de la imagen (multi-GT) cuando se proporciona `csv_path`.

## `src.utils`

- `src/utils/config_loader.py`: carga y validacion de YAML
- `src/utils/checkpoint.py`: guardado `best.pth` y `last.pth`
- `src/utils/logger.py`: utilidades de logging

## Scripts clave

- Entrenamiento por seed:

```bash
python3 scripts/train.py --config config/exp1_simple_rgb.yaml --seed 0
```

- Evaluacion y predicciones:

```bash
python3 scripts/evaluate.py --checkpoint experiments/EXP1_SIMPLE_RGB/seed_0/checkpoints/best.pth --config config/exp1_simple_rgb.yaml
```

- Resumen global:

```bash
python3 scripts/summarize_results.py --experiments-root experiments --output reports/tables/summary_results.csv
```

**Ultima actualizacion:** 2026-03-07
