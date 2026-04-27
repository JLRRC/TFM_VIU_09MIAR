# ResNet18Grasp

## 1. Que es y donde esta en el repo

`ResNet18Grasp` es la familia profunda de referencia usada en:

- `EXP3_RESNET18_RGB_AUGMENT`
- `EXP4_RESNET18_RGBD`

Codigo relevante:

- `src/models/resnet_variants.py`
- `src/models/factory.py`
- `config/exp3_resnet18_rgb_augment.yaml`
- `config/exp4_resnet18_rgbd.yaml`
- `scripts/run_experiment.py`
- `scripts/train.py`
- `src/training/trainer.py`
- `src/evaluation/evaluator.py`

Es la arquitectura con mejores resultados oficiales del bloque experimental del
TFM.

## 2. Arquitectura real codificada

Implementacion actual en `src/models/resnet_variants.py`:

- backbone base: `torchvision.models.resnet18`
- si `pretrained: true`, usa `ResNet18_Weights.DEFAULT`
- si `input_channels != 3`, sustituye `conv1` para aceptar mas canales
- cabeza final original `fc` se reemplaza por:
  - `Dropout(0.2)`
  - `Linear(in_features, 5)`
- `freeze_backbone` existe en codigo, pero en las configs oficiales esta en
  `false`

Salida:

- `cx`
- `cy`
- `w`
- `h`
- `angle_deg`

Targets:

- los mismos del resto del pipeline
- coordenadas y tamano normalizados
- angulo normalizado como `angle_deg/90`

## 3. Pseudocodigo del forward

```text
function build_model(input_channels, pretrained, dropout):
    backbone = resnet18(pretrained=pretrained)

    if input_channels != 3:
        old_conv1 = backbone.conv1
        backbone.conv1 = new Conv2d(input_channels -> 64, 7x7, stride=2, padding=3)
        copiar pesos RGB de old_conv1 a los 3 primeros canales
        si hay canal extra:
            inicializarlo copiando el primer canal preentrenado

    backbone.fc = Dropout(dropout) + Linear(512 -> 5)
    return backbone

function forward(x):
    return resnet18_backbone_y_head(x)
```

En la variante RGB-D, la adaptacion clave esta en `conv1`.

## 4. Como se entrena en este repo

### 4.1 Pipeline real

El flujo sigue `run_experiment.py -> train.py -> trainer.py`:

```text
for seed in [0,1,2]:
    cargar YAML de EXP3 o EXP4
    construir datasets Cornell
    resize a 224x224
    aplicar augmentation segun config
    construir ResNet18Grasp con pretrained=true
    optimizer = Adam(...)
    criterion = SmoothL1Loss()

    for epoch in 1..50:
        entrenar en train
        evaluar en val con Evaluator oficial
        guardar checkpoints
        si mejora val_success:
            guardar best.pth
```

### 4.2 Configuraciones oficiales

| Variante | Experimento | Canales | Pretrained | Freeze backbone | Augmentation | Epochs | Batch | LR | Weight decay | Loss |
|---|---|---:|---|---|---|---:|---:|---:|---:|---|
| RGB | `EXP3_RESNET18_RGB_AUGMENT` | 3 | si | no | si | 50 | 32 | 0.0005 | 0.0001 | `SmoothL1Loss` |
| RGB-D | `EXP4_RESNET18_RGBD` | 4 | si | no | no | 50 | 32 | 0.0005 | 0.0001 | `SmoothL1Loss` |

Augmentation real en la variante RGB:

- `RandomHorizontalFlip(p=0.5)`
- `RandomRotation(15)`
- `ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2)`

### 4.3 Evaluacion oficial

Igual que en `SimpleGraspCNN`:

- `Evaluator` oficial
- `iou_axis_aligned_boxes`
- criterio Cornell:
  - `IoU >= 0.25`
  - `error angular <= 30 grados`

Esto importa porque las metricas oficiales de `EXP3` y `EXP4` no usan todavia
la variante orientada posterior del repo.

## 5. Que se ha obtenido

### 5.1 Resultados agregados oficiales

Fuente: `report/metrics/validated/chapter5_experiment_summary_validated.csv`

| Variante | Experimento | best_epoch medio | val_success | val_iou | val_angle_deg | val_loss |
|---|---|---:|---:|---:|---:|---:|
| RGB | `EXP3_RESNET18_RGB_AUGMENT` | 29.6667 +- 12.4722 | 0.6801 +- 0.0245 | 0.4010 +- 0.0046 | 10.6071 +- 1.1800 | 0.0297 +- 0.0007 |
| RGB-D | `EXP4_RESNET18_RGBD` | 43.6667 +- 6.9442 | 0.6492 +- 0.0102 | 0.3879 +- 0.0148 | 11.4260 +- 0.5339 | 0.0283 +- 0.0006 |

Lectura rapida:

- es la familia con mejor `val_success` del bloque oficial
- la variante RGB oficial sale ligeramente mejor que la RGB-D en `val_success`
- tambien consigue mejor error angular medio que los modelos ligeros

### 5.2 Detalle por seed

| Experimento | Seed | best_epoch | val_success | val_iou | val_angle_deg | val_loss |
|---|---:|---:|---:|---:|---:|---:|
| `EXP3_RESNET18_RGB_AUGMENT` | 0 | 13 | 0.6902 | 0.4023 | 9.7076 | 0.0301 |
| `EXP3_RESNET18_RGB_AUGMENT` | 1 | 33 | 0.6463 | 0.4059 | 12.2741 | 0.0303 |
| `EXP3_RESNET18_RGB_AUGMENT` | 2 | 43 | 0.7036 | 0.3948 | 9.8396 | 0.0288 |
| `EXP4_RESNET18_RGBD` | 0 | 34 | 0.6558 | 0.3961 | 12.1646 | 0.0289 |
| `EXP4_RESNET18_RGBD` | 1 | 47 | 0.6348 | 0.3671 | 10.9212 | 0.0285 |
| `EXP4_RESNET18_RGBD` | 2 | 50 | 0.6571 | 0.4005 | 11.1922 | 0.0274 |

### 5.3 Comparativa oficial frente a SimpleGraspCNN

Fuente: `report/tables/cap5/Tabla_5-4_comparativa_por_modalidad_entre_simplegraspcnn_y_resnet18grasp.csv`

| Modalidad | val_success ResNet18Grasp | val_success SimpleGraspCNN | Delta absoluto |
|---|---:|---:|---:|
| RGB | 0.6801 | 0.2741 | 0.4060 |
| RGB-D | 0.6492 | 0.3805 | 0.2687 |

### 5.4 Tamano y latencia

Fuente: `report/tables/cap5/Tabla_5-3_medicion_de_latencia_de_inferencia_por_experimento_y_dispositivo.csv`

| Variante | Parametros | Tamano aprox. fp32 | CPU ms | CPU FPS | CUDA ms | CUDA FPS |
|---|---:|---:|---:|---:|---:|---:|
| RGB | 11179077 | 42.6448 MB | 7.6887 | 130.06 | 1.3091 | 763.87 |
| RGB-D | 11182213 | 42.6568 MB | 7.7802 | 128.53 | 1.3548 | 738.13 |

Es bastante mas pesado que los modelos ligeros, pero el salto de precision
compensa en el bloque oficial.

## 6. Lectura tecnica

Fortalezas observadas:

- mejor `val_success` oficial del repo
- mejor `val_iou` oficial del repo
- error angular bastante mejor que en los modelos ligeros
- el backbone preentrenado reduce la dependencia de una cabeza muy pequena

Limitaciones observadas:

- coste de memoria y latencia mucho mayor que en `SimpleGraspCNN`
- el mejor epoch suele aparecer bastante mas tarde
- la variante RGB-D no supera a la RGB oficial en el estado actual del repo

## 7. Resumen corto

`ResNet18Grasp` es la opcion de mayor calidad en el workspace para el bloque
experimental oficial. Es mas caro y mas lento, pero es la arquitectura que
realmente empuja el rendimiento del capitulo 5 y la referencia practica si la
prioridad es precision y no minimalismo.
