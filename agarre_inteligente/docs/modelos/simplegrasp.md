# SimpleGrasp

## 1. Que es y donde esta en el repo

`SimpleGrasp` es la implementacion del modelo ligero alineado con la
arquitectura teorica del TFM. En este repo se usa en:

- `EXP1.1_SIMPLEGRASP_RGB`
- `EXP1.2_SIMPLEGRASP_RGBD`

Codigo relevante:

- `src/models/simple_grasp.py`
- `src/models/factory.py`
- `config/exp1_1_simplegrasp_rgb.yaml`
- `config/exp1_2_simplegrasp_rgbd.yaml`
- `scripts/run_experiment.py`
- `scripts/train.py`
- `src/training/trainer.py`
- `src/evaluation/evaluator.py`

Importante para trazabilidad:

- estos experimentos existen en el repo y estan operativos
- no son la base oficial de las tablas del capitulo 5
- si son la materializacion en codigo del diseno ligero descrito en teoria

## 2. Arquitectura real codificada

Implementacion actual en `src/models/simple_grasp.py`:

- entrada `C x 224 x 224`, con `C=3` o `C=4`
- bloque 1:
  - `Conv2d(C, 32, kernel=7, stride=2, padding=3)`
  - `BatchNorm2d`
  - `ReLU`
  - `MaxPool2d(2)`
- bloque 2:
  - `Conv2d(32, 64, kernel=3, stride=1, padding=1)`
  - `BatchNorm2d`
  - `ReLU`
  - `MaxPool2d(2)`
- bloque 3:
  - `Conv2d(64, 128, kernel=3, stride=1, padding=1)`
  - `BatchNorm2d`
  - `ReLU`
  - `AdaptiveAvgPool2d((7,7))`
- cabeza:
  - `Flatten`
  - `Linear(128*7*7, 256)`
  - `ReLU`
  - `Linear(256, 5)`

Salida:

- `cx`
- `cy`
- `w`
- `h`
- `angle_deg`

Normalizacion de targets:

- identica al resto del pipeline
- `cx, cy, w, h` relativos al tamano final
- `angle = angle_deg / 90`

## 3. Pseudocodigo del forward

```text
function forward(x):
    x = conv7x7_stride2(C -> 32) + bn + relu
    x = maxpool(x)

    x = conv3x3(32 -> 64) + bn + relu
    x = maxpool(x)

    x = conv3x3(64 -> 128) + bn + relu
    x = adaptive_avg_pool(x, output=(7,7))

    x = flatten(x)              # 128*7*7 = 6272
    x = linear(6272 -> 256)
    x = relu(x)
    x = linear(256 -> 5)
    return x
```

Con entrada `224 x 224`, el flujo espacial principal queda asi:

- `224 -> 112 -> 56 -> 28 -> 7`

## 4. Como se entrena en este repo

### 4.1 Pipeline real

El pipeline es el mismo de `scripts/run_experiment.py` + `scripts/train.py`,
pero con configs propias para `SimpleGrasp`:

```text
for seed in [0,1,2]:
    cargar YAML de EXP1.1 o EXP1.2
    construir dataset train/val
    resize a 224x224
    aplicar augmentation solo si el YAML la activa
    construir SimpleGrasp desde factory.py
    optimizer = Adam(...)
    criterion = SmoothL1Loss()

    for epoch in 1..10:
        entrenar
        evaluar con Evaluator oficial
        guardar checkpoints
        fijar best_epoch por max(val_success)
```

### 4.2 Configuraciones activas

| Variante | Experimento | Canales | Augmentation | Epochs | Batch | Optimizer | LR | Weight decay | Loss |
|---|---|---:|---|---:|---:|---|---:|---:|---|
| RGB | `EXP1.1_SIMPLEGRASP_RGB` | 3 | no | 10 | 32 | Adam | 0.0005 | 0.0001 | `SmoothL1Loss` |
| RGB-D | `EXP1.2_SIMPLEGRASP_RGBD` | 4 | si | 10 | 32 | Adam | 0.0010 | 0.0000 | `SmoothL1Loss` |

Detalles diferenciales frente a `EXP1/EXP2`:

- `data_root` queda en `..`
- `train_size_expected` en estas configs es `3541`
- `val_size_expected` es `1569`

### 4.3 Nota de trazabilidad importante

El estado actual del repo ya incorpora el retrain corregido de `EXP1.1` y
`EXP1.2`. La correccion clave fue dejar `data_root: ".."` para no romper las
rutas al dataset en estas variantes.

Eso significa que los resultados actuales de `SimpleGrasp` en este workspace
deben leerse como los resultados corregidos y no como una corrida defectuosa
antigua.

### 4.4 Evaluacion

Igual que en el bloque oficial:

- `Evaluator` oficial
- `iou_axis_aligned_boxes`
- criterio Cornell:
  - `IoU >= 0.25`
  - `error angular <= 30 grados`

## 5. Que se ha obtenido

### 5.1 Resultados agregados en el repo

Fuente: `agarre_inteligente/experiments/EXP1.1_SIMPLEGRASP_RGB/best_epoch_summary.csv`
y `agarre_inteligente/experiments/EXP1.2_SIMPLEGRASP_RGBD/best_epoch_summary.csv`

| Variante | Experimento | best_epoch medio | val_success | val_iou | val_angle_deg | val_loss |
|---|---|---:|---:|---:|---:|---:|
| RGB | `EXP1.1_SIMPLEGRASP_RGB` | 6.6667 +- 1.6997 | 0.3646 +- 0.0828 | 0.2477 +- 0.0660 | 14.5377 +- 1.2565 | 0.0336 +- 0.0019 |
| RGB-D | `EXP1.2_SIMPLEGRASP_RGBD` | 2.3333 +- 1.2472 | 0.4143 +- 0.0330 | 0.3079 +- 0.0422 | 17.1200 +- 0.1670 | 0.0334 +- 0.0005 |

Lectura rapida:

- RGB-D vuelve a ser la variante mas robusta
- en este workspace `SimpleGrasp` mejora a `SimpleGraspCNN` en `val_success`
  tanto en RGB como en RGB-D
- no obstante, sus resultados siguen por debajo de `ResNet18Grasp`

### 5.2 Detalle por seed

| Experimento | Seed | best_epoch | val_success | val_iou | val_angle_deg | val_loss |
|---|---:|---:|---:|---:|---:|---:|
| `EXP1.1_SIMPLEGRASP_RGB` | 0 | 6 | 0.2683 | 0.1607 | 16.2624 | 0.0360 |
| `EXP1.1_SIMPLEGRASP_RGB` | 1 | 5 | 0.4704 | 0.3204 | 14.0459 | 0.0335 |
| `EXP1.1_SIMPLEGRASP_RGB` | 2 | 9 | 0.3550 | 0.2620 | 13.3049 | 0.0313 |
| `EXP1.2_SIMPLEGRASP_RGBD` | 0 | 2 | 0.3881 | 0.2525 | 17.1993 | 0.0335 |
| `EXP1.2_SIMPLEGRASP_RGBD` | 1 | 4 | 0.3939 | 0.3164 | 17.2731 | 0.0327 |
| `EXP1.2_SIMPLEGRASP_RGBD` | 2 | 1 | 0.4608 | 0.3547 | 16.8877 | 0.0339 |

### 5.3 Tamano del modelo

Calculado localmente con la implementacion actual:

| Variante | Parametros | Tamano aprox. fp32 |
|---|---:|---:|
| RGB | 1704709 | 6.5029 MB |
| RGB-D | 1706277 | 6.5089 MB |

No existe en `report/` un benchmark de latencia curado y oficial para
`SimpleGrasp`, asi que aqui conviene no mezclarlo con las tablas oficiales del
capitulo 5.

## 6. Lectura tecnica

Fortalezas observadas:

- representa mejor el modelo ligero teorico que `SimpleGraspCNN`
- la cabeza densa tiene mas capacidad
- mejora las metricas de la baseline ligera en este workspace

Limitaciones observadas:

- sigue siendo claramente inferior a `ResNet18Grasp`
- la variabilidad por seed en RGB aun es apreciable
- la mejora de capacidad incrementa mucho el numero de parametros frente a
  `SimpleGraspCNN`

## 7. Resumen corto

`SimpleGrasp` es el ligero "serio" del repo: mantiene simplicidad, pero ya no
es una baseline minima. En el estado actual del workspace sus resultados son
mejores que `SimpleGraspCNN`, sobre todo como implementacion fiel del diseno
teorico, pero la referencia mas fuerte en precision sigue siendo
`ResNet18Grasp`.
