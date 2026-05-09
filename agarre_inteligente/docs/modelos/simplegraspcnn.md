# SimpleGraspCNN

## 1. Que es y donde esta en el repo

`SimpleGraspCNN` es la familia ligera usada en los experimentos oficiales:

- `EXP1_SIMPLE_RGB`
- `EXP2_SIMPLE_RGBD`

Codigo relevante:

- `src/models/simple_cnn.py`
- `src/models/factory.py`
- `config/exp1_simple_rgb.yaml`
- `config/exp2_simple_rgbd.yaml`
- `scripts/run_experiment.py`
- `scripts/train.py`
- `src/training/trainer.py`
- `src/evaluation/evaluator.py`

En el codigo la clase se llama `SimpleCNN`, pero la factoria la expone como
`SimpleGraspCNN`.

## 2. Arquitectura real codificada

Implementacion actual en `src/models/simple_cnn.py`:

- entrada `C x 224 x 224`, con `C=3` para RGB o `C=4` para RGB-D
- bloque 1: `Conv2d(C, 32, 3x3, padding=1) -> BatchNorm2d -> ReLU -> MaxPool2d(2)`
- bloque 2: `Conv2d(32, 64, 3x3, padding=1) -> BatchNorm2d -> ReLU -> MaxPool2d(2)`
- bloque 3: `Conv2d(64, 128, 3x3, padding=1) -> BatchNorm2d -> ReLU -> MaxPool2d(2)`
- cuello: `AdaptiveAvgPool2d((1,1))`
- cabeza: `Flatten -> Linear(128,128) -> ReLU -> Dropout(0.2) -> Linear(128,5)`

Los 5 valores de salida son:

- `cx`
- `cy`
- `w`
- `h`
- `angle_deg`

En entrenamiento no salen en pixeles/grados puros, sino normalizados:

- `cx, cy, w, h` en rango relativo respecto a la imagen redimensionada
- `angle` como `angle_deg / 90`

## 3. Pseudocodigo del forward

```text
function forward(x):
    x = conv3x3(C -> 32) + bn + relu
    x = maxpool(x)

    x = conv3x3(32 -> 64) + bn + relu
    x = maxpool(x)

    x = conv3x3(64 -> 128) + bn + relu
    x = maxpool(x)

    x = adaptive_avg_pool(x, output=(1,1))
    x = flatten(x)              # 128
    x = linear(128 -> 128)
    x = relu(x)
    x = dropout(x, p=0.2)
    x = linear(128 -> 5)
    return x
```

Con entrada `224 x 224`, el flujo espacial queda asi:

- `224 -> 112 -> 56 -> 28 -> 1`

## 4. Como se entrena en este repo

### 4.1 Pipeline real

El entrenamiento no esta hardcodeado dentro del modelo. El flujo real es:

```text
for seed in seeds_del_yaml:
    cargar YAML
    construir GraspDataset(train) y GraspDataset(val)
    aplicar Resize(224,224)
    aplicar augmentation si el YAML la activa
    construir SimpleGraspCNN desde factory.py
    crear optimizer Adam
    usar SmoothL1Loss oficial

    for epoch in 1..N:
        entrenar sobre train_loader
        evaluar sobre val_loader con Evaluator oficial
        guardar checkpoint last
        si val_success mejora:
            guardar checkpoint best

    escribir metrics.csv
    escribir best_epoch.txt
    guardar config_snapshot.yaml
```

### 4.2 Dataset y targets

`GraspDataset` hace lo siguiente:

1. Lee `train.csv` o `val.csv`.
2. Carga la imagen RGB.
3. Si la modalidad es `rgbd`, carga profundidad y la concatena como cuarto canal.
4. Redimensiona a `224 x 224`.
5. Reescala `cx, cy, w, h` al nuevo tamano.
6. Normaliza:
   - `cx/final_w`
   - `cy/final_h`
   - `w/final_w`
   - `h/final_h`
   - `angle_deg/90`

### 4.3 Configuraciones oficiales

| Variante | Experimento | Canales | Augmentation | Epochs | Batch | Optimizer | LR | Weight decay | Loss |
|---|---|---:|---|---:|---:|---|---:|---:|---|
| RGB | `EXP1_SIMPLE_RGB` | 3 | no | 10 | 32 | Adam | 0.0005 | 0.0001 | `SmoothL1Loss` |
| RGB-D | `EXP2_SIMPLE_RGBD` | 4 | si | 10 | 32 | Adam | 0.0010 | 0.0000 | `SmoothL1Loss` |

Augmentation real cuando `augmentation: true`:

- `RandomHorizontalFlip(p=0.5)`
- `RandomRotation(15)`
- `ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2)`

### 4.4 Evaluacion oficial

Las metricas oficiales de `EXP1` y `EXP2` usan:

- `Evaluator` de `src/evaluation/evaluator.py`
- `iou_axis_aligned_boxes`
- criterio Cornell con:
  - `IoU >= 0.25`
  - `error angular <= 30 grados`

Detalle importante:

- el evaluador agrupa todos los GT de una misma imagen
- una prediccion cuenta como acierto si al menos un GT de esa imagen cumple el criterio
- por tanto, estas metricas son las oficiales del bloque `EXP1..EXP4`

## 5. Que se ha obtenido

### 5.1 Resultados agregados oficiales

Fuente: `report/metrics/validated/chapter5_experiment_summary_validated.csv`

| Variante | Experimento | best_epoch medio | val_success | val_iou | val_angle_deg | val_loss |
|---|---|---:|---:|---:|---:|---:|
| RGB | `EXP1_SIMPLE_RGB` | 6.0000 +- 3.7417 | 0.2741 +- 0.0440 | 0.2254 +- 0.0496 | 17.1910 +- 1.3745 | 0.0321 +- 0.0008 |
| RGB-D | `EXP2_SIMPLE_RGBD` | 9.3333 +- 0.9428 | 0.3805 +- 0.0051 | 0.3169 +- 0.0033 | 17.8730 +- 0.3608 | 0.0320 +- 0.0001 |

Lectura rapida:

- la variante RGB-D mejora claramente el exito final frente a RGB
- el coste en perdida final es muy parecido
- el error angular medio sigue siendo relativamente alto frente a ResNet18

### 5.2 Detalle por seed

Fuente: `agarre_inteligente/experiments/EXP*/best_epoch_summary.csv`

| Experimento | Seed | best_epoch | val_success | val_iou | val_angle_deg | val_loss |
|---|---:|---:|---:|---:|---:|---:|
| `EXP1_SIMPLE_RGB` | 0 | 1 | 0.2218 | 0.1724 | 17.6584 | 0.0323 |
| `EXP1_SIMPLE_RGB` | 1 | 10 | 0.2709 | 0.2121 | 15.3233 | 0.0310 |
| `EXP1_SIMPLE_RGB` | 2 | 7 | 0.3295 | 0.2917 | 18.5913 | 0.0329 |
| `EXP2_SIMPLE_RGBD` | 0 | 10 | 0.3856 | 0.3125 | 18.3805 | 0.0322 |
| `EXP2_SIMPLE_RGBD` | 1 | 8 | 0.3735 | 0.3206 | 17.5741 | 0.0320 |
| `EXP2_SIMPLE_RGBD` | 2 | 10 | 0.3824 | 0.3175 | 17.6643 | 0.0319 |

### 5.3 Tamano y latencia

Fuente de latencia: `report/tables/cap5/Tabla_5-3_medicion_de_latencia_de_inferencia_por_experimento_y_dispositivo.csv`

| Variante | Parametros | Tamano aprox. fp32 | CPU ms | CPU FPS | CUDA ms | CUDA FPS |
|---|---:|---:|---:|---:|---:|---:|
| RGB | 110853 | 0.4229 MB | 5.5347 | 180.68 | 0.3339 | 2995.05 |
| RGB-D | 111141 | 0.4240 MB | 4.9699 | 201.21 | 0.3326 | 3006.29 |

Es un modelo muy pequeno y muy rapido.

## 6. Lectura tecnica

Fortalezas observadas:

- arquitectura minima y facil de mantener
- coste de inferencia muy bajo
- escala bien a RGB-D sin aumentar casi el numero de parametros

Limitaciones observadas:

- la capacidad de representacion es claramente menor que `ResNet18Grasp`
- en RGB puro el `val_success` oficial se queda en `0.2741`
- el `val_angle_deg` queda peor que en ResNet18

## 7. Resumen corto

`SimpleGraspCNN` es la baseline oficial y ligera del repo. Esta bien codificada,
es facil de entrenar y extremadamente rapida, pero su techo de precision queda
por debajo de `ResNet18Grasp`. Dentro de esta familia, `EXP2_SIMPLE_RGBD`
es la variante mas solida del bloque oficial.
