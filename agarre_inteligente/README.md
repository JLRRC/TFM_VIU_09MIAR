# agarre_inteligente

Bloque de vision del TFM. Aqui viven el pipeline de datos, los modelos de agarre, el entrenamiento, la evaluacion y los resultados por experimento.

## Estructura

- `config/`: configuraciones YAML activas de los experimentos.
- `data/raw/`: dataset Cornell bruto y recursos asociados.
- `data/processed/`: splits y derivados consumidos por entrenamiento y evaluacion.
- `experiments/`: salidas por experimento y por `seed`, incluyendo checkpoints y metricas.
- `scripts/`: utilidades de preparacion, entrenamiento, evaluacion, benchmarking y regeneracion de artefactos.
- `src/`: implementacion principal del pipeline.
- `graspnet/`: compatibilidad con layouts previos y utilidades heredadas.

## Modelos y experimentos

Experimentos base del capitulo 5:

- `EXP1_SIMPLE_RGB`
- `EXP2_SIMPLE_RGBD`
- `EXP3_RESNET18_RGB_AUGMENT`
- `EXP4_RESNET18_RGBD`

Experimentos adicionales conservados en el repo:

- `EXP1.1_SIMPLEGRASP_RGB` (config: `exp1_1_simplegrasp_rgb.yaml`)
- `EXP1.2_SIMPLEGRASP_RGBD` (config: `exp1_2_simplegrasp_rgbd.yaml`)

Las familias de modelos activas en el codigo son:

- `SimpleGraspCNN` / `simple_cnn`
- `SimpleGrasp` / `simple_grasp`
- `ResNet18Grasp` / `resnet18`

## Detalle del modelo del TFM

Conviene distinguir dos familias ligeras que hoy conviven en el repo:

- `EXP1` y `EXP2` usan `SimpleGraspCNN`, que en codigo se implementa como `SimpleCNN` en `src/models/simple_cnn.py`.
- `EXP1.1` y `EXP1.2` usan `SimpleGrasp`, implementado en `src/models/simple_grasp.py`.

La diferencia importa porque el apartado `4.6.2` de la memoria distingue entre:

- el diseño objetivo del modelo ligero documentado en teoria
- la version finalmente integrada y evaluada en el pipeline experimental

En la memoria entregada, el apartado `4.6.2` describe el diseño objetivo del modelo ligero. En el workspace actual ese diseño esta implementado en `SimpleGrasp`, mientras que el bloque experimental presentado en resultados se mantiene sobre `EXP1` y `EXP2`.

La lectura correcta para este workspace es la siguiente:

- el diseño de `4.6.2` esta implementado y disponible en codigo mediante `SimpleGrasp` y los experimentos `EXP1.1` y `EXP1.2`
- los resultados oficiales de la memoria, las tablas del capitulo 5 y la narrativa comparativa del documento siguen apoyandose en `EXP1` y `EXP2`, porque esos fueron los experimentos efectivamente ejecutados, trazados y consolidados en el estudio

Esto no es una contradiccion, sino una separacion entre:

- implementacion del diseño teorico
- resultados experimentales oficialmente presentados

La justificacion practica es sencilla:

- `EXP1` y `EXP2` son la base reproducible de las figuras, tablas y conclusiones publicadas en la memoria
- `EXP1.1` y `EXP1.2` amplian el workspace sin cambiar el bloque experimental presentado
- mantener ambos niveles separados preserva a la vez la fidelidad teorica del apartado `4.6.2` y la trazabilidad del bloque de resultados

Resumen practico:

- `SimpleGraspCNN` / `simple_cnn`:
  - tres convoluciones `3x3`
  - `MaxPool2d` entre bloques
  - `AdaptiveAvgPool2d(1,1)`
  - cabeza `128 -> 128 -> 5`
  - corresponde a la formulacion simple finalmente integrada y evaluada en el pipeline
  - es la familia usada en `EXP1` y `EXP2`
- `SimpleGrasp` / `simple_grasp`:
  - primera convolucion `7x7` con `stride=2`
  - `MaxPool2d` tras el primer bloque
  - bloques posteriores `32 -> 64 -> 128`
  - `AdaptiveAvgPool2d(7,7)`
  - cabeza `128*7*7 -> 256 -> 5`
  - corresponde al diseño objetivo descrito en la teoria del apartado `4.6.2`
  - es la familia que se ha dejado operativa en `EXP1.1` y `EXP1.2`

Correspondencia con el codigo:

- La seleccion del modelo se hace en `src/models/factory.py`.
- Si `model.name` es `SimpleGraspCNN`, la factoria construye `SimpleCNN`.
- Si `model.name` es `SimpleGrasp`, la factoria construye `SimpleGrasp`.

Por tanto, a efectos del workspace actual:

- `EXP1` y `EXP2` son la referencia de resultados de la memoria: son los experimentos que sustentan tablas, figuras y comparativas del documento.
- `EXP1.1` y `EXP1.2` materializan en codigo el diseño objetivo descrito en `4.6.2`.
- Si la pregunta es "que arquitectura recoge la teoria", la respuesta es `EXP1.1` y `EXP1.2`; si la pregunta es "que experimentos respaldan los resultados presentados en la memoria", la respuesta es `EXP1` y `EXP2`.

## Flujo tipico

Preparar CSVs del Cornell:

```bash
python3 scripts/prepare_cornell_csv.py
```

Lanzar un experimento:

```bash
python3 scripts/run_experiment.py --config config/exp3_resnet18_rgb_augment.yaml
```

Lanzar `EXP1.1` o `EXP1.2`:

```bash
python3 scripts/run_experiment.py --config config/exp1_1_simplegrasp_rgb.yaml
python3 scripts/run_experiment.py --config config/exp1_2_simplegrasp_rgbd.yaml
```

Entrenamiento directo:

```bash
python3 scripts/train.py --config config/exp3_resnet18_rgb_augment.yaml --seed 0
```

Evaluar un experimento:

```bash
python3 scripts/evaluate.py \
  --config config/exp3_resnet18_rgb_augment.yaml \
  --checkpoint experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth \
  --output reports/eval_exp3_seed0.csv
```

Bench de latencia:

```bash
python3 scripts/benchmark_latency.py \
  --config config/exp3_resnet18_rgb_augment.yaml \
  --checkpoint experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth \
  --output reports/bench/latency_exp3_seed0.csv
```

## Fuente de verdad

- Split operativo object-wise:
  - `data/processed/cornell/splits/object_wise/train.csv`
  - `data/processed/cornell/splits/object_wise/val.csv`
- Configuraciones activas:
  - `config/exp1_simple_rgb.yaml`
  - `config/exp2_simple_rgbd.yaml`
  - `config/exp3_resnet18_rgb_augment.yaml`
  - `config/exp4_resnet18_rgbd.yaml`
  - `config/exp1_1_simplegrasp_rgb.yaml`
  - `config/exp1_2_simplegrasp_rgbd.yaml`
- Resultados por seed:
  - `experiments/EXP*/seed_*/metrics.csv`
- Resumen por experimento:
  - `experiments/EXP*/best_epoch_summary.csv`

## Guardas de recreacion oficial

- La recreacion oficial del TFM debe mantenerse sobre `EXP1`, `EXP2`, `EXP3` y `EXP4`.
- `scripts/summarize_results.py` y `scripts/generate_figures.py` trabajan en modo oficial por defecto; solo incorporan `EXP1.1` y `EXP1.2` si se usa `--include-aux`.
- `scripts/validate_official_scope.py` verifica que los artefactos oficiales no hayan quedado contaminados por experimentos auxiliares.
- `scripts/regenerate_tfm_block.sh` y `scripts/generate_all_plots.py` ejecutan esa validacion de forma automatica.

El entrenamiento valida que el tamano real de los splits cargados coincida con lo declarado en cada YAML activo.

## Scripts mas usados

- `scripts/prepare_cornell_csv.py`
- `scripts/train.py`
- `scripts/run_experiment.py`
- `scripts/evaluate.py`
- `scripts/select_best_epoch.py`
- `scripts/predict.py`
- `scripts/benchmark_latency.py`
- `scripts/regenerate_chapter5_post_retrain.py`
- `scripts/generate_tfm_chapter_reports.py`
- `scripts/validate_artifacts.py`

## Relacion con `reports/`

Este directorio produce resultados tecnicos y artefactos intermedios. Las figuras, tablas y metricas finales que se citan en memoria se curan y publican en `../reports/`.
