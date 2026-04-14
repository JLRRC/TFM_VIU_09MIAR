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

Experimentos auxiliares conservados en el repo:

- `EXP5_SIMPLEGRASP_RGB`
- `EXP6_SIMPLEGRASP_RGBD`

Las familias de modelos activas en el codigo son:

- `SimpleGraspCNN` / `simple_cnn`
- `ResNet18Grasp` / `resnet18`

## Flujo tipico

Preparar CSVs del Cornell:

```bash
python scripts/prepare_cornell_csv.py
```

Lanzar un experimento:

```bash
python scripts/run_experiment.py --config config/exp3_resnet18_rgb_augment.yaml
```

Entrenamiento directo:

```bash
python scripts/train.py --config config/exp3_resnet18_rgb_augment.yaml --seed 0
```

Evaluar un experimento:

```bash
python scripts/evaluate.py --experiment experiments/EXP3_RESNET18_RGB_AUGMENT
```

Bench de latencia:

```bash
python scripts/benchmark_latency.py --config config/exp3_resnet18_rgb_augment.yaml
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
- Resultados por seed:
  - `experiments/EXP*/seed_*/metrics.csv`
- Resumen por experimento:
  - `experiments/EXP*/best_epoch_summary.csv`

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
