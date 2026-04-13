# agarre_inteligente

Bloque de vision del TFM. Aqui estan el codigo de entrenamiento, evaluacion, configuraciones y resultados de los experimentos oficiales.

## Contenido

- `src/`: implementacion principal del pipeline de vision.
- `config/`: configuraciones oficiales de los experimentos.
- `data/raw/`: datos de entrada.
- `data/processed/`: particiones y derivados usados en entrenamiento y evaluacion.
- `experiments/`: resultados por experimento y por seed.
- `scripts/`: automatizaciones para preparar datos, entrenar, evaluar y regenerar artefactos.
- `graspnet/`: codigo auxiliar conservado por compatibilidad.

## Experimentos oficiales

- `EXP1_SIMPLE_RGB`
- `EXP2_SIMPLE_RGBD`
- `EXP3_RESNET18_RGB_AUGMENT`
- `EXP4_RESNET18_RGBD`
- `EXP5_SIMPLEGRASP_RGB` Modelo definido en el TFM apartado 4.6.2
- `EXP6_SIMPLEGRASP_RGBD` Modelo definido en el TFM apartado 4.6.2

## Scripts principales

- `scripts/prepare_cornell_csv.py`
- `scripts/train.py`
- `scripts/run_experiment.py`
- `scripts/evaluate.py`
- `scripts/select_best_epoch.py`
- `scripts/regenerate_chapter5_post_retrain.py`
- `scripts/benchmark_latency.py`

## Fuente de verdad de resultados

- Resultados por seed: `experiments/EXP*/seed_*/metrics.csv`
- Resumen por experimento: `experiments/EXP*/best_epoch_summary.csv`
- Split usado en el capitulo 5:
  - `data/processed/cornell/splits/object_wise/train.csv`
  - `data/processed/cornell/splits/object_wise/val.csv`

## Uso tipico

Preparar datos:

```bash
python scripts/prepare_cornell_csv.py
```

Lanzar un experimento:

```bash
python scripts/run_experiment.py --config config/exp3_resnet18_rgb_augment.yaml
```

Evaluar:

```bash
python scripts/evaluate.py --experiment experiments/EXP3_RESNET18_RGB_AUGMENT
```

Las figuras, tablas y metricas curadas que acaban en la memoria se publican en `../reports/`.
