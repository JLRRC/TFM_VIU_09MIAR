# TRAZABILIDAD TFM: modelos, experimentos, resultados, tablas e ilustraciones

## Alcance

Este documento establece la trazabilidad operativa para la parte experimental de `agarre_inteligente`.

Regla de oro de trazabilidad:

`config + datos + script -> experimento -> metrics.csv -> resumen -> tabla/figura -> conclusion`

## Modelos y variantes

- Modelo B: `SimpleGraspCNN`
- Modelo A: `ResNet18Grasp`
- Modalidades: `RGB` y `RGB-D`

Implementaciones:

- `src/models/simple_cnn.py`
- `src/models/resnet_variants.py`

## Experimentos base

- `EXP1_SIMPLE_RGB` -> `config/exp1_simple_rgb.yaml`
- `EXP2_SIMPLE_RGBD` -> `config/exp2_simple_rgbd.yaml`
- `EXP3_RESNET18_RGB_AUGMENT` -> `config/exp3_resnet18_rgb_augment.yaml`
- `EXP4_RESNET18_RGBD` -> `config/exp4_resnet18_rgbd.yaml`

Runner por experimento:

- `scripts/run_experiment.py --config <yaml> [--allow-synthetic]`

Entrenamiento por seed:

- `scripts/train.py --config <yaml> --seed <n> [--allow-synthetic]`

## Artefactos primarios por seed

Salida por seed:

- `experiments/<EXP>/seed_<N>/metrics.csv`
- `experiments/<EXP>/seed_<N>/checkpoints/best.pth`
- `experiments/<EXP>/seed_<N>/checkpoints/last.pth`
- `experiments/<EXP>/seed_<N>/best_epoch.txt`
- `experiments/<EXP>/seed_<N>/config_snapshot.yaml`

Campos esperados en `metrics.csv` por epoca:

- `epoch`
- `train_loss`
- `val_loss`
- `val_success`
- `val_iou`
- `val_angle_deg`

## Best epoch y resumen por experimento

- Script: `scripts/select_best_epoch.py --experiment-dir experiments/<EXP>`
- Entrada: `experiments/<EXP>/seed_*/metrics.csv`
- Salida: `experiments/<EXP>/best_epoch_summary.csv`

## Agregacion global

- Script: `scripts/summarize_results.py --experiments-root experiments --output reports/tables/summary_results.csv`
- Entradas: `experiments/<EXP>/best_epoch_summary.csv`
- Salidas:
  - `reports/tables/summary_results.csv`
  - `reports/tables/results_by_seed.csv`

## Tablas del TFM (fuentes y scripts)

- Tabla agregada validacion:
  - Script: `scripts/generate_tables.py`
  - Entrada: `reports/tables/summary_results.csv`
  - Salida: `reports/tables/table_validation_aggregated.csv`

- Tabla metricas finales por experimento:
  - Script: `scripts/generate_tables.py`
  - Entrada: `reports/tables/summary_results.csv`
  - Salida: `reports/tables/table_metrics_final.csv`

- Comparativa A/B por modalidad:
  - Script: `scripts/generate_tables.py`
  - Entrada: `reports/tables/summary_results.csv`
  - Salida: `reports/tables/table_ab_comparison_by_modality.csv`

- Tabla de latencia:
  - Script: `scripts/benchmark_latency.py` + `scripts/generate_tables.py`
  - Entrada: checkpoints + config de experimento
  - Salida: `reports/bench/latency_results.csv` y `reports/tables/table_latency.csv`

## Figuras del TFM (fuentes y scripts)

- Curvas por epoca:
  - Script: `scripts/generate_figures.py`
  - Entrada: `experiments/<EXP>/seed_*/metrics.csv`
  - Salidas:
    - `reports/figures/loss_train_by_epoch.png`
    - `reports/figures/loss_val_by_epoch.png`
    - `reports/figures/val_success_by_epoch.png`
    - `reports/figures/val_iou_by_epoch.png`
    - `reports/figures/val_angle_deg_by_epoch.png`

- Barras agregadas finales:
  - Script: `scripts/generate_figures.py`
  - Entrada: `reports/tables/summary_results.csv`
  - Salidas:
    - `reports/figures/bar_val_success_final.png`
    - `reports/figures/bar_val_iou_final.png`
    - `reports/figures/bar_val_angle_final.png`

- Galeria cualitativa GT vs Pred:
  - Script: `scripts/generate_qualitative_overlays.py`
  - Entrada: `predictions.csv` generado por `scripts/evaluate.py`
  - Salida: `reports/figures/qualitative/overlay_*.png`

## Eficiencia y benchmark

- Script: `scripts/benchmark_latency.py`
- Mide:
  - `latency_ms_mean`
  - `latency_ms_std`
  - `latency_ms_p95`
  - `fps`
  - `n_params`
  - `model_size_mb`
- Salida: `reports/bench/latency_results.csv`

## Pipeline maestro

Script maestro de regeneracion:

- `scripts/regenerate_tfm_block.sh`

Secuencia:

1. Entrenamiento por experimento y seed
2. Seleccion de best_epoch por experimento
3. Resumen global
4. Generacion de figuras
5. Generacion de tablas
6. Validacion de artefactos

## Dependencia de datos reales

Para resultados cientificos del TFM se requiere dataset Cornell preparado en:

- `data/processed/cornell/`
- `data/processed/cornell/splits/object_wise/train.csv`
- `data/processed/cornell/splits/object_wise/val.csv`

Si no hay datos, se permite modo de validacion de pipeline con sinteticos:

- `--allow-synthetic`

Este modo solo valida integridad tecnica del flujo, no validez cientifica de resultados.

## Evidencia para conclusiones

Cada conclusion debe referenciar:

- Archivo fuente de metricas: `experiments/<EXP>/best_epoch_summary.csv`
- Resumen global: `reports/tables/summary_results.csv`
- Tabla derivada: `reports/tables/table_*.csv`
- Figura soporte: `reports/figures/*.png`
- Benchmark: `reports/bench/latency_results.csv`

## Estado de cierre (2026-03-07)

- Dataset Cornell real conectado y operativo en `data/processed/cornell/splits/object_wise/`.
- Entrenamiento y evaluacion completados para `EXP1..EXP4` con `n=3` seeds.
- Evaluacion Cornell corregida a protocolo multi-GT por imagen.
- Tablas y figuras regeneradas desde artefactos actualizados.

Pendiente opcional:

- Alinear narrativa del PDF final con los resultados corregidos del pipeline actual.
