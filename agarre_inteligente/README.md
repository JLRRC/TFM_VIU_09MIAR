# Agarre Inteligente

Proyecto de deteccion de agarres 2D/2.5D para TFM con pipeline reproducible.

## Objetivo academico

Este repositorio reconstruye la parte experimental de `agarre_inteligente` para poder regenerar:

- modelos A/B (`SimpleGraspCNN`, `ResNet18Grasp`),
- experimentos base (`EXP1..EXP4`),
- resultados por epoca y por semilla,
- tablas y figuras de resultados,
- benchmark de latencia/FPS,
- trazabilidad para conclusiones del TFM.

## Estructura relevante

- `config/` configuraciones EXP1..EXP4
- `src/` implementacion de modelos, dataset, trainer, evaluator
- `scripts/` entrenamiento, evaluacion, resumen, tablas, figuras, benchmark
- `experiments/` artefactos por experimento/seed (`metrics.csv`, checkpoints)
- `reports/tables/` tablas regeneradas
- `reports/figures/` figuras regeneradas
- `reports/bench/` resultados de latencia
- `docs/TRAZABILIDAD_TFM.md` mapa de trazabilidad cientifica
- `docs/REGISTRO_RECONSTRUCCION.md` bitacora tecnica completa

## Setup rapido

```bash
./bootstrap.sh
source venv/bin/activate
./check_system.sh
./check_python_deps.sh
./check_project.sh
```

El `venv` oficial tambien se usa como fallback para el panel ROS 2 cuando no exista `.venv-tfm`.

## Ejecucion reproducible del bloque TFM

Con datos reales de Cornell preparados en `data/processed/cornell`:

```bash
bash scripts/regenerate_tfm_block.sh
```

Solo para validar pipeline tecnico sin dataset real:

```bash
ALLOW_SYNTHETIC=1 bash scripts/regenerate_tfm_block.sh
```

## Ejecucion manual por pasos

```bash
python3 scripts/run_experiment.py --config config/exp1_simple_rgb.yaml
python3 scripts/select_best_epoch.py --experiment-dir experiments/EXP1_SIMPLE_RGB
python3 scripts/summarize_results.py --experiments-root experiments --output reports/tables/summary_results.csv
python3 scripts/generate_figures.py --experiments-root experiments --summary reports/tables/summary_results.csv --out-dir reports/figures
python3 scripts/generate_tables.py --summary reports/tables/summary_results.csv --out-dir reports/tables
python3 scripts/validate_artifacts.py --strict
```

## Artefactos clave

- Por seed: `experiments/<EXP>/seed_<N>/metrics.csv`
- Best epoch por experimento: `experiments/<EXP>/best_epoch_summary.csv`
- Agregado global: `reports/tables/summary_results.csv`
- Tablas finales: `reports/tables/table_*.csv`
- Figuras finales: `reports/figures/*.png`
- Latencia/FPS: `reports/bench/latency_results.csv`

## Requisitos de datos

### Opción 1: Descarga automática (recomendado)

```bash
./scripts/download_cornell.sh
```

Este script descarga, extrae y procesa automáticamente el Cornell Grasping Dataset.
Ver documentación completa en: [docs/DATASET_SETUP.md](docs/DATASET_SETUP.md)

### Opción 2: Dataset ya descargado

Si ya tienes el dataset Cornell, procesa las anotaciones:

```bash
python3 scripts/prepare_cornell_csv.py \
    --raw-dir /ruta/a/cornell \
    --out-dir data/processed/cornell
```

### Estructura esperada

Split object-wise:

- `data/processed/cornell/splits/object_wise/train.csv` (3542 samples)
- `data/processed/cornell/splits/object_wise/val.csv` (1569 samples)

CSV esperado:

- `image_path,depth_path,cx,cy,w,h,angle_deg`

## Notas

- ROS 2/Gazebo no son requisito para entrenar/evaluar `agarre_inteligente`.
- La integracion ROS 2 del TFM se trata en `agarre_ros2_ws`.
- `requirements.txt` incluye las dependencias de UI necesarias para que el panel pueda reutilizar este entorno (`PyQt5`, OpenCV, NumPy 1.26.4).

## Estado actual (2026-03-07)

- Entrenamiento completado para `EXP1..EXP4` con `n=3` seeds por experimento.
- Evaluacion corregida en estilo Cornell: cada prediccion se compara contra todos los GT de la imagen (multi-GT), no contra un solo GT.
- Resumen vigente en `reports/tables/summary_results.csv`:
    - `EXP1_SIMPLE_RGB`: val_success media 0.2029
    - `EXP2_SIMPLE_RGBD`: val_success media 0.3786
    - `EXP3_RESNET18_RGB_AUGMENT`: val_success media 0.6769
    - `EXP4_RESNET18_RGBD`: val_success media 0.6541

Notas historicas previas se conservan en `OLD/` como referencia, pero no representan el estado vigente del pipeline.
