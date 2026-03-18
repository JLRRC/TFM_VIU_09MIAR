# agarre_inteligente

Proyecto oficial de vision por computador del TFM.

Que contiene esta parte del proyecto:

- `src/`: implementacion principal del pipeline de vision.
  Incluye dataset, transformaciones, modelos, entrenamiento, evaluacion y utilidades.
- `config/`: configuraciones YAML oficiales de los cuatro experimentos base del TFM.
- `data/raw/cornell/`: copia de trabajo del Cornell Grasping Dataset utilizada en el proyecto.
- `data/processed/cornell/splits/object_wise/`: particion reproducible final usada en los experimentos del Capitulo 5.
- `experiments/`: resultados oficiales por experimento y por seed.
  Cada experimento conserva `metrics.csv`, `best_epoch.txt`, `config_snapshot.yaml`, checkpoints y `best_epoch_summary.csv`.
- `scripts/`: scripts activos para preparar datos, entrenar, evaluar y regenerar artefactos del TFM.
- `graspnet/`: codigo auxiliar heredado que se conserva por compatibilidad y referencias internas.
- `requirements.txt`, `bootstrap.sh`, `check_python_deps.sh`, `check_system.sh`: soporte de entorno y validacion local.

Experimentos oficiales del TFM:

- `EXP1_SIMPLE_RGB`
- `EXP2_SIMPLE_RGBD`
- `EXP3_RESNET18_RGB_AUGMENT`
- `EXP4_RESNET18_RGBD`

Scripts oficiales a priorizar:

- `scripts/prepare_cornell_csv.py`: genera la particion reproducible `train.csv` / `val.csv`.
- `scripts/train.py`: entrenamiento de una ejecucion concreta.
- `scripts/run_experiment.py`: lanza las seeds de un experimento completo.
- `scripts/evaluate.py`: evaluacion sobre validacion.
- `scripts/select_best_epoch.py`: seleccion de `best_epoch`.
- `scripts/regenerate_chapter5_post_retrain.py`: regeneracion curada del bloque oficial del Capitulo 5.
- `scripts/generar_ilustraciones_tfm_5_1.py`: curvas de `train_loss` / `val_loss`.
- `scripts/generate_boundary_failure_figure.py`: figura cualitativa del falso negativo plausible.
- `scripts/benchmark_latency.py`: medicion de latencia de inferencia.

Fuente oficial de verdad para resultados:

- Dataset final usado en Capitulo 5:
  `data/processed/cornell/splits/object_wise/train.csv`
  `data/processed/cornell/splits/object_wise/val.csv`
- Resultados por seed:
  `experiments/EXP*/seed_*/metrics.csv`
- Resumen por experimento:
  `experiments/EXP*/best_epoch_summary.csv`
- Metricas, tablas y figuras curadas para el documento:
  `../report/`

Nota:

- Los tamanos `train_size_expected` y `val_size_expected` que aparecen en algunos YAML son historicos y no deben tomarse como la fuente real del split final.
- La fuente metodologica vigente para el Capitulo 5 es la particion materializada en `train.csv` y `val.csv`.
