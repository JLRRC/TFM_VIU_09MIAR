# agarre_inteligente

Proyecto oficial de vision por computador del TFM.

Contenido que permanece como oficial:

- `src/`: librerias de dataset, modelos, entrenamiento, evaluacion y utilidades.
- `config/`: configuraciones YAML de los experimentos oficiales.
- `data/`: dataset Cornell crudo y procesado utilizado en el TFM.
- `experiments/`: resultados oficiales por experimento y seed, incluyendo `best_epoch_summary.csv`, `metrics.csv`, `best_epoch.txt` y checkpoints.
- `scripts/`: scripts vigentes para entrenamiento, evaluacion y regeneracion de artefactos.
- `graspnet/`: codigo auxiliar mantenido por compatibilidad del proyecto.
- `requirements.txt`, `bootstrap.sh`, `check_python_deps.sh`, `check_system.sh`: soporte reproducible del entorno.

Scripts oficiales a priorizar:

- `scripts/train.py`
- `scripts/evaluate.py`
- `scripts/run_experiment.py`
- `scripts/select_best_epoch.py`
- `scripts/summarize_results.py`
- `scripts/generate_tables.py`
- `scripts/generar_ilustraciones_tfm_5_1.py`
- `scripts/generate_boundary_failure_figure.py`

Fuente oficial para metricas y figuras:

- Metrica primaria: `experiments/EXP*/best_epoch_summary.csv`
- Historicos por seed: `experiments/EXP*/seed_*/metrics.csv`
- Figuras y tablas curadas para el documento: `../report/`

Todo lo no esencial para reproducibilidad o trazabilidad academica se ha movido a `../BORRAR/agarre_inteligente_extra/`.
