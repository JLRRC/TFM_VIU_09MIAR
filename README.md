# TFM Workspace

Raiz oficial del TFM. La estructura visible valida es:

- `agarre_inteligente/`: codigo, datos, configuraciones y experimentos oficiales de vision.
- `agarre_ros2_ws/`: workspace ROS 2 oficial para panel, simulacion e integracion.
- `report/`: fuente oficial del TFM para figuras, tablas, metricas, logs y evidencias.
- `README.md`: este documento.
- `lanzar_panelv2.sh`: lanzador oficial del panel.
- `recrear_artefactos_tfm.sh`: recreador oficial de artefactos curados del capitulo 5.
- `recrear_experimentos_cap5_gpu.sh`: relanzador oficial de los experimentos EXP1..EXP4 sobre la GPU CUDA del entorno `.venv-tfm`.
- `09MIAR_10_Z_2025-26.TFM.V11.6.pdf`: copia actual del TFM en PDF para contraste editorial.

Reglas de limpieza aplicadas:

- `report/` pasa a ser la fuente de verdad editorial. El arbol antiguo `reports/` se ha movido a `BORRAR/root_extra/reports/`.
- Se han apartado a `BORRAR/` auditorias antiguas, `build/install/log`, herramientas de prueba, `venv`, manifests `.md` redundantes y scripts no esenciales.
- La revision temporal del PDF se ha archivado en `BORRAR/root_extra/final_closure_20260318/`.
- Se ha mantenido solo un `.md` activo por carpeta principal relevante: raiz, `agarre_inteligente/`, `agarre_ros2_ws/` y `report/`.
- La metrica oficial no sale del PDF: sale de `agarre_inteligente/experiments/` y de sus copias validadas en `report/metrics/`.

Fuente oficial de resultados:

- Figuras: `report/figures/`
- Tablas: `report/tables/`
- Historicos oficiales de trabajo: `report/history/`
- Metricas crudas por experimento y seed: `report/metrics/raw/experiments/`
- Metricas validadas para capitulo 5: `report/metrics/validated/chapter5_experiment_summary_validated.csv`
- Comparativa historica pre/post reentrenamiento: `report/metrics/aggregated/chapter5_pre_vs_post_retrain_comparison.csv`
- Evidencia ROS 2 y cualitativa: `report/evidence/`
- Logs de reproducibilidad y manifiestos de limpieza: `report/logs/reproducibility/`
- Indice de snapshots historicos preservados: `report/logs/reproducibility/historical_snapshots_index.tsv`

Nota tecnica:

- Se conservan `.git/` y `.gitignore` como metadatos ocultos del repositorio.
