# report

Fuente oficial del TFM.

Estructura:

- `history/`: historicos oficiales preservados para comparativa, sin mezclar con residuos de limpieza.
- `figures/`: figuras finales organizadas por capitulo.
- `tables/`: tablas finales organizadas por capitulo.
- `metrics/raw/experiments/`: copia trazable de metricas crudas por experimento y seed.
- `metrics/aggregated/`: agregaciones intermedias curadas.
- `metrics/validated/`: resumenes validados que deben citarse en el documento.
- `metrics/aggregated/chapter5_pre_vs_post_retrain_comparison.csv`: comparativa entre el estado historico previo al reentrenamiento y el estado oficial actual.
- `logs/training/`: lanzamientos, colas de reentrenamiento y logs oficiales de los EXP del capitulo 5.
- `logs/evaluation/`: logs de evaluacion y ejecucion relevantes.
- `logs/reproducibility/`: manifiestos de limpieza, criterios de regeneracion y logs ROS 2 reproducibles.
- `logs/reproducibility/historical_snapshots_index.tsv`: indice unico de snapshots historicos conservados en `BORRAR/`.
- `exports/chapter_artifacts/`: artefactos no textuales asociados a capitulos.
- `evidence/chapter5/`: evidencia de regeneracion y seleccion cualitativa del capitulo 5.
- `evidence/ros2/`: evidencias funcionales ROS 2 + Gazebo.
- `scripts/`: reservado para utilidades de curacion de report si hicieran falta.

Reglas curatoriales:

- Las figuras del capitulo 5 en `figures/cap5/` siguen la numeracion oficial alineada con el PDF.
- Los historicos utiles ya no viven en `BORRAR/`; el archivo oficial para comparativas queda en `history/chapter5/`.
- `Ilustracion_5-15` oficial es el caso limite de falso negativo visualmente plausible, no el caso antiguo de objeto pequeno.
- La referencia numerica oficial para resultados del capitulo 5 es `metrics/validated/chapter5_experiment_summary_validated.csv`.
- La trazabilidad de lo movido a `BORRAR/` queda en `logs/reproducibility/borrar_manifest.tsv`.
