# reports

Directorio curado del documento del TFM y de sus artefactos finales.

## Contenido

- `figures/`: figuras finales organizadas por capitulo.
- `tables/`: tablas finales organizadas por capitulo.
- `metrics/raw/`: metricas copiadas desde los experimentos para trazabilidad.
- `metrics/aggregated/`: agregaciones intermedias.
- `metrics/validated/`: metricas finales que se deben citar.
- `history/`: material historico util para comparativas internas del documento.
- `evidence/`: evidencias funcionales y cualitativas.
- `logs/`: logs de entrenamiento, evaluacion y reproducibilidad.
- `exports/`: artefactos exportados por capitulo.

## Ubicaciones utiles

- Figuras del capitulo 5: `figures/cap5/`
- Tablas del capitulo 5: `tables/cap5/`
- Resumen validado del capitulo 5: `metrics/validated/chapter5_experiment_summary_validated.csv`
- Evidencia ROS 2: `evidence/ros2/`
- Evidencia del capitulo 5: `evidence/chapter5/`

## Uso

Este directorio no es para lanzar el sistema ROS 2. Sirve para:

- consultar las figuras y tablas finales
- revisar metricas validadas
- localizar evidencias que respaldan la memoria

Para regenerar artefactos del documento desde la raiz del proyecto:

```bash
./recrear_artefactos_tfm.sh
```

La parte operativa del panel y sus logs de ejecucion en caliente se encuentra en `../agarre_ros2_ws/` y `../auditoria/panel_audit/`.
