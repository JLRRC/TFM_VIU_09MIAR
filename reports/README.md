# reports

Directorio curado del documento del TFM y de los artefactos que respaldan la memoria final.

## Que hay aqui

- `09MIAR_10_Z_2025-26.TFM..DEFINITIVO.pdf`: memoria final de referencia.
- `figures/`: figuras finales organizadas por capitulo.
- `tables/`: tablas finales organizadas por capitulo.
- `metrics/raw/`: metricas copiadas desde los experimentos para trazabilidad.
- `metrics/aggregated/`: agregaciones intermedias.
- `metrics/validated/`: metricas finales que deben citarse.
- `evidence/`: evidencias funcionales y cualitativas.
- `logs/`: logs de entrenamiento, regeneracion y reproducibilidad.
- `exports/`: exportaciones por capitulo o por bloque.
- `bench/`: mediciones de latencia y benchmarks auxiliares.
- `cornell_audit/`: soporte minimo de auditoria del split Cornell.
- `history/`: snapshots historicos utiles para comparativas internas.

## Ubicaciones canonicas

- Figuras del capitulo 5: `figures/cap5/`
- Tablas del capitulo 5: `tables/cap5/`
- Tablas de anexos: `tables/anexos/`
- Resumen validado del capitulo 5:
  - `metrics/validated/chapter5_experiment_summary_validated.csv`
- Evidencia del bloque TFM y casos cualitativos:
  - `evidence/chapter5/`
- Evidencia ROS 2 y auditoria del panel:
  - `evidence/ros2/`
  - `evidence/ros2/tfm_panel_repro_seed0_raw_trace_20260415.md`
  - `evidence/ros2/tfm_session_exports/`
- Nota de rigor del workspace:
  - `evidence/workspace_rigour_notes_20260415.md`
- Trazabilidad de `EXP1.1` y `EXP1.2` como implementacion de `4.6.2`:
  - `evidence/exp1_1_exp1_2_theoretical_implementation_trace_20260415.md`
- Logs de regeneracion y trazabilidad:
  - `logs/reproducibility/`

## Notas de rigor del workspace

- El split Cornell operativo actual queda fijado en `3541` muestras de entrenamiento y `1569` de validacion.
- Esta cifra sustituye a recuentos historicos mas altos porque el `train.csv` activo se saneo para eliminar una fila corrupta con valores no finitos; el objetivo es que el CSV y el dataset realmente cargado coincidan sin ambiguedad.
- La recreacion oficial del TFM sigue referida a `EXP1`, `EXP2`, `EXP3` y `EXP4`.
- `EXP1.1` y `EXP1.2` quedan en el workspace como implementacion en codigo del diseño teorico descrito en `4.6.2`, pero no forman parte de las tablas y figuras oficiales del documento.
- En el panel pueden usarse para inferencia como variantes auxiliares, sin mezclar su uso con la reproduccion oficial del bloque experimental de la memoria.

## Uso esperado

Este directorio no es el lugar para arrancar ROS 2 ni para entrenar modelos. Sirve para:

- consultar figuras y tablas finales
- revisar metricas validadas
- localizar evidencias que respaldan la memoria
- seguir la trazabilidad entre resultados crudos y artefactos publicados

## Regeneracion

Desde la raiz del proyecto:

```bash
./recrear_artefactos_tfm.sh
```

Para la tabla de latencia:

```bash
./recrear_tabla_5_3_latencia.sh
```

## Relacion con el resto del repo

- `../agarre_inteligente/` produce experimentos, checkpoints y metricas base.
- `../agarre_ros2_ws/` produce evidencias operativas, logs de panel y trazas ROS 2.
- `history/` conserva material historico; no siempre coincide con el estado operativo actual.
