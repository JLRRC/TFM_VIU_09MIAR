# TFM

Workspace principal del TFM de percepcion e inferencia de agarre para UR5 con ROS 2, Gazebo y MoveIt 2.

## Mapa rapido

- `agarre_inteligente/`: bloque de vision, entrenamiento, evaluacion y resultados por experimento.
- `agarre_ros2_ws/`: workspace ROS 2 del panel, simulacion, planificacion e integracion del bloque TFM.
- `reports/`: artefactos curados de memoria, metricas validadas, evidencias y exportaciones, incluyendo el PDF final actualmente tomado como referencia.

## Flujos principales

Arrancar el panel completo desde la raiz:

```bash
./lanzar_panelv2.sh --fg
```

Parar el stack ROS 2:

```bash
./agarre_ros2_ws/scripts/stop_panel_v2.sh
```

Relanzar entrenamientos base del capitulo 5:

```bash
./recrear_experimentos_cap5_gpu.sh
```

Regenerar artefactos curados del documento:

```bash
./recrear_artefactos_tfm.sh
```

Regenerar la tabla de latencia de inferencia:

```bash
./recrear_tabla_5_3_latencia.sh
```

## Fuente de verdad por area

- Vision y resultados experimentales: `agarre_inteligente/`
- Integracion ROS 2 y panel: `agarre_ros2_ws/`
- Figuras, tablas y metricas que respaldan la memoria: `reports/`

En vision conviven dos familias ligeras:

- `EXP1` y `EXP2`: referencia de resultados de la memoria; son los experimentos realmente usados en tablas, figuras y comparativas.
- `EXP1.1` y `EXP1.2`: experimentos adicionales que materializan en codigo el diseño objetivo descrito en `4.6.2`.

Notas de rigor:

- La recreacion oficial del TFM se mantiene sobre `EXP1..EXP4`.
- El PDF de referencia del TFM en este workspace es `reports/TFM_Lozano_Rodriguez-Jesus.pdf`.
- Todos los experimentos disponibles pueden cargarse y usarse desde el panel de inferencia.

## Convenciones utiles

- El codigo editable vive sobre todo en `agarre_inteligente/` y `agarre_ros2_ws/src/`.
- Solo existen `README` en la raiz y en los bloques principales del proyecto; cada uno explica su zona.

## Siguiente lectura

- `agarre_inteligente/README.md`
- `agarre_ros2_ws/README.md`
- `reports/README.md`

## Nota final sobre el split Cornell

- El TFM documenta el split final como `3542/1569`.
- En el workspace actual se asume como estado operativo real `3541/1569`.
- La causa es una anotacion corrupta en `agarre_inteligente/data/raw/cornell/01/pcd0165cpos.txt`, donde aparece un rectangulo con vertices `NaN NaN`.
- El generador de CSV puede volver a materializar `3542` filas en `train.csv`, pero una de ellas queda no finita y el dataset efectivo la descarta al cargar.
- Mientras no aparezca una version valida de esa anotacion dentro del propio workspace o de una copia externa fiable, la referencia tecnica real del repo debe considerarse `3541/1569`.

## Nota final sobre el panel

- En la memoria se cita `main_panel.py` como artefacto principal del panel.
- En el workspace actual el panel operativo y mantenido es `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py`.
- `main_panel.py` no existe ya en el arbol actual del repositorio.
- Se asume esta discrepancia como una diferencia literal entre la redaccion del TFM y la evolucion posterior del workspace, sin impacto sobre los resultados presentados.

## Nota final sobre los desajustes metodologicos no aplicados

- Se reconocen tres desajustes metodologicos pendientes respecto a la memoria: la IoU Cornell con rectangulos orientados, el calculo de `grasp success` por imagen y una posible funcion de perdida mas alineada con la formulacion teorica.
- No se aplican cambios sobre esos puntos en este workspace porque dejarian de ser estrictamente comparables los resultados ya presentados en el TFM.
- Ajustar la IoU y el criterio de `grasp success` obligaria como minimo a reevaluar `EXP1..EXP4`.
- Cambiar la funcion de perdida obligaria a reentrenar los experimentos oficiales.
- Por tanto, el estado actual del repositorio conserva deliberadamente la base experimental ya consolidada en la memoria, y estos puntos quedan solo anotados como trabajo pendiente no ejecutado.
