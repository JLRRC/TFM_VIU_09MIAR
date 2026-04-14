# agarre_ros2_ws

Workspace ROS 2 del TFM para panel, simulacion y planificacion del UR5 con Gazebo y MoveIt 2.
Se conserva el nombre `agarre_ros2_ws` por compatibilidad con scripts y rutas absolutas ya validadas.

## Estructura

- `src/ur5_qt_panel/`: panel Qt principal y logica de UI.
- `src/tfm_grasping/`: wrapper del modelo de agarre e inferencia ROS 2.
- `src/ur5_bringup/`: lanzamiento del stack y del mundo de simulacion.
- `src/ur5_moveit_config/`: configuracion de MoveIt 2.
- `src/ur5_description/`: descripcion del robot, controladores y modelos asociados.
- `src/ur5_tools/`: bridges, backends de attach y utilidades auxiliares.
- `src/ur5_panel_interfaces/`: interfaces ROS 2 del panel.
- `scripts/`: arranque, parada, validacion, benchmarks y diagnostico.
- `worlds/`: mundos SDF.
- `models/`: modelos de Gazebo.

Directorios generados:

- `build/`
- `install/`
- `log/`

## Arranque

Desde la raiz del proyecto:

```bash
./lanzar_panelv2.sh --fg
```

Desde este workspace:

```bash
./scripts/start_panel_v2.sh --fg
```

Modo reproduccion TFM del caso principal documentado:

```bash
./scripts/start_panel_v2.sh --fg --tfm-repro
```

Prediccion raw sin ajustes del panel:

```bash
./scripts/start_panel_v2.sh --fg --tfm-raw
```

Tambien puede activarse por entorno:

```bash
PANEL_TFM_REPRO_MODE=1 ./scripts/start_panel_v2.sh --fg
```

```bash
PANEL_TFM_RAW_OUTPUT=1 ./scripts/start_panel_v2.sh --fg
```

Parada limpia:

```bash
./scripts/stop_panel_v2.sh
```

Estado del stack:

```bash
./scripts/status_panel_v2.sh
```

Recuperacion del panel:

```bash
./scripts/recover_panel_v2.sh
```

## Bloque `TFM - Agarre Inteligente`

El panel incluye un bloque dedicado al TFM con este flujo base:

1. Aplicar experimento
2. Inferir agarre
3. Comparar grasp/ref
4. Ejecutar agarre

Notas importantes:

- El selector trabaja con los experimentos `EXP1` a `EXP4`.
- En modo normal, el panel puede autoelegir checkpoints segun `val_success`.
- En modo `--tfm-repro`, fija el caso `EXP3_RESNET18_RGB_AUGMENT / seed_0`.
- Puede ejecutarse tambien en modo `raw` sin los ajustes heurísticos posteriores del panel.
- La UI y los logs indican ahora tanto la politica de seleccion del checkpoint como si la prediccion ha recibido ajustes posteriores de angulo, centro o tamano.

## Scripts operativos utiles

Arranque y estado:

- `scripts/start_panel_v2.sh`
- `scripts/start_panel_v2_venv.sh`
- `scripts/start_panel_with_xvfb.sh`
- `scripts/status_panel_v2.sh`
- `scripts/stop_panel_v2.sh`
- `scripts/recover_panel_v2.sh`

Validacion y diagnostico:

- `scripts/validate_panel_flow.sh`
- `scripts/validate_startup_repro.sh`
- `scripts/validate_pick_3_cycles.sh`
- `scripts/panel_block_smoke_test.sh`
- `scripts/tfm_smoketest.py`
- `scripts/diag_tf_tcp.sh`
- `scripts/diag_startup_health.sh`
- `scripts/audit_moveit2_system.py`

Benchmark y trazas:

- `scripts/bench_infer_log.py`
- `scripts/grasp_audit_benchmark.py`
- `scripts/grasp_audit_trace_capture.py`
- `scripts/panel_perf_measure.py`
- `scripts/panel_perf_compare.py`

## Paquetes ROS 2 presentes

- `tfm_grasping`
- `ur5_bringup`
- `ur5_description`
- `ur5_moveit_config`
- `ur5_panel_interfaces`
- `ur5_qt_panel`
- `ur5_tools`

## Evidencias y trazabilidad

- Evidencias curadas del sistema ROS 2: `../reports/evidence/ros2/`
- Evidencias del bloque del TFM en resultados: `../reports/evidence/chapter5/`
- Logs de reproducibilidad: `../reports/logs/reproducibility/`

La fuente editable del workspace esta en `src/`, `scripts/`, `worlds/` y `models/`. Los artefactos de compilacion o de ejecucion no deben editarse a mano.
