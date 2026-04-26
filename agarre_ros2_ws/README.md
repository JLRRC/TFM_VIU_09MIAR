# agarre_ros2_ws

Workspace ROS 2 del TFM para panel, simulacion y planificacion del UR5 con Gazebo y MoveIt 2.
Se conserva el nombre `agarre_ros2_ws` por compatibilidad con scripts y rutas absolutas ya validadas.

## Documentación de arquitectura

El documento [ARCHITECTURE.md](ARCHITECTURE.md) describe la arquitectura completa del sistema:
- Diagrama C4 nivel 1 (sistema → stack ROS 2 → panel Qt)
- Mapa de módulos de `ur5_qt_panel` y `ur5_tools` con capas de responsabilidad
- Flujos de datos de los pipelines DIRECTO y MoveIt
- Topología de nodos ROS 2 en runtime
- Geometría TCP semántico del gripper RG2
- Inventario de tests y guía de CI

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

Desde la raiz del proyecto, flujo canónico oficial:

```bash
./lanzar_panelc2.sh
```

(`./lanzar_panelv2.sh` se mantiene como alias de compatibilidad.)

Notas de ejecucion canonica:

- La repeticion manual final y cualquier demo de defensa deben arrancarse con `./lanzar_panelc2.sh` desde la raiz del proyecto.
- Ese launcher activa el overlay de `install/setup.bash` del workspace y es la referencia operativa valida para reproducibilidad.
- `./scripts/start_panel_v2.sh --fg` y variantes `--tfm-repro` o `--tfm-raw` se mantienen como wrappers internos de depuracion, validacion y QA; no sustituyen al launcher canonico para una repeticion final.

Desde este workspace, wrapper interno para depuración:

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

- El selector trabaja con `EXP1` a `EXP4` y tambien permite usar `EXP1.1` y `EXP1.2` para inferencia.
- En modo normal, el panel puede autoelegir checkpoints segun `val_success`.
- Para `EXP1.1` y `EXP1.2`, si `val_success` no discrimina entre seeds, el panel prioriza la seed con mejor `val_loss`.
- En modo `--tfm-repro`, fija el caso `EXP3_RESNET18_RGB_AUGMENT / seed_0`.
- Puede ejecutarse tambien en modo `raw` sin los ajustes heurísticos posteriores del panel.
- El boton `Caso TFM Memoria` deja activado de una vez `EXP3 seed_0 + raw` y aplica el experimento.
- Todos los experimentos disponibles pueden cargarse y usarse desde el panel.
- La UI y los logs indican ahora tanto la politica de seleccion del checkpoint como si la prediccion ha recibido ajustes posteriores de angulo, centro o tamano.

## CI rápido (sin ROS)

```bash
# AST parse + F401 + 179 tests unitarios — ~5 s, no requiere Gazebo ni ROS
bash scripts/smoke_test.sh --fast

# Checklist pre-demo completo (requiere stack arrancado)
bash scripts/validate_before_demo.sh
```

## Limpieza de emergencia

Cuando el daemon DDS queda colgado, hay procesos zombi del stack o la memoria compartida FastDDS está contaminada:

```bash
# Desde la raiz del proyecto:
./limpia_stack.sh

# Con PIDs concretos conocidos que no responden:
./limpia_stack.sh 103569 103652
```

O directamente desde este workspace:

```bash
./scripts/limpia_stack.sh
```

Garantías del script:
- Nunca llama a `ros2` sin `timeout` (raíz del cuelgue crónico entre reinicios).
- Mata primero cualquier `ros2 node list / topic list / ...` colgado con SIGKILL.
- Para el daemon con `timeout 5s ros2 daemon stop` y lo reinicia.
- Limpia `/dev/shm/fastrtps_*`, `fastdds_*`, `sem.*`, `cyclonedds*`.
- Idempotente: siempre devuelve `exit 0`.
- Verificación final no bloqueante: `ps aux` + `timeout 5s ros2 node list --no-daemon`.

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
- `scripts/export_tfm_evidence.py`
- `scripts/diag_tf_tcp.sh`
- `scripts/diag_startup_health.sh`
- `scripts/audit_moveit2_system.py`

Validacion corta del caso TFM documentado:

```bash
python3 ./scripts/tfm_smoketest.py --check-session --require-repro --require-raw
```

Exportar la evidencia mas reciente del panel TFM:

```bash
python3 ./scripts/export_tfm_evidence.py
```

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

Artefactos operativos utiles del panel:

- `../reports/evidence/ros2/panel_audit/artifacts/checkpoints_index.json`
- `../reports/evidence/ros2/panel_audit/artifacts/grasp_last.json`
- `../reports/evidence/ros2/panel_audit/figures/overlay_last.png`
- `../reports/evidence/ros2/tfm_session_exports/`: exportaciones curadas de sesiones TFM listas para compartir o revisar.

Notas de uso de estos artefactos:

- `checkpoints_index.json`, `grasp_last.json` y `overlay_last.png` son snapshots del ultimo uso exportado del bloque TFM y deben leerse junto a su timestamp.
- Si esos artefactos no estan frescos, la referencia operativa mas reciente del runtime no es este bloque curado sino `../auditoria/`.
- `tfm_session_exports/` puede estar vacio si no se ha ejecutado una exportacion reciente con `scripts/export_tfm_evidence.py`.

La fuente editable del workspace esta en `src/`, `scripts/`, `worlds/` y `models/`. Los artefactos de compilacion o de ejecucion no deben editarse a mano.
