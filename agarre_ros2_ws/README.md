<!-- Ruta/archivo: agarre_ros2_ws/README.md -->
<!-- Contenido: Resumen operativo del workspace ROS 2 del TFM. -->
<!-- Uso breve: Se consulta para build, bringup y diagnostico del stack ROS 2. -->

<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/README.md -->
<!-- Summary: ROS 2 Jazzy workspace overview and build/run notes. -->
# agarre_ros2_ws

Workspace ROS 2 (Jazzy) para el trabajo de agarre inteligente.

## Build
```bash
cd ~/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Lanzador recomendado del panel

Desde la raiz del workspace:

```bash
~/TFM/lanzar_panelv2.sh
```

El wrapper detecta automaticamente el entorno virtual de vision en este orden:

- `~/TFM/agarre_inteligente/.venv-tfm`
- `~/TFM/agarre_inteligente/venv`
- `~/TFM/agarre_inteligente/.venv`

Para pruebas sin display grafico:

```bash
export PANEL_FORCE_OFFSCREEN=1
~/TFM/lanzar_panelv2.sh
```

## Dependencias ROS 2 (TF2 Python)

Para evitar errores `TypeException` en transformaciones TF2 (PoseStamped/PointStamped), instala el paquete:

```bash
sudo apt install ros-jazzy-tf2-geometry-msgs
```

## Bringup oficial (único)
```bash
ros2 launch ur5_bringup ur5_stack.launch.py
```

## Validación obligatoria post-bringup (profesional)
```bash
bash scripts/validate_panel_flow.sh
```

Variables útiles:
- `PANEL_COLD_BOOT=1` (limpia procesos previos antes de arrancar si usas el wrapper)
- `PANEL_GZ_GUI=1` (lanza Gazebo con GUI desde el wrapper)
- `PANEL_MANAGED=1` (panel guiado por `/system_state`, no lanza procesos críticos)
- `PANEL_MOVEIT_REQUIRED=1` (bloquea READY si MoveIt no está listo)
- `PANEL_USE_SIM_TIME=1` (fuente de verdad para sim time en panel/TF helper)
- `PANEL_START_STACK=0` (modo panel-only desde el wrapper)
- `PANEL_CONTROLLER_MANAGER=/ns/controller_manager` (override del namespace de controller_manager)

### Tuning TEST ROBOT y MoveIt bridge (sin recompilar)
```bash
export PANEL_TEST_TUNING_FILE="$WS_DIR/src/ur5_qt_panel/config/panel_test_tuning.yaml"
```

Parámetros de ajuste rápido (override por entorno):
- `PANEL_TEST_TABLE_CHECK_ENABLED`
- `PANEL_TEST_TABLE_CHECK_SKIP_HOME`
- `PANEL_TEST_CORNER_MAX_ATTEMPTS`
- `PANEL_TEST_CORNER_ADAPT_GAIN`
- `PANEL_TEST_CORNER_ADAPT_MAX_XY_M`
- `PANEL_TEST_CORNER_ADAPT_MAX_Z_M`
- `PANEL_TEST_CORNER_TOL_XY_M`
- `PANEL_TEST_CORNER_TOL_Z_M`
- `PANEL_TEST_CORNER_LIVE_TRACE`
- `PANEL_TEST_FRAME_SAFE_PROBE_MARGIN_M`

Parámetros del bridge MoveIt (aplican al pulsar `Lanzar MoveIt bridge`):
- `PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC`
- `PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC`
- `PANEL_MOVEIT_BRIDGE_MIN_PLAN_INTERVAL_SEC`
- `PANEL_MOVEIT_BRIDGE_STALE_REQUEST_TTL_SEC`
- `PANEL_MOVEIT_BRIDGE_HEARTBEAT_RATE_HZ`
- `PANEL_MOVEIT_BRIDGE_REQUIRE_REQUEST_ID`
- `PANEL_MOVEIT_BRIDGE_DROP_PENDING_ON_TAGGED`

Argumentos útiles del launch:
- `headless:=true|false`
- `launch_panel:=true|false`
- `launch_gazebo:=true|false`
- `launch_rsp:=true|false`
- `launch_bridge:=true|false`
- `launch_ros2_control:=true|false`
- `control_backend:=gz_ros2_control|ros2_control_node`
- `launch_moveit:=true|false`
- `moveit_start_ros2_control:=true|false`
- `controller_manager:=/ns/controller_manager`

## Arquitectura de control (Gazebo vs ROS puro)
- Gazebo: usa `control_backend:=gz_ros2_control` (default). No lanzar `ros2_control_node`.
- ROS puro (sin Gazebo): usa `control_backend:=ros2_control_node` y `launch_gazebo:=false`.
- Con Gazebo activo, `moveit_start_ros2_control` se fuerza a `false` para evitar doble controller_manager.

## Verificaciones objetivas (sin UI)
Chequeo de fisica y caida de objetos:
```bash
python3 tools/drop_check.py --ws ~/TFM/agarre_ros2_ws
```

Chequeo del servicio /release_objects:
```bash
bash tools/verify_release_objects.sh
```

## Auditoria end-to-end (sin UI)
```bash
bash tools/audit_end_to_end.sh
```

## Check no-attach (pre TEST ROBOT)
```bash
python3 tools/check_no_attach.py
```

## Rendimiento del panel (ultra-rapido)
Objetivo: minimizar CPU/latencia del panel sin romper funciones criticas.

Preset recomendado:
```bash
PANEL_CAMERA_COPY_FRAME=0 \
PANEL_CAMERA_MAX_SIZE=720 \
PANEL_CAMERA_DISPLAY_MS=16 \
PANEL_CAMERA_FAST_SCALE=1 \
PANEL_CAMERA_UI_SKIP_HIDDEN=1 \
PANEL_CAMERA_INFO_INTERVAL_SEC=0.5 \
PANEL_CAMERA_TRACK_FPS=0 \
PANEL_OVERLAY_CALIB=0 \
PANEL_OVERLAY_REACH=0 \
PANEL_OVERLAY_SELECTION=0 \
PANEL_OVERLAY_ANTIALIAS=0 \
PANEL_DEPTH_FAST=1 \
PANEL_CAMERA_SKIP_TFM_INPUT=1 \
PANEL_CONTROLLER_READY_CACHE_SEC=0.5 \
PANEL_STATUS_TOPIC_CACHE_SEC=0.5 \
PANEL_GZ_SERVICE_CHECK_SEC=2.5 \
PANEL_POSE_INFO_ALLOW_STALE=1 \
PANEL_POSE_CLI_ENABLED=0 \
PANEL_MAX_FPS=90
```

Notas:
- `PANEL_CAMERA_COPY_FRAME=0` usa buffers directos; si ves glitches, vuelve a `1`.
- `PANEL_CAMERA_SKIP_TFM_INPUT=1` desactiva alimentacion del modelo en vivo.
- `PANEL_POSE_CLI_ENABLED=0` evita el fallback `gz topic` (menos CPU).

Variables nuevas (detalle):
- `PANEL_CAMERA_MAX_SIZE` limite del lado mayor en pixeles (0 = sin limite).
- `PANEL_CAMERA_COPY_FRAME` copia o no los frames en QImage.
- `PANEL_CAMERA_DISPLAY_MS` intervalo de refresco UI.
- `PANEL_CAMERA_FAST_SCALE` usa escalado rapido en UI.
- `PANEL_CAMERA_UI_SKIP_HIDDEN` no repinta si ventana/viewport no es visible.
- `PANEL_CAMERA_INFO_INTERVAL_SEC` limita actualizacion de label de camara.
- `PANEL_CAMERA_TRACK_FPS` habilita calculo de FPS interno.
- `PANEL_OVERLAY_CALIB`, `PANEL_OVERLAY_REACH`, `PANEL_OVERLAY_SELECTION` overlays.
- `PANEL_OVERLAY_ANTIALIAS` antialiasing en overlays.
- `PANEL_DEPTH_FAST` usa min/max en vez de percentiles.
- `PANEL_CAMERA_SKIP_TFM_INPUT` no envia frames a TFM.
- `PANEL_CONTROLLER_READY_CACHE_SEC` cache corto de controladores.
- `PANEL_STATUS_TOPIC_CACHE_SEC` cache de estado de topics.
- `PANEL_GZ_SERVICE_CHECK_SEC` intervalo de chequeo `gz service -l`.
- `PANEL_POSE_INFO_ALLOW_STALE` acepta pose/info viejo.
- `PANEL_POSE_CLI_ENABLED` habilita/deshabilita fallback `gz topic`.
- `PANEL_POSE_CLI_MIN_INTERVAL_SEC` minimo entre llamadas al CLI.
- `PANEL_MAX_FPS` limita emision de frames.

## Medicion de rendimiento
Script de medicion (CPU/RSS y opcional Hz de camara):
```bash
python3 scripts/panel_perf_measure.py --match panel_v2 --duration 12 --interval 0.5
python3 scripts/panel_perf_measure.py --match panel_v2 --camera-topic /camera_overhead/image --duration 12
```

Comparacion before/after:
```bash
python3 scripts/panel_perf_compare.py \
  --match panel_v2 \
  --duration 12 \
  --interval 0.5 \
  --camera-topic /camera_overhead/image \
  --before-env "export PANEL_CAMERA_MAX_SIZE=960" \
  --after-env "export PANEL_CAMERA_MAX_SIZE=720 PANEL_CAMERA_COPY_FRAME=0"
```

Para detalle completo: `docs/PERFORMANCE_PANEL.md`

## Debug manual (solo si hace falta)
- El panel ya no genera YAML runtime del bridge (usa `scripts/bridge_cameras.yaml`).
- `/clock` y `/world/<world>/pose/info` los aporta el launch oficial.
- `/system_state` lo publica `ur5_tools/system_state_manager`.

## Objetos y mesa (fuente única)
- `scripts/object_positions.json` es la fuente de verdad para posiciones iniciales.
- Para sincronizar el SDF con el JSON:
  ```bash
  python3 scripts/sync_object_positions_to_sdf.py \
    --sdf worlds/ur5_mesa_objetos.sdf \
    --json scripts/object_positions.json
  ```

## MoveIt (bringup independiente)
```bash
ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py start_ros2_control:=false launch_rviz:=false
```
