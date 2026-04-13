# agarre_ros2_ws

Workspace ROS 2 del TFM para panel, simulacion y planificacion del UR5 con Gazebo y MoveIt 2.
Se conserva el nombre `agarre_ros2_ws` por compatibilidad con scripts y rutas absolutas ya validadas.

## Contenido

- `src/ur5_qt_panel/`: panel Qt principal.
- `src/ur5_bringup/`: lanzamiento del stack ROS 2 y simulacion.
- `src/ur5_moveit_config/`: configuracion de MoveIt 2.
- `src/ur5_description/`: robot, controladores y modelos asociados.
- `src/ur5_tools/`: bridge MoveIt, backend de attach y utilidades auxiliares.
- `src/tfm_grasping/`: nodos ROS 2 relacionados con grasping e inferencia.
- `src/ur5_panel_interfaces/`: interfaces ROS 2 del panel.
- `scripts/`: scripts operativos de arranque, parada, estado, validacion y diagnostico.
- `worlds/`: mundos SDF.
- `models/`: modelos de Gazebo.

## Arranque

Desde la raiz del proyecto:

```bash
./lanzar_panelv2.sh --fg
```

Desde este workspace:

```bash
./scripts/start_panel_v2.sh --fg
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

## Botones y flujo de uso

En la UI actual, los dos flujos de agarre visibles son:

- `Agarre Objeto (Directo)`: demo directa de agarre sobre mesa.
- `Agarre Objeto (MoveIT)`: flujo con percepcion, ROS 2 y MoveIt 2.

Secuencia practica recomendada:

1. `START ALL`
2. esperar `READY`
3. `Conectar camara`
4. `TEST ROBOT`
5. `Soltar objetos`
6. seleccionar experimento TFM
7. `Inferir agarre`
8. `Visualizar`
9. `Ejecutar agarre` o `Agarre Objeto (MoveIT)`

## Scripts operativos mas utiles

- `scripts/start_panel_v2.sh`
- `scripts/stop_panel_v2.sh`
- `scripts/status_panel_v2.sh`
- `scripts/recover_panel_v2.sh`
- `scripts/validate_panel_flow.sh`
- `scripts/validate_startup_repro.sh`
- `scripts/validate_pick_3_cycles.sh`
- `scripts/evidence_startup_ready.sh`
- `scripts/diag_tf_tcp.sh`
- `scripts/diag_startup_health.sh`

## Evidencias y trazas

- Evidencias curadas para memoria: `../reports/evidence/ros2/`
- Logs reproducibles: `../reports/logs/reproducibility/`
