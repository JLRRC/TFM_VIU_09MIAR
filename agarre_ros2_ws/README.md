# agarre_ros2_ws

Workspace ROS 2 oficial del TFM para simulacion, panel e integracion del pipeline de agarre.

Que contiene esta parte del proyecto:

- `src/tfm_grasping/`: nodo ROS 2 que integra inferencia, mensajes y adaptacion del detector de agarre.
- `src/ur5_qt_panel/`: panel Qt principal usado en la demostracion del TFM.
- `src/ur5_bringup/`: bringup del entorno UR5.
- `src/ur5_moveit_config/`: configuracion oficial de MoveIt 2.
- `src/ur5_description/`: descripcion del robot, URDF/Xacro y controladores asociados.
- `src/ur5_tools/`: utilidades de operacion y soporte.
- `src/ur5_panel_interfaces/`: interfaces ROS 2 del panel.
- `scripts/`: lanzadores, validadores, diagnosticos y utilidades de operacion del workspace.
- `worlds/`: escenas SDF del entorno simulado.
- `models/`: modelos usados en Gazebo.
- `build/` e `install/`: build actual del workspace ROS 2, conservado para no perder arranque inmediato.

Flujo de arranque recomendado:

- Desde la raiz: `./lanzar_panelv2.sh`
- Dentro del workspace: `./scripts/start_panel_v2.sh`
- Parada limpia: `./scripts/stop_panel_v2.sh`

Scripts operativos a priorizar:

- `scripts/start_panel_v2.sh`
- `scripts/stop_panel_v2.sh`
- `scripts/status_panel_v2.sh`
- `scripts/validate_startup_repro.sh`
- `scripts/validate_panel_flow.sh`
- `scripts/evidence_startup_ready.sh`
- `scripts/bench_infer_log.py`

Dependencias criticas:

- ROS 2 Jazzy
- MoveIt 2
- Gazebo / ros_gz
- Qt5 con plugins del sistema

Fuente oficial de evidencias para el TFM:

- `../report/evidence/ros2/`
- `../report/logs/evaluation/`
- `../report/logs/reproducibility/ros2_repro_startup/`

Notas de mantenimiento:

- Los logs operativos antiguos y reportes temporales ya se han movido a `../BORRAR/agarre_ros2_ws_extra/`.
- El arbol `build/` e `install/` actual se mantiene porque sigue siendo util para lanzar el stack sin recompilar desde cero.
- La fuente oficial para capturas, evidencias funcionales y trazabilidad documental no esta en este workspace, sino en `../report/`.
