# agarre_ros2_ws

Workspace ROS 2 oficial del TFM para simulacion, panel e integracion del pipeline de agarre.

Contenido oficial conservado:

- `src/tfm_grasping/`: nodo de inferencia y adaptacion ROS 2 del modelo de agarre.
- `src/ur5_qt_panel/`: panel Qt oficial.
- `src/ur5_bringup/`: bringup del stack UR5.
- `src/ur5_moveit_config/`: configuracion MoveIt 2 oficial.
- `src/ur5_description/`: URDF/Xacro y controladores de la descripcion del robot.
- `src/ur5_tools/`: utilidades operativas necesarias del stack.
- `src/ur5_panel_interfaces/`: interfaces del panel.
- `scripts/`: lanzadores y validadores operativos oficiales.
- `worlds/`, `models/`: escenario y modelos SDF usados en la simulacion oficial.

Flujo de arranque recomendado:

- Desde la raiz: `./lanzar_panelv2.sh`
- Dentro del workspace: `./scripts/start_panel_v2.sh`
- Parada limpia: `./scripts/stop_panel_v2.sh`

Dependencias criticas:

- ROS 2 Jazzy
- MoveIt 2
- Gazebo / ros_gz
- Qt5 con plugins del sistema

Evidencias y logs oficiales para el TFM:

- `../report/evidence/ros2/`
- `../report/logs/evaluation/`
- `../report/logs/reproducibility/ros2_repro_startup/`

Todo lo no imprescindible para el flujo oficial, incluyendo auditorias antiguas, `build/install/log`, tests operativos aislados y reportes heredados, se ha movido a `../BORRAR/agarre_ros2_ws_extra/`.
