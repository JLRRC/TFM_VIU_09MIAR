# Log de localizacion del workspace ROS 2 + Gazebo

## Objetivo

Identificar el workspace ROS 2 + Gazebo realmente valido para el TFM y registrar las evidencias que justifican esa decision.

## Restricciones

- No usar contenido dentro de carpetas `BORRAR`.
- No asumir que todo lo encontrado es valido por defecto.
- Diferenciar material activo de copias historicas.

## Workspaces candidatos localizados

- `/home/laboratorio/TFM/agarre_ros2_ws`
- `/home/laboratorio/TFM/agarre_ros2_ws_hisorico`

## Documento actual del TFM localizado

- `/home/laboratorio/TFM/agarre_inteligente/docs/TFM_V7.9.pdf`

## Rutas revisadas y utilidad

- `/home/laboratorio/TFM/agarre_ros2_ws/README.md`
  - Resumen operativo del workspace ROS 2 Jazzy.
  - Declara `ros2 launch ur5_bringup ur5_stack.launch.py` como bringup oficial.
- `/home/laboratorio/TFM/agarre_ros2_ws/src`
  - Paquetes ROS 2 del stack activo.
  - Paquetes encontrados: `ur5_bringup`, `ur5_qt_panel`, `ur5_moveit_config`, `tfm_grasping`, `ur5_tools`, `ur5_description`.
- `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py`
  - Launch principal del sistema.
  - Arranca `gz sim`, `ros_gz_bridge`, `gz_pose_bridge`, `world_tf_publisher`, `system_state_manager` y el panel ROS 2.
- `/home/laboratorio/TFM/agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf`
  - Mundo de simulacion usado para la escena principal del TFM.
  - Incluye mesa, objeto `pick_demo`, camaras y joints desmontables para objetos.
- `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro`
  - Modelo UR5 + RG2 con `gz_ros2_control`.
- `/home/laboratorio/TFM/agarre_ros2_ws/scripts/bridge_cameras.yaml`
  - Configuracion del `ros_gz_bridge` para `/clock`, pose world y camaras RGB/depth.
- `/home/laboratorio/TFM/agarre_ros2_ws/scripts/object_positions.json`
  - Posiciones de referencia de objetos del mundo.
- `/home/laboratorio/TFM/agarre_ros2_ws/log`
  - Evidencia primaria del workspace activo: capturas, overlays y logs de arranque.
  - Archivos especialmente relevantes: `pick_demo_run_final.log`, `camera_overhead_capture.png`, `exp3_vs_ref.png`.
- `/home/laboratorio/TFM/reports/panel_audit`
  - Evidencia externa pero directamente conectada al panel del mismo workspace.
  - Contiene logs de `apply`, `infer`, `visualize`, `execute`, ademas de `grasp_last.json` y `overlay_last.png`.
- `/home/laboratorio/TFM/agarre_ros2_ws_hisorico`
  - Copia historica del workspace.
  - No aporta evidencia mas fuerte que la hallada en el workspace principal.

## Launch files relevantes localizados

- `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py`
- `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_ros2_control.launch.py`
- `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_rsp.launch.py`
- `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py`

## Evidencias usadas para decidir el workspace valido

- `agarre_ros2_ws` tiene fecha de modificacion mas reciente que `agarre_ros2_ws_hisorico`.
- `agarre_ros2_ws/log/pick_demo_run_final.log` contiene arranque completo del stack con procesos, topics y transiciones de estado.
- `agarre_ros2_ws/log/` contiene capturas y material visual generado durante ejecuciones reales.
- `reports/panel_audit/logs/panel_block_smoke_test.log` fija el `cwd` en `/home/laboratorio/TFM/agarre_ros2_ws`.
- `reports/panel_audit/logs/infer.log` y `execute.log` documentan ejecuciones reales del pipeline desde el panel.
- El README del workspace principal habla de `Bringup oficial (unico)`, mientras que el workspace historico no presenta mejores logs ni mejor trazabilidad.

## Decision adoptada

- Workspace ROS 2 + Gazebo valido para el dossier:
  - `/home/laboratorio/TFM/agarre_ros2_ws`
- Workspace secundario tratado solo como respaldo historico:
  - `/home/laboratorio/TFM/agarre_ros2_ws_hisorico`

## Dudas o ambiguedades registradas

- El workspace historico comparte parte de la estructura y del README, por lo que no puede descartarse como antecedente tecnico.
- Aun asi, para resultados del TFM se prioriza el workspace principal porque es el unico enlazado con logs recientes y evidencia directa de ejecucion.
- La version exacta usada en cada corrida no queda reflejada en una rama Git concreta dentro de los logs revisados.
- Por tanto, el criterio de validez adoptado es operativo y documental, no de control de versiones.

## Registro de acciones

- 2026-03-15: localizada la pareja de workspaces `agarre_ros2_ws` y `agarre_ros2_ws_hisorico`.
- 2026-03-15: localizado el PDF actual `TFM_V7.9.pdf`.
- 2026-03-15: inspeccionados paquetes, launch files, mundo SDF, bridge de camaras y logs activos del workspace principal.
- 2026-03-15: fijado `agarre_ros2_ws` como workspace valido para el dossier ROS 2 + Gazebo.
