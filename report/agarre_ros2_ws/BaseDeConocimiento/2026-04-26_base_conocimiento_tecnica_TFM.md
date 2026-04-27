---
title: "Base de Conocimiento Técnica — Sistema UR5 + RG2 Pick & Place"
subtitle: "Generado: 2026-04-26 22:52:40 | Workspace: /home/laboratorio/TFM/agarre_ros2_ws"
author: "generar_base_conocimiento_tfm.sh (Claude Code)"
date: "2026-04-26"
---

> **MODO ACUMULATIVO** — Esta base refleja el estado real del workspace en la fecha indicada.
> Los valores anteriores se conservan con indicación de cambio.
> Fuente de verdad: código fuente actual > logs recientes > configuración cargada.
> Si un dato no pudo confirmarse en esta ejecución se indica como **[NO CONFIRMADO]**.


# 1. Resumen Ejecutivo

| Campo | Valor |
|-------|-------|
| Fecha de generación | 2026-04-26 22:52:40 |
| Script | `scripts/generar_base_conocimiento_tfm.sh` |
| Workspace | `/home/laboratorio/TFM/agarre_ros2_ws` |
| Robot | UR5 con gripper OnRobot RG2 (modelo: ur5_rg2) |
| Stack | ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2 + ros2_control + ros_gz |
| Rama git actual | `ENTREGA.V3` |
| Último commit | `06afabe fix(gripper+validate): proportional hysteresis for gripper state + correct READY sentinel` |
| Base anterior | `historico/2026-04-18_base_conocimiento_tecnica.md` |
| Motivo actualización | Auditoría + gran refactorización post-23-04-2026 |

## 1.1 Estado Global

**Cambios principales desde base anterior (23-04-2026):**

- panel_v2.py refactorizado: 18 600 → 2 859 líneas (−85%). 15 módulos extraídos.
- RG2 gripper migrado de revoluto a prismático: rango 0–0.055 m por dedo.
- TCP semántico unificado: rg2_pinch_center = 0.0050885 m desde tool0.
- SDF corregido: ur5_hand_joint rpy=0 (antes pitch=−π/2 causaba 247 mm de error).
- 593+ tests unitarios sin ROS (todos puros).
- CachedKDL IK solver añadido (min_pose_distance=0.005).
- Módulos de análisis extraídos: directo_geometry, directo_gate_evaluator, moveit_bridge_utils, launch_helpers.
- Script limpieza limpiar_stack.sh / limpia_stack.sh añadidos.
- **ALERTA CRÍTICA:** 10 archivos en conflicto de merge (UU) sin resolver.

## 1.2 Funciona / No Funciona / Pendiente

| Estado | Área |
|--------|------|
| ✅ Funciona | Build colcon (7 paquetes) |
| ✅ Funciona | 593+ tests unitarios puros (sin ROS) |
| ✅ Funciona | Geometría TCP unificada (0.0050885 m) |
| ✅ Funciona | Gripper prismático 0–0.055 m |
| ✅ Funciona | SDF/URDF coherentes (DH/SDF divergencia resuelta) |
| ✅ Funciona | DIRECTO pick completo (ATTACH→LIFT→CARRY→CESTA) |
| ⚠️ Pendiente | Resolver 10 conflictos de merge (UU) |
| ⚠️ Pendiente | Validar runtime con stack completo post-refactor |
| ❌ Activo | kinematics.yaml en conflicto: CachedKDL vs plain KDL |
| ❌ Activo | joint_limits.yaml en conflicto |
| ❌ Activo | ur5_stack.launch.py en conflicto |

# 2. Estado Git y Cambios Recientes


## 2.1 Log (últimos 30 commits)

**$ git log --oneline -30

```
06afabe fix(gripper+validate): proportional hysteresis for gripper state + correct READY sentinel
503aa4a feat: add limpieza script for ROS 2 stack management
aab67f8 fix(panel): restore imports across 23 modules after modular refactoring
83950db fix(panel): añadir imports faltantes de panel_config en 14 módulos
ddb7961 fix(panel): añadir import faltante timestamped_line en panel_helpers.py
557d1ad fix(launch): corregir import relativo launch_helpers + smoke_test con ROS sourced
37807d5 feat(tests+docs): moveit_bridge_utils 100% cobertura + ARCHITECTURE.md limpio
9a45570 feat(tests): cobertura 100% en todos los módulos puros — 593 tests sin ROS
cc43e72 feat(tests): extend panel_utils and gripper_geometry coverage to 444 total
fb903c1 fix(lint): remove 3 unused imports from test files + extend F401 to test dirs
9a013b7 feat(tests): add test_ur5_kinematics (20 tests) — FK/IK DH analytic solver
e7c3d0b feat(tests): add test_param_utils (27 tests) — duck-typed, no ROS node needed
a1d2487 feat(tests): add test_attach_gate_evaluator (20 tests) + smoke_test 388 total
3e512ff fix(tests): repair two stale tests + expand smoke_test to 366 tests
a4d14bb feat(utils): add test_panel_config, TF freshness helpers, update arch docs
698ec0f feat+refactor(fase7+fase8): CycleLogger JSON, CachedKDL IK, bridge audit
b2be04a docs+test: README → ARCHITECTURE.md link, 27 launch_helpers tests, smoke_test 206 total
55d4327 docs(fase10): add ARCHITECTURE.md — C4 overview, module maps, data flows, testing guide
f2127d8 refactor(fase5b): extract launch_helpers.py — ur5_stack.launch.py 1176 → 863 lines (−27%)
fecd42f refactor(fase5a): extract moveit_bridge_utils.py — 16 pure static helpers from UR5MoveItBridge
66c3ce3 feat(fase7): add smoke_test.sh and validate_before_demo.sh
dd995f2 test(fase4): 129 unit tests for directo_geometry + directo_gate_evaluator
8d72785 refactor(fase3b): extract directo_gate_evaluator.py — 11 pure gate/joint functions
a3018a0 refactor(fase3a): extract directo_geometry.py — 20 pure functions from panel_pick_demo.py
256fe70 refactor(fase2): replace hardcoded 0.850 table height with TABLE_TOP_Z constant
f44b0a0 refactor(fase1): remove 57 unused imports across 19 files — zero F401 violations
a7e254c refactor(panel): consolidate fmt_vec3/fmt_pose_dict/fmt_age_sec into panel_utils
7ae5d37 chore(panel): remove _robot_test_active dead attribute
861e5ef refactor(scripts): improve process cleanup in limpiar_stack.sh and start_panel_v2.sh
e7998a8 refactor(panel): extract 15 modules from panel_v2.py — 18600 → 2859 lines (−85%)
```


## 2.2 Status actual (¡MERGE CONFLICTS!)

**$ git status --short

```
 M ../.gitignore
?? report/BaseDeConocimiento/
?? report/orden_limpieza/
?? scripts/generar_base_conocimiento_tfm.sh
```


## 2.3 Archivos modificados (diff stat)


```
 .gitignore | 2 ++
 1 file changed, 2 insertions(+)
```


## 2.4 Archivos en conflicto de merge (UU)


```
Ninguno
```

> **ACCIÓN REQUERIDA:** Los archivos marcados UU tienen conflictos sin resolver.
> El stack NO es buildable en estado de merge sin resolver estos conflictos.

# 3. Estructura del Workspace


## 3.1 Árbol src (3 niveles)

**$ find src -maxdepth 3 -type f -name *.py -o -name *.xml -o -name *.yaml -o -name *.launch.py -o -name *.srdf

```
src/ur5_bringup/config/ur5_mock_controllers.yaml
src/ur5_bringup/config/system_state_manager.yaml
src/ur5_bringup/launch/ur5_ros2_control.launch.py
src/ur5_bringup/launch/ur5_stack.launch.py
src/ur5_bringup/launch/ur5_rsp.launch.py
src/ur5_bringup/package.xml
src/ur5_qt_panel/config/panel_test_tuning.yaml
src/ur5_qt_panel/config/panel_settings.yaml
src/ur5_qt_panel/package.xml
src/ur5_moveit_config/config/moveit_controllers.yaml
src/ur5_moveit_config/config/ompl_planning.yaml
src/ur5_moveit_config/config/joint_limits.yaml
src/ur5_moveit_config/config/planning_scene_monitor_parameters.yaml
src/ur5_moveit_config/config/ur5_strict.srdf
src/ur5_moveit_config/config/kinematics.yaml
src/ur5_moveit_config/config/ur5.srdf
src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py
src/ur5_moveit_config/package.xml
src/tfm_grasping/package.xml
src/ur5_tools/package.xml
src/ur5_panel_interfaces/package.xml
src/ur5_description/config/ur5_controllers.yaml
src/ur5_description/package.xml
```


## 3.2 Scripts

**$ find scripts -maxdepth 1 -type f

```
scripts/show_latest_startup_evidence.sh
scripts/diag_startup_health.sh
scripts/verify_object_state_invariants.py
scripts/ur5_home_pose.env
scripts/status_panel_v2.sh
scripts/sync_object_positions_to_sdf.py
scripts/audit_moveit2_system.py
scripts/repro_release_objects_idempotent.sh
scripts/panel_block_smoke_test.sh
scripts/run5_launcher.sh
scripts/capture_camera_frames.py
scripts/start_panel_v2.sh
scripts/fase_b_validate.py
scripts/table_pixel_map.json
scripts/ros2_bringup_checklist.sh
scripts/prune_startup_evidence.sh
scripts/run_directo2_button_offscreen.py
scripts/panel_perf_measure.py
scripts/panel_perf_compare.py
scripts/run_moveit_lift_validation.sh
scripts/export_tfm_evidence.py
scripts/01_preflight_check.sh
scripts/bridge_cameras.yaml
scripts/diag_gazebo_camera_ctrl.sh
scripts/grasp_audit_benchmark.py
scripts/ci_local.sh
scripts/evidence_startup_ready.sh
scripts/run_directo_visual_smoke.sh
scripts/panel_runtime_validated.env
scripts/start_panel_v2_venv.sh
scripts/validate_before_demo.sh
scripts/stop_panel_v2.sh
scripts/directo_visual_capture.py
scripts/generar_base_conocimiento_tfm.sh
scripts/limpia_stack.sh
scripts/validate_touch_dual.sh
scripts/freeze_touch_baseline.sh
scripts/grasp_audit_trace_capture.py
scripts/run_directo_button_offscreen.py
scripts/run_directo_batch_validation.sh
scripts/smoke_test.sh
scripts/limpiar_stack.sh
scripts/kill_all.sh
scripts/01_install_ros2_jazzy.sh
scripts/validate_startup_repro.sh
scripts/recover_panel_v2.sh
scripts/monitor_attach_publishers.sh
scripts/validate_pick_3_cycles.sh
scripts/gen_drop_grid.py
scripts/run_touch_tuner_front_left.sh
scripts/run_directo2_validation.sh
scripts/monitor_tip_table_clearance.py
scripts/bench_infer_log.py
scripts/validate_panel_flow.sh
scripts/tfm_smoketest.py
scripts/01_validate_installation.sh
scripts/run_panel_objectstate_check.sh
scripts/fastdds_no_shm.xml
scripts/capture_system_diag.py
scripts/run_daily_stack.sh
scripts/run_touch_tuner_front_right.sh
scripts/diag_tf_tcp.sh
scripts/object_positions.json
scripts/run_directo_validation.sh
scripts/01_run_all_preflight.sh
scripts/summarize_directo_batch.py
scripts/check_qt_env.sh
scripts/start_panel_with_xvfb.sh
```


## 3.3 Modelos

**$ find models -maxdepth 4 -type f

```
models/ur5_rg2_BACKUP_CLEAN_TCP_20260425_210914/thumbnails/1.png
models/ur5_rg2_BACKUP_CLEAN_TCP_20260425_210914/thumbnails/2.png
models/ur5_rg2/ur5_controllers.yaml
models/ur5_rg2/model.config
models/ur5_rg2/model.sdf
```


## 3.4 Worlds

**$ find worlds -maxdepth 2 -type f

```
worlds/ur5_mesa_objetos.sdf
worlds/ur5_debug_empty.sdf
```


## 3.5 Reports y evidencias

**$ find report -maxdepth 4 -type f

```
report/orden_limpieza/20260426_225050/inventario_auditoria.txt
report/orden_limpieza/20260426_225050/archivos_vacios.txt
report/orden_limpieza/20260426_225050/inventario_historico.txt
report/orden_limpieza/20260426_225050/tamanos_auditoria.txt
report/orden_limpieza/20260426_225050/tamanos_historico.txt
report/incidents/2026-04-22_rg2_tcp_incident_closure.md
report/incidents/2026-04-23_directo_basket_transport_ik_followup.md
report/BaseDeConocimiento/2026-04-26_base_conocimiento_tecnica_TFM.md
report/BaseDeConocimiento/.tmp_base_2026-04-26/generacion.log
report/BaseDeConocimiento/.tmp_base_2026-04-26/fuentes_verificadas.txt
report/repro_startup/20260423_161929/summary.log
report/repro_startup/20260423_161929/cycle1_system_diag.log
report/repro_startup/20260423_161929/cycle1_stop.log
report/repro_startup/20260423_161929/cycle1_post_stop.log
report/repro_startup/20260423_161929/cycle1_start.log
report/repro_startup/20260423_161929/cycle1_pipeline.log
report/repro_startup/20260423_161929/cycle1_system_diag.json
report/auditoria_moveit_step_20260424.md
report/runtime_moveit_step_20260424.md
report/evidence/auto_validation/20260420_161037/ros2_topic_list.txt
report/evidence/auto_validation/20260420_161037/analysis.md
report/evidence/auto_validation/20260420_161037/joint_states_once.txt
report/evidence/auto_validation/20260420_161037/summary.json
report/evidence/auto_validation/20260420_161037/tf_rg2_pinch_center.txt
report/evidence/auto_validation/20260420_161037/pick_run.log
report/evidence/auto_validation/20260420_161037/env.txt
report/evidence/auto_validation/20260420_161037/launch.log
report/evidence/auto_validation/20260420_161037/ros2_node_list.txt
```


## 3.6 Histórico

**$ find historico -maxdepth 3 -type f

```
historico/2026-04-18_base_conocimiento_tecnica.md
historico/2026-04-18_base_conocimiento_tecnica.pdf
historico/2026-04-18_base_conocimiento_tecnica.html
historico/step_cartesian_debug/manual_moves.jsonl
historico/2026-04-17_auditoria_arquitectura_completa.md
```


# 4. Paquetes ROS 2


## 4.x tfm_grasping


### package.xml


```
<?xml version="1.0"?>
<!-- Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/package.xml -->
<!-- Contenido: Codigo de percepcion y agarre del paquete tfm_grasping. -->
<!-- Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre. -->

<package format="3">
  <name>tfm_grasping</name>
  <version>0.0.0</version>
  <description>TFM grasping module (perception, grasp representation, ROS publishing).</description>
  <maintainer email="jesus.lozano.rodriguez@gmail.com">laboratorio</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```


### setup.py (entry_points)


```
26:    entry_points={
27:        "console_scripts": [
28:            "grasp_inference=tfm_grasping.grasp_inference:main",
```


### Nodos Python


```
src/tfm_grasping/tfm_grasping/config.py
src/tfm_grasping/tfm_grasping/geometry.py
src/tfm_grasping/tfm_grasping/grasp_inference.py
src/tfm_grasping/tfm_grasping/grasp_module.py
src/tfm_grasping/tfm_grasping/__init__.py
src/tfm_grasping/tfm_grasping/model.py
src/tfm_grasping/tfm_grasping/perception.py
src/tfm_grasping/tfm_grasping/ros_interface.py
```


### Launch files


```
```


### Config YAML


```
```


### Tests


```
src/tfm_grasping/test/test_model_load_exp11.py
src/tfm_grasping/test/test_model_output_scaling.py
Total funciones test: 7
```


## 4.x ur5_bringup


### package.xml


```
<?xml version="1.0"?>
<!-- Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/package.xml -->
<!-- Contenido: Configuracion de bringup ROS 2 para lanzar el sistema UR5. -->
<!-- Uso breve: Colcon/ros2 launch lo usan para arrancar simulacion y componentes principales. -->

<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/package.xml -->
<!-- Summary: ROS 2 package manifest for ur5_bringup. -->
<package format="3">
  <name>ur5_bringup</name>
  <version>0.1.0</version>
  <description>Bringup launch files for UR5 simulation/control.</description>
  <maintainer email="laboratorio@example.com">laboratorio</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ur5_tools</exec_depend>
  <exec_depend>ur5_qt_panel</exec_depend>
  <exec_depend>ur5_moveit_config</exec_depend>
  <exec_depend>ur5_description</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```


### Nodos Python


```
src/ur5_bringup/launch/launch_helpers.py
src/ur5_bringup/launch/ur5_ros2_control.launch.py
src/ur5_bringup/launch/ur5_rsp.launch.py
src/ur5_bringup/launch/ur5_stack.launch.py
```


### Launch files


```
src/ur5_bringup/launch/ur5_ros2_control.launch.py
src/ur5_bringup/launch/ur5_rsp.launch.py
src/ur5_bringup/launch/ur5_stack.launch.py
```


### Config YAML


```
src/ur5_bringup/config/system_state_manager.yaml
src/ur5_bringup/config/ur5_mock_controllers.yaml
```


### Tests


```
src/ur5_bringup/test/test_launch_helpers.py
Total funciones test: 0
```


## 4.x ur5_description


### package.xml


```
<?xml version="1.0"?>
<!-- Ruta/archivo: agarre_ros2_ws/src/ur5_description/package.xml -->
<!-- Contenido: Descripcion URDF/Xacro y configuracion base del robot UR5. -->
<!-- Uso breve: Se usa al publicar el robot model y al preparar simulacion y MoveIt. -->

<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description/package.xml -->
<!-- Summary: ROS 2 package manifest for ur5_description. -->
<package format="3">
  <name>ur5_description</name>
  <version>0.1.0</version>
  <description>UR5 robot description (URDF/Xacro and controllers config).</description>
  <maintainer email="laboratorio@example.com">laboratorio</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ur_description</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```


### Nodos Python


```
```


### Launch files


```
```


### Config YAML


```
src/ur5_description/config/ur5_controllers.yaml
```


### Tests


```
Total funciones test: 0
```


## 4.x ur5_moveit_config


### package.xml


```
<?xml version="1.0"?>
<!-- Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/package.xml -->
<!-- Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion. -->
<!-- Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot. -->

<package format="3">
  <name>ur5_moveit_config</name>
  <version>0.1.0</version>
  <description>Minimal MoveIt 2 configuration for the UR5 robot.</description>
  <license>Apache-2.0</license>

  <maintainer email="jesus.lozano.rodriguez@gmail.com">laboratorio</maintainer>
  <author email="jesus.lozano.rodriguez@gmail.com">laboratorio</author>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rclcpp</build_depend>
  <build_depend>moveit_ros_move_group</build_depend>
  <build_depend>moveit_ros_planning</build_depend>
  <build_depend>moveit_ros_planning_interface</build_depend>
  <build_depend>moveit_ros_visualization</build_depend>
  <build_depend>moveit_msgs</build_depend>
  <build_depend>ur_description</build_depend>
  <build_depend>moveit_configs_utils</build_depend>
  <build_depend>ur5_description</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>moveit_ros_planning</exec_depend>
  <exec_depend>moveit_ros_planning_interface</exec_depend>
  <exec_depend>moveit_ros_visualization</exec_depend>
  <exec_depend>moveit_msgs</exec_depend>
  <exec_depend>ur_description</exec_depend>
  <exec_depend>moveit_configs_utils</exec_depend>
  <exec_depend>ur5_description</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```


### Nodos Python


```
src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py
```


### Launch files


```
src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py
```


### Config YAML


```
src/ur5_moveit_config/config/joint_limits.yaml
src/ur5_moveit_config/config/kinematics.yaml
src/ur5_moveit_config/config/moveit_controllers.yaml
src/ur5_moveit_config/config/ompl_planning.yaml
src/ur5_moveit_config/config/planning_scene_monitor_parameters.yaml
```


### Tests


```
Total funciones test: 0
```


## 4.x ur5_panel_interfaces


### package.xml


```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ur5_panel_interfaces</name>
  <version>0.1.0</version>
  <description>Interfaces ROS 2 para triggers fiables del panel UR5.</description>
  <maintainer email="jesus.lozano.rodriguez@gmail.com">laboratorio</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>```


### Nodos Python


```
```


### Launch files


```
```


### Config YAML


```
```


### Tests


```
Total funciones test: 0
```


## 4.x ur5_qt_panel


### package.xml


```
<?xml version="1.0"?>
<!-- Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/package.xml -->
<!-- Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5. -->
<!-- Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2. -->

<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/package.xml -->
<!-- Summary: ROS 2 package manifest for ur5_qt_panel. -->
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ur5_qt_panel</name>
  <version>0.0.0</version>
  <description>Qt panel for UR5 simulation control, cameras, and evidence capture.</description>
  <maintainer email="jesus.lozano.rodriguez@gmail.com">laboratorio</maintainer>
  <license>MIT</license>

  <depend>geometry_msgs</depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>rosgraph_msgs</depend>
  <depend>tf2_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>controller_manager_msgs</depend>
  <depend>control_msgs</depend>
  <depend>moveit_msgs</depend>
  <depend>tfm_grasping</depend>
  <depend>ur5_panel_interfaces</depend>
  <depend>ur5_tools</depend>

  <exec_depend>python3-pyqt5</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-scipy</exec_depend>
  <exec_depend>python3-psutil</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-yaml</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```


### setup.py (entry_points)


```
30:    entry_points={
31:        'console_scripts': [
32:            'panel_v2 = ur5_qt_panel.panel_v2:main',
33:            'main_panel = ur5_qt_panel.main_panel:main',
```


### Nodos Python


```
src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py
src/ur5_qt_panel/ur5_qt_panel/calibration_service.py
src/ur5_qt_panel/ur5_qt_panel/cameras_tab.py
src/ur5_qt_panel/ur5_qt_panel/directo_gate_evaluator.py
src/ur5_qt_panel/ur5_qt_panel/directo_geometry.py
src/ur5_qt_panel/ur5_qt_panel/__init__.py
src/ur5_qt_panel/ur5_qt_panel/logging_utils.py
src/ur5_qt_panel/ur5_qt_panel/main_panel.py
src/ur5_qt_panel/ur5_qt_panel/panel_calib_actions.py
src/ur5_qt_panel/ur5_qt_panel/panel_calibration.py
src/ur5_qt_panel/ur5_qt_panel/panel_calib_selection.py
src/ur5_qt_panel/ur5_qt_panel/panel_camera_controllers.py
src/ur5_qt_panel/ur5_qt_panel/panel_camera_helpers.py
src/ur5_qt_panel/ur5_qt_panel/panel_camera.py
src/ur5_qt_panel/ur5_qt_panel/panel_config.py
src/ur5_qt_panel/ur5_qt_panel/panel_controllers.py
src/ur5_qt_panel/ur5_qt_panel/panel_direct2.py
src/ur5_qt_panel/ur5_qt_panel/panel_draw_overlays.py
src/ur5_qt_panel/ur5_qt_panel/panel_env.py
src/ur5_qt_panel/ur5_qt_panel/panel_external_state.py
src/ur5_qt_panel/ur5_qt_panel/panel_fatal.py
src/ur5_qt_panel/ur5_qt_panel/panel_gz_objects.py
src/ur5_qt_panel/ur5_qt_panel/panel_gz_startup.py
src/ur5_qt_panel/ur5_qt_panel/panel_helpers.py
src/ur5_qt_panel/ur5_qt_panel/panel_launch_control.py
src/ur5_qt_panel/ur5_qt_panel/panel_launchers.py
src/ur5_qt_panel/ur5_qt_panel/panel_main_ui.py
src/ur5_qt_panel/ur5_qt_panel/panel_motion_control.py
src/ur5_qt_panel/ur5_qt_panel/panel_motion_helpers.py
src/ur5_qt_panel/ur5_qt_panel/panel_moveit_flow.py
src/ur5_qt_panel/ur5_qt_panel/panel_moveit_publishers.py
src/ur5_qt_panel/ur5_qt_panel/panel_moveit_ready.py
src/ur5_qt_panel/ur5_qt_panel/panel_moveit_wait.py
src/ur5_qt_panel/ur5_qt_panel/panel_object_mgmt.py
src/ur5_qt_panel/ur5_qt_panel/panel_objects.py
src/ur5_qt_panel/ur5_qt_panel/panel_physics.py
src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py
src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py
src/ur5_qt_panel/ur5_qt_panel/panel_process.py
src/ur5_qt_panel/ur5_qt_panel/panel_readiness.py
src/ur5_qt_panel/ur5_qt_panel/panel_remote_callbacks.py
src/ur5_qt_panel/ur5_qt_panel/panel_robot_presets.py
src/ur5_qt_panel/ur5_qt_panel/panel_ros_publishers.py
src/ur5_qt_panel/ur5_qt_panel/panel_ros.py
src/ur5_qt_panel/ur5_qt_panel/panel_runtime_pose_auditor.py
src/ur5_qt_panel/ur5_qt_panel/panel_safety.py
src/ur5_qt_panel/ur5_qt_panel/panel_settings.py
src/ur5_qt_panel/ur5_qt_panel/panel_shutdown.py
src/ur5_qt_panel/ur5_qt_panel/panel_startup.py
src/ur5_qt_panel/ur5_qt_panel/panel_state_machine.py
src/ur5_qt_panel/ur5_qt_panel/panel_state_methods.py
src/ur5_qt_panel/ur5_qt_panel/panel_state.py
src/ur5_qt_panel/ur5_qt_panel/panel_status_mgmt.py
src/ur5_qt_panel/ur5_qt_panel/panel_step_callbacks.py
src/ur5_qt_panel/ur5_qt_panel/panel_step_ui.py
src/ur5_qt_panel/ur5_qt_panel/panel_tf_diagnose.py
src/ur5_qt_panel/ur5_qt_panel/panel_tf_monitor.py
src/ur5_qt_panel/ur5_qt_panel/panel_tfm.py
src/ur5_qt_panel/ur5_qt_panel/panel_tfm_science.py
src/ur5_qt_panel/ur5_qt_panel/panel_tf.py
src/ur5_qt_panel/ur5_qt_panel/panel_trace_callbacks.py
src/ur5_qt_panel/ur5_qt_panel/panel_trace_ui.py
src/ur5_qt_panel/ur5_qt_panel/panel_ui_state.py
src/ur5_qt_panel/ur5_qt_panel/panel_utils.py
src/ur5_qt_panel/ur5_qt_panel/panel_v2.py
src/ur5_qt_panel/ur5_qt_panel/panel_watchdog.py
src/ur5_qt_panel/ur5_qt_panel/panel_workers.py
src/ur5_qt_panel/ur5_qt_panel/step_pipeline_helpers.py
src/ur5_qt_panel/ur5_qt_panel/tf_pose_utils.py
src/ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py
```


### Launch files


```
```


### Config YAML


```
src/ur5_qt_panel/config/panel_settings.yaml
src/ur5_qt_panel/config/panel_test_tuning.yaml
```


### Tests


```
src/ur5_qt_panel/test/test_attach_gate_evaluator.py
src/ur5_qt_panel/test/test_copyright.py
src/ur5_qt_panel/test/test_directo_gate_evaluator.py
src/ur5_qt_panel/test/test_directo_geometry.py
src/ur5_qt_panel/test/test_flake8.py
src/ur5_qt_panel/test/test_logging_utils.py
src/ur5_qt_panel/test/test_panel_config.py
src/ur5_qt_panel/test/test_panel_pick_demo_basket_confirmation.py
src/ur5_qt_panel/test/test_panel_pick_demo_direct_grasp_z.py
src/ur5_qt_panel/test/test_panel_pick_demo_live_pose.py
src/ur5_qt_panel/test/test_panel_pick_demo_transport_follow.py
src/ur5_qt_panel/test/test_panel_pick_object_moveit_init.py
src/ur5_qt_panel/test/test_panel_settings_defaults.py
src/ur5_qt_panel/test/test_panel_tfm_apply_gate.py
src/ur5_qt_panel/test/test_panel_utils.py
src/ur5_qt_panel/test/test_pep257.py
src/ur5_qt_panel/test/test_pick_geometry.py
src/ur5_qt_panel/test/test_state_machine.py
src/ur5_qt_panel/test/test_step_pipeline.py
src/ur5_qt_panel/test/test_ur5_kinematics.py
Total funciones test: 375
```


## 4.x ur5_tools


### package.xml


```
<?xml version="1.0"?>
<!-- Ruta/archivo: agarre_ros2_ws/src/ur5_tools/package.xml -->
<!-- Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5. -->
<!-- Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema. -->

<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/package.xml -->
<!-- Summary: ROS 2 package manifest for ur5_tools. -->
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ur5_tools</name>
  <version>0.0.0</version>
  <description>Utilities and helper nodes for UR5 simulation (MoveIt bridge, release service).</description>
  <maintainer email="jesus.lozano.rodriguez@gmail.com">laboratorio</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>moveit_py</depend>
  <depend>ament_index_python</depend>
  <depend>geometry_msgs</depend>
  <depend>moveit_msgs</depend>
  <depend>ros_gz_interfaces</depend>
  <depend>shape_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>tf2_msgs</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>rosgraph_msgs</depend>
  <depend>controller_manager_msgs</depend>
  <depend>std_srvs</depend>
  <exec_depend>moveit_configs_utils</exec_depend>
  <exec_depend>moveit_ros_planning_interface</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```


### setup.py (entry_points)


```
27:    entry_points={
28:        "console_scripts": [
29:            "controller_bootstrap = ur5_tools.controller_bootstrap:main",
30:            "gz_pose_bridge = ur5_tools.gz_pose_bridge:main",
31:            "gz_ros_control_guard = ur5_tools.gz_ros_control_guard:main",
32:            "gripper_attach_backend = ur5_tools.gripper_attach_backend:main",
33:            "planning_scene_sync = ur5_tools.planning_scene_sync:main",
34:            "ur5_moveit_bridge = ur5_tools.ur5_moveit_bridge:main",
35:            "release_objects_service = ur5_tools.release_objects_service:main",
36:            "system_state_manager = ur5_tools.system_state_manager:main",
37:            "world_tf_publisher = ur5_tools.world_tf_publisher:main",
38:            "tf_probe = ur5_tools.tf_probe:main",
39:            "clock_probe = ur5_tools.clock_probe:main",
40:            "jt_smoke_test = ur5_tools.jt_smoke_test:main",
```


### Nodos Python


```
src/ur5_tools/ur5_tools/clock_probe.py
src/ur5_tools/ur5_tools/controller_bootstrap.py
src/ur5_tools/ur5_tools/cycle_logger.py
src/ur5_tools/ur5_tools/gripper_attach_backend.py
src/ur5_tools/ur5_tools/gripper_geometry.py
src/ur5_tools/ur5_tools/gz_pose_bridge.py
src/ur5_tools/ur5_tools/gz_ros_control_guard.py
src/ur5_tools/ur5_tools/__init__.py
src/ur5_tools/ur5_tools/moveit_bridge_utils.py
src/ur5_tools/ur5_tools/param_utils.py
src/ur5_tools/ur5_tools/planning_scene_sync.py
src/ur5_tools/ur5_tools/release_objects_service.py
src/ur5_tools/ur5_tools/system_state_manager.py
src/ur5_tools/ur5_tools/tf_probe.py
src/ur5_tools/ur5_tools/ur5_moveit_bridge.py
src/ur5_tools/ur5_tools/world_tf_publisher.py
```


### Launch files


```
```


### Config YAML


```
```


### Tests


```
src/ur5_tools/test/test_copyright.py
src/ur5_tools/test/test_cycle_logger.py
src/ur5_tools/test/test_flake8.py
src/ur5_tools/test/test_gripper_attach_backend.py
src/ur5_tools/test/test_gripper_geometry.py
src/ur5_tools/test/test_moveit_bridge_plan_success.py
src/ur5_tools/test/test_moveit_bridge_utils.py
src/ur5_tools/test/test_param_utils.py
src/ur5_tools/test/test_pep257.py
src/ur5_tools/test/test_system_state_machine.py
Total funciones test: 165
```


# 5. Entry Points Confirmados

| Paquete | Entry Point | Módulo | Función |
|---------|-------------|--------|---------|
| tfm_grasping | grasp_inference | tfm_grasping.grasp_inference | main |
| ur5_qt_panel | panel_v2 | ur5_qt_panel.panel_v2 | main |
| ur5_qt_panel | main_panel | ur5_qt_panel.main_panel | main |
| ur5_tools | controller_bootstrap | ur5_tools.controller_bootstrap | main |
| ur5_tools | gz_pose_bridge | ur5_tools.gz_pose_bridge | main |
| ur5_tools | gz_ros_control_guard | ur5_tools.gz_ros_control_guard | main |
| ur5_tools | gripper_attach_backend | ur5_tools.gripper_attach_backend | main |
| ur5_tools | planning_scene_sync | ur5_tools.planning_scene_sync | main |
| ur5_tools | ur5_moveit_bridge | ur5_tools.ur5_moveit_bridge | main |
| ur5_tools | release_objects_service | ur5_tools.release_objects_service | main |
| ur5_tools | system_state_manager | ur5_tools.system_state_manager | main |
| ur5_tools | world_tf_publisher | ur5_tools.world_tf_publisher | main |
| ur5_tools | tf_probe | ur5_tools.tf_probe | main |
| ur5_tools | clock_probe | ur5_tools.clock_probe | main |
| ur5_tools | jt_smoke_test | ur5_tools.jt_smoke_test | main |

# 6. Launch Principal


## 6.1 ur5_stack.launch.py (primeras 200 líneas)


```
#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Contenido: Configuracion de bringup ROS 2 para lanzar el sistema UR5.
# Uso breve: Colcon/ros2 launch lo usan para arrancar simulacion y componentes principales.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Resumen: Launch unificado oficial para Gazebo + bridge + ros2_control + panel.
"""Launch unificado oficial del stack UR5 (Gazebo, bridge, ros2_control, panel)."""

from __future__ import annotations

import os
import sys
from typing import List

# ros2 launch loads this file without a parent package, so relative imports
# don't work. Add the launch directory to sys.path for the helper module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from launch_helpers import (
    PANEL_ENV_DEFAULTS,
    build_gz_plugin_path,
    build_gz_resource_path,
    copy_runtime_model,
    patch_bridge_yaml,
    prepare_runtime_world_sdf,
    resolve_gz_partition,
    resolve_world_name,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ur5_tools.gripper_geometry import (
    RG2_TCP_FRAME,
    load_gripper_geometry,
    patch_runtime_model_sdf,
    validate_pick_demo_anchor,
)


def _env_flag(name: str, default: bool) -> bool:
    raw = str(os.environ.get(name, "1" if default else "0") or "").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    raw = str(os.environ.get(name, str(default)) or "").strip()
    try:
        value = float(raw)
    except Exception:
        value = float(default)
    return value


def _prepare_runtime(context, *_args) -> List[object]:
    logger = get_logger("ur5_stack")
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
    world_file = LaunchConfiguration("world_file").perform(context)
    strict_physics_mode = (
        str(LaunchConfiguration("strict_physics_mode").perform(context)).strip().lower()
        in ("1", "true", "yes", "on")
    )
    log_dir = os.path.join(ws_dir, "log")
    os.makedirs(log_dir, exist_ok=True)

    world_name = resolve_world_name(world_file)
    if world_name == "ur5_mesa_objetos" and world_file:
        logger.debug("Using default world_name (could not read from %s)", world_file)

    gz_partition = resolve_gz_partition(log_dir)

    base_yaml = os.path.join(ws_dir, "scripts", "bridge_cameras.yaml")
    panel_settings_yaml = os.path.join(
        ws_dir, "src", "ur5_qt_panel", "config", "panel_settings.yaml"
    )
    runtime_yaml = patch_bridge_yaml(
        base_yaml, os.path.join(log_dir, "bridge_runtime.yaml"), world_name
    )

    runtime_models_root = os.path.join(log_dir, "gz_models")
    runtime_ur5_model = os.path.join(runtime_models_root, "ur5_rg2")
    copy_runtime_model(os.path.join(ws_dir, "models", "ur5_rg2"), runtime_ur5_model)

    resource_path = build_gz_resource_path(runtime_models_root, ws_dir)
    plugin_path = build_gz_plugin_path()
    render_engine = os.environ.get("GZ_RENDER_ENGINE", "").strip() or "ogre2"
    fastdds_profile = os.path.join(ws_dir, "scripts", "fastdds_no_shm.xml")
    # Prefer the explicit launch argument (camera_required:="0/1") over the env var,
    # because env var propagation via SetEnvironmentVariable is unreliable for
    # ExecuteProcess (panel) when DISPLAY is not usable (offscreen mode).
    launch_arg_camera = LaunchConfiguration("camera_required").perform(context).strip()
    camera_required_env = os.environ.get("PANEL_CAMERA_REQUIRED", "").strip()
    if launch_arg_camera in ("0", "false", "False", "no", "off"):
        camera_required_env = "0"
    elif launch_arg_camera in ("1", "true", "True", "yes", "on"):
        camera_required_env = "1"
    elif not camera_required_env:
        camera_required_env = "1"
    camera_required = camera_required_env in ("1", "true", "True")
    control_backend = (
        LaunchConfiguration("control_backend").perform(context).strip().lower()
    )
    moveit_mode = (
        LaunchConfiguration("moveit_mode").perform(context).strip().lower()
    )
    if moveit_mode not in ("auto", "move_group", "bridge"):
        logger.warning("moveit_mode invalido '%s'; usando auto", moveit_mode)
        moveit_mode = "auto"
    launch_gazebo_val = LaunchConfiguration("launch_gazebo").perform(context)
    launch_ros2_control_val = LaunchConfiguration("launch_ros2_control").perform(
        context
    )
    moveit_start_ros2_control_val = LaunchConfiguration(
        "moveit_start_ros2_control"
    ).perform(context)
    launch_ros2_control_eff = launch_ros2_control_val
    if control_backend in ("gz", "gazebo", "gz_ros2_control"):
        if str(launch_ros2_control_val).lower() in ("1", "true", "yes"):
            logger.warning(
                "control_backend=gz_ros2_control: desactivando ros2_control_node para evitar doble controller_manager."
            )
        launch_ros2_control_eff = "false"
    elif control_backend in ("ros2_control", "ros2_control_node"):
        if str(launch_gazebo_val).lower() in ("1", "true", "yes"):
            logger.error(
                "control_backend=ros2_control_node no es compatible con Gazebo: desactivando ros2_control_node."
            )
            launch_ros2_control_eff = "false"
        else:
            launch_ros2_control_eff = "true"
    else:
        logger.warning(
            "control_backend desconocido '%s'; se mantiene launch_ros2_control=%s",
            control_backend,
            launch_ros2_control_val,
        )
    launch_moveit_eff = LaunchConfiguration("launch_moveit").perform(context)
    panel_auto_bridge_eff = LaunchConfiguration("panel_auto_bridge").perform(context)
    if moveit_mode == "move_group":
        # Solo forzar launch_moveit si no se pasó explícitamente como false.
        # Con PANEL_START_STACK=0 (stack externo), start_panel_v2.sh pasa launch_moveit:=false
        # para evitar un segundo move_group que causaría el dual-bridge bug.
        if str(launch_moveit_eff).lower() not in ("0", "false", "no"):
            launch_moveit_eff = "true"
        if str(panel_auto_bridge_eff).lower() in ("1", "true", "yes"):
            logger.warning(
                "moveit_mode=move_group: desactivando panel_auto_bridge para evitar doble backend."
            )
        panel_auto_bridge_eff = "0"
    elif moveit_mode == "bridge":
        if str(launch_moveit_eff).lower() in ("1", "true", "yes"):
            logger.warning(
                "moveit_mode=bridge: desactivando launch_moveit para evitar doble backend."
            )
        launch_moveit_eff = "false"
    launch_scene_sync_eff = "true" if moveit_mode == "move_group" else "false"
    moveit_start_ros2_control_eff = moveit_start_ros2_control_val
    if str(launch_gazebo_val).lower() in ("1", "true", "yes"):
        if str(moveit_start_ros2_control_val).lower() in ("1", "true", "yes"):
            logger.warning(
                "Gazebo activo: desactivando moveit_start_ros2_control para evitar duplicar controller_manager."
            )
        moveit_start_ros2_control_eff = "false"
    launch_attach_backend_eff = LaunchConfiguration("launch_attach_backend").perform(context)
    if strict_physics_mode and str(launch_attach_backend_eff).lower() in ("1", "true", "yes"):
        logger.warning(
            "strict_physics_mode activo: desactivando launch_attach_backend para evitar agarre asistido por software."
        )
        launch_attach_backend_eff = "false"
    launch_flags = [
        LaunchConfiguration("launch_gazebo").perform(context),
        LaunchConfiguration("launch_rsp").perform(context),
        LaunchConfiguration("launch_bridge").perform(context),
        launch_ros2_control_eff,
        launch_moveit_eff,
    ]
    managed = any(str(flag).lower() in ("1", "true", "yes") for flag in launch_flags)
    managed_str = "1" if managed else "0"
    controllers_yaml = os.path.join(
        get_package_share_directory("ur5_description"),
        "config",
```


## 6.2 Nodos lanzados (grep Node)


```
405:    controller_bootstrap = Node(
473:    bridge = Node(
493:    gz_control_guard = Node(
504:    gz_pose_bridge = Node(
524:    world_tf = Node(
546:    system_state = Node(
591:    release_service = Node(
660:    gripper_attach_backend = Node(
668:    planning_scene_sync = Node(
735:    moveit_bridge = Node(
```


## 6.3 launch_helpers.py — funciones auxiliares


```
20:def resolve_world_name(world_file: str, default: str = "ur5_mesa_objetos") -> str:
33:def resolve_gz_partition(log_dir: str) -> str:
46:def patch_bridge_yaml(base_yaml: str, runtime_yaml: str, world_name: str) -> str:
63:def copy_runtime_model(src: str, dst: str) -> None:
72:def build_gz_resource_path(runtime_models_root: str, ws_dir: str) -> str:
79:def build_gz_plugin_path() -> str:
86:def prepare_runtime_world_sdf(
196:def env_passthrough_actions(
```


## 6.4 ur5_moveit_bringup.launch.py


```
# Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py
# Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion.
# Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot.
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ws_dir = os.environ.get("WS_DIR", "")
    ws_src = Path(ws_dir) / "src" if ws_dir else None
    strict_self_collision = os.environ.get("STRICT_SELF_COLLISION", "0").strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    def _pkg_share(pkg_name: str) -> str:
        try:
            return get_package_share_directory(pkg_name)
        except PackageNotFoundError:
            if ws_src:
                candidate = ws_src / pkg_name
                if candidate.is_dir():
                    return str(candidate)
            raise

    ur5_bringup_share = _pkg_share("ur5_bringup")
    ur5_description_share = _pkg_share("ur5_description")
    moveit_share = _pkg_share("ur5_moveit_config")

    srdf_file = "ur5_strict.srdf" if strict_self_collision else "ur5.srdf"
    srdf_path = Path(moveit_share) / "config" / srdf_file
    if not srdf_path.is_file() and ws_src:
        fallback = ws_src / "ur5_moveit_config" / "config" / srdf_file
        if fallback.is_file():
            srdf_path = fallback

    start_ros2_control = LaunchConfiguration("start_ros2_control")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur5_bringup_share, "launch", "ur5_ros2_control.launch.py"])
        ),
        condition=IfCondition(start_ros2_control),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5_rg2", package_name="ur5_moveit_config")
        .robot_description(
            file_path=str(Path(ur5_description_share) / "urdf" / "ur5.urdf.xacro"),
            mappings={"ur_type": "ur5", "name": "ur5_rg2"},
        )
        .robot_description_semantic(file_path=str(srdf_path))
        .robot_description_kinematics(
            file_path=str(Path(moveit_share) / "config" / "kinematics.yaml")
        )
        .joint_limits(
            file_path=str(Path(moveit_share) / "config" / "joint_limits.yaml")
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(
            file_path=str(Path(moveit_share) / "config" / "moveit_controllers.yaml")
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.to_dict(),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(launch_rviz),
    )
```


## 6.5 ur5_rsp.launch.py


```
#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/ur5_rsp.launch.py
# Contenido: Configuracion de bringup ROS 2 para lanzar el sistema UR5.
# Uso breve: Colcon/ros2 launch lo usan para arrancar simulacion y componentes principales.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_rsp.launch.py
# Summary: Launches robot_state_publisher for UR5 using joint_states from bridge.
"""Launch robot_state_publisher for UR5 (use_sim_time)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable


def generate_launch_description():
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    ur_type = LaunchConfiguration("ur_type")
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare(description_pkg),
            "urdf",
            xacro_file,
        ]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_path,
                " ",
                "ur_type:=",
                ur_type,
                " ",
                "name:=",
                robot_name,
            ]
        ),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description_content},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("description_pkg", default_value="ur5_description"),
            DeclareLaunchArgument("xacro_file", default_value="ur5.urdf.xacro"),
            DeclareLaunchArgument("ur_type", default_value="ur5"),
            DeclareLaunchArgument("robot_name", default_value="ur5"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            rsp,
        ]
    )
```


## 6.6 Variables de entorno del launcher (start_panel_v2.sh)

> Archivo: `scripts/start_panel_v2.sh` — 910 líneas

```
13:log() { echo "[START_PANEL_V2] $*"; }
14:err() { echo "[START_PANEL_V2] ERROR: $*" >&2; }
37:  local strict_sanity="${PANEL_STRICT_RUNTIME_SANITY:-1}"
55:    err "Detén esos procesos o exporta PANEL_STRICT_RUNTIME_SANITY=0 bajo tu responsabilidad."
87:      export DISPLAY="$candidate_display"
89:        export XAUTHORITY="$candidate_xauth"
92:        export XDG_SESSION_TYPE="$candidate_session"
101:# Detectar WS_DIR (raíz del repo)
103:WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
104:RUNTIME_PROFILE="${RUNTIME_PROFILE:-$WS_DIR/scripts/panel_runtime_validated.env}"
105:export WS_DIR
108:: "${ROS_DISTRO:=jazzy}"
109:: "${PANEL_COLD_BOOT:=1}"          # 1 = mata procesos antes de arrancar
110:: "${PANEL_V2_PREFER_INSTALLED:=1}"# 1 = usa install/.../panel_v2 si existe
111:: "${PANEL_MODE:=auto}"            # auto=stack completo por defecto (usar manual para modo legacy)
112:: "${PANEL_START_STACK:=1}"        # 1 = autoarranca RSP+Gazebo
113:: "${PANEL_MANAGED:=0}"            # 1 = panel guiado por /system_state
114:: "${PANEL_CONTROLLER_MANAGER:=/controller_manager}" # namespace del controller_manager
115:: "${PANEL_WRITE_PID:=0}"          # 1 = ejecutar en background y crear pidfile
116:: "${PANEL_PROMPT_PID:=0}"         # 1 = preguntar si se quiere pidfile (solo TTY)
117:: "${PANEL_VENV_AUTO:=1}"          # 1 = activa venv automaticamente si existe
118:: "${PANEL_VENV_DIR:=}"
119:: "${PANEL_STRICT_PHYSICS_MODE:=0}" # 1 = desactiva attach asistido y usa self-collision mas estricta
120:: "${PANEL_STRICT_RUNTIME_SANITY:=1}" # 1 = bloquea arranque si detecta otro stack ROS activo
121:: "${PANEL_MOVEIT_MODE:=auto}"        # auto|move_group|bridge
122:: "${PANEL_ROS_EXECUTOR_THREADS:=3}"  # executor del RosWorker para no bloquear /clock durante servicios largos
123:: "${PANEL_GRIPPER_OPEN_RAD:=0.0425}"  # Gripper prismático: apertura máxima 0.0425 m por dedo
124:: "${PANEL_GRIPPER_CLOSED_RAD:=0.0}"  # RG2 prismático: cierre completo en 0.0 m
125:: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
126:if [[ "${PANEL_FORCE_OFFSCREEN:-0}" != "1" ]] && ! display_is_usable; then
130:    err "Si realmente quieres ejecutar sin GUI, exporta PANEL_FORCE_OFFSCREEN=1."
134:if [[ -z "${PANEL_GZ_GUI+x}" ]]; then
135:  if [[ -n "${DISPLAY:-}" && "${PANEL_FORCE_OFFSCREEN:-0}" != "1" ]]; then
136:    PANEL_GZ_GUI="1"
138:    PANEL_GZ_GUI="0"
141:if [[ "${PANEL_GZ_GUI:-0}" == "1" && -z "${PANEL_QT_PLATFORM:-}" && "${QT_QPA_PLATFORM:-}" == "offscreen" ]]; then
145:export RMW_IMPLEMENTATION
146:export PANEL_CONTROLLER_MANAGER
147:export PANEL_ROS_EXECUTOR_THREADS
148:export PANEL_GRIPPER_OPEN_RAD
149:export PANEL_GRIPPER_CLOSED_RAD
150:export PANEL_GZ_GUI
152:if [[ -z "${PANEL_VENV_DIR}" ]]; then
158:      PANEL_VENV_DIR="$candidate"
163:if [[ -z "${PANEL_VENV_DIR}" ]]; then
164:  PANEL_VENV_DIR="/home/laboratorio/TFM/agarre_inteligente/.venv-tfm"
171:      PANEL_WRITE_PID=1
175:      PANEL_PROMPT_PID=1
179:      PANEL_WRITE_PID=0
183:      export PANEL_TFM_REPRO_MODE=1
187:      export PANEL_TFM_RAW_OUTPUT=1
197:log "WS_DIR=$WS_DIR"
198:log "PANEL_CONTROLLER_MANAGER=${PANEL_CONTROLLER_MANAGER}"
199:if [[ -n "${PANEL_TFM_REPRO_MODE:-}" ]]; then
200:  log "PANEL_TFM_REPRO_MODE=${PANEL_TFM_REPRO_MODE} (perfil EXP3 seed_0 para reproduccion TFM)"
202:if [[ -n "${PANEL_TFM_RAW_OUTPUT:-}" ]]; then
203:  log "PANEL_TFM_RAW_OUTPUT=${PANEL_TFM_RAW_OUTPUT} (prediccion raw sin ajustes panel)"
207:if [[ "$PANEL_VENV_AUTO" == "1" && -z "${VIRTUAL_ENV:-}" ]]; then
208:  if [[ -f "$PANEL_VENV_DIR/bin/activate" ]]; then
209:    log "activando venv: $PANEL_VENV_DIR"
212:    source "$PANEL_VENV_DIR/bin/activate"
216:if [[ -n "${VIRTUAL_ENV:-}" && -x "$PANEL_VENV_DIR/bin/python" ]]; then
217:  export PANEL_PYTHON="$PANEL_VENV_DIR/bin/python"
220:  export VISION_DIR="/home/laboratorio/TFM/agarre_inteligente"
224:if [[ "$PANEL_COLD_BOOT" == "1" ]]; then
336:log "cargando entorno ROS 2 ${ROS_DISTRO} ($WS_DIR)"
338:ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
339:if [[ ! -f "$ROS_SETUP" ]]; then
340:  err "No existe $ROS_SETUP. ¿Seguro que estás en ROS 2 ${ROS_DISTRO}?"
347:source "$ROS_SETUP"
349:if [[ -f "$WS_DIR/install/setup.bash" ]]; then
351:  source "$WS_DIR/install/setup.bash"
358:export GZ_SIM_RESOURCE_PATH="$WS_DIR/models:$WS_DIR/worlds:$WS_DIR/install${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
359:export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
360:export PANEL_AUTO_BRIDGE_DELAY_MS="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}"
361:export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"
362:export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$WS_DIR/scripts/fastdds_no_shm.xml}"
363:export USE_SIM_TIME="${USE_SIM_TIME:-1}"
364:log "GZ_SIM_RESOURCE_PATH set to: $GZ_SIM_RESOURCE_PATH"
366:if [[ "${PANEL_FORCE_SOFTWARE_GL:-0}" == "1" ]]; then
```


# 7. Variables de Entorno y Parámetros


## 7.1 panel_runtime_validated.env (perfil validado)


```
# Perfil runtime validado para UR5 + RG2 tras el cierre del incidente TCP.
# Compartido por el launcher canonico y por el harness de validacion DIRECTO.
# Las pruebas finales deben repetirse manualmente con ./lanzar_panelc2.sh.

export PANEL_RUNTIME_VALIDATED_PROFILE="${PANEL_RUNTIME_VALIDATED_PROFILE:-rg2_tcp_fix_20260422}"

export SYSTEM_STATE_STARTUP_TIMEOUT_SEC="${SYSTEM_STATE_STARTUP_TIMEOUT_SEC:-15.0}"
export SYSTEM_STATE_GEOMETRY_OFFSET_TOL_M="${SYSTEM_STATE_GEOMETRY_OFFSET_TOL_M:-0.002}"
export SYSTEM_STATE_GEOMETRY_PAIR_TOL_M="${SYSTEM_STATE_GEOMETRY_PAIR_TOL_M:-0.001}"
export STRICT_SELF_COLLISION="${STRICT_SELF_COLLISION:-1}"
export PANEL_MOVEIT_BRIDGE_UNWRAP_CONTINUOUS_JOINTS="${PANEL_MOVEIT_BRIDGE_UNWRAP_CONTINUOUS_JOINTS:-1}"
export PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD="${PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD:-0.35}"
export PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD="${PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD:-0.10}"

export PANEL_PICK_DEMO_MOVE_SEC="${PANEL_PICK_DEMO_MOVE_SEC:-15}"
export PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC="${PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC:-60}"
export PANEL_TF_INIT_GRACE_SEC="${PANEL_TF_INIT_GRACE_SEC:-300}"
export PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL="${PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL:-0.200}"
export PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL="${PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL:-0.200}"
export PANEL_PICK_DEMO_IK_SEED_JOINTS="${PANEL_PICK_DEMO_IK_SEED_JOINTS:-0.0,-1.5708,0.0,-1.5708,0.0,-3.07}"
export PANEL_FATAL_STOPS_ALL="${PANEL_FATAL_STOPS_ALL:-0}"
export PANEL_GZ_HEALTH_FREEZE_SEC="${PANEL_GZ_HEALTH_FREEZE_SEC:-30}"
export PANEL_ALLOW_UNSETTLED_ON_TIMEOUT="${PANEL_ALLOW_UNSETTLED_ON_TIMEOUT:-1}"
export PANEL_PICK_DEMO_MAX_PROMOTED_STABLE_AGE_SEC="${PANEL_PICK_DEMO_MAX_PROMOTED_STABLE_AGE_SEC:-900}"
export PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC="${PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC:-120}"
export PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M="${PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M:-0.015}"
export PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M="${PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M:-0.025}"
export PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT="${PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT:-0.035}"
export PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_SEED_WEIGHT="${PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_SEED_WEIGHT:-0.035}"
export PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK="${PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK:-0}"
export PANEL_PICK_DEMO_ALIGN_IK_SEED_WEIGHT="${PANEL_PICK_DEMO_ALIGN_IK_SEED_WEIGHT:-0.035}"
export PANEL_PICK_DEMO_POSE_SOURCE_TOL_M="${PANEL_PICK_DEMO_POSE_SOURCE_TOL_M:-1.0}"
export PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC="${PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC:-300.0}"
export PANEL_PICK_DEMO_POSE_SOURCE_SYNC_TOL_SEC="${PANEL_PICK_DEMO_POSE_SOURCE_SYNC_TOL_SEC:-300.0}"
export PANEL_MOVEIT_STARTUP_TIMEOUT_SEC="${PANEL_MOVEIT_STARTUP_TIMEOUT_SEC:-300}"
export PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M="${PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M:-0.016}"
export PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M="${PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M:-0.020}"
export PANEL_PICK_DEMO_ATTACH_XY_TOL_M="${PANEL_PICK_DEMO_ATTACH_XY_TOL_M:-0.020}"
export PANEL_PICK_DEMO_ATTACH_SETTLE_SEC="${PANEL_PICK_DEMO_ATTACH_SETTLE_SEC:-4.0}"
export PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M="${PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M:-0.050}"
export PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M="${PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M:-0.080}"
export PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES="${PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES:-3}"
export PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC="${PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC:-0.25}"
export PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_FK_ERR_M="${PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_FK_ERR_M:-0.012}"
export PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_RAD="${PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_RAD:-0.90}"
export PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_SUM_RAD="${PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_SUM_RAD:-2.75}"
export PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE="${PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE:-0.35}"
export PANEL_MOVEIT_BRIDGE_ACCEL_SCALE="${PANEL_MOVEIT_BRIDGE_ACCEL_SCALE:-0.35}"
export PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M="${PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M:-0.500}"
export PANEL_PICK_DEMO_TRANSPORT_MIN_WORLD_Z_M="${PANEL_PICK_DEMO_TRANSPORT_MIN_WORLD_Z_M:-0.720}"
export PANEL_PICK_DEMO_TRANSPORT_BASKET_MAX_TCP_DIST_M="${PANEL_PICK_DEMO_TRANSPORT_BASKET_MAX_TCP_DIST_M:-0.220}"
export PANEL_PICK_DEMO_TRANSPORT_RELEASE_MAX_TCP_DIST_M="${PANEL_PICK_DEMO_TRANSPORT_RELEASE_MAX_TCP_DIST_M:-0.180}"
export PANEL_PICK_DEMO_BASKET_TRANSPORT_IK_ERR_TOL_M="${PANEL_PICK_DEMO_BASKET_TRANSPORT_IK_ERR_TOL_M:-0.012}"
export PANEL_PICK_DEMO_BASKET_TRANSPORT_ROT_WEIGHT="${PANEL_PICK_DEMO_BASKET_TRANSPORT_ROT_WEIGHT:-0.020}"
export PANEL_PICK_DEMO_BASKET_TRANSPORT_JOINT_WEIGHT="${PANEL_PICK_DEMO_BASKET_TRANSPORT_JOINT_WEIGHT:-0.020}"
export PANEL_PICK_DEMO_TRANSPORT_SPLIT_STAGES="${PANEL_PICK_DEMO_TRANSPORT_SPLIT_STAGES:-5}"
export PANEL_PICK_DEMO_TRANSPORT_MAX_STAGES="${PANEL_PICK_DEMO_TRANSPORT_MAX_STAGES:-24}"
export PANEL_PICK_DEMO_TRANSPORT_MAX_STAGE_DIST_M="${PANEL_PICK_DEMO_TRANSPORT_MAX_STAGE_DIST_M:-0.060}"
export PANEL_PICK_DEMO_TRANSPORT_RUNTIME_GRACE_SEC="${PANEL_PICK_DEMO_TRANSPORT_RUNTIME_GRACE_SEC:-35.0}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_ENABLE="${PANEL_PICK_DEMO_TRANSPORT_PREP_ENABLE:-1}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_BLEND="${PANEL_PICK_DEMO_TRANSPORT_PREP_BLEND:-0.550}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_MAX_JOINT_DELTA_RAD="${PANEL_PICK_DEMO_TRANSPORT_PREP_MAX_JOINT_DELTA_RAD:-0.160}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_MAX_SHOULDER_DELTA_RAD="${PANEL_PICK_DEMO_TRANSPORT_PREP_MAX_SHOULDER_DELTA_RAD:-0.050}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_SUM_JOINT_DELTA_RAD="${PANEL_PICK_DEMO_TRANSPORT_PREP_SUM_JOINT_DELTA_RAD:-0.450}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_MAX_STEPS="${PANEL_PICK_DEMO_TRANSPORT_PREP_MAX_STEPS:-8}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_JOINT_TOL_RAD="${PANEL_PICK_DEMO_TRANSPORT_PREP_JOINT_TOL_RAD:-0.060}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_DIRECT_REPLAN_MIN_FAILED_FRACTION="${PANEL_PICK_DEMO_TRANSPORT_PREP_DIRECT_REPLAN_MIN_FAILED_FRACTION:-0.700}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_DIRECT_REPLAN_MAX_RESIDUAL_RAD="${PANEL_PICK_DEMO_TRANSPORT_PREP_DIRECT_REPLAN_MAX_RESIDUAL_RAD:-0.120}"
export PANEL_PICK_DEMO_TRANSPORT_PREP_DIRECT_REPLAN_MAX_SHOULDER_RESIDUAL_RAD="${PANEL_PICK_DEMO_TRANSPORT_PREP_DIRECT_REPLAN_MAX_SHOULDER_RESIDUAL_RAD:-0.100}"
export PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_ATTEMPTS="${PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_ATTEMPTS:-1}"
export PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_STAGES="${PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_STAGES:-2}"
export PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_STAGES="${PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_STAGES:-4}"
export PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_REMAINING_DIST_M="${PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_REMAINING_DIST_M:-0.060}"
export PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_STAGE_DIST_M="${PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_STAGE_DIST_M:-0.050}"
export PANEL_PICK_DEMO_TRANSPORT_PREEXEC_MODEL_TOL_M="${PANEL_PICK_DEMO_TRANSPORT_PREEXEC_MODEL_TOL_M:-0.012}"
export PANEL_PICK_DEMO_TRANSPORT_POSTCHECK_MODEL_TOL_M="${PANEL_PICK_DEMO_TRANSPORT_POSTCHECK_MODEL_TOL_M:-0.040}"
```


## 7.2 Variables PANEL_* detectadas en el launcher


```
PANEL_ALLOW_UNSETTLED_ON_TIMEOUT
PANEL_AUTO_BRIDGE
PANEL_AUTO_BRIDGE_DELAY_MS
PANEL_AUTO_EXIT_ON_PANEL
PANEL_AUTO_RELEASE_DROP_OBJECTS
PANEL_CAMERA_REQUIRED
PANEL_COLD_BOOT
PANEL_CONTROLLER_MANAGER
PANEL_CONTROLLER_READY_TIMEOUT_SEC
PANEL_CRITICAL_CLOCK_TIMEOUT_SEC
PANEL_CRITICAL_POSE_TIMEOUT_SEC
PANEL_FORCE_OFFSCREEN
PANEL_FORCE_SOFTWARE_GL
PANEL_GRIPPER_CLOSED_RAD
PANEL_GRIPPER_OPEN_RAD
PANEL_GZ_GUI
PANEL_KEEP_CAMERAS
PANEL_KILL_STALE
PANEL_LAUNCH_BRIDGE
PANEL_LAUNCH_MOVEIT
PANEL_LAUNCH_SYSTEM_STATE
PANEL_LAUNCH_WORLD_TF
PANEL_LOG_FILTER
PANEL_MANAGED
PANEL_MODE
PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_FK_ERR_M
PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_RAD
PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_SUM_RAD
PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN
PANEL_MOVEIT_BRIDGE_APPROACH_PATH_CONSTRAINT_TOL_RAD
PANEL_MOVEIT_BRIDGE_APPROACH_SKIP_CONSTRAINTS
PANEL_MOVEIT_BRIDGE_CONTROLLER_EXPECTED_GOAL_TIME_SEC
PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TIME_TOL_SEC
PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD
PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD
PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC
PANEL_MOVEIT_BRIDGE_FORCE_FJT_DIRECT
PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_STABLE_SEC
PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TIMEOUT_SEC
PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TOL_RAD
PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC
PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC
PANEL_MOVEIT_BRIDGE_PATH_CONSTRAINT_TOL_RAD
PANEL_MOVEIT_BRIDGE_PREGRASP_EE_TARGET_TOL_M
PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC
PANEL_MOVEIT_BRIDGE_START_BLEND_ERR_RAD
PANEL_MOVEIT_BRIDGE_START_BLEND_RATIO
PANEL_MOVEIT_BRIDGE_START_LEAD_GAIN
PANEL_MOVEIT_BRIDGE_START_LEAD_MAX_SEC
PANEL_MOVEIT_MODE
PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M
PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M
PANEL_PICK_OBJECT_ALLOW_JOINT_PREFLIGHT_IN_MOVEIT
PANEL_PICK_OBJECT_ALLOW_JOINT_RECOVERY_IN_MOVEIT
PANEL_PICK_OBJECT_APPROACH_CARTESIAN
PANEL_PICK_OBJECT_APPROACH_JOINT_FALLBACK
PANEL_PICK_OBJECT_APPROACH_SKIP_RETRY_ON_FALLBACK
PANEL_PICK_OBJECT_APPROACH_TOL_M
PANEL_PICK_OBJECT_CARRY_GATE_ENABLE
PANEL_PICK_OBJECT_CARRY_GATE_MAX_DIST_M
PANEL_PICK_OBJECT_CARRY_GATE_TIMEOUT_SEC
PANEL_PICK_OBJECT_CARRY_JOINT_TIME_SEC
PANEL_PICK_OBJECT_CONTACT_DOWN_Z_M
PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MAX_M
PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MIN_CONSECUTIVE
PANEL_PICK_OBJECT_DETERMINISTIC_JOINT_AFTER_APPROACH
PANEL_PICK_OBJECT_EXTRA_DOWN_Z
PANEL_PICK_OBJECT_FORCE_HOME_START
PANEL_PICK_OBJECT_GRASP_CARTESIAN
PANEL_PICK_OBJECT_GRASP_JOINT_FALLBACK
PANEL_PICK_OBJECT_GRASP_STEP_TOL_M
PANEL_PICK_OBJECT_GRASP_Z_REACH_TOL_FALLBACK_M
PANEL_PICK_OBJECT_HOME_BEFORE_CESTA
PANEL_PICK_OBJECT_HOME_START_DUR_SEC
PANEL_PICK_OBJECT_HOME_START_READY_WAIT_SEC
PANEL_PICK_OBJECT_HOME_TOL_RAD
PANEL_PICK_OBJECT_LIFT_CARTESIAN
PANEL_PICK_OBJECT_MOVEIT_ACTIVE_REQUEST_GRACE_SEC
PANEL_PICK_OBJECT_MOVEIT_EXCLUSIVE
PANEL_PICK_OBJECT_MOVEIT_WAIT_RECOVERED_SEC
PANEL_PICK_OBJECT_MOVEIT_WAIT_SEC
PANEL_PICK_OBJECT_OBJECT_XY_GATE_TOL_M
PANEL_PICK_OBJECT_ORIENTATION_XYZW
PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_REQUIRE_REACHED
PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_TOL_RAD
PANEL_PICK_OBJECT_POST_LIFT_MAX_DIST_M
PANEL_PICK_OBJECT_POST_LIFT_MIN_CONSECUTIVE
PANEL_PICK_OBJECT_PREFLIGHT_MODE
PANEL_PICK_OBJECT_PRE_GRASP_ABORT_RECOVERY_TOL_M
PANEL_PICK_OBJECT_PRE_GRASP_ALIGN_XY_TOL_M
PANEL_PICK_OBJECT_PRE_GRASP_JOINT_FALLBACK
PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_ENABLE
PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_TOL_M
PANEL_PICK_OBJECT_PRE_GRASP_TOL_M
PANEL_PICK_OBJECT_RETURN_TO_MESA
PANEL_PICK_OBJECT_STAGED_LIFT_ALWAYS
PANEL_PICK_OBJECT_STEP_TOL_M
PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_MAX_DIST_M
PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_STEP_M
PANEL_PICK_OBJECT_TCP_OBJ_DIST_MAX_M
PANEL_PICK_OBJECT_TRANSPORT_CARTESIAN
PANEL_PICK_OBJECT_TRANSPORT_JOINT_FALLBACK
PANEL_PICK_OBJECT_TRANSPORT_JOINT_ONLY
PANEL_PICK_OBJECT_TRANSPORT_MOVEIT_WAIT_SEC
PANEL_PICK_OBJECT_TRANSPORT_SKIP_CONSTRAINTS
PANEL_PICK_OBJECT_WORLD_READY_WAIT_SEC
PANEL_PROMPT_PID
PANEL_PYTHON
PANEL_QT_PLATFORM
PANEL_ROS_EXECUTOR_THREADS
PANEL_RUNTIME_PROFILE_PATH
PANEL_RUNTIME_VALIDATED_PROFILE
PANEL_SELECT_OBJECT_SERVICE_TIMEOUT_SEC
PANEL_SETTINGS_YAML
PANEL_SKIP_CLEANUP
PANEL_STALE_GRACE_SEC
PANEL_START_ROS2_CONTROL
PANEL_START_STACK
PANEL_STEP_HISTORY_TARGET_XY_TOL_M
PANEL_STEP_HISTORY_TARGET_Z_TOL_M
PANEL_STRICT_PHYSICS_MODE
PANEL_STRICT_RUNTIME_SANITY
PANEL_TFM_CANONICAL_GRASP_XY_MAX_DELTA_M
PANEL_TFM_CANONICAL_SNAP_SELECTED_XY
PANEL_TFM_CANONICAL_USE_GRASP_YAW
PANEL_TFM_CANONICAL_USE_PICK_OBJECT
PANEL_TFM_EXECUTE_SERVICE_TIMEOUT_SEC
PANEL_TFM_GRASP_CARTESIAN
PANEL_TFM_INFER_SERVICE_TIMEOUT_SEC
PANEL_TFM_INFER_USE_ROI
PANEL_TFM_MOVEIT_ACTIVE_REQUEST_GRACE_SEC
PANEL_TFM_MOVEIT_RESULT_TIMEOUT_SEC
PANEL_TFM_RAW_OUTPUT
PANEL_TFM_REPRO_MODE
PANEL_V2
PANEL_V2_PREFER_INSTALLED
PANEL_VENV_AUTO
PANEL_VENV_DIR
PANEL_WRITE_PID
```


## 7.3 Variables GZ_* / GAZEBO_* / ROS_*


```
GZ_GUI
GZ_RENDER_ENGINE
GZ_SIM_RESOURCE_PATH
GZ_SIM_SYSTEM_PLUGIN_PATH
ROS_DISTRO
ROS_EXECUTOR_THREADS
ROS_SETUP
```


## 7.4 Constantes en panel_config.py


```
17:SETTINGS = PanelSettings.from_env()
19:WS_DIR = SETTINGS.ws_dir
20:SCRIPTS_DIR = SETTINGS.scripts_dir
21:WORLDS_DIR = SETTINGS.worlds_dir
22:MODELS_DIR = SETTINGS.models_dir
23:LOG_DIR = SETTINGS.log_dir
24:BAGS_DIR = SETTINGS.bags_dir
25:FIG_DIR = SETTINGS.fig_dir
26:VISION_DIR = SETTINGS.vision_dir
27:VISION_EXP_DIR = SETTINGS.vision_exp_dir
28:VISION_PLOTS_DIR = SETTINGS.vision_plots_dir
29:VISION_SUMMARY = SETTINGS.vision_summary
30:VISION_FIG_DIR = SETTINGS.vision_fig_dir
32:TABLE_SIZE_X = SETTINGS.table_size_x
33:TABLE_SIZE_Y = SETTINGS.table_size_y
34:TABLE_CENTER_X = SETTINGS.table_center_x
35:TABLE_CENTER_Y = SETTINGS.table_center_y
36:TABLE_IMAGE_SWAP_XY = SETTINGS.table_image_swap_xy
37:TABLE_IMAGE_FLIP_X = SETTINGS.table_image_flip_x
38:TABLE_IMAGE_FLIP_Y = SETTINGS.table_image_flip_y
39:TABLE_CALIB_PATH = os.path.join(SCRIPTS_DIR, "table_pixel_map.json")
44:TABLE_OBJECT_XY_MARGIN = SETTINGS.table_object_xy_margin
45:TABLE_OBJECT_Z_MIN = SETTINGS.table_object_z_min
46:TABLE_OBJECT_Z_MAX = SETTINGS.table_object_z_max
47:TABLE_TOP_Z = SETTINGS.reach_overlay_z
49:NUDGE_DROP_OBJECTS = SETTINGS.nudge_drop_objects
50:NUDGE_DROP_DZ = SETTINGS.nudge_drop_dz
51:NUDGE_DROP_Z_MIN = SETTINGS.nudge_drop_z_min
52:SELECTION_SNAP_DIST = SETTINGS.selection_snap_dist
53:OBJECT_POS_PATH = SETTINGS.object_pos_path
54:SAVE_POSE_INFO_POSITIONS = SETTINGS.save_pose_info_positions
61:PICK_DEMO_SPAWN_POSE = (-0.42, 0.00, 0.875)
62:_PICK_DEMO_OBJECTS = {
65:ATTACHABLE_OBJECTS = ("pick_demo",)
66:PICK_DEMO_NAME_SET = set(ATTACHABLE_OBJECTS)
68:_DROP_AIR_OBJECTS = {
80:DROP_NAME_SET = {
94:OBJECT_POSITIONS = {**_PICK_DEMO_OBJECTS, **_DROP_AIR_OBJECTS}
130:OBJECT_SHAPES = {
143:OBJECT_LABELS = {
156:OBJECT_COLORS = {
170:BASKET_DROP = (-1.30, 0.00, 0.82)
171:GZ_WORLD = SETTINGS.gz_world
172:GRIPPER_ATTACH_PREFIX = SETTINGS.gripper_attach_prefix
173:DROP_ANCHOR_PREFIX = "/drop_anchor"
174:GZ_PARTITION_FILE = os.path.join(LOG_DIR, "gz_partition.txt")
176:INFER_SCRIPT = SETTINGS.infer_script
177:INFER_CKPT = SETTINGS.infer_ckpt
178:INFER_ROI_SIZE = SETTINGS.infer_roi_size
179:INFER_RETRY_ERR_PX = SETTINGS.infer_retry_err_px
180:FASTRTPS_PROFILES = SETTINGS.fastrtps_profiles
184:BRIDGE_BASE_YAML = SETTINGS.bridge_base_yaml
185:EGL_VENDOR = SETTINGS.egl_vendor
186:AUTO_START_BRIDGE = SETTINGS.auto_start_bridge
187:AUTO_START_BRIDGE_DELAY_MS = SETTINGS.auto_start_bridge_delay_ms
188:AUTO_START_BRIDGE_MAX_RETRIES = SETTINGS.auto_start_bridge_max_retries
190:DEFAULT_WORLD_CANDIDATES = SETTINGS.default_world_candidates
192:DEBUG_FRAME_LOG = SETTINGS.debug_frame_log
194:BASE_FRAME = SETTINGS.base_frame
195:WORLD_FRAME = SETTINGS.world_frame
200:GLOBAL_FRAME_EFFECTIVE = "base_link"
202:ARM_TRAJ_TOPIC_DEFAULT = SETTINGS.arm_traj_topic_default
204:GRIPPER_JOINT_NAMES = SETTINGS.gripper_joint_names
208:JOINT_SLIDER_DEG_MIN = SETTINGS.joint_slider_deg_min
209:JOINT_SLIDER_DEG_MAX = SETTINGS.joint_slider_deg_max
210:JOINT_SLIDER_SCALE = SETTINGS.joint_slider_scale
211:DEFAULT_JOINT_MOVE_SEC = SETTINGS.default_joint_move_sec
212:DEPTH_PCTL_REFRESH_FRAMES = SETTINGS.depth_pctl_refresh_frames
213:DEPTH_PCTL_STRIDE = SETTINGS.depth_pctl_stride
214:DEPTH_FAST = SETTINGS.depth_fast
215:DEBUG_LOGS_TO_STDOUT = SETTINGS.debug_logs_to_stdout
216:PANEL_GZ_GUI = SETTINGS.panel_gz_gui
217:DEBUG_JOINTS_TO_STDOUT = SETTINGS.debug_joints_to_stdout
218:PANEL_JOINT_STATES_TOPIC = SETTINGS.joint_states_topic
219:PANEL_CAMERA_TOPIC = SETTINGS.camera_topic
220:USE_SIM_TIME = SETTINGS.use_sim_time
221:PANEL_SKIP_CLEANUP = SETTINGS.skip_cleanup
222:PANEL_KILL_STALE = SETTINGS.kill_stale
223:SELECTION_TIMEOUT_SEC = SETTINGS.selection_timeout_sec
224:TRAJ_ACTION_FALLBACK = SETTINGS.traj_action_fallback
```


## 7.5 panel_settings.py — dataclass campos


```
87:        print(f"[PANEL][WARN] Error leyendo YAML {path}: {exc}", file=sys.stderr, flush=True)
95:@dataclass(frozen=True)
96:class PanelSettings:
109:    table_size_x: float = 0.768
110:    table_size_y: float = 0.80
111:    table_center_x: float = -0.17
112:    table_center_y: float = 0.0
113:    table_image_swap_xy: bool = True
114:    table_image_flip_x: bool = True
115:    table_image_flip_y: bool = True
116:    table_object_xy_margin: float = 0.09
117:    table_object_z_min: float = 0.6
118:    table_object_z_max: float = 1.55
119:    table_object_whitelist: Optional[List[str]] = None
120:    nudge_drop_objects: bool = False
121:    nudge_drop_dz: float = 0.08
122:    nudge_drop_z_min: float = 1.6
123:    selection_snap_dist: float = 0.0
124:    object_pos_path: str = ""
125:    save_pose_info_positions: bool = False
126:    ur5_base_x: float = -0.85
127:    ur5_base_y: float = 0.0
130:    ur5_base_z: float = 0.850
131:    ur5_reach_radius: float = 0.85
132:    gz_world: str = "ur5_mesa_objetos"
133:    gripper_attach_prefix: str = "/gripper"
134:    gz_partition_file: str = ""
135:    infer_script: str = ""
136:    infer_ckpt: str = ""
137:    infer_roi_size: int = 96
138:    infer_retry_err_px: float = 60.0
139:    fastrtps_profiles: str = ""
140:    ur5_controllers_yaml: str = ""
141:    ur5_joint_limits_yaml: str = ""
142:    bridge_base_yaml: str = ""
143:    egl_vendor: str = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
144:    auto_start_bridge: bool = True
145:    auto_start_bridge_delay_ms: int = 1200
146:    auto_start_bridge_max_retries: int = 30
147:    default_world_candidates: List[str] = field(default_factory=list)
148:    debug_frame_log: bool = False
149:    base_frame: Optional[str] = None
150:    world_frame: Optional[str] = None
151:    arm_traj_topic_default: str = "/joint_trajectory_controller/joint_trajectory"
152:    ur5_joint_names: List[str] = field(default_factory=list)
153:    gripper_joint_names: List[str] = field(default_factory=list)
154:    ur5_home_env: str = ""
155:    ur5_home_default: List[float] = field(default_factory=list)
156:    ur5_model_name: str = "ur5_rg2"
157:    joint_slider_deg_min: float = -180.0
158:    joint_slider_deg_max: float = 180.0
159:    joint_slider_scale: float = 10.0
160:    default_joint_move_sec: float = 2.0
161:    depth_pctl_refresh_frames: int = 15
162:    depth_pctl_stride: int = 4
163:    depth_fast: bool = False
164:    debug_logs_to_stdout: bool = False
165:    panel_gz_gui: bool = False
166:    debug_joints_to_stdout: bool = False
167:    joint_states_topic: str = "/joint_states"
168:    camera_topic: str = "/camera_overhead/image"
169:    use_sim_time: bool = True
170:    skip_cleanup: bool = False
171:    kill_stale: bool = True
172:    selection_timeout_sec: float = 12.0
173:    traj_action_fallback: bool = True
174:    traj_action_fallback_delay_sec: float = 1.0
175:    traj_action_fallback_eps_rad: float = 0.002
176:    traj_action_fallback_timeout_sec: float = 2.0
177:    controller_ready_timeout_sec: float = 20.0
178:    controller_ready_cache_sec: float = 0.0
179:    moveit_ready_timeout_sec: float = 20.0
180:    gz_launch_timeout_sec: float = 20.0
181:    bridge_launch_timeout_sec: float = 12.0
182:    moveit_launch_timeout_sec: float = 25.0
183:    moveit_bridge_launch_timeout_sec: float = 15.0
184:    controller_drop_grace_sec: float = 3.0
185:    trace_print_period_sec: float = 3.0
186:    debug_poses_period_sec: float = 3.0
187:    pick_log_min_interval_sec: float = 2.0
```


# 8. URDF / Xacro — ur5.urdf.xacro


## 8.1 Propiedades xacro (frames, offsets)


```
7:<!-- Summary: UR5 description with mock ros2_control and RG2 gripper joints. -->
8:<robot xmlns:xacro="http://wiki.ros.org/xacro" name="ur5_rg2">
9:  <xacro:property name="ur_type" value="ur5"/>
10:  <xacro:property name="safety_limits" value="false"/>
11:  <xacro:property name="safety_pos_margin" value="0.15"/>
12:  <xacro:property name="safety_k_position" value="20"/>
13:  <xacro:property name="force_abs_paths" value="false"/>
15:  <xacro:property name="ur_pkg_share" value="$(optenv UR_DESCRIPTION_SHARE /opt/ros/jazzy/share/ur_description)"/>
16:  <xacro:property name="robot_name" value="ur5_rg2"/>
17:  <xacro:property name="tf_prefix_value" value=""/>
18:  <!-- Offset físico real flange→yemas: 0.175 m (ur5_hand_joint Z=0.07 + finger_joint X=0.105).
20:  <xacro:property name="rg2_contact_tcp_xyz" value="0 0 0.175"/>
21:  <xacro:property name="joint_limits_file" value="${ur_pkg_share}/config/${ur_type}/joint_limits.yaml"/>
22:  <xacro:property name="kinematics_file" value="${ur_pkg_share}/config/${ur_type}/default_kinematics.yaml"/>
23:  <xacro:property name="physical_file" value="${ur_pkg_share}/config/${ur_type}/physical_parameters.yaml"/>
24:  <xacro:property name="visual_file" value="${ur_pkg_share}/config/${ur_type}/visual_parameters.yaml"/>
50:  <!-- RG2 gripper: simplified prismatic box geometry, coherent with SDF/TF/ros2_control. -->
51:  <link name="rg2_base_link">
60:      <material name="rg2_gray"><color rgba="0.4 0.4 0.4 1"/></material>
68:  <joint name="rg2_mount_joint" type="fixed">
69:    <parent link="tool0"/>
70:    <child link="rg2_base_link"/>
75:       The RG2 contact midpoint is slightly in front of tool0 in tool local +Z:
76:       the previous -0.0877005 value landed near the finger hinges, leaving the
77:       semantic TCP about 9.3 cm behind the visible pinch center. -->
78:  <link name="rg2_tcp">
86:  <joint name="rg2_tcp_joint" type="fixed">
87:    <parent link="tool0"/>
88:    <child link="rg2_tcp"/>
89:    <origin xyz="${rg2_contact_tcp_xyz}" rpy="0 0 0"/>
92:    <!-- Operational pinch-center: contact midpoint between the two RG2 fingers.
95:  <link name="rg2_pinch_center">
103:  <joint name="rg2_pinch_center_joint" type="fixed">
104:    <parent link="tool0"/>
105:    <child link="rg2_pinch_center"/>
106:    <origin xyz="${rg2_contact_tcp_xyz}" rpy="0 0 0"/>
109:  <!-- Finger links: box 12x10x175 mm; joint origin at ±20 mm Y from rg2_base_link. -->
110:  <!-- Visual center at 0 0 0.0875 (half of 175 mm) in finger-link frame. -->
111:  <link name="rg2_finger_link1">
120:      <material name="rg2_dark_gray"><color rgba="0.25 0.25 0.25 1"/></material>
128:  <link name="rg2_finger_link2">
137:      <material name="rg2_dark_gray"/>
145:  <joint name="rg2_finger_joint1" type="prismatic">
146:    <parent link="rg2_base_link"/>
147:    <child link="rg2_finger_link1"/>
153:  <joint name="rg2_finger_joint2" type="prismatic">
154:    <parent link="rg2_base_link"/>
155:    <child link="rg2_finger_link2"/>
210:    <joint name="rg2_finger_joint1">
217:    <joint name="rg2_finger_joint2">
```


## 8.2 Joints relevantes


```
47:    <origin xyz="-0.85 0 0.850" rpy="0 0 0"/>
54:      <origin xyz="0 0 0.010" rpy="0 0 0"/>
58:      <origin xyz="0 0 0.010" rpy="0 0 0"/>
63:      <origin xyz="0 0 0.010" rpy="0 0 0"/>
68:  <joint name="rg2_mount_joint" type="fixed">
69:    <parent link="tool0"/>
70:    <child link="rg2_base_link"/>
71:    <origin xyz="0 0 0" rpy="0 0 0"/>
81:      <origin xyz="0 0 0" rpy="0 0 0"/>
86:  <joint name="rg2_tcp_joint" type="fixed">
87:    <parent link="tool0"/>
88:    <child link="rg2_tcp"/>
89:    <origin xyz="${rg2_contact_tcp_xyz}" rpy="0 0 0"/>
98:      <origin xyz="0 0 0" rpy="0 0 0"/>
103:  <joint name="rg2_pinch_center_joint" type="fixed">
104:    <parent link="tool0"/>
105:    <child link="rg2_pinch_center"/>
106:    <origin xyz="${rg2_contact_tcp_xyz}" rpy="0 0 0"/>
114:      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
118:      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
123:      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
131:      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
135:      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
140:      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
145:  <joint name="rg2_finger_joint1" type="prismatic">
146:    <parent link="rg2_base_link"/>
147:    <child link="rg2_finger_link1"/>
148:    <origin xyz="0 0.020 0" rpy="0 0 0"/>
153:  <joint name="rg2_finger_joint2" type="prismatic">
154:    <parent link="rg2_base_link"/>
155:    <child link="rg2_finger_link2"/>
156:    <origin xyz="0 -0.020 0" rpy="0 0 0"/>
167:    <joint name="shoulder_pan_joint">
174:    <joint name="shoulder_lift_joint">
181:    <joint name="elbow_joint">
188:    <joint name="wrist_1_joint">
195:    <joint name="wrist_2_joint">
202:    <joint name="wrist_3_joint">
210:    <joint name="rg2_finger_joint1">
217:    <joint name="rg2_finger_joint2">
```


## 8.3 Xacro completo (primeras 300 líneas)


```
<?xml version="1.0"?>
<!-- Ruta/archivo: agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro -->
<!-- Contenido: Descripcion URDF/Xacro y configuracion base del robot UR5. -->
<!-- Uso breve: Se usa al publicar el robot model y al preparar simulacion y MoveIt. -->

<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro -->
<!-- Summary: UR5 description with mock ros2_control and RG2 gripper joints. -->
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="ur5_rg2">
  <xacro:property name="ur_type" value="ur5"/>
  <xacro:property name="safety_limits" value="false"/>
  <xacro:property name="safety_pos_margin" value="0.15"/>
  <xacro:property name="safety_k_position" value="20"/>
  <xacro:property name="force_abs_paths" value="false"/>

  <xacro:property name="ur_pkg_share" value="$(optenv UR_DESCRIPTION_SHARE /opt/ros/jazzy/share/ur_description)"/>
  <xacro:property name="robot_name" value="ur5_rg2"/>
  <xacro:property name="tf_prefix_value" value=""/>
  <!-- Offset físico real flange→yemas: 0.175 m (ur5_hand_joint Z=0.07 + finger_joint X=0.105).
       Corregido con Opción 3 SDF fix (2026-04-24). -->
  <xacro:property name="rg2_contact_tcp_xyz" value="0 0 0.175"/>
  <xacro:property name="joint_limits_file" value="${ur_pkg_share}/config/${ur_type}/joint_limits.yaml"/>
  <xacro:property name="kinematics_file" value="${ur_pkg_share}/config/${ur_type}/default_kinematics.yaml"/>
  <xacro:property name="physical_file" value="${ur_pkg_share}/config/${ur_type}/physical_parameters.yaml"/>
  <xacro:property name="visual_file" value="${ur_pkg_share}/config/${ur_type}/visual_parameters.yaml"/>

  <xacro:include filename="${ur_pkg_share}/urdf/ur_macro.xacro"/>

  <link name="world"/>

  <xacro:ur_robot
    name="${robot_name}"
    tf_prefix="${tf_prefix_value}"
    parent="world"
    joint_limits_parameters_file="${joint_limits_file}"
    kinematics_parameters_file="${kinematics_file}"
    physical_parameters_file="${physical_file}"
    visual_parameters_file="${visual_file}"
    safety_limits="${safety_limits}"
    safety_pos_margin="${safety_pos_margin}"
    safety_k_position="${safety_k_position}"
    force_abs_paths="${force_abs_paths}"
    >
    <!-- Keep the URDF world origin aligned with the Gazebo world include pose.
         If these diverge, robot_state_publisher TF and the rendered Gazebo model
         disagree in Z, which makes the panel report a different TCP height than
         the one visible in the simulator. -->
    <origin xyz="-0.85 0 0.850" rpy="0 0 0"/>
  </xacro:ur_robot>

  <!-- RG2 gripper: simplified prismatic box geometry, coherent with SDF/TF/ros2_control. -->
  <link name="rg2_base_link">
    <inertial>
      <mass value="0.30"/>
      <origin xyz="0 0 0.010" rpy="0 0 0"/>
      <inertia ixx="0.00050" ixy="0.0" ixz="0.0" iyy="0.00040" iyz="0.0" izz="0.00060"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.010" rpy="0 0 0"/>
      <geometry><box size="0.060 0.080 0.020"/></geometry>
      <material name="rg2_gray"><color rgba="0.4 0.4 0.4 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.010" rpy="0 0 0"/>
      <geometry><box size="0.060 0.080 0.020"/></geometry>
    </collision>
  </link>

  <joint name="rg2_mount_joint" type="fixed">
    <parent link="tool0"/>
    <child link="rg2_base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Virtual TCP frame at the operational grasp center used by planning/attach.
       The RG2 contact midpoint is slightly in front of tool0 in tool local +Z:
       the previous -0.0877005 value landed near the finger hinges, leaving the
       semantic TCP about 9.3 cm behind the visible pinch center. -->
  <link name="rg2_tcp">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="rg2_tcp_joint" type="fixed">
    <parent link="tool0"/>
    <child link="rg2_tcp"/>
    <origin xyz="${rg2_contact_tcp_xyz}" rpy="0 0 0"/>
  </joint>

    <!-- Operational pinch-center: contact midpoint between the two RG2 fingers.
      Keep it coincident with the semantic TCP so DIRECTO, MoveIt and attach all
      reason about the same visible closure center. -->
  <link name="rg2_pinch_center">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="rg2_pinch_center_joint" type="fixed">
    <parent link="tool0"/>
    <child link="rg2_pinch_center"/>
    <origin xyz="${rg2_contact_tcp_xyz}" rpy="0 0 0"/>
  </joint>

  <!-- Finger links: box 12x10x175 mm; joint origin at ±20 mm Y from rg2_base_link. -->
  <!-- Visual center at 0 0 0.0875 (half of 175 mm) in finger-link frame. -->
  <link name="rg2_finger_link1">
    <inertial>
      <mass value="0.05"/>
      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
      <inertia ixx="1.3e-4" ixy="0.0" ixz="0.0" iyy="1.3e-4" iyz="0.0" izz="1.0e-6"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
      <geometry><box size="0.012 0.010 0.175"/></geometry>
      <material name="rg2_dark_gray"><color rgba="0.25 0.25 0.25 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
      <geometry><box size="0.012 0.010 0.175"/></geometry>
    </collision>
  </link>

  <link name="rg2_finger_link2">
    <inertial>
      <mass value="0.05"/>
      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
      <inertia ixx="1.3e-4" ixy="0.0" ixz="0.0" iyy="1.3e-4" iyz="0.0" izz="1.0e-6"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
      <geometry><box size="0.012 0.010 0.175"/></geometry>
      <material name="rg2_dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.0875" rpy="0 0 0"/>
      <geometry><box size="0.012 0.010 0.175"/></geometry>
    </collision>
  </link>

  <joint name="rg2_finger_joint1" type="prismatic">
    <parent link="rg2_base_link"/>
    <child link="rg2_finger_link1"/>
    <origin xyz="0 0.020 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.0425" effort="20" velocity="0.2"/>
  </joint>

  <joint name="rg2_finger_joint2" type="prismatic">
    <parent link="rg2_base_link"/>
    <child link="rg2_finger_link2"/>
    <origin xyz="0 -0.020 0" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0.0" upper="0.0425" effort="20" velocity="0.2"/>
  </joint>

  <!-- ros2_control (gz_ros2_control) with UR5 + RG2 joints -->
  <ros2_control name="${robot_name}" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <joint name="shoulder_pan_joint">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="shoulder_lift_joint">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">-1.5707963267948966</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="elbow_joint">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wrist_1_joint">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">-1.5707963267948966</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wrist_2_joint">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wrist_3_joint">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>

    <joint name="rg2_finger_joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="rg2_finger_joint2">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
```


# 9. SDF Model — models/ur5_rg2/model.sdf


## 9.1 Joints y frames relevantes


```
2:<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/models/ur5_rg2/model.sdf -->
5:    <model name="ur5_rg2">
58:            <pose relative_to="shoulder_pan_joint">0 0 0 0 0 0</pose>
96:            <pose relative_to="shoulder_lift_joint">0 0 0 0 0 0</pose>
134:            <pose relative_to="elbow_joint">0 0 0 0 0 0</pose>
172:            <pose relative_to="wrist_1_joint">0 0 0 0 0 0</pose>
210:            <pose relative_to="wrist_2_joint">0 0 0 0 0 0</pose>
248:            <pose relative_to="wrist_3_joint">0 0 0 0 0 0</pose>
285:        <joint name="shoulder_pan_joint" type="revolute">
286:            <pose relative_to="base_link">0 0 0.089159 0 0 0</pose>
287:            <parent>base_link</parent>
288:            <child>shoulder_link</child>
305:        <joint name="shoulder_lift_joint" type="revolute">
306:            <pose relative_to="shoulder_link">0 0.13585 0 0 0 0</pose>
307:            <parent>shoulder_link</parent>
308:            <child>upper_arm_link</child>
325:        <joint name="elbow_joint" type="revolute">
326:            <pose relative_to="upper_arm_link">0 -0.1197 0.425 0 0 0</pose>
327:            <parent>upper_arm_link</parent>
328:            <child>forearm_link</child>
345:        <joint name="wrist_1_joint" type="revolute">
346:            <pose relative_to="forearm_link">0 0 0.39225 0 0 0</pose>
347:            <parent>forearm_link</parent>
348:            <child>wrist_1_link</child>
365:        <joint name="wrist_2_joint" type="revolute">
366:            <pose relative_to="wrist_1_link">0 0.093 0 0 0 0</pose>
367:            <parent>wrist_1_link</parent>
368:            <child>wrist_2_link</child>
385:        <joint name="wrist_3_joint" type="revolute">
386:            <pose relative_to="wrist_2_link">0 0 0.09465 0 0 0</pose>
387:            <parent>wrist_2_link</parent>
388:            <child>wrist_3_link</child>
406:        <!-- rg2_base_link at flange: tool0 is 0.275m past wrist_3_link (Z axis), flange is at
407:             0.0823m, so offset = -(0.275-0.0823) = -0.1927m along tool0 Z. -->
408:        <joint name="rg2_mount_joint" type="fixed">
409:            <pose relative_to="tool0">0 0 -0.1927 0 0 0</pose>
410:            <parent>tool0</parent>
411:            <child>rg2_base_link</child>
413:        <link name="rg2_base_link">
415:            <pose relative_to="rg2_mount_joint">0 0 0 0 0 0</pose>
416:            <visual name="rg2_base_link_visual">
426:            <collision name="rg2_base_link_collision">
444:        <!-- Finger links: boxes 12×10×175 mm, joint origin at ±20 mm in Y from rg2_base_link. -->
445:        <link name="rg2_finger_link1">
447:            <pose relative_to="rg2_finger_joint1">0 0 0 0 0 0</pose>
448:            <visual name="rg2_finger_link1_visual">
458:            <collision name="rg2_finger_link1_collision">
488:        <link name="rg2_finger_link2">
490:            <pose relative_to="rg2_finger_joint2">0 0 0 0 0 0</pose>
491:            <visual name="rg2_finger_link2_visual">
501:            <collision name="rg2_finger_link2_collision">
532:        <joint name="rg2_finger_joint1" type="prismatic">
533:            <pose relative_to="rg2_base_link">0 0.020 0 0 0 0</pose>
534:            <parent>rg2_base_link</parent>
535:            <child>rg2_finger_link1</child>
550:        <joint name="rg2_finger_joint2" type="prismatic">
551:            <pose relative_to="rg2_base_link">0 -0.020 0 0 0 0</pose>
552:            <parent>rg2_base_link</parent>
553:            <child>rg2_finger_link2</child>
569:        <link name="tool0">
571:            <pose relative_to="end_effector_frame_fixed_joint">0 0 0 0 0 0</pose>
573:        <joint name="end_effector_frame_fixed_joint" type="fixed">
574:            <pose relative_to="wrist_3_link">0 0.275 0 -1.570796325 0 0</pose>
575:            <parent>wrist_3_link</parent>
576:            <child>tool0</child>
580:            <pose relative_to="pick_demo_anchor_joint">0 0 0 0 0 0</pose>
582:        <joint name="pick_demo_anchor_joint" type="fixed">
584:                 tool0 -> rg2_pinch_center is +0.175 m (URDF rg2_pinch_center_joint).
586:                 so the anchor must be at +0.175 m to match rg2_pinch_center exactly. -->
587:            <pose relative_to="tool0">0 0 0.175 0 0 0</pose>
588:            <parent>tool0</parent>
589:            <child>pick_demo_anchor</child>
592:        <!-- Wrist camera: mounted on rg2_base_link looking along tool0 +Z toward the object.
596:            <pose relative_to="camera_wrist_joint">0 0 0 0 0 0</pose>
608:        <joint name="camera_wrist_joint" type="fixed">
609:            <pose relative_to="rg2_base_link">0 0.04 0.01 0 0 0</pose>
610:            <parent>rg2_base_link</parent>
611:            <child>camera_wrist_link</child>
617:            <parent_link>rg2_base_link</parent_link>
626:            <parent_link>rg2_base_link</parent_link>
```


## 9.2 Gripper SDF — comentarios de geometría


```
405:        <!-- Simplified RG2 gripper: prismatic box fingers, coherent with URDF/TF/ros2_control. -->
406:        <!-- rg2_base_link at flange: tool0 is 0.275m past wrist_3_link (Z axis), flange is at
407:             0.0823m, so offset = -(0.275-0.0823) = -0.1927m along tool0 Z. -->
408:        <joint name="rg2_mount_joint" type="fixed">
411:            <child>rg2_base_link</child>
413:        <link name="rg2_base_link">
415:            <pose relative_to="rg2_mount_joint">0 0 0 0 0 0</pose>
416:            <visual name="rg2_base_link_visual">
426:            <collision name="rg2_base_link_collision">
444:        <!-- Finger links: boxes 12×10×175 mm, joint origin at ±20 mm in Y from rg2_base_link. -->
445:        <link name="rg2_finger_link1">
447:            <pose relative_to="rg2_finger_joint1">0 0 0 0 0 0</pose>
448:            <visual name="rg2_finger_link1_visual">
451:                    <box><size>0.012 0.010 0.175</size></box>
458:            <collision name="rg2_finger_link1_collision">
461:                    <box><size>0.012 0.010 0.175</size></box>
488:        <link name="rg2_finger_link2">
490:            <pose relative_to="rg2_finger_joint2">0 0 0 0 0 0</pose>
491:            <visual name="rg2_finger_link2_visual">
494:                    <box><size>0.012 0.010 0.175</size></box>
501:            <collision name="rg2_finger_link2_collision">
504:                    <box><size>0.012 0.010 0.175</size></box>
531:        <!-- Finger Joints: prismatic, symmetric, axis Y/-Y, limit 0..0.0425 m -->
532:        <joint name="rg2_finger_joint1" type="prismatic">
533:            <pose relative_to="rg2_base_link">0 0.020 0 0 0 0</pose>
534:            <parent>rg2_base_link</parent>
535:            <child>rg2_finger_link1</child>
550:        <joint name="rg2_finger_joint2" type="prismatic">
551:            <pose relative_to="rg2_base_link">0 -0.020 0 0 0 0</pose>
552:            <parent>rg2_base_link</parent>
553:            <child>rg2_finger_link2</child>
584:                 tool0 -> rg2_pinch_center is +0.175 m (URDF rg2_pinch_center_joint).
585:                 The tcp_z_offset=0.05 m was removed in commit 9b58910 (_DIRECTO_GRASP_Z=0.0),
586:                 so the anchor must be at +0.175 m to match rg2_pinch_center exactly. -->
587:            <pose relative_to="tool0">0 0 0.175 0 0 0</pose>
592:        <!-- Wrist camera: mounted on rg2_base_link looking along tool0 +Z toward the object.
609:            <pose relative_to="rg2_base_link">0 0.04 0.01 0 0 0</pose>
610:            <parent>rg2_base_link</parent>
617:            <parent_link>rg2_base_link</parent_link>
626:            <parent_link>rg2_base_link</parent_link>
635:            <parent_link>rg2_base_link</parent_link>
644:            <parent_link>rg2_base_link</parent_link>
653:            <parent_link>rg2_base_link</parent_link>
662:            <parent_link>rg2_base_link</parent_link>
671:            <parent_link>rg2_base_link</parent_link>
680:            <parent_link>rg2_base_link</parent_link>
689:            <parent_link>rg2_base_link</parent_link>
698:            <parent_link>rg2_base_link</parent_link>
777:            <joint name="rg2_finger_joint1">
784:            <joint name="rg2_finger_joint2">
```


## 9.3 SDF completo (primeras 300 líneas)


```
<?xml version="1.0"?>
<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/models/ur5_rg2/model.sdf -->
<!-- Summary: SDF model of the UR5 robot with RG2 gripper meshes and joints. -->
<sdf version="1.7">
    <model name="ur5_rg2">
        <!-- Arm Links -->
        <link name="base_link">
            <gravity>false</gravity>
            <visual name="base_link_visual">
                <geometry>
                    <mesh>
                        <uri>meshes/visual/ur5/base.dae</uri>
                    </mesh>
                </geometry>
            </visual>
            <collision name="base_link_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/collision/ur5/base.stl</uri>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1.2</mu>
                            <mu2>1.2</mu2>
                        </ode>
                    </friction>
                    <bounce>
                        <restitution_coefficient>0.0</restitution_coefficient>
                        <threshold>100000</threshold>
                    </bounce>
                    <contact>
                        <ode>
                            <kp>150000</kp>
                            <kd>20.0</kd>
                            <max_vel>0.1</max_vel>
                            <min_depth>0.001</min_depth>
                        </ode>
                    </contact>
                </surface>
            </collision>
            <inertial>
                <pose>-7.83914e-07 0.00442178 0.184343 0 0 0</pose>
                <mass>4.210389694173812</mass>
                <inertia>
                    <ixx>0.09309670560602573</ixx>
                    <iyy>0.09257037651287492</iyy>
                    <izz>0.004310645613985299</izz>
                    <ixy>5.711270872759674e-09</ixy>
                    <ixz>2.0039833861772347e-07</ixz>
                    <iyz>0.0047053532779924665</iyz>
                </inertia>
            </inertial>
        </link>
        <link name="shoulder_link">
            <gravity>false</gravity>
            <pose relative_to="shoulder_pan_joint">0 0 0 0 0 0</pose>
            <visual name="shoulder_link_visual">
                <geometry>
                    <mesh>
                        <uri>meshes/visual/ur5/shoulder.dae</uri>
                    </mesh>
                </geometry>
            </visual>
            <collision name="shoulder_link_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/collision/ur5/shoulder.stl</uri>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1.2</mu>
                            <mu2>1.2</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <inertial>
                <pose>-3.81853e-06 0.00927291 -0.0018563 0 0 0</pose>
                <mass>2.6078628081050645</mass>
                <inertia>
                    <ixx>0.00615890739477387</ixx>
                    <iyy>0.005562932983198189</iyy>
                    <izz>0.005255082470414319</izz>
                    <ixy>8.486678716437087e-07</ixy>
                    <ixz>1.436028981488676e-07</ixz>
                    <iyz>-0.00018809980689883695</iyz>
                </inertia>
            </inertial>
        </link>
        <link name="upper_arm_link">
            <gravity>false</gravity>
            <pose relative_to="shoulder_lift_joint">0 0 0 0 0 0</pose>
            <visual name="upper_arm_link_visual">
                <geometry>
                    <mesh>
                        <uri>meshes/visual/ur5/upperarm.dae</uri>
                    </mesh>
                </geometry>
            </visual>
            <collision name="upper_arm_link_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/collision/ur5/upperarm.stl</uri>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1.2</mu>
                            <mu2>1.2</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <inertial>
                <pose>9.68139e-10 -0.00107158 0.2125 0 0 0</pose>
                <mass>9.034556202946057</mass>
                <inertia>
                    <ixx>0.2566537454248199</ixx>
                    <iyy>0.25483850258961777</iyy>
                    <izz>0.016022276702360172</izz>
                    <ixy>-2.2173460061911962e-09</ixy>
                    <ixz>2.3397723734924104e-06</ixz>
                    <iyz>5.701897149736449e-09</iyz>
                </inertia>
            </inertial>
        </link>
        <link name="forearm_link">
            <gravity>false</gravity>
            <pose relative_to="elbow_joint">0 0 0 0 0 0</pose>
            <visual name="forearm_link_visual">
                <geometry>
                    <mesh>
                        <uri>meshes/visual/ur5/forearm.dae</uri>
                    </mesh>
                </geometry>
            </visual>
            <collision name="forearm_link_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/collision/ur5/forearm.stl</uri>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1.2</mu>
                            <mu2>1.2</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <inertial>
                <pose>-7.83914e-07 0.00442178 0.184343 0 0 0</pose>
                <mass>4.210389694173812</mass>
                <inertia>
                    <ixx>0.09309670560602573</ixx>
                    <iyy>0.09257037651287492</iyy>
                    <izz>0.004310645613985299</izz>
                    <ixy>5.711270872759674e-09</ixy>
                    <ixz>2.0039833861772347e-07</ixz>
                    <iyz>0.0047053532779924665</iyz>
                </inertia>
            </inertial>
        </link>
        <link name="wrist_1_link">
            <gravity>false</gravity>
            <pose relative_to="wrist_1_joint">0 0 0 0 0 0</pose>
            <visual name="wrist_1_link_visual">
                <geometry>
                    <mesh>
                        <uri>meshes/visual/ur5/wrist1.dae</uri>
                    </mesh>
                </geometry>
            </visual>
            <collision name="wrist_1_link_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/collision/ur5/wrist1.stl</uri>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1.2</mu>
                            <mu2>1.2</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <inertial>
                <pose>2.12195e-06 0.0886236 -0.00357675 0 0 0</pose>
                <mass>0.8652783044597422</mass>
                <inertia>
                    <ixx>0.0011006744464830784</ixx>
                    <iyy>0.0010246557119841899</iyy>
                    <izz>0.0007125213984803068</izz>
                    <ixy>-4.062057055066431e-08</ixy>
                    <ixz>-2.441133081746347e-08</ixz>
                    <iyz>1.000924524333879e-05</iyz>
                </inertia>
            </inertial>
        </link>
        <link name="wrist_2_link">
            <gravity>false</gravity>
            <pose relative_to="wrist_2_joint">0 0 0 0 0 0</pose>
            <visual name="wrist_2_link_visual">
                <geometry>
                    <mesh>
                        <uri>meshes/visual/ur5/wrist2.dae</uri>
                    </mesh>
                </geometry>
            </visual>
            <collision name="wrist_2_link_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/collision/ur5/wrist2.stl</uri>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1.2</mu>
                            <mu2>1.2</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <inertial>
                <pose>-2.12306e-06 -0.00357678 0.0901045 0 0 0</pose>
                <mass>0.8652786866955592</mass>
                <inertia>
                    <ixx>0.0010980920821000012</ixx>
                    <iyy>0.0007099378137563144</iyy>
                    <izz>0.0010246570857519936</izz>
                    <ixy>2.436032506659575e-08</ixy>
                    <ixz>4.047086050601145e-08</ixz>
                    <iyz>4.96085194908414e-06</iyz>
                </inertia>
            </inertial>
        </link>
        <link name="wrist_3_link">
            <gravity>false</gravity>
            <pose relative_to="wrist_3_joint">0 0 0 0 0 0</pose>
            <visual name="wrist_3_link_visual">
                <geometry>
                    <mesh>
                        <uri>meshes/visual/ur5/wrist3.dae</uri>
                    </mesh>
                </geometry>
            </visual>
            <collision name="wrist_3_link_collision">
                <geometry>
                    <mesh>
                        <uri>meshes/collision/ur5/wrist3.stl</uri>
                    </mesh>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>1.2</mu>
                            <mu2>1.2</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <inertial>
                <pose>8.12548e-09 0.0630914 0.000189932 0 0 0</pose>
                <mass>0.23285976783205536</mass>
                <inertia>
                    <ixx>0.00010068520800989181</ixx>
                    <iyy>0.00016016377462915846</iyy>
                    <izz>9.951199288758246e-05</izz>
                    <ixy>-1.3195380569038253e-11</ixy>
                    <ixz>2.0466677535340047e-11</ixz>
                    <iyz>3.421454203362164e-08</iyz>
                </inertia>
            </inertial>
        </link>
        <!-- Arm Joints -->
        <joint name="shoulder_pan_joint" type="revolute">
            <pose relative_to="base_link">0 0 0.089159 0 0 0</pose>
            <parent>base_link</parent>
            <child>shoulder_link</child>
            <axis>
                <xyz>0 0 1</xyz>
                <limit>
                    <effort>150</effort>
                    <lower>-6.28319</lower>
                    <upper>6.28319</upper>
                    <velocity>3.15</velocity>
                </limit>
                <dynamics>
                    <spring_reference>0</spring_reference>
                    <spring_stiffness>0</spring_stiffness>
                    <damping>1.0</damping>
```


## 9.4 ur5_controllers.yaml (modelo)


```
# Ruta/archivo: agarre_ros2_ws/models/ur5_rg2/ur5_controllers.yaml
# Contenido: Recurso de modelo de simulacion para Gazebo.
# Uso breve: Gazebo lo carga como parte del entorno del workspace ROS 2.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description/config/ur5_controllers.yaml
# Summary: Controller configuration for UR5 and RG2 gripper.
controller_manager:
  ros__parameters:
    update_rate: 125

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    arm_position_controller:
      type: forward_command_controller/ForwardCommandController

    gripper_controller:
      type: forward_command_controller/ForwardCommandController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    interpolate_from_desired_state: false
    set_last_command_interface_value_as_state_on_activation: true
    speed_scaling:
      initial_scaling_factor: 1.0
      state_interface: ""
      command_interface: ""
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
      shoulder_pan_joint:
        trajectory: 3.4
        goal: 0.1
      shoulder_lift_joint:
        trajectory: 3.4
        goal: 0.1
      elbow_joint:
        trajectory: 3.4
        goal: 0.1
      wrist_1_joint:
        trajectory: 3.4
        goal: 0.1
      wrist_2_joint:
        trajectory: 3.4
        goal: 0.1
      wrist_3_joint:
        trajectory: 3.4
        goal: 0.1

arm_position_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    interface_name: position

gripper_controller:
  ros__parameters:
    joints:
      - rg2_finger_joint1
      - rg2_finger_joint2
    interface_name: position
```


# 10. World SDF


## 10.1 ur5_mesa_objetos.sdf (primeras 200 líneas)


```
<sdf version="1.10">
  <world name="ur5_mesa_objetos">

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />
    
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.95 0.95 0.95 1</background>
      <shadows>true</shadows>
    </scene>

    <physics name="ode_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <dart>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    
    <light name="sun_world" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.5 -1</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><diffuse>0.85 0.85 0.85 1</diffuse></material>
        </visual>
      </link>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_pose>true</publish_pose>
        <publish_link_pose>true</publish_link_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <static_publisher>false</static_publisher>
      </plugin>
    </model>

    
    <model name="mesa_pro">
      <static>true</static>
      <pose>-0.17 0 0.075 0 0 0</pose>
      <link name="link">

        <collision name="tablero_collision">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><box><size>0.768 0.80 0.05</size></box></geometry>
        </collision>
        <visual name="tablero_visual">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><box><size>0.768 0.80 0.05</size></box></geometry>
          <material><diffuse>0.92 0.92 0.92 1</diffuse></material>
        </visual>

        
        
        <collision name="wall_far_x_pos_c">
          <pose>0.374 0 0.785 0 0 0</pose>
          <geometry><box><size>0.02 0.76 0.02</size></box></geometry>
        </collision>
        <visual name="wall_far_x_pos_v">
          <pose>0.374 0 0.785 0 0 0</pose>
          <geometry><box><size>0.02 0.76 0.02</size></box></geometry>
          <material><diffuse>0.35 0.35 0.35 1</diffuse></material>
        </visual>

        <collision name="wall_near_x_neg_c">
          <pose>-0.374 0 0.785 0 0 0</pose>
          <geometry><box><size>0.02 0.76 0.02</size></box></geometry>
        </collision>
        <visual name="wall_near_x_neg_v">
          <pose>-0.374 0 0.785 0 0 0</pose>
          <geometry><box><size>0.02 0.76 0.02</size></box></geometry>
          <material><diffuse>0.35 0.35 0.35 1</diffuse></material>
        </visual>

        <collision name="wall_north_c">
          <pose>0 0.39 0.785 0 0 0</pose>
          <geometry><box><size>0.728 0.02 0.02</size></box></geometry>
        </collision>
        <visual name="wall_north_v">
          <pose>0 0.39 0.785 0 0 0</pose>
          <geometry><box><size>0.728 0.02 0.02</size></box></geometry>
          <material><diffuse>0.35 0.35 0.35 1</diffuse></material>
        </visual>

        <collision name="wall_south_c">
          <pose>0 -0.39 0.785 0 0 0</pose>
          <geometry><box><size>0.728 0.02 0.02</size></box></geometry>
        </collision>
        <visual name="wall_south_v">
          <pose>0 -0.39 0.785 0 0 0</pose>
          <geometry><box><size>0.728 0.02 0.02</size></box></geometry>
          <material><diffuse>0.35 0.35 0.35 1</diffuse></material>
        </visual>

        <!-- TEST markers physically painted on top of the table (inside front corners). -->
        <!-- They match PANEL_TEST_CORNER_INSET_M=0.06 with current table geometry. -->
        <visual name="test_marker_front_left_disk_v">
          <pose>-0.324 0.340 0.776 0 0 0</pose>
          <geometry><cylinder><radius>0.025</radius><length>0.003</length></cylinder></geometry>
          <material>
            <ambient>0.96 0.96 0.96 1</ambient>
            <diffuse>0.96 0.96 0.96 1</diffuse>
            <emissive>0.05 0.05 0.05 1</emissive>
          </material>
        </visual>
        <visual name="test_marker_front_left_cross_h_v">
          <pose>-0.324 0.340 0.778 0 0 0</pose>
          <geometry><box><size>0.034 0.005 0.002</size></box></geometry>
          <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material>
        </visual>
        <visual name="test_marker_front_left_cross_v_v">
          <pose>-0.324 0.340 0.778 0 0 0</pose>
          <geometry><box><size>0.005 0.034 0.002</size></box></geometry>
          <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material>
        </visual>

        <visual name="test_marker_front_right_disk_v">
          <pose>-0.324 -0.340 0.776 0 0 0</pose>
          <geometry><cylinder><radius>0.025</radius><length>0.003</length></cylinder></geometry>
          <material>
            <ambient>0.96 0.96 0.96 1</ambient>
            <diffuse>0.96 0.96 0.96 1</diffuse>
            <emissive>0.05 0.05 0.05 1</emissive>
          </material>
        </visual>
        <visual name="test_marker_front_right_cross_h_v">
          <pose>-0.324 -0.340 0.778 0 0 0</pose>
          <geometry><box><size>0.034 0.005 0.002</size></box></geometry>
          <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material>
        </visual>
        <visual name="test_marker_front_right_cross_v_v">
          <pose>-0.324 -0.340 0.778 0 0 0</pose>
          <geometry><box><size>0.005 0.034 0.002</size></box></geometry>
          <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material>
        </visual>

        
        <collision name="p1c"><pose> 0.334  0.35 0.375 0 0 0</pose><geometry><box><size>0.05 0.05 0.75</size></box></geometry></collision>
        <visual name="p1v"><pose> 0.334  0.35 0.375 0 0 0</pose><geometry><box><size>0.05 0.05 0.75</size></box></geometry><material><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>

        <collision name="p2c"><pose> 0.334 -0.35 0.375 0 0 0</pose><geometry><box><size>0.05 0.05 0.75</size></box></geometry></collision>
        <visual name="p2v"><pose> 0.334 -0.35 0.375 0 0 0</pose><geometry><box><size>0.05 0.05 0.75</size></box></geometry><material><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>

        <collision name="p3c"><pose>-0.334  0.35 0.375 0 0 0</pose><geometry><box><size>0.05 0.05 0.75</size></box></geometry></collision>
        <visual name="p3v"><pose>-0.334  0.35 0.375 0 0 0</pose><geometry><box><size>0.05 0.05 0.75</size></box></geometry><material><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>

        <collision name="p4c"><pose>-0.334 -0.35 0.375 0 0 0</pose><geometry><box><size>0.05 0.05 0.75</size></box></geometry></collision>
        <visual name="p4v"><pose>-0.334 -0.35 0.375 0 0 0</pose><geometry><box><size>0.05 0.05 0.75</size></box></geometry><material><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>

      </link>
    </model>

    
    <model name="pick_demo">
      <static>false</static>
      <allow_auto_disable>true</allow_auto_disable>
      <pose>-0.42 0.00 0.876 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.08</mass>
          <inertia>
            <ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><cylinder><radius>0.025</radius><length>0.05</length></cylinder></geometry>
          <surface>
            <friction>
              <ode>
                <mu>2.0</mu>
                <mu2>2.0</mu2>
              </ode>
            </friction>
            <bounce>
              <restitution_coefficient>0.0</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
```


## 10.2 ur5_debug_empty.sdf


```
<sdf version="1.10">
  <world name="ur5_debug_empty">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <gravity>0 0 -9.81</gravity>
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.1 -1</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <include>
      <uri>model://ur5_rg2</uri>
      <name>ur5_rg2</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <joint name="ur5_rg2_fixed_to_world" type="fixed">
      <parent>world</parent>
      <child>ur5_rg2::base_link</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
  </world>
</sdf>
```


# 11. MoveIt 2


## 11.1 ur5.srdf


```
<!-- Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/config/ur5.srdf -->
<!-- Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion. -->
<!-- Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot. -->
<robot name="ur5_rg2">
  <virtual_joint name="world_joint" parent_frame="world" child_link="base_link" type="fixed" />

  <group name="manipulator">
    <chain base_link="base_link" tip_link="rg2_tcp" />
    <joint name="shoulder_pan_joint" />
    <joint name="shoulder_lift_joint" />
    <joint name="elbow_joint" />
    <joint name="wrist_1_joint" />
    <joint name="wrist_2_joint" />
    <joint name="wrist_3_joint" />
  </group>

  <!-- Disable collisions between adjacent links and known fixed pairs. -->
  <disable_collisions link1="base_link_inertia" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
  <disable_collisions link1="upper_arm_link" link2="forearm_link" reason="Adjacent"/>
  <disable_collisions link1="forearm_link" link2="wrist_1_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_1_link" link2="wrist_2_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_3_link" link2="tool0" reason="Adjacent"/>
  <disable_collisions link1="tool0" link2="rg2_tcp" reason="Adjacent"/>
  <disable_collisions link1="tool0" link2="rg2_base_link" reason="Adjacent"/>
  <disable_collisions link1="rg2_base_link" link2="rg2_finger_link1" reason="Adjacent"/>
  <disable_collisions link1="rg2_base_link" link2="rg2_finger_link2" reason="Adjacent"/>

  <!-- Non-adjacent pairs that never collide (standard UR5 geometry/joint limits). -->
  <disable_collisions link1="base_link_inertia" link2="forearm_link" reason="Never"/>
  <disable_collisions link1="base_link_inertia" link2="upper_arm_link" reason="Never"/>
  <disable_collisions link1="base_link_inertia" link2="wrist_1_link" reason="Never"/>
  <disable_collisions link1="base_link_inertia" link2="wrist_2_link" reason="Never"/>
  <disable_collisions link1="forearm_link" link2="wrist_2_link" reason="Never"/>
  <disable_collisions link1="forearm_link" link2="wrist_3_link" reason="Never"/>
  <disable_collisions link1="shoulder_link" link2="forearm_link" reason="Never"/>
  <disable_collisions link1="shoulder_link" link2="wrist_1_link" reason="Never"/>
  <disable_collisions link1="shoulder_link" link2="wrist_2_link" reason="Never"/>
  <disable_collisions link1="upper_arm_link" link2="wrist_1_link" reason="Never"/>
  <disable_collisions link1="upper_arm_link" link2="wrist_2_link" reason="Never"/>
  <disable_collisions link1="upper_arm_link" link2="wrist_3_link" reason="Never"/>

  <!-- RG2 gripper pairs that never collide. -->
  <disable_collisions link1="rg2_finger_link1" link2="rg2_finger_link2" reason="Never"/>
  <disable_collisions link1="wrist_3_link" link2="rg2_base_link" reason="Never"/>
  <disable_collisions link1="wrist_3_link" link2="rg2_tcp" reason="Never"/>
  <disable_collisions link1="wrist_2_link" link2="rg2_base_link" reason="Never"/>
  <disable_collisions link1="wrist_2_link" link2="tool0" reason="Never"/>
  <disable_collisions link1="wrist_1_link" link2="wrist_3_link" reason="Never"/>
</robot>
```


## 11.2 ur5_strict.srdf


```
<!-- Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/config/ur5_strict.srdf -->
<!-- Contenido: Variante estricta de self-collision para MoveIt del UR5. -->
<!-- Uso breve: Se carga cuando STRICT_SELF_COLLISION=1 o strict_physics_mode:=true. -->
<robot name="ur5_rg2">
  <virtual_joint name="world_joint" parent_frame="world" child_link="base_link" type="fixed" />

  <group name="manipulator">
    <chain base_link="base_link" tip_link="rg2_tcp" />
    <joint name="shoulder_pan_joint" />
    <joint name="shoulder_lift_joint" />
    <joint name="elbow_joint" />
    <joint name="wrist_1_joint" />
    <joint name="wrist_2_joint" />
    <joint name="wrist_3_joint" />
  </group>

  <!-- Solo pares adyacentes o fijos inevitables por diseño del útil. -->
  <disable_collisions link1="base_link_inertia" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
  <disable_collisions link1="upper_arm_link" link2="forearm_link" reason="Adjacent"/>
  <disable_collisions link1="forearm_link" link2="wrist_1_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_1_link" link2="wrist_2_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_3_link" link2="tool0" reason="Adjacent"/>
  <disable_collisions link1="tool0" link2="rg2_tcp" reason="Adjacent"/>
  <disable_collisions link1="tool0" link2="rg2_base_link" reason="Adjacent"/>
  <disable_collisions link1="rg2_base_link" link2="rg2_finger_link1" reason="Adjacent"/>
  <disable_collisions link1="rg2_base_link" link2="rg2_finger_link2" reason="Adjacent"/>

  <!-- Pares del útil/gripper que permanecen fijos o geométricamente inevitables. -->
  <disable_collisions link1="rg2_finger_link1" link2="rg2_finger_link2" reason="Never"/>
  <disable_collisions link1="wrist_3_link" link2="rg2_base_link" reason="Never"/>
  <disable_collisions link1="wrist_3_link" link2="rg2_tcp" reason="Never"/>
  <disable_collisions link1="wrist_2_link" link2="rg2_base_link" reason="Never"/>
  <disable_collisions link1="wrist_2_link" link2="tool0" reason="Never"/>
</robot>```


## 11.3 kinematics.yaml


```
# Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/config/kinematics.yaml
# Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion.
# Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot.
#
# IK solver evaluation (FASE 8, 2026-04-26):
#   - trac_ik: NOT available in ROS Jazzy binary install
#   - KDL (kdl_kinematics_plugin): available, works correctly
#   - CachedKDL (cached_ik_kinematics_plugin): available, wraps KDL with a
#     pose/config lookup table. For pick & place cycles with repeated poses
#     (approach, grasp, lift, carry) this reduces IK time on cache hits
#     without changing the solver numerics. min_pose_distance=0.005 (5mm)
#     keeps the cache conservative to avoid returning wrong-branch solutions.
#
# To revert to plain KDL: change solver to kdl_kinematics_plugin/KDLKinematicsPlugin
# and remove the cached_ik_kinematics block.
manipulator:
  kinematics_solver: cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin
  kinematics_solver_timeout: 0.2
  kinematics_solver_search_resolution: 0.005
  cached_ik_kinematics:
    max_cache_size: 5000
    min_pose_distance: 0.005
    min_joint_config_distance: 0.05
    cached_ik_path: ""
```


## 11.4 joint_limits.yaml


```
# Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/config/joint_limits.yaml
# Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion.
# Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot.
joint_limits:
  shoulder_pan_joint:
    has_acceleration_limits: true
    has_effort_limits: true
    has_position_limits: true
    has_velocity_limits: true
    max_acceleration: 1.2
    max_effort: 150.0
    max_position: 6.283185307179586
    max_velocity: 1.2
    min_position: -6.283185307179586
  shoulder_lift_joint:
    has_acceleration_limits: true
    has_effort_limits: true
    has_position_limits: true
    has_velocity_limits: true
    max_acceleration: 1.2
    max_effort: 150.0
    max_position: 6.283185307179586
    max_velocity: 1.2
    min_position: -6.283185307179586
  elbow_joint:
    has_acceleration_limits: true
    has_effort_limits: true
    has_position_limits: true
    has_velocity_limits: true
    max_acceleration: 1.2
    max_effort: 150.0
    max_position: 3.141592653589793
    max_velocity: 1.2
    min_position: -3.141592653589793
  wrist_1_joint:
    has_acceleration_limits: true
    has_effort_limits: true
    has_position_limits: true
    has_velocity_limits: true
    max_acceleration: 1.2
    max_effort: 28.0
    max_position: 6.283185307179586
    max_velocity: 1.6
    min_position: -6.283185307179586
  wrist_2_joint:
    has_acceleration_limits: true
    has_effort_limits: true
    has_position_limits: true
    has_velocity_limits: true
    max_acceleration: 1.2
    max_effort: 28.0
    max_position: 6.283185307179586
    max_velocity: 1.6
    min_position: -6.283185307179586
  wrist_3_joint:
    has_acceleration_limits: true
    has_effort_limits: true
    has_position_limits: true
    has_velocity_limits: true
    max_acceleration: 1.2
    max_effort: 28.0
    max_position: 6.283185307179586
    max_velocity: 1.6
    min_position: -6.283185307179586
  rg2_finger_joint1:
    has_acceleration_limits: false
    has_effort_limits: true
    has_position_limits: true
    has_velocity_limits: true
    max_effort: 20.0
    max_position: 0.0425
    max_velocity: 0.2
    min_position: 0.0
  rg2_finger_joint2:
    has_acceleration_limits: false
    has_effort_limits: true
    has_position_limits: true
    has_velocity_limits: true
    max_effort: 20.0
    max_position: 0.0425
    max_velocity: 0.2
    min_position: 0.0
```


## 11.5 moveit_controllers.yaml


```
# Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/config/moveit_controllers.yaml
# Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion.
# Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot.
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller
  joint_trajectory_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
```


## 11.6 ompl_planning.yaml


```
# Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/config/ompl_planning.yaml
# Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion.
# Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot.
planning_plugins:
  - ompl_interface/OMPLPlanner
request_adapters:
  - default_planning_request_adapters/ResolveConstraintFrames
  - default_planning_request_adapters/ValidateWorkspaceBounds
  - default_planning_request_adapters/CheckStartStateBounds
  - default_planning_request_adapters/CheckStartStateCollision
response_adapters:
  - default_planning_response_adapters/AddTimeOptimalParameterization
  - default_planning_response_adapters/ValidateSolution
  - default_planning_response_adapters/DisplayMotionPath
start_state_max_bounds_error: 0.1
use_different_state_validity_filter: false
planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0
    goal_joint_tolerance: 0.001
    goal_position_tolerance: 0.001
    goal_orientation_tolerance: 0.001
manipulator:
  planner_configs:
    - RRTConnectkConfigDefault
```


## 11.7 planning_scene_monitor_parameters.yaml


```
# Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/config/planning_scene_monitor_parameters.yaml
# Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion.
# Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot.
planning_scene_monitor:
  publish_planning_scene: true
  publish_geometry_updates: true
  publish_state_updates: true
  publish_transforms_updates: true
```


## 11.8 ur5_moveit_bridge.py — funciones principales


```
101:class UR5MoveItBridge(Node):
104:    def _log_bridge_status(self, message: str, *, level: str = "info") -> None:
110:    def __init__(self) -> None:
524:    def _publish_heartbeat(self) -> None:
536:    def _update_sim_wall_rate_estimate(self) -> None:
559:    def _estimated_sim_seconds_per_wall_second(self) -> float:
569:    def _timeout_sim_seconds_per_wall_second(
601:    def _fjt_timeout_for_trajectory(
644:    def _effective_request_timeout_sec(self) -> float:
672:    def _joint_state_cb(self, msg: JointState) -> None:
722:    def _normalize_joint_position(joint_name: str, value: float) -> float:
727:    def _is_skip_constraints_request(self, request_uuid: str) -> bool:
731:    def _build_joint_path_constraints(
816:    def _build_start_robot_state(self) -> tuple[Any | None, str]:
875:    def _set_planning_start_state_from_joint_state(self) -> tuple[bool, str]:
892:    def _build_start_robot_state_msg(self) -> tuple[MoveItRobotStateMsg | None, str]:
927:    def _configure_move_group_scaling(self, group: Any) -> None:
941:    def _joint_state_ready_status(self) -> tuple[bool, str]:
961:    def _wait_for_valid_joint_state(self, timeout_sec: float) -> tuple[bool, str]:
973:    def _current_arm_joint_vector(self) -> tuple[list[float], str] | tuple[None, str]:
994:    def _wait_for_joint_state_settled(
1049:    def _env_float(name: str, default: float) -> float:
1052:    def _available_action_names(self) -> list[str]:
1060:    def _normalize_action_name(name: str) -> str:
1063:    def _destroy_fjt_action_client(self) -> None:
1075:    def _ensure_fjt_action_client(self) -> ActionClient | None:
1102:    def _prime_fjt_action_client(self) -> None:
1118:    def _controller_manager_service_name(self, service: str) -> str:
1127:    def _controller_action_candidates(self, action_names: list[str]) -> list[str]:
1161:    def _active_controllers(self, timeout_sec: float = 0.8) -> list[str]:
1189:    def _wait_for_expected_controller_action(
1213:    def _topic_endpoint_counts(self, topic: str) -> tuple[int, int]:
1226:    def _exec_diagnostics(self) -> dict[str, Any]:
1279:    def _diag_to_message(diag: dict[str, Any]) -> str:
1283:    def _describe_execute_result(result: Any) -> dict[str, Any]:
1287:    def _plan_success_code(success: Any) -> int | None:
1291:    def _plan_success_ok(cls, success: Any) -> bool:
1295:    def _plan_error_code_val(plan) -> int | None:
1299:    def _result_meta_to_message(meta: dict[str, Any]) -> str:
1303:    def _goal_status_text(status: int) -> str:
1307:    def _wait_future_done(future, timeout_sec: float) -> bool:
1310:    def _execute_moveit_py_with_timeout(self, trajectory) -> tuple[bool, Any, str]:
1348:    def _joint_goal_reached(self, jt: JointTrajectory, tol_rad: float = 0.08) -> tuple[bool, str]:
1390:    def _wait_joint_goal_reached(
1407:    def _joint_motion_since_vector(
1432:    def _ee_target_reached(
1481:    def _wait_ee_target_reached(
1498:    def _joint_goal_success_consistent_with_ee(
1524:    def _planned_ee_pose_from_joint_trajectory(
1626:    def _planned_trajectory_target_consistent(
1674:    def _feedback_goal_reached(
1733:    def _execute_joint_trajectory_action(
3085:    def _joint_trajectory_initial_segment_max_delta(jt: JointTrajectory) -> float:
3088:    def _prepare_joint_trajectory_for_controller(
3423:    def _extract_joint_trajectory_msg(self, trajectory) -> JointTrajectory | None:
3478:    def _scale_joint_trajectory_timing(jt: JointTrajectory, scale: float = 2.0) -> JointTrajectory:
3482:    def _joint_trajectory_duration_sec(jt: JointTrajectory | None) -> float:
3486:    def _qos_summary(qos: QoSProfile) -> str:
3493:    def _parse_request_meta(frame_raw: str) -> tuple[str, int | None, str, dict[str, Any]]:
3496:    def _pose_callback(self, msg: PoseStamped, cartesian: bool = False, topic_name: str = "") -> None:
3687:    def _ensure_base_frame(self, msg: PoseStamped) -> PoseStamped | None:
3721:    def _poll_tf_ready(self) -> None:
3736:    def _dispatch_plan_request(
3755:    def _dispatch_plan_request_with_timeout(
3883:    def _plan_worker(self) -> None:
4095:    def _publish_result(
4175:    def _pose_to_matrix(pose: "Pose") -> "np.ndarray":
4179:    def _matrix_to_pose(T: "np.ndarray") -> "Any":
4182:    def _robot_state_frame_transform_matrix(
4199:    def _compute_approach_ik_seeded(
4510:    def _plan_with_moveit_py(
5073:    def _init_moveit_py(self) -> None:
5220:    def _strip_qos_overrides(data: object) -> object:
5234:    def _plan_with_moveit_commander(self, target: PoseStamped) -> tuple[bool, str, bool, bool]:
5321:    def _get_cartesian_group(self) -> MoveGroupCommander | None:
5337:    def _plan_cartesian(self, target: PoseStamped) -> tuple[bool, str, bool, bool]:
5472:    def _extract_trajectory(plan) -> RobotTrajectory | None:
5488:    def _publish_planned_joint_trajectory(self, trajectory) -> bool:
5517:    def _load_controller_contract(self, controllers_path: Path) -> None:
5551:    def shutdown(self) -> None:
```


## 11.9 moveit_bridge_utils.py — helpers


```
31:def bridge_env_float(name: str, default: float) -> float:
46:def normalize_action_name(name: str) -> str:
56:def parse_request_meta(frame_raw: str) -> tuple[str, Optional[int], str, dict[str, Any]]:
95:def plan_success_code(success: Any) -> Optional[int]:
112:def plan_success_ok(success: Any) -> bool:
130:def plan_error_code_val(plan: Any) -> Optional[int]:
148:def describe_execute_result(result: Any) -> dict[str, Any]:
163:def result_meta_to_message(meta: dict[str, Any]) -> str:
195:def diag_to_message(diag: dict[str, Any]) -> str:
214:def goal_status_text(status: int) -> str:
238:def wait_future_done(future: Any, timeout_sec: float) -> bool:
252:def joint_trajectory_duration_sec(jt: Any) -> float:
268:def joint_trajectory_initial_segment_max_delta(jt: Any) -> float:
283:def scale_joint_trajectory_timing(jt: Any, scale: float = 2.0) -> Any:
310:def pose_to_matrix(pose: Any) -> "np.ndarray":
327:def matrix_to_pose(T: "np.ndarray") -> Any:
371:def stamp_age_sec(stamp_sec: float, now_sec: float) -> float:
376:def is_stamp_fresh(stamp_sec: float, now_sec: float, max_age_sec: float) -> bool:
```


# 12. ros2_control y Gazebo


## 12.1 ur5_controllers.yaml (descripción)


```
# Ruta/archivo: agarre_ros2_ws/src/ur5_description/config/ur5_controllers.yaml
# Contenido: Descripcion URDF/Xacro y configuracion base del robot UR5.
# Uso breve: Se usa al publicar el robot model y al preparar simulacion y MoveIt.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description/config/ur5_controllers.yaml
# Summary: Controller configuration for UR5 and RG2 gripper.
controller_manager:
  ros__parameters:
    update_rate: 125

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: forward_command_controller/ForwardCommandController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    interpolate_from_desired_state: false
    set_last_command_interface_value_as_state_on_activation: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
      shoulder_pan_joint:
        trajectory: 3.4
        goal: 0.08
      shoulder_lift_joint:
        trajectory: 3.4
        goal: 0.08
      elbow_joint:
        trajectory: 3.4
        goal: 0.08
      wrist_1_joint:
        trajectory: 3.4
        goal: 0.08
      wrist_2_joint:
        trajectory: 3.4
        goal: 0.08
      wrist_3_joint:
        trajectory: 3.4
        goal: 0.08


gripper_controller:
  ros__parameters:
    joints:
      - rg2_finger_joint1
      - rg2_finger_joint2
    interface_name: position
```


## 12.2 ur5_mock_controllers.yaml


```
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml
# Contenido: Configuracion de bringup ROS 2 para lanzar el sistema UR5.
# Uso breve: Colcon/ros2 launch lo usan para arrancar simulacion y componentes principales.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml
# Summary: Mock controller configuration for UR5 ros2_control.
controller_manager:
  ros__parameters:
    update_rate: 125

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    gripper_controller:
      type: forward_command_controller/ForwardCommandController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

gripper_controller:
  ros__parameters:
    joints:
      - rg2_finger_joint1
      - rg2_finger_joint2
    interface_name: position
```


## 12.3 Bridge YAML base


```
# Ruta/archivo: agarre_ros2_ws/scripts/bridge_cameras.yaml
# Contenido: Mapping base ros_gz_bridge: cámeras, objetos y control Gazebo↔ROS 2.
# Uso breve: patch_bridge_yaml() copia y sustituye el world_name en runtime.
#
# AUDITORÍA DE BRIDGES (FASE 8, 2026-04-26) — 78 bridges totales
# ─────────────────────────────────────────────────────────────────
# Categoría              Cantidad   Estado        Motivo
# ─────────────────────────────────────────────────────────────────
# /clock                 1          NECESARIO     sim_time para todos los nodos
# /world/.../pose/info   2          NECESARIO     gz_pose_bridge + panel PoseArray
# Cámara RGB             6          NECESARIO     panel multi-vista + TFM overhead
# Cámara depth           4          ACEPTABLE     inferencia de agarre (overhead+debug)
# Cámara muñeca          1          RESERVADO     futura integración wrist-cam
# drop_anchor ×10 obj    30         NECESARIO     release_objects_service (drop)
# gripper_anchor ×10+1   33         NECESARIO     gripper_attach_backend (attach)
# ─────────────────────────────────────────────────────────────────
# Conclusión: todos los bridges están justificados. Los de cámara son lazy
# (ros_gz_bridge no transmite si no hay suscriptor activo). El coste real
# dominante es /clock + /pose/info (siempre activos, ~1 kHz cada uno).
# No se eliminan bridges — remover uno rompe la suscripción del panel si
# el topic existe en el SDF pero no en el YAML.

- ros_topic_name: /clock
  gz_topic_name: /world/ur5_mesa_objetos/clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS

- ros_topic_name: /world/ur5_mesa_objetos/pose/info_raw
  gz_topic_name: /world/ur5_mesa_objetos/pose/info
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS

- ros_topic_name: /world/ur5_mesa_objetos/pose/info_array
  gz_topic_name: /world/ur5_mesa_objetos/pose/info
  ros_type_name: geometry_msgs/msg/PoseArray
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS

- ros_topic_name: /camera_overhead/image
  gz_topic_name: /camera_overhead/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS

- ros_topic_name: /camera_north/image
  gz_topic_name: /camera_north/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS

- ros_topic_name: /camera_south/image
  gz_topic_name: /camera_south/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS

- ros_topic_name: /camera_east/image
  gz_topic_name: /camera_east/image
```


## 12.4 gz_pose_bridge.py


```
23:class GzPoseBridge(Node):
197:def main() -> None:
```


## 12.5 gz_ros_control_guard.py


```
23:class GzRosControlGuard(Node):
132:def main(args=None) -> None:
```


## 12.6 planning_scene_sync.py


```
51:class PrimitiveSpec:
58:class ModelGeometry:
66:class PoseSample:
77:def _strip_ns(tag: str) -> str:
83:def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
98:def _quat_multiply(
112:def _quat_conjugate(quat: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
117:def _rotate_vector(
127:def _compose_pose(
138:def _parse_pose_text(text: str) -> Tuple[float, float, float, float, float, float, float]:
150:def _tuple_to_pose(values: Tuple[float, float, float, float, float, float, float]) -> Pose:
162:def _world_file_default(world_name: str) -> str:
167:class PlanningSceneSync(Node):
632:def main() -> None:
```


# 13. Panel Qt — Arquitectura Modular


## 13.1 Módulos ur5_qt_panel (post-refactoring)


```
src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py
src/ur5_qt_panel/ur5_qt_panel/calibration_service.py
src/ur5_qt_panel/ur5_qt_panel/cameras_tab.py
src/ur5_qt_panel/ur5_qt_panel/directo_gate_evaluator.py
src/ur5_qt_panel/ur5_qt_panel/directo_geometry.py
src/ur5_qt_panel/ur5_qt_panel/__init__.py
src/ur5_qt_panel/ur5_qt_panel/logging_utils.py
src/ur5_qt_panel/ur5_qt_panel/main_panel.py
src/ur5_qt_panel/ur5_qt_panel/panel_calib_actions.py
src/ur5_qt_panel/ur5_qt_panel/panel_calibration.py
src/ur5_qt_panel/ur5_qt_panel/panel_calib_selection.py
src/ur5_qt_panel/ur5_qt_panel/panel_camera_controllers.py
src/ur5_qt_panel/ur5_qt_panel/panel_camera_helpers.py
src/ur5_qt_panel/ur5_qt_panel/panel_camera.py
src/ur5_qt_panel/ur5_qt_panel/panel_config.py
src/ur5_qt_panel/ur5_qt_panel/panel_controllers.py
src/ur5_qt_panel/ur5_qt_panel/panel_direct2.py
src/ur5_qt_panel/ur5_qt_panel/panel_draw_overlays.py
src/ur5_qt_panel/ur5_qt_panel/panel_env.py
src/ur5_qt_panel/ur5_qt_panel/panel_external_state.py
src/ur5_qt_panel/ur5_qt_panel/panel_fatal.py
src/ur5_qt_panel/ur5_qt_panel/panel_gz_objects.py
src/ur5_qt_panel/ur5_qt_panel/panel_gz_startup.py
src/ur5_qt_panel/ur5_qt_panel/panel_helpers.py
src/ur5_qt_panel/ur5_qt_panel/panel_launch_control.py
src/ur5_qt_panel/ur5_qt_panel/panel_launchers.py
src/ur5_qt_panel/ur5_qt_panel/panel_main_ui.py
src/ur5_qt_panel/ur5_qt_panel/panel_motion_control.py
src/ur5_qt_panel/ur5_qt_panel/panel_motion_helpers.py
src/ur5_qt_panel/ur5_qt_panel/panel_moveit_flow.py
src/ur5_qt_panel/ur5_qt_panel/panel_moveit_publishers.py
src/ur5_qt_panel/ur5_qt_panel/panel_moveit_ready.py
src/ur5_qt_panel/ur5_qt_panel/panel_moveit_wait.py
src/ur5_qt_panel/ur5_qt_panel/panel_object_mgmt.py
src/ur5_qt_panel/ur5_qt_panel/panel_objects.py
src/ur5_qt_panel/ur5_qt_panel/panel_physics.py
src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py
src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py
src/ur5_qt_panel/ur5_qt_panel/panel_process.py
src/ur5_qt_panel/ur5_qt_panel/panel_readiness.py
src/ur5_qt_panel/ur5_qt_panel/panel_remote_callbacks.py
src/ur5_qt_panel/ur5_qt_panel/panel_robot_presets.py
src/ur5_qt_panel/ur5_qt_panel/panel_ros_publishers.py
src/ur5_qt_panel/ur5_qt_panel/panel_ros.py
src/ur5_qt_panel/ur5_qt_panel/panel_runtime_pose_auditor.py
src/ur5_qt_panel/ur5_qt_panel/panel_safety.py
src/ur5_qt_panel/ur5_qt_panel/panel_settings.py
src/ur5_qt_panel/ur5_qt_panel/panel_shutdown.py
src/ur5_qt_panel/ur5_qt_panel/panel_startup.py
src/ur5_qt_panel/ur5_qt_panel/panel_state_machine.py
src/ur5_qt_panel/ur5_qt_panel/panel_state_methods.py
src/ur5_qt_panel/ur5_qt_panel/panel_state.py
src/ur5_qt_panel/ur5_qt_panel/panel_status_mgmt.py
src/ur5_qt_panel/ur5_qt_panel/panel_step_callbacks.py
src/ur5_qt_panel/ur5_qt_panel/panel_step_ui.py
src/ur5_qt_panel/ur5_qt_panel/panel_tf_diagnose.py
src/ur5_qt_panel/ur5_qt_panel/panel_tf_monitor.py
src/ur5_qt_panel/ur5_qt_panel/panel_tfm.py
src/ur5_qt_panel/ur5_qt_panel/panel_tfm_science.py
src/ur5_qt_panel/ur5_qt_panel/panel_tf.py
src/ur5_qt_panel/ur5_qt_panel/panel_trace_callbacks.py
src/ur5_qt_panel/ur5_qt_panel/panel_trace_ui.py
src/ur5_qt_panel/ur5_qt_panel/panel_ui_state.py
src/ur5_qt_panel/ur5_qt_panel/panel_utils.py
src/ur5_qt_panel/ur5_qt_panel/panel_v2.py
src/ur5_qt_panel/ur5_qt_panel/panel_watchdog.py
src/ur5_qt_panel/ur5_qt_panel/panel_workers.py
src/ur5_qt_panel/ur5_qt_panel/step_pipeline_helpers.py
src/ur5_qt_panel/ur5_qt_panel/tf_pose_utils.py
src/ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py

Total módulos: 71
```


## 13.2 panel_v2.py — callbacks registrados


```
671:        self.signal_status.connect(self._set_status_async)
672:        self.signal_refresh_controls.connect(self._refresh_controls)
673:        self.signal_set_led.connect(self._set_led_async)
674:        self.signal_update_objects.connect(self._update_objects)
675:        self.signal_start_objects_settle_watch.connect(self._start_objects_settle_watch)
676:        self.signal_handle_objects_settled.connect(self._handle_objects_settled)
677:        self.signal_schedule_camera_health_check.connect(self._camera_ctrl.schedule_health_check)
678:        self.signal_update_camera_topics.connect(self._camera_ctrl.update_topics_async)
679:        self.signal_connect_camera.connect(self._camera_ctrl.connect)
680:        self.signal_request_auto_bridge_start.connect(self._request_auto_bridge_start)
681:        self.signal_bridge_ready.connect(self._on_bridge_ready)
682:        self.signal_calibration_check.connect(self._on_calibration_check)
683:        self.signal_trace_ready.connect(self._on_trace_ready)
684:        self.signal_schedule_home_offset.connect(self._schedule_home_offset_retry)
685:        self.signal_close_panel.connect(self.close)
699:        self.signal_tf_ready.connect(self._on_tf_ready_signal)
700:        self.signal_calib_ready.connect(self._on_calib_ready_signal)
701:        self.signal_controllers_ready.connect(self._on_controllers_ready_signal)
702:        self.signal_error.connect(self._on_error_signal)
703:        self.signal_moveit_state.connect(self._on_moveit_state_signal)
704:        self.signal_run_ui.connect(self._run_ui_callable)
705:        self.signal_run_ui_delayed.connect(self._run_ui_delayed)
708:        self._watchdog_timer.timeout.connect(self._check_critical_timeouts)
1060:        self.runner.line.connect(lambda msg: self._log(msg))
1081:        self.ros_worker.image.connect(self._camera_ctrl.on_image)
1082:        self.ros_worker.joint_state.connect(self._on_joint_state)
1083:        self.ros_worker.grasp_rect.connect(self._on_grasp_rect)
1084:        self.ros_worker.log.connect(self._log_ros_message)
1085:        self.ros_worker.system_state.connect(self._on_system_state_update)
1086:        self.ros_worker.camera_connect_request.connect(self._on_remote_camera_connect_request)
1087:        self.ros_worker.camera_disconnect_request.connect(self._on_remote_camera_disconnect_request)
1088:        self.ros_worker.recover_request.connect(self._on_remote_recover_request)
1089:        self.ros_worker.tfm_infer_request.connect(self._on_remote_tfm_infer_request)
1090:        self.ros_worker.tfm_execute_request.connect(self._on_remote_tfm_execute_request)
1091:        self.ros_worker.pick_demo_request.connect(self._on_remote_pick_demo_request)
1092:        self.ros_worker.pick_object_request.connect(self._on_remote_pick_object_request)
1093:        self.ros_worker.object_select_request.connect(self._on_remote_object_select_request)
1102:        self.retry_send_joints.connect(self._send_joints_retry)
1114:        self.objects_timer.timeout.connect(self._update_objects)
1121:        self.joint_timer.timeout.connect(self._auto_subscribe_joints)
1126:        self._drop_hold_timer.timeout.connect(self._drop_hold_tick)
1367:    def _on_tf_ready_signal(self, *args, **kwargs):
1370:    def _on_calib_ready_signal(self, *args, **kwargs):
1376:    def _on_controllers_ready_signal(self, *args, **kwargs):
1379:    def _on_error_signal(self, *args, **kwargs):
1382:    def _on_moveit_state_signal(self, *args, **kwargs):
1385:    def _on_trace_ready(self, *args, **kwargs):
1388:    def _on_calibration_check(self, *args, **kwargs):
1579:    def _on_async_error(self, *args, **kwargs):
1591:    def _on_step_mode_combo_changed(self, *args, **kwargs):
1633:    def _on_step_continue_clicked(self, *args, **kwargs):
1636:    def _on_step_phase_start_clicked(self, *args, **kwargs):
1639:    def _on_step_window_finished(self, *args, **kwargs):
1795:    def _on_debug_motion_button(self, *args, **kwargs):
1807:    def _on_system_state_update(self, *args, **kwargs):
1980:    def _on_bridge_ready(self, *args, **kwargs):
1983:    def _on_image(self, *args, **kwargs):
1986:    def _on_grasp_rect(self, *args, **kwargs):
1998:    def _on_joint_state(self, *args, **kwargs):
2028:    def _on_start_fatal(self, *args, **kwargs):
2389:    def _on_tfm_grasp_object_clicked(self, *args, **kwargs):
2395:    def _on_remote_camera_connect_request(self, source: str) -> None:
2398:    def _on_remote_camera_disconnect_request(self, source: str) -> None:
2401:    def _on_remote_recover_request(self, source: str) -> None:
2404:    def _on_remote_tfm_infer_request(self, source: str) -> None:
2407:    def _on_remote_tfm_execute_request(self, source: str) -> None:
2410:    def _on_remote_pick_demo_request(self, source: str) -> None:
2413:    def _on_remote_pick_object_request(self, source: str) -> None:
2416:    def _on_remote_object_select_request(self, name: str, source: str) -> None:
2446:    def _on_slider_change(self, *args, **kwargs):
2491:    def _on_camera_click(self, *args, **kwargs):
2599:    def _on_object_clicked(self, *args, **kwargs):
2629:    def _on_tfm_repro_mode_changed(self, *args, **kwargs):
2632:    def _on_tfm_postprocess_mode_changed(self, *args, **kwargs):
2635:    def _on_tfm_checkpoint_selection_changed(self, *args, **kwargs):
```


## 13.3 panel_pick_demo.py — funciones principales


```
573:    def _dist3(a, b):
806:    def _fresh_demo_world() -> tuple[float, float, float] | None:
1252:    def _set_pick_demo_result(success: bool, reason: str, *, executed: bool) -> None:
1264:    def _sync_gate_emit(stage: str, ok: bool, detail: str = "") -> None:
1490:    def worker():
```


## 13.4 panel_pick_object.py — funciones principales


```
81:    def _dbg(msg: str) -> None:
85:    def _moveit2_log(scope: str, msg: str) -> None:
90:    def _step_phase_gate(phase: str, *, position=None, decision: str = "", object_position=None) -> None:
124:    def _quat_multiply(
137:    def _pick_orientation(yaw_deg: Optional[float] = None) -> tuple[float, float, float, float]:
169:    def _block(reason: str, *, status_text: Optional[str] = None, error: bool = False) -> None:
192:    def _clear_stale_ui_selection(reason: str) -> None:
216:    def _canonical_phase(state: str, detail: str = "") -> None:
224:    def _canonical_finish(success: bool, message: str, final_state: str) -> None:
318:    def _world_ready_scope() -> str:
324:    def _world_ready_tracked_names() -> list[str]:
339:    def _world_ready_pending() -> list[str]:
360:    def _wait_for_world_ready() -> bool:
1130:    def worker():
```


## 13.5 panel_pick_geometry.py


```
12:class PickHeightProfile:
24:def compute_pick_height_profile(
```


## 13.6 panel_config.py — constantes de fase


```
#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_config.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Configuración compartida y constantes globales del panel SUPER PRO."""
import os
import sys
from typing import Dict, List, Optional, Tuple, Set

from ur5_tools.gripper_geometry import RG2_PINCH_CENTER_FRAME, contact_z_correction_for_frame

from .panel_settings import PanelSettings

# Disable FastDDS SHM early to avoid noisy startup errors.
os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")

SETTINGS = PanelSettings.from_env()

WS_DIR = SETTINGS.ws_dir
SCRIPTS_DIR = SETTINGS.scripts_dir
WORLDS_DIR = SETTINGS.worlds_dir
MODELS_DIR = SETTINGS.models_dir
LOG_DIR = SETTINGS.log_dir
BAGS_DIR = SETTINGS.bags_dir
FIG_DIR = SETTINGS.fig_dir
VISION_DIR = SETTINGS.vision_dir
VISION_EXP_DIR = SETTINGS.vision_exp_dir
VISION_PLOTS_DIR = SETTINGS.vision_plots_dir
VISION_SUMMARY = SETTINGS.vision_summary
VISION_FIG_DIR = SETTINGS.vision_fig_dir

TABLE_SIZE_X = SETTINGS.table_size_x
TABLE_SIZE_Y = SETTINGS.table_size_y
TABLE_CENTER_X = SETTINGS.table_center_x
TABLE_CENTER_Y = SETTINGS.table_center_y
TABLE_IMAGE_SWAP_XY = SETTINGS.table_image_swap_xy
TABLE_IMAGE_FLIP_X = SETTINGS.table_image_flip_x
TABLE_IMAGE_FLIP_Y = SETTINGS.table_image_flip_y
TABLE_CALIB_PATH = os.path.join(SCRIPTS_DIR, "table_pixel_map.json")
TABLE_PIXEL_AFFINE: Optional[List[List[float]]] = None
TABLE_PIXEL_RECT: Optional[Dict[str, Tuple[float, float]]] = None
TABLE_PIXEL_HOMOGRAPHY: Optional[List[List[float]]] = None
TABLE_CAM_INFO: Optional[Dict[str, object]] = None
TABLE_OBJECT_XY_MARGIN = SETTINGS.table_object_xy_margin
TABLE_OBJECT_Z_MIN = SETTINGS.table_object_z_min
TABLE_OBJECT_Z_MAX = SETTINGS.table_object_z_max
TABLE_TOP_Z = SETTINGS.reach_overlay_z
TABLE_OBJECT_WHITELIST: Optional[List[str]] = SETTINGS.table_object_whitelist
NUDGE_DROP_OBJECTS = SETTINGS.nudge_drop_objects
NUDGE_DROP_DZ = SETTINGS.nudge_drop_dz
NUDGE_DROP_Z_MIN = SETTINGS.nudge_drop_z_min
SELECTION_SNAP_DIST = SETTINGS.selection_snap_dist
OBJECT_POS_PATH = SETTINGS.object_pos_path
SAVE_POSE_INFO_POSITIONS = SETTINGS.save_pose_info_positions
UR5_BASE_X = SETTINGS.ur5_base_x
UR5_BASE_Y = SETTINGS.ur5_base_y
UR5_BASE_Z = SETTINGS.ur5_base_z
UR5_REACH_RADIUS = SETTINGS.ur5_reach_radius

# Pick demo objects: objetos de demostracion (definidos solo por nombre).
PICK_DEMO_SPAWN_POSE = (-0.42, 0.00, 0.875)
_PICK_DEMO_OBJECTS = {
    "pick_demo": PICK_DEMO_SPAWN_POSE,
}
ATTACHABLE_OBJECTS = ("pick_demo",)
PICK_DEMO_NAME_SET = set(ATTACHABLE_OBJECTS)
# Drop objects: racimo compacto suspendido a ~2 m hasta pulsar "Soltar objetos".
_DROP_AIR_OBJECTS = {
    "box_red": (-0.260, 0.100, 2.000),
    "box_blue": (-0.180, 0.100, 2.000),
    "box_green": (-0.100, 0.100, 2.000),
    "cyl_gray": (-0.020, 0.100, 2.000),
    "cyl_orange": (-0.260, 0.020, 2.000),
    "cyl_purple": (-0.180, 0.020, 2.000),
    "box_lightblue": (-0.100, 0.020, 2.000),
    "cyl_green": (-0.020, 0.020, 2.000),
    "box_yellow": (-0.220, -0.060, 2.000),
    "cross_cyan": (-0.080, -0.060, 2.000),
}
DROP_NAME_SET = {
    "box_blue",
    "box_green",
    "box_lightblue",
    "box_red",
    "box_yellow",
    "cross_cyan",
    "cyl_gray",
    "cyl_green",
    "cyl_orange",
    "cyl_purple",
}
UNKNOWN_NAME_SET: Set[str] = set()

OBJECT_POSITIONS = {**_PICK_DEMO_OBJECTS, **_DROP_AIR_OBJECTS}
DROP_OBJECTS: Dict[str, Tuple[float, float, float]] = {}
PICK_DEMO_OBJECTS: Dict[str, Tuple[float, float, float]] = {}
UNKNOWN_OBJECTS: Dict[str, Tuple[float, float, float]] = {}
DROP_OBJECT_NAMES: List[str] = []
PICK_DEMO_OBJECT_NAMES: List[str] = []
UNKNOWN_OBJECT_NAMES: List[str] = []
DYNAMIC_OBJECTS: Set[str] = set()
_UNKNOWN_WARNED: Set[str] = set()


def refresh_object_groups() -> None:
    """Recompute derived object groups from OBJECT_POSITIONS."""
    DROP_OBJECTS.clear()
    PICK_DEMO_OBJECTS.clear()
    UNKNOWN_OBJECTS.clear()
    UNKNOWN_NAME_SET.clear()
    for name, pos in OBJECT_POSITIONS.items():
        if name in DROP_NAME_SET:
            DROP_OBJECTS[name] = pos
        elif name in PICK_DEMO_NAME_SET:
            PICK_DEMO_OBJECTS[name] = pos
        else:
            UNKNOWN_OBJECTS[name] = pos
            UNKNOWN_NAME_SET.add(name)
            if name not in _UNKNOWN_WARNED:
                _UNKNOWN_WARNED.add(name)
                print(f"[OBJECTS][WARN] object_type=UNKNOWN name={name}", file=sys.stderr, flush=True)
    DROP_OBJECT_NAMES[:] = list(DROP_OBJECTS.keys())
    PICK_DEMO_OBJECT_NAMES[:] = list(PICK_DEMO_OBJECTS.keys())
    UNKNOWN_OBJECT_NAMES[:] = list(UNKNOWN_OBJECTS.keys())
    DYNAMIC_OBJECTS.clear()
    DYNAMIC_OBJECTS.update(OBJECT_POSITIONS.keys())


refresh_object_groups()
OBJECT_SHAPES = {
    "pick_demo": "circle",
    "box_red": "square",
    "box_blue": "rect_h",
    "box_green": "rect_v",
    "cyl_gray": "circle",
    "cyl_orange": "circle",
    "cyl_purple": "circle",
    "box_lightblue": "square",
    "cyl_green": "circle",
    "box_yellow": "rect_h",
    "cross_cyan": "cross",
}
OBJECT_LABELS = {
    "pick_demo": "DEM",
    "box_red": "ROJ",
    "box_blue": "AZL",
    "box_green": "VER",
    "cyl_gray": "GRI",
    "cyl_orange": "NAR",
    "cyl_purple": "MOR",
    "box_lightblue": "CEL",
    "cyl_green": "CVE",
    "box_yellow": "AMA",
    "cross_cyan": "CRZ",
}
OBJECT_COLORS = {
    "pick_demo": "#f59e0b",
    "box_red": "#d94141",
    "box_blue": "#3b82f6",
    "box_green": "#22c55e",
    "cyl_gray": "#6b7280",
    "cyl_orange": "#f59e0b",
    "cyl_purple": "#a855f7",
    "box_lightblue": "#93c5fd",
    "cyl_green": "#34d399",
    "box_yellow": "#facc15",
    "cross_cyan": "#22d3ee",
}

BASKET_DROP = (-1.30, 0.00, 0.82)
GZ_WORLD = SETTINGS.gz_world
GRIPPER_ATTACH_PREFIX = SETTINGS.gripper_attach_prefix
DROP_ANCHOR_PREFIX = "/drop_anchor"
GZ_PARTITION_FILE = os.path.join(LOG_DIR, "gz_partition.txt")

INFER_SCRIPT = SETTINGS.infer_script
INFER_CKPT = SETTINGS.infer_ckpt
INFER_ROI_SIZE = SETTINGS.infer_roi_size
INFER_RETRY_ERR_PX = SETTINGS.infer_retry_err_px
FASTRTPS_PROFILES = SETTINGS.fastrtps_profiles
UR5_CONTROLLERS_YAML = SETTINGS.ur5_controllers_yaml
UR5_JOINT_LIMITS_YAML = SETTINGS.ur5_joint_limits_yaml

BRIDGE_BASE_YAML = SETTINGS.bridge_base_yaml
EGL_VENDOR = SETTINGS.egl_vendor
AUTO_START_BRIDGE = SETTINGS.auto_start_bridge
AUTO_START_BRIDGE_DELAY_MS = SETTINGS.auto_start_bridge_delay_ms
AUTO_START_BRIDGE_MAX_RETRIES = SETTINGS.auto_start_bridge_max_retries

DEFAULT_WORLD_CANDIDATES = SETTINGS.default_world_candidates

DEBUG_FRAME_LOG = SETTINGS.debug_frame_log

BASE_FRAME = SETTINGS.base_frame
WORLD_FRAME = SETTINGS.world_frame

# FASE 1: Marco global único para operaciones del panel.
# Todas las operaciones de control/planning se realizan en base_link.
# Esto elimina dependencias de TF world->base_link y previene spam de timeouts.
GLOBAL_FRAME_EFFECTIVE = "base_link"
```


## 13.7 panel_settings.py (completo)


```
#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_settings.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Typed settings loader for the UR5 panel."""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except Exception:
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)))
    except Exception:
        return default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip() not in ("0", "false", "False", "")

def _env_str(name: str, default: str) -> str:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return str(raw)

def _env_optional_bool(name: str) -> Optional[bool]:
    raw = os.environ.get(name)
    if raw is None:
        return None
    return raw.strip().lower() not in ("0", "false", "no", "off", "")


def _warn_ignored_legacy_gripper_tcp_z_offset(raw_value: object = None, *, source: str) -> None:
    raw = raw_value
    if raw is None:
        raw = os.environ.get("PANEL_GRIPPER_TCP_Z_OFFSET")
    if raw is None or not str(raw).strip():
        return
    try:
        value = float(raw)
    except Exception:
        print(
            "[PANEL][WARN] PANEL_GRIPPER_TCP_Z_OFFSET inválida en "
            f"{source}; se ignora y se mantiene la geometría del URDF canónico.",
            file=sys.stderr,
            flush=True,
        )
        return
    print(
        "[PANEL][WARN] PANEL_GRIPPER_TCP_Z_OFFSET/gripper_tcp_z_offset es legacy. "
        f"Valor ignorado ({value:.6f}) en {source}. "
        "La fuente de verdad geométrica es el URDF canónico y el frame operativo "
        "debe ser rg2_pinch_center.",
        file=sys.stderr,
        flush=True,
    )

def _load_yaml_overrides(path: str) -> Dict[str, object]:
    if not path:
        return {}
    path = os.path.expandvars(os.path.expanduser(path))
    if not os.path.isfile(path):
        return {}
    try:
        import yaml  # type: ignore
    except Exception:
        print(f"[PANEL][WARN] PyYAML no disponible; ignorando {path}", file=sys.stderr, flush=True)
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:
        print(f"[PANEL][WARN] Error leyendo YAML {path}: {exc}", file=sys.stderr, flush=True)
        return {}
    if isinstance(data, dict) and "panel_settings" in data:
        data = data.get("panel_settings") or {}
    if not isinstance(data, dict):
        return {}
    return {str(k): v for k, v in data.items()}

@dataclass(frozen=True)
class PanelSettings:
    ws_dir: str
    scripts_dir: str
    worlds_dir: str
    models_dir: str
    log_dir: str
    bags_dir: str
    fig_dir: str
    vision_dir: str
    vision_exp_dir: str
    vision_plots_dir: str
    vision_summary: str
    vision_fig_dir: str
    table_size_x: float = 0.768
    table_size_y: float = 0.80
    table_center_x: float = -0.17
    table_center_y: float = 0.0
    table_image_swap_xy: bool = True
    table_image_flip_x: bool = True
    table_image_flip_y: bool = True
    table_object_xy_margin: float = 0.09
    table_object_z_min: float = 0.6
    table_object_z_max: float = 1.55
    table_object_whitelist: Optional[List[str]] = None
    nudge_drop_objects: bool = False
    nudge_drop_dz: float = 0.08
    nudge_drop_z_min: float = 1.6
    selection_snap_dist: float = 0.0
    object_pos_path: str = ""
    save_pose_info_positions: bool = False
    ur5_base_x: float = -0.85
    ur5_base_y: float = 0.0
    # Must match the UR5 spawn pose in the Gazebo world to keep world<->base
    # fallbacks coherent when TF is temporarily unavailable.
    ur5_base_z: float = 0.850
    ur5_reach_radius: float = 0.85
    gz_world: str = "ur5_mesa_objetos"
    gripper_attach_prefix: str = "/gripper"
    gz_partition_file: str = ""
    infer_script: str = ""
    infer_ckpt: str = ""
    infer_roi_size: int = 96
    infer_retry_err_px: float = 60.0
    fastrtps_profiles: str = ""
    ur5_controllers_yaml: str = ""
    ur5_joint_limits_yaml: str = ""
    bridge_base_yaml: str = ""
    egl_vendor: str = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    auto_start_bridge: bool = True
    auto_start_bridge_delay_ms: int = 1200
    auto_start_bridge_max_retries: int = 30
    default_world_candidates: List[str] = field(default_factory=list)
    debug_frame_log: bool = False
    base_frame: Optional[str] = None
    world_frame: Optional[str] = None
    arm_traj_topic_default: str = "/joint_trajectory_controller/joint_trajectory"
    ur5_joint_names: List[str] = field(default_factory=list)
    gripper_joint_names: List[str] = field(default_factory=list)
    ur5_home_env: str = ""
    ur5_home_default: List[float] = field(default_factory=list)
    ur5_model_name: str = "ur5_rg2"
    joint_slider_deg_min: float = -180.0
    joint_slider_deg_max: float = 180.0
    joint_slider_scale: float = 10.0
    default_joint_move_sec: float = 2.0
    depth_pctl_refresh_frames: int = 15
    depth_pctl_stride: int = 4
    depth_fast: bool = False
    debug_logs_to_stdout: bool = False
    panel_gz_gui: bool = False
    debug_joints_to_stdout: bool = False
    joint_states_topic: str = "/joint_states"
    camera_topic: str = "/camera_overhead/image"
    use_sim_time: bool = True
    skip_cleanup: bool = False
    kill_stale: bool = True
    selection_timeout_sec: float = 12.0
    traj_action_fallback: bool = True
    traj_action_fallback_delay_sec: float = 1.0
    traj_action_fallback_eps_rad: float = 0.002
    traj_action_fallback_timeout_sec: float = 2.0
    controller_ready_timeout_sec: float = 20.0
    controller_ready_cache_sec: float = 0.0
    moveit_ready_timeout_sec: float = 20.0
    gz_launch_timeout_sec: float = 20.0
    bridge_launch_timeout_sec: float = 12.0
    moveit_launch_timeout_sec: float = 25.0
    moveit_bridge_launch_timeout_sec: float = 15.0
    controller_drop_grace_sec: float = 3.0
    trace_print_period_sec: float = 3.0
    debug_poses_period_sec: float = 3.0
    pick_log_min_interval_sec: float = 2.0
    gripper_cmd_topic: str = "/gripper_controller/commands"
    # Gripper prismático: apertura máxima 0.0425 m por dedo (límite del joint en URDF/SDF).
    gripper_open_rad: float = 0.0425
    gripper_closed_rad: float = 0.0
    gripper_joint2_sign: float = 1.0
    panel_managed: bool = False
    panel_moveit_required: bool = True
    allow_unsettled_on_timeout: bool = False
    camera_ready_frames: int = 3
    camera_init_grace_sec: float = 4.0
    camera_ready_max_age_sec: float = 2.5
    camera_required: Optional[bool] = None
    camera_max_size: int = 960
```


## 13.8 attach_gate_evaluator.py


```
26:class PoseSample:
48:class AttachGateConfig:
70:class AttachGateResult:
98:def _dist3(a: Optional[Vec3], b: Optional[Vec3]) -> Optional[float]:
104:def _fmt3(v: Optional[Vec3], dec: int = 3) -> str:
110:def _fmts(v: Optional[float], dec: int = 4) -> str:
118:class AttachGateEvaluator:
```


## 13.9 directo_gate_evaluator.py


```
21:def _should_apply_global_step_timeout_extra(
33:def _coerce_ur5_joint_vector(values) -> list[float] | None:
45:def _normalize_joint_goal_near_seed(joint_goal, seed) -> list[float]:
55:def _resolve_joint_goal_normalization_seed(
72:def _normalize_joint_goal_for_execution(
87:def _build_transport_seed_candidates(
137:def _evaluate_transport_stage_preexec_model_guard(
200:def _direct_pregrasp_gate_caps(phase: str | None) -> dict[str, float] | None:
219:def _should_transport_prep_failure_jump_to_replan(
250:def _evaluate_transport_stage_postcheck(
298:def _transport_prep_failure_policy(*, strict_mode: bool) -> str:
```


## 13.10 directo_geometry.py


```
21:def angle_shortest_diff_rad(current: float, target: float) -> float:
31:def _pick_demo_tuple3(data) -> Optional[tuple[float, float, float]]:
44:def _pick_demo_fmt_scalar(value, *, digits: int = 3) -> str:
55:def _pick_demo_env_float(
72:def _pick_demo_env_int(name: str, default: int, *, minimum: int = 0) -> int:
80:def _pick_demo_env_flag(name: str, default: bool) -> bool:
94:def _effective_direct_grasp_z(source_frame: str, requested_offset_m: float) -> float:
106:def _direct_runtime_target_tol_m(label: str) -> float:
146:def pick_demo_target_semantics(phase_name: str) -> tuple[str, str]:
184:def _is_demo_basket_transport_stage(label: str) -> bool:
188:def _is_demo_basket_transport_motion(label: str) -> bool:
197:def _compute_demo_basket_targets(
223:def _compute_demo_linear_stage_targets(
247:def _compute_demo_stage_count_for_distance(
271:def _compute_demo_transport_recovery_stage_targets(
307:def _compute_demo_transport_micro_recovery_target(
342:def _compute_demo_joint_prep_waypoint(
359:def _compute_demo_joint_prep_waypoints(
428:def _compute_demo_transport_prep_joint_tol(
453:def _joint_step_wait_timeout(
```


## 13.11 ur5_kinematics.py


```
23:def _dh(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
39:def fk_ur5(q: Iterable[float]) -> Tuple[np.ndarray, np.ndarray]:
52:def ik_ur5(
88:def rot_z(theta: float) -> np.ndarray:
94:def rot_x(theta: float) -> np.ndarray:
```


## 13.12 panel_readiness.py


```
20:def _managed_ready_status(panel) -> Tuple[bool, str]:
27:def manual_control_status(panel) -> Tuple[bool, str]:
65:def camera_ready_status(panel) -> Tuple[bool, str]:
79:def pose_info_ready_status(panel) -> Tuple[bool, str]:
106:def tf_ready_status(panel) -> Tuple[bool, str]:
137:def tf_not_ready_reason(panel) -> str:
155:def camera_not_ready_reason(panel) -> str:
161:def pose_info_not_ready_reason(panel) -> str:
167:def controllers_not_ready_reason(panel) -> str:
173:def ros_node_not_ready_reason() -> str:
177:def controller_manager_not_ready_reason() -> str:
181:def list_controllers_not_ready_reason(kind: str) -> str:
185:def moveit_not_ready_reason(panel) -> str:
191:def set_moveit_wait_status(panel, label: str, reason: Optional[str] = None) -> str:
197:def moveit_control_status(panel) -> Tuple[bool, str]:
218:def pick_ui_status(panel) -> Tuple[bool, str]:
```


## 13.13 panel_state_machine.py


```
15:class StateDecision:
21:class PanelStateMachine:
```


## 13.14 step_pipeline_helpers.py


```
12:def step_present_flow_name(flow: str) -> str:
23:def step_phase_sequence(flow: str) -> List[str]:
70:def step_predict_next_phase(flow: str, phase: str) -> str:
80:def step_phase_intent(flow: str, phase: str) -> str:
130:def step_phase_action_text(flow: str, phase: str, decision: str) -> str:
168:def step_phase_gripper_state(flow: str, phase: str) -> str:
```


# 14. ur5_tools — Nodos Backend


## 14.x gripper_attach_backend


```
5:"""Backend de attach para topics del gripper con movimiento fisico en Gazebo."""
97:class PoseSample:
109:class AttachedTarget:
123:class DemoTransportState:
137:def _quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
146:def _quat_inverse(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
151:def _quat_multiply_raw(
165:def _quat_multiply(
174:def _rotate_vector(
188:def _matmul3(
201:def _matvec3(
211:def _dh_transform(
230:def _quat_from_rot3(
270:class GripperAttachBackend(Node):
285:        self.declare_parameter("pose_topic", "")
286:        self.declare_parameter("joint_states_topic", "/joint_states")
288:        self.declare_parameter("set_pose_service", "")
291:        self.declare_parameter("service_timeout_sec", 0.2)
294:        self.declare_parameter("gz_service_timeout_ms", 400)
349:        self._pose_topic = str(self.get_parameter("pose_topic").value or "").strip()
350:        self._joint_states_topic = str(
351:            self.get_parameter("joint_states_topic").value or "/joint_states"
353:        if not self._pose_topic:
354:            self._pose_topic = f"/world/{self._world_name}/pose/info"
355:        self._set_pose_service = str(
356:            self.get_parameter("set_pose_service").value or ""
364:        self._service_timeout_sec = max(
365:            0.05, float(self.get_parameter("service_timeout_sec").value or 0.2)
371:        self._gz_service_timeout_ms = max(
372:            100, int(self.get_parameter("gz_service_timeout_ms").value or 400)
458:            self._gripper_state_pubs[name] = self.create_publisher(
462:                self.create_subscription(
465:                    partial(self._on_gripper_attach, name=name, src_topic=grip_attach),
470:                self.create_subscription(
473:                    partial(self._on_gripper_detach, name=name, src_topic=grip_detach),
483:            self._drop_attach_pubs[name] = self.create_publisher(
486:            self._drop_detach_pubs[name] = self.create_publisher(
495:            self._tool_attach_pubs[name] = self.create_publisher(
498:            self._tool_detach_pubs[name] = self.create_publisher(
503:                self.create_subscription(
512:        self._pose_sub = self.create_subscription(
514:            self._pose_topic,
518:        self._joint_state_sub = self.create_subscription(
520:            self._joint_states_topic,
540:        self._gz_set_pose_service: Optional[str] = None
541:        self._gz_spawn_service: Optional[str] = None
542:        self._gz_delete_service: Optional[str] = None
576:            f"pose_topic={self._pose_topic} joint_states_topic={self._joint_states_topic} "
658:    def _gz_service_exists(self, env_prefix: str, service: str) -> bool:
659:        cmd = f"{env_prefix}gz service -s {service} -i"
671:    def _resolve_gz_service(self, candidates: List[str], env_prefix: str) -> Optional[str]:
673:            if svc and self._gz_service_exists(env_prefix, svc):
677:    def _ensure_demo_transport_services(self) -> Tuple[Optional[str], Optional[str], str]:
679:        if not self._gz_delete_service:
681:            # estable en runtime. Evitamos la pre-introspeccion: `gz service -i`
683:            self._gz_delete_service = f"/world/{self._world_name}/remove/blocking"
684:            if not self._gz_delete_service:
686:                    f"[ATTACH_BACKEND] demo_transport_missing_delete_service world={self._world_name}"
688:        if not self._gz_spawn_service:
689:            self._gz_spawn_service = f"/world/{self._world_name}/create/blocking"
690:            if not self._gz_spawn_service:
692:                    f"[ATTACH_BACKEND] demo_transport_missing_spawn_service world={self._world_name}"
694:        return self._gz_delete_service, self._gz_spawn_service, env_prefix
698:        service: str,
706:            f"{env_prefix}gz service -s {service} "
708:            f"--timeout {int(self._gz_service_timeout_ms)} --req '{req}'"
733:        service: str,
750:            f"{env_prefix}gz service -s {service} "
752:            f"--timeout {int(max(600, self._gz_service_timeout_ms))} --req '{req}'"
789:        delete_service, spawn_service, env_prefix = self._ensure_demo_transport_services()
790:        if not delete_service or not spawn_service:
792:                "[ATTACH_BACKEND] demo_transport_services_unavailable "
793:                f"object={name} delete={delete_service} spawn={spawn_service}"
798:            self._gz_delete_entity_cli(delete_service, env_prefix, name, allow_missing=True)
802:                spawn_service,
872:        delete_service, spawn_service, env_prefix = self._ensure_demo_transport_services()
873:        if not delete_service or not spawn_service:
880:        self._gz_delete_entity_cli(delete_service, env_prefix, name, allow_missing=True)
884:            spawn_service,
1007:        pub = self._gripper_state_pubs.get(name)
```


## 14.x world_tf_publisher


```
2:# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py
5:# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py
27:class WorldTfPublisher(Node):
31:        super().__init__("world_tf_publisher")
60:        self._topic = f"/world/{self._world_name}/pose/info"
61:        self._tf_pub = TransformBroadcaster(self)
65:        self._static_tf_pub = StaticTransformBroadcaster(self)
83:        self._sub = self.create_subscription(
85:            self._topic,
92:            f"WorldTfPublisher listening on {self._topic} "
347:                    f"No pose for model '{self._model_name}' yet on {self._topic}. "
466:def main(args=None) -> None:
```


## 14.x system_state_manager


```
41:class SystemState(str, Enum):
56:class DependencySnapshot:
76:class SystemInputs:
82:class SystemStateMachine:
172:class StateDecision:
178:class SystemStateManager(Node):
188:        self.declare_parameter("pose_topic", "")
189:        self.declare_parameter("camera_topic", "/camera_overhead/image")
202:            "moveit_service_names",
228:        pose_topic = read_str_param(self, "pose_topic", "")
229:        self._pose_topic = pose_topic or f"/world/{self._world_name}/pose/info"
230:        self._camera_topic = read_str_param(
231:            self, "camera_topic", "/camera_overhead/image"
241:        self._moveit_services = read_str_list_param(self, "moveit_service_names")
338:        self._state_pub = self.create_publisher(String, "/system_state", 10)
339:        self._diag_pub = self.create_publisher(String, "/system_diag", 10)
341:        self.create_subscription(
347:        self.create_subscription(
349:            self._pose_topic,
353:        if self._camera_topic:
354:            self.create_subscription(
356:                self._camera_topic,
362:                self._fatal("camera_topic vacío; cámaras obligatorias")
365:                "camera_topic vacío; cámaras deshabilitadas por configuración"
368:        self._controller_client = self.create_client(
376:            f"SystemStateManager listo: pose={self._pose_topic} camera={self._camera_topic} "
519:        if not self._controller_client.service_is_ready():
605:            services = self.get_service_names_and_types()
610:        for name, _types in services:
611:            if name in self._moveit_services:
802:        if self._controller_client.service_is_ready() and self._controllers_state:
805:        if not self._controller_client.service_is_ready():
832:def main(args=None) -> None:
```


## 14.x gripper_geometry


```
32:class FixedJointOrigin:
42:class GripperGeometry:
63:def _candidate_urdf_paths(ws_dir: Optional[str] = None) -> Iterable[Path]:
102:def _read_text(path: Path) -> str:
107:def _parse_xyz(raw_xyz: str, *, source_path: str, joint_name: str) -> Tuple[float, float, float]:
116:def _parse_xacro_properties(urdf_text: str) -> Dict[str, str]:
127:def _resolve_xyz_value(
143:def _parse_joint_origin(
175:def load_gripper_geometry(ws_dir: Optional[str] = None) -> GripperGeometry:
202:def format_xyz(xyz: Tuple[float, float, float], *, digits: int = 7) -> str:
208:def vector_distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
218:def contact_z_correction_for_frame(
239:def tool0_offset_for_frame(
255:def _runtime_geometry_snapshot(
291:def evaluate_runtime_geometry(
373:def evaluate_geometry_snapshot(
438:def patch_runtime_model_sdf(
491:def read_pick_demo_anchor_xyz(model_sdf_path: str) -> Tuple[float, float, float]:
511:def validate_pick_demo_anchor(
```


## 14.x controller_bootstrap


```
6:# Summary: Load/configure/activate ros2_control controllers once using controller_manager services.
33:class ControllerBootstrap(Node):
49:        self.declare_parameter("service_timeout_sec", 5.0)
73:        self._service_timeout = read_float_param(
74:            self, "service_timeout_sec", 5.0, min_value=0.5
93:        self._service = self.create_service(
100:        self.create_subscription(
107:        self.create_subscription(
115:        self._list_client = self.create_client(
120:        self._load_client = self.create_client(
125:        self._configure_client = self.create_client(
130:        self._switch_client = self.create_client(
156:    def _wait_for_services(self) -> bool:
157:        deadline = time.monotonic() + self._service_timeout
160:                self._list_client.service_is_ready()
161:                and self._load_client.service_is_ready()
162:                and self._configure_client.service_is_ready()
163:                and self._switch_client.service_is_ready()
205:        req.timeout.sec = int(max(2.0, self._service_timeout))
206:        return self._call(self._switch_client, req, max(3.0, self._service_timeout))
211:        return self._call(self._configure_client, req, max(2.0, self._service_timeout))
216:        return self._call(self._load_client, req, max(2.0, self._service_timeout))
291:        if not self._wait_for_services():
292:            self.get_logger().error("controller_manager services no disponibles")
360:def main(args=None) -> None:
```


## 14.x release_objects_service


```
2:# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/release_objects_service.py
5:"""ROS 2 service to release DROP objects in Gazebo (spawn as dynamic on demand)."""
49:class ReleaseObjectsService(Node):
51:        super().__init__("release_objects_service")
57:        self.declare_parameter("delete_service", "")
58:        self.declare_parameter("spawn_service", "")
60:        self.declare_parameter("service_timeout_sec", 0.35)
61:        self.declare_parameter("service_wait_sec", 0.2)
62:        self.declare_parameter("service_wait_total_sec", 0.8)
74:        self._service = self.create_service(
91:            name: self.create_publisher(Empty, f"/drop_anchor/{name}/detach", 10)
146:                # Fail-fast: avoid expensive `gz service -l` here.
149:                return False, "ros_gz services no disponibles"
177:                "[PHYSICS][DROP] ros services no disponibles; usando gz transport"
181:        timeout = read_float_param(self, "service_timeout_sec", 1.0, min_value=0.1)
276:        delete_service = read_str_param(self, "delete_service", "").strip()
277:        spawn_service = read_str_param(self, "spawn_service", "").strip()
278:        if not delete_service:
279:            delete_service = (
280:                self._find_service_by_type(world_name, "DeleteEntity")
283:        if not spawn_service:
284:            spawn_service = (
285:                self._find_service_by_type(world_name, "SpawnEntity")
292:                "srv_name",
295:            != delete_service
297:            self._delete_client = self.create_client(DeleteEntity, delete_service)
302:                "srv_name",
305:            != spawn_service
307:            self._spawn_client = self.create_client(SpawnEntity, spawn_service)
308:        wait_sec = read_float_param(self, "service_wait_sec", 1.5, min_value=0.1)
310:            self, "service_wait_total_sec", 4.0, min_value=0.2
314:            delete_ready = self._delete_client.wait_for_service(timeout_sec=wait_sec)
315:            spawn_ready = self._spawn_client.wait_for_service(timeout_sec=wait_sec)
319:        if not self._delete_client.service_is_ready():
321:                f"[PHYSICS][DROP] servicio delete no disponible: {delete_service}"
323:        if not self._spawn_client.service_is_ready():
325:                f"[PHYSICS][DROP] servicio spawn no disponible: {spawn_service}"
428:        delete_service: str,
429:        spawn_service: str,
450:                delete_service,
458:            spawn_service,
509:        topic = f"/world/{world_name}/pose/info"
510:        cmd = f"{env_prefix}gz topic -t {topic} -n 1 -e"
572:            ready, published = self._publish_detach_topics(
588:    def _publish_detach_topics(
597:            pub = self._drop_detach_pubs.get(name)
699:                self._publish_detach_topics(
781:        delete_service = self._resolve_delete_service(world_name, env_prefix)
782:        if not delete_service:
784:        spawn_service = self._resolve_spawn_service(world_name, env_prefix)
785:        if not spawn_service:
788:            f"[PHYSICS][DROP] gz_services delete={delete_service} spawn={spawn_service}"
792:            delete_service=delete_service,
793:            spawn_service=spawn_service,
800:            if not self._gz_delete_entity(env_prefix, delete_service, name):
812:            if not self._gz_spawn_entity(env_prefix, spawn_service, name, sdf, pose):
823:    def _gz_service_exists(self, env_prefix: str, service: str) -> bool:
825:        cmd = f"{env_prefix}gz service -s {service} -i"
837:    def _resolve_delete_service(self, world_name: str, env_prefix: str) -> Optional[str]:
838:        explicit = read_str_param(self, "delete_service", "").strip()
840:            self.get_logger().info(f"[PHYSICS][DROP] delete_service (explicit): {explicit}")
849:        return self._resolve_gz_service(candidates, env_prefix, "delete")
851:    def _resolve_spawn_service(self, world_name: str, env_prefix: str) -> Optional[str]:
852:        explicit = read_str_param(self, "spawn_service", "").strip()
854:            self.get_logger().info(f"[PHYSICS][DROP] spawn_service (explicit): {explicit}")
862:        return self._resolve_gz_service(candidates, env_prefix, "spawn")
864:    def _resolve_gz_service(
873:            if self._gz_service_exists(env_prefix, svc):
902:    def _list_gz_services(self, env_prefix: str) -> List[str]:
910:                    ["bash", "-lc", f"{env_prefix}gz service -l"],
920:                services = [
925:                if services:
926:                    return services
931:                f"[PHYSICS][DROP] gz service list error: {last_error}"
935:    def _pick_gz_service(
937:        services: List[str],
941:        scoped = [s for s in services if f"/world/{world_name}/" in s]
956:        service: str,
967:            f"{env_prefix}gz service -s {service} "
1004:        service: str,
```


## 14.x tf_probe


```
33:class ProbeState:
40:class TfProbe(Node):
189:def main() -> None:
```


## 14.x clock_probe


```
26:class ClockState:
33:class ClockProbe(Node):
50:        self._sub = self.create_subscription(Clock, "/clock", self._on_clock, qos)
94:def main() -> None:
```


## 14.x cycle_logger


```
56:def _next_cycle_id() -> str:
65:class CycleLogger:
```


## 14.x jt_smoke_test


```
15:from control_msgs.action import FollowJointTrajectory
18:from rclpy.action import ActionClient
40:class JointSnapshot:
45:class JointTrajectorySmokeTest(Node):
51:            "traj_topic", "/joint_trajectory_controller/joint_trajectory"
54:            "traj_action", "/joint_trajectory_controller/follow_joint_trajectory"
64:        self._traj_topic = read_str_param(
66:            "traj_topic",
69:        self._traj_action = read_str_param(
71:            "traj_action",
84:        self._pub = self.create_publisher(JointTrajectory, self._traj_topic, qos)
85:        self._action = ActionClient(
88:            self._traj_action,
90:        self._sub = self.create_subscription(
105:            f"JT smoke test ready (topic={self._traj_topic}, action={self._traj_action}, joints={len(self._joints)})"
172:        if self._action.wait_for_server(timeout_sec=2.0):
173:            self._action.send_goal_async(goal).add_done_callback(self._on_goal_response)
178:                "FollowJointTrajectory action unavailable; fallback a topic publish"
226:def main() -> None:
```


# 15. tfm_grasping — Percepción


## 15.1 grasp_inference.py


```
40:class GraspInferenceNode(Node):
196:def main(args=None):
```


## 15.2 grasp_module.py


```
15:class TFMGraspModule:
```


## 15.3 geometry.py


```
11:class Grasp2D:
```


## 15.4 config.py


```
# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/config.py
# Contenido: Codigo de percepcion y agarre del paquete tfm_grasping.
# Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre.
"""Configuration defaults for the TFM grasping module."""

DEFAULT_MODEL_PATH = ""
DEFAULT_ROS_TOPIC = "/tfm_grasp"
DEFAULT_FRAME_ID = "camera_overhead"
DEFAULT_MIN_CONFIDENCE = 0.2
```


## 15.5 ros_interface.py


```
20:class RosGraspPublisher:
```


# 16. Frames TF y Geometría


## 16.1 Tabla de frames

| Frame | Parent | Offset (xyz rpy) | Fuente | Uso |
|-------|--------|-------------------|--------|-----|
| world | — | 0 0 0 | ur5_stack.launch.py (world_tf_publisher) | Origen global |
| base_link | world | env WS_X/Y/Z | world_tf_publisher | Base del robot |
| base_link_inertia | base_link | 0 0 0 | ur5.urdf.xacro | Inercial UR5 |
| shoulder_link | base_link_inertia | DH | ur5.urdf.xacro | Hombro |
| upper_arm_link | shoulder_link | DH | ur5.urdf.xacro | Brazo superior |
| forearm_link | upper_arm_link | DH | ur5.urdf.xacro | Antebrazo |
| wrist_1_link | forearm_link | DH | ur5.urdf.xacro | Muñeca 1 |
| wrist_2_link | wrist_1_link | DH | ur5.urdf.xacro | Muñeca 2 |
| wrist_3_link | wrist_2_link | DH | ur5.urdf.xacro | Muñeca 3 |
| tool0 | wrist_3_link | 0 0 0 | ur5.urdf.xacro | Flange UR5 |
| rg2_hand | tool0 | 0 0 0 (rpy=0) | ur5.urdf.xacro + SDF | Cuerpo gripper |
| rg2_tcp | tool0 | 0 0 0.0050885 | ur5.urdf.xacro | Alias TCP semántico |
| rg2_pinch_center | tool0 | 0 0 0.0050885 | ur5.urdf.xacro (SRDF tip) | TCP operacional MoveIt |
| rg2_leftfinger | rg2_hand | 0 +0.Y 0.120 | ur5.urdf.xacro | Dedo izquierdo prismático |
| rg2_rightfinger | rg2_hand | 0 −0.Y 0.120 | ur5.urdf.xacro | Dedo derecho prismático |
| pick_demo_anchor | world | gz pose | gz_pose_bridge | Pose objeto pick_demo |

## 16.2 TCP semántico — historial de cambios

| Fecha | Valor anterior | Valor actual | Motivo |
|-------|----------------|--------------|--------|
| pre-2026-04-22 | 0.175 m (yemas visuales) | — | Error: confundía TCP con yemas |
| 2026-04-22 | 0.175 | 0.05 (GRIPPER_TCP_Z_OFFSET) | Fix parcial |
| 2026-04-25 | 0.05 | **0.0050885 m** | Unificación TCP canónico |
| 2026-04-25 | pitch=−π/2 (end_effector_frame_fixed_joint) | rpy=0 0 0 | Fix SDF 90° mismatch |

## 16.3 world_tf_publisher.py — offsets publicados


```
2:# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py
5:# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py
6:# Summary: Publishes world->base_link TF from Gazebo pose/info.
7:"""Publish world->base_link using Gazebo model pose bridged to ROS."""
18:from geometry_msgs.msg import TransformStamped
28:    """Publish world->base_link from /world/<world>/pose/info."""
31:        super().__init__("world_tf_publisher")
32:        self.declare_parameter("world_name", "ur5_mesa_objetos")
34:        self.declare_parameter("base_frame", "base_link")
35:        self.declare_parameter("world_frame", "world")
36:        self.declare_parameter("world_file", "")
43:        self._world_name = str(self.get_parameter("world_name").value)
46:        self._world_frame = str(self.get_parameter("world_frame").value)
47:        self._world_file = str(self.get_parameter("world_file").value)
60:        self._topic = f"/world/{self._world_name}/pose/info"
62:        # Static broadcaster used only for the pre-warm transform when static_grace_sec < 0.
63:        # Published once on /tf_static so tf2 consumers have world->base_link immediately,
89:        self._timer = self.create_timer(0.1, self._publish_timer)
96:            self.get_logger().info("Waiting for /clock before publishing TF.")
100:            # static_grace_sec < 0: publish world->base_link immediately on /tf_static
103:            self._publish_static_prewarm()
105:    def _publish_static_prewarm(self) -> None:
106:        """Publish world->base_link on /tf_static immediately, before clock/Gazebo are ready.
109:        world->base_link from the very first second of startup.  The dynamic TF published
110:        by _publish_timer will automatically take precedence once Gazebo is live.
114:                "static_grace_sec<0 pero no hay pose estática disponible en el world file; "
119:        out = TransformStamped()
122:        out.header.frame_id = self._world_frame or "world"
124:        out.transform.translation.x = tx
125:        out.transform.translation.y = ty
126:        out.transform.translation.z = tz
127:        out.transform.rotation.x = rx
128:        out.transform.rotation.y = ry
129:        out.transform.rotation.z = rz
130:        out.transform.rotation.w = rw
133:            f"[PRE-WARM] Publicado {self._world_frame}->{self._base_frame} en /tf_static "
134:            f"(pose estática del world file: x={tx:.3f} y={ty:.3f} z={tz:.3f}). "
139:    def _name_from_tf(self, tf: TransformStamped) -> str:
222:        world_file = self._world_file
223:        if not world_file:
227:            world_file = os.path.join(ws_dir, "worlds", f"{self._world_name}.sdf")
228:        if not os.path.isfile(world_file):
231:            with open(world_file, "r", encoding="utf-8") as f:
253:    def _is_identity(self, tf: TransformStamped) -> bool:
254:        t = tf.transform.translation
255:        r = tf.transform.rotation
290:        if name.endswith("::base_link"):
297:            # model-level pose accepted as fallback when link-level (::base_link) unavailable
301:    def _select_transform(self, msg: TFMessage) -> Optional[TransformStamped]:
302:        names: dict[str, TransformStamped] = {}
303:        for tf in msg.transforms:
323:    def _stamp_valid(self, tf: TransformStamped) -> bool:
336:        for tf in msg.transforms:
340:        tf = self._select_transform(msg)
361:            tf.transform.translation.x,
362:            tf.transform.translation.y,
363:            tf.transform.translation.z,
364:            tf.transform.rotation.x,
365:            tf.transform.rotation.y,
366:            tf.transform.rotation.z,
367:            tf.transform.rotation.w,
378:        ctx = f"(world={self._world_name} model={self._model_name} base={self._base_frame})"
389:    def _publish_timer(self) -> None:
426:                        f"(world={self._world_name} "
431:                        "Pose/info no disponible; usando pose estática del world file para TF. "
452:        out = TransformStamped()
454:        out.header.frame_id = self._world_frame or "world"
456:        out.transform.translation.x = tx
457:        out.transform.translation.y = ty
458:        out.transform.translation.z = tz
459:        out.transform.rotation.x = rx
460:        out.transform.rotation.y = ry
461:        out.transform.rotation.z = rz
462:        out.transform.rotation.w = rw
```


## 16.4 gripper_geometry.py — constantes


```
23:RG2_TCP_FRAME = "rg2_tcp"
24:RG2_PINCH_CENTER_FRAME = "rg2_pinch_center"
25:RG2_TCP_JOINT = "rg2_tcp_joint"
26:RG2_PINCH_CENTER_JOINT = "rg2_pinch_center_joint"
27:PICK_DEMO_ANCHOR_JOINT = "pick_demo_anchor_joint"
28:CONTACT_SEMANTIC_FRAMES = frozenset((RG2_TCP_FRAME, RG2_PINCH_CENTER_FRAME))
32:class FixedJointOrigin:
42:class GripperGeometry:
45:    tcp: FixedJointOrigin
46:    pinch_center: FixedJointOrigin
48:    def origin_for_frame(self, frame_name: str) -> FixedJointOrigin:
51:            return self.tcp
53:            return self.pinch_center
56:    def xyz_for_frame(self, frame_name: str) -> Tuple[float, float, float]:
59:    def z_for_frame(self, frame_name: str) -> float:
63:def _candidate_urdf_paths(ws_dir: Optional[str] = None) -> Iterable[Path]:
66:    def _push(path: Path) -> Iterable[Path]:
102:def _read_text(path: Path) -> str:
107:def _parse_xyz(raw_xyz: str, *, source_path: str, joint_name: str) -> Tuple[float, float, float]:
116:def _parse_xacro_properties(urdf_text: str) -> Dict[str, str]:
127:def _resolve_xyz_value(
143:def _parse_joint_origin(
175:def load_gripper_geometry(ws_dir: Optional[str] = None) -> GripperGeometry:
176:    """Carga del URDF los offsets canonicos del TCP y del pinch-center del RG2."""
184:        tcp = _parse_joint_origin(
190:        pinch_center = _parse_joint_origin(
196:        return GripperGeometry(tcp=tcp, pinch_center=pinch_center)
202:def format_xyz(xyz: Tuple[float, float, float], *, digits: int = 7) -> str:
208:def vector_distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
218:def contact_z_correction_for_frame(
225:    `rg2_tcp` y `rg2_pinch_center` ya son frames semanticos de contacto y, por tanto,
239:def tool0_offset_for_frame(
255:def _runtime_geometry_snapshot(
268:        "source_path": geometry.tcp.source_path,
269:        "urdf_source": geometry.tcp.source_path,
291:def evaluate_runtime_geometry(
368:        snapshot.get("urdf_source") or snapshot.get("source_path") or geometry.tcp.source_path
373:def evaluate_geometry_snapshot(
390:        "source_path": geometry.tcp.source_path,
430:            "rg2_tcp_vs_rg2_pinch_center err_m="
431:            f"{pair_err_m:.4f} actual_tcp=({format_xyz(normalized_actual[RG2_TCP_FRAME], digits=4)}) "
432:            f"actual_pinch=({format_xyz(normalized_actual[RG2_PINCH_CENTER_FRAME], digits=4)})",
438:def patch_runtime_model_sdf(
491:def read_pick_demo_anchor_xyz(model_sdf_path: str) -> Tuple[float, float, float]:
511:def validate_pick_demo_anchor(
```


# 17. Tests y Validación


## 17.1 Resumen de tests


```
Paquetes con tests:
  tfm_grasping: 2 archivos, 7 funciones test_*
  ur5_bringup: 1 archivos, 32 funciones test_*
  ur5_qt_panel: 20 archivos, 375 funciones test_*
  ur5_tools: 10 archivos, 165 funciones test_*
```


## 17.2 validate_startup_repro.sh


```
#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/validate_startup_repro.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
CYCLES="${1:-3}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$WS_DIR/report/repro_startup/$STAMP"
mkdir -p "$OUT_DIR"

cd "$WS_DIR"

echo "[REPRO] ws=$WS_DIR" | tee -a "$OUT_DIR/summary.log"
echo "[REPRO] cycles=$CYCLES" | tee -a "$OUT_DIR/summary.log"
echo "[REPRO] out=$OUT_DIR" | tee -a "$OUT_DIR/summary.log"

fails=0

source_ros() {
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$WS_DIR/install/setup.bash"
  set -u
}

run_pipeline_check() {
  source_ros
  local expected_action="${PANEL_EXPECTED_TRAJ_ACTION:-/joint_trajectory_controller/follow_joint_trajectory}"
  local controller_manager="${PANEL_CONTROLLER_MANAGER:-/controller_manager}"
  local nodes actions controllers joints result="PASS"
  local joint_regex="shoulder_pan_joint|shoulder_lift_joint|elbow_joint|wrist_1_joint|wrist_2_joint|wrist_3_joint"

  nodes="$(ros2 node list 2>/dev/null | grep '^/' | wc -l | tr -d ' ' || true)"
  actions="$(ros2 action list 2>/dev/null | grep -c "^${expected_action}$" || true)"
  controllers=0
  for _ in $(seq 1 10); do
    controllers="$(ros2 control list_controllers --controller-manager "$controller_manager" 2>/dev/null | grep -Ec '[[:space:]]active$' || true)"
    [[ "$controllers" =~ ^[0-9]+$ ]] || controllers=0
    if [[ "$controllers" -ge 3 ]]; then
      break
    fi
    sleep 1
  done

  local js_msg
  js_msg="$(timeout 8s ros2 topic echo --once /joint_states 2>/dev/null || true)"
  joints="$(printf '%s\n' "$js_msg" | grep -Eo "$joint_regex" | sort -u | wc -l | tr -d ' ' || true)"

  echo "NODES=$nodes"
  echo "ACTION_AVAILABLE=$actions ($expected_action)"
  echo "ACTIVE_CONTROLLERS=$controllers"
  echo "UR5_JOINTS_IN_JOINT_STATES=$joints"

  [[ "$nodes" -ge 20 ]] || result="FAIL"
  [[ "$actions" -ge 1 ]] || result="FAIL"
  [[ "$controllers" -ge 3 ]] || result="FAIL"
  [[ "$joints" -eq 6 ]] || result="FAIL"

  echo "RESULTADO: $result"
  [[ "$result" == "PASS" ]]
}

capture_diag_check() {
  local output_json="$1"
  source_ros
  python3 ./scripts/capture_system_diag.py \
    --timeout 15.0 \
    --require-geometry-ok \
    --require-state READY \
    --output "$output_json"
}

for i in $(seq 1 "$CYCLES"); do
  echo "" | tee -a "$OUT_DIR/summary.log"
  echo "=== CYCLE $i START ===" | tee -a "$OUT_DIR/summary.log"

  ./scripts/stop_panel_v2.sh > "$OUT_DIR/cycle${i}_stop.log" 2>&1 || true
  ./scripts/start_panel_v2.sh --bg > "$OUT_DIR/cycle${i}_start.log" 2>&1

  sleep 10

  diag_ok=0
  if capture_diag_check "$OUT_DIR/cycle${i}_system_diag.json" > "$OUT_DIR/cycle${i}_system_diag.log" 2>&1; then
    diag_ok=1
  fi

  pipeline_ok=0
  if run_pipeline_check > "$OUT_DIR/cycle${i}_pipeline.log" 2>&1; then
    pipeline_ok=1
  fi

  if [[ "$diag_ok" -eq 1 && "$pipeline_ok" -eq 1 ]]; then
    ./scripts/stop_panel_v2.sh > "$OUT_DIR/cycle${i}_post_stop.log" 2>&1 || true
    source_ros
    post_nodes="$(ros2 node list 2>/dev/null | grep '^/' | wc -l | tr -d ' ' || true)"
    post_nodes="${post_nodes:-0}"
    echo "POST_STOP_NODES=$post_nodes" >> "$OUT_DIR/cycle${i}_pipeline.log"

```


## 17.3 validate_before_demo.sh


```
#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/validate_before_demo.sh
# Contenido: Checklist pre-demo — verifica que el stack está listo para ejecutar DIRECTO.
# Uso breve: ./scripts/validate_before_demo.sh
#   Requiere ROS 2 sourced y stack arrancado (Gazebo + MoveIt + panel).
#
# Devuelve 0 si todos los checks OBLIGATORIOS pasan, 1 en caso contrario.
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PASS=0
FAIL=0
WARN=0
ERRORS=()
WARNINGS=()

_ok()   { echo "[PRE_DEMO][OK]   $*"; PASS=$((PASS + 1)); }
_fail() { echo "[PRE_DEMO][FAIL] $*"; FAIL=$((FAIL + 1)); ERRORS+=("$*"); }
_warn() { echo "[PRE_DEMO][WARN] $*"; WARN=$((WARN + 1)); WARNINGS+=("$*"); }
_sep()  { echo ""; echo "--- $* ---"; }

echo "[PRE_DEMO] workspace=$ROOT_DIR  $(date -Iseconds)"

# ---------------------------------------------------------------------------
# 0. Smoke test (static checks, no ROS needed)
# ---------------------------------------------------------------------------
_sep "Smoke test (static)"
if bash "$ROOT_DIR/scripts/smoke_test.sh" --fast > /tmp/smoke_test_out.txt 2>&1; then
  _ok "smoke_test.sh --fast PASS"
else
  _fail "smoke_test.sh --fast FAIL — run: bash scripts/smoke_test.sh --fast"
  tail -20 /tmp/smoke_test_out.txt
fi

# ---------------------------------------------------------------------------
# 1. ROS 2 environment
# ---------------------------------------------------------------------------
_sep "ROS 2 environment"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  _fail "ROS_DISTRO not set — source /opt/ros/jazzy/setup.bash"
else
  _ok "ROS_DISTRO=$ROS_DISTRO"
fi

if [[ -z "${GZ_PARTITION:-}" ]]; then
  _warn "GZ_PARTITION not set — Gazebo topics may not resolve"
else
  _ok "GZ_PARTITION=$GZ_PARTITION"
fi

# ---------------------------------------------------------------------------
# 2. ROS 2 nodes
# ---------------------------------------------------------------------------
_sep "ROS 2 nodes"
if ! command -v ros2 >/dev/null 2>&1; then
  _fail "ros2 CLI not found — source ROS 2"
else
  NODE_LIST=$(timeout 5 ros2 node list 2>/dev/null || true)
  for expected_node in \
    "/move_group" \
    "/robot_state_publisher" \
    "/gz_ros2_control"; do
    if echo "$NODE_LIST" | grep -q "$expected_node"; then
      _ok "node found: $expected_node"
    else
      _fail "node MISSING: $expected_node"
    fi
  done
fi

# ---------------------------------------------------------------------------
# 3. TF frames
# ---------------------------------------------------------------------------
_sep "TF frames"
REQUIRED_FRAMES=(
  "world"
  "base_link"
  "tool0"
  "rg2_pinch_center"
)
for frame in "${REQUIRED_FRAMES[@]}"; do
  if timeout 5 ros2 run tf2_ros tf2_echo world "$frame" 2>/dev/null | grep -q "Translation"; then
    _ok "TF frame reachable: $frame"
  else
    _warn "TF frame not confirmed: $frame (may be normal if robot just started)"
  fi
done

# ---------------------------------------------------------------------------
# 4. Gazebo object topic
# ---------------------------------------------------------------------------
_sep "Gazebo object bridge"
if timeout 5 ros2 topic echo /world/ur5_world/dynamic_pose/info \
   --once 2>/dev/null | grep -q "name"; then
  _ok "/world/ur5_world/dynamic_pose/info responding"
else
  _warn "/world/ur5_world/dynamic_pose/info not responding — check gz_bridge"
fi
```


## 17.4 validate_pick_3_cycles.sh


```
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CYCLES="${1:-${PICK_VALIDATE_CYCLES:-3}}"
WAIT_LOOPS="${PICK_VALIDATE_WAIT_LOOPS:-600}"
WAIT_SLEEP_SEC="${PICK_VALIDATE_WAIT_SLEEP_SEC:-1}"
SUMMARY_FILE="$ROOT_DIR/auditoria/pick_${CYCLES}_cycles_summary_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$ROOT_DIR/auditoria"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

run_cycle() {
  local cycle="$1"
  local log_file="$ROOT_DIR/log/ros2_launch.log"
  local start_line=1
  echo "=== CYCLE ${cycle} START ===" | tee -a "$SUMMARY_FILE"
  cd "$ROOT_DIR"
  ./scripts/stop_panel_v2.sh >/dev/null 2>&1 || true
  sleep 4
  PANEL_COLD_BOOT=1 PANEL_FORCE_OFFSCREEN=1 PANEL_START_STACK=1 PANEL_LAUNCH_MOVEIT=1 MOVEIT_MODE=move_group PANEL_AUTO_BRIDGE=0 ./scripts/start_panel_v2.sh --bg >/dev/null 2>&1

  if [[ -f "$log_file" ]]; then
    start_line=$(( $(wc -l < "$log_file") + 1 ))
  fi

  for _ in $(seq 1 50); do
    if ros2 service list 2>/dev/null | grep -q '^/panel/pick_demo$' && ros2 service list 2>/dev/null | grep -q '^/panel/select_object$'; then
      break
    fi
    sleep 1
  done

  for _ in $(seq 1 120); do
    if tail -n "+${start_line}" "$log_file" | grep -qE '\[PICK\]\[MOVEIT\]\[INIT\].*moveit_state=READY'; then
      if ! tail -n "+${start_line}" "$log_file" | grep -q 'ERROR_FATAL'; then
        sleep 3
        break
      fi
    fi
    sleep 1
  done

  local select_reply=""
  select_reply="$(ros2 service call /panel/select_object ur5_panel_interfaces/srv/SelectObject "{name: pick_demo}" 2>&1)" || {
    echo "CYCLE_${cycle}_SELECT_CALL=ERROR" | tee -a "$SUMMARY_FILE"
    echo "$select_reply" | tee -a "$SUMMARY_FILE"
    return 1
  }
  if ! grep -Eiq 'success:\s*true|success\s*=\s*True' <<<"$select_reply"; then
    echo "CYCLE_${cycle}_SELECT_CALL=REJECTED" | tee -a "$SUMMARY_FILE"
    echo "$select_reply" | tee -a "$SUMMARY_FILE"
    return 1
  fi
  sleep 1
  local pick_reply=""
  pick_reply="$(ros2 service call /panel/pick_demo std_srvs/srv/Trigger "{}" 2>&1)" || {
    echo "CYCLE_${cycle}_PICK_CALL=ERROR" | tee -a "$SUMMARY_FILE"
    echo "$pick_reply" | tee -a "$SUMMARY_FILE"
    return 1
  }
  if ! grep -Eiq 'success:\s*true|success\s*=\s*True' <<<"$pick_reply"; then
    echo "CYCLE_${cycle}_PICK_CALL=REJECTED" | tee -a "$SUMMARY_FILE"
    echo "$pick_reply" | tee -a "$SUMMARY_FILE"
    return 1
  fi

  local result="TIMEOUT"
  for _ in $(seq 1 "$WAIT_LOOPS"); do
    if tail -n "+${start_line}" "$log_file" | grep -q 'SECUENCIA COMPLETADA EXITOSAMENTE'; then
      result="PASS"
      break
    fi
    if tail -n "+${start_line}" "$log_file" | grep -Eq 'carry_coherence_failed|\[PICK_OBJ\]\[ABORT\]|\[PICK\]\[DIRECT\]\[ABORT\]|Error en pick objeto|\[PICK_OBJ\]\[FAIL_CLASS\]'; then
      result="FAIL"
      break
    fi
```


## 17.5 smoke_test.sh


```
#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/smoke_test.sh
# Contenido: Smoke test rápido del workspace — sin ROS/Gazebo.
# Uso breve: ./scripts/smoke_test.sh [--fast]
#   --fast: omit colcon build, only run lint + unit tests.
#
# Devuelve 0 si todos los checks pasan, 1 en caso contrario.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

FAST=0
for arg in "$@"; do
  [[ "$arg" == "--fast" ]] && FAST=1
done

PASS=0
FAIL=0
ERRORS=()

_ok()   { echo "[SMOKE][OK]   $*"; PASS=$((PASS + 1)); }
_fail() { echo "[SMOKE][FAIL] $*"; FAIL=$((FAIL + 1)); ERRORS+=("$*"); }
_skip() { echo "[SMOKE][SKIP] $*"; }
_sep()  { echo ""; echo "--- $* ---"; }

echo "[SMOKE] workspace=$ROOT_DIR  fast=$FAST  $(date -Iseconds)"

# ---------------------------------------------------------------------------
# 1. Python parse check: all modules in ur5_qt_panel and ur5_tools
# ---------------------------------------------------------------------------
_sep "AST parse"
PY_FILES_QT=$(find src/ur5_qt_panel/ur5_qt_panel -name "*.py" 2>/dev/null | sort)
PY_FILES_UT=$(find src/ur5_tools/ur5_tools -name "*.py" 2>/dev/null | sort)
PY_FILES_BU=$(find src/ur5_bringup/launch -name "*.py" 2>/dev/null | sort)
parse_errors=0
for f in $PY_FILES_QT $PY_FILES_UT $PY_FILES_BU; do
  if ! python3 -c "import ast; ast.parse(open('$f').read())" 2>/dev/null; then
    _fail "Syntax error in $f"
    parse_errors=$((parse_errors + 1))
  fi
done
if [ "$parse_errors" -eq 0 ]; then
  count=$(echo "$PY_FILES_QT $PY_FILES_UT $PY_FILES_BU" | wc -w)
  _ok "All $count Python files parse cleanly"
fi

# ---------------------------------------------------------------------------
# 2. F401 unused imports (flake8)
# ---------------------------------------------------------------------------
_sep "F401 unused imports"
if command -v python3 -m flake8 >/dev/null 2>&1 || python3 -c "import flake8" 2>/dev/null; then
  f401_out=$(python3 -m flake8 \
    src/ur5_qt_panel/ur5_qt_panel \
    src/ur5_qt_panel/test \
    src/ur5_tools/ur5_tools \
    src/ur5_tools/test \
    src/ur5_bringup/launch \
    src/ur5_bringup/test \
    --select=F401 2>&1 || true)
  if [[ -n "$f401_out" ]]; then
    _fail "F401 violations:\n$f401_out"
  else
    _ok "Zero F401 unused import violations"
  fi
else
  _skip "flake8 not available"
fi

# ---------------------------------------------------------------------------
# 3. Unit tests — directo_geometry and directo_gate_evaluator (no ROS needed)
# ---------------------------------------------------------------------------
_sep "Unit tests (ur5_qt_panel — no ROS)"
ut1_ok=0
# Strip the ROS Python path so the conftest stubs don't clash with rclpy C
# extensions when this script is sourced from a ROS-active shell. Tests are
# designed to run without ROS, so the isolated PYTHONPATH is correct here.
if PYTHONPATH="src/ur5_qt_panel:src/ur5_tools" python3 -m pytest \
     src/ur5_qt_panel/test/test_directo_geometry.py \
     src/ur5_qt_panel/test/test_directo_gate_evaluator.py \
```


## 17.6 Tabla de tests clave

| Test | Archivo | Qué valida | Requiere ROS |
|------|---------|------------|--------------|
| test_gripper_geometry | ur5_tools | Geometría RG2, offsets, ranges | No |
| test_moveit_bridge_utils | ur5_tools | 16 helpers IK/FK/bridge | No |
| test_moveit_bridge_plan_success | ur5_tools | Integración plan+execute | No |
| test_attach_gate_evaluator | ur5_qt_panel | Lógica attach gate | No |
| test_directo_gate_evaluator | ur5_qt_panel | Gates DIRECTO | No |
| test_directo_geometry | ur5_qt_panel | Geometría DIRECTO | No |
| test_ur5_kinematics | ur5_qt_panel | FK/IK DH analítico | No |
| test_param_utils | ur5_tools | duck-typed, sin ROS | No |
| test_panel_config | ur5_qt_panel | Constantes panel | No |
| test_state_machine | ur5_qt_panel | Estado máquina | No |
| test_step_pipeline | ur5_qt_panel | Pipeline steps | No |
| test_panel_pick_object_moveit_init | ur5_qt_panel | MoveIt init audit | No |
| test_launch_helpers | ur5_bringup | 27 tests launch helpers | No |
| validate_startup_repro.sh | scripts | Arranque reproducible | Sí |
| validate_pick_3_cycles.sh | scripts | 3 ciclos pick completos | Sí |

# 18. Logs Recientes


## 18.1 Logs más recientes


```
log/ros2_launch.log
log/ros/python3_126851_1777235357223.log
log/ros/robot_state_publisher_126873_1777235356940.log
log/ros/parameter_bridge_126823_1777235356570.log
log/attach_backend.log
log/world_tf_publisher.log
log/ros_gz_bridge.log
log/gz_pose_bridge.log
log/ur5_rsp.log
log/ros/2026-04-26-22-29-16-608370-laboratorio-MS-7C56-126783/launch.log
log/bridge_runtime.yaml
log/world_runtime.sdf
log/gz_models/ur5_rg2/model.sdf
log/gz_partition.txt
log/ros/python3_124292_1777235107318.log
log/attach_backend.log.20260426_222915.bak
log/ros_gz_bridge.log.20260426_222915.bak
log/ros/parameter_bridge_124252_1777235106564.log
log/ros/robot_state_publisher_124312_1777235106893.log
log/world_tf_publisher.log.20260426_222915.bak
log/gz_pose_bridge.log.20260426_222915.bak
log/ur5_rsp.log.20260426_222915.bak
log/ros/2026-04-26-22-25-06-527797-laboratorio-MS-7C56-124209/launch.log
log/build_2026-04-26_22-21-52/logger_all.log
log/build_2026-04-26_22-21-52/events.log
log/build_2026-04-26_22-21-52/ur5_qt_panel/streams.log
log/build_2026-04-26_22-21-52/ur5_qt_panel/command.log
log/build_2026-04-26_22-21-52/ur5_qt_panel/stdout_stderr.log
log/build_2026-04-26_22-21-52/ur5_qt_panel/stdout.log
log/build_2026-04-26_22-21-52/ur5_qt_panel/stderr.log
```


## 18.2 ros2_launch.log (últimas 100 líneas)


```
[ros2-6] [2026-04-26T22:52:07] [PHYSICS][POSE_INFO] ready=true count=119569 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:08] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.00s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.003
[ros2-6] [2026-04-26T22:52:08] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:08] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:09] [PHYSICS][POSE_INFO] ready=true count=119759 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:09] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.00s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.004
[ros2-6] [2026-04-26T22:52:09] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:09] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:10] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:10] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:10] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.02s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.018
[ros2-6] [2026-04-26T22:52:11] [PHYSICS][POSE_INFO] ready=true count=119962 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:11] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:11] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:12] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.010
[ros2-6] [2026-04-26T22:52:12] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.011
[ros2-6] [2026-04-26T22:52:12] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:12] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:13] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.02s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.018
[ros2-6] [2026-04-26T22:52:13] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:13] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:14] [PHYSICS][POSE_INFO] ready=true count=120175 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:14] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:14] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:15] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.02s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.017
[ros2-6] [2026-04-26T22:52:16] [PHYSICS][POSE_INFO] ready=true count=120351 age=0.03s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:16] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:16] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:16] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.015
[ros2-6] [2026-04-26T22:52:17] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:17] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:17] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.010
[ros2-6] [2026-04-26T22:52:18] [PHYSICS][POSE_INFO] ready=true count=120553 age=0.02s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:18] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:18] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:18] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.006
[ros2-6] [2026-04-26T22:52:19] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:19] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:19] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.015
[ros2-6] [2026-04-26T22:52:20] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.02s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.023
[ros2-6] [2026-04-26T22:52:20] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:20] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:20] [PHYSICS][POSE_INFO] ready=true count=120744 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:21] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:21] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:22] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.014
[ros2-6] [2026-04-26T22:52:23] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:23] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:23] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.012
[ros2-6] [2026-04-26T22:52:23] [PHYSICS][POSE_INFO] ready=true count=120979 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:24] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:24] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:24] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.00s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.004
[ros2-6] [2026-04-26T22:52:25] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:25] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:25] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.10s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.105
[ros2-6] [2026-04-26T22:52:25] [PHYSICS][POSE_INFO] ready=true count=121158 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:26] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:26] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:26] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.03s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.027
[ros2-6] [2026-04-26T22:52:27] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:27] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:27] [PHYSICS][POSE_INFO] ready=true count=121364 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:27] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.010
[ros2-6] [2026-04-26T22:52:28] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:28] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:28] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.00s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.002
[ros2-6] [2026-04-26T22:52:29] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:29] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:30] [PHYSICS][POSE_INFO] ready=true count=121554 age=0.01s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:30] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.02s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.022
[ros2-6] [2026-04-26T22:52:31] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.02s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.015
[ros2-6] [2026-04-26T22:52:31] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:31] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:32] [PHYSICS][POSE_INFO] ready=true count=121727 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:32] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.013
[ros2-6] [2026-04-26T22:52:32] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:32] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:33] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.13s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.133
[ros2-6] [2026-04-26T22:52:33] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:33] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:34] [PHYSICS][POSE_INFO] ready=true count=121909 age=0.00s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:34] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.010
[ros2-6] [2026-04-26T22:52:34] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:34] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:35] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.03s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.028
[ros2-6] [2026-04-26T22:52:36] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:36] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:36] [PHYSICS][POSE_INFO] ready=true count=122119 age=0.01s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:36] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.02s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.015
[ros2-6] [2026-04-26T22:52:37] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:37] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:38] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.03s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.030
[ros2-6] [2026-04-26T22:52:38] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:38] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:38] [PHYSICS][POSE_INFO] ready=true count=122293 age=0.01s entities=74 topic=/world/ur5_mesa_objetos/pose/info
[ros2-6] [2026-04-26T22:52:38] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.01s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.008
[ros2-6] [2026-04-26T22:52:39] [PICK][OVERLAY] draw px=(160,171) base=yes
[ros2-6] [2026-04-26T22:52:39] [PICK][VISUAL][TCP_TARGET_LINE] origin=rg2_tcp@base_link obj=pick_demo tcp_base=(0.503,0.366,0.599) tcp_world=(-0.308,0.792,1.248) tcp_px=(0.0,45.3) src=world_xyz_3d tcp_source=tf2:rg2_tcp@base_link topic=/camera_overhead/image
[ros2-6] [2026-04-26T22:52:40] [PICK][DIRECT][PANEL_TRACE] world_frame=world base_frame=base_link ee_frame=rg2_pinch_center selected_object=pick_demo object_source=stable_object_cache:fallback age=0.02s object_pose_base=obj: pos=(0.150,0.208,-0.193) quat=(0.460,0.537,0.536,0.461) tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.503,0.366,0.599) quat=(-0.535,0.462,0.459,0.537) tcp_panel_fk_base=-- tcp_panel_fk_rpy_deg=-- tcp_live_rpy_deg=(-90.152,81.306,-0.435) panel_live_delta=-- panel_live_dist_m=-- panel_fk_age_sec=0.000 tcp_live_age_sec=0.000 object_age_sec=0.021
```


## 18.3 Errores en logs recientes


```
log/build_2026-04-25_21-29-04/events.log:[0.258529] (ur5_description) Command: {'cmd': ['/usr/bin/cmake', '/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description', '-DAMENT_CMAKE_SYMLINK_INSTALL=1', '-DCMAKE_INSTALL_PREFIX=/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[0.262568] (ur5_panel_interfaces) Command: {'cmd': ['/usr/bin/cmake', '/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_panel_interfaces', '-DAMENT_CMAKE_SYMLINK_INSTALL=1', '-DCMAKE_INSTALL_PREFIX=/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[1.227276] (tfm_grasping) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[1.228667] (ur5_tools) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[1.415992] (ur5_description) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[1.484150] (ur5_description) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[1.774862] (ur5_moveit_config) Command: {'cmd': ['/usr/bin/cmake', '/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_moveit_config', '-DAMENT_CMAKE_SYMLINK_INSTALL=1', '-DCMAKE_INSTALL_PREFIX=/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[4.068356] (ur5_panel_interfaces) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[5.928564] (ur5_moveit_config) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[5.983138] (ur5_moveit_config) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[11.527009] (ur5_panel_interfaces) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[13.342931] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[14.138191] (ur5_bringup) Command: {'cmd': ['/usr/bin/cmake', '/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup', '-DAMENT_CMAKE_SYMLINK_INSTALL=1', '-DCMAKE_INSTALL_PREFIX=/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_bringup'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[14.989039] (ur5_bringup) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/events.log:[15.039379] (ur5_bringup) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_21-29-04/logger_all.log:[0.110s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=None, packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x773773ea4dd0>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x773773fc10d0>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x773773fc10d0>>, mixin_verb=('build',))
log/build_2026-04-25_21-32-59/events.log:[0.911123] (ur5_tools) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_21-32-59/logger_all.log:[0.113s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_tools'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x73c2c7490ef0>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x73c2c75cd580>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x73c2c75cd580>>, mixin_verb=('build',))
log/build_2026-04-25_22-25-44/events.log:[0.008571] (ur5_description) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-25-44/events.log:[0.177216] (ur5_description) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-25-44/events.log:[0.212203] (ur5_moveit_config) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-25-44/events.log:[0.253197] (ur5_moveit_config) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-25-44/events.log:[0.610267] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_22-25-44/logger_all.log:[0.072s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_description', 'ur5_moveit_config', 'ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x79b0e78b4f50>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x79b0e79d1310>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x79b0e79d1310>>, mixin_verb=('build',))
log/build_2026-04-25_22-27-41/events.log:[0.163926] (ur5_description) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[0.165411] (ur5_panel_interfaces) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[0.334427] (ur5_description) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_description', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[0.371862] (ur5_moveit_config) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[0.414948] (ur5_moveit_config) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_moveit_config', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[0.485703] (ur5_panel_interfaces) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_panel_interfaces', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[0.760281] (tfm_grasping) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[0.771963] (ur5_tools) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[1.851194] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[2.353723] (ur5_bringup) Command: {'cmd': ['/usr/bin/cmake', '--build', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', '--', '-j16', '-l16'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-27-41/events.log:[2.387394] (ur5_bringup) Command: {'cmd': ['/usr/bin/cmake', '--install', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'env': OrderedDict({'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_bringup', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_moveit_config:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_description:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor:/opt/ros/jazzy', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}), 'shell': False}
log/build_2026-04-25_22-27-41/logger_all.log:[0.073s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=None, packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x7caf70f8ce30>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x7caf710c9460>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x7caf710c9460>>, mixin_verb=('build',))
log/build_2026-04-25_22-32-45/events.log:[0.594356] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_common_vendor/share/gz', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.98', 'SHLVL': '2', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'ROS_PYTHON_VERSION': '3', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'ROS_DISTRO': 'jazzy', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'ROS_VERSION': '2', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping:/opt/ros/jazzy', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages', 'COLCON': '1', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor', 'COPILOT_OTEL_EXPORTER_TYPE': 'file'}, 'shell': False}
log/build_2026-04-25_22-32-45/logger_all.log:[0.072s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x7f669999cd70>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x7f6699ab9580>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x7f6699ab9580>>, mixin_verb=('build',))
log/build_2026-04-25_22-57-40/events.log:[0.437370] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'OLDPWD': '/home/laboratorio/TFM', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-25_22-57-40/logger_all.log:[0.064s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x738304632f00>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x73830477b650>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x73830477b650>>, mixin_verb=('build',))
log/build_2026-04-25_23-39-27/events.log:[0.441289] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-25_23-39-27/logger_all.log:[0.064s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x7290fa72af60>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x7290fa873590>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x7290fa873590>>, mixin_verb=('build',))
log/build_2026-04-25_23-46-49/events.log:[0.444816] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-25_23-46-49/logger_all.log:[0.064s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x773b9045afc0>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x773b905838c0>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x773b905838c0>>, mixin_verb=('build',))
log/build_2026-04-25_23-58-08/events.log:[0.443586] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-25_23-58-08/logger_all.log:[0.065s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x7dd4862d2ea0>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x7dd486477710>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x7dd486477710>>, mixin_verb=('build',))
log/build_2026-04-26_00-07-55/events.log:[0.441949] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'OLDPWD': '/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-26_00-07-55/logger_all.log:[0.066s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x7560f0a32f00>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x7560f0b7b830>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x7560f0b7b830>>, mixin_verb=('build',))
log/build_2026-04-26_00-16-31/events.log:[0.440553] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'OLDPWD': '/home/laboratorio/TFM', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-26_00-16-31/logger_all.log:[0.063s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x7255ff13ee10>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x7255ff267590>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x7255ff267590>>, mixin_verb=('build',))
log/build_2026-04-26_00-22-23/events.log:[0.441490] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-26_00-22-23/logger_all.log:[0.063s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x70e97952ec90>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x70e979677770>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x70e979677770>>, mixin_verb=('build',))
log/build_2026-04-26_00-24-01/events.log:[0.433700] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-26_00-24-01/logger_all.log:[0.064s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x796e26a47170>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x796e26b6f950>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x796e26b6f950>>, mixin_verb=('build',))
log/build_2026-04-26_00-33-12/events.log:[0.441726] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-26_00-33-12/logger_all.log:[0.064s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x757fc4436c90>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x757fc457ed50>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x757fc457ed50>>, mixin_verb=('build',))
log/build_2026-04-26_00-40-05/events.log:[0.442448] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-26_00-40-05/logger_all.log:[0.067s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x73160052ed50>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x7316006776e0>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x7316006776e0>>, mixin_verb=('build',))
log/build_2026-04-26_00-47-33/events.log:[0.440870] (ur5_qt_panel) Command: {'cmd': ['/usr/bin/python3', '-W', 'ignore:setup.py install is deprecated', '-W', 'ignore:easy_install command is deprecated', 'setup.py', 'develop', '--editable', '--build-directory', '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/build', '--no-deps', 'symlink_data'], 'cwd': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'env': {'LESSOPEN': '| /usr/bin/lesspipe %s', 'VSCODE_CWD': '/home/laboratorio', 'VSCODE_ESM_ENTRYPOINT': 'vs/workbench/api/node/extensionHostProcess', 'AI_AGENT': 'claude-code/2.1.120/agent', 'USER': 'laboratorio', 'SSH_CLIENT': '192.168.1.36 52715 22', 'CLAUDE_CODE_ENTRYPOINT': 'claude-vscode', 'VSCODE_NLS_CONFIG': '{"userLocale":"es","osLocale":"es","resolvedLanguage":"es","defaultMessagesFile":"/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/out/nls.messages.json","languagePack":{"translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","messagesFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/nls.messages.json","corruptMarkerFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"},"locale":"es","availableLanguages":{"*":"es"},"_languagePackId":"8243b20f9beb3dae2d02a0e1c0d20da1.es","_languagePackSupport":true,"_translationsConfigFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/tcf.json","_cacheRoot":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es","_resolvedLanguagePackCoreLocation":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/10c8e557c8b9f9ed0a87f61f1c9a44bde731c409","_corruptedFile":"/home/laboratorio/.vscode-server/data/clp/8243b20f9beb3dae2d02a0e1c0d20da1.es/corrupted.info"}', 'GIT_EDITOR': 'true', 'VSCODE_HANDLES_UNCAUGHT_ERRORS': 'true', 'XDG_SESSION_TYPE': 'tty', 'CLAUDE_AGENT_SDK_VERSION': '0.2.120', 'SHLVL': '2', 'BROWSER': '/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/helpers/browser.sh', 'LD_LIBRARY_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib', 'HOME': '/home/laboratorio', 'VSCODE_RECONNECTION_GRACE_TIME': '10800000', 'VSCODE_IPC_HOOK_CLI': '/run/user/1000/vscode-ipc-f35e62c8-a511-45ba-b1d4-0292d9f49cb1.sock', 'COPILOT_OTEL_FILE_EXPORTER_PATH': '/dev/null', 'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1000/bus', 'DEBUGINFOD_URLS': 'https://debuginfod.ubuntu.com', 'VSCODE_L10N_BUNDLE_LOCATION': 'vscode-local:/Users/jlozano/.vscode/extensions/ms-ceintl.vscode-language-pack-es-1.110.2026041514/translations/extensions/vscode.markdown-language-features.i18n.json', 'APPLICATION_INSIGHTS_NO_STATSBEAT': 'true', 'LOGNAME': 'laboratorio', 'OTEL_INSTRUMENTATION_GENAI_CAPTURE_MESSAGE_CONTENT': 'true', 'VSCODE_HANDLES_SIGPIPE': 'true', '_': '/usr/bin/colcon', 'XDG_SESSION_CLASS': 'user', 'XDG_SESSION_ID': '4', 'OTEL_EXPORTER_OTLP_METRICS_TEMPORALITY_PREFERENCE': 'delta', 'VSCODE_CLI_REQUIRE_TOKEN': 'dc907c5c-cd50-4b4c-8df2-dff634a92dba', 'MCP_CONNECTION_NONBLOCKING': 'true', 'PATH': '/home/laboratorio/.local/bin:/home/laboratorio/.vscode-server/cli/servers/Stable-10c8e557c8b9f9ed0a87f61f1c9a44bde731c409/server/bin/remote-cli:/home/laboratorio/.local/bin:/home/laboratorio/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin', 'VSCODE_AGENT_FOLDER': '/home/laboratorio/.vscode-server', 'COREPACK_ENABLE_AUTO_PIN': '0', 'XDG_RUNTIME_DIR': '/run/user/1000', 'DISPLAY': 'localhost:10.0', 'NoDefaultCurrentDirectoryInExePath': '1', 'LANG': 'es_ES.UTF-8', 'LS_COLORS': '', 'COPILOT_OTEL_ENABLED': 'true', 'SSH_AUTH_SOCK': '/run/user/1000/vscode-ssh-auth-sock-859868616', 'AMENT_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping', 'SHELL': '/bin/bash', 'ELECTRON_RUN_AS_NODE': '1', 'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING': 'true', 'LESSCLOSE': '/usr/bin/lesspipe %s %s', 'CLAUDECODE': '1', 'PWD': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel', 'SSH_CONNECTION': '192.168.1.36 52715 192.168.1.21 22', 'XDG_DATA_DIRS': '/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop', 'CLAUDE_CODE_EXECPATH': '/home/laboratorio/.vscode-server/extensions/anthropic.claude-code-2.1.120-linux-x64/resources/native-binary/claude', 'PYTHONPATH': '/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_qt_panel/prefix_override:/usr/lib/python3/dist-packages/colcon_core/task/python/colcon_distutils_commands:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_qt_panel/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/ur5_tools:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_tools/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces/lib/python3.12/site-packages:/home/laboratorio/TFM/agarre_ros2_ws/build/tfm_grasping:/home/laboratorio/TFM/agarre_ros2_ws/install/tfm_grasping/lib/python3.12/site-packages', 'COLCON': '1', 'COPILOT_OTEL_EXPORTER_TYPE': 'file', 'CMAKE_PREFIX_PATH': '/home/laboratorio/TFM/agarre_ros2_ws/install/ur5_panel_interfaces'}, 'shell': False}
log/build_2026-04-26_00-47-33/logger_all.log:[0.065s] DEBUG:colcon:Parsed command line arguments: Namespace(log_base=None, log_level=None, verb_name='build', build_base='build', install_base='install', merge_install=False, symlink_install=True, test_result_base=None, continue_on_error=False, executor='parallel', parallel_workers=16, event_handlers=None, ignore_user_meta=False, metas=['./colcon.meta'], base_paths=['.'], packages_ignore=None, packages_ignore_regex=None, paths=None, packages_up_to=None, packages_up_to_regex=None, packages_above=None, packages_above_and_dependencies=None, packages_above_depth=None, packages_select_by_dep=None, packages_skip_by_dep=None, packages_skip_up_to=None, packages_select_build_failed=False, packages_skip_build_finished=False, packages_select_test_failures=False, packages_skip_test_passed=False, packages_select=['ur5_qt_panel'], packages_skip=None, packages_select_regex=None, packages_skip_regex=None, packages_start=None, packages_end=None, allow_overriding=[], cmake_args=None, cmake_target=None, cmake_target_skip_unavailable=False, cmake_clean_cache=False, cmake_clean_first=False, cmake_force_configure=False, ament_cmake_args=None, catkin_cmake_args=None, catkin_skip_building_tests=False, mixin_files=None, mixin=None, verb_parser=<colcon_mixin.mixin.mixin_argument.MixinArgumentDecorator object at 0x7ce7b252ad50>, verb_extension=<colcon_core.verb.build.BuildVerb object at 0x7ce7b2673530>, main=<bound method BuildVerb.main of <colcon_core.verb.build.BuildVerb object at 0x7ce7b2673530>>, mixin_verb=('build',))
```


## 18.4 attach_backend.log


```
[INFO] [1777235357.380922588] [gripper_attach_backend]: [ATTACH_BACKEND] ready objects=box_blue,box_green,box_lightblue,box_red,box_yellow,cross_cyan,cyl_gray,cyl_green,cyl_orange,cyl_purple,pick_demo mode=detachable_joint gripper_prefix=/gripper tool_anchor_prefix=/gripper_anchor prefer_tool_anchor=none demo_transport=pick_demo pose_topic=/world/ur5_mesa_objetos/pose/info joint_states_topic=/joint_states base_frame=base_link tcp_frame=rg2_pinch_center
[INFO] [1777235358.971705662] [gripper_attach_backend]: [ATTACH_BACKEND] startup_tool_detach sent=11 ready=11/11 attempts_left=11
```


## 18.5 world_tf_publisher.log


```
[INFO] [1777235357.157758969] [world_tf_publisher]: WorldTfPublisher listening on /world/ur5_mesa_objetos/pose/info (model=ur5_rg2, base=base_link)
[INFO] [1777235357.158686484] [world_tf_publisher]: Waiting for /clock before publishing TF.
[INFO] [1777235358.030772657] [world_tf_publisher]: /clock listo; publicando TF.
```


## 18.6 Reportes de auditoría


### report/auditoria_moveit_step_20260424.md


```
# Auditoría MOVEIT + STEP — UR5+RG2 Panel Qt

**Fecha:** 2026-04-24  
**Rama:** ENTREGA.V2  
**Stack:** ROS 2 Jazzy · Gazebo Sim Harmonic · MoveIt 2 · ros2_control · UR5+OnRobot RG2  
**Modo auditado:** `run_pick_object()` → `panel_pick_object.py` (MODO MOVEIT)  
**Modo de referencia:** `run_pick_demo()` → `panel_pick_demo.py` (DIRECTO) — NO modificado

---

## Resumen ejecutivo

Se realizó una auditoría completa del modo MOVEIT (PICK Objeto) en busca de:

1. Falta de validación explícita de TF/EE al arranque (equivalente al `SYNC_GATE stage=tf_and_ee` de DIRECTO)
2. Fallback incorrecto del frame EE en el worker (`RG2_PINCH_CENTER_FRAME` en lugar de `RG2_TCP_FRAME`)
3. Ausencia de prefijos de log estructurados `[PICK][MOVEIT][*]` para trazabilidad homogénea
4. Modo STEP sin logs de auditoría en el gate de fases

Se corrigieron **7 incidencias** en `panel_pick_object.py`, se añadió **1 nuevo test** con 12 casos, y la build completó limpia en 2 paquetes.

---

## Ficheros inspeccionados

| Fichero | Rol | Modificado |
|---------|-----|-----------|
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py` | Pipeline MOVEIT principal | **Sí** |
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` | Pipeline DIRECTO (referencia) | No (sesión) |
| `src/ur5_qt_panel/ur5_qt_panel/panel_v2.py` | Panel principal Qt | No |
| `src/ur5_tools/ur5_tools/ur5_moveit_bridge.py` | Bridge MoveIt↔Panel | No |
| `src/ur5_tools/ur5_tools/gripper_geometry.py` | Constantes geométricas | No |
| `src/ur5_tools/test/test_gripper_geometry.py` | Tests geometría canónica | No (sesión) |
| `src/ur5_qt_panel/test/test_panel_pick_object_moveit_init.py` | Tests MOVEIT init/step | **Creado** |

---

## Bugs encontrados y corregidos

### BUG-M01 — Importación faltante: `RG2_TCP_FRAME`

**Severidad:** Media  
**Ubicación:** `panel_pick_object.py` línea 28-33 (bloque import gripper_geometry)

`RG2_TCP_FRAME` no estaba importado desde `ur5_tools.gripper_geometry`. El worker usaba el frame `rg2_pinch_center` como fallback, pero el bridge devuelve `ee_link="rg2_tcp"`. La comparación de strings en la guarda del check EE link (línea ~3486) habría disparado un `RuntimeError` con `ee_link mismatch` si `_ee_frame_effective` fuera `None`.

**Fix:**
```diff
 from ur5_tools.gripper_geometry import (
     RG2_PINCH_CENTER_FRAME,
+    RG2_TCP_FRAME,
     contact_z_correction_for_frame,
     load_gripper_geometry,
 )
```

---

### BUG-M02 — Importación faltante: `tf_ready_status`

**Severidad:** Alta (bloquea el nuevo bloque INIT)  
**Ubicación:** `panel_pick_object.py` línea 70

`tf_ready_status` no estaba importado desde `.panel_readiness`. Necesario para la validación explícita de TF en caliente equivalente a la de DIRECTO.

**Fix:**
```diff
-from .panel_readiness import pick_ui_status
+from .panel_readiness import pick_ui_status, tf_ready_status
```

---

### BUG-M03 — Ausencia de validación explícita TF/EE en el arranque MOVEIT

**Severidad:** Alta  
**Ubicación:** `panel_pick_object.py` después del check de controladores (~línea 292)

DIRECTO tiene 6 fases de `SYNC_GATE` explícitas incluyendo `stage=tf_and_ee`. MOVEIT no tenía ninguna validación de TF antes de lanzar el worker — un TF no listo provocaba fallos silenciosos en `_selection_to_base()` mucho más tarde en el pipeline.

```


### report/runtime_moveit_step_20260424.md


```
# Validación Runtime MOVEIT + STEP — UR5+RG2

**Fecha:** 2026-04-24  
**Rama:** ENTREGA.V2  
**Stack:** ROS 2 Jazzy · Gazebo Sim Harmonic · MoveIt 2 · ros2_control · UR5+OnRobot RG2  
**Estado final:** PARCIAL — infraestructura OK, GUI pendiente + BUG-M08 detectado y corregido

---

## Entorno detectado

```
DISPLAY=localhost:10.0   → no accesible desde proceso Claude (xdpyinfo: unable to open)
WAYLAND_DISPLAY=         → no disponible
XDG_SESSION_TYPE=tty     → sesión no gráfica
```

El panel Qt requiere entorno gráfico para ejecutar MOVEIT en modo AUTO y STEP_BY_STEP.
Las validaciones de comportamiento GUI quedan pendientes de ejecución manual.

---

## Build

```
colcon build --symlink-install --packages-select ur5_qt_panel
→ Finished <<< ur5_qt_panel [1.09s]   PASS
```

Ejecutado desde `/home/laboratorio/TFM/agarre_ros2_ws/` (correcto).

---

## Nodos detectados (stack en ejecución)

`ros2 node list` (con entorno FastDDS correcto):

```
/controller_bootstrap
/controller_manager
/gripper_attach_backend
/gripper_controller
/gz_pose_bridge
/gz_ros_control
/joint_state_broadcaster
/joint_trajectory_controller
/move_group
/move_group/moveit
/panel_superpro          ← panel Qt (en sesión gráfica del usuario)
/panel_tf_helper
/panel_v2_moveit_publisher
/planning_scene_sync
/release_objects_service
/robot_state_publisher
/ros_gz_bridge_main
/system_state_manager
/ur5_moveit_bridge       ← bridge MoveIt activo
/ur5_moveit_py
/world_tf_publisher
```

---

## Controladores activos

```
ros2 control list_controllers:

gripper_controller          forward_command_controller   active  ✓
joint_trajectory_controller joint_trajectory_controller  active  ✓
joint_state_broadcaster     joint_state_broadcaster      active  ✓
```

---

## Topics clave verificados

| Topic | Tipo | Publishers | Subscriptores |
|-------|------|-----------|--------------|
| `/desired_grasp` | `geometry_msgs/PoseStamped` | 0 (panel cerrado) | 2 (bridge×2) |
```


## 18.7 Repro startup último ciclo


### cycle1_pipeline.log


```
NODES=22
ACTION_AVAILABLE=1 (/joint_trajectory_controller/follow_joint_trajectory)
ACTIVE_CONTROLLERS=3
UR5_JOINTS_IN_JOINT_STATES=6
RESULTADO: PASS
POST_STOP_NODES=0
```


### cycle1_post_stop.log


```
[STOP_PANEL_V2] PANEL_CONTROLLER_MANAGER=/controller_manager
[STOP_PANEL_V2] Stopping PID 1751231
```


### cycle1_start.log


```
[START_PANEL_V2] display recuperado automáticamente: DISPLAY=:1 (pid=3425)
[START_PANEL_V2] WS_DIR=/home/laboratorio/TFM/agarre_ros2_ws
[START_PANEL_V2] PANEL_CONTROLLER_MANAGER=/controller_manager
[START_PANEL_V2] cold boot: matando procesos previos...
[START_PANEL_V2] limpiando procesos zombis...
[START_PANEL_V2] cargando entorno ROS 2 jazzy (/home/laboratorio/TFM/agarre_ros2_ws)
[START_PANEL_V2] GZ_SIM_RESOURCE_PATH set to: /home/laboratorio/TFM/agarre_ros2_ws/models:/home/laboratorio/TFM/agarre_ros2_ws/worlds:/home/laboratorio/TFM/agarre_ros2_ws/install:/opt/ros/jazzy/share
[START_PANEL_V2] perfil runtime validado: rg2_tcp_fix_20260422 (/home/laboratorio/TFM/agarre_ros2_ws/scripts/panel_runtime_validated.env)
[start_panel_v2] geometry_sync pick_demo_anchor_joint ok err_m=0.000000
[START_PANEL_V2] Wrapper interno de panel; flujo canónico oficial: ./lanzar_panelv2.sh
[START_PANEL_V2] PANEL_GZ_GUI=1 HEADLESS=false
[START_PANEL_V2] ros2 launch PID=1751231 (pidfile: /home/laboratorio/TFM/agarre_ros2_ws/log/ros2_launch.pid)
[START_PANEL_V2] Para detener: ./scripts/stop_panel_v2.sh
```


### cycle1_stop.log


```
[STOP_PANEL_V2] PANEL_CONTROLLER_MANAGER=/controller_manager
[STOP_PANEL_V2] No PID file at /home/laboratorio/TFM/agarre_ros2_ws/log/ros2_launch.pid
[STOP_PANEL_V2] Parada idempotente: intentando detener procesos residuales.
```


### cycle1_system_diag.log


```
{
  "reason": "Sistema listo",
  "state": "READY",
  "clock_age_sec": 0.0006784469587728381,
  "pose_age_sec": 0.00564413599204272,
  "camera_age_sec": 0.0313317229738459,
  "controllers_ready": true,
  "controllers_reason": "controllers activos",
  "moveit_ready": true,
  "geometry_ok": true,
  "geometry_reason": "ok",
  "geometry_tool_frame": "tool0",
  "geometry_urdf_source": "/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro",
  "geometry_expected_xyz": {
    "rg2_tcp": [
      0.0,
      0.0,
      0.0050885
    ],
    "rg2_pinch_center": [
      0.0,
      0.0,
      0.0050885
    ]
  },
  "geometry_actual_xyz": {
    "rg2_tcp": [
      0.0,
      0.0,
      0.0050885
```


### summary.log


```
[REPRO] ws=/home/laboratorio/TFM/agarre_ros2_ws
[REPRO] cycles=1
[REPRO] out=/home/laboratorio/TFM/agarre_ros2_ws/report/repro_startup/20260423_161929

=== CYCLE 1 START ===
[REPRO] cycle=1 PASS
    "joint_trajectory_controller": "active",
    "joint_state_broadcaster": "active"
  },
  "fatal": false,
  "clock_ok": true,
  "pose_ok": true,
  "camera_ok": true
}
NODES=22
ACTION_AVAILABLE=1 (/joint_trajectory_controller/follow_joint_trajectory)
ACTIVE_CONTROLLERS=3
UR5_JOINTS_IN_JOINT_STATES=6
RESULTADO: PASS
POST_STOP_NODES=0

[REPRO] RESULT=PASS fails=0/1
```


# 19. Runtime Actual

> **Nota:** Esta sección sólo tiene datos si el stack ROS 2 está arrancado durante la generación.


## 19.1 Paquetes instalados (ur5/tfm/panel)

**$ ros2 pkg list

```
action_msgs
action_tutorials_cpp
action_tutorials_interfaces
action_tutorials_py
actionlib_msgs
actuator_msgs
ament_cmake
ament_cmake_auto
ament_cmake_copyright
ament_cmake_core
ament_cmake_cppcheck
ament_cmake_cpplint
ament_cmake_export_definitions
ament_cmake_export_dependencies
ament_cmake_export_include_directories
ament_cmake_export_interfaces
ament_cmake_export_libraries
ament_cmake_export_link_flags
ament_cmake_export_targets
ament_cmake_flake8
ament_cmake_gen_version_h
ament_cmake_gmock
ament_cmake_gtest
ament_cmake_include_directories
ament_cmake_libraries
ament_cmake_lint_cmake
ament_cmake_pep257
ament_cmake_pytest
ament_cmake_python
ament_cmake_ros
ament_cmake_target_dependencies
ament_cmake_test
ament_cmake_uncrustify
ament_cmake_version
ament_cmake_xmllint
ament_copyright
ament_cppcheck
ament_cpplint
ament_flake8
ament_index_cpp
ament_index_python
ament_lint
ament_lint_auto
ament_lint_cmake
ament_lint_common
ament_package
ament_pep257
ament_uncrustify
ament_xmllint
angles
builtin_interfaces
class_loader
common_interfaces
composition
composition_interfaces
compressed_depth_image_transport
compressed_image_transport
console_bridge_vendor
control_msgs
control_toolbox
controller_interface
controller_manager
controller_manager_msgs
cv_bridge
demo_nodes_cpp
demo_nodes_cpp_native
demo_nodes_py
depthimage_to_laserscan
desktop
diagnostic_msgs
diagnostic_updater
domain_coordinator
dummy_map_server
dummy_robot_bringup
dummy_sensors
eigen3_cmake_module
eigen_stl_containers
example_interfaces
examples_rclcpp_minimal_action_client
examples_rclcpp_minimal_action_server
examples_rclcpp_minimal_client
examples_rclcpp_minimal_composition
examples_rclcpp_minimal_publisher
examples_rclcpp_minimal_service
examples_rclcpp_minimal_subscriber
examples_rclcpp_minimal_timer
examples_rclcpp_multithreaded_executor
examples_rclpy_executors
examples_rclpy_minimal_action_client
examples_rclpy_minimal_action_server
examples_rclpy_minimal_client
examples_rclpy_minimal_publisher
examples_rclpy_minimal_service
examples_rclpy_minimal_subscriber
fastrtps_cmake_module
filters
forward_command_controller
generate_parameter_library
generate_parameter_library_py
geometric_shapes
geometry2
geometry_msgs
gps_msgs
gz_cmake_vendor
gz_common_vendor
gz_dartsim_vendor
gz_fuel_tools_vendor
gz_gui_vendor
gz_math_vendor
gz_msgs_vendor
gz_ogre_next_vendor
gz_physics_vendor
gz_plugin_vendor
gz_rendering_vendor
gz_ros2_control
gz_sensors_vendor
gz_sim_vendor
gz_tools_vendor
gz_transport_vendor
gz_utils_vendor
hardware_interface
image_geometry
image_tools
image_transport
image_transport_plugins
interactive_markers
intra_process_demo
joint_limits
joint_state_broadcaster
joint_state_publisher
joint_state_publisher_gui
joint_trajectory_controller
joy
kdl_parser
keyboard_handler
laser_geometry
launch
launch_param_builder
launch_ros
launch_testing
launch_testing_ament_cmake
launch_testing_ros
launch_xml
launch_yaml
libcurl_vendor
liblz4_vendor
libstatistics_collector
libyaml_vendor
lifecycle
lifecycle_msgs
logging_demo
map_msgs
marine_acoustic_msgs
mcap_vendor
message_filters
moveit_common
moveit_configs_utils
moveit_core
moveit_kinematics
moveit_msgs
moveit_planners_ompl
moveit_py
moveit_ros_move_group
moveit_ros_occupancy_map_monitor
moveit_ros_planning
moveit_ros_planning_interface
moveit_ros_robot_interaction
moveit_ros_visualization
moveit_ros_warehouse
moveit_simple_controller_manager
nav_msgs
object_recognition_msgs
octomap_msgs
ompl
orocos_kdl_vendor
osqp_vendor
osrf_pycommon
pal_statistics
pal_statistics_msgs
parameter_traits
pcl_conversions
pcl_msgs
pendulum_control
pendulum_msgs
pluginlib
point_cloud_transport
pybind11_vendor
python_cmake_module
python_orocos_kdl_vendor
python_qt_binding
qt_dotgraph
qt_gui
qt_gui_cpp
qt_gui_py_common
quality_of_service_demo_cpp
quality_of_service_demo_py
random_numbers
rcl
rcl_action
rcl_interfaces
rcl_lifecycle
rcl_logging_interface
rcl_logging_spdlog
rcl_yaml_param_parser
rclcpp
rclcpp_action
rclcpp_components
rclcpp_lifecycle
rclpy
rcpputils
rcutils
realtime_tools
resource_retriever
rmw
rmw_dds_common
rmw_fastrtps_cpp
rmw_fastrtps_shared_cpp
rmw_implementation
rmw_implementation_cmake
robot_state_publisher
ros2_control_cmake
ros2action
ros2bag
ros2cli
ros2cli_common_extensions
ros2component
ros2controlcli
ros2doctor
ros2interface
ros2launch
ros2lifecycle
ros2multicast
ros2node
ros2param
ros2pkg
ros2plugin
ros2run
ros2service
ros2topic
ros_base
ros_core
ros_environment
ros_gz
ros_gz_bridge
ros_gz_image
ros_gz_interfaces
ros_gz_sim
ros_gz_sim_demos
ros_workspace
rosbag2
rosbag2_compression
rosbag2_compression_zstd
rosbag2_cpp
rosbag2_interfaces
rosbag2_py
rosbag2_storage
rosbag2_storage_default_plugins
rosbag2_storage_mcap
rosbag2_storage_sqlite3
rosbag2_transport
rosgraph_msgs
rosidl_adapter
rosidl_cli
rosidl_cmake
rosidl_core_generators
rosidl_core_runtime
rosidl_default_generators
rosidl_default_runtime
rosidl_dynamic_typesupport
rosidl_dynamic_typesupport_fastrtps
rosidl_generator_c
rosidl_generator_cpp
rosidl_generator_py
rosidl_generator_rs
rosidl_generator_type_description
rosidl_parser
rosidl_pycommon
rosidl_runtime_c
rosidl_runtime_cpp
rosidl_runtime_py
rosidl_typesupport_c
rosidl_typesupport_cpp
rosidl_typesupport_fastrtps_c
rosidl_typesupport_fastrtps_cpp
rosidl_typesupport_interface
rosidl_typesupport_introspection_c
rosidl_typesupport_introspection_cpp
rpyutils
rqt_action
rqt_bag
rqt_bag_plugins
rqt_common_plugins
rqt_console
rqt_graph
rqt_gui
rqt_gui_cpp
rqt_gui_py
rqt_image_view
rqt_msg
rqt_plot
rqt_publisher
rqt_py_common
rqt_py_console
rqt_reconfigure
rqt_service_caller
rqt_shell
rqt_srv
rqt_topic
rttest
rviz2
rviz_assimp_vendor
rviz_common
rviz_default_plugins
rviz_ogre_vendor
rviz_rendering
sdformat_urdf
sdformat_vendor
sdl2_vendor
sensor_msgs
sensor_msgs_py
service_msgs
shape_msgs
simulation_interfaces
spdlog_vendor
sqlite3_vendor
srdfdom
sros2
sros2_cmake
statistics_msgs
std_msgs
std_srvs
stereo_msgs
tango_icons_vendor
tcb_span
teleop_twist_joy
teleop_twist_keyboard
tf2
tf2_bullet
tf2_eigen
tf2_eigen_kdl
tf2_geometry_msgs
tf2_kdl
tf2_msgs
tf2_py
tf2_ros
tf2_ros_py
tf2_sensor_msgs
tf2_tools
tfm_grasping
theora_image_transport
tinyxml2_vendor
tl_expected
tlsf
tlsf_cpp
topic_monitor
tracetools
trajectory_msgs
turtlesim
type_description_interfaces
uncrustify_vendor
unique_identifier_msgs
ur5_bringup
ur5_description
ur5_moveit_config
ur5_panel_interfaces
ur5_qt_panel
ur5_tools
ur_description
urdf
urdf_parser_plugin
urdfdom_py
vision_msgs
visualization_msgs
warehouse_ros
xacro
yaml_cpp_vendor
zstd_image_transport
zstd_vendor
```


## 19.2 Executables por paquete


### ur5_qt_panel

**$ ros2 pkg executables ur5_qt_panel

```
ur5_qt_panel main_panel
ur5_qt_panel panel_v2
```


### ur5_tools

**$ ros2 pkg executables ur5_tools

```
ur5_tools clock_probe
ur5_tools controller_bootstrap
ur5_tools find_topdown_rg2_pose
ur5_tools gripper_attach_backend
ur5_tools gz_pose_bridge
ur5_tools gz_ros_control_guard
ur5_tools inspect_rg2_visual_pose
ur5_tools jt_smoke_test
ur5_tools planning_scene_sync
ur5_tools release_objects_service
ur5_tools system_state_manager
ur5_tools tf_probe
ur5_tools ur5_moveit_bridge
ur5_tools validate_dh_vs_tf
ur5_tools world_tf_publisher
```


### tfm_grasping

**$ ros2 pkg executables tfm_grasping

```
tfm_grasping grasp_inference
```


### ur5_bringup

**$ ros2 pkg executables ur5_bringup

```
```


## 19.3 Nodos activos

**$ ros2 node list

```
/controller_bootstrap
/controller_bootstrap
/controller_manager
/gripper_attach_backend
/gripper_attach_backend
/gripper_attach_backend
/gripper_controller
/gz_pose_bridge
/gz_pose_bridge
/gz_ros_control
/joint_state_broadcaster
/joint_trajectory_controller
/move_group
/move_group/moveit
/move_group_private_107378015441552
/moveit_597573625
/moveit_661316211
/moveit_simple_controller_manager
/moveit_simple_controller_manager
/panel_superpro
/panel_tf_helper
/panel_v2_moveit_publisher
/planning_scene_sync
/planning_scene_sync
/release_objects_service
/release_objects_service
/robot_state_publisher
/robot_state_publisher
/ros_gz_bridge
/ros_gz_bridge_main
/system_state_manager
/transform_listener_impl_61a8e49be400
/transform_listener_impl_7ba6ec60f560
/ur5_moveit_bridge
/ur5_moveit_py
/ur5_moveit_py_private_135956858338400
/world_tf_publisher
/world_tf_publisher
```


## 19.4 Topics activos

**$ ros2 topic list

```
/attached_collision_object
/camera_debug_top/depth_image
/camera_debug_top/image
/camera_east/depth_image
/camera_east/image
/camera_north/depth_image
/camera_north/image
/camera_overhead/depth_image
/camera_overhead/image
/camera_south/image
/camera_west/image
/camera_wrist/image
/clock
/collision_object
/controller_manager/activity
/controller_manager/introspection_data/full
/controller_manager/introspection_data/names
/controller_manager/introspection_data/values
/controller_manager/statistics/full
/controller_manager/statistics/names
/controller_manager/statistics/values
/desired_grasp
/desired_grasp/result
/desired_grasp_cartesian
/diagnostics
/display_contacts
/display_planned_path
/drop_anchor/box_blue/attach
/drop_anchor/box_blue/detach
/drop_anchor/box_blue/state
/drop_anchor/box_green/attach
/drop_anchor/box_green/detach
/drop_anchor/box_green/state
/drop_anchor/box_lightblue/attach
/drop_anchor/box_lightblue/detach
/drop_anchor/box_lightblue/state
/drop_anchor/box_red/attach
/drop_anchor/box_red/detach
/drop_anchor/box_red/state
/drop_anchor/box_yellow/attach
/drop_anchor/box_yellow/detach
/drop_anchor/box_yellow/state
/drop_anchor/cross_cyan/attach
/drop_anchor/cross_cyan/detach
/drop_anchor/cross_cyan/state
/drop_anchor/cyl_gray/attach
/drop_anchor/cyl_gray/detach
/drop_anchor/cyl_gray/state
/drop_anchor/cyl_green/attach
/drop_anchor/cyl_green/detach
/drop_anchor/cyl_green/state
/drop_anchor/cyl_orange/attach
/drop_anchor/cyl_orange/detach
/drop_anchor/cyl_orange/state
/drop_anchor/cyl_purple/attach
/drop_anchor/cyl_purple/detach
/drop_anchor/cyl_purple/state
/drop_anchor/pick_demo/attach
/drop_anchor/pick_demo/detach
/drop_anchor/pick_demo/state
/dynamic_joint_states
/grasp_pose
/grasp_rect
/gripper/box_blue/attach
/gripper/box_blue/detach
/gripper/box_blue/state
/gripper/box_green/attach
/gripper/box_green/detach
/gripper/box_green/state
/gripper/box_lightblue/attach
/gripper/box_lightblue/detach
/gripper/box_lightblue/state
/gripper/box_red/attach
/gripper/box_red/detach
/gripper/box_red/state
/gripper/box_yellow/attach
/gripper/box_yellow/detach
/gripper/box_yellow/state
/gripper/cross_cyan/attach
/gripper/cross_cyan/detach
/gripper/cross_cyan/state
/gripper/cyl_gray/attach
/gripper/cyl_gray/detach
/gripper/cyl_gray/state
/gripper/cyl_green/attach
/gripper/cyl_green/detach
/gripper/cyl_green/state
/gripper/cyl_orange/attach
/gripper/cyl_orange/detach
/gripper/cyl_orange/state
/gripper/cyl_purple/attach
/gripper/cyl_purple/detach
/gripper/cyl_purple/state
/gripper/pick_demo/attach
/gripper/pick_demo/detach
/gripper/pick_demo/state
/gripper_anchor/box_blue/attach
/gripper_anchor/box_blue/detach
/gripper_anchor/box_blue/state
/gripper_anchor/box_green/attach
/gripper_anchor/box_green/detach
/gripper_anchor/box_green/state
/gripper_anchor/box_lightblue/attach
/gripper_anchor/box_lightblue/detach
/gripper_anchor/box_lightblue/state
/gripper_anchor/box_red/attach
/gripper_anchor/box_red/detach
/gripper_anchor/box_red/state
/gripper_anchor/box_yellow/attach
/gripper_anchor/box_yellow/detach
/gripper_anchor/box_yellow/state
/gripper_anchor/cross_cyan/attach
/gripper_anchor/cross_cyan/detach
/gripper_anchor/cross_cyan/state
/gripper_anchor/cyl_gray/attach
/gripper_anchor/cyl_gray/detach
/gripper_anchor/cyl_gray/state
/gripper_anchor/cyl_green/attach
/gripper_anchor/cyl_green/detach
/gripper_anchor/cyl_green/state
/gripper_anchor/cyl_orange/attach
/gripper_anchor/cyl_orange/detach
/gripper_anchor/cyl_orange/state
/gripper_anchor/cyl_purple/attach
/gripper_anchor/cyl_purple/detach
/gripper_anchor/cyl_purple/state
/gripper_anchor/pick_demo/attach
/gripper_anchor/pick_demo/detach
/gripper_anchor/pick_demo/state
/gripper_controller/commands
/gripper_controller/transition_event
/joint_state_broadcaster/transition_event
/joint_states
/joint_trajectory_controller/controller_state
/joint_trajectory_controller/joint_trajectory
/joint_trajectory_controller/speed_scaling_input
/joint_trajectory_controller/transition_event
/monitored_planning_scene
/panel/pick_object
/panel/recover
/panel/select_object
/panel/tfm_execute
/panel/tfm_infer
/parameter_events
/pipeline_state
/planning_scene
/planning_scene_world
/robot_description
/rosout
/system_diag
/system_state
/tf
/tf_static
/trajectory_execution_event
/ur5_moveit_bridge/heartbeat
/world/ur5_mesa_objetos/pose/info
/world/ur5_mesa_objetos/pose/info_array
/world/ur5_mesa_objetos/pose/info_raw
```


## 19.5 Services activos

**$ ros2 service list

```
/apply_planning_scene
/check_state_validity
/clear_octomap
/compute_cartesian_path
/compute_fk
/compute_ik
/controller_bootstrap/describe_parameters
/controller_bootstrap/get_parameter_types
/controller_bootstrap/get_parameters
/controller_bootstrap/get_type_description
/controller_bootstrap/list_parameters
/controller_bootstrap/run
/controller_bootstrap/set_parameters
/controller_bootstrap/set_parameters_atomically
/controller_manager/cleanup_controller
/controller_manager/configure_controller
/controller_manager/describe_parameters
/controller_manager/get_logger_levels
/controller_manager/get_parameter_types
/controller_manager/get_parameters
/controller_manager/get_type_description
/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/list_hardware_components
/controller_manager/list_hardware_interfaces
/controller_manager/list_parameters
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/set_hardware_component_state
/controller_manager/set_logger_levels
/controller_manager/set_parameters
/controller_manager/set_parameters_atomically
/controller_manager/switch_controller
/controller_manager/unload_controller
/get_planner_params
/get_planning_scene
/get_urdf
/gripper_attach_backend/describe_parameters
/gripper_attach_backend/get_parameter_types
/gripper_attach_backend/get_parameters
/gripper_attach_backend/get_type_description
/gripper_attach_backend/list_parameters
/gripper_attach_backend/set_parameters
/gripper_attach_backend/set_parameters_atomically
/gripper_controller/describe_parameters
/gripper_controller/get_logger_levels
/gripper_controller/get_parameter_types
/gripper_controller/get_parameters
/gripper_controller/get_type_description
/gripper_controller/list_parameters
/gripper_controller/set_logger_levels
/gripper_controller/set_parameters
/gripper_controller/set_parameters_atomically
/gz_pose_bridge/describe_parameters
/gz_pose_bridge/get_parameter_types
/gz_pose_bridge/get_parameters
/gz_pose_bridge/get_type_description
/gz_pose_bridge/list_parameters
/gz_pose_bridge/set_parameters
/gz_pose_bridge/set_parameters_atomically
/gz_ros_control/describe_parameters
/gz_ros_control/get_parameter_types
/gz_ros_control/get_parameters
/gz_ros_control/get_type_description
/gz_ros_control/list_parameters
/gz_ros_control/set_parameters
/gz_ros_control/set_parameters_atomically
/joint_state_broadcaster/describe_parameters
/joint_state_broadcaster/get_logger_levels
/joint_state_broadcaster/get_parameter_types
/joint_state_broadcaster/get_parameters
/joint_state_broadcaster/get_type_description
/joint_state_broadcaster/list_parameters
/joint_state_broadcaster/set_logger_levels
/joint_state_broadcaster/set_parameters
/joint_state_broadcaster/set_parameters_atomically
/joint_trajectory_controller/describe_parameters
/joint_trajectory_controller/get_logger_levels
/joint_trajectory_controller/get_parameter_types
/joint_trajectory_controller/get_parameters
/joint_trajectory_controller/get_type_description
/joint_trajectory_controller/list_parameters
/joint_trajectory_controller/query_state
/joint_trajectory_controller/set_logger_levels
/joint_trajectory_controller/set_parameters
/joint_trajectory_controller/set_parameters_atomically
/load_geometry_from_file
/load_map
/move_group/describe_parameters
/move_group/get_parameter_types
/move_group/get_parameters
/move_group/get_type_description
/move_group/list_parameters
/move_group/moveit/describe_parameters
/move_group/moveit/get_parameter_types
/move_group/moveit/get_parameters
/move_group/moveit/get_type_description
/move_group/moveit/list_parameters
/move_group/moveit/set_parameters
/move_group/moveit/set_parameters_atomically
/move_group/set_parameters
/move_group/set_parameters_atomically
/move_group_private_107378015441552/describe_parameters
/move_group_private_107378015441552/get_parameter_types
/move_group_private_107378015441552/get_parameters
/move_group_private_107378015441552/get_type_description
/move_group_private_107378015441552/list_parameters
/move_group_private_107378015441552/set_parameters
/move_group_private_107378015441552/set_parameters_atomically
/moveit_597573625/describe_parameters
/moveit_597573625/get_parameter_types
/moveit_597573625/get_parameters
/moveit_597573625/get_type_description
/moveit_597573625/list_parameters
/moveit_597573625/set_parameters
/moveit_597573625/set_parameters_atomically
/moveit_661316211/describe_parameters
/moveit_661316211/get_parameter_types
/moveit_661316211/get_parameters
/moveit_661316211/get_type_description
/moveit_661316211/list_parameters
/moveit_661316211/set_parameters
/moveit_661316211/set_parameters_atomically
/moveit_simple_controller_manager/describe_parameters
/moveit_simple_controller_manager/get_parameter_types
/moveit_simple_controller_manager/get_parameters
/moveit_simple_controller_manager/get_type_description
/moveit_simple_controller_manager/list_parameters
/moveit_simple_controller_manager/set_parameters
/moveit_simple_controller_manager/set_parameters_atomically
/panel/camera_connect
/panel/camera_disconnect
/panel/pick_demo
/panel/pick_object
/panel/recover
/panel/select_object
/panel/tfm_execute
/panel/tfm_infer
/panel_superpro/describe_parameters
/panel_superpro/get_parameter_types
/panel_superpro/get_parameters
/panel_superpro/get_type_description
/panel_superpro/list_parameters
/panel_superpro/set_parameters
/panel_superpro/set_parameters_atomically
/panel_tf_helper/describe_parameters
/panel_tf_helper/get_parameter_types
/panel_tf_helper/get_parameters
/panel_tf_helper/get_type_description
/panel_tf_helper/list_parameters
/panel_tf_helper/set_parameters
/panel_tf_helper/set_parameters_atomically
/panel_tf_topics/get_type_description
/panel_v2_moveit_publisher/describe_parameters
/panel_v2_moveit_publisher/get_parameter_types
/panel_v2_moveit_publisher/get_parameters
/panel_v2_moveit_publisher/get_type_description
/panel_v2_moveit_publisher/list_parameters
/panel_v2_moveit_publisher/set_parameters
/panel_v2_moveit_publisher/set_parameters_atomically
/plan_kinematic_path
/planning_scene_sync/describe_parameters
/planning_scene_sync/get_parameter_types
/planning_scene_sync/get_parameters
/planning_scene_sync/get_type_description
/planning_scene_sync/list_parameters
/planning_scene_sync/set_parameters
/planning_scene_sync/set_parameters_atomically
/query_planner_interface
/release_objects
/release_objects_service/describe_parameters
/release_objects_service/get_parameter_types
/release_objects_service/get_parameters
/release_objects_service/get_type_description
/release_objects_service/list_parameters
/release_objects_service/set_parameters
/release_objects_service/set_parameters_atomically
/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/get_type_description
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
/ros_gz_bridge/describe_parameters
/ros_gz_bridge/get_parameter_types
/ros_gz_bridge/get_parameters
/ros_gz_bridge/get_type_description
/ros_gz_bridge/list_parameters
/ros_gz_bridge/set_parameters
/ros_gz_bridge/set_parameters_atomically
/ros_gz_bridge_main/describe_parameters
/ros_gz_bridge_main/get_parameter_types
/ros_gz_bridge_main/get_parameters
/ros_gz_bridge_main/get_type_description
/ros_gz_bridge_main/list_parameters
/ros_gz_bridge_main/set_parameters
/ros_gz_bridge_main/set_parameters_atomically
/save_geometry_to_file
/save_map
/set_planner_params
/system_state_manager/describe_parameters
/system_state_manager/get_parameter_types
/system_state_manager/get_parameters
/system_state_manager/get_type_description
/system_state_manager/list_parameters
/system_state_manager/set_parameters
/system_state_manager/set_parameters_atomically
/transform_listener_impl_61a8e49be400/get_type_description
/transform_listener_impl_7ba6ec60f560/get_type_description
/ur5_moveit_bridge/describe_parameters
/ur5_moveit_bridge/get_parameter_types
/ur5_moveit_bridge/get_parameters
/ur5_moveit_bridge/get_type_description
/ur5_moveit_bridge/list_parameters
/ur5_moveit_bridge/set_parameters
/ur5_moveit_bridge/set_parameters_atomically
/ur5_moveit_py/describe_parameters
/ur5_moveit_py/get_parameter_types
/ur5_moveit_py/get_parameters
/ur5_moveit_py/get_type_description
/ur5_moveit_py/list_parameters
/ur5_moveit_py/set_parameters
/ur5_moveit_py/set_parameters_atomically
/ur5_moveit_py_private_135956858338400/describe_parameters
/ur5_moveit_py_private_135956858338400/get_parameter_types
/ur5_moveit_py_private_135956858338400/get_parameters
/ur5_moveit_py_private_135956858338400/get_type_description
/ur5_moveit_py_private_135956858338400/list_parameters
/ur5_moveit_py_private_135956858338400/set_parameters
/ur5_moveit_py_private_135956858338400/set_parameters_atomically
/world_tf_publisher/describe_parameters
/world_tf_publisher/get_parameter_types
/world_tf_publisher/get_parameters
/world_tf_publisher/get_type_description
/world_tf_publisher/list_parameters
/world_tf_publisher/set_parameters
/world_tf_publisher/set_parameters_atomically
```


## 19.6 Actions activas

**$ ros2 action list

```
/execute_trajectory
/joint_trajectory_controller/follow_joint_trajectory
/move_action
```


## 19.7 Controladores

**$ ros2 control list_controllers

```
gripper_controller          forward_command_controller/ForwardCommandController    active
joint_trajectory_controller joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster          active
```


## 19.8 TF frames

**$ timeout 5 ros2 run tf2_tools view_frames --ros-args -p output_file:=/tmp/tf_frames_bk

```
```


# 20. Bugs Conocidos y Estado

| Bug | Síntoma | Causa raíz | Fix aplicado | Archivo | Estado | Fecha |
|-----|---------|------------|--------------|---------|--------|-------|
| Doble offset Z | TCP 50mm erróneo | GRIPPER_TCP_Z_OFFSET=0.05 aplicado a rg2_pinch_center | _DIRECTO_GRASP_Z=0.0 | panel_pick_demo.py | Resuelto | 2026-04-08 |
| False positive grasp | min_lift_delta demasiado bajo | 0.025 m | 0.025→0.060 | panel_pick_demo.py | Resuelto | 2026-04-07 |
| APPROACH_COARSE_NOT_READY | H1=overshoot 18mm IK | ik_err_tol=0.035 demasiado estricto | approach_clearance_ok gate | ur5_moveit_bridge.py | Resuelto | 2026-04-15 |
| SDF pitch=−π/2 | 247mm error visual TCP | end_effector_frame_fixed_joint rpy=(-π/2,0,0) | rpy=0 + ur5_hand_joint Z=0.07 | model.sdf | Resuelto | 2026-04-25 |
| RG2 revoluto legacy | RuntimeError RELEASE | PANEL_GRIPPER_OPEN_RAD=0.5 (revoluto) en prismático | 0.5→0.055 m | start_panel_v2.sh | Resuelto | 2026-04-25 |
| ee_link mismatch MoveIt | Ciclos MOVEIT bloqueados | ee_link incorrecto en bridge | ee_link=rg2_pinch_center | ur5_moveit_bridge.py | Resuelto | 2026-04-25 |
| NEGATE_XY dead code | — | Var inactiva | Eliminada | panel_pick_demo.py | Resuelto | 2026-04-16 |
| RSP muerto sin guard | TF stale, TCP OK falso | RSP sin respawn | Plan FASE A-E | ur5_stack.launch.py | Pendiente | 2026-04-24 |
| detachable_shadow_follow=False | CARRY fallaba | Bloqueaba follow loop cinemático | =True | panel_pick_demo.py | Resuelto | 2026-04-19 |
| DH/SDF divergencia 343mm | Benchmarks con artefactos | SDF offsets erróneos | 9 fixes SDF Opción 3 | model.sdf | Resuelto | 2026-04-23 |
| **10 merge conflicts (UU)** | Build puede fallar | Merge incompleto ENTREGA.V3 | **PENDIENTE** | ver git status | **Activo** | 2026-04-26 |

# 21. Troubleshooting


## Panel no arranca

**Diagnóstico:** `bash scripts/smoke_test.sh`
**Causas comunes:** imports rotos, entry_points no instalados, Qt no disponible.
**Fix:** `colcon build && source install/setup.bash && ros2 run ur5_qt_panel panel_v2`

## Panel arranca pero botones no funcionan

**Diagnóstico:** revisar callbacks en panel_v2.py, verificar que el stack está arrancado.
**Causa:** nodos backend no activos (ur5_moveit_bridge, gripper_attach_backend).
**Fix:** arrancar stack completo con `bash scripts/start_panel_v2.sh`

## Sistema no llega a READY

**Diagnóstico:** `bash scripts/diag_startup_health.sh`
**Causa:** TF no fresco, controladores inactivos, MoveIt no listo.
**Fix:** verificar `ros2 control list_controllers`, `ros2 topic echo /tf_static`

## Merge conflicts bloquean el build

**Diagnóstico:** `git status --short | grep UU`
**Fix:** resolver cada archivo UU con `git checkout --ours` o `git checkout --theirs` + `git add`
**Archivos afectados:**

```
Ninguno
```


## Gripper no cierra / error RELEASE

**Causa:** PANEL_GRIPPER_OPEN_RAD con valor legacy revoluto (0.5) en sistema prismático.
**Fix:** verificar `PANEL_GRIPPER_OPEN_RAD=0.055` en scripts/start_panel_v2.sh

## TCP incorrecto / grasp falla

**Causa:** rg2_pinch_center no apunta a 0.0050885 m desde tool0.
**Fix:** verificar SRDF tip_link=rg2_pinch_center, URDF rg2_contact_tcp_xyz=0.0050885

## MoveIt no responde

**Diagnóstico:** `ros2 action list | grep follow`
**Causa:** move_group no arrancado o controladores inactivos.
**Fix:** `ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py`

## TF roto / stale

**Diagnóstico:** `ros2 run ur5_tools tf_probe`
**Causa:** RSP muerto, world_tf_publisher no arrancado.
**Fix:** verificar `ros2 node list | grep robot_state_publisher`

# 22. Discrepancias Abiertas

| Área | Descripción | Riesgo | Estado |
|------|-------------|--------|--------|
| Merge conflicts | 10 archivos UU sin resolver | Alto | Activo |
| kinematics.yaml | CachedKDL vs plain KDL en conflicto | Medio | Activo |
| PANEL_GRIPPER_OPEN_RAD | Dos valores en start_panel_v2.sh (0.0425 y 0.055) | Medio | Activo (merge) |
| RSP sin guard/respawn | TF puede quedar stale sin reinicio | Medio | Pendiente |
| panel_settings.py merge | Dos valores gripper_open_rad (0.0425/0.055) | Medio | Activo (merge) |

# 23. Próximos Pasos Recomendados

1. **[CRÍTICO] Resolver los 10 conflictos de merge (UU)** antes de cualquier otra acción.
   `git status --short | grep UU` — resolver uno a uno.
2. **Verificar build limpio** tras resolver conflictos: `colcon build --symlink-install`
3. **Ejecutar smoke test**: `bash scripts/smoke_test.sh`
4. **Validar runtime completo**: `bash scripts/validate_startup_repro.sh`
5. **Validar 3 ciclos pick**: `bash scripts/validate_pick_3_cycles.sh`
6. **Añadir guard/respawn para RSP** en ur5_stack.launch.py (issue pendiente).
7. **Actualizar ARCHITECTURE.md** post-merge para reflejar estado final.

# 24. Apéndices


## 24.1 Índice de fuentes verificadas


```
attach-gate-fns
directo-gate-fns
directo-geom-fns
executables-tfm_grasping
executables-ur5_bringup
executables-ur5_qt_panel
executables-ur5_tools
git-log
git-status
grasp-inference-fns
grasp-module-fns
gripper-geom-consts
gz-control-guard-fns
gz-pose-bridge-fns
historico-list
kinematics-fns
launcher-env
launch-helpers-fns
launch-nodes
log/attach_backend.log
log/ros2_launch.log
log/world_tf_publisher.log
models-list
models/ur5_rg2/model.sdf
models/ur5_rg2/ur5_controllers.yaml
moveit-bridge-fns
moveit-utils-fns
panel-config-consts
panel-settings-fields
panel-v2-callbacks
pick-demo-fns
pick-geom-fns
pick-obj-fns
planning-scene-fns
readiness-fns
report/auditoria_moveit_step_20260424.md
reports-list
report/repro_startup/20260423_161929/cycle1_pipeline.log
report/repro_startup/20260423_161929/cycle1_post_stop.log
report/repro_startup/20260423_161929/cycle1_start.log
report/repro_startup/20260423_161929/cycle1_stop.log
report/repro_startup/20260423_161929/cycle1_system_diag.log
report/repro_startup/20260423_161929/summary.log
report/runtime_moveit_step_20260424.md
ros2-action-list
ros2-control-list
ros2-node-list
ros2-pkg-list
ros2-service-list
ros2-tf-frames
ros2-topic-list
scripts/bridge_cameras.yaml
scripts-list
scripts/panel_runtime_validated.env
scripts/smoke_test.sh
scripts/validate_before_demo.sh
scripts/validate_pick_3_cycles.sh
scripts/validate_startup_repro.sh
sdf-gripper-geom
sdf-joints
setup-tfm_grasping
setup-ur5_qt_panel
setup-ur5_tools
src/tfm_grasping
src/tfm_grasping/package.xml
src/tfm_grasping/tfm_grasping/config.py
src-tree
src/ur5_bringup
src/ur5_bringup/config/ur5_mock_controllers.yaml
src/ur5_bringup/launch/ur5_rsp.launch.py
src/ur5_bringup/launch/ur5_stack.launch.py
src/ur5_bringup/package.xml
src/ur5_description
src/ur5_description/config/ur5_controllers.yaml
src/ur5_description/package.xml
src/ur5_description/urdf/ur5.urdf.xacro
src/ur5_moveit_config
src/ur5_moveit_config/config/joint_limits.yaml
src/ur5_moveit_config/config/kinematics.yaml
src/ur5_moveit_config/config/moveit_controllers.yaml
src/ur5_moveit_config/config/ompl_planning.yaml
src/ur5_moveit_config/config/planning_scene_monitor_parameters.yaml
src/ur5_moveit_config/config/ur5.srdf
src/ur5_moveit_config/config/ur5_strict.srdf
src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py
src/ur5_moveit_config/package.xml
src/ur5_panel_interfaces
src/ur5_panel_interfaces/package.xml
src/ur5_qt_panel
src/ur5_qt_panel/package.xml
src/ur5_qt_panel/ur5_qt_panel/panel_config.py
src/ur5_qt_panel/ur5_qt_panel/panel_settings.py
src/ur5_tools
src/ur5_tools/package.xml
state-machine-fns
step-pipeline-fns
tfm-geom-fns
tfm-ros-fns
ur5tools-clock_probe
ur5tools-controller_bootstrap
ur5tools-cycle_logger
ur5tools-gripper_attach_backend
ur5tools-gripper_geometry
ur5tools-jt_smoke_test
ur5tools-release_objects_service
ur5tools-system_state_manager
ur5tools-tf_probe
ur5tools-world_tf_publisher
urdf-joints
urdf-props
worlds-list
worlds/ur5_debug_empty.sdf
worlds/ur5_mesa_objetos.sdf
world-tf-offsets
```


## 24.2 Estadísticas de generación


```
Fecha               : 2026-04-26 22:52:40
Líneas en MD final  : 7135
Fuentes verificadas : 114
Paquetes ROS 2      : 7
Entry points        : 3 bloques (ver Sec 5)
Launch files        : 4
Módulos panel       : 71
Tests (archivos)    : 33
Conflicts (UU)      : 0
0
Bugs documentados   : 11
```


## 24.3 Referencia a base anterior

- Ubicación: `historico/2026-04-18_base_conocimiento_tecnica.md` (MD)
- PDF: `historico/2026-04-18_base_conocimiento_tecnica.pdf`
- HTML: `historico/2026-04-18_base_conocimiento_tecnica.html`
- Auditoría arquitectura: `historico/2026-04-17_auditoria_arquitectura_completa.md`

## 24.4 Comandos ejecutados en esta generación


```
git log --oneline -30
git status --short
find src -maxdepth 3 -type f -name *.py -o -name *.xml -o -name *.yaml -o -name *.launch.py -o -name *.srdf
find scripts -maxdepth 1 -type f
find models -maxdepth 4 -type f
find worlds -maxdepth 2 -type f
find report -maxdepth 4 -type f
find historico -maxdepth 3 -type f
ros2 pkg list
ros2 pkg executables ur5_qt_panel
ros2 pkg executables ur5_tools
ros2 pkg executables tfm_grasping
ros2 pkg executables ur5_bringup
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 control list_controllers
timeout 5 ros2 run tf2_tools view_frames --ros-args -p output_file:=/tmp/tf_frames_bk
```

