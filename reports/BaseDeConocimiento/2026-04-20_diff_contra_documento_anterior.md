# Diff contra documento anterior

```diff
--- reports/BaseDeConocimiento/2026-04-20_base_conocimiento_tecnica_TFM.md
+++ 2026-04-20_base_conocimiento_tecnica_TFM.md
@@ -12,10 +12,6 @@
 - Recuperación histórica: se reincorporan arquitectura extendida, tabla amplia de frames, geometría, flujo fase por fase, inventario amplio de variables, controladores/topics, bugs legacy y troubleshooting operativo de 2026-04-18.
 - Discrepancias abiertas resaltadas: world->base_link actual=-0.85 0 0.850 frente a simplificaciones históricas; attach backend launch=0.08 / wrapper runtime=0.06; max_pose_age launch=1.5 / wrapper runtime=2.5.
 - Regla de trazabilidad aplicada: cuando una fuente histórica difiere de la actual, se conserva como histórico/documentado previamente en lugar de borrarla.
-
-### Bloque vigente preservado (base actual)
-
-- No disponible en las fuentes cargadas.
 
 ### Detalle histórico recuperado (2026-04-18)
 
@@ -134,77 +130,6 @@
 - annotate conflicts instead of dropping them
 - never compress specialized diagnostics into generic summaries
 
-### Bloque vigente preservado (fuentes de 2026-04-20)
-
-#### 2.1 Prioridad de fuentes aplicada
-
-1. Código fuente inspeccionado en esta ejecución.
-2. Logs y evidencias runtime locales actuales.
-3. Reports recientes del workspace.
-4. Base 2026-04-18 como referencia histórica estructural.
-
-#### 2.2 Archivos y artefactos usados en esta generación
-
-- `agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py`
-- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`
-- `agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py`
-- `agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro`
-- `agarre_ros2_ws/models/ur5_rg2/model.sdf`
-- `agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf`
-- `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml`
-- `agarre_ros2_ws/src/ur5_qt_panel/config/panel_settings.yaml`
-- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py`
-- `agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py`
-- `agarre_ros2_ws/scripts/start_panel_v2.sh`
-- `agarre_ros2_ws/src/ur5_bringup/package.xml`
-- `agarre_ros2_ws/src/ur5_description/package.xml`
-- `agarre_ros2_ws/src/ur5_moveit_config/package.xml`
-- `agarre_ros2_ws/src/ur5_qt_panel/package.xml`
-- `agarre_ros2_ws/src/tfm_grasping/package.xml`
-- `agarre_ros2_ws/src/ur5_tools/package.xml`
-- `agarre_ros2_ws/src/ur5_panel_interfaces/package.xml`
-- `agarre_ros2_ws/src/ur5_qt_panel/setup.py`
-- `agarre_ros2_ws/src/ur5_tools/setup.py`
-- `agarre_ros2_ws/src/tfm_grasping/setup.py`
-- `reports/BaseDeConocimiento/2026-04-20_base_conocimiento_tecnica_TFM.md`
-- `agarre_ros2_ws/historico/2026-04-18_base_conocimiento_tecnica.md`
-- `reports/BaseDeConocimiento/2026-04-18_base_conocimiento_tecnica_TFM.pdf`
-- `auditoria/informe_fix_visual_grasp_20260419.md`
-- `reports/evidence/ros2/moveit2_system_status.json`
-- `auditoria/spatial_20260418/directo_validation_20260418_201902/helper.log`
-- `auditoria/spatial_20260419/directo_validation_20260419_164750/helper.log`
-- `auditoria/spatial_20260419/directo_validation_20260419_181539/helper.log`
-- `auditoria/spatial_20260419/directo_validation_20260419_184049/stack.log`
-- `auditoria/spatial_20260419/directo_validation_20260419_164750/stack.log`
-
-#### 2.3 Política de merge aplicada
-
-- append, don’t replace
-- prefer current for runtime truth
-- preserve historical structure when still useful
-- annotate conflicts instead of dropping them
-- never compress specialized diagnostics into generic summaries
-
-#### Bloque vigente preservado (fuentes de 2026-04-20)
-
-##### 2.1 Código fuente inspeccionado
-
-- agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
-- agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
-- agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py
-- agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro
-- agarre_ros2_ws/models/ur5_rg2/model.sdf
-
-##### 2.2 Auditorías, histórico y reports usados
-
-- auditoria/informe_fix_visual_grasp_20260419.md
-- auditoria/spatial_20260418/directo_validation_20260418_201902/helper.log
-- auditoria/spatial_20260419/directo_validation_20260419_164750/helper.log
-- auditoria/spatial_20260419/directo_validation_20260419_184049/stack.log
-- auditoria/spatial_20260419/directo_validation_20260419_164750/stack.log
-- reports/evidence/ros2/moveit2_system_status.json
-- reports/2026-04-18_base_conocimiento_tecnica_TFM.pdf
-
 ## 3. Arquitectura del Proyecto
 
 ### 3.1 Paquetes ROS 2 del workspace y responsabilidad
@@ -287,100 +212,9 @@
 
 `panel_pick_demo.py` → `/desired_grasp/request` → `ur5_moveit_bridge` → `move_group.computeCartesianPath()/execute()` → `/desired_grasp/result` → panel. El bridge se lanza ya en el stack principal y no sólo on-demand.
 
-### Bloque vigente preservado (arquitectura resumida actual)
-
-#### 3.1 Paquetes ROS 2 del workspace y responsabilidad
-
-| Paquete | Ruta | Responsabilidad / descripción |
-|---|---|---|
-| `ur5_bringup` | `agarre_ros2_ws/src/ur5_bringup` | Bringup launch files for UR5 simulation/control. |
-| `ur5_description` | `agarre_ros2_ws/src/ur5_description` | UR5 robot description (URDF/Xacro and controllers config). |
-| `ur5_moveit_config` | `agarre_ros2_ws/src/ur5_moveit_config` | Minimal MoveIt 2 configuration for the UR5 robot. |
-| `ur5_qt_panel` | `agarre_ros2_ws/src/ur5_qt_panel` | Qt panel for UR5 simulation control, cameras, and evidence capture. |
-| `tfm_grasping` | `agarre_ros2_ws/src/tfm_grasping` | TFM grasping module (perception, grasp representation, ROS publishing). |
-| `ur5_tools` | `agarre_ros2_ws/src/ur5_tools` | Utilities and helper nodes for UR5 simulation (MoveIt bridge, release service). |
-| `ur5_panel_interfaces` | `agarre_ros2_ws/src/ur5_panel_interfaces` | Interfaces ROS 2 para triggers fiables del panel UR5. |
-
-#### 3.2 Nodos principales y función
-
-| Nodo / entry point | Fuente | Función |
-|---|---|---|
-| `robot_state_publisher` | `built-in` | Publica robot_description y árbol TF semántico. |
-| `gz_sim` | `built-in` | Motor de simulación Gazebo. |
-| `ros_gz_bridge` | `built-in` | Puente Gazebo ↔ ROS 2 para clock, cámaras y pose/info. |
-| `joint_state_broadcaster` | `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml` | Publica /joint_states del robot y gripper. |
-| `joint_trajectory_controller` | `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml` | Ejecuta trayectorias del brazo UR5. |
-| `gripper_controller` | `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml` | Recibe comandos de apertura/cierre de RG2. |
-| `controller_bootstrap` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.controller_bootstrap:main |
-| `gz_pose_bridge` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.gz_pose_bridge:main |
-| `gz_ros_control_guard` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.gz_ros_control_guard:main |
-| `gripper_attach_backend` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.gripper_attach_backend:main |
-| `planning_scene_sync` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.planning_scene_sync:main |
-| `ur5_moveit_bridge` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.ur5_moveit_bridge:main |
-| `release_objects_service` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.release_objects_service:main |
-| `system_state_manager` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.system_state_manager:main |
-| `world_tf_publisher` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.world_tf_publisher:main |
-| `tf_probe` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.tf_probe:main |
-| `clock_probe` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.clock_probe:main |
-| `jt_smoke_test` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.jt_smoke_test:main |
-| `panel_v2` | `agarre_ros2_ws/src/ur5_qt_panel/setup.py` | Entry point Python: ur5_qt_panel.panel_v2:main |
-| `grasp_inference` | `agarre_ros2_ws/src/tfm_grasping/setup.py` | Entry point Python: tfm_grasping.grasp_inference:main |
-
-#### 3.3 Launch files relevantes y qué lanza cada uno
-
-| Archivo | Ruta | Qué lanza |
-|---|---|---|
-| `ur5_stack.launch.py` | `agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py` | Launch principal: Gazebo, RSP, bridges, attach backend, scene sync, bridge MoveIt y panel. |
-| `ur5_rsp.launch.py` | `agarre_ros2_ws/src/ur5_bringup/launch/ur5_rsp.launch.py` | Robot state publisher del UR5/RG2. |
-| `ur5_ros2_control.launch.py` | `agarre_ros2_ws/src/ur5_bringup/launch/ur5_ros2_control.launch.py` | Bringup de ros2_control y controller_manager. |
-| `ur5_moveit_bringup.launch.py` | `agarre_ros2_ws/src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py` | Bringup de MoveIt 2 / move_group. |
-
-#### 3.4 Qué archivo controla cada parte
-
-| Parte del sistema | Archivo de control |
-|---|---|
-| Frames TF del robot | `agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro` |
-| Modelo físico Gazebo | `agarre_ros2_ws/models/ur5_rg2/model.sdf` |
-| Mundo de simulación | `agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf` |
-| Controladores ros2_control | `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml` |
-| Variables de entorno del pick demo | `agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py` |
-| Overrides runtime del panel | `agarre_ros2_ws/scripts/start_panel_v2.sh` |
-| Lógica de fases del pick | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` |
-| Gate de attach | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py` |
-| Backend attach/transporte | `agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py` |
-| TF world -> base_link | `agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py` |
-| Geometría vertical del pick | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py` |
-| Interfaz Qt / orquestación | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py` |
-
-#### 3.5 Relación entre módulos y flujo de comandos
-
-- panel_v2 orquesta interacción de usuario, estados UI y disparo del ciclo.
-- panel_pick_demo ejecuta fases, targets, settle, gates y validación física del carry.
-- ur5_kinematics aplica IK/FK directo en base_link_inertia; el panel traduce desde base_link con NEGATE_XY.
-- ur5_moveit_bridge publica/consume desired_grasp para el camino MoveIt.
-- joint_trajectory_controller y gripper_controller ejecutan los comandos físicos en Gazebo.
-- gripper_attach_backend decide attach_route_decision entre follow_tcp, tool_anchor y demo_transport/world_locked según objeto y configuración.
-
-#### 3.6 Flujo panel → IK → controller
-
-`panel_pick_demo.py` → `_move_tcp_direct()` / `_send_ik_motion()` → `ur5_kinematics.ik_ur5()` → publicación a `/joint_trajectory_controller/joint_trajectory` → `joint_trajectory_controller` → `/joint_states` → verificación de convergencia y settle.
-
-#### 3.7 Flujo panel → MoveIt bridge → move_group → resultado
-
-`panel_pick_demo.py` → `/desired_grasp/request` → `ur5_moveit_bridge` → `move_group.computeCartesianPath()/execute()` → `/desired_grasp/result` → panel. El bridge se lanza ya en el stack principal y no sólo on-demand.
-
-#### Bloque vigente preservado (arquitectura resumida actual)
-
-- El launch del stack publica el entorno de pick directo y lanza el backend de attach cuando launch_attach_backend=true.
-- El panel ejecuta la secuencia de grasp, cierre, attach, lift, carry, transporte a cesta y release.
-- El backend implementa dos comportamientos conceptuales distintos:
-  - follow_tcp como modo general de attach cinemático.
-  - demo_transport para pick_demo, con rama world_locked activa por defecto.
-- Según reports/evidence/ros2/moveit2_system_status.json, Mueve fisicamente objetos siguiendo rg2_tcp en modo follow_tcp cuando esta habilitado., lo que refuerza que follow_tcp sigue siendo la semántica base del backend fuera del caso pick_demo.
-
-#### Detalle estructural recuperado (2026-04-18)
-
-##### 2.1 Paquetes ROS 2
+### Detalle estructural recuperado (2026-04-18)
+
+#### 2.1 Paquetes ROS 2
 
 | Paquete | Ruta | Responsabilidad |
 |---|---|---|
@@ -391,7 +225,7 @@
 | `tfm_grasping` | `src/tfm_grasping/` | Percepción e inferencia de grasp (visión) |
 | `ur5_tools` | `src/ur5_tools/` | Herramientas auxiliares (system_state, attach_backend) |
 
-##### 2.2 Nodos principales
+#### 2.2 Nodos principales
 
 | Nodo | Archivo fuente | Función |
 |---|---|---|
@@ -409,7 +243,7 @@
 | `ur5_moveit_bridge` | `ur5_tools/ur5_moveit_bridge` | Puente /desired_grasp → MoveIt |
 | `panel_v2` | `ur5_qt_panel/panel_v2.py` | Interfaz Qt principal + orquestación |
 
-##### 2.3 Launch files relevantes
+#### 2.3 Launch files relevantes
 
 | Archivo | Ruta | Qué hace |
 |---|---|---|
@@ -418,7 +252,7 @@
 | `ur5_ros2_control.launch.py` | `src/ur5_bringup/launch/` | Solo controladores ros2_control |
 | `ur5_moveit_bringup.launch.py` | `src/ur5_moveit_config/launch/` | Solo MoveIt 2 (move_group) |
 
-##### 2.4 Qué archivo controla cada parte
+#### 2.4 Qué archivo controla cada parte
 
 | Parte del sistema | Archivo de control |
 |---|---|
@@ -437,7 +271,7 @@
 | Geometría de agarre | `src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py` |
 | Estado lógico objetos | `src/ur5_qt_panel/ur5_qt_panel/panel_objects.py` |
 
-##### 2.5 Relación entre módulos
+#### 2.5 Relación entre módulos
 
 ```
                     ┌─────────────────────────────────┐
@@ -489,117 +323,6 @@
 
 ---
 
-### Detalle estructural recuperado (2026-04-18)
-
-#### 2.1 Paquetes ROS 2
-
-| Paquete | Ruta | Responsabilidad |
-|---|---|---|
-| `ur5_bringup` | `src/ur5_bringup/` | Launch files, configuración de controladores |
-| `ur5_description` | `src/ur5_description/` | URDF/Xacro del robot UR5 + RG2 |
-| `ur5_moveit_config` | `src/ur5_moveit_config/` | Configuración MoveIt 2 (SRDF, kinematics, pipelines) |
-| `ur5_qt_panel` | `src/ur5_qt_panel/` | Panel Qt, lógica de pick demo, módulos de geometría |
-| `tfm_grasping` | `src/tfm_grasping/` | Percepción e inferencia de grasp (visión) |
-| `ur5_tools` | `src/ur5_tools/` | Herramientas auxiliares (system_state, attach_backend) |
-
-#### 2.2 Nodos principales
-
-| Nodo | Archivo fuente | Función |
-|---|---|---|
-| `robot_state_publisher` | (ros2 built-in) | Publica /robot_description + árbol TF |
-| `gz_sim` | (gz-sim built-in) | Motor de simulación Gazebo |
-| `ros_gz_bridge` | (ros_gz built-in) | Puente topics Gazebo ↔ ROS 2 |
-| `joint_trajectory_controller` | (ros2_control) | Ejecuta trayectorias del brazo |
-| `gripper_controller` | (ros2_control) | Controla apertura/cierre pinza |
-| `joint_state_broadcaster` | (ros2_control) | Publica `/joint_states` |
-| `gz_pose_bridge` | `ur5_tools/gz_pose_bridge` | Poses Gazebo → TF ROS 2 |
-| `world_tf_publisher` | `ur5_tools/world_tf_publisher` | TF estático world → base_link |
-| `system_state_manager` | `ur5_tools/system_state_manager` | Monitor estado global del sistema |
-| `gripper_attach_backend` | `ur5_tools/gripper_attach_backend` | Gestión detachable joints Gazebo |
-| `planning_scene_sync` | `ur5_tools/planning_scene_sync` | Sincroniza escena MoveIt con Gazebo |
-| `ur5_moveit_bridge` | `ur5_tools/ur5_moveit_bridge` | Puente /desired_grasp → MoveIt |
-| `panel_v2` | `ur5_qt_panel/panel_v2.py` | Interfaz Qt principal + orquestación |
-
-#### 2.3 Launch files relevantes
-
-| Archivo | Ruta | Qué hace |
-|---|---|---|
-| `ur5_stack.launch.py` | `src/ur5_bringup/launch/` | Launch principal: todo el stack completo |
-| `ur5_rsp.launch.py` | `src/ur5_bringup/launch/` | Solo robot_state_publisher |
-| `ur5_ros2_control.launch.py` | `src/ur5_bringup/launch/` | Solo controladores ros2_control |
-| `ur5_moveit_bringup.launch.py` | `src/ur5_moveit_config/launch/` | Solo MoveIt 2 (move_group) |
-
-#### 2.4 Qué archivo controla cada parte
-
-| Parte del sistema | Archivo de control |
-|---|---|
-| Frames TF del robot | `src/ur5_description/urdf/ur5.urdf.xacro` |
-| Modelo físico Gazebo | `models/ur5_rg2/model.sdf` |
-| Mundo de simulación | `worlds/ur5_mesa_objetos.sdf` |
-| Controladores | `src/ur5_bringup/config/ur5_mock_controllers.yaml` |
-| Configuración MoveIt | `src/ur5_moveit_config/config/` |
-| Variables de entorno pick | `src/ur5_bringup/launch/ur5_stack.launch.py` |
-| Lógica pick demo (fases) | `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` |
-| Orquestación UI | `src/ur5_qt_panel/ur5_qt_panel/panel_v2.py` |
-| Detección de objetos | `src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py` |
-| Parámetros globales | `src/ur5_qt_panel/config/panel_settings.yaml` |
-| Gate ATTACH | `src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py` |
-| IK/FK custom | `src/ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py` |
-| Geometría de agarre | `src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py` |
-| Estado lógico objetos | `src/ur5_qt_panel/ur5_qt_panel/panel_objects.py` |
-
-#### 2.5 Relación entre módulos
-
-```
-                    ┌─────────────────────────────────┐
-                    │         panel_v2.py              │
-                    │   (Qt UI, event handling)        │
-                    └──────┬──────────────────────────┘
-                           │ run_pick_demo()
-                    ┌──────▼──────────────────────────┐
-                    │      panel_pick_demo.py           │
-                    │  (orquestador de fases DIRECTO)   │
-                    └──┬───────────────┬───────────────┘
-                       │               │
-          ┌────────────▼───┐    ┌──────▼──────────────┐
-          │ ur5_kinematics │    │ panel_pick_object.py │
-          │ (IK/FK solver) │    │ (pose objetos, TF)   │
-          └────────────────┘    └──────────────────────┘
-                       │               │
-          ┌────────────▼───────────────▼───────────────┐
-          │           ROS 2 Infrastructure              │
-          │  joint_states · TF2 · MoveIt bridge · Gz   │
-          └────────────────────────────────────────────┘
-```
-
-**Flujo de un comando de movimiento:**
-
```
