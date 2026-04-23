# Base de Conocimiento Técnica — Sistema UR5 + RG2 Pick & Place

Fecha de generación: 2026-04-23

Documento generado como base acumulativa y no sustitutiva. Esta versión une el estado confirmado en código/logs vigentes, el contenido especializado ya presente en la versión actual y el detalle estructural recuperado de la base 2026-04-18.

## 1. Resumen Ejecutivo

- Modo de generación: acumulativo; append, don’t replace.
- Verdad runtime priorizada: código fuente inspeccionado en esta ejecución, seguido de logs/evidencias actuales y reports recientes.
- Contenido vigente preservado explícitamente: attach lógico vs transporte físico, ATTACH_GATE vs CARRY, follow_tcp vs world_locked, stale_tcp_pose_soft_follow, discrepancias metadata/llamada real/telemetría y diagnósticos best_obj_move / best_lift_delta / best_tcp_dist.
- Recuperación histórica: se reincorporan arquitectura extendida, tabla amplia de frames, geometría, flujo fase por fase, inventario amplio de variables, controladores/topics, bugs legacy y troubleshooting operativo de 2026-04-18.
- Discrepancias abiertas resaltadas: world->base_link actual=-0.85 0 0.850 frente a simplificaciones históricas; attach backend launch=0.08 / wrapper runtime=0.06; max_pose_age launch=1.5 / wrapper runtime=2.5.
- Cronología reciente incorporada: `2026-04-22` cerró operativamente el incidente TCP con valor canónico `+0.0050885`; `2026-04-23` lo ratifica geométricamente en el workspace y deja abierto un follow-up separado de cesta / IK (`abierto`).
- Nuevas fuentes de detalle incorporadas al generador: `gripper_geometry.py`, `panel_runtime_validated.env`, `validate_startup_repro.sh` y los incidentes recientes de 2026-04-21/22/23.
- Regla de trazabilidad aplicada: cuando una fuente histórica difiere de la actual, se conserva como histórico/documentado previamente en lugar de borrarla.

### Detalle histórico recuperado (2026-04-18)

#### 1.1 Qué hace el proyecto

Este proyecto implementa un sistema completo de **pick & place** simulado con un robot UR5 equipado con el gripper OnRobot RG2. El sistema selecciona un objeto cilíndrico de una mesa, lo agarra con precisión y lo transporta a una cesta destino. Toda la lógica opera en simulación Gazebo con tiempo de simulación, sin hardware real.

**Flujo operativo de alto nivel:**

```
Lanzar sistema → Panel Qt → Seleccionar objeto → Run Pick Demo →
HOME → APPROACH → GRASP_DOWN → GRASP_ALIGN_IK → PRE_CLOSE →
CLOSE → ATTACH_GATE → LIFT → TRANSPORT → CESTA → HOME_FINAL
```

#### 1.2 Componentes principales

| Componente | Rol |
|---|---|
| UR5 (6 DOF) | Brazo robótico universal |
| OnRobot RG2 | Pinza paralela de 2 fingers |
| Gazebo Sim | Simulación física + sensores |
| MoveIt 2 | Planificación de trayectorias |
| ros2_control | Controladores de joints |
| Panel Qt (panel_v2.py) | Interfaz de usuario + orquestación |
| ur5_moveit_bridge | Puente peticiones panel → MoveIt |
| gripper_attach_backend | Gestión de sujeción virtual (detachable joints) |
| Cámara overhead | Visión para detección de objetos |

#### 1.3 Stack tecnológico

- **ROS 2 Jazzy** (Python rclpy + C++ donde necesario)
- **Gazebo Sim** (gz-sim 8.x) con plugins SDF
- **MoveIt 2** (move_group + computeCartesianPath)
- **ros2_control** (joint_trajectory_controller + gripper_controller)
- **Python 3.12** para toda la lógica del panel
- **Qt5/PyQt5** para la interfaz gráfica
- **IK solver propio** (ur5_kinematics.py, DH params UR5)

#### 1.4 Cómo se lanza

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
source install/setup.bash

# Lanzar stack completo
ros2 launch ur5_bringup ur5_stack.launch.py

# El panel Qt arranca automáticamente
# Alternativamente, lanzar panel por separado:
ros2 run ur5_qt_panel panel_v2
```

**Variables de entorno de configuración** se inyectan en el launch file o mediante export antes del launch. Ver Sección 6.

#### 1.5 Flujo operativo general

1. Sistema arranca: Gazebo, RSP, controladores, MoveIt, panel
2. Usuario abre panel Qt
3. Cámara overhead detecta objeto en mesa
4. Usuario selecciona objeto en UI
5. Usuario pulsa "Run Pick Demo"
6. El panel ejecuta ciclo secuencial de fases (Sección 5)
7. Al finalizar: objeto en cesta, robot en HOME_FINAL

---

## 2. Fuentes Verificadas

### 2.1 Prioridad de fuentes aplicada

1. Código fuente inspeccionado en esta ejecución.
2. Logs y evidencias runtime locales actuales.
3. Reports recientes del workspace.
4. Base 2026-04-18 como referencia histórica estructural.

### 2.2 Archivos y artefactos usados en esta generación

- `agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py`
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`
- `agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py`
- `agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_geometry.py`
- `agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro`
- `agarre_ros2_ws/models/ur5_rg2/model.sdf`
- `agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf`
- `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml`
- `agarre_ros2_ws/src/ur5_qt_panel/config/panel_settings.yaml`
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py`
- `agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py`
- `agarre_ros2_ws/scripts/start_panel_v2.sh`
- `agarre_ros2_ws/scripts/panel_runtime_validated.env`
- `agarre_ros2_ws/scripts/validate_startup_repro.sh`
- `agarre_ros2_ws/src/ur5_bringup/package.xml`
- `agarre_ros2_ws/src/ur5_description/package.xml`
- `agarre_ros2_ws/src/ur5_moveit_config/package.xml`
- `agarre_ros2_ws/src/ur5_qt_panel/package.xml`
- `agarre_ros2_ws/src/tfm_grasping/package.xml`
- `agarre_ros2_ws/src/ur5_tools/package.xml`
- `agarre_ros2_ws/src/ur5_panel_interfaces/package.xml`
- `agarre_ros2_ws/src/ur5_qt_panel/setup.py`
- `agarre_ros2_ws/src/ur5_tools/setup.py`
- `agarre_ros2_ws/src/tfm_grasping/setup.py`
- `agarre_ros2_ws/historico/2026-04-18_base_conocimiento_tecnica.md`
- `reports/BaseDeConocimiento/2026-04-18_base_conocimiento_tecnica_TFM.pdf`
- `auditoria/informe_fix_visual_grasp_20260419.md`
- `reports/evidence/ros2/moveit2_system_status.json`
- `reports/incidents/2026-04-21_tcp_geometry_incident.md`
- `agarre_ros2_ws/reports/incidents/2026-04-22_rg2_tcp_incident_closure.md`
- `agarre_ros2_ws/reports/incidents/2026-04-23_directo_basket_transport_ik_followup.md`
- `auditoria/spatial_20260418/directo_validation_20260418_201902/helper.log`
- `auditoria/spatial_20260419/directo_validation_20260419_164750/helper.log`
- `auditoria/spatial_20260419/directo_validation_20260419_181539/helper.log`
- `auditoria/directo_batch_20260421_083327/directo_validation_20260421_083327_run01/stack.log`

### 2.3 Política de merge aplicada

- append, don’t replace
- prefer current for runtime truth
- preserve historical structure when still useful
- annotate conflicts instead of dropping them
- never compress specialized diagnostics into generic summaries

## 3. Arquitectura del Proyecto

### 3.1 Paquetes ROS 2 del workspace y responsabilidad

| Paquete | Ruta | Responsabilidad / descripción |
|---|---|---|
| `ur5_bringup` | `agarre_ros2_ws/src/ur5_bringup` | Bringup launch files for UR5 simulation/control. |
| `ur5_description` | `agarre_ros2_ws/src/ur5_description` | UR5 robot description (URDF/Xacro and controllers config). |
| `ur5_moveit_config` | `agarre_ros2_ws/src/ur5_moveit_config` | Minimal MoveIt 2 configuration for the UR5 robot. |
| `ur5_qt_panel` | `agarre_ros2_ws/src/ur5_qt_panel` | Qt panel for UR5 simulation control, cameras, and evidence capture. |
| `tfm_grasping` | `agarre_ros2_ws/src/tfm_grasping` | TFM grasping module (perception, grasp representation, ROS publishing). |
| `ur5_tools` | `agarre_ros2_ws/src/ur5_tools` | Utilities and helper nodes for UR5 simulation (MoveIt bridge, release service). |
| `ur5_panel_interfaces` | `agarre_ros2_ws/src/ur5_panel_interfaces` | Interfaces ROS 2 para triggers fiables del panel UR5. |

### 3.2 Nodos principales y función

| Nodo / entry point | Fuente | Función |
|---|---|---|
| `robot_state_publisher` | `built-in` | Publica robot_description y árbol TF semántico. |
| `gz_sim` | `built-in` | Motor de simulación Gazebo. |
| `ros_gz_bridge` | `built-in` | Puente Gazebo ↔ ROS 2 para clock, cámaras y pose/info. |
| `joint_state_broadcaster` | `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml` | Publica /joint_states del robot y gripper. |
| `joint_trajectory_controller` | `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml` | Ejecuta trayectorias del brazo UR5. |
| `gripper_controller` | `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml` | Recibe comandos de apertura/cierre de RG2. |
| `controller_bootstrap` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.controller_bootstrap:main |
| `gz_pose_bridge` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.gz_pose_bridge:main |
| `gz_ros_control_guard` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.gz_ros_control_guard:main |
| `gripper_attach_backend` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.gripper_attach_backend:main |
| `planning_scene_sync` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.planning_scene_sync:main |
| `ur5_moveit_bridge` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.ur5_moveit_bridge:main |
| `release_objects_service` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.release_objects_service:main |
| `system_state_manager` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.system_state_manager:main |
| `world_tf_publisher` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.world_tf_publisher:main |
| `tf_probe` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.tf_probe:main |
| `clock_probe` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.clock_probe:main |
| `jt_smoke_test` | `agarre_ros2_ws/src/ur5_tools/setup.py` | Entry point Python: ur5_tools.jt_smoke_test:main |
| `panel_v2` | `agarre_ros2_ws/src/ur5_qt_panel/setup.py` | Entry point Python: ur5_qt_panel.panel_v2:main |
| `main_panel` | `agarre_ros2_ws/src/ur5_qt_panel/setup.py` | Entry point Python: ur5_qt_panel.main_panel:main |
| `grasp_inference` | `agarre_ros2_ws/src/tfm_grasping/setup.py` | Entry point Python: tfm_grasping.grasp_inference:main |

### 3.3 Launch files relevantes y qué lanza cada uno

| Archivo | Ruta | Qué lanza |
|---|---|---|
| `ur5_stack.launch.py` | `agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py` | Launch principal: Gazebo, RSP, bridges, attach backend, scene sync, bridge MoveIt y panel. |
| `ur5_rsp.launch.py` | `agarre_ros2_ws/src/ur5_bringup/launch/ur5_rsp.launch.py` | Robot state publisher del UR5/RG2. |
| `ur5_ros2_control.launch.py` | `agarre_ros2_ws/src/ur5_bringup/launch/ur5_ros2_control.launch.py` | Bringup de ros2_control y controller_manager. |
| `ur5_moveit_bringup.launch.py` | `agarre_ros2_ws/src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py` | Bringup de MoveIt 2 / move_group. |

### 3.4 Qué archivo controla cada parte

| Parte del sistema | Archivo de control |
|---|---|
| Frames TF del robot | `agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro` |
| Geometría canónica runtime del RG2 | `agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_geometry.py` |
| Modelo físico Gazebo | `agarre_ros2_ws/models/ur5_rg2/model.sdf` |
| Mundo de simulación | `agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf` |
| Controladores ros2_control | `agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml` |
| Variables de entorno del pick demo | `agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py` |
| Overrides runtime del panel | `agarre_ros2_ws/scripts/start_panel_v2.sh` |
| Perfil runtime validado | `agarre_ros2_ws/scripts/panel_runtime_validated.env` |
| Harness de validación de arranque | `agarre_ros2_ws/scripts/validate_startup_repro.sh` |
| Lógica de fases del pick | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` |
| Gate de attach | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py` |
| Backend attach/transporte | `agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py` |
| TF world -> base_link | `agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py` |
| Geometría vertical del pick | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py` |
| Interfaz Qt / orquestación | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py` |

### 3.5 Relación entre módulos y flujo de comandos

- panel_v2 orquesta interacción de usuario, estados UI y disparo del ciclo.
- panel_pick_demo ejecuta fases, targets, settle, gates y validación física del carry.
- ur5_kinematics aplica IK/FK directo en base_link_inertia; el panel traduce desde base_link con NEGATE_XY.
- gripper_geometry centraliza la lectura canónica de `rg2_tcp` y `rg2_pinch_center` desde el URDF para evitar offsets duplicados en runtime.
- ur5_moveit_bridge publica/consume desired_grasp para el camino MoveIt.
- joint_trajectory_controller y gripper_controller ejecutan los comandos físicos en Gazebo.
- gripper_attach_backend decide attach_route_decision entre follow_tcp, tool_anchor y demo_transport/world_locked según objeto y configuración.
- start_panel_v2 carga `panel_runtime_validated.env`; `validate_startup_repro.sh` revalida que el stack sólo llegue a `READY` con geometría coherente.

### 3.6 Flujo panel → IK → controller

`panel_pick_demo.py` → `_move_tcp_direct()` / `_send_ik_motion()` → `ur5_kinematics.ik_ur5()` → publicación a `/joint_trajectory_controller/joint_trajectory` → `joint_trajectory_controller` → `/joint_states` → verificación de convergencia y settle.

### 3.7 Flujo panel → MoveIt bridge → move_group → resultado

`panel_pick_demo.py` → `/desired_grasp/request` → `ur5_moveit_bridge` → `move_group.computeCartesianPath()/execute()` → `/desired_grasp/result` → panel. El bridge se lanza ya en el stack principal y no sólo on-demand.

### Detalle estructural recuperado (2026-04-18)

#### 2.1 Paquetes ROS 2

| Paquete | Ruta | Responsabilidad |
|---|---|---|
| `ur5_bringup` | `src/ur5_bringup/` | Launch files, configuración de controladores |
| `ur5_description` | `src/ur5_description/` | URDF/Xacro del robot UR5 + RG2 |
| `ur5_moveit_config` | `src/ur5_moveit_config/` | Configuración MoveIt 2 (SRDF, kinematics, pipelines) |
| `ur5_qt_panel` | `src/ur5_qt_panel/` | Panel Qt, lógica de pick demo, módulos de geometría |
| `tfm_grasping` | `src/tfm_grasping/` | Percepción e inferencia de grasp (visión) |
| `ur5_tools` | `src/ur5_tools/` | Herramientas auxiliares (system_state, attach_backend) |

#### 2.2 Nodos principales

| Nodo | Archivo fuente | Función |
|---|---|---|
| `robot_state_publisher` | (ros2 built-in) | Publica /robot_description + árbol TF |
| `gz_sim` | (gz-sim built-in) | Motor de simulación Gazebo |
| `ros_gz_bridge` | (ros_gz built-in) | Puente topics Gazebo ↔ ROS 2 |
| `joint_trajectory_controller` | (ros2_control) | Ejecuta trayectorias del brazo |
| `gripper_controller` | (ros2_control) | Controla apertura/cierre pinza |
| `joint_state_broadcaster` | (ros2_control) | Publica `/joint_states` |
| `gz_pose_bridge` | `ur5_tools/gz_pose_bridge` | Poses Gazebo → TF ROS 2 |
| `world_tf_publisher` | `ur5_tools/world_tf_publisher` | TF estático world → base_link |
| `system_state_manager` | `ur5_tools/system_state_manager` | Monitor estado global del sistema |
| `gripper_attach_backend` | `ur5_tools/gripper_attach_backend` | Gestión detachable joints Gazebo |
| `planning_scene_sync` | `ur5_tools/planning_scene_sync` | Sincroniza escena MoveIt con Gazebo |
| `ur5_moveit_bridge` | `ur5_tools/ur5_moveit_bridge` | Puente /desired_grasp → MoveIt |
| `panel_v2` | `ur5_qt_panel/panel_v2.py` | Interfaz Qt principal + orquestación |

#### 2.3 Launch files relevantes

| Archivo | Ruta | Qué hace |
|---|---|---|
| `ur5_stack.launch.py` | `src/ur5_bringup/launch/` | Launch principal: todo el stack completo |
| `ur5_rsp.launch.py` | `src/ur5_bringup/launch/` | Solo robot_state_publisher |
| `ur5_ros2_control.launch.py` | `src/ur5_bringup/launch/` | Solo controladores ros2_control |
| `ur5_moveit_bringup.launch.py` | `src/ur5_moveit_config/launch/` | Solo MoveIt 2 (move_group) |

#### 2.4 Qué archivo controla cada parte

| Parte del sistema | Archivo de control |
|---|---|
| Frames TF del robot | `src/ur5_description/urdf/ur5.urdf.xacro` |
| Modelo físico Gazebo | `models/ur5_rg2/model.sdf` |
| Mundo de simulación | `worlds/ur5_mesa_objetos.sdf` |
| Controladores | `src/ur5_bringup/config/ur5_mock_controllers.yaml` |
| Configuración MoveIt | `src/ur5_moveit_config/config/` |
| Variables de entorno pick | `src/ur5_bringup/launch/ur5_stack.launch.py` |
| Lógica pick demo (fases) | `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` |
| Orquestación UI | `src/ur5_qt_panel/ur5_qt_panel/panel_v2.py` |
| Detección de objetos | `src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py` |
| Parámetros globales | `src/ur5_qt_panel/config/panel_settings.yaml` |
| Gate ATTACH | `src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py` |
| IK/FK custom | `src/ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py` |
| Geometría de agarre | `src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py` |
| Estado lógico objetos | `src/ur5_qt_panel/ur5_qt_panel/panel_objects.py` |

#### 2.5 Relación entre módulos

```
                    ┌─────────────────────────────────┐
                    │         panel_v2.py              │
                    │   (Qt UI, event handling)        │
                    └──────┬──────────────────────────┘
                           │ run_pick_demo()
                    ┌──────▼──────────────────────────┐
                    │      panel_pick_demo.py           │
                    │  (orquestador de fases DIRECTO)   │
                    └──┬───────────────┬───────────────┘
                       │               │
          ┌────────────▼───┐    ┌──────▼──────────────┐
          │ ur5_kinematics │    │ panel_pick_object.py │
          │ (IK/FK solver) │    │ (pose objetos, TF)   │
          └────────────────┘    └──────────────────────┘
                       │               │
          ┌────────────▼───────────────▼───────────────┐
          │           ROS 2 Infrastructure              │
          │  joint_states · TF2 · MoveIt bridge · Gz   │
          └────────────────────────────────────────────┘
```

**Flujo de un comando de movimiento:**

```
panel_pick_demo.py
  → _send_ik_motion(target_pose_base_link)
  → ur5_kinematics.ik_ur5(target)  [IK directo NEGATE_XY]
  → publish /joint_trajectory_controller/joint_trajectory
  → joint_trajectory_controller ejecuta en Gazebo
  → joint_state_broadcaster publica /joint_states
  → panel lee /joint_states para verificar convergencia
```

**Flujo de un comando MoveIt (GRASP_DOWN cartesiano):**

```
panel_pick_demo.py
  → _request_moveit_cartesian(waypoints)
  → publish /desired_grasp/request (PoseStamped[])
  → ur5_moveit_bridge recibe request
  → move_group.computeCartesianPath()
  → move_group.execute()
  → joint_states convergen
  → ur5_moveit_bridge publica /desired_grasp/result
  → panel_pick_demo.py recibe resultado
```

---

## 4. Frames y Offsets

### 4.1 Tabla de frames y offsets consolidada

| Frame | Tipo | Padre | Offset / estado | Uso operativo | Riesgos al mezclar frames |
|---|---|---|---|---|---|
| `world` | origen Gazebo | — | — | Origen absoluto de simulación | Mezclar world con base_link desplaza targets del pick. |
| `base_link` | frame URDF publicado | `world` | actual confirmado: -0.85 0 0.850 | Base del UR5 para IK, panel y MoveIt | No asumir sólo offset Z: la traslación actual también incluye X=-0.85 m. |
| `base_link_inertia` | frame cinemático DH | `base_link` | histórico/documentado: (0,0,0) + Rz(pi) | Marco interno del solver IK/FK | Si se ignora, reaparece el bug NEGATE_XY / simetría especular. |
| `cadena cinemática UR5` | joints UR5 | `base_link_inertia` | per DH / ur_description | Shoulder -> wrist_3 del brazo | No mezclar valores DH con poses world sin transformación previa. |
| `tool0` | TCP semántico del brazo | `wrist_3_link` | SDF visual actual: 0 0.275 0 -1.570796325 0 0 | Padre semántico de RG2 y ancla de offset del gripper | Confundir tool0 con el punto real de grasp desplaza el contacto 175 mm. |
| `rg2_base_link` | frame URDF fijo | `tool0` | actual confirmado: 0 0 0 | Base semántica del gripper RG2 | No equivale al pinch center operativo. |
| `rg2_tcp` | frame URDF fijo | `tool0` | actual confirmado: 0 0 0.0050885 | TCP virtual/semántico del gripper | No confundir con geometría visible SDF del cuerpo RG2. |
| `rg2_pinch_center` | frame URDF fijo | `tool0` | actual confirmado: 0 0 0.0050885 | Punto operacional de grasp y attach | Es el frame correcto para lógica de pick; usar tool0 degrada Z y distancias. |
| `camera_wrist_link` | frame SDF fijo | `rg2_hand` | actual confirmado en SDF: 0 0 0.05 0 0 0 | Cámara montada en la muñeca/gripper | Es geometría SDF, no el TCP operacional. |
| `pick_demo_anchor` | frame SDF fijo / runtime rewrite | `tool0` | SDF fuente=0 0 0.0050885 0 0 0; runtime actual sincronizado con la geometría canónica (`rg2_tcp`=0.0050885) | Ancla auxiliar para attach backend / probes | Si el runtime deja de sincronizarse con el URDF, el ancla vuelve a desviarse de rg2_pinch_center. |

### 4.2 Discrepancias actuales vs históricas

| Elemento | Valor actual confirmado | Valor histórico / documentado | Observación |
|---|---|---|---|
| world -> base_link | actual confirmado: -0.85 0 0.850 | histórico simplificado: (0,0,0.850) | La geometría actual incluye traslación en X; documentar sólo Z ya no es suficiente. |
| tool0 -> rg2_pinch_center | actual confirmado: 0 0 0.0050885 | histórico: 0 0 0.175 | Sin discrepancia de valor; se conserva como confirmación actual. |
| SDF ur5_hand_joint | actual confirmado: relative_to=wrist_3_link, pose=0 0.0823 0 1.570796325 0 1.570796325 | memorias previas con otros valores | Priorizar el source tree actual; los valores previos quedan como históricos o desactualizados. |
| pick_demo_anchor | actual runtime efectivo: 0.0050885 m | histórico documentado: 0.0050885 m | El launch ahora sincroniza `pick_demo_anchor` directamente con la geometría canónica del URDF; ya no depende de `gripper_tcp_z_offset`. |

### 4.3 Observaciones obligatorias preservadas y reforzadas

- `tool0` vs `rg2_pinch_center`: el offset semántico actual confirmado sigue siendo 0 0 0.0050885. La geometría visual SDF del gripper se modela por `ur5_hand_joint=0 0.0823 0 1.570796325 0 1.570796325` y no debe confundirse con el TCP semántico.
- `world` vs `base_link`: el estado actual confirmado no es sólo Z. La traslación usada por URDF/world file es `-0.85 0 0.850`.
- `NEGATE_XY`: sigue siendo consecuencia del uso de `base_link_inertia` en el solver DH; no es una preferencia opcional ni debe tratarse como flag de runtime activa.
- TCP semántico vs geometría visual SDF: la mano visible del SDF y el punto de grasp semántico viven en capas distintas; la segunda es la que manda para IK, attach y carry validation.

### Detalle histórico recuperado (2026-04-18)

#### 3.1 Tabla completa de frames

| Frame | Tipo | Padre | Offset (xyz) | Descripción |
|---|---|---|---|---|
| `world` | Gazebo origin | — | — | Origen absoluto de la simulación |
| `base_link` | URDF fixed | `world` | (0, 0, 0.850) | Base del UR5; 850 mm sobre el suelo |
| `base_link_inertia` | URDF fixed | `base_link` | (0, 0, 0) Rz(π) | Raíz cinemática DH — rotada 180° en Z respecto a base_link |
| `shoulder_link` | joint | `base_link_inertia` | per DH | Hombro UR5 |
| `upper_arm_link` | joint | `shoulder_link` | per DH | Brazo superior |
| `forearm_link` | joint | `upper_arm_link` | per DH | Antebrazo |
| `wrist_1_link` | joint | `forearm_link` | per DH | Muñeca 1 |
| `wrist_2_link` | joint | `wrist_1_link` | per DH | Muñeca 2 |
| `wrist_3_link` | joint | `wrist_2_link` | per DH | Muñeca 3 |
| `tool0` | URDF fixed | `wrist_3_link` | (0, 0, 0) | TCP del brazo UR5 (sin gripper) |
| `rg2_base_link` | URDF fixed | `tool0` | (0, 0, 0) | Base del gripper RG2 |
| `rg2_tcp` | URDF fixed | `tool0` | (0, 0, 0.175) | TCP virtual del gripper (alias de rg2_pinch_center) |
| `rg2_pinch_center` | URDF fixed | `tool0` | (0, 0, 0.175) | **Frame operacional de contacto** — punto entre fingers |
| `rg2_hand` | URDF fixed | `rg2_base_link` | (0.105, 0, 0) | Cuerpo principal del gripper |
| `rg2_leftfinger` | revoluto | `rg2_hand` | (0.105, 0.017, 0) | Finger izquierdo (joint1) |
| `rg2_rightfinger` | revoluto | `rg2_hand` | (0.105, -0.017, 0) | Finger derecho (joint2) |
| `camera_wrist_link` | URDF fixed | `rg2_hand` | (var) | Cámara montada en gripper |
| `pick_demo_anchor` | static | `tool0` | (0, 0, 0.175) | Ancla para gripper_attach_backend |

#### 3.2 Significado y usos de cada frame clave

**`world`**  
- Origen de Gazebo. Todas las poses de objetos simulados se expresan primero en `world`.
- Usado por: `gz_pose_bridge`, `world_tf_publisher`, `panel_pick_object.py` (conversión a base_link).
- NUNCA usar directamente en comandos IK del panel.

**`base_link`**  
- Raíz del árbol TF del robot. Publicado por RSP.
- Frame de referencia para **todos los targets IK** que genera el panel.
- Usado por: `panel_pick_demo.py`, `ur5_kinematics.ik_ur5()`, MoveIt, `ur5_moveit_bridge`.

**`base_link_inertia`**  
- Frame interno del solver DH. Tiene rotación Rz(π) respecto a `base_link`.
- ⚠️ **CRÍTICO:** El solver IK (`ur5_kinematics.py`) opera en este frame. Por eso se aplica NEGATE_XY a los targets antes de pasar al solver. Ver Sección 8.4.
- No se referencia explícitamente en el código del panel, pero es el sistema de coordenadas subyacente del IK.

**`tool0`**  
- TCP del brazo UR5 sin gripper. Publicado por RSP vía FK.
- ⚠️ **Trampa:** La UI históricamente mostraba la pose del TCP como `tool0`. Esto causa confusión porque el contacto físico real ocurre en `rg2_pinch_center`, 175 mm más arriba.
- Usado por: URDF (padre de rg2_base_link), MoveIt internamente.

**`rg2_pinch_center`**  
- **El frame correcto para toda la lógica de grasp.** Punto central entre los dos fingers en posición de cierre.
- Offset respecto a tool0: z = +0.175 m (fijo, sin dependencia del estado del gripper).
- Usado por: `panel_pick_demo.py` para todos los targets de APPROACH, GRASP_DOWN, GRASP_ALIGN_IK, CLOSE, ATTACH_GATE.
- Definido en: `src/ur5_description/urdf/ur5.urdf.xacro`.

#### 3.3 Relaciones clave entre frames

```
world
  └─ base_link (+0.850 m Z en world)
       └─ [cadena DH UR5: 6 joints]
            └─ tool0
                 ├─ rg2_tcp        (+0.175 m Z) ← alias
                 ├─ rg2_pinch_center (+0.175 m Z) ← FRAME OPERACIONAL
                 ├─ rg2_base_link  (+0.000 m Z)
                 │    └─ rg2_hand (+0.105 m X)
                 │         ├─ rg2_leftfinger  (revoluto, +Y)
                 │         └─ rg2_rightfinger (revoluto, -Y)
                 └─ pick_demo_anchor (+0.175 m Z) ← usado por attach_backend
```

**Transformación objeto world → base_link:**

```python
# panel_pick_object.py
tf_buffer.lookup_transform("base_link", "world", rclpy.time.Time())
# Aplica: P_base = R_world_to_base @ P_world + t_world_to_base
# Donde t = (0, 0, -0.850) porque base_link está +0.850 sobre world
```

#### 3.4 Errores típicos por mezcla de frames

| Error | Síntoma | Causa |
|---|---|---|
| Usar `tool0` en lugar de `rg2_pinch_center` para target Z | Robot baja 175 mm de más, colisiona con mesa | No añadir offset del gripper al target |
| Usar coordenadas `world` como si fueran `base_link` | Robot va a posición incorrecta ~850 mm desplazada en Z | No aplicar transformación world→base_link |
| No aplicar NEGATE_XY al pasar target al solver IK | Error de posición especular en XY | base_link_inertia tiene Rz(π) |
| Mezclar poses snapshot (TF live) con poses stable cache (panel state) | Divergencia si el objeto se mueve entre actualizaciones | Usar siempre la misma fuente dentro de una fase |
| Aplicar GRIPPER_TCP_Z_OFFSET sobre rg2_pinch_center | Doble offset: target Z incorrecto +0.05 m adicional | Bug histórico corregido (ver Sección 8) |

---

## 5. Geometría del Gripper, Objeto y Workspace

### 5.1 Geometría actual confirmada desde código y SDF

| Elemento | Valor actual confirmado | Fuente |
|---|---|---|
| Apertura máxima por finger | 1.18 rad | agarre_ros2_ws/models/ur5_rg2/model.sdf |
| Fricción fingers | mu=2.15 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
| Stiffness contacto fingers | kp=200000 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
| Damping contacto fingers | kd=30.0 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
| Masa cuerpo RG2 (rg2_hand) | 0.29881464108881906 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
| tool0 -> rg2_pinch_center | 0 0 0.0050885 | agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro |
| Pose visual ur5_hand_joint | relative_to=wrist_3_link, pose=0 0.0823 0 1.570796325 0 1.570796325 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
| Finger izquierdo respecto a rg2_hand | 0.105 0.017 0 0 0 0 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
| Finger derecho respecto a rg2_hand | 0.105 -0.017 0 0 0 0 | agarre_ros2_ws/models/ur5_rg2/model.sdf |

### 5.2 Objeto, mesa, cesta y restricciones del workspace

| Elemento | Valor / estado | Fuente |
|---|---|---|
| Objeto pick_demo | cilindro radio=0.025 m, longitud=0.05 m, masa=0.08 kg | agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf |
| Spawn pick_demo (world) | -0.42 0.00 0.876 0 0 0 | agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf |
| Mesa útil | centro world=(-0.170, 0.000), tablero=0.768 x 0.800 x 0.050 m | agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf |
| Altura superficie mesa | actual confirmada ≈ 0.850 m en world | agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf |
| Cesta / bandeja destino | pose world base=-1.30 0.00 0.780 0 0 0; superficie base≈0.785 m | agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf |
| Reach operativo UR5 | histórico/documentado previamente: ~0.85 m | agarre_ros2_ws/historico/2026-04-18_base_conocimiento_tecnica.md |
| Residual geométrico | actual: pendiente de validación runtime fina; histórico: residual DH/SDF mitigado vía bias loop | agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py |

### 5.3 Estado de validación de geometría

- Cierre geométrico documentado entre `2026-04-22` y `2026-04-23`: `tool0 -> rg2_tcp.z = +0.0050885` validado y sincronizado con `rg2_pinch_center`; `gripper_geometry.py` centraliza esta lectura desde el URDF.
- `tool0 -> rg2_pinch_center`: actual confirmado en URDF.
- `pick_demo`: actual confirmado en world file con radio=0.025 m y longitud=0.05 m.
- Guardrail de arranque actual: `validate_startup_repro.sh` exige `geometry_ok=true` y `state=READY` antes de considerar reproducible el stack.
- Reach operativo UR5 ≈ 0.85 m: histórico no revalidado en esta corrida; se conserva por utilidad operativa.
- Residual DH/SDF fino subcentimétrico: pendiente de validación runtime detallada; se mantiene el conocimiento histórico sobre bias loop y residual en Z.

### 5.4 Notas de tolerancia y residual geométrico

- La compensación geométrica fina sigue ocurriendo en GRASP_ALIGN_IK y no debe sustituirse por asumir que el SDF visible coincide exactamente con el TCP.
- `gripper_tcp_z_offset` ya no existe en `panel_settings.yaml`; la sincronización runtime de `pick_demo_anchor` sale del URDF canónico.
- Cuando un dato aquí no ha podido revalidarse automáticamente, se mantiene como histórico/documentado o inferido desde documentación previa, en vez de eliminarse.

### Detalle histórico recuperado (2026-04-18)

#### 4.1 Dimensiones útiles del RG2

| Parámetro | Valor | Fuente |
|---|---|---|
| Apertura máxima (física) | 1.18 rad por finger | `model.sdf`, joint limits |
| Apertura máxima (mm apertura lineal) | ~110 mm estimado | Geometría finger revoluto |
| Punto TCP operacional | rg2_pinch_center (+0.175 m sobre tool0) | `ur5.urdf.xacro` |
| Distancia tool0 → contacto real | 0.175 m | URDF offset |
| Fricción fingers (μ) | 2.15 | `model.sdf` contact surface |
| Stiffness contacto (kp) | 200,000 | `model.sdf` |
| Damping contacto (kd) | 30.0 | `model.sdf` |
| Masa gripper completo | ~2.0 kg (estimado) | `model.sdf` inertials |

#### 4.2 Punto TCP operativo

El **TCP operativo** del sistema es `rg2_pinch_center`, que se encuentra exactamente **0.175 m en el eje Z positivo** de `tool0`.

Esto significa:
- Cuando el panel calcula un target en `base_link` para que el TCP llegue a una posición (x, y, z), el IK debe posicionar `tool0` en (x, y, z - 0.175).
- El código del panel **ya tiene en cuenta este offset** al calcular los targets. El punto de referencia en todos los cálculos es `rg2_pinch_center`, no `tool0`.

#### 4.3 TCP virtual vs contacto físico real

| Aspecto | TCP Virtual (rg2_pinch_center) | Contacto Físico Real |
|---|---|---|
| Posición | Exactamente z = +0.175 sobre tool0 | Depende de apertura del gripper y geometría del finger |
| Cálculo | Offset fijo URDF (no varía con estado gripper) | Varía con la apertura |
| Uso en código | Target de todos los movimientos | Solo relevante para análisis de fuerza |
| Offset residual | 0 mm (por definición del URDF) | ~0–5 mm según apertura |

**Nota práctica:** En el sistema actual, la alineación final de Z se ajusta mediante el bias loop de `GRASP_ALIGN_IK` para compensar el residual entre el modelo DH y el SDF (ver Sección 8.1).

#### 4.4 Objeto de pick (cilindro)

| Parámetro | Valor | Fuente |
|---|---|---|
| Forma | Cilindro (circle en OBJECT_SHAPES) | `panel_pick_object.py` |
| Pose spawn (world) | (-0.42, 0.00, 0.875) | `worlds/ur5_mesa_objetos.sdf` |
| Altura estimada | ~50 mm | Inferido de GRASP_CONTACT_Z_OFFSET_M=0.13 m |
| Radio estimado | ~30 mm | Inferido de tolerancias XY |
| Nombre en Gazebo | `pick_demo` | `models/`, `ur5_stack.launch.py` |

**Cálculo de target Z para grasp:**

```
Z_target_rg2_pinch_center = Z_objeto_base + GRASP_CONTACT_Z_OFFSET_M
                          = Z_objeto_base + 0.13

Donde:
  Z_objeto_base = pose_objeto_en_base_link.z
  GRASP_CONTACT_Z_OFFSET_M = 0.13 m (lleva pinza al ecuador del cilindro)
```

#### 4.5 Mesa y restricciones del workspace

| Parámetro | Valor |
|---|---|
| Tamaño mesa | 768 × 800 mm |
| Centro mesa (world XY) | (-0.17, 0.00) |
| Altura mesa (world Z) | ~0.850 m (igual que base_link Z) |
| Z efectiva del objeto en base_link | ~0.025 m (objeto está ~25 mm sobre la mesa en frame base) |
| Margen objeto desde borde | 90 mm mínimo |
| Cesta destino (world) | (-1.30, 0.00, 0.82) |
| Reach del UR5 | 0.85 m radio |

**Nota sobre alturas:** La mesa está a la misma Z en `world` que `base_link`. Por lo tanto, un objeto sobre la mesa tiene Z positiva pequeña en `base_link` (la superficie de la mesa está en z ≈ 0 en base_link, y el objeto tiene su centro a ~25 mm sobre ella).

---

## 6. Flujo Completo del Pick

### 6.1 Fases reconstruidas y anotadas

Las siguientes fases se presentan como unión consistente entre la base 2026-04-18, el documento vigente 2026-04-20 y el código inspeccionado en esta ejecución.

### 6.2 HOME

- Estado: histórico revalidado por labels actuales
- Objetivo: Llevar el robot a una postura segura de reposo antes de iniciar o cerrar ciclo.
- Target: Preset articular HOME / JOINT_TABLE_POSE_RAD.
- Frame: joint space
- Método: Preset articular vía joint_trajectory_controller.
- Criterio de éxito: Convergencia articular dentro de tolerancia.
- Criterio de fallo: Timeout de movimiento o controlador no activo.
- Criterio de skip: Puede omitirse si ya está en HOME y la lógica de pre-check lo confirma.
- Logs relevantes: [PICK][HOME] o trazas DIRECT equivalentes.
- Variables de entorno asociadas: PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC (si entra por MoveIt).
- Observaciones y discrepancias: Se conserva como fase histórica útil; la implementación actual sigue manejando HOME como estado seguro aunque el pipeline directo haya cambiado de detalle interno.

### 6.3 MESA

- Estado: histórico no revalidado completamente en esta corrida
- Objetivo: Llevar el robot a la pose de imagen / pre-pick sobre la mesa.
- Target: Preset tipo JOINT_PICK_IMAGE_POSE_RAD o equivalente de captura.
- Frame: joint space
- Método: Preset articular previo a PICK_IMAGE / selección.
- Criterio de éxito: Robot estabilizado en pose de observación.
- Criterio de fallo: Timeout, controladores inactivos o joints no convergentes.
- Criterio de skip: Puede integrarse con PICK_IMAGE o ser absorbida por la lógica actual del panel.
- Logs relevantes: Trazas con label MESA / PICK_IMAGE.
- Variables de entorno asociadas: Sin variable exclusiva confirmada; depende del pipeline de observación.
- Observaciones y discrepancias: Se recupera desde 2026-04-18 porque sigue siendo útil para entender el flujo panel -> visión -> pick.

### 6.4 APPROACH_COARSE

- Estado: actual confirmado
- Objetivo: Posicionar rg2_pinch_center sobre el objeto con margen seguro antes del descenso.
- Target: (x_obj, y_obj, z_obj + clearance) en base_link.
- Frame: base_link / rg2_pinch_center
- Método: IK directo con conversión a base_link_inertia y NEGATE_XY permanente.
- Criterio de éxito: Gate XY/Z de aproximación dentro de tolerancia.
- Criterio de fallo: IK no converge, settle insuficiente o divergencia FK/TF-live.
- Criterio de skip: No suele omitirse; puede haber correcciones XY/Z intermedias.
- Logs relevantes: [PICK][DIRECT][APPROACH_COARSE] y gates asociados.
- Variables de entorno asociadas: PANEL_PICK_DEMO_APPROACH_COARSE_*.
- Observaciones y discrepancias: Aquí es crítica la diferencia world vs base_link y la semántica base_link_inertia del solver DH.

### 6.5 GRASP_DOWN

- Estado: actual confirmado
- Objetivo: Descender el TCP hasta la altura de grasp sin saltar de rama IK.
- Target: Altura de contacto sobre el cilindro en base_link.
- Frame: base_link / rg2_pinch_center
- Método: MoveIt cartesiano si está habilitado, con fallback a IK segmentado conservador.
- Criterio de éxito: Error XY/Z y dist3D dentro de umbral; sin cambio de rama crítico.
- Criterio de fallo: IK fuera de tolerancia, cartesian path fallido o branch jump.
- Criterio de skip: No se omite salvo abortos tempranos del pipeline.
- Logs relevantes: [PICK][DIRECT][GRASP_DOWN], [GRASP_DOWN_CARTESIAN], [GRASP_DOWN_FALLBACK].
- Variables de entorno asociadas: PANEL_PICK_DEMO_GRASP_DOWN_*.
- Observaciones y discrepancias: La mitigación de branch jump y los steps de 5 mm se preservan como parte del conocimiento histórico útil.

### 6.6 GRASP_ALIGN_IK

- Estado: actual confirmado
- Objetivo: Compensar residual geométrico y afinar Z/XY antes de cerrar.
- Target: Pose de contacto con bias iterativo en Z.
- Frame: base_link
- Método: Loop IK con bias Z y criterios de salida XY/Z.
- Criterio de éxito: ALIGN_EXIT_XY/Z dentro de tolerancia y residual aceptable.
- Criterio de fallo: Intentos agotados o sin mejora de residual.
- Criterio de skip: Puede omitirse si el target ya es alcanzable y el panel así lo permite.
- Logs relevantes: [PICK][DIRECT][GRASP_ALIGN_IK] attempt=... bias=...
- Variables de entorno asociadas: PANEL_PICK_DEMO_ALIGN_*.
- Observaciones y discrepancias: Se enlaza con el bug histórico del residual DH/SDF en Z y con la semántica actual tool0 vs rg2_pinch_center.

### 6.7 PRE_CLOSE

- Estado: actual confirmado
- Objetivo: Validar alineación final antes de mandar el cierre del gripper.
- Target: Pose actual TCP vs objeto inmediatamente antes de CLOSE.
- Frame: base_link
- Método: Lectura FK/TF + comparación contra el objeto; puede reintentar realineación.
- Criterio de éxito: XY y Z_err dentro de tolerancia.
- Criterio de fallo: Error fuera de umbral tras reintentos.
- Criterio de skip: Puede omitirse si el pipeline marca reachable alignment suficiente.
- Logs relevantes: [PICK][DIRECT][PRE_CLOSE] xy_err=... z_err=...
- Variables de entorno asociadas: PANEL_PICK_DEMO_PRE_CLOSE_*.
- Observaciones y discrepancias: Sigue siendo una fase distinta de CLOSE: valida pose, no confirma grasp.

### 6.8 CLOSE

- Estado: actual confirmado
- Objetivo: Cerrar RG2 y confirmar que hubo movimiento y/o apertura medida consistente.
- Target: Comando a rg2_finger_joint1/2 hasta cierre objetivo.
- Frame: joint space (gripper)
- Método: Publicación a gripper_controller + espera de confirmación por joint_states.
- Criterio de éxito: Delta de joints y/o opening_sum dentro de heurística de confirmación.
- Criterio de fallo: CLOSE en PEND, controlador inactivo o timeout de confirmación.
- Criterio de skip: No se omite en el ciclo de pick.
- Logs relevantes: [PICK][DIRECT][CLOSE], wait_start/wait_ok/wait_timeout.
- Variables de entorno asociadas: PANEL_PICK_DEMO_CLOSE_* y PANEL_PICK_DEMO_GRIPPER_*.
- Observaciones y discrepancias: Mantener explícito el bug histórico CLOSE en PEND y que cerrar no demuestra por sí solo grasp físico estable.

### 6.9 ATTACH_GATE

- Estado: actual confirmado
- Objetivo: Aprobar attach lógico sólo cuando proximidad, drift y cierre son coherentes.
- Target: TCP-objeto dentro de 0.020 m XY y drift relativo dentro de 0.012 m.
- Frame: base_link / world según muestras del evaluador
- Método: AttachGateEvaluator con ventana temporal multi-fuente.
- Criterio de éxito: Distancia TCP-objeto, drift, cierre de gripper y backend OK dentro de ventana estable.
- Criterio de fallo: AttachGateEvaluator FAIL o backend sin confirmar attach.
- Criterio de skip: No se omite; es gate de seguridad lógico.
- Logs relevantes: [ATTACH_GATE][CHECK|PASS|FAIL|WARN].
- Variables de entorno asociadas: PANEL_PICK_DEMO_ATTACH_*.
- Observaciones y discrepancias: ATTACH_GATE correcto = attach lógico aprobado. No equivale a transporte físico confirmado; esa separación debe preservarse y enlazarse con LIFT/CARRY.

### 6.10 LIFT

- Estado: actual confirmado
- Objetivo: Elevar el objeto lo suficiente para demostrar levantamiento real tras el grasp.
- Target: Subida corta post-grasp antes del carry largo.
- Frame: base_link
- Método: IK directo o ruta equivalente con validación física post-grasp.
- Criterio de éxito: _validate_demo_carry(post_grasp_lift) con timeout=3.0s, min_obj_move=0.020 m, min_lift_delta=0.025 m y max_tcp_dist=0.120 m.
- Criterio de fallo: demo_carry_validation_failed, object_not_updated, carry_follow_lost o tcp_dist_above_max.
- Criterio de skip: No se omite si ATTACH_GATE pasó.
- Logs relevantes: [PICK][DIRECT][FINAL_TRACE] phase=CARRY event=wait_done ..., [PICK][DIRECT][PHYSICS] phase=post_grasp_lift ...
- Variables de entorno asociadas: PANEL_PICK_DEMO_CARRY_SETTLE_SEC=3.0 y thresholds de carry/lift en panel.
- Observaciones y discrepancias: Aquí se decide el primer veredicto físico serio. El attach lógico puede existir y aun así el objeto quedarse inmóvil o descoherente respecto al TCP.

### 6.11 CARRY / TRANSPORT

- Estado: actual confirmado
- Objetivo: Transportar el objeto manteniendo coherencia física entre pose del objeto y TCP.
- Target: Ruta desde pick hasta cesta / home intermedio según pipeline.
- Frame: base_link y validación cruzada con world
- Método: Backend follow_tcp como semántica base; pick_demo entra por demo_transport con world_locked por defecto.
- Criterio de éxito: Carry validation OK y seguimiento coherente del objeto con el TCP.
- Criterio de fallo: carry_follow_lost, object_not_updated, best_lift_delta < 0, best_tcp_dist > máximo o stale_tcp_pose_soft_follow degradando el seguimiento.
- Criterio de skip: No se omite cuando el objeto sigue adjunto lógicamente y debe ir a entrega.
- Logs relevantes: [ATTACH_BACKEND] demo_transport_follow_tick ..., [PICK][DIRECT][FINAL_TRACE] phase=CARRY ...
- Variables de entorno asociadas: ATTACH_BACKEND_* y PANEL_PICK_DEMO_CARRY_*.
- Observaciones y discrepancias: Mantener explícita la discrepancia entre metadata de fase, llamada real y telemetría; no simplificarla.

### 6.12 HOME_WITH_OBJECT

- Estado: actual confirmado parcialmente
- Objetivo: Pasar por una postura segura con objeto antes o durante el transporte largo.
- Target: Pose segura intermedia de home con objeto aún transportado.
- Frame: joint space / base_link según implementación concreta.
- Método: Preset o IK intermedio con validación carry específica para home_with_object.
- Criterio de éxito: _validate_demo_carry(home_with_object) con timeout=1.2s, min_obj_move=0.080 m, min_lift_delta=0.060 m y max_tcp_dist default=0.200 m.
- Criterio de fallo: Carry deja de ser coherente durante el retorno intermedio.
- Criterio de skip: Si la ruta actual salta directamente a cesta o la fase no aplica.
- Logs relevantes: Trazas home_with_object y validación carry asociada.
- Variables de entorno asociadas: PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M y thresholds asociados.
- Observaciones y discrepancias: La fase existe en el código actual aunque el documento 2026-04-18 la dejaba más implícita que explícita.

### 6.13 CESTA

- Estado: histórico recuperado + target actual inferido del world file
- Objetivo: Posicionar el TCP sobre la bandeja de depósito antes de soltar.
- Target: bandeja_deposito en world≈(-1.300, 0.000, 0.785), con altura efectiva según release del pipeline.
- Frame: world para geometría de escena; base_link para ejecución.
- Método: Preset/IK/MoveIt según ruta actual del panel.
- Criterio de éxito: TCP en zona de entrega sin perder coherencia del objeto.
- Criterio de fallo: No converge la ruta o el objeto deja de seguir al TCP antes de soltar.
- Criterio de skip: Si se aborta antes del transporte final.
- Logs relevantes: [PICK][DIRECT][TRANSPORT] y trazas CESTA/CESTA_RELEASE.
- Variables de entorno asociadas: Sin variable exclusiva confirmada en esta corrida; depende del pipeline de entrega.
- Observaciones y discrepancias: La geometría actual de la cesta se reconfirma desde worlds/ur5_mesa_objetos.sdf; los detalles finos de altura de release siguen parte históricos.

### 6.14 RELEASE / CESTA_RELEASE

- Estado: histórico recuperado + actual parcialmente revalidado
- Objetivo: Abrir el gripper y desacoplar el objeto para dejarlo en la cesta.
- Target: Gripper abierto + detach lógico/físico.
- Frame: joint space (gripper) y backend attach/detach.
- Método: Comando de apertura + detach del backend/Gazebo.
- Criterio de éxito: Apertura confirmada y objeto liberado del backend de attach.
- Criterio de fallo: Timeout de apertura, detach no procesado o objeto sigue attached.
- Criterio de skip: No se omite si el ciclo llega a la cesta.
- Logs relevantes: [PICK][DIRECT][CESTA_RELEASE], detach_request_received, gazebo_detach_applied=...
- Variables de entorno asociadas: PANEL_PICK_DEMO_RELEASE_*.
- Observaciones y discrepancias: Conviene validarlo también con topics del backend actual, no sólo con la semántica legacy de gripper_anchor.

### 6.15 HOME_FINAL

- Estado: histórico revalidado por labels actuales
- Objetivo: Retornar a postura segura tras release o tras abortar en fase tardía.
- Target: Preset HOME final.
- Frame: joint space
- Método: Preset articular final.
- Criterio de éxito: Robot estable y ciclo cerrado en estado seguro.
- Criterio de fallo: Timeout o controlador no listo.
- Criterio de skip: Puede omitirse sólo en abortos tempranos con parada manual.
- Logs relevantes: Trazas HOME_FINAL / retorno a home.
- Variables de entorno asociadas: Sin variable exclusiva confirmada; usa timeouts generales de movimiento.
- Observaciones y discrepancias: Se conserva por trazabilidad operativa y por cierre de ciclo completo.

### 6.99 Enlace obligatorio con la validación física post-grasp

Las fases `ATTACH_GATE`, `LIFT`, `CARRY / TRANSPORT` y `HOME_WITH_OBJECT` deben interpretarse junto con el bloque especializado de validación física post-grasp / carry. `ATTACH_GATE` aprueba attach lógico; la confirmación física sólo llega cuando CARRY supera sus thresholds y telemetría coherente.

### Detalle histórico recuperado (2026-04-18)

#### 5.1 Diagrama de fases

```
┌─────────────┐
│    HOME     │ → Preset articular: JOINT_TABLE_POSE_RAD
└──────┬──────┘
       ↓
┌─────────────┐
│    MESA     │ → Preset articular: JOINT_PICK_IMAGE_POSE_RAD
└──────┬──────┘
       ↓
┌──────────────────┐
│ APPROACH_COARSE  │ → IK directo, TCP sobre objeto +35 mm
└──────┬───────────┘
       ↓
┌──────────────────┐
│  GRASP_DOWN      │ → Descenso cartesiano por segmentos IK (5 mm/seg)
└──────┬───────────┘
       ↓
┌──────────────────┐
│ GRASP_ALIGN_IK   │ → Loop bias Z hasta alineación (<10 mm error)
└──────┬───────────┘
       ↓
┌──────────────────┐
│   PRE_CLOSE      │ → Validación final de pose antes de cerrar
└──────┬───────────┘
       ↓
┌──────────────────┐
│     CLOSE        │ → Cerrar gripper + confirmación delta joints
└──────┬───────────┘
       ↓
┌──────────────────┐
│  ATTACH_GATE     │ → Validación temporal objeto-TCP antes de attach
└──────┬───────────┘
       ↓
┌──────────────────┐
│      LIFT        │ → Elevar objeto seguro fuera de la mesa
└──────┬───────────┘
       ↓
┌──────────────────┐
│   TRANSPORT      │ → IK hacia posición cesta
└──────┬───────────┘
       ↓
┌──────────────────┐
│ HOME_WITH_OBJECT │ → Pose segura intermedia (si aplica)
└──────┬───────────┘
       ↓
┌──────────────────┐
│     CESTA        │ → Preset/IK sobre cesta
└──────┬───────────┘
       ↓
┌──────────────────┐
│ CESTA_RELEASE    │ → Abrir gripper + detach Gazebo
└──────┬───────────┘
       ↓
┌──────────────────┐
│   HOME_FINAL     │ → Retorno a HOME
└──────────────────┘
```

---

#### 5.2 Fase HOME

| Campo | Detalle |
|---|---|
| **Objetivo** | Llevar el robot a posición articular de reposo segura |
| **Target** | `JOINT_TABLE_POSE_RAD` (constante en `panel_robot_presets.py`) |
| **Frame** | Joint space (no cartesiano) |
| **Método** | Preset articular vía `joint_trajectory_controller` |
| **Criterio éxito** | Joints convergen dentro de tolerancia articular |
| **Criterio fallo** | Timeout de movimiento (150 s por defecto) |
| **Criterio skip** | Robot ya está en HOME (check pre-fase) |
| **Logs** | `[PICK][HOME] Moving to home preset` |
| **Env vars** | `PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC=150.0` |

---

#### 5.3 Fase APPROACH_COARSE

| Campo | Detalle |
|---|---|
| **Objetivo** | Posicionar TCP (rg2_pinch_center) sobre el objeto con margen de seguridad |
| **Target** | `(x_obj, y_obj, z_obj_base + APPROACH_COARSE_EXTRA_Z_M)` en base_link |
| **Frame** | `base_link` (target IK), `rg2_pinch_center` (TCP) |
| **Método** | IK directo (`ur5_kinematics.ik_ur5`) |
| **Criterio éxito** | Gate: XY < 12 mm, Z < 12 mm (`APPROACH_COARSE_GATE_XY/Z_TOL_M`) |
| **Criterio fallo** | IK no converge o error > umbral tras settle |
| **Criterio skip** | Ninguno (siempre se ejecuta) |
| **Logs** | `[PICK][DIRECT][APPROACH_COARSE] target=... gate_xy=... gate_z=...` |
| **Env vars** | `PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M=0.035`, `PANEL_PICK_DEMO_APPROACH_COARSE_GATE_XY_TOL_M=0.012`, `PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M=0.012` |

**Lógica de target:**

```python
target_z = object_z_in_base_link + APPROACH_COARSE_EXTRA_Z_M  # +35 mm sobre objeto
target = (object_x, object_y, target_z)  # en base_link
# Convertir a frame IK: negate XY
ik_result = ik_ur5((-target_x, -target_y, target_z - 0.175), seed=current_joints)
```

---

#### 5.4 Fase GRASP_DOWN

| Campo | Detalle |
|---|---|
| **Objetivo** | Descender el TCP hasta la altura de agarre (z_contact) de forma controlada |
| **Target** | `(x_obj, y_obj, z_obj_base + GRASP_CONTACT_Z_OFFSET_M)` en base_link |
| **Frame** | `base_link` |
| **Método** | MoveIt `computeCartesianPath` (si `GRASP_DOWN_USE_MOVEIT_CARTESIAN=1`) o IK segmentado (5 mm/step) |
| **Criterio éxito** | XY < 8 mm, Z < 8 mm, dist3D < 12 mm (STRICT tolerances) |
| **Criterio fallo** | Error > umbral tras 4 intentos, IK no converge en segmento |
| **Criterio skip** | No hay skip |
| **Logs** | `[PICK][DIRECT][GRASP_DOWN] segment=N/7 target_z=... xy_err=... z_err=...` |
| **Env vars** | Ver tabla completa en Sección 6 |

**Lógica IK segmentada:**

```python
# Divide el descenso en segmentos de GRASP_DOWN_SEGMENT_Z_STEP_M (5 mm)
# Aproximadamente 7 segmentos para bajar ~35 mm
for seg_z in np.arange(approach_z, contact_z, -0.005):
    ik_result = ik_ur5(target_at_seg_z, seed=prev_joints, weight=0.65)
    if validate(ik_result):
        execute(ik_result)
        prev_joints = ik_result
    else:
        retry (max 4 attempts per segment)
```

**Por qué segmentado:** Evitar cambios de "rama" de solución IK. Con steps > 17 mm, el solver puede saltar a otra configuración articular causando errores de hasta 48 mm en Y. 5 mm/step mantiene el solver en la misma rama.

---

#### 5.5 Fase GRASP_ALIGN_IK

| Campo | Detalle |
|---|---|
| **Objetivo** | Compensar el residual Z sistemático entre modelo DH y SDF (~13 mm) |
| **Target** | `(x_obj, y_obj, z_contact)` con bias Z ajustado iterativamente |
| **Frame** | `base_link` |
| **Método** | Loop IK con bias adaptativo en Z |
| **Criterio éxito** | XY < 10 mm (`ALIGN_EXIT_XY_TOL_M`) **y** Z < 10 mm (`ALIGN_EXIT_Z_TOL_M`) |
| **Criterio fallo** | Más de 3 intentos sin convergencia |
| **Criterio skip** | Si XY y Z ya cumplen desde GRASP_DOWN |
| **Logs** | `[PICK][DIRECT][GRASP_ALIGN_IK] attempt=N z_err=... bias=... align_ok=True/False` |
| **Env vars** | `PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL=0.08`, `PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M=0.010`, `PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M=0.010`, `PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M=0.008` |

**Lógica bias loop:**

```python
target_z = z_contact  # objetivo nominal
for attempt in range(3):
    execute_ik(target_x, target_y, target_z)
    real_z = fk_rg2_pinch_center_z()  # FK real del TCP
    z_error = real_z - z_contact       # error residual
    if abs(z_error) > ALIGN_Z_RESIDUAL_TOL (8mm):
        target_z -= z_error  # bias: corrige el target
    if xy_error < 10mm and abs(z_error) < 10mm:
        align_ok = True
        break
```

---

#### 5.6 Fase PRE_CLOSE

| Campo | Detalle |
|---|---|
| **Objetivo** | Validación final de alineación antes de ejecutar cierre de pinza |
| **Target** | Pose actual del TCP vs objeto |
| **Frame** | `base_link` |
| **Método** | Lectura FK + comparación con pose objetivo |
| **Criterio éxito** | XY < 10 mm, Z_err < 10 mm (`PRE_CLOSE_XY/Z_ERR_TOL_M`) |
| **Criterio fallo** | Error fuera de umbral tras reintentos |
| **Criterio skip** | `PANEL_PICK_DEMO_SKIP_ALIGN_IF_REACHABLE=1` (si configurado) |
| **Logs** | `[PICK][DIRECT][PRE_CLOSE] xy_err=... z_err=...` |
| **Env vars** | `PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M=0.010`, `PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M=0.010`, `PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES=2` |

---

#### 5.7 Fase CLOSE

| Campo | Detalle |
|---|---|
| **Objetivo** | Cerrar el gripper RG2 hasta la posición de agarre |
| **Target** | `rg2_finger_joint1 = rg2_finger_joint2 = 1.18 rad` |
| **Frame** | Joint space (gripper) |
| **Método** | Comando `gripper_controller` + espera confirmación delta joints |
| **Criterio éxito** | Suma delta joints > `CLOSE_MIN_DELTA_SUM` (0.01 m) en < timeout |
| **Criterio fallo** | Timeout (`CLOSE_CONFIRM_TIMEOUT_SEC=3.0 s`) sin delta suficiente |
| **Criterio skip** | Ninguno |
| **Logs** | `[PICK][DIRECT][CLOSE] delta_sum=... confirmed=True/False` |
| **Env vars** | `PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC=3.0`, `PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM=0.01`, `PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M=0.12`, `PANEL_PICK_DEMO_CLOSE_XY_TOL_M=0.008`, `PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M=0.008` |

**Confirmación de cierre:**

```python
# Leer /joint_states continuamente
# Calcular: delta = |current_j1 - initial_j1| + |current_j2 - initial_j2|
# Confirmar si delta > CLOSE_MIN_DELTA_SUM (0.01 m = 10 mm)
# Timeout si no ocurre en CLOSE_CONFIRM_TIMEOUT_SEC (3 s)
```

---

#### 5.8 Fase ATTACH_GATE

| Campo | Detalle |
|---|---|
| **Objetivo** | Verificar que el objeto está bien sujeto antes de levantar |
| **Target** | Estabilidad TCP-objeto en ventana temporal |
| **Frame** | `base_link` (distancias TCP↔objeto) |
| **Método** | `AttachGateEvaluator` — ventana temporal de 350 ms con 5+ muestras |
| **Criterio éxito** | dist_TCP_obj < 40 mm, drift < 12 mm, gripper cerrado, backend OK |
| **Criterio fallo** | Cualquier condición no cumplida en ventana temporal |
| **Criterio skip** | Ninguno (gate crítico de seguridad) |
| **Logs** | `[ATTACH_GATE][CHECK] dist=... drift=... closed=T/F` / `[ATTACH_GATE][PASS]` / `[ATTACH_GATE][FAIL]` |
| **Env vars** | `PANEL_PICK_DEMO_ATTACH_XY_TOL_M=0.008`, `PANEL_PICK_DEMO_ATTACH_Z_TOL_M=0.010`, `PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M=0.040`, `PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M=0.012`, `PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC=0.35`, `PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES=5` |

**Condiciones del gate (todas deben cumplirse):**

1. `dist(rg2_pinch_center, objeto) < 40 mm`
2. `drift(objeto, TCP) < 12 mm` en ventana 350 ms
3. `suma_joints_gripper < 20 mm` (gripper cerrado)
4. Backend Gazebo confirma detachable joint creado

---

#### 5.9 Fase LIFT

| Campo | Detalle |
|---|---|
| **Objetivo** | Elevar el objeto suficientemente para despejarlo de la mesa |
| **Target** | TCP actual + delta_Z (altura de seguridad ~150 mm sobre mesa) |
| **Frame** | `base_link` |
| **Método** | IK directo o MoveIt cartesiano (subida vertical) |
| **Criterio éxito** | FK Z del TCP > umbral de seguridad |
| **Criterio fallo** | `min_lift_delta` no alcanzado (objeto se cae), timeout |
| **Criterio skip** | Ninguno |
| **Logs** | `[PICK][DIRECT][LIFT] lift_delta=... min_lift_delta=0.060` |
| **Env vars** | `min_lift_delta` (actualmente 0.060 m, tuneado en 2026-04-07 desde 0.025) |

**Nota:** `min_lift_delta` fue aumentado de 0.025 a 0.060 para evitar falso positivo de grasp donde el sistema detectaba éxito antes de que el objeto se levantara realmente.

---

#### 5.10 Fase CARRY / TRANSPORT

| Campo | Detalle |
|---|---|
| **Objetivo** | Transportar objeto desde posición de pick hasta zona de entrega |
| **Target** | Pose sobre cesta: `(-1.30, 0.00, 0.82 + margen)` en world → base_link |
| **Frame** | `base_link` |
| **Método** | IK directo o MoveIt con waypoints |
| **Criterio éxito** | TCP en zona de entrega dentro de tolerancia |
| **Criterio fallo** | IK no converge, colisión, objeto se suelta en transporte |
| **Logs** | `[PICK][DIRECT][TRANSPORT] target=...` |

---

#### 5.11 Fase CESTA_RELEASE

| Campo | Detalle |
|---|---|
| **Objetivo** | Soltar el objeto en la cesta |
| **Target** | Gripper abierto (0.0 rad ambos joints) |
| **Frame** | Joint space (gripper) |
| **Método** | Comando `gripper_controller` apertura + `detach` topic Gazebo |
| **Criterio éxito** | Gripper abierto + detachable joint eliminado |
| **Criterio fallo** | Timeout |
| **Logs** | `[PICK][DIRECT][CESTA_RELEASE] detach=True` |

---

#### 5.12 Fase HOME_FINAL

| Campo | Detalle |
|---|---|
| **Objetivo** | Retornar a posición de reposo tras completar el ciclo |
| **Target** | `JOINT_TABLE_POSE_RAD` (igual que HOME inicial) |
| **Método** | Preset articular |
| **Criterio éxito** | Joints en HOME dentro de tolerancia |

---

## 7. Variables de Entorno y Parámetros Críticos

### 7.0 Cómo leer esta sección

- `Launch`: default declarado en `ur5_stack.launch.py`.
- `Runtime wrapper`: valor efectivo exportado por `start_panel_v2.sh` y por `panel_runtime_validated.env` cuando el wrapper lo sourcea.
- `Fallback panel`: valor local que usa `panel_pick_demo.py` si lee la variable directamente y no recibe otra cosa.
- El inventario exhaustivo completo se conserva en el apéndice 15.5; aquí se muestran sólo parámetros de alto impacto operativo.

### 7.1 Geometría, descenso y alineación

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado |
|---|---|---|---|---|---|
| PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M | 0.020 | 0.020 | - | 0.010 | discrepancia abierta |
| PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M | 0.010 | - | - | 0.010 | default launch |
| PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL | 0.200 | 0.200 | - | 0.08 | discrepancia abierta |
| PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M | 0.008 | - | - | 0.008 | default launch |
| PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M | 0.035 | - | 0.10 | 0.035 | discrepancia abierta |
| PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M | - | - | - | - | alineado |
| PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL | 0.200 | 0.200 | 0.080 | - | discrepancia abierta |
| PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT | 0.035 | 0.035 | 0.65 | 0.65 | discrepancia abierta |
| PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M | 0.005 | - | 0.005 | 0.005 | default launch |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M | 0.012 | 0.025 | 0.025 | 0.012 | discrepancia abierta |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M | 0.015 | - | 0.012 | 0.008 | discrepancia abierta |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M | 0.008 | 0.025 | 0.025 | 0.008 | discrepancia abierta |
| PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN | 1 | - | 1 | 1 | default launch |
| PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M | 0.020 | 0.016 | - | 0.010 | discrepancia abierta |
| PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M | 0.010 | - | - | 0.010 | default launch |

### 7.2 Cierre, attach y carry

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado |
|---|---|---|---|---|---|
| ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS | - | pick_demo | - | - | override runtime activo |
| ATTACH_BACKEND_FOLLOW_BREAK_DIST_M | 0.18 | 0.45 | - | - | discrepancia abierta |
| ATTACH_BACKEND_FOLLOW_RATE_HZ | 20.0 | 10.0 | - | - | discrepancia abierta |
| ATTACH_BACKEND_MAX_DIST_M | 0.08 | 0.06 | - | 0.06 | discrepancia abierta |
| ATTACH_BACKEND_MAX_POSE_AGE_SEC | 1.5 | 2.5 | - | - | discrepancia abierta |
| ATTACH_BACKEND_MODE | follow_tcp | follow_tcp | - | - | override runtime activo |
| PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M | 0.040 | 0.080 | 0.040 | 0.040 | discrepancia abierta |
| PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M | 0.012 | 0.050 | 0.012 | 0.012 | discrepancia abierta |
| PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES | 5 | 3 | 5 | 5 | discrepancia abierta |
| PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC | 0.35 | 0.25 | 0.35 | 0.35 | discrepancia abierta |
| PANEL_PICK_DEMO_ATTACH_XY_TOL_M | 0.020 | 0.020 | 0.012 | 0.008 | discrepancia abierta |
| PANEL_PICK_DEMO_ATTACH_Z_TOL_M | 0.010 | - | 0.015 | 0.010 | discrepancia abierta |
| PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M | - | 0.500 | 0.200 | - | discrepancia abierta |
| PANEL_PICK_DEMO_CARRY_SETTLE_SEC | - | - | 3.0 | - | fallback local panel |
| PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC | 3.0 | 120 | - | 3.0 | discrepancia abierta |
| PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM | 0.01 | - | - | 0.01 | default launch |
| PANEL_PICK_DEMO_CLOSE_XY_TOL_M | 0.008 | - | 0.012 | 0.008 | discrepancia abierta |
| PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M | 0.008 | - | 0.012 | 0.008 | discrepancia abierta |

### 7.3 Frescura de pose y MoveIt bridge

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado |
|---|---|---|---|---|---|
| PANEL_MOVEIT_BRIDGE_ACCEL_SCALE | - | 0.35 | - | 0.30 | discrepancia abierta |
| PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC | - | 25.0 | - | 150.0 | discrepancia abierta |
| PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC | - | 2.5 | - | 2.5 | override runtime activo |
| PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC | - | 6.0 | - | 6.0 | override runtime activo |
| PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC | - | 80.0 | - | 180.0 | discrepancia abierta |
| PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE | - | 0.35 | - | 0.30 | discrepancia abierta |
| PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M | 0.003 | - | - | 0.003 | default launch |
| PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC | 2.5 | - | - | 2.5 | default launch |
| PANEL_PICK_DEMO_PHASE_JUMP_TOL_M | 0.010 | - | - | 0.010 | default launch |
| PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC | 0.400 | 300.0 | - | 0.400 | discrepancia abierta |
| PANEL_PICK_DEMO_POSE_SOURCE_TOL_M | 0.006 | 1.0 | - | 0.006 | discrepancia abierta |

### 7.4 Guardrails de arranque y corredor de cesta

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado |
|---|---|---|---|---|---|
| PANEL_CONTROLLER_READY_TIMEOUT_SEC | - | 60.0 | - | - | override runtime activo |
| PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD | - | 0.20 | - | - | override runtime activo |
| PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD | - | 4.0 | - | - | override runtime activo |
| PANEL_MOVEIT_STARTUP_TIMEOUT_SEC | - | 300 | - | - | override runtime activo |
| PANEL_PICK_DEMO_TRANSPORT_POSTCHECK_MODEL_TOL_M | - | 0.040 | - | - | override runtime activo |
| PANEL_PICK_DEMO_TRANSPORT_PREEXEC_MODEL_TOL_M | - | 0.012 | - | - | override runtime activo |
| PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_STAGE_DIST_M | - | 0.050 | - | - | override runtime activo |
| PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_REMAINING_DIST_M | - | 0.060 | - | - | override runtime activo |
| PANEL_RUNTIME_VALIDATED_PROFILE | - | rg2_tcp_fix_20260422 | - | - | override runtime activo |
| STRICT_SELF_COLLISION | - | 1 | - | - | override runtime activo |
| SYSTEM_STATE_GEOMETRY_OFFSET_TOL_M | 0.002 | 0.002 | - | - | override runtime activo |
| SYSTEM_STATE_GEOMETRY_PAIR_TOL_M | 0.001 | 0.001 | - | - | override runtime activo |
| SYSTEM_STATE_STARTUP_TIMEOUT_SEC | 15.0 | 15.0 | - | - | override runtime activo |

### 7.5 Lectura efectiva del runtime

- `start_panel_v2.sh` exporta defaults base y luego sourcea `panel_runtime_validated.env`.
- Si una variable ya fue exportada por el wrapper, el perfil validado no la pisa; si no existe aún, el perfil la completa.
- Esto permite documentar en un mismo inventario tanto los defaults base del launcher como los guardrails añadidos tras el cierre TCP del `2026-04-22` y el follow-up de cesta del `2026-04-23`.

### 7.4 Discrepancias de parámetros

| Variable | Grupo | Launch | Runtime wrapper | Fallback panel | Histórico | Lectura |
|---|---|---|---|---|---|---|
| ATTACH_BACKEND_FOLLOW_BREAK_DIST_M | backend attach/transporte | 0.18 | 0.45 | - | - | actual difiere entre fuentes; wrapper pisa launch |
| ATTACH_BACKEND_FOLLOW_RATE_HZ | backend attach/transporte | 20.0 | 10.0 | - | - | actual difiere entre fuentes; wrapper pisa launch |
| ATTACH_BACKEND_MAX_DIST_M | backend attach/transporte | 0.08 | 0.06 | - | 0.06 | actual difiere entre fuentes; wrapper pisa launch |
| ATTACH_BACKEND_MAX_POSE_AGE_SEC | backend attach/transporte | 1.5 | 2.5 | - | - | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_MOVEIT_BRIDGE_ACCEL_SCALE | MoveIt bridge | - | 0.35 | - | 0.30 | histórico=0.30 |
| PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC | MoveIt bridge | - | 25.0 | - | 150.0 | histórico=150.0 |
| PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC | MoveIt bridge | - | 80.0 | - | 180.0 | histórico=180.0 |
| PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE | MoveIt bridge | - | 0.35 | - | 0.30 | histórico=0.30 |
| PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M | GRASP_ALIGN_IK | 0.020 | 0.020 | - | 0.010 | histórico=0.010 |
| PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL | GRASP_ALIGN_IK | 0.200 | 0.200 | - | 0.08 | histórico=0.08 |
| PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M | APPROACH_COARSE | 0.035 | - | 0.10 | 0.035 | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M | APPROACH_COARSE | 0.012 | - | 0.008 | 0.012 | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M | ATTACH_GATE | 0.040 | 0.080 | 0.040 | 0.040 | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M | ATTACH_GATE | 0.012 | 0.050 | 0.012 | 0.012 | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES | ATTACH_GATE | 5 | 3 | 5 | 5 | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_ATTACH_SETTLE_SEC | ATTACH_GATE | - | 4.0 | 1.8 | - | actual difiere entre fuentes |
| PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC | ATTACH_GATE | 0.35 | 0.25 | 0.35 | 0.35 | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_ATTACH_XY_TOL_M | ATTACH_GATE | 0.020 | 0.020 | 0.012 | 0.008 | actual difiere entre fuentes; histórico=0.008; panel usa fallback distinto |
| PANEL_PICK_DEMO_ATTACH_Z_TOL_M | ATTACH_GATE | 0.010 | - | 0.015 | 0.010 | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M | LIFT / CARRY | - | 0.500 | 0.200 | - | actual difiere entre fuentes |
| PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC | CLOSE | 3.0 | 120 | - | 3.0 | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_CLOSE_XY_TOL_M | CLOSE | 0.008 | - | 0.012 | 0.008 | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M | CLOSE | 0.008 | - | 0.012 | 0.008 | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK | GRASP_DOWN | 1 | 0 | - | - | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL | GRASP_DOWN | 0.200 | 0.200 | 0.080 | - | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT | GRASP_DOWN | 0.035 | 0.035 | 0.65 | 0.65 | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M | GRASP_DOWN | 0.005 | 0.015 | - | 0.003 | actual difiere entre fuentes; histórico=0.003; wrapper pisa launch |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M | GRASP_DOWN | 0.012 | 0.025 | 0.025 | 0.012 | actual difiere entre fuentes; wrapper pisa launch; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M | GRASP_DOWN | 0.015 | - | 0.012 | 0.008 | actual difiere entre fuentes; histórico=0.008; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M | GRASP_DOWN | 0.008 | 0.025 | 0.025 | 0.008 | actual difiere entre fuentes; wrapper pisa launch; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_UTIL_XY_TOL_M | GRASP_DOWN | 0.015 | - | 0.012 | - | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC | freshness / pose source | 0.400 | 300.0 | - | 0.400 | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_POSE_SOURCE_TOL_M | freshness / pose source | 0.006 | 1.0 | - | 0.006 | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M | PRE_CLOSE | 0.020 | 0.016 | - | 0.010 | actual difiere entre fuentes; histórico=0.010; wrapper pisa launch |
| PANEL_PICK_DEMO_SKIP_ALIGN_IF_REACHABLE | GRASP_ALIGN_IK | - | - | 1 | 0 | histórico=0 |
| PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC | otros | - | 60 | 0 | - | actual difiere entre fuentes |

### 7.98 Discrepancias explícitas que no deben perderse

- `PANEL_PICK_DEMO_ATTACH_XY_TOL_M`: actual en launch=0.020; histórico 2026-04-18 documentado como 0.008 en la tabla anterior. Mantener ambos valores con trazabilidad.
- `ATTACH_BACKEND_MAX_POSE_AGE_SEC`: launch actual=1.5; wrapper `start_panel_v2.sh` exporta 2.5 por defecto.
- `ATTACH_BACKEND_MAX_DIST_M`: launch actual=0.08; wrapper `start_panel_v2.sh` exporta 0.06 por defecto.
- CARRY metadata vs llamada real: metadata CARRY=(0.030, 0.060, 0.080) frente a llamada post_grasp_lift=(0.020, 0.025, 0.120).
- `SYSTEM_STATE_GEOMETRY_OFFSET_TOL_M` y `SYSTEM_STATE_GEOMETRY_PAIR_TOL_M` viven hoy en `panel_runtime_validated.env`; perderlos borraría del documento el gate geométrico de arranque introducido tras el cierre TCP.
- `PANEL_PICK_DEMO_TRANSPORT_PREEXEC_MODEL_TOL_M`, `PANEL_PICK_DEMO_TRANSPORT_POSTCHECK_MODEL_TOL_M` y `PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_*` son claves para entender el atasco residual de cesta descrito en el follow-up `2026-04-23`.

### 7.99 Nota sobre trazabilidad histórica

- El bloque histórico extenso de variables ya no se inserta entero en el cuerpo principal para evitar ruido operativo.
- Su contenido queda absorbido en la columna `Histórico` y en la columna `Discrepancia` del apéndice 15.5, donde se conserva el inventario exhaustivo actual con trazabilidad de overrides y defaults previos.

## 8. Validación Física Post-Grasp / Carry

### 8.1 Estado actual confirmado

- ATTACH_GATE correcto no equivale a grasp físico confirmado.
- `follow_confirmed_only_after_carry` en fuente actual: confirmado.
- `pick_demo` sigue entrando por `demo_transport` porque `ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS=pick_demo` y el backend activa `use_world_locked_pose=True` en esa rama.
- `attach_backend_mode` por launch sigue siendo `follow_tcp` y representa la semántica base para objetos no desviados a demo transport.
- `stale_tcp_pose_soft_follow` aparece cuando `tcp_age` supera `max_pose_age_sec=1.5`; los logs inspeccionados muestran separación efectiva entre ticks world_locked de 2.492 s entre dos ticks consecutivos.
- El follow-up de `2026-04-23` separa un incidente residual de cesta/IK con señal principal `CESTA_STAGE_1_RECOVER_1_postcheck_failed, runtime_target_dist=0.055/0.040, model_target_err=0.055/0.040`; no debe leerse como reapertura del fix TCP.

### 8.2 Diagnóstico físico por métricas best_*

- `best_obj_move` bajo umbral: el objeto no se movió lo suficiente desde su pose inicial. Evidencia actual: `[2026-04-18T20:23:30] [PICK][DIRECT][FINAL_TRACE] phase=CARRY event=wait_done expected=carry_validation_ok received=false timeout=1.60 request=none logical=CARRIED physical=none reason=demo_carry_validation_failed phase=post_grasp_lift best_obj_move=0.000 best_lift_delta=0.000 best_tcp_dist=0.098 fail_reasons=obj_move_below_min,lift_delta_below_min,tcp_dist_above_max last_obj_world=(-0.420,0.000,0.875) last_obj_base=(0.430,0.000,0.025) last_tcp_base=(0.445,0.002,0.144)`.
- `best_lift_delta < 0`: el objeto cambió de pose, pero no acompañó el lift del TCP. Evidencia actual: `[2026-04-19T16:52:50] [PICK][DIRECT][FINAL_TRACE] phase=CARRY event=wait_done expected=carry_validation_ok received=false timeout=1.60 request=none logical=CARRIED physical=none reason=demo_carry_validation_failed phase=post_grasp_lift best_obj_move=0.870 best_lift_delta=-0.194 best_tcp_dist=0.881 fail_reasons=lift_delta_below_min,tcp_dist_above_max carry_detail=carry_follow_lost last_obj_world=(0.396,0.222,0.681) last_obj_base=(1.246,0.222,-0.169) last_tcp_base=(0.440,-0.001,0.125)`.
- `best_tcp_dist > máximo`: hubo movimiento, pero no coherente con un grasp estable respecto al TCP; esto invalida el carry aunque el objeto se haya desplazado.
- `object_not_updated / object_never_moved`: evidencia actual `[2026-04-19T18:24:30] [PICK][DIRECT][PHYSICS] phase=post_grasp_lift carry_follow_lost=true detail=object_never_moved best_obj_move=0.000 hint=check_demo_transport_set_pose_ok_in_backend_logs last_obj_world=(-0.420,0.000,0.875)`.

### 8.3 Discrepancias abiertas que deben quedar explícitas

- Metadata de fase CARRY: min_obj_move=0.030, min_lift_delta=0.060, max_tcp_dist=0.080.
- Llamada real `_validate_demo_carry(post_grasp_lift)`: min_obj_move=0.020, min_lift_delta=0.025, max_tcp_dist=0.120, timeout=3.0.
- La telemetría `FINAL_TRACE wait_done` sigue pudiendo cerrar con timeout 1.60 aunque la llamada real sea 3.0 s; esto es inconsistencia de observabilidad, no ruido cosmético.

### Bloque vigente preservado íntegro (2026-04-20)

- No disponible en las fuentes cargadas.

## 9. Controladores, Topics y Semántica de Attach

### 9.1 Controladores ros2_control activos y función

| Controlador | Tipo | Joints | Topic comando / salida | Estado |
|---|---|---|---|---|
| `joint_state_broadcaster` | joint_state_broadcaster/JointStateBroadcaster | todos los joints | `/joint_states` | Actual confirmado en ur5_mock_controllers.yaml |
| `joint_trajectory_controller` | joint_trajectory_controller/JointTrajectoryController | shoulder_pan .. wrist_3 | `/joint_trajectory_controller/joint_trajectory` | Actual confirmado en ur5_mock_controllers.yaml |
| `gripper_controller` | forward_command_controller/ForwardCommandController | rg2_finger_joint1/2 | `/gripper_controller/commands` | Actual confirmado en ur5_mock_controllers.yaml |

### 9.2 Topics principales y quién publica / consume

| Topic / patrón | Dirección | Quién publica / expone | Quién consume |
|---|---|---|---|
| `/joint_states` | PUB | joint_state_broadcaster | panel, bridge MoveIt, diagnósticos |
| `/tf` y `/tf_static` | PUB | RSP, gz_pose_bridge, world_tf_publisher | todos los consumidores TF |
| `/desired_grasp/request` | PUB | panel | ur5_moveit_bridge |
| `/desired_grasp/result` | PUB | ur5_moveit_bridge | panel |
| `/gripper/<obj>/attach` | SUB backend | panel / caller | gripper_attach_backend |
| `/gripper/<obj>/detach` | SUB backend | panel / caller | gripper_attach_backend |
| `/gripper/<obj>/state` | PUB backend | gripper_attach_backend | panel / diagnósticos |
| `/drop_anchor/<obj>/attach|detach|state` | PUB/SUB backend ↔ plugin Gazebo | gripper_attach_backend / plugin detachable | backend / Gazebo |
| `/gripper_anchor/<obj>/attach|detach` | PUB backend | gripper_attach_backend | plugin/ancla de tool anchor cuando aplica |
| `/world/ur5_mesa_objetos/pose/info` | PUB Gazebo bridge | ros_gz_bridge | world_tf_publisher y gripper_attach_backend |

### 9.3 Semántica actual de attach

- Prefijo lógico principal actual del backend: `/gripper` (`gripper_prefix=/gripper`).
- Prefijo de ancla de tool-anchor: `/gripper_anchor` (`tool_anchor_prefix=/gripper_anchor`).
- Prefijo de drop-anchor: `/drop_anchor` (`drop_anchor_prefix=/drop_anchor`).
- Modo nominal global: `follow_tcp`; pero `pick_demo` entra por demo transport y usa `world_locked` por defecto dentro del backend actual.
- Esto obliga a documentar por separado attach lógico, attach por tool anchor y transporte físico real.

### 9.4 Comandos de verificación runtime

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [1.18, 1.18]" --once
ros2 topic echo /gripper/pick_demo/state --once
ros2 topic echo /gripper_anchor/pick_demo/state --once
ros2 run tf2_ros tf2_monitor world base_link rg2_pinch_center
```

### Detalle histórico recuperado (2026-04-18)

#### 7.1 Controladores ros2_control activos

| Controlador | Tipo | Joints | Topic comando | Frecuencia |
|---|---|---|---|---|
| `joint_state_broadcaster` | broadcaster | todos | — | 125 Hz |
| `joint_trajectory_controller` | trajectory | shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 | `/joint_trajectory_controller/joint_trajectory` | 125 Hz |
| `gripper_controller` | position | rg2_finger_joint1, rg2_finger_joint2 | `/gripper_controller/commands` | 125 Hz |

**Archivo de configuración:** `src/ur5_bringup/config/ur5_mock_controllers.yaml`

```yaml
controller_manager:
  update_rate: 125  # Hz

joint_trajectory_controller:
  joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
  command_interfaces: [position]
  state_interfaces: [position, velocity]

gripper_controller:
  joints:
    - rg2_finger_joint1
    - rg2_finger_joint2
  interface_name: position
```

#### 7.2 Tópicos principales

| Topic | Tipo msg | Dirección | Publicador | Suscriptor |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | PUB | `joint_state_broadcaster` | panel, MoveIt, FK |
| `/robot_description` | `std_msgs/String` | PUB | `robot_state_publisher` | MoveIt, rviz |
| `/tf` | `tf2_msgs/TFMessage` | PUB | `robot_state_publisher`, `gz_pose_bridge` | todos |
| `/tf_static` | `tf2_msgs/TFMessage` | PUB | `world_tf_publisher` | todos |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | SUB | `joint_trajectory_controller` | panel (pub) |
| `/gripper_controller/commands` | `std_msgs/Float64MultiArray` | SUB | `gripper_controller` | panel (pub) |
| `/gripper_anchor/pick_demo/attach` | `std_msgs/Empty` | SUB | Gazebo plugin | `gripper_attach_backend` (pub) |
| `/gripper_anchor/pick_demo/detach` | `std_msgs/Empty` | SUB | Gazebo plugin | `gripper_attach_backend` (pub) |
| `/gripper_anchor/pick_demo/state` | Gazebo msg | PUB | Gazebo plugin | `attach_gate_evaluator` |
| `/desired_grasp/request` | `geometry_msgs/PoseStamped[]` | SUB | `ur5_moveit_bridge` | panel (pub) |
| `/desired_grasp/result` | (moveit result) | PUB | `ur5_moveit_bridge` | panel (sub) |
| `/clock` | `rosgraph_msgs/Clock` | PUB | Gazebo | todos (use_sim_time=true) |

#### 7.3 Comandos de verificación en runtime

**Verificar que los controladores están activos:**

```bash
ros2 control list_controllers
# Esperado: joint_state_broadcaster [active], joint_trajectory_controller [active], gripper_controller [active]
```

**Verificar que el gripper recibe joint_states:**

```bash
ros2 topic echo /joint_states --once
# Buscar rg2_finger_joint1 y rg2_finger_joint2 en la lista
```

**Verificar TF world → base_link:**

```bash
ros2 run tf2_ros tf2_echo world base_link
# Esperado: translation (0, 0, 0.850), rotation identity
```

**Verificar TCP del gripper:**

```bash
ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
# Muestra la pose actual del TCP operacional
```

**Verificar pose del objeto en simulación:**

```bash
ros2 run tf2_ros tf2_echo world pick_demo
# Pose del objeto en Gazebo
```

**Mover gripper manualmente:**

```bash
# Abrir gripper
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0]" --once

# Cerrar gripper
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [1.18, 1.18]" --once
```

**Verificar estado attach backend:**

```bash
ros2 topic echo /gripper_anchor/pick_demo/state
```

#### 7.4 Cómo comprobar que el gripper realmente se mueve

El gripper **se mueve físicamente en Gazebo** si:

1. `/joint_states` muestra cambio en `rg2_finger_joint1` y `rg2_finger_joint2`
2. La vista 3D de Gazebo muestra los fingers moviéndose
3. El panel recibe confirmación de delta_sum > 0.01 m

**Si el gripper no se mueve a pesar del comando:**

1. Verificar que `gripper_controller` está `[active]` con `ros2 control list_controllers`
2. Verificar que no hay joint limit violation (joints en 0.0 y comando 0.0 → no mueve)
3. Verificar que el topic `/gripper_controller/commands` tiene suscriptores

#### 7.5 Validar TF y poses en runtime

**Árbol TF completo:**

```bash
ros2 run tf2_tools view_frames
# Genera frames.pdf con árbol completo
```

**Verificar edad de transforms (staleness):**

```bash
ros2 topic hz /tf
# Si baja frecuencia → TF puede estar stale
```

**Verificar que un frame específico está publicándose:**

```bash
ros2 run tf2_ros tf2_monitor world base_link rg2_pinch_center
```

---

## 10. Bugs Conocidos y Fixes Aplicados

### 10.1 Bugs / hallazgos actuales ligados a carry y attach

| Bug / hallazgo | Síntoma | Causa raíz | Cómo se detectó | Fix / tratamiento actual | Estado |
|---|---|---|---|---|---|
| Attach lógico aprobado pero transporte físico fallido | ATTACH_GATE puede pasar aunque CARRY falle | Diseño deliberadamente separado entre attach lógico y confirmación física | Código actual + logs FINAL_TRACE/CARRY | Mantener separación explícita; no tratar ATTACH_GATE como éxito final | Vigente / documentado |
| `world_locked` puede arrastrar una referencia retrasada | Objeto no sigue al TCP o queda incoherente | Demo transport usa `use_world_locked_pose=True` para `pick_demo` | Log world_locked: [gripper_attach_backend-11] [INFO] [1776753421.406565612] [gripper_attach_backend]: [ATTACH_BACKEND] demo_transport_follow_tick object=pick_demo mode=world_locked desired=(-0.414,-0.000,0.857) tcp=(-0.417,-0.000,0.872) tcp_src=base_chain | Ajustar frescura de pose y no ocultar la discrepancia frente a follow_tcp | Vigente / parcialmente mitigado |
| `stale_tcp_pose_soft_follow` degrada carry | Warnings de stale y carry_follow_lost | Pose TCP demasiado vieja respecto a max_pose_age=1.5 | Log stale: [gripper_attach_backend-11] [WARN] [1776753424.953371587] [gripper_attach_backend]: [ATTACH_BACKEND] stale_tcp_pose_soft_follow age=1.567s max=1.500s hard=4.500s src=world_tcp base_chain_ok=True base_chain_age=1.552 world_tcp_ok=True world_tcp_age=1.567 world_base_age=1.567 base_tcp_age=1.552 stable_world_base_ok=True stable_world_base_age=1.567 stable_world_base_mode=cached_fixed_base cache_ok=None cache_age=None tool_fallback_ok=True tool_fallback_age=1.567 world_base_ok=True base_tcp_ok=True | Se elevó max pose age en wrapper runtime, pero sigue siendo riesgo abierto | Vigente / riesgo abierto |
| Corredor de cesta / IK residual abierto el 2026-04-23 | DIRECTO falla en `CESTA_STAGE_*` aunque el TCP ya es correcto | Se aceptan soluciones IK que no reproducen el target cartesiano tras ejecución | Follow-up 2026-04-23: CESTA_STAGE_1_RECOVER_1_postcheck_failed, runtime_target_dist=0.055/0.040, model_target_err=0.055/0.040 | Añadir guard de modelo pre-ejecución, reseeding/branching y revalidar sin tocar el TCP | Abierto / incidente separado |
| Metadata, llamada real y telemetría de CARRY divergen | Timeouts y thresholds no coinciden según la fuente que se mire | Valores codificados en sitios distintos del panel | Inspección de panel_pick_demo.py + logs helper/stack | Conservar discrepancia anotada en vez de resumirla | Vigente / sin cierre |
| Diagnóstico best_* mal interpretado | Se concluye grasp o fallo con criterio insuficiente | Se ignora la relación entre best_obj_move, best_lift_delta y best_tcp_dist | Auditorías 2026-04-18/19 y documento actual | Mantener criterios especializados y troubleshooting específico | Vigente / documentado |

### 10.2 Cronología reciente de incidentes

- `2026-04-22`: `reports/incidents/2026-04-21_tcp_geometry_incident.md` documenta el cierre operativo del TCP con `tool0 -> rg2_tcp.z = +0.0050885`, batch `DIRECTO=5/5` y repetición manual final `2026-04-22` con `./lanzar_panelc2.sh`.
- `2026-04-23`: `agarre_ros2_ws/reports/incidents/2026-04-22_rg2_tcp_incident_closure.md` ratifica el cierre geométrico del TCP (`incidente TCP cerrado; follow-up funcional de cesta abierto`) y deja separado el corredor de cesta / IK con señal representativa `CESTA_STAGE_1_RECOVER_1_postcheck_failed, runtime_target_dist=0.055/0.040, model_target_err=0.055/0.040`.

### Detalle histórico recuperado (2026-04-18)

#### 8.1 Residual DH/SDF de ~13 mm en Z

| Campo | Detalle |
|---|---|
| **Síntoma** | TCP llega a Z incorrecto (~13 mm alto) tras GRASP_DOWN |
| **Causa raíz** | Divergencia entre parámetros DH del solver IK (del paquete `ur_description`) y la geometría real del SDF de Gazebo |
| **Cómo se detectó** | Medición sistemática z_error en GRASP_ALIGN_IK: residual consistente de 13 mm en dirección positiva Z |
| **Fix aplicado** | Bias loop en `GRASP_ALIGN_IK`: ajusta target_z = target_z − z_error iterativamente hasta convergencia |
| **Archivo modificado** | `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` (fase GRASP_ALIGN_IK) |
| **Parámetros** | `ALIGN_Z_RESIDUAL_TOL_M=0.008` activa bias; `ALIGN_EXIT_Z_TOL_M=0.010` define convergencia |
| **Impacto** | Sistema funcional con residual ~6-7 mm post-bias (dentro del umbral de éxito de 10 mm) |

---

#### 8.2 Falso positivo en detección de grasp (min_lift_delta)

| Campo | Detalle |
|---|---|
| **Síntoma** | Sistema reportaba GRASP exitoso pero el objeto no estaba sujeto; objeto caía al iniciar LIFT |
| **Causa raíz** | `min_lift_delta = 0.025 m` demasiado bajo: el TCP subía 25 mm pero el objeto (en reposo sobre la mesa) no se levantaba; Gazebo no había creado el detachable joint correctamente |
| **Cómo se detectó** | Observación visual en Gazebo: robot subía pero objeto quedaba en mesa |
| **Fix aplicado** | `min_lift_delta` aumentado de 0.025 a 0.060 m. También se añadió función `_fresh_gazebo_object_world()` para verificar pose fresca del objeto |
| **Archivo modificado** | `panel_pick_demo.py` |
| **Fecha** | 2026-04-07 |
| **Impacto** | Eliminado el falso positivo. El sistema ahora verifica que el objeto realmente se eleva 60 mm antes de continuar |

---

#### 8.3 Doble offset Z en gripper (GRIPPER_TCP_Z_OFFSET bug)

| Campo | Detalle |
|---|---|
| **Síntoma** | TCP bajaba 50 mm de más, llegando casi a la mesa |
| **Causa raíz** | `GRIPPER_TCP_Z_OFFSET = 0.05 m` se aplicaba **sobre** `rg2_pinch_center`, que ya tiene el offset de 0.175 m respecto a `tool0`. Double counting del offset. |
| **Cómo se detectó** | Inspección de código: el offset se sumaba tanto en el URDF como en el cálculo del target |
| **Fix aplicado** | `_DIRECTO_GRASP_Z = 0.0` (reset del offset manual), eliminando la variable `GRIPPER_TCP_Z_OFFSET` de los cálculos del panel |
| **Archivo modificado** | `panel_pick_demo.py` |
| **Commit** | `9b58910` (2026-04-08) |
| **Impacto** | Target Z correcto. El gripper llega exactamente a `Z_objeto + GRASP_CONTACT_Z_OFFSET_M` |

---

#### 8.4 NEGATE_XY permanente en solver IK

| Campo | Detalle |
|---|---|
| **Síntoma** | Si se deshabilitaba la negación, robot iba a posición especular incorrecta |
| **Causa raíz** | `base_link_inertia` tiene rotación Rz(π) respecto a `base_link`. El solver DH de `ur5_kinematics.py` usa `base_link_inertia` como raíz cinemática. Para convertir un target de `base_link` al frame del modelo IK, hay que negar X e Y. |
| **Cómo se detectó** | Diagnóstico espacial: poses FK y targets divergían en X e Y de forma especular |
| **Fix aplicado** | La negación es **permanente y correcta**. Se eliminó la variable de entorno `PANEL_PICK_DEMO_DIRECT_IK_NEGATE_XY` que podía desactivarla accidentalmente. |
| **Archivo modificado** | `panel_pick_demo.py`, `ur5_stack.launch.py` (nota eliminada en líneas 584-588) |
| **Fecha** | 2026-04-16 |
| **Impacto** | IK correcto siempre. No existe forma de desactivar la negación (letra muerta eliminada) |

---

#### 8.5 Divergencia FK/TF-live durante settle (APPROACH_COARSE_NOT_READY)

| Campo | Detalle |
|---|---|
| **Síntoma** | APPROACH_COARSE falla con "APPROACH_COARSE_NOT_READY": overshoot de 18 mm en error IK y divergencia FK/TF-live de 88-364 mm durante el settle |
| **Causa raíz** | H1: `ik_err_tol=0.035` demasiado estricto para el solver. H2: FK del panel (~220 ms de edad) rechazado por `POSE_SOURCE_AGE_TOL_SEC=0.200` s — el panel usaba datos obsoletos del FK |
| **Cómo se detectó** | Diagnóstico 2026-04-15: medición de z_error y TF age durante ciclo real |
| **Fix aplicado** | (1) `ik_err_tol` relajado. (2) `PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC` aumentado de 0.200 a 0.400 s. (3) 7 variables de entorno adicionales tuneadas (settle, tolerancias). |
| **Archivo modificado** | `ur5_stack.launch.py` |
| **Fecha** | 2026-04-15 |
| **Impacto** | Primer ciclo completo exitoso a las 19:14-19:15 de 2026-04-15 |

---

#### 8.6 Variables de entorno no inyectadas correctamente en launch

| Campo | Detalle |
|---|---|
| **Síntoma** | Variables de entorno ignoradas en runtime; sistema usa valores por defecto aunque se hayan exportado |
| **Causa raíz** | En `ur5_stack.launch.py`, algunas variables se leían con `os.environ.get(VAR, DEFAULT)` dentro del scope de declaración del nodo, pero la variable había sido exportada **después** de sourcer el setup.bash |
| **Fix** | Verificar que el export se hace ANTES de ejecutar el launch. Usar `printenv | grep PANEL_PICK` para confirmar que están en el entorno |
| **Diagnóstico** | Añadir `print(os.environ.get(VAR))` en el inicio del launch para verificar |

---

#### 8.7 IK cambia de rama en GRASP_DOWN con steps > 17 mm

| Campo | Detalle |
|---|---|
| **Síntoma** | Error de posición de ~48 mm en eje Y después de un step de GRASP_DOWN |
| **Causa raíz** | El solver IK tiene múltiples soluciones ("ramas"). Sin seed fuerte, puede saltar a una rama diferente en cada step, causando un cambio articular brusco que no lleva al punto deseado |
| **Fix** | `GRASP_DOWN_SEGMENT_Z_STEP_M = 0.005` (5 mm). `GRASP_DOWN_IK_SEED_WEIGHT = 0.65` (65% hacia seed = rama anterior). Con 7 segmentos de 5 mm para bajar 35 mm total, se mantiene la misma rama. |
| **Archivo** | `ur5_stack.launch.py`, `panel_pick_demo.py` |

---

#### 8.8 CLOSE en estado PEND (gripper no confirma cierre)

| Campo | Detalle |
|---|---|
| **Síntoma** | La fase CLOSE queda en estado PEND, no avanza a ATTACH_GATE |
| **Posibles causas** | (1) Gripper ya estaba cerrado (delta_sum ≈ 0). (2) `gripper_controller` inactivo. (3) Timeout muy corto. (4) Objeto demasiado grande para el gripper. |
| **Diagnóstico** | Ver sección 9.1 |
| **Fix típico** | Verificar estado inicial del gripper (debería estar abierto antes de CLOSE). Verificar `ros2 control list_controllers`. |

---

## 11. Guía de Troubleshooting

### 11.1 Síntomas actuales específicos del carry y attach

| Síntoma | Diagnóstico operativo | Referencia |
|---|---|---|
| `carry_validation` fallido | Revisar `FINAL_TRACE phase=CARRY`, thresholds reales `_validate_demo_carry`, y si el fallo es `object_not_updated`, `carry_follow_lost` o `tcp_dist_above_max`. | Código actual + helper/stack logs |
| `object_not_updated` / `best_obj_move=0` | Confirmar `/gripper/pick_demo/state`, `demo_transport_set_pose_ok` y si el objeto sigue sobre la mesa. Si nunca abandona la mesa, el attach fue sólo lógico. | [2026-04-18T20:23:30] [PICK][DIRECT][FINAL_TRACE] phase=CARRY event=wait_done expected=carry_validation_ok received=false timeout=1.60 request=none logical=CARRIED physical=none reason=demo_carry_validation_failed phase=post_grasp_lift best_obj_move=0.000 best_lift_delta=0.000 best_tcp_dist=0.098 fail_reasons=obj_move_below_min,lift_delta_below_min,tcp_dist_above_max last_obj_world=(-0.420,0.000,0.875) last_obj_base=(0.430,0.000,0.025) last_tcp_base=(0.445,0.002,0.144) |
| `carry_follow_lost` / `best_lift_delta < 0` | Comparar pose objeto vs TCP durante LIFT/CARRY y buscar `stale_tcp_pose_soft_follow` y `world_locked` retrasado. | [2026-04-19T16:52:50] [PICK][DIRECT][FINAL_TRACE] phase=CARRY event=wait_done expected=carry_validation_ok received=false timeout=1.60 request=none logical=CARRIED physical=none reason=demo_carry_validation_failed phase=post_grasp_lift best_obj_move=0.870 best_lift_delta=-0.194 best_tcp_dist=0.881 fail_reasons=lift_delta_below_min,tcp_dist_above_max carry_detail=carry_follow_lost last_obj_world=(0.396,0.222,0.681) last_obj_base=(1.246,0.222,-0.169) last_tcp_base=(0.440,-0.001,0.125) |
| `best_tcp_dist > máximo` | El objeto se mueve, pero no acompaña al TCP. Verificar offset ancla, follow mode, frescura de pose y thresholds reales de carry. | [gripper_attach_backend-11] [INFO] [1776753421.406565612] [gripper_attach_backend]: [ATTACH_BACKEND] demo_transport_follow_tick object=pick_demo mode=world_locked desired=(-0.414,-0.000,0.857) tcp=(-0.417,-0.000,0.872) tcp_src=base_chain |
| `stale_tcp_pose_soft_follow` | Inspeccionar `/world/ur5_mesa_objetos/pose/info`, TF freshness y valores `ATTACH_BACKEND_MAX_POSE_AGE_SEC` launch=1.5 / wrapper=2.5. | [gripper_attach_backend-11] [WARN] [1776753424.953371587] [gripper_attach_backend]: [ATTACH_BACKEND] stale_tcp_pose_soft_follow age=1.567s max=1.500s hard=4.500s src=world_tcp base_chain_ok=True base_chain_age=1.552 world_tcp_ok=True world_tcp_age=1.567 world_base_age=1.567 base_tcp_age=1.552 stable_world_base_ok=True stable_world_base_age=1.567 stable_world_base_mode=cached_fixed_base cache_ok=None cache_age=None tool_fallback_ok=True tool_fallback_age=1.567 world_base_ok=True base_tcp_ok=True |
| `CESTA_STAGE_*_postcheck_failed` | Tomar `runtime_target_dist` y `model_target_err` como señal primaria. Si ambos superan 0.040, tratarlo como incidente de corredor IK/cesta y no como bug del TCP. | CESTA_STAGE_1_RECOVER_1_postcheck_failed, runtime_target_dist=0.055/0.040, model_target_err=0.055/0.040 |
| CLOSE en PEND | Verificar gripper_controller activo, joint_states del gripper y delta de cierre. No confundir cierre medido con attach/carry confirmado. | Bug legacy aún relevante |
| UI muestra poses incoherentes | Separar FK base_link_inertia vs TF-live base_link y revisar world->base_link actual con X=-0.85 además de Z. | world_tf_publisher + panel traces |

### Detalle histórico recuperado (2026-04-18)

#### 9.1 CLOSE se queda en PEND

**Síntomas:** La fase CLOSE no avanza, log muestra `delta_sum=0.000` o `confirmed=False`.

**Diagnóstico paso a paso:**

```bash
# 1. Verificar estado actual del gripper
ros2 topic echo /joint_states --once | grep rg2
# Si joints ya están en ~1.18 → gripper ya estaba cerrado antes del comando
# Fix: abrir gripper manualmente antes del ciclo

# 2. Verificar controlador activo
ros2 control list_controllers | grep gripper
# Debe mostrar: gripper_controller [active]

# 3. Verificar que el topic recibe comandos
ros2 topic hz /gripper_controller/commands
# Si no hay mensajes → el panel no está publicando comandos

# 4. Enviar comando manual y observar
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [1.18, 1.18]" --once
ros2 topic echo /joint_states --once | grep rg2
# Si no cambia → problema en el controlador
```

**Causas más comunes y fixes:**

| Causa | Fix |
|---|---|
| Gripper ya en posición cerrada | Reiniciar ciclo con gripper en posición abierta (0.0 rad) |
| `gripper_controller` inactivo | `ros2 control set_controller_state gripper_controller active` |
| Timeout muy corto | Aumentar `PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC` |
| `CLOSE_MIN_DELTA_SUM` demasiado alto | Reducir a 0.005 m si objeto es pequeño |
| Objeto demasiado grande (bloquea gripper) | Verificar tamaño objeto vs apertura máxima |

---

#### 9.2 La pinza no se mueve (gripper inactivo)

```bash
# Verificar estado completo del sistema ros2_control
ros2 control list_controllers
ros2 control list_hardware_interfaces

# Verificar que Gazebo está corriendo
ros2 topic hz /clock
# Si no hay mensajes → Gazebo no corre

# Verificar bridge Gazebo ↔ ROS 2
ros2 topic list | grep gz
ros2 topic echo /gz/clock --once

# Reiniciar controlador
ros2 control set_controller_state gripper_controller inactive
ros2 control set_controller_state gripper_controller active
```

---

#### 9.3 La UI muestra poses incoherentes

**Síntomas:** La posición del TCP en la UI no coincide con la vista de Gazebo.

```bash
# Verificar TF tree
ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
# Si transformación es errónea → problema en robot_state_publisher o URDF

# Verificar joint_states
ros2 topic echo /joint_states --once
# Si joints no coinciden con posición visual → retraso o desconexión

# Verificar age del TF
ros2 topic hz /tf
# < 10 Hz puede indicar problema

# Reiniciar RSP
# (reiniciar el launch completo si TF está corrupto)
```

**Causa frecuente:** La pose que muestra la UI proviene del FK calculado con `joint_states` de `/tf`. Si `robot_state_publisher` está lento o hay lag en `use_sim_time`, el FK puede ser stale. Verificar `PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC` (400 ms máximo).

---

#### 9.4 El objeto no se adjunta (ATTACH_GATE falla)

**Síntomas:** ATTACH_GATE emite `[ATTACH_GATE][FAIL]`, el robot sube pero el objeto queda en la mesa.

```bash
# Verificar estado del detachable joint
ros2 topic echo /gripper_anchor/pick_demo/state

# Verificar distancia TCP-objeto
ros2 run tf2_ros tf2_echo base_link pick_demo
ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
# Calcular distancia manual

# Verificar que el gripper está cerrado
ros2 topic echo /joint_states --once | grep rg2
# Suma joints debe ser > 0.01 m (cerrado)

# Verificar attach backend
ros2 topic echo /gripper_anchor/pick_demo/attach
# Si no hay mensajes → backend no envía attach
```

**Diagnóstico por condición fallida:**

| Condición | Cómo verificar | Fix |
|---|---|---|
| dist_TCP_obj > 40 mm | tf2_echo base_link pick_demo vs rg2_pinch_center | Revisar GRASP_ALIGN_IK, z_contact offset |
| Gripper no cerrado | joint_states rg2 joints | Ver 9.1 |
| Backend no responde | topic list gripper_anchor | Verificar nodo gripper_attach_backend activo |
| ATTACH_BACKEND_MAX_DIST_M muy pequeño | — | Aumentar a 0.08 m temporalmente para diagnóstico |

---

#### 9.5 El grasp es débil / objeto cae al inicio del LIFT

**Síntomas:** ATTACH_GATE pasa, pero al ejecutar LIFT el objeto cae.

**Causas probables:**

1. **min_lift_delta muy pequeño:** El sistema "confirma" el lift antes de que el objeto realmente se eleve. Verificar que `min_lift_delta = 0.060` (no 0.025).

2. **Detachable joint no creado:** Backend envió attach pero Gazebo no procesó a tiempo. Verificar `/gripper_anchor/pick_demo/state`.

3. **TCP no en contacto real:** El objeto está dentro del umbral de 40 mm pero no entre los fingers. Revisar XY error en GRASP_ALIGN_IK.

4. **Fricción insuficiente en SDF:** En entorno de alta velocidad, la fricción μ=2.15 puede ser insuficiente. Reducir velocidad LIFT.

```bash
# Verificar que el lift realmente ocurre
ros2 run tf2_ros tf2_echo base_link pick_demo
# Monitorear Z del objeto durante LIFT
# Si Z no sube → objeto no adjunto (bug detachable joint)
# Si Z sube y luego cae → friction issue
```

---

#### 9.6 El TCP parece correcto pero visualmente no cuadra

**Causa más probable:** Confusión entre `tool0` y `rg2_pinch_center`.

- `tool0` está 175 mm **por debajo** de `rg2_pinch_center`
- Si la UI muestra `tool0`, la posición real del contacto es 175 mm más alta

```bash
# Comparar los dos frames
ros2 run tf2_ros tf2_echo base_link tool0
ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
# La diferencia Z debe ser exactamente 0.175 m
```

**Si la diferencia no es 0.175 m:** Problema en el URDF. Verificar `ur5.urdf.xacro` linea del joint `rg2_pinch_center_joint`.

---

#### 9.7 Problemas de frames TF

**Diagnóstico general:**

```bash
# Ver árbol completo
ros2 run tf2_tools view_frames
evince frames.pdf

# Verificar que world → base_link está publicado
ros2 topic echo /tf_static | grep base_link

# Verificar latencia de transforms
ros2 run tf2_ros tf2_monitor world base_link

# Si falta un frame → verificar que el nodo publicador está activo
ros2 node list | grep -E "rsp|tf_pub|gz_pose"
```

**Errores de lookup_transform comunes:**

| Error | Causa | Fix |
|---|---|---|
| `ExtrapolationException: target_time` | TF demasiado antiguo | Reducir frecuencia de peticiones o aumentar cache time |
| `LookupException: frame does not exist` | Nodo RSP caído o URDF sin publicar | Reiniciar robot_state_publisher |
| `ConnectivityException: no connection` | Frame no conectado al árbol | Verificar que world_tf_publisher está activo |

---

#### 9.8 Problemas de variables de entorno

**Verificar que las variables están correctamente inyectadas:**

```bash
# Antes de lanzar
printenv | grep -E "PANEL_|GRASP_|ATTACH_"

# En el launch, añadir temporalmente:
# import os; print(os.environ.get('GRASP_CONTACT_Z_OFFSET_M', 'NOT_SET'))
```

**Problema frecuente:** Exportar la variable en una terminal y lanzar en otra. Las variables de entorno no son globales — deben estar en la misma sesión de shell que el launch.

```bash
# Correcto:
export GRASP_CONTACT_Z_OFFSET_M=0.12
ros2 launch ur5_bringup ur5_stack.launch.py

# Incorrecto (no funciona entre terminales separadas):
# Terminal 1: export GRASP_CONTACT_Z_OFFSET_M=0.12
# Terminal 2: ros2 launch ur5_bringup ur5_stack.launch.py  ← no ve la variable
```

---

#### 9.9 Problemas de controladores

```bash
# Estado completo
ros2 control list_controllers -v

# Si un controlador está inactive/error:
ros2 control set_controller_state NOMBRE active

# Si el controller_manager no responde:
# → Gazebo probablemente no está corriendo o gz_ros2_control no arrancó
ros2 node list | grep controller_manager

# Verificar que gz_ros2_control_guard funciona:
ros2 topic echo /gz_ros2_control/status  # si existe
```

---

## 12. Estado Actual del Sistema

### 12.1 Cronología reciente confirmada

- `2026-04-22`: cierre operativo documentado en `reports/incidents/2026-04-21_tcp_geometry_incident.md`; valor canónico `+0.0050885` y aceptación manual final con `./lanzar_panelc2.sh`.
- `2026-04-23`: nota técnica de workspace en `agarre_ros2_ws/reports/incidents/2026-04-22_rg2_tcp_incident_closure.md` ratifica el cierre geométrico y deja abierto el follow-up de cesta / IK (`abierto`), detallado también en `agarre_ros2_ws/reports/incidents/2026-04-23_directo_basket_transport_ik_followup.md`.

### 12.2 Estado operativo sintetizado

- Geometría semántica actual confirmada: `tool0 -> rg2_tcp = tool0 -> rg2_pinch_center = 0 0 0.0050885`.
- Geometría visual SDF actual confirmada: `ur5_hand_joint relative_to=wrist_3_link, pose=0 0.0823 0 1.570796325 0 1.570796325`.
- `pick_demo_anchor` runtime actual: 0 0 0.0050885 sobre `tool0`, sin offsets legacy intermedios.
- Backend actual: modo launch `follow_tcp`, pero `pick_demo` en `pick_demo` usa `world_locked` en demo transport.
- Validación carry post-grasp real: timeout=3.0s, min_obj_move=0.020, min_lift_delta=0.025, max_tcp_dist=0.120.

## 13. Riesgos Abiertos

- Riesgo de leer thresholds de CARRY desde metadata de fase y no desde la llamada efectiva.
- Riesgo de asumir que ATTACH_GATE correcto implica carry físico correcto.
- Riesgo de degradación por pose TCP stale cuando el backend entra en soft follow con referencia vieja.
- Riesgo operativo abierto desde el follow-up `2026-04-23`: el corredor de cesta puede aceptar una IK numéricamente válida pero fallar el post-check runtime/modelo.
- Riesgo de reabrir falsamente el TCP para corregir un problema que hoy ya está separado como incidente de transporte / IK.
- Riesgo documental si se simplifica `world -> base_link` a sólo Z y se pierde el desplazamiento actual en X.

## 14. Próximos Pasos

- Añadir o revalidar el guard pre-ejecución de modelo en `CESTA_STAGE_*` y rechazar `solved_q` si `fk_ur5(solved_q)` no cae dentro de tolerancia frente a `target_ik`.
- Probar reseeding / branching explícito para el corredor de cesta antes de aceptar una solución IK como válida.
- Repetir batches `RUNS=5` y `RUNS=10` del flujo DIRECTO cuando el follow-up `2026-04-23` quede mitigado.
- Revalidar estadísticamente el carry con múltiples corridas y registrar distribución de `best_obj_move`, `best_lift_delta` y `best_tcp_dist`.
- Mantener una tabla de cambios de defaults entre launch, wrapper runtime y documento histórico para no perder trazabilidad en futuras revisiones.
- Repetir la validación manual final con `./lanzar_panelc2.sh` sólo después de cerrar el incidente residual de cesta / IK.

### Próximos pasos recuperados (2026-04-18)

1. **Validación estadística:** Ejecutar N > 10 ciclos consecutivos y medir tasa de éxito, varianza de errores.

2. **Reducir residual DH/SDF:** Investigar calibración exacta de parámetros DH en `ur_description`. Si el SDF usa valores ligeramente diferentes al DH estándar UR5, ajustar los parámetros del solver IK.

3. **Multi-objeto:** Validar el ciclo con 3-5 objetos diferentes en posiciones variadas de la mesa.

4. **Reducir timeouts:** Con el sistema estabilizado, los timeouts conservadores (CLOSE=3s, EXECUTE=150s) pueden reducirse para ciclos más rápidos.

5. **Documentar SRDF MoveIt:** La configuración MoveIt (`src/ur5_moveit_config/config/`) tiene grupos de movimiento y posiciones preset no completamente documentados en este análisis.

---

## 15. Apéndices

### 15.1 Evidencia cruzada preservada del documento vigente

### Bloque vigente preservado (evidencia cruzada)

- No disponible en las fuentes cargadas.

### 15.2 Índice de fuentes usadas por el generador

- `reports/BaseDeConocimiento/.tmp_base_conocimiento_2026-04-23/fuentes_verificadas.txt`
- `reports/BaseDeConocimiento/.tmp_base_conocimiento_2026-04-23`

### 15.3 Confirmación de merge

- Se preserva íntegramente la base vigente 2026-04-20 como snapshot estable en el apéndice 15.4.
- Se reincorpora el detalle histórico faltante mediante bloques recuperados desde la base 2026-04-18 en Markdown.
- Las discrepancias entre valores actuales e históricos se anotan como `valor actual confirmado`, `histórico/documentado previamente` o `riesgo / discrepancia abierta`.

### 15.4 Snapshot íntegro de la base vigente 2026-04-20

- No se pudo cargar una semilla vigente versionada para preservación íntegra.

### 15.5 Inventario exhaustivo de variables actuales

#### geometría y altura de agarre

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M | - | - | - | - | alineado | sin discrepancia relevante |
| PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_XYZ | - | - | - | - | alineado | sin discrepancia relevante |

#### APPROACH_COARSE

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M | 0.035 | - | 0.10 | 0.035 | discrepancia abierta | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_APPROACH_COARSE_GATE_XY_TOL_M | 0.012 | - | - | 0.012 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M | 0.012 | - | 0.008 | 0.012 | discrepancia abierta | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_APPROACH_COARSE_KEEP_XY_TOL_M | - | - | 0.06 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_APPROACH_COARSE_MAX_SKIP_M | - | - | 0.06 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_APPROACH_COARSE_SKIP_XY_TOL_M | - | - | 0.03 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_APPROACH_COARSE_SKIP_Z_TOL_M | - | - | 0.04 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_APPROACH_COARSE_Z_CORR_TOL_M | - | - | 0.020 | - | fallback local panel | sin discrepancia relevante |

#### GRASP_DOWN

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_EXTRA_GRASP_DOWN_M | - | - | 0.0 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_ELBOW_DELTA_RAD | - | - | 0.85 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_MAX_DELTA_RAD | - | - | 0.95 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_SHOULDER_LIFT_DELTA_RAD | - | - | 0.80 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_SUM_DELTA_RAD | - | - | 1.80 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_WRIST1_DELTA_RAD | - | - | 0.85 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK | 1 | 0 | - | - | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_GRASP_DOWN_FORCE_INHERIT_XY | 1 | - | - | - | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL | 0.200 | 0.200 | 0.080 | - | discrepancia abierta | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT | 0.035 | 0.035 | 0.65 | 0.65 | discrepancia abierta | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M | 0.005 | 0.015 | - | 0.003 | discrepancia abierta | actual difiere entre fuentes; histórico=0.003; wrapper pisa launch |
| PANEL_PICK_DEMO_GRASP_DOWN_MAX_ATTEMPTS | 4 | - | 4 | 4 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_SEED_WEIGHT | - | 0.035 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_PHASE_CRITICAL_SUM_DELTA_RAD | - | - | 2.85 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_PHASE_ELBOW_DELTA_RAD | - | - | 1.15 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_PHASE_MAX_DELTA_RAD | - | - | 2.35 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_PHASE_SHOULDER_LIFT_DELTA_RAD | - | - | 1.20 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_PHASE_SUM_DELTA_RAD | - | - | 6.20 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_PHASE_WRIST1_DELTA_RAD | - | - | 1.10 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_ROT_WEIGHT | - | - | 0.10 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_XY_STEP_M | - | - | 0.020 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M | 0.005 | - | 0.005 | 0.005 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M | 0.012 | 0.025 | 0.025 | 0.012 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M | 0.015 | - | 0.012 | 0.008 | discrepancia abierta | actual difiere entre fuentes; histórico=0.008; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M | 0.008 | 0.025 | 0.025 | 0.008 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_TCP_TOL_M | - | - | 0.020 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN | 1 | - | 1 | 1 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_DOWN_UTIL_XY_TOL_M | 0.015 | - | 0.012 | - | discrepancia abierta | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M | - | 0.025 | 0.025 | - | override runtime activo | sin discrepancia relevante |

#### GRASP_ALIGN_IK

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M | 0.020 | 0.020 | - | 0.010 | discrepancia abierta | histórico=0.010 |
| PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M | 0.010 | - | - | 0.010 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL | 0.200 | 0.200 | - | 0.08 | discrepancia abierta | histórico=0.08 |
| PANEL_PICK_DEMO_ALIGN_IK_SEED_WEIGHT | - | 0.035 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_PICK_DEMO_ALIGN_NO_EFFECT_TOL_M | - | - | 0.002 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M | 0.008 | - | - | 0.008 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_DEBUG_PAUSE_GRASP_ALIGN_IK | - | - | 0 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_ALIGN_TCP_TOL_M | - | - | 0.015 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES | 2 | - | - | 2 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_SKIP_ALIGN_IF_REACHABLE | - | - | 1 | 0 | discrepancia abierta | histórico=0 |

#### PRE_CLOSE

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M | 0.020 | 0.016 | - | 0.010 | discrepancia abierta | actual difiere entre fuentes; histórico=0.010; wrapper pisa launch |
| PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M | 0.010 | - | - | 0.010 | default launch | sin discrepancia relevante |

#### CLOSE

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC | 3.0 | 120 | - | 3.0 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM | 0.01 | - | - | 0.01 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_CLOSE_XY_TOL_M | 0.008 | - | 0.012 | 0.008 | discrepancia abierta | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M | 0.008 | - | 0.012 | 0.008 | discrepancia abierta | actual difiere entre fuentes; panel usa fallback distinto |
| PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M | 0.020 | - | 0.020 | - | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_POST_CLOSE_MODE | - | - | basket | - | fallback local panel | sin discrepancia relevante |

#### ATTACH_GATE

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M | 0.040 | 0.080 | 0.040 | 0.040 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M | 0.012 | 0.050 | 0.012 | 0.012 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M | 0.020 | - | 0.020 | 0.020 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES | 5 | 3 | 5 | 5 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_ATTACH_SETTLE_SEC | - | 4.0 | 1.8 | - | discrepancia abierta | actual difiere entre fuentes |
| PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC | 0.35 | 0.25 | 0.35 | 0.35 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_ATTACH_XY_TOL_M | 0.020 | 0.020 | 0.012 | 0.008 | discrepancia abierta | actual difiere entre fuentes; histórico=0.008; panel usa fallback distinto |
| PANEL_PICK_DEMO_ATTACH_Z_TOL_M | 0.010 | - | 0.015 | 0.010 | discrepancia abierta | actual difiere entre fuentes; panel usa fallback distinto |

#### LIFT / CARRY

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M | - | 0.500 | 0.200 | - | discrepancia abierta | actual difiere entre fuentes |
| PANEL_PICK_DEMO_CARRY_SETTLE_SEC | - | - | 3.0 | - | fallback local panel | sin discrepancia relevante |

#### freshness / pose source

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_MANUAL_REF_STALE_XY_TOL_M | - | - | 0.08 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_MANUAL_REF_STALE_Z_BELOW_TOL_M | - | - | 0.005 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC | 0.400 | 300.0 | - | 0.400 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| PANEL_PICK_DEMO_POSE_SOURCE_SYNC_TOL_SEC | - | 300.0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_PICK_DEMO_POSE_SOURCE_TOL_M | 0.006 | 1.0 | - | 0.006 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |

#### settle IK directo

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_PICK_DEMO_DIRECT_IK_ERR_TOL | - | - | - | - | alineado | sin discrepancia relevante |
| PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M | 0.003 | - | - | 0.003 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_POLL_SEC | 0.10 | - | - | 0.10 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SAMPLES | 3 | - | - | 3 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC | 2.5 | - | - | 2.5 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M | - | - | 0.040 | - | fallback local panel | sin discrepancia relevante |

#### MoveIt bridge

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_MOVEIT_BRIDGE_ACCEL_SCALE | - | 0.35 | - | 0.30 | discrepancia abierta | histórico=0.30 |
| PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_FK_ERR_M | - | 0.012 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_RAD | - | 0.90 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_SUM_RAD | - | 2.75 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN | - | 1 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_APPROACH_PATH_CONSTRAINT_TOL_RAD | - | 0.35 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_APPROACH_SKIP_CONSTRAINTS | - | 0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_CONTROLLER_EXPECTED_GOAL_TIME_SEC | - | 300.0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TIME_TOL_SEC | - | 300.0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD | - | 0.20 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD | - | 4.0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC | - | 25.0 | - | 150.0 | discrepancia abierta | histórico=150.0 |
| PANEL_MOVEIT_BRIDGE_FORCE_FJT_DIRECT | - | 0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_STABLE_SEC | - | 0.45 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TIMEOUT_SEC | - | 3.0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TOL_RAD | - | 0.015 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC | - | 2.5 | - | 2.5 | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC | - | 6.0 | - | 6.0 | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_PATH_CONSTRAINT_TOL_RAD | - | 0.35 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_PREGRASP_EE_TARGET_TOL_M | - | 0.05 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC | - | 80.0 | - | 180.0 | discrepancia abierta | histórico=180.0 |
| PANEL_MOVEIT_BRIDGE_START_BLEND_ERR_RAD | - | 0.80 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_START_BLEND_RATIO | - | 0.55 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_START_LEAD_GAIN | - | 0.60 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_START_LEAD_MAX_SEC | - | 4.0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_UNWRAP_CONTINUOUS_JOINTS | - | 1 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE | - | 0.35 | - | 0.30 | discrepancia abierta | histórico=0.30 |

#### backend attach/transporte

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS | - | pick_demo | - | - | override runtime activo | sin discrepancia relevante |
| ATTACH_BACKEND_FOLLOW_BREAK_DIST_M | 0.18 | 0.45 | - | - | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| ATTACH_BACKEND_FOLLOW_RATE_HZ | 20.0 | 10.0 | - | - | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| ATTACH_BACKEND_GZ_CMD_TIMEOUT_SEC | 3.0 | - | - | - | default launch | sin discrepancia relevante |
| ATTACH_BACKEND_GZ_SERVICE_TIMEOUT_MS | 2000 | - | - | - | default launch | sin discrepancia relevante |
| ATTACH_BACKEND_MAX_DIST_M | 0.08 | 0.06 | - | 0.06 | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| ATTACH_BACKEND_MAX_POSE_AGE_SEC | 1.5 | 2.5 | - | - | discrepancia abierta | actual difiere entre fuentes; wrapper pisa launch |
| ATTACH_BACKEND_MODE | follow_tcp | follow_tcp | - | - | override runtime activo | sin discrepancia relevante |

#### otros

| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |
|---|---|---|---|---|---|---|
| PANEL_CONTROLLER_READY_TIMEOUT_SEC | - | 60.0 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_MOVEIT_STARTUP_TIMEOUT_SEC | - | 300 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_PICK_DEMO_AC_PHASE_CHECK_SETTLE_SEC | - | - | 3.0 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_AC_PHASE_CHECK_STABLE_SAMPLES | - | - | 3 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_AC_PHASE_CHECK_THRESHOLD_M | - | - | 0.004 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_FALLBACK_PRESET_MAX_DIST_M | - | - | 0.10 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_FALLBACK_PRESET_MAX_XY_M | - | - | 0.05 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRASP_TCP_Z_OFFSET_M | - | - | 0.0 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M | 0.12 | - | - | 0.12 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_IK_SEED_JOINTS | - | 0.0,-1.5708,0.0,-1.5708,0.0,-3.07 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_PICK_DEMO_MAX_PROMOTED_STABLE_AGE_SEC | - | 900 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_PICK_DEMO_MOVE_SEC | - | 15 | - | - | override runtime activo | sin discrepancia relevante |
| PANEL_PICK_DEMO_PHASE_JUMP_TOL_M | 0.010 | - | - | 0.010 | default launch | sin discrepancia relevante |
| PANEL_PICK_DEMO_ROUTE_MODE | - | - | direct_ik_hybrid | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_SHORT_RELEASE_ONLY | - | - | 0 | - | fallback local panel | sin discrepancia relevante |
| PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC | - | 60 | 0 | - | discrepancia abierta | actual difiere entre fuentes |
