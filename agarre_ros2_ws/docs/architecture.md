# Arquitectura del sistema TFM — UR5 + RG2

> Generado el 2026-04-27 tras el refactor estructural de la rama `ENTREGA.V3`.
> Snapshot vivo del estado de los paquetes ROS 2 y los nodos del proyecto.

## 1. Visión general

Sistema simulado de **pick & place** con un brazo UR5 + gripper OnRobot RG2
sobre **ROS 2 Jazzy**, **Gazebo Sim moderno** (Harmonic) y **MoveIt 2**.
La interfaz operativa es un **panel Qt5** con pipeline de demo guiado.

### 1.1 Stack tecnológico

| Capa | Tecnología |
|---|---|
| Middleware | ROS 2 Jazzy (rclpy) |
| Simulador | Gazebo Sim moderno (`gz sim`, `ros_gz_bridge`, `gz_ros2_control`) |
| Planificador | MoveIt 2 (`moveit_py` + `moveit_commander` + `move_group`) |
| Control | `ros2_control` con `joint_trajectory_controller` |
| Visualización | RViz 2, panel Qt5 propio |
| Lenguaje | Python 3.12 + minimal C++ |

### 1.2 Frames TF canónicos

```
world ──▶ base_link ──▶ shoulder_link ──▶ ... ──▶ tool0 ──▶ rg2_tcp ──▶ rg2_pinch_center
```

- `base_link` es la base estática del UR5 en el mundo.
- `tool0` es el flange estándar UR5.
- `rg2_tcp` es el TCP semántico del gripper (offset Z = 0.18 m desde tool0 — TCP "limpio").
- `rg2_pinch_center` es el punto de contacto de las pinzas; coincide con `rg2_tcp` tras la migración limpia (commit `503aa4a`).

Todas las llamadas MoveIt usan `tip_link = rg2_tcp` (consistente entre URDF, SRDF y kinematics.yaml).

## 2. Paquetes ROS 2

```
agarre_ros2_ws/src/
├── ur5_bringup            # ament_cmake — launch unificado + config YAML
├── ur5_description        # ament_cmake — URDF/Xacro + meshes + SDF
├── ur5_moveit_config      # ament_cmake — SRDF + kinematics + planning pipelines
├── ur5_qt_panel           # ament_python — panel Qt5 + lógica del pick demo
├── ur5_tools              # ament_python — bridges + nodos auxiliares (12 entry points)
├── tfm_grasping           # ament_python — modelo de inferencia de grasps (TFM)
└── ur5_panel_interfaces   # ament_cmake (rosidl) — .srv compartidos del panel
```

### 2.1 Responsabilidades

| Paquete | Función | Líneas Python |
|---|---|---|
| `ur5_bringup` | Launch (`ur5_stack.launch.py`) + `runtime_defaults.yaml` (81 tunables) | 870 |
| `ur5_description` | URDF/Xacro UR5+RG2 + SDF runtime + meshes | (cmake/xml) |
| `ur5_moveit_config` | SRDF, kinematics.yaml, OMPL, joint_limits.yaml, controllers | (yaml/srdf) |
| `ur5_qt_panel` | Panel UI + pick demo + state machine + camera + grasp inference UI | ~38.000 |
| `ur5_tools` | `ur5_moveit_bridge`, `gripper_attach_backend`, `system_state_manager`, `world_tf_publisher`, `release_objects_service`, `gz_pose_bridge`, `controller_bootstrap`, `planning_scene_sync`, +más | ~9.000 |
| `tfm_grasping` | Inferencia de pose de agarre desde imagen RGB | ~1.000 |
| `ur5_panel_interfaces` | `SelectObject.srv` | (.srv) |

## 3. Nodos ROS 2 principales

```
┌─────────────────────────────┐    ┌────────────────────────────┐
│  panel_v2 (ur5_qt_panel)    │    │ system_state_manager       │
│  - UI Qt5                   │◀──▶│  - /system_state           │
│  - Pick demo state machine  │    │  - /system_diag            │
│  - Camera + grasp UI        │    └────────────────────────────┘
└──────────┬──────────────────┘
           │ topics + services
           ▼
┌─────────────────────────────┐    ┌────────────────────────────┐
│ ur5_moveit_bridge           │◀──▶│ MoveItPy + move_group      │
│  - 8 mixins:                │    │  - planning_pipelines      │
│    GoalValidation           │    │  - OMPL                    │
│    ControllerManagement     │    │  - kinematics (KDL)        │
│    JointStateHelpers        │    └────────────────────────────┘
│    Geometry (FK/IK)         │
│    TrajectoryPrep           │
│    MoveItCommander          │
│    MoveItPyPlanner          │
│    Executor (FJT)           │
└──────────┬──────────────────┘
           │ FollowJointTrajectory action
           ▼
┌─────────────────────────────┐
│ joint_trajectory_controller │ (ros2_control + gz_ros2_control)
│  - 6 UR5 joints             │
│  - 2 RG2 prismatic          │
└─────────────────────────────┘

┌─────────────────────────────┐    ┌────────────────────────────┐
│ gripper_attach_backend      │    │ gz_pose_bridge             │
│  - 7 mixins:                │    │  - /world/.../pose/info    │
│    AnchorMixin              │    └────────────────────────────┘
│    DemoTransportMixin       │
│    GzCliMixin               │    ┌────────────────────────────┐
│    PoseLookupMixin          │    │ release_objects_service    │
│    PoseSubscriberMixin      │    │  - /panel/release          │
│    SetPoseMixin             │    └────────────────────────────┘
│    + attach_math (helpers)  │
└─────────────────────────────┘    ┌────────────────────────────┐
                                   │ world_tf_publisher         │
                                   │  - TF estática del mundo   │
                                   └────────────────────────────┘
```

## 4. Refactor estructural (sesión 2026-04-27)

### 4.1 Métricas globales

29 commits + 1 tag (`pre-refactor-2026-04-27`) en una sesión.
**603 tests** verdes tras cada commit (red de seguridad).

| Archivo | Inicio | Final | Δ | Mecanismo |
|---|---|---|---|---|
| `ur5_moveit_bridge.py` | 5.654 L | **1.706 L** | **−69.8%** | 8 mixins en `moveit_bridge/` |
| `gripper_attach_backend.py` | 2.152 L | **950 L** | **−56%** | 7 mixins en módulos `attach_*.py` |
| `panel_utils.py` | 2.328 L | **1.010 L** | **−57%** | 6 sub-paquetes |
| `panel_pick_demo.py` | 12.249 L | **11.144 L** | −9% | 1 sub-paquete |
| `panel_v2.py` | 312 wrappers rotos | 320 fixes (audit AST) | red restaurada | F1 |

**Total**: ~9.000 líneas reorganizadas en **22 módulos nuevos**.

### 4.2 Mixins extraídos (`ur5_moveit_bridge`)

```
UR5MoveItBridge
  ├── MoveItPyPlannerMixin    — _plan_with_moveit_py + _init_moveit_py
  ├── MoveItCommanderMixin    — moveit_commander + cartesian path
  ├── GeometryMixin           — pose_to_matrix + IK seeded para APPROACH
  ├── TrajectoryPrepMixin     — preparación JT para FJT controller
  ├── ExecutorMixin           — FollowJointTrajectory action loop (~1.350 L)
  ├── JointStateHelpersMixin  — cache + start state + 4 constantes joints
  ├── ControllerManagementMixin — FJT client + ListControllers query
  ├── GoalValidationMixin     — joint/EE goal reached + planned consistency
  └── Node (rclpy)
```

### 4.3 Mixins extraídos (`gripper_attach_backend`)

```
GripperAttachBackend
  ├── AnchorMixin            — drop/tool anchor relay + startup detach
  ├── DemoTransportMixin     — SDFs dinámicos + spawn/delete kinematic
  ├── GzCliMixin             — gz service exists/resolve/delete/spawn
  ├── PoseLookupMixin        — TF lookups + composition + fallback chain
  ├── PoseSubscriberMixin    — /pose/info + /joint_states + FK manual
  ├── SetPoseMixin           — SetEntityPose service + gz CLI fallback
  └── Node (rclpy)
+ attach_math.py (helpers puros: quaternion, matriz, DH transform)
```

### 4.4 Sub-paquetes extraídos (`panel_utils`)

| Módulo | Responsabilidad |
|---|---|
| `panel_pixel_geometry` | Mapeo pixel ↔ world ↔ table (homography) |
| `panel_controllers_query` | gripper_controller_defined, resolve_controller_manager, list_*_controllers |
| `panel_system_status` | gz_sim_status, bridge_status, clock_status |
| `panel_tf_discovery` | discover_robot_base_frame, discover_world_frame, debug_dump_tf |
| `panel_table_objects` | nearest_table_object, get_object_pose_gz, load_home_pose |
| `panel_pose_helpers` | _select_base_frame, _select_ee_frame, get_pose, effective_*_frame |

## 5. Configuración centralizada

### 5.1 `runtime_defaults.yaml`

81 tunables centralizados en `ur5_bringup/config/runtime_defaults.yaml`. Dominios:

- Sistema y paths (WS_DIR, GZ_RENDER_ENGINE, PANEL_PYTHON).
- Panel (CAMERA_REQUIRED, KEEP_CAMERAS).
- system_state_manager (3 tolerancias geométricas).
- attach_backend (7 timeouts y modo follow).
- moveit_bridge (16 timeouts + scales + flags).
- pick_demo (50 tolerancias por fase: GRASP_DOWN, ATTACH gate, gripper close, PRE_CLOSE, CLOSE, GRASP_ALIGN, etc.).

**Prioridad de resolución**: env var → YAML → literal default.

## 6. Convenciones de logging

Cada fase del pick demo emite logs con prefijo etiquetado:

- `[PICK][MOVEIT][PLAN_RESULT]` — éxito/fallo de planificación.
- `[PICK][MOVEIT][PLAN_FK]` — verificación de endpoint via FK.
- `[PICK][MOVEIT][CONSTRAINTS]` — constraints aplicados.
- `[PICK][DIRECT][PANEL_TRACE]` — trace del flujo direct.
- `[PICK][REMOTE][TRACE]` — trace del flujo remote.
- `[BRIDGE_EXEC]` / `[BRIDGE_PLAN]` / `[BRIDGE_CART]` — bridge MoveIt.
- `[BRIDGE_STATUS]` — eventos discretos del bridge (plan_ok, exec_fail, etc.).
- `[ATTACH_BACKEND]` — backend de attach físico.
- `[PHYSICS][POSE_INFO]` — estado del bridge gz↔ros.
- `[PHYSICS][DROP]` — release/detach de objetos.
- `[APPROACH_IK_SEED]` — diagnóstico de seed selection en IK.

## 7. Pipeline del pick demo

```
INITIAL_SNAPSHOT → HOME_INITIAL → MESA → APPROACH_COARSE
  → GRASP_DOWN → GRASP_ALIGN → PRE_CLOSE → CLOSE
  → ATTACH (validation gate)
  → LIFT → CARRY → BASKET_TRANSPORT → BASKET_DROP
  → RELEASE → HOME_FINAL
```

Cada fase tiene su tolerancia configurable en YAML y produce un log
estructurado en `[PICK][MOVEIT][...]` o `[PICK][DIRECT][...]`.

## 8. Testing

### 8.1 Suite actual

**603 tests automatizados** distribuidos:

| Paquete | Tests | Categoría |
|---|---|---|
| `ur5_qt_panel` | 400 | unitarios + smoke |
| `ur5_tools` | 196 | unitarios |
| `ur5_bringup` | resto | launch helpers |
| `tfm_grasping` | resto | model load + scaling |

Lint: `flake8` + `pep257` con config local en cada paquete (`ament_flake8.ini`).

### 8.2 Smoke E2E

```bash
bash agarre_ros2_ws/scripts/validate_pick_3_cycles.sh 3
```

Lanza el stack 3 veces, ejecuta un ciclo de pick por iteración, valida sentinel `READY` final.

## 9. Tag de rollback

```bash
git checkout pre-refactor-2026-04-27
```

Restaura el estado funcional pre-refactor para comparar regresiones.

## 10. Archivos pendientes (futuro)

| Archivo | Estado |
|---|---|
| `panel_pick_demo.run_pick_demo` | ~11.000 L con closures anidadas — sesión dedicada |
| `panel_v2.__init__` | 700 L sin estructura modular — 2–3 sesiones |
| `ur5_stack.launch.py` | 868 L — split en 5 launches modulares |
| `panel_pick_object.py` | 4.868 L — fusionar con orquestador en F5 |
| `panel_step_callbacks.py` | 2.227 L — closures interdependientes |
| `panel_ros.py` / `panel_tfm.py` | ~2.000 L cada uno |
