# Arquitectura del sistema TFM — UR5 + RG2

> Última actualización: 2026-05-01 (post F0–F10, F11 decisiones canónicas).
> Snapshot vivo del estado de los paquetes ROS 2 y los nodos del proyecto.

## 1. Visión general

Sistema simulado de **pick & place** con un brazo UR5 + gripper OnRobot RG2
sobre **ROS 2 Jazzy**, **Gazebo Sim moderno** (Harmonic) y **MoveIt 2**.
La interfaz operativa es un **panel Qt5** con pipeline de demo guiado y un
**orchestrator ROS 2** (`tfm_orchestrator`) que aloja `PickPlace.action`.

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
- `rg2_tcp` es el TCP semántico del gripper.
- `rg2_pinch_center` es el punto de contacto de las pinzas; coincide con `rg2_tcp` tras la migración limpia.

Todas las llamadas MoveIt usan `tip_link = rg2_tcp` (consistente entre URDF, SRDF y kinematics.yaml).

## 2. Paquetes ROS 2 (8 paquetes)

```
agarre_ros2_ws/src/
├── ur5_bringup            # ament_cmake — launch unificado + config YAML
├── ur5_description        # ament_cmake — URDF/Xacro + meshes + SDF
├── ur5_moveit_config      # ament_cmake — SRDF + kinematics + planning pipelines
├── ur5_qt_panel           # ament_python — panel Qt5 + lógica del pick demo
├── ur5_tools              # ament_python — bridges + nodos auxiliares (14 entry points)
├── tfm_grasping           # ament_python — modelo de inferencia de grasps (TFM)
├── tfm_orchestrator       # ament_python — PickPlace.action + FSM puro + LifecycleNode
└── ur5_panel_interfaces   # ament_cmake (rosidl) — 8 .srv + 2 .action compartidos
```

### 2.1 Responsabilidades

| Paquete | Función | LOC Python |
|---|---|---|
| `ur5_bringup` | Launch (`ur5_stack.launch.py`) + `runtime_defaults.yaml` | ~870 |
| `ur5_description` | URDF/Xacro UR5+RG2 + SDF runtime + meshes | (cmake/xml) |
| `ur5_moveit_config` | SRDF, kinematics.yaml, OMPL, joint_limits.yaml, controllers | (yaml/srdf) |
| `ur5_qt_panel` | Panel UI + pick demo + state machine + camera + grasp inference UI | ~46.000 |
| `ur5_tools` | `ur5_moveit_bridge`, `gripper_attach_backend`, `system_state_manager`, `world_tf_publisher`, `release_objects_service`, `gz_pose_bridge`, `controller_bootstrap`, `planning_scene_sync`, `evidence_logger`, `plan_to_pose_server`, +más | ~9.000 |
| `tfm_grasping` | Inferencia de pose de agarre desde imagen RGB | ~1.000 |
| `tfm_orchestrator` | `pick_fsm` puro + `pick_orchestrator_lifecycle` (LifecycleNode) | ~1.700 |
| `ur5_panel_interfaces` | 8 .srv (Select/Open/Close/SetWidth/Attach/Detach/WorldToBase/ComputeApproachPose) + 2 .action (PickPlace, PlanToPose) | (.srv/.action) |

### 2.2 Métricas globales (post F0–F10)

| Métrica | Valor |
|---|---|
| LOC Python totales | 91 692 |
| Tests | 1077 funciones / 65 archivos |
| Commits sobre `main` | 338 |
| Action / Service interfaces | 2 / 8 |
| LifecycleNodes | 1 (`pick_orchestrator_lifecycle`, F9) |
| Dataclasses frozen para parámetros | 6 |

## 3. Decisiones canónicas (F11, 2026-05-01)

### 3.1 Entry-point del panel: **`panel_v2`** canónico

| Entry-point | Estado | Uso |
|---|---|---|
| `ros2 run ur5_qt_panel panel_v2` | ✅ **Canónico** | Implementación operativa completa |
| `ros2 run ur5_qt_panel main_panel` | Alias TFM | Wrapper de 38 LOC sobre `panel_v2` (reexporta `ControlPanelV2` y `main`) — alineado con la nomenclatura usada en la memoria del TFM |

`main_panel.py` es un wrapper trivial — se mantiene por nomenclatura académica, **no es una implementación alternativa**.

### 3.2 Orchestrator: **`pick_orchestrator_lifecycle`** canónico

| Entry-point | Estado | Uso |
|---|---|---|
| `ros2 run tfm_orchestrator pick_orchestrator_lifecycle` | ✅ **Canónico** (F9) | LifecycleNode con `configure/activate/deactivate/cleanup/shutdown`. Acepta goals solo en `ACTIVE`. |
| `ros2 run tfm_orchestrator pick_orchestrator` | ⚠️ Legacy F5/F6 | `Node` plano. Se mantiene para retrocompatibilidad y tests; emite `DeprecationWarning` al arrancar. Migrar clientes al lifecycle. |

Ambos comparten la misma lógica de FSM puro (`pick_fsm.py`) — solo cambia el ciclo de vida del nodo.

### 3.3 SRDF: **dos variantes intencionales**

| Archivo | Uso | Activación |
|---|---|---|
| `ur5.srdf` | ✅ **Default** — colisiones permisivas para planificar pick & place sin choques internos espurios | Por defecto (`strict_self_collision=false`) |
| `ur5_strict.srdf` | Estricto — solo desactiva colisiones adyacentes/fijas inevitables | `STRICT_SELF_COLLISION=1` o `strict_physics_mode:=true` |

**No es duplicación**: la lógica de selección está en `ur5_moveit_bringup.launch.py:41`. Si se modifica un SRDF debe aplicarse el cambio paralelo al otro.

### 3.4 Jerarquía de configuración: **env > YAML > defaults**

Ver [`docs/CONFIG_HIERARCHY.md`](CONFIG_HIERARCHY.md) para el detalle.

Resumen:
1. **Variable de entorno** (`os.environ`) — mayor precedencia, para overrides ad-hoc o debug.
2. **YAML** (`runtime_defaults.yaml`, `panel_*_runtime.yaml`) — fuente de verdad para despliegue.
3. **Default literal en código** — solo si las dos anteriores faltan.

Total env vars en panel: **133** (reducido desde ~600 en F2).

## 4. Arquitectura actual (post F0–F10)

```
┌────────────────────────────────────────────────────────────────────┐
│  ur5_qt_panel (panel_v2 entrypoint, 102 módulos, ~46 kLOC)         │
│  ─ UI Qt + nodo ROS embebido + FSM legacy + ROS pubs/subs          │
│  ─ panel_pick_demo (10.4k) + panel_pick_object (4.6k) + panel_v2…  │
└──────────┬──────────────────────────────────────────┬──────────────┘
           │ opt-in PickPlaceClient                   │ ROS topics directos
           │  (F6.4 dispatch_pick_demo)               │ (legacy run_pick_demo)
           ▼                                          ▼
┌────────────────────────┐    ┌──────────────────────────────────────┐
│ tfm_orchestrator       │───▶│ ur5_tools (microservicios ROS 2)     │
│  pick_fsm.py (puro)    │ srv│  ─ ur5_moveit_bridge (1.7k)          │
│  PickPlace.action      │    │  ─ system_state_manager (849)        │
│  Lifecycle node (F9)   │    │  ─ gripper_attach_backend (959)      │
│  PlanToPose.action     │    │  ─ release_objects_service (1.1k)    │
└────────────────────────┘    │  ─ world_tf_publisher                │
                              │  ─ planning_scene_sync, evidence…    │
                              │  ─ plan_to_pose_server (F6.5)        │
                              └──────────────────────────────────────┘
                                          │
                                          ▼
                              ┌──────────────────────────────────────┐
                              │ ur5_moveit_config + ur5_description  │
                              │ Gazebo Harmonic + gz_ros2_control    │
                              └──────────────────────────────────────┘
                              ┌──────────────────────────────────────┐
                              │ tfm_grasping (perception/inference)  │
                              └──────────────────────────────────────┘
```

## 5. Arquitectura objetivo (microservicios maduros)

```
                           ┌──────────────────────────────┐
                           │  panel_ui_node (Qt thin)     │ ← solo UI
                           │   - publica intents          │
                           │   - consume feedback action  │
                           └──────────────┬───────────────┘
                                          │ /pick_place (action)
                                          │ /plan_to_pose (action)
                                          ▼
                           ┌──────────────────────────────┐
                           │  pick_sequence_orchestrator  │ ← LifecycleNode ✓
                           │   pick_fsm puro              │
                           │   service_clients            │
                           └─┬───┬──┬──┬──┬──┬──┬─────────┘
                             │   │  │  │  │  │  │
       ┌─────────────────────┘   │  │  │  │  │  └─────────────────┐
       ▼                         ▼  ▼  ▼  ▼  ▼                    ▼
┌──────────────┐    ┌──────────────┐ ┌──────────────┐  ┌──────────────────┐
│ object_      │    │ tf_geometry_ │ │ motion_      │  │ gripper_         │
│ detection    │    │ service      │ │ planner_node │  │ controller_node  │
│ (Lifecycle)  │    │ (Lifecycle)  │ │ (Lifecycle)  │  │ (Lifecycle)      │
└──────────────┘    └──────────────┘ └──────────────┘  └──────────────────┘
       │                   │                │                  │
       ▼                   ▼                ▼                  ▼
┌──────────────┐    ┌──────────────┐ ┌──────────────┐  ┌──────────────────┐
│ grasp_       │    │ trajectory_  │ │ attach_      │  │ evidence_logger  │
│ inference    │    │ executor     │ │ backend      │  │ (CSV/JSON)       │
│ (Lifecycle)  │    │              │ │ (Lifecycle)  │  │                  │
└──────────────┘    └──────────────┘ └──────────────┘  └──────────────────┘

         ┌─── system_state_manager (Lifecycle, GLOBAL coordinator) ───┐
         │  arbitra qué nodos están activos según fase del pipeline   │
         └────────────────────────────────────────────────────────────┘
```

Reglas de comunicación:
- **Topics**: telemetría continua (poses, estado de objetos, joints).
- **Services**: queries síncronos puros (`WorldToBase`, `ComputeApproachPose`, `SelectObject`, `Attach/Detach`, `Open/Close`).
- **Actions**: tareas con feedback y cancel (`PickPlace`, `PlanToPose`, `ExecuteTrajectory`).
- **Parameters**: tunings del nodo concreto, declarados en `__init__`, no por env var.
- **Lifecycle**: todo nodo crítico al pipeline. Ver [`LIFECYCLE.md`](LIFECYCLE.md).

## 6. Interfaces (`ur5_panel_interfaces`)

### 6.1 Services (8)

| Service | Server | Cliente | Propósito |
|---|---|---|---|
| `SelectObject.srv` | system_state_manager | orchestrator | Marcar objeto target |
| `Open.srv` / `Close.srv` / `SetWidth.srv` | gripper_attach_backend | orchestrator | Comando gripper |
| `Attach.srv` / `Detach.srv` | gripper_attach_backend | orchestrator | Attach lógico/físico |
| `WorldToBase.srv` | (pendiente tf_geometry_service) | orchestrator | Conversión TF |
| `ComputeApproachPose.srv` | (pendiente tf_geometry_service) | orchestrator | Pose pre-grasp |

### 6.2 Actions (2)

| Action | Server | Cliente | Propósito |
|---|---|---|---|
| `PickPlace.action` | `tfm_orchestrator/pick_orchestrator_lifecycle` | panel | Pick & place E2E con feedback por fase |
| `PlanToPose.action` | `ur5_tools/plan_to_pose_server` | orchestrator | Planificar+ejecutar pose objetivo (F6.3-F6.5) |

## 7. Convenciones de logging

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

## 8. Pipeline del pick demo

```
INITIAL_SNAPSHOT → HOME_INITIAL → MESA → APPROACH_COARSE
  → GRASP_DOWN → GRASP_ALIGN → PRE_CLOSE → CLOSE
  → ATTACH (validation gate)
  → LIFT → CARRY → BASKET_TRANSPORT → BASKET_DROP
  → RELEASE → HOME_FINAL
```

Cada fase tiene su tolerancia configurable en YAML y produce un log
estructurado en `[PICK][MOVEIT][...]` o `[PICK][DIRECT][...]`.

El FSM puro vive en `tfm_orchestrator/pick_fsm.py` y es 100% testeable sin ROS.

## 9. Testing

### 9.1 Suite actual

**1077 tests automatizados** distribuidos:

| Paquete | Categoría |
|---|---|
| `ur5_qt_panel` | unitarios + smoke |
| `ur5_tools` | unitarios |
| `ur5_bringup` | launch helpers, urdf_frames, interfaces_contracts |
| `tfm_grasping` | model load + scaling |
| `tfm_orchestrator` | FSM puro + lifecycle |

CI partido en:
- **`offline-tests`** — sin colcon build, ejecutado en cada push, debe estar verde.
- **`colcon-full`** — build completo + integración, manual.

Lint: `flake8` + `pep257` con config local en cada paquete (`ament_flake8.ini`).

### 9.2 Smoke E2E

```bash
bash agarre_ros2_ws/scripts/validate_pick_3_cycles.sh 3
```

Lanza el stack 3 veces, ejecuta un ciclo de pick por iteración, valida sentinel `READY` final.

## 10. Tags de rollback

| Tag | Estado |
|---|---|
| `pre-refactor-2026-04-27` | Pre-refactor estructural inicial |
| `audit-pre-fase1-20260428` | Pre-F1 limpieza |
| `audit-pre-fase2-continue-20260430` | Pre-F2 dataclasses |
| `audit-pre-f8-20260501` | Pre-F8 instrumentación latencias |
| `audit-pre-f9-20260501` | Pre-F9 lifecycle node |
| `audit-pre-f3-extended-20260501` | Pre-F3 extended (módulos puros) |
| `audit-pre-f3-full-20260501` | Pre-F3 full attempt |
| **`audit-pre-f11-20260501`** | **Pre-F11 decisiones canónicas (este commit)** |

## 11. Deuda pendiente (Ruta B, F11–F19)

| Fase | Acción | Esfuerzo |
|---|---|---|
| **F11 ✓** | Decisiones canónicas + docs | 4-6 h |
| **F12** | Drenar `panel_pick_demo.py` (orchestrator default) | 12-20 h |
| **F13** | Lifecycle en moveit_bridge + state_manager + attach + tf_pub | 8-12 h |
| **F14** | Romper `panel_v2.py` en 4 ficheros <800 LOC | 6-10 h |
| **F15** | Romper `panel_pick_object.py` + `panel_ros.py` | 8-12 h |
| **F17** | Launch modular (`ur5_stack.launch.py`) | 4-6 h |
| **F16** | `tf_geometry_service` como microservicio dedicado | 10-15 h |
| **F18** | Telemetría/observabilidad extendida | 6-10 h |
| **F19** | Optimización rendimiento | 8-12 h |
