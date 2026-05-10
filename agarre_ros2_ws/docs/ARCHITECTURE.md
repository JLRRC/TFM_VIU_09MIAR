# Arquitectura del TFM UR5+RG2 / ROS 2 Jazzy + Gazebo Harmonic

**Fecha:** 2026-05-10 (auditoría profesional + F0-F10).
**Branch:** `refactor/audit-fases-f0-f5-20260510`.

Este documento describe la arquitectura **objetivo y vigente** del workspace tras las fases F0-F10 del plan de refactorización profesional. Es el documento canónico para defender el TFM y para incorporar a un nuevo desarrollador.

---

## 1. Visión general

El sistema implementa un pipeline completo de **pick & place simulado** sobre un brazo UR5 con gripper OnRobot RG2, integrado con ROS 2 Jazzy, Gazebo Harmonic y MoveIt 2. La arquitectura sigue principios de microservicio:

- **Separación clara de responsabilidades** entre paquetes ROS 2.
- **LifecycleNodes** en todos los nodos críticos.
- **Comunicación por action / service / topic** según el tipo de interacción.
- **Configuración declarativa** (YAML) separada de la lógica.
- **Tests offline puros** (>2.000) que no requieren ROS vivo.

```
┌───────────────────────────────────────────────────────────────────────┐
│                         CAPA UI (Qt)                                   │
│        ur5_qt_panel — vista + input usuario                            │
│   (en migración: panel_ui_node + panel_backend_node, F6 audit)         │
└──────────────────────────────┬────────────────────────────────────────┘
                               │ /panel_backend/{pick_place,cancel,health}
                               │ /panel_launch_control/{start_stack,…}
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│                     CAPA BACKEND (rclpy puro)                          │
│   panel_backend_node          panel_launch_control_node                │
│   panel_v2 RosWorker (legacy, en deprecación)                          │
└──────────────────────────────┬────────────────────────────────────────┘
                               │ /pick_place (PickPlace.action)
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│                       CAPA ORQUESTACIÓN                                │
│   pick_orchestrator_lifecycle (LifecycleNode)                          │
│   ├─ pick_fsm.py            (FSM puro, testeable sin ROS)              │
│   ├─ phase_dispatch.py      (handlers por fase + tabla declarativa)    │
│   ├─ pick_gates.py          (gates puros: attach_distance, etc)        │
│   ├─ retry.py               (back-off + retry_with_backoff)            │
│   └─ initial_snapshot.py    (TF + JointState capture)                  │
└──────────────────────────────┬────────────────────────────────────────┘
                               │
        ┌──────────────────────┼──────────────────────┐
        ▼                      ▼                      ▼
┌────────────────┐    ┌────────────────────┐  ┌──────────────────────┐
│ MOTION         │    │ GRIPPER + ATTACH   │  │ PERCEPCIÓN           │
│ plan_to_pose_  │    │ gripper_attach_    │  │ tfm_grasping/        │
│   server       │    │   backend (LC)     │  │   grasp_inference    │
│ (3 modos)      │    │ release_objects_   │  │   grasp_selector     │
│                │    │   service (LC)     │  │                      │
└────────┬───────┘    └─────────┬──────────┘  └──────────────────────┘
         │                      │
         ▼                      ▼
┌────────────────────────────────────────────────────────────────────┐
│                   CAPA INFRAESTRUCTURA                              │
│   system_state_manager (LC)         tf_geometry_service             │
│   evidence_logger (LC)              object_pose_resolver_service    │
│   world_tf_publisher                planning_scene_sync (LC)        │
│   gz_pose_bridge                    gz_ros_control_guard            │
│   controller_bootstrap              controller_health_monitor (LC)  │
│   simulation_reset_service (LC)                                     │
└──────────────────────────┬─────────────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────────────┐
│       SIMULACIÓN — Gazebo Harmonic (sin Gazebo Classic)             │
│   ros_gz_sim · ros_gz_bridge · gz_ros2_control                      │
│   ur5_gazebo (worlds/ models/ config/bridges/)                      │
│   ur5_description (URDF/Xacro + geometry.yaml)                      │
│   ur5_moveit_config (SRDF + moveit_controllers + ompl_planning)     │
└────────────────────────────────────────────────────────────────────┘

LC = LifecycleNode managed (configure / activate / deactivate / cleanup).
```

---

## 2. Paquetes ROS 2

| Paquete | Tipo | Responsabilidad |
|---|---|---|
| **ur5_description** | ament_cmake | URDF/Xacro UR5+RG2, `geometry.yaml` (single source of truth de offsets), controllers YAML base. |
| **ur5_gazebo** *(F5)* | ament_cmake | Assets de simulación: `worlds/`, `models/ur5_rg2/`, `config/ros_gz_bridge.yaml`, `config/bridges/*.yaml`. |
| **ur5_moveit_config** | ament_cmake | SRDF, `kinematics.yaml`, `moveit_controllers.yaml`, `ompl_planning.yaml`, launches MoveIt. |
| **ur5_bringup** | ament_cmake | Launches del stack, factories Python, configuración runtime, schemas. |
| **ur5_panel_interfaces** | ament_cmake | Definiciones IDL: `Attach.srv`, `Open.srv`, `PickPlace.action`, `PlanToPose.action`, etc. |
| **ur5_tools** | ament_python | 21 entry_points: orquestación, gripper backend, plan_to_pose server, system state, evidence logger, etc. |
| **tfm_orchestrator** | ament_python | LifecycleNode `pick_orchestrator_lifecycle` + FSM puro + dispatch + gates + retry. |
| **tfm_grasping** | ament_python | Inferencia ML de grasps (`grasp_inference`) + selector top-1 (`grasp_selector_node`). |
| **ur5_qt_panel** | ament_python | Panel Qt (UI). En migración a `panel_ui_node` + `panel_backend_node` (F6). |

---

## 3. Comunicación inter-nodo

**Reglas aplicadas:**

| Tipo | Cuándo | Ejemplos |
|---|---|---|
| **action** | Operaciones largas, cancelables, con feedback | `/pick_place` (PickPlace), `/orchestrator/plan_to_pose`, `/joint_trajectory_controller/follow_joint_trajectory` |
| **service** | Petición síncrona puntual | `/gripper/{open,close}`, `/orchestrator/{attach,detach}`, `/orchestrator/resolve_object_pose_world`, `/panel_launch_control/start_stack`, `/controller_health_monitor/health` |
| **topic** | Streams continuos / estado | `/tf`, `/joint_states`, `/clock`, `/system_state`, `/controller_health` (latched), `/panel_backend/feedback` (latched) |
| **parameter** | Configuración estática | `use_sim_time`, `bridge_*`, `attach_backend_*`, `home_*` |
| **TF** | Transformaciones espaciales | `world` ↔ `base_link` ↔ `tool0` ↔ `rg2_tcp` |

---

## 4. LifecycleNodes (managed)

Lista completa de nodos managed:

1. `pick_orchestrator_lifecycle` (tfm_orchestrator)
2. `gripper_attach_backend` (ur5_tools)
3. `release_objects_service` (ur5_tools)
4. `system_state_manager` (ur5_tools)
5. `object_pose_resolver_service` (ur5_tools)
6. `planning_scene_sync` (ur5_tools)
7. `evidence_logger` (ur5_tools)
8. `panel_launch_control_node` *(F6 nuevo)*
9. `panel_backend_node` *(F6 nuevo)*
10. `controller_health_monitor_node` *(F9 nuevo)*
11. `simulation_reset_service` *(F9 nuevo)*

**Pendiente migración:** `controller_bootstrap` (F9.1, NOTE block en código).

---

## 5. Decisiones arquitectónicas clave

### 5.1 Gripper RG2 fuera de MoveIt (P-04)

El RG2 simplificado del workspace es prismático con limit `[0, 0.0425] m`. **No** está bajo `moveit_controllers.yaml` por diseño: se controla vía servicios `/gripper/open|close` que expone `gripper_attach_backend`. SRDF declara las colisiones del gripper para que MoveIt valide trayectorias del brazo sin chocar con los dedos.

Validado por test `test_moveit_controllers_design.py` (5 tests).

### 5.2 Single source of truth de geometría (P-03, F2)

Offsets críticos viven en `ur5_description/config/geometry.yaml` (declarativo) y `ur5_tools/geometry_constants.py` (mirror Python). El test `test_geometry_yaml_consistency.py` garantiza coherencia entre YAML, Python y URDF.

### 5.3 Assets Gazebo en paquete ROS (P-02, F5)

`worlds/` y `models/` viven en `ur5_gazebo/` (paquete `ament_cmake`). Pre-F5 estaban fuera de cualquier paquete, lo que rompía `colcon install`. Resolución de paths centralizada en `ur5_tools/workspace_paths.py`:

- `resolve_ur5_gazebo_share()` → `ament_index` → fallback source tree.
- `resolve_world_file(name)` → SDF empaquetado.

### 5.4 Gazebo Harmonic puro (test_no_classic_gazebo)

100% plugins SDF son `gz-sim-*` o `gz_ros2_control`. Cero referencias a `gazebo_ros`, `libgazebo_*`, `gazebo_msgs`, `gazebo_plugins`, `gazebo_ros2_control`. Validado como regresión por `test_no_classic_gazebo.py`.

### 5.5 FSM puro (testabilidad)

El FSM del pick (`tfm_orchestrator/pick_fsm.py`) es Python puro sin ROS. Todas las transiciones son testeables offline. La capa ROS solo cablea servicios/actions externos y publica run_id latched.

---

## 6. Trazabilidad y evidencia

**`run_id` latched:** cada ciclo de pick genera un ID único de 8 hex (`format_log_line` en `tfm_orchestrator/run_id.py`) publicado como `/pick/run_id` (latched). Todos los logs incluyen el ID en el formato:

```
[run_id] [phase] [orch] [status] msg=... extra_fields=...
```

**Evidence logger:** subscrito a `/pick/run_id`, escribe JSONL por ciclo con timings, gates, attach distance.

**KPIs (F7):** `validate_pick_3_cycles.sh` exporta `PICK_VALIDATE_KPI_FILE` con JSON aggregable; el workflow `e2e-live-on-demand.yml` lo sube como artifact 30 días.

---

## 7. CI/CD

| Workflow | Trigger | Coste | Cobertura |
|---|---|---|---|
| `colcon.yml` | push/PR cualquier rama | <1 min | Tests offline puros (~2.050) — sin ROS, sin Gazebo. |
| `colcon-full.yml` *(F4 actualizado)* | main / refactor / PR / manual | ~10 min | Build + test colcon completo en `ros:jazzy-desktop` con cobertura HTML/XML como artifact. |
| `ci.yml` | main / refactor / PR | <2 min | Lint (flake8 + pep257) + pytest-offline + policy-checks (no Gazebo Classic). |
| `e2e-live-on-demand.yml` *(F7 actualizado)* | manual / label `run-t35-on-demand` | 5-20 min | T35 × N ciclos en `ros:jazzy-desktop` + Xvfb + Gazebo headless. KPI JSON como artifact. |

---

## 8. Documentos relacionados

- `docs/PICK_SEQUENCE.md` *(F10.2)*: secuencia detallada del pick con gates y timeouts.
- `docs/FRAMES.md` *(F10.3)*: tree TF + offsets canónicos.
- `docs/HARDWARE_CHECKLIST.md` *(F10.4)*: pre-flight para validación con UR5 real (hardware CB-series).
- `auditoria/audit_profesional_20260510.md`: auditoría profesional completa (20 secciones, 6 tablas).

---

## 9. Estado actual de la deuda técnica

- ✅ **F0-F9 ejecutados** (commits `audit-baseline-20260510` → `cb0e769`).
- ✅ Tests offline: 994+ verde.
- ⚠️ **Pendiente F6.3 final:** migrar `panel_v2` + `RosWorker` a wrappers triviales sobre `panel_backend_node` (requiere validación con Qt loop vivo).
- ⚠️ **Pendiente F9.1 final:** migrar `controller_bootstrap` a LifecycleNode (requiere validación con `controller_manager` vivo).
- ⚠️ **Pendiente F8 final:** sustituir el OpaqueFunction `_prepare_runtime` del launch por llamada a `scripts/prepare_runtime.py` (riesgo medio si el script falla, requiere fallback graceful).

---

## 10. Glosario

- **Pick FSM:** State machine `IDLE → INITIAL_SNAPSHOT → HOME_INITIAL → SELECT_OBJECT → APPROACH → GRASP_DOWN → GRASP → LIFT → TRANSPORT → RELEASE → DONE` con terminales `FAILED` / `ABORTED`.
- **TCP operacional:** `rg2_tcp` (canónico SRDF). Alias: `rg2_pinch_center` (geometría idéntica).
- **drop_xyz_world:** punto de soltado en frame `world`; default `(-1.30, 0, 0.82)` (geometry.yaml).
- **ee_frame:** end-effector frame, default `rg2_tcp`.
- **planning frame:** `base_link` (MoveIt).
