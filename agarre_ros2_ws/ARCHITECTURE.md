# Architecture — agarre_ros2_ws

Sistema de agarre pick & place para UR5+RG2 sobre ROS 2 Jazzy / Gazebo Harmonic / MoveIt 2.

---

## Visión general (C4 — nivel 1: Sistema)

```
┌─────────────────────────────────────────────────────────────────────┐
│  Operador                                                           │
│    │                                                                │
│    ▼                                                                │
│  Panel Qt (ur5_qt_panel)          ◄──────────────────────────────┐ │
│    │  publishes: /grasp_pose,      ROS 2 topics / actions / TF   │ │
│    │             /gripper_cmd                                     │ │
│    ▼                                                              │ │
│  Stack ROS 2                                                      │ │
│    ├── Gazebo Harmonic (simulación física)                        │ │
│    ├── robot_state_publisher  (URDF → TF)                        │ │
│    ├── ros2_control + gz_ros2_control  (controladores)           │ │
│    ├── MoveIt 2 / move_group   (planificación de movimiento)     │ │
│    ├── UR5MoveItBridge          (bridge pose→trayectoria)        │ │
│    └── servicios auxiliares    (attach, release, scene_sync)     │ │
│         │                                                         │ │
│         └── /joint_states, /tf, /move_action ──────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Paquetes ROS 2

| Paquete | Tipo | Responsabilidad |
|---------|------|-----------------|
| `ur5_qt_panel` | ament_python | Panel Qt, lógica de UI, pipeline DIRECTO y MoveIt, evaluadores de gate |
| `ur5_tools` | ament_python | Nodos auxiliares: bridge MoveIt, attach, release, pose bridge, gripper geometry |
| `ur5_bringup` | ament_cmake | Launch files, mundos, configuración de arranque |
| `ur5_description` | ament_cmake | URDF, XACRO, meshes, configs de controladores |
| `ur5_moveit_config` | ament_cmake | SRDF, kinematics.yaml, ompl_planning.yaml, bringup MoveIt |
| `ur5_panel_interfaces` | ament_cmake | Interfaces ROS 2 custom (msgs/srvs) del panel |
| `tfm_grasping` | ament_python | Modelo de inferencia de agarre (wrapper ROS 2) |

---

## Arquitectura de módulos — ur5_qt_panel

```
panel_v2.py  (entry point, 2859 líneas — orquestador de UI)
│
├── CAPA UI
│   ├── panel_main_ui.py          — construcción de widgets Qt
│   ├── panel_step_ui.py          — UI del pipeline por pasos
│   ├── panel_trace_ui.py         — traza/log de pasos en pantalla
│   ├── panel_settings.py         — diálogo de configuración
│   ├── panel_calib_selection.py  — selección de calibración
│   ├── panel_camera.py           — visualización de cámara
│   ├── panel_cameras_tab.py      — gestión de pestañas de cámara
│   └── panel_draw_overlays.py    — overlays gráficos sobre imagen
│
├── CAPA ROS
│   ├── panel_ros.py              — suscripciones y publishers ROS 2
│   ├── panel_ros_publishers.py   — publishers específicos
│   ├── panel_tf.py               — lookup TF y buffer
│   ├── panel_tf_monitor.py       — monitorización de frescura TF
│   ├── panel_tf_diagnose.py      — diagnóstico de árboles TF
│   └── tf_pose_utils.py          — conversiones pose↔matrix sin estado
│
├── CAPA PIPELINE DIRECTO (pick sin MoveIt)
│   ├── panel_pick_demo.py        — orquestador DIRECTO (ciclo por fases)
│   ├── directo_geometry.py       ★ módulo puro: 20 funciones geométricas
│   ├── directo_gate_evaluator.py ★ módulo casi-puro: 11 evaluadores de gate
│   ├── attach_gate_evaluator.py  — evaluador de ventana de estabilidad ATTACH
│   └── panel_runtime_pose_auditor.py — auditoría de poses en tiempo real
│
├── CAPA PIPELINE MOVEIT
│   ├── panel_pick_object.py      — orquestador MoveIt pick (STEP/MOVEIT)
│   ├── panel_moveit_flow.py      — flujo de planificación y ejecución
│   ├── panel_moveit_ready.py     — guard de disponibilidad MoveIt
│   ├── panel_moveit_wait.py      — espera de resultados async
│   └── panel_moveit_publishers.py — publicación de goals MoveIt
│
├── CAPA ESTADO Y CONFIGURACIÓN
│   ├── panel_config.py           — constantes globales y parámetros de entorno
│   ├── panel_env.py              — lectura de variables de entorno
│   ├── panel_state.py            — estado mutable del panel
│   ├── panel_state_machine.py    — máquina de estados del ciclo
│   ├── panel_state_methods.py    — métodos de transición de estado
│   └── panel_external_state.py  — estado derivado de ROS (objetos, poses)
│
├── CAPA CALIBRACIÓN
│   ├── panel_calibration.py      — lógica de calibración mesa/pixel↔world
│   ├── panel_calib_actions.py    — acciones de calibración (botones)
│   └── calibration_service.py   — servicio ROS 2 de calibración
│
├── CAPA WORKERS Y TIMING
│   ├── panel_workers.py          — QThread workers para operaciones bloqueantes
│   ├── panel_watchdog.py         — watchdog de timeout de pasos
│   └── panel_process.py         — gestión de subprocesos (Gazebo, etc.)
│
├── UTILIDADES PURAS
│   ├── ur5_kinematics.py         ★ IK/FK DH analítico — sin dependencias ROS
│   ├── panel_utils.py            — formatters y helpers de presentación
│   ├── logging_utils.py          — configuración de logging
│   └── step_pipeline_helpers.py  — helpers para pipeline de pasos
│
└── REFERENCIA (solo lectura)
    └── panel_direct2.py          — implementación DIRECTO2 (golden reference)
```

★ = módulo sin dependencias ROS/Qt, unit-testable directamente.

---

## Arquitectura de módulos — ur5_tools

```
ur5_moveit_bridge.py     — Nodo ROS 2 principal: bridge pose→MoveIt→ejecución (5800 líneas)
moveit_bridge_utils.py   ★ 16 helpers puros extraídos de UR5MoveItBridge
gripper_geometry.py      — Geometría del gripper RG2 (URDF/SDF parse + validación)
gripper_attach_backend.py — Backend de attach/detach cinemático en Gazebo
gz_pose_bridge.py        — Bridge de poses Gazebo→ROS 2 con estampas TF
release_objects_service  — Servicio de release (detach + drop) de objetos
planning_scene_sync.py   — Sincronización de la escena de planificación MoveIt
system_state_manager.py  — Gestor de estado del sistema (ready/error/idle)
controller_bootstrap.py  — Bootstrap automático de controladores ros2_control
world_tf_publisher.py    — Publicador del frame estático /world
tf_probe.py              — Diagnóstico de frescura y conectividad TF
param_utils.py           ★ Helpers de lectura de parámetros ROS 2
```

★ = módulo sin dependencias ROS a nivel de import, unit-testable directamente.

---

## Flujo de datos — pipeline DIRECTO

```
Cámara overhead
    │ imagen
    ▼
Panel Qt (panel_camera.py)
    │ click pixel / detección objeto
    ▼
panel_pick_demo.py  ←─── panel_config.py (tolerancias, env vars)
    │
    ├─ APPROACH_COARSE   → IK analítico (ur5_kinematics.py)
    │                    → /joint_trajectory_controller/follow_joint_trajectory
    │
    ├─ GRASP_DOWN        → segmented IK ó MoveIt Cartesian
    │                    → gate: directo_gate_evaluator._evaluate_transport_stage_*
    │
    ├─ GRASP_ALIGN_IK    → IK iterativo con bias Z (gripper_geometry.contact_z_correction)
    │
    ├─ PRE_CLOSE / CLOSE → /gripper_cmd → gripper_attach_backend.py
    │                    → gate: attach_gate_evaluator._evaluate_attach_window
    │
    ├─ ATTACH            → /gripper_attach/attach → Gazebo Joint
    │
    ├─ LIFT / CARRY      → IK analítico → transport seed candidates
    │                    → gate: directo_gate_evaluator._evaluate_transport_stage_postcheck
    │
    └─ RELEASE           → /release_objects → Gazebo detach + drop
```

---

## Flujo de datos — pipeline MoveIt (STEP/MOVEIT)

```
Panel Qt (panel_pick_object.py)
    │ PoseStamped → /ur5_moveit_bridge/grasp_pose
    ▼
ur5_moveit_bridge.py  (UR5MoveItBridge Node)
    │
    ├─ PLAN  → MoveItPy ó moveit_commander
    │        → plan_success_ok() ← moveit_bridge_utils.py
    │
    ├─ EXECUTE → FollowJointTrajectory action ó trajectory topic
    │          → scale_joint_trajectory_timing() ← moveit_bridge_utils.py
    │
    └─ RESULT → /ur5_moveit_bridge/result → panel_pick_object.py
              → goal_status_text() ← moveit_bridge_utils.py
```

---

## Topología ROS 2 (nodos principales en runtime)

```
/gz_sim                       — simulador Gazebo Harmonic
/robot_state_publisher        — URDF → /tf, /tf_static
/gz_ros2_control              — controladores (FJT)
/move_group                   — MoveIt 2 (opcional)
/ur5_moveit_bridge            — bridge pose → plan → execute
/gripper_attach_backend       — attach/detach Joint Gazebo
/gz_pose_bridge               — poses GZ → ROS 2
/world_tf_publisher           — frame /world estático
/release_objects_service      — release de objetos
/planning_scene_sync          — sync escena MoveIt ↔ Gazebo
/system_state_manager         — estado global del stack
/ur5_panel_v2                 — panel Qt (producer + consumer)
```

---

## Geometría del robot (TCP semántico)

| Frame | Descripción | Z desde tool0 |
|-------|-------------|---------------|
| `tool0` | flange del UR5 | 0.0 m |
| `rg2_pinch_center` | TCP semántico (contacto) | +0.0050885 m |
| `rg2_tcp` | TCP de planificación MoveIt | +0.175 m (legacy) |

La divergencia DH/SDF históricamente medida es ~13 mm (confirmada 2026-04-15).
Se gestiona en tiempo de ejecución mediante el bias loop de GRASP_ALIGN_IK.

---

## Testing

```
src/ur5_qt_panel/test/
    conftest.py                    — stub de ur5_tools sin colcon build
    test_directo_geometry.py       — 84 tests de directo_geometry.py
    test_directo_gate_evaluator.py — 45 tests de directo_gate_evaluator.py

src/ur5_tools/test/
    test_moveit_bridge_utils.py    — 50 tests de moveit_bridge_utils.py
    test_gripper_geometry.py       — tests de gripper_geometry.py
    test_system_state_machine.py   — tests del state manager

scripts/
    smoke_test.sh                  — CI rápido: AST parse, F401, unit tests (--fast)
    validate_before_demo.sh        — checklist pre-demo con ROS activo
```

Ejecutar sin ROS:
```bash
cd agarre_ros2_ws
bash scripts/smoke_test.sh --fast   # ~6 s, 444 tests
```

---

## Refactoring aplicado (ENTREGA.V3)

| FASE | Descripción | Impacto |
|------|-------------|---------|
| 0 | Auditoría completa arquitectura | +reports/auditoria_moveit_step_20260424.md |
| 1 | Eliminar 57 imports F401 en 19 ficheros | −57 imports no usados |
| 2 | Constante TABLE_TOP_Z, eliminar 0.850 hardcoded | +mantenibilidad |
| 3a | Extraer `directo_geometry.py` | 20 funciones puras, sin ROS/Qt |
| 3b | Extraer `directo_gate_evaluator.py` | 11 evaluadores de gate |
| 4 | 129 tests unitarios (geometry + gate evaluator) | cobertura sin ROS |
| 5a | Extraer `moveit_bridge_utils.py` | 16 helpers puros + 2 TF freshness helpers |
| 5b | Extraer `launch_helpers.py` | ur5_stack.launch.py −27% (1176→863 líneas) |
| 6 | `CycleLogger` (cycle_logger.py) | logger JSON thread-safe por ciclo pick |
| 7 | IK: CachedKDLKinematicsPlugin | kinematics.yaml — cache 5000 poses |
| 8 | Auditoría bridge_cameras.yaml | 78 bridges categorizados, todos justificados |
| 9 | `smoke_test.sh` + `validate_before_demo.sh` | CI: AST + F401 + unit tests sin ROS |
| 10 | `ARCHITECTURE.md` + README link | documentación de arquitectura C4 |
| 11 | TF freshness: `stamp_age_sec` / `is_stamp_fresh` | helpers para guards de frescura TF |
| 12 | Campaña 100% cobertura — 15 módulos puros | 593 tests, todos los módulos testables al 100% |

**Totales de la rama ENTREGA.V3 vs. main:**
- Ficheros nuevos: 11 módulos + 17 ficheros de test + 2 scripts bash
- Tests: 593 tests sin ROS en <6 s
- Imports F401: 0 violaciones
- `panel_v2.py`: 18 600 → 2 859 líneas (−85%) en commits previos a esta rama
- `directo_geometry.py`: 65% → 100% cobertura
- `directo_gate_evaluator.py`: 98% → 100% cobertura
- `attach_gate_evaluator.py`: 90% → 100% cobertura
- `ur5_kinematics.py`: 95% → 100% cobertura
- `panel_config.py`: 97% → 100% cobertura
- `panel_state.py`: 0% → 100% cobertura
- `panel_state_machine.py`: 90% → 100% cobertura
- `panel_external_state.py`: 69% → 100% cobertura
- `logging_utils.py`: 64% → 100% cobertura
- `step_pipeline_helpers.py`: 98% → 100% cobertura
- `cycle_logger.py`: 97% → 100% cobertura
- `moveit_bridge_utils.py`: 75% → 100% cobertura
- `gripper_geometry.py`: 74% → 100% cobertura
- `param_utils.py`: 90% → 100% cobertura
- `launch_helpers.py`: 93% → 100% cobertura
