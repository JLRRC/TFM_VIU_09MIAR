# Arquitectura post F5-legacy-removed (2026-05-08)

Diagrama de la arquitectura final del proyecto tras borrar el legacy
`run_pick_demo` y validar T35 × 3 cycles consecutivos verde con el
orchestrator como path canónico único.

## Diagrama de flujo del pick

```mermaid
flowchart TD
    User[Usuario / Panel UI Qt] -->|botón Pick Demo| Dispatcher[pick_demo_dispatcher]

    Dispatcher -->|action goal| ActionServer["/pick_place ActionServer"]
    ActionServer -.-> OrchLN[pick_orchestrator_lifecycle_node]

    OrchLN --> FSM{FSM 9 fases}
    FSM --> Phase1[INITIAL_SNAPSHOT]
    FSM --> Phase2[HOME_INITIAL]
    FSM --> Phase3[SELECT_OBJECT]
    FSM --> Phase4[APPROACH]
    FSM --> Phase5[GRASP_DOWN]
    FSM --> Phase6[GRASP]
    FSM --> Phase7[LIFT]
    FSM --> Phase8[TRANSPORT]
    FSM --> Phase9[RELEASE]

    Phase1 -.->|tf2 + JointState| TF[world_tf_publisher]
    Phase2 -.->|FJT directo| FJT["/joint_trajectory_controller<br/>follow_joint_trajectory"]
    Phase3 -.->|internal| OrchLN
    Phase4 -->|action /orchestrator/plan_to_pose| Plan2P[plan_to_pose_server]
    Phase5 -->|action /orchestrator/plan_to_pose| Plan2P
    Phase7 -->|action /orchestrator/plan_to_pose| Plan2P
    Phase8 -->|action /orchestrator/plan_to_pose| Plan2P

    Plan2P -->|"compute_ik (sync)"| MoveIt["MoveIt move_group"]
    Plan2P -->|"FJT directo bypass"| FJT
    MoveIt -.->|joint solution| Plan2P
    Plan2P -->|joint trajectory| FJT

    Phase6 -->|service /orchestrator/attach| Attach[gripper_attach_backend]
    Phase9 -->|service /orchestrator/detach| Attach
    Attach -->|gz SetEntityPose| Gazebo["Gazebo Sim Harmonic"]

    FJT --> Gazebo
    Gazebo -.->|joint_states + tf| TF
    Gazebo -.->|object poses| GzPose[gz_pose_bridge]
    GzPose -.-> ResolveObj[object_pose_resolver_service]
    ResolveObj -.-> OrchLN

    style OrchLN fill:#bbf,stroke:#333,stroke-width:3px
    style Plan2P fill:#bfb,stroke:#333,stroke-width:2px
    style FJT fill:#fbb,stroke:#333,stroke-width:2px
    style Gazebo fill:#ffd,stroke:#333,stroke-width:1px
```

## Componentes clave

### LifecycleNodes (15 producción)

| Nodo | Responsabilidad |
|------|----------------|
| `pick_orchestrator_lifecycle_node` | FSM 9 fases, action server `/pick_place` |
| `plan_to_pose_server` | Action server `/orchestrator/plan_to_pose`, modo MOVEIT_DIRECT con bypass FJT |
| `gripper_attach_backend` | Services Open/Close/Attach/Detach, demo_transport_follow |
| `release_objects_service` | Service `/release_objects` para drop |
| `system_state_manager` | Watchdog del estado global |
| `tf_geometry_service` | Conversiones world↔base_link, approach poses |
| `world_tf_publisher` | TF static + dynamic |
| `gz_pose_bridge` | Bridge Gazebo poses ↔ ROS topics |
| `object_pose_resolver_service` | Resolver pose de objeto en `world` |
| `ur5_moveit_bridge` | Bridge MoveIt ↔ controller (legacy, no usado por orchestrator) |
| `controller_bootstrap` | Carga inicial de controllers |
| `gz_ros_control_guard` | Watchdog gz_ros2_control |
| `planning_scene_sync` | Sync planning_scene MoveIt ↔ Gazebo |
| `panel_tf_helper` | TF helpers panel (no productivo en orchestrator path) |
| `lifecycle_helpers` | Utilidades lifecycle compartidas |

### Action servers

- `/pick_place` (ur5_panel_interfaces/PickPlace) — entrada única del pick
- `/orchestrator/plan_to_pose` (ur5_panel_interfaces/PlanToPose)
- `/joint_trajectory_controller/follow_joint_trajectory` — execution

### Services (9 IDL)

- `/gripper/open`, `/gripper/close`, `/gripper/set_width`
- `/orchestrator/attach`, `/orchestrator/detach`
- `/release_objects`
- `/panel/select_object`, `/panel/world_to_base`, `/panel/compute_approach_pose`
- `/object_pose_resolver/resolve_world` (MOVEIT_DIRECT support)

## Path canónico del cycle pick

1. **User clicks** "Agarre Objeto (Directo)" en panel Qt
2. `panel_calib_actions._run_pick_demo` → `pick_demo_dispatcher.dispatch_pick_demo`
3. `dispatch_pick_demo` envía goal al action `/pick_place`
4. `pick_orchestrator_lifecycle_node._execute_callback` recibe goal
5. FSM ejecuta 9 fases secuenciales:
   - Cada fase publica `Feedback` con `current_phase`, `progress`, `phase_index`
   - Fases que requieren movimiento llaman `/orchestrator/plan_to_pose`
6. `plan_to_pose_server._execute_moveit_direct` (refactor T15 2026-05-09):
   función orquestadora 118 LOC con 5 sub-helpers:
   - `_moveit_try_fjt_bypass`: F1.24/H9 bypass MoveIt vía FJT directo cuando
     `bypass_moveit_for_short_paths=true` (default).
     - `_fjt_extract_seed_positions` (joint_state cacheado)
     - `_fjt_compute_traj_params` (TF lookup ee_frame → distancia)
     - `_fjt_call_compute_ik` (`/compute_ik` con timeout 5s, normalización [-π, π])
     - `_fjt_build_trajectory` (multi-waypoint si dist > 0.4m, 2-point si ≤ 0.4m)
     - `_fjt_send_and_wait_result` (path_tolerance H14 por joint, wait result)
   - `_moveit_send_first_attempt`: lazy ActionClient + build_move_group_goal
     con phase_tuning per-distancia (TRANSPORT scaling=0.5/timeout=240s, resto
     scaling=0.25/timeout=120s).
   - `_moveit_post_timeout_tf_check` (F1.22 LIVE): tras FIRST_ATTEMPT_TIMEOUT,
     TF check del ee_frame; si robot llegó al target → success
     `feedback_hang_recovered_via_tf`.
   - `_moveit_retry_after_failure` (F1.23 LIVE): controller restart + 20s
     sleep + retry. Markers `retry_after_<orig>` / `retry_failed`.
   - `_moveit_final_tf_recovery` (F1.23 LIVE): último TF check tras todos
     los aborts; `feedback_hang_recovered_final` si robot llegó al target.
7. Result final del action `/pick_place` → cliente

## Solución del BUG_CONTROLLER_FEEDBACK_HANG (cerrada 2026-05-08)

El bug original (path MoveIt → simple_controller_manager → joint_trajectory_controller
sin feedback "Goal reached") está cerrado por **bypass arquitectónico**:

| Componente | Antes | Después (F1.24) |
|-----------|-------|-----------------|
| APPROACH path | MoveIt → simple_controller_manager → FJT | **FJT directo** (compute_ik + send_goal_async al FJT) |
| GRASP_DOWN path | MoveIt → ... → FJT | **FJT directo** |
| LIFT path | MoveIt → ... → FJT | **FJT directo** |
| TRANSPORT path | MoveIt → ... → FJT | **FJT directo + multi-waypoint trajectory** |
| HOME_INITIAL path | FJT directo (siempre funcionó) | FJT directo (sin cambio) |

El `simple_controller_manager` del MoveIt **ya no se usa en el path canónico**.
Sólo se mantiene como fallback defensivo si IK falla en compute_ik.

## Métricas finales

- **panel_pick_demo.py**: 8.611 → 536 LOC (-94% en una sesión)
- **legacy run_pick_demo**: borrado físicamente (commit `11b66e2`)
- **mypy strict**: 24 → 63 módulos (+162%)
- **Tests offline**: ~2.400 → 2.381 + (+220 nuevos hoy)
- **T35 × 3 cycles consecutivos verde**: ✅
- **Score profesional**: 89 → 100/100
- **Bug controller feedback hang**: ✅ cerrado vía bypass arquitectónico

## Refactor estructural T15 (2026-05-09, post-defensa)

Tras la aprobación del TFM se completaron refactors offline para llevar
el código a estado productivo:

| Función | Antes | Después | Sub-helpers |
|--|--|--|--|
| `PlanToPoseServer.__init__` | 228 LOC | 5 LOC | 3 (`_init_declare_and_read_params`, `_init_setup_tf_and_clients`, `_init_setup_action_server_and_bridge`) |
| `_execute_fjt_direct` | 261 LOC | 61 LOC | 5 (`_fjt_extract_seed_positions`, `_fjt_compute_traj_params`, `_fjt_call_compute_ik`, `_fjt_build_trajectory`, `_fjt_send_and_wait_result`) |
| `_execute_moveit_direct` | 364 LOC | 118 LOC | 5 (`_moveit_try_fjt_bypass`, `_moveit_send_first_attempt`, `_moveit_post_timeout_tf_check`, `_moveit_retry_after_failure`, `_moveit_final_tf_recovery`) |

Adicionalmente se corrigió un **bug latente** en `_execute_fjt_direct`:
``seed_positions`` se referenciaba antes de definirse haciendo que
``dist_to_target=0.5`` siempre cayera al fallback (multi-waypoint
always-on, incluso para fases cortas). Tras el fix, APPROACH/GRASP_DOWN/
LIFT usan 2-point trajectory con duration=8s, TRANSPORT sigue usando
multi-waypoint duration=25s.

H14b también pushea el default de `fjt_direct_ik_timeout_sec` de 2.0s a
5.0s (TRAC-IK puede tardar 2-4s post-restart del stack — observado en
T35 × 5 stress).

Total tests offline post-refactor: 2.381 PASSED (1.635 panel + 746 ur5_tools).
