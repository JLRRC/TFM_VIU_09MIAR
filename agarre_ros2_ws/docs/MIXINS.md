# MIXINS — Arquitectura del UR5MoveItBridge

Documenta los 9 mixins que componen `UR5MoveItBridge` (LifecycleNode), cada
uno con responsabilidad única extraída de un god-file de ~5.000 LOC original.

## MRO (Method Resolution Order)

```python
class UR5MoveItBridge(
    MoveItPyPlannerMixin,        # 1: backend principal MoveItPy + init
    MoveItCommanderMixin,        # 2: backend alternativo MoveItCommander
    GeometryMixin,               # 3: utilidades geométricas (poses, frames)
    TrajectoryPrepMixin,         # 4: preparación trajectory (rescaling, dedupe)
    FjtLifecycleMixin,           # 5: ciclo de vida del goal FJT
    ExecutorMixin,               # 6: bucle plan-execute principal
    JointStateHelpersMixin,      # 7: cacheo joint_state + queries
    ControllerManagementMixin,   # 8: switch_controller, restart, params env
    GoalValidationMixin,         # 9: validación poses, framas, tolerances
    LifecycleNode,               # rclpy LifecycleNode
):
```

Python resuelve dependencias entre mixins por nombre vía `self.<method>`.
Cada mixin documenta en su docstring qué métodos/atributos espera vía `self`
(provistos por otros mixins más a la derecha del MRO o por LifecycleNode).

## Resumen por mixin

| Mixin | Responsabilidad | Líneas | Origen |
|--|--|--|--|
| `MoveItPyPlannerMixin` | Pipeline plan+execute con MoveItPy. APPROACH IK seeded, path constraints relaxed retry, endpoint validation, dry run, force FJT direct, fallback FJT directo, fallback topic publish, replan desde current state. Init `MoveItConfigsBuilder` + `MoveItPy` + `PlanningComponent`. | ~700 | ur5_moveit_bridge.py:2855-3562 (audit-v4.0) |
| `MoveItCommanderMixin` | Backend alternativo via `moveit_commander` Python (legacy). Mantiene compatibilidad con stacks pre-MoveItPy. | ~250 | F3-step refactor |
| `GeometryMixin` | Utilidades geométricas: composition de transforms, normalize_quat, lookups TF, conversiones world↔base. | ~150 | F3-step refactor |
| `TrajectoryPrepMixin` | Preparación de `JointTrajectory` antes de enviarlo: rescaling per-fase (TRANSPORT 0.5x, otros 0.25x), `_prepare_joint_trajectory_for_controller`, dedupe puntos, cap APPROACH timeout. | ~300 | F5-iter1/2/3 |
| `FjtLifecycleMixin` | Ciclo de vida del goal FJT: `_prepare_fjt_execution`, `_send_and_accept_fjt_goal`, `_setup_post_accept_state`. | 230 | audit-v4.1/A (commit 13ac4f5) |
| `ExecutorMixin` | Bucle plan-execute principal `_execute_joint_trajectory_action` (~1027 LOC tras F3-step6 a..f). Coordina: build → send → wait → accept → execute → wait_result → handle_outcome. | 1491 (file) | F3-step6 a..f |
| `JointStateHelpersMixin` | Cacheo del último `JointState` + helpers de queries: `_current_arm_joint_vector`, `_set_planning_start_state_from_joint_state`, freshness checks. | ~200 | F3-step refactor |
| `ControllerManagementMixin` | `switch_controller` client, restart joint_trajectory_controller, params env helpers (`_env_float`, `bridge_env_*`). | ~200 | F3-step refactor |
| `GoalValidationMixin` | Validación de target poses (frame, tolerance, workspace), construcción de constraints (PositionConstraint + OrientationConstraint). | ~150 | F3-step refactor |

## Diagrama de dependencias entre mixins

```
                     LifecycleNode
                          ▲
                          │
    GoalValidationMixin ──┤
ControllerManagementMixin ┤
   JointStateHelpersMixin ┤
            ExecutorMixin ──── usa todos los mixins de la izquierda vía self
        FjtLifecycleMixin ──── usa Executor + TrajectoryPrep + JointStateHelpers
        TrajectoryPrepMixin ── usa Geometry + JointStateHelpers
              GeometryMixin
       MoveItCommanderMixin
       MoveItPyPlannerMixin ─── llama a Executor para ejecutar trayectorias
                          │
                          ▼
                  UR5MoveItBridge
```

Las flechas indican "usa vía `self.<method>`". Python resuelve con MRO de
izquierda a derecha → padre a hijo. Cada mixin documenta sus dependencias
explícitamente en su docstring para facilitar el mantenimiento.

## Cómo añadir un nuevo mixin

1. Crear `moveit_bridge/mi_mixin.py` con `class MiMixin:` (sin Node base).
2. Documentar en docstring qué `self.<method>/<attr>` espera.
3. Implementar métodos sin estado propio (todo el estado vive en
   `UR5MoveItBridge` vía `self.X`).
4. Registrar en MRO de `UR5MoveItBridge` en posición coherente:
   - **Más a la izquierda** → métodos resueltos primero (override).
   - **Más a la derecha** → métodos de "infraestructura" (lifecycle, env).
5. Actualizar este documento + `architecture_post_legacy.md`.
6. Tests offline en `test_<mi_mixin>.py` con mocks de `self.X` mínimos.

## Plan F5-iter5 / F5-iter6 (post-defensa)

Mixin extraction roadmap pendiente — cada uno reduce `ur5_moveit_bridge.py`
(actualmente 1.849 LOC) en ~200-400 LOC:

- `F5-iter5`: extraer `_init_moveit_py` (~146 L) a `init_helpers.py` o
  similar.
- `F5-iter6`: extraer `_resolve_target_for_*` y `_validate_endpoint_*`
  (geometría/validación combinadas) a `endpoint_validation_mixin.py`.
- `F5-iter7`: extraer fallback paths (`_fallback_to_fjt_direct`,
  `_fallback_publish_to_topic`) a `fallback_paths_mixin.py`.

Objetivo final: `ur5_moveit_bridge.py` < 800 LOC (sólo wiring + lifecycle
transitions). El "código que hace cosas" vive en mixins.
