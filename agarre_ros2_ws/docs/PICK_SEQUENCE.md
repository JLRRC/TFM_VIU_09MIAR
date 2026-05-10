# Secuencia del Pick & Place — UR5+RG2

**Fecha:** 2026-05-10 (auditoría F10.2).

Este documento describe la secuencia completa del pick & place orquestada por `tfm_orchestrator/pick_orchestrator_lifecycle`. Es el contrato funcional del pipeline.

---

## 1. Diagrama de fases

```
   IDLE
    │ (goal /pick_place recibido)
    ▼
INITIAL_SNAPSHOT  ──► capturar TCP/joints/objeto vía TF + ResolveObjectPoseWorld
    │
    ▼
HOME_INITIAL      ──► FollowJointTrajectory hasta UR5_HOME_POSITIONS_RAD
    │
    ▼
SELECT_OBJECT     ──► validar object_name (internal-only)
    │
    ▼
APPROACH          ──► PlanToPose: 10cm sobre objeto, TCP top-down (yaw alineado)
    │
    ▼
GRASP_DOWN        ──► PlanToPose cartesiano: descender 8cm hasta 2cm sobre objeto
    │
    ▼
GRASP             ──► /gripper/close + /orchestrator/attach
    │                  └─ gate: attach_distance ≤ 0.25m
    ▼
LIFT              ──► PlanToPose: subir 30cm sobre objeto, TCP top-down
    │
    ▼
TRANSPORT         ──► PlanToPose hacia drop_xyz_world (con retry × 2, backoff)
    │
    ▼
RELEASE           ──► /orchestrator/detach + /gripper/open
    │                  └─ gate: open_full ≥ 0.025m
    ▼
   DONE                (success=True en PickPlace.Result)
```

Caminos de fallo: cualquier fase puede transicionar a `FAILED` o `ABORTED` (cancel).

---

## 2. Tabla detallada por fase

| Fase | Service / Action invocado | Gates evaluados | Timeout | Notas |
|---|---|---|---|---|
| **INITIAL_SNAPSHOT** | TF lookup (base_link↔rg2_tcp) + JointState + `ResolveObjectPoseWorld` | freshness (informativo, no bloquea) | TF 0.5s, discovery 0.3s | Si TF/JointState no inicializados → fallback graceful (no aborta) |
| **HOME_INITIAL** | `/joint_trajectory_controller/follow_joint_trajectory` | posición ≤ 1.0 rad tol | result 30s, server 3s | F1.7: defaults relajados para Gazebo Sim |
| **SELECT_OBJECT** | (internal-only — no llamada externa) | `object_name` ≠ vacío | n/a | F12: bypass del legacy `/panel/select_object` |
| **APPROACH** | `/orchestrator/plan_to_pose` | TCP top-down (180° X + yaw objeto) | 700s | Si no hay hint TF, fallback a `ResolveObjectPoseWorld` |
| **GRASP_DOWN** | `/orchestrator/plan_to_pose` (cartesian=true) | TCP apunta -Z world | 700s | Insertado 2026-05-07 para que RG2 toque el objeto antes del attach |
| **GRASP** | `/gripper/close` + `/orchestrator/attach` | `attach_distance ≤ 0.25m` (B-iter8 gate) | 10s + 10s | Detecta "drop_anchor placebo" donde Attach success=True pero TCP a 1m+ del objeto |
| **LIFT** | `/orchestrator/plan_to_pose` | TCP top-down coherente con APPROACH | 700s | |
| **TRANSPORT** | `/orchestrator/plan_to_pose` con retry | TCP top-down | 700s + 3s wait | retry.py: `max_attempts=2`, backoff 1s→2s exp |
| **RELEASE** | `/orchestrator/detach` + `/gripper/open` | gripper abierto ≥ 0.025m | 10s + 10s | |

---

## 3. Tabla declarativa (machine-readable)

`tfm_orchestrator.phase_dispatch.PHASE_DISPATCH_METADATA` (F9.4) expone esta tabla en código Python para tests parametrizados:

```python
{
    PickPhase.SELECT_OBJECT: {"type": "internal", "purpose": "..."},
    PickPhase.APPROACH:      {"type": "action", "actions_used": ["/orchestrator/plan_to_pose"], ...},
    PickPhase.GRASP:         {"type": "service", "services_used": ["/gripper/close", "/orchestrator/attach"], "gates": ["attach_distance"], ...},
    ...
}
```

Test `test_phase_dispatch_table.py` garantiza coverage del FSM completo.

---

## 4. Comunicación con MoveIt 2

`/orchestrator/plan_to_pose` (PlanToPose.action) tiene 3 modos (parameter `mode`):

- **MOVEIT_DIRECT** (default desde B-iter3): cliente directo a `/move_action` de MoveIt. Bypassa el bridge al panel. **Ruta canónica**.
- **REAL_BRIDGE** (legacy): usaba el `ur5_moveit_bridge` (borrado 2026-05-09).
- **STUB** (testing): no toca MoveIt, devuelve success tras `step_delay_sec`.

`MOVEIT_DIRECT` añade además un *FJT bypass* opcional para path cortos (`bypass_moveit_for_short_paths=True`): si el target está cerca del seed actual, ejecuta directo via `FollowJointTrajectory` sin pasar por simple_controller_manager. Mitigación de un bug histórico de MoveIt hung en el bridge.

---

## 5. Gates puros (testables sin ROS)

Definidos en `tfm_orchestrator/pick_gates.py`:

```python
evaluate_attach_distance_gate(tcp_pos, obj_pos, max_dist=0.25)
    → (ok, reason)

evaluate_close_delta_gate(opening_before, opening_after, min_delta=0.005)
    → (ok, reason)

evaluate_release_open_gate(opening_after, min_open=0.025)
    → (ok, reason)
```

21 tests en `test_pick_gates.py`.

---

## 6. Retry con back-off (TRANSPORT)

Definido en `tfm_orchestrator/retry.py`:

```python
retry_with_backoff(
    call_fn,                  # callable a invocar
    max_attempts=2,
    initial_backoff=1.0,
    factor=2.0,
    sleep_fn=None,            # default time.sleep, override en tests
)
    → resultado de call_fn (excepción capturada como intento gastado)
```

Solo activo en TRANSPORT. APPROACH/GRASP_DOWN/LIFT/RELEASE no tienen retry porque su fallo es informativo (revisar geometría, no insistir).

---

## 7. Cancelación

`PickPlaceClient` (panel) puede llamar `cancel_goal_async()`. El orchestrator:

1. `_cancel_callback` setea `_cancel_requested=True`.
2. `_execute_callback` chequea la flag al final de cada fase.
3. Si activa: `ctx.abort()` → `goal_handle.canceled()` → `result(success=False, reason="canceled")`.

`panel_backend_node` expone `~/cancel_pick` (Trigger) que delega esto.

---

## 8. Logging estructurado

Formato canónico (run_id.py:format_log_line):

```
[abcd1234][APPROACH][orch][STARTED] object=box_red plan_time_ms=240
[abcd1234][APPROACH][orch][FINISHED] success=True duration_sec=2.4
[abcd1234][GRASP][orch][GATE_FAILED] gate=attach_distance dist=0.34 max=0.25
```

`evidence_logger` consume el run_id latched y escribe JSONL agregado por ciclo en `historico/<run_id>/`.

---

## 9. Validación E2E

**Manual:**
```bash
ros2 launch ur5_bringup ur5_stack.launch.py
ros2 lifecycle set /pick_orchestrator_lifecycle configure
ros2 lifecycle set /pick_orchestrator_lifecycle activate
ros2 action send_goal /pick_place ur5_panel_interfaces/action/PickPlace \
    "{ object_name: 'box_red', drop_xyz_world: [-1.30, 0.0, 0.82], timeout_sec: 700.0 }"
```

**Automatizado:**
```bash
./scripts/validate_pick_3_cycles.sh        # 3 ciclos, KPI JSON en log/
PICK_VALIDATE_CYCLES=5 ./scripts/validate_pick_3_cycles.sh
```

**CI:** workflow `e2e-live-on-demand.yml` con label `run-t35-on-demand` o `workflow_dispatch`.
