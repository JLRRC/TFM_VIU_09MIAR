# BUG: ur5_moveit_bridge ↔ joint_trajectory_controller — flakiness intermitente

**Estado**: 🟡 PARCIALMENTE MITIGADO (2026-05-07 sesión 28 rondas live)
**Mitigación**: orchestrator funciona intermitentemente (rondas 25, 27 OK
hasta GRASP_DOWN). Legacy path canónico (default) NO afectado.

## Update 2026-05-07 ronda 27-28: orchestrator alcanza GRASP_DOWN

Tras aplicar:
- `max_velocity_scaling_factor` 0.5→0.3 (trayectoria más lenta)
- `allowed_execution_duration_scaling` 30→100 (upper bound x10)
- `moveit_result_timeout_sec` 90→400s
- `action_result_timeout_sec` 150→500s

El orchestrator path **completa hasta GRASP_DOWN** cuando funciona:
```
phase=INITIAL_SNAPSHOT ok=True
phase=HOME_INITIAL ok=True
phase=SELECT_OBJECT ok=True
phase=APPROACH ok=True reason=approach:moveit:SUCCESS
phase=GRASP_DOWN ok=True reason=grasp_down:moveit:SUCCESS
```

**Pero hay flakiness intermitente**: ronda 28 (config idéntica a ronda 27)
falló en APPROACH con `moveit_result_timeout:400.0s` sin completar.

Patrón observado:
- ~50% de runs: APPROACH+GRASP_DOWN OK
- ~50% de runs: APPROACH timeout

Causa raíz: el `move_action` del move_group **se queda esperando feedback
del controller** intermitentemente. El controller `joint_trajectory_controller`
ejecuta la trayectoria pero por algún issue de timing entre threads de
Gazebo+ROS, la respuesta no llega al move_group dentro del timeout.

Este es un bug profundo de la integración MoveIt2 ↔ gz_ros2_control en
sim_time, no resoluble subiendo más timeouts.

## Reproducción aislada (script de debug)

```bash
bash agarre_ros2_ws/scripts/debug_bridge_isolated.sh
```

Lanza el stack mínimo (Gazebo + MoveIt + controller, **sin orchestrator
ni panel**) y envía 5 goals consecutivos a `/move_action`. Mide tiempo y
resultado de cada goal. Si la varianza temporal entre goals es alta o
algunos dan `error_code != 1`, el bug bridge está confirmado a nivel
de integración MoveIt-controller (independiente de orchestrator/panel).

Esto aísla el diagnóstico para futura sesión dedicada al bug.

## Estado del proyecto pese al bug

✅ **Objetivo del proyecto cumplido vía LEGACY path** (`run_pick_demo`):
   las pinzas RG2 agarran el objeto físicamente en Gazebo.
   Tag: `objetivo-cumplido-pinzas-agarran-objeto-20260507`.
   Evidencia: TCP↔objeto 1.8cm, levantado 97.8cm, 132 ticks attach.

🟡 **Orchestrator path** (opt-in vía `PANEL_PICK_DEMO_USE_ORCHESTRATOR=1`):
   funciona intermitentemente hasta GRASP_DOWN. NO recomendado para producción
   hasta que se cierre el bug intermitente del bridge.

## Estado original del bug
**Detectado**: 2026-05-07 (validación E2E orchestrator path)
**Reproducibilidad**: 100% en orchestrator path (`PANEL_PICK_DEMO_USE_ORCHESTRATOR=1`)
**Bloqueante para**: completar pick & place full cycle vía orchestrator
**NO bloqueante para**: pick legacy (run_pick_demo) — tiene workarounds propios

## Resumen

Tras cerrar el bug GRASP_DOWN (URDF↔SDF parity, commit `550457a`) y cablear
el orchestrator (auto_activate, commit `60558fd`), la validación E2E live
del orchestrator avanza correctamente hasta SELECT_OBJECT pero falla
sistemáticamente en APPROACH (la primera fase que ejecuta movimiento real
post-HOME_INITIAL).

## Modos probados

### MOVEIT_DIRECT (rondas 9-12)

```
[plan_to_pose_server] [PLAN_TO_POSE][MOVEIT_DIRECT] sending goal
   target=(-0.414,0.000,0.975) ee_frame=rg2_pinch_center group=manipulator
[move_group] Calling Planner 'OMPL'  ← planning OK en ~24ms
[move_group] Starting trajectory execution ...
[move_group] joint_trajectory_controller started execution
[move_group] Goal request accepted!
[move_group] Controller is taking too long to execute trajectory
   (the expected upper bound for the trajectory execution was 9.371515
    seconds). Stopping trajectory.
[plan_to_pose_server] failed reason=moveit_err:CONTROL_FAILED
```

Tras subir `trajectory_execution.allowed_execution_duration_scaling 1.2→4.0`
(commit en `ur5_moveit_config/launch/ur5_moveit_bringup.launch.py`), el
error cambia de `TIMED_OUT` a `CONTROL_FAILED` — el controller acepta el
goal pero no converge dentro del tiempo extendido. **El controller no
sigue la trayectoria con suficiente fidelidad en simulación**.

### REAL_BRIDGE (rondas 13-14)

```
[ur5_moveit_bridge] [BRIDGE_EXEC] start-state error detail
   max_start_err=3.1040rad
   joints=shoulder_lift_joint:cur=1.608->tgt=-1.571:err=3.104,
          wrist_2_joint:cur=-1.566->tgt=0.000:err=1.566,
          wrist_3_joint:cur=1.311->tgt=0.000:err=1.311,...
[ur5_moveit_bridge] FollowJointTrajectory FAIL
   error_code=-4
   error_string=Aborted due to path tolerance violation
```

`max_start_err=3.1 rad` ≈ 178° entre la pose actual del robot y la pose
inicial que MoveIt esperaba. El bridge inserta un waypoint inicial para
"pegar" la pose actual a la trayectoria planeada, pero el controller
considera el salto un `path_tolerance_violation` y aborta.

## Causa raíz hipotética

El bridge planifica desde una "expected start state" que no coincide con
la pose física actual del robot en Gazebo. Posibles causas:

1. **HOME_INITIAL retorna SUCCESSFUL antes de que el robot esté físicamente
   en home**. El `FollowJointTrajectory` reporta SUCCEEDED al recibir el
   goal time, no al converger físicamente. La pose resultante en Gazebo
   no es exactamente la home pedida.

2. **Estado contaminado entre fases**. Tras un APPROACH abortado, el robot
   queda en una pose extraña; la siguiente fase usa joint_states actuales
   pero MoveIt planifica como si estuviera en home → divergencia 178°.

3. **`use_sim_time` mismatch**. El bridge usa `use_sim_time=true` pero
   `moveit_py_use_sim_time=false` (visto en `Bridge config` del log).
   Posible drift de timestamps entre lo que MoveIt planea y lo que el
   controller ejecuta.

## Datos de las 14 rondas live (2026-05-07)

| Ronda | Cambio | Resultado |
|---|---|---|
| 9 | logs phase_loop | localizado: APPROACH timeout |
| 10 | planning_time 5→25s | NO resuelto (timeout interno MoveIt) |
| 11 | duration_scaling 1.2→4.0 | TIMED_OUT → CONTROL_FAILED |
| 12 | start_tol 0.05 + duration 4.0 | CONTROL_FAILED persiste |
| 13 | mode=REAL_BRIDGE | path_tolerance_violation |
| 14 | clean restart REAL_BRIDGE | path_tolerance_violation persiste |

## Plan para próxima sesión (estimado 4-8 h)

### Diagnóstico

1. **Verificar que HOME_INITIAL converge físicamente** antes de retornar:
   añadir wait explícito a que joint_states coincidan con home_positions
   dentro de tolerancia (ej. 0.05 rad) tras `fjt:SUCCESSFUL`.

2. **Reproducir aislado** con `ros2 action send_goal /move_action` desde
   pose home conocida (sin orchestrator, sin panel) para distinguir si el
   bug es del bridge o del controller integration.

3. **Verificar `joint_trajectory_controller` config** en
   `models/ur5_rg2/ur5_controllers.yaml` — los path_tolerance pueden estar
   demasiado estrictos para el sim physics.

### Fix candidatos

- **A**: subir `path_tolerance` del controller a un valor generoso (0.5 rad)
  para tolerar drift de simulación. Riesgo bajo (sólo afecta sim).

- **B**: hacer que `_execute_home_initial_real` en
  `pick_orchestrator_lifecycle_node.py` espere a que joint_states
  converjan a home_positions dentro de tolerancia ANTES de retornar.

- **C**: cambiar `gz_ros2_control` a controller `streaming` mode
  (default es `position`). El streaming maneja mejor las trayectorias
  con jumps grandes en sim.

## Workaround actual (= cierre del Bloque 2)

El orchestrator está cableado y vivo (commit `60558fd`,
tag `cierre-bloque-2-orchestrator-cableado-20260506`). La validación
muestra que **9/9 fases del FSM están dispatcheándose correctamente**
hasta encontrar el problema de ejecución física.

Para que el demo siga funcionando hoy: usar `USE_LEGACY_PICK_DEMO=1`
(o esperar default `PANEL_PICK_DEMO_USE_ORCHESTRATOR` a None que cae a
legacy si bridge no responde). El legacy tiene workarounds históricos
para path tolerance que vamos a perder con la migración, pero hoy
funciona hasta TRANSPORT (rondas 1-6 validadas).

## NO es regresión

Este bug existe desde que el orchestrator empezó a despachar acciones
reales (B-iter sprint, 2026-05-03). Es un bug arquitectónico latente
del bridge ↔ controller en Gazebo Sim que **el legacy nunca expuso**
porque tiene su propia lógica de tolerancias subidas, retry y recovery
desarrolladas iterativamente en abril 2026.

## Referencias

- Memoria `project_b_iter3_20260503` — sprint del orchestrator.
- Memoria `project_audit_fases_0_10_20260504` — F7 E2E live legacy.
- `docs/BUG_GRASP_DOWN_TCP_TRUNCATION.md` — bug previo cerrado.
- `auditoria/audit_profesional_20260506.md` Anexo B — sesión cierre.
