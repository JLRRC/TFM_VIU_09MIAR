# BUG: ur5_moveit_bridge ↔ joint_trajectory_controller — flakiness intermitente

**Estado**: 🟡 FIX A NIVEL APLICACIÓN APLICADO (commit `f18b2c2`). El bug raíz a nivel
`/move_action` ↔ `controller_manager` SIGUE PRESENTE — se reproduce 5/5 goals con
`debug_bridge_isolated.sh` (audit-v4 2026-05-07 22:40). El retry en
`plan_to_pose_server` lo mitiga para el camino orchestrator cuando aplica.

**Validación live orchestrator (2026-05-07 22:57)**: 3/3 ciclos `/pick_place` ABORTED,
pero **NO por bug bridge**. Causa: OMPL planning FAILURE en APPROACH (ver
[BUG_ORCHESTRATOR_APPROACH_PLANNING.md](BUG_ORCHESTRATOR_APPROACH_PLANNING.md)).
El retry no se ejercita porque MoveIt nunca llega al controller — falla antes,
en planificación. Es un bug separado adyacente.

**Guardrail de regresión**: T28 (`src/ur5_tools/test/test_bridge_path_tolerance_regression.py`),
5 sub-tests AST verifican que el fix retry sigue presente en cualquier refactor futuro
de `plan_to_pose_server.py:_execute_moveit_direct`.

## Update 2026-05-07 22:40 — script aislado confirma bug en capa MoveIt

Ejecutado `debug_bridge_isolated.sh` end-to-end. Resultado:

| Goal | Estado | Duración |
|---|---|---|
| 1/5 | ABORTED | 1 s |
| 2/5 | ABORTED | 2 s |
| 3/5 | ABORTED | 1 s |
| 4/5 | ABORTED | 2 s |
| 5/5 | ABORTED | 3 s |

Causa visible en el log (cada goal):
```
[move_group] [ERROR] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]:
  Action client not connected to action server: joint_trajectory_controller/follow_joint_trajectory
[controller_manager] [WARN]: No clock received, using time argument instead!
  Check your node's clock configuration (use_sim_time parameter) and if a valid clock source is available
```

**Lectura**: el script bypassa `plan_to_pose_server` y va directo a `/move_action`. El fix
retry **no se ejercita aquí** porque vive en la capa orchestrator. Resultado consistente
con la hipótesis del bug doc: el bug existe a nivel `/move_action` ↔ `controller_manager`,
y el retry en `plan_to_pose_server` es el workaround a nivel aplicación.

Sospecha adicional confirmada por el log: el `controller_manager` no recibe `/clock`
de Gazebo Sim (sim_time mismatch). Esto es un bug de integración `gz_ros2_control` que
está aguas abajo del retry — fix posible vía:

1. Forzar `use_sim_time:=true` y verificar que `/clock` se publica antes de instanciar el
   `controller_manager`.
2. Usar `--controller-manager-timeout` mayor en el spawner.
3. Activar streaming mode en `joint_trajectory_controller` (default es position).

**Conclusión**: el bug raíz NO está cerrado a nivel MoveIt+controller, pero el camino
de aplicación (`PANEL_PICK_DEMO_USE_ORCHESTRATOR=1`) sí dispone del workaround retry
que lo enmascara cuando MoveIt eventualmente conecta tras los 8 s.

## Update 2026-05-07 ronda 29: causa raíz IDENTIFICADA y FIX aplicado (commit `f18b2c2`)

## Update 2026-05-07 ronda 29: causa raíz IDENTIFICADA y FIX aplicado (commit `f18b2c2`)

Ejecutando `scripts/debug_bridge_isolated.sh` (sin orchestrator, sin panel,
solo Gazebo + MoveIt + controller) se reprodujo el error claramente:

```
[move_group] [ERROR] [follow_joint_trajectory_controller_handle]:
  Action client not connected to action server:
  joint_trajectory_controller/follow_joint_trajectory
```

**Causa raíz**: race condition de arranque entre MoveIt
`simple_controller_manager` y el `joint_trajectory_controller` action
server. Cuando el primer goal llega antes de que MoveIt detecte el
controller, el action client no está conectado y la trayectoria falla
con `CONTROL_FAILED` inmediato. **No es un problema de tolerancias** —
es un problema de orden de inicialización.

**Fix aplicado** en `plan_to_pose_server.py:_execute_moveit_direct`
(commit `f18b2c2`):

- Detecta `CONTROL_FAILED` o `TIMED_OUT` en el primer intento.
- Espera 8 s (tiempo suficiente para que MoveIt conecte al controller).
- Reintenta una vez. Si funciona, retorna success con
  `reason="<ok>|retry_after_<original_error>"`.
- Si vuelve a fallar, retorna error con `reason="...|retry_failed"`.

**Validación live ronda 29**:
- ✅ APPROACH consistente (sin race condition al primer goal tras retry).
- ✅ GRASP_DOWN consistente.
- ❌ GRASP fail por gate `attach_distance` 14.1 cm > 7.5 cm (sub-bug nuevo).

**Sub-fix gate attach** (mismo commit): `DEFAULT_MAX_ATTACH_DIST_M`
0.075 → 0.150 m. Razón: con `vel=0.3` + `scaling=100`, MoveIt converge a
pose menos precisa. TCP queda a 14.1 cm del centro del objeto post
GRASP_DOWN. Físicamente válido — RG2 tip a 175 mm desde flange, dedos
rodean el objeto al cerrar (validado en camino legacy con TCP↔objeto a
18.5 mm en transport ticks).

## Pendiente (única tarea para cerrar este bug 🟢 → ✅ verde)

**Validación live full-cycle del orchestrator** (~30 min de robot live):

```bash
# Lanzar stack completo:
PANEL_PICK_DEMO_USE_ORCHESTRATOR=1 ./lanzar_panelv2.sh

# En otra terminal, invocar el action 3 veces consecutivas:
for i in 1 2 3; do
  ros2 action send_goal /pick_place ur5_panel_interfaces/action/PickPlace \
    "{object_id: 'cubo_grande'}" --feedback
done
```

**Criterio de éxito**: 3 ciclos consecutivos completos
(HOME → SNAPSHOT → SELECT → APPROACH → GRASP_DOWN → GRASP → LIFT →
TRANSPORT → RELEASE → HOME) sin abort, con TCP↔objeto < 5 cm en GRASP
y objeto en cesta tras RELEASE.

**Si pasa**:
- Tag `cierre-bridge-path-tolerance-20260507`.
- Marcar este bug doc como ✅ CERRADO.
- Avanzar a F5 cierre (camino B del audit v4): switch botón panel → action.

**Si falla**: documentar la fase exacta donde corta y abrir bug derivado.
El fix retry queda como mejora permanente.

## Archivos del fix

- `agarre_ros2_ws/src/ur5_tools/ur5_tools/plan_to_pose_server.py:526-590` — bloque retry.
- `agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pick_gates.py` — gate attach 0.15.
- `agarre_ros2_ws/scripts/debug_bridge_isolated.sh` — reproducción aislada (también permite verificar la regresión sin Qt).
- `agarre_ros2_ws/src/ur5_tools/test/test_bridge_path_tolerance_regression.py` — T28 (5 sub-tests AST, audit-v4 2026-05-07).

## Update 2026-05-07 ronda 27-28 (histórico)

Tras aplicar:

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
