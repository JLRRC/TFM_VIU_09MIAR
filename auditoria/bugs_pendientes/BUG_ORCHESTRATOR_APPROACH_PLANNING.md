# BUG: orchestrator path APPROACH — OMPL planning FAILURE

**Estado**: 🟡 CAUSA RAÍZ IDENTIFICADA (audit-v4 F1.6 2026-05-07 23:05).
**Causa**: cliente envió `object_pose_world_hint` con position=(0,0,0) +
identity quaternion → `is_no_hint(pose) == True` → orchestrator usa placeholder
goal a `(0,0,0)` en `base_link` (la base del robot) → OMPL FAILURE garantizado.

**Fix sugerido**: cliente debe llamar `/orchestrator/resolve_object_pose_world`
ANTES de invocar `/pick_place` y pasar la pose real como `object_pose_world_hint`.
F5-step3 ya implementa el resolver — sólo falta cablearlo.

**Reproducibilidad**: 100% cuando se invoca con hint=identity (caso del
audit-v4 F1.5 ronda inicial 22:57).
**Bloqueante para**: cierre F1.5 al 100% con criterio "3 ciclos sin abort".
**NO bloqueante para**: camino legacy `run_pick_demo` (que computa la pose
del objeto internamente sin depender del hint del cliente).

## Resumen

Tras lanzar el stack vivo headless con
`launch_pick_orchestrator_lifecycle:=true`, soltar objetos vía
`/release_objects`, y enviar 3 goals consecutivos a `/pick_place` con
`object_name='box_red'`, los 3 ciclos abortan en fase **APPROACH** con
error `moveit_err:FAILURE` (planner OMPL no encuentra solución).

**No es el bug bridge**. El bridge fix (`f18b2c2`) sigue válido y necesario
para `CONTROL_FAILED` / `TIMED_OUT`, pero aquí MoveIt nunca envía
trayectoria al controller — falla en planificación.

## Evidencia live (2026-05-07 22:57-22:59)

| Ciclo | reason | duration | Detalle |
|---|---|---|---|
| 1/3 | `approach:moveit_err:FAILURE` | 35 s | OMPL FAILURE tras 24 s de planning |
| 2/3 | `approach:moveit_err:FAILURE` | 35 s | OMPL FAILURE consistente |
| 3/3 | `approach:action_returned_false` | 8 s | Orchestrator abortó preemptivamente |

Goals enviados (idénticos los 3):
```yaml
object_name: "box_red"
drop_xyz_world: {x: -1.30, y: 0.0, z: 0.85}
object_pose_world_hint: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1.0}}
```

## Logs relevantes

```
[move_group] [INFO] MoveGroupMoveAction: Received request
[move_group] [INFO] Combined planning and execution request received
[move_group] [ERROR] Planner 'OMPL' failed with error code FAILURE
[move_group] [INFO] FAILURE
[plan_to_pose_server] [WARN] [PLAN_TO_POSE][MOVEIT_DIRECT] failed reason=moveit_err:FAILURE
[plan_to_pose_server] [WARN] [PLAN_TO_POSE] aborted reason=moveit_err:FAILURE
[pick_orchestrator_lifecycle] [INFO] phase_loop _execute_phase returned phase=APPROACH ok=False reason=approach:moveit_err:FAILURE
```

## Causas hipotéticas (a investigar)

1. **`object_pose_world_hint` con identity = "no hint" no resuelve la pose
   correctamente**. El orchestrator usa placeholder o resuelve via
   `ComputeApproachPose` (F5-step3 documentado como "pendiente" en bug
   doc original). Si `ComputeApproachPose` devuelve una pose inalcanzable
   o en colisión, OMPL falla.
2. **Approach pose calculada está en colisión con la mesa**. Si el
   orchestrator computa approach_pose a 5 cm encima de `box_red`
   (z ≈ 0.83 m) y la celda de colisión incluye la mesa, OMPL declara
   FAILURE.
3. **TF stale del objeto**. `box_red` recién liberado, pose no se ha
   propagado a `world_tf_publisher` en tiempo. Resolución vía TF devuelve
   pose home antigua o (0,0,0).
4. **Workspace bounds insuficientes**. Log muestra
   `It looks like the planning volume was not specified. Using default
   values` — el OMPL volumen por defecto (cube 5 m alrededor del robot)
   debería bastar pero conviene verificar.
5. **Robot start state inválido**. Si el robot quedó en pose extraña tras
   ciclo previo, planning desde esa pose puede ser geométricamente
   inviable a la pose objetivo.

## Reproducción mínima

Tras lanzar el stack vivo:
```bash
source /opt/ros/jazzy/setup.bash
source agarre_ros2_ws/install/setup.bash

# Verificar action ready:
ros2 lifecycle get /pick_orchestrator_lifecycle  # debe ser "active [3]"
ros2 action list | grep /pick_place              # debe estar listado

# Soltar objetos:
ros2 service call /release_objects std_srvs/srv/Trigger "{}"
sleep 4

# Enviar goal:
ros2 action send_goal /pick_place ur5_panel_interfaces/action/PickPlace \
  "{object_name: 'box_red', drop_xyz_world: {x: -1.30, y: 0.0, z: 0.85}, \
    object_pose_world_hint: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1.0}}}" \
  --feedback
```

Resultado esperado actual: `success: false`, `reason: approach:moveit_err:FAILURE`,
`duration_sec: ~35s`.

## Update 2026-05-08 07:30 — F1.8 helper offline-shipped, live blocked por /clock

**F1.8 helper** `compute_top_down_grasp_quat` shipped en commit `34672b0`:
- 8 unit tests verdes verifican el cálculo de TCP top-down quat con yaw del objeto.
- Wired en `build_plan_to_pose_goal_for_approach` y `_grasp_down`.
- Fix matemático correcto y tested.

**Validación live BLOQUEADA** por bug raíz adyacente al stack:
```
[controller_bootstrap] [ERROR] /clock no disponible; abortando bootstrap
```
Cada relanzamiento del stack:
1. `ros_gz_bridge` arranca pero `/clock` topic no publica (sim_time stuck).
2. `controller_bootstrap` espera `/clock` con timeout corto → aborta.
3. Sin controllers cargados → no `/joint_states` → TF tree desconectado
   (`base_link` ↔ `rg2_tcp` not in same tree).
4. Orchestrator `INITIAL_SNAPSHOT` falla con
   `tf_lookup_exception:ConnectivityException`.

Esto es **el bug raíz aguas arriba** documentado en `BUG_BRIDGE_PATH_TOLERANCE.md`
sección "Update 2026-05-07 22:40 — script aislado confirma bug en capa MoveIt"
(`No clock received, using time argument instead!`).

**Conclusión F1.8**:
- ✅ Helper math + integration shipped (offline-correct).
- ❌ Validación live full-cycle requiere primero arreglar el sim_time clock sync.
- El bug raíz del clock impide cualquier ciclo live, no sólo F1.8.

**Plan F1.9** (futura sesión): cerrar el sim_time clock sync. Candidatos:
- Forzar `use_sim_time:=true` en TODO el stack (ya está pero algo no propaga).
- Subir timeout de `controller_bootstrap` (ahora ~10s, probar 60s).
- Activar `streaming` mode en `joint_trajectory_controller`.
- Verificar que `gz_ros2_control` plugin del SDF tiene `<sim_time_publisher>` activo.

## Update 2026-05-07 23:51 — cadena de bugs en orchestrator path confirmada

Tras pasar la pose real del objeto vía `object_pose_world_hint`, la APPROACH
ya planifica correctamente. Pero el ciclo completo revela una **cadena** de
bugs adicionales en el orchestrator path:

| # | Fase | Bug | Status | Workaround |
|---|---|---|---|---|
| A | HOME_INITIAL | `fjt_err:PATH_TOLERANCE_VIOLATED` cuando el robot no está exactamente en home (controller rechaza al primer instante de tracking). Default `position_tol_rad=0.10` es demasiado tight para drift de Gazebo+sim_time. | Abierto | Subir `home_position_tol_rad` a 1.0 vía launch arg (param dinámico vía `ros2 param set` NO se aplica al atributo Python del nodo — bug D). |
| B | APPROACH (execute) | `moveit_result_timeout:400.0s` cuando el bridge bug (`Action client not connected`) hace que `move_action` no devuelva resultado. El retry `f18b2c2` se dispara pero llega tarde (timeout exterior). | Abierto | Bajar `moveit_result_timeout_sec` y subir agresividad del retry, o resolver el bridge bug raíz (sim_time clock sync). |
| C | GRASP attach gate | `attach_distance:too_far:0.19-0.22m > 0.150m`. El TCP queda a 19-22 cm del centro del objeto (no 14 cm como en rondas previas). MoveIt converge a pose menos precisa con `vel=0.3+scaling=100`. | Abierto fix candidato: 0.150 → 0.250 m (commit en branch). |
| D | Param live `home_position_tol_rad` | `ros2 param set` actualiza el param pero no actualiza el atributo `self._home_position_tol_rad` del nodo — falta `add_on_set_parameters_callback`. | Abierto | Restart con launch arg, o añadir callback. |

**Resultado live (3 ciclos consecutivos en v4 con orchestrator rebuild + gate 0.250):**

| Ciclo | reason | duration | Bug que disparó |
|---|---|---|---|
| 1/3 | `approach:moveit_result_timeout:400.0s` | 409 s | Bug B (bridge → timeout exterior) |
| 2/3 | `home_initial:fjt_err:PATH_TOLERANCE_VIOLATED` | 4 s | Bug A (param D no se aplicó live) |
| 3/3 | `home_initial:fjt_err:PATH_TOLERANCE_VIOLATED` | 4 s | Bug A (idem) |

**Conclusión**: el camino orchestrator tiene **al menos 4 bugs concatenados**.
Cerrar "3 ciclos sin abort" requiere fix de los 4. Cada uno lleva entre
30 min y varias horas. **No es factible en una sola sesión**, y NO es
trabajo de F1.5 (que es sólo el bug bridge `CONTROL_FAILED` retry).

## Plan de fix (estimado 12-30 h, multi-sesión)

### Diagnóstico

1. Llamar manualmente `/orchestrator/compute_approach_pose` (si existe) o
   inspeccionar el log del orchestrator para ver qué pose intenta el
   APPROACH. Verificar si está en colisión, fuera del workspace, o en
   pose imposible.
2. Capturar el RobotState que MoveIt usa como start state — verificar
   que coincide con la pose física del robot en Gazebo.
3. Capturar el goal pose enviado a `/move_action` — comprobar que es
   alcanzable (workspace, joint limits) y libre de colisión.

### Fix candidatos (por orden de coste)

- **A**: forzar el orchestrator a usar `object_pose_world_hint` no-identity
  (cliente provee la pose conocida del SDF). Bypassa `ComputeApproachPose`.
- **B**: subir `allowed_planning_time` de 25 s → 60 s (default actual).
  Si OMPL falla por tiempo insuficiente (no por geometría), esto basta.
- **C**: revisar la collision matrix — añadir `box_red`/`mesa_pro` como
  pares deshabilitados de colisión durante APPROACH.
- **D**: reset explícito del robot a home antes de cada ciclo
  (`/joint_trajectory_controller` con home positions).
- **E**: cambiar planner de `RRTConnect` a `BiTRRT` o `LBKPIECE1` —
  `RRTConnect` puede tener problemas con poses cerca del límite del
  workspace.

### Validación

- Repetir reproducción mínima → 3/3 ciclos con `success=true`.
- Tag `cierre-orchestrator-approach-planning-20260507` o posterior.

## Referencias

- [BUG_BRIDGE_PATH_TOLERANCE.md](BUG_BRIDGE_PATH_TOLERANCE.md) — bug
  adyacente, retry en `plan_to_pose_server` cubre `CONTROL_FAILED` /
  `TIMED_OUT` (aplica una vez se resuelve este APPROACH FAILURE).
- [auditoria/audit_profesional_20260507.md](../auditoria/audit_profesional_20260507.md)
  §F1.5 — contexto del intento de validación live.
- Memoria `project_b_iter1_iter2_20260503` — `ComputeApproachPose` marcada
  como "F5-step3 pendiente" desde 2026-05-03; consistente con la
  hipótesis #1 ("hint no resuelto → placeholder o pose inválida").
