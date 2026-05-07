# BUG: orchestrator path APPROACH — OMPL planning FAILURE

**Estado**: 🔴 ABIERTO (detectado 2026-05-07 22:57 en validación live F1.5).
**Reproducibilidad**: 100% en el camino `/pick_place` con `object_name='box_red'`.
**Bloqueante para**: cierre F1.5 al 100% con criterio "3 ciclos sin abort".
**NO bloqueante para**: camino legacy `run_pick_demo` (que tiene su propia
ruta de cálculo de approach pose con tolerancias y retries históricos).

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

## Plan de fix (estimado 4-12 h, futura sesión)

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
