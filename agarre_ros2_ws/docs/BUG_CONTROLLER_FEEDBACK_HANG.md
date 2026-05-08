# BUG — Controller feedback hang post-F1.18

**Estado**: ✅ **CERRADO 2026-05-08** vía F1.24 H9+H10+H11 (bypass MoveIt + joint normalization + multi-waypoint trajectory). T35 verde con 3 cycles consecutivos SUCCEEDED.

**Estado anterior**: ABIERTO — bug profundo de simulación (mitigación parcial F1.22 aplicada)
**Detectado**: 2026-05-08 14:17 (validación live de F1.18, HEAD `744006a`)
**Re-confirmado live**: 2026-05-08 18:13 + 18:19 (2 runs cycle 1, post-F1.22)
**Mitigación parcial**: F1.22 (TF pose-check post-FIRST_ATTEMPT_TIMEOUT)
**Última auditoría**: `auditoria/audit_profesional_20260508_v5.md`
**Bugs relacionados**:
- [BUG_BRIDGE_PATH_TOLERANCE.md](./BUG_BRIDGE_PATH_TOLERANCE.md) (síntoma adyacente)
- [BUG_FJT_GOAL_TIME_TOLERANCE.md](./BUG_FJT_GOAL_TIME_TOLERANCE.md) (cerrado en F1.11)

## Validación live confirmada (2026-05-08 sesión rutacrítica)

Reproducido 2/2 runs en cycle 1 APPROACH:
- Run 1 (HEAD post-F1.18, 18:13): FIRST_ATTEMPT_TIMEOUT 120s exacto.
  TCP llegó al target (panel_live_dist_m=0). Bug = solo feedback hang.
- Run 2 (HEAD post-F1.22, 18:19): FIRST_ATTEMPT_TIMEOUT 120s exacto.
  TCP NO llegó al target (dist=0.6060m base_link). Bug = feedback hang
  + trayectoria incompleta (cancel mid-execution puede dejar robot en
  estado intermedio).

Logs canónicos del bug:
```
[plan_to_pose_server] [PLAN_TO_POSE][MOVEIT_DIRECT] sending goal target=(0.440,0.003,0.125)
[move_group] simple_controller_manager.follow_joint_trajectory_controller_handle: Goal request accepted!
< 120 segundos sin más logs del controller >
[plan_to_pose_server] first attempt hang timeout=120.0s — cancelando goal
[move_group] move_group.move_group.move_action: MoveGroupMoveAction: Received request to cancel goal
[joint_trajectory_controller]: Got request to cancel goal
[joint_trajectory_controller]: Canceling active action goal because cancel callback received.
< NUNCA aparece "Goal reached, success!" del APPROACH >
```

Comparativa: HOME_INITIAL siempre funciona (Goal reached, success! a los 17s).
La diferencia es que HOME_INITIAL bypasea MoveIt y va directo al
controller via FollowJointTrajectory; APPROACH va por MoveIt → simple_controller_manager.

## F1.22 mitigación parcial (commit pendiente)

`plan_to_pose_server._execute_moveit_direct` añade TF check post-FIRST_ATTEMPT_TIMEOUT:
- Si robot está dentro de `moveit_position_tol * 5 = 0.25m` del target → success.
- Si no, cae al retry path original (sleep 20s + send_goal_async retry).

Resultado live:
- Cuando el robot SÍ llega al target (run 1) → recovery funcionaría.
- Cuando el robot NO llega (run 2) → retry sigue colgándose por el mismo bug.

## F1.23 LIVE intentado (2026-05-08 18:30) — controller restart NO resuelve

Implementación: cliente `/controller_manager/switch_controller` que
deactiva + activa `joint_trajectory_controller` ANTES del retry path
tras FIRST_ATTEMPT_TIMEOUT.

Resultado live:
```
[WARN] first attempt hang timeout=120.0s — cancelando goal
[WARN] reset joint_trajectory_controller antes del retry
[INFO] joint_trajectory_controller restarted OK   ← restart exitoso
[WARN] failed reason=FIRST_ATTEMPT_TIMEOUT:120.0s — intentando retry tras 20s
[INFO] move_group: Starting trajectory execution ...   ← retry attempt
[INFO] simple_controller_manager: Goal request accepted!
< NUNCA llega "Goal reached, success!" del retry attempt >
```

El **restart del controller funciona técnicamente** (el service responde OK),
pero el bug del feedback hang **persiste en el retry attempt**. Robot
medido en (0.029, -0.294, -0.246) base, target (0.433, 0.001, 0.125).
panel_live_dist_m=0.000 → robot **estático**, NO se mueve durante el retry.

Conclusión: el bug NO está en el controller en sí (restart no ayuda).
Está en el path **MoveIt simple_controller_manager → action goal handle**:
una vez que el primer goal queda en estado fantasma (cancelled pero el
goal_handle sigue activo en el bridge), los siguientes goals se aceptan
pero no se traducen en trajectory execution real.

## Hipótesis nuevas (post-F1.23)

| H | Hipótesis | Verificación |
|---|-----------|--------------|
| H6 | `simple_controller_manager` mantiene un goal_handle stale tras cancel mid-flight | restart de **move_group** (no del controller) entre attempts |
| H7 | gz_ros2_control plugin no procesa el segundo Trajectory msg porque el primero quedó "in flight" en su queue interna | inspect plugin source / añadir reset call al plugin |
| H8 | El cancel desde MoveIt llega después del cancel desde plan_to_pose, dejando el bridge en un estado donde el siguiente goal entra a una cola que ya no se procesa | flujo cancel mejorado: esperar confirm del cancel del move_group antes del restart |
| H9 | Bug específico del simulador (gz_ros2_control fork de ros2_control) — probaria con ros2_control real | requiere hardware real |

## Plan de fix (próxima sesión live)

1. **Restart del move_group node** (no solo del controller) tras
   FIRST_ATTEMPT_TIMEOUT. Usando `/move_group/get_state` + transition
   o lanzando un wrapper proceso. Más invasivo pero más probable.
2. **Aumentar `allowed_execution_duration_scaling`** a 5.0+ y reducir
   `allowed_goal_duration_margin` para forzar a MoveIt a abortar el
   goal completo si el feedback no llega en X segundos.
3. **Bypass MoveIt en APPROACH**: pre-computar trajectoria via
   `/move_group/compute_cartesian_path` y enviar directamente al
   FollowJointTrajectory action (igual que HOME_INITIAL hace).
4. **Investigar gz_ros2_control source** para identificar la queue
   interna que no se reinicia tras cancel.

## Síntoma

En cycles 2/3 del run E2E live `cycles 1/3` post-F1.18:

```
[PLAN_TO_POSE][MOVEIT_DIRECT] sending goal
target=(0.500,0.100,0.300) ee_frame=rg2_pinch_center group=manipulator
[move_group]: Starting trajectory execution ...
< 120s sin más logs >
[PLAN_TO_POSE][MOVEIT_DIRECT] first attempt hang
timeout=120.0s — cancelando goal y disparando retry
```

El `move_group` **acepta** el goal y arranca la ejecución, pero el feedback de
"trajectory completed" nunca llega a `plan_to_pose_server`. El goal acaba
cancelándose por `_MOVEIT_FIRST_ATTEMPT_TIMEOUT_SEC`.

## Lo que F1.18 SÍ resuelve

- Per-fase scaling (TRANSPORT 0.5, otras 0.25). Confirmado por test offline
  `test_classify_phase_*` (7 tests verde).
- Per-fase first_attempt_timeout (TRANSPORT 240s, otras 120s).
- Heurística pura testeable (`classify_phase_by_target_z`).

## Lo que F1.18 NO resuelve (controller recovery)

El feedback hang sigue ocurriendo en cycles 2/3 incluso con scaling=0.5 y
timeout=240s. El problema **no es** la duración de la trayectoria, sino la
ausencia de feedback del controller.

### Hipótesis (no validadas live)

| H | Hipótesis | Cómo verificar | Coste fix |
|---|-----------|----------------|-----------|
| H1 | Race en `controller_manager` post-attach (lock interno) | Inspeccionar logs de `controller_manager` en cycle 2 | Medio |
| H2 | `joint_trajectory_controller` deja de aceptar goals tras attach físico | `ros2 control list_controllers` post-cycle 1 | Medio |
| H3 | `gz_ros2_control` plugin cuelga en `update()` post-detach | `gz topic -e /clock` durante el hang | Alto |
| H4 | `move_group` cliente cachea handle stale del controller | restart de `move_group` entre cycles | Bajo |
| H5 | Zombies de `move_group` (3 instancias detectadas en F1.16) | `pgrep -c move_group` en CI | Bajo |

**H5 ya tiene fix parcial**: `cleanup_zombies.sh` ampliado (commit F1.16).

## Mitigaciones offline implementadas

1. **F1.18 `classify_phase_by_target_z`** — función pura con tests offline.
2. **first_attempt_timeout corto** (120-240s) detecta el hang sin
   bloquear al orchestrator hasta `_moveit_result_timeout` completo (400s).
3. **Retry con sleep 20s** post-cancel para dar tiempo al controller a
   recuperarse (F1.12).

## Plan de fix (próxima sesión LIVE)

1. Lanzar stack vivo + ejecutar 1 cycle E2E completo.
2. Inmediatamente despues del cycle 1 (antes de cycle 2): inspeccionar
   `ros2 control list_controllers --type` y `pgrep -c move_group`.
3. Si H5: confirmar que `cleanup_zombies.sh` se llama entre cycles del E2E
   driver (verificar `c3d4372 feat(F1.13): canonical e2e cycles driver`).
4. Si H4: añadir `move_group` restart entre cycles del driver.
5. Si H1/H2: añadir `/controller_manager/switch_controller` deactivate +
   activate de `joint_trajectory_controller` en el retry path de
   `plan_to_pose_server` cuando `reason ∈ {FIRST_ATTEMPT_TIMEOUT,
   CONTROL_FAILED}` y attempts > 1.
6. Si H3: bug de gz_ros2_control upstream — abrir issue.

## Tests guardrail (offline-friendly)

- ✅ `test_classify_phase_*` — 7 tests sobre la heurística (PASS HEAD post-F1.18)
- ✅ `test_build_goal_default_scaling_is_0_25` — default invariante (PASS)
- ✅ `test_build_goal_transport_scaling_0_5` — TRANSPORT scaling (PASS)
- 🔴 T31 contract test `path_tolerance MoveIt↔controller` — **PENDIENTE**
- 🔴 T33 guardrail "0 zombies move_group" — **PENDIENTE**
- 🔴 T35 3 cycles E2E consecutivos — **PENDIENTE LIVE**

## Referencias

- Commit F1.18: `744006a fix(F1.18): per-phase scaling/timeout — TRANSPORT detectado por Z<0.05`
- Commit F1.16: `cleanup_zombies.sh ampliado` (memoria 2026-05-08)
- Commit F1.13: `c3d4372 feat(F1.13): canonical e2e cycles driver con hard reset HOME`
- Auditoría v5: `auditoria/audit_profesional_20260508_v5.md` (a generar)

## F1.24 / H9 LIVE (2026-05-08): bypass MoveIt vía FJT directo — ÉXITO PARCIAL

Implementación: `bypass_moveit_for_short_paths=true` en plan_to_pose_server.
Flujo:
1. Lee `/joint_states` (seed IK).
2. Llama `/compute_ik` (síncrono, NO usa simple_controller_manager).
3. Construye JointTrajectory de 2 puntos (current → target).
4. Envía a `/joint_trajectory_controller/follow_joint_trajectory` directo.
5. Espera "Goal reached, success!" del controller.

Helper puro: `ur5_tools/fjt_direct_helpers.py` (16 tests offline + mypy strict).

**Resultado live (2026-05-08 19:24-19:27 cycle 1)**:

| Fase | Resultado FJT_DIRECT | Tiempo |
|------|---------------------|--------|
| INITIAL_SNAPSHOT | N/A (no usa plan_to_pose) | 50ms |
| HOME_INITIAL | ✅ FJT directo (no usa MoveIt) | 17s |
| SELECT_OBJECT | ✅ Internal | 1ms |
| **APPROACH** | **✅ fjt_direct:SUCCESSFUL** | **27s** |
| **GRASP_DOWN** | **✅ fjt_direct:SUCCESSFUL** | **26s** |
| GRASP | ✅ Internal (attach service) | 1s |
| **LIFT** | **✅ fjt_direct:SUCCESSFUL** | **33s** |
| **TRANSPORT** | **❌ FJT result timeout 90s** | (timeout) |

**Conclusión**: `BUG_CONTROLLER_FEEDBACK_HANG` se mitiga
**completamente para movimientos cortos/medios** (< 0.5m): el path FJT
directo bypasea simple_controller_manager y el controller responde
"Goal reached, success!" como en HOME_INITIAL. **APPROACH+GRASP_DOWN+LIFT
funcionan perfectamente** — primera vez en el historial del proyecto que
estas 3 fases del orchestrator pasan live sin tocar el legacy.

**TRANSPORT abierto**: el IK calcula joints con wraps angulares fuera
de los límites UR5 (e.g. `-3.387, +6.202, -1.699, +0.210, +4.712, -4.957`).
El robot intenta moverse y se queda parado por límites físicos. Solución
pendiente: post-procesamiento de IK que normalice joints a [-π, π]
módulo 2π, o usar seed más cuidadoso.

### Hipótesis nuevas (post-F1.24)

| H | Idea | Coste | Estado |
|---|------|-------|--------|
| H10 | Normalizar joints IK al rango [-π, π] antes del FJT | Bajo | ✅ APLICADO commit `57f29ad` |
| H11 | Trayectoria multi-waypoint para distancias largas (TRANSPORT) | Medio | ✅ APLICADO commit `<HEAD>` |
| H12 | Detectar TRANSPORT por distancia y usar duration ≥25s | Bajo | ✅ APLICADO (parte de H11) |
| H13 | Usar Trac-IK con joint limits estrictos en lugar de KDL default | Medio | No necesario (H10 suficiente) |

## ✅ CIERRE 2026-05-08 (T35 VERDE — 3 cycles consecutivos SUCCEEDED)

Validación live final del proyecto (orchestrator default, panel sin tocar
legacy borrado en F5-legacy-removed):

```
Cycle 1: SUCCEEDED  duration=232.4s  reason=ok  fases=7/7
Cycle 2: SUCCEEDED  duration=204.2s  reason=ok  fases=7/7
Cycle 3: SUCCEEDED  duration=206.8s  reason=ok  fases=7/7
```

Logs canónicos del cierre:
```
[ORCHESTRATOR_LC] phase_loop _execute_phase returned phase=APPROACH ok=True reason=approach:fjt_direct:SUCCESSFUL
[ORCHESTRATOR_LC] phase_loop _execute_phase returned phase=GRASP_DOWN ok=True reason=grasp_down:fjt_direct:SUCCESSFUL
[ORCHESTRATOR_LC] phase_loop _execute_phase returned phase=GRASP ok=True reason=grasp_attach:attach_dispatched
[ORCHESTRATOR_LC] phase_loop _execute_phase returned phase=LIFT ok=True reason=lift:fjt_direct:SUCCESSFUL
[ORCHESTRATOR_LC] phase_loop _execute_phase returned phase=TRANSPORT ok=True reason=transport:fjt_direct:SUCCESSFUL
[ORCHESTRATOR_LC] phase_loop _execute_phase returned phase=RELEASE ok=True reason=release_open:target_rad=0.0425
```

**Conclusión**: el bug estaba en el path MoveIt → simple_controller_manager.
La solución NO fue arreglar ese bug (es upstream gz_ros2_control / MoveIt).
La solución FUE bypassar ese path entero usando FJT directo (mismo path
que HOME_INITIAL siempre usó). Combinado con joint normalization (H10)
y multi-waypoint trajectory (H11) para distancias largas, el orchestrator
completa 7/7 fases consistentemente.

Tag de cierre: `T35-3-cycles-verde-20260508`.
Tag rollback (legacy aún recuperable): `audit-pre-borrar-legacy-20260508`.
