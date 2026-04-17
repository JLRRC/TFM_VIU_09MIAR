# Evidencias Test 19 — Ciclo Pick-and-Place MoveIt COMPLETO

**Fecha**: 2026-04-17  
**Resultado**: ✅ SECUENCIA COMPLETADA EXITOSAMENTE  
**Log fuente**: `historico/moveit_grasp_test19_20260417.log`

---

## Secuencia ejecutada

| Fase | Inicio | Fin | Wall-s | dist | tol | Estado |
|---|---|---|---|---|---|---|
| MESA_GLOBAL (joint) | 09:18:44 | 09:18:48 | 4s | — | — | ✅ |
| HOME_START (joint) | 09:18:48 | 09:19:02 | 14s | 0.0016 rad | 0.08 rad | ✅ |
| PICK_IMAGE + gripper open | 09:19:02 | 09:19:06 | 4s | — | — | ✅ |
| APPROACH (MoveIt) | 09:19:06 | 09:19:47 | 41s | 0.096m | 0.100m | ✅ |
| PRE_GRASP (MoveIt) | 09:19:48 | 09:19:54 | 6s | 0.057m | 0.100m | ✅ |
| GRASP_DOWN (MoveIt) | 09:19:55 | 09:20:41 | 46s | 0.040m | 0.040m | ✅ |
| Gripper close + attach | 09:20:43 | 09:20:43 | <1s | 0.039m dist TCP-obj | 0.060m | ✅ |
| LIFT (MoveIt) | 09:20:43 | 09:21:48 | 65s | 0.029m | 0.030m | ✅ |
| MESA_WITH_OBJECT (joint) | 09:21:48 | 09:22:06 | 18s | — | — | ✅ |
| TRANSPORT_STAGE_1 (MoveIt) | 09:22:06 | 09:23:00 | 54s | 0.025m | 0.030m | ✅ |
| TRANSPORT_STAGE_2 (MoveIt) | 09:23:01 | 09:24:17 | 76s | 0.020m | 0.030m | ✅ |
| TRANSPORT_STAGE_3 (MoveIt) | 09:24:18 | 09:24:50 | 32s | 0.026m | 0.030m | ✅ |
| DROP (MoveIt) | 09:24:50 | 09:25:47 | 57s | 0.027m | 0.030m | ✅ |
| RELEASE (gripper open + detach) | 09:25:47 | 09:25:47 | <1s | — | — | ✅ |
| HOME_FINAL (joint) | 09:25:49 | 09:25:56 | 7s | 0.0000 rad | 0.020 rad | ✅ |

**Tiempo total**: 09:18:44 → 09:25:56 = **432s wall** (~7m12s)

---

## Evidencias clave

### Attach físico confirmado
```
[ATTACH_BACKEND] gazebo_attach_applied=true object=pick_demo method=follow_tcp detail=gz_service_ok
```
Confirmado continuamente desde GRASP_DOWN hasta RELEASE.

### TRANSPORT sin wrong IK branch (fix SKIP_CONSTRAINTS=0)
```
[PICK_OBJ][STEP] label=TRANSPORT_STAGE_1 request_uuid=117fc8fec68a4f96a6ecce7e74219610
[PICK_OBJ][STEP] label=TRANSPORT_STAGE_2 request_uuid=1337f7faf0a745569ca4edce7ac62f81
[PICK_OBJ][STEP] label=TRANSPORT_STAGE_3 request_uuid=7558effac8494d78ab04368cd0e67f19
```
UUIDs sin prefijo `skip_constraints:` → path constraints activos → OMPL confinado al branch IK correcto.

### RELEASE confirmado
```
[OBJECTS] state pick_demo -> RELEASED owner=NONE attached=false reason=pick_object
[PICK_OBJ] POST-CHECK: pick_demo estado = RELEASED ✓
```

### Cierre de secuencia
```
[PICK_OBJ] === SECUENCIA COMPLETADA EXITOSAMENTE ===
```

---

## Fixes activos en este test

| Fix | Variable | Valor |
|---|---|---|
| Modo bridge MoveIt | `PANEL_MOVEIT_MODE` | `moveit_py` |
| Constraints wrist_1/2 en APPROACH | `_APPROACH_PATH_CONSTRAINT_JOINTS` | código |
| Constraints shoulder/elbow/wrist_1/2 resto | `_PATH_CONSTRAINT_JOINTS` | código |
| POST_LIFT gate ampliado | `POST_LIFT_MAX_DIST_M` | 0.30m |
| FJT goal_time_tol | `CONTROLLER_GOAL_TIME_TOL_SEC` | 300s |
| Velocity/accel scale | `VELOCITY_SCALE` | 0.30 |
| TRANSPORT con constraints | `TRANSPORT_SKIP_CONSTRAINTS` | 0 |
| TRANSPORT timeout ampliado | `TRANSPORT_MOVEIT_WAIT_SEC` | 600s |
| Carry gate deshabilitado | `CARRY_GATE_ENABLE` | 0 |

---

## Geometría verificada (runtime)

- **Objeto pick_demo** en world: (-0.420, 0.000, 0.875) → base_link: (0.430, 0.000, 0.025)
- **APPROACH target** base_link: (0.430, 0.000, 0.350)
- **GRASP target** base_link: (0.430, 0.000, 0.030)
- **LIFT target** base_link: (0.430, 0.000, 0.350)
- **TRANSPORT_STAGE_1** base_link: (0.137, 0.000, 0.350)
- **TRANSPORT_STAGE_2** base_link: (-0.157, 0.000, 0.350)
- **TRANSPORT_STAGE_3** base_link: (-0.450, 0.000, 0.350) ← posición cesta
- **DROP target** base_link: (-0.450, 0.000, 0.020)
