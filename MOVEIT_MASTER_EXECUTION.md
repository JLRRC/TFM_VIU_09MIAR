# MOVEIT_MASTER_EXECUTION

## Registro de sesion
- Fecha: 2026-03-23
- Alcance: `Agarre Objeto (MoveIt)`, `Ejecutar Agarre`, ruta `TFM -> MoveIt`.
- Exclusiones respetadas: Directo, MoveIt config estructural, UI cosmetica.

## Cambios aplicados (minimos)
1. `panel_pick_object.py`
- `APPROACH` ahora usa `PANEL_PICK_OBJECT_APPROACH_TOL_M` (default `0.10`) en vez de `0.04` fijo.
- Objetivo: eliminar falso negativo `exec_succeeded_but_tf_mismatch` cuando bridge ya acepta llegada por EE (`tol=0.10`).

2. `start_panel_v2.sh`
- Nueva export: `PANEL_PICK_OBJECT_APPROACH_TOL_M=${...:-0.10}`.
- Objetivo: control explicito y trazable del gate TF de `APPROACH`.

3. `ur5_moveit_bridge.py`
- En `_dispatch_plan_request_with_timeout`, para `phase=APPROACH` se eleva timeout de request al presupuesto de replan:
  - `approach_budget = APPROACH_MAX_TOTAL_TIMEOUT * (REPLAN_MAX_ATTEMPTS + 1) + cushion`
  - default efectivo: `120 * 3 + 20 = 380s`.
- Objetivo: evitar `execute_timeout` prematuro del request mientras FJT sigue en replan/ejecucion.

## Fase 1 — APPROACH
### Bloqueo dominante observado
- Antes del ultimo parche: timeout de request (`execute_timeout`) durante replans de APPROACH.
- Despues del ultimo parche: desaparece ese timeout prematuro; persiste inestabilidad por timeout FJT real en algunos arranques.

### Evidencia clave
- `R4B` (pre-parche de request timeout): fallo en `execute_timeout: timeout_sec=286.0`.
  - Fuente: `moveit_bridge.log.20260323_220255.bak` (lineas 139-145)
- `R5` (pre-parche de request timeout): mismo patron `execute_timeout: timeout_sec=296.2`.
  - Fuente: `moveit_bridge.log` previo (lineas 141-147 en captura)
- `R6` (post-parche request timeout):
  - bridge eleva timeout: `from=292.5s to=380.0s`.
  - ya no hay `execute_timeout` de request.
  - termina por `exec_failed_fjt_direct:fjt_result_timeout` tras consumir replans.
  - Fuente: `moveit_bridge.log.20260323_222105.bak` (lineas 62, 142-147)
- `R7` (post-parche request timeout): APPROACH cierra con exito y avanza a PRE_GRASP.
  - Fuente: `/tmp/moveit_pick_object_MVT_F1_FIX_R7.log` (lineas 549-555)

### Causa raiz dominante (actual)
- Inestabilidad de ejecucion FJT en APPROACH: segun estado inicial, algunas corridas alcanzan objetivo y otras agotan timeout de ejecucion por intento.
- El cuello ya no esta en request/result gating; esta en ejecucion/seguimiento del controlador para APPROACH.

### Estado de fase
- `VALIDADA CON LIMITACIONES`.

## Fase 2 — PRE_GRASP
### Bloqueo dominante observado
- `exec_succeeded_but_tf_mismatch` en gate TF del panel, muy cerca del umbral.

### Evidencia clave
- `R7`: PRE_GRASP recibe `success=true` del bridge y luego cae por TF panel:
  - dist final `0.101` vs tol `0.100`.
  - Fuente: `/tmp/moveit_pick_object_MVT_F1_FIX_R7.log` (lineas 589-617)
- `R4` (historico de sesion): tambien avanzo a PRE_GRASP y cayo por mismatch TF.
  - Bridge confirma request_id=2 exitoso en PRE_GRASP.
  - Fuente: `moveit_bridge.log.20260323_215503.bak` (lineas 157-162)

### Causa raiz dominante
- Criterio TF de PRE_GRASP demasiado estricto en borde frente a ejecucion real (caso limite recurrente cerca del umbral).

### Estado de fase
- `BLOQUEADA`.

## Fase 3 — GRASP_DOWN / MICRODESCENSOS
- No se alcanza por bloqueo previo en PRE_GRASP.
- Estado: `BLOQUEADA` (dependencia).

## Fase 4 — LIFT
- No evaluable por dependencia de Fase 3.
- Estado: `BLOQUEADA`.

## Fase 5 — TRANSPORT / RELEASE
- No evaluable por dependencia de Fase 4.
- Estado: `BLOQUEADA`.

## Fase 6 — REPETIBILIDAD
- No cerrable mientras Fase 1 y Fase 2 no sean estables.
- Estado: `BLOQUEADA`.

## Ruta canónica `TFM -> MoveIt` (`/panel/tfm_execute`)
### Corrida real
- `MVT_TFM_R1`: servicio responde `success=false` con mensaje `controladores no verificados`.
- Fuente:
  - `/tmp/MVT_TFM_R1_tfm.out`
  - `ros2_launch.log` lineas 355-357 (`[TFM][REMOTE] ...`, `ERROR ... controladores no verificados`, `EXEC_ACK success=false`)

### Estado
- Bloqueada antes de despachar al bridge MoveIt (gate de controladores).

## Cierre de sesion
- Build final: `ur5_qt_panel` y `ur5_tools` compilados correctamente (ROS jazzy).
- Disponibilidad runtime en cierre: sin servicios `/panel/*` activos; no se ejecutan corridas adicionales en este cierre.
- Se mantiene como evidencia real vigente: corridas R4/R4B/R5/R6/R7 y `MVT_TFM_R1`.
