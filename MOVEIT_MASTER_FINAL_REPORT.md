# MOVEIT_MASTER_FINAL_REPORT

## Alcance de este cierre
- Ruta `Agarre Objeto (MoveIt)`.
- Ruta `Ejecutar Agarre` / canónica `TFM -> MoveIt`.
- Sin cambios en Directo, sin cambios en MoveIt core/arquitectura.

## Cambios mínimos aplicados
1. `panel_pick_object.py`
- `APPROACH` deja de usar tolerancia fija `0.04` y pasa a `PANEL_PICK_OBJECT_APPROACH_TOL_M` (default `0.10`).
- Objetivo: alinear gate TF del panel con el criterio de llegada por EE del bridge.

2. `start_panel_v2.sh`
- Export explícito de `PANEL_PICK_OBJECT_APPROACH_TOL_M` (default `0.10`).
- Objetivo: trazabilidad y control del cierre de fase `APPROACH`.

3. `ur5_moveit_bridge.py`
- En `APPROACH`, el timeout de request se eleva al presupuesto de replan (`max_total_timeout * (attempts + 1) + cushion`; default efectivo `380s`).
- Objetivo: eliminar timeout prematuro a nivel request mientras FJT sigue ejecutando/replanificando.

## Evidencia real usada
- Corridas MoveIt `pick_object`: R4/R4B/R5/R6/R7 (registro en `MOVEIT_MASTER_TRACE.log` + `moveit_bridge.log*.bak`).
- Corrida canónica `tfm_execute`: `MVT_TFM_R1`.
- Verificación de compilación final de paquetes tocados: `ur5_qt_panel`, `ur5_tools` (OK).

## Qué funciona ya en MoveIt
- `APPROACH` puede cerrar con resultado útil y avanzar a `PRE_GRASP` (evidencia R7).
- El timeout prematuro de request en `APPROACH` quedó mitigado por presupuesto de timeout (evidencia R6, uplift a `380s`).
- Pipeline request/result por `request_id` y `request_uuid` mantiene correlación correcta en las corridas analizadas.

## Qué sigue con limitaciones
- `APPROACH` no es 100% repetible: persisten corridas con timeout terminal de FJT (`exec_failed_fjt_direct:fjt_result_timeout`) según estado inicial.

## Qué sigue bloqueado
1. `PRE_GRASP`
- Caso límite recurrente `exec_succeeded_but_tf_mismatch` en panel (ejemplo real `dist=0.101`, `tol=0.100` en R7).

2. `GRASP_DOWN`, `LIFT`, `TRANSPORT`, `RELEASE`
- Bloqueados por dependencia de `PRE_GRASP`.

3. Ruta canónica `TFM -> MoveIt` (`/panel/tfm_execute`)
- Bloqueada antes de dispatch por gate `controladores no verificados` (`MVT_TFM_R1`).

## Ruta principal recomendada ahora
- Para evidencia técnica del TFM en este estado: usar `Agarre Objeto (MoveIt)` como ruta principal de validación incremental por fases.
- La ruta canónica `TFM -> MoveIt` debe reabrirse cuando se desbloquee explícitamente el gate de controladores.

## Evidencia útil para TFM
- `MOVEIT_MASTER_TRACE.log`: cronología de corridas, `request_id`, etapas, fallos y éxitos.
- `MOVEIT_MASTER_PATCHES.diff`: parche mínimo aplicado en este frente.
- `MOVEIT_MASTER_MATRIX.csv`: estado por fase/ruta con bloqueo dominante y siguiente paso.
- `MOVEIT_MASTER_EXECUTION.md`: causa raíz dominante por fase y validación.

## Estado global actual MoveIt
- Nivel: **APTO CON LIMITACIONES**.
- Motivo: fase `APPROACH` funcional pero no robusta/repetible al 100%; `PRE_GRASP` aún bloquea progreso profundo; canónica bloqueada por gate previo.
