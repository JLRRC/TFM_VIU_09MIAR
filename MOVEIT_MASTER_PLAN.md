# MOVEIT_MASTER_PLAN

## Fase 1 — APPROACH
- Estado: VALIDADA CON LIMITACIONES
- Cuello dominante: timeout real de ejecucion FJT en algunos arranques (ya no timeout prematuro de request).
- Criterio de cierre: `APPROACH` terminal util y repetible (>=2 corridas seguidas sin fallo).
- Siguiente paso: estabilizar ejecucion FJT en APPROACH (no request/result), sin abrir fases profundas.

## Fase 2 — PRE_GRASP
- Estado: BLOQUEADA
- Cuello dominante: `exec_succeeded_but_tf_mismatch` en chequeo TF panel (caso limite 0.101 vs 0.100).
- Criterio de cierre: `PREGRASP_REACHED` repetible sin mismatch TF.
- Siguiente paso: ajuste minimo de criterio TF/recovery en PRE_GRASP tras estabilizar Fase 1.

## Fase 3 — GRASP_DOWN / MICRODESCENSOS
- Estado: BLOQUEADA
- Cuello dominante: dependencia de Fase 2.
- Criterio de cierre: cierre util de `GRASP_DOWN` con salida terminal consistente.
- Siguiente paso: abrir solo tras cierre de PRE_GRASP.

## Fase 4 — LIFT
- Estado: BLOQUEADA
- Cuello dominante: dependencia de Fase 3.
- Criterio de cierre: `LIFT_SUCCEEDED` con coherencia TCP-objeto.
- Siguiente paso: abrir solo tras cierre de GRASP_DOWN.

## Fase 5 — TRANSPORT / RELEASE
- Estado: BLOQUEADA
- Cuello dominante: dependencia de Fase 4.
- Criterio de cierre: `TRANSPORT_SUCCEEDED` + `RELEASE_SUCCEEDED`.
- Siguiente paso: abrir solo tras cierre de LIFT.

## Fase 6 — REPETIBILIDAD
- Estado: BLOQUEADA
- Cuello dominante: Fase 1 y Fase 2 no cerradas.
- Criterio de cierre: mini bateria separada y estable para `pick_object` y `tfm_execute`.
- Siguiente paso: ejecutar bateria una vez cerradas fases 1-5.
