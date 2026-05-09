# DIRECTO - estado estable sin slice de cierre helper (2026-04-24)

## Alcance

Esta nota fija el estado estable alcanzado en DIRECTO sin seguir empujando el cierre del helper.
No se reabre geometria terminal, no se toca model.sdf en la cadena principal del brazo y no se cambia la politica final de cierre del offscreen helper.

## Cambio estable conservado

El unico ajuste nuevo que se conserva en esta iteracion esta en panel_pick_demo.py:

- _run_joint_step registra de donde vino la aceptacion final del paso:
  - local_joint_state
  - joint_wait
  - joint_wait_retry
  - runtime_target
- _evaluate_transport_stage_postcheck usa esa senal para no tumbar un recovery de cesta cuando:
  - el label es CESTA_STAGE_*_RECOVER_*
  - runtime_target_ok=true
  - la aceptacion del paso vino por runtime_target
  - el model_target_err queda alto por desfase entre joint-state local y TCP live

Motivo:

- En recoveries de transporte a cesta el criterio operativo relevante es cartesiano.
- Se observaron casos donde runtime_target_dist ya estaba dentro de tolerancia, pero model_target_err seguia alto y provocaba un falso negativo local del postcheck.

## Cambios previos del mismo bloque que siguen vigentes

El estado actual de panel_pick_demo.py mantiene tambien los fixes locales ya validados en esta sesion:

- execution_rot live_joints acotado a APPROACH_COARSE_REFINE
- IK alineada con la misma orientacion usada para proyectar el offset
- phase_end de APPROACH_COARSE reanclado tras coarse refine aceptado
- handoff de GRASP_DOWN reanclado al XY refinado validado
- TRANSPORT_PREP no fuerza prep_replan_exc cuando ya no quedan reintentos

## Validacion de referencia

La corrida de referencia para este estado es:

- auditoria/directo_validation_fix_20260424_10

Self-check de arranque en esa misma corrida:

- state = READY
- geometry_ok = true
- moveit_ready = true
- rg2_tcp esperado/actual = (0.0, 0.0, 0.175)
- rg2_pinch_center esperado/actual = (0.0, 0.0, 0.175)
- geometry_pair_error_m = 0.0

Resultado funcional:

- approach = correcto
- acquisition = correcto
- physical = correcto
- carry = correcto
- release = correcto
- operational = correcto
- pipeline_global = correcto

Metrica destacada:

- release_destination_error = 0.0048873325311193875 m

## Limite conocido que se deja fuera

En la misma corrida de referencia el helper todavia cierra con rc=1 por un timeout/resolucion de recovery de cesta, aunque el benchmark funcional ya da pipeline_global correcto.

Se probaron dos lineas adicionales y ambas se descartaron por no dar una mejora robusta:

- relajar tolerancias de recovery de cesta
- diferir el resultado final del helper a una comprobacion tardia de cesta

Ambas variantes fueron revertidas. El arbol queda en el estado minimo que si tuvo validacion funcional fuerte.

## Conclusion operativa

El problema geometrico y de handoff operativo queda resuelto en el estado actual.
Lo unico pendiente fuera de esta nota es la politica de cierre del helper frente a recoveries de cesta, que no se persigue en esta iteracion.