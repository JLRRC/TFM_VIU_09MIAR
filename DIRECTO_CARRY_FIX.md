# DIRECTO_CARRY_FIX

## Alcance
Cierre quirúrgico exclusivo del tramo Directo:
- `ATTACH_GATE`
- `LIFT`
- `CARRY`

Archivo tocado:
- `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`

No se tocó MoveIt ni fases previas ya validadas.

## Respuestas obligatorias

1. ¿El attach se está ejecutando realmente o solo lógicamente?
- Se ejecuta realmente por backend (publicación de attach/follow), no solo lógico.
- Evidencia de flujo físico: `attach_result=true` + `attach_follow_result=ok`.
- Referencias de código:
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1803`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1847`

2. ¿El objeto queda realmente unido al TCP o solo se marca como agarrado?
- Primero queda unido físicamente (backend attach/follow). El marcado lógico (`mark_object_attached`) se hace después de validar `CARRY`.
- Referencias:
  - attach físico en `ATTACH_GATE`: `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1803`
  - marcado lógico posterior: `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1981`

3. ¿Qué condición exacta dispara `demo_carry_validation_failed`?
- En `_validate_demo_carry`, si dentro de `timeout_sec` no se obtienen muestras consecutivas con:
  - `obj_move >= min_obj_move_m`
  - `lift_delta >= min_lift_delta_m`
  - `tcp_dist <= max_tcp_dist_m`
- entonces se lanza `RuntimeError("demo_carry_validation_failed ...")`.
- Referencia:
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1206`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1304`

4. ¿El TCP sube pero el objeto no acompaña?
- En la corrida fallida base, no hubo subida útil sostenida: `tcp_lift_delta=0.003` y `obj_lift_delta=0.024` (por debajo de `0.025`), luego cayó.
- Evidencia:
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:11`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:16`

5. ¿El objeto acompaña parcialmente pero fuera de tolerancia?
- Sí, parcial y transitorio en la corrida fallida (`obj_move` sí, `lift_delta` no sostenido; luego `tcp_dist` también rompe tolerancia).
- Evidencia:
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:16`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:20`

6. ¿La validación está demasiado estricta o el carry físico falla de verdad?
- El fallo dominante observado es físico/sincronización temporal post-attach: el carry no consolida elevación sostenida antes de validar.
- La validación detecta correctamente ese estado (no es falso positivo puro).

7. ¿Dónde está el problema dominante?
- En `hold post-attach` / sincronización objeto-TCP antes de `LIFT`.
- `LIFT` arrancaba inmediatamente tras `attach_follow_result=ok`, sin margen para consolidar el lock de seguimiento del backend.

8. ¿Qué cambio mínimo hace que pase?
- Añadir espera corta post-attach, configurable, antes de `LIFT`:
  - `PANEL_PICK_DEMO_POST_ATTACH_HOLD_SEC` (default `0.90s`).
- Referencia:
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1855`

## Instrumentación permanente añadida

- `ATTACH_GATE`:
  - `attach_result`, `attach_follow_result`, `post_attach_hold`
  - muestra TCP/objeto, `tcp_obj_dist`, `z_gap`, condición de follow.
- `LIFT`:
  - `tcp_before/after`, `obj_world_before/after`, `obj_base_after`, `tcp_lift_delta`, `obj_lift_delta`, `tcp_obj_dist_after`, `expected_z_gap`.
- `CARRY`:
  - muestreo continuo con `cond_obj_move`, `cond_lift`, `cond_tcp`
  - error final con `fail_reasons` exactos.

## Parche aplicado
Ver:
- `/home/laboratorio/TFM/DIRECTO_CARRY_PARCHE.diff`

