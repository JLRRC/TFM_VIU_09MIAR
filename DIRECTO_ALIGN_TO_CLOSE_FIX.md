# DIRECTO_ALIGN_TO_CLOSE_FIX

## Alcance aplicado
Cierre quirúrgico en la ruta Directo (`panel_pick_demo.py`) sobre la transición:
`GRASP_ALIGN_IK -> PRE_CLOSE -> CLOSE -> ATTACH_GATE`.

No se tocó MoveIt, ni TFM visual, ni otros frentes.

## Preguntas obligatorias (respuesta exacta)

1. ¿Después de `GRASP_ALIGN_IK` qué condición exacta decide pasar a `PRE_CLOSE`?
- En código, el paso efectivo ocurre cuando:
  - termina `GRASP_ALIGN_IK`,
  - se completa `extra_down` (si aplica),
  - se espera `post_align_settle_sec`,
  - existe `initial_obj_world` (directa o fallback),
  - y se ejecuta `_phase_begin("PRE_CLOSE", ...)`.
- Referencias:
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1374`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1461`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1478`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1527`

2. ¿Esa condición se cumple y no se ejecuta, o no se cumple nunca?
- Se cumplía de forma inestable: sí se entraba en `PRE_CLOSE`, pero tarde y con deriva fuerte tras align (espera fija larga), lo que desplazaba la geometría antes de cerrar.
- Evidencia previa al parche:
  - `RUN2`: `gate_check` en tcp_obj_dist=0.216 y entrada real a PRE_CLOSE en tcp_obj_dist=0.590.
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:84`
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:88`

3. ¿Existe un return temprano, timeout, guard o validación que corte la secuencia justo después de `GRASP_ALIGN_IK`?
- Sí, existía guard duro: si `initial_obj_world` era `None`, abortaba con `RuntimeError("demo_object_pose_unavailable_before_close")` antes de `PRE_CLOSE`.
- También puede bloquear por excepción en `extra_down`.
- Referencias:
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1450`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1499`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1507`

4. ¿La transición a `PRE_CLOSE/CLOSE` depende de qué?
- `PRE_CLOSE` depende de disponibilidad de pose objeto (`initial_obj_world`) y del flujo post-align.
- `CLOSE` depende de `pre_close_ok` (`_wait_pre_close_alignment`: XY/Z tolerancias y consecutivos).
- `ATTACH_GATE` depende de `close_confirmed` (`_wait_for_gripper_target`) y `close_metrics.ok`.
- Referencias:
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:653`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1589`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1616`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1653`
  - `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py:1675`

5. ¿El sistema entra en `PRE_CLOSE` pero no lo deja claro en logs, o realmente nunca entra?
- Entraba, pero no quedaba suficientemente trazado el motivo/condición de transición.
- Se instrumentó con logs explícitos `ENTER/EXIT` y `TRANSITION decision=...` para confirmar cada salto.
- Evidencia post-instrumentación:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:137`
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:148`
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:155`

6. ¿Qué cambio mínimo hace que el flujo pase correctamente?
- Cambio mínimo dominante:
  - eliminar espera rígida de `3.0s` post-align,
  - sustituir por settle corto configurable (`PANEL_PICK_DEMO_POST_ALIGN_SETTLE_SEC`, default `0.20s`),
  - y fallback de `initial_obj_world` cuando la lectura puntual no está disponible.
- Esto evita deriva grande entre `GRASP_ALIGN_IK` y `PRE_CLOSE` y elimina abortos innecesarios previos al cierre.

## Instrumentación añadida (permanente)
- `EXIT_PHASE GRASP_ALIGN_IK` (ya existente en flujo de fase).
- `ENTER_PHASE PRE_CLOSE`, `ENTER_PHASE CLOSE`, `ENTER_PHASE ATTACH_GATE`.
- Nueva traza estructurada `[PICK][DIRECT][TRANSITION]` con:
  - `tcp`, `obj`, `tcp_obj_dist`, `xy_dist`, `z_error`, `z_gap`, tolerancias,
  - estado de pinza (`closed_flag`, `measured_target_ok`, `opening_sum`, `max_abs_err`),
  - estado lógico attach,
  - `decision`, `reason`, `condition`.

## Causa raíz dominante
Espera post-align fija y demasiado larga (`3.0s`) entre `GRASP_ALIGN_IK` y `PRE_CLOSE`, que permitía deriva significativa de TCP/objeto antes del gate de cierre, provocando cierres fuera de ventana o comportamientos erráticos justo antes de cerrar.

## Parche mínimo aplicado
Ver diff en:
- `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_PARCHE.diff`

