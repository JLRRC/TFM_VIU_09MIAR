# Cierre tecnico del incidente TCP RG2 en UR5

Fecha de cierre: 2026-04-22
Estado: cerrado y validado
Sistema: UR5 + RG2 + ROS 2 Jazzy + Gazebo Sim + MoveIt 2

## Resumen ejecutivo

La causa raiz real del incidente fue un TCP mal definido en la geometria del efector final. Los frames `rg2_tcp` y `rg2_pinch_center` estaban mal posicionados respecto a `tool0`, lo que desplazaba la referencia de ejecucion del agarre.

El offset incorrecto `-0.0877005` quedo sustituido por el valor canonico `+0.0050885`. La evidencia valida fue visual y no de logs aislados.

No fueron causa raiz:

- salto de rama IK;
- perdida de XY en `GRASP_DOWN`.

## Fix canonico congelado

La geometria canonica queda congelada en:

- `tool0 -> rg2_tcp = (0, 0, 0.0050885)`
- `tool0 -> rg2_pinch_center = (0, 0, 0.0050885)`

La fuente numerica editable queda en:

- `src/ur5_description/urdf/ur5.urdf.xacro`

La lectura runtime y el consumo programatico quedan centralizados en:

- `src/ur5_tools/ur5_tools/gripper_geometry.py`

Queda explicitamente descartado volver a tocar este TCP para ajustar comportamiento de planificacion, grasp, attach o transporte sin una nueva validacion visual dedicada.

## Implementacion aplicada

1. Se elimino el uso operativo de offsets legacy o duplicados en panel, DIRECTO y fallbacks FK.
2. Se promovio el self-check geometrico a gate de arranque con publicacion detallada en `/system_diag`.
3. Se endurecio MoveIt y el bridge sin cambiar `ee_frame` ni la geometria del TCP.
4. Se alineo el harness de validacion DIRECTO con el launcher canonico `./lanzar_panelc2.sh`, incluyendo el mismo margen de `home_with_object`.

## Archivos clave afectados

- `src/ur5_tools/ur5_tools/gripper_geometry.py`
- `src/ur5_tools/ur5_tools/system_state_manager.py`
- `src/ur5_tools/ur5_tools/ur5_moveit_bridge.py`
- `src/ur5_bringup/config/system_state_manager.yaml`
- `src/ur5_bringup/launch/ur5_stack.launch.py`
- `src/ur5_qt_panel/ur5_qt_panel/panel_settings.py`
- `src/ur5_qt_panel/ur5_qt_panel/panel_v2.py`
- `src/ur5_qt_panel/ur5_qt_panel/panel_direct2.py`
- `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`
- `scripts/capture_system_diag.py`
- `scripts/run_directo_validation.sh`
- `scripts/summarize_directo_batch.py`
- `scripts/diag_tf_tcp.sh`
- `scripts/diag_startup_health.sh`
- `scripts/validate_panel_flow.sh`
- `../lanzar_panelv2.sh`

## Gate geometrico de arranque

El sistema no debe declarar `READY` si la geometria runtime no coincide con la geometria canonica. El snapshot publicado en `/system_diag` incluye:

- `geometry_ok`
- `geometry_reason`
- `geometry_expected_xyz`
- `geometry_actual_xyz`
- `geometry_frame_error_m`
- `geometry_pair_error_m`
- `geometry_urdf_source`

## Validacion ejecutada

### 1. Validacion DIRECTO

- Corrida unica de comprobacion: OK
- Batch `RUNS=5`: OK
- Batch `RUNS=10`: OK

Artefacto agregado final:

- `../auditoria/directo_batch_20260422_144041/batch_summary.json`

Resultado final del batch largo:

- `runs=10`
- `passed=10`
- `failed=0`

Todas las corridas aceptadas cerraron con:

- `helper_rc=0`
- `benchmark_rc=0`
- `geometry_ok=true`
- `system_state=READY`
- smoke visual completo con `pre_grasp`, `grasp_confirmed`, `lift_with_object` y `basket_drop`

### 2. Repeticion final con launcher canonico

Se repitio la validacion final usando:

```bash
./lanzar_panelc2.sh
```

Resultado:

- seleccion de objeto: `success=True`
- arranque de `pick_demo`: `success=True`
- cierre de secuencia: `SECUENCIA COMPLETADA EXITOSAMENTE route=basket`
- confirmacion de cesta: `confirmacion cesta OK`

Artefactos de cierre:

- `../auditoria/panelc2_final_20260422_141028/system_diag_ready.json`
- `../auditoria/panelc2_final_20260422_141028/visual_smoke/visual_capture_manifest.json`
- `../auditoria/panelc2_final_20260422_141028/visual_smoke/pre_grasp.png`
- `../auditoria/panelc2_final_20260422_141028/visual_smoke/grasp_confirmed.png`
- `../auditoria/panelc2_final_20260422_141028/visual_smoke/lift_with_object.png`
- `../auditoria/panelc2_final_20260422_141028/visual_smoke/basket_drop.png`

## Criterios de aceptacion cumplidos

- `tool0 -> rg2_tcp` y `tool0 -> rg2_pinch_center` coinciden con `+0.0050885`
- el self-check geometrico bloquea el arranque si la geometria no coincide
- MoveIt quedo endurecido sin retocar el TCP
- DIRECTO paso `5/5` y despues `10/10`
- la evidencia visual minima quedo completa
- la repeticion final se ejecuto con `./lanzar_panelc2.sh`

## Riesgos residuales y politica futura

- La evidencia visual sigue teniendo prioridad sobre logs parciales.
- Si en el futuro aparece inestabilidad de carry o planificacion, el primer sitio a tocar no es el TCP.
- Cualquier ajuste futuro de tolerancias de transporte o de validacion de `home_with_object` debe validarse contra el launcher canonico y con evidencia visual.

## Declaracion de cierre

El incidente queda cerrado como un problema de definicion geometrica del TCP, ya corregido, congelado y validado en arranque, en DIRECTO y en el launcher canonico del sistema.
