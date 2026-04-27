# Cierre tecnico del incidente TCP RG2 en UR5

Fecha de cierre geometrico: 2026-04-23
Estado: incidente TCP cerrado; follow-up funcional de cesta abierto
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

Queda explicitamente descartado volver a tocar este TCP para ajustar planificacion, grasp, attach o transporte sin una nueva validacion visual dedicada.

## Implementacion aplicada

1. Se elimino el uso operativo de offsets legacy o duplicados en panel, DIRECTO y launch.
2. Se promovio el self-check geometrico a gate de arranque con publicacion detallada en `/system_diag` y captura obligatoria por ciclo en `scripts/validate_startup_repro.sh`.
3. Se centralizo el perfil runtime validado en `scripts/panel_runtime_validated.env`.
4. Se endurecio DIRECTO en transporte con retries, replan segmentado, guard pre-ejecucion FK/modelo para `CESTA_STAGE_*` y post-check runtime/modelo sin volver a tocar el TCP.
5. Se alineo la repeticion manual final con el launcher canonico `./lanzar_panelc2.sh`.

## Evidencia de cierre del incidente TCP

### 1. Gate geometrico de arranque

En las corridas manuales recientes el sistema solo avanzo a `READY` cuando la geometria runtime coincidio con el URDF canonico. Snapshots representativos:

- `../auditoria/manual_directo_20260423_053012/system_diag_startup.json`
- `../auditoria/manual_directo_startupretry_20260423_090729/system_diag_startup.json`
- `../auditoria/manual_directo_postcheck_20260423_092724/system_diag_startup.json`

En todos ellos:

- `geometry_ok=true`
- `rg2_tcp = rg2_pinch_center = +0.0050885`
- `state=READY`

Adicionalmente, el harness de reproducibilidad de arranque ya deja evidencia por ciclo:

- `report/repro_startup/20260423_161929/cycle1_system_diag.json`
- `report/repro_startup/20260423_161929/summary.log`

### 2. Evidencia visual minima del fix TCP

La evidencia visual reciente valida el comportamiento geometrico hasta lift:

- `../auditoria/manual_directo_startupretry_20260423_090729/visual_smoke/pre_grasp.png`
- `../auditoria/manual_directo_startupretry_20260423_090729/visual_smoke/grasp_confirmed.png`
- `../auditoria/manual_directo_startupretry_20260423_090729/visual_smoke/lift_with_object.png`

La secuencia se ve consistente con el nuevo TCP:

- el pinch center queda alineado con el objeto en `pre_grasp`;
- el objeto queda entre dedos en `grasp_confirmed`;
- el objeto se eleva con el gripper en `lift_with_object`.

Esto cierra el incidente TCP como problema geometrico.

## Estado funcional residual

El flujo DIRECTO completo a cesta sigue bloqueado, pero ya no por el TCP.

### 1. Evidencia de que el bloqueo ya no es geometrico

Corrida:

- `../auditoria/manual_directo_postcheck_20260423_092724`

Hallazgo principal en `helper.log`:

- `CESTA_STAGE_1_RECOVER_1_postcheck_failed`
- `runtime_target_dist=0.055/0.040`
- `model_target_err=0.055/0.040`

Interpretacion:

- el TCP runtime ya esta bien;
- el sub-stage de transporte falla porque el objetivo articular ejecutado no deja el modelo ni el runtime dentro de tolerancia.

### 2. Evidencia de que mas granularidad sola no resuelve

Corrida con micro-replan adicional:

- `../auditoria/manual_directo_postcheck_min035_20260423_093612`

Hallazgos en `helper.log`:

- `CESTA_STAGE_1_RECOVER_1_RECOVER_2` entro en tolerancia con `runtime_target_dist=0.038/0.040`
- `CESTA_STAGE_1_RECOVER_1_RECOVER_3` volvio a fallar con `runtime_target_dist=0.050/0.040`
- el cierre quedo en `cesta_stage_1_recover_1_recover_3_postcheck_failed`

Interpretacion:

- bajar `PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_REMAINING_DIST_M` ayuda;
- pero el cuello ya no es de simple segmentacion;
- el problema residual esta en la consistencia/calidad del IK y del seed/branch en el corredor de cesta.

## Criterios de aceptacion del fix geometrico cumplidos

- `tool0 -> rg2_tcp` y `tool0 -> rg2_pinch_center` coinciden con `+0.0050885`
- el self-check geometrico bloquea `READY` si la geometria no coincide
- ya no quedan overrides editables del TCP en panel/launch/runtime
- MoveIt quedo endurecido sin retocar el TCP
- la evidencia visual valida el fix hasta `lift_with_object`

## Criterios funcionales pendientes

- completar `basket_drop` con evidencia visual valida
- cerrar `helper_final_rc=0` en DIRECTO end-to-end
- pasar `RUNS=5`
- pasar `RUNS=10`
- repetir la validacion manual final con `./lanzar_panelc2.sh`

## Follow-up abierto

El incidente residual queda separado de este cierre y pasa a:

- `report/incidents/2026-04-23_directo_basket_transport_ik_followup.md`

Ese follow-up cubre:

- consistencia `fk_ur5(solved_q)` vs `target_ik` en `CESTA_STAGE_1`
- estrategia de seed/branch para transporte a cesta
- criterios para revalidar `RUNS=5`, `RUNS=10` y la repeticion final con `./lanzar_panelc2.sh`

## Declaracion de cierre

El incidente de geometria del TCP queda cerrado como problema de definicion geometrica ya corregido, congelado y validado.

La aceptacion funcional completa del flujo DIRECTO no queda cerrada por este documento. Permanece bloqueada por un incidente distinto de transporte/IK en cesta, que debe resolverse y volver a validarse con `./lanzar_panelc2.sh`.
