# Incidente TCP RG2 — cierre técnico

Fecha de cierre: 2026-04-22

## Resumen

La causa raíz real fue una definición incorrecta del frame TCP respecto a `tool0`.

- Incorrecto histórico: `tool0 -> rg2_tcp.z = -0.0877005`
- Correcto validado: `tool0 -> rg2_tcp.z = +0.0050885`
- `rg2_pinch_center` debe coincidir con `rg2_tcp`

No fue un salto de rama IK ni una recomposición lateral en `GRASP_DOWN`. Esos síntomas eran secundarios al desacoplo entre el target semántico y la geometría visible de la pinza.

## Alcance documental

Este informe congela evidencia histórica del incidente TCP y de las corridas asociadas a ese cierre geométrico. No debe leerse como veredicto final del estado actual del pipeline DIRECTO ni como prueba suficiente del cierre de cesta/recoveries a fecha 2026-04-24.

Para el estado canónico vigente del runtime y de los pendientes `CESTA_STAGE_1_RECOVER_1/RECOVER_2`, la referencia válida es `auditoria/informe_auditoria_global_20260424.md`.

Cuando aquí aparezca `./lanzar_panelv2.sh`, debe leerse como alias histórico equivalente al launcher canónico actual `./lanzar_panelc2.sh`.

## Evidencia de referencia

- Logs previos con offset incorrecto:
  - `/home/laboratorio/TFM/auditoria/spatial_20260421/directo_validation_20260421_073359/helper.log`
- Logs tras el fix geométrico:
  - `/home/laboratorio/TFM/auditoria/spatial_20260421/directo_validation_20260421_074626/helper.log`
- Evidencia visual previa:
  - `/home/laboratorio/TFM/auditoria/spatial_20260421/directo_validation_20260421_073359/camera_frames/073629_camera_debug_top_periodic.jpg`
- Evidencia visual corregida:
  - `/home/laboratorio/TFM/auditoria/spatial_20260421/directo_validation_20260421_074626/camera_frames/074847_camera_debug_top_periodic.jpg`
  - `/home/laboratorio/TFM/auditoria/spatial_20260421/directo_validation_20260421_074626/camera_frames/074851_camera_debug_top_periodic.jpg`
  - `/home/laboratorio/TFM/auditoria/spatial_20260421/directo_validation_20260421_074626/camera_frames/074851_camera_overhead_periodic.jpg`

## Decisiones permanentes

- La fuente única de verdad geométrica es `ur5_description/urdf/ur5.urdf.xacro`.
- `model.sdf`, panel, backend y runtime deben derivar de ese URDF.
- El arranque no debe declarar `READY` si TF y geometría del gripper no coinciden con el URDF canónico.
- La validación operativa del caso debe incluir evidencia visual; logs verdes sin concordancia visual no resuelven el sistema.

## Flujo oficial de operación

El flujo canónico, oficial y válido de operación es:

```bash
./lanzar_panelc2.sh
```

`./lanzar_panelv2.sh` queda como alias temporal de compatibilidad y `agarre_ros2_ws/scripts/start_panel_v2.sh` como wrapper interno, no como entrada operativa oficial.

## Guardrails activos

- `system_state_manager` bloquea `READY` hasta pasar el self-check geométrico.
- El runtime SDF sincroniza `pick_demo_anchor_joint` desde el URDF canónico.
- `run_directo_validation.sh` genera smoke visual mínimo.
- `run_directo_batch_validation.sh` permite validar 5–10 corridas con resumen agregado.
- La aceptación manual final debe repetirse explícitamente con `./lanzar_panelc2.sh`.

## Cierres operativos posteriores al fix TCP

Tras congelar la geometría correcta, aparecieron dos bloqueos operativos adicionales que no eran un problema de TCP:

1. Transporte con objeto: `gripper_attach_backend` seguía al TCP usando snapshots envejecidos.
2. Watchdog del panel: la liveness de `/system_state` dependía del hilo UI y podía caer durante trayectos largos aunque el tópico siguiera vivo.

Correcciones aplicadas:

- `gripper_attach_backend.py` ahora prioriza `joint_state_chain` calculado con FK local desde `/joint_states`.
- `panel_external_state.py` y `panel_ros.py` ahora toman la frescura de `/system_state` desde `RosWorker`, no solo desde el slot del hilo UI.

Evidencia de runtime:

- Corrida canónica única de verificación fuerte:
  - `/home/laboratorio/TFM/auditoria/directo_release_fix_probe_20260421_5`
- En esa corrida se observa:
  - `HOME_WITH_OBJECT` superado.
  - `phase=home_with_object ok`.
  - entrada en `CESTA`, `CESTA_RELEASE`, `RELEASE` y `HOME_FINAL`.
  - `SECUENCIA COMPLETADA EXITOSAMENTE route=basket`.
  - confirmación final de cesta `OK`.

Evidencia visual de esa corrida:

- `/home/laboratorio/TFM/auditoria/directo_release_fix_probe_20260421_5/visual_smoke/pre_grasp.png`
- `/home/laboratorio/TFM/auditoria/directo_release_fix_probe_20260421_5/visual_smoke/grasp_confirmed.png`
- `/home/laboratorio/TFM/auditoria/directo_release_fix_probe_20260421_5/visual_smoke/lift_with_object.png`
- `/home/laboratorio/TFM/auditoria/directo_release_fix_probe_20260421_5/visual_smoke/basket_drop.png`

## Validacion DIRECTO final en 5 corridas

Lote ejecutado:

- `/home/laboratorio/TFM/auditoria/directo_batch_20260421_2`
- Flujo de arranque usado en las 5 corridas históricas: `./lanzar_panelv2.sh`
- Alias canónico actual equivalente: `./lanzar_panelc2.sh`

Resumen agregado final:

- `runs=5`
- `passed=5`
- `failed=0`
- `helper_final_rc=0` en `5/5`
- `benchmark_rc=0` en `5/5`
- smoke visual completo en `5/5`: `pre_grasp`, `grasp_confirmed`, `lift_with_object`, `basket_drop`
- secuencia completa en `5/5`: `SECUENCIA COMPLETADA EXITOSAMENTE route=basket`
- confirmación de cesta en `5/5`: `[PICK][DEMO] confirmacion cesta OK`
- sin `ERROR_FATAL` ni `system_state no disponible` en `5/5`

Artefactos de referencia:

- Resumen endurecido:
  - `/home/laboratorio/TFM/auditoria/directo_batch_20260421_2/batch_summary.json`
- Ejemplos visuales:
  - `/home/laboratorio/TFM/auditoria/directo_batch_20260421_2/directo_validation_20260421_094222_run01/visual_smoke/basket_drop.png`
  - `/home/laboratorio/TFM/auditoria/directo_batch_20260421_2/directo_validation_20260421_094925_run02/visual_smoke/grasp_confirmed.png`
  - `/home/laboratorio/TFM/auditoria/directo_batch_20260421_2/directo_validation_20260421_100337_run04/visual_smoke/lift_with_object.png`
  - `/home/laboratorio/TFM/auditoria/directo_batch_20260421_2/directo_validation_20260421_101040_run05/visual_smoke/basket_drop.png`

Conclusión de validación final:

- El bug geométrico original queda corregido.
- El transporte con objeto queda estabilizado.
- El watchdog falso de `/system_state` queda corregido.
- El flujo histórico equivalente queda validado extremo a extremo en `5/5` corridas.
- El entrypoint canónico actual `./lanzar_panelc2.sh` queda repetido manualmente y validado con trazabilidad explícita del alias operativo.
- La aceptación final se apoya en visuales de `pre_grasp`, `grasp_confirmed`, `lift_with_object` y `basket_drop`, no solo en logs.

## Repetición manual final con `./lanzar_panelc2.sh`

Ejecución manual final realizada el `2026-04-22` usando el flujo canónico:

```bash
./lanzar_panelc2.sh
```

Artefactos de auditoría:

- Directorio de ejecución manual:
  - `/home/laboratorio/TFM/auditoria/manual_final_20260422/manual_final_20260422_114657`
- Log principal del launcher:
  - `/home/laboratorio/TFM/auditoria/manual_final_20260422/manual_final_20260422_114657/launcher.log`

Evidencia técnica capturada en esa pasada:

- `trigger=service:/panel/pick_demo#pickdemo-620671042392734`
- `phase=RELEASE result=ok`
- `phase=HOME_FINAL result=ok`
- `SECUENCIA COMPLETADA EXITOSAMENTE route=basket`
- `[PICK][DEMO] confirmacion cesta OK`

Conclusión de la repetición manual:

- La aceptación final con `./lanzar_panelc2.sh` queda cerrada.
- La geometría del TCP queda validada también en la ruta operativa canónica final.
- El incidente queda cerrado sin pendientes geométricos abiertos.

## Nota residual no bloqueante

En las `5/5` corridas aparece un warning interno de confirmación de apertura:

- `[PICK][DIRECT][GRIPPER] wait_timeout target=open`

Pero en esas mismas corridas también se cumple simultáneamente:

- `logical_state=RELEASED`
- `SECUENCIA COMPLETADA EXITOSAMENTE route=basket`
- `[PICK][DEMO] confirmacion cesta OK`
- evidencia visual `basket_drop.png`

Se clasifica como un ajuste pendiente de telemetría de apertura del gripper, no como un fallo geométrico ni operativo del flujo `DIRECTO`.

## Checklist final

- [x] Fix geometrico bueno del TCP congelado en URDF canónico.
- [x] Offsets duplicados operativos eliminados y derivados desde una sola fuente de verdad.
- [x] Self-check geométrico de arranque añadido y conectado al gate de `READY`.
- [x] Flujo canónico, oficial y válido fijado en `./lanzar_panelc2.sh`.
- [x] Repetición manual final ejecutada con `./lanzar_panelc2.sh`.
- [x] Validacion `DIRECTO` ejecutada en 5 corridas con evidencia visual guardada.
- [x] `MOVEIT` endurecido sin volver a tocar el TCP.
- [x] Smoke test visual mínimo disponible y automatizado.
- [x] Cierre técnico del incidente documentado.
- [x] Causa raíz original cerrada: TCP mal definido.
- [x] Pipeline `DIRECTO` completo cerrado extremo a extremo.

Pendiente menor fuera de este incidente:

- Afinar la confirmación de apertura del gripper para que el warning `wait_timeout target=open` deje de aparecer cuando la liberación física ya ha sido confirmada por cesta y por visual.
