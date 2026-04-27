# Diff contra documento anterior

```diff
--- report/BaseDeConocimiento/2026-04-23_base_conocimiento_tecnica_TFM.md
+++ 2026-04-23_base_conocimiento_tecnica_TFM.md
@@ -11,7 +11,7 @@
 - Contenido vigente preservado explícitamente: attach lógico vs transporte físico, ATTACH_GATE vs CARRY, follow_tcp vs world_locked, stale_tcp_pose_soft_follow, discrepancias metadata/llamada real/telemetría y diagnósticos best_obj_move / best_lift_delta / best_tcp_dist.
 - Recuperación histórica: se reincorporan arquitectura extendida, tabla amplia de frames, geometría, flujo fase por fase, inventario amplio de variables, controladores/topics, bugs legacy y troubleshooting operativo de 2026-04-18.
 - Discrepancias abiertas resaltadas: world->base_link actual=-0.85 0 0.850 frente a simplificaciones históricas; attach backend launch=0.08 / wrapper runtime=0.06; max_pose_age launch=1.5 / wrapper runtime=2.5.
-- Cronología reciente incorporada: `2026-04-22` cerró el incidente geométrico del TCP con valor canónico `${rg2_contact_tcp_xyz}`; `2026-04-23` abrió un follow-up separado de transporte a cesta / IK (`abierto`) que no debe confundirse con una regresión del TCP.
+- Cronología reciente incorporada: `2026-04-22` cerró operativamente el incidente TCP con valor canónico `+0.0050885`; `2026-04-23` lo ratifica geométricamente en el workspace y deja abierto un follow-up separado de cesta / IK (`abierto`).
 - Nuevas fuentes de detalle incorporadas al generador: `gripper_geometry.py`, `panel_runtime_validated.env`, `validate_startup_repro.sh` y los incidentes recientes de 2026-04-21/22/23.
 - Regla de trazabilidad aplicada: cuando una fuente histórica difiere de la actual, se conserva como histórico/documentado previamente en lugar de borrarla.
 
@@ -347,23 +347,23 @@
 | `cadena cinemática UR5` | joints UR5 | `base_link_inertia` | per DH / ur_description | Shoulder -> wrist_3 del brazo | No mezclar valores DH con poses world sin transformación previa. |
 | `tool0` | TCP semántico del brazo | `wrist_3_link` | SDF visual actual: 0 0.275 0 -1.570796325 0 0 | Padre semántico de RG2 y ancla de offset del gripper | Confundir tool0 con el punto real de grasp desplaza el contacto 175 mm. |
 | `rg2_base_link` | frame URDF fijo | `tool0` | actual confirmado: 0 0 0 | Base semántica del gripper RG2 | No equivale al pinch center operativo. |
-| `rg2_tcp` | frame URDF fijo | `tool0` | actual confirmado: ${rg2_contact_tcp_xyz} | TCP virtual/semántico del gripper | No confundir con geometría visible SDF del cuerpo RG2. |
-| `rg2_pinch_center` | frame URDF fijo | `tool0` | actual confirmado: ${rg2_contact_tcp_xyz} | Punto operacional de grasp y attach | Es el frame correcto para lógica de pick; usar tool0 degrada Z y distancias. |
+| `rg2_tcp` | frame URDF fijo | `tool0` | actual confirmado: 0 0 0.0050885 | TCP virtual/semántico del gripper | No confundir con geometría visible SDF del cuerpo RG2. |
+| `rg2_pinch_center` | frame URDF fijo | `tool0` | actual confirmado: 0 0 0.0050885 | Punto operacional de grasp y attach | Es el frame correcto para lógica de pick; usar tool0 degrada Z y distancias. |
 | `camera_wrist_link` | frame SDF fijo | `rg2_hand` | actual confirmado en SDF: 0 0 0.05 0 0 0 | Cámara montada en la muñeca/gripper | Es geometría SDF, no el TCP operacional. |
-| `pick_demo_anchor` | frame SDF fijo / runtime rewrite | `tool0` | SDF fuente=0 0 0.0050885 0 0 0; runtime actual sincronizado con la geometría canónica (`rg2_tcp`=${rg2_contact_tcp_xyz}) | Ancla auxiliar para attach backend / probes | Si el runtime deja de sincronizarse con el URDF, el ancla vuelve a desviarse de rg2_pinch_center. |
+| `pick_demo_anchor` | frame SDF fijo / runtime rewrite | `tool0` | SDF fuente=0 0 0.0050885 0 0 0; runtime actual sincronizado con la geometría canónica (`rg2_tcp`=0.0050885) | Ancla auxiliar para attach backend / probes | Si el runtime deja de sincronizarse con el URDF, el ancla vuelve a desviarse de rg2_pinch_center. |
 
 ### 4.2 Discrepancias actuales vs históricas
 
 | Elemento | Valor actual confirmado | Valor histórico / documentado | Observación |
 |---|---|---|---|
 | world -> base_link | actual confirmado: -0.85 0 0.850 | histórico simplificado: (0,0,0.850) | La geometría actual incluye traslación en X; documentar sólo Z ya no es suficiente. |
-| tool0 -> rg2_pinch_center | actual confirmado: ${rg2_contact_tcp_xyz} | histórico: 0 0 0.175 | Sin discrepancia de valor; se conserva como confirmación actual. |
+| tool0 -> rg2_pinch_center | actual confirmado: 0 0 0.0050885 | histórico: 0 0 0.175 | Sin discrepancia de valor; se conserva como confirmación actual. |
 | SDF ur5_hand_joint | actual confirmado: relative_to=wrist_3_link, pose=0 0.0823 0 1.570796325 0 1.570796325 | memorias previas con otros valores | Priorizar el source tree actual; los valores previos quedan como históricos o desactualizados. |
-| pick_demo_anchor | actual runtime efectivo: ${rg2_contact_tcp_xyz} m | histórico documentado: 0.0050885 m | El launch ahora sincroniza `pick_demo_anchor` directamente con la geometría canónica del URDF; ya no depende de `gripper_tcp_z_offset`. |
+| pick_demo_anchor | actual runtime efectivo: 0.0050885 m | histórico documentado: 0.0050885 m | El launch ahora sincroniza `pick_demo_anchor` directamente con la geometría canónica del URDF; ya no depende de `gripper_tcp_z_offset`. |
 
 ### 4.3 Observaciones obligatorias preservadas y reforzadas
 
-- `tool0` vs `rg2_pinch_center`: el offset semántico actual confirmado sigue siendo ${rg2_contact_tcp_xyz}. La geometría visual SDF del gripper se modela por `ur5_hand_joint=0 0.0823 0 1.570796325 0 1.570796325` y no debe confundirse con el TCP semántico.
+- `tool0` vs `rg2_pinch_center`: el offset semántico actual confirmado sigue siendo 0 0 0.0050885. La geometría visual SDF del gripper se modela por `ur5_hand_joint=0 0.0823 0 1.570796325 0 1.570796325` y no debe confundirse con el TCP semántico.
 - `world` vs `base_link`: el estado actual confirmado no es sólo Z. La traslación usada por URDF/world file es `-0.85 0 0.850`.
 - `NEGATE_XY`: sigue siendo consecuencia del uso de `base_link_inertia` en el solver DH; no es una preferencia opcional ni debe tratarse como flag de runtime activa.
 - TCP semántico vs geometría visual SDF: la mano visible del SDF y el punto de grasp semántico viven en capas distintas; la segunda es la que manda para IK, attach y carry validation.
@@ -469,7 +469,7 @@
 | Stiffness contacto fingers | kp=200000 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
 | Damping contacto fingers | kd=30.0 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
 | Masa cuerpo RG2 (rg2_hand) | 0.29881464108881906 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
-| tool0 -> rg2_pinch_center | ${rg2_contact_tcp_xyz} | agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro |
+| tool0 -> rg2_pinch_center | 0 0 0.0050885 | agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro |
 | Pose visual ur5_hand_joint | relative_to=wrist_3_link, pose=0 0.0823 0 1.570796325 0 1.570796325 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
 | Finger izquierdo respecto a rg2_hand | 0.105 0.017 0 0 0 0 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
 | Finger derecho respecto a rg2_hand | 0.105 -0.017 0 0 0 0 | agarre_ros2_ws/models/ur5_rg2/model.sdf |
@@ -488,7 +488,7 @@
 
 ### 5.3 Estado de validación de geometría
 
-- Cierre geométrico `2026-04-22`: `tool0 -> rg2_tcp.z = ${rg2_contact_tcp_xyz}` validado y sincronizado con `rg2_pinch_center`; `gripper_geometry.py` centraliza esta lectura desde el URDF.
+- Cierre geométrico documentado entre `2026-04-22` y `2026-04-23`: `tool0 -> rg2_tcp.z = +0.0050885` validado y sincronizado con `rg2_pinch_center`; `gripper_geometry.py` centraliza esta lectura desde el URDF.
 - `tool0 -> rg2_pinch_center`: actual confirmado en URDF.
 - `pick_demo`: actual confirmado en world file con radio=0.025 m y longitud=0.05 m.
 - Guardrail de arranque actual: `validate_startup_repro.sh` exige `geometry_ok=true` y `state=READY` antes de considerar reproducible el stack.
@@ -1447,8 +1447,8 @@
 
 ### 10.2 Cronología reciente de incidentes
 
-- `2026-04-22`: el cierre geométrico del TCP fija `tool0 -> rg2_tcp.z = ${rg2_contact_tcp_xyz}` y consigna batch `DIRECTO=no consignado en la fuente cargada` más repetición manual final `fecha no extraída` con `./lanzar_panelc2.sh`.
-- `2026-04-23`: se abre un follow-up distinto y explícitamente separado del TCP (`abierto`) para el corredor de cesta / IK, con señal representativa `CESTA_STAGE_1_RECOVER_1_postcheck_failed, runtime_target_dist=0.055/0.040, model_target_err=0.055/0.040`.
+- `2026-04-22`: `report/incidents/2026-04-21_tcp_geometry_incident.md` documenta el cierre operativo del TCP con `tool0 -> rg2_tcp.z = +0.0050885`, batch `DIRECTO=5/5` y repetición manual final `2026-04-22` con `./lanzar_panelc2.sh`.
+- `2026-04-23`: `agarre_ros2_ws/report/incidents/2026-04-22_rg2_tcp_incident_closure.md` ratifica el cierre geométrico del TCP (`incidente TCP cerrado; follow-up funcional de cesta abierto`) y deja separado el corredor de cesta / IK con señal representativa `CESTA_STAGE_1_RECOVER_1_postcheck_failed, runtime_target_dist=0.055/0.040, model_target_err=0.055/0.040`.
 
 ### Detalle histórico recuperado (2026-04-18)
 
@@ -1811,14 +1811,14 @@
 
 ### 12.1 Cronología reciente confirmada
 
-- `2026-04-22`: cierre geométrico del TCP RG2; valor canónico validado `${rg2_contact_tcp_xyz}` y aceptación manual final con `./lanzar_panelc2.sh` según `agarre_ros2_ws/report/incidents/2026-04-22_rg2_tcp_incident_closure.md`.
-- `2026-04-23`: follow-up técnico abierto y separado para transporte DIRECTO a cesta / IK (`abierto`), con señal `CESTA_STAGE_1_RECOVER_1_postcheck_failed, runtime_target_dist=0.055/0.040, model_target_err=0.055/0.040` según `agarre_ros2_ws/report/incidents/2026-04-23_directo_basket_transport_ik_followup.md`.
+- `2026-04-22`: cierre operativo documentado en `report/incidents/2026-04-21_tcp_geometry_incident.md`; valor canónico `+0.0050885` y aceptación manual final con `./lanzar_panelc2.sh`.
+- `2026-04-23`: nota técnica de workspace en `agarre_ros2_ws/report/incidents/2026-04-22_rg2_tcp_incident_closure.md` ratifica el cierre geométrico y deja abierto el follow-up de cesta / IK (`abierto`), detallado también en `agarre_ros2_ws/report/incidents/2026-04-23_directo_basket_transport_ik_followup.md`.
 
 ### 12.2 Estado operativo sintetizado
 
-- Geometría semántica actual confirmada: `tool0 -> rg2_tcp = tool0 -> rg2_pinch_center = ${rg2_contact_tcp_xyz}`.
+- Geometría semántica actual confirmada: `tool0 -> rg2_tcp = tool0 -> rg2_pinch_center = 0 0 0.0050885`.
 - Geometría visual SDF actual confirmada: `ur5_hand_joint relative_to=wrist_3_link, pose=0 0.0823 0 1.570796325 0 1.570796325`.
-- `pick_demo_anchor` runtime actual: 0 0 ${rg2_contact_tcp_xyz} sobre `tool0`, sin offsets legacy intermedios.
+- `pick_demo_anchor` runtime actual: 0 0 0.0050885 sobre `tool0`, sin offsets legacy intermedios.
 - Backend actual: modo launch `follow_tcp`, pero `pick_demo` en `pick_demo` usa `world_locked` en demo transport.
 - Validación carry post-grasp real: timeout=3.0s, min_obj_move=0.020, min_lift_delta=0.025, max_tcp_dist=0.120.
 
```
