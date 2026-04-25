# Incidente: Pick físico real pick_demo — UR5 + RG2 en Gazebo

**Fecha:** 2026-04-24  
**Rama:** `audit/tcp-geometry-fix`  
**Objetivo:** Lograr agarre físico real del objeto `pick_demo` con LIFT demostrable.

---

## Estado inicial (FASE 0 — Checkpoint)

### Git status
```
M  agarre_ros2_ws/src/ur5_tools/ur5_tools/ur5_moveit_bridge.py   (+98 líneas, audit logs MOVEIT)
?? reports/incidents/2026-04-24_rg2_visual_geometry_direct_moveit_fix.md
```

### Commits previos relevantes
```
08c6bdb fix(rsp+tf): guard RSP respawn and add TF freshness validation end-to-end
b1c46b2 feat: Add UR5 robot description and RG2 gripper integration
```

### Archivos modificados previamente en esta rama
- `ur5_moveit_bridge.py`: +98 líneas — `[MOVEIT][TIP_LINK]`, `[MOVEIT][TF_FRESHNESS]`, `[MOVEIT][GEOM_AUDIT]`
- RSP guard + respawn ya aplicado en launch (commit 08c6bdb)
- TF freshness validation en panel_pick_demo.py ya implementada

---

## Restricciones

- NO tocar `tool0 → rg2_pinch_center = (0, 0, 0.175)` salvo evidencia numérica irrefutable.
- NO tocar URDF/SDF salvo que el runtime contradiga la auditoría previa.
- NO declarar éxito si solo pasa ATTACH_GATE. El éxito real requiere LIFT con objeto moviéndose.
- NO usar Gazebo Classic ni ROS 1. Mantener ROS 2 Jazzy.
- Cambios mínimos, trazables y reversibles.

---

## Objetivo de la tarea

1. Llegar al objeto `pick_demo` con la pinza visual real.
2. Cerrar el RG2 con el objeto dentro del volumen de cierre.
3. Pasar ATTACH_GATE solo si la pose es físicamente válida.
4. Hacer LIFT y demostrar que el objeto sube con el TCP.
5. Aplicar fixes mínimos y dejar logs y evidencias de agarre físico real.

---

## Arquitectura del mecanismo de agarre (referencia)

| Componente | Detalle |
|---|---|
| TCP operacional | `rg2_pinch_center` (alias `rg2_tcp`) |
| Offset tool0→TCP | `(0, 0, 0.175)` m |
| Backend attach | `demo_transport` para `pick_demo` |
| Mecanismo | Delete pick_demo → spawn kinematic carry → teleport via `set_entity_pose` |
| Follow rate | 20 Hz (configurable) |
| CARRY settle | 3.0 s (espera backend) |
| CARRY thresholds | `min_obj_move=0.020 m`, `min_lift_delta=0.025 m`, `max_tcp_dist=0.120 m` |

---

## Variables de entorno para prueba

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
source install/setup.bash
export PANEL_PICK_DEMO_REQUIRE_FRESH_TF=1
export PANEL_PICK_DEMO_TF_MAX_AGE_SEC=0.5
```

---

## Cambios aplicados en FASE 0 → FASE 7

### FASE 3 (nueva) — `[PICK][VISUAL_ALIGN_FIX]` tras GRASP_ALIGN_IK
**Archivo:** `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`  
**Línea de inserción:** ~11549 (entre cleanup de `_align_seed_injected` y `post_align_metrics`)  
**Motivo:** La evidencia previa muestra TCP semántico a ~5 mm del objeto pero pinza visual lejos.  
Esta corrección mueve el TCP en base a la diferencia `gz_obj - gz_mid` (Gazebo world frame),  
transformada a `base_link` vía `_target_base_from_world()`, y llama a `_move_tcp_direct()`.  
**Variables de control:**
- `PANEL_PICK_DEMO_REQUIRE_VISUAL_GRASP=1` (default 1)
- `PANEL_PICK_DEMO_VISUAL_GRASP_TOL_M=0.020`
- `PANEL_PICK_DEMO_VISUAL_ALIGN_MAX_STEP_M=0.015`
- `PANEL_PICK_DEMO_VISUAL_ALIGN_MAX_ATTEMPTS=3`

**Formato log:**
```
[PICK][VISUAL_ALIGN_FIX] phase=GRASP_ALIGN_IK attempt=.../... visual_mid=(...) object_world=(...)
  pinch_world=(...) err_world=(...) err_norm=... corr_world=(...) capped=false
  target_before=(...) target_after=(...) dist_visual_mid_obj=... tol_m=0.020 verdict=APPLIED/OK/FAILED

[PICK][VISUAL_ALIGN_FIX] phase=GRASP_ALIGN_IK_final visual_mid_final=(...) dist_visual_mid_obj_final=... verdict=OK/APPLIED_NOT_CONVERGED

[PICK][VISUAL_ALIGN_FIX] phase=PRE_CLOSE_GATE dist_visual_mid_obj=... tol_m=0.020 verdict=BLOCKED
```

---

### FASE 3 — `[PICK][LOCAL_ALIGN_FIX]` en GRASP_ALIGN_IK
**Archivo:** `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`  
**Línea:** ~11497 (después de `[GRASP_Z_TRACE] phase_snapshot=GRASP_ALIGN_IK_end`)  
**Cambio:** Añadido diagnóstico del objeto en frame local del TCP tras GRASP_ALIGN_IK.  
**Motivo:** Auditar si el objeto está dentro del volumen físico de cierre antes de PRE_CLOSE.  
**Formato log:**
```
[PICK][LOCAL_ALIGN_FIX] phase=GRASP_ALIGN_IK tcp_base=(...) obj_base=(...)
  obj_dx=... obj_dy=... obj_dz=... lateral_err_m=... depth_err_m=...
  lat_tol=... dep_tol=... in_grasp_volume=true/false verdict=IN_VOLUME/LATERAL_ERR/DEPTH_ERR
```

### FASE 4 — `[PICK][GRIPPER_CLOSE_AUDIT]` en CLOSE
**Archivo:** `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`  
**Línea:** ~12012 (después de `_append_trace(_cl_check_msg)`)  
**Cambio:** Añadido resumen estructurado del resultado de CLOSE.  
**Motivo:** Un único log que consolida pre_opening, post_opening, delta y veredicto.  
**Formato log:**
```
[PICK][GRIPPER_CLOSE_AUDIT] pre_opening_sum=... post_opening_sum=... delta_sum=...
  confirm_mode=... measured_ok=... closed_flag=... geometry_ok=... verdict=CLOSED/NOT_CLOSED
```

### FASE 5 — `[PICK][ATTACH_PHYSICAL_GATE]` en ATTACH_GATE
**Archivo:** `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`  
**Línea:** ~12211 (antes de `attach_ok = panel._attempt_attach(...)`)  
**Cambio:** Comprobación física previa al attach: volumen PRE_CLOSE, TF fresco, visual_mid_obj, TCP-obj, gripper cerrado.  
**Motivo:** Gate físico explícito antes de activar demo_transport.  
**Formato log:**
```
[PICK][ATTACH_PHYSICAL_GATE] preclose_volume=... lateral_err_mm=... depth_err_mm=...
  tf_fresh=... visual_mid_obj=(...) tcp_obj_dist_m=... gripper_closed=...
  opening_sum=... geometry_ok=... backend=pending verdict=PASS/WARN
```

### FASE 7 — `[ATTACH_BACKEND][ROUTE]` y `[ATTACH_BACKEND][FOLLOW]`
**Archivo:** `src/ur5_tools/ur5_tools/gripper_attach_backend.py`  
**ROUTE:** `_on_gripper_attach` ~línea 2011 — sustituye/complementa `attach_route_decision` con formato estandarizado.  
**Formato:**
```
[ATTACH_BACKEND][ROUTE] object=pick_demo route=demo_transport anchor=pick_demo_anchor
  dist=... max=... geometry_ok=... tf_fresh=... tcp_age=... tcp_src=...
```
**FOLLOW:** `_demo_transport_update` ~línea 842 — añade log de before/after pose del objeto.  
**Formato:**
```
[ATTACH_BACKEND][FOLLOW] object=pick_demo tcp=(...) obj_before=(...) obj_after=(...)
  delta_z=... delta_3d=... verdict=ok
```

---

## Tabla de resultados (rellenar tras run)

| Check | Resultado | Evidencia |
|---|---|---|
| TF fresco | PENDIENTE | `[PICK][TF_FRESHNESS]` |
| Pinza visual sobre objeto | PENDIENTE | `[PICK][RG2_VISUAL_AUDIT]` |
| LOCAL_ALIGN_FIX in_grasp_volume | PENDIENTE | `[PICK][LOCAL_ALIGN_FIX]` |
| PRE_CLOSE_VOLUME | PENDIENTE | `[PICK][PRE_CLOSE_VOLUME]` |
| CLOSE confirmado | PENDIENTE | `[PICK][GRIPPER_CLOSE_AUDIT]` |
| ATTACH_PHYSICAL_GATE | PENDIENTE | `[PICK][ATTACH_PHYSICAL_GATE]` |
| ATTACH_GATE PASS | PENDIENTE | `[ATTACH_GATE][PASS]` |
| ATTACH_BACKEND ROUTE | PENDIENTE | `[ATTACH_BACKEND][ROUTE]` |
| ATTACH_BACKEND FOLLOW | PENDIENTE | `[ATTACH_BACKEND][FOLLOW]` |
| LIFT best_obj_move | PENDIENTE | `[PICK][DIRECT][PHYSICS]` |
| LIFT best_lift_delta | PENDIENTE | `[PICK][DIRECT][PHYSICS]` |
| LIFT best_tcp_dist | PENDIENTE | `[PICK][DIRECT][PHYSICS]` |
| Objeto levantado visualmente | PENDIENTE | observación Gazebo |
| DIRECTO ciclo completo | PENDIENTE | — |
| MOVEIT geométrico | PENDIENTE | `[MOVEIT][GEOM_AUDIT]` |

---

## Comandos para reproducir

```bash
# Terminal 1 — arranque
cd /home/laboratorio/TFM/agarre_ros2_ws
source install/setup.bash
export PANEL_PICK_DEMO_REQUIRE_FRESH_TF=1
export PANEL_PICK_DEMO_TF_MAX_AGE_SEC=0.5
ros2 launch ur5_bringup ur5_stack.launch.py 2>&1 | tee /tmp/ur5_real_grasp_fix.log

# Terminal 2 — verificación base
source install/setup.bash
ros2 node list | grep robot_state_publisher
ros2 topic info /tf -v
timeout 5 ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
ros2 run ur5_tools validate_dh_vs_tf
ros2 run ur5_tools inspect_rg2_visual_pose --ros-args -p once:=true -p object_name:=pick_demo

# Filtro logs clave
grep -E "\[PICK\]\[TF_FRESHNESS\]|\[PICK\]\[RG2_VISUAL_AUDIT\]|\[PICK\]\[LOCAL_ALIGN_FIX\]|\[PICK\]\[PRE_CLOSE_VOLUME\]|\[PICK\]\[GRIPPER_CLOSE_AUDIT\]|\[PICK\]\[ATTACH_PHYSICAL_GATE\]|\[ATTACH_BACKEND\]\[ROUTE\]|\[ATTACH_BACKEND\]\[FOLLOW\]|\[PICK\]\[DIRECT\]\[PHYSICS\]" /tmp/ur5_real_grasp_fix.log
```

---

## Criterio de éxito

El objeto `pick_demo` debe levantarse en LIFT en ruta DIRECTO:
- `best_obj_move >= 0.020 m`
- `best_lift_delta >= 0.025 m`
- `best_tcp_dist <= 0.120 m`

NO declarar éxito solo porque TCP está cerca, PRE_CLOSE pasa, CLOSE pasa o ATTACH_GATE pasa.  
Solo éxito si LIFT demuestra movimiento físico real del objeto.
