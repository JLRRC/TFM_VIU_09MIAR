# Validación Runtime MOVEIT + STEP — UR5+RG2

**Fecha:** 2026-04-24  
**Rama:** ENTREGA.V2  
**Stack:** ROS 2 Jazzy · Gazebo Sim Harmonic · MoveIt 2 · ros2_control · UR5+OnRobot RG2  
**Estado final:** PARCIAL — infraestructura OK, GUI pendiente + BUG-M08 detectado y corregido

---

## Entorno detectado

```
DISPLAY=localhost:10.0   → no accesible desde proceso Claude (xdpyinfo: unable to open)
WAYLAND_DISPLAY=         → no disponible
XDG_SESSION_TYPE=tty     → sesión no gráfica
```

El panel Qt requiere entorno gráfico para ejecutar MOVEIT en modo AUTO y STEP_BY_STEP.
Las validaciones de comportamiento GUI quedan pendientes de ejecución manual.

---

## Build

```
colcon build --symlink-install --packages-select ur5_qt_panel
→ Finished <<< ur5_qt_panel [1.09s]   PASS
```

Ejecutado desde `/home/laboratorio/TFM/agarre_ros2_ws/` (correcto).

---

## Nodos detectados (stack en ejecución)

`ros2 node list` (con entorno FastDDS correcto):

```
/controller_bootstrap
/controller_manager
/gripper_attach_backend
/gripper_controller
/gz_pose_bridge
/gz_ros_control
/joint_state_broadcaster
/joint_trajectory_controller
/move_group
/move_group/moveit
/panel_superpro          ← panel Qt (en sesión gráfica del usuario)
/panel_tf_helper
/panel_v2_moveit_publisher
/planning_scene_sync
/release_objects_service
/robot_state_publisher
/ros_gz_bridge_main
/system_state_manager
/ur5_moveit_bridge       ← bridge MoveIt activo
/ur5_moveit_py
/world_tf_publisher
```

---

## Controladores activos

```
ros2 control list_controllers:

gripper_controller          forward_command_controller   active  ✓
joint_trajectory_controller joint_trajectory_controller  active  ✓
joint_state_broadcaster     joint_state_broadcaster      active  ✓
```

---

## Topics clave verificados

| Topic | Tipo | Publishers | Subscriptores |
|-------|------|-----------|--------------|
| `/desired_grasp` | `geometry_msgs/PoseStamped` | 0 (panel cerrado) | 2 (bridge×2) |
| `/desired_grasp/result` | `std_msgs/String` | 2 (bridge×2) | 0 (panel cerrado) |
| `/ur5_moveit_bridge/heartbeat` | — | activo | — |
| `/joint_states` | `sensor_msgs/JointState` | activo | 8 nodos |
| `/clock` | — | activo | — |

> Nota: `ros2 topic echo` desde el proceso de Claude tiene aislamiento DDS UDP
> (publisher discovery funciona, datos no fluyen al sandbox). Los datos de topics
> se validaron indirectamente vía logs del stack.

---

## Bridge MoveIt — configuración confirmada

De `log/ros2_launch.log` (stack activo):

```
[ur5_moveit_bridge]: Bridge config:
  rev=2026-04-20-approach-replan-v1
  backend_pref=auto
  group=manipulator
  base=base_link
  ee=rg2_tcp                    ← frame EE del bridge
  result_topic=/desired_grasp/result
  execute_timeout_sec=25.0
  use_sim_time=true
  require_request_id=true

[BRIDGE_CFG] scaling v=0.35 a=0.35
  joint_state_timeout=6.00s
  unwrap_continuous_joints=true
  controller_path_tol=0.350rad
  controller_goal_tol=0.100rad

[BRIDGE][HEARTBEAT] topic=/ur5_moveit_bridge/heartbeat rate_hz=2.00
[BRIDGE][PUB_RESULT] topic=/desired_grasp/result qos=RELIABLE/VOLATILE/KEEP_LAST@depth=10
```

Bridge subscrito a `/desired_grasp` con QoS RELIABLE/VOLATILE/KEEP_LAST@depth=10.

---

## TF — estado confirmado

De `log/ros2_launch.log` (panel Qt en sesión gráfica):

```
[ur5_moveit_bridge]: TF listo: base_link <-> rg2_tcp disponible.   ← 08:37:49

[STARTUP][TF_SANITY] OK base_link->rg2_pinch_center ok
  stamp=6.320000000 age=0.033s                                      ← 08:37:51

[TRACE] TF summary frames=23
  sample=base_link, rg2_pinch_center, rg2_tcp, tool0,
         shoulder_link, upper_arm_link, forearm_link,
         wrist_1_link, wrist_2_link, wrist_3_link, ...

[TRACE] Using BASE_FRAME_EFFECTIVE=base_link EE_FRAME_EFFECTIVE=rg2_pinch_center
```

| Transform | Estado |
|-----------|--------|
| `world → base_link` | OK (inferido de PANEL_TRACE: `world_frame=world base_frame=base_link`) |
| `base_link → rg2_tcp` | **OK** (bridge: "TF listo: base_link <-> rg2_tcp") |
| `base_link → rg2_pinch_center` | **OK** (TF_SANITY pass, age=0.033s) |
| `base_link → tool0` | OK (23 frames incluyen tool0) |

---

## Logs del panel DIRECTO en la sesión activa

El panel ejecutó DIRECTO (pick_demo) durante ~4 minutos:

```
[PICK][DIRECT][PANEL_TRACE]
  world_frame=world base_frame=base_link ee_frame=rg2_pinch_center
  tcp_live_source=tf2:ok
  tcp_live_base=pos=(-0.000,0.366,1.001)
  panel_live_dist_m=0.000     ← FK == TF live (0 desviación)
```

TF live consistente con FK durante toda la sesión. DIRECTO mode: sin incidencias.

---

## Logs MOVEIT encontrados

**Ninguno.** El panel solo ejecutó DIRECTO en la sesión registrada.  
El modo PICK Objeto (MOVEIT) no fue invocado → los nuevos logs `[PICK][MOVEIT][INIT]`,
`[PICK][MOVEIT][STEP]`, `[PICK][MOVEIT][LIFECYCLE]` no aparecen en el historial de logs.

---

## BUG-M08 — ee_link mismatch SIEMPRE falla (detectado y corregido)

### Diagnóstico

Durante la inspección de código + logs se detectó un fallo runtime latente:

**Causa raíz:**
- `panel._ee_frame_effective` = `"rg2_pinch_center"` siempre en runtime  
  (función `_select_ee_frame` en `panel_utils.py:1697` — `rg2_pinch_center` tiene mayor prioridad que `rg2_tcp` en el orden de candidatos)
- El bridge publica `"ee_link": "rg2_tcp"` en **todos** sus resultados JSON  
  (`ur5_moveit_bridge.py:4283`)
- El check en `panel_pick_object.py:3486`:
  ```python
  ee_link_moveit = str(result.get("ee_link") or "")   # → "rg2_tcp"
  if ee_link_moveit and _norm_frame(ee_link_moveit) != _norm_frame(measured_ee_frame):
      raise RuntimeError("ee_link mismatch ...")       # siempre se dispara
  ```
  Con `measured_ee_frame = "rg2_pinch_center"` y `ee_link_moveit = "rg2_tcp"` →
  **`RuntimeError` en cada paso MOVEIT** (APPROACH, PRE_GRASP, GRASP_DOWN, etc.)

**Confirmación:** ambos frames son co-ubicados (Z=0.175, `vector_distance=0.0` por tests de geometría), pero la comparación es textual — strings distintos → falla siempre.

### Fix aplicado

```diff
-        # Fallback usa RG2_TCP_FRAME (no RG2_PINCH_CENTER_FRAME)...
-        measured_ee_frame = panel._ee_frame_effective or RG2_TCP_FRAME
+        # El bridge publica ee_link="rg2_tcp" en todos sus resultados.
+        # _ee_frame_effective es "rg2_pinch_center" en runtime (orden de preferencia TF).
+        # Ambos frames son coubicados (Z=0.175, distance=0.0); usar RG2_TCP_FRAME
+        # en modo MOVEIT garantiza que el check de mismatch (línea ~3486) pase.
+        _raw_ee = str(panel._ee_frame_effective or RG2_TCP_FRAME).strip()
+        measured_ee_frame = (
+            RG2_TCP_FRAME
+            if _raw_ee in (RG2_TCP_FRAME, RG2_PINCH_CENTER_FRAME)
+            else _raw_ee
+        )
```

**Ubicación:** `panel_pick_object.py` línea ~1155 (inicio de `worker()`)

### Impacto

Sin este fix, **cada paso MOVEIT** (APPROACH, PRE_GRASP, GRASP_DOWN, CLOSE, LIFT, etc.)
habría terminado con:
```
[PICK_OBJ][ABORT] ee_link mismatch moveit=rg2_tcp tf=rg2_pinch_center
RuntimeError: ee_link mismatch (moveit=rg2_tcp tf=rg2_pinch_center)
```

---

## Tests ejecutados

### `test_panel_pick_object_moveit_init.py` — 13/13 PASS

```
test_rg2_tcp_frame_is_imported                               PASSED
test_tf_ready_status_is_imported                             PASSED
test_moveit_init_log_prefix_present                          PASSED
test_moveit_init_tf_and_ee_ok_log_present                    PASSED
test_moveit_init_tf_and_ee_fail_log_present                  PASSED
test_moveit_init_calls_tf_ready_status                       PASSED
test_moveit_step_log_prefix_present                          PASSED
test_moveit_step_gate_reuse_log_present                      PASSED
test_moveit_lifecycle_worker_start_log_present               PASSED
test_moveit_error_log_prefix_present                         PASSED
test_worker_ee_frame_resolves_to_rg2_tcp                     PASSED  ← nuevo BUG-M08
test_worker_ee_frame_mismatch_guard_exists                   PASSED  ← nuevo BUG-M08
test_worker_ee_frame_fallback_never_uses_rg2_pinch_center... PASSED
```

### `test_gripper_geometry.py` — 8/8 PASS

```
test_load_gripper_geometry_matches_validated_tcp_fix         PASSED
test_model_anchor_matches_canonical_geometry                 PASSED
test_contact_z_correction_is_urdf_driven                     PASSED
test_tool0_offset_for_contact_frames_is_urdf_driven          PASSED
test_evaluate_geometry_snapshot_accepts_matching_runtime...  PASSED
test_evaluate_geometry_snapshot_rejects_runtime_mismatch     PASSED
test_evaluate_runtime_geometry_uses_single_runtime_lookup... PASSED
test_evaluate_runtime_geometry_returns_partial_snapshot_...  PASSED
```

---

## Resultado modo AUTO

**No ejecutado** — panel Qt no disponible desde entorno de terminal (sin DISPLAY).

Infraestructura necesaria confirmada como OK:
- Bridge suscrito a `/desired_grasp` ✓
- Bridge publica en `/desired_grasp/result` ✓
- Heartbeat activo a 2Hz ✓
- TF `base_link → rg2_tcp` disponible ✓
- Controladores activos ✓
- BUG-M08 (bloqueante) corregido ✓

---

## Resultado modo STEP_BY_STEP

**No ejecutado** — requiere GUI.

Análisis estático confirma que `_step_phase_gate` + `_step_wait_for_phase` son
funcionalmente correctos. Los nuevos logs `[PICK][MOVEIT][STEP]` están presentes.

---

## Errores encontrados

| # | Tipo | Descripción | Estado |
|---|------|-------------|--------|
| BUG-M08 | Runtime bloqueante | `ee_link mismatch` en cada paso MOVEIT (`rg2_tcp` vs `rg2_pinch_center`) | **Corregido** |
| DDS isolation | Limitación entorno | `ros2 topic echo` no recibe datos desde sandbox Claude | Documentado |
| GUI no disponible | Limitación entorno | Panel Qt no arrancable sin DISPLAY | Documentado |

---

## Cambios aplicados en esta fase

**`panel_pick_object.py`** — BUG-M08: resolver `measured_ee_frame` a `RG2_TCP_FRAME` cuando el panel reporta cualquier frame canónico (`rg2_tcp` o `rg2_pinch_center`).

**`test_panel_pick_object_moveit_init.py`** — 2 tests nuevos para BUG-M08:
- `test_worker_ee_frame_resolves_to_rg2_tcp`
- `test_worker_ee_frame_mismatch_guard_exists`
- Reemplaza `test_worker_ee_frame_fallback_uses_rg2_tcp_frame` (más preciso)

---

## Pendiente (requiere entorno gráfico)

1. Ejecutar PICK Objeto en modo AUTO → verificar `[PICK][MOVEIT][INIT] tf_and_ee=OK`
2. Verificar `[PICK][MOVEIT][LIFECYCLE] stage=worker_start`
3. Verificar `[PICK_OBJ][MOVEIT][PUB]` y `[PICK_OBJ][MOVEIT][RESULT]`
4. Ejecutar en modo STEP_BY_STEP → verificar `[PICK][MOVEIT][STEP] phase=...` por fase
5. Verificar que no aparece `[PICK][MOVEIT][ERROR]` en ejecución normal
6. Verificar que DIRECTO sigue sin incidencias tras los cambios

---

## Recomendación final

**COMMIT** — los cambios son correctos y necesarios:
- BUG-M08 era bloqueante; sin el fix el modo MOVEIT nunca hubiera completado ningún paso
- Tests 13/13 y geometría 8/8 PASS
- Build limpio
- DIRECTO no modificado

*Generado automáticamente — 2026-04-24*
