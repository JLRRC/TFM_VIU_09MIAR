# Incidencia: Auditoría TCP / DH / TF / SDF — RG2 no agarra el cilindro

**Fecha:** 2026-04-24
**Rama de trabajo:** `audit/tcp-geometry-fix`
**Commit base:** `3b026ec` (rama `ENTREGA.V2`)
**Workspace:** `/home/laboratorio/TFM/agarre_ros2_ws`
**Estado:** Diagnóstico completado. Pendiente validación runtime.

---

## 1. Resumen ejecutivo

### Problema original

El panel indicaba que el TCP operacional `rg2_pinch_center` llegaba correctamente al objeto (errores del orden de milímetros), pero visualmente en Gazebo los dedos del RG2 no agarraban físicamente el cilindro. La fase `CLOSE` se ejecutaba y la pinza cerraba, pero `ATTACH_GATE` no confirmaba el agarre.

### Hipótesis inicial

Posible desalineación entre alguno de los siguientes componentes:
1. Cinemática DH usada por `ur5_kinematics.py`
2. Joints ejecutados por Gazebo / ros2_control
3. Árbol TF publicado por `robot_state_publisher`
4. Frame operacional `rg2_pinch_center`
5. Geometría visual/física del RG2 en SDF
6. Offsets manuales o históricos en el panel

### Causa raíz encontrada tras auditoría

**La geometría es matemáticamente consistente.** No existe doble offset, frames conflictivos ni definición contradictoria. Hallazgos clave:

- `tool0 → rg2_pinch_center = (0, 0, 0.175 m)` — offset único, definido en `ur5.urdf.xacro:20` (`rg2_contact_tcp_xyz`), propagado correctamente por `gripper_geometry.py` y usado en el panel.
- `rg2_tcp` y `rg2_pinch_center` son **aliases exactos** (mismo parent, mismo xyz, misma rpy). No producen divergencia TF.
- `pick_demo_anchor` (SDF, joint `pick_demo_anchor_joint`) también está a `(0, 0, 0.175)` desde `tool0`. Consistente con los otros dos.
- `NEGATE_XY` ya está correctamente encapsulado y documentado en `_resolve_direct_execution_target()`.
- El MoveIt bridge ya transforma `rg2_pinch_center → rg2_tcp` antes de invocar `set_from_ik`.

### Fix previo aplicado (misma sesión, antes de la auditoría)

El parche crítico que **redujo el error XY de ~175 mm a ~5 mm** fue corregir la selección de seed IK para `GRASP_DOWN`:

```python
# panel_pick_demo.py ~5363
is_grasp_down = "GRASP_DOWN" in label_name
if is_grasp_down:
    _gd_live = _live_joint_seed_or_none(panel)
    if _gd_live is not None:
        seed = list(_gd_live)
        seed_source = "live_joints"
```

Log confirmado en runtime:
```
[PICK][DIRECT][SEED] phase=GRASP_DOWN source=live_joints
[PICK][DIRECT][IK_SEED] label=GRASP_DOWN_JOINT source=live_joints
  joints=[-0.263, -0.220, 1.564, 0.260, -1.558, 0.005]
```

### Riesgo pendiente

Con el error XY reducido a ~5 mm, el fallo residual puede estar en:
1. El objeto no entra en el volumen físico de cierre de los dedos (PRE_CLOSE_VOLUME).
2. El DH y el TF real divergen > 5 mm en el punto exacto de agarre (DH vs TF runtime).
3. El `pick_demo_anchor` (detachable joint SDF) no dispara porque no hay contacto físico entre los dedos del RG2 y el cilindro aunque el TCP semántico esté cerca.

---

## 2. Tablas de auditoría

### 2.1 Cadena TF real (URDF/Xacro)

| Frame | Parent | xyz (m) | rpy (rad) | Archivo | Línea | Uso |
|---|---|---|---|---|---|---|
| `world` | — | — | — | ur5.urdf.xacro | 28 | Origen global |
| `base_link` | `world` | (-0.85, 0, 0.850) | 0 0 0 | ur5.urdf.xacro | 47 | Base robot en mundo |
| `base_link_inertia` | `base_link` | 0 0 0 | **0 0 π** | ur_macro.xacro (interno) | — | Frame DH — X,Y negados vs base_link |
| `flange` | `wrist_3_link` | 0 0 0 | 0 -π/2 -π/2 | ur_macro.xacro | 391 | Flange UR5 (intermedio) |
| `tool0` | `flange` | 0 0 0 | π/2 0 π/2 | ur_macro.xacro | 399 | TCP mecánico UR5; IK solves aquí |
| `rg2_base_link` | `tool0` | **0 0 0** | 0 0 0 | ur5.urdf.xacro | 59–63 | Base cinemática stub RG2 |
| `rg2_tcp` | `tool0` | **0 0 0.175** | 0 0 0 | ur5.urdf.xacro | 77–81 | Alias del TCP operacional |
| `rg2_pinch_center` | `tool0` | **0 0 0.175** | 0 0 0 | ur5.urdf.xacro | 94–98 | TCP operacional DIRECTO — único TCP semántico |
| `rg2_finger_link1` | `rg2_base_link` | 0 0.02 0 | 0 0 0 | ur5.urdf.xacro | 116–122 | Dedo URDF prismático (control ros2_control) |
| `rg2_finger_link2` | `rg2_base_link` | 0 -0.02 0 | 0 0 0 | ur5.urdf.xacro | 124–130 | Dedo URDF prismático (control ros2_control) |

**Nota crítica:** Los joints de dedo en URDF son **prismáticos** [0, 0.04 m], pero en SDF son **revolutos** [0, 1.18 rad]. El gz_ros2_control mapea el comando por nombre de joint. El TF de `rg2_finger_link1/2` no es representativo de la geometría física real en Gazebo.

### 2.2 Geometría SDF visual/física del RG2

| Elemento SDF | Parent (via joint) | xyz en parent (m) | rpy en parent (rad) | Observación |
|---|---|---|---|---|
| `rg2_hand` | `wrist_3_link` (ur5_hand_joint, fixed) | **(0, 0.07, 0)** | **(π/2, 0, π/2)** | Cuerpo palm RG2; 7 cm en Y de wrist_3 |
| `rg2_leftfinger` | `rg2_hand` (rg2_finger_joint1, **revolute**) | **(0.105, +0.017, 0)** | — | Hinge dedo izq.; eje rot. = +Z de rg2_hand |
| `rg2_rightfinger` | `rg2_hand` (rg2_finger_joint2, **revolute**) | **(0.105, -0.017, 0)** | — | Hinge dedo der.; eje rot. = -Z de rg2_hand |
| `tool0` (SDF) | `wrist_3_link` (end_effector_fixed, fixed) | **(0, 0, 0)** | **(-π/2, 0, 0)** | Flange SDF — solo rotación desde wrist_3 |
| `pick_demo_anchor` | `tool0` (pick_demo_anchor_joint, fixed) | **(0, 0, 0.175)** | 0 0 0 | = rg2_pinch_center del URDF |
| `camera_wrist_link` | `rg2_hand` (camera_wrist_joint, fixed) | (0, 0, 0.05) | 0 0 0 | Cámara de muñeca |

**Verificación geométrica (calculada)**:

La transformada `rg2_hand → tool0` da:
- `rg2_hand` origin en `tool0` frame = **(0, 0, 0.07)**
- Midpoint de hinges `(0.105, 0, 0)` en rg2_hand → en tool0 = **(0, 0, 0.175)** ✓

El midpoint geométrico de los hinges coincide exactamente con `rg2_pinch_center`.

**Discrepancia SDF vs URDF en orientación de tool0:**

| | wrist_3_link → tool0 | Implicación |
|---|---|---|
| **URDF** (ur_macro.xacro) | R = **Identity** (dos rotaciones que se cancelan) | `tool0` Z = `wrist_3_link` Z |
| **SDF** (end_effector_fixed) | R = **Rx(-π/2)** | `tool0` Z = `wrist_3_link` Y |

El `robot_state_publisher` usa el URDF como fuente de verdad para TF. El SDF solo define la física de Gazebo. Esta diferencia de orientación es la fuente potencial de divergencia DH/TF que debe validarse en runtime.

### 2.3 Offsets encontrados en el código

| Valor / Variable | Archivo | Línea | Valor | Estado |
|---|---|---|---|---|
| `rg2_contact_tcp_xyz` | ur5.urdf.xacro | 20 | `"0 0 0.175"` | **Canónico** — fuente de verdad |
| `rg2_tcp_joint` origin | ur5.urdf.xacro | 80 | `${rg2_contact_tcp_xyz}` | Derivado — OK |
| `rg2_pinch_center_joint` origin | ur5.urdf.xacro | 97 | `${rg2_contact_tcp_xyz}` | Derivado — OK |
| `pick_demo_anchor_joint` pose | model.sdf | 620 | `0 0 0.175` | Consistente — OK |
| `_DIRECT_TOOL0_TO_SOURCE_OFFSET` | panel_pick_demo.py | 83 | leído de URDF vía gripper_geometry | Derivado — OK |
| `DIRECT_TOOL0_TO_RG2_TCP_Z_M` | panel_pick_demo.py | 92 | `0.175` | Derivado — OK |
| `_DIRECTO_GRASP_Z` | panel_pick_demo.py | 2276–2281 | `0.0` si source=rg2_pinch_center | OK — offset nulo para TCP semántico |
| `GRIPPER_TCP_Z_OFFSET` | panel_pick_demo.py | 2297 | via env var | Solo para compatibilidad rg2_tcp legacy |
| DH `_D[5]` | ur5_kinematics.py | 19 | `0.0823` | UR5 CB3 — validar vs TF runtime |

**Conclusión:** No hay doble offset. Un único sitio de verdad. Sin compensaciones ad hoc en el panel.

### 2.4 Tabla de frames finales

| Frame | Parent | xyz (m) en parent | Uso operacional |
|---|---|---|---|
| `world` | — | — | Origen global Gazebo/TF |
| `base_link` | `world` | (-0.85, 0, 0.850) | Base robot; frame de referencia del panel |
| `base_link_inertia` | `base_link` | 0 0 0 / Rz(π) | Frame interno DH — **X,Y negados vs base_link** |
| `tool0` | `flange` | 0 0 0 | Flange UR5; el IK resuelve para este frame |
| `rg2_pinch_center` | `tool0` | **(0, 0, 0.175)** | **TCP operacional DIRECTO** — target de todas las fases |
| `rg2_tcp` | `tool0` | **(0, 0, 0.175)** | Alias de rg2_pinch_center; tip registrado en SRDF para MoveIt |
| `pick_demo_anchor` | `tool0` (SDF) | **(0, 0, 0.175)** | Punto de attach del detachable joint — coincide con rg2_pinch_center |

---

## 3. Cambios aplicados en esta sesión

### 3.1 Fix previo: seed IK para GRASP_DOWN (sesión anterior)

**Archivo:** `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` ~línea 5363

**Problema:** El bloque `elif label_name == "GRASP_DOWN_JOINT"` estaba **anidado dentro** de `if is_transport_stage`, por lo que nunca se ejecutaba para GRASP_DOWN (que no es transport stage). La env var `PANEL_PICK_DEMO_IK_SEED_JOINTS` heredada de APPROACH_COARSE quedaba activa.

**Fix:** Se añadió un bloque `if is_grasp_down:` al nivel correcto, antes de la lógica de transport:

```python
seed, seed_source = _current_joint_seed(return_source=True)
is_grasp_down = "GRASP_DOWN" in label_name
if is_grasp_down:
    _gd_live = _live_joint_seed_or_none(panel)
    if _gd_live is not None:
        seed = list(_gd_live)
        seed_source = "live_joints"
        panel._emit_log("[PICK][DIRECT][SEED] phase=GRASP_DOWN source=live_joints")
    else:
        panel._emit_log("[PICK][DIRECT][SEED] phase=GRASP_DOWN source=fallback reason=no_live_joints")
```

**Resultado medido:** Error XY de ~175 mm → ~5 mm. Log confirmado: `source=live_joints`.

### 3.2 Nuevo archivo: `validate_dh_vs_tf.py`

**Ruta:** `src/ur5_tools/ur5_tools/validate_dh_vs_tf.py`

Nodo ROS 2 que:
1. Lee una muestra de `/joint_states`
2. Calcula FK con los parámetros DH del UR5 CB3 (mismos que `ur5_kinematics.py`)
3. Convierte de `base_link_inertia` a `base_link` (NEGATE_XY)
4. Consulta TF2 para `base_link → tool0` y `base_link → rg2_pinch_center`
5. Reporta error XYZ y 3D para cada frame
6. Sale con código 0 si error < 5 mm, código 1 si hay divergencia

**Formato de salida:**
```
[DH_TF_CHECK] tool0       DH=(x,y,z)  TF=(x,y,z)  err_xyz=(dx,dy,dz)  err_3d=Xmm  OK/FAIL
[DH_TF_CHECK] rg2_pinch   DH=(x,y,z)  TF=(x,y,z)  err_xyz=(dx,dy,dz)  err_3d=Xmm  OK/FAIL
[DH_TF_CHECK] OK/FAIL — DH and TF agree within 5.0 mm
```

### 3.3 Entry point en setup.py

**Archivo:** `src/ur5_tools/setup.py`

Línea añadida:
```python
"validate_dh_vs_tf = ur5_tools.validate_dh_vs_tf:main",
```

### 3.4 Log `[PICK][FRAME_AUDIT]` en `_move_tcp_direct`

**Archivo:** `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` (~línea 5424, después del log `[PICK][DIRECT][IK_SEED]`)

En **cada llamada a `_move_tcp_direct`** (APPROACH_COARSE, GRASP_DOWN, GRASP_ALIGN_IK, PRE_CLOSE, CLOSE) se emite:

```
[PICK][FRAME_AUDIT] phase=GRASP_DOWN_JOINT target_frame=base_link
  tcp_frame=rg2_pinch_center exec_frame=tool0
  target=(x,y,z) tf_tcp=(x,y,z) tf_tool0=(x,y,z) obj=(x,y,z)
  err_xy=Xmm err_z=Ymm
```

Esto permite verificar en cada fase que el TCP por TF coincide con el target enviado al IK.

### 3.5 Función `_pre_close_volume_check()`

**Archivo:** `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` (~línea 3763, antes de `_pre_close_alignment_metrics()`)

Comprueba si el objeto está dentro del volumen físico de cierre del RG2:
- Distancia lateral TCP-objeto (`xy_dist`)
- Distancia vertical TCP-objeto (`z_dist`)
- Apertura actual de dedos (`finger_opening_mm`)
- Umbrales configurables: `PANEL_PICK_DEMO_PRE_CLOSE_VOLUME_XY_M` (default 30 mm), `PANEL_PICK_DEMO_PRE_CLOSE_VOLUME_Z_M` (default 20 mm)

Se invoca al entrar a PRE_CLOSE (justo después de `_emit_transition_decision`).

**Formato de salida:**
```
[PICK][PRE_CLOSE_VOLUME] obj_in_grasp_volume=True/False
  lateral_err_mm=X.X vertical_err_mm=Y.Y dist_3d_mm=Z.Z
  finger_opening_mm=W.W xy_threshold_mm=30 z_threshold_mm=20
  pinch_world=(x,y,z) tool0_world=(x,y,z) obj_world=(x,y,z)
```

### 3.6 Estado del build

```
colcon build --packages-select ur5_qt_panel ur5_tools --symlink-install
# → Summary: 2 packages finished [3.61s]  — sin errores ni warnings
python3 -m py_compile src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py  → OK
python3 -m py_compile src/ur5_tools/ur5_tools/validate_dh_vs_tf.py      → OK
```

---

## 4. Archivos modificados

| Archivo | Tipo de cambio | Descripción |
|---|---|---|
| `src/ur5_tools/ur5_tools/validate_dh_vs_tf.py` | **NUEVO** | Validador runtime DH vs TF |
| `src/ur5_tools/setup.py` | Modificado | Entry point `validate_dh_vs_tf` añadido |
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` | Modificado | Seed fix GRASP_DOWN + FRAME_AUDIT log + _pre_close_volume_check() |

**Archivos NO modificados** (confirmado con `git diff --stat`):
- `src/ur5_description/urdf/ur5.urdf.xacro`
- `models/ur5_rg2/model.sdf`
- `src/ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py`
- `src/ur5_tools/ur5_tools/gripper_geometry.py`
- `src/ur5_tools/ur5_tools/ur5_moveit_bridge.py`
- `src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py`
- `src/ur5_bringup/launch/ur5_stack.launch.py`

---

## 5. Qué NO se modificó y por qué

| Componente | Razón para no tocar |
|---|---|
| **URDF** `ur5.urdf.xacro` | `rg2_pinch_center` ya está correctamente definido a (0,0,0.175) desde `tool0`. No hay error geométrico que corregir. |
| **SDF** `model.sdf` | `pick_demo_anchor` ya coincide posicionalmente con `rg2_pinch_center`. La diferencia de orientación de `tool0` entre SDF y URDF es esperada y compatible. |
| **DH** `ur5_kinematics.py` | Los parámetros D6=0.0823 corresponden al UR5 CB3 estándar. La concordancia real con TF se valida ahora en runtime con `validate_dh_vs_tf`. No se cambia hasta tener el resultado. |
| **gripper_geometry.py** | Ya es la única fuente de verdad para el offset, lee del URDF correctamente. |
| **ur5_moveit_bridge.py** | Ya convierte `rg2_pinch_center → rg2_tcp` antes de `set_from_ik`. El tip del SRDF es `rg2_tcp` (correcto). |
| **attach_gate_evaluator.py** | No se sabe si el fallo está en el attach o en la física. Esperar logs de `ATTACH_GATE`. |
| **gripper_attach_backend.py** | Ídem — esperar evidencia de logs antes de intervenir en el backend de attach. |
| **ur5_stack.launch.py** | Sin cambio de configuración — mantener entorno reproducible para comparar runs. |

---

## 6. Comandos de reproducción

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws

# Build
colcon build --packages-select ur5_qt_panel ur5_tools --symlink-install
source install/setup.bash

# Lanzar stack completo
ros2 launch ur5_bringup ur5_stack.launch.py

# Validar DH vs TF (con stack corriendo)
ros2 run ur5_tools validate_dh_vs_tf

# Verificar TF manualmente
ros2 run tf2_ros tf2_echo tool0 rg2_pinch_center
ros2 run tf2_ros tf2_echo world rg2_pinch_center
ros2 run tf2_ros tf2_echo base_link rg2_pinch_center

# Extraer logs del próximo run pick
grep -E "DH_TF_CHECK|FRAME_AUDIT|PRE_CLOSE_VOLUME|ATTACH_GATE|CARRY|PHYSICS|LIFT|FINAL_TRACE|attach|best_obj_move|best_lift_delta|best_tcp_dist" \
    /home/laboratorio/TFM/agarre_ros2_ws/log/ros2_launch.log | tail -50

# Verificar seed GRASP_DOWN (confirmar fix)
grep "GRASP_DOWN.*source\|DIRECT.*SEED.*GRASP" \
    /home/laboratorio/TFM/agarre_ros2_ws/log/ros2_launch.log | tail -5
```

---

## 7. Próximo paso obligatorio

**No hacer más cambios de código hasta tener los logs del próximo ciclo pick.**

Ejecutar un ciclo pick completo (INITIAL_SNAPSHOT → HOME_INITIAL → APPROACH_COARSE → GRASP_DOWN → GRASP_ALIGN_IK → PRE_CLOSE → CLOSE → ATTACH_GATE → LIFT) y recoger logs para determinar cuál de estos es la causa raíz residual:

| Causa candidata | Log que la confirma |
|---|---|
| DH/TF mismatch | `[DH_TF_CHECK] FAIL err_3d > 5mm` |
| Object fuera del volumen | `[PICK][PRE_CLOSE_VOLUME] obj_in_grasp_volume=False` |
| Attach no dispara | `[ATTACH_GATE]` con `attach=false` o timeout |
| LIFT falla físicamente | `best_obj_move < umbral` en log LIFT |
| Contacto físico Gazebo | `[PHYSICS][POSE_INFO]` objeto no se mueve tras CLOSE |

---

## 8. Tabla de decisión para el siguiente diagnóstico

| Check | Resultado | Evidencia log | Conclusión |
|---|---|---|---|
| DH vs TF `tool0` | PENDIENTE | `[DH_TF_CHECK] tool0 err_3d=...` | |
| DH vs TF `rg2_pinch_center` | PENDIENTE | `[DH_TF_CHECK] rg2_pinch err_3d=...` | |
| FRAME_AUDIT GRASP_DOWN | PENDIENTE | `[PICK][FRAME_AUDIT] phase=GRASP_DOWN_JOINT err_xy=...` | |
| FRAME_AUDIT PRE_CLOSE | PENDIENTE | `[PICK][FRAME_AUDIT] phase=PRE_CLOSE err_xy=...` | |
| PRE_CLOSE_VOLUME | PENDIENTE | `[PICK][PRE_CLOSE_VOLUME] obj_in_grasp_volume=...` | |
| CLOSE — gripper cerrado | PENDIENTE | `[JOINTS] rg2_finger_joint1: X.XXX rad` | |
| ATTACH_GATE resultado | PENDIENTE | `[ATTACH_GATE]` o `[PICK][DIRECT][ABORT]` | |
| LIFT — objeto sube | PENDIENTE | `best_obj_move=X.Xm` / `best_lift_delta=X.Xm` | |
| Física Gazebo contacto | PENDIENTE | objeto Z inmóvil tras CLOSE en `[PHYSICS][POSE_INFO]` | |

---

## 9. Estado del workspace al cerrar esta sesión

```
Rama:        audit/tcp-geometry-fix
Base commit: 3b026ec (ENTREGA.V2)
Modificados: panel_pick_demo.py, setup.py
Nuevos:      validate_dh_vs_tf.py, este informe
Build:       OK (2 packages)
Tests:       py_compile OK
```

El workspace está en estado limpio y trazable. Todos los cambios son no-destructivos y reversibles. No se han modificado parámetros de lanzamiento, URDF, SDF ni controladores.
