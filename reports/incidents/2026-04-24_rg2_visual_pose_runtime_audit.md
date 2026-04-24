# Auditoría runtime: RG2 visual pose — inspect_rg2_visual_pose
**Fecha:** 2026-04-24  
**Rama:** audit/tcp-geometry-fix  
**Estado:** COMPLETADO — diagnóstico técnico completo

---

## 1. Entidades detectadas en `/world/ur5_mesa_objetos/pose/info`

**Fuente:** `GZ_PARTITION=ur5pro_manual_1777057394` → `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz topic -e -t /world/ur5_mesa_objetos/pose/info --json-output`

**Total:** 70+ entidades. Extracto relevante (ordenado):

```
bandeja_deposito   base_link    base_link_visual   box_blue    box_green
box_lightblue      box_red      box_yellow         drop_anchor  forearm_link
ground_plane       mesa_pro     mesa_robot         pick_demo    pick_demo_anchor
rg2_hand           rg2_hand_visual   rg2_leftfinger   rg2_leftfinger_visual
rg2_rightfinger    rg2_rightfinger_visual   shoulder_link   sun_world
tool0              upper_arm_link   ur5_rg2   wrist_1_link  wrist_2_link  wrist_3_link
...
```

### Presencia de entidades clave

| Entidad | Presente en pose/info | Notas |
|---|---|---|
| `rg2_hand` | **SÍ** | Posición física del cuerpo del gripper en Gazebo |
| `rg2_leftfinger` | **SÍ** | Dedo izquierdo SDF (revolute, ≠ URDF prismatic) |
| `rg2_rightfinger` | **SÍ** | Dedo derecho SDF |
| `pick_demo` | **SÍ** | Objeto target — entidad independiente |
| `pick_demo_anchor` | **SÍ** | Anchor cinematográfico — correctamente separado de `pick_demo` |

**Verificación `key_check` en inspect_rg2_visual_pose:**
```
key_check  pick_demo: PRESENT
key_check  rg2_hand: PRESENT
key_check  rg2_leftfinger: PRESENT
key_check  rg2_rightfinger: PRESENT
```

---

## 2. Muestra real de `inspect_rg2_visual_pose`

**Comando:**
```bash
GZ_PARTITION=ur5pro_manual_1777057394 ros2 run ur5_tools gz_pose_bridge \
    --ros-args -p world_name:=ur5_mesa_objetos -p startup_timeout_sec:=60.0 &
ros2 run ur5_tools inspect_rg2_visual_pose \
    --ros-args -p once:=true -p object_name:=pick_demo
```

**Resultado completo (estado: robot NO sobre el objeto):**
```
[RG2_VISUAL_POSE] ── TF FRAMES (robot_state_publisher / URDF) ──
[RG2_VISUAL_POSE] tool0_tf_world=N/A q=N/A
[RG2_VISUAL_POSE] pinch_tf_world=N/A q=N/A
[RG2_VISUAL_POSE] rg2_tcp_tf_world=N/A
[RG2_VISUAL_POSE] rg2_base_link_tf_world=N/A
[RG2_VISUAL_POSE] rg2_finger_link1_tf_world=N/A
[RG2_VISUAL_POSE] rg2_finger_link2_tf_world=N/A
[RG2_VISUAL_POSE] urdf_finger_midpoint_world=N/A

[RG2_VISUAL_POSE] ── GAZEBO VISUAL (SDF / gz_pose_bridge) ──
[RG2_VISUAL_POSE] rg2_hand_gz_world=(0.4132, 0.0025, 0.5196) q=(-0.5587,-0.4405,0.5661,-0.4163)
[RG2_VISUAL_POSE] leftfinger_gz_world=(0.4266, 0.0001, 0.4141)
[RG2_VISUAL_POSE] rightfinger_gz_world=(0.3938, 0.0092, 0.4153)
[RG2_VISUAL_POSE] visual_finger_midpoint_world=(0.4102, 0.0047, 0.4147)

[RG2_VISUAL_POSE] ── OBJECT ──
[RG2_VISUAL_POSE] object_world=(-0.4131, -0.0004, 0.8756) object_name=pick_demo

[RG2_VISUAL_POSE] ── DISTANCES ──
[RG2_VISUAL_POSE] dist_pinch_to_obj=N/A                    ← pinch TF no disponible externamente
[RG2_VISUAL_POSE] dist_gz_visual_mid_to_obj=0.9435m        ← robot lejos del objeto (estado reposo)
[RG2_VISUAL_POSE] dist_urdf_finger_mid_to_obj=N/A
[RG2_VISUAL_POSE] dist_pinch_to_gz_visual_mid=N/A
[RG2_VISUAL_POSE] dist_pinch_to_urdf_finger_mid=N/A
[RG2_VISUAL_POSE] dist_tool0_to_pinch=N/A
[RG2_VISUAL_POSE] dist_rg2_hand_to_pinch=N/A

[RG2_VISUAL_POSE] ── JOINT STATE ──
[RG2_VISUAL_POSE] rg2_finger_joint1=-0.0000
[RG2_VISUAL_POSE] rg2_finger_joint2=0.0000
[RG2_VISUAL_POSE] finger_opening_sum=0.0000              ← gripper completamente cerrado
[RG2_VISUAL_POSE] joint_state_age=0.01s

[RG2_VISUAL_POSE] ── ORIENTATION (FASE 5) ──
[RG2_VISUAL_POSE] rot_err_pinch_vs_hand_deg=N/A
[RG2_VISUAL_POSE] orientation_check=UNAVAILABLE reason=pinch_tf_quat_missing

[RG2_VISUAL_POSE] ── DATA SOURCES ──
[RG2_VISUAL_POSE] gz_pose_info_msgs=163
[RG2_VISUAL_POSE] joint_state_msgs=65

[RG2_VISUAL_POSE] ── VERDICT ──
[RG2_VISUAL_POSE] verdict=UNKNOWN reason=dist_pinch_gz_mid_unavailable
```

**Estado de `finger_opening_sum`:** `0.0000` — gripper completamente cerrado en el instante de la muestra (robot en posición de reposo/hold tras un ciclo previo).

---

## 3. Estado de `gz_pose_bridge` — diagnóstico GZ_PARTITION

### Síntoma
`/world/ur5_mesa_objetos/pose/info` → Publisher count: 0 (al inspeccionar desde terminal externa)

### Causa raíz identificada
El stack lanza `gz_pose_bridge` con `condition=IfCondition(launch_gazebo)`. El bridge usa `subprocess.Popen(["gz", "topic", "-e", ...])` internamente. Para conectar con la instancia Gazebo en curso, necesita que `GZ_PARTITION` coincida.

El partition de la sesión actual: `GZ_PARTITION=ur5pro_manual_1777057394` (generado en `lanzar_panelv2.sh` como `ur5pro_manual_$(date +%s)`).

**Verificación de conectividad manual:**
```bash
GZ_PARTITION=ur5pro_manual_1777057394 gz topic -l | grep pose
# → /world/ur5_mesa_objetos/pose/info  ✓ presente
```

**Bridge manual arrancando correctamente:**
```bash
GZ_PARTITION=ur5pro_manual_1777057394 ros2 run ur5_tools gz_pose_bridge \
    --ros-args -p world_name:=ur5_mesa_objetos
# → [gz_pose_bridge]: GzPoseBridge active ✓
```

**Estado actual del stack:** `gz_pose_bridge` no aparece en `ros2 node list`. Lo más probable es que el proceso haya fallado en el startup del stack (timeout=5s antes de que Gazebo estuviese listo y sirviendo el topic), y el `gz_pose_guard` no disparó shutdown porque la condición puede no haberse registrado correctamente en ese ciclo. El panel mantiene **caché estantada** de los datos que el bridge publicó mientras estuvo vivo.

Nota adicional: el stack usa un `parameter_bridge` de `ros_gz_bridge` que publica en `/world/ur5_mesa_objetos/pose/info_raw` (NOT el topic que consume el panel). Esa bridge también está caída.

---

## 4. Verificación explícita de TF externo

### Tabla de resultados

| Check | Resultado | Evidencia | Conclusión |
|---|---|---|---|
| `ROS_DOMAIN_ID` | [no set] = 0 (default) | `echo $ROS_DOMAIN_ID` | Mismo dominio que el stack |
| `GZ_PARTITION` | `ur5pro_manual_1777057394` | `/proc/2431826/environ` (Gazebo PID) | Correcto para la sesión |
| `/tf` existe | Sí, 0 publishers, 17 subscribers | `ros2 topic info /tf` | El topic existe pero nadie publica |
| `/tf_static` existe | Sí, 0 publishers, 17 subscribers | `ros2 topic info /tf_static` | Idem — TRANSIENT_LOCAL preserva historial DDS |
| `robot_state_publisher` activo | **NO** | `ps -p 2431825` → DEAD; `ps aux | grep robot_state` vacío | RSP arrancó (PID 2431825 en log), luego murió |
| `/clock` activo | **NO** | `ros2 topic info /clock` → 0 publishers | Sin reloj Gazebo bridgeado activo |
| `tf2_echo world→tool0` | **TIMEOUT** (exit 143) | `timeout 5 ros2 run tf2_ros tf2_echo world tool0` | TF externo inaccesible |
| `tf2_echo world→rg2_pinch_center` | **TIMEOUT** | `timeout 5 ros2 run tf2_ros tf2_echo world rg2_pinch_center` | Idem |
| `tf2_echo base_link→rg2_pinch_center` | **TIMEOUT** | `timeout 5 ros2 run tf2_ros tf2_echo base_link rg2_pinch_center` | Idem |
| Panel `tcp_live_source=tf2:ok` | **SÍ** | `ros2_launch.log`: múltiples entradas 21:09+ | Panel accede TF — ver explicación |

### Diagnóstico TF: Conclusión B + C

**B — TF externo no funciona** (problema de entorno):
- `robot_state_publisher` (PID 2431825) arrancó a las 21:03:59 con la sesión, luego murió sin registro de muerte en el log del stack
- `/tf` tiene 0 publishers → ningún nodo externo recibe datos dinámicos de articulaciones
- `tf2_echo` falla en todos los pares probados
- Causa: RSP muerto, stack en estado degradado

**C — Panel accede TF via mecanismo no disponible externamente:**
- El `TfHelper` del panel (`panel_tf.py`) se inicializó cuando RSP estaba vivo
- Suscripción a `/tf_static` (TRANSIENT_LOCAL + QoS RELIABLE) → historial DDS persiste en el broker aunque RSP haya muerto → transforms estáticos (fixed joints: `tool0 → rg2_pinch_center`, `world → base_link` vía `world_tf_publisher`) siguen disponibles en el buffer
- `lookup_transform(target, source, rclpy.time.Time())` usa `Time()` = 0 → "último dato disponible sin restricción temporal" → no falla si el transform estático está en buffer aunque tenga horas de antigüedad
- Los transforms dinámicos (articulaciones del brazo) de `/tf` se gestionan con caché de 10s: con RSP muerto, estos expirarían; la posición que muestra el panel puede estar stale, o coincidir con el último estado real si el brazo no ha movido significativamente

**Implicación para `inspect_rg2_visual_pose`:** La herramienta externa instancia su propio nodo y buffer TF desde cero → recibe `/tf_static` TRANSIENT_LOCAL pero no recibe datos dinámicos de `/tf` → TF frames N/A para links dinámicos. **Fix requerido: arrancar RSP antes de usar la herramienta, o integrar el inspect en el mismo proceso que ya tiene el buffer poblado (el panel).**

---

## 5. Fix aplicado: mensaje `orientation_check`

**Síntoma:** El log mostraba `orientation_check=UNAVAILABLE reason=rg2_hand_gz_quat_missing` aunque el quaternion de `rg2_hand` SÍ llegaba de Gazebo. La causa real era `pinch_q` (TF) = None.

**Fix en `inspect_rg2_visual_pose.py` (`_print_report`):**
```python
# Antes (mensaje hardcoded incorrecto):
"orientation_check=UNAVAILABLE reason=rg2_hand_gz_quat_missing"

# Después (distingue qué falta):
_pq = kw.get("pinch_q")
_hq = kw.get("rg2_hand_gz_q")
if _pq is None and _hq is None:
    _or = "pinch_tf_and_hand_gz_quat_missing"
elif _pq is None:
    _or = "pinch_tf_quat_missing"
else:
    _or = "rg2_hand_gz_quat_missing"
```

**Verificación:** `python3 -m py_compile inspect_rg2_visual_pose.py` → OK

---

## 6. Archivos modificados

| Archivo | Tipo de cambio |
|---|---|
| `src/ur5_tools/ur5_tools/inspect_rg2_visual_pose.py` | NUEVO — nodo de diagnóstico standalone |
| `src/ur5_tools/setup.py` | Añadido entry_point `inspect_rg2_visual_pose` |
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` | Añadidos logs `[PICK][RG2_VISUAL_AUDIT]` en 5 fases |

---

## 7. git diff --stat / git status --short

```
git diff --stat HEAD:
 agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py | 144 +++++++++++++++++++++
 agarre_ros2_ws/src/ur5_tools/setup.py                           |   1 +
 2 files changed, 145 insertions(+)

git status --short:
 M agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
 M agarre_ros2_ws/src/ur5_tools/setup.py
?? agarre_ros2_ws/src/ur5_tools/ur5_tools/inspect_rg2_visual_pose.py
```

---

## 8. Comandos de reproducción

### Requisito previo: GZ_PARTITION correcto
```bash
cat /home/laboratorio/TFM/agarre_ros2_ws/log/gz_partition.txt
# → ur5pro_manual_XXXXXXXXXX
export GZ_PARTITION=$(cat /home/laboratorio/TFM/agarre_ros2_ws/log/gz_partition.txt)
```

### Verificar entidades Gazebo directamente (sin ROS)
```bash
source /opt/ros/jazzy/setup.bash
GZ_PARTITION=$GZ_PARTITION /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz topic -l | grep pose
GZ_PARTITION=$GZ_PARTITION /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz topic -e \
    -t /world/ur5_mesa_objetos/pose/info --json-output 2>&1 | head -100
```

### Arrancar gz_pose_bridge manualmente (necesario para que pose/info tenga publisher)
```bash
source /opt/ros/jazzy/setup.bash
source /home/laboratorio/TFM/agarre_ros2_ws/install/setup.bash
export GZ_PARTITION=$GZ_PARTITION
ros2 run ur5_tools gz_pose_bridge \
    --ros-args -p world_name:=ur5_mesa_objetos -p startup_timeout_sec:=60.0 &
BRIDGE_PID=$!
```

### Ejecutar inspect_rg2_visual_pose con bridge activo
```bash
ros2 run ur5_tools inspect_rg2_visual_pose \
    --ros-args -p once:=true -p object_name:=pick_demo \
    -p rate_hz:=2.0 \
    -p max_pinch_visual_mid_dist_m:=0.050 \
    -p max_visual_mid_obj_dist_m:=0.100
kill $BRIDGE_PID
```

### Verificar TF externo (confirmar estado)
```bash
ros2 topic info /tf --verbose | head -5
ros2 topic info /tf_static --verbose | head -5
timeout 5 ros2 run tf2_ros tf2_echo world rg2_pinch_center
timeout 5 ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
```

---

## 9. Experimento pendiente — Pick hasta PRE_CLOSE con logs de auditoría

### Objetivo
Capturar en un ciclo de pick real los valores en cada fase para `inspect_rg2_visual_pose` y `[PICK][RG2_VISUAL_AUDIT]`.

### Valores objetivo en PRE_CLOSE
- `dist_pinch_to_obj`
- `dist_gz_visual_mid_to_obj`
- `dist_pinch_to_gz_visual_mid`
- `obj_local_in_gripper`
- `rot_err_pinch_vs_hand_deg`

### Comandos de captura
```bash
# 1. Arranca gz_pose_bridge (necesario para inspect)
export GZ_PARTITION=$(cat /home/laboratorio/TFM/agarre_ros2_ws/log/gz_partition.txt)
source /opt/ros/jazzy/setup.bash && source /home/laboratorio/TFM/agarre_ros2_ws/install/setup.bash
ros2 run ur5_tools gz_pose_bridge \
    --ros-args -p world_name:=ur5_mesa_objetos -p startup_timeout_sec:=120.0 &
BRIDGE_PID=$!

# 2. Lanza inspect en modo continuo (rate 5Hz) en paralelo
ros2 run ur5_tools inspect_rg2_visual_pose \
    --ros-args -p once:=false -p rate_hz:=5.0 -p object_name:=pick_demo \
    > /tmp/inspect_rg2_run.log 2>&1 &
INSPECT_PID=$!

# 3. Lanza el pick desde el panel (o desde terminal si tienes el servicio expuesto)
# El pick se ejecuta desde el panel con el botón "DIRECTO" o "PICK"
# Captura el log del stack que tiene los [PICK][RG2_VISUAL_AUDIT] logs
tail -f /home/laboratorio/TFM/agarre_ros2_ws/log/ros2_launch.log | \
    grep -E "\[PICK\]\[RG2_VISUAL_AUDIT\]|\[RG2_VISUAL_POSE\]|\[PICK\]\[FRAME_AUDIT\]|\[PICK\]\[PRE_CLOSE_VOLUME\]|APPROACH_COARSE|GRASP_DOWN_JOINT|GRASP_ALIGN_IK|PRE_CLOSE"

# 4. Al finalizar
kill $BRIDGE_PID $INSPECT_PID 2>/dev/null
cat /tmp/inspect_rg2_run.log | grep -E "PRE_CLOSE|GRASP_DOWN|APPROACH_COARSE"
```

### Filtro para el log del panel (run completo hasta PRE_CLOSE)
```bash
grep -E "\[PICK\]\[RG2_VISUAL_AUDIT\]|\[RG2_VISUAL_POSE\]|APPROACH_COARSE|GRASP_DOWN_JOINT|GRASP_ALIGN_IK|PRE_CLOSE" \
    /home/laboratorio/TFM/agarre_ros2_ws/log/ros2_launch.log | tail -100
```

---

*Generado: 2026-04-24 | Rama: audit/tcp-geometry-fix | Herramienta: inspect_rg2_visual_pose v1.0*
