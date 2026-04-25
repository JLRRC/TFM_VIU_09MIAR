# Auditoría geometría visual RG2 — DIRECTO + MoveIt TCP coherencia

**Fecha:** 2026-04-24  
**Rama:** audit/tcp-geometry-fix  
**Estado:** FASE 0–6 completas (análisis estático); FASES 1/2/8/9 requieren stack activo  

---

## 1 · Resumen ejecutivo

El análisis estático de URDF + SDF demuestra que la geometría visual ya es consistente:

| Punto de referencia | Posición en tool0 |
|---|---|
| `rg2_pinch_center` (URDF) | (0, 0, 0.175) |
| `rg2_tcp` (URDF) | (0, 0, 0.175) |
| Midpoint finger joints SDF (calculado) | **(0, 0, 0.175)** ← coincide |
| `pick_demo_anchor` (SDF) | (0, 0, 0.175) |

**No se requieren cambios en URDF ni SDF.** La coherencia geométrica visual ↔ TCP semántico está demostrada algebraicamente.

**Único cambio aplicado:** Añadir logs auditables al puente MoveIt:
`[MOVEIT][TIP_LINK]`, `[MOVEIT][TF_FRESHNESS]`, `[MOVEIT][GEOM_AUDIT]`.

---

## 2 · Estado inicial (FASE 0)

```
git status --short:
M  src/ur5_bringup/launch/ur5_rsp.launch.py
M  src/ur5_bringup/launch/ur5_stack.launch.py
M  src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
M  src/ur5_qt_panel/ur5_qt_panel/panel_tf.py
M  src/ur5_tools/setup.py
A  src/ur5_tools/ur5_tools/inspect_rg2_visual_pose.py
M  src/ur5_tools/ur5_tools/system_state_manager.py
A  reports/incidents/2026-04-24_rg2_visual_pose_runtime_audit.md
A  reports/incidents/2026-04-24_rsp_dead_stale_tf_panel_false_tcp_ok.md
```

**Archivos candidatos candidatos a cambio:**
- `src/ur5_tools/ur5_tools/ur5_moveit_bridge.py` (único modificado)
- `src/ur5_description/urdf/ur5.urdf.xacro` (solo lectura, sin cambios necesarios)
- `models/ur5_rg2/model.sdf` (solo lectura, sin cambios necesarios)

---

## 3 · FASE 1 — Verificación runtime base

> **Estado:** no ejecutada (stack no activo durante la auditoría estática).  
> Comandos para reproducir con stack activo:

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
source install/setup.bash
export PANEL_PICK_DEMO_REQUIRE_FRESH_TF=1
export PANEL_PICK_DEMO_TF_MAX_AGE_SEC=0.5

ros2 node list | grep robot_state_publisher
ros2 topic info /tf -v
timeout 5 ros2 run tf2_ros tf2_echo base_link tool0
timeout 5 ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
timeout 5 ros2 run tf2_ros tf2_echo tool0 rg2_pinch_center
ros2 run ur5_tools validate_dh_vs_tf
ros2 run ur5_tools inspect_rg2_visual_pose --ros-args -p once:=true -p object_name:=pick_demo
```

---

## 4 · FASE 3 — Geometría URDF vs SDF (análisis estático)

### 4.1 · Tabla de elementos

| Elemento | Fuente | Parent | xyz (parent) | rpy | Uso |
|---|---|---|---|---|---|
| `tool0` | URDF/SDF | wrist_3_link | 0 0 0 | -π/2 0 0 | Flange UR5 |
| `rg2_base_link` | URDF | tool0 | 0 0 0 | 0 0 0 | Stub cinético |
| `rg2_tcp` | URDF | tool0 | **0 0 0.175** | 0 0 0 | TCP semántico alias |
| `rg2_pinch_center` | URDF | tool0 | **0 0 0.175** | 0 0 0 | TCP operacional |
| `rg2_finger_link1` | URDF | rg2_base_link | 0 0.02 0 | 0 0 0 | Stub prismático (Z=0 de tool0) |
| `rg2_finger_link2` | URDF | rg2_base_link | 0 -0.02 0 | 0 0 0 | Stub prismático (Z=0 de tool0) |
| `rg2_hand` | SDF | wrist_3_link | **0 0.07 0** | π/2 0 π/2 | Cuerpo visual gripper |
| `rg2_leftfinger` | SDF | rg2_hand | 0.105 0.017 0 | 0 0 0 | Dedo visual (revolute) |
| `rg2_rightfinger` | SDF | rg2_hand | 0.105 -0.017 0 | 0 0 0 | Dedo visual (revolute) |
| `pick_demo_anchor` | SDF | tool0 | **0 0 0.175** | 0 0 0 | Anclaje detachable |

### 4.2 · Transformaciones derivadas

**Cadena SDF: wrist_3_link → tool0**

```
R_tool0 = Rx(-π/2) = [[1,0,0],[0,0,1],[0,-1,0]]
```

**Cadena SDF: wrist_3_link → rg2_hand**

```
rpy = (π/2, 0, π/2)  →  R_rg2hand_wrist3 = Rz(π/2)·Rx(π/2) = [[0,0,1],[1,0,0],[0,1,0]]
xyz = (0, 0.07, 0) en wrist_3_link
```

**rg2_hand en frame tool0:**

```
p_rg2hand_in_tool0 = R_tool0^T · (0, 0.07, 0) = Rx(π/2)·(0, 0.07, 0) = (0, 0, 0.07)
R_rg2hand_in_tool0 = R_tool0^T · R_rg2hand_wrist3 = [[0,0,1],[0,-1,0],[1,0,0]]
```

**rg2_finger_joint1 en frame tool0** (origen en rg2_hand: 0.105, 0.017, 0):
```
p = (0,0,0.07) + [[0,0,1],[0,-1,0],[1,0,0]] · (0.105, 0.017, 0)
  = (0,0,0.07) + (0, -0.017, 0.105)
  = (0, -0.017, 0.175)
```

**rg2_finger_joint2 en frame tool0** (origen en rg2_hand: 0.105, -0.017, 0):
```
p = (0,0,0.07) + [[0,0,1],[0,-1,0],[1,0,0]] · (0.105, -0.017, 0)
  = (0,0,0.07) + (0, 0.017, 0.105)
  = (0, 0.017, 0.175)
```

**Midpoint finger joints en tool0:**
```
midpoint = ( (0+0)/2, (-0.017+0.017)/2, (0.175+0.175)/2 ) = (0, 0, 0.175)
```

### 4.3 · Verificación de coherencia

| Check | Valor esperado | Valor calculado | Coherente |
|---|---|---|---|
| rg2_pinch_center en tool0 | (0,0,0.175) | (0,0,0.175) URDF | ✅ |
| rg2_tcp en tool0 | (0,0,0.175) | (0,0,0.175) URDF | ✅ |
| SDF finger joint midpoint en tool0 | (0,0,0.175) | (0,0,0.175) calculado | ✅ |
| pick_demo_anchor en tool0 | (0,0,0.175) | (0,0,0.175) SDF | ✅ |
| Eje de cierre dedos en tool0 | eje Y tool0 | eje Y tool0 (rg2_hand X→tool0 Z, cierre en ±Y) | ✅ |
| dist_pinch_to_urdf_finger_mid | ~0.175m (stub Z=0) | esperado (stubs prismáticos) | ✅ (esperado) |

**NOTA sobre URDF finger links:** `rg2_finger_link1/2` son stubs cinemáticos con Z=0 desde tool0. El midpoint de estos stubs **NO** coincide con rg2_pinch_center. Esto es correcto e intencionado — el midpoint visual relevante es el de los dedos SDF (gz_visual_mid), no el de los stubs URDF.

### 4.4 · Orientación rg2_hand vs rg2_pinch_center

El frame rg2_pinch_center hereda la orientación de tool0 (rpy=0,0,0 en URDF).
El frame rg2_hand en tool0 tiene R=[[0,0,1],[0,-1,0],[1,0,0]]:
- rg2_hand X (approachdir) → tool0 Z
- rg2_hand Y (closing axis) → -tool0 Y

El eje de cierre de los dedos (movimiento ±Y de rg2_hand) mapea a **∓Y de rg2_pinch_center** en world cuando el robot está en posición nominal. El `inspect_rg2_visual_pose.py` calcula `closing_axis_world = _rotate_vec_by_quat((0,1,0), pinch_q)` que es correcto para detectar si el objeto cae en el eje de cierre.

---

## 5 · FASE 4 — Decisión de fix

**CASO 5 aplicado:** geometría visual y TCP semántico son coherentes (dist_pinch_to_gz_visual_mid nominal = 0, confirmado algebraicamente). DIRECTO ya usa rg2_pinch_center como DIRECT_SOURCE_FRAME. MoveIt usa rg2_tcp como IK tip (coincidente). 

**Único gap:** faltan logs `[MOVEIT][TF_FRESHNESS]`, `[MOVEIT][TIP_LINK]`, `[MOVEIT][GEOM_AUDIT]` en el bridge.

---

## 6 · FASE 6 — Cambios aplicados en MoveIt bridge

**Archivo:** `src/ur5_tools/ur5_tools/ur5_moveit_bridge.py`

**Cambio 1 — `[MOVEIT][TIP_LINK]` + `[MOVEIT][TF_FRESHNESS]`** insertados inmediatamente tras `[PICK][MOVEIT][TARGET]` (antes del dispatch de planificación):

- `[MOVEIT][TIP_LINK]`: confirma `ee_frame` y `ik_tip=rg2_tcp` para cada request.
- `[MOVEIT][TF_FRESHNESS]`: comprueba edad del TF `base_link→tool0`, misma lógica que `_check_pick_tf_freshness` del panel. No bloquea el request; solo informa.

**Cambio 2 — `[MOVEIT][GEOM_AUDIT]`** insertado en `_publish_result` tras `[PICK][MOVEIT][EXEC_RESULT]`:

- Lee TF actual de `ee_frame` (rg2_tcp) y `rg2_pinch_center` en base_link.
- Calcula distancia al target.
- Emite veredicto OK/FAIL con umbral 20mm.
- Si TF no disponible → UNAVAILABLE (nunca bloquea).

---

## 7 · FASE 7 — Build

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur5_tools --symlink-install
source install/setup.bash
python3 -m py_compile src/ur5_tools/ur5_tools/ur5_moveit_bridge.py
```

---

## 8 · Tabla final

| Check | DIRECTO | MOVEIT | Evidencia |
|---|---|---|---|
| Geometría URDF/SDF coherente | ✅ (estático) | ✅ (estático) | Cálculo algebraico §4.2 |
| rg2_pinch_center en tool0 = (0,0,0.175) | ✅ | ✅ | URDF rg2_pinch_center_joint |
| SDF finger midpoint = (0,0,0.175) en tool0 | ✅ | ✅ | Cálculo §4.2 |
| pick_demo_anchor = (0,0,0.175) en tool0 | ✅ | ✅ | SDF pick_demo_anchor_joint |
| DIRECT_SOURCE_FRAME = rg2_pinch_center | ✅ | — | panel_pick_demo.py:89 |
| IK tip = rg2_tcp (≡ pinch_center) | — | ✅ | ur5_moveit_bridge.py:4510 |
| `[MOVEIT][TIP_LINK]` log | — | ✅ | añadido en FASE 6 |
| `[MOVEIT][TF_FRESHNESS]` log | ✅ (panel) | ✅ | añadido en FASE 6 |
| `[MOVEIT][GEOM_AUDIT]` log | — | ✅ | añadido en FASE 6 |
| `[PICK][RG2_VISUAL_AUDIT]` log | ✅ | — | panel_pick_demo.py:4926 |
| TF_FRESHNESS guard en fases críticas | ✅ | info-only | REQUIRE_FRESH_TF env |

---

## 9 · Qué queda pendiente

- **FASES 1/2/8/9:** Verificación runtime con stack activo (TF en vivo, gz_visual_mid real, dist_pinch_gz_visual_mid medida).
- **Cierre físico:** contact/friction/attach backend — explícitamente excluido de esta tarea.
- **LIFT/CARRY:** Post-CLOSE, no dentro del alcance de esta auditoría.
- **Gazebo pose/info partición (GZ_PARTITION):** si gz_pose_bridge no fluye, dist_pinch_gz_visual_mid quedará UNKNOWN. Verificar con inspect_rg2_visual_pose.

---

## 10 · Comandos para reproducir

```bash
# Stack
cd /home/laboratorio/TFM/agarre_ros2_ws
source install/setup.bash
export PANEL_PICK_DEMO_REQUIRE_FRESH_TF=1
export PANEL_PICK_DEMO_TF_MAX_AGE_SEC=0.5
ros2 launch ur5_bringup ur5_stack.launch.py

# Inspector visual (otra terminal)
ros2 run ur5_tools inspect_rg2_visual_pose \
  --ros-args -p once:=true -p object_name:=pick_demo

# Validador DH/TF
ros2 run ur5_tools validate_dh_vs_tf

# Filtrar logs MoveIt
grep -E "\[MOVEIT\]\[GEOM_AUDIT\]|\[MOVEIT\]\[TF_FRESHNESS\]|\[MOVEIT\]\[TIP_LINK\]|\[PICK\]\[RG2_VISUAL_AUDIT\]|\[PICK\]\[TF_FRESHNESS\]" /tmp/ur5_*.log
```
