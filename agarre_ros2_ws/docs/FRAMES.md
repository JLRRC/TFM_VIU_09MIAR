# Frames y geometría — UR5+RG2

**Fecha:** 2026-05-10 (auditoría F10.3).

Este documento es la referencia canónica del TF tree, offsets geométricos y reglas de uso de frames operativos.

---

## 1. TF tree

```
world
  │
  │ static, publicado por:
  │   * robot_state_publisher (URDF línea 47:
  │     <origin xyz="-0.85 0 0.850" rpy="0 0 0"/>)
  │   * world_tf_publisher (override dinámico desde /world/<name>/pose/info)
  │
  └── base_link
        │
        ├── base_link_inertia
        │     └── shoulder_link
        │           └── upper_arm_link
        │                 └── forearm_link
        │                       └── wrist_1_link
        │                             └── wrist_2_link
        │                                   └── wrist_3_link
        │                                         └── flange
        │                                               └── tool0
        │
        ▼
       (cadena UR5 publicada por robot_state_publisher desde joint_states)
                │
                └── tool0
                      ├── rg2_base_link    (mount fijo, xyz=0,0,0)
                      │     ├── rg2_finger_link1  (prismatic +Y, [0, 0.0425] m)
                      │     └── rg2_finger_link2  (prismatic -Y, [0, 0.0425] m)
                      │
                      ├── rg2_tcp           (offset 0,0,0.175 m) ← TIP_LINK SRDF
                      └── rg2_pinch_center  (offset 0,0,0.175 m) ← alias
```

---

## 2. Frames operativos

| Frame | Origen | Offset desde padre | Uso |
|---|---|---|---|
| `world` | raíz | n/a | Frame inercial fijo de Gazebo |
| `base_link` | URDF (`<origin xyz="-0.85 0 0.850">`) | (-0.85, 0, 0.850) m desde world | **Planning frame de MoveIt** |
| `tool0` | UR macro (oficial) | cadena UR5 | Frame estándar UR |
| `rg2_base_link` | URDF línea 51 | (0, 0, 0) desde tool0 | Base del gripper, contiene los dedos |
| `rg2_finger_link{1,2}` | URDF prismáticos | ±0.020 m Y desde rg2_base_link | Dedos físicos |
| `rg2_tcp` | URDF línea 78 | (0, 0, 0.175) m desde tool0 | **TCP semántico canónico (SRDF tip_link)** |
| `rg2_pinch_center` | URDF línea 95 | (0, 0, 0.175) m desde tool0 | Alias semántico de `rg2_tcp` |

---

## 3. Single source of truth

**Fuente declarativa:** `src/ur5_description/config/geometry.yaml` *(F2 audit)*.
**Mirror Python:** `src/ur5_tools/ur5_tools/geometry_constants.py` *(F2 audit)*.
**Fuente cinemática:** `src/ur5_description/urdf/ur5.urdf.xacro`.

Test `test_geometry_yaml_consistency.py` *(F2.3)* garantiza coherencia:

- `BASE_LINK_IN_WORLD == (-0.85, 0.0, 0.850)`
- `RG2_TCP_OFFSET_FROM_TOOL0 == (0.0, 0.0, 0.175)`
- `UR5_HOME_POSITIONS_RAD == (0, -π/2, 0, -π/2, 0, 0)`
- URDF contiene literal `xyz="-0.85 0 0.850"` y `value="0 0 0.175"`.

---

## 4. Conversión world ↔ base_link

**Fórmula:**
```
pos_base  = pos_world - BASE_LINK_IN_WORLD
pos_world = pos_base  + BASE_LINK_IN_WORLD
```

**Helpers Python:**
```python
from ur5_tools.geometry_constants import (
    BASE_LINK_IN_WORLD,    # (-0.85, 0.0, 0.850)
    world_to_base,
    base_to_world,
)
```

**Servicio ROS:** `/tf_geometry/world_to_base` (`WorldToBase.srv`) — para nodos que no quieren importar la constante.

---

## 5. TCP operacional: `rg2_tcp` vs `rg2_pinch_center`

Ambos frames tienen el **mismo offset** (`0, 0, 0.175 m` desde `tool0`). La diferencia es semántica:

- **`rg2_tcp`**: tip_link declarado en `ur5.srdf`. Es el frame de **planning** (MoveIt) y **attach** (gripper_attach_backend). **Autoridad**.
- **`rg2_pinch_center`**: alias creado para legibilidad en logs y código del panel. Conceptualmente es "el centro físico de cierre del RG2".

**Regla:** usar `rg2_tcp` siempre en código de planning/attach. `rg2_pinch_center` solo en mensajes humanos (debug, UI).

Validado por test `test_urdf_xacro_parses` que verifica que ambos frames coexisten con el mismo offset.

---

## 6. Publicadores TF

| Publicador | Frames publicados | Tipo |
|---|---|---|
| `robot_state_publisher` | toda la cadena URDF (world→tool0→rg2_*) | Standard ROS, lee `/joint_states` |
| `world_tf_publisher` | `world → base_link` (override dinámico) | Lee `/world/<name>/pose/info` de Gazebo, scoring para evitar mal frame raíz (bug 2026-04-27 mitigado) |
| `gz_pose_bridge` | poses de entidades Gazebo en `/tf` | Bridge ros_gz custom (ur5_tools) |

**Posible duplicación:** `world → base_link` puede publicarse por `robot_state_publisher` (vía URDF estático) y `world_tf_publisher` (vía Gazebo pose). El segundo gana si está activo (último timestamp). El test `test_tf_world_base_link_unique` (F4) puede añadirse al CI live para detectar conflictos.

---

## 7. SRDF planning groups

**Grupo `manipulator`** (`ur5.srdf:11-19`):
```xml
<group name="manipulator">
  <chain base_link="base_link" tip_link="rg2_tcp"/>
</group>
```

Joints incluidos:
- `shoulder_pan_joint`
- `shoulder_lift_joint`
- `elbow_joint`
- `wrist_1_joint`
- `wrist_2_joint`
- `wrist_3_joint`

**No hay grupo `gripper`** porque el RG2 se controla fuera de MoveIt (decisión P-04).

---

## 8. Reglas de uso de frames

1. **Planning (MoveIt):** target pose en `base_link`, tip frame `rg2_tcp`.
2. **Attach lógico:** `gripper_attach_backend` lee TCP en `rg2_tcp`, calcula distancia al objeto en `world`.
3. **Drop / spawn objetos:** poses en `world`.
4. **Evidencia / logs:** poses en `world` (frame humano-friendly).
5. **APPROACH / GRASP_DOWN:** TCP debe apuntar -Z world (top-down) con yaw alineado al eje mayor del objeto. Computado por `compute_top_down_grasp_quat` en `phase_dispatch.py`.

---

## 9. Workspace UR5+RG2

- `radius_max_m: 0.85` (workspace cinemático)
- `radius_min_m: 0.20` (zona muerta cerca de la base)

`grasp_selector_node` filtra candidatos fuera de este rango.

---

## 10. Bugs históricos relacionados con frames (cerrados)

| Fecha | Bug | Causa raíz | Fix |
|---|---|---|---|
| 2026-04-25 | SDF tool0 mismatch 90° | `end_effector_frame_fixed_joint` rpy=(-π/2,0,0) en SDF vs URDF rpy=0 → error 247mm | rpy=0 en SDF + ur5_hand_joint Z=0.07 |
| 2026-04-27 | world→base_link con pose del gripper | `world_tf_publisher` aceptaba `rg2_base_link` como base_link | scoring `_score_name` rechaza nombres de gripper (commit 309a88e) |
| 2026-04-23 | Divergencia DH↔SDF 343mm | 9 fixes en model.sdf + xacro 0.175 restaurado (Opción 3) | `rg2_contact_tcp_xyz = "0 0 0.175"` en URDF línea 20 |
| 2026-05-04 | URDF↔SDF gripper offset ~165mm | rg2_pinch_center URDF a 167mm Y del centro físico Gazebo | Refactor 2026-05-06: SDF y URDF aligned en `tool0 → rg2_tcp` |
