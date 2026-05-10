# Tabla de frames esperados — UR5 + RG2 (HEAD 34daa7e working tree, post-fix 550457a)

| Frame | Padre | Fuente | Offset esperado (xyz) | Rotación esperada (rpy) | Uso | Riesgo |
|---|---|---|---|---|---|---|
| `world` | — | world SDF | — | — | origen Gazebo | mezclar con base_link |
| `base_link` | `world` | URDF/world (xacro origin xyz=-0.85 0 0.850) | (-0.85, 0, 0.850) | (0,0,0) | IK/MoveIt | error global |
| `base_link_inertia` | `base_link` | ur_macro URDF | (0,0,0) | (0,0,π) | solver IK | NEGATE_XY si se usa |
| `wrist_3_link` | `wrist_2_link` | URDF revolute | (0, 0.0823, 0) | dependiente joint | flange brazo | — |
| `flange` | `wrist_3_link` | URDF fixed (wrist_3-flange) | (0,0,0) | (0, -π/2, -π/2) | ROS-Industrial std | — |
| `tool0` | `flange` | URDF fixed (flange-tool0) | (0,0,0) | (π/2, 0, π/2) | TCP brazo | NO es contacto RG2 |
| `tool0` (SDF) | `wrist_3_link` | SDF end_effector_frame_fixed_joint | (0,0,0) post-fix 550457a | (-π/2, 0, 0) | físico Gazebo | rpy difiere de URDF — solo afecta pose Gazebo |
| `rg2_base_link` | `tool0` | URDF + SDF rg2_mount_joint | (0,0,0) post-fix 550457a | (0,0,0) | mount RG2 | recuperado de 0.1927m bug |
| `rg2_finger_link1` | `rg2_base_link` | URDF/SDF rg2_finger_joint1 (prismatic Y+) | (0, +0.020+q1, 0) | (0,0,0) | dedo 1 | — |
| `rg2_finger_link2` | `rg2_base_link` | URDF/SDF rg2_finger_joint2 (prismatic -Y) | (0, -0.020-q2, 0) | (0,0,0) | dedo 2 | — |
| `rg2_tcp` | `tool0` | URDF rg2_tcp_joint | (0, 0, 0.175) | (0,0,0) | TCP virtual | == rg2_pinch_center |
| `rg2_pinch_center` | `tool0` | URDF rg2_pinch_center_joint | (0, 0, 0.175) | (0,0,0) | TCP operacional | debe coincidir con yemas |
| `pick_demo_anchor` | `tool0` | SDF pick_demo_anchor_joint | (0, 0, 0.175) | (0,0,0) | attach backend | == rg2_pinch_center post-fix |
| `camera_wrist_link` | `rg2_base_link` | SDF camera_wrist_joint | (0, 0.04, 0.01) | (0,0,0) | visión | — |
| `pick_demo` (objeto) | `world` | world SDF (-0.42, 0, 0.876) inicial | dinámico | — | objeto cinemático | timestamp/freshness |
| `mesa_pro` (top) | `world` | world SDF (-0.17,0,0.075)+(0,0,0.75)+0.025 = z=0.85 | — | — | mesa | objeto a 26mm sobre mesa |
| `base_link` (UR5) en world | `world` | URDF xacro origin (-0.85,0,0.850) | (-0.85, 0, 0.850) | (0,0,0) | base robot | mesa_robot top a z=0.825 (25mm gap aceptable) |

## Cálculo de coherencia URDF↔SDF post-fix (commit 550457a)

**URDF** (ur5.urdf.xacro líneas 87-89, 103-107):
- `tool0` → `rg2_pinch_center` = (0, 0, 0.175) en tool0 frame

**SDF** (model.sdf líneas 593-601, post-fix):
- `tool0` → `pick_demo_anchor` = (0, 0, 0.175) en tool0 frame

→ **Coinciden**: el TCP semántico URDF = anchor físico SDF a 0.175m sobre tool0 +Z.

**Yemas físicas** (URDF/SDF lines 109-159 / 446-569):
- `rg2_finger_link1/2` con visual+collision en `(0, 0, 0.0875)` size `0.012 x 0.010 x 0.175`
- joint en `(0, ±0.020, 0)` con axis ±Y, lower=0, upper=0.0425
- yema (top del finger) está a Z=0.175 desde base, igual que pinch_center

→ **Coinciden**: la yema del dedo está exactamente al nivel del rg2_pinch_center.

## Conclusión documental

Tras el fix 550457a, la geometría URDF y SDF describen el MISMO gripper. La paridad se valida vía test T22 (test_urdf_sdf_parity.py).
