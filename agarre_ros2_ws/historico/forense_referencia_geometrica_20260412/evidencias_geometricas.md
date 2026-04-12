# Evidencias forenses — referencia geométrica vs. visual
# Fecha: 2026-04-12
# Pregunta: ¿Por qué el ojo ve ~11 cm de brecha y el código muestra ~1 cm?

## 1. FUENTES DE DATOS

### 1.1 URDF (ur5.urdf.xacro)
- `rg2_pinch_center_joint`: parent=tool0, `<origin xyz="0 0 0.175" rpy="0 0 0"/>`
  → rg2_pinch_center = tool0 + 0.175 m en el eje local Z de tool0
- `rg2_tcp_joint`:           parent=tool0, `<origin xyz="0 0 0.175" rpy="0 0 0"/>`
  → rg2_tcp y rg2_pinch_center son CO-LOCALIZADOS en la definición URDF
- `rg2_mount_joint`:         parent=tool0, `<origin xyz="0 0 0" rpy="0 0 0"/>` (sin offset)

### 1.2 SDF Gazebo (models/ur5_rg2/model.sdf)
- anchor gripper_attach_base: `<pose relative_to="tool0">0 0 0.175 0 0 0</pose>`
  → confirma: contacto/agarre = tool0 + 0.175 m en Z de tool0
- `ur5_hand_joint` (body de la pinza): `<pose relative_to="wrist_3_link">0 0.0823 0 1.5708 0 1.5708</pose>`
- `tool0` link:               `<pose relative_to="wrist_3_link">0 0.275  0 -1.5708 0 0</pose>`
- `rg2_finger_joint1`:        `<pose relative_to="rg2_hand">0.105 0.017 0 0 0 0</pose>`
  → pivote del dedo a 0.105 m del origen rg2_hand en X de rg2_hand

### 1.3 Mundo Gazebo (worlds/ur5_mesa_objetos.sdf)
- Robot spawn: `<pose>-0.85 0.00 0.850 0 0 0</pose>` → base_link world_z = 0.850 m
- Mesa pro tablero: mesa_pose_z=0.075, tablero_offset=0.750, grosor=0.05
  → Superficie mesa world_z = 0.075 + 0.750 + 0.025 = **0.850 m** ✓
- pick_demo: `<pose>-0.42 0.00 0.925 0 0 0</pose>`, cilindro radius=0.025, length=0.05
  → Centro spawn world_z = 0.925 m (inicial, antes de caída)
  → En reposo sobre mesa: centro world_z = 0.850 + 0.025 = **0.875 m**
  → En base_link: obj_z = 0.875 − 0.850 = **0.025 m** ✓ (confirma valor observado)

### 1.4 world_tf_publisher.py
- Lee pose del modelo ur5_rg2 desde Gazebo pose/info (z=0.850)
- Publica TF world → base_link con z=0.850 (sobreescribe el URDF origin=0.825)
- URDF z=0.825 es INACTIVO en runtime; el valor efectivo es 0.850 del SDF/Gazebo

### 1.5 panel_settings.py (constantes)
- `gripper_tcp_z_offset = 0.05` (GRIPPER_TCP_Z_OFFSET — no aplicado a rg2_pinch_center)
- `ur5_base_z = 0.825` (valor URDF, NO es lo que usa world_tf_publisher en runtime)

### 1.6 panel_pick_demo.py (lógica)
- DIRECT_SOURCE_FRAME = "rg2_pinch_center"
- `grasp_z_for_source_frame = 0.0` (cuando source_frame == "rg2_pinch_center")
- APPROACH_COARSE target_z = obj_z + 0.0 + coarse_extra_z_m
  - coarse_extra_z_m = PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M (default 0.015, en corrida activa = 0.1)

## 2. DATOS TF2 EN VIVO (captura 2026-04-12T09:54, fase GRASP_ALIGN_IK)
Fuente: HISTORICO/runtime_vivo_approach_coarse_20260412_094015/tf_actual.txt

| Frame         | X (m)  | Y (m)  | Z (m)  |
|---------------|--------|--------|--------|
| rg2_pinch_center (base_link) | 0.426 | 0.001 | **0.034** |
| tool0 (base_link)            | 0.562 | -0.085 | **-0.035** |
| pick_demo (base_link)        | 0.430 | -0.000 | **0.025** |

Rotación compartida tool0/rg2_pinch_center (xyzw): (-0.219, 0.503, 0.691, -0.470)
RPY en grados: (66.2°, -9.8°, -118.0°) → POSE INCLINADA, NO vertical pura

Vector tool0→rg2_pinch_center en base_link: (-0.136, +0.086, +0.069), norma=0.176 m ≈ 0.175 ✓

Columna Z del eje tool0 en base_link (dirección "hacia adelante del gripper"):
→ Extraída de la rotation matrix: Z_tool0_in_base = (-0.776, 0.490, 0.397)
→ rg2_pinch_center = tool0 + 0.175 × (-0.776, 0.490, 0.397)
   = (0.562−0.1358, −0.085+0.0858, −0.035+0.0695) = (0.426, 0.001, 0.034) ✓

## 3. CÁLCULO DEL ORIGEN rg2_hand EN BASE_LINK

Rotación wrist_3_link en base_link (calculada de tool0 TF y SDF joint offsets):
R_wrist3 ≈ [[-0.462, -0.776, -0.429],
             [-0.870,  0.490,  0.052],
             [ 0.170,  0.397, -0.902]]

Posición wrist_3_link en base_link:
wrist3 = tool0_base − R_wrist3 × (0, 0.275, 0)
R_wrist3×(0,0.275,0) = (-0.2134, 0.1348, 0.1092)
wrist3_base = (0.562+0.2134, -0.085-0.1348, -0.035-0.1092) = (0.775, -0.220, -0.144)

Posición rg2_hand origin en base_link:
rg2_hand_base = wrist3_base + R_wrist3 × (0, 0.0823, 0)
R_wrist3×(0,0.0823,0) = (-0.0639, 0.0403, 0.0327)
rg2_hand_base = (0.775−0.064, −0.220+0.040, −0.144+0.033) = (0.711, -0.180, **-0.112**)

rg2_hand_origin world_z = 0.850 + (-0.112) = **0.738 m**

## 4. PIVOTE DE DEDO EN BASE_LINK

Rotación rg2_hand en base_link:
R_rg2hand_X (en base_link) = (-0.776, 0.490, 0.397)  [rg2_hand's X → wrist3 Y mapped to base]

finger_pivot_base = rg2_hand_base + R_rg2hand × (0.105, 0.017, 0)
delta_z = 0.105 × 0.397 + 0.017 × (−0.902) ≈ 0.042 − 0.015 = **+0.026**
finger_pivot_z_base = −0.112 + 0.026 = **−0.085 m**
finger_pivot_world_z = 0.850 + (−0.085) = **0.765 m**

## 5. TABLA RESUMEN DE POSICIONES (base_link / world)

| Punto                        | Z base_link (m) | Z world (m) | Descripción |
|------------------------------|-----------------|-------------|-------------|
| Mesa sup. / base_link origin  | 0.000           | 0.850       | Mismo plano en runtime |
| pick_demo centro             | **0.025**       | 0.875       | Cilindro en reposo |
| pick_demo top                | **0.050**       | 0.900       | centro + radio (0.025) |
| rg2_pinch_center             | **0.034**       | 0.884       | PLANO DE CONTACTO |
| tool0                        | **-0.035**      | 0.815       | Muñeca TCP |
| finger_pivot (base de dedo)  | **-0.085**      | 0.765       | Donde el dedo sale del body |
| rg2_hand origin              | **-0.112**      | 0.738       | Montaje de la pinza |

## 6. DELTAS CRÍTICOS

| Par                                     | Delta Z (m) | Delta Z (cm) |
|-----------------------------------------|-------------|--------------|
| rg2_pinch_center − obj_centro           | +0.009      | +0.9 cm      |
| rg2_pinch_center − obj_top              | -0.016      | -1.6 cm      | ← fingertip DENTRO del objeto |
| tool0 − obj_centro                      | -0.060      | -6.0 cm      |
| rg2_hand_origin − obj_top (world)       | 0.738−0.900 | **-16.2 cm** |
| finger_pivot − obj_top (world)          | 0.765−0.900 | **-13.5 cm** | ← "brecha visual" observada |
| finger_pivot − rg2_pinch_center (world) | 0.765−0.884 | **-11.9 cm** | ← diferencia referencia visual vs. código |

## 7. CONCLUSIÓN

NO ES PROBLEMA DE UNIDADES. Todo está en metros. Los valores son geométricamente coherentes.

ES PROBLEMA DE REFERENCIA GEOMÉTRICA:
- El código mide desde rg2_pinch_center = plano virtual de contacto en punta de dedos
- El ojo mide desde el cuerpo visual de la pinza (hand.dae body), específicamente su cara
  inferior (finger_pivot area), que en la pose inclinada actual está ~11.9 cm por ENCIMA
  de rg2_pinch_center en world Z

La diferencia entre el punto que el código usa y el que el ojo ve: **11.9 cm en Z world** ≈ "11 cm" observados visualmente.
