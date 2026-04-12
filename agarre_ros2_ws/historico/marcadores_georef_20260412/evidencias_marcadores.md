# Evidencias — Marcadores de referencia geométrica en overlay cenital
# Fecha: 2026-04-12
# Contexto: Corrección visual de la brecha CTRL (rg2_pinch_center) vs. BODY (cuerpo físico pinza)

## 1. PROBLEMA QUE SE RESUELVE

El informe forense (auditoria/forense_referencia_geometrica_20260412/informe_forense.md) estableció que
la brecha visual de ~11 cm observada en Gazebo es puramente geométrica: el código controla desde
rg2_pinch_center (punta virtual de dedos, a +17.5 cm de tool0) mientras el ojo mide desde el cuerpo
físico de la pinza (rg2_hand body / finger_pivot area, ~8.8 cm "atrás" de tool0 a lo largo del eje Z
del gripper).

## 2. ARCHIVO MODIFICADO

- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py`
- Método: `_draw_tcp_pose_overlay` (línea ~14564)
- Tipo de cambio: SOLO visual (overlay de cámara). Sin modificación de lógica de agarre, IK, TF, URDF,
  SDF, world, ni Directo2.

## 3. MARCADORES AÑADIDOS

### 3.1 CTRL — rojo (ya existía, reforzado con etiqueta)
- **Frame**: `rg2_pinch_center` (frame TF2 real, offset +0.175 m de tool0 en Z local)
- **Proyección**: coordenadas XY de rg2_pinch_center en base_link, proyectadas a la imagen
  usando `REACH_OVERLAY_Z = 0.850` (plano de mesa) y `world_xyz_to_pixel_float`
- **Color**: rojo `(239, 68, 68)` — círculo + cruceta + etiqueta "CTRL"
- **Cambio respecto a antes**: añadida etiqueta de texto "CTRL" junto al punto

### 3.2 WRIST — naranja (nuevo)
- **Frame**: `tool0` (muñeca TCP, 17.5 cm más arriba en Z local que rg2_pinch_center)
- **Cómo se obtiene**: segunda llamada a `tf_get_tcp_in_base(ee_frame="tool0")` dentro del bloque
  `if TCP_XY_PROJ_OVERLAY`
- **Proyección**: misma proyección (XY de tool0 en base_link → REACH_OVERLAY_Z → píxel)
- **Color**: naranja `(251, 146, 60)` — círculo + cruceta + etiqueta "WRIST"

### 3.3 BODY — azul cielo (nuevo)
- **Punto**: aproximación del finger_pivot / cara inferior visible del cuerpo de la pinza
- **Cómo se calcula**:
  ```
  body = tool0_base - 0.088 * R_tool0_Zcol_in_base
  ```
  donde R_tool0_Zcol es la columna Z de la matriz de rotación de tool0 en base_link, derivada
  del cuaternión TF2 de tool0:
  ```
  qx, qy, qz, qw = tool0.orientation
  rz = (2*(qx*qz + qw*qy),  2*(qy*qz - qw*qx),  1 - 2*(qx² + qy²))
  body = (t0x - 0.088*rz_x, t0y - 0.088*rz_y, t0z - 0.088*rz_z)
  ```
  La constante `_GRIPPER_BODY_DEPTH_M = 0.088` se deriva del análisis forense:
  - Distancia wrist_3 → tool0 = 0.275 m (en Y de wrist_3)
  - Distancia wrist_3 → finger_pivot = 0.1873 m (0.0823 + 0.105)
  - → finger_pivot está 0.0877 m "antes" de tool0 en el eje del brazo ≈ 0.088 m
- **Proyección**: misma (XY de body en base_link → REACH_OVERLAY_Z → píxel)
- **Color**: azul cielo `(56, 189, 248)` — círculo + cruceta + etiqueta "BODY"
- **Línea discontinua azul**: BODY → CTRL (muestra la separación XY proyectada)

## 4. HUD AMPLIADO

- Línea 5 añadida (azul cielo): `GEO-REF  CTRL(pinch)z=+X.XXX  WRIST z=+X.XXX  BODY≈z=+X.XXX  BODY↔CTRL ΔZ=+X.XXXm`
- Panel HUD redimensionado: 86 px → 108 px de alto

## 5. VALIDACIÓN ESPERADA (valores de referencia)

A partir del análisis forense con datos TF2 en vivo (2026-04-12T09:54, fase GRASP_ALIGN_IK):

| Marcador | Frame       | Z en base_link | Z en world |
|----------|-------------|----------------|------------|
| CTRL     | rg2_pinch_center | +0.034 m  | 0.884 m    |
| WRIST    | tool0       | -0.035 m       | 0.815 m    |
| BODY     | derivado    | ≈ -0.068 m *   | ≈ 0.782 m  |

* En pose GRASP_ALIGN_IK con RPY≈(66°,-10°,-118°):
  rz = (-0.776, 0.490, 0.397)
  body_z = -0.035 - 0.088 × 0.397 = -0.035 - 0.035 = -0.070 m (aprox.)

Delta esperado BODY↔CTRL (Z base_link): ≈ +0.034 - (-0.070) = +0.104 m ≈ +10.4 cm
(vs. 11.9 cm del análisis forense con finger_pivot exacto — la diferencia es la aproximación
0.088 m vs. finger_pivot calculado exacto 0.0877 m + diferencias de proyección en pose inclinada)

## 6. LO QUE NO CAMBIA

- `panel_pick_demo.py`: sin modificaciones
- `DIRECTO_SOURCE_FRAME`: sigue siendo `rg2_pinch_center`
- `DIRECTO2`: sin modificaciones
- MoveIt, URDF, SDF, worlds: sin modificaciones
- Lógica de agarre, IK, TF: sin modificaciones

## 7. REVERSIBILIDAD

El cambio es completamente reversible: si `TCP_XY_PROJ_OVERLAY = False` (env var), los marcadores
nuevos no se calculan ni dibujan (el bloque entero está bajo `if TCP_XY_PROJ_OVERLAY:`).
Si `TCP_POSE_TEXT_OVERLAY = False`, la línea 5 del HUD no se muestra.
