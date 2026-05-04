# BUG: GRASP_DOWN cartesian — TCP no alcanza el objeto (35mm shortfall)

**Estado**: 🔴 ABIERTO
**Detectado**: 2026-05-04 (sesión live E2E con stack ROS completo)
**Reproducibilidad**: 100% (cada `pick_demo`)
**Bloqueante para**: Ciclo pick & place completo, F8 (medición de latencias live)

## Resumen

Tras pulsar "Pick demo" (legacy `run_pick_demo`):
- ✅ APPROACH_COARSE pasa (con tolerancias `e6eb936`: 0.035 m)
- ✅ HANDOFF a GRASP_DOWN_JOINT correcto
- ✅ `_run_grasp_down_cartesian` se invoca
- ✅ Joint trajectory completa: `Goal reached, success!`
- ❌ **El TCP queda 35mm arriba del objeto** y nunca llega al contacto
- ❌ `gripper_closed=False`, `attach=false`, `result=exception`

El sistema entra en loop de fallback (`permissive_direct_descent`,
`retry_segmented_refine`) sin éxito.

## Datos hard de la última reproducción (08:03:42 → 08:04:05)

### TF live confirmado por `tf2_echo`

| Frame | Position en base_link |
|---|---|
| tool0 | (0.462, -0.189, 0.060) |
| rg2_tcp | (0.463, -0.014, 0.053) |
| rg2_pinch_center | (0.463, -0.014, 0.053) |
| Objeto pick_demo | (0.438, 0.000, 0.025) |

`rg2_tcp` y `rg2_pinch_center` están en la **misma posición** (legacy alias).
La diferencia tool0 ↔ rg2_tcp es 0.175 m en y (no z) por la rotación
RPY ≈ (90°, 0, 180°) del flange.

### Configuración relevante

| Parámetro | Valor |
|---|---|
| SRDF `tip_link` | `rg2_tcp` (en `ur5_strict.srdf`) |
| `DIRECT_SOURCE_FRAME` | `rg2_pinch_center` |
| `_DIRECTO_GRASP_Z` | 0.0 (forzado a 0 cuando frame es semántico) |
| `GRIPPER_TCP_Z_OFFSET` | 0.0 (de `contact_z_correction_for_frame`) |
| `target_base` enviado | (0.438, 0.000, 0.025) (= centro objeto) |

### Resultado observado

```
TCP before: (0.431, 0.003, 0.056)  → tcp_obj_dist = 31mm
TCP after:  (0.434, 0.000, 0.060)  → tcp_obj_dist = 35mm
```

**El TCP NO baja**, sube 4mm. El sistema cree que MoveIt fue exitoso
porque el joint_trajectory_controller responde "Goal reached, success!".

## Cadena de hipótesis

### H1 — `computeCartesianPath` trunca por colisión 🟢 ALTA PROBABILIDAD

`compute_cartesian_path` con `avoid_collisions=true` se detiene
prematuramente cuando detecta colisión inminente con la mesa
(distancia muy pequeña al objeto/mesa).

Indicios:
- "Goal reached" del controller pero TCP a 35mm del target
- Patrón consistente desde abril: TCP siempre se queda 25–35mm arriba
- Logs `wrist_3_joint=1.6 rad no alcanzado` sugieren IK en límite
  de soluciones válidas

### H2 — IK target está mal calculado 🟡 MEDIA

`apply_local_offset_to_fk(..., flip_xy=True)` aplica un flip xy
heredado del frame DH del modelo. Posible que con SDF actual ya no
sea necesario y esté metiendo error.

Indicios:
- `target_exec_tool0=(-0.419,-0.000,0.735)` en world tiene xy=(-0.419,0)
  vs tool0 actual=(-0.426,0.004,0.730). Diferencia xy ≈ 7mm
- Si flip_xy no fuera necesario, el target_tool0 sería distinto

### H3 — `compute_cartesian_path` con max_step muy grande 🟡 MEDIA

Si `max_step` (default 0.01m) es mayor que algún segmento crítico,
puede saltarse waypoints y dejar la trayectoria incompleta.

## Plan de fix (próxima sesión, 1.5–3 h)

### Paso 1 — Capturar mensaje real publicado en `/desired_grasp_cartesian`

```bash
ros2 topic echo /desired_grasp_cartesian > /tmp/cart.log &
# Disparar pick_demo
ros2 service call /panel/pick_demo std_srvs/srv/Trigger
# Inspeccionar cart.log: ¿pose, frame_id, ee_frame metadata?
```

### Paso 2 — Inspeccionar respuesta del bridge

```bash
ros2 topic echo /desired_grasp/result > /tmp/result.log &
# Buscar request_uuid del cartesian + parsing JSON
# Campo "fraction" del compute_cartesian_path indica % completado
```

### Paso 3 — Revisar parámetros `compute_cartesian_path` en
`ur5_moveit_bridge.py` o `moveit_bridge/moveit_commander_planner.py`:

- `max_step`: bajar a 0.005 m
- `jump_threshold`: relajar
- `avoid_collisions`: probar `False` (riesgo: puede embestir mesa)

### Paso 4 — Si H1 confirma: añadir margen vertical al target

Cambiar target en `_run_grasp_down_cartesian`:
```python
target_base_3 = (target[0], target[1], target[2] - 0.005)  # margen 5mm
```
Para compensar la truncación de `computeCartesianPath`.

### Paso 5 — Verificar SRDF `tip_link`

Si MoveIt está usando `tool0` en lugar de `rg2_tcp` (memoria 2026-04-25
del fix `tool0` 90°), el target estaría a +175mm en y → 175mm
shortfall. **No coincide con nuestros 35mm**, pero verificar.

### Paso 6 — Test E2E final

1. Reset stack: `PANEL_FORCE_COLD_BOOT=1 ./lanzar_panelc2.sh`
2. Pulsar "Pick demo" → verificar ciclo completo hasta DONE
3. Si OK, commit + tag `directo-completo-fix-grasp-down-YYYYMMDD`

## Información para el debugger

### Funciones clave a inspeccionar

- `panel_pick_demo.py:3939` — `_run_grasp_down_cartesian` (pose construction)
- `panel_motion_helpers.py:636` — `publish_moveit_pose` (TF normalize + emit)
- `pick_demo/geometry.py` — `apply_local_offset_to_fk` (flip_xy)
- `ur5_moveit_bridge.py` — handling de `/desired_grasp_cartesian`
- `moveit_bridge/moveit_commander_planner.py` — `compute_cartesian_path`

### Logs útiles

- `historico/DIRECTO_DEBUG_TRACE.log` — traza del ciclo direct
- `historico/stack_manual_*.log` — stack ROS + controllers
- Topics en vivo:
  - `/desired_grasp_cartesian` (input cartesian)
  - `/desired_grasp/result` (output con fraction y reason)
  - `/joint_trajectory_controller/joint_trajectory` (lo que llega)

### Comandos para reproducir

```bash
# 1. Stack
PANEL_FORCE_COLD_BOOT=1 ./lanzar_panelc2.sh
# Esperar READY (~2 min)

# 2. En otra terminal, monitor topics
ros2 topic echo /desired_grasp_cartesian > /tmp/cart.log &
ros2 topic echo /desired_grasp/result > /tmp/result.log &

# 3. Disparar
ros2 service call /panel/select_object ur5_panel_interfaces/srv/SelectObject "{name: pick_demo}"
ros2 service call /panel/pick_demo std_srvs/srv/Trigger

# 4. Esperar 60s, inspeccionar logs
```

## Referencias históricas

- Memoria `project_directo_z_offset_20260408` — fix `_DIRECTO_GRASP_Z=0.0`
- Memoria `project_directo_f4_f7_fixes_20260418` — primer ciclo exitoso
- Memoria `project_geom_restore_fase5_20260420` — restauración geométrica
- Memoria `project_sdf_tool0_90deg_fix_20260425` — fix tool0 RPY
- Memoria `project_pick_validation_audit_20260425` — ee_link mismatch + midpoint
- Memoria `project_audit_fases_0_10_20260504` — audit dice F7 bloqueado por
  "tuning preexistente (gate pregrasp_strict, dist+dz exceeded)" — confirma
  que este es bug **preexistente, no regresión** de los refactor recientes

## NO es regresión

Este bug **existe desde abril** según logs históricos:
- `2026-04-28T22:11:03 [GRASP_DOWN_FALLBACK] reason=direct_ik_exception:GRASP_DOWN_JOINT no alcanzado (timeout)`
- `2026-04-28T22:13:43 [GRASP_DOWN_FALLBACK] reason=direct_ik_exception (timeout)`
- `2026-05-03T20:54:31` — patrón idéntico

El último cycle exitoso fue el **2026-04-18**. Algo entre 04-18 y 04-28
rompió GRASP_DOWN. Las extracciones puras de F3 (mayo) NO afectan al
flujo cartesian — el bug está en el cálculo del target o en los
parámetros de `computeCartesianPath`.
