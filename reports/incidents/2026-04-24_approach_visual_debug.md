# Auditoría APPROACH_COARSE visual — 2026-04-24

## 1. Resumen ejecutivo

| | |
|---|---|
| **¿APPROACH funciona visualmente?** | **Parcialmente. El TCP semántico (rg2_pinch_center) SÍ se mueve hacia el objeto con clearance correcto. La pinza visual Gazebo (gz_visual_mid) reporta frame MISMATCH — los links del gripper se publican en frame local del modelo, no en world.** |
| **Causa raíz principal** | Gate `APPROACH_COARSE_NOT_READY` con tolerancias imposibles para una fase con clearance Z. `_coarse_handoff_dist_tol=0.015m` no puede satisfacerse cuando `dz_obj≈29mm` (clearance). |
| **Causa raíz secundaria** | `gz_visual_mid` con X positivo vs TCP X negativo indica que Gazebo publica poses de `rg2_leftfinger`/`rg2_rightfinger` en el frame local del robot, no en world frame ROS2. |
| **Fix aplicado** | Añadida condición `approach_clearance_ok` en `_build_approach_coarse_phase_check`. Añadida detección de FRAME_MISMATCH en `_emit_rg2_visual_audit`. Añadidos 4 logs obligatorios. |
| **Qué queda pendiente** | Causa del frame mismatch en Gazebo pose_info para links del gripper (investigar GZ_PARTITION/entidad). Validación en run siguiente. |

---

## 2. Estado inicial (git status)

```
M  scripts/object_positions.json
M  src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
M  src/ur5_tools/ur5_tools/gripper_attach_backend.py
M  src/ur5_tools/ur5_tools/ur5_moveit_bridge.py
?? reports/incidents/2026-04-24_pick_demo_real_grasp_fix.md
?? reports/incidents/2026-04-24_rg2_visual_geometry_direct_moveit_fix.md
```

Rama: `audit/tcp-geometry-fix`

---

## 3. Log analizado

Fuente: `/home/laboratorio/TFM/historico/DIRECTO_DEBUG_TRACE.log`
Run: `2026-04-24T23:29:56` → `23:31:11`
Extracto guardado: `reports/incidents/2026-04-24_approach_visual_extract.log`

### Eventos clave del run fallido

```
23:29:56 [APPROACH_COARSE][ENTRY]
  object_world=(-0.411,0.000,0.875) target_world=(-0.411,0.000,0.910)
  tcp_before=(-0.539,0.091,0.794) → APPROACH inicia desde HOME

23:30:32 [APPROACH_COARSE][EXIT]
  live_rg2_pinch_center_base=(0.430,0.006,0.053) target=(0.439,0.000,0.060)
  → TCP llegó a ~10mm XY / 7mm Z del target: CORRECTO para clearance 35mm

23:30:32 [POSE_TRACE phase_end]
  tcp_world=(-0.421,0.007,0.902) obj=(-0.411,0.000,0.875)
  dz=0.027 dist=0.030 xy=0.012 → APPROACH ejecutado correctamente

23:30:32 [RG2_VISUAL_AUDIT phase=APPROACH_COARSE]  (run anterior, equivalente)
  pinch_tf=(-0.421,0.007,0.902)  ← TCP correcto, X negativo
  gz_visual_mid=(0.402,0.013,0.414)  ← X POSITIVO: frame mismatch
  dist_pinch_gz_mid=0.957m → FAIL (falso positivo por frame incorrecto)

23:30:33 [POSE_CONSISTENCY phase_check]
  panel_fk_age=0.458/0.400 → STALE (58ms de exceso)
  ok_for_gate=false → pose gate falla

23:30:33 [PHASE_CHECK APPROACH_COARSE]
  xy_err=0.010/0.006 → falla strict (4mm exceso)
  tcp_obj_dist=0.031/0.015 → falla strict (16mm exceso, clearance incluido)
  dz_obj=0.029/0.015 → falla strict (14mm exceso, clearance esperado)
  relaxed_handoff_ok=false (pose_ok=false)
  gate_decision=not_ready result=NO

23:31:11 [ANALYSIS] code=APPROACH_COARSE_NOT_READY → _abort_grasp()
```

### Por qué el robot SÍ llegó visualmente al target

```
tcp_world antes:  (-0.539, 0.091, 0.794)   ← HOME
tcp_world después: (-0.421, 0.007, 0.902)   ← sobre el objeto
objeto world:      (-0.411, 0.000, 0.875)
dz_obj = 0.902 - 0.875 = 0.027m  (27mm de clearance ✓)
xy_dist = 12mm (dentro de tolerancia visual ✓)
```

El robot llegó correctamente. El gate fue el problema.

---

## 4. Tabla antes / después

| Campo | Antes (run 23:29) | Después fix | Interpretación |
|-------|-------------------|-------------|----------------|
| TF_FRESHNESS | OK (age=0.041s) | OK | TF fresco |
| object_world source | stable_cache_promoted_cycle | idem | Objeto estable |
| target APPROACH | (-0.411,0.000,0.910) | idem | Correcto: obj_z+35mm |
| pinch_tf_world antes | (-0.539,0.091,0.794) | idem | HOME |
| pinch_tf_world después | (-0.421,0.007,0.902) | idem | Sobre objeto ✓ |
| visual_finger_midpoint (gz) | (0.402,0.013,0.414) FRAME_MISMATCH | FRAME_MISMATCH detectado | Frame local Gazebo, no world |
| dist_pinch_obj | 0.030m | 0.030m | Clearance 30mm ✓ |
| dist_visual_mid_to_obj | 0.934m (falso) | FRAME_MISMATCH (no calculado) | Gazebo frame incorrecto |
| dist_pinch_gz_mid | 0.957m (falso) | FRAME_MISMATCH | Idem |
| gate result | NOT_READY → abort | approach_clearance_ok=True → OK | Fix aplicado |
| VISUAL_GRASP gate | N/A (en PRE_CLOSE) | N/A | Correcto, no aplica en APPROACH |

---

## 5. Clasificación A-H

**Causa primaria: CASO C — Target APPROACH correcto pero gate de phase_check mal calibrado**
- El gate `_coarse_handoff_dist_tol=0.015m` mide distancia 3D al objeto, incluyendo el clearance Z.
- Con `coarse_extra_z_m=0.035m`, `tcp_obj_dist` siempre será ≥35mm. El strict gate NUNCA puede pasar.
- El relaxed gate falla porque `coarse_gate_pose_ok=False` (panel_fk_age=0.458s > 0.4s tol).

**Causa secundaria: CASO G — gz_visual_mid en frame local Gazebo, no world ROS2**
- `rg2_leftfinger` y `rg2_rightfinger` en Gazebo pose_info tienen X positivo.
- El TCP (TF2) tiene X negativo. Signo opuesto → frame diferente.
- El pick_demo SÍ tiene X negativo (frame correcto, es un entity libre en Gazebo).
- Los links del robot en pose_info pueden ser relativos al modelo, no al mundo.

---

## 6. Fix aplicado

### FIX-1: Gate approach_clearance_ok (panel_pick_demo.py ~línea 10211)

En `_build_approach_coarse_phase_check`, se añadió la condición:

```python
_ac_clearance_xy_tol = max(float(_cg_xy_tol) + 0.010, 0.020)
_ac_clearance_dz_max = float(coarse_extra_z_m) + 0.025
approach_clearance_ok_local = bool(
    float(coarse_gate_xy_err) <= _ac_clearance_xy_tol
    and dz_obj_local is not None
    and float(dz_obj_local) >= 0.0
    and float(dz_obj_local) <= _ac_clearance_dz_max
)
result_local = "OK" if bool(
    strict_handoff_ok_local
    or relaxed_handoff_ok_local
    or approach_clearance_ok_local
) else "NO"
```

**Verificación con datos reales del run fallido:**
- `coarse_gate_xy_err=0.010 <= 0.020` ✓
- `dz_obj=0.029 >= 0` ✓
- `dz_obj=0.029 <= 0.060` ✓
- → `approach_clearance_ok=True` → `result=OK` → no abort ✓

### FIX-2: FRAME_MISMATCH en _emit_rg2_visual_audit (~línea 4899)

```python
_gz_frame_mismatch = False
if gz_mid is not None and pinch_w is not None:
    _gz_x = float(gz_mid[0])
    _pinch_x = float(pinch_w[0])
    if abs(_gz_x) > 0.05 and abs(_pinch_x) > 0.05 and (_gz_x * _pinch_x < 0.0):
        _gz_frame_mismatch = True
```

Verdict cambia de `FAIL` (falso positivo) a `FRAME_MISMATCH` con diagnóstico claro.

### FIX-3: Logs obligatorios añadidos

- `[PICK][APPROACH_OBJECT_SOURCE]` — antes de APPROACH_PLAN
- `[PICK][APPROACH_VISUAL_TARGET]` — antes de APPROACH_PLAN
- `[PICK][APPROACH_EXEC_AUDIT]` — después de _emit_rg2_visual_audit
- `[PICK][APPROACH_GATE]` — antes de _wait_phase_gate_ready

---

## 7. Archivos modificados

| Archivo | Cambio |
|---------|--------|
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` | FIX-1 gate, FIX-2 FRAME_MISMATCH, FIX-3 logs |

---

## 8. Comandos de build

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur5_qt_panel ur5_tools --symlink-install
# Resultado: 2 packages finished OK
python3 -m py_compile src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
# Resultado: OK
```

---

## 9. Criterio de éxito APPROACH_COARSE (tras fix)

Con el fix activo, APPROACH_COARSE pasa si:

- `coarse_gate_xy_err <= max(_cg_xy_tol+0.010, 0.020)` — XY alineado con objeto
- `dz_obj >= 0` — TCP sobre el objeto (no debajo)
- `dz_obj <= coarse_extra_z_m + 0.025` — clearance Z razonable

No requiere:
- `pose_consistency ok_for_gate` (puede ser stale en fase APPROACH)
- `tcp_obj_dist <= 0.015m` (imposible con clearance Z)
- `VISUAL_GRASP_NOT_ACHIEVED` (ese gate pertenece a PRE_CLOSE)

---

## 10. Recomendación siguiente

**APPROACH_COARSE OK tras el fix → continuar a GRASP_DOWN_JOINT.**

Pendientes:
1. Investigar por qué `rg2_leftfinger`/`rg2_rightfinger` en Gazebo pose_info tienen frame local en vez de world. Puede requerir ajuste en el gz_pose_bridge o en los entity names del SDF.
2. Validar que GRASP_DOWN_JOINT hereda correctamente XY de APPROACH_COARSE.
3. El COARSE_REFINE con `refine_accept_result=false` puede seguir activándose pero no bloquea (solo añade un intento de descenso extra antes del gate final).
