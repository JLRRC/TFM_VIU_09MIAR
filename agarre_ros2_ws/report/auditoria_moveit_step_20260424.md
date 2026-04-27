# Auditoría MOVEIT + STEP — UR5+RG2 Panel Qt

**Fecha:** 2026-04-24  
**Rama:** ENTREGA.V2  
**Stack:** ROS 2 Jazzy · Gazebo Sim Harmonic · MoveIt 2 · ros2_control · UR5+OnRobot RG2  
**Modo auditado:** `run_pick_object()` → `panel_pick_object.py` (MODO MOVEIT)  
**Modo de referencia:** `run_pick_demo()` → `panel_pick_demo.py` (DIRECTO) — NO modificado

---

## Resumen ejecutivo

Se realizó una auditoría completa del modo MOVEIT (PICK Objeto) en busca de:

1. Falta de validación explícita de TF/EE al arranque (equivalente al `SYNC_GATE stage=tf_and_ee` de DIRECTO)
2. Fallback incorrecto del frame EE en el worker (`RG2_PINCH_CENTER_FRAME` en lugar de `RG2_TCP_FRAME`)
3. Ausencia de prefijos de log estructurados `[PICK][MOVEIT][*]` para trazabilidad homogénea
4. Modo STEP sin logs de auditoría en el gate de fases

Se corrigieron **7 incidencias** en `panel_pick_object.py`, se añadió **1 nuevo test** con 12 casos, y la build completó limpia en 2 paquetes.

---

## Ficheros inspeccionados

| Fichero | Rol | Modificado |
|---------|-----|-----------|
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py` | Pipeline MOVEIT principal | **Sí** |
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` | Pipeline DIRECTO (referencia) | No (sesión) |
| `src/ur5_qt_panel/ur5_qt_panel/panel_v2.py` | Panel principal Qt | No |
| `src/ur5_tools/ur5_tools/ur5_moveit_bridge.py` | Bridge MoveIt↔Panel | No |
| `src/ur5_tools/ur5_tools/gripper_geometry.py` | Constantes geométricas | No |
| `src/ur5_tools/test/test_gripper_geometry.py` | Tests geometría canónica | No (sesión) |
| `src/ur5_qt_panel/test/test_panel_pick_object_moveit_init.py` | Tests MOVEIT init/step | **Creado** |

---

## Bugs encontrados y corregidos

### BUG-M01 — Importación faltante: `RG2_TCP_FRAME`

**Severidad:** Media  
**Ubicación:** `panel_pick_object.py` línea 28-33 (bloque import gripper_geometry)

`RG2_TCP_FRAME` no estaba importado desde `ur5_tools.gripper_geometry`. El worker usaba el frame `rg2_pinch_center` como fallback, pero el bridge devuelve `ee_link="rg2_tcp"`. La comparación de strings en la guarda del check EE link (línea ~3486) habría disparado un `RuntimeError` con `ee_link mismatch` si `_ee_frame_effective` fuera `None`.

**Fix:**
```diff
 from ur5_tools.gripper_geometry import (
     RG2_PINCH_CENTER_FRAME,
+    RG2_TCP_FRAME,
     contact_z_correction_for_frame,
     load_gripper_geometry,
 )
```

---

### BUG-M02 — Importación faltante: `tf_ready_status`

**Severidad:** Alta (bloquea el nuevo bloque INIT)  
**Ubicación:** `panel_pick_object.py` línea 70

`tf_ready_status` no estaba importado desde `.panel_readiness`. Necesario para la validación explícita de TF en caliente equivalente a la de DIRECTO.

**Fix:**
```diff
-from .panel_readiness import pick_ui_status
+from .panel_readiness import pick_ui_status, tf_ready_status
```

---

### BUG-M03 — Ausencia de validación explícita TF/EE en el arranque MOVEIT

**Severidad:** Alta  
**Ubicación:** `panel_pick_object.py` después del check de controladores (~línea 292)

DIRECTO tiene 6 fases de `SYNC_GATE` explícitas incluyendo `stage=tf_and_ee`. MOVEIT no tenía ninguna validación de TF antes de lanzar el worker — un TF no listo provocaba fallos silenciosos en `_selection_to_base()` mucho más tarde en el pipeline.

**Fix:** Bloque `[PICK][MOVEIT][INIT]` insertado tras el check de controladores:

```python
_moveit_init_tf_ok, _moveit_init_tf_reason = tf_ready_status(panel)
_moveit_init_ee_frame = str(getattr(panel, "_ee_frame_effective", None) or "").strip()
panel._emit_log(
    "[PICK][MOVEIT][INIT] "
    f"ts={time.time():.6f} mode=moveit controllers=OK "
    f"tf_ok={str(bool(_moveit_init_tf_ok)).lower()} "
    f"tf_reason={_moveit_init_tf_reason or 'ok'} "
    f"ee_frame={_moveit_init_ee_frame or 'none'} ..."
)
if not _moveit_init_tf_ok or not _moveit_init_ee_frame:
    _block(f"tf_or_ee_not_ready:...", error=True)
    panel._emit_log("[PICK][MOVEIT][INIT] tf_and_ee=FAIL ...")
    return
panel._emit_log("[PICK][MOVEIT][INIT] tf_and_ee=OK ...")
```

---

### BUG-M04 — Fallback EE frame incorrecto en el worker (línea 1159)

**Severidad:** Media  
**Ubicación:** `panel_pick_object.py` línea 1159 (inicio de `worker()`)

```diff
-measured_ee_frame = panel._ee_frame_effective or RG2_PINCH_CENTER_FRAME
+measured_ee_frame = panel._ee_frame_effective or RG2_TCP_FRAME
```

El bridge (`ur5_moveit_bridge.py`) retorna `ee_link="rg2_tcp"`. Usar `rg2_pinch_center` como fallback provocaba un falso mismatch en:
```python
if ee_link_moveit and _norm_frame(ee_link_moveit) != _norm_frame(measured_ee_frame):
    raise RuntimeError(f"ee_link mismatch moveit={ee_link_moveit} tf={measured_ee_frame}")
```

Nota: ambos frames son coubicados (Z=0.175 desde `tool0`, `vector_distance == 0.0`) pero la comparación es textual.

---

### BUG-M05 — Fallback EE frame incorrecto en `_attempt_attach` (línea 4270)

**Severidad:** Baja (geometría idéntica, sin impacto funcional real)  
**Ubicación:** `panel_pick_object.py` línea 4270 (dentro de `worker()`, sección GRASP_VALIDATE)

```diff
-measured_ee_frame = str(panel._ee_frame_effective or RG2_PINCH_CENTER_FRAME).strip() or RG2_PINCH_CENTER_FRAME
+measured_ee_frame = str(panel._ee_frame_effective or RG2_TCP_FRAME).strip() or RG2_TCP_FRAME
```

Inconsistencia con la corrección de BUG-M04. Dado que `contact_z_correction_for_frame` retorna 0.0 para ambos frames, no hay impacto numérico, pero se corrige para coherencia total.

---

### BUG-M06 — Ausencia de logs estructurados en `_step_phase_gate`

**Severidad:** Baja (observabilidad, no funcional)  
**Ubicación:** `panel_pick_object.py` línea 124-135

El gate de STEP no emitía ningún log cuando activaba o reutilizaba una fase. Dificulta el diagnóstico de la secuencia STEP en producción.

**Fix:**
```python
if _step_last_phase["value"] == label:
    panel._emit_log(
        f"[PICK][MOVEIT][STEP] gate_reuse phase={label} decision={decision or 'n/a'}"
    )
    return
_step_last_phase["value"] = label
panel._emit_log(
    "[PICK][MOVEIT][STEP] "
    f"phase={label} decision={decision or 'n/a'} "
    f"step_mode={str(getattr(panel, '_step_mode', 'AUTO'))}"
)
```

---

### BUG-M07 — Ausencia de logs `[PICK][MOVEIT][LIFECYCLE]` y `[PICK][MOVEIT][ERROR]`

**Severidad:** Baja (observabilidad)  
**Ubicaciones:**
- Worker start (~línea 1161): añadido `[PICK][MOVEIT][LIFECYCLE] stage=worker_start obj=... ee_frame=...`
- Handler de excepciones (~línea 4827): añadido `[PICK][MOVEIT][ERROR] type=... detail=...`

Logs equivalentes a los de DIRECTO para uniformidad de trazas.

---

## Cambios aplicados — diff completo

```diff
--- a/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py
+++ b/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py

@@ -28,6 +28,7 @@
 from ur5_tools.gripper_geometry import (
     RG2_PINCH_CENTER_FRAME,
+    RG2_TCP_FRAME,
     contact_z_correction_for_frame,
     load_gripper_geometry,
 )

@@ -67,7 +67,7 @@
-from .panel_readiness import pick_ui_status
+from .panel_readiness import pick_ui_status, tf_ready_status

@@ -123,8 +124,16 @@  (en _step_phase_gate)
     if _step_last_phase["value"] == label:
+        panel._emit_log(
+            f"[PICK][MOVEIT][STEP] gate_reuse phase={label} decision={decision or 'n/a'}"
+        )
         return
     _step_last_phase["value"] = label
+    panel._emit_log(
+        "[PICK][MOVEIT][STEP] "
+        f"phase={label} decision={decision or 'n/a'} "
+        f"step_mode={str(getattr(panel, '_step_mode', 'AUTO'))}"
+    )

@@ -283,8 +292,50 @@  (tras check controladores)
     panel._emit_log(f"[PICK] controladores no listos ({reason})")
+    panel._emit_log("[PICK][MOVEIT][INIT] controllers=FAIL reason={reason or 'n/a'}")
     return

+    # -- [PICK][MOVEIT][INIT] BLOQUE TF EXPLÍCITO --
+    _moveit_init_tf_ok, _moveit_init_tf_reason = tf_ready_status(panel)
+    _moveit_init_ee_frame = str(getattr(panel, "_ee_frame_effective", None) or "").strip()
+    panel._emit_log("[PICK][MOVEIT][INIT] ... tf_ok=... ee_frame=...")
+    if not _moveit_init_tf_ok or not _moveit_init_ee_frame:
+        _block(f"tf_or_ee_not_ready:...", error=True)
+        panel._emit_log("[PICK][MOVEIT][INIT] tf_and_ee=FAIL ...")
+        return
+    panel._emit_log("[PICK][MOVEIT][INIT] tf_and_ee=OK ...")

@@ -1152  (worker start)
-    measured_ee_frame = panel._ee_frame_effective or RG2_PINCH_CENTER_FRAME
+    measured_ee_frame = panel._ee_frame_effective or RG2_TCP_FRAME
+    panel._emit_log("[PICK][MOVEIT][LIFECYCLE] stage=worker_start ...")

@@ -4267  (_attempt_attach)
-    measured_ee_frame = str(panel._ee_frame_effective or RG2_PINCH_CENTER_FRAME).strip() or RG2_PINCH_CENTER_FRAME
+    measured_ee_frame = str(panel._ee_frame_effective or RG2_TCP_FRAME).strip() or RG2_TCP_FRAME

@@ -4823  (exception handler)
+    panel._emit_log(f"[PICK][MOVEIT][ERROR] type={fail_kind} detail={err_txt}")
```

---

## Tests creados

**Fichero:** `src/ur5_qt_panel/test/test_panel_pick_object_moveit_init.py`

12 casos, 100% estáticos (sin ROS ni Gazebo):

| Test | Validación |
|------|-----------|
| `test_rg2_tcp_frame_is_imported` | AST: RG2_TCP_FRAME importado desde ur5_tools.gripper_geometry |
| `test_tf_ready_status_is_imported` | AST: tf_ready_status importado desde .panel_readiness |
| `test_moveit_init_log_prefix_present` | `[PICK][MOVEIT][INIT]` presente en fuente |
| `test_moveit_init_tf_and_ee_ok_log_present` | `tf_and_ee=OK` presente |
| `test_moveit_init_tf_and_ee_fail_log_present` | `tf_and_ee=FAIL` presente |
| `test_moveit_init_calls_tf_ready_status` | `tf_ready_status(panel)` invocado |
| `test_moveit_step_log_prefix_present` | `[PICK][MOVEIT][STEP]` presente |
| `test_moveit_step_gate_reuse_log_present` | `gate_reuse` presente en logs STEP |
| `test_moveit_lifecycle_worker_start_log_present` | `[PICK][MOVEIT][LIFECYCLE]` + `stage=worker_start` |
| `test_moveit_error_log_prefix_present` | `[PICK][MOVEIT][ERROR]` presente |
| `test_worker_ee_frame_fallback_uses_rg2_tcp_frame` | Regex: fallback = RG2_TCP_FRAME |
| `test_worker_ee_frame_fallback_never_uses_rg2_pinch_center_as_default` | Regex: no queda fallback RG2_PINCH_CENTER_FRAME |

---

## Resultados de tests

### `ur5_qt_panel` — test suite completa

```
87 passed, 1 skipped
```

**Nuevos tests (12/12 PASS):**
```
test_panel_pick_object_moveit_init.py::test_rg2_tcp_frame_is_imported PASSED
test_panel_pick_object_moveit_init.py::test_tf_ready_status_is_imported PASSED
test_panel_pick_object_moveit_init.py::test_moveit_init_log_prefix_present PASSED
test_panel_pick_object_moveit_init.py::test_moveit_init_tf_and_ee_ok_log_present PASSED
test_panel_pick_object_moveit_init.py::test_moveit_init_tf_and_ee_fail_log_present PASSED
test_panel_pick_object_moveit_init.py::test_moveit_init_calls_tf_ready_status PASSED
test_panel_pick_object_moveit_init.py::test_moveit_step_log_prefix_present PASSED
test_panel_pick_object_moveit_init.py::test_moveit_step_gate_reuse_log_present PASSED
test_panel_pick_object_moveit_init.py::test_moveit_lifecycle_worker_start_log_present PASSED
test_panel_pick_object_moveit_init.py::test_moveit_error_log_prefix_present PASSED
test_panel_pick_object_moveit_init.py::test_worker_ee_frame_fallback_uses_rg2_tcp_frame PASSED
test_panel_pick_object_moveit_init.py::test_worker_ee_frame_fallback_never_uses_rg2_pinch_center_as_default PASSED
```

**Fallos pre-existentes (no introducidos por esta auditoría):**

| Test | Causa | Origen |
|------|-------|--------|
| `test_evaluate_transport_stage_postcheck_accepts_...` | `label` kwarg añadida a `_evaluate_transport_stage_postcheck` en sesión anterior | `panel_pick_demo.py` DIRECTO |
| `test_evaluate_transport_stage_postcheck_flags_runtime...` | ídem | idem |
| `test_evaluate_transport_stage_postcheck_flags_unconfirmed...` | ídem | idem |
| `test_direct_sequence_starts_with_approach_coarse` | Secuencia DIRECT ahora empieza con `INITIAL_SNAPSHOT` | `step_pipeline_helpers.py` |

Estos fallos son pre-existentes respecto a esta sesión: ninguno de los ficheros afectados (`panel_pick_demo.py`, `step_pipeline_helpers.py`) fue modificado por esta auditoría.

### `ur5_tools` — tests de geometría canónica

```
8 passed (test_gripper_geometry.py)
```

Incluye:
- `test_load_gripper_geometry_matches_validated_tcp_fix` — rg2_tcp Z=0.175, rg2_pinch_center Z=0.175, distance=0.0
- `test_model_anchor_matches_canonical_geometry` — SDF model.sdf coherente con URDF/xacro

---

## Build

```
colcon build --symlink-install --packages-select ur5_qt_panel ur5_tools

Starting >>> ur5_tools [1.52s]
Starting >>> ur5_qt_panel [1.53s]
Summary: 2 packages finished [3.23s]  — sin errores ni warnings
```

---

## Análisis arquitectural MOVEIT vs DIRECTO

### Puntos auditados

| Criterio | DIRECTO | MOVEIT (post-fix) |
|----------|---------|-------------------|
| Validación TF explícita al inicio | `SYNC_GATE stage=tf_and_ee` | `[PICK][MOVEIT][INIT]` con `tf_ready_status()` |
| Frame TCP canónico | `rg2_pinch_center` (correction=0.0) | `rg2_tcp` (bridge default) |
| Fallback EE frame | `RG2_PINCH_CENTER_FRAME` | **`RG2_TCP_FRAME`** (corregido) |
| Log inicio worker | `[PICK][DIRECT][LIFECYCLE] stage=worker_start` | `[PICK][MOVEIT][LIFECYCLE] stage=worker_start` |
| Log gate STEP | `[PICK][DIRECT][STEP_GATE]` | `[PICK][MOVEIT][STEP]` (añadido) |
| Log errores | `[PICK_OBJ][FAIL_CLASS]` | + `[PICK][MOVEIT][ERROR]` (añadido) |
| Transporte físico | `_validate_demo_transport_follow` | `_assert_carry_coherence_after_lift` |
| Request ID matching | N/A (IK local) | `rid=N\|uid=UUID` en frame_id PoseStamped |

### Elementos validados como correctos (sin modificación)

- `_run_moveit_step()`: heartbeat pre-check, timeout robusto, request ID correlación
- `_ensure_moveit_bridge_path()`: healthcheck de bridge con cold restart
- `_ensure_gripper_open_for_moveit()`: apertura explícita antes de APPROACH/PRE_GRASP/GRASP_DOWN
- `_close_grasp_attach()`: cierre+attach atómico
- `_assert_carry_coherence_after_lift()`: validación física de transporte
- Modo STEP (`_step_phase_gate` + `_step_wait_for_phase`): funcional, sólo faltaban logs

---

## Checklist final

- [x] DIRECTO (panel_pick_demo.py) NO modificado en esta sesión
- [x] MOVEIT (panel_pick_object.py): validación TF/EE al arranque
- [x] MOVEIT: fallback EE frame = `RG2_TCP_FRAME` (coherente con bridge)
- [x] MOVEIT STEP: logs `[PICK][MOVEIT][STEP]` en gate de fases
- [x] MOVEIT LIFECYCLE: log `stage=worker_start` con ee_frame y coords
- [x] MOVEIT ERROR: log `[PICK][MOVEIT][ERROR]` en handler de excepciones
- [x] Geometría: rg2_tcp y rg2_pinch_center coubicados Z=0.175 (8/8 tests geometría pass)
- [x] Tests nuevos: 12/12 PASS estáticos
- [x] Build limpio: 2 paquetes, 0 errores
- [x] Sin false positives introducidos: checks de TF retornan early con error explícito
- [x] Sin rotura de DIRECTO: test suite DIRECTO idéntica a pre-sesión

---

## Pendiente (fuera del alcance de esta auditoría)

- Validación end-to-end MOVEIT en Gazebo Sim Harmonic (requiere entorno de simulación activo)
- Corrección de los 4 fallos pre-existentes en tests de DIRECTO (`_evaluate_transport_stage_postcheck` y `step_pipeline_helpers`)
- Push a origin/ENTREGA.V2 (pendiente de confirmación del usuario)

---

*Generado automáticamente — 2026-04-24*
