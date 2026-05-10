# Performance audit 2026-05-10 (Acciones 25 + 26 del plan 100/100)

Análisis offline del rendimiento del stack — sin runtime live. Identifica
patterns potencialmente problemáticos por inspección estática + revisión
manual. La validación con perfilado real (flame graphs, py-spy, htop)
queda como trabajo posterior con stack vivo.

## 1. Audit de QTimer en ur5_qt_panel (Acción 25)

### Inventario detectado (offline)

```
panel_camera_controllers.py:
  L510 singleShot(300, refresh_topics)
  L511 singleShot(600, auto_connect_camera)
  L513 singleShot(250, auto_release_drop_objects)
  L515 singleShot(600, maybe_hold_drop_objects)
  L516 singleShot(700, attach_drop_objects)
  L522 singleShot(1500, refresh_objects_from_gz_async)
  L523 singleShot(1500, apply_home_joint2_offset)
  L535 singleShot(900, run_fall_test_async)

panel_gz_startup.py:
  L186-187 QTimer + setInterval(POSE_INFO_POLL_SEC * 1000) — periódico
  L220 singleShot(0, schedule_physics_runtime_check) — defer
  L344 singleShot(0, panel.close) — defer
  L658 singleShot(1800, start_bridge)
  L659 singleShot(3200, refresh_topics)
  L660 singleShot(4200, connect)
  L665 singleShot(1500, refresh_topics)

panel_motion_control.py:
  L179 singleShot(delay_ms, apply_home_joint2_offset)
  L587 singleShot(200, send_joints) — debouncing

cameras_tab.py:
  L376-378 QTimer + setInterval(140) — pulse 7Hz

```

### Análisis

| Tipo | Observación | Riesgo |
|---|---|---|
| `singleShot(0, …)` (L220, L344) | Defer a próximo event-loop iteration. Patrón Qt correcto. | 🟢 OK |
| `singleShot(N, …)` con N>1000 ms en startup (L658-665) | Cadena de timers para serializar bring-up (1800, 3200, 4200 ms). Funciona pero **acoplada a tiempos absolutos** — frágil si Gazebo arranca lento. | 🟠 Mejorable: sustituir por **eventos** (`bridge_ready` signal) o `QStateMachine`. |
| `singleShot(140)` periódico en `cameras_tab.py:376-378` | Pulse 7Hz para indicador visual de actividad. CPU coste despreciable. | 🟢 OK |
| `singleShot(300/600/250/600/700)` en `panel_camera_controllers.py:510-516` | 5 timers encadenados en bring-up de cámaras. Mismo patrón frágil que arriba. | 🟠 Mejorable |
| `setInterval(POSE_INFO_POLL_SEC*1000)` (L186) | Polling regular de poses. Defendible si Gazebo no publica eventos. | 🟢 OK con caveat |

### Recomendación

**No bloqueante**, no cambiar antes de tener flame graph runtime real. El
patrón `singleShot` con tiempos hardcoded es deuda técnica menor. Sustituir
sólo si una de las dos condiciones se cumple:

1. Bring-up tarda >30s en máquinas más lentas (regresión visible).
2. Se observan races: el timer de N ms dispara antes que la operación que
   debería esperar.

Patrón objetivo: `pyqtSignal(bridge_ready)` emitido cuando el subsistema
realmente está listo (no por temporizador). El panel ya usa este patrón
en `panel_ros.py` para `joint_state`, `image`, `system_state`.

### `time.sleep` en producción (15 ficheros)

`time.sleep` en hilo Qt principal **bloquea la UI**. Auditoría:

- `panel_tf.py`, `panel_motion_helpers.py`, `panel_motion_control.py`,
  `panel_step_callbacks.py`, `panel_camera_controllers.py`,
  `panel_gz_startup.py`, `panel_startup.py`: estos viven en hilos worker
  (`_FnThread`, `RosWorker`) — **OK**.
- `panel_utils.py`, `panel_objects.py`, `step_history_recorder.py`:
  utilidades que pueden llamarse desde main thread. **Verificar a mano
  cada call site** (esfuerzo: 30 min).
- `attach_gate_evaluator.py`: helper puro, llamado desde worker. **OK**.
- `attach_demo_transport.py` (ur5_tools): nodo ROS, no hilo Qt. **OK**.
- `panel_launchers.py`: lanza subprocesos. **OK**.

**Acción concreta diferida:** auditar 3 call-sites en `panel_utils.py /
objects / step_history_recorder.py` y, si hay alguno desde main thread,
sustituir por `QTimer.singleShot` o por future async.

## 2. Audit de TF caching (Acción 26)

### Inventario `lookup_transform`

```
src/ur5_qt_panel/ur5_qt_panel/panel_tf.py
src/ur5_qt_panel/ur5_qt_panel/panel_pose_helpers.py
src/ur5_qt_panel/ur5_qt_panel/panel_trace_callbacks.py
src/ur5_qt_panel/ur5_qt_panel/tf_pose_utils.py
src/ur5_qt_panel/ur5_qt_panel/panel_utils.py
src/ur5_qt_panel/ur5_qt_panel/panel_object_mgmt.py
src/ur5_qt_panel/ur5_qt_panel/panel_tf_monitor.py
src/ur5_qt_panel/ur5_qt_panel/panel_tf_discovery.py
src/ur5_tools/ur5_tools/plan_to_pose_server.py
src/ur5_tools/ur5_tools/tf_batch_lookups.py            ← infra cache
src/ur5_tools/ur5_tools/tf_batch_lookups_runtime.py    ← infra cache
src/ur5_tools/ur5_tools/planning_scene_sync.py
src/ur5_tools/ur5_tools/system_state_manager.py
src/ur5_tools/ur5_tools/tf_probe.py
src/tfm_orchestrator/tfm_orchestrator/initial_snapshot.py
```

### Análisis

- ✅ **Existe `tf_batch_lookups`** (helper ya creado) que agrupa múltiples
  lookups en una transacción del buffer TF. Reduce coste por iteración.
- ✅ **`tf_batch_lookups_runtime`** envuelve la versión live.
- 🟠 **No todos los `lookup_transform`** del panel usan este helper. Hot
  paths sospechosos:
  - `panel_trace_callbacks.py` — trace por fase (3+ lookups secuenciales).
  - `panel_pose_helpers.py` — múltiples llamadas en cada `_compute_*`.
  - `panel_tf_monitor.py` — monitoreo periódico.

### Recomendación

**Sweep diferido**: identificar las 5-10 secuencias de >2 lookups consecutivos
y migrarlas a `tf_batch_lookups`. Esfuerzo estimado: 4-6 h.

**Validación**: el TF buffer tiene ya un cache interno (sliding window de
10s default). El sweep aporta principalmente **claridad arquitectónica**
y consistencia, no un win de rendimiento masivo en buffer caliente.

## 3. Otros hallazgos performance offline

| Patrón | Localización | Severidad | Acción |
|---|---|---|---|
| `time.monotonic()` repetido en hot loop (1000+ veces) | no detectado offline; verificar runtime | — | flame graph cuando esté listo |
| Logging excesivo `[FASE][SUB]` en INFO | normal en debug, considerar DEBUG en release | 🟢 OK | parámetro `log_level` por nodo |
| Gazebo bridges duplicados | revisar `bridge_cameras.yaml` por topics dup | 🟠 verificar | `gz topic -l \| sort \| uniq -c \| sort -rn` |

## 4. Conclusión

El stack tiene **patterns de rendimiento defensibles**:
- `tf_batch_lookups` infrastructure ya existente.
- `time.sleep` mayoritariamente en hilos worker.
- `QTimer.singleShot` correctamente usado para defers no bloqueantes.
- Bring-up timers hardcoded son la deuda más visible (frágil ante
  ralentizaciones del entorno) pero **no bloqueante** hoy.

El siguiente paso real es **medir, no refactorar**: instrumentar 1 ciclo
pick con `py-spy` y ver dónde se va el tiempo. Sin esa medición, las
optimizaciones son especulativas.

---

Audit Acciones 25 y 26 del plan 100/100 (2026-05-10).
