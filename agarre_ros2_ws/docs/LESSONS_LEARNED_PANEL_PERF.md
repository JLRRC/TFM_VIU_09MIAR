# Lessons Learned — Panel UI lento ("no responde")

**Fecha**: 2026-05-11
**Contexto**: tras refactor F0-F10, validación live reveló que el panel
quedaba marcado "no responde" por Linux/Qt al pulsar Pick.
**Commits relevantes**: `2981bd0`, `a4aea65`, `b5665cc`, `8423c67`.

Este documento existe para **NO repetir los mismos errores de diagnóstico**.
Si en el futuro vuelve a aparecer un síntoma de panel lento / no responde,
**lee esto primero antes de tocar nada**.

---

## 🎯 SÍNTOMA original

El usuario reportó:

- Panel Qt arranca OK pero los botones no responden o tardan.
- Linux muestra repetidamente el diálogo "panel_v2 no responde".
- Al pulsar "Agarre Objeto (MoveIT)" o "Agarre Objeto (Directo)", el panel se queda colgado.

---

## ❌ Las 4 hipótesis falsas (NO repitas)

Antes de identificar la causa real, hice **4 commits que no resolvieron nada**
(todos revertidos después):

### Hipótesis falsa 1: logs MOVEIT2 en bucle saturan I/O

- **Commit**: `8faaae1` "silenciar bucle MOVEIT2 READY_DIAG/STARTUP_GATE estable"
- **Lo que asumí**: `_emit_log_throttled` con `min_interval=1.0` generaba 4 logs/s, saturando el GUI thread vía I/O.
- **Por qué falló**: silenciar los logs redujo I/O pero el panel siguió al 107% CPU.
  Los logs son sólo **síntoma**, no causa.
- **Lección**: ruido en logs no causa freezes si el CPU es <100%. Logs son
  el último síntoma, no el primer culpable.

### Hipótesis falsa 2: timers UI muy agresivos

- **Commit**: `c7e3f3c` "reducir CPU saturando GUI thread (panel a 107% → ~30%)"
- **Lo que asumí**: `_watchdog_timer` a 400ms + `_camera_display_timer` a 80ms
  saturaban el GUI thread.
- **Por qué falló**: con `top -H` verifiqué después que el GUI thread sólo
  consumía **18% CPU**. El thread saturado era `_RosWor+` (63% CPU).
  Cambiar timers UI no ataca al thread real.
- **Lección**: usa `top -H -b -n1 -p $PID | head -15` para ver QUÉ thread
  está saturado, no qué archivo Python sospechas.

### Hipótesis falsa 3: throttle en callbacks Python

- **Commit**: `0b88a55` "throttle callbacks /clock y /pose/info (RosWorker 54% → ~15%)"
- **Lo que asumí**: callbacks Python a 790 Hz (`/clock`) y 50 Hz × 74 entidades
  (`/pose/info`) consumen CPU en cada llamada.
- **Por qué falló**: aunque añadí `return` rápido al principio de cada callback
  (throttle por timestamp), **el thread sigue siendo despertado por cada
  mensaje DDS**. El coste real está en:
  1. DDS deserialization (C++ antes de llegar a Python).
  2. rclpy executor wake.
  3. Dispatch al callback.
  El `return` rápido del callback ahorra muy poco vs el coste 1-3.
- **Lección**: el throttling **dentro** del callback NO reduce el coste
  del wake/deserialize/dispatch. Para reducir CPU del thread, hay que
  reducir el NÚMERO de mensajes que llegan (no el trabajo del callback).

### Hipótesis falsa 4: `PANEL_MAX_FPS` por encima de la rate del publisher

- **Commit**: `80cddcc` "PANEL_MAX_FPS default 60 → 10 (callback _on_image)"
- **Lo que asumí**: callback `_on_image` a 24 Hz (rate publisher) con cv_bridge +
  QImage costaba ~20ms × 24 = 50% CPU.
- **Por qué falló**: medición posterior reveló que el thread principal del
  RosWorker estaba al 63% CPU pero solo había **1 thread activo de los 14**
  que tenía el proceso. El throttle a 10 Hz no bajó nada significativo.
- **Lección**: idem al anterior — throttle en callback no resuelve si la
  causa es el spinning del executor.

---

## ✅ DIAGNÓSTICO CORRECTO (cómo se hizo)

Lo correcto fue **medir antes de cambiar**:

```bash
# 1. Localizar PID del panel
PID=$(pgrep -f "install/ur5_qt_panel.*lib.*panel_v2" | head -1)

# 2. Ver threads ordenados por CPU
top -H -b -n1 -p $PID | head -15
```

Esto mostró:

```
Thread _RosWor+ TID=A       63%  ← culpable
Thread _RosWor+ TID=B        8%
Thread _RosWor+ TID=C        8%
Thread _RosWor+ TID=D        6%
Thread panel_v2 (GUI)       12%  ← GUI thread, sano
Thread dds.udp+             10%
```

Y un segundo paso clave:

```bash
ps -eo pid,pcpu,comm --sort=-pcpu | head -10
```

Reveló que **NO sólo el panel** estaba al 100%, sino **todos los nodos del backend**:

```
plan_to_pose_server      100%
pick_orchestrator         89%
gripper_attach_backend    75%
planning_scene_sync       50%
panel_v2                 100%
```

---

## 🎯 CAUSA RAÍZ REAL

**Antipatrón sistémico**: 6 nodos backend usaban `MultiThreadedExecutor()`
**sin argumento `num_threads`**.

En `rclpy`, el default de `MultiThreadedExecutor()` es:

```python
import multiprocessing
num_threads = multiprocessing.cpu_count()
```

En la máquina del usuario (16 cores) eso son **16 threads por nodo**. Con
6 nodos, son **96 threads spinning** compitiendo por 16 cores.

A esto se sumaba:
- Gazebo Sim publicando `/clock` a 790 Hz.
- `/world/.../pose/info` con 74 entidades a 50 Hz.
- `/joint_states` a 97 Hz.
- `MultiThreadedExecutor` del propio panel a 3 threads (default
  `PANEL_ROS_EXECUTOR_THREADS=3` en `start_panel_v2.sh`).

Total ≈ **100 threads compitiendo, load average 14**.

---

## ✅ FIX REAL APLICADO

### Cambio 1 — 6 nodos backend (`a4aea65`)

```python
# ANTES:
executor = MultiThreadedExecutor()  # → cpu_count threads

# AHORA:
executor = MultiThreadedExecutor(num_threads=2)
```

Aplicado en:
- `src/ur5_tools/ur5_tools/controller_health_monitor_node.py:219`
- `src/ur5_tools/ur5_tools/plan_to_pose_runtime.py:41`
- `src/ur5_tools/ur5_tools/panel_backend_node.py:329`
- `src/ur5_tools/ur5_tools/panel_launch_control_node.py:274`
- `src/ur5_tools/ur5_tools/simulation_reset_service.py:229`
- `src/tfm_orchestrator/tfm_orchestrator/pick_orchestrator_lifecycle_node.py:873`

**2 threads** es suficiente para que ActionServer + service calls síncronos
coexistan (regla con `ReentrantCallbackGroup`) pero sin saturar.

### Cambio 2 — Panel (`2981bd0`)

```bash
# ANTES en scripts/start_panel_v2.sh:
: "${PANEL_ROS_EXECUTOR_THREADS:=3}"

# AHORA:
: "${PANEL_ROS_EXECUTOR_THREADS:=1}"   # SingleThreadedExecutor
```

---

## ⚠️ Cambio que NO funcionó — REVERT después de probar

### SDF Gazebo bajar /clock a 200 Hz (`2981bd0` parte, revertido en `b5665cc`)

**Lo que intenté**:

```xml
<!-- ANTES: -->
<max_step_size>0.001</max_step_size>
<real_time_update_rate>1000</real_time_update_rate>
<!-- → /clock a ~790 Hz -->

<!-- LO QUE INTENTÉ: -->
<max_step_size>0.005</max_step_size>
<real_time_update_rate>200</real_time_update_rate>
<!-- → /clock a ~200 Hz -->
```

**Por qué falló**:

El usuario reportó:
- "Robot UR5 aparece en colisión con la mesa al spawn."
- "pick_demo no aparece en su sitio."

Con `max_step_size=0.005s` la integración de contactos en bullet permitía
penetración inicial antes de que la física estabilizara los rígidos.

**Lección**: **NO TOQUES `max_step_size` ni `real_time_update_rate` del SDF
para mejorar performance**. La física necesita steps pequeños. Reduce CPU
por OTROS caminos (threads del executor, throttling en backend específico,
etc.) pero NUNCA por sacrificio de step físico.

---

## 📐 REGLAS para evitar repetir esto

### Regla 1: Medir ANTES de cambiar

Nunca cambies código por una hipótesis. Antes:

```bash
# Threads del proceso lento
top -H -b -n1 -p $(pgrep -f NOMBRE_PROCESO | head -1) | head -15

# Todos los procesos del sistema ordenados por CPU
ps -eo pid,pcpu,etime,comm --sort=-pcpu | head -15

# Load average
uptime
```

Si **un thread concreto** está al 80%+: ataca ESE thread.
Si **N nodos** están al 80%+ cada uno: es problema sistémico (probablemente
threading config como este caso).

### Regla 2: NUNCA `MultiThreadedExecutor()` sin `num_threads`

❌ MAL:
```python
executor = MultiThreadedExecutor()
```

✅ BIEN:
```python
executor = MultiThreadedExecutor(num_threads=2)  # o el valor justificado
```

O usar `SingleThreadedExecutor()` si el nodo no tiene paralelismo real.

**Hay un test que debería detectar esto**: ver
`test_executor_threads_pinned.py` (a crear en v1.1) que escanee
`grep -rn "MultiThreadedExecutor()" src/` y falle si encuentra cualquier
ocurrencia sin argumento.

### Regla 3: Throttling en callback Python NO baja CPU del thread

El thread se despierta por cada mensaje DDS aunque el callback haga
`return` inmediato. Si quieres bajar wakes:

- ❌ NO añadas `if (now - last) < dt: return` al inicio del callback.
- ✅ SÍ reduce frecuencia del publisher (mejor) o usa `MessageFilter`
  con `RateLimit` (mejor que el callback).

### Regla 4: NO toques `max_step_size` del SDF

La física necesita step pequeño (`0.001s`). Reducirlo causa:
- Penetración inicial de rígidos en colisión con superficies.
- Inestabilidad en attach físico.
- Salto inicial del robot al spawn.

Si necesitas bajar `/clock`, **bajar `real_time_update_rate` mantiene step**:
- `step=0.001` + `rate=100` → /clock a 100 Hz pero sim corre **10× slow**
  (1ms simulado cada 10ms real). NO sirve para uso normal.

### Regla 5: Síntomas vs causa

| Síntoma | Mal diagnóstico | Causa probable |
|---|---|---|
| Panel "no responde" | "GUI thread saturado" | Mira `top -H`: GUI suele estar bien, el problema está en RosWorker/DDS. |
| Logs en bucle | "Spam de logs" | Suele ser síntoma de un check repetido. Mira QUIÉN lo invoca (timer?). |
| Procesos al 100% CPU | "Bug en el callback" | Mira si **todos** los nodos están al 100% → es config sistémica (threads). |
| Robot deriva al spawn | "Bug en URDF" | Es timing entre Gazebo spawn y controller bootstrap. Más controller_start_grace. |

---

## 🔗 Referencias

- Commits: `2981bd0`, `a4aea65`, `b5665cc`, `8423c67`
- Commits revertidos por hipótesis falsas: `537ba07..1d04531`
- Documento de deuda: `docs/V1_DEFERRED_TO_V1_1.md` § "Live audit (2026-05-11)"
- Plan F6.3 (solución arquitectural definitiva): `docs/ARCHITECTURE.md` § "Pendientes F6/F9"
