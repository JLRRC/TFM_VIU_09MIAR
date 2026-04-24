# INCIDENTE CRÍTICO: robot_state_publisher muerto — TF stale — TCP OK falso en panel
**Fecha:** 2026-04-24  
**Severidad:** CRÍTICA  
**Rama:** audit/tcp-geometry-fix  
**Estado:** ABIERTO — fixes pendientes

---

## 1. Resumen ejecutivo

`robot_state_publisher` (PID 2431825) murió durante la sesión de runtime sin que el stack lo detectase ni se recuperase. Como consecuencia:

- `/tf` perdió todos sus publishers → los transforms dinámicos de articulaciones del brazo dejaron de actualizarse
- El panel continuó funcionando con su buffer TF pre-cargado (poblado cuando RSP estaba vivo)
- `lookup_transform(..., Time())` devuelve el último dato del buffer sin verificar su antigüedad → el panel reporta `tcp_live_source=tf2:ok` con datos potencialmente estantados
- `system_state_manager._tf_ok()` usa la misma semántica de `Time(0)` → no detecta el estado degradado
- Las fases APPROACH, GRASP, PRE_CLOSE del pick recibieron coordenadas TCP aparentemente válidas, pero derivadas de TF stale
- `inspect_rg2_visual_pose` ejecutado externamente muestra todos los frames URDF como N/A porque su propio buffer TF no pudo recibir datos dinámicos de un RSP ya muerto

**El robot puede estar ejecutando picks con localización TCP incorrecta sin saberlo.**

---

## 2. Evidencia

### 2.1 PID de robot_state_publisher muerto

```
# Stack log: RSP arrancó correctamente
[INFO] [robot_state_publisher-2]: process started with pid [2431825]
[robot_state_publisher-2] [INFO] [1777057394.828877944] [robot_state_publisher]: Robot initialized

# Verificación posterior: PID muerto
$ ps -p 2431825 -o pid,ppid,stat,comm,args
    PID    PPID STAT COMMAND         COMMAND
→ salida vacía → DEAD

# Búsqueda exhaustiva: ningún proceso robot_state_publisher en sistema
$ ps aux | grep robot_state_publisher  → vacío
$ ls -la /proc/*/exe 2>/dev/null | grep robot_state → vacío
```

**Tiempo de vida estimado:** RSP arrancó a las 21:03:59. La última entrada de su log es timestamp 1777047760 (~18:09 de la misma sesión o de una sesión anterior del mismo día). Muerte no registrada en stack log (589 líneas, sin entrada de muerte de RSP).

### 2.2 `/tf` con 0 publishers

```
$ ros2 topic info /tf --verbose
Type: tf2_msgs/msg/TFMessage
Publisher count: 0
Subscription count: 17

$ ros2 topic info /tf_static --verbose
Type: tf2_msgs/msg/TFMessage
Publisher count: 0       ← publisher muerto
Subscription count: 17   ← datos históricos aún en DDS history (TRANSIENT_LOCAL)
```

### 2.3 `/tf_static` disponible por TRANSIENT_LOCAL

`/tf_static` usa `DurabilityPolicy.TRANSIENT_LOCAL` en FastDDS. Los mensajes publicados por RSP cuando estaba vivo persisten en el historial DDS aunque el publisher haya muerto. Cualquier suscriptor nuevo recibe esos mensajes históricos.

Esto afecta a los **fixed joints**: `world → base_link` (vía `world_tf_publisher`), `tool0 → rg2_pinch_center`, y todas las articulaciones fijas del URDF.

Los **joints revolute** (arm joints: shoulder, elbow, wrist) se publican en `/tf` con `DurabilityPolicy.VOLATILE` → no persisten → un nodo externo nuevo no puede reconstruir la pose del brazo.

### 2.4 Panel con buffer TF pre-cargado

`panel_tf.py` crea `TfHelper` con:
```python
self._buffer = Buffer()  # cache_time=10s por defecto para dinámicos
self._listener = TransformListener(self._buffer, self._node)
```

El helper se inicializó **mientras RSP estaba vivo**, recibió:
- Transforms estáticos (persistent en buffer, sin expiración)
- Transforms dinámicos (arm joints, TTL ~10s en buffer normal)

Con `use_sim_time=true` y `/clock` sin publishers (también caído), el reloj interno del nodo puede haberse congelado o avanzar erróneamente. En ese contexto, el buffer TF no expira los transforms dinámicos de la forma esperada porque el tiempo de referencia no avanza.

`lookup_transform` en `panel_tf.py` usa:
```python
return self._buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
# rclpy.time.Time() = Time(sec=0, nanosec=0) → "último disponible, sin restricción temporal"
```

**No hay ninguna comprobación de `header.stamp` ni de antigüedad del transform.**

Evidencia de que el panel reporta TF válido con RSP muerto:
```
[PANEL_TRACE] tcp_live_source=tf2:ok tcp_live_base=tcp_live: pos=(0.431,0.000,0.027)
              → reportado HORAS después de que RSP murió
```

### 2.5 Nodos externos sin TF dinámico

```bash
$ timeout 5 ros2 run tf2_ros tf2_echo world tool0
→ exit 143 (SIGTERM, timeout) — ningún dato recibido

$ timeout 5 ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
→ exit 143

$ timeout 5 ros2 run tf2_ros tf2_echo world rg2_pinch_center
→ exit 143
```

Un nodo externo nuevo suscribe `/tf` con `VOLATILE` → no hay datos porque RSP está muerto.
Puede recibir `/tf_static` histórico (TRANSIENT_LOCAL) pero no tiene suficiente para reconstruir la pose del brazo con articulaciones en la posición actual.

---

## 3. Impacto

### 3.1 TCP stale → picks con localización incorrecta

El panel valida TCP antes de ejecutar cada fase:
- `APPROACH_COARSE`: calcula distancia al objeto usando `tcp_live_base` (TF)
- `GRASP_DOWN_JOINT`: verifica convergencia usando `tcp_live` 
- `GRASP_ALIGN_IK`: usa TCP para calcular seeds IK
- `PRE_CLOSE`: comprueba posición antes de cerrar gripper

Si TF devuelve datos de cuando el brazo estaba en posición diferente, **todas estas validaciones pueden dar OK falso**. El robot podría:
- Cerrar el gripper sin estar sobre el objeto
- Reportar ATTACH exitoso con el TCP fuera de rango
- Pasar PRE_CLOSE_VOLUME con el gripper mal posicionado

### 3.2 `inspect_rg2_visual_pose` incapaz de comparar TF vs Gazebo

La métrica principal de la herramienta (`dist_pinch_to_gz_visual_mid`) requiere `pinch_q` de TF. Con RSP muerto, todos los lookups TF retornan N/A → verdict siempre UNKNOWN. La herramienta es inútil en el estado actual del stack.

### 3.3 `system_state_manager` no detecta el problema

`_tf_ok()` usa `can_transform(..., Time())` y `lookup_transform(..., Time())`:
```python
# src/ur5_tools/ur5_tools/system_state_manager.py:562
def _tf_ok(self) -> Tuple[bool, str]:
    timeout = Duration(seconds=self._tf_timeout)
    try:
        if not self._tf_buffer.can_transform(
            self._world_frame, self._base_frame, rclpy.time.Time(), timeout=timeout
        ):
            return False, "world->base"
        tf = self._tf_buffer.lookup_transform(
            self._world_frame, self._base_frame, rclpy.time.Time(), timeout=timeout
        )
        if self._is_identity(tf):
            return False, "world->base identity"
        ...
    return True, "ok"
```

`Time(0)` = "dame el último disponible". Si el buffer tiene datos estantados, `can_transform` devuelve `True` y `_tf_ok` devuelve `(True, "ok")`. **El sistema no cambia a WAITING_TF aunque RSP esté muerto.**

---

## 4. Causa raíz probable

### 4.1 RSP sin guard ni respawn

`ur5_stack.launch.py` tiene guards para: `gz_sim`, `parameter_bridge`, `gz_pose_bridge`, `system_state_manager`. **No tiene guard ni respawn para `robot_state_publisher`.**

```python
# Lo que existe para otros procesos críticos:
gz_pose_guard = RegisterEventHandler(
    OnProcessExit(
        target_action=gz_pose_bridge,
        on_exit=[EmitEvent(event=Shutdown(reason="gz_pose_bridge exited"))],
    ),
    condition=IfCondition(launch_gazebo),
)

# Lo que no existe para RSP:
# → ningún OnProcessExit, ningún respawn=True, ningún guard
```

`ur5_rsp.launch.py` tampoco tiene `respawn=True` en el Node.

### 4.2 Trigger probable: Gazebo GUI crash + cascada de relojes

El stack log registra:
```
[ERROR] [gz-4]: process has died [pid 2431827, exit code -6,
        cmd 'gz sim -g --gui-config ...']
```

La GUI de Gazebo crashed (SIGABRT, código -6). Aunque el servidor Gazebo sigue vivo, el crash puede haber perturbado el transporte GZ momentáneamente. Esto puede haber afectado al `parameter_bridge` que bridgea `/clock` desde Gazebo. Con `/clock` sin datos, RSP con `use_sim_time=true` recibe "Moved backwards in time" en bucle (confirmado en `ur5_rsp.log`) y eventualmente puede crash.

### 4.3 `tf_probe.py` existe pero no está lanzado

`ur5_tools` incluye `tf_probe.py` con lógica correcta de `max_age_sec` y detección de staleness. Está registrado en `setup.py` como entry_point. **No está incluido en `ur5_stack.launch.py` ni en ningún launch file.**

---

## 5. Archivos candidatos a revisar

| Archivo | Problema identificado | Fix propuesto |
|---|---|---|
| `ur5_bringup/launch/ur5_rsp.launch.py` | No tiene `respawn=True` | Añadir `respawn=True, respawn_delay=2.0` |
| `ur5_bringup/launch/ur5_stack.launch.py` | Sin guard para RSP; `tf_probe` no lanzado | Añadir `OnProcessExit` guard para RSP + lanzar `tf_probe` |
| `ur5_tools/ur5_tools/system_state_manager.py` | `_tf_ok()` no comprueba antigüedad del stamp | Añadir validación de `header.stamp` vs clock |
| `ur5_qt_panel/ur5_qt_panel/panel_tf.py` | `lookup_transform` sin validación de frescura | Añadir `_lookup_fresh` que valida `header.stamp` |
| `ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py` | `_pose_position` no valida frescura TF | Añadir logs `[PICK][TF_FRESHNESS]` con edad del stamp |
| `ur5_tools/ur5_tools/tf_probe.py` | Implementación correcta, no integrada | Integrar en launch + configurar pares críticos |
| `scripts/validate_startup_repro.sh` | No existe | Crear script de checklist de arranque |

---

## 6. Plan de fix: faseado y mínimo

### FASE A — Diagnosticar por qué muere RSP

**Objetivo:** obtener exit code y traceback real del próximo ciclo.

**Acciones:**
1. Añadir logging explícito de RSP a un fichero dedicado desde el launch:
   ```python
   rsp = Node(
       ...,
       output="both",       # stdout + stderr
       emulate_tty=True,
   )
   ```
2. En el próximo arranque, capturar `ur5_rsp.log` completo y buscar:
   - `Traceback` / `Error` / `Exception`
   - exit code (el guard que añadiremos en FASE B lo loguea)
3. Comprobar si `robot_description` llega correctamente al arrancar:
   ```bash
   ros2 topic echo /robot_description --once
   ```
4. Comprobar si `/clock` sigue vivo durante el ciclo de pick:
   ```bash
   ros2 topic hz /clock --window 10
   ```
5. Buscar en el stack log entradas `[robot_state_publisher-2]` con errores antes de la muerte.

**Sin cambios funcionales en esta fase.**

---

### FASE B — RSP no puede morir silenciosamente

**Objetivo:** si RSP muere, el stack lo sabe y reacciona.

**Opción B1 — respawn (preferida si RSP muere por clock transiente):**
En `ur5_rsp.launch.py`:
```python
rsp = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    respawn=True,
    respawn_delay=2.0,   # espera antes de relanzar
    parameters=[...],
)
```

**Opción B2 — guard (si queremos fail-fast):**
En `ur5_stack.launch.py`, añadir después de `rsp_launch`:
```python
rsp_guard = RegisterEventHandler(
    OnProcessExit(
        target_action=rsp_launch,  # ← o el Node si se refactoriza
        on_exit=[EmitEvent(event=Shutdown(reason="robot_state_publisher exited"))],
    ),
    condition=IfCondition(launch_rsp),
)
```

**Restricción:** elegir B1 o B2, no ambos. B1 es más robusto para crashes transientes; B2 es más agresivo (fail-fast). **Se recomienda B1 primero con guard como fallback si los respawns superan N veces.**

**Adicionalmente:** integrar `tf_probe` en el launch con pares críticos:
```python
tf_probe = Node(
    package="ur5_tools",
    executable="tf_probe",
    output="screen",
    parameters=[{
        "pairs": ["world,base_link", "base_link,tool0", "base_link,rg2_pinch_center"],
        "max_age_sec": 0.5,
        "period_sec": 1.0,
        "use_sim_time": use_sim_time,
    }],
    condition=IfCondition(launch_rsp),
)
```

---

### FASE C — Validación de frescura TF en sistema

**Objetivo:** que `system_state_manager` y el panel rechacen TF stale.

**C1 — `system_state_manager._tf_ok()`:**
Añadir comprobación de `header.stamp` tras el lookup:

```python
def _tf_ok(self) -> Tuple[bool, str]:
    timeout = Duration(seconds=self._tf_timeout)
    try:
        ...
        tf = self._tf_buffer.lookup_transform(
            self._world_frame, self._base_frame, rclpy.time.Time(), timeout=timeout
        )
        if self._is_identity(tf):
            return False, "world->base identity"
        # NUEVO: validar frescura
        stamp_ns = int(tf.header.stamp.sec) * 1_000_000_000 + int(tf.header.stamp.nanosec)
        if stamp_ns == 0:
            return False, "world->base zero_stamp"
        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - stamp_ns) * 1e-9
        if age_sec > self._tf_max_age_sec:   # nuevo param, default 0.5
            return False, f"world->base stale age={age_sec:.2f}s"
        ...
    return True, "ok"
```

**C2 — `panel_tf.py` — añadir `lookup_fresh`:**

```python
def lookup_transform_fresh(
    self, target_frame: str, source_frame: str,
    timeout_sec: float = 1.0,
    max_age_sec: float = 0.5,
) -> tuple:
    """Returns (transform, age_sec) or (None, None) if stale/unavailable."""
    tf = self.lookup_transform(target_frame, source_frame, timeout_sec)
    if tf is None:
        return None, None
    stamp_ns = int(tf.header.stamp.sec) * 1_000_000_000 + int(tf.header.stamp.nanosec)
    if stamp_ns == 0:
        return None, None  # zero stamp = probablemente stale static
    # usar wall time si no hay sim time disponible
    try:
        now_ns = self._node.get_clock().now().nanoseconds if self._node else 0
    except Exception:
        now_ns = 0
    if now_ns <= 0:
        return tf, None  # clock no disponible, no podemos validar
    age_sec = (now_ns - stamp_ns) * 1e-9
    if age_sec > max_age_sec:
        return None, age_sec  # stale
    return tf, age_sec
```

**C3 — Bloqueo de APPROACH/GRASP/PRE_CLOSE si TF stale:**

En `panel_pick_demo.py`, antes de cada fase crítica añadir:
```python
_tf_age = _check_tf_freshness("rg2_pinch_center", max_age_sec=0.5)
if _tf_age is None or _tf_age > 0.5:
    return _phase_fail("APPROACH_COARSE", reason=f"TF_STALE age={_tf_age}")
```

---

### FASE D — Logs de frescura TF en el pick

**Objetivo:** cada validación TCP emite un log `[PICK][TF_FRESHNESS]` con los datos precisos.

**Formato:**
```
[PICK][TF_FRESHNESS] frame=rg2_pinch_center stamp=1777057394.123 age_sec=0.032
                     max_age_sec=0.500 tf_source=dynamic verdict=OK
[PICK][TF_FRESHNESS] frame=rg2_pinch_center stamp=1777047760.000 age_sec=9634.2
                     max_age_sec=0.500 tf_source=unknown verdict=STALE → BLOCK
```

**Implementación:** añadir función `_emit_tf_freshness(frame, tf_stamp, age_sec, verdict)` en `panel_pick_demo.py`, llamada desde cada `_emit_rg2_visual_audit`.

---

### FASE E — Revalidación con stack limpio

**Secuencia de verificación obligatoria tras aplicar FASE B+C:**

```bash
# 1. RSP vivo
ros2 node list | grep robot_state_publisher
# Esperado: /robot_state_publisher

# 2. /tf con publishers
ros2 topic info /tf | grep "Publisher count"
# Esperado: Publisher count: 1 (o más)

# 3. TF externo funcional
timeout 5 ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
# Esperado: transformación válida con stamps recientes

# 4. Validación geométrica
ros2 run ur5_tools validate_dh_vs_tf
# Esperado: divergencia < 15mm (tolerancia validada 2026-04-23)

# 5. Inspect tool con TF fresco
GZ_PARTITION=$(cat agarre_ros2_ws/log/gz_partition.txt)
ros2 run ur5_tools gz_pose_bridge --ros-args -p world_name:=ur5_mesa_objetos &
ros2 run ur5_tools inspect_rg2_visual_pose --ros-args -p once:=true -p object_name:=pick_demo
# Esperado: pinch_tf_world≠N/A, verdict≠UNKNOWN, dist_pinch_gz_mid disponible

# 6. Pick hasta PRE_CLOSE con logs de frescura
# En el panel: ejecutar pick y capturar:
grep -E "\[PICK\]\[TF_FRESHNESS\]|\[PICK\]\[RG2_VISUAL_AUDIT\]|PRE_CLOSE" \
    agarre_ros2_ws/log/ros2_launch.log | tail -50
# Esperado: TF_FRESHNESS verdict=OK en todas las fases, dist_pinch_gz_mid < 0.05m en PRE_CLOSE
```

---

## 7. Restricciones para la implementación

- **No tocar offsets URDF/SDF/TCP**
- **No tocar GRASP_DOWN_JOINT hasta que TF fresco esté garantizado**
- **No tocar attach backend**
- **No modificar lógica de movimiento** — solo robustez de detección y logging
- **Orden estricto:** FASE A → B → C → D → E. No saltarse el diagnóstico.

---

## 8. Resumen de brechas identificadas

| Componente | Brecha | Severidad |
|---|---|---|
| `ur5_rsp.launch.py` | Sin `respawn=True` | CRÍTICA |
| `ur5_stack.launch.py` | Sin guard para RSP | CRÍTICA |
| `system_state_manager._tf_ok()` | No valida `header.stamp` → stale pasa como OK | CRÍTICA |
| `panel_tf.py lookup_transform` | `Time(0)` sin validación de antigüedad | ALTA |
| `panel_pick_demo.py` fases críticas | No bloquea si TF stale | ALTA |
| `tf_probe.py` | Implementado correctamente, no lanzado | MEDIA |
| `ur5_rsp.log` | No captura exit code del proceso RSP | BAJA |

---

*Generado: 2026-04-24 | Rama: audit/tcp-geometry-fix*  
*Informe relacionado: `2026-04-24_rg2_visual_pose_runtime_audit.md`*
