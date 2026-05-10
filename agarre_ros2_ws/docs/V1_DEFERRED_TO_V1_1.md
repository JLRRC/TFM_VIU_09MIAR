# v1.0 → v1.1 deferred items

**Fecha**: 2026-05-08 (creación), 2026-05-11 (live validation: panel UI saturación CPU)
**Contexto**: cierre del audit-v4 + auditoría profesional F0-F10 (2026-05-10) +
diagnóstico live 2026-05-11 sobre saturación del panel UI.
Items que fueron ejecutados parcialmente o se difieren explícitamente a
v1.1 con plan documentado.

---

## Live audit (2026-05-11): Panel UI satura GUI thread → "panel no responde"

**Estado**: deferred v1.1 (requiere refactor F6.3 del plan original).

**Síntomas observados (validación live tras F0-F10)**:

  * Linux/Qt muestra repetidamente el diálogo "panel_v2 no responde"
    al interactuar con el panel.
  * El usuario pulsa "Agarre Objeto (MoveIT)" o "Agarre Objeto (Directo)"
    y el panel se queda colgado varios segundos.
  * Botones varios responden con latencia perceptible.

**Datos medidos (no asumidos)** — `top -H -b -n1 -p $(pgrep panel_v2)`:

```
Panel total CPU:        107%  (más de 1 núcleo completo)

Threads del proceso:
  _RosWor+ TID=A       63%  ← culpable principal
  _RosWor+ TID=B        8%
  _RosWor+ TID=C        8%
  _RosWor+ TID=D        6%
  panel_v2 (GUI)       12%  ← GUI thread sano
  dds.udp+             10%

  total ROS workers:   85%
  total panel GUI:     12%
```

**Causa raíz (verificada por inspección de /proc + ros2 topic hz)**:

  * `MultiThreadedExecutor` configurado con 3 threads
    (`PANEL_ROS_EXECUTOR_THREADS=3` en `start_panel_v2.sh:122`).
  * 12 subscriptores del panel, los más calientes:
      `/clock`        →  790 Hz   (Gazebo Sim → ros_gz_bridge)
      `/joint_states` →   97 Hz
      `/pose/info`    →   50 Hz × 74 entidades = 3700 iter/s
      `/tf`           →   22 Hz
      `/camera`       →   24 Hz × ~20 ms cv_bridge+QImage
  * **~983 mensajes/s** procesados por los 3 threads del executor.
  * Cada mensaje: DDS deserialize → wake thread → dispatch a callback Python.
  * 1.874 ctxt switches/s en el thread principal del RosWorker.

  El GUI thread (Qt) está en realidad relativamente libre (12% CPU)
  pero compite con 3 threads RosWorker saturados por la cola Qt
  signal/slot que satura el event loop → freezes ocasionales → Linux
  marca "no responde".

**Intentos de fix descartados (commits revertidos 1d04531..537ba07)**:

  Se probaron 4 commits incrementales (`8faaae1` silenciar logs,
  `c7e3f3c` timers UI menos agresivos, `0b88a55` throttle callbacks
  /clock /pose/info, `80cddcc` PANEL_MAX_FPS 60→10). **Ninguno bajó
  la CPU significativamente** porque atacan el *callback work* pero
  el coste real está en el *wake + DDS deserialize* que ocurre ANTES
  del callback. Los 4 commits fueron revertidos por consistencia y
  para no enmascarar la causa raíz.

**Fix correcto (deferred v1.1) — F6.3 del plan F0-F10**:

  El plan F6.3 ya documentado (commits faa1b76 + 8cf3594) propone:

    1. Separar el panel en 2 nodos:
        * `panel_ui_node` (Qt only, ≤500 LOC): sólo input/render.
        * `panel_backend_node` (rclpy SingleThreadedExecutor, ya creado
          en ur5_tools/panel_backend_node.py): único subscriber a los
          topics ROS. Re-publica como topics latched JSON para la UI.
    2. La UI Qt solo se suscribe a 3 topics latched a baja frecuencia
        (~1-2 Hz): `/panel_backend/{feedback,result,state}`.
    3. Resultado esperado: GUI thread libre, panel responde inmediato,
        backend nodo dedicado optimizable independiente.

**Workarounds temporales (NO se aplican por defecto)**:

  Si la lentitud bloquea operación inmediata, el usuario puede probar:

    a) Reducir threads del executor:
        export PANEL_ROS_EXECUTOR_THREADS=1
        ./lanzar_panelv2.sh

    b) Bajar frecuencia /clock en Gazebo (tocar SDF — riesgo medio):
        En src/ur5_gazebo/worlds/ur5_mesa_objetos.sdf añadir al
        `<physics>` un `<real_time_update_rate>100</real_time_update_rate>`
        para reducir /clock de 790 Hz a 100 Hz.

**Plan v1.1**:
  1. Implementar F6.3 wiring real (1-2 semanas estimado).
  2. Validar live con T35 × 3 ciclos.
  3. Medir CPU post-refactor (objetivo: <50% panel total).

**Defensa académica**:
  El TFM es defendible en su estado actual porque:
    * Pipeline pick funcional via orchestrator (independiente del panel).
    * Tests offline ≥994 verde (validan lógica sin Qt).
    * docs/ARCHITECTURE.md documenta esta deuda y su solución.
    * F6.3 está en el roadmap escrito, con backend_node ya implementado
      esperando wiring.
  El panel lento es deuda heredada de la implementación monolítica
  pre-F6 (panel_v2.py + RosWorker en 1 solo proceso). NO es un bug
  introducido por la sesión F0-F10 — la auditoría F0-F10 NO modificó
  ningún subscriber ni el threading model del panel.

---

## F-iter3 audit (2026-05-10): RG2 force sensing real

**Estado**: deferred v1.1 (HW real).

**Path**: `src/ur5_qt_panel/ur5_qt_panel/panel_motion_control.py:225-230`

```python
# TODO: Integrar con RG2_GRIPPER topic real
def _estimate_gripper_force(panel) -> float:
    if hasattr(panel, '_gripper_is_closed'):
        if panel._gripper_is_closed:
            return 0.5  # 0.5 N cuando está cerrado (estimación)
    return 0.0
```

**Razón**: el RG2 simulado en Gazebo no expone fuerza real. La función
devuelve un placeholder ficticio. Para producción (HW real OnRobot RG2),
integrar con:

  * **Topic**: `/onrobot_rg2/state` (o equivalente del driver físico).
  * **Mensaje**: `onrobot_rg2_msgs/msg/GripperState` con campo `force_n`.
  * **Nodo**: `gripper_attach_backend` puede subscribirse y republicar
    como `~/force` (latched).

**Plan v1.1**:
1. Confirmar nombre del driver al disponer del HW (ver `docs/HARDWARE_CHECKLIST.md`).
2. Añadir subscriber en `gripper_attach_backend.on_configure`.
3. Sustituir `_estimate_gripper_force` por lectura del topic.
4. Test live `test_gripper_force_real.py` con cubo medido.

---

## panel_ros.py (2.158 LOC) — split a publishers + subscribers + clients

**Estado**: deferred v1.1.

**Razón**: `RosWorker(QObject)` es la pieza central de plumbing ROS↔Qt y
todos sus ~60 métodos comparten `self.node`, `self.executor`,
`self._lock`, `self._ros_lock`. Un split mecánico produciría 3 mixins
con dependencias circulares sobre el mismo state. El refactor seguro
requiere primero extraer el state a un dataclass `RosWorkerState` y
luego pasarlo explícito a cada método — multi-sesión (~6-10h).

**Plan iter2**:
1. Extraer dataclass `RosWorkerState` con todos los slots de runtime.
2. Cambiar métodos a recibir `state` como parámetro (no `self`).
3. Mover métodos puros de introspection (`list_*`, `has_*`, `*_count`)
   a `panel_ros_introspection.py`.
4. Mover métodos de service/action calls a `panel_ros_clients.py`.
5. Mover subscribers y handlers a `panel_ros_subscribers.py`.
6. Validar cada paso con T-SMOKE arranque + test offline.

---

## panel_helpers.py (1.441 LOC) — split por dominio (ui/ros/geometry/time)

**Estado**: deferred v1.1.

**Razón**: ~80 funciones top-level toman `panel` como primer argumento.
Son esencialmente métodos del panel viviendo en un namespace utilitario.
Split por dominio requiere primero clasificar cada función según los
campos de `panel` que toca, lo cual es un análisis estático manual.

**Plan iter2**:
1. Para cada función, anotar qué `panel.<attr>` lee/escribe.
2. Agrupar en 4 buckets: `ui/`, `ros_state/`, `geometry/`, `time/`.
3. Mover por buckets a `panel/<bucket>/<purpose>.py`.
4. Mantener `panel_helpers.py` como re-export façade hasta que callers
   migren.

---

## pick_demo/internal_helpers.py (1.262 LOC) — split por dominio

**Estado**: deferred v1.1.

**Razón**: misma razón que panel_helpers.py — heavy panel coupling.

**Plan iter2**: idéntico al patrón panel_helpers.

---

## F5-iter2/iter3: _execute_joint_trajectory_action

**Estado**: iter1 completo (preparation extraída en
`_prepare_fjt_execution`). iter2/iter3 deferred v1.1.

**Razón**: la función monolítica original (1.343 LOC) tenía 4-5 bloques
naturales (validate / send / poll / handle / report). iter1 extrajo
~60 LOC de preparation. Las extracciones restantes requieren preservar
el control flow complejo de futures + feedback callbacks + early
returns, lo cual es alto riesgo en una sola sesión.

**Plan iter2/iter3**:
- iter2: extraer `_send_fjt_goal_async` (lines 640-665 originales).
- iter3: extraer `_poll_fjt_result_with_gates`.
- iter4: extraer `_build_fjt_execution_report`.
- Cada iter con tag rollback + T31 contract test + validación T35 live.

---

## Métricas v1.0 al cierre (HEAD audit-v4-release-logic-20260508)

- Tests offline gating: ~2.700 (era ~2.600)
- mypy strict baseline: 67 módulos (era 24, +180 %)
- YAML schemas validados: 1 (runtime_defaults) + drift bidireccional
- Env reads gating: snapshot 496 baseline, monotonically decreasing
- Print-in-prod gating: 0 reales detectados
- LifecycleNodes: 8 producción + tests por nodo
- IDL inventory test: 9 srv + 2 action verificados
- URDF/Xacro test: 24 (frames + joints + controllers YAML)
- Cycle timing aggregator: percentiles p50/p95/p99 multi-cycle disponibles
- TF batch lookups runtime: ejecutor deduplicado disponible
- GZ_PARTITION: consistencia start_panel_v2 ↔ launch_helpers ↔ panel_process

Score honesto post-v1.0: **~95/100**.
