# v1.0 → v1.1 deferred items

**Fecha**: 2026-05-08
**Contexto**: cierre del audit-v4. Items que fueron ejecutados parcialmente
o se difieren explícitamente a v1.1 con plan documentado.

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
