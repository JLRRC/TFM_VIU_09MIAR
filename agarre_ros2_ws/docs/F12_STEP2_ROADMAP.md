# F12-step2 — Drenaje real de `panel_pick_demo.py` (roadmap)

> Estado: **F12-step2c.1 cerrado offline 2026-05-02**. Resto sigue pendiente
> de ROS vivo + Gazebo para validación E2E.
> Última actualización: 2026-05-02.

## F12-step2c.1 — INITIAL_SNAPSHOT + HOME_INITIAL (cerrado offline)

Primer corte real al god-file. Trabajo offline preparatorio:

* **FSM granulado**: `PickPhase` añade `INITIAL_SNAPSHOT` y `HOME_INITIAL`
  entre `IDLE` y `SELECT_OBJECT` (`pick_fsm.py`). Happy path ahora 10
  fases. Cero consumidores externos dependen de un `phase_index` literal.
* **Módulo puro `tfm_orchestrator/preflight.py`**:
    - `compose_initial_snapshot(...)` — construye `InitialSnapshot`
      frozen con TCP/objeto/joints, calcula deltas y `dist3d`.
    - `validate_home_target(current, target=UR5_HOME_JOINTS_RAD, tol_rad)`
      — comparación joint a joint con wrap angular [-π, π].
    - 100% testeable sin ROS.
* **Cableado en orchestrator**: `pick_orchestrator_lifecycle_node.py`
  + `pick_orchestrator_node.py` (legacy F5/F6) emiten las dos nuevas
  fases en `next_phases`. En modo `use_stubs=True` (default) los stubs
  de delay 0.2s mantienen paridad. En modo `use_stubs=False` el
  fallback `no_op` deja la fase pasar sin ejecutar service calls.
* **Tests offline**: `test_preflight.py` (20 tests) +
  `test_pick_fsm.py` ampliado (3 tests nuevos) → 119 tests verdes en
  `tfm_orchestrator/`, 172 en orchestrator+bringup.

**No tocado** (sigue requiriendo ROS vivo):
* `panel_pick_demo.py` permanece intacto. El movimiento físico HOME y
  los traces `[PICK][DIRECT]` siguen viviendo en el legacy.
* Wiring real `_dispatch_phase_service` para INITIAL_SNAPSHOT y
  HOME_INITIAL — pendiente de F12-step2c.2 cuando haya stack vivo.

**Tag de rollback**: `audit-pre-f12-step2c-INITIAL_SNAPSHOT-HOME_INITIAL-20260502`.

## F12-step2c.2 — Wiring real con stack vivo (PENDIENTE)

Cuando haya stack vivo:

1. Decidir si las dos fases se sirven por:
    * Service nuevo `/orchestrator/initial_snapshot` (request → snapshot
      composer en panel + return), o
    * Action `PreflightAction` que aglutine ambas, o
    * Lógica interna del orchestrator que llame a `tf_geometry_service`
      + service de comando joint trajectory para HOME.
2. Implementar y migrar el bloque equivalente en `panel_pick_demo.py`
   (líneas 802-815 + 1589 + 6596) al orchestrator.
3. Validar `validate_pick_3_cycles.sh 3` antes y después.
4. Eliminar `_emit_initial_snapshot_trace` y `_emit_home_initial_trace`
   del legacy.

## 1. Contexto

`panel_pick_demo.py` mantiene 10 397 LOC tras F0–F11. Es el
**último god-file crítico** del proyecto. Su componente central
es la closure gigante ``run_pick_demo(panel)`` que aloja una
copia legacy del FSM pick & place.

Desde **F12-step1** (commit `b309b3e`):
- El path canónico es `tfm_orchestrator` vía la action
  `/pick_place` (LifecycleNode F9).
- `dispatch_pick_demo` (default desde F12-step1) es el wrapper
  que decide entre orchestrator y `run_pick_demo` legacy.
- `USE_LEGACY_PICK_DEMO=1` fuerza el legacy como rollback.
- `run_pick_demo` emite `[PICK_DEMO][DEPRECATED]` cuando se
  invoca.

F12-step2 es el drenaje real: migrar las fases que aún viven en
`run_pick_demo` al orchestrator y eliminar el legacy.

## 2. Por qué requiere ROS vivo

El refactor mecánico (extraer helpers, partir en mixins) no es
suficiente: hay que **migrar comportamiento operativo**, lo cual
exige reproducir cada fase con la simulación corriendo y validar
que el ciclo completo `MESA → CESTA` sigue funcionando.

El smoke E2E del proyecto es:

```bash
bash agarre_ros2_ws/scripts/validate_pick_3_cycles.sh 3
```

Esto requiere:
- Gazebo Sim moderno arrancado.
- MoveIt 2 cargado.
- ros2_control con joint_trajectory_controller activo.
- Panel Qt + bridges + cámaras.
- ~10 minutos por iteración.

Sin esa red de seguridad, cualquier extracción de fases puede
romper el ciclo silenciosamente.

## 3. Estrategia de drenaje (multi-commit, 8-15 h)

### F12-step2a — Identificar fases drenables

```python
# Estructura actual de run_pick_demo:
INITIAL_SNAPSHOT → HOME_INITIAL → MESA → APPROACH_COARSE
  → GRASP_DOWN → GRASP_ALIGN → PRE_CLOSE → CLOSE
  → ATTACH (validation gate)
  → LIFT → CARRY → BASKET_TRANSPORT → BASKET_DROP
  → RELEASE → HOME_FINAL
```

Cada fase es ~200-700 LOC dentro de la closure. La FSM pura del
orchestrator (`tfm_orchestrator/pick_fsm.py`) ya define las
transiciones; falta cablear los **service calls** + **action calls**
de cada fase.

### F12-step2b — Cablear service calls del orchestrator

Las srv ya están en `ur5_panel_interfaces`:
- `SelectObject` — usado en MESA / APPROACH_COARSE
- `ComputeApproachPose` (ahora servido por `tf_geometry_service` F16)
- `Open` / `Close` / `SetWidth` — comandos gripper
- `Attach` / `Detach` — attach lógico
- `WorldToBase` — conversión TF (también en F16)

`tfm_orchestrator/service_clients.py` (F6) ya las invoca.

### F12-step2c — Migrar fase a fase

Por cada fase:

1. Confirmar que la lógica equivalente está en
   `tfm_orchestrator/pick_orchestrator_lifecycle_node.py` (revisar
   commit `5e55f5c` F9).
2. Si falta: añadirla al orchestrator + tests offline del FSM puro.
3. Eliminar la fase correspondiente de `run_pick_demo`.
4. **Validar `validate_pick_3_cycles.sh`** tras cada fase migrada.
5. Commit con mensaje `F12-step2c.X: drain phase <PHASE> from legacy`.

### F12-step2d — Eliminar el legacy

Tras drenar todas las fases:

1. `run_pick_demo` queda como stub que solo logea
   "F12-step2d: legacy fully drained, use orchestrator".
2. Eliminar `dispatch_pick_demo` (ya no hay decisión).
3. Llamadas directas al orchestrator desde el panel.
4. `panel_pick_demo.py` baja de ~10 000 a ~500-1 000 LOC
   (solo helpers UI residuales).
5. Eliminar la env var `USE_LEGACY_PICK_DEMO`.

### Validación del drenaje completo

- `validate_pick_3_cycles.sh 5` con éxito en 5 ciclos consecutivos.
- `tests/test_pick_fsm.py` cubre todas las transiciones (ya hace
  algo).
- `evidence_logger` mostrando `grasp_success_rate >= 0.8` en
  `metrics.json` para una sesión de 10 picks.

## 4. Riesgos y mitigaciones

| Riesgo | Mitigación |
|---|---|
| Regresión en pick (objeto no agarra) | Tag `audit-pre-f12-step2-<fase>-YYYYMMDD` antes de cada fase migrada |
| Drift entre legacy y orchestrator | Tests `test_pick_fsm.py` ya validan FSM puro; ampliar con cada migración |
| Tiempos largos en validación E2E | Limitar a 1-2 fases por sesión presencial |
| Pérdida de instrumentación específica del legacy | Migrar primero los `[PICK][DIRECT][TRACE]` y `[PICK][MOVEIT][...]` al orchestrator |

## 5. Estimación final

| Sub-fase | Esfuerzo | Riesgo |
|---|---|---|
| F12-step2a (mapeo) | 1 h | Bajo |
| F12-step2b (cablear srv) | 2-3 h | Bajo (offline) |
| F12-step2c (migrar 13 fases) | 6-10 h | Medio-alto (E2E required) |
| F12-step2d (eliminar legacy) | 1-2 h | Bajo (cuando todo lo demás funciona) |
| **TOTAL** | **10-16 h** | — |

## 6. Cuándo abordarlo

**Recomendable**: en una sesión presencial dedicada con el stack
completo arrancado (Gazebo + MoveIt + panel) y una hora de buffer
por si hay regresión que rebobinar al tag previo.

**No recomendable**: como parte del trabajo offline del audit.

## 7. Referencia cruzada

- `tfm_orchestrator/pick_fsm.py` — FSM canónico
- `tfm_orchestrator/pick_orchestrator_lifecycle_node.py` — F9
  LifecycleNode
- `tfm_orchestrator/service_clients.py` — F6.x
- `panel_pick_demo.run_pick_demo` — código legacy a drenar
- `pick_demo_dispatcher.dispatch_pick_demo` — eliminar tras F12-step2d
- `pick_place_client.PickPlaceClient` — cliente real ya operativo
- `validate_pick_3_cycles.sh` — smoke E2E
- `audit-pre-f12-20260501` — tag de rollback al inicio de F12
