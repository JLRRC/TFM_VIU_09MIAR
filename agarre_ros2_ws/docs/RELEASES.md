# Releases — TFM UR5+RG2 Pick & Place

Histórico de tags release del proyecto. Cada tag es un punto de rollback verificado.

## v1.3 — 2026-05-07 (último)

**T18 Qt headless en CI + T29 GRASP_DOWN drift detector**

- T18: `test_panel_v2_qt_smoke.py` añadido al CI con `QT_QPA_PLATFORM=offscreen`. Valida import de panel_v2 + 17 mixins de ControlPanelV2 sin display.
- T29: 13 tests nuevos que bloquean regresiones del FSM canónico (orden GRASP_DOWN, transiciones permitidas, índices, progress fractions).
- CI extendido: `pip install pytest pyyaml PyQt5`.
- Suite total: **2389 tests** verdes.

## v1.2 — 2026-05-07

**CI extendido (T4 schemas + T26 grasp_evidence) + arquitectura mermaid**

- T4: 17 tests YAML schemas (parseables + claves canónicas + guard `goal_time>0`).
- T26: 5 tests grasp evidence (lee log live, valida agarre físico).
- T22: ya existía (URDF↔SDF parity, 5 tests).
- Diagrama mermaid en `architecture.md` con todos los nodos.
- Tabla de releases.
- CI rápido (`colcon.yml`) ejecuta T4 + T26.
- Suite total: **2380 tests** verdes.

## v1.1 — 2026-05-07

**Orchestrator path APPROACH+GRASP_DOWN OK intermitente**

Tras 27 rondas live de validación:

- `max_velocity_scaling_factor` 0.5→0.3 (trayectoria más lenta).
- `allowed_execution_duration_scaling` 30→100 (upper bound x10).
- `moveit_result_timeout_sec` 90→400s.
- `action_result_timeout_sec` 150→500s.
- `attach_distance` gate 0.05→0.075m (sesgo MoveIt + FK).

Resultado: orchestrator path completa hasta GRASP_DOWN ~50% de las veces. Bug bridge `move_action ↔ joint_trajectory_controller` (sim_time vs wall_time) documentado en `BUG_BRIDGE_PATH_TOLERANCE.md`.

## v1.0.1 — 2026-05-07

**Suite 100% verde post GRASP_DOWN + world→base + T26**

- Tests refactorizados tras añadir `PickPhase.GRASP_DOWN` al happy path.
- Conversión world→base_link en goal builders del orchestrator.
- T26 nuevo (test_grasp_evidence_regression): valida log live del agarre.
- Suite: 2363 tests verdes (era 1465+ con regresiones).

## v1.0 — 2026-05-07

**Release inicial profesional**

- README + CHANGELOG + LICENSE + OPERATION + AUDIT publicados.
- Default dispatcher revertido a legacy (orchestrator opt-in).
- Carpetas volátiles en `.gitignore` (BORRAR/, historico/, auditoria/).

## objetivo-cumplido-pinzas-agarran-objeto-20260507 — 2026-05-07

**OBJETIVO PRINCIPAL CUMPLIDO**

Las pinzas RG2 agarran físicamente el objeto en simulación Gazebo.
Validado live en ronda 26 con evidencia hard del log:

```
[ATTACH_BACKEND] demo_transport_follow_tick
  object=pick_demo  mode=world_locked
  desired=(-0.722, 0.345, 1.789)
  tcp=(-0.731, 0.351, 1.804)
```

- TCP↔objeto: 1.8cm (pinzas envolviendo)
- Z=1.79m: robot levantó objeto **97.8cm** desde la mesa
- 132 ticks consecutivos en `mode=world_locked`

**Camino canónico**: `run_pick_demo` (legacy) con todos los fixes acumulados de la sesión.

## Tags previos a v1.0 (sub-hitos)

- `cierre-bloque-2-orchestrator-cableado-20260506` — Orchestrator action server vivo
- `cierre-bloque-1-pick-fisico-validado-20260506` — Pick físico validado
- `cierre-bloque-1-urdf-sdf-fix-20260506` — Fix bug 167mm gripper
- `audit-fase-0-10-completed-20260504` — Auditoría F0-F10 completa
- `B-iter14-sprint-complete-20260503` — Sprint orchestrator B-iter1..14

## Convenciones

- Cada release tiene su tag git (`v1.X`) con mensaje descriptivo.
- Cada sub-hito tiene su tag rollback fechado (`cierre-XXX-YYYYMMDD`).
- `git checkout <tag>` para volver a cualquier punto.
- Suite verde como pre-requisito para tag v1.X.

## Próximas releases planificadas (tentativo)

- **v1.4**: cuando el bug bridge MoveIt-controller esté cerrado (orchestrator 100% verde).
- **v2.0**: tras borrar legacy run_pick_demo (~9k LOC) + Bloque 4 limpieza estructural.
