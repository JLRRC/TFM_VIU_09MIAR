# F5 — Estado de cierre (delta 2026-05-08)

**Status**: `IMPLEMENTADO + COBERTURA TEST` ✅ · `ACTIVACIÓN POR DEFAULT` 🔴 (bloqueado por bug bridge)
**Doc histórico**: [F5_CLOSURE_PLAN.md](./F5_CLOSURE_PLAN.md) (2026-05-03)
**Bug bloqueante**: [BUG_CONTROLLER_FEEDBACK_HANG.md](./BUG_CONTROLLER_FEEDBACK_HANG.md) + [BUG_BRIDGE_PATH_TOLERANCE.md](./BUG_BRIDGE_PATH_TOLERANCE.md)

## TL;DR

El cableado panel → action client `PickPlace` está **completo** y testeado:

- `pick_demo_dispatcher.dispatch_pick_demo` ya elige entre legacy y orchestrator.
- 51 tests offline (`19 dispatcher + 32 client_logic`) cubren las 4 rutas.
- Default actual = **legacy** (revertido 2026-05-07 por bug bridge).
- Activación = `PANEL_PICK_DEMO_USE_ORCHESTRATOR=1` (opt-in).

El cierre F5 completo (default = orchestrator + borrado del legacy) requiere
que el bug `BUG_CONTROLLER_FEEDBACK_HANG` esté cerrado (cycles 2/3 limpios
en T35 live).

## Estado fáctico HOY (HEAD post-T35-smoke)

### Implementado ✅

| Pieza | Archivo | LOC | Tests |
|-------|---------|-----|-------|
| Action server `PickPlace` | tfm_orchestrator/`pick_orchestrator_lifecycle_node.py` | — | suite tfm_orchestrator (313+) |
| Cliente Qt thin | ur5_qt_panel/`pick_place_client.py` | 200+ | parte de los 32 |
| Lógica pura (build_goal, parse_feedback) | ur5_qt_panel/`pick_place_client_logic.py` | 250+ | 32 |
| Dispatcher legacy/orch | ur5_qt_panel/`pick_demo_dispatcher.py` | 230 | 19 |
| Botón "Agarre Objeto (Directo)" | panel_calib_actions:268 | — | (testado vía dispatcher) |
| Service `/panel/pick_demo` | panel_remote_callbacks:286,317 | — | (testado vía dispatcher) |

### Default revertido a legacy (2026-05-07)

```python
# pick_place_client_logic.should_use_orchestrator
# 2026-05-07 (sesión 25 rondas live): default revertido a **legacy**
# porque el orchestrator está bloqueado por un bug arquitectónico de
# integración MoveIt ↔ joint_trajectory_controller en Gazebo Sim
# (timing sim_time vs wall_time, BUG_BRIDGE_PATH_TOLERANCE.md).
```

Reglas activas:
- `USE_LEGACY_PICK_DEMO=1` → legacy (rollback rápido).
- `PANEL_PICK_DEMO_USE_ORCHESTRATOR=1` → orchestrator (opt-in).
- Cualquier otro caso → legacy.

## Criterio de cierre F5 completo

El cierre F5 al 100% requiere:

1. ✅ Tests dispatcher / client_logic (51 verdes).
2. ✅ Botón panel ya no llama directamente `run_pick_demo` (pasa por dispatcher).
3. ✅ Service `/panel/pick_demo` ya pasa por dispatcher.
4. 🔴 Bug `BUG_CONTROLLER_FEEDBACK_HANG` cerrado (live).
5. 🔴 T35 verde (3 cycles consecutivos con `PANEL_PICK_DEMO_USE_ORCHESTRATOR=1`).
6. 🔴 Default invertido en `should_use_orchestrator` (con migration test).
7. 🔴 Borrado físico de `run_pick_demo` (~8.000 LOC) tras 1 sesión sin
   regresiones.

Cuando 4-7 estén verdes, el panel será **action client puro** y el legacy
puede borrarse — el indicador único del audit ("panel_pick_demo ≤ 1.500 LOC")
se cumple instantáneamente.

## Tag de hito

Cuando T35 verde × 3 con `PANEL_PICK_DEMO_USE_ORCHESTRATOR=1`:

```bash
git tag F5-orchestrator-default-active-<fecha>
```

Y a continuación:

```bash
# Invertir default en should_use_orchestrator (1 línea)
# Ejecutar T35 × 5 (stress)
# Si verde × 5: borrar run_pick_demo
git rm <secciones run_pick_demo legacy>
git tag F5-legacy-removed-<fecha>
```

## Referencias

- Tests: `src/ur5_qt_panel/test/test_pick_demo_dispatcher.py` (19),
  `test_pick_place_client_logic.py` (32).
- Memoria sesión 2026-05-08: `project_session_close_20260508.md`.
- Status v5 audit: `auditoria/audit_profesional_20260508_v5.md` (sección 7
  FASE 5 cierre).
