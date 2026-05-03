# F5 — Plan de cierre (panel → cliente PickPlace puro)

**Fecha**: 2026-05-03
**HEAD de referencia**: `0b9566e`
**Estado actual**: infraestructura completa; pendiente: borrado legacy + validación live.

---

## Resumen ejecutivo

F5 está **arquitectónicamente cerrada**. Los 8 sub-pasos (F5-step1..8)
han dejado el sistema en un estado donde:

1. **El orchestrator es el path canónico** del botón "Agarre Objeto (Directo)".
2. El panel YA llama por defecto a [`pick_demo_dispatcher.dispatch_pick_demo`](../src/ur5_qt_panel/ur5_qt_panel/pick_demo_dispatcher.py),
   que envía goal a `/pick_place` (action server del orchestrator).
3. El legacy `run_pick_demo` sólo se ejecuta si `USE_LEGACY_PICK_DEMO=1`
   o `PANEL_PICK_DEMO_USE_ORCHESTRATOR=0` (rollback rápido).
4. `pick_orchestrator_lifecycle_node` **auto-launched** en el stack
   (commit `0ec810c`, F5-step8).
5. `plan_to_pose_server` cableado con `use_real_bridge=true` (commit
   `c63d6b2`, F5-step6d).
6. 4 services reales en `gripper_attach_backend` (commit `c477cab`,
   F5-step6b): `Open` / `Close` / `Attach` / `Detach`.
7. `phase_dispatch.py` puro con paridad LifecycleNode ↔ Node legacy
   (commit `9dc9c90`, F5-step1).
8. `ResolveObjectPoseWorld.srv` + `object_pose_resolver_service`
   LifecycleNode operativo (commits `4c76795`+`615513c`+`048c323`,
   F5-step3..5).

## Lo que sigue pendiente (TRABAJO INVASIVO)

El cierre estricto requiere:

### Paso A — Validación live de paridad (BLOQUEANTE; requiere ROS vivo)

| # | Acción | Comando | Criterio éxito |
|---|--------|---------|----------------|
| A1 | Lanzar stack completo | `./lanzar_panelv2.sh` | Stack live, panel arranca, orchestrator `active` |
| A2 | Ejecutar 3 ciclos pick por orchestrator (default) | botón "Agarre Objeto (Directo)" sin envs | success_rate=1.0 |
| A3 | Ejecutar 3 ciclos pick por legacy | `USE_LEGACY_PICK_DEMO=1 ./lanzar_panelv2.sh` | success_rate=1.0 |
| A4 | Comparar JSON de evidence_logger entre A2 y A3 | `diff <(jq -S 'del(.timing,.duration_*)' ev_orchestrator.json) <(jq -S 'del(.timing,.duration_*)' ev_legacy.json)` | sin diff funcional |
| A5 | Validar que los 9 phases del orchestrator se ejecutan | `ros2 topic echo /pick_phase_event` durante A2 | 9 fases publicadas |

**Sin Paso A no se puede borrar el legacy.** El audit del 2026-05-03
confirma que use_stubs=False ya está desbloqueado para gripper_attach
y plan_to_pose, pero los 9 phases live no se han comparado E2E.

### Paso B — Borrado del legacy (IMPRESCINDIBLE)

Sólo después de A4/A5 verde:

```bash
git tag audit-pre-f5-closure-20260503

# 1. Borrar el cuerpo legacy de run_pick_demo (8.410 LOC en función única)
#    y todos los closures internos (worker 8.137 LOC).
#    panel_pick_demo.py: 8.985 → ~575 LOC (sólo el log de deprecation
#    y el dispatch al orchestrator).

# 2. Borrar los helpers exclusivos del legacy:
#    - _move_tcp_direct (1.008 LOC)
#    - _execute_tfm_canonical_pick_object_route
#    - todo lo que vive en pick_demo/internal_helpers.py NO usado por
#      el orchestrator path

# 3. Borrar el flag USE_LEGACY_PICK_DEMO de:
#    - pick_demo_dispatcher.py (legacy_dispatch + should_use_orchestrator)
#    - panel_pick_demo_params.py
#    - test_pick_demo_dispatcher.py (el subset que probaba el legacy)
#    - LEGACY_UNDOC_ENV_VARS de test_quality_metrics.py

# 4. Reducir LEGACY_OVERSIZE_FILES_LOC para panel_pick_demo:
#    8985 → ~600 (entry points + log de deprecation, no más).

# 5. Actualizar audit profesional con el nuevo SCORE (estimado 88/28 → 23/28
#    permanece, pero panel_pick_demo cae al T14 cumplido global).
```

### Paso C — Re-validación post-borrado (IMPRESCINDIBLE)

```bash
# Tras Paso B:
colcon build --packages-select ur5_qt_panel
colcon test --packages-select ur5_qt_panel  # sin regresiones
PICK_E2E_LIVE=1 pytest src/ur5_bringup/test/test_e2e_pick_cycles.py  # 3 ciclos verde
```

### Paso D — Cleanup de tests obsoletos

| Test | Decisión |
|------|----------|
| `test_pick_demo_pure_helpers.py` | mantener (helpers reusados por orchestrator) |
| `test_pick_demo_dispatcher.py` | reducir a "test_dispatcher_routes_to_orchestrator" |
| `test_panel_pick_demo_basket_confirmation.py` | mantener si el orchestrator delega esa lógica; borrar si no |
| `test_panel_pick_demo_direct_grasp_z.py` | candidato a borrado tras Paso B |
| `test_panel_pick_demo_live_pose.py` | mantener |
| `test_panel_pick_demo_transport_follow.py` | revisar — posiblemente legacy |

---

## Riesgos

| Riesgo | Probabilidad | Impacto | Mitigación |
|--------|--------------|---------|------------|
| Orchestrator path produce timings distintos al legacy | Media | Medio | Excluir `timing` del diff del Paso A4 |
| Una fase no migrada al orchestrator (e.g. `move_tcp_direct`) | Media | Alto | Paso A5 valida 9 fases; si falla, F6 (extraer fase faltante) |
| Tests offline rompen tras borrado por imports residuales | Alta | Bajo | Paso C los detecta; ruff F401 + ajustes triviales |
| Panel arranca pero "Agarre Objeto" no funciona | Baja | Crítico | Tag `audit-pre-f5-closure-20260503` + git revert |
| El borrado deja panel_pick_demo.py con < 600 LOC pero | | | |
| panel_pick_object.py (3.823) sigue oversize | Cierta | Esperado | F3 continúa con panel_pick_object como nuevo top |

---

## Indicadores objetivos

Tras F5 cerrada al 100 %:

- [ ] `panel_pick_demo.py` ≤ 600 LOC (hoy 8.985)
- [ ] `run_pick_demo` función sólo emite log + dispatch al orchestrator (≤ 30 LOC)
- [ ] `USE_LEGACY_PICK_DEMO` no aparece en código activo
- [ ] T14 cumplido sin baseline para panel_pick_demo.py
- [ ] T15 cumplido sin baseline para run_pick_demo
- [ ] Score profesional 25/28 ≈ 89 % (subida desde 23/28 = 82 %)
- [ ] Demo `./lanzar_panelv2.sh` + botón pick → verde sin envs
- [ ] CI completo verde con todas las suites

## Estimación de esfuerzo

| Paso | Tiempo | Bloqueante |
|------|--------|------------|
| A — Validación live (3 ciclos × 2 modos × análisis) | 4–6 h | Sí (requiere stack vivo) |
| B — Borrado legacy + ajuste de baselines | 6–8 h | Sí, depende de A |
| C — Re-validación tests + live | 2–4 h | Sí, depende de B |
| D — Cleanup tests obsoletos | 2–3 h | No, opcional |
| **Total** | **14–21 h** | — |

## Por qué este plan se documenta y no se ejecuta hoy

El cierre estricto de F5 requiere **levantar el stack ROS+Gazebo** y
ejecutar pick cycles reales para validar paridad orchestrator↔legacy.
La sesión actual de refactor offline no puede validar eso de forma
segura. Borrar 8.985 LOC sin validación live equivale a dejar la
demo en estado dudoso — riesgo inaceptable.

**Decisión**: documentar el plan completo, ejecutar lo seguro hoy
(F1+F2+F3-step6+F4), y dejar el borrado para una sesión con stack
vivo. La trayectoria es correcta y el delta cuantificado.
