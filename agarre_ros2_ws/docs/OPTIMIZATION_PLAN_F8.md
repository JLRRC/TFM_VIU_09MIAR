# F8 — Plan de optimización rendimiento ciclo pick

**Fecha**: 2026-05-08
**Baseline**: 174.9s por ciclo (run F1.14+F1.15, HEAD `32119d0`, primer cycle E2E completo del historial)
**Objetivo**: ≤ 60s por ciclo (publicable + estable para 3 cycles consecutivos)
**Status**: helpers offline ✅ (este commit) — aplicación a archivos pendiente live

## Baseline observado (cycle 1/3, box_red, 174.9s)

Datos del run live 2026-05-08 (memoria `project_e2e_first_cycle_20260508`):

| Fase | Duración estimada | Bottleneck |
|------|------|------|
| HOME_INITIAL | ~5s | OK |
| APPROACH | ~30-40s | OMPL planning + execution lento |
| GRASP_DOWN | ~15-20s | OK |
| GRASP | ~5-10s | OK |
| LIFT | ~15-20s | OK |
| **TRANSPORT** | **~85s** | **CRÍTICO** — F1.18 lo subió de scaling 0.25 → 0.5 (50% reducción esperada) |
| RELEASE | ~10s | OK |

**Bottleneck #1**: TRANSPORT (~85s, ~49% del cycle). F1.18 lo mitiga subiendo scaling
0.25 → 0.5; estimación post-fix: ~45s.

**Bottleneck #2**: APPROACH (~30-40s). OMPL planning con `num_planning_attempts=5` +
trayectoria con scaling 0.25. F1.17 ya lo subió de 0.1 → 0.25 (60% más rápido).

**Bottleneck #3**: lookups TF redundantes. El closure `worker` de `panel_pick_demo`
hace cientos de `tf2_ros.Buffer.lookup_transform` en cada cycle, muchos del mismo
(target, source) en la misma fase. Sin batching, cada lookup paga el lock + búsqueda
en buffer (≈1-3ms cada uno × 200+ = 200-600ms por cycle, contributos pequeños pero
acumulativos).

## Helpers offline disponibles HOY (commit F8 / #20 partial)

### `ur5_tools.tf_batch_lookups`

Función pura `batch_lookup_requests(requests: List[LookupRequest]) → BatchPlan`:
- Deduplica peticiones idénticas (mismo target+source+time).
- Latest (Time(0)) y timestamped se tratan separadamente.
- Cuando se deduplica, gana el timeout más permisivo (max).
- Devuelve `BatchPlan` con estadísticas (`saved_calls`, `duplicates_grouped`).

Workload realista del run E2E (5 lookups del mismo par + 3 distintos + 2
timestamped) → 60% reducción (10 → 4 únicos).

**Aplicación pendiente**: `panel_pick_demo` y `panel_pick_object` deben acumular
sus lookups en una lista, llamar `batch_lookup_requests`, y ejecutar sólo los
únicos. Esto requiere refactor del closure `worker` (alto riesgo sin live test).

### `ur5_tools.cycle_timing_analyzer`

Parser de `log/ros2_launch.log` que detecta markers `[ORCHESTRATOR_LC][<PHASE>] start`
y `[ORCHESTRATOR_LC][<PHASE>] end success=...` y reporta:
- Duración de cada fase.
- Total cycle.
- Success/fail per fase.

Uso (programático):

```python
from ur5_tools.cycle_timing_analyzer import analyze_cycle, format_cycle_summary
with open("log/ros2_launch.log") as f:
    cycle = analyze_cycle(f.readlines())
print(format_cycle_summary(cycle))
```

Output ejemplo (caso real test_analyze_cycle_full_pipeline):

```
cycle: success=True n_phases=7
total_duration=175.0s
        HOME_INITIAL:    5.0s [OK]
            APPROACH:   35.0s [OK]
          GRASP_DOWN:   15.0s [OK]
               GRASP:    7.0s [OK]
                LIFT:   18.0s [OK]
           TRANSPORT:   85.0s [OK]
             RELEASE:   10.0s [OK]
```

**Uso académico**: facilita comparar runs (ANTES de F1.18 / DESPUÉS de F1.18)
sin parseo manual.

## Plan de aplicación (post-T35 verde)

### F8-step1: aplicar `tf_batch_lookups` (offline factible tras live ack)

1. Identificar callsites de `lookup_transform` en `panel_pick_demo.py` (grep
   `lookup_transform`, ~30 sites).
2. Agruparlos por fase (HOME_INITIAL, APPROACH, ...).
3. Refactor: cada fase acumula sus lookups → llama `batch_lookup_requests` →
   ejecuta los únicos.
4. Test: comparar `cycle_timing_analyzer` antes/después.
5. Estimación impacto: 200-600ms ahorrados por cycle (3% del cycle).

### F8-step2: subir scaling de TRANSPORT 0.5 → 0.7 (live)

F1.18 dejó TRANSPORT en scaling 0.5. Subirlo a 0.7 reduciría TRANSPORT de ~45s
a ~32s (33%). Pero requiere validación live de path_tolerance — si scaling
es muy alto, el controller pierde tracking y dispara `path_tolerance_violation`.

Pendiente:
- Cerrar `BUG_CONTROLLER_FEEDBACK_HANG` primero.
- Run × 3 con scaling 0.6, 0.65, 0.7. Identificar sweet spot.

### F8-step3: eliminar timers polling redundantes

`panel_pick_demo` tiene varios `QTimer` con períodos cortos (50-100ms) que
duplican información (ej. tcp_pose_world se publica ya en `world_tf_publisher`
a 50Hz pero el panel también lo lee independientemente).

Plan:
1. Listar timers Qt activos en `panel_v2._init_timers`.
2. Identificar redundancias con publishers ROS.
3. Reemplazar timers por subscribers.
4. Estimación impacto: 5-10% CPU del panel ahorrado, no afecta directamente
   tiempo de cycle pero reduce jitter.

### F8-step4: optimizar OMPL planning

`num_planning_attempts=5` con `allowed_planning_time=5.0s` puede gastar 25s
sólo en planning. Para targets fáciles (HOME, drop pose) podría usarse
`RRTConnect` con 1 attempt.

Plan:
1. Categorizar targets por dificultad (heurística workspace + collision).
2. Aplicar planner_id + planning_time per fase.
3. Estimación impacto: 10-15s ahorrados en APPROACH+TRANSPORT.

## Referencias

- Memoria: `project_e2e_first_cycle_20260508` (174.9s baseline).
- Bug docs: `BUG_BRIDGE_PATH_TOLERANCE.md`, `BUG_CONTROLLER_FEEDBACK_HANG.md`.
- F1.18 commit: `744006a fix(F1.18): per-phase scaling/timeout`.
- F1.17 commit: `818742c fix(F1.17 partial): trajectory scaling 0.1→0.25`.
- T35 guide: `T35_E2E_3_CYCLES_GUIDE.md`.

## Tests offline (este commit)

- `test/test_tf_batch_lookups.py` — 13 tests (empty, single, identical-collapse,
  different-pairs, timeout-max-wins, timestamped-vs-latest, real-workload,
  reduction-ratio, whitespace-strip).
- `test/test_cycle_timing_analyzer.py` — 15 tests (parse empty/single/full/
  failed/multi-phase/dedup-takes-first; analyze legacy/orchestrator/full-pipeline;
  format empty/basic/failed/incomplete).

Total: 28 tests offline + mypy strict verde.
