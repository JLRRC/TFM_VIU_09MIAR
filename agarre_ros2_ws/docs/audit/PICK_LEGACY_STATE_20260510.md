# Estado real del "pick legacy" — 2026-05-10

Este documento corrige una conclusión del audit `AUDIT_20260510` que
sobre-estimaba el legacy residual.

## TL;DR

El **closure principal** `run_pick_demo` (~8080 LOC monolíticas) fue
**borrado el 2026-05-08** (tag `audit-pre-borrar-legacy-20260508`).
Lo que el audit llamó "pick legacy 5637 LOC" son **helpers de soporte
que aún sirven a otros módulos del panel** (debug, geometry, physics,
trace). NO son el legacy crítico.

## Estado real

### Path crítico del botón "Pick Demo" (post-F5)

```
panel_v2.btn_pick_demo.clicked
  → panel._run_pick_demo                            (mixin)
  → panel_v2_calibpick_mixin._run_pick_demo
  → panel_calib_actions._run_pick_demo              ← punto único de
                                                       entrada moderno
  → pick_demo_dispatcher.dispatch_pick_demo
  → PickPlaceClient.send_goal('/pick_place', ...)   ← action server
                                                       (orchestrator)
```

### Helpers residuales activos (aún usados)

| Módulo | LOC | Importadores |
|---|---:|---|
| `panel_pick_demo.py` | 535 | directo_geometry, panel_physics, panel_trace_callbacks, panel_pick_demo_params |
| `panel_pick_demo_params.py` | ~700 | dataclass de params del pick (config) |
| `pick_demo/` (16 submódulos) | 5102 | cada submódulo importado por 1-2 callers fuera del subpaquete |

Cada submódulo de `pick_demo/` tiene **1 caller (no-test, no-self)** por
la auditoría hecha hoy. Eso indica acoplamiento bajo: cada submódulo
es candidato a:

1. Mover a un home más natural (e.g. `pick_demo/geometry.py` puede
   pasar a `panel_pose_helpers.py` si su único caller es ese).
2. Borrar si el caller a su vez es huérfano.
3. Mantener si su caller es activo.

## Acciones pendientes para llegar a 100/100 con B4

1. Auditar los 16 submódulos de `pick_demo/` con un script:
   - Para cada submódulo, identificar SU único caller.
   - Decidir: ¿el caller sigue siendo usado por el panel activo?
   - Si NO → submódulo es transitivamente huérfano → borrar.
   - Si SÍ → mover el helper al módulo del caller, eliminar el subpaquete.

2. Auditar `panel_pick_demo.py`:
   - 4 importadores: ver qué importan exactamente.
   - Si son símbolos puntuales (constantes, formatters), moverlos
     al sitio del consumidor.
   - Borrar el shell.

3. Validar runtime tras cada borrado:
   - `python3 scripts/run_single_pick_pickdemo.py --object pick_demo`
   - Esperar SUCCESS.

4. Limpiar tests obsoletos:
   - Los tests `test_pick_demo_*.py` que cubren submódulos borrados.
   - Mantener los que cubren la lógica que sobrevive (movida).

## Estimación de tiempo

| Acción | Estimación |
|---|---|
| Audit submódulos (16) | 2-3 h |
| Mover/borrar helpers | 4-6 h |
| Borrar `panel_pick_demo*.py` | 1-2 h |
| Limpieza tests | 2-3 h |
| **Total** | **9-14 h** |

(Era el estimado original 30-50h porque asumía borrado del closure
gigante, que ya estaba hecho. La cifra correcta es 9-14 h con runtime.)

## Riesgo

**Bajo**:
- Closure crítico ya fuera del path.
- Botones del panel van por orchestrator vía dispatcher.
- Cada submódulo es atomicamente borrable (acoplamiento bajo).
- Stack vivo permite validar después de cada borrado.

**No tocar**:
- `pick_demo_dispatcher.py` (es el bridge moderno).
- `pick_place_client.py` + `pick_place_client_logic.py` (cliente nuevo).
- `panel_calib_actions._run_pick_demo` (entry point moderno).

## Score impact corregido

- Audit original asumía B4 = +2.5 score (borrado del legacy).
- Realidad: B4 ya estaba ~80% cerrado en F5-legacy-removed-20260508.
- Pendiente real: ~+1 score (limpiar helpers residuales).

---

Documento Action 7 (revisión de B4) — 2026-05-10.
