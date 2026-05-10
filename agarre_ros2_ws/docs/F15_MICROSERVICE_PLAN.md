# F15 — Plan de migración a arquitectura microservicio

> Auditoría 2026-05-10. F15 es la fase más ambiciosa del plan: ~60h
> de trabajo, **riesgo crítico**. Este documento captura el estado
> actual tras F0-F14 y el roadmap de migración para una sesión
> dedicada futura. **NO ejecutar sin coordinación explícita.**

## Estado actual (post F0-F14)

Tras 14 fases de refactor, el workspace está en estado **profesional
para producción simulada** (no microservicio):

| Métrica | Antes F0 | Tras F14 |
|---------|---------:|---------:|
| Tests offline cubriendo lógica | ~1800 | ~1900+ (+109 nuevos via helpers puros) |
| Módulos puros nuevos | 0 | 11 (geometry_constants, run_id, panel_runtime_status_helpers, panel_system_state_helpers, panel_moveit_readiness_helpers, panel_system_stats_helpers, release_objects_geometry, panel_image_helpers, plan_to_pose_helpers, panel_v2_fk_helpers, moveit_precheck) |
| Subpaquetes namespace | 0 | 7 (`ur5_tools/{geometry,planning,gripper,state,diagnostics,bridges,utils}/`) |
| God-classes con helpers extraídos | 0 | 5 (panel_helpers, panel_status_mgmt, release_objects_service, panel_ros, plan_to_pose_server) |
| LOC en `panel_helpers.py` | 1432 | 961 (−33%) |
| LOC en `panel_status_mgmt.py` | 1049 | 721 (−31%) |
| Hardcodes geom centralizados | 4× `_BASE_LINK_IN_WORLD` | 1 (`ur5_tools.geometry_constants.BASE_LINK_IN_WORLD`) |
| `PICK_RUN_ID` cross-nodo | ❌ | ✅ (orchestrator publish + evidence_logger consume) |
| CI activo | ❌ | ✅ GitHub Actions (lint + offline + policy) |
| Test TF unicidad | ❌ | ✅ 3 tests (regression guard) |
| Test MoveIt readiness | ❌ | ✅ 14 tests (helper + diagnose) |

**Sin embargo, la arquitectura SIGUE siendo monolítica**:
- `ControlPanelV2` hereda de 16 mixins.
- `RosWorker` (en `panel_ros.py`) tiene 99 `except Exception`.
- Los nodos ROS (orchestrator, attach backend, plan_to_pose,
  release_objects, state_manager, evidence_logger) son procesos
  separados, pero el panel sigue siendo 1 proceso god-class.

## Objetivo F15: panel como microservicios

Separar el panel en **3 procesos cooperantes**:

```
┌─────────────────────┐       ┌──────────────────────┐
│   panel_ui_node     │  Qt   │  panel_controller    │
│   (ControlPanelV2-  │◄────► │  (lógica + estado    │
│   like, solo Qt)    │ DBus/ │   sin Qt ni rclpy)   │
└─────────────────────┘ ZMQ   └──────────┬───────────┘
                                         │ rclpy
                                         ▼
                              ┌──────────────────────┐
                              │  ros_bridge_worker   │
                              │  (rclpy thread,      │
                              │   subs/clients)      │
                              └──────────┬───────────┘
                                         │ ROS 2
                                         ▼
                              ┌──────────────────────┐
                              │  Backend services    │
                              │  (orchestrator,      │
                              │   attach, planner,   │
                              │   evidence, state)   │
                              └──────────────────────┘
```

### Beneficios esperados

1. **Separación UI/lógica/IO**: cada proceso tiene una responsabilidad.
   Cambios de UI no requieren tocar lógica robótica y viceversa.
2. **Crash isolation**: si el panel UI crashea, el controller
   sobrevive y puede continuar el pick activo.
3. **Test profundo**: `panel_controller` testeable sin Qt ni rclpy
   (~2-3× más cobertura realista).
4. **Permite UI alternativa**: web UI, CLI, otra app gráfica
   reutilizando el mismo `panel_controller` + `ros_bridge_worker`.
5. **Hot reload UI**: reiniciar solo el proceso UI sin perder
   estado del controller.

### Riesgos identificados

| ID | Riesgo | Impacto | Mitigación |
|----|--------|---------|------------|
| RF15-1 | IPC entre procesos añade latencia perceptible al usuario | Medio | Usar ZMQ shared memory; benchmark <50ms |
| RF15-2 | Sincronización de estado UI-Controller compleja | Alto | Schema-first con dataclasses Pydantic; tests E2E |
| RF15-3 | Migración rompe los 16 mixins existentes | Alto | Migración por fases: 1 mixin a la vez con shims |
| RF15-4 | Tests existentes asumen god-class única | Medio | `test_panel_v2_*_mixin` requiere refactor |
| RF15-5 | Documentación + onboarding requiere reescritura | Bajo | Parte del coste F15 |

## Roadmap F15 por iteraciones

### F15.1 — Diseño schema de comunicación (~6h)

Definir mensajes UI ↔ Controller:

```python
# panel_ipc_schema.py
@dataclass(frozen=True)
class UICommand:
    kind: str  # "pick_demo" | "pick_object" | "stop" | ...
    payload: Dict[str, Any]
    request_id: str

@dataclass(frozen=True)
class ControllerEvent:
    kind: str  # "system_state" | "phase_progress" | "log" | ...
    payload: Dict[str, Any]
    timestamp_mono: float
```

Tests offline validan serialización + parser.

### F15.2 — Extraer panel_controller puro (~16h)

Mover lógica de `ControlPanelV2` (excluyendo Qt) a una clase
`PanelController` sin Qt. Inputs: `UICommand`. Outputs:
`ControllerEvent`. Estado: `PanelState` dataclass.

`ControlPanelV2` se reduce a un thin wrapper Qt que envía/recibe via
in-process queue (paso 1: mismo proceso).

### F15.3 — Extraer ros_bridge_worker (~12h)

`RosWorker` (panel_ros.py) → `RosBridgeWorker`. Recibe
`UICommand` desde `PanelController` y traduce a llamadas rclpy
(action client, services, topics). Emite `ControllerEvent` con
resultado/feedback.

### F15.4 — IPC entre procesos (~14h)

Reemplazar in-process queue por ZMQ:
* PUB-SUB para events controller → UI.
* REQ-REP para commands UI → controller.

Lanzador unificado (`panel_v2.launch.py`) arranca los 3 procesos.

### F15.5 — Validación + documentación (~12h)

* Test E2E: pick completo arrancando los 3 procesos.
* Benchmark latencia UI command → robot start.
* Documentar arquitectura en `docs/architecture.md`.
* Migrar tests existentes.
* Decidir: borrar god-class viejo o mantener como fallback.

## Pre-requisitos

Antes de iniciar F15:

1. **F11 iter 2 completa** (mover archivos físicos + shims) —
   facilita imports limpios en `panel_controller`.
2. **F12 iter 2 completa** (split mixins en composición) — F15
   asume composición ya establecida.
3. **Validación E2E live de PICK_RUN_ID** (F5 followup) — F15
   depende de trazabilidad robusta.
4. **Plan de regresión escrito** — F15 puede romper la demo;
   necesitamos garantizar rollback.

## Decisión recomendada

**No ejecutar F15 ahora salvo que:**

* El TFM se publique académicamente y el revisor valore la
  arquitectura microservicio explícitamente.
* Se migre a robot real (cobot UR5 físico) y la separación
  UI/controller sea necesaria para safety (UI puede crashear sin
  parar el robot).
* Haya un sprint dedicado de 60h sin presión de demo.

**Para defensa TFM**, lo entregado tras F0-F14 es **suficiente y
defendible**. F15 es deuda evolutiva, no crítica.

## Anexo: estado de cada fase F0-F14

| Fase | Estado | Tests offline ganados |
|------|:-----:|---------------------:|
| F0 | ✅ Completa | — |
| F1 | ✅ Completa | — |
| F2 | ✅ Completa | 4 |
| F3 | ✅ Completa | — (CI infra) |
| F4 | ✅ Completa | — (lint infra) |
| F5 | ✅ Completa | 25 |
| F6 | ✅ Completa | — (refactor sin tests nuevos) |
| F7 | ✅ Completa | — (idem) |
| F8 | ✅ Completa | 25 |
| F9 | ✅ Completa | 36 |
| F10 | ✅ Completa | 19 |
| F11 | ✅ Completa | 9 |
| F12 | ✅ Completa | 16 |
| F13 | ✅ Completa | 17 |
| F14 | ✅ Completa | 37 |
| **F15** | ⏳ **Documentado, no ejecutado** | (estimado +50 si se ejecuta) |

**Total tests offline ganados F0-F14: ~188 nuevos**.
