# Estado final del refactor profesional — TFM agarre_ros2_ws

> Última actualización: 2026-05-02.
> Rama: `audit/fase-0-1-cleanup` (26 commits sobre `main`).

## 1. Resumen ejecutivo

Sistema completamente refactorizado a **arquitectura profesional ROS 2**:

* **8 paquetes ROS 2** con responsabilidades bien delimitadas.
* **8 LifecycleNodes** en producción (era 1).
* **16 mixins** organizan los métodos del panel Qt.
* **2 microservicios standalone** dedicados (`tfm_orchestrator`,
  `tf_geometry_service`).
* **~860 tests offline** en CI verde por commit.
* **17 tags de rollback** con trazabilidad completa.
* **5 docs canónicos** publicados.

## 2. Métricas finales

### God-files reducidos

| Fichero | Inicial | Final | Δ |
|---|---:|---:|---:|
| `panel_v2.py` | 2 795 | **1 197** | **-57%** |
| `ur5_stack.launch.py` | 887 | 706 | -20% |

### Code organización

| Concepto | Antes | Después |
|---|---:|---:|
| Mixins de `panel_v2` | 0 | **16** |
| LOC en mixins | 0 | ~2 700 |
| Microservicios standalone | 0 | 2 |
| LifecycleNodes en producción | 1 | **8** |
| Tests offline en CI | ~430 | **~860** |
| Tags rollback | 7 | **17+** |
| Docs canónicos | 1 | **6** |

## 3. Arquitectura de mixins (`panel_v2.py`)

```python
class ControlPanelV2(
    PanelV2PublisherMixin,            # 11 — publishers MoveIt
    PanelV2BasePoseMixin,             # 11 — TCP / base / world frame
    PanelV2GripperAttachMixin,        #  6 — gripper + attach
    PanelV2MotionMixin,               #  9 — action client + waits
    PanelV2TrajSettleMixin,           # 13 — trayectoria + objects settle
    PanelV2SystemStateMixin,          # 29 — UI build + signals + state
    PanelV2StepDebugMixin,            # 72 — step UI + cart debug + direct flow
    PanelV2RuntimeDiagnosticsMixin,   # 97 — logs + gazebo + camera + controllers
    PanelV2SubprocessMotionMixin,     # 64 — start/stop + motion control
    PanelV2AuditLogMixin,             # 29 — audit + ready + control_status
    PanelV2TfmRemoteMixin,            # 27 — TFM grasp + remote callbacks
    PanelV2ReadyReasonsMixin,         # 15 — *_not_ready_reason + status
    PanelV2DropRecoverMixin,          # 19 — drop / release / recover / hold
    PanelV2CalibPickMixin,            # 32 — calibración + pick + slider + objects
    PanelV2OverlaysSelectionMixin,    # 32 — overlays + selección + history
    PanelV2TfmScienceTraceMixin,      # 64 — TFM science + trace + tf sanity
    QMainWindow,
)
# 530+ wrappers extraídos a 16 mixins testeados estructuralmente
```

## 4. LifecycleNodes en producción

| Nodo | Tipo | Tests |
|---|---|---|
| `pick_orchestrator_lifecycle` | ✅ Pleno (F9) | 97 (FSM puro) |
| `world_tf_publisher` | ✅ Pleno (F13) | 9 (lifecycle contract) |
| `system_state_manager` | ✅ Pleno (F13) | 9 |
| `gripper_attach_backend` | ✅ Observable (F13) | 9 |
| `ur5_moveit_bridge` | ✅ Observable (F13) | 9 |
| `tf_geometry_service` | ✅ Pleno (F16) | 9 + 16 lógica pura |
| `release_objects_service` | ✅ Observable (F13b) | 9 |
| `gz_pose_bridge` | ✅ **Pleno (F13c)** | 9 |

## 5. Microservicios dedicados

### `tfm_orchestrator`

* `pick_orchestrator_lifecycle` — LifecycleNode F9 con `PickPlace.action`.
* `pick_fsm.py` — FSM puro testeable sin ROS.
* `service_clients.py` — wrappers de `Open/Close/SetWidth/Attach/Detach/SelectObject`.
* 97 tests (FSM, service_clients, phase_timings, lifecycle_helpers).

### `tf_geometry_service` (F16, 2026-05-01)

* LifecycleNode Pleno standalone.
* Servicios `/tf_geometry/world_to_base` y `/tf_geometry/compute_approach_pose`.
* `tf_geometry_logic.py` — lógica matemática pura (rotación quaternion,
  transformaciones rígidas, approach pose).
* `panel_tf_geometry_client.py` (F16-step3) — cliente Python wrapper
  para consumidores con timeout, fallback graceful y validación.

## 6. Docs canónicos publicados

| Doc | Contenido |
|---|---|
| `architecture.md` | Arquitectura actual + objetivo + decisiones canónicas |
| `LIFECYCLE.md` | Política + 8 LifecycleNodes (Pleno vs Observable) |
| `CONFIG_HIERARCHY.md` | env > YAML > defaults |
| `REFACTOR_F14_PATTERN.md` | Patrón mixin establecido + roadmap |
| `F12_STEP2_ROADMAP.md` | Plan multi-commit drenaje legacy |
| `REFACTOR_FINAL_STATE.md` | (este doc) Estado final |

## 7. Telemetría / observabilidad

### `evidence_logger` (F10/F18/F19)

* Schema documentado (`events.jsonl`, `summary.csv`, `metrics.json`).
* Helpers F18 puros: `compute_inter_event_latencies`,
  `compute_event_rates`, `generate_latency_report_md`.
* F19: `MutuallyExclusiveCallbackGroup` para serializar I/O en
  callbacks (evita race conditions sobre file descriptors).

### `perf_helpers` (F19)

* `compute_percentiles(samples, percentiles)` — p50/p95/p99 con
  ordenación + índice discreto.
* `detect_periodic_oscillation` — detecta publishers desincronizados.
* `classify_topic_health` — `ok`/`slow`/`fast`/`missing`.
* `summarize_performance_run` — agrega n+mean+min+max+percentiles
  por métrica, listo para LaTeX/Markdown.
* 20 tests offline.

## 8. Estado de F12-step2 (drenaje legacy)

* **F12-step1 (cerrado)**: orchestrator es default; legacy es rollback
  controlado por `USE_LEGACY_PICK_DEMO=1`.
* **F12-step2 (cerrado parcial)**:
    - Cada llamada a `run_pick_demo` incrementa contador y emite log.
    - Cada llamada se registra en `audit/legacy_pick_demo.log`.
    - Permite auditar uso residual antes de eliminar el legacy.
* **F12-step2c (pendiente, requiere ROS vivo)**: migración real de
  fases legacy → orchestrator (10-16 h, plan en
  `docs/F12_STEP2_ROADMAP.md`).

## 9. Estado de migración `tf_geometry_service`

* Nodo cableado en `runtime_nodes_factory` (F16-step2).
* Cliente Python listo: `TfGeometryClient` (F16-step3).
* **Pendiente** (requiere ROS vivo): consumidores actuales en panel +
  `panel_state_methods` deben sustituir cálculos TF locales por
  llamadas al servicio.

## 10. Optimizaciones aplicadas (F19)

| Optimización | Estado |
|---|---|
| `MutuallyExclusiveCallbackGroup` en `evidence_logger` | ✅ aplicado |
| `gz_pose_bridge` recursos segregados (Pleno) | ✅ aplicado |
| Helpers analíticos (`perf_helpers`) | ✅ disponibles |
| Reducir Hz de bridges innecesarios | ⏳ requiere medición ROS vivo |
| `MutuallyExclusiveCallbackGroup` en orchestrator | ⏳ requiere medición |

## 11. Pendiente real (todo lo offline está cerrado)

| Tarea | Esfuerzo | Bloqueo |
|---|---|---|
| F12-step2c — drenaje legacy fase a fase | 10-16 h | ROS vivo + `validate_pick_3_cycles.sh` |
| F13c-step2/3 — segregar release/attach/moveit_bridge a Pleno | 6-10 h | ROS vivo |
| Migración consumidores tf_geometry_service | 3-5 h | ROS vivo |
| Optimizaciones reales con `perf_helpers` | 8-12 h | ROS vivo |

**Total restante (todo requiere ROS vivo):** ~30-45 h.

## 12. Cómo usar este refactor

```bash
# Verificar que todo arranca limpio
cd /home/laboratorio/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/start_panel_v2.sh

# Ver el ciclo de vida de los 8 lifecycle nodes
ros2 lifecycle nodes
ros2 lifecycle get /pick_orchestrator_lifecycle
ros2 lifecycle get /tf_geometry_service

# Llamar a los servicios geométricos
ros2 service call /tf_geometry/world_to_base \
  ur5_panel_interfaces/srv/WorldToBase \
  "{world_xyz: {x: 0.5, y: 0.0, z: 1.0}}"

# Ver los 16 mixins de panel_v2
grep -E "^class PanelV2.*Mixin" \
  src/ur5_qt_panel/ur5_qt_panel/panel_v2_*_mixin.py

# Volver a un punto previo si hay regresión
git tag --list 'audit-pre-*'
git checkout audit-pre-f14-step13-20260502
```

## 13. Cierre

El proyecto está en **estado profesional** para defensa académica
y para producción simulada. Lo que queda pendiente es trabajo de
validación E2E con stack arrancado, no más refactor estructural.
