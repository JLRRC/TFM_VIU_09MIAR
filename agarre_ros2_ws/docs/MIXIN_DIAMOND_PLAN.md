# Plan elimination del mixin diamond en ControlPanelV2

**Estado**: deferred a v1.1 (post audit-v4 close).
**Fecha**: 2026-05-08

## Estado actual

`ControlPanelV2` (panel_v2.py) hereda de **17 mixins** + `QMainWindow`:

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
):
    ...
```

## Problema

- **Diamond implícito**: cada mixin asume `self.X` para multiplicidad de slots. La ejecución del MRO es lineal pero acoplada.
- **Tests por mixin son estructurales**: validan método-presence, no comportamiento.
- **17 mixins comparten state vía `self`** — funcionalmente Python no detecta el "diamond" porque no usa diamond formal, pero hay ambigüedad de responsabilidad.

## Target: composición + delegación

```python
class ControlPanelV2(QMainWindow):
    def __init__(self, ...):
        super().__init__()
        self._publishers = PanelV2Publishers(self)
        self._motion = PanelV2Motion(self)
        # ... 17 sub-objetos en lugar de 17 mixins
        self._publishers.init_publishers()
```

Ventajas:
- Each component is testable in isolation.
- No MRO complexity.
- Lifecycle explícito (init, destroy).

## Costes

- ~20-30h de refactor (todos los call sites cambian de `self.method()` a `self._comp.method()`).
- Re-testear cada mixin tras conversión.
- Riesgo medio (Qt slots/signals deben re-cablearse).

## Plan iter por iter

| Iter | Mixin convertido | Objetivo | Validación |
|---|---|---|---|
| 1 | PanelV2AuditLogMixin (29 wrappers) | proof of pattern | offline tests |
| 2 | PanelV2ReadyReasonsMixin (15) | quick win | offline |
| 3 | PanelV2DropRecoverMixin (19) | quick win | offline |
| 4 | PanelV2BasePoseMixin (11) | media | live + offline |
| 5 | PanelV2GripperAttachMixin (6) | media | live |
| 6+ | resto progresivamente | media-alta | live por iter |
| 17 | Delete inheritance, sólo QMainWindow | final | T35 × 3 verde |

## Test invariant (audit-v4): MRO sin conflictos

Mientras la conversión no se haga, este test gating al menos detecta
si alguien **añade un nuevo mixin sin pensar**:

Ver `src/ur5_qt_panel/test/test_panel_v2_mro.py` (audit-v4).
