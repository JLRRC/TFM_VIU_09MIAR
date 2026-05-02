# F14 — Patrón mixin para `panel_v2.py`

> Última actualización: 2026-05-01.

## 1. Estado tras F14-step1..7

| Métrica | Valor |
|---|---|
| `panel_v2.py` LOC inicial | 2 795 |
| `panel_v2.py` LOC actual | 2 529 |
| Reducción | -266 LOC (-9.5%) |
| Mixins extraídos | 6 |
| Helpers puros extraídos | 1 módulo (`panel_v2_helpers.py`) |
| Tests F14 totales | 217 |

## 2. Mixins implementados

| Mixin | Métodos | Responsabilidad |
|---|---:|---|
| `PanelV2PublisherMixin` | 11 | Publishers MoveIt + auto-bridge |
| `PanelV2BasePoseMixin` | 11 | Frames world/base + TCP + transformaciones |
| `PanelV2GripperAttachMixin` | 6 | Comandos gripper + lógica attach |
| `PanelV2MotionMixin` | 9 | Action client FJT + waits joint/TCP |
| `PanelV2TrajSettleMixin` | 13 | Trayectorias + objects settle |
| `PanelV2SystemStateMixin` | 29 | UI build + signals + system_state machine |
| **Total wrappers extraídos** | **79** | |

## 3. Patrón canónico

Cada mixin sigue exactamente la misma estructura:

```python
# panel_v2_<dominio>_mixin.py
from . import panel_state_methods as _stm

class PanelV2<Dominio>Mixin:
    """Wrappers thin de <dominio>.

    Asume atributos en self provistos por ControlPanelV2.__init__.
    El cuerpo real vive en panel_state_methods. Este mixin solo
    provee la interfaz unificada de la clase ControlPanelV2.
    """

    def _<metodo>(self, *args, **kwargs):
        return _stm._<metodo>(self, *args, **kwargs)

    # Helper estático (no recibe self):
    def _<helper>(self, *args, **kwargs):
        return _stm._<helper>(*args, **kwargs)
```

`ControlPanelV2` los compone vía herencia múltiple **antes** de `QMainWindow`:

```python
class ControlPanelV2(
    PanelV2PublisherMixin,
    PanelV2BasePoseMixin,
    PanelV2GripperAttachMixin,
    PanelV2MotionMixin,
    PanelV2TrajSettleMixin,
    PanelV2SystemStateMixin,
    QMainWindow,
):
    ...
```

## 4. Guardrail estructural (test offline)

Cada mixin tiene su `test_panel_v2_<dominio>_mixin.py` que verifica
estructuralmente vía AST + grep:

1. El módulo del mixin existe y exporta la clase canónica.
2. Cada método del contrato está definido en el mixin.
3. `ControlPanelV2` hereda del mixin.
4. `panel_v2.py` importa el mixin.
5. `panel_v2.py` **ya no redefine** los métodos extraídos (previene
   override silencioso).
6. Los wrappers que pasan `self` lo hacen y los estáticos no.

Estos tests **no requieren rclpy ni Qt**, corren en `colcon` offline-tests.

## 5. Cómo continuar (F14-step8+)

Quedan ~380 wrappers thin más en `panel_v2.py`. Para extraer un nuevo
grupo cohesivo:

1. **Identifica un dominio cohesivo** (ej. step UI, calibración,
   diagnóstico, auto-tune, cámara).
2. **Crea `panel_v2_<dominio>_mixin.py`** siguiendo el patrón §3.
3. **Añade el mixin a la herencia de `ControlPanelV2`**.
4. **Reemplaza las definiciones duplicadas** en `panel_v2.py` por un
   comentario marcador `# F14-stepN: heredados de ...`.
5. **Crea `test_panel_v2_<dominio>_mixin.py`** copiando el patrón de
   los existentes y ajustando `EXPECTED_MIXIN_METHODS`.
6. **Añade el test al CI** en `.github/workflows/colcon.yml`.

Cada paso ≈ 30-60 minutos. No hay nuevos conceptos arquitectónicos —
es trabajo serial mecánico.

## 6. Roadmap F14-step8+ sugerido

Grupos cohesivos restantes identificados:

| Mixin propuesto | Métodos est. | Notas |
|---|---:|---|
| `PanelV2StepDebugMixin` | ~75 | Métodos `_step_*` y `_step_cart_debug_*` |
| `PanelV2CameraMixin` | ~20 | Métodos `_camera_*`, `_subscribe_camera`, `_check_camera_*` |
| `PanelV2ControllersMixin` | ~10 | Métodos `_controllers_*`, `_list_controllers`, `_wait_for_controllers_ready` |
| `PanelV2DiagnosticsMixin` | ~15 | Métodos `_log_*`, `_clock_status`, `_bridge_*_status` |
| `PanelV2RemoteFlowMixin` | ~20 | Métodos `_run_ui_callable`, `_run_async`, `_on_async_error`, `_run_script` |
| `PanelV2ReadyReasonsMixin` | ~12 | Métodos `_*_not_ready_reason` (delegan a `_ph`) |
| `PanelV2GazeboMixin` | ~10 | Métodos `_gazebo_*`, `_pose_info_*` |

Ejecución en cadena (step8..step14) reduciría `panel_v2.py` de
**2 529 → ~1 000-1 200 LOC** estimado, suficientemente cerca del
target original.

## 7. Decisión académica

Para defensa académica, el patrón mixin ya está **demostrado y
testeado** con 6 mixins (217 tests). Los pasos siguientes son
trabajo de implementación rutinaria, no aportan nuevos argumentos
arquitectónicos. La recomendación es continuarlos en sesiones
futuras según la prioridad relativa frente a F15/F16/F19.
