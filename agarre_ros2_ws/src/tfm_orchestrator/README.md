# tfm_orchestrator

Nodo ROS 2 que aloja `PickPlace.action` y orquesta el flujo pick &
place mediante una **state machine pura** (`pick_fsm.PickContext`)
testeable sin ROS.

## Por qué este paquete

Antes de F5/F6 toda la lógica del state machine vivía dentro de
`panel_pick_demo.run_pick_demo` (10.7 kLOC) — una closure gigante
embedida en el proceso del panel Qt. Cualquier cambio del flujo
requería tocar el panel; el flujo no era testeable sin Qt + ROS +
Gazebo.

`tfm_orchestrator` separa el flujo en:

* **`pick_fsm.py`** — FSM puro, sin imports de ROS. 100% testeable
  en pytest sin entorno. Define los estados (`PickPhase`),
  transiciones permitidas y `PickContext` con el snapshot dict
  mapeable directamente al feedback de la action.

* **`pick_orchestrator_lifecycle_node.py`** — `LifecycleNode` canónico
  (desde F9) que aloja el action server `/pick_place`. Instancia
  `PickContext`, lo avanza por las fases y publica feedback al cliente.
  Ciclo: `configure → activate → deactivate → cleanup → shutdown`.

* **`phase_dispatch.py`** — capa pura que invoca services/actions
  externos por cada fase (Open/Close/Attach/Detach + PlanToPose).

* **`run_id.py`** — generador y formateador puro del **PICK_RUN_ID**
  (8 chars hex). Cada goal `/pick_place` arranca con un ID nuevo que
  el orchestrator publica en `/pick/run_id` (`std_msgs/String`,
  latched: TRANSIENT_LOCAL+RELIABLE). Cualquier nodo —`evidence_logger`
  ya lo hace— puede suscribirse y etiquetar sus eventos con el ID
  activo, permitiendo correlacionar por sesión sin emparejar
  timestamps.

  Formato canónico de log:

      [PICK_RUN_ID=<id8>][PHASE=<name>][NODE=<short>][STATUS=<S/F/W/I>] msg key=val ...

  Helper: `tfm_orchestrator.run_id.format_log_line(...)`. El
  orchestrator usa el wrapper `_run_log(phase, status, msg, **kw)`
  para no recordar el ID en cada call.

## Arquitectura objetivo

```
                 ┌────────────────────┐
                 │  panel_v2 (Qt)     │  ← cliente thin
                 │  ActionClient      │
                 └─────────┬──────────┘
                           │ /pick_place (PickPlace.action)
                           ▼
                 ┌────────────────────┐
                 │ tfm_orchestrator   │  ← este paquete
                 │ ActionServer +     │
                 │ PickFSM            │
                 └─┬────┬────┬────┬───┘
                   │    │    │    │
       ┌───────────┘    │    │    └────────────┐
       ▼                ▼    ▼                 ▼
   Open/Close/      Attach/Detach     ComputeApproachPose
   SetWidth.srv     (gripper          + WorldToBase.srv
   (gripper          attach            (tf_geometry)
    ctrl)            backend)
```

Las service clients son F6 (siguiente paso). F5 implementa el FSM
y el action server con stubs.

## Estados del FSM

```
IDLE → SELECT_OBJECT → APPROACH → GRASP → LIFT → TRANSPORT → RELEASE → DONE
```

Estados terminales: `DONE`, `FAILED`, `ABORTED`. Cualquier fase
no-terminal puede transicionar a `FAILED` (excepción interna) o
`ABORTED` (cancelación del cliente). No se puede saltar fases ni
retroceder.

## Action contract — `PickPlace.action`

Definida en `ur5_panel_interfaces/action/PickPlace.action`.

```
# Goal
string object_name
geometry_msgs/Point drop_xyz_world
---
# Result
bool success
string reason
float64 duration_sec
int32 cycles_completed
---
# Feedback (publicado en cada transición de fase)
string current_phase   # PickPhase enum value (str)
float32 progress       # [0.0, 1.0] del happy path
int32 phase_index      # 0..N en happy path; -1 si FAILED/ABORTED
string detail          # detalle libre del orchestrator
```

## Lanzar

```bash
ros2 run tfm_orchestrator pick_orchestrator
```

Probar desde otra terminal:

```bash
ros2 action send_goal --feedback /pick_place \
  ur5_panel_interfaces/action/PickPlace \
  "{object_name: box_red, drop_xyz_world: {x: 0.5, y: 0.0, z: 0.05}}"
```

## Tests

```bash
cd agarre_ros2_ws/src/tfm_orchestrator
PYTHONPATH=. pytest -q test/test_pick_fsm.py   # 23 tests, ~30ms
```

Cubren: happy path completo, transiciones permitidas/denegadas, fail
desde cualquier fase, abort, snapshot de feedback, progresión
monotónica, estados terminales no transicionan.

## Estado F5/F6

* ✅ F5: paquete + FSM + action server + 23 tests.
* ✅ F6.1: helpers de service call con timeout
  (`service_clients.call_service_with_timeout`) +
  `PhaseServiceMap` configurable + 15 tests offline (mockable).
* ✅ F6.2: nodo cablea fase → service real (cuando
  ``use_stubs:=false``):
  - `SELECT_OBJECT` → `SelectObject.srv` (`/panel/select_object`)
  - `GRASP` → `Close.srv` + `Attach.srv`
  - `RELEASE` → `Detach.srv` + `Open.srv`
* ✅ F6.3: `PlanToPose.action` definida + APPROACH/LIFT/TRANSPORT
  cableados al action client.
* ✅ F6.5: stub action server PlanToPose en
  ``ur5_tools/plan_to_pose_server.py`` (lógica pura testeable en
  ``plan_to_pose_logic.py``, 24 tests offline). Cierra la cadena
  end-to-end: orchestrator → /orchestrator/plan_to_pose stub → ok.
* ✅ F6.6: server PlanToPose con wiring real al bridge MoveIt
  opt-in via param ``use_real_bridge:=true``. Publica PoseStamped
  a ``/desired_grasp`` con frame_id codificado (``base_link|rid=N|
  uid=UUID|phase=PLAN_TO_POSE``) y espera el result en
  ``/desired_grasp/result`` correlando por UUID. Helpers
  ``encode_request_frame``/``parse_bridge_result``/
  ``result_matches_request`` testables sin ROS (15 tests offline
  adicionales).
* 🟡 F6.4 PARCIAL: cliente thin del orchestrator añadido al panel
  como **opt-in** (sin tocar el legacy `run_pick_demo` 10.7k LOC):
  - `ur5_qt_panel/pick_place_client_logic.py` — lógica pura
    (validation, conversion, feature flag) con 30 tests offline.
  - `ur5_qt_panel/pick_place_client.py` — `PickPlaceClient` class
    wrapping ActionClient + callbacks panel-friendly.
  - Activación: env var `PANEL_PICK_DEMO_USE_ORCHESTRATOR=1`. Por
    defecto el panel sigue usando el legacy run_pick_demo.
  - Pendiente: cablear el flag en `panel_v2` para que el botón
    "Pick Demo" envíe el goal vía cliente cuando esté activo. Es
    una integración de pocas líneas — queda para F6.4 final cuando
    el orchestrator esté validado E2E con servers reales.

## Parámetros runtime

| Parámetro | Default | Descripción |
|---|---|---|
| `use_stubs` | `true` | Si false, el orchestrator llama a los services reales en cada fase. |
| `service_discovery_timeout_sec` | `2.0` | Cuánto esperar a que aparezca cada service. |
| `service_call_timeout_sec` | `10.0` | Cuánto esperar a la respuesta tras enviar la request. |

```bash
# Modo real (requiere todos los services levantados):
ros2 run tfm_orchestrator pick_orchestrator --ros-args -p use_stubs:=false

# Modo stub (default, para CI / smoke / dev sin Gazebo):
ros2 run tfm_orchestrator pick_orchestrator
```
