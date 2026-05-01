# tfm_orchestrator — F5/F6

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

* **`pick_orchestrator_node.py`** — nodo ROS 2 que aloja el action
  server `/pick_place`. Instancia `PickContext`, lo avanza por las
  fases y publica feedback al cliente.

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
* 🔴 F6.4 pendiente: reescribir `panel_v2.run_pick_demo` (10.7k LOC)
  como cliente de `/pick_place` — panel queda thin.
* 🔴 F6.6 pendiente: substituir el stub server PlanToPose por
  wiring real al bridge MoveIt (publish a `/desired_grasp` +
  consume `/desired_grasp/result`).

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
