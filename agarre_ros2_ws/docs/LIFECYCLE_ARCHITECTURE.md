# Arquitectura Lifecycle vs Node — F10 audit

## Inventario

| Nodo | Tipo | Razón | LC migration |
|---|---|---|---|
| `pick_orchestrator_lifecycle_node` | LC | Action server `/pick_place`, transiciones managed | ✓ ya |
| `gripper_attach_backend` | LC | Bridge attach físico/lógico crítico, requiere managed | ✓ ya |
| `system_state_manager` | LC | FSM global readiness, vital | ✓ ya |
| `world_tf_publisher` | LC | TF estático world→base_link, pre-warm | ✓ ya |
| `release_objects_service` | LC | Respawn DROP objects, recursos GZ | ✓ ya |
| `tf_geometry_service` | LC | Services geométricos centralizados | ✓ ya |
| `object_pose_resolver_service` | LC | Cache de poses, sub TF dinámico | ✓ ya |
| `gz_pose_bridge` | LC | Subprocess `gz topic` watchdog | ✓ ya |
| `evidence_logger` | LC | F10b cerrado 2026-05-10: managed con `auto_activate` | ✓ |
| `planning_scene_sync` | Node | **Pendiente** — sync MoveIt scene, riesgo medio | F10b |
| `plan_to_pose_server` | Node | **Pendiente** — 1430 LOC, riesgo alto, requiere split previo (F7) | F7+F10b |
| `controller_bootstrap` | Node | **NO migra** — one-shot bootstrap (load+config+activate). LC añadiría complejidad sin beneficio. | n/a |
| `gz_ros_control_guard` | Node | **NO migra** — one-shot config push remoto. LC innecesario. | n/a |
| `clock_probe`, `tf_probe`, `jt_smoke_test` | Node | **NO migra** — utilidades de debug, no pipeline. | n/a |

## Cobertura

* **9/12 nodos productivos** son LifecycleNode (75 %).
* **2 nodos productivos** quedan como Node: `planning_scene_sync`
  y `plan_to_pose_server`. Ambos requieren validación live para
  migrar sin regresión.
* **3 utilities/probes** no requieren LC por diseño.

## Plan de cierre F10b (cuando haya live testing budget)

### evidence_logger → LifecycleNode

* **Cambios**: file opens en `on_activate` (no `__init__`); cierre en
  `on_deactivate` + `on_cleanup`; subscriptions en `on_activate`.
* **Beneficio**: arrancar el evidence sólo cuando hay un goal pick
  activo (vía `lifecycle_msgs/Activate`), reduciendo ruido en sesiones
  de testing.
* **Riesgo**: bajo (observability, no critical path).
* **Validación**: ``ros2 lifecycle set /evidence_logger activate`` →
  verificar que se crea el directorio de sesión.

### planning_scene_sync → LifecycleNode

* **Cambios**: TF buffer + sub poses en `on_activate`; `on_deactivate`
  pausa la sincronización; `on_cleanup` libera buffers.
* **Beneficio**: poder pausar la sync MoveIt cuando no se está
  planificando (e.g. tras un release_objects).
* **Riesgo**: medio. Si la activación tarda, MoveIt pierde objetos de
  colisión durante la transición.
* **Validación**: ``ros2 launch ur5_bringup ur5_stack.launch.py
  launch_panel:=false`` → verificar planning scene poblada al
  activar.

### plan_to_pose_server → LifecycleNode

* **Pre-requisito**: F7b (split físico de los modos). Migrar el server
  monolítico a LC sin previo split aumenta el coste por las 3 ramas
  internas del action callback.
* **Riesgo**: alto. Los timeouts y la race condition documentada en
  `BUG_CONTROLLER_FEEDBACK_HANG.md` añaden complicación.

## Lint

`src/ur5_bringup/test/test_lifecycle_inventory.py` documenta y
valida la arquitectura: cualquier cambio futuro de Node ↔ LC
requerirá actualizar este doc + el test.
