# Baseline pre-auditoría 2026-05-10

Tag: `audit-pre-2026-05-10-fXX-start`
HEAD: cde45a0

## Resultado `colcon test`

| Paquete | Pasados | Fallos | Skipped | Notas |
|---|---|---|---|---|
| ur5_description | OK | 0 | 0 | linters OK |
| ur5_panel_interfaces | OK | 0 | 0 | IDL OK |
| ur5_moveit_config | OK | 0 | 0 | config OK |
| tfm_grasping | 80 | 0 | 0 | offline OK |
| tfm_orchestrator | 424 | 3 | 2 | ver abajo |
| ur5_tools | 725 | 4 | 24 | ver abajo |

## Fallos pre-existentes (no causados por la auditoría)

### tfm_orchestrator
1. `test_mypy_strict_baseline` — `panel_ros_handlers.py:213: Missing type arguments for generic type "dict"` → será resuelto en F1.
2. `test_pick_orchestrator_lifecycle_launch::test_action_pick_place_published_after_configure` — timeout de configure (requiere ROS vivo).
3. `test_pick_orchestrator_lifecycle_launch::test_proc_terminates_cleanly` — `TypeError` (regresión de framework `launch_testing`).

### ur5_tools
1. `test_pep257` — ~40 errores estilo en `plan_to_pose_server.py`, `release_objects_service.py`, `system_health_helpers.py`, `tf_batch_lookups.py`, `tf_geometry_logic.py`, `trajectory_executor_contract.py` → resueltos en F1.
2. `test_object_pose_resolver_launch::test_service_returns_pose_after_publishing_tfmessage` — sub no llega (requiere ROS vivo).
3. `test_object_pose_resolver_launch::test_proc_terminates_cleanly` — mismo `TypeError` framework.
