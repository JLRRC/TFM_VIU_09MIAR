# Diff contra documento anterior

```diff
--- report/BaseDeConocimiento/2026-05-10_base_conocimiento_tecnica_TFM.md
+++ 2026-05-10_base_conocimiento_tecnica_TFM.md
@@ -1825,6 +1825,234 @@
 
 ---
 
+## 14b. Arquitectura de Microservicios y LifecycleNodes
+
+Inventario auto-descubierto del workspace. Esta sección refleja la migración a una arquitectura de nodos lifecycle más allá del panel monolítico (Ruta B + sprints F1-F19, B-iter1..14, F1.14-F1.18). Los datos provienen del árbol de fuentes en el momento de la generación.
+
+### 14b.1 LifecycleNodes detectados
+
+| Clase | Fichero | Descripción / docstring |
+|---|---|---|
+| `PickOrchestratorLifecycleNode` | `agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pick_orchestrator_lifecycle_node.py` | Action server `/pick_place` con ciclo de vida managed. |
+| `GripperAttachBackend` | `agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py` | Backend que mantiene los objetos adheridos siguiendo fisicamente el TCP. |
+| `GzPoseBridge` | `agarre_ros2_ws/src/ur5_tools/ur5_tools/gz_pose_bridge.py` | Read gz pose/info and publish a TFMessage with named frames. |
+| `ObjectPoseResolverService` | `agarre_ros2_ws/src/ur5_tools/ur5_tools/object_pose_resolver_service.py` | Server `/orchestrator/resolve_object_pose_world` con ciclo de vida managed. |
+| `ReleaseObjectsService` | `agarre_ros2_ws/src/ur5_tools/ur5_tools/release_objects_service.py` | Servicio que orquesta el respawn físico de objetos en Gazebo. |
+| `SystemStateManager` | `agarre_ros2_ws/src/ur5_tools/ur5_tools/system_state_manager.py` | Sigue el estado global del sistema y publica /system_state + /system_diag. |
+| `TfGeometryService` | `agarre_ros2_ws/src/ur5_tools/ur5_tools/tf_geometry_service.py` | LifecycleNode que aloja los servicios geométricos TF. |
+| `WorldTfPublisher` | `agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py` | Publish world->base_link from /world/<world>/pose/info. |
+
+### 14b.2 Paquete `tfm_orchestrator` (orquestador del pick)
+
+- Ruta: `agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator`
+- Módulos puros y nodos:
+  - `cartesian_segments.py`
+  - `errors.py`
+  - `gripper_monitor.py`
+  - `home_initial.py`
+  - `initial_snapshot.py`
+  - `lifecycle_helpers.py`
+  - `phase_dispatch.py`
+  - `phase_progress.py`
+  - `phase_timings.py`
+  - `pick_fsm.py`
+  - `pick_gates.py`
+  - `pick_orchestrator_lifecycle_node.py`
+  - `pick_orchestrator_node.py`
+  - `pose_consistency.py`
+  - `preflight.py`
+  - `retry.py`
+  - `service_clients.py`
+
+### 14b.3 Submódulos `pick_demo/` extraídos del panel
+
+| Módulo | Propósito (primera línea de docstring/comentario) |
+|---|---|
+| `align_grasp.py` | Align demo grasp direct extraído del closure run_pick_demo.worker. |
+| `audit_emit.py` | ``audit_emit`` — emisor pure-ish de los logs estructurados del Pick Demo. |
+| `debug_markers.py` | Construcción y publicación de MarkerArray para visualización RViz del flujo Direct. |
+| `decision_helpers.py` | F3-step40 (2026-05-08) — Decision string classifiers (puros). |
+| `geometry.py` | Funciones geométricas puras del flujo pick_demo. |
+| `grasp_down.py` | Grasp down conservative phase extraído del closure run_pick_demo.worker. |
+| `internal_helpers.py` | Helpers internos del pick demo: resolucion de objeto + validacion transporte. |
+| `joint_step.py` | Joint-space step executor extraído del closure run_pick_demo.worker. |
+| `marker_helpers.py` | Helpers puros para construir Marker (RViz) del pick_demo. |
+| `metrics.py` | Funciones puras de cálculo de métricas para el flujo pick_demo. |
+| `phase_checks.py` | Lógica pura del phase check de APPROACH_COARSE. |
+| `pure_helpers.py` | Funciones puras extraídas del closure ``run_pick_demo``. |
+| `seed_metrics.py` | F3-step40 (2026-05-08) — IK seed deviation metrics (puras). |
+| `transport_replan.py` | Transport replan helper extraído del closure _move_tcp_direct. |
+| `wait_gripper_target.py` | Wait loop para confirmar gripper abierto/cerrado. |
+| `wait_runtime_tcp_stable.py` | Wait loop hasta que el TCP runtime se estabilice cerca de un target. |
+
+## 14c. Interfaces ROS 2 (services y actions)
+
+Contratos públicos del paquete `ur5_panel_interfaces`. Estas interfaces son las fronteras estables entre orquestador, panel y nodos de soporte; conviene documentarlas explícitamente porque son el equivalente al "API" del sistema.
+
+### 14c.1 Actions
+
+| Action | Fichero | Cabecera (3 primeras líneas no-comentadas) |
+|---|---|---|
+| `PickPlace.action` | `agarre_ros2_ws/src/ur5_panel_interfaces/action/PickPlace.action` | string object_name ; geometry_msgs/Point drop_xyz_world ; geometry_msgs/Pose object_pose_world_hint |
+| `PlanToPose.action` | `agarre_ros2_ws/src/ur5_panel_interfaces/action/PlanToPose.action` | geometry_msgs/Pose target_pose_base ; string ee_frame                # rg2_pinch_center | rg2_tcp | tool0 ; bool cartesian                  # true: linear cartesian, false: joint plan |
+
+### 14c.2 Services
+
+| Service | Fichero | Cabecera (3 primeras líneas no-comentadas) |
+|---|---|---|
+| `Attach.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/Attach.srv` | string object_name ; --- ; bool success |
+| `Close.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/Close.srv` | --- ; bool success ; string message |
+| `ComputeApproachPose.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/ComputeApproachPose.srv` | geometry_msgs/Pose object_pose_base ; float64 z_clearance_m ; --- ; geometry_msgs/Pose approach_pose_base |
+| `Detach.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/Detach.srv` | string object_name ; --- ; bool success |
+| `Open.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/Open.srv` | --- ; bool success ; string message |
+| `ResolveObjectPoseWorld.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/ResolveObjectPoseWorld.srv` | string object_name ; --- ; geometry_msgs/Pose pose_world |
+| `SelectObject.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/SelectObject.srv` | string name ; --- ; bool success |
+| `SetWidth.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/SetWidth.srv` | float64 width_m ; --- ; bool success |
+| `WorldToBase.srv` | `agarre_ros2_ws/src/ur5_panel_interfaces/srv/WorldToBase.srv` | geometry_msgs/Point world_xyz ; --- ; geometry_msgs/Point base_xyz |
+
+## 14d. Snapshot del runtime configurable (runtime_defaults.yaml)
+
+`runtime_defaults.yaml` es la fuente de verdad declarativa de los tunables del stack. El launcher hace merge: env > yaml > literal codificado. Este snapshot del fichero al momento de la generación se incluye íntegro para que el documento se pueda auditar sin acceso al checkout.
+
+```yaml
+# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/config/runtime_defaults.yaml
+# Source of truth para todos los tunables del stack UR5+RG2 (F2 del refactor TFM).
+# Cada clave aquí es el nombre EXACTO de la env var equivalente. Override en runtime:
+#   1. Env var con ese nombre (gana sobre YAML).
+#   2. Si no, YAML (este archivo).
+#   3. Si no, literal default codificado en el launch (último recurso).
+#
+# Mantén esto sincronizado con:
+#   - agarre_ros2_ws/src/ur5_bringup/launch/launch_helpers.py (PANEL_ENV_DEFAULTS)
+#   - agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py (env vars directos)
+
+# --- Sistema y paths -----------------------------------------------------------
+WS_DIR: ~/TFM/agarre_ros2_ws
+GZ_RENDER_ENGINE: ogre2
+
+# --- Panel: comportamiento básico ---------------------------------------------
+PANEL_CAMERA_REQUIRED: "1"
+PANEL_KEEP_CAMERAS: "0"
+PANEL_PYTHON: ""
+
+# --- system_state_manager: tolerancias de geometría y arranque -----------------
+SYSTEM_STATE_GEOMETRY_OFFSET_TOL_M: "0.002"
+SYSTEM_STATE_GEOMETRY_PAIR_TOL_M: "0.001"
+SYSTEM_STATE_STARTUP_TIMEOUT_SEC: "15.0"
+
+# --- attach_backend: timeouts y modo follow ------------------------------------
+ATTACH_BACKEND_GZ_SERVICE_TIMEOUT_MS: "2000"
+ATTACH_BACKEND_GZ_CMD_TIMEOUT_SEC: "3.0"
+ATTACH_BACKEND_MODE: follow_tcp
+ATTACH_BACKEND_MAX_POSE_AGE_SEC: "1.5"
+ATTACH_BACKEND_FOLLOW_RATE_HZ: "20.0"
+ATTACH_BACKEND_FOLLOW_BREAK_DIST_M: "0.18"
+ATTACH_BACKEND_MAX_DIST_M: "0.08"
+STRICT_PHYSICS_MODE: "false"
+
+# --- moveit_bridge: timeouts y scales ------------------------------------------
+PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC: "150.0"
+PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC: "180.0"
+PANEL_MOVEIT_BRIDGE_STALE_REQUEST_TTL_SEC: "120.0"
+PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC: "6.0"
+PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC: "2.5"
+PANEL_MOVEIT_BRIDGE_FORCE_FJT_DIRECT: "0"
+PANEL_MOVEIT_BRIDGE_UNWRAP_CONTINUOUS_JOINTS: "0"
+PANEL_MOVEIT_BRIDGE_REQUIRE_REQUEST_ID: "1"
+PANEL_MOVEIT_BRIDGE_DROP_PENDING_ON_TAGGED: "1"
+PANEL_MOVEIT_BRIDGE_DRY_RUN: "0"
+PANEL_MOVEIT_BRIDGE_PATH_CONSTRAINT_TOL_RAD: "0.35"
+PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TIME_TOL_SEC: "45.0"
+PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD: "4.0"
+PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD: "0.20"
+PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE: "0.30"
+PANEL_MOVEIT_BRIDGE_ACCEL_SCALE: "0.30"
+
+# --- Debug / audit root --------------------------------------------------------
+PANEL_DIRECT_DEBUG_ROOT: /home/laboratorio/TFM/historico
+
+# --- pick_demo: GRASP_DOWN segmented IK descent --------------------------------
+PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M: "0.005"
+PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN: "0"  # Fix 2026-05-04: cartesian devuelve fraction=0 en este setup
+PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT: "0.035"
+PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL: "0.200"
+PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M: "0.005"
+PANEL_PICK_DEMO_GRASP_DOWN_FORCE_INHERIT_XY: "1"
+PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK: "1"
+PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_MAX_DEV_RAD: "0.28"
+PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_SUM_DEV_RAD: "0.60"
+PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M: "0.015"
+PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M: "0.008"
+PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M: "0.012"
+PANEL_PICK_DEMO_GRASP_DOWN_MAX_ATTEMPTS: "4"
+PANEL_PICK_DEMO_GRASP_DOWN_UTIL_XY_TOL_M: "0.015"
+
+# --- pick_demo: transport runtime no_progress tuning (Fix 2026-05-04) ----------
+# Con stall=8s y min_progress=8mm el detector cortaba en los últimos ~28mm a
+# la cesta. 15s + 3mm permite que el FollowJointTrajectory termine el goal.
+PANEL_PICK_DEMO_TRANSPORT_RUNTIME_STALL_TIMEOUT_SEC: "15.0"
+PANEL_PICK_DEMO_TRANSPORT_RUNTIME_MIN_PROGRESS_M: "0.003"
+
+# --- pick_demo: ATTACH gate tolerances -----------------------------------------
+PANEL_PICK_DEMO_ATTACH_XY_TOL_M: "0.020"
+PANEL_PICK_DEMO_ATTACH_Z_TOL_M: "0.010"
+PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M: "0.040"
+PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M: "0.012"
+PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC: "0.35"
+PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES: "5"
+PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M: "0.020"
+
+# --- pick_demo: Gripper close confirmation -------------------------------------
+PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M: "0.020"
+# Fix 2026-05-04 (bug falso positivo cierre): 3.0s+10mm permitía confirmar
+# "cerrado" con la pinza casi abierta. 8.0s+60mm exige cierre real.
+PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC: "8.0"
+PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM: "0.060"
+PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M: "0.12"
+
+# --- pick_object: TF stability gate for GRASP_DOWN post-motion check -----------
+PANEL_PICK_OBJECT_GRASP_TF_STABLE_TOL_M: "0.045"
+PANEL_PICK_OBJECT_GRASP_TF_STABLE_SAMPLES: "5"
+PANEL_PICK_OBJECT_GRASP_TF_STABLE_MIN_OK: "4"
+
+# --- pick_demo: Approach coarse gate -------------------------------------------
+PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M: "0.035"
+# Tolerancias subidas 2026-05-06 tras validación live: la FK del panel (DH UR5
+# estándar, D[5]=0.0823) tiene un sesgo determinista de ~13mm respecto al TF
+# publicado por el RSP (URDF/ur_macro). Las anteriores 0.012m fallaban el gate
+# de forma sistemática por ~1mm. 0.020m da margen para el sesgo + ruido sin
+# comprometer el agarre (RG2 abre 0.055m, objeto 0.025m alto).
+PANEL_PICK_DEMO_APPROACH_COARSE_GATE_XY_TOL_M: "0.020"
+PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M: "0.020"
+
+# --- pick_demo: PRE_CLOSE tolerances -------------------------------------------
+PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M: "0.020"
+PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M: "0.010"
+PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES: "2"
+
+# --- pick_demo: CLOSE tolerances -----------------------------------------------
+PANEL_PICK_DEMO_CLOSE_XY_TOL_M: "0.008"
+PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M: "0.008"
+
+# --- pick_demo: GRASP_ALIGN_IK convergence -------------------------------------
+PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL: "0.200"
+PANEL_PICK_DEMO_ALIGN_IK_SEED_WEIGHT: "0.80"
+PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M: "0.020"
+PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M: "0.010"
+PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M: "0.008"
+
+# --- pick_demo: Pose source freshness ------------------------------------------
+PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC: "0.400"
+PANEL_PICK_DEMO_POSE_SOURCE_TOL_M: "0.006"
+PANEL_PICK_DEMO_PHASE_JUMP_TOL_M: "0.010"
+
+# --- pick_demo: Direct IK runtime settle ---------------------------------------
+PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC: "2.5"
+PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M: "0.003"
+PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SAMPLES: "3"
+PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_POLL_SEC: "0.10"
+```
+
 ## 15. Apéndices
 
 ### 15.1 Evidencia cruzada preservada del documento vigente
```
