# Changelog — tuning crítico

Registro de incidentes y ajustes de tunables (timeouts, tolerancias, escalas)
que históricamente acumulaban comentarios fechados en YAML/launch. F1 audit
2026-05-10: extraídos a este archivo para mantener limpio el código de
configuración. Cada entrada referencia el commit/tag de origen y el rationale.

## 2026-05-08 (F1.9 audit-v4)

* `ur5_stack.launch.py` — `controller_bootstrap.clock_timeout_sec`: 12 → 60 s.
  En sim con muchos plugins (10+ detachable_joints), `gz sim` puede tardar
  > 12 s en publicar el primer `/clock`. Logging periódico cada 5 s
  documenta el progreso.

## 2026-05-08 (F1.11 audit-v4)

* `ur5_controllers.yaml` — `joint_trajectory_controller.interpolate_from_desired_state`:
  false → true. El controller interpola desde el desired state (último
  comando) en lugar del measured state. Evita discrepancias de tracking
  que impiden convergencia al goal en sim. Test directo FJT funciona OK; el
  problema era el feedback loop con trajectory desde MoveIt.
* `ur5_controllers.yaml` — `stopped_velocity_tolerance`: 0.10 → 0.50 rad/s.
  FJT abortaba con goal_time_tolerance porque el robot llegaba a posición
  pero oscilaba en velocidad > 0.10 rad/s, nunca declaraba "stopped".
  0.50 rad/s ≈ 28°/s es lenient pero detecta convergencia en sim.

## 2026-05-07 (rondas 11-26 tuning MoveIt)

* `ur5_moveit_bringup.launch.py` — `trajectory_execution.allowed_execution_duration_scaling`:
  1.2 → 4.0 → 10.0 → 30.0 → **100.0** (final). max_velocity_scaling_factor
  pasa a 0.30 (en `plan_to_pose_moveit_direct.py`) para que la trayectoria
  sea más lenta Y el upper bound más holgado.
* Tolerancias subidas para Gazebo Sim (sim_per_wall ≈ 0.58 produce drift
  severo entre trajectory_duration planeada y real):
  * `goal_time`: 300.0 (permite sim a 1/5 de wall sin abortar)
  * por joint `goal`: 0.30 rad (17°) — sim physics jitter al goal
  * por joint `trajectory`: 100.0 (efectivamente infinito) — path
    tolerance descartada

## 2026-05-06 (validación live)

* `runtime_defaults.yaml` — `PANEL_PICK_DEMO_APPROACH_COARSE_GATE_*_TOL_M`:
  0.012 → 0.020 m. La FK del panel (DH UR5 estándar, D[5]=0.0823) tiene un
  sesgo determinista de ~13 mm respecto al TF publicado por el RSP
  (URDF/ur_macro). Las anteriores 0.012 m fallaban el gate por ~1 mm de
  forma sistemática. 0.020 m da margen para el sesgo + ruido sin
  comprometer el agarre (RG2 abre 0.055 m, objeto 0.025 m alto).

## 2026-05-04 (fixes pick_demo)

* `runtime_defaults.yaml` — `PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN`:
  habilitado → 0. MoveIt Cartesian devuelve `fraction=0` en este setup;
  uso del descenso segmentado IK directo.
* `runtime_defaults.yaml` — `PANEL_PICK_DEMO_TRANSPORT_RUNTIME_*`:
  stall=8 s + min_progress=8 mm cortaba en los últimos ~28 mm a la cesta.
  15 s + 3 mm permite que el FollowJointTrajectory termine el goal.
* `runtime_defaults.yaml` — `PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC` y
  `PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM`: 3.0 s + 10 mm permitía confirmar
  "cerrado" con la pinza casi abierta. 8.0 s + 60 mm exige cierre real.

## 2026-04-08

* `panel_pick_demo` — `_DIRECTO_GRASP_Z`: 0.0 (commit 9b58910). Anteriormente
  `GRIPPER_TCP_Z_OFFSET=0.05` se aplicaba a `rg2_pinch_center` causando
  offset duplicado.
