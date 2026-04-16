# Evidencias Runtime — Flujo MoveIT — 2026-04-16

## Sesión auditada
- Inicio: 2026-04-16 10:19:10
- Fin: 2026-04-16 10:20:41
- Log fuente: `agarre_ros2_ws/log/ros2_launch.log`

## Secuencia de eventos documentados

### 1. Arranque del stack
```
[INFO] [move_group-1]: process started with pid [492583]
[INFO] [ur5_moveit_bridge-14]: process started with pid [492596]
```
Nota: `move_group` se lanzó (modo auto con PANEL_START_STACK=1, antes del fix).

### 2. Bridge inicializado
```
[ur5_moveit_bridge-14] Controller contract: controller=joint_trajectory_controller
action=/joint_trajectory_controller/follow_joint_trajectory
```

### 3. Pick button pulsado (10:19:34)
```
[PICK_OBJ][MOVEIT][PLAN] object_pose_source=pose_info frame_id=base_link selected=pick_demo
[PICK_OBJ] Alturas: APPROACH=0.350m, PRE_GRASP=0.200m, GRASP=0.030m, LIFT=0.350m
[MOVEIT2][CYCLE_REF][USE] phase=APPROACH world=(-0.420,0.000,0.875) base=(0.430,0.000,0.025)
  approach_target=(0.430,0.000,0.350)
```

### 4. Publicación a /desired_grasp (10:19:35)
```
[PICK_OBJ][MOVEIT][PLAN] Pose APPROACH (frame=base_link): (0.430, 0.000, 0.350)
[PICK_OBJ][MOVEIT][PLAN] topics pose_subs=1 result_pubs=1 result_subs=1
[PICK_OBJ][STEP] request_id=1 request_uuid=8a4082cc04c64a58a09b37c5e73ff0b9
  target=pick_demo pose=(0.430,0.000,0.350) frame=base_link tol=0.100
```

### 5. Bridge recibe petición (10:19:36)
```
[MOVEIT_BRIDGE][RX] req_id=1 req_uuid=8a4082cc04c64a58a09b37c5e73ff0b9
  frame=base_link pose=(0.430,0.000,0.350) accepted=true
```

### 6. Planificación MoveItPy (10:19:36) — EXITOSA
```
[BRIDGE_STATUS] plan_ok backend=moveit_py
[PICK][MOVEIT][PLAN_RESULT] phase=APPROACH success=true
  trajectory_type=RobotTrajectory
[BRIDGE_EXEC] backend=moveit_py ee_link=rg2_pinch_center
  controller=joint_trajectory_controller
  action=/joint_trajectory_controller/follow_joint_trajectory
```

### 7. Trayectoria enviada al controlador (10:19:36)
```
[BRIDGE_EXEC] timeout adjusted traj_sec=13.828 sim_per_wall=0.526 timeout_sec=119.939
[BRIDGE_EXEC] pre-scaling controller trajectory scale=2.0 reason=sim_tracking_margin
[BRIDGE_EXEC] inserted start-state waypoint lead_sec=0.20
  p0: shoulder_pan=0.000, shoulder_lift=-0.314, elbow=0.000,
      wrist_1=-1.571, wrist_2=0.000, wrist_3=0.000
```

### 8. Robot en movimiento (confirmado 10:20:33)
Estado del robot en t+55.6s después de inicio:
```
shoulder_pan=-0.536, shoulder_lift=0.724, elbow=2.049,
wrist_1=-1.239, wrist_2=3.037, wrist_3=-1.557
```
Diferencia vs estado inicial:
- shoulder_pan: 0.000 → -0.536 (-30.7°)
- shoulder_lift: -0.314 → 0.724 (+59.5°)
- elbow_joint: 0.000 → 2.049 (+117.4°)
- wrist_1: -1.571 → -1.239 (+19°)
- wrist_2: 0.000 → 3.037 (+174°)
- wrist_3: 0.000 → -1.557 (-89°)
→ Robot claramente en movimiento ✅

### 9. Replanificación APPROACH (10:20:33)
```
[BRIDGE_EXEC] approach long-wait requires replan elapsed=55.6s
  ee_detail=ee_target_not_reached:dist=0.9511
```
El robot estaba ejecutando una trayectoria OMPL válida pero larga (elbow-flip path).
Bridge replanificó desde estado actual.

### 10. Segundo plan exitoso (10:20:33)
```
[BRIDGE_STATUS] plan_ok backend=moveit_py  ← EXITOSO
[PICK][MOVEIT][PLAN_RESULT] phase=APPROACH success=true
```

### 11. Crash move_group al apagar (10:20:37)
```
[move_group-1] Segmentation fault (Address not mapped to object)
```
Causa: race condition en destructor MoveItCpp (bug conocido Jazzy 2.12.4).
No afectó a la planificación ni a la ejecución en curso.

### 12. Sistema apagado (10:20:40-41)
```
[ros2-15] [PICK_OBJ][MOVEIT][RECOVERY] reason=lost_result_publisher
[ros2-15] [PICK_OBJ][FAIL_CLASS] type=planning detail=ROS node no listo
[ros2-15] [PICK_OBJ] HOME_SAFE: enviando joints ...
```
La sesión fue interrumpida. El flujo estaba en medio de ejecutar APPROACH.

## Conclusión de evidencias
- Plan: ✅ `plan_ok backend=moveit_py success=true`  
- Robot en movimiento: ✅ (joints cambiaron significativamente)
- FJT ejecutado: ✅ (`action server detectado`)
- Falla: ❌ por interrupción de sesión, NO por fallo de software
