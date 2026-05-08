# BUG F1.11: FJT goal_time_tolerance abort consistente en APPROACH

**Estado**: 🔴 ABIERTO (audit-v4 2026-05-08, sesión post-F1.10).
**Reproducibilidad**: 100% — cycles 1/2/3 v6 ABORTED tras ~400s cada uno.
**Bloqueante para**: F1.7-validate-live full-cycle (3 ciclos sin abort).
**NO bloqueante para**: F1.7 code-complete, F1.8 code-complete, F1.9 (clock OK), F1.10 (release OK).

## Resumen

Tras cerrar F1.9 (`/clock` QoS) y F1.10 (release_objects asienta 10/10 objetos), la
validación live v6 vuelve a fallar en APPROACH. Pero esta vez **NO** es OMPL FAILURE
ni INVALID_MOTION_PLAN — es el **`joint_trajectory_controller`** abortando con
`GOAL_TOLERANCE_VIOLATED` por exceder `goal_time_tolerance` (300 s) sobre el final
de la trayectoria.

## Evidencia (cycles v6 con stack F1.10)

| Cycle | Reason | Duración wall |
|---|---|---|
| 1/3 box_red | `approach:moveit_err:CONTROL_FAILED\|retry_failed` | 399 s |
| 2/3 box_blue | `approach:moveit_err:CONTROL_FAILED\|retry_failed` | 401 s |
| 3/3 box_green | `approach:moveit_err:CONTROL_FAILED\|retry_failed` | 405 s |

Patrón estable: ~400 s = `goal_time` (300 s) + first-attempt-timeout (60 s) +
retry sleep (8 s) + retry timeout (~30 s) → ~400 s.

## Logs relevantes (cycle 1)

```
[joint_trajectory_controller] Received new action goal
[joint_trajectory_controller] Accepted new action goal
[follow_joint_trajectory_controller_handle] joint_trajectory_controller started execution
[follow_joint_trajectory_controller_handle] Goal request accepted!

# ... 300+ segundos pasan ...

[joint_trajectory_controller] Aborted due to goal_time_tolerance exceeding by 300.002426 seconds
[follow_joint_trajectory_controller_handle] Controller 'joint_trajectory_controller' failed with error
  GOAL_TOLERANCE_VIOLATED: Aborted due to goal_time_tolerance exceeding by 300.002426 seconds
[move_group.moveit.moveit.ros.move_group.move_action] CONTROL_FAILED
[plan_to_pose_server] [PLAN_TO_POSE][MOVEIT_DIRECT] failed reason=moveit_err:CONTROL_FAILED|retry_failed
```

## Config actual (todas relevantes)

| Param | Valor | Archivo |
|---|---|---|
| `joint_trajectory_controller.constraints.goal_time` | 300.0 | `ur5_description/config/ur5_controllers.yaml:48` |
| `*.trajectory` (path) | 100.0 | `ur5_controllers.yaml` |
| `*.goal` | 0.30 | `ur5_controllers.yaml` |
| `trajectory_execution.allowed_execution_duration_scaling` | 100.0 | `ur5_moveit_bringup.launch.py:92` |
| `trajectory_execution.allowed_goal_duration_margin` | 120.0 | `ur5_moveit_bringup.launch.py:93` |

## Causa raíz hipotética

El robot **no converge a la pose objetivo dentro de 300 s** después de que la
trayectoria nominal termina. Posibles razones:

1. **Real-Time Factor (RTF) muy bajo en sim**: Gazebo Sim ejecuta < 0.5x real
   time. Trayectoria de 5 s sim_time = 10 s wall. 300 s `goal_time` × RTF =
   muy poco tiempo sim para que el controller PID converja.
2. **PID del controller mal sintonizado**: con vel_scaling 0.3 + tracking
   imperfecto, el robot oscila alrededor del goal sin converger dentro de
   `goal: 0.30` rad.
3. **JointTrajectoryController interpolation_method "splines"** con
   `interpolate_from_desired_state: false` puede causar tracking lento.
4. **Race entre /clock y joint_states**: el controller compara t_now con
   t_goal usando reloj, si `use_sim_time` no propaga consistente, calcula
   mal el tolerance.

## Reproducción

```bash
# Stack levantado tras F1.9 + F1.10:
PANEL_AUTO_RELEASE_DROP_OBJECTS=1 ros2 launch ur5_bringup ur5_stack.launch.py \
    headless:=true launch_panel:=false launch_moveit:=true camera_required:=0 \
    panel_managed:=0 launch_pick_orchestrator_lifecycle:=true

# Esperar a que pick_orchestrator_lifecycle esté active.

# Run v6 driver:
python3 /tmp/run_3_cycles_v6.py
```

Resultado esperado actual: 0/3 cycles → cada uno aborta APPROACH a ~400 s con
`CONTROL_FAILED|retry_failed`.

## Plan F1.11 (estimado 4-12 h, futura sesión)

### Diagnóstico

1. **Medir RTF actual**: `ros2 topic hz /clock` para ver rate; comparar con
   wall clock. Si RTF < 0.5, ese es el problema base.
2. **Inspeccionar joint_states durante un goal**: `ros2 topic echo /joint_states`
   en otra terminal mientras corre APPROACH. Ver si el robot SE MUEVE o se queda
   estacionado. Si no se mueve → controller PID muerto. Si oscila → tracking
   pobre.
3. **Compare t_now vs trajectory time_from_start**: log debug para ver el desfase.

### Fix candidatos

- **A** (más rápido): bajar `max_velocity_scaling_factor` y
  `max_acceleration_scaling_factor` de 0.3 → 0.1 en MoveIt. Trayectorias más
  lentas → más tiempo para que el controller siga.
- **B**: subir `goal_time` de 300 → 900 s (deshabilitar tolerancia
  efectivamente) para validar si converge eventualmente.
- **C**: cambiar `interpolation_method` de `splines` → `linear` en
  `ur5_controllers.yaml`. Linear interp es más simple y suele tener menor
  tracking error.
- **D**: tunear PIDs del joint_trajectory_controller (más Kp menos Kd) para
  convergencia más agresiva.
- **E**: revisar `gz_ros2_control` plugin update_rate vs sim physics rate —
  si rates están desincronizados, el controller no aplica comandos a tiempo.

### Validación

- Con cualquiera de los fixes: re-correr v6 driver y verificar que cycle 1
  completa APPROACH (no abort por goal_time_tolerance) en < 60 s.
- Tag de cierre: `cierre-fase-1.11-fjt-tracking-resolved-20260508`.

## NO es regresión

Este bug ha estado latente desde el principio (los workarounds de path/goal
tolerance + scaling 100 + duration_margin 120 evidencian que históricamente
el sim sufre tracking pobre). Solo que **antes** los ciclos no llegaban tan
lejos (caían en HOME_INITIAL o APPROACH OMPL FAILURE), por lo que este
goal_time_tolerance abort no era visible. Tras F1.7+F1.8+F1.9+F1.10, ahora
SÍ se llega al execute → este bug se vuelve el bloqueador único.

## Referencias

- [BUG_BRIDGE_PATH_TOLERANCE.md](BUG_BRIDGE_PATH_TOLERANCE.md) — bug bridge
  retry shipped en `f18b2c2` (still relevant — el retry firó pero ambos
  intentos hit goal_time_tolerance).
- [BUG_ORCHESTRATOR_APPROACH_PLANNING.md](BUG_ORCHESTRATOR_APPROACH_PLANNING.md) —
  F1.6/F1.7/F1.8 cierran las capas previas.
- [auditoria/audit_profesional_20260507.md](../auditoria/audit_profesional_20260507.md)
  Anexo F1.10 — sesión que descubre F1.11.
