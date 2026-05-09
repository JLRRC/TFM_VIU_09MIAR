# Informe de reparación MoveIt — UR5 + RG2 (2026-04-28)

## 1. Resumen

MoveIt no podía operar porque los controladores `joint_trajectory_controller`
y `gripper_controller` quedaban en estado `unconfigured` durante el bootstrap.
El nodo `controller_bootstrap` invocaba `LoadController` y, antes de que el
`controller_manager` (gz_ros2_control) terminara la transición, llamaba a
`SwitchController` con `STRICT`, que abortaba con
`"Controller is in 'unconfigured' state. The controller needs to be configured
to be in 'inactive' state before it can be checked and activated"`. El sistema
quedaba en `STATE ERROR (controllers not active)`, MoveIt no podía ejecutar
trayectorias y el flujo de pick se bloqueaba.

Tras parchear `controller_bootstrap.py` para esperar las transiciones reales
de estado del controlador (`load → inactive → active`) antes de cada paso, el
stack arranca completo: `move_group` planifica, `ur5_moveit_bridge` queda
`listo`, controladores activos y `system_state_manager` reporta `STATE READY`.

## 2. Síntoma inicial

- `system_state_manager`: `STATE ERROR (drop: controllers not active:
  joint_trajectory_controller:inactive, gripper_controller:inactive)`.
- `controller_manager`: `Aborting, no controller is switched! (::STRICT
  switch)`.
- `controller_bootstrap`: `[CTRL] activate reported failed; ... state=active`
  (mensaje engañoso: el `_list_controllers()` devolvía valores cacheados o
  desactualizados durante la propia transición).
- MoveIt cargado correctamente (`Added FollowJointTrajectory controller for
  joint_trajectory_controller`) pero ningún controlador activo en runtime.

## 3. Causa raíz

Race condition en `ControllerBootstrap.run_once()`
(`src/ur5_tools/ur5_tools/controller_bootstrap.py`):

1. Llamada a `LoadController` → respuesta inmediata pero el controlador entra
   en `unconfigured` en el siguiente ciclo del controller_manager.
2. Llamada inmediata a `ConfigureController` → puede fallar porque aún no se
   ha publicado el estado.
3. Llamada inmediata a `SwitchController(STRICT)` → falla siempre porque el
   controlador no llegó a `inactive`.
4. El bootstrap interpretaba el `state=active` posterior como éxito sin
   reintentar la activación.

## 4. Evidencias

- `reports/moveit_repair_20260428_212504/logs/stack_120s.log` (antes del fix):
  - L466-467: `Could not activate ... is in 'unconfigured' state`,
    `Aborting, no controller is switched! (::STRICT switch)`.
  - L5500+: `system_state_manager: STATE ERROR ... controllers not active`
    repetido durante toda la ejecución.
- `reports/moveit_repair_20260428_212504/logs/stack_post_fix.log` (después):
  - L341-375: Bootstrap secuencial limpio, los tres controladores activos.
  - L383: `system_state_manager: STATE READY (Sistema listo)`.
  - L121: `UR5 MoveIt bridge listo.`
  - L59-76: `move_group` carga OMPL, `move_action`, `execute_trajectory_action`,
    `CartesianPathService`, `kinematics_service`, etc.

## 5. Cambios realizados

| Archivo | Cambio | Motivo | Backup |
|---------|--------|--------|--------|
| `src/ur5_tools/ur5_tools/controller_bootstrap.py` | Añadido helper `_wait_state()` que sondea `ListControllers` hasta que el controlador esté en el estado objetivo. Refactorizado `run_once()` para esperar las transiciones reales tras `LoadController`/`ConfigureController` y reintentar hasta 5 veces `SwitchController` confirmando con polling. Verificación final: re-listar controladores y confirmar todos `active`. | Eliminar la race condition entre el bootstrap y el `controller_manager` con `STRICT switch`. | `reports/moveit_repair_20260428_212504/backups/controller_bootstrap.py.bak` |

Diff completo: `reports/moveit_repair_20260428_212504/patches/controller_bootstrap.diff`.

No se modificaron SRDF, kinematics, moveit_controllers, joint_limits, URDF,
SDF ni launches: la configuración estática ya era correcta. Group
`manipulator`, base `base_link`, tip `rg2_tcp`, controlador
`joint_trajectory_controller` con `follow_joint_trajectory` coinciden entre
SRDF / `moveit_controllers.yaml` / `ur5_controllers.yaml` / bridge.

## 6. Pruebas ejecutadas

| Prueba | Comando | Resultado |
|--------|---------|-----------|
| Build inicial | `colcon build --symlink-install --packages-select ur5_bringup ur5_moveit_config ur5_tools ur5_qt_panel ur5_panel_interfaces ur5_description tfm_grasping` | OK (7 paquetes) |
| Stack inicial (reproducción del bug) | `ros2 launch ur5_bringup ur5_stack.launch.py launch_panel:=false launch_rviz:=false moveit_mode:=move_group launch_moveit:=true` | FAIL: controllers `unconfigured`, `STATE ERROR` repetido |
| Build post-fix | `colcon build --symlink-install --packages-select ur5_tools` | OK |
| Stack post-fix | `ros2 launch ur5_bringup ur5_stack.launch.py launch_panel:=false launch_rviz:=false moveit_mode:=move_group launch_moveit:=true camera_required:=0` | OK: 3 controladores activos, `STATE READY`, bridge `listo`, MoveIt completo |

Logs íntegros en `reports/moveit_repair_20260428_212504/logs/`.

## 7. Estado final (checklist)

- [OK] `colcon build` sin errores.
- [OK] `move_group` arranca (`Loaded robot model`, OMPL, todas las
  adapters, `move_action`, `execute_trajectory_action`).
- [OK] `robot_description` cargado (URDF expandido por
  `MoveItConfigsBuilder`).
- [OK] `robot_description_semantic` cargado (`ur5.srdf`).
- [OK] Planning group `manipulator` válido (chain `base_link`→`rg2_tcp`).
- [OK] Kinematics plugin `cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin`
  activo (`Joint weights for group 'manipulator': 1 1 1 1 1 1`).
- [OK] `joint_state_broadcaster`, `joint_trajectory_controller`,
  `gripper_controller` activos (verificado vía bootstrap + `STATE READY`).
- [OK] MoveIt controller (`joint_trajectory_controller`) coincide con
  ros2_control.
- [OK] Action `FollowJointTrajectory` disponible (`Action status changes will
  be monitored at 20.00 Hz`).
- [OK] `ur5_moveit_bridge` arranca (`UR5 MoveIt bridge listo`,
  `MoveItPy backend seleccionado`).
- [OK] Bridge se comunica con MoveIt (`Controller contract:
  controller=joint_trajectory_controller
  action=/joint_trajectory_controller/follow_joint_trajectory`).
- [OK] No hay errores críticos de TF `base_link/tool0` (sólo avisos al inicio
  durante warmup, normales).

## 8. Pendientes

- Validar end-to-end un ciclo de pick completo con panel: este informe
  reproduce hasta `STATE READY`; la prueba de botón pick queda fuera del
  alcance del fix mínimo solicitado, pero queda desbloqueada al estar los
  controladores y MoveIt operativos.
- Mensaje `No 3D sensor plugin(s) defined for octomap updates`: no es
  bloqueante (no se usa octomap en este pipeline); puede silenciarse desactivando
  `OccupancyMapMonitor` en `ompl_planning.yaml` si se desea.
- Aviso `WARNING:root:Cannot infer URDF/SRDF from
  install/ur5_moveit_config/share/ur5_moveit_config`: cosmético de
  `moveit_configs_utils`; el launch carga las rutas explícitas correctas.

## 9. Cómo reproducir validación

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ur5_tools
source install/setup.bash
export WS_DIR=$PWD
timeout 90s ros2 launch ur5_bringup ur5_stack.launch.py \
  launch_panel:=false launch_rviz:=false \
  moveit_mode:=move_group launch_moveit:=true camera_required:=0 \
  > /tmp/stack.log 2>&1
grep -E "STATE READY|MoveIt bridge listo|\[CTRL\] .* activo" /tmp/stack.log
```

Salida esperada incluye `STATE READY (Sistema listo)`, `UR5 MoveIt bridge
listo` y `[CTRL] joint_state_broadcaster|joint_trajectory_controller|gripper_controller activo`.

## 10. Qué no se tocó

- SRDF (`ur5.srdf`, `ur5_strict.srdf`).
- `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`,
  `ompl_planning.yaml`, `planning_scene_monitor_parameters.yaml`.
- URDF/Xacro (`ur5.urdf.xacro`).
- SDF (`models/ur5_rg2/model.sdf`, `worlds/ur5_mesa_objetos.sdf`).
- Launches (`ur5_stack.launch.py`, `ur5_moveit_bringup.launch.py`,
  `ur5_ros2_control.launch.py`, `ur5_rsp.launch.py`).
- `ur5_moveit_bridge.py` y subpaquete `moveit_bridge/`.
- `system_state_manager.py`, `gripper_attach_backend.py`,
  `planning_scene_sync.py`.
- Configuración del panel (`ur5_qt_panel`).
