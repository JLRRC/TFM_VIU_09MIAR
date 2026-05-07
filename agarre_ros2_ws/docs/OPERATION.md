# Manual Operativo — TFM UR5+RG2 Pick & Place

Guía rápida para arrancar, ejecutar y diagnosticar el sistema en simulación.

## Requisitos

- Ubuntu 24.04 LTS
- ROS 2 Jazzy (`/opt/ros/jazzy`)
- Gazebo Harmonic (Sim moderno, NO Classic)
- MoveIt 2 (jazzy)
- gz_ros2_control
- Python 3.12

## Arranque del sistema

### Cold boot completo (todo desde cero)

```bash
cd /home/laboratorio/TFM
PANEL_FORCE_COLD_BOOT=1 ./lanzar_panelc2.sh
```

Lanza secuencialmente:
1. Gazebo Sim (`gz sim` headless o con render).
2. ros2_control + joint_trajectory_controller + gripper_controller.
3. MoveIt 2 (`move_group` + planning pipelines).
4. Bridges Gazebo↔ROS 2 (cámaras, joint_states, TF).
5. Servicios canónicos: `/orchestrator/attach`, `/orchestrator/detach`, `/gripper/open`, `/gripper/close`, `/orchestrator/resolve_object_pose_world`, `/orchestrator/plan_to_pose`, `tf_geometry_service`.
6. LifecycleNode `pick_orchestrator_lifecycle` con `auto_activate=True`.
7. Panel Qt (`panel_v2`).

Espera ~2 minutos hasta `/system_state=READY`.

### Arranque rápido (stack ya vivo)

```bash
./lanzar_panelc2.sh   # detecta si stack ya está READY y solo abre el panel
```

### Headless / offscreen (CI o SSH sin display)

```bash
PANEL_FORCE_OFFSCREEN=1 ./lanzar_panelv2.sh
```

## Ejecutar Pick Demo

### Vía panel Qt (canónico)

1. Esperar al estado READY del panel.
2. Seleccionar objeto del dropdown (default: `pick_demo`).
3. Pulsar botón **"Pick demo"**.
4. Observar Gazebo: el robot aproxima, baja las pinzas, agarra el objeto, lo levanta y lo transporta.

### Vía servicio ROS (programático)

```bash
ros2 service call /panel/select_object \
  ur5_panel_interfaces/srv/SelectObject "{name: pick_demo}"
ros2 service call /panel/pick_demo std_srvs/srv/Trigger
```

### Vía test E2E (validación automatizada)

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
PICK_E2E_LIVE=1 PICK_VALIDATE_CYCLES=3 \
  python3 -m pytest src/ur5_bringup/test/test_e2e_pick_cycles.py -v
```

## Camino canónico activo: LEGACY (`run_pick_demo`)

Por defecto el dispatcher (`pick_demo_dispatcher.py`) usa el camino **legacy**.

Razón: el camino orchestrator está bloqueado por un bug de timing
(`sim_time` vs `wall_time`) en la integración MoveIt↔`joint_trajectory_controller` en Gazebo Sim. Doc en `BUG_BRIDGE_PATH_TOLERANCE.md`.

Para forzar el camino orchestrator (cuando el bug esté cerrado):
```bash
PANEL_PICK_DEMO_USE_ORCHESTRATOR=1 ./lanzar_panelc2.sh
```

Para forzar el camino legacy (override):
```bash
USE_LEGACY_PICK_DEMO=1 ./lanzar_panelc2.sh
```

## Diagnóstico de fallos comunes

### Las pinzas no tocan el objeto

Verificar URDF↔SDF parity:
```bash
cd agarre_ros2_ws
python3 -m pytest src/ur5_tools/test/test_urdf_sdf_parity.py -v
```

Si el test falla con `rg2_mount_joint translation != (0,0,0)` o
`end_effector_frame_fixed_joint translation != (0,0,0)`, el SDF se ha
contaminado. Borrar `log/gz_models/` y relanzar para regenerar.

### Test E2E timeout en APPROACH

Verificar `sim_per_wall` en logs:
```bash
grep "sim_per_wall" agarre_ros2_ws/log/ros2_launch.log | head -5
```

Si `sim_per_wall < 0.5`, la simulación va lenta. Subir timeouts:
- `trajectory_execution.allowed_execution_duration_scaling` en `ur5_moveit_bringup.launch.py`.
- `goal_time` en `ur5_controllers.yaml`.

### MoveIt no encuentra plan

Verificar SRDF tip_link:
```bash
ros2 param get /move_group robot_description_semantic | grep tip_link
```

Debe ser `rg2_tcp` (no `tool0` ni `rg2_pinch_center`).

### Stack no termina de arrancar / quedó zombie

Hard reset:
```bash
pkill -9 -f "gz sim|move_group|panel_v2|pick_orchestrator|tf_geometry|object_pose|gripper_attach|world_tf|gz_pose|controller_manager|robot_state_publisher|parameter_bridge|ros2 launch"
sleep 5
rm -rf agarre_ros2_ws/log/gz_models agarre_ros2_ws/log/ros2_launch.log
PANEL_FORCE_COLD_BOOT=1 ./lanzar_panelc2.sh
```

## Tests offline (CI rápido)

```bash
cd agarre_ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
# Ejecutar paquete por paquete (preexistente: namespace conflict si se mezcla)
for pkg in tfm_orchestrator ur5_tools ur5_qt_panel ur5_bringup; do
  echo "=== $pkg ==="
  python3 -m pytest src/$pkg/test/ -q \
    --ignore-glob='*test_copyright.py' \
    --ignore-glob='*test_flake8.py' \
    --ignore-glob='*test_pep257.py'
done
```

Suite verde esperada (post sesión 2026-05-07):
- ur5_qt_panel: 1465/1465
- ur5_bringup: 75/75
- tfm_orchestrator: ~305 (los launch_testing requieren stack vivo)
- ur5_tools: ~501

## Estructura del workspace

```
agarre_ros2_ws/
├── models/ur5_rg2/        # SDF + meshes para Gazebo
├── src/
│   ├── ur5_description/   # URDF/Xacro + kinematics + SRDF refs
│   ├── ur5_bringup/       # launch files + runtime configs
│   ├── ur5_moveit_config/ # MoveIt config + SRDF + planning yaml
│   ├── ur5_tools/         # nodos backend (bridge, attach, gripper, tf_geometry)
│   ├── ur5_qt_panel/      # panel Qt + run_pick_demo (legacy)
│   ├── tfm_orchestrator/  # FSM canónico + action server PickPlace
│   ├── tfm_grasping/      # modelo de inferencia de grasps
│   └── ur5_panel_interfaces/  # 9 srv + 2 action IDL
├── docs/                  # docs canónicos (este archivo, audit, bugs, etc.)
├── auditoria/             # evidencias (gitignored)
└── worlds/                # SDF de mundos
```

## Tags relevantes

```bash
git tag --sort=-creatordate | head -10
```

- `objetivo-cumplido-pinzas-agarran-objeto-20260507` — agarre físico validado.
- `cierre-bloque-2-orchestrator-cableado-20260506` — orchestrator action listo.
- `cierre-bloque-1-pick-fisico-validado-20260506` — pick físico validado.
- `cierre-bloque-1-urdf-sdf-fix-20260506` — fix bug 167mm.

## Logs y telemetría

- Runtime: `agarre_ros2_ws/log/ros2_launch.log`.
- Evidencias por sesión: `agarre_ros2_ws/auditoria/pick_*_pytest_*.log`.
- Reports automatizados: `reports/`.

## Soporte

Para diagnosticar bugs nuevos, consultar:
- `docs/BUG_BRIDGE_PATH_TOLERANCE.md` — bug arquitectónico orchestrator.
- `docs/BUG_GRASP_DOWN_TCP_TRUNCATION.md` — bug histórico cerrado.
- `docs/AUDIT_20260506.md` — auditoría profesional completa.
- `CHANGELOG.md` — hitos del proyecto.
