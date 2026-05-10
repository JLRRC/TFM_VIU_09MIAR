# Sim vs Hardware Real — diferencias y plan F14

Este documento captura las diferencias entre el perfil de simulación
(Gazebo Harmonic + gz_ros2_control) y el perfil de hardware real
(UR_robot_driver + ur_dashboard_msgs). Está pensado para que la
transición a robot real sea trazable y reversible.

## Estado actual

* **Sim**: completamente funcional con Gazebo Sim 8 + ros2_control vía
  `gz_ros2_control::GazeboSimROS2ControlPlugin`.
* **Real**: NO probado todavía. F14 documenta el plan y proporciona el
  perfil base de controllers.

## Diferencias clave

| Aspecto | Sim (Gazebo) | Hardware real |
|---|---|---|
| Plugin de control | `gz_ros2_control/GazeboSimSystem` | `ur_robot_driver/URPositionHardwareInterface` |
| Driver | `gz_ros2_control` (binary) | `ur_robot_driver` (paquete `Universal_Robots_ROS2_Driver`) |
| Trayectoria | Comando interpolado en Gazebo | Stream RTDE a UR controller |
| Frecuencia | `update_rate=125 Hz` (sim) | 125 Hz (CB-series) o 500 Hz (e-series) |
| `interpolate_from_desired_state` | `true` (sim drift) | `false` (measured canónico) |
| `goal_time` | 300.0 s (sim_per_wall ≈ 0.58) | 30.0 s (margen razonable) |
| `stopped_velocity_tolerance` | 0.50 rad/s | 0.05 rad/s |
| Gripper | Prismatic joints en URDF + Gazebo physics | RG2 vía `onrobot_rg2_msgs` o I/O del UR |
| TF world→base | `world_tf_publisher` desde SDF | URDF estático + base del robot |
| Objetos | Gazebo entities con `<detachable_joint>` | No aplica (entorno real) |

## Cambios necesarios para hardware real

### Paquetes a instalar

```bash
sudo apt install ros-jazzy-ur-robot-driver ros-jazzy-ur-bringup
# OnRobot RG2: depende de la integración (TCP/IP del controlador o I/O)
```

### Archivos a añadir/modificar

1. **`src/ur5_bringup/launch/ur5_real.launch.py`** (NUEVO)
   * Lanza `ur_robot_driver` con `robot_ip` parametrizado.
   * Carga `ur5_controllers_real.yaml` (ya creado en F14).
   * Lanza `robot_state_publisher` con el URDF (sin `<gz_ros2_control>`).
   * NO lanza Gazebo, NO lanza bridges, NO lanza `gripper_attach_backend`.
   * Lanza `pick_orchestrator_lifecycle` igual que en sim.
   * Lanza el panel Qt con `panel_settings.use_sim_time:=false`.

2. **`src/ur5_description/urdf/ur5_real.urdf.xacro`** (NUEVO)
   * Mismo URDF que la versión sim PERO con
     `<plugin>ur_robot_driver/...</plugin>` en lugar de
     `gz_ros2_control/GazeboSimSystem`.
   * Parámetros: `robot_ip`, `script_filename`, `output_recipe_filename`,
     `input_recipe_filename` (estándares de `ur_robot_driver`).

3. **`src/ur5_description/config/ur5_controllers_real.yaml`** (CREADO en F14)
   * Perfil documentado con tolerancias razonables para hardware.

### Migración del gripper RG2

* **Sim**: prismatic joints comandados por `forward_command_controller`,
  attach lógico vía `gripper_attach_backend`.
* **Real**: tres opciones, en orden de simplicidad:
  1. Control por **I/O digital** del controlador UR (señal binaria
     open/close) — requiere relé.
  2. Control por **TCP/IP** al controlador OnRobot Compute Box vía
     `onrobot_rg2_driver` (no oficial, hay implementaciones community).
  3. Control por **URCap** desde el controller UR (más complejo,
     pero estándar OnRobot).

  Recomendado: opción (1) si el robot está cableado para I/O directo
  (chequear el conector tool flange); opción (2) si hay Compute Box.

### Servicios que NO aplican en real

* `release_objects_service` — los objetos físicos no se "respawnan",
  hay que recolocarlos manualmente. El servicio puede mantenerse para
  testing en sim.
* `gripper_attach_backend` — el attach es físico (las pinzas reales
  agarran). Mantener sólo para sim.
* `gz_pose_bridge`, `world_tf_publisher` — TF se publica por
  `robot_state_publisher` desde el URDF.

### Servicios que SE MANTIENEN en real

* `pick_orchestrator_lifecycle` — orquestación independiente del backend.
* `tf_geometry_service` — operaciones TF puras.
* `object_pose_resolver_service` — necesita un nuevo backend (e.g.
  AprilTag detection o cámara overhead) en lugar de Gazebo poses.
* `system_state_manager` — readiness de los nuevos componentes
  (ur_robot_driver, gripper RG2 driver, percepción).
* `evidence_logger` — sigue grabando JSONL/CSV.

## Plan de validación

1. **Smoke test sin movimiento** (~1 h):
   * `ros2 launch ur5_bringup ur5_real.launch.py robot_ip:=...`
   * Verificar `ros2 control list_controllers` muestra todos active.
   * Verificar `ros2 topic echo /joint_states` lee del robot real.

2. **Movimiento controlado** (~2 h):
   * Joint goal pequeño (joint2 -10°) vía `ros2 action send_goal
     /joint_trajectory_controller/follow_joint_trajectory`.
   * Verificar tolerancias del perfil real son alcanzables.

3. **Pick simple sin gripper** (~3 h):
   * `MoveIt` plan + execute desde HOME a APPROACH (no descender).
   * Validar `cached_ik` funciona en hardware real (caché en
     `log/cached_ik.bin` desde F12).

4. **Pick completo con gripper** (~4-8 h):
   * Integrar driver RG2 (opción 1 o 2).
   * Ejecutar `/pick_place` action con un objeto físico.

## Riesgos

* **Colisiones**: SRDF estricto (`ur5_strict.srdf`) NO está validado
  contra cinemática real con tolerancias de 1 mm. Probar primero en
  modo permisivo (`STRICT_SELF_COLLISION=0`) y migrar después.
* **Tolerancias**: el perfil F14 es conservador pero debe medirse el
  jitter real del UR para ajustar `stopped_velocity_tolerance`.
* **TCP frame**: `rg2_pinch_center` está calibrado contra el SDF de
  Gazebo (ver `models/ur5_rg2/model.sdf`). En real, requiere
  recalibración con regla / probador para verificar offset físico.

## Tag de rollback

Antes del primer arranque real:
```bash
git tag pre-real-robot-attempt-$(date -I)
```
