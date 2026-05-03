# F10 — Roadmap para robot UR5 real / defensa académica

Estado actual: stack verificado en simulación (Gazebo Harmonic). Para
transicionar a hardware real y/o cerrar la defensa académica, este es
el plan de mínima viabilidad.

## Estado por dominio

| Dominio | Sim (hoy) | Real (pendiente) |
|---|---|---|
| Driver | `gz_ros2_control` + mock controllers | `ur_robot_driver` (Universal_Robots_ROS2_Driver) |
| Hardware interface | URDF `xacro` con `<ros2_control>` mock | `ur_calibration` ext + `ur_robot_driver` HardwareInterface |
| Safety limits | Joint limits en URDF | + `safety_limits.yaml` + e-stop topic + zonas restringidas |
| Gripper | OnRobot RG2 schemático prismático | OnRobot Compute Box ROS interface o dryver propio |
| Cámara | Gazebo image_raw | RealSense / cámara externa con calibración handeye |
| Sim time | `use_sim_time: true` global | toggle dinámico real_robot vs sim |
| Detección objeto | hardcoded names + GZ pose | percepción real (YOLOv8/SAM) |
| TF base→tool | publicado por Gazebo plugin | publicado por driver (real EE state) |

## Plan en 5 hitos

### Hito 1 — abstracción `RobotInterface` (offline)
- Nuevo paquete `ur5_robot_interface/` con interfaz pura (Protocol)
- Implementaciones: `SimRobotInterface` (actual stack) + `RealRobotInterface` (nuevo)
- Test contract que valida ambas exponen el mismo ABI
- **Riesgo**: bajo. **Tiempo**: 8 h.

### Hito 2 — instalar UR driver + ur_calibration
```bash
sudo apt install ros-jazzy-ur-robot-driver ros-jazzy-ur-calibration
```
- `ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=...`
- Generar `ur5_calibration.yaml` específico del robot
- Cargar en URDF como argumento

### Hito 3 — adaptar URDF/launch para dual-mode
- `ur5.urdf.xacro` con argumento `mode={sim,real}`
- `ur5_stack.launch.py` con argumento `robot_mode={sim,real}` que selecciona:
  - sim: lanza Gazebo + gz_ros2_control + plugin attach
  - real: lanza ur_robot_driver + RG2 driver + handeye TF
- **Riesgo**: medio. **Tiempo**: 16 h.

### Hito 4 — safety
- `safety_limits.yaml` con velocity/accel limits agresivos para tests
- Topic `/emergency_stop` (Bool) cableado a un Trigger del orchestrator
  que cancela cualquier action en curso
- Zonas restringidas en MoveIt collision matrix
- **Riesgo**: alto (toca hardware). **Tiempo**: 12 h.

### Hito 5 — percepción real
- Cambiar `panel_camera.py` para suscribirse a topic real (RealSense)
- Calibración handeye (extrinsics camera↔base_link) con MoveIt2 `handeye_calibration`
- Sustituir nombres hardcoded por inferencia visual (tfm_grasping ya tiene `grasp_inference`)
- **Riesgo**: medio. **Tiempo**: 24 h.

## Para defensa académica (sin hardware)

El proyecto YA es defendible al 100% en simulación. Para reforzar:

1. **Diagrama de microservicios** (PNG/SVG) basado en `agarre_ros2_ws/docs/architecture.md`.
2. **Demo grabada en vídeo** de 3 ciclos pick_demo con orchestrator vivo.
3. **Tabla de métricas** del último run E2E live (`scripts/generate_latency_table.sh` ya existe).
4. **Sección "trabajo futuro"** en la memoria académica que cite los 5 hitos de arriba.
5. **Comparativa cuantitativa** legacy vs orchestrator (LOC, tests, latencia).

## Métricas de cierre

| Indicador | Sim hoy | Sim post-F8 | Real hito 5 |
|---|---|---|---|
| Paquetes ROS 2 | 8 | 8 | 9 (+robot_interface) |
| LOC panel total | ~70 k | ~62 k | ~62 k |
| Tests offline | 117 | 130+ | 150+ |
| Lifecycle nodes | 8 | 8 | 8+drivers |
| Score audit | 87 % | 92 % | 95 %+ |
