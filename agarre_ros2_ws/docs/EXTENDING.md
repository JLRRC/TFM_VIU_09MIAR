# Extender el TFM a otro robot / gripper / objetos

Este documento explica los puntos canónicos de extensión del stack y cómo
aplicarlos sin romper el contrato actual UR5+RG2. Asume familiaridad con
ROS 2, MoveIt 2, Xacro/URDF, SDF y los conceptos de FSM/LifecycleNode del
orchestrator.

---

## TL;DR — los 4 frames de extensión

| Quieres cambiar | Toca esto | Tests obligatorios tras cambio |
|---|---|---|
| **Robot manipulador** | `ur5_description/urdf/*.urdf.xacro` + `ur5_moveit_config/config/*.srdf` + `ur5_bringup/launch/runtime_nodes_factory.py` | T22, T19 IDL hash, mypy strict, launch_testing |
| **Gripper** | `models/<name>/model.sdf` + `gripper_geometry.py::TCP_FRAMES` + `gripper_attach_backend.py` | T22 (parity URDF↔SDF), T25 attach_distance gate |
| **Mundo / objetos** | `worlds/<name>.sdf` + `release_objects_service.py::DEFAULT_OBJECTS` + `evidence_logger.py::_DEFAULT_GRIPPER_OBJECTS` | T11 planning_scene_consistency, T8b object_pose_resolver_launch |
| **Estrategia de agarre** | `tfm_grasping/model.py` + `tfm_grasping/grasp_module.py` | T34 grasp_module API, T10 grasp_inference smoke |

---

## 1. Cambiar de UR5 a otro brazo (ej. UR5e, UR10, Franka)

### Pasos

1. **Reemplazar URDF/Xacro** en `agarre_ros2_ws/src/ur5_description/`:
   - Mantener el frame `base_link` como pivote del manipulador.
   - El TCP del gripper debe ser un frame fijo bajo `tool0` (o equivalente).

2. **Actualizar SRDF** en `agarre_ros2_ws/src/ur5_moveit_config/config/ur5.srdf`:
   - Renombrar el `group` `manipulator` si quieres semántica distinta —
     pero el orchestrator referencia "manipulator" en
     `phase_dispatch.py::build_move_group_goal`. Si lo renombras, también
     ajusta la constante.
   - Actualizar `end_effector` con el nuevo TCP.
   - Ajustar la collision matrix con `moveit_setup_assistant`.

3. **Actualizar `kinematics.yaml`**: cambiar el solver y los grupos.

4. **Actualizar `joint_limits.yaml`** con los límites del nuevo robot.

5. **Validar parity URDF↔SDF**: el test `T22 test_urdf_sdf_parity` verifica
   que `rg2_pinch_center` (o el frame del gripper) coincide ≤ 5 mm
   entre URDF (lo que ve MoveIt) y SDF (lo que simula Gazebo).
   Si cambias el robot, este test te dirá inmediatamente si la geometría
   está rota.

6. **Verificar offset world→base_link**: en `phase_dispatch.py` hay
   constantes literales `_BASE_LINK_IN_WORLD = (-0.85, 0.0, 0.850)` —
   ajustar a la pose del nuevo robot en el world SDF.

### Tests críticos

```bash
colcon test --packages-select ur5_description ur5_moveit_config ur5_tools
pytest src/ur5_tools/test/test_urdf_sdf_parity.py
```

---

## 2. Cambiar de RG2 a otro gripper

### Pasos

1. **Modelo SDF**: añadir `models/<gripper_name>/model.sdf` con:
   - Detachable joint plugin (gz-sim-detachable-joint-system) para attach lógico.
   - Visual + collision aproximada (la real puede tener > 1000 triángulos —
     usa `box`/`cylinder` como collision para no saturar OMPL).

2. **TCP frame canonical**: el orchestrator asume que el TCP se llama
   `rg2_pinch_center` (o el frame que aparezca como tip en SRDF
   `end_effector` group).
   - Actualizar `gripper_geometry.py::TCP_FRAMES` con la lista de TCPs
     válidos del nuevo gripper.
   - Actualizar `phase_dispatch.py::ee_frame` si cambia el nombre.

3. **Servicios de gripper**: el orchestrator habla con 4 servicios:
   `/orchestrator/attach`, `/orchestrator/detach`, `/gripper/open`,
   `/gripper/close`. El nuevo gripper debe exponer estos endpoints
   (vía `gripper_attach_backend.py` o un node propio).
   - Para grippers continuos (Schunk, Robotiq), `Open.srv` y `Close.srv`
     ya soportan `width` parameter (ver IDL).
   - Para grippers binarios (Suction), implementar como wrapper.

4. **Geometría del attach**: el gate `attach_distance:0.250m` (default
   tras F1.6) acepta TCP↔objeto ≤ 25 cm. Para grippers de tip más largo
   o más corto, ajustar `DEFAULT_MAX_ATTACH_DIST_M` en
   `pick_gates.py`.

### Tests críticos

```bash
pytest src/tfm_orchestrator/test/test_pick_gates.py
pytest src/ur5_tools/test/test_gripper_geometry.py
pytest src/ur5_tools/test/test_gripper_attach_backend_services.py
```

---

## 3. Cambiar el mundo / objetos pickeables

### Pasos

1. **World SDF**: editar `worlds/<name>.sdf`:
   - Mesa, suelo, iluminación.
   - Modelos pickeables con un `<model name="...">` y `<plugin
     filename="gz-sim-detachable-joint-system">` para soportar attach.
   - Pose inicial suspendida (z alto) para drop on-demand vía
     `/release_objects` service.

2. **Lista de objetos esperados**: actualizar:
   - `release_objects_service.py::DEFAULT_OBJECTS` — el set que el
     servicio `/release_objects` libera al disparar.
   - `evidence_logger.py::_DEFAULT_GRIPPER_OBJECTS` — el set monitoreado
     para attach/detach events.

3. **Object pose resolver**: el servicio
   `/orchestrator/resolve_object_pose_world` consulta `gz_pose_bridge`
   por nombre. Asegura que cada objeto del world se publique en
   `/world/<world_name>/pose/info` (Gazebo Sim lo hace automático para
   `<model>` con detachable_joint).

4. **Drop position**: el orchestrator action `PickPlace.action` recibe
   `drop_xyz_world` como parte del goal. No requiere cambio de código,
   solo pasar las coordenadas correctas en el cliente.

### Tests críticos

```bash
pytest src/ur5_tools/test/test_object_pose_resolver_launch.py
pytest src/ur5_tools/test/test_release_integrity.py
```

---

## 4. Cambiar la estrategia de inferencia de agarres

### Pasos

1. **Modelo PyTorch**: el módulo `tfm_grasping/model.py::GraspModel`
   carga checkpoints arbitrarios. Para usar otro modelo:
   - Implementar la misma interfaz: `(rgb_tensor, depth_tensor) →
     (cx, cy, w, h, angle_deg)` en pixels.
   - Añadir el modelo a la lista de candidatos en
     `_build_model_candidates()` con `(model_class, in_channels,
     model_name)`.

2. **Pipeline de percepción**: `perception.py::PerceptionPipeline`
   prepara las entradas (resize, normalize, ROI). Para nuevas
   modalidades (point cloud, depth-only), extender este módulo.

3. **Output post-processing**: `model.py::_decode_prediction`
   transforma la salida del modelo a `Grasp2D` (estructura del
   sistema). Si tu modelo emite poses 6D, extender `Grasp2D` a
   `Grasp6D` (no incluido por defecto — sería una extensión natural
   del proyecto).

4. **Action contract**: el orchestrator consume `object_pose_world_hint`
   en `PickPlace.action`. Si la inferencia produce poses 6D directamente
   (no requiere reconstrucción 2D→3D), bypassa
   `ComputeApproachPose.srv` y pasa la pose directamente al hint.

### Tests críticos

```bash
pytest src/tfm_grasping/test/
```

---

## 5. Migrar a otra distribución de ROS 2 (Kilted, Lyrical)

### Cambios mínimos esperados

- ROS 2 Jazzy → Kilted: API estable; típicamente cambian solo dependencias
  externas (gz_ros2_control versions).
- Verificar `package.xml` exec_depend versions.
- Verificar `mypy.ini` — algunos type stubs pueden mover de paquete.

### Comandos

```bash
# Bajar Jazzy, instalar Kilted, sourcing nuevo
source /opt/ros/kilted/setup.bash
cd agarre_ros2_ws && colcon build --symlink-install
colcon test
```

---

## 6. Pasar a robot real (no simulación)

Esta es la fase F10 documentada. Pasos resumidos:

1. **Hardware interface**: usar `ur_robot_driver` (UR Universal Robot
   Driver) en lugar de `gz_ros2_control`.
2. **Calibración del workspace**: `world→base_link` en el world SDF
   debe coincidir con la pose física del robot real (medida con tape
   measure o calibración óptica).
3. **Safety**: el orchestrator NO tiene checks de zona de seguridad.
   Para hardware real:
   - Añadir un `safety_state_manager` que monitoree e-stop, fuerza, velocidad.
   - Wirear como kill-switch en cada acción.
4. **Tolerancias FJT**: las defaults `position_tol_rad=1.0` (audit-v4 F1.7)
   son sim-friendly. Para hardware: bajar a 0.05–0.10 rad.
5. **Cámaras**: reemplazar `gz_pose_bridge` (TF de Gazebo) por
   detección visual real (Aruco/AprilTag o modelo CNN).

### Tests críticos antes de mover el robot real

```bash
# Dry-run: planificar sin ejecutar
ros2 launch ur5_bringup ur5_real_bringup.launch.py dry_run:=true

# Verificar TF tree consistente
ros2 run tf2_tools view_frames
```

---

## Referencias

- [docs/architecture.md](architecture.md) — vista general del stack
- [docs/LIFECYCLE.md](LIFECYCLE.md) — patrón LifecycleNode usado
- [docs/CONFIG_HIERARCHY.md](CONFIG_HIERARCHY.md) — orden URDF/SRDF/YAML/env
- [docs/REFACTOR_F14_PATTERN.md](REFACTOR_F14_PATTERN.md) — patrón de refactor de mixins
- [docs/F10_ROADMAP_ROBOT_REAL.md](F10_ROADMAP_ROBOT_REAL.md) — roadmap robot real
- [auditoria/audit_profesional_20260507.md](../../auditoria/audit_profesional_20260507.md) — audit v4 actual
