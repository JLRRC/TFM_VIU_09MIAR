# Rescate — Integridad del modelo Gazebo UR5+RG2

Fecha: 2026-05-11
Rama: `rescue/gazebo-model-integrity-<timestamp>`
Estado base: HEAD = `e421421` (rama `release/estable-20260511`).

## 1. Resumen del problema

Gazebo mostraba el UR5 + OnRobot RG2 visualmente roto: eslabones rotados respecto al pivote, brazo desencajado, gripper aparentemente flotando. RViz y `/tf` se veían bien (porque consumen URDF/RSP), lo cual ocultaba la inconsistencia hasta abrir Gazebo.

## 2. Causa raíz

El archivo `src/ur5_gazebo/models/ur5_rg2/model.sdf` tenía **cambios sin commit que reescribían toda la cinemática del UR5 al convenio URDF/DH**:

- Ejes Z en todos los joints móviles (en vez del convenio SDF axis-Y/Z mixto del paquete ros-industrial).
- Poses tipo DH (`-0.425 0 0`, `-0.39225 0 0.10915`) en lugar de las poses originales (`0 -0.1197 0.425`, `0 0 0.39225`).
- Rotaciones inline π/2 en joints móviles del brazo.
- `end_effector_frame_fixed_joint` quedaba a identidad en lugar de `-π/2 0 0`.

Las mallas `.dae`/`.stl` en `src/ur5_gazebo/models/ur5_rg2/meshes/visual/ur5/` están autoradas para el convenio SDF original. Al cambiar ejes y poses sin migrar las mallas, los eslabones aparecen rotados respecto a sus pivotes → "piezas flotando".

Los cambios formaban parte de un intento de "URDF parity" inacabado y sin commit. El commit relacionado `2908e6e` ("Stabilize URDF-based UR5+RG2 Gazebo spawn") había sido revertido el día anterior por `ab0c355`; los cambios sueltos eran probablemente residuos manuales de un nuevo intento.

## 3. Por qué `model.sdf` DH/URDF rompe las mallas SDF

En SDF las mallas se cargan en el frame del link al que pertenecen. Cuando el joint padre rota sobre un eje y con un offset distinto al que las mallas asumieron, el efecto visual es exactamente un eslabón girado o desplazado respecto a su pivote. La cadena cinemática puede ser matemáticamente válida, pero el render es inconsistente con la realidad mecánica.

Convención SDF correcta (HEAD) vs convención URDF/DH rota:

| Joint | SDF original (HEAD, casa con mallas) | URDF/DH (sin commit, rompe mallas) |
|---|---|---|
| shoulder_pan | `pose 0 0 0.089159 0 0 0` axis `0 0 1` | `pose 0 0 0.089159 0 0 π` |
| shoulder_lift | `pose 0 0.13585 0 0 0 0` axis `0 1 0` | `pose 0 0 0 π/2 0 0` axis `0 0 1` |
| elbow | `pose 0 -0.1197 0.425 0 0 0` axis `0 1 0` | `pose -0.425 0 0 0 0 0` axis `0 0 1` |
| wrist_1 | `pose 0 0 0.39225 0 0 0` axis `0 1 0` | `pose -0.39225 0 0.10915 0 0 0` axis `0 0 1` |
| wrist_2 | `pose 0 0.093 0 0 0 0` axis `0 0 1` | `pose 0 -0.09465 0 π/2 0 0` axis `0 0 1` |
| wrist_3 | `pose 0 0 0.09465 0 0 0` axis `0 1 0` | `pose 0 0.0823 0 -π/2 0 0` axis `0 0 1` |
| end_effector_frame_fixed | `pose 0 0 0 -π/2 0 0` | `pose 0 0 0 0 0 0` |

## 4. Archivo restaurado

- `src/ur5_gazebo/models/ur5_rg2/model.sdf` → revertido a HEAD vía `git stash push`.
- Stash retenido como `stash@{0}: rescate-sdf-20260511`. Recuperable con `git stash show stash@{0}` o `git stash pop stash@{0}` si se quiere volver a inspeccionar.
- Ningún otro archivo modificado se ha alterado.

## 5. Scripts creados o ampliados

| Script | Propósito | Requiere stack vivo |
|---|---|---|
| `scripts/check_ur5_rg2_sdf_convention.sh` | Verificación estática del convenio SDF del `model.sdf` (joints, poses, anti-patrones DH). | No |
| `scripts/check_static_robot_model_sources.sh` | 9 chequeos pre-launch: URDF/Xacro válido, model.sdf presente y con convenio correcto, world con un único include, sin spawns duplicados, sin model.sdf alternativos. | No |
| `scripts/check_gazebo_model_integrity.sh` | 10 chequeos runtime: `gz model --list`, `pose/info`, TF base_link→{tool0,rg2_base_link,rg2_pinch_center,rg2_finger_*}, distancias internas, `/joint_states`, `ros2 control list_controllers`. Si todo pasa, escribe el flag `/tmp/gazebo_model_integrity_ok`. | Sí |

## 6. Gate de pick (`pick_orchestrator_lifecycle_node.py`)

Inserción mínima en `_goal_callback`:

```python
ok, gate_reason = _check_gazebo_integrity_gate()
if not ok:
    self.get_logger().error(
        "PICK BLOQUEADO: Gazebo model integrity no validada. "
        "Ejecuta ./scripts/check_gazebo_model_integrity.sh y corrige Gazebo "
        f"antes de pick. ({gate_reason})"
    )
    return GoalResponse.REJECT
```

Helper puro `_check_gazebo_integrity_gate(env=None, flag_path=None)`:

1. Si `ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY=1` (también `true`/`yes`/`on`/`True`/`TRUE`) → permitido (debug override).
2. Si existe el flag escrito por `check_gazebo_model_integrity.sh` (default `/tmp/gazebo_model_integrity_ok`, override con `GAZEBO_INTEGRITY_FLAG`) → permitido.
3. Cualquier otro caso → bloqueado, goal rechazado con mensaje claro en el log.

Tests offline: `src/tfm_orchestrator/test/test_gazebo_integrity_gate.py` (17 casos, todos PASS).

## 7. Inventario de vías de carga del robot

| Archivo | Tipo | Carga robot | Carga gripper | Carga mesa | Carga objetos | Vía | Riesgo duplicado | Acción tomada | Acción recomendada |
|---|---|---|---|---|---|---|---|---|---|
| `src/ur5_description/urdf/ur5.urdf.xacro` | URDF/Xacro | Sí (cinemática + ros2_control) | Sí | No | No | RSP → `/robot_description` → TF | con SDF | Mantener | Mantener |
| `src/ur5_gazebo/models/ur5_rg2/model.sdf` | SDF | Sí (meshes + física + plugins) | Sí | No | No | `<include>` del world | con URDF | **Restaurado a HEAD** | Mantener |
| `src/ur5_gazebo/worlds/ur5_mesa_objetos.sdf` | World | Vía `<include>model://ur5_rg2</include>` línea 422 | Vía include | Sí | Sí (`pick_demo` + cilindros) | `gz sim -s -r world_runtime.sdf` | OK | Mantener | Mantener |
| `src/ur5_gazebo/worlds/ur5_debug_empty.sdf` | World alternativo | Vía `<include>` | Vía include | No | No | No referenciado desde launch | OK | Mantener (dev) | Mantener |
| `src/ur5_bringup/launch/ur5_rsp.launch.py` | Launch | No spawn — solo publica `/robot_description` | No | No | No | xacro → RSP | OK | Mantener | Mantener |
| `src/ur5_bringup/launch/ur5_stack.launch.py` | Launch | No spawn — orquesta RSP + Gazebo + bridge + panel | — | — | — | top-level | OK | Mantener | Mantener |
| `src/ur5_bringup/launch/gz_factory.py` | Launch | No spawn — arranca `gz sim -s` con el world | Vía world | Vía world | Vía world | `ExecuteProcess` | OK | Mantener | Mantener |
| `src/ur5_bringup/launch/launch_helpers.py:135` | Helper | Reescribe URI `model://ur5_rg2` → `file://<runtime>` | — | — | — | post-procesado SDF | OK (no spawn) | Mantener | Mantener |
| `src/ur5_tools/ur5_tools/gripper_geometry.py::patch_runtime_model_sdf` | Helper | Parche runtime de `controllers_yaml` + `pick_demo_anchor_joint` | — | No | No | post-procesado SDF | OK | Mantener | Mantener |
| `src/ur5_qt_panel/ur5_qt_panel/panel_launchers.py:331` | Helper UI | Mismo reescribe URI que `launch_helpers.py` | — | — | — | invocado desde panel | OK | Mantener | Mantener |
| `src/ur5_gazebo/launch/spawn_objects.launch.py` | Launch | No carga robot. Spawnea solo objetos via `ros_gz_sim CreateService`. | No | No | Sí | service call | OK | Mantener | Mantener |

**Resultado**: no hay duplicados de spawn del robot. El robot entra a Gazebo exclusivamente vía el `<include>` del world. Existe duplicidad **estructural** URDF↔SDF (RSP usa URDF, Gazebo usa SDF) pero está fuera del alcance de este rescate.

## 8. Comandos de validación

### Estático (sin Gazebo)

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash   # tras colcon build
./scripts/check_static_robot_model_sources.sh
```

### Runtime (con stack vivo)

```bash
# 1) Arrancar stack
./scripts/start_panel_v2.sh   # o launch directo

# 2) Cargar GZ_PARTITION
export GZ_PARTITION="$(cat log/gz_partition.txt)"

# 3) Inspección rápida
gz model --list
gz topic -e -n 1 -t /world/ur5_mesa_objetos/pose/info > /tmp/gz_pose_info.txt
ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
ros2 control list_controllers

# 4) Test automático (escribe /tmp/gazebo_model_integrity_ok si todo OK)
./scripts/check_gazebo_model_integrity.sh
```

## 9. Criterio "Gazebo limpio"

`GAZEBO_MODEL_INTEGRITY_OK=true` solo cuando, simultáneamente:

- `check_static_robot_model_sources.sh` exit 0.
- `check_gazebo_model_integrity.sh` exit 0 (lo cual implica los 10 chequeos: un único `ur5_rg2`, sin links sueltos como modelos top-level, `/world/.../pose/info` contiene `ur5_rg2`, TF base_link→{tool0, rg2_base_link, rg2_pinch_center, rg2_finger_link1, rg2_finger_link2}, distancias dentro de tolerancias, `/joint_states` con los 6 joints del UR5 y los del gripper, `joint_state_broadcaster` + controlador del brazo activos).
- Inspección visual: brazo continuo, gripper unido al flange/tool0, dedos unidos al cuerpo del RG2, sin piezas flotando ni rotaciones absurdas en eslabones.

## 10. Estado del pick

- Por defecto: **BLOQUEADO** hasta que `check_gazebo_model_integrity.sh` escriba `/tmp/gazebo_model_integrity_ok`.
- Override para depuración: `ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY=1`.
- Mensaje al rechazar: `"PICK BLOQUEADO: Gazebo model integrity no validada. Ejecuta ./scripts/check_gazebo_model_integrity.sh y corrige Gazebo antes de pick. (...)"`.

## 11. Rollback

### Revertir el rescate del SDF

Si tras inspeccionar quieres recuperar los cambios DH (no recomendado hasta migrar las mallas):

```bash
git stash pop stash@{0}      # restaura model.sdf al estado roto
```

### Revertir el gate de pick

```bash
# Volver a HEAD del archivo del orchestrator
git restore src/tfm_orchestrator/tfm_orchestrator/pick_orchestrator_lifecycle_node.py
# Borrar tests
rm src/tfm_orchestrator/test/test_gazebo_integrity_gate.py
# Rebuild
colcon build --packages-select tfm_orchestrator --symlink-install
```

### Revertir los scripts

```bash
rm scripts/check_ur5_rg2_sdf_convention.sh
rm scripts/check_static_robot_model_sources.sh
# check_gazebo_model_integrity.sh fue ampliado; el original ya estaba untracked → simplemente:
git clean -i scripts/
```

### Volver de rama

```bash
git switch release/estable-20260511
git branch -d rescue/gazebo-model-integrity-<timestamp>
```

## 12. Siguiente paso recomendado

1. Levantar el stack y validar manualmente que Gazebo se ve correcto.
2. Ejecutar `./scripts/check_gazebo_model_integrity.sh` y confirmar `GAZEBO_MODEL_INTEGRITY_OK=true`.
3. Solo entonces, evaluar **Path B URDF-only**:
   - Investigar antes por qué se revirtió el commit `2908e6e`.
   - Re-aplicar la idea (spawn desde `/robot_description` con `ros_gz_sim create -topic robot_description`, eliminar `<include>` del world) con diagnóstico previo.
   - Migrar la lógica de las DetachableJoint plugins y el contact-system al URDF/Xacro.
   - Validar de nuevo con el script de integridad antes de declarar Path B cerrado.
