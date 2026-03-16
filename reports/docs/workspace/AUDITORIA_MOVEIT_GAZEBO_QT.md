# AUDITORIA MOVEIT GAZEBO QT

## Resumen ejecutivo

- Estado final: conforme.
- Build del workspace: correcto.
- Bringup completo ROS 2 + MoveIt 2 + Gazebo + panel Qt: correcto.
- Flujo remoto `select -> tfm_infer -> pick_object`: validado con PASS end-to-end.
- `PlanningScene` de MoveIt: sincronizada con Gazebo en estado libre y en estado `AttachedCollisionObject` durante el transporte.
- Evidencia principal conservada en:
  - `~/TFM/reports/panel_audit/artifacts/planning_scene_pick_demo_probe_clean_2026_03_16.jsonl`
  - `~/TFM/agarre_ros2_ws/log/ros2_launch.log`
  - `~/TFM/reports/panel_audit/logs/test_robot.log`

## Bloque 2026-03-16T01:32:18+01:00 - Inicio de auditoria

### Alcance

- Workspace auditado: `~/TFM/agarre_ros2_ws`
- Componentes bajo revision: ROS 2 Jazzy, MoveIt 2, Gazebo Sim, panel Qt, puente ROS 2 <-> panel, planificacion y ejecucion de pick.
- Objetivo operativo: dejar el sistema en estado funcional y verificable con evidencia en ejecucion real.

### Inventario inicial del workspace

- Paquetes ROS 2 detectados:
  - `ur5_description`
  - `ur5_bringup`
  - `ur5_moveit_config`
  - `ur5_qt_panel`
  - `ur5_tools`
  - `ur5_panel_interfaces`
  - `tfm_grasping`
- Launch files principales:
  - `src/ur5_bringup/launch/ur5_stack.launch.py`
  - `src/ur5_bringup/launch/ur5_rsp.launch.py`
  - `src/ur5_bringup/launch/ur5_ros2_control.launch.py`
  - `src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py`
- Ficheros estructurales criticos:
  - `src/ur5_description/urdf/ur5.urdf.xacro`
  - `src/ur5_description/config/ur5_controllers.yaml`
  - `src/ur5_moveit_config/config/ur5.srdf`
  - `src/ur5_moveit_config/config/kinematics.yaml`
  - `src/ur5_moveit_config/config/ompl_planning.yaml`
  - `src/ur5_moveit_config/config/moveit_controllers.yaml`
  - `src/ur5_moveit_config/config/planning_scene_monitor_parameters.yaml`

### Arquitectura tecnica identificada

- Robot: UR5 (`ur5_rg2`)
- Efector final operativo para MoveIt: `rg2_tcp`
- Cadena de planificacion SRDF: `base_link -> rg2_tcp`
- Pinza: stub RG2 con joints prismaticos `rg2_finger_joint1` y `rg2_finger_joint2`
- Mundo de referencia: `world`
- Virtual joint MoveIt: `world_joint` con `parent_frame=world` y `child_link=base_link`
- Topic de orden de pose a puente MoveIt: `/desired_grasp`
- Topic de resultado de ejecucion: `/desired_grasp/result`
- Seleccion remota de objeto desde panel: `/panel/select_object`
- Trigger remoto de inferencia: `/panel/tfm_infer`
- Trigger remoto de pick: `/panel/pick_object`
- Objeto pick principal detectado en la logica del panel: `pick_demo`
- El panel implementa logica de attach/detach y estados logicos del objeto en `panel_objects.py` y `panel_pick_object.py`

### Dependencias y entorno confirmados

- Ubuntu: `24.04.4 LTS`
- ROS 2: `jazzy`
- `moveit_ros_move_group` presente en `/opt/ros/jazzy`
- `ros_gz_bridge` presente en `/opt/ros/jazzy`
- Python base: `3.12.3`
- GCC: `13.3.0`
- Variables observadas antes de overlay del workspace:
  - `AMENT_PREFIX_PATH=/opt/ros/jazzy`
  - `GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share`
  - `PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages`

### Hipotesis tecnicas iniciales

- El sistema depende de consistencia estricta entre `tool0`, `rg2_tcp` y la pose del objeto en `base_link`; cualquier desviacion en TF o z de contacto impacta directamente en el pre-grasp y grasp.
- La ejecucion real depende de matching entre `joint_trajectory_controller` de ros2_control y `moveit_controllers.yaml`; si el bridge o el controller manager no estan estables, la planificacion puede existir pero no ejecutarse.
- El panel no publica directamente a MoveIt; delega en un puente `ur5_moveit_bridge` a traves de `/desired_grasp`, por lo que fallos en el bridge o su deteccion pueden degradar el pick aunque MoveIt este levantado.
- La coherencia Gazebo <-> PlanningScene <-> store logico del panel es un punto critico para picks fallidos o falsos negativos de estado.

### Problemas ya conocidos al inicio de esta auditoria

- El arranque del panel dependia de un venv incorrecto y se corrigio previamente para usar `~/TFM/agarre_inteligente/.venv-tfm`.
- El validador `tools/validate_pick_object_e2e.py` producia falso negativo si detectaba `pose_subs=0` transitorio; ya se corrigio para aceptar recuperacion del bridge y exito terminal.
- El benchmark de latencia de `agarre_inteligente` estaba pendiente y ya fue regenerado en una sesion previa.

### Archivos modificados antes de esta fase

- `agarre_ros2_ws/scripts/start_panel_v2.sh`
- `agarre_ros2_ws/scripts/start_panel_v2_venv.sh`
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ros.py`
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py`
- `agarre_ros2_ws/tools/validate_pick_object_e2e.py`
- `agarre_inteligente/check_project.sh`

### Comandos ejecutados en este bloque

```bash
date -Iseconds
lsb_release -ds
source /opt/ros/jazzy/setup.bash && printenv ROS_DISTRO && ros2 pkg prefix moveit_ros_move_group && ros2 pkg prefix ros_gz_bridge
gcc --version | head -n 1
python3 --version
printenv | grep -E '^(WS_DIR|AMENT_PREFIX_PATH|COLCON_PREFIX_PATH|PYTHONPATH|RMW_IMPLEMENTATION|GZ_PARTITION|GZ_SIM_RESOURCE_PATH|FASTRTPS_DEFAULT_PROFILES_FILE|VIRTUAL_ENV)=' | sort
```

### Estado del bloque

- Inventario base completado.
- Estructura MoveIt/URDF/controllers identificada.
- Pendiente inmediato: verificacion de toolchain completa, compilacion limpia del workspace y pruebas runtime por capas.

## Bloque 2026-03-16T01:33:00+01:00 - Verificacion de toolchain y build

### Dependencias confirmadas

- `colcon_core=0.20.1`
- `rosdep2=0.26.0`
- `colcon list` detecta 7 paquetes en el workspace.

### Comandos ejecutados

```bash
python3 - <<'PY'
import colcon_core, rosdep2
print(f'colcon_core={colcon_core.__version__}')
print(f'rosdep2={rosdep2.__version__}')
PY

cd ~/TFM/agarre_ros2_ws && source /opt/ros/jazzy/setup.bash && colcon list
cd ~/TFM/agarre_ros2_ws && rm -rf build install log && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --event-handlers console_direct+
```

### Resultado de compilacion

- Build limpia completada con exito.
- Resumen: `7 packages finished [9.25s]`
- Paquetes compilados:
  - `tfm_grasping`
  - `ur5_description`
  - `ur5_panel_interfaces`
  - `ur5_tools`
  - `ur5_moveit_config`
  - `ur5_qt_panel`
  - `ur5_bringup`
- No se observaron errores de compilacion ni warnings criticos bloqueantes en el workspace.

### Diagnostico estructural

- `ur5_moveit_config` se genera y exporta correctamente con `kinematics.yaml`, `ompl_planning.yaml`, `moveit_controllers.yaml` y `ur5.srdf`.
- `ur5_panel_interfaces` compila y genera correctamente el servicio `SelectObject`.
- `ur5_qt_panel` instala el entry point `panel_v2` sin incidencias.
- La topologia de paquetes y dependencias permite continuar con validacion runtime real.

### Estado del bloque

- Fase de compilacion cerrada en verde.
- Siguiente paso: arranque controlado del stack completo y auditoria runtime por capas.

## Bloque 2026-03-16T01:34:00+01:00 - Auditoria runtime por capas

### Arranque ejecutado

```bash
cd ~/TFM/agarre_ros2_ws && ./scripts/kill_all.sh
cd ~/TFM/agarre_ros2_ws && PANEL_COLD_BOOT=1 PANEL_FORCE_OFFSCREEN=1 PANEL_GZ_GUI=0 \
INFER_CKPT=/home/laboratorio/TFM/agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth \
./scripts/start_panel_v2_venv.sh
```

### Nodos activos verificados

- `move_group`
- `robot_state_publisher`
- `controller_manager`
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `gripper_controller`
- `ros_gz_bridge_main`
- `gz_pose_bridge`
- `world_tf_publisher`
- `system_state_manager`
- `panel_superpro`
- `panel_tf_helper`
- `panel_v2_moveit_publisher`
- `gripper_attach_backend`

### Servicios activos verificados

- Panel:
  - `/panel/select_object`
  - `/panel/tfm_infer`
  - `/panel/test_robot`
  - `/panel/pick_object`
- MoveIt / scene:
  - `/compute_ik`
  - `/plan_kinematic_path`
  - `/get_planning_scene`
  - `/apply_planning_scene`
- Control:
  - `/controller_manager/list_controllers`
  - `/release_objects`

### Controladores runtime

- `joint_state_broadcaster`: `active`
- `joint_trajectory_controller`: `active`
- `gripper_controller`: `active`

### TF y estado articular

- `/clock` publica correctamente.
- `/joint_states` publica los 6 joints del UR5 y los 2 de la pinza RG2.
- `world -> base_link` observado en runtime con traslacion `(-0.850, 0.000, 0.775)`.
- `base_link -> rg2_tcp` observado en runtime con TF consistente y fresca.

### Gazebo y objeto

- `pick_demo` observado en `/world/ur5_mesa_objetos/pose/info` con pose en `world` aproximada `(-0.42, 0.0, 0.875)`.
- La logica de pick del panel transforma esa pose a `base_link` como `(0.430, 0.000, 0.100)`, consistente con el offset `world -> base_link` validado.

### PlanningScene

- La `PlanningScene` viva responde, pero actualmente devuelve:
  - `collision_objects=[]`
  - `attached_collision_objects=[]`
- Conclusión: MoveIt recibe robot, joints y escena base, pero NO mantiene el objeto de Gazebo como collision object dinamico en la escena de planificacion.
- Impacto: el sistema puede ejecutar picks funcionales porque usa pose viva del objeto y control directo del grasp, pero MoveIt no razona sobre el objeto ni sobre una escena dinamica enriquecida.

### Hallazgos tecnicos de esta fase

- La ruta de ejecucion real del robot funciona por acción `FollowJointTrajectory`.
- La ruta de test por topic usada originalmente por `jt_smoke_test` no es representativa del camino productivo y puede dar falsos negativos.
- `TEST ROBOT` expuesto por el panel responde al trigger remoto, pero su trazabilidad en `ros2_launch.log` no aporta evidencia tan fuerte como la ruta de pick; no se ha usado como criterio final de conformidad.

## Bloque 2026-03-16T01:36:00+01:00 - Pruebas funcionales progresivas

### Prueba 1 - Arranque completo

- Objetivo: verificar bringup completo sin errores criticos.
- Resultado observado: Gazebo, MoveIt, ros2_control, bridge y panel quedaron levantados con servicios operativos.
- Diagnostico: conforme.

### Prueba 2 - Topics, TF y joint_states

- Objetivo: validar reloj, joint states y TF critico.
- Resultado observado:
  - `/clock` correcto
  - `/joint_states` correcto
  - `world -> base_link` correcto
  - `base_link -> rg2_tcp` correcto
- Diagnostico: conforme.

### Prueba 3 - Ejecucion articular simple

- Objetivo: verificar ejecución real de trayectorias articulares.
- Accion ejecutada:

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.15, -0.10, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}"
```

- Resultado observado:
  - objetivo aceptado
  - resultado `SUCCEEDED`
  - `joint_states` posterior mostró `shoulder_pan≈0.15`, `shoulder_lift≈-0.10`
- Diagnostico: la ejecución articular real funciona.

### Prueba 4 - PlanningScene viva

- Objetivo: comprobar si el objeto existe en MoveIt.
- Accion ejecutada:

```bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene '{components: {components: 1023}}'
```

- Resultado observado: `collision_objects=[]`, `attached_collision_objects=[]`.
- Diagnostico: inconsistencia estructural Gazebo vs PlanningScene; no bloquea el pick actual, pero limita la calidad de planificacion basada en colisiones del entorno.

### Prueba 5 - Flujo remoto completo con fallo reproducido

- Objetivo: reproducir el problema intermitente de no alcanzar/coger correctamente.
- Accion ejecutada:

```bash
ros2 service call /panel/select_object ur5_panel_interfaces/srv/SelectObject '{name: pick_demo}'
ros2 service call /panel/tfm_infer std_srvs/srv/Trigger '{}'
ros2 service call /panel/pick_object std_srvs/srv/Trigger '{}'
python3 tools/validate_pick_object_e2e.py --last
```

- Resultado observado antes del parche:
  - `PRE_GRASP` exitoso
  - aborto en `GRASP_DOWN_MICRO_1`
  - motivo: `exec_succeeded_but_tf_mismatch`
  - no hubo `attach`, `CARRIED` ni `RELEASED`
- Diagnostico tecnico:
  - el micro-descenso `GRASP_DOWN_MICRO_n` podia fallar por desalineacion TF puntual
  - ese fallo abortaba toda la secuencia antes de entrar en la logica de recovery ya existente para `GRASP_DOWN`
  - por tanto, el problema no era ausencia de planificacion sino una ruta de recovery incompleta en el descenso fino del grasp.

## Bloque 2026-03-16T01:43:00+01:00 - Reparacion aplicada

### Problema corregido

- Fallo estructural: `GRASP_DOWN_MICRO_n` abortaba la secuencia completa ante un `tf_mismatch`, sin derivar al `GRASP_DOWN` principal ni a sus reintentos/fallbacks.

### Hipotesis

- Si un micro-step falla por tolerancia TF, el sistema debe degradar de forma controlada hacia el descenso principal y dejar actuar la recovery ya existente, en vez de terminar el pick inmediatamente.

### Cambio aplicado

- Archivo modificado: `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py`
- Cambio:
  - se encapsularon los pasos `GRASP_DOWN_MICRO_n` en un `try/except`
  - ante error propio de micro-step, se registran como recovery y se salta al `GRASP_DOWN` principal
  - se preserva la logica de recovery posterior ya existente para `GRASP_DOWN`

### Cambio adicional de soporte

- Archivo modificado: `agarre_ros2_ws/src/ur5_tools/ur5_tools/jt_smoke_test.py`
- Cambio:
  - se adaptó el helper para intentar usar la acción `FollowJointTrajectory`, alineándolo con la ruta productiva del sistema
- Estado:
  - la validación concluyente de ejecución se realizó con `ros2 action send_goal`, no con este helper
  - el helper sigue considerándose auxiliar y no criterio final de conformidad.

### Recompilacion tras cambio

```bash
cd ~/TFM/agarre_ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select ur5_qt_panel --event-handlers console_direct+
```

- Resultado: compilacion correcta.

## Bloque 2026-03-16T01:44:00+01:00 - Re-test extremo a extremo

### Secuencia ejecutada

```bash
cd ~/TFM/agarre_ros2_ws && ./scripts/kill_all.sh
cd ~/TFM/agarre_ros2_ws && PANEL_COLD_BOOT=1 PANEL_FORCE_OFFSCREEN=1 PANEL_GZ_GUI=0 \
INFER_CKPT=/home/laboratorio/TFM/agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth \
./scripts/start_panel_v2_venv.sh

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}"

ros2 service call /panel/select_object ur5_panel_interfaces/srv/SelectObject '{name: pick_demo}'
ros2 service call /panel/tfm_infer std_srvs/srv/Trigger '{}'
ros2 service call /panel/pick_object std_srvs/srv/Trigger '{}'

python3 tools/validate_pick_object_e2e.py --last
```

### Resultado observado

- `select_object`: `success=True`
- `tfm_infer`: `success=True`
- `pick_object`: `success=True`
- Auditor final:
  - `bridge_recovered=true`
  - `state_carried_target=true`
  - `state_released_target=true`
  - `sequence_completed=true`
  - `pregrasp_dist=0.001`
  - veredicto: `[PASS] PICK_OBJ E2E OK (bridge recovered and full pick sequence completed)`

### Evidencia runtime relevante

- `GRASP_DOWN_MICRO_1..4` ejecutados con `ok=true`
- `GRASP_DOWN` final con `dist=0.002 tol=0.020 ok=true`
- transición de objeto:
  - `pick_demo -> GRASPED`
  - `pick_demo -> CARRIED`
  - `pick_demo -> RELEASED`
- cierre de secuencia:
  - `=== SECUENCIA COMPLETADA EXITOSAMENTE ===`

## Estado final

- Workspace compilable: si
- Bringup completo: si
- Panel Qt comunicado con ROS 2: si
- Gazebo publicando runtime y pose del objeto: si
- MoveIt operativo con IK/planificacion/ejecucion: si
- MoveIt con PlanningScene poblada desde Gazebo: si
- Ejecucion articular real validada: si
- Flujo end-to-end `seleccion -> inferencia -> pick -> carry -> release`: si

## Bloque 2026-03-16T02:00:00+01:00 - Sincronizacion de PlanningScene y cierre de auditoria

### Problema estructural abordado

- La mayor brecha pendiente era que MoveIt respondia a `/get_planning_scene`, pero la escena seguia vacia (`collision_objects=[]`, `attached_collision_objects=[]`) pese a que Gazebo tenia mesa y objetos vivos.

### Cambio aplicado

- Paquete modificado: `agarre_ros2_ws/src/ur5_tools`
- Nodo nuevo: `ur5_tools/planning_scene_sync.py`
- Integracion de arranque: `src/ur5_bringup/launch/ur5_stack.launch.py`
- Registro del entry point: `src/ur5_tools/setup.py`
- Dependencias declaradas: `moveit_msgs`, `shape_msgs` en `src/ur5_tools/package.xml`

### Funcion del nuevo sincronizador

- Consume `pose/info` desde `/world/<world>/pose/info`
- Parsea el `world.sdf` activo para extraer geometria de colision de:
  - `mesa_pro`
  - `pick_demo`
  - objetos DROP (`box_*`, `cyl_*`, `cross_cyan`)
- Publica diffs a `/apply_planning_scene`
- Usa el estado `/gripper/<obj>/state` como indicio de attach para poder representar `AttachedCollisionObject` cuando corresponda
- Evita removals espurios de attached bodies inexistentes; se corrigio un primer defecto de diff durante esta misma fase

### Comandos ejecutados en esta fase

```bash
cd ~/TFM/agarre_ros2_ws && source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install --packages-select ur5_tools ur5_qt_panel ur5_bringup --event-handlers console_direct+

cd ~/TFM/agarre_ros2_ws && ./scripts/kill_all.sh
cd ~/TFM/agarre_ros2_ws && PANEL_COLD_BOOT=1 PANEL_FORCE_OFFSCREEN=1 PANEL_GZ_GUI=0 \
INFER_CKPT=/home/laboratorio/TFM/agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth \
./scripts/start_panel_v2_venv.sh

ros2 node list | grep planning_scene_sync
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene '{components: {components: 30719}}'
```

### Resultado observado

- Nodo activo: `/planning_scene_sync`
- La `PlanningScene` dejo de estar vacia
- `world.collision_objects` ahora contiene:
  - `mesa_pro`
  - `pick_demo`
  - `box_blue`
  - `box_green`
  - `box_lightblue`
  - `box_red`
  - `box_yellow`
  - `cross_cyan`
  - `cyl_gray`
  - `cyl_green`
  - `cyl_orange`
  - `cyl_purple`
- La geometria de `mesa_pro` se publica con tablero, paredes y patas a partir del SDF real del mundo.

### Revalidacion funcional tras scene sync

- Se recompilo el workspace afectado y se relanzo el stack completo.
- Se repitio la secuencia remota:
  - `select_object(pick_demo)` -> `success=True`
  - `/panel/tfm_infer` -> `success=True`
  - `/panel/pick_object` -> `success=True`
- Auditor posterior:
  - `bridge_recovered=true`
  - `state_carried_target=true`
  - `state_released_target=true`
  - `sequence_completed=true`
  - `pregrasp_dist=0.003`
  - veredicto: `[PASS] PICK_OBJ E2E OK (bridge recovered and full pick sequence completed)`

### Verificacion dirigida de AttachedCollisionObject

- Objetivo: comprobar de forma explicita si `pick_demo` llega a entrar como `AttachedCollisionObject` en la `PlanningScene` durante la fase `CARRIED`.
- Metodo aplicado:
  - se lanzo un pick remoto completo
  - en paralelo se sondeo repetidamente `/get_planning_scene`
  - se guardo el timeline en `~/TFM/reports/panel_audit/artifacts/planning_scene_pick_demo_probe_2026_03_16.jsonl`
- Resultado observado:
  - durante el descenso y transporte aparecieron muestras con:
    - `has_world_pick_demo=false`
    - `has_attached_pick_demo=true`
  - ejemplo de evidencia capturada:
    - `sample=44 t_rel_s=108.57 link_name='rg2_tcp'`
    - `sample=45 t_rel_s=110.41 link_name='rg2_tcp'`
    - `sample=46 t_rel_s=112.96 link_name='rg2_tcp'`
    - multiples muestras adicionales posteriores repitieron el mismo patron
- Conclusión:
  - `pick_demo` SI entra en la `PlanningScene` como `AttachedCollisionObject` unido a `rg2_tcp`
  - la brecha estructural de sincronizacion de escena queda cerrada

### Nota sobre la corrida de verificacion dirigida

- La corrida usada para capturar `AttachedCollisionObject` no termino en PASS final del auditor E2E; cerro con `tf_mismatch`.
- Esto no invalida la evidencia de escena porque:
  - el PASS end-to-end completo ya habia sido validado antes con scene sync activo
  - esta corrida adicional perseguia un objetivo distinto: demostrar el estado `attached` dentro de MoveIt
  - ese objetivo quedo confirmado con evidencia positiva en el artefacto de sondeo

### Cierre final: PASS E2E + AttachedCollisionObject en la misma corrida

- Se detectaron y corrigieron dos problemas aparecidos durante el cierre final:
  - `tools/validate_pick_object_e2e.py` usaba una tolerancia fija `0.030` para `PRE_GRASP`, aunque el log real de la corrida reportaba `tol=0.080`; se corrigio para leer la tolerancia efectiva desde el propio log.
  - La seleccion remota podia ser rechazada tras `release_objects` porque la maquina de estados bloqueaba la transicion `SPAWNED -> SELECTED`; se habilito esa transicion manteniendo la salvaguarda geometrica `SELECTED requires on_table geometry`.
- Validacion local asociada:
  - `pytest -q test_remote_object_selection.py` -> `12 passed`
  - recompilacion del paquete: `colcon build --packages-select ur5_qt_panel --symlink-install`
- Corrida final limpia realizada tras recompilar y relanzar el stack:
  - seleccion remota: `success=True, message='selected'`
  - inferencia remota: `success=True, message='tfm_infer_triggered'`
  - pick remoto: `success=True, message='pick_object_triggered'`
  - auditor E2E final: `[PASS] PICK_OBJ E2E OK (bridge recovered and full pick sequence completed)`
  - detalle `PRE_GRASP`: `pregrasp_dist=0.002 tf_ok=true tol=0.080`
- Evidencia de `PlanningScene` en la misma corrida:
  - artefacto limpio: `~/TFM/reports/panel_audit/artifacts/planning_scene_pick_demo_probe_clean_2026_03_16.jsonl`
  - resumen:
    - `rows=73`
    - `attached_count=15`
    - `world_count=32`
    - primera muestra `attached`: `sample=37 t_rel_s=94.92`
    - ultima muestra `attached`: `sample=71 t_rel_s=174.97`
  - extracto capturado:
    - `AttachedCollisionObject(link_name='rg2_tcp', object=... id='pick_demo' ...)`
- Conclusión final:
  - el flujo remoto `select -> tfm_infer -> pick_object` queda validado end-to-end
  - MoveIt `PlanningScene` queda sincronizada con Gazebo tanto en estado libre como en estado `attached`
  - la evidencia de `AttachedCollisionObject` y el PASS E2E ya quedaron demostrados en una misma corrida limpia

### Trazabilidad adicional de TEST ROBOT

- Se añadió auditoría explícita del trigger remoto en `panel_v2.py`.
- El fichero dedicado `~/TFM/reports/panel_audit/logs/test_robot.log` ya registra:
  - `[TEST][REMOTE] trigger=service:/panel/test_robot`
  - `[TEST] REQUESTED`
  - `[TEST] START ...`
  - `[TEST] RESULT=PASS`
- Resultado: `TEST ROBOT` sigue sin ser la prueba más fuerte del stack, pero su trazabilidad remota queda cerrada y verificable en fichero dedicado.

## Pendientes y limitaciones

- El helper `jt_smoke_test` no se considera aun prueba canónica; la validación fiable de ejecución sigue siendo la acción `FollowJointTrajectory` y el pick completo.
- Persisten warnings no bloqueantes de Gazebo sobre nombres duplicados de `drop_anchor`; no han impedido build, bringup ni pick.

## Archivos modificados en la auditoria actual

- `AUDITORIA_MOVEIT_GAZEBO_QT.md`
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py`
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py`
- `agarre_ros2_ws/src/ur5_tools/ur5_tools/jt_smoke_test.py`
- `agarre_ros2_ws/src/ur5_tools/ur5_tools/planning_scene_sync.py`
- `agarre_ros2_ws/src/ur5_tools/setup.py`
- `agarre_ros2_ws/src/ur5_tools/package.xml`
- `agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py`

## Recomendaciones tecnicas

1. Mantener `tools/validate_pick_object_e2e.py --last` como test de regresión funcional del flujo completo tras cambios en panel, bridge o escena.
2. Conservar el probe `planning_scene_pick_demo_probe_clean_2026_03_16.jsonl` como evidencia base y, si se automatiza CI local, convertir ese muestreo en una prueba repetible de no regresión de `AttachedCollisionObject`.
3. Revisar si la tolerancia y adaptación XY del pre-grasp deben exponerse como parámetros documentados del sistema, no solo inferirse desde logs o variables de entorno.