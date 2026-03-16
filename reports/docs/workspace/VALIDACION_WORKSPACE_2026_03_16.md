# Validacion Integral del Workspace - 2026-03-16

## Alcance

Se valida el estado real de:

- `agarre_inteligente`
- `agarre_ros2_ws`
- coherencia con la memoria TFM v10
- flujo de control ROS 2 + Gazebo + MoveIt + panel TFM

La validacion se ha ejecutado por fases, distinguiendo entre:

- conformidad del codigo y artefactos
- conformidad del entorno
- conformidad del flujo robotico real

## Fase 1. `agarre_inteligente`

### Verificaciones ejecutadas

- `./check_system.sh`
- `source venv/bin/activate && ./check_python_deps.sh`
- `source venv/bin/activate && ./check_project.sh`
- `source venv/bin/activate && python3 scripts/validate_artifacts.py --strict`
- `./project_status.sh`

### Resultado

- El entorno del proyecto es correcto cuando se usa su `venv/`.
- Las dependencias Python del proyecto pasan correctamente.
- Los artefactos minimos exigidos por el TFM estan presentes.
- Los resultados agregados publicados coinciden con `reports/tables/summary_results.csv`.

### Hallazgos

- Se actualizo `check_project.sh` para reflejar la estructura real vigente del proyecto.
- Se ha regenerado `reports/bench/latency_results.csv` y su tabla derivada `reports/tables/table_latency.csv` usando los checkpoints `best.pth` de `seed_0` para `EXP1..EXP4`.
- Los archivos `test_2_epochs.py` y `test_evaluator_fix.py` son scripts pesados ejecutados en importacion; no se comportan como tests unitarios ligeros de CI.

## Fase 2. `agarre_ros2_ws` estatico

### Verificaciones ejecutadas

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q test_panel_settings_infer_roi.py test_tfm_preprocessed_decode.py test_panel_ui_state_tfm.py`
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q test_panel_tfm_module_artifacts.py test_remote_object_selection.py`
- `bash scripts/01_validate_installation.sh`
- `bash scripts/panel_block_smoke_test.sh`

### Resultado

- Los tests unitarios rapidos del panel y del bloque TFM pasan.
- La instalacion ROS 2 Jazzy y MoveIt se considera valida.
- El smoketest del bloque TFM encuentra checkpoints y modulo de inferencia.

### Hallazgos

- `test_camera_auto_reconnect.py` no podia ni recolectarse al principio por falta de PyQt5 en el entorno del panel.
- El wrapper del panel apuntaba a un venv inexistente y se corrigio.

## Fase 3. Control robotico base con MoveIt

### Verificaciones ejecutadas

- `ros2 launch ur5_bringup ur5_stack.launch.py launch_panel:=false launch_moveit:=true headless:=true`
- `bash scripts/validate_panel_flow.sh`
- inspeccion de nodos y servicios ROS 2

### Resultado

El stack base de control queda validado:

- `/clock` publica
- `/world/ur5_mesa_objetos/pose/info` publica
- TF `world -> base_link` correcto
- TF `base_link -> tool0` correcto
- `/camera_overhead/image` publica
- `controller_manager` operativo
- `move_group` operativo
- `/release_objects` disponible
- `jt_smoke_test` correcto

### Conformidad

La parte de control del robot mediante MoveIt y ros2_control es conforme en este workspace.

## Fase 4. Panel TFM e inferencia ROS 2

### Correcciones aplicadas durante la validacion

- Se corrigio el venv objetivo del panel:
  - `agarre_ros2_ws/scripts/start_panel_v2.sh`
  - `agarre_ros2_ws/scripts/start_panel_v2_venv.sh`
- Se hizo robusto `start_panel_v2_venv.sh` al `source` de `install/setup.bash` con `set -u`.
- Se completo el venv del panel con dependencias necesarias para arrancar:
  - `PyQt5`
  - `PyYAML`
  - `pandas`
- Se alineo `numpy` del venv del panel a `1.26.4` para compatibilidad binaria.

### Verificaciones ejecutadas

- import directo de `ur5_qt_panel.panel_v2` con ROS cargado
- arranque offscreen del panel con `start_panel_v2_venv.sh`
- comprobacion de servicios `/panel/*`
- llamada remota a `/panel/tfm_infer`
- seleccion remota de objeto con `/panel/select_object`
- extension del panel con trigger remoto `/panel/pick_object`
- ejecucion remota completa `select -> tfm_infer -> pick_object`

### Resultado

El panel queda operativo a nivel ROS:

- `/panel/select_object`
- `/panel/test_robot`
- `/panel/tfm_infer`
- `/panel/pick_object`

La llamada remota a `/panel/tfm_infer` responde:

- `success=True`
- `message='tfm_infer_triggered'`

La llamada remota a `/panel/pick_object` responde:

- `success=True`
- `message='pick_object_triggered'`

### Hallazgo importante

La evidencia fuerte ya queda cerrada en esta sesion:

- se forzo `INFER_CKPT` hacia `EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth`
- la inferencia remota genero artefactos reales en `agarre_ros2_ws/log/panel_infer/`
- el log del stack muestra publicacion y ejecucion MoveIt sobre `/desired_grasp`
- el flujo de pick alcanzo `CARRIED`, despues `RELEASED`, y termino con `=== SECUENCIA COMPLETADA EXITOSAMENTE ===`

Por tanto:

- el panel esta funcional
- el disparo remoto existe y opera
- la inferencia efectiva queda demostrada con artefactos persistidos
- el flujo `modelo seleccionado -> inferencia -> grasp -> pick -> release` queda validado en vivo

## Fase 5. Conformidad con el TFM v10

### Verificacion documental

Se contrasto:

- `agarre_inteligente/docs/TFM_Jesus_Lozano_V10.pdf`
- `agarre_inteligente/docs/TRAZABILIDAD_TFM.md`
- `agarre_inteligente/docs/EXPERIMENTS.md`
- inventario de integracion ROS 2 + Gazebo en `reports/tfm_ros_gazebo_results/`

### Resultado

- Las metricas agregadas del bloque experimental son coherentes con los CSV vigentes.
- La parte metodologica y el uso de `EXP1..EXP4` es consistente con el repo.
- La parte ROS 2 + MoveIt es defendible como demostrador de integracion.

### Brecha documental detectada

La memoria v10 mezcla dos referencias distintas para el caso principal de integracion:

- partes del PDF apuntan a `EXP3_RESNET18_RGB_AUGMENT` como caso principal
- una tabla del anexo sigue mencionando `EXP4_RESNET18_RGBD` como experimento de referencia de integracion ROS 2

Esto deberia unificarse antes de considerar la memoria plenamente alineada con la evidencia del workspace.

## Veredicto global

### Conforme

- `agarre_inteligente` como bloque experimental reproducible
- `agarre_ros2_ws` como stack ROS 2 + Gazebo + MoveIt + control base
- disponibilidad del panel TFM y de sus servicios ROS tras corregir el entorno
- flujo completo remoto `seleccion -> inferencia -> pick -> release` validado en el stack vivo

### No plenamente conforme aun

- coherencia final EXP3/EXP4 en la narrativa del PDF v10

## Archivos modificados durante la validacion

- `agarre_ros2_ws/scripts/start_panel_v2.sh`
- `agarre_ros2_ws/scripts/start_panel_v2_venv.sh`
- `agarre_inteligente/check_project.sh`
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ros.py`
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py`
- `agarre_ros2_ws/tools/validate_pick_object_e2e.py`

## Recomendacion inmediata

La validacion funcional principal queda cerrada. Los siguientes pasos recomendables son:

1. unificar en la memoria final del TFM la referencia principal entre `EXP3` y `EXP4`
2. mantener el auditor `validate_pick_object_e2e.py` como comprobacion de regresion del flujo remoto completo
