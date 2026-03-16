# Casos candidatos de integracion ROS 2 + Gazebo

## Criterio de seleccion

Se priorizan casos que permitan demostrar, con el menor salto interpretativo posible:

- escena simulada operativa;
- adquisicion de imagen;
- ejecucion del modelo;
- obtencion de hipotesis de agarre o pose objetivo;
- publicacion ROS 2;
- consumo por otro componente;
- evidencia de movimiento o de estado final.

## Caso 1. Pipeline principal con modelo e integracion a MoveIt

- Fecha mas fuerte:
  - 2026-03-15
- Evidencia principal:
  - `evidence/terminal/ros_gazebo_apply_experiment.log`
  - `evidence/terminal/ros_gazebo_infer.log`
  - `evidence/terminal/ros_gazebo_execute.log`
  - `evidence/terminal/ros_gazebo_grasp_last.json`
  - `evidence/images/ros_gazebo_overlay_prediccion.png`
- Hechos soportados:
  - el checkpoint `EXP3_RESNET18_RGB_AUGMENT/seed_0` se carga correctamente;
  - el panel infiere sobre `/camera_overhead/image`;
  - la salida se convierte a `grasp_base`;
  - hay publicacion hacia `/desired_grasp`;
  - existe consumidor MoveIt y aparecen varios `execute OK mode=moveit_sequence source=infer_model`.
- Ejemplos claros:
  - `2026-03-15T16:36:06`
  - `2026-03-15T16:37:19`
  - `2026-03-15T16:47:45`
  - `2026-03-15T16:48:37`
- Valor para el TFM:
  - es el mejor caso para una subseccion de resultados de integracion ROS 2 + Gazebo ligada al modelo de percepcion.
- Limitacion:
  - no hay evidencia igual de fuerte que conecte esos `execute OK` con un agarre fisico exitoso y traslado completo del objeto.

## Caso 2. Demo manual de manipulacion completa en el stack simulado

- Fecha:
  - 2026-03-11
- Evidencia principal:
  - `evidence/terminal/ros_gazebo_pick_demo_run_final.log`
- Hechos soportados:
  - el stack Gazebo + ROS 2 arranca correctamente;
  - el robot recorre la secuencia `HOME -> MESA -> PICK_IMAGE -> GRASP_DOWN_JOINT -> MESA_POST_GRASP -> HOME_WITH_OBJECT -> CESTA -> CESTA_RELEASE -> HOME_FINAL`;
  - los estados del objeto `pick_demo` cambian a `GRASPED`, `CARRIED`, `RELEASED`;
  - el log termina con `Secuencia PICK completada`.
- Valor para el TFM:
  - demuestra que el stack de manipulacion en simulacion funciona.
- Limitacion:
  - no es una manipulacion guiada por la salida del modelo; es una secuencia demo/manual del stack.

## Caso 3. Verificacion intermedia de publicacion sin movimiento

- Fecha:
  - 2026-03-15 entre 15:11 y 15:34
- Evidencia principal:
  - `evidence/terminal/ros_gazebo_execute.log`
- Hecho soportado:
  - aparece `execute OK action=publish_only note=no_motion_implemented`.
- Valor:
  - demuestra un estadio intermedio del pipeline donde la publicacion ya funcionaba antes de consolidar el consumidor MoveIt.
- Limitacion:
  - insuficiente como caso principal de resultados.

## Caso principal seleccionado

Se selecciona como caso principal para el dossier:

- Caso 1. Pipeline principal con modelo e integracion a MoveIt

Motivo:

- es el unico que conecta de forma documentada imagen simulada, modelo, hipotesis de agarre, publicacion ROS 2 y consumo mediante secuencia MoveIt.

## Caso complementario recomendado

Conviene mencionar como apoyo, pero no confundir con el caso principal:

- Caso 2. Demo manual de manipulacion completa en el stack simulado

Motivo:

- permite afirmar que el stack de manipulacion en Gazebo funciona, aunque no cierre por si solo la demostracion de una manipulacion completa guiada por percepcion.
