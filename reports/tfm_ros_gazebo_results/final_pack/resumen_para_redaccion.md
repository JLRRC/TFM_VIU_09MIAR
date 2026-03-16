# Resumen para redaccion de la subseccion ROS 2 + Gazebo

## Titulo de subseccion propuesto

- `Resultados de integracion en ROS 2 + Gazebo`

## Workspace ROS 2 valido

- Ruta:
  - `/home/laboratorio/TFM/agarre_ros2_ws`
- Workspace historico no seleccionado:
  - `/home/laboratorio/TFM/agarre_ros2_ws_hisorico`
- Criterio:
  - logs recientes, capturas, arranque documentado y `cwd` de auditoria apuntando al workspace principal.

## Entorno de trabajo

- SO:
  - Ubuntu 24.04.4 LTS
- ROS 2:
  - Jazzy
- Gazebo:
  - Gazebo Sim 8.10.0
- CPU:
  - AMD Ryzen 7 5800X
- GPU disponible:
  - NVIDIA RTX 4070
- Nota:
  - el uso efectivo de CPU esta corroborado;
  - el uso efectivo de GPU no queda demostrado en los logs revisados.

## Modelo usado en el caso principal

- Experimento:
  - `EXP3_RESNET18_RGB_AUGMENT`
- Seed:
  - `seed_0`
- Checkpoint:
  - `/home/laboratorio/TFM/agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth`
- Arquitectura:
  - ResNet18 RGB, `in_channels=3`, `img_size=224`

## Como se adquirio la imagen

- Mundo:
  - `/home/laboratorio/TFM/agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf`
- Topic principal de camara:
  - `/camera_overhead/image`
- Resolucion:
  - `320 x 240`
- Bridge:
  - `ros_gz_bridge_main`
  - configurado en `scripts/bridge_cameras.yaml`

## Como se obtuvo y publico la hipotesis

- El panel infiere una pinza 2D en pixeles:
  - `cx, cy, w, h, angle_deg`
- Despues la convierte a `grasp_base`:
  - `x, y, z, yaw_deg`
- Artefacto principal:
  - `evidence/terminal/ros_gazebo_grasp_last.json`
- Topic de publicacion para MoveIt:
  - `/desired_grasp`
- Topic de resultado del consumidor:
  - `/desired_grasp/result`

## Que consume la salida

- Nodo consumidor:
  - `ur5_moveit_bridge`
- Funcion:
  - suscribirse a `PoseStamped` en `/desired_grasp`
  - ejecutar planificacion y secuencia MoveIt
  - publicar estado/resultados en `/desired_grasp/result`

## Mejor caso de exito seleccionado

- Caso principal:
  - pipeline con modelo e integracion a MoveIt del 2026-03-15
- Evidencia fuerte:
  - `apply_experiment OK`
  - `infer_end status=OK`
  - `grasp_base` disponible
  - varios `execute OK mode=moveit_sequence source=infer_model`

## Nivel de evidencia

- Nivel global recomendado para la subseccion:
  - `B`
- Motivo:
  - hay validacion funcional `percepcion -> publicacion -> consumo`;
  - no hay prueba igual de cerrada de agarre completo asociado al mismo caso inferido.

## Capturas e imagenes recomendadas

- escena / frame:
  - `/home/laboratorio/TFM/agarre_inteligente/reports/tfm_ros_gazebo_results/evidence/images/ros_gazebo_imagen_camara.png`
  - `/home/laboratorio/TFM/agarre_inteligente/reports/tfm_ros_gazebo_results/evidence/images/ros_gazebo_frame_panel_infer.png`
- overlay de prediccion:
  - `/home/laboratorio/TFM/agarre_inteligente/reports/tfm_ros_gazebo_results/evidence/images/ros_gazebo_overlay_prediccion.png`
- comparativa simple:
  - `/home/laboratorio/TFM/agarre_inteligente/reports/tfm_ros_gazebo_results/evidence/images/ros_gazebo_exp3_vs_referencia.png`

## Estructura interna sugerida para redactar despues

- 1. Entorno ROS 2 + Gazebo utilizado
- 2. Escena y adquisicion de imagen simulada
- 3. Modelo integrado en el panel
- 4. Obtencion y publicacion de la hipotesis de agarre
- 5. Consumo por MoveIt y validacion funcional del pipeline
- 6. Limites de la evidencia y alcance de la validacion

## Bloqueos o incertidumbres

- No hay bloqueo tecnico para redactar una subseccion solida de resultados.
- La principal cautela editorial es no presentar la demo manual de `pick_demo` como si fuera un exito completo guiado por la red.
- Si se quisiera afirmar exito manipulativo autonomo completo, haria falta evidencia adicional que una de forma directa la prediccion inferida con los estados finales del objeto.
