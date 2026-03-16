# Secuencia del pipeline ROS 2 + Gazebo

## Secuencia tecnica documentada

### 1. Arranque del entorno simulado

- `ros2 launch ur5_bringup ur5_stack.launch.py`
- El launch principal arranca:
  - `gz sim`
  - `ros_gz_bridge_main`
  - `gz_pose_bridge`
  - `world_tf_publisher`
  - `system_state_manager`
  - `gripper_attach_backend`
  - panel ROS 2
- Evidencia:
  - `evidence/terminal/ros_gazebo_pick_demo_run_final.log`

### 2. Escena y sensores disponibles

- Se carga el mundo `ur5_mesa_objetos`.
- Gazebo expone la camara principal `/camera_overhead/image` y su profundidad `/camera_overhead/depth_image`.
- `ros_gz_bridge` las lleva a ROS 2.
- Evidencia:
  - `evidence/terminal/ros_gazebo_pick_demo_run_final.log`
  - `evidence/terminal/ros_gazebo_panel_block_smoke_test.log`
  - `evidence/images/ros_gazebo_imagen_camara.png`

### 3. Aplicacion del modelo

- Desde el panel se selecciona un checkpoint.
- El caso principal usa `EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth`.
- El panel registra la carga en `apply_experiment.log`.
- Evidencia:
  - `evidence/terminal/ros_gazebo_apply_experiment.log`

### 4. Inferencia sobre la imagen simulada

- El panel lanza la inferencia sobre `/camera_overhead/image`.
- La salida se registra en pixeles como `(cx, cy, w, h, angle_deg)`.
- Evidencia:
  - `evidence/terminal/ros_gazebo_infer.log`
  - `evidence/terminal/ros_gazebo_grasp_last.json`

### 5. Conversion de la hipotesis a una pose util para el robot

- El panel calcula `grasp_world` y despues `grasp_base`.
- El resultado queda almacenado en `grasp_last.json`.
- Evidencia:
  - `evidence/terminal/ros_gazebo_grasp_last.json`
  - `inventory/pose_y_publicacion.md`

### 6. Publicacion ROS 2

- El panel publica una `PoseStamped` en `/desired_grasp` y puede usar `/desired_grasp_cartesian`.
- Tambien mantiene la hipotesis visual via `/grasp_rect`.
- Evidencia:
  - codigo `panel_v2.py`
  - `evidence/terminal/ros_gazebo_panel_block_smoke_test.log`
  - `evidence/images/ros_gazebo_overlay_prediccion.png`

### 7. Consumo por MoveIt

- `ur5_moveit_bridge` esta suscrito a `/desired_grasp` y publica resultado en `/desired_grasp/result`.
- En el caso principal aparecen varios `execute OK mode=moveit_sequence source=infer_model`.
- Evidencia:
  - `evidence/terminal/ros_gazebo_execute.log`
  - `evidence/terminal/ros_gazebo_panel_block_smoke_test.log`

### 8. Resultado observable

- Resultado fuerte del pipeline integrado:
  - la salida inferida llega al consumidor y desencadena secuencias MoveIt correctas.
- Resultado fuerte del stack de manipulacion simulado:
  - existe ademas una demo manual completa con `GRASPED -> CARRIED -> RELEASED`.
- Evidencia:
  - `evidence/terminal/ros_gazebo_execute.log`
  - `evidence/terminal/ros_gazebo_pick_demo_run_final.log`

## Secuencia resumida en una linea

`Gazebo world -> camera_overhead/image -> inferencia del panel -> grasp_base -> /desired_grasp -> ur5_moveit_bridge -> secuencia MoveIt`

## Punto que debe quedar claro en la redaccion futura

La secuencia anterior esta validada funcionalmente. Lo que no queda cerrado con igual fuerza es que la maniobra completa de agarre y transporte del objeto en el caso inferido sea la misma que la documentada en la demo manual.
