# Pose / hipotesis de agarre y publicacion

## Forma de la salida del detector

La inferencia del panel produce una hipotesis de agarre en coordenadas de imagen con los campos:

- `cx`
- `cy`
- `w`
- `h`
- `angle_deg`

Evidencia:

- `evidence/terminal/ros_gazebo_infer.log`
- `evidence/terminal/ros_gazebo_grasp_last.json`

Ejemplo documentado en `grasp_last.json` del `2026-03-15T19:04:31`:

- `cx = 153.0290`
- `cy = 124.8542`
- `w = 22.1831`
- `h = 16.1914`
- `angle_deg = -2.4203`

## Conversion a coordenadas utilizables por el robot

El panel transforma esa salida a una hipotesis en el marco base del robot:

- `grasp_base.x`
- `grasp_base.y`
- `grasp_base.z`
- `grasp_base.yaw_deg`

Ejemplo documentado:

- `x = 0.5973872102997781`
- `y = 0.02005852400336036`
- `z = 0.07499999999999996`
- `yaw_deg = -83.34943276889119`

## Como se obtiene la pose

Segun `ur5_qt_panel/panel_v2.py`, el flujo es:

- guardar la prediccion en pixeles;
- calcular una pose en mundo con `_compute_world_grasp(...)`;
- transformar a `base_link` con `_ensure_base_coords(...)`;
- almacenar el resultado en `_last_grasp_base`;
- persistirlo en `artifacts/grasp_last.json`.

## Publicacion ROS 2 involucrada

Contrato del panel:

- publica `PoseStamped` en:
  - `/desired_grasp`
  - `/desired_grasp_cartesian`
- usa `/grasp_rect` como canal visual o de hipotesis de rectangulo

Contrato del consumidor:

- `ur5_moveit_bridge.py` se suscribe a:
  - `/desired_grasp`
  - `/grasp_pose`
  - `/desired_grasp_cartesian`
- publica resultados en:
  - `/desired_grasp/result`

Tipos confirmados por codigo:

- `/desired_grasp`:
  - `geometry_msgs/msg/PoseStamped`
- `/desired_grasp/result`:
  - `std_msgs/msg/String`

## Evidencia de que la publicacion y la hipotesis existieron

- `evidence/terminal/ros_gazebo_panel_block_smoke_test.log`
  - confirma la presencia de:
    - `/desired_grasp`
    - `/desired_grasp_cartesian`
    - `/grasp_rect`
- `evidence/terminal/ros_gazebo_visualize.log`
  - registra overlays correctos en `overlay_last.png`
- `evidence/images/ros_gazebo_overlay_prediccion.png`
  - muestra la hipotesis dibujada sobre la imagen simulada
- `evidence/terminal/ros_gazebo_grasp_last.json`
  - demuestra que la prediccion ya estaba convertida a una pose base consumible por el robot

## Nota de rigor

- La forma exacta del mensaje de `/grasp_rect` no queda fijada de una sola manera en la evidencia revisada porque el panel lo descubre dinamicamente.
- Para el TFM no hace falta forzar ese detalle: lo importante es que la hipotesis visual existe, se registra y se acompana de una pose base que despues se publica para MoveIt.
