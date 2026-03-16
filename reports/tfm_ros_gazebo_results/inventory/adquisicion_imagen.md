# Adquisicion de imagen simulada

## Escena y sensor principal

La escena de simulacion usada en el workspace valido es:

- Mundo:
  - `/home/laboratorio/TFM/agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf`

Sensor principal usado en el caso mas fuerte:

- Modelo de camara:
  - `camera_overhead`
- Sensor RGB:
  - `camera_overhead_rgb`
- Tipo:
  - `camera`
- Topic:
  - `/camera_overhead/image`
- FOV horizontal:
  - `0.75`
- Resolucion:
  - `320 x 240`
- Formato:
  - `R8G8B8`

Tambien existe camara de profundidad asociada:

- Sensor:
  - `camera_overhead_depth`
- Tipo:
  - `depth_camera`
- Topic:
  - `/camera_overhead/depth_image`
- Resolucion:
  - `320 x 240`

## Otras camaras disponibles en la escena

La escena expone tambien:

- `/camera_north/image`
- `/camera_north/depth_image`
- `/camera_east/image`
- `/camera_east/depth_image`
- `/camera_south/image`
- `/camera_west/image`

## Como se puentea la imagen desde Gazebo a ROS 2

La adquisicion no es directa desde Gazebo al panel, sino a traves de `ros_gz_bridge`.

Evidencia de configuracion:

- `/home/laboratorio/TFM/agarre_ros2_ws/scripts/bridge_cameras.yaml`
  - mapea `GZ_TO_ROS` para `/camera_overhead/image`
  - mapea `GZ_TO_ROS` para `/camera_overhead/depth_image`
- `/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py`
  - arranca `ros_gz_bridge_main`

Evidencia de ejecucion:

- `evidence/terminal/ros_gazebo_pick_demo_run_final.log`
  - crea el bridge:
    - `/camera_overhead/image -> /camera_overhead/image`
    - `/camera_overhead/depth_image -> /camera_overhead/depth_image`
- `evidence/terminal/ros_gazebo_panel_block_smoke_test.log`
  - lista `/camera_overhead/image` y `/camera_overhead/depth_image` entre los topics activos

## Nodo o componente que consume la imagen

- El panel `ur5_qt_panel/panel_v2.py` trabaja por defecto con:
  - `camera_topic: /camera_overhead/image`
- `system_state_manager.yaml` tambien declara:
  - `camera_topic: /camera_overhead/image`

## Evidencia visual disponible

- Imagen capturada desde la escena simulada:
  - `evidence/images/ros_gazebo_imagen_camara.png`
- Captura de frame usada en el panel:
  - `evidence/images/ros_gazebo_frame_panel_infer.png`

## Observaciones y limites

- En la evidencia revisada no aparece fijado de forma explicita el `frame_id` ROS de la imagen dentro del mensaje `sensor_msgs/Image`.
- Lo que si queda claramente fijado es el topic de adquisicion y el uso de esa imagen por el panel para inferencia.
