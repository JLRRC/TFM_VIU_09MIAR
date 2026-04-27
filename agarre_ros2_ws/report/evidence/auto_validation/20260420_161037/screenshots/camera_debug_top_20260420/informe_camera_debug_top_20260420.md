# Informe fix camera_debug_top 2026-04-20

## 1. Causa raiz

- La camara `camera_debug_top` estaba definida en `agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf` como modelo estatico del world.
- El sensor RGB publicaba correctamente en `/camera_debug_top/image` y el depth en `/camera_debug_top/depth_image`, ambos a `640x480`.
- El problema no era bridge, render headless, topico ni clipping: la pose tenia `pitch=-1.5708`.
- En Gazebo Sim, el sensor de camara usa el eje local `+X` como forward. Con `rpy = 0 -1.5708 0`, el forward queda alineado con `+Z`, es decir, la camara miraba hacia arriba.
- Evidencia previa al fix: la captura `before/125440_camera_debug_top_periodic.jpg` sale completamente blanca. Medida objetiva: media RGB `(249.0, 249.0, 249.0)` y `bright>=245 = 1.0`.

## 2. Localizacion exacta

- Modelo: `camera_debug_top`
- Archivo fuente: `agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf`
- Runtime world: `agarre_ros2_ws/log/world_runtime.sdf`
- Parent model: `camera_debug_top`
- Parent link: `camera_debug_top::link`
- Frame RGB real: `camera_debug_top/link/camera_debug_top_rgb`
- Frame depth real: `camera_debug_top/link/camera_debug_top_depth`
- Topic RGB: `/camera_debug_top/image`
- Topic depth: `/camera_debug_top/depth_image`
- Resolucion: `640x480`
- FOV horizontal: `1.2 rad`
- Clip: `near=0.05`, `far=10.0`

## 3. Cambio aplicado

Antes:

- Pose: `-0.50 0.00 2.35 0 -1.5708 0`
- Diagnostico: forward hacia `+Z` (cielo)

Iteracion 1:

- Pose: `-0.50 0.00 2.35 0 1.5708 0`
- Resultado: la imagen deja de salir blanca y aparece una vista cenital funcional, pero el robot quedaba algo justo en el borde inferior.

Definitivo:

- Pose: `-0.60 0.00 2.55 0 1.5708 0`
- Se mantiene `horizontal_fov=1.2`, `near=0.05`, `far=10.0`.
- Razon: sesgo ligero hacia el robot y aumento de altura para ver mesa completa, UR5 entero y `pick_demo` en la misma toma.

## 4. Validacion

- `/camera_debug_top/image`: publisher `ros_gz_bridge_main`, tipo `sensor_msgs/msg/Image`
- `/camera_debug_top/depth_image`: publisher `ros_gz_bridge_main`, tipo `sensor_msgs/msg/Image`
- RGB final recibido:
  - `frame_id = camera_debug_top/link/camera_debug_top_rgb`
  - `size = 640x480`
  - `encoding = rgb8`
- Depth final recibido:
  - `frame_id = camera_debug_top/link/camera_debug_top_depth`
  - `size = 640x480`
  - `encoding = 32FC1`
- La captura final `after/130911_camera_debug_top_periodic.jpg` ya no esta saturada. Medida objetiva: media RGB `(204.7, 204.93, 204.97)` y `bright>=245 = 0.0001`.
- Inspeccion visual final:
  - se ve la mesa completa,
  - se ve el UR5 en vista superior,
  - se ve la pinza,
  - se ve la zona de grasp,
  - se ve `pick_demo`.

## 5. Evidencia

Capturas antes:

- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/before/125440_camera_debug_top_periodic.jpg`
- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/before/125440_camera_overhead_periodic.jpg`
- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/before/launch_before.log`

Capturas iteracion 1:

- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/after/130128_camera_debug_top_periodic.jpg`
- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/after/launch_after_iter1.log`

Capturas definitivas:

- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/after/130911_camera_debug_top_periodic.jpg`
- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/after/130911_camera_overhead_periodic.jpg`
- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/after/130911_camera_north_periodic.jpg`
- `/home/laboratorio/TFM/historico/camera_debug_top_20260420/after/launch_after_iter2.log`

## 6. Recomendacion final

- Dejar `camera_debug_top` como camara estatica en el world, no atada al robot.
- Conservar la pose definitiva `-0.60 0.00 2.55 0 1.5708 0`.
- Mantener `horizontal_fov=1.2`, `640x480`, `near=0.05`, `far=10.0`.
- No tocar el render engine: el headless con EGL y `ogre2` ya funciona; el fallo era puramente geometrico por orientacion.