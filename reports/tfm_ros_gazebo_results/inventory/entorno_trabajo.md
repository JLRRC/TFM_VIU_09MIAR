# Entorno de trabajo

## Base documental usada

- Host actual donde reside el workspace y donde se han consultado las herramientas:
  - `uname -a`
  - `lsb_release -a`
  - `lscpu`
  - `nvidia-smi`
  - `source /opt/ros/jazzy/setup.bash && gz sim --versions`
  - `dpkg -l` para ROS 2, MoveIt y `ros_gz`
- Logs del propio proyecto:
  - `evidence/terminal/ros_gazebo_panel_block_smoke_test.log`
  - `evidence/terminal/ros_gazebo_pick_demo_run_final.log`

## Plataforma de trabajo identificada

- Sistema operativo:
  - Ubuntu 24.04.4 LTS (`noble`)
- Kernel:
  - `Linux laboratorio-MS-7C56 6.17.0-14-generic`
- CPU:
  - AMD Ryzen 7 5800X 8-Core Processor
  - 16 hilos visibles
- GPU disponible en el host:
  - NVIDIA GeForce RTX 4070
  - Driver `590.48.01`
  - Memoria `12282 MiB`

## ROS 2, Gazebo y librerias principales

- ROS 2:
  - ROS 2 Jazzy
  - Evidencia:
    - `agarre_ros2_ws/README.md`
    - `agarre_ros2_ws/scripts/01_install_ros2_jazzy.sh`
    - paquete instalado `ros-jazzy-desktop 0.11.0-1noble.20260126.203157`
- Gazebo:
  - Gazebo Sim `8.10.0`
  - Evidencia:
    - `gz sim --versions`
    - uso explicito de `gz sim` en `ur5_stack.launch.py`
- Integracion ROS 2 <-> Gazebo:
  - `ros-jazzy-ros-gz 1.0.18-1noble.20260126.203319`
  - `ros-jazzy-ros-gz-bridge 1.0.18-1noble.20260126.180325`
  - `ros-jazzy-ros-gz-sim 1.0.18-1noble.20260126.180616`
- MoveIt:
  - familia `2.12.4`
  - paquetes confirmados: `moveit-py`, `moveit-ros-move-group`, `moveit-ros-planning-interface`, `moveit-planners-ompl`
- Librerias Python comprobadas en el host:
  - `torch 2.10.0+cpu`
  - `cv2 4.6.0`
  - `numpy 1.26.4`

## Uso de CPU o GPU

- El codigo del stack soporta seleccion de dispositivo `cpu`, `cuda` o `auto`.
- El host dispone de GPU NVIDIA, pero los logs revisados no registran de forma explicita que la inferencia del caso principal usara CUDA.
- El interprete `python3` comprobado en este host carga `torch 2.10.0+cpu`.
- Afirmacion rigurosa:
  - ejecucion del pipeline de inferencia confirmada;
  - uso de CPU compatible y directamente corroborado;
  - uso efectivo de GPU no demostrado por la evidencia revisada.

## Evidencia de que este era el entorno de trabajo real

- `evidence/terminal/ros_gazebo_panel_block_smoke_test.log` fija:
  - `cwd=/home/laboratorio/TFM/agarre_ros2_ws`
  - `Python 3.12.3`
  - nodo ROS 2 operativo y topics del pipeline activos
- `evidence/terminal/ros_gazebo_pick_demo_run_final.log` muestra el arranque real de:
  - `gz`
  - `parameter_bridge`
  - `gz_pose_bridge`
  - `world_tf_publisher`
  - `system_state_manager`
  - `gripper_attach_backend`
  - panel ROS 2

## Limitaciones del inventario

- No se ha encontrado un archivo unico de "environment freeze" o `requirements.lock` que congele todo el entorno exacto de la corrida.
- La version exacta de todas las dependencias Python del panel no queda registrada en los logs.
- El inventario anterior es suficiente para describir el entorno operativo principal, pero no para una reproduccion bit a bit.
