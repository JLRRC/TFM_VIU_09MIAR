# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/CMakeLists.txt
# Contenido: Configuracion de bringup ROS 2 para lanzar el sistema UR5.
# Uso breve: Colcon/ros2 launch lo usan para arrancar simulacion y componentes principales.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/CMakeLists.txt
# Summary: CMake build config for ur5_bringup package.
cmake_minimum_required(VERSION 3.8)
project(ur5_bringup)

find_package(ament_cmake REQUIRED)

# Instala launch/ y config/ en share/<pkg>
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
