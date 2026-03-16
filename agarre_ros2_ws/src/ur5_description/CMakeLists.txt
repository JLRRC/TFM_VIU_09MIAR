# Ruta/archivo: agarre_ros2_ws/src/ur5_description/CMakeLists.txt
# Contenido: Descripcion URDF/Xacro y configuracion base del robot UR5.
# Uso breve: Se usa al publicar el robot model y al preparar simulacion y MoveIt.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description/CMakeLists.txt
# Summary: CMake build config for ur5_description package.
cmake_minimum_required(VERSION 3.8)
project(ur5_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
