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

# F4 (auditoría 2026-05-10): linters Python sobre test/.
if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  ament_flake8(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg)
  ament_pep257()
endif()

ament_package()
