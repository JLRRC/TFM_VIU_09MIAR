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

# F4 (auditoría 2026-05-10): linters Python sobre launch/ y test/.
# Configuración de tolerancias en setup.cfg (deuda histórica documentada).
if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  ament_flake8(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg)
  ament_pep257()
endif()

ament_package()
