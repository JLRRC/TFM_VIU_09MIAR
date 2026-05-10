# Ruta/archivo: agarre_ros2_ws/src/ur5_gazebo/CMakeLists.txt
# Contenido: F5 audit (2026-05-10) — assets Gazebo Harmonic empaquetados.
# Uso breve: instala worlds/, models/, config/ y bridges/ en share/ur5_gazebo.
cmake_minimum_required(VERSION 3.8)
project(ur5_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
