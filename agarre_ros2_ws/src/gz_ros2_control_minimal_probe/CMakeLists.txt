cmake_minimum_required(VERSION 3.8)
project(gz_ros2_control_minimal_probe)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
