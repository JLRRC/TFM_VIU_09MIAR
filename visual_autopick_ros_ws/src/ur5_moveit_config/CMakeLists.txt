# Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/CMakeLists.txt
# Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion.
# Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot.
cmake_minimum_required(VERSION 3.22)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_msgs REQUIRED)

ament_package()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
