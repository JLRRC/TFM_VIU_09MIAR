cmake_minimum_required(VERSION 3.8)
project(ur5_panel_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SelectObject.srv"
  "srv/Open.srv"
  "srv/Close.srv"
  "srv/SetWidth.srv"
  "srv/Attach.srv"
  "srv/Detach.srv"
  "srv/WorldToBase.srv"
  "srv/ComputeApproachPose.srv"
  "srv/ResolveObjectPoseWorld.srv"
  "action/PickPlace.action"
  "action/PlanToPose.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs)
ament_package()