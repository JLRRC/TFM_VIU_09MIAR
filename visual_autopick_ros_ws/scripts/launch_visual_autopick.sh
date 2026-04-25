#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/laboratorio/TFM/visual_autopick_ros_ws/install/setup.bash
ros2 launch visual_autopick_bringup visual_autopick.launch.py \
  use_moveit:=true use_panel:=true panel_mode:=simple use_attach:=false
