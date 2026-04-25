#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
cd /home/laboratorio/TFM/visual_autopick_ros_ws
colcon build --symlink-install
echo "Build completo. Source con: source install/setup.bash"
