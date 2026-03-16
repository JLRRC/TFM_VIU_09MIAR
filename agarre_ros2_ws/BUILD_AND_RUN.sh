#!/bin/bash
# Script para compilar y ejecutar el nodo de inferencia ROS2

set -e

ROS2_WS="$HOME/TFM/agarre_ros2_ws"
VISION_DIR="$HOME/TFM/agarre_inteligente"

echo "🔨 Compilando paquete tfm_grasping..."
cd "$ROS2_WS"
source /opt/ros/jazzy/setup.bash

colcon build --packages-select tfm_grasping --symlink-install

echo ""
echo "✅ Compilación completada"
echo ""
echo "Para ejecutar el nodo:"
echo ""
echo "  source $ROS2_WS/install/setup.bash"
echo "  export VISION_DIR=$VISION_DIR"
echo "  ros2 run tfm_grasping grasp_inference"
echo ""
echo "Para ver predicciones en tiempo real (otra terminal):"
echo "  source $ROS2_WS/install/setup.bash"
echo "  ros2 topic echo /tfm/grasp_prediction"
echo ""
