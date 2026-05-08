#!/usr/bin/env python3
"""F6 audit-v4 (2026-05-08): launcher dedicado para grasp_selector_node.

Lanza el ROS 2 wrapper sobre ``tfm_grasping.grasp_selector`` (puro). Aún
NO se mete en ur5_stack.launch.py por defecto — primero hay que validar
live. Este launcher permite iniciar el nodo en paralelo al stack para
testing.

Uso:
    ros2 launch ur5_bringup grasp_selector.launch.py
    ros2 launch ur5_bringup grasp_selector.launch.py ws_radius_m:=0.90 \\
        require_tcp_down:=false

Topics esperados (defaults):
- Sub: ~/grasp_candidates           (PoseArray)
- Sub: ~/grasp_candidate_scores     (Float32MultiArray)
- Pub: ~/best_grasp                  (PoseStamped)
- Pub: ~/grasp_selector_status       (String JSON)
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ws_radius_m = LaunchConfiguration("ws_radius_m")
    ws_min_radius_m = LaunchConfiguration("ws_min_radius_m")
    tcp_down_tolerance_rad = LaunchConfiguration("tcp_down_tolerance_rad")
    require_tcp_down = LaunchConfiguration("require_tcp_down")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ws_radius_m",
            default_value="0.85",
            description="Workspace max radius (UR5+RG2). Default: 0.85m",
        ),
        DeclareLaunchArgument(
            "ws_min_radius_m",
            default_value="0.20",
            description="Workspace min radius (singularity zone). Default: 0.20m",
        ),
        DeclareLaunchArgument(
            "tcp_down_tolerance_rad",
            default_value="0.20",
            description="TCP-down tolerance ~11.5°. Default: 0.20",
        ),
        DeclareLaunchArgument(
            "require_tcp_down",
            default_value="true",
            description="Apply TCP-down filter. Default: true",
        ),
        Node(
            package="tfm_grasping",
            executable="grasp_selector_node",
            name="grasp_selector_node",
            output="screen",
            parameters=[{
                "ws_radius_m": ws_radius_m,
                "ws_min_radius_m": ws_min_radius_m,
                "tcp_down_tolerance_rad": tcp_down_tolerance_rad,
                "require_tcp_down": require_tcp_down,
            }],
        ),
    ])
