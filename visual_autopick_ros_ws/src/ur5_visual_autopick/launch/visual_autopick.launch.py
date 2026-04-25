"""Launch file for visual_autopick — autopick panel + comprehensive trace logger."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("ur5_visual_autopick")
    default_config = os.path.join(pkg, "config", "visual_autopick.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to visual_autopick YAML config",
    )

    autopick_node = Node(
        package="ur5_visual_autopick",
        executable="visual_autopick_node",
        name="visual_autopick_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("config")],
    )

    trace_logger_node = Node(
        package="ur5_visual_autopick",
        executable="ros_trace_logger",
        name="ros_trace_logger",
        output="screen",
        emulate_tty=True,
        additional_env={"ROS_TRACE_LOG_DIR": EnvironmentVariable(
            "ROS_TRACE_LOG_DIR",
            default_value="/tmp/ros_trace_latest"
        )},
    )

    return LaunchDescription([config_arg, autopick_node, trace_logger_node])
