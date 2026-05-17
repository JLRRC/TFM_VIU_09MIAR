"""Minimal probe launch for gz_ros2_control validation.

Spawns a single-joint revolute robot in Gazebo Harmonic with gz_ros2_control,
forward_position_controller, and verifies the joint moves when a position
command is sent.
"""
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gz_ros2_control_minimal_probe")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    urdf_xacro = PathJoinSubstitution([pkg_share, "urdf", "minimal_revolute.urdf.xacro"])
    robot_description_content = Command(["xacro ", urdf_xacro])

    # 1) gz sim server (headless) with empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros_gz_sim_share, "/launch/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": "-s -r empty.sdf",  # -s: server only; -r: run on start
        }.items(),
    )

    # 2) robot_state_publisher publishing /robot_description
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content, "use_sim_time": True}
        ],
    )

    # 3) spawn entity from /robot_description into gz
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "minimal_revolute",
            "-z", "0.5",
        ],
        output="screen",
    )

    # 4) /clock bridge (so ROS sees sim time)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # 5) Controller spawners — load and activate after spawn finished
    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )

    load_position = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "position_controller"],
        output="screen",
    )

    # Sequence: spawn -> wait -> load jsb -> wait -> load position
    after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[TimerAction(period=2.0, actions=[load_jsb])],
        )
    )
    after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=load_jsb,
            on_exit=[TimerAction(period=1.0, actions=[load_position])],
        )
    )

    return LaunchDescription(
        [
            gz_sim,
            rsp,
            clock_bridge,
            spawn,
            after_spawn,
            after_jsb,
        ]
    )
