"""Launch VARIANT B (UR5-style plugin block) of the minimal probe."""
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gz_ros2_control_minimal_probe")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    urdf_xacro = PathJoinSubstitution(
        [pkg_share, "urdf", "minimal_revolute_ur5plugin.urdf.xacro"]
    )
    robot_description_content = Command(["xacro ", urdf_xacro])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros_gz_sim_share, "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": "-s -r empty.sdf"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description_content, value_type=str),
                "use_sim_time": True,
            }
        ],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "minimal_revolute_ur5p",
            "-z", "0.5",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

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
        [gz_sim, rsp, clock_bridge, spawn, after_spawn, after_jsb]
    )
