"""MoveIt bringup for ur5_moveit_config — self-contained, no WS_DIR dependency."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("ur5_description")
    moveit_share = get_package_share_directory("ur5_moveit_config")

    xacro_file = str(Path(description_share) / "urdf" / "ur5.urdf.xacro")
    srdf_file = str(Path(moveit_share) / "config" / "ur5.srdf")
    kinematics_yaml = str(Path(moveit_share) / "config" / "kinematics.yaml")
    joint_limits_yaml = str(Path(moveit_share) / "config" / "joint_limits.yaml")
    moveit_controllers_yaml = str(Path(moveit_share) / "config" / "moveit_controllers.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    moveit_config = (
        MoveItConfigsBuilder("ur5_rg2", package_name="ur5_moveit_config")
        .robot_description(
            file_path=xacro_file,
            mappings={"ur_type": "ur5", "name": "ur5_rg2"},
        )
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_yaml)
        .joint_limits(file_path=joint_limits_yaml)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path=moveit_controllers_yaml)
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        move_group_node,
        rviz_node,
    ])
