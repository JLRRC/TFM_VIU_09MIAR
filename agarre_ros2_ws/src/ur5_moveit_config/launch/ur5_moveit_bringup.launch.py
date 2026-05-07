# Ruta/archivo: agarre_ros2_ws/src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py
# Contenido: Configuracion MoveIt del UR5 para planificacion y ejecucion.
# Uso breve: MoveIt y ros2 launch lo usan durante el bringup del robot.
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ws_dir = os.environ.get("WS_DIR", "")
    ws_src = Path(ws_dir) / "src" if ws_dir else None
    strict_self_collision = os.environ.get("STRICT_SELF_COLLISION", "0").strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    def _pkg_share(pkg_name: str) -> str:
        try:
            return get_package_share_directory(pkg_name)
        except PackageNotFoundError:
            if ws_src:
                candidate = ws_src / pkg_name
                if candidate.is_dir():
                    return str(candidate)
            raise

    ur5_bringup_share = _pkg_share("ur5_bringup")
    ur5_description_share = _pkg_share("ur5_description")
    moveit_share = _pkg_share("ur5_moveit_config")

    srdf_file = "ur5_strict.srdf" if strict_self_collision else "ur5.srdf"
    srdf_path = Path(moveit_share) / "config" / srdf_file
    if not srdf_path.is_file() and ws_src:
        fallback = ws_src / "ur5_moveit_config" / "config" / srdf_file
        if fallback.is_file():
            srdf_path = fallback

    start_ros2_control = LaunchConfiguration("start_ros2_control")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur5_bringup_share, "launch", "ur5_ros2_control.launch.py"])
        ),
        condition=IfCondition(start_ros2_control),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5_rg2", package_name="ur5_moveit_config")
        .robot_description(
            file_path=str(Path(ur5_description_share) / "urdf" / "ur5.urdf.xacro"),
            mappings={"ur_type": "ur5", "name": "ur5_rg2"},
        )
        .robot_description_semantic(file_path=str(srdf_path))
        .robot_description_kinematics(
            file_path=str(Path(moveit_share) / "config" / "kinematics.yaml")
        )
        .joint_limits(
            file_path=str(Path(moveit_share) / "config" / "joint_limits.yaml")
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(
            file_path=str(Path(moveit_share) / "config" / "moveit_controllers.yaml")
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.to_dict(),
            # 2026-05-07: trajectory_execution timeout scaling.
            # Iteraciones live (rondas 11-24):
            # - 1.2 → CONTROL_FAILED a 9s
            # - 4.0 → CONTROL_FAILED a 71s
            # - 10.0 → en ronda 24 fail con upper bound 105s, robot tardó más.
            # 30.0 × 10.5s = 315s upper bound. Sim lento (sim_per_wall=0.58)
            # necesita margen muy generoso. Si el robot no ha llegado en
            # 315s, hay un bug real (no timeout artificial).
            {"trajectory_execution.allowed_execution_duration_scaling": 30.0},
            {"trajectory_execution.allowed_goal_duration_margin": 90.0},
            {"trajectory_execution.allowed_start_tolerance": 0.15},
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_ros2_control", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "strict_self_collision",
                default_value=os.environ.get("STRICT_SELF_COLLISION", "false"),
            ),
            bringup_launch,
            move_group_node,
            rviz_node,
        ]
    )
