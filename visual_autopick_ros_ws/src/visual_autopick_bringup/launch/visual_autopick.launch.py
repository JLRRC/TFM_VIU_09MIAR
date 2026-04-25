"""Master launch for visual_autopick_ros_ws.

Usage:
  ros2 launch visual_autopick_bringup visual_autopick.launch.py use_moveit:=false use_panel:=true panel_mode:=simple
  ros2 launch visual_autopick_bringup visual_autopick.launch.py use_moveit:=false use_panel:=true panel_mode:=step
  ros2 launch visual_autopick_bringup visual_autopick.launch.py use_panel:=false
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    EqualsSubstitution,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    # ── Rutas absolutas ───────────────────────────────────────────────────────
    bringup_share = get_package_share_directory("visual_autopick_bringup")
    description_share = get_package_share_directory("ur5_description")
    moveit_share = get_package_share_directory("ur5_moveit_config")

    ws_root = Path(bringup_share).parents[3]  # .../visual_autopick_ros_ws

    world_file   = str(ws_root / "worlds" / "visual_autopick_world.sdf")
    models_dir   = str(ws_root / "models")
    model_sdf    = str(ws_root / "models" / "ur5_rg2" / "model.sdf")
    xacro_file   = str(Path(description_share) / "urdf" / "ur5.urdf.xacro")
    gz_bridge_yaml = str(Path(bringup_share) / "config" / "gz_bridge.yaml")

    srdf_file             = str(Path(moveit_share) / "config" / "ur5.srdf")
    kinematics_yaml       = str(Path(moveit_share) / "config" / "kinematics.yaml")
    joint_limits_yaml     = str(Path(moveit_share) / "config" / "joint_limits.yaml")
    moveit_controllers_yaml = str(Path(moveit_share) / "config" / "moveit_controllers.yaml")

    # ── Argumentos ───────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("use_moveit",  default_value="false",
                              description="Arrancar move_group"),
        DeclareLaunchArgument("use_panel",   default_value="true",
                              description="Arrancar panel Qt"),
        DeclareLaunchArgument("panel_mode",  default_value="autopick",
                              description="autopick | simple | step | none"),
        DeclareLaunchArgument("use_attach",  default_value="false",
                              description="Attach físico al cerrar gripper"),
    ]

    use_moveit = LaunchConfiguration("use_moveit")
    use_panel  = LaunchConfiguration("use_panel")
    panel_mode = LaunchConfiguration("panel_mode")

    # Condiciones compuestas (Jazzy: EqualsSubstitution + AndSubstitution)
    panel_is_simple   = EqualsSubstitution(panel_mode, "simple")
    panel_is_step     = EqualsSubstitution(panel_mode, "step")
    panel_is_autopick = EqualsSubstitution(panel_mode, "autopick")

    cond_simple   = IfCondition(AndSubstitution(use_panel, panel_is_simple))
    cond_step     = IfCondition(AndSubstitution(use_panel, panel_is_step))
    cond_autopick = IfCondition(AndSubstitution(use_panel, panel_is_autopick))

    # ── URDF ─────────────────────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " ur_type:=ur5 name:=ur5_rg2"]),
        value_type=str,
    )

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH":      f"{models_dir}:/opt/ros/jazzy/share",
            "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/ros/jazzy/lib",
        },
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    # ── 3. Spawn robot desde SDF (gz_ros2_control carga controllers.yaml) ────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "ur5_rg2", "-file", model_sdf,
                   "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen",
    )

    # ── 4. Bridge gz → ROS ───────────────────────────────────────────────────
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={gz_bridge_yaml}"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── 5. Spawners de controladores (con delay) ──────────────────────────────
    def _spawner(name: str) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "-c", "/controller_manager"],
            output="screen",
        )

    controllers = TimerAction(period=6.0, actions=[
        _spawner("joint_state_broadcaster"),
        _spawner("joint_trajectory_controller"),
        _spawner("gripper_controller"),
    ])

    # ── 6. World TF publisher ─────────────────────────────────────────────────
    world_tf = Node(
        package="visual_autopick_tools",
        executable="world_tf_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "world_name": "visual_autopick_world",
            "model_name": "ur5_rg2",
            "world_file": world_file,
            "static_grace_sec": -1.0,
            "wait_for_clock": True,
            "clock_ready_min_sec": 0.5,
        }],
    )

    # ── 7. MoveIt move_group (opcional) ──────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("ur5_rg2", package_name="ur5_moveit_config")
        .robot_description(file_path=xacro_file,
                           mappings={"ur_type": "ur5", "name": "ur5_rg2"})
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_yaml)
        .joint_limits(file_path=joint_limits_yaml)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path=moveit_controllers_yaml)
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        condition=IfCondition(use_moveit),
    )

    # ── 8. Step pick server ───────────────────────────────────────────────────
    # Activo en modos "step" y "autopick". Se duplica el nodo porque launch no
    # tiene OrCondition — ambos tienen condition exclusiva con su modo.
    step_server_for_step = Node(
        package="visual_autopick_tools",
        executable="step_pick_server",
        name="step_pick_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=cond_step,
    )
    step_server_for_autopick = Node(
        package="visual_autopick_tools",
        executable="step_pick_server",
        name="step_pick_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=cond_autopick,
    )

    # ── 9. Paneles Qt ─────────────────────────────────────────────────────────
    simple_panel = Node(
        package="visual_autopick_panel",
        executable="simple_panel",
        output="screen",
        condition=cond_simple,
    )

    step_panel = Node(
        package="visual_autopick_panel",
        executable="step_panel",
        output="screen",
        condition=cond_step,
    )

    # Panel unificado: AUTO + STEP_BY_STEP (nuevo — modo por defecto)
    _display = os.environ.get("DISPLAY", ":0")
    _xauth   = os.environ.get("XAUTHORITY", "")
    autopick_panel = Node(
        package="visual_autopick_panel",
        executable="autopick_panel",
        output="screen",
        condition=cond_autopick,
        additional_env={
            "DISPLAY":          _display,
            "XAUTHORITY":       _xauth,
            "QT_X11_NO_MITSHM": "1",
        },
    )

    return LaunchDescription([
        *args,
        gz_sim,
        rsp,
        gz_bridge,
        TimerAction(period=2.0, actions=[spawn_robot]),
        controllers,
        world_tf,
        move_group,
        step_server_for_step,
        step_server_for_autopick,
        simple_panel,
        step_panel,
        autopick_panel,
    ])
