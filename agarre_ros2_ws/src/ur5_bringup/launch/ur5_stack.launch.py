#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Contenido: Configuracion de bringup ROS 2 para lanzar el sistema UR5.
# Uso breve: Colcon/ros2 launch lo usan para arrancar simulacion y componentes principales.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Summary: Official unified launch for Gazebo + bridge + ros2_control + panel.
"""Official unified launch for the UR5 stack (Gazebo, bridge, ros2_control, panel)."""

from __future__ import annotations

import os
import re
import time
import shutil
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _prepare_runtime(context, *_args) -> List[object]:
    logger = get_logger("ur5_stack")
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
    world_file = LaunchConfiguration("world_file").perform(context)
    log_dir = os.path.join(ws_dir, "log")
    os.makedirs(log_dir, exist_ok=True)

    world_name = "ur5_mesa_objetos"
    try:
        with open(world_file, "r", encoding="utf-8") as f:
            data = f.read()
        match = re.search(r"<world name=\"([^\"]+)\">", data)
        if match:
            world_name = match.group(1)
    except Exception as exc:
        logger.warning("No se pudo leer world_name desde %s: %s", world_file, exc)

    gz_partition = os.environ.get("GZ_PARTITION", "")
    if not gz_partition:
        gz_partition = f"ur5pro_{int(time.time())}"
    try:
        with open(
            os.path.join(log_dir, "gz_partition.txt"), "w", encoding="utf-8"
        ) as f:
            f.write(gz_partition)
    except Exception:
        pass

    base_yaml = os.path.join(ws_dir, "scripts", "bridge_cameras.yaml")
    panel_settings_yaml = os.path.join(
        ws_dir, "src", "ur5_qt_panel", "config", "panel_settings.yaml"
    )
    runtime_yaml = os.path.join(log_dir, "bridge_runtime.yaml")
    try:
        with open(base_yaml, "r", encoding="utf-8") as f:
            yaml_text = f.read()
        if world_name:
            yaml_text = yaml_text.replace(
                "/world/ur5_mesa_objetos/",
                f"/world/{world_name}/",
            )
        with open(runtime_yaml, "w", encoding="utf-8") as f:
            f.write(yaml_text)
    except Exception as exc:
        logger.warning("No se pudo preparar bridge_runtime.yaml: %s", exc)
        runtime_yaml = base_yaml

    runtime_models_root = os.path.join(log_dir, "gz_models")
    runtime_ur5_model = os.path.join(runtime_models_root, "ur5_rg2")
    src_ur5_model = os.path.join(ws_dir, "models", "ur5_rg2")
    try:
        if os.path.isdir(src_ur5_model):
            shutil.copytree(src_ur5_model, runtime_ur5_model, dirs_exist_ok=True)
    except Exception as exc:
        logger.warning("No se pudo copiar modelo ur5_rg2 a runtime: %s", exc)

    resource_path = (
        f"{runtime_models_root}:{ws_dir}/models:{ws_dir}/worlds:{ws_dir}/install"
    )
    existing_resource = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource:
        resource_path = f"{resource_path}:{existing_resource}"
    plugin_path = "/opt/ros/jazzy/lib"
    existing_plugin = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    if existing_plugin:
        plugin_path = f"{plugin_path}:{existing_plugin}"
    render_engine = os.environ.get("GZ_RENDER_ENGINE", "").strip() or "ogre2"
    fastdds_profile = os.path.join(ws_dir, "scripts", "fastdds_no_shm.xml")
    camera_required_env = os.environ.get("PANEL_CAMERA_REQUIRED", "").strip()
    if not camera_required_env:
        camera_required_env = "1"
    camera_required = camera_required_env in ("1", "true", "True")
    control_backend = (
        LaunchConfiguration("control_backend").perform(context).strip().lower()
    )
    launch_gazebo_val = LaunchConfiguration("launch_gazebo").perform(context)
    launch_ros2_control_val = LaunchConfiguration("launch_ros2_control").perform(
        context
    )
    moveit_start_ros2_control_val = LaunchConfiguration(
        "moveit_start_ros2_control"
    ).perform(context)
    launch_ros2_control_eff = launch_ros2_control_val
    if control_backend in ("gz", "gazebo", "gz_ros2_control"):
        if str(launch_ros2_control_val).lower() in ("1", "true", "yes"):
            logger.warning(
                "control_backend=gz_ros2_control: desactivando ros2_control_node para evitar doble controller_manager."
            )
        launch_ros2_control_eff = "false"
    elif control_backend in ("ros2_control", "ros2_control_node"):
        if str(launch_gazebo_val).lower() in ("1", "true", "yes"):
            logger.error(
                "control_backend=ros2_control_node no es compatible con Gazebo: desactivando ros2_control_node."
            )
            launch_ros2_control_eff = "false"
        else:
            launch_ros2_control_eff = "true"
    else:
        logger.warning(
            "control_backend desconocido '%s'; se mantiene launch_ros2_control=%s",
            control_backend,
            launch_ros2_control_val,
        )
    moveit_start_ros2_control_eff = moveit_start_ros2_control_val
    if str(launch_gazebo_val).lower() in ("1", "true", "yes"):
        if str(moveit_start_ros2_control_val).lower() in ("1", "true", "yes"):
            logger.warning(
                "Gazebo activo: desactivando moveit_start_ros2_control para evitar duplicar controller_manager."
            )
        moveit_start_ros2_control_eff = "false"
    launch_flags = [
        LaunchConfiguration("launch_gazebo").perform(context),
        LaunchConfiguration("launch_rsp").perform(context),
        LaunchConfiguration("launch_bridge").perform(context),
        launch_ros2_control_eff,
        LaunchConfiguration("launch_moveit").perform(context),
    ]
    managed = any(str(flag).lower() in ("1", "true", "yes") for flag in launch_flags)
    managed_str = "1" if managed else "0"
    controllers_yaml = os.path.join(
        get_package_share_directory("ur5_description"),
        "config",
        "ur5_controllers.yaml",
    )
    try:
        model_sdf = os.path.join(runtime_ur5_model, "model.sdf")
        if os.path.isfile(model_sdf):
            with open(model_sdf, "r", encoding="utf-8") as f:
                sdf_text = f.read()
            plugin_re = re.compile(
                r'(<plugin filename="gz_ros2_control-system"[^>]*>)(.*?)(</plugin>)',
                re.DOTALL,
            )
            match = plugin_re.search(sdf_text)
            if match:
                header, body, footer = match.groups()
                if "<parameters>" in body:
                    body = re.sub(
                        r"<parameters>.*?</parameters>",
                        f"<parameters>{controllers_yaml}</parameters>",
                        body,
                        flags=re.DOTALL,
                    )
                else:
                    body = (
                        body
                        + f"\n            <parameters>{controllers_yaml}</parameters>\n"
                    )
                sdf_text = (
                    sdf_text[: match.start()]
                    + header
                    + body
                    + footer
                    + sdf_text[match.end() :]
                )
                with open(model_sdf, "w", encoding="utf-8") as f:
                    f.write(sdf_text)
    except Exception as exc:
        logger.warning("No se pudo ajustar plugin gz_ros2_control: %s", exc)

    runtime_world = world_file
    try:
        if os.path.isfile(world_file):
            with open(world_file, "r", encoding="utf-8") as f:
                world_text = f.read()
            headless_mode = LaunchConfiguration("headless").perform(context)
            keep_cameras = os.environ.get("PANEL_KEEP_CAMERAS", "").strip() in (
                "1",
                "true",
                "True",
            )
            if not keep_cameras:
                keep_cameras = camera_required
            if str(headless_mode).lower() in ("1", "true", "yes") and not keep_cameras:
                logger.warning(
                    "Headless sin cámaras: el sistema fallará por requisito crítico de visión."
                )
                for cam_name in (
                    "camera_overhead",
                    "camera_north",
                    "camera_south",
                    "camera_east",
                    "camera_west",
                ):
                    pattern = rf"<model\s+name=\"{cam_name}\">.*?</model>"
                    world_text = re.sub(pattern, "", world_text, flags=re.DOTALL)
            world_text = world_text.replace(
                "<uri>model://ur5_rg2</uri>",
                f"<uri>file://{runtime_ur5_model}</uri>",
            )
            runtime_world = os.path.join(log_dir, "world_runtime.sdf")
            with open(runtime_world, "w", encoding="utf-8") as f:
                f.write(world_text)
    except Exception as exc:
        logger.warning("No se pudo preparar world_runtime.sdf: %s", exc)
        runtime_world = world_file
    return [
        SetEnvironmentVariable("WS_DIR", ws_dir),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("GZ_RENDER_ENGINE", render_engine),
        SetEnvironmentVariable(
            "__EGL_VENDOR_LIBRARY_FILENAMES",
            "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
        ),
        SetEnvironmentVariable("UR5_CONTROLLERS_YAML", controllers_yaml),
        SetEnvironmentVariable("UR5_CONTROLLERS_FILE", controllers_yaml),
        SetEnvironmentVariable(
            "PANEL_CONTROLLER_MANAGER", LaunchConfiguration("controller_manager")
        ),
        SetEnvironmentVariable(
            "PANEL_AUTO_BRIDGE", LaunchConfiguration("panel_auto_bridge")
        ),
        SetEnvironmentVariable(
            "PANEL_AUTO_BRIDGE_DELAY_MS",
            LaunchConfiguration("panel_auto_bridge_delay_ms"),
        ),
        SetEnvironmentVariable("PANEL_MANAGED", managed_str),
        SetEnvironmentVariable("PANEL_CAMERA_REQUIRED", camera_required_env),
        SetEnvironmentVariable(
            "PANEL_MOVEIT_REQUIRED", LaunchConfiguration("launch_moveit")
        ),
        SetEnvironmentVariable("PANEL_SETTINGS_YAML", panel_settings_yaml),
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
        ),
        SetEnvironmentVariable("RMW_FASTRTPS_USE_SHM", "0"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
        SetLaunchConfiguration("runtime_yaml", runtime_yaml),
        SetLaunchConfiguration("controllers_file", controllers_yaml),
        SetLaunchConfiguration("world_file", runtime_world),
        SetLaunchConfiguration("world_name", world_name),
        SetLaunchConfiguration(
            "gz_delete_service",
            f"/world/{world_name}/remove/blocking",
        ),
        SetLaunchConfiguration(
            "gz_spawn_service",
            f"/world/{world_name}/create/blocking",
        ),
        SetLaunchConfiguration("camera_required", camera_required_env),
        SetLaunchConfiguration("panel_managed", managed_str),
        SetLaunchConfiguration("launch_ros2_control", launch_ros2_control_eff),
        SetLaunchConfiguration(
            "moveit_start_ros2_control", moveit_start_ros2_control_eff
        ),
    ]


def _maybe_moveit(context, *_args) -> List[object]:
    launch_moveit = LaunchConfiguration("launch_moveit").perform(context)
    if str(launch_moveit).lower() not in ("1", "true", "yes"):
        return []
    moveit_start_ros2_control = LaunchConfiguration("moveit_start_ros2_control")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur5_moveit_config"),
                        "launch",
                        "ur5_moveit_bringup.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "start_ros2_control": moveit_start_ros2_control,
                "launch_rviz": "false",
                "use_sim_time": use_sim_time,
            }.items(),
        )
    ]


def generate_launch_description():
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
    world_default = os.path.join(ws_dir, "worlds", "ur5_mesa_objetos.sdf")

    headless = LaunchConfiguration("headless")
    launch_panel = LaunchConfiguration("launch_panel")
    launch_bridge = LaunchConfiguration("launch_bridge")
    launch_gazebo = LaunchConfiguration("launch_gazebo")
    launch_rsp = LaunchConfiguration("launch_rsp")
    launch_ros2_control = LaunchConfiguration("launch_ros2_control")
    launch_world_tf = LaunchConfiguration("launch_world_tf")
    launch_release_service = LaunchConfiguration("launch_release_service")
    launch_attach_backend = LaunchConfiguration("launch_attach_backend")
    launch_scene_sync = LaunchConfiguration("launch_scene_sync")
    launch_system_state = LaunchConfiguration("launch_system_state")
    launch_moveit = LaunchConfiguration("launch_moveit")
    camera_required = LaunchConfiguration("camera_required")
    bootstrap_controllers = LaunchConfiguration("bootstrap_controllers")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")
    render_engine = LaunchConfiguration("render_engine")

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur5_bringup"), "launch", "ur5_rsp.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(launch_rsp),
    )

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur5_bringup"),
                    "launch",
                    "ur5_ros2_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "controllers_yaml": "ur5_mock_controllers.yaml",
        }.items(),
        condition=IfCondition(launch_ros2_control),
    )

    controller_bootstrap = Node(
        package="ur5_tools",
        executable="controller_bootstrap",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"controller_manager": LaunchConfiguration("controller_manager")},
            {"wait_for_clock": True},
            {"clock_timeout_sec": 12.0},
            {"service_timeout_sec": 8.0},
            {"autostart": True},
            {"stay_alive": True},
        ],
        condition=IfCondition(bootstrap_controllers),
    )

    gz_headless = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            "--headless-rendering",
            "--render-engine",
            render_engine,
            world_file,
        ],
        output="screen",
        condition=IfCondition(headless),
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_group = GroupAction(
        actions=[gz_headless, gz_gui],
        condition=IfCondition(launch_gazebo),
    )
    gz_shutdown_headless = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_headless,
            on_exit=[EmitEvent(event=Shutdown(reason="gz sim exited (headless)"))],
        ),
        condition=IfCondition(launch_gazebo),
    )
    gz_shutdown_gui = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_gui,
            on_exit=[EmitEvent(event=Shutdown(reason="gz sim exited (gui)"))],
        ),
        condition=IfCondition(launch_gazebo),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_main",
        output="screen",
        arguments=["--ros-args", "-p", "use_sim_time:=true"],
        parameters=[
            {"config_file": LaunchConfiguration("runtime_yaml")},
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(launch_bridge),
    )
    bridge_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="parameter_bridge exited"))],
        ),
        condition=IfCondition(launch_bridge),
    )

    gz_control_guard = Node(
        package="ur5_tools",
        executable="gz_ros_control_guard",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"hold_joints": False},
        ],
        condition=IfCondition(launch_gazebo),
    )

    gz_pose_bridge = Node(
        package="ur5_tools",
        executable="gz_pose_bridge",
        output="screen",
        parameters=[
            {"world_name": LaunchConfiguration("world_name")},
            {"world_frame": "world"},
            {"startup_timeout_sec": 5.0},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_gazebo),
    )
    gz_pose_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_pose_bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="gz_pose_bridge exited"))],
        ),
        condition=IfCondition(launch_gazebo),
    )

    world_tf = Node(
        package="ur5_tools",
        executable="world_tf_publisher",
        output="screen",
        parameters=[
            {"world_name": LaunchConfiguration("world_name")},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"clock_timeout_sec": 5.0},
            {"pose_timeout_sec": 5.0},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_world_tf),
    )
    # world_tf_publisher is helpful but non-critical; do not shutdown whole stack if it exits.
    world_tf_guard = GroupAction(actions=[])

    system_state = Node(
        package="ur5_tools",
        executable="system_state_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ur5_bringup"), "config", "system_state_manager.yaml"]
            ),
            {"use_sim_time": use_sim_time},
            {"world_name": LaunchConfiguration("world_name")},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"ee_frame": "rg2_tcp"},
            {"camera_topic": "/camera_overhead/image"},
            {"camera_required": ParameterValue(camera_required, value_type=bool)},
            {"controller_manager": LaunchConfiguration("controller_manager")},
            {"moveit_required": ParameterValue(launch_moveit, value_type=bool)},
            {"startup_timeout_sec": 5.0},
        ],
        condition=IfCondition(launch_system_state),
    )

    system_state_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=system_state,
            on_exit=[EmitEvent(event=Shutdown(reason="system_state_manager exited"))],
        ),
        condition=IfCondition(launch_system_state),
    )

    release_service = Node(
        package="ur5_tools",
        executable="release_objects_service",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_sdf": LaunchConfiguration("world_file")},
            {"world_name": LaunchConfiguration("world_name")},
            {"delete_service": LaunchConfiguration("gz_delete_service")},
            {"spawn_service": LaunchConfiguration("gz_spawn_service")},
        ],
        condition=IfCondition(launch_release_service),
    )

    gripper_attach_backend = Node(
        package="ur5_tools",
        executable="gripper_attach_backend",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_attach_backend),
    )

    planning_scene_sync = Node(
        package="ur5_tools",
        executable="planning_scene_sync",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_name": LaunchConfiguration("world_name")},
            {"world_file": LaunchConfiguration("world_file")},
            {"world_frame": "world"},
            {"base_frame": "base_link"},
            {"ee_frame": "rg2_tcp"},
        ],
        condition=IfCondition(launch_scene_sync),
    )

    panel_python = os.environ.get("PANEL_PYTHON", "")
    if panel_python:
        panel_cmd = [panel_python, "-m", "ur5_qt_panel.panel_v2"]
    else:
        panel_cmd = ["ros2", "run", "ur5_qt_panel", "panel_v2"]
    panel = ExecuteProcess(
        cmd=panel_cmd,
        output="screen",
        condition=IfCondition(launch_panel),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=world_default),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_panel", default_value="true"),
            DeclareLaunchArgument("launch_bridge", default_value="true"),
            DeclareLaunchArgument("launch_gazebo", default_value="true"),
            DeclareLaunchArgument("launch_rsp", default_value="true"),
            DeclareLaunchArgument("launch_ros2_control", default_value="false"),
            DeclareLaunchArgument("control_backend", default_value="gz_ros2_control"),
            DeclareLaunchArgument("launch_world_tf", default_value="true"),
            DeclareLaunchArgument("launch_release_service", default_value="true"),
            DeclareLaunchArgument("launch_attach_backend", default_value="true"),
            DeclareLaunchArgument("launch_scene_sync", default_value="true"),
            DeclareLaunchArgument("launch_system_state", default_value="true"),
            DeclareLaunchArgument("launch_moveit", default_value="false"),
            DeclareLaunchArgument("camera_required", default_value="1"),
            DeclareLaunchArgument("moveit_start_ros2_control", default_value="false"),
            DeclareLaunchArgument("bootstrap_controllers", default_value="true"),
            DeclareLaunchArgument(
                "controller_manager", default_value="/controller_manager"
            ),
            DeclareLaunchArgument("panel_auto_bridge", default_value="0"),
            DeclareLaunchArgument("panel_auto_bridge_delay_ms", default_value="1200"),
            DeclareLaunchArgument("panel_managed", default_value="1"),
            DeclareLaunchArgument(
                "rmw_implementation", default_value="rmw_fastrtps_cpp"
            ),
            DeclareLaunchArgument("render_engine", default_value="ogre2"),
            OpaqueFunction(function=_prepare_runtime),
            OpaqueFunction(function=_maybe_moveit),
            rsp_launch,
            ros2_control_launch,
            gz_group,
            gz_shutdown_headless,
            gz_shutdown_gui,
            bridge,
            bridge_guard,
            controller_bootstrap,
            gz_control_guard,
            gz_pose_bridge,
            gz_pose_guard,
            world_tf,
            world_tf_guard,
            system_state,
            system_state_guard,
            release_service,
            gripper_attach_backend,
            planning_scene_sync,
            panel,
        ]
    )
