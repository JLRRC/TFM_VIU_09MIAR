#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Contenido: Configuracion de bringup ROS 2 para lanzar el sistema UR5.
# Uso breve: Colcon/ros2 launch lo usan para arrancar simulacion y componentes principales.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Resumen: Launch unificado oficial para Gazebo + bridge + ros2_control + panel.
"""Launch unificado oficial del stack UR5 (Gazebo, bridge, ros2_control, panel)."""

from __future__ import annotations

import os
import sys
from typing import List

# ros2 launch loads this file without a parent package, so relative imports
# don't work. Add the launch directory to sys.path for the helper module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from launch_helpers import (
    build_gz_plugin_path,
    build_gz_resource_path,
    copy_runtime_model,
    load_runtime_defaults,
    patch_bridge_yaml,
    prepare_runtime_world_sdf,
    resolve_gz_partition,
    resolve_world_name,
    resolve_ws_dir,
)
from gz_factory import build_gz_actions
from runtime_nodes_factory import build_runtime_node_actions
from stack_factories import (
    build_bridge_params,
    build_launch_arguments,
    build_runtime_environment_actions,
    build_system_state_and_attach_extras,
)


# Runtime defaults loaded from YAML at module import time.
# Resolution priority for any tunable: env var > this dict > literal default.
# See: agarre_ros2_ws/src/ur5_bringup/config/runtime_defaults.yaml
_RUNTIME_DEFAULTS: dict[str, str] = load_runtime_defaults()

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ur5_tools.gripper_geometry import (
    RG2_TCP_FRAME,
    load_gripper_geometry,
    patch_runtime_model_sdf,
    validate_pick_demo_anchor,
)


def _resolve_runtime(name: str, default: str) -> str:
    """Env var > runtime_defaults.yaml > literal default. Returns string."""
    raw = os.environ.get(name)
    if raw is not None and raw.strip() != "":
        return raw
    yaml_val = _RUNTIME_DEFAULTS.get(name)
    if yaml_val is not None and yaml_val.strip() != "":
        return yaml_val
    return default


def _env_flag(name: str, default: bool) -> bool:
    raw = _resolve_runtime(name, "1" if default else "0").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    raw = _resolve_runtime(name, str(default)).strip()
    try:
        value = float(raw)
    except Exception:
        value = float(default)
    return value


def _env_flag(name: str, default: bool) -> bool:
    raw = str(os.environ.get(name, "1" if default else "0") or "").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    raw = str(os.environ.get(name, str(default)) or "").strip()
    try:
        value = float(raw)
    except Exception:
        value = float(default)
    return value


def _panel_qt_platform() -> str:
    explicit = os.environ.get("PANEL_QT_PLATFORM") or os.environ.get("QT_QPA_PLATFORM")
    if explicit:
        return explicit
    if os.environ.get("PANEL_FORCE_OFFSCREEN", "0") == "1" or not os.environ.get("DISPLAY"):
        return "offscreen"
    return "xcb"


def _prepare_runtime(context, *_args) -> List[object]:
    logger = get_logger("ur5_stack")
    # F2-step3 (audit-v4): WS_DIR resuelto via launch_helpers.resolve_ws_dir.
    ws_dir = resolve_ws_dir()
    world_file = LaunchConfiguration("world_file").perform(context)
    strict_physics_mode = (
        str(LaunchConfiguration("strict_physics_mode").perform(context)).strip().lower()
        in ("1", "true", "yes", "on")
    )
    log_dir = os.path.join(ws_dir, "log")
    os.makedirs(log_dir, exist_ok=True)

    world_name = resolve_world_name(world_file)
    if world_name == "ur5_mesa_objetos" and world_file:
        logger.debug("Using default world_name (could not read from %s)", world_file)

    gz_partition = resolve_gz_partition(log_dir)

    base_yaml = os.path.join(ws_dir, "scripts", "bridge_cameras.yaml")
    panel_settings_yaml = os.path.join(
        ws_dir, "src", "ur5_qt_panel", "config", "panel_settings.yaml"
    )
    runtime_yaml = patch_bridge_yaml(
        base_yaml, os.path.join(log_dir, "bridge_runtime.yaml"), world_name
    )

    runtime_models_root = os.path.join(log_dir, "gz_models")
    runtime_ur5_model = os.path.join(runtime_models_root, "ur5_rg2")
    copy_runtime_model(os.path.join(ws_dir, "models", "ur5_rg2"), runtime_ur5_model)

    resource_path = build_gz_resource_path(runtime_models_root, ws_dir)
    plugin_path = build_gz_plugin_path()
    render_engine = _resolve_runtime("GZ_RENDER_ENGINE", "ogre").strip() or "ogre"
    fastdds_profile = os.path.join(ws_dir, "scripts", "fastdds_no_shm.xml")
    # Prefer the explicit launch argument (camera_required:="0/1") over the env var,
    # because env var propagation via SetEnvironmentVariable is unreliable for
    # ExecuteProcess (panel) when DISPLAY is not usable (offscreen mode).
    launch_arg_camera = LaunchConfiguration("camera_required").perform(context).strip()
    camera_required_env = _resolve_runtime("PANEL_CAMERA_REQUIRED", "").strip()
    if launch_arg_camera in ("0", "false", "False", "no", "off"):
        camera_required_env = "0"
    elif launch_arg_camera in ("1", "true", "True", "yes", "on"):
        camera_required_env = "1"
    elif not camera_required_env:
        camera_required_env = "1"
    camera_required = camera_required_env in ("1", "true", "True")
    control_backend = (
        LaunchConfiguration("control_backend").perform(context).strip().lower()
    )
    moveit_mode = (
        LaunchConfiguration("moveit_mode").perform(context).strip().lower()
    )
    if moveit_mode not in ("auto", "move_group", "bridge"):
        logger.warning("moveit_mode invalido '%s'; usando auto", moveit_mode)
        moveit_mode = "auto"
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
    launch_moveit_eff = LaunchConfiguration("launch_moveit").perform(context)
    panel_auto_bridge_eff = LaunchConfiguration("panel_auto_bridge").perform(context)
    if moveit_mode == "move_group":
        # Solo forzar launch_moveit si no se pasó explícitamente como false.
        # Con PANEL_START_STACK=0 (stack externo), start_panel_v2.sh pasa launch_moveit:=false
        # para evitar un segundo move_group que causaría el dual-bridge bug.
        if str(launch_moveit_eff).lower() not in ("0", "false", "no"):
            launch_moveit_eff = "true"
        if str(panel_auto_bridge_eff).lower() in ("1", "true", "yes"):
            logger.warning(
                "moveit_mode=move_group: desactivando panel_auto_bridge para evitar doble backend."
            )
        panel_auto_bridge_eff = "0"
    elif moveit_mode == "bridge":
        if str(launch_moveit_eff).lower() in ("1", "true", "yes"):
            logger.warning(
                "moveit_mode=bridge: desactivando launch_moveit para evitar doble backend."
            )
        launch_moveit_eff = "false"
    launch_scene_sync_eff = "true" if moveit_mode == "move_group" else "false"
    moveit_start_ros2_control_eff = moveit_start_ros2_control_val
    if str(launch_gazebo_val).lower() in ("1", "true", "yes"):
        if str(moveit_start_ros2_control_val).lower() in ("1", "true", "yes"):
            logger.warning(
                "Gazebo activo: desactivando moveit_start_ros2_control para evitar duplicar controller_manager."
            )
        moveit_start_ros2_control_eff = "false"
    launch_attach_backend_eff = LaunchConfiguration("launch_attach_backend").perform(context)
    if strict_physics_mode and str(launch_attach_backend_eff).lower() in ("1", "true", "yes"):
        logger.warning(
            "strict_physics_mode activo: desactivando launch_attach_backend para evitar agarre asistido por software."
        )
        launch_attach_backend_eff = "false"
    launch_flags = [
        LaunchConfiguration("launch_gazebo").perform(context),
        LaunchConfiguration("launch_rsp").perform(context),
        LaunchConfiguration("launch_bridge").perform(context),
        launch_ros2_control_eff,
        launch_moveit_eff,
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
        if not os.path.isfile(model_sdf):
            raise RuntimeError(f"Runtime model.sdf no encontrado: {model_sdf}")
        geometry = load_gripper_geometry(ws_dir=ws_dir)
        patch_runtime_model_sdf(
            model_sdf,
            controllers_yaml=controllers_yaml,
            geometry=geometry,
        )
        anchor_ok, anchor_reason = validate_pick_demo_anchor(
            model_sdf,
            geometry=geometry,
            tolerance_m=1e-6,
        )
        if not anchor_ok:
            raise RuntimeError(anchor_reason)
        logger.info(
            "runtime model geometry synced "
            f"source={geometry.tcp.source_path} "
            f"tcp_z={geometry.z_for_frame(RG2_TCP_FRAME):.7f} "
            f"{anchor_reason}"
        )
    except Exception as exc:
        logger.error("Runtime geometry sync failed: %s", exc)
        raise RuntimeError(f"Runtime geometry sync failed: {exc}") from exc

    headless_mode = LaunchConfiguration("headless").perform(context)
    keep_cameras = _resolve_runtime("PANEL_KEEP_CAMERAS", "0").strip() in ("1", "true", "True")
    if not keep_cameras:
        keep_cameras = camera_required
    if str(headless_mode).lower() in ("1", "true", "yes") and not keep_cameras:
        logger.warning(
            "Headless sin cámaras: el sistema fallará por requisito crítico de visión."
        )
    runtime_world = prepare_runtime_world_sdf(
        world_file, log_dir, runtime_ur5_model,
        headless=str(headless_mode).lower() in ("1", "true", "yes"),
        keep_cameras=keep_cameras,
    )
    return build_runtime_environment_actions(
        ws_dir=ws_dir,
        gz_partition=gz_partition,
        resource_path=resource_path,
        plugin_path=plugin_path,
        render_engine=render_engine,
        controllers_yaml=controllers_yaml,
        panel_auto_bridge_eff=panel_auto_bridge_eff,
        managed_str=managed_str,
        camera_required_env=camera_required_env,
        fastdds_profile=fastdds_profile,
        launch_moveit_eff=launch_moveit_eff,
        moveit_mode=moveit_mode,
        panel_settings_yaml=panel_settings_yaml,
        strict_physics_mode=strict_physics_mode,
        runtime_yaml=runtime_yaml,
        runtime_world=runtime_world,
        world_name=world_name,
        launch_ros2_control_eff=launch_ros2_control_eff,
        launch_attach_backend_eff=launch_attach_backend_eff,
        launch_scene_sync_eff=launch_scene_sync_eff,
        moveit_start_ros2_control_eff=moveit_start_ros2_control_eff,
    )


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
                "strict_self_collision": LaunchConfiguration("strict_physics_mode"),
            }.items(),
        )
    ]


def generate_launch_description():
    # F2-step3 (audit-v4): WS_DIR centralizado en launch_helpers.resolve_ws_dir.
    ws_dir = resolve_ws_dir()
    world_default = os.path.join(ws_dir, "worlds", "ur5_mesa_objetos.sdf")
    gui_config_default = (
        "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config"
    )

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
    gui_config_file = LaunchConfiguration("gui_config_file")

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
            # F1.9 (audit-v4 2026-05-08): subido 12 → 60s. En sim con muchos
            # plugins (10+ detachable joints), gz sim puede tardar > 12s en
            # publicar el primer /clock. Logging periódico cada 5s ahora
            # documenta el progreso.
            {"clock_timeout_sec": 60.0},
            {"service_timeout_sec": 8.0},
            {"autostart": True},
            {"stay_alive": True},
        ],
        condition=IfCondition(bootstrap_controllers),
    )

    # F17 (2026-05-01): el subgrupo Gazebo se construye en gz_factory.
    gz_actions = build_gz_actions(
        world_file=world_file,
        render_engine=render_engine,
        gui_config_file=gui_config_file,
        headless=headless,
        launch_gazebo=launch_gazebo,
        launch_bridge=launch_bridge,
        runtime_yaml=LaunchConfiguration("runtime_yaml"),
        world_name=LaunchConfiguration("world_name"),
        use_sim_time=use_sim_time,
    )

    system_state_extras, attach_extras, demo_transport_objects = (
        build_system_state_and_attach_extras(_resolve_runtime)
    )
    bridge_params = build_bridge_params(use_sim_time, _env_float, _env_flag)

    runtime_actions = build_runtime_node_actions(
        use_sim_time=use_sim_time,
        world_name=LaunchConfiguration("world_name"),
        world_file=LaunchConfiguration("world_file"),
        camera_required=camera_required,
        controller_manager=LaunchConfiguration("controller_manager"),
        launch_moveit=launch_moveit,
        launch_world_tf=launch_world_tf,
        launch_system_state=launch_system_state,
        launch_release_service=launch_release_service,
        launch_attach_backend=launch_attach_backend,
        launch_scene_sync=launch_scene_sync,
        launch_moveit_bridge=LaunchConfiguration("launch_moveit_bridge"),
        launch_tf_geometry_service=LaunchConfiguration(
            "launch_tf_geometry_service"
        ),
        launch_object_pose_resolver=LaunchConfiguration(
            "launch_object_pose_resolver"
        ),
        launch_plan_to_pose_server=LaunchConfiguration(
            "launch_plan_to_pose_server"
        ),
        launch_pick_orchestrator_lifecycle=LaunchConfiguration(
            "launch_pick_orchestrator_lifecycle"
        ),
        pick_orchestrator_use_stubs=LaunchConfiguration(
            "pick_orchestrator_use_stubs"
        ),
        gz_delete_service=LaunchConfiguration("gz_delete_service"),
        gz_spawn_service=LaunchConfiguration("gz_spawn_service"),
        attach_backend_mode=LaunchConfiguration("attach_backend_mode"),
        attach_backend_max_pose_age_sec=LaunchConfiguration(
            "attach_backend_max_pose_age_sec"
        ),
        attach_backend_follow_rate_hz=LaunchConfiguration(
            "attach_backend_follow_rate_hz"
        ),
        attach_backend_follow_break_dist_m=LaunchConfiguration(
            "attach_backend_follow_break_dist_m"
        ),
        attach_backend_max_dist_m=LaunchConfiguration(
            "attach_backend_max_dist_m"
        ),
        demo_transport_objects=demo_transport_objects,
        bridge_params=bridge_params,
        system_state_yaml=PathJoinSubstitution(
            [FindPackageShare("ur5_bringup"), "config", "system_state_manager.yaml"]
        ),
        system_state_extras=system_state_extras,
        attach_extras=attach_extras,
    )

    panel_python = _resolve_runtime("PANEL_PYTHON", "")
    if panel_python:
        panel_cmd = [panel_python, "-m", "ur5_qt_panel.panel_v2"]
    else:
        panel_cmd = ["ros2", "run", "ur5_qt_panel", "panel_v2"]
    panel = ExecuteProcess(
        cmd=panel_cmd,
        output="screen",
        additional_env={
            "PANEL_CAMERA_REQUIRED": LaunchConfiguration("camera_required"),
            "QT_QPA_PLATFORM": _panel_qt_platform(),
        },
        condition=IfCondition(launch_panel),
    )

    return LaunchDescription(
        [
            *build_launch_arguments(
                world_default=world_default,
                gui_config_default=gui_config_default,
                resolve_runtime_fn=_resolve_runtime,
            ),
            OpaqueFunction(function=_prepare_runtime),
            OpaqueFunction(function=_maybe_moveit),
            rsp_launch,
            ros2_control_launch,
            controller_bootstrap,
            *gz_actions,
            *runtime_actions,
            panel,
        ]
    )
