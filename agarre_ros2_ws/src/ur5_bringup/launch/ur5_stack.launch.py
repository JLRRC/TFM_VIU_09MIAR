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
    PANEL_ENV_DEFAULTS,
    build_gz_plugin_path,
    build_gz_resource_path,
    copy_runtime_model,
    load_runtime_defaults,
    patch_bridge_yaml,
    prepare_runtime_world_sdf,
    resolve_gz_partition,
    resolve_world_name,
)
from gz_factory import build_gz_actions
from runtime_nodes_factory import build_runtime_node_actions


# Runtime defaults loaded from YAML at module import time.
# Resolution priority for any tunable: env var > this dict > literal default.
# See: agarre_ros2_ws/src/ur5_bringup/config/runtime_defaults.yaml
_RUNTIME_DEFAULTS: dict[str, str] = load_runtime_defaults()

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
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


def _build_runtime_environment_actions(
    *,
    ws_dir: str,
    gz_partition: str,
    resource_path: str,
    plugin_path: str,
    render_engine: str,
    controllers_yaml: str,
    panel_auto_bridge_eff: str,
    managed_str: str,
    camera_required_env: str,
    fastdds_profile: str,
    launch_moveit_eff: str,
    moveit_mode: str,
    panel_settings_yaml: str,
    strict_physics_mode: bool,
    runtime_yaml: str,
    runtime_world: str,
    world_name: str,
    launch_ros2_control_eff: str,
    launch_attach_backend_eff: str,
    launch_scene_sync_eff: str,
    moveit_start_ros2_control_eff: str,
) -> List[object]:
    """F3-step20: ensambla la lista completa de SetEnvironmentVariable +
    SetLaunchConfiguration para _prepare_runtime (~80 LOC).
    """
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
        SetEnvironmentVariable("PANEL_AUTO_BRIDGE", panel_auto_bridge_eff),
        SetEnvironmentVariable(
            "PANEL_AUTO_BRIDGE_DELAY_MS",
            LaunchConfiguration("panel_auto_bridge_delay_ms"),
        ),
        SetEnvironmentVariable("PANEL_MANAGED", managed_str),
        SetEnvironmentVariable("PANEL_CAMERA_REQUIRED", camera_required_env),
        SetEnvironmentVariable("USE_SIM_TIME", LaunchConfiguration("use_sim_time")),
        *[
            SetEnvironmentVariable(name, os.environ.get(name, default))
            for name, default in PANEL_ENV_DEFAULTS
        ],
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M",
            "0.025",
        ),
        SetEnvironmentVariable("PANEL_MOVEIT_REQUIRED", launch_moveit_eff),
        SetEnvironmentVariable("PANEL_MOVEIT_MODE", moveit_mode),
        SetEnvironmentVariable("PANEL_SETTINGS_YAML", panel_settings_yaml),
        SetEnvironmentVariable(
            "STRICT_SELF_COLLISION",
            "1" if strict_physics_mode else "0",
        ),
        SetEnvironmentVariable(
            "PANEL_STRICT_PHYSICS_MODE",
            "1" if strict_physics_mode else "0",
        ),
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
        SetLaunchConfiguration("launch_moveit", launch_moveit_eff),
        SetLaunchConfiguration("panel_auto_bridge", panel_auto_bridge_eff),
        SetLaunchConfiguration("launch_ros2_control", launch_ros2_control_eff),
        SetLaunchConfiguration("launch_attach_backend", launch_attach_backend_eff),
        SetLaunchConfiguration("launch_scene_sync", launch_scene_sync_eff),
        SetLaunchConfiguration(
            "moveit_start_ros2_control", moveit_start_ros2_control_eff
        ),
    ]


def _prepare_runtime(context, *_args) -> List[object]:
    logger = get_logger("ur5_stack")
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
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
    render_engine = _resolve_runtime("GZ_RENDER_ENGINE", "ogre2").strip() or "ogre2"
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
    return _build_runtime_environment_actions(
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


def _build_bridge_params(use_sim_time):
    """F3-step26a: ensambla bridge_params para ur5_moveit_bridge (~80 LOC).

    Lee 16 env vars PANEL_MOVEIT_BRIDGE_* con clamp + default + devuelve
    lista de dicts compatible con el ``parameters=[...]`` de Node.
    """
    bridge_exec_timeout = max(1.0, _env_float("PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC", 150.0))
    bridge_request_timeout = max(2.0, _env_float("PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC", 180.0))
    bridge_stale_request_ttl_sec = max(20.0, _env_float("PANEL_MOVEIT_BRIDGE_STALE_REQUEST_TTL_SEC", 120.0))
    bridge_joint_state_timeout = max(0.2, _env_float("PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC", 6.0))
    bridge_joint_state_max_age = max(0.1, _env_float("PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC", 2.5))
    bridge_force_fjt_direct = _env_flag("PANEL_MOVEIT_BRIDGE_FORCE_FJT_DIRECT", False)
    bridge_unwrap_continuous = _env_flag("PANEL_MOVEIT_BRIDGE_UNWRAP_CONTINUOUS_JOINTS", False)
    bridge_require_request_id = _env_flag("PANEL_MOVEIT_BRIDGE_REQUIRE_REQUEST_ID", True)
    bridge_drop_pending = _env_flag("PANEL_MOVEIT_BRIDGE_DROP_PENDING_ON_TAGGED", True)
    bridge_dry_run = _env_flag("PANEL_MOVEIT_BRIDGE_DRY_RUN", False)
    bridge_path_constraint_tol = max(0.0, _env_float("PANEL_MOVEIT_BRIDGE_PATH_CONSTRAINT_TOL_RAD", 0.35))
    bridge_controller_goal_time_tol = max(0.0, _env_float("PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TIME_TOL_SEC", 45.0))
    bridge_controller_expected_goal_time = max(
        0.0,
        _env_float(
            "PANEL_MOVEIT_BRIDGE_CONTROLLER_EXPECTED_GOAL_TIME_SEC",
            max(45.0, bridge_controller_goal_time_tol),
        ),
    )
    bridge_controller_path_tol = max(0.0, _env_float("PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD", 4.0))
    bridge_controller_goal_tol = max(0.0, _env_float("PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD", 0.20))
    bridge_velocity_scale = max(0.05, min(1.0, _env_float("PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE", 0.30)))
    bridge_accel_scale = max(0.05, min(1.0, _env_float("PANEL_MOVEIT_BRIDGE_ACCEL_SCALE", 0.30)))
    return [
        {"use_sim_time": use_sim_time},
        {"backend": "auto"},
        {"base_frame": "base_link"},
        {"ee_frame": "rg2_tcp"},
        {"result_topic": "/desired_grasp/result"},
        {"controller_manager": LaunchConfiguration("controller_manager")},
        {"execute_timeout_sec": bridge_exec_timeout},
        {"request_timeout_sec": bridge_request_timeout},
        {"stale_request_ttl_sec": bridge_stale_request_ttl_sec},
        {"joint_state_valid_timeout_sec": bridge_joint_state_timeout},
        {"joint_state_valid_max_age_sec": bridge_joint_state_max_age},
        {"force_fjt_direct_for_walltime_sim": bridge_force_fjt_direct},
        {"unwrap_continuous_joints": bridge_unwrap_continuous},
        {"require_request_id": bridge_require_request_id},
        {"drop_pending_on_tagged_request": bridge_drop_pending},
        {"dry_run_plan_only": bridge_dry_run},
        {"path_constraint_joint_tolerance_rad": bridge_path_constraint_tol},
        {"controller_goal_time_tolerance_sec": bridge_controller_goal_time_tol},
        {"controller_expected_goal_time_sec": bridge_controller_expected_goal_time},
        {"controller_path_tolerance_rad": bridge_controller_path_tol},
        {"controller_goal_tolerance_rad": bridge_controller_goal_tol},
        {"max_velocity_scaling_factor": bridge_velocity_scale},
        {"max_acceleration_scaling_factor": bridge_accel_scale},
    ]


def _build_system_state_and_attach_extras():
    """F3-step26b: extras runtime para system_state_manager y attach_backend
    (~36 LOC). Lee env vars con _resolve_runtime y devuelve tupla
    (system_state_extras, attach_extras, demo_transport_objects).
    """
    system_state_extras = [
        {"geometry_offset_tol_m": float(_resolve_runtime("SYSTEM_STATE_GEOMETRY_OFFSET_TOL_M", "0.002"))},
        {"geometry_pair_tol_m": float(_resolve_runtime("SYSTEM_STATE_GEOMETRY_PAIR_TOL_M", "0.001"))},
        {"startup_timeout_sec": float(_resolve_runtime("SYSTEM_STATE_STARTUP_TIMEOUT_SEC", "15.0"))},
    ]
    demo_transport_objects_env = os.environ.get("ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS", "pick_demo")
    demo_transport_objects = [v.strip() for v in demo_transport_objects_env.split(",") if v.strip()]
    attach_extras = [
        {"gz_service_timeout_ms": int(_resolve_runtime("ATTACH_BACKEND_GZ_SERVICE_TIMEOUT_MS", "2000"))},
        {"gz_cmd_timeout_sec": float(_resolve_runtime("ATTACH_BACKEND_GZ_CMD_TIMEOUT_SEC", "3.0"))},
    ]
    return system_state_extras, attach_extras, demo_transport_objects


def _build_launch_arguments(world_default: str, gui_config_default: str) -> list:
    """F3-step26c: ensambla la lista de DeclareLaunchArgument (~84 LOC).

    Centraliza ~35 args que controlan el stack: world/headless/sim_time +
    flags launch_* + attach_backend params + ros2_control + moveit + rmw.
    """
    return [
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
        DeclareLaunchArgument(
            "attach_backend_mode",
            default_value=_resolve_runtime("ATTACH_BACKEND_MODE", "follow_tcp"),
        ),
        DeclareLaunchArgument(
            "strict_physics_mode",
            default_value=_resolve_runtime("STRICT_PHYSICS_MODE", "false"),
        ),
        DeclareLaunchArgument(
            "attach_backend_max_pose_age_sec",
            default_value=_resolve_runtime("ATTACH_BACKEND_MAX_POSE_AGE_SEC", "1.5"),
        ),
        DeclareLaunchArgument(
            "attach_backend_follow_rate_hz",
            default_value=_resolve_runtime("ATTACH_BACKEND_FOLLOW_RATE_HZ", "20.0"),
        ),
        DeclareLaunchArgument(
            "attach_backend_follow_break_dist_m",
            default_value=_resolve_runtime("ATTACH_BACKEND_FOLLOW_BREAK_DIST_M", "0.18"),
        ),
        DeclareLaunchArgument(
            "attach_backend_max_dist_m",
            default_value=_resolve_runtime("ATTACH_BACKEND_MAX_DIST_M", "0.08"),
        ),
        DeclareLaunchArgument("launch_moveit_bridge", default_value="true"),
        DeclareLaunchArgument("launch_tf_geometry_service", default_value="true"),
        DeclareLaunchArgument("launch_object_pose_resolver", default_value="true"),
        DeclareLaunchArgument("launch_plan_to_pose_server", default_value="true"),
        DeclareLaunchArgument("launch_scene_sync", default_value="true"),
        DeclareLaunchArgument("launch_system_state", default_value="true"),
        DeclareLaunchArgument("launch_moveit", default_value="false"),
        DeclareLaunchArgument("moveit_mode", default_value="auto"),
        DeclareLaunchArgument("camera_required", default_value="1"),
        DeclareLaunchArgument("moveit_start_ros2_control", default_value="false"),
        DeclareLaunchArgument("bootstrap_controllers", default_value="true"),
        DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
        DeclareLaunchArgument("panel_auto_bridge", default_value="0"),
        DeclareLaunchArgument("panel_auto_bridge_delay_ms", default_value="1200"),
        DeclareLaunchArgument("panel_managed", default_value="1"),
        DeclareLaunchArgument("rmw_implementation", default_value="rmw_fastrtps_cpp"),
        DeclareLaunchArgument("render_engine", default_value="ogre2"),
        DeclareLaunchArgument("gui_config_file", default_value=gui_config_default),
    ]


def generate_launch_description():
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
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
            {"clock_timeout_sec": 12.0},
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
        _build_system_state_and_attach_extras()
    )
    bridge_params = _build_bridge_params(use_sim_time)

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
        additional_env={"PANEL_CAMERA_REQUIRED": LaunchConfiguration("camera_required")},
        condition=IfCondition(launch_panel),
    )

    return LaunchDescription(
        [
            *_build_launch_arguments(world_default, gui_config_default),
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
