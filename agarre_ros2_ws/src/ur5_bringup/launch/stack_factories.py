#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/stack_factories.py
"""F3-step39: factories del stack extraídas de ur5_stack.launch.py.

Cuatro builders puros (cada uno aceptado por launch framework):

* ``build_runtime_environment_actions`` — ensambla SetEnvironmentVariable +
  SetLaunchConfiguration con el set de env vars consolidado del stack.
* ``build_bridge_params`` — parámetros para ur5_moveit_bridge a partir de
  16 env vars PANEL_MOVEIT_BRIDGE_* con clamp y defaults.
* ``build_system_state_and_attach_extras`` — extras runtime para
  system_state_manager y attach_backend.
* ``build_launch_arguments`` — DeclareLaunchArgument list (~35 args).

Importado por ``ur5_stack.launch.py``. Sin lógica nueva — solo extracción
mecánica para reducir LOC del archivo principal.
"""

from __future__ import annotations

import os
from typing import List

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

from launch_helpers import PANEL_ENV_DEFAULTS


def build_runtime_environment_actions(
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
    """Ensambla la lista completa de SetEnvironmentVariable +
    SetLaunchConfiguration para _prepare_runtime."""
    return [
        SetEnvironmentVariable("WS_DIR", ws_dir),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("GZ_RENDER_ENGINE", render_engine),
        SetEnvironmentVariable("QT_QPA_PLATFORM", os.environ.get("QT_QPA_PLATFORM", "offscreen")),
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


def build_bridge_params(use_sim_time, env_float_fn, env_flag_fn) -> list:
    """Parámetros de Node para ur5_moveit_bridge.

    Lee 16 env vars PANEL_MOVEIT_BRIDGE_* con clamp + default + devuelve
    lista de dicts compatible con el ``parameters=[...]`` de Node.

    ``env_float_fn`` y ``env_flag_fn`` son los lectores del
    archivo principal (resuelven env > runtime_yaml > default literal).
    """
    bridge_exec_timeout = max(1.0, env_float_fn("PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC", 150.0))
    bridge_request_timeout = max(2.0, env_float_fn("PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC", 180.0))
    bridge_stale_request_ttl_sec = max(20.0, env_float_fn("PANEL_MOVEIT_BRIDGE_STALE_REQUEST_TTL_SEC", 120.0))
    bridge_joint_state_timeout = max(0.2, env_float_fn("PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC", 6.0))
    bridge_joint_state_max_age = max(0.1, env_float_fn("PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC", 2.5))
    bridge_force_fjt_direct = env_flag_fn("PANEL_MOVEIT_BRIDGE_FORCE_FJT_DIRECT", False)
    bridge_unwrap_continuous = env_flag_fn("PANEL_MOVEIT_BRIDGE_UNWRAP_CONTINUOUS_JOINTS", False)
    bridge_require_request_id = env_flag_fn("PANEL_MOVEIT_BRIDGE_REQUIRE_REQUEST_ID", True)
    bridge_drop_pending = env_flag_fn("PANEL_MOVEIT_BRIDGE_DROP_PENDING_ON_TAGGED", True)
    bridge_dry_run = env_flag_fn("PANEL_MOVEIT_BRIDGE_DRY_RUN", False)
    bridge_path_constraint_tol = max(0.0, env_float_fn("PANEL_MOVEIT_BRIDGE_PATH_CONSTRAINT_TOL_RAD", 0.35))
    bridge_controller_goal_time_tol = max(0.0, env_float_fn("PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TIME_TOL_SEC", 45.0))
    bridge_controller_expected_goal_time = max(
        0.0,
        env_float_fn(
            "PANEL_MOVEIT_BRIDGE_CONTROLLER_EXPECTED_GOAL_TIME_SEC",
            max(45.0, bridge_controller_goal_time_tol),
        ),
    )
    bridge_controller_path_tol = max(0.0, env_float_fn("PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD", 4.0))
    bridge_controller_goal_tol = max(0.0, env_float_fn("PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD", 0.20))
    bridge_velocity_scale = max(0.05, min(1.0, env_float_fn("PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE", 0.30)))
    bridge_accel_scale = max(0.05, min(1.0, env_float_fn("PANEL_MOVEIT_BRIDGE_ACCEL_SCALE", 0.30)))
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


def build_system_state_and_attach_extras(resolve_runtime_fn):
    """Extras runtime para system_state_manager y attach_backend.

    Devuelve (system_state_extras, attach_extras, demo_transport_objects).
    ``resolve_runtime_fn(name, default)`` es el resolver del archivo principal.
    """
    system_state_extras = [
        {"geometry_offset_tol_m": float(resolve_runtime_fn("SYSTEM_STATE_GEOMETRY_OFFSET_TOL_M", "0.002"))},
        {"geometry_pair_tol_m": float(resolve_runtime_fn("SYSTEM_STATE_GEOMETRY_PAIR_TOL_M", "0.001"))},
        {"startup_timeout_sec": float(resolve_runtime_fn("SYSTEM_STATE_STARTUP_TIMEOUT_SEC", "15.0"))},
    ]
    demo_transport_objects_env = os.environ.get("ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS", "pick_demo")
    demo_transport_objects = [v.strip() for v in demo_transport_objects_env.split(",") if v.strip()]
    attach_extras = [
        {"gz_service_timeout_ms": int(resolve_runtime_fn("ATTACH_BACKEND_GZ_SERVICE_TIMEOUT_MS", "2000"))},
        {"gz_cmd_timeout_sec": float(resolve_runtime_fn("ATTACH_BACKEND_GZ_CMD_TIMEOUT_SEC", "3.0"))},
    ]
    return system_state_extras, attach_extras, demo_transport_objects


def build_launch_arguments(
    *,
    world_default: str,
    gui_config_default: str,
    resolve_runtime_fn,
) -> list:
    """Lista de DeclareLaunchArgument (~35 args).

    Centraliza los args que controlan el stack: world/headless/sim_time +
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
            default_value=resolve_runtime_fn("ATTACH_BACKEND_MODE", "follow_tcp"),
        ),
        DeclareLaunchArgument(
            "strict_physics_mode",
            default_value=resolve_runtime_fn("STRICT_PHYSICS_MODE", "false"),
        ),
        DeclareLaunchArgument(
            "attach_backend_max_pose_age_sec",
            default_value=resolve_runtime_fn("ATTACH_BACKEND_MAX_POSE_AGE_SEC", "1.5"),
        ),
        DeclareLaunchArgument(
            "attach_backend_follow_rate_hz",
            default_value=resolve_runtime_fn("ATTACH_BACKEND_FOLLOW_RATE_HZ", "20.0"),
        ),
        DeclareLaunchArgument(
            "attach_backend_follow_break_dist_m",
            default_value=resolve_runtime_fn("ATTACH_BACKEND_FOLLOW_BREAK_DIST_M", "0.18"),
        ),
        DeclareLaunchArgument(
            "attach_backend_max_dist_m",
            default_value=resolve_runtime_fn("ATTACH_BACKEND_MAX_DIST_M", "0.08"),
        ),
        DeclareLaunchArgument("launch_moveit_bridge", default_value="true"),
        DeclareLaunchArgument("launch_tf_geometry_service", default_value="true"),
        DeclareLaunchArgument("launch_object_pose_resolver", default_value="true"),
        DeclareLaunchArgument("launch_plan_to_pose_server", default_value="true"),
        DeclareLaunchArgument("launch_pick_orchestrator_lifecycle", default_value="true"),
        DeclareLaunchArgument("pick_orchestrator_use_stubs", default_value="false"),
        DeclareLaunchArgument("launch_scene_sync", default_value="true"),
        DeclareLaunchArgument("launch_system_state", default_value="true"),
        DeclareLaunchArgument("launch_moveit", default_value="true"),
        DeclareLaunchArgument("moveit_mode", default_value="auto"),
        DeclareLaunchArgument(
            "camera_required",
            default_value=resolve_runtime_fn("PANEL_CAMERA_REQUIRED", "0"),
        ),
        DeclareLaunchArgument("moveit_start_ros2_control", default_value="false"),
        DeclareLaunchArgument("bootstrap_controllers", default_value="true"),
        DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
        DeclareLaunchArgument("panel_auto_bridge", default_value="0"),
        DeclareLaunchArgument("panel_auto_bridge_delay_ms", default_value="1200"),
        DeclareLaunchArgument("panel_managed", default_value="1"),
        DeclareLaunchArgument("rmw_implementation", default_value="rmw_fastrtps_cpp"),
        DeclareLaunchArgument("render_engine", default_value=resolve_runtime_fn("GZ_RENDER_ENGINE", "ogre")),
        DeclareLaunchArgument("gui_config_file", default_value=gui_config_default),
    ]
