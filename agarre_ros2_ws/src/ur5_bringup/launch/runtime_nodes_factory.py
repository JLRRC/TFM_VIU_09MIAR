#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/runtime_nodes_factory.py
# Contenido: F17 — factory de actions de los nodos runtime del stack (TF, state, attach, MoveIt bridge).
"""Factory de actions de los nodos runtime del stack UR5.

Extraído de ``ur5_stack.launch.py`` en F17 (2026-05-01) para reducir
el tamaño del launch principal y mejorar la separación de
responsabilidades.

Incluye:
  - ``world_tf_publisher`` (LifecycleNode F13).
  - ``system_state_manager`` (LifecycleNode F13).
  - ``release_objects_service``.
  - ``gripper_attach_backend`` (LifecycleNode F13 observable).
  - ``planning_scene_sync``.
  - ``ur5_moveit_bridge`` (LifecycleNode F13 observable).

Uso desde ``ur5_stack.launch.py``::

    from runtime_nodes_factory import build_runtime_node_actions
    runtime_actions = build_runtime_node_actions(
        use_sim_time=use_sim_time,
        ...
    )

Devuelve una lista de actions lista para incluir en el
``LaunchDescription``. No cambia el comportamiento runtime — solo
extrae el wiring.
"""

from __future__ import annotations

from typing import Any, Dict, List

from launch.actions import (
    EmitEvent,
    GroupAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def build_runtime_node_actions(
    *,
    use_sim_time: LaunchConfiguration,
    world_name: LaunchConfiguration,
    world_file: LaunchConfiguration,
    camera_required: LaunchConfiguration,
    controller_manager: LaunchConfiguration,
    launch_moveit: LaunchConfiguration,
    launch_world_tf: LaunchConfiguration,
    launch_system_state: LaunchConfiguration,
    launch_release_service: LaunchConfiguration,
    launch_attach_backend: LaunchConfiguration,
    launch_scene_sync: LaunchConfiguration,
    launch_moveit_bridge: LaunchConfiguration,
    launch_tf_geometry_service: LaunchConfiguration,
    gz_delete_service: LaunchConfiguration,
    gz_spawn_service: LaunchConfiguration,
    attach_backend_mode: LaunchConfiguration,
    attach_backend_max_pose_age_sec: LaunchConfiguration,
    attach_backend_follow_rate_hz: LaunchConfiguration,
    attach_backend_follow_break_dist_m: LaunchConfiguration,
    attach_backend_max_dist_m: LaunchConfiguration,
    demo_transport_objects: List[str],
    bridge_params: List[Dict[str, Any]],
    system_state_yaml,
    system_state_extras: List[Dict[str, Any]],
    attach_extras: List[Dict[str, Any]],
) -> List[object]:
    """Construye las actions de los nodos runtime del stack UR5.

    Cada Node está condicionado por su flag de launch. Los nodos críticos
    al pipeline (system_state) tienen un ``RegisterEventHandler`` que
    derriba el stack si terminan inesperadamente; el ``world_tf_publisher``
    es no-crítico y no propaga shutdown al stack.
    """
    world_tf = Node(
        package="ur5_tools",
        executable="world_tf_publisher",
        output="screen",
        parameters=[
            {"world_name": world_name},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"clock_timeout_sec": 20.0},
            {"pose_timeout_sec": 12.0},
            {"use_sim_time": use_sim_time},
            # static_grace_sec < 0: publish world->base_link on /tf_static
            # immediately from the world file pose, without waiting for
            # /clock or Gazebo. Eliminates the TF-null window during the
            # first 3-5 minutes of startup.
            {"static_grace_sec": -1.0},
        ],
        condition=IfCondition(launch_world_tf),
    )
    # world_tf_publisher is helpful but non-critical; do not shutdown the
    # whole stack if it exits.
    world_tf_guard = GroupAction(actions=[])

    system_state = Node(
        package="ur5_tools",
        executable="system_state_manager",
        output="screen",
        parameters=[
            system_state_yaml,
            {"use_sim_time": use_sim_time},
            {"world_name": world_name},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"ee_frame": "rg2_tcp"},
            {"camera_topic": "/camera_overhead/image"},
            {"camera_required": ParameterValue(camera_required, value_type=bool)},
            {"controller_manager": controller_manager},
            {"moveit_required": ParameterValue(launch_moveit, value_type=bool)},
            *system_state_extras,
        ],
        condition=IfCondition(launch_system_state),
    )

    system_state_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=system_state,
            on_exit=[
                EmitEvent(event=Shutdown(reason="system_state_manager exited"))
            ],
        ),
        condition=IfCondition(launch_system_state),
    )

    release_service = Node(
        package="ur5_tools",
        executable="release_objects_service",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_sdf": world_file},
            {"world_name": world_name},
            {"delete_service": gz_delete_service},
            {"spawn_service": gz_spawn_service},
            {"settle_timeout_sec": 12.0},
            {"settle_confirmations": 6},
        ],
        condition=IfCondition(launch_release_service),
    )

    gripper_attach_backend_params: List[Dict[str, Any]] = [
        {"use_sim_time": use_sim_time},
        {"attach_mode": attach_backend_mode},
        {"tcp_frame": "rg2_pinch_center"},
        {"tool_anchor_prefix": "/gripper_anchor"},
        {"max_pose_age_sec": attach_backend_max_pose_age_sec},
        {"follow_rate_hz": attach_backend_follow_rate_hz},
        {"follow_break_dist_m": attach_backend_follow_break_dist_m},
        # FIX-ATTACH-THRESHOLD: tighten from 0.15 m to 0.05 m so that the
        # backend only attaches when the TCP is geometrically close
        # to the object (~contact). 0.15 m allowed false grasps; 0.05 m
        # requires the gripper to be within ~5 cm.
        {"attach_max_dist_m": attach_backend_max_dist_m},
        *attach_extras,
        # detachable_shadow_follow=False: disables the physics-joint shadow
        # path for non-demo-transport objects. pick_demo goes through
        # demo_transport exclusively (demo_transport_objects check fires
        # first in _on_gripper_attach), so this flag only affects other
        # objects that use detachable_joint mode.
        {"detachable_shadow_follow": False},
    ]
    if demo_transport_objects:
        gripper_attach_backend_params.append(
            {"demo_transport_objects": demo_transport_objects}
        )

    gripper_attach_backend = Node(
        package="ur5_tools",
        executable="gripper_attach_backend",
        output="screen",
        parameters=gripper_attach_backend_params,
        condition=IfCondition(launch_attach_backend),
    )

    planning_scene_sync = Node(
        package="ur5_tools",
        executable="planning_scene_sync",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_name": world_name},
            {"world_file": world_file},
            {"world_frame": "world"},
            {"base_frame": "base_link"},
            {"ee_frame": "rg2_tcp"},
        ],
        condition=IfCondition(launch_scene_sync),
    )

    # FIX-DESIRED-GRASP: launch ur5_moveit_bridge as a standalone node so
    # that /desired_grasp always has a real subscriber in the normal boot.
    # Previously Subscription count was 0 because the bridge was only
    # started on-demand via the panel button; this makes it part of the
    # managed launch. When MoveIt (move_group) is not running the bridge
    # will subscribe but will fall back to a no-op, ensuring the topic is
    # live without crashing the stack.
    moveit_bridge = Node(
        package="ur5_tools",
        executable="ur5_moveit_bridge",
        output="screen",
        parameters=bridge_params,
        condition=IfCondition(launch_moveit_bridge),
    )

    # F16 (2026-05-01): tf_geometry_service — microservicio dedicado que
    # aloja /tf_geometry/world_to_base y /tf_geometry/compute_approach_pose.
    # LifecycleNode con auto-activate. Centraliza cálculos TF que estaban
    # duplicados entre world_tf_publisher, panel y panel_state_methods.
    tf_geometry_service = Node(
        package="ur5_tools",
        executable="tf_geometry_service",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_frame": "world"},
            {"base_frame": "base_link"},
            {"tf_timeout_sec": 0.2},
        ],
        condition=IfCondition(launch_tf_geometry_service),
    )

    return [
        world_tf,
        world_tf_guard,
        system_state,
        system_state_guard,
        release_service,
        gripper_attach_backend,
        planning_scene_sync,
        moveit_bridge,
        tf_geometry_service,
    ]
