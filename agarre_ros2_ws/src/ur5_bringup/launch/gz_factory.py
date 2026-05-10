#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/gz_factory.py
# Contenido: F17 — factory de actions del subgrupo Gazebo (gz sim + bridge + pose bridge).
"""Factory de actions del subgrupo Gazebo del stack UR5.

Extraído de ``ur5_stack.launch.py`` en F17 (2026-05-01) para reducir
el tamaño del launch principal y mejorar la separación de
responsabilidades.

Uso desde ``ur5_stack.launch.py``::

    from gz_factory import build_gz_actions
    gz_actions = build_gz_actions(
        world_file=world_file,
        render_engine=render_engine,
        gui_config_file=gui_config_file,
        headless=headless,
        launch_gazebo=launch_gazebo,
        launch_bridge=launch_bridge,
        runtime_yaml=runtime_yaml,
        world_name=world_name,
        use_sim_time=use_sim_time,
    )

Devuelve una lista de actions lista para incluir en el
``LaunchDescription``. No cambia el comportamiento runtime —
únicamente extrae el wiring de ``gz sim`` y bridges.
"""

from __future__ import annotations

from typing import List

from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def build_gz_actions(
    *,
    world_file,
    render_engine,
    gui_config_file,
    headless,
    launch_gazebo,
    launch_bridge,
    runtime_yaml,
    world_name,
    use_sim_time,
) -> List[object]:
    """Construye las actions de Gazebo + bridges para el stack UR5.

    Incluye:
      - ``gz sim`` headless o GUI (según ``headless``).
      - ``ros_gz_bridge/parameter_bridge`` con el YAML runtime.
      - ``ur5_tools/gz_ros_control_guard`` (guardia controlador).
      - ``ur5_tools/gz_pose_bridge`` (publicación de poses Gazebo).
      - ``RegisterEventHandler`` para shutdown coordinado al exit
        de cualquier proceso crítico.

    Cada acción está condicionada por su flag de launch.
    """
    gz_headless = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            world_file,
        ],
        output="screen",
        condition=IfCondition(headless),
    )
    gz_gui_server = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            world_file,
        ],
        output="screen",
        condition=UnlessCondition(headless),
    )
    # F-audit (2026-05-10): forzar `--render-engine` también en el cliente
    # GUI. Algunos sistemas (Mesa + libOgreNextMain.so 2.3.3) segfaultean
    # con el default OGRE2; respetar la elección del usuario via
    # GZ_RENDER_ENGINE evita SIGSEGV reproducible en arranques con GUI.
    gz_gui = ExecuteProcess(
        cmd=[
            "gz", "sim", "-g",
            "--render-engine", render_engine,
            "--gui-config", gui_config_file,
        ],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_group = GroupAction(
        actions=[gz_headless, gz_gui_server, gz_gui],
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
            target_action=gz_gui_server,
            on_exit=[
                EmitEvent(event=Shutdown(reason="gz sim server exited (gui mode)"))
            ],
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
            {"config_file": runtime_yaml},
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
            {"world_name": world_name},
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

    return [
        gz_group,
        gz_shutdown_headless,
        gz_shutdown_gui,
        bridge,
        bridge_guard,
        gz_control_guard,
        gz_pose_bridge,
        gz_pose_guard,
    ]
