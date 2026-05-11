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
    TimerAction,
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
      - ``gz sim -s`` (servidor único, mismo proceso en headless o GUI mode).
      - ``gz sim -g`` (cliente GUI, sólo si ``headless=false``).
      - ``ros_gz_bridge/parameter_bridge`` con el YAML runtime.
      - ``ur5_tools/gz_ros_control_guard`` (guardia controlador).
      - ``ur5_tools/gz_pose_bridge`` (publicación de poses Gazebo).
      - ``RegisterEventHandler`` para shutdown coordinado al exit
        de cualquier proceso crítico.

    F1.2 audit (2026-05-10): consolidado el servidor en una sola
    ``ExecuteProcess`` ``gz_server`` (antes había ``gz_headless`` y
    ``gz_gui_server`` con el mismo comando, mutuamente exclusivos por
    condition) — simplifica el wiring y el shutdown coordinado.

    Cada acción está condicionada por su flag de launch.
    """
    gz_server = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            world_file,
        ],
        output="screen",
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
        actions=[gz_server, gz_gui],
        condition=IfCondition(launch_gazebo),
    )
    gz_shutdown_server = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_server,
            on_exit=[EmitEvent(event=Shutdown(reason="gz sim server exited"))],
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

    # Fix 2026-05-11: spawn UR5+RG2 desde /robot_description (URDF).
    # El SDF custom (`models/ur5_rg2/model.sdf`) tenía convención cinemática
    # distinta a la URDF y causaba ~1.3m de desfase entre tool0 TF (panel)
    # y tool0 físico (Gazebo). Ahora la URDF es la única fuente de verdad
    # cinemática para Gazebo, RSP y MoveIt.
    #
    # El `world` link del URDF se mapea automáticamente al world frame
    # de Gazebo (convención gz-sim). La URDF spawnea base_link a
    # (-0.85, 0, 0.85) vía el `base_joint` del macro ur_robot.
    #
    # TimerAction 5s: damos margen para que `gz sim` esté listo y RSP
    # haya publicado /robot_description antes de invocar `create`.
    spawn_robot_create = Node(
        package="ros_gz_sim",
        executable="create",
        name="ur5_rg2_spawner",
        output="screen",
        arguments=[
            "-name", "ur5_rg2",
            "-topic", "robot_description",
            "-allow_renaming", "false",
            "-x", "-0.85",
            "-y", "0.0",
            "-z", "0.85",
        ],
        condition=IfCondition(launch_gazebo),
    )
    spawn_robot = TimerAction(
        period=5.0,
        actions=[spawn_robot_create],
        condition=IfCondition(launch_gazebo),
    )
    # Anchor attach se dispara DESPUÉS de que el spawner termine, no por
    # timer ciego (que disparaba ANTES de que el modelo existiera).
    anchor_attach_on_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot_create,
            on_exit=[
                # Esperamos 3s adicionales para que gz_ros2_control cargue
                # los controllers; luego attach world_anchor.
                TimerAction(
                    period=3.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2", "topic", "pub", "--once",
                                "/world_anchor/attach",
                                "std_msgs/msg/Empty", "{}",
                            ],
                            output="screen",
                        ),
                    ],
                ),
            ],
        ),
        condition=IfCondition(launch_gazebo),
    )

    return [
        gz_group,
        gz_shutdown_server,
        bridge,
        bridge_guard,
        gz_control_guard,
        gz_pose_bridge,
        gz_pose_guard,
        spawn_robot,
        anchor_attach_on_spawn,
    ]
