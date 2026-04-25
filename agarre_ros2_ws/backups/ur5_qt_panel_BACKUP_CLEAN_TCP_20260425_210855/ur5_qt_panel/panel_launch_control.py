#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_launch_control.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Launch/stop control helpers for the panel."""
from __future__ import annotations

import os
import subprocess
import time

from . import panel_launchers
from .panel_config import LOG_DIR, CRITICAL_CLOCK_TIMEOUT_SEC, CRITICAL_POSE_TIMEOUT_SEC
from .panel_process import ensure_dir, rotate_log, bash_preamble, with_line_buffer, build_gz_env


def start_gazebo(panel) -> None:
    panel._critical_clock_deadline = time.monotonic() + max(0.1, CRITICAL_CLOCK_TIMEOUT_SEC)
    panel_launchers.start_gazebo(panel)


def stop_gazebo(panel) -> None:
    panel._critical_clock_deadline = 0.0
    panel_launchers.stop_gazebo(panel)


def start_bridge(panel) -> None:
    panel._critical_pose_deadline = time.monotonic() + max(0.1, CRITICAL_POSE_TIMEOUT_SEC)
    panel_launchers.start_bridge(panel)


def stop_bridge(panel) -> None:
    panel._critical_pose_deadline = 0.0
    panel_launchers.stop_bridge(panel)


def start_moveit(panel) -> None:
    panel_launchers.start_moveit(panel)


def stop_moveit(panel) -> None:
    panel_launchers.stop_moveit(panel)


def start_moveit_bridge(panel) -> None:
    panel_launchers.start_moveit_bridge(panel)


def stop_moveit_bridge(panel) -> None:
    panel_launchers.stop_moveit_bridge(panel)


def start_world_tf_publisher(panel, world_name: str) -> None:
    panel_launchers.start_world_tf_publisher(panel, world_name)


def stop_world_tf_publisher(panel) -> None:
    panel_launchers.stop_world_tf_publisher(panel)


def start_robot_state_publisher(panel) -> None:
    panel_launchers.start_robot_state_publisher(panel)


def start_release_service(panel) -> None:
    if panel.release_service_proc is not None and panel.release_service_proc.poll() is None:
        return
    try:
        ensure_dir(LOG_DIR)
        svc_log = os.path.join(LOG_DIR, "release_objects_service.log")
        rotate_log(svc_log)
        env = build_gz_env(panel.gz_partition) + f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
        cmd_core = with_line_buffer("ros2 run ur5_tools release_objects_service")
        cmd = bash_preamble(panel.ws_dir) + env + f"{cmd_core} > '{svc_log}' 2>&1"
        panel.release_service_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            preexec_fn=os.setsid,
        )
        panel._started_release_service = True
    except Exception as exc:
        panel._log_error(f"Error iniciando release_objects_service: {exc}")
