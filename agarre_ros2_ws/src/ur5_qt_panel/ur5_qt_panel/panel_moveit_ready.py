#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_moveit_ready.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""MoveIt readiness helpers for the panel."""
from __future__ import annotations


def moveit_topics_ready(panel) -> bool:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return False
    return panel._topic_has_any_publishers(
        [
            "/move_action/status",
            "/move_group/status",
            "/planning_scene",
        ]
    )


def moveit_status_ready(panel) -> bool:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return False
    if moveit_topics_ready(panel):
        return True
    return (
        panel.ros_worker.has_service("/get_planning_scene")
        or panel.ros_worker.has_service("/move_group/get_planning_scene")
    )


def moveit_action_ready(panel) -> bool:
    if panel.ActionClient is None or panel.MoveGroup is None:
        return False
    if panel._moveit_node is None:
        try:
            panel._ensure_moveit_node()
        except Exception as exc:
            panel._log_warning(f"[MOVEIT] init moveit publisher: {exc}")
            return False
    if panel._moveit_node is None:
        return False
    action_names = ["/move_action", "/move_group"]
    for name in action_names:
        panel._moveit_action_client = panel._get_action_client(
            name,
            panel.MoveGroup,
            log_ctx="moveit_action",
        )
        if panel._moveit_action_client is None:
            continue
        if panel._wait_action_server(
            panel._moveit_action_client,
            timeout_sec=0.2,
            log_ctx="moveit_action",
            action_name=name,
        ):
            return True
    return False
