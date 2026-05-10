#!/usr/bin/env python3
"""F7 (auditoría 2026-05-10): MoveIt readiness helpers.

Funciones que evalúan disponibilidad de MoveIt y de su action server
``follow_joint_trajectory`` desde el panel. Cada función recibe ``panel``
(instancia de ControlPanelV2) como primer argumento y consulta atributos
cacheados / estado del ros_worker.

Extraídas de ``panel_status_mgmt.py`` líneas ~495–670. ``panel_status_mgmt``
re-exporta cada símbolo para preservar la API que usan los mixins
``panel_v2_*_mixin.py`` (que importan ``panel_status_mgmt as _sm``).
"""
from __future__ import annotations

import time
from typing import List, Optional

from .panel_config import (
    MOVEIT_POSE_TOPIC,
    ROS_AVAILABLE,
    STATUS_TOPIC_CACHE_SEC,
    WORLD_FRAME,
)
from .panel_moveit_ready import (
    moveit_action_ready,
    moveit_status_ready,
    moveit_topics_ready,
)
from .panel_ros_params import get_panel_ros_params as _get_panel_ros_params

try:
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
except Exception:
    ActionClient = FollowJointTrajectory = None  # type: ignore[assignment,misc]


def _moveit_topics_ready(panel) -> bool:
    return moveit_topics_ready(panel)


def _moveit_status_ready(panel) -> bool:
    return moveit_status_ready(panel)


def _moveit_action_ready(panel) -> bool:
    return moveit_action_ready(panel)


def _list_topic_names(panel) -> List[str]:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return []
    try:
        return panel.ros_worker.list_topic_names()
    except Exception:
        return []


def _list_action_names(panel) -> List[str]:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return []
    try:
        return panel.ros_worker.list_action_names()
    except Exception:
        return []


def _topic_has_any_publishers(panel, topics: List[str]) -> bool:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return False
    for topic in topics:
        if panel.ros_worker.topic_has_publishers(topic):
            return True
    return False


def _world_frame_last_first(panel, fallback: Optional[str] = None) -> str:
    frame = (
        fallback
        or WORLD_FRAME
        or panel._last_selection_frame
        or "world"
    )
    frame_norm = str(frame or "").split("|", 1)[0].strip() or "world"
    base_frame = str(panel._business_base_frame() or "base_link").strip() or "base_link"
    if frame_norm in {base_frame, "base", "tool0", "rg2_tcp", "rg2_pinch_center"}:
        # Guardrail: las poses de pose/info llegan en world, no en base_link.
        # Si este valor se contamina, los cálculos geométricos se desalinean.
        frame_norm = str(WORLD_FRAME or "world").strip() or "world"
        panel._emit_log_throttled(
            "FRAME:world_frame_guard",
            f"[FRAME] world_frame inválido ({frame}); usando {frame_norm}",
            min_interval=2.0,
        )
    return frame_norm


def _world_frame_config_first(panel) -> str:
    return panel._world_frame_last_first(WORLD_FRAME or "world")


def _follow_joint_traj_ready(panel) -> bool:
    if not ROS_AVAILABLE or ActionClient is None or FollowJointTrajectory is None:
        return False
    _ros_params = _get_panel_ros_params()
    strict_action = _ros_params.strict_traj_action
    expected_action = _ros_params.expected_traj_action.strip()
    action_names = set(panel._list_action_names())
    action_topics = set(panel._list_topic_names())

    def _action_graph_ready(action_name: str) -> bool:
        if not action_name:
            return False
        status_topic = f"{action_name}/_action/status"
        feedback_topic = f"{action_name}/_action/feedback"
        goal_topic = f"{action_name}/_action/send_goal"
        if status_topic in action_topics or feedback_topic in action_topics or goal_topic in action_topics:
            return True
        if not panel.ros_worker or not panel.ros_worker.node_ready():
            return False
        return bool(
            panel.ros_worker.topic_has_publishers(status_topic)
            or panel.ros_worker.topic_has_publishers(feedback_topic)
            or panel.ros_worker.topic_has_subscribers(goal_topic)
        )

    if strict_action and expected_action:
        if expected_action not in action_names:
            if not _action_graph_ready(expected_action):
                return False
        action_name = expected_action
    else:
        action_name = ""

    # Fallback de robustez: si el action server esperado ya existe en el grafo
    # ROS pero el nodo local de MoveIt aun no esta inicializado, aceptar READY.
    if panel._moveit_node is None:
        if strict_action and expected_action:
            return expected_action in action_names or _action_graph_ready(expected_action)
        return bool(action_names)

    traj_topic = panel._select_traj_topic()
    if not action_name:
        action_name = panel._resolve_traj_action_name(traj_topic, allow_fallback=True)
    if not action_name:
        return False
    if panel._traj_action_client is None or panel._traj_action_name != action_name:
        panel._traj_action_client = panel._get_action_client(
            action_name,
            FollowJointTrajectory,
            log_ctx="follow_traj",
        )
    client = panel._traj_action_client
    if client is None:
        return False
    return panel._wait_action_server(
        client,
        timeout_sec=0.2,
        log_ctx="follow_traj",
        action_name=action_name,
    )


def _moveit_bridge_detected(panel) -> bool:
    if panel._proc_alive(panel.moveit_bridge_proc):
        panel._moveit_bridge_detected_cache = True
        panel._moveit_bridge_detected_ts = time.monotonic()
        return True
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False
    now = time.monotonic()
    if (
        STATUS_TOPIC_CACHE_SEC > 0.0
        and (now - panel._moveit_bridge_detected_ts) < STATUS_TOPIC_CACHE_SEC
    ):
        return panel._moveit_bridge_detected_cache
    pose_ready = (
        panel.ros_worker.topic_has_subscribers(MOVEIT_POSE_TOPIC)
        or panel.ros_worker.topic_has_subscribers("/grasp_pose")
    )
    result_ready = panel.ros_worker.topic_has_publishers("/desired_grasp/result")
    detected = pose_ready and result_ready
    panel._moveit_bridge_detected_cache = bool(detected)
    panel._moveit_bridge_detected_ts = now
    return detected


def _move_group_startup_ready(panel) -> bool:
    status_ready = panel._moveit_status_ready()
    action_ready = panel._moveit_action_ready()
    ready = status_ready or action_ready
    ros_node_ready = bool(panel.ros_worker and panel.ros_worker.node_ready())
    panel._emit_log_throttled(
        "MOVEIT:startup_gate",
        "[MOVEIT2][STARTUP_GATE] "
        f"ready={str(bool(ready)).lower()} "
        f"status={str(bool(status_ready)).lower()} "
        f"action={str(bool(action_ready)).lower()} "
        f"moveit_proc={str(bool(panel._proc_alive(panel.moveit_proc))).lower()} "
        f"ros_worker_started={str(bool(panel._ros_worker_started)).lower()} "
        f"ros_node_ready={str(bool(ros_node_ready)).lower()} "
        f"moveit_state={panel._moveit_state.value}",
        min_interval=1.0,
    )
    return ready


def _move_group_ready(panel) -> bool:
    return panel._move_group_startup_ready() and panel._follow_joint_traj_ready()


def _moveit_ready(panel) -> bool:
    if panel._moveit_bridge_detected():
        return True
    return panel._move_group_startup_ready()


def _update_moveit_status_label(panel) -> None:
    if panel.lbl_moveit_status is not None:
        state_label = panel._moveit_state.value
        panel.lbl_moveit_status.setText("MoveIt")
        reason = panel._moveit_state_reason or state_label
        panel.lbl_moveit_status.setToolTip(f"{state_label}: {reason}")
    if panel.lbl_moveit_bridge_status is not None:
        bridge_label = "ON" if panel._moveit_bridge_running else "OFF"
        panel.lbl_moveit_bridge_status.setText("MoveIt bridge")


__all__ = [
    "_moveit_topics_ready",
    "_moveit_status_ready",
    "_moveit_action_ready",
    "_list_topic_names",
    "_list_action_names",
    "_topic_has_any_publishers",
    "_world_frame_last_first",
    "_world_frame_config_first",
    "_follow_joint_traj_ready",
    "_moveit_bridge_detected",
    "_move_group_startup_ready",
    "_move_group_ready",
    "_moveit_ready",
    "_update_moveit_status_label",
]
