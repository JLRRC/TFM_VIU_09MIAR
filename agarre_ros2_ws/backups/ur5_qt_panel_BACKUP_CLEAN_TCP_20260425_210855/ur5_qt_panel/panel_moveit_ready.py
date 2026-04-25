#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_moveit_ready.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""MoveIt readiness helpers for the panel."""
from __future__ import annotations


def _emit_moveit_ready_diag(panel, stage: str, detail: str) -> None:
    emit_throttled = getattr(panel, "_emit_log_throttled", None)
    if callable(emit_throttled):
        emit_throttled(
            f"MOVEIT:READY:{stage}",
            f"[MOVEIT2][READY_DIAG] stage={stage} {detail}",
            min_interval=1.0,
        )
        return
    emit_log = getattr(panel, "_emit_log", None)
    if callable(emit_log):
        emit_log(f"[MOVEIT2][READY_DIAG] stage={stage} {detail}")


def moveit_topics_ready(panel) -> bool:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        _emit_moveit_ready_diag(
            panel,
            "topics",
            "ready=false node_ready=false move_action_status=false move_group_status=false planning_scene=false",
        )
        return False
    move_action_status = panel.ros_worker.topic_has_publishers("/move_action/status")
    move_group_status = panel.ros_worker.topic_has_publishers("/move_group/status")
    planning_scene = panel.ros_worker.topic_has_publishers("/planning_scene")
    ready = bool(move_action_status or move_group_status or planning_scene)
    _emit_moveit_ready_diag(
        panel,
        "topics",
        f"ready={str(ready).lower()} node_ready=true "
        f"move_action_status={str(bool(move_action_status)).lower()} "
        f"move_group_status={str(bool(move_group_status)).lower()} "
        f"planning_scene={str(bool(planning_scene)).lower()}",
    )
    return ready


def moveit_status_ready(panel) -> bool:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        _emit_moveit_ready_diag(
            panel,
            "status",
            "ready=false node_ready=false topics=false get_planning_scene=false move_group_get_planning_scene=false",
        )
        return False
    topics_ready = moveit_topics_ready(panel)
    service_global = panel.ros_worker.has_service("/get_planning_scene")
    service_namespaced = panel.ros_worker.has_service("/move_group/get_planning_scene")
    ready = bool(topics_ready or service_global or service_namespaced)
    _emit_moveit_ready_diag(
        panel,
        "status",
        f"ready={str(ready).lower()} node_ready=true "
        f"topics={str(bool(topics_ready)).lower()} "
        f"get_planning_scene={str(bool(service_global)).lower()} "
        f"move_group_get_planning_scene={str(bool(service_namespaced)).lower()}",
    )
    return ready


def moveit_action_ready(panel) -> bool:
    if panel.ActionClient is None or panel.MoveGroup is None:
        _emit_moveit_ready_diag(
            panel,
            "action",
            "ready=false reason=action_client_or_movegroup_missing",
        )
        return False
    if panel._moveit_node is None:
        try:
            panel._ensure_moveit_node()
        except Exception as exc:
            panel._log_warning(f"[MOVEIT] init moveit publisher: {exc}")
            _emit_moveit_ready_diag(
                panel,
                "action",
                f"ready=false reason=ensure_moveit_node_exception err={exc}",
            )
            return False
    if panel._moveit_node is None:
        _emit_moveit_ready_diag(
            panel,
            "action",
            "ready=false reason=moveit_node_none",
        )
        return False
    action_names = ["/move_action", "/move_group"]
    action_results = []
    for name in action_names:
        panel._moveit_action_client = panel._get_action_client(
            name,
            panel.MoveGroup,
            log_ctx="moveit_action",
        )
        if panel._moveit_action_client is None:
            action_results.append(f"{name}=client_none")
            continue
        action_ok = panel._wait_action_server(
            panel._moveit_action_client,
            timeout_sec=0.2,
            log_ctx="moveit_action",
            action_name=name,
        )
        action_results.append(f"{name}={'ok' if action_ok else 'timeout'}")
        if action_ok:
            _emit_moveit_ready_diag(
                panel,
                "action",
                "ready=true " + " ".join(action_results),
            )
            return True
    _emit_moveit_ready_diag(
        panel,
        "action",
        "ready=false " + " ".join(action_results or ["no_actions_checked"]),
    )
    return False
