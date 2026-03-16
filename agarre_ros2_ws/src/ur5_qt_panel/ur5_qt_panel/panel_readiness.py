#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_readiness.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Readiness helpers for panel subsystems."""
from __future__ import annotations

from typing import Optional, Tuple

from .panel_utils import (
    get_tf_helper,
    _can_transform_between,
    effective_base_frame,
    effective_world_frame,
    tf_world_base_valid,
    tf_world_base_label,
)
from .panel_state import MoveItState, SystemState


def _managed_ready_status(panel) -> Tuple[bool, str]:
    if panel._state_ready_basic():
        return True, ""
    reason = panel._system_state_reason or panel._system_state.value
    return False, reason


def manual_control_status(panel) -> Tuple[bool, str]:
    if getattr(panel, "_system_state", None) is not None:
        if panel._system_state == SystemState.ERROR_FATAL:
            reason = panel._system_state_reason or SystemState.ERROR_FATAL.value
            return False, reason
    if panel._managed_mode:
        if panel._state_ready_basic():
            return True, ""
        return _managed_ready_status(panel)
    if panel._gazebo_state() != "GAZEBO_READY":
        # Fallback: when the stack is launched externally, process-based Gazebo detection can
        # transiently report OFF while /clock and pose streams are already alive.
        clock_ok, _clock_reason = panel._clock_status()
        pose_ok = panel._pose_info_active()
        if not (clock_ok and pose_ok):
            return False, "Gazebo no listo"
    if not panel._controllers_ok:
        return False, controllers_not_ready_reason(panel)
    return True, ""


def camera_ready_status(panel) -> Tuple[bool, str]:
    if panel._managed_mode:
        if not panel._camera_required:
            return True, ""
        if panel._state_ready_vision():
            return True, ""
        return _managed_ready_status(panel)
    if not panel._camera_required:
        return True, ""
    if panel._camera_stream_ok:
        return True, ""
    return False, camera_not_ready_reason(panel)


def pose_info_ready_status(panel) -> Tuple[bool, str]:
    if panel._managed_mode:
        return _managed_ready_status(panel)
    if panel._pose_info_ok:
        return True, ""
    return False, pose_info_not_ready_reason(panel)


def tf_ready_status(panel) -> Tuple[bool, str]:
    if panel._managed_mode:
        return _managed_ready_status(panel)
    checker = getattr(panel, "_tf_chain_ready_status", None)
    if callable(checker):
        try:
            ok, reason = checker()
            if ok:
                panel._tf_ready_state = True
                panel._tf_ever_ok = True
                return True, ""
            return False, f"TF no listo ({reason})"
        except Exception:
            pass
    if panel._tf_ready_state:
        return True, ""
    # FASE 1: Solo chequeamos base_link->ee (sin requerir world->base).
    helper = get_tf_helper()
    base_frame = effective_base_frame(panel)
    if helper:
        ee_frame = str(getattr(panel, "_required_ee_frame", "") or "rg2_tcp").strip() or "rg2_tcp"
        if _can_transform_between(helper, base_frame, ee_frame, timeout_sec=0.3):
            panel._tf_ready_state = True
            panel._tf_ever_ok = True
            return True, ""
    return False, tf_not_ready_reason(panel)


def tf_not_ready_reason(panel) -> str:
    checker = getattr(panel, "_tf_chain_ready_status", None)
    if callable(checker):
        try:
            ok, reason = checker()
            if not ok and reason:
                return f"TF no listo ({reason})"
        except Exception:
            pass
    # FASE 1: el mensaje indica que falta base_link->ee (no world->base).
    ee = str(getattr(panel, "_required_ee_frame", "") or "rg2_tcp").strip() or "rg2_tcp"
    base = effective_base_frame(panel)
    return f"TF {base}->{ee} no disponible"


def camera_not_ready_reason(panel) -> str:
    if panel is None:
        return "Cámara no lista"
    return panel._camera_not_ready_reason()


def pose_info_not_ready_reason(panel) -> str:
    if panel is None:
        return "pose/info no disponible"
    return panel._pose_info_not_ready_reason()


def controllers_not_ready_reason(panel) -> str:
    if panel is None:
        return "controladores no listos"
    return panel._controllers_reason or "controladores no listos"


def ros_node_not_ready_reason() -> str:
    return "Nodo ROS no listo"


def controller_manager_not_ready_reason() -> str:
    return "controller_manager no disponible"


def list_controllers_not_ready_reason(kind: str) -> str:
    return f"list_controllers no disponible ({kind})"


def moveit_not_ready_reason(panel) -> str:
    if panel is None:
        return "MoveIt no listo"
    return panel._moveit_state_reason or "MoveIt no listo"


def set_moveit_wait_status(panel, label: str, reason: Optional[str] = None) -> str:
    reason = reason or moveit_not_ready_reason(panel)
    panel._set_status(f"MoveIt no listo; esperando {label} ({reason})", error=False)
    return reason


def moveit_control_status(panel) -> Tuple[bool, str]:
    ok, reason = manual_control_status(panel)
    if not ok:
        return False, reason
    if panel._managed_mode:
        # In managed mode we allow motion control from READY_BASIC when MoveIt
        # itself is already ready, even if camera vision is still warming up.
        if panel._moveit_required and panel._moveit_state != MoveItState.READY:
            return False, moveit_not_ready_reason(panel)
        if panel._state_ready_basic():
            return True, ""
        reason = panel._system_state_reason or panel._system_state.value
        return False, reason
    if not panel._moveit_ready():
        return False, moveit_not_ready_reason(panel)
    ok, reason = camera_ready_status(panel)
    if not ok:
        return False, reason
    return True, ""


def pick_ui_status(panel) -> Tuple[bool, str]:
    ok, reason = manual_control_status(panel)
    if not ok:
        panel._emit_log_throttled(
            f"PICK_GATE:manual:{reason}",
            f"[PICK_GATE] manual_control blocked: {reason}",
        )
        return False, reason
    ok, reason = pose_info_ready_status(panel)
    if not ok:
        panel._emit_log_throttled(
            f"PICK_GATE:pose:{reason}",
            f"[PICK_GATE] pose_info blocked: {reason}",
        )
        return False, reason
    ok, reason = camera_ready_status(panel)
    if not ok:
        panel._emit_log_throttled(
            f"PICK_GATE:camera:{reason}",
            f"[PICK_GATE] camera blocked: {reason}",
        )
        return False, reason
    ok, reason = tf_ready_status(panel)
    if not ok:
        panel._emit_log_throttled(
            f"PICK_GATE:tf:{reason}",
            f"[PICK_GATE] tf blocked: {reason}",
        )
        return False, reason
    panel._emit_log_throttled("PICK_GATE:ok", "[PICK_GATE] All checks passed")
    return True, ""
