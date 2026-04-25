#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_state.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""State primitives and evaluation for the UR5 panel."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class SystemState(Enum):
    BOOT = "BOOT"
    WAITING_GAZEBO = "WAITING_GAZEBO"
    WAITING_CONTROLLERS = "WAITING_CONTROLLERS"
    WAITING_TF = "WAITING_TF"
    WAITING_CAMERA = "WAITING_CAMERA"
    READY_BASIC = "READY_BASIC"
    READY_VISION = "READY_VISION"
    READY_MOVEIT = "READY_MOVEIT"
    ERROR = "ERROR"
    ERROR_FATAL = "ERROR_FATAL"


class MoveItState(Enum):
    OFF = "OFF"
    STARTING = "STARTING"
    WAITING_MOVEIT_READY = "WAITING_MOVEIT_READY"
    READY = "READY"
    ERROR = "ERROR"


EXTERNAL_STATE_MAP = {
    "BOOT": SystemState.BOOT,
    "BOOTING": SystemState.BOOT,
    "WAITING_GAZEBO": SystemState.WAITING_GAZEBO,
    "WAITING_CONTROLLERS": SystemState.WAITING_CONTROLLERS,
    "WAITING_TF": SystemState.WAITING_TF,
    "WAITING_BRIDGE": SystemState.WAITING_GAZEBO,
    "WAITING_CAMERA": SystemState.WAITING_CAMERA,
    "WAITING_SETTLE": SystemState.READY_BASIC,
    "WAITING_MOVEIT": SystemState.READY_VISION,
    "READY_BASIC": SystemState.READY_BASIC,
    "READY_VISION": SystemState.READY_VISION,
    "READY_MOVEIT": SystemState.READY_MOVEIT,
    "READY": SystemState.READY_VISION,
    "ERROR": SystemState.ERROR,
    "ERROR_FATAL": SystemState.ERROR_FATAL,
}


@dataclass(frozen=True)
class PanelStateSnapshot:
    gazebo_state: str
    controllers_ok: bool
    controllers_reason: str
    tf_ready: bool
    bridge_running: bool
    camera_required: bool
    camera_stream_ok: bool
    pose_info_ok: bool
    calibration_ready: bool
    objects_settled: bool
    moveit_required: bool
    moveit_state: MoveItState
    moveit_state_reason: str


class PanelStateEvaluator:
    @staticmethod
    def is_ready_level(state: SystemState, level: str) -> bool:
        if level == "basic":
            return state in (SystemState.READY_BASIC, SystemState.READY_VISION, SystemState.READY_MOVEIT)
        if level == "vision":
            return state in (SystemState.READY_VISION, SystemState.READY_MOVEIT)
        return state == SystemState.READY_MOVEIT

    @staticmethod
    def resolve(snapshot: PanelStateSnapshot, tf_reason: Optional[str] = None) -> Tuple[SystemState, str]:
        if snapshot.gazebo_state != "GAZEBO_READY":
            return SystemState.WAITING_GAZEBO, f"Gazebo {snapshot.gazebo_state}"
        if not snapshot.controllers_ok:
            reason = snapshot.controllers_reason or "controladores no listos"
            return SystemState.WAITING_CONTROLLERS, reason
        if not snapshot.tf_ready:
            return SystemState.READY_BASIC, tf_reason or "TF world->base no disponible"
        if (
            not snapshot.bridge_running
            or (snapshot.camera_required and not snapshot.camera_stream_ok)
            or not snapshot.pose_info_ok
        ):
            if not snapshot.bridge_running:
                return SystemState.READY_BASIC, "Bridge no listo"
            if snapshot.camera_required and not snapshot.camera_stream_ok:
                return SystemState.READY_BASIC, "Cámara no lista"
            return SystemState.READY_BASIC, "pose/info no disponible"
        if not snapshot.calibration_ready or not snapshot.objects_settled:
            if not snapshot.objects_settled:
                return SystemState.READY_BASIC, "Objetos no estabilizados"
            return SystemState.READY_BASIC, "Calibración pendiente"
        if snapshot.moveit_required and snapshot.moveit_state != MoveItState.READY:
            reason = snapshot.moveit_state_reason or "MoveIt no listo"
            return SystemState.READY_VISION, reason
        if snapshot.moveit_required:
            return SystemState.READY_MOVEIT, "Sistema listo (MoveIt)"
        return SystemState.READY_VISION, "Sistema listo (visión)"
