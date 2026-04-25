"""Helpers puros para el pipeline visual STEP by STEP — visual_autopick_ros_ws.

Flujos: DIRECTO | MOVEIT
Fases:  HOME | APPROACH | DOWN | CLOSE | LIFT
"""
from __future__ import annotations

import math
from typing import Dict, List

# ── Poses articulares (rad) — idénticas al workspace agarre_ros2_ws ──────────
ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

JOINT_HOME_RAD = [
    math.radians(0.0),
    math.radians(-90.0),
    math.radians(0.0),
    math.radians(-90.0),
    math.radians(0.0),
    math.radians(0.0),
]

JOINT_MESA_RAD = [
    math.radians(-2.7),
    math.radians(-0.1),
    math.radians(90.0),
    math.radians(-5.5),
    math.radians(-90.7),
    math.radians(0.9),
]

JOINT_CESTA_RAD = [
    math.radians(167.6),
    math.radians(-4.8),
    math.radians(110.9),
    math.radians(-21.1),
    math.radians(-92.0),
    math.radians(0.9),
]


_DIRECTO_SEQ: List[str] = ["HOME", "APPROACH", "DOWN", "CLOSE", "LIFT"]
_MOVEIT_SEQ:  List[str] = ["HOME", "APPROACH", "DOWN", "CLOSE", "LIFT"]

_PHASE_INTENT: Dict[str, Dict[str, str]] = {
    "DIRECTO": {
        "HOME":     "Mover el robot a posición segura inicial (joints en home).",
        "APPROACH": "Acercarse al objeto desde arriba (~15 cm sobre pick_demo).",
        "DOWN":     "Bajar la pinza hasta la superficie del objeto (pick_demo).",
        "CLOSE":    "Cerrar la pinza sobre el objeto.",
        "LIFT":     "Elevar el objeto tras el agarre.",
    },
    "MOVEIT": {
        "HOME":     "Mover el robot a posición home con MoveIt.",
        "APPROACH": "Acercarse al objeto con MoveIt (pre-grasp ~15 cm).",
        "DOWN":     "Descender hasta la altura de agarre con MoveIt.",
        "CLOSE":    "Cerrar la pinza sobre el objeto.",
        "LIFT":     "Elevar el objeto con MoveIt.",
    },
}

_PHASE_GRIPPER: Dict[str, Dict[str, str]] = {
    "DIRECTO": {
        "HOME":     "Abierta",
        "APPROACH": "Abierta",
        "DOWN":     "Abierta",
        "CLOSE":    "Cerrada",
        "LIFT":     "Cerrada",
    },
    "MOVEIT": {
        "HOME":     "Abierta",
        "APPROACH": "Abierta",
        "DOWN":     "Abierta",
        "CLOSE":    "Cerrada",
        "LIFT":     "Cerrada",
    },
}

_PHASE_ACTION_TEXT: Dict[str, Dict[str, str]] = {
    "DIRECTO": {
        "HOME":     "Publicará JointTrajectory hacia posición home [0,−π/2,0,−π/2,0,0].",
        "APPROACH": "Publicará JointTrajectory hacia joints de aproximación sobre pick_demo.",
        "DOWN":     "Publicará JointTrajectory para descender la pinza hasta el objeto.",
        "CLOSE":    "Enviará comando Float64MultiArray al gripper_controller (cerrar).",
        "LIFT":     "Publicará JointTrajectory para elevar la pinza con el objeto.",
    },
    "MOVEIT": {
        "HOME":     "Ejecutará move_group.go('home') con MoveIt.",
        "APPROACH": "Planificará y ejecutará trayectoria cartesiana hacia pre-grasp.",
        "DOWN":     "Ejecutará trayectoria cartesiana descendente hasta el objeto.",
        "CLOSE":    "Enviará comando Float64MultiArray al gripper_controller (cerrar).",
        "LIFT":     "Ejecutará trayectoria cartesiana ascendente con el objeto.",
    },
}

# Approximate TCP world positions per phase
# Base_link at (-0.85, 0, 0.850) in world. Object at (-0.42, 0, 0.876) in world.
_KNOWN_TCP_WORLD: Dict[str, tuple] = {
    "HOME":     (-0.617, 0.0, 1.738),
    "APPROACH": (-0.420, 0.0, 1.026),
    "DOWN":     (-0.420, 0.0, 0.880),
    "CLOSE":    (-0.420, 0.0, 0.880),
    "LIFT":     (-0.420, 0.0, 1.026),
}

_PHASE_DELAYS: Dict[str, float] = {
    "HOME":     5.0,
    "APPROACH": 4.5,
    "DOWN":     3.0,
    "CLOSE":    2.0,
    "LIFT":     3.5,
}


def step_phase_sequence(flow: str) -> List[str]:
    f = str(flow or "").strip().upper()
    return _DIRECTO_SEQ[:] if f == "DIRECTO" else _MOVEIT_SEQ[:]


def step_predict_next_phase(flow: str, phase: str) -> str:
    seq = step_phase_sequence(flow)
    p = str(phase or "").strip().upper()
    try:
        idx = seq.index(p)
        return seq[idx + 1] if idx + 1 < len(seq) else "--"
    except ValueError:
        return "--"


def step_phase_intent(flow: str, phase: str) -> str:
    f = str(flow or "").strip().upper()
    p = str(phase or "").strip().upper()
    return _PHASE_INTENT.get(f, {}).get(p, "Sin descripción.")


def step_phase_gripper_state(flow: str, phase: str) -> str:
    f = str(flow or "").strip().upper()
    p = str(phase or "").strip().upper()
    return _PHASE_GRIPPER.get(f, {}).get(p, "--")


def step_phase_action_text(flow: str, phase: str) -> str:
    f = str(flow or "").strip().upper()
    p = str(phase or "").strip().upper()
    return _PHASE_ACTION_TEXT.get(f, {}).get(p, step_phase_intent(flow, phase))


def step_present_flow_name(flow: str) -> str:
    f = str(flow or "").strip().upper()
    return {"DIRECTO": "DIRECTO", "MOVEIT": "MOVEIT"}.get(f, f or "--")


def step_known_tcp_world(phase: str) -> tuple:
    return _KNOWN_TCP_WORLD.get(str(phase or "").strip().upper(), (0.0, 0.0, 0.0))


def step_phase_delay(phase: str) -> float:
    return _PHASE_DELAYS.get(str(phase or "").strip().upper(), 4.0)
