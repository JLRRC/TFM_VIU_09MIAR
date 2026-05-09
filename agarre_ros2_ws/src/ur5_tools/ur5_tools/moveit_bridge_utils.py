#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/moveit_bridge_utils.py
"""Minimal env helpers — restos del path MoveIt-classic borrado el 2026-05-09.

Este módulo originalmente contenía utilidades del UR5MoveItBridge. Tras
borrar el path MoveIt-classic, se conservan SOLO las helpers ``bridge_env_*``
que siguen siendo consumidas por código backend (cycle_logger,
gripper_geometry, attach_set_pose, planning_scene_sync, evidence_logger).

Para una migración futura: estos helpers son espejo de los de
``ur5_qt_panel.panel_env`` y podrían unificarse en un paquete común
``ur5_env`` o similar (deuda audit-v4.2).
"""
from __future__ import annotations

import os


def bridge_env_float(name: str, default: float) -> float:
    """Read a float from an environment variable, returning *default* on failure."""
    raw = os.environ.get(name, "")
    if not raw:
        return float(default)
    try:
        return float(raw)
    except Exception:
        return float(default)


def bridge_env_str(name: str, default: str) -> str:
    """Read a string from an env var, returning *default* if unset/empty/whitespace."""
    raw = os.environ.get(name)
    if raw is None:
        return str(default)
    s = str(raw).strip()
    if not s:
        return str(default)
    return s


def bridge_env_int(name: str, default: int) -> int:
    """Read an int from an env var, returning *default* on failure."""
    raw = os.environ.get(name, "")
    if not raw:
        return int(default)
    try:
        return int(raw)
    except Exception:
        return int(default)


def bridge_env_flag(name: str, default: bool = False) -> bool:
    """Read a boolean flag (1/true/yes/on)."""
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}
