#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_helpers.py
# Contenido: F14 (2026-05-01) — helpers puros extraídos de panel_v2.py.
"""Helpers puros de ``panel_v2`` (sin Qt, sin ROS).

F14 (2026-05-01) extrae las funciones top-level utilitarias de
``panel_v2.py`` (2 795 LOC) a este módulo, con dos objetivos:

1. Reducir el tamaño del archivo principal.
2. Hacer los helpers testeables offline sin importar ``panel_v2``
   (que requiere Qt + 100+ módulos del paquete).

Funciones extraídas:

* ``runtime_time`` — wallclock monotónico para watchdog/freshness.
* ``camera_required_label`` — label canónico Bool/None → str.
* ``env_float`` / ``env_flag`` — lectura de env vars con default.
* ``proto_time_to_seconds`` — convierte gz proto Time {sec, nsec} → float.
* ``normalize_joint_name`` — strip namespaces "::" y "/" del nombre joint.
* ``rot_to_rpy`` — matriz rotación 3x3 → roll/pitch/yaw (XYZ).

``log_exception`` permanece en panel_v2 porque depende de un flag
runtime y de ``timestamped_line`` ya importado allí.

Las funciones FK (``fk_model_to_base_link``, ``fk_tool0_to_ee_base_link``)
quedan en panel_v2 porque dependen de TF runtime + canonical
gripper geometry. Su extracción se evaluará en F14-step2.
"""

from __future__ import annotations

import math
import os
from typing import Dict, Optional


def runtime_time() -> float:
    """Devuelve un timestamp monotónico local para freshness/watchdog.

    F15 (2026-05-01): delega al helper canónico
    ``panel_clock_helpers.steady_time`` para evitar duplicación con
    ``panel_ros._steady_time``. Se mantiene por compat con el resto de
    panel_v2 (que importa con el alias ``_runtime_time``).
    """
    from .panel_clock_helpers import steady_time
    return steady_time()


def camera_required_label(value: Optional[bool]) -> str:
    """Etiqueta canónica del flag camera_required: ``unset/true/false``."""
    if value is None:
        return "unset"
    return "true" if value else "false"


def env_float(name: str, default: float) -> float:
    """Lee una variable de entorno como float; default si ausente o inválida."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except Exception:
        return default


def env_flag(name: str, default: bool) -> bool:
    """Lee una variable de entorno como flag bool truthy/falsy."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def proto_time_to_seconds(value: Dict[str, object]) -> float:
    """Convierte un proto Time {sec, nsec} a segundos float.

    Tolerante: si el dict no tiene una de las claves o no es dict
    devuelve 0.0.
    """
    if not isinstance(value, dict):
        return 0.0
    sec = float(value.get("sec", 0.0)) if value.get("sec") is not None else 0.0
    nsec = (
        float(value.get("nsec", 0.0)) if value.get("nsec") is not None else 0.0
    )
    return sec + nsec * 1e-9


def normalize_joint_name(name) -> str:
    """Devuelve el último segmento del nombre tras strip de ``::`` y ``/``.

    Ejemplos::

        "ur5_rg2::shoulder_pan_joint" → "shoulder_pan_joint"
        "/joint/elbow"                → "elbow"
        "wrist_3_joint"               → "wrist_3_joint"
    """
    text = str(name).strip()
    if "::" in text:
        text = text.split("::")[-1]
    if "/" in text:
        text = text.split("/")[-1]
    return text.strip()


def rot_to_rpy(rot):
    """Convierte una matriz de rotación 3x3 a ángulos roll/pitch/yaw (rad).

    Utiliza la convención XYZ extrínseca usada por el panel para
    inspección visual de poses. Maneja el caso singular (gimbal lock)
    cuando ``cos(pitch)`` es cercano a cero.

    Acepta cualquier objeto con indexación ``rot[i, j]`` (numpy array,
    list de lists indexable, etc.).
    """
    sy = math.sqrt((rot[0, 0] * rot[0, 0]) + (rot[1, 0] * rot[1, 0]))
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw
