#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/step_pose_format.py
# Contenido: F3 — formatters de pose / fetch de TCP live para el step pipeline.
"""Helpers de formato y fetch de pose (live) para el step pipeline.

Extraídas de ``panel_step_callbacks.py`` (god-file 2.0 kLOC tras
extraer step_cart_debug). 7 funciones cohesivas que:

* Formatean tuplas (x, y, z) y ángulos RPY.
* Convierten posiciones de base_link a world.
* Consultan TF en runtime para obtener TCP live.
* Resuelven el nombre canónico del frame operacional.

Las funciones siguen tomando ``panel`` como primer parámetro para
acceder a sus atributos (``_business_base_frame``, ``_required_ee_frame``,
etc.). Se re-exportan desde ``panel_step_callbacks`` para preservar
la API pública.
"""

from __future__ import annotations

from typing import Optional, Tuple

from .panel_utils import base_to_world
from .tf_pose_utils import get_tcp_in_base as tf_get_tcp_in_base


def _step_format_xyz(
    panel, position: Optional[Tuple[float, float, float]]
) -> Tuple[str, str, str]:
    if position is None:
        return ("--", "--", "--")
    return (
        f"{float(position[0]):.3f}",
        f"{float(position[1]):.3f}",
        f"{float(position[2]):.3f}",
    )


def _step_display_position(
    panel, position: Optional[Tuple[float, float, float]]
) -> Optional[Tuple[float, float, float]]:
    if position is None:
        return None
    try:
        pos3 = (float(position[0]), float(position[1]), float(position[2]))
    except Exception:
        return None
    try:
        wx, wy, wz = base_to_world(pos3[0], pos3[1], pos3[2])
        return (float(wx), float(wy), float(wz))
    except Exception:
        return pos3


def _step_format_inline_xyz(
    panel, position: Optional[Tuple[float, float, float]]
) -> str:
    x, y, z = panel._step_format_xyz(position)
    if x == "--":
        return "--"
    return f"({x}, {y}, {z})"


def _step_format_inline_rpy(
    panel, rpy_deg: Optional[Tuple[float, float, float]]
) -> str:
    if rpy_deg is None:
        return "--"
    try:
        return (
            f"({float(rpy_deg[0]):.1f}, {float(rpy_deg[1]):.1f}, "
            f"{float(rpy_deg[2]):.1f})"
        )
    except Exception:
        return "--"


def _step_fetch_live_pose(
    panel, ee_frame: str
) -> Optional[Tuple[float, float, float]]:
    """Consulta TF en tiempo real. Devuelve None si TF no disponible; nunca usa caché stale."""
    frame_name = str(ee_frame or "").strip()
    if not frame_name:
        return None
    base_frame = panel._business_base_frame()
    tcp_pose_base, _tcp_rpy_deg, _tcp_reason = tf_get_tcp_in_base(
        base_frame=base_frame,
        ee_frame=frame_name,
        timeout=0.20,
        logger=None,
    )
    if tcp_pose_base is None:
        return None
    return (
        float(tcp_pose_base.pose.position.x),
        float(tcp_pose_base.pose.position.y),
        float(tcp_pose_base.pose.position.z),
    )


def _step_operational_frame_name(panel) -> str:
    return str(
        getattr(panel, "_step_target_frame", "")
        or getattr(panel, "_ee_frame_effective", "")
        or panel._required_ee_frame
        or "rg2_pinch_center"
    ).strip() or "rg2_pinch_center"


def _step_live_pose_text(
    panel,
    label: str,
    ee_frame: str,
    position: Optional[Tuple[float, float, float]],
) -> str:
    frame_name = str(ee_frame or "").strip() or "--"
    xyz_txt = panel._step_format_inline_xyz(position)
    return f"{label}: {xyz_txt} | frame: {frame_name}"
