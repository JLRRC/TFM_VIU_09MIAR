#!/usr/bin/env python3
"""F12 (auditoría 2026-05-10): helpers FK puros extraídos de panel_v2.

Funciones de forward kinematics y conversión de frames extraídas del
top-level de ``panel_v2.py``. Cero Qt, cero ROS — solo numpy. La
versión que requiere TF lookup (``fk_tool0_to_ee_base_link``) acepta
el lookup como callable inyectable para mantenerse pura.

panel_v2 mantiene los wrappers top-level por compat (varios consumers
internos importan las funciones directamente con el prefix `_`).
"""
from __future__ import annotations

from typing import Callable, Optional, Tuple

import numpy as np


def canonical_tool0_to_semantic_frame(
    frame_name: str,
    tool0_offset_for_frame: Callable[[str], Tuple[float, float, float]],
) -> Optional[Tuple[float, float, float]]:
    """Devuelve el offset canónico tool0→frame para frames RG2 conocidos.

    Args:
        frame_name: nombre del frame destino. Solo retorna offset para
            ``rg2_pinch_center`` y ``rg2_tcp``; cualquier otro → None.
        tool0_offset_for_frame: función que devuelve la tupla offset
            para un frame. Inyectable para test (en producción se
            pasa ``ur5_tools.gripper_geometry.tool0_offset_for_frame``).

    Returns:
        ``(x, y, z)`` o ``None``.
    """
    frame = str(frame_name or "").strip()
    if frame not in {"rg2_pinch_center", "rg2_tcp"}:
        return None
    return tool0_offset_for_frame(frame)


def fk_model_to_base_link(
    pos_model,
    rot_model: np.ndarray,
) -> Tuple[Tuple[float, float, float], np.ndarray]:
    """Convierte FK output del UR5 modelo a frame ``base_link`` de runtime.

    El URDF del UR5 usa ``base_link_inertia`` como base FK, mientras
    que ``base_link`` runtime difiere en una rotación de π alrededor
    de Z (la convención URDF estándar de UR). Esta función aplica
    esa rotación y la inversión de signos en X/Y.

    Args:
        pos_model: tupla/list/array (x, y, z) del modelo.
        rot_model: matriz 3x3 de rotación del modelo.

    Returns:
        ``((base_x, base_y, base_z), base_rot 3x3)``.
    """
    rz_pi = np.array(
        [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    base_pos = (
        -float(pos_model[0]),
        -float(pos_model[1]),
        float(pos_model[2]),
    )
    base_rot = rz_pi @ np.asarray(rot_model, dtype=float)
    return base_pos, base_rot


def fk_tool0_to_ee_base_link(
    pos_model,
    rot_model: np.ndarray,
    ee_frame: str,
    *,
    tf_lookup: Optional[Callable[[str, str], Optional[Tuple[float, float, float]]]] = None,
    canonical_offset: Optional[Callable[[str], Optional[Tuple[float, float, float]]]] = None,
) -> Tuple[Tuple[float, float, float], np.ndarray]:
    """Aplica FK de tool0 a un EE frame (rg2_*) en frame base_link.

    Args:
        pos_model, rot_model: idem ``fk_model_to_base_link``.
        ee_frame: frame destino. Si está vacío o "tool0", devuelve
            FK directo de tool0 sin offset.
        tf_lookup: callable ``(parent, child)`` → ``(x,y,z)`` o None.
            Si TF está disponible se prefiere sobre canonical_offset.
        canonical_offset: callable ``(frame)`` → ``(x,y,z)`` o None.
            Se usa como fallback si TF lookup falla. En producción
            ``canonical_tool0_to_semantic_frame``.

    Returns:
        ``((ee_x, ee_y, ee_z), base_rot 3x3)``. Si no se encuentra
        offset, devuelve la pose tool0 sin modificar.
    """
    base_pos, base_rot = fk_model_to_base_link(pos_model, rot_model)
    frame_name = str(ee_frame or "").strip()
    if not frame_name or frame_name == "tool0":
        return base_pos, base_rot

    local_offset: Optional[Tuple[float, float, float]] = None
    if tf_lookup is not None:
        try:
            local_offset = tf_lookup("tool0", frame_name)
        except Exception:
            local_offset = None
    if local_offset is None and canonical_offset is not None:
        local_offset = canonical_offset(frame_name)

    if local_offset is None:
        return base_pos, base_rot

    offset_base = tuple(
        (np.asarray(base_rot, dtype=float) @ np.asarray(local_offset, dtype=float)).tolist()
    )
    return (
        (
            float(base_pos[0]) + float(offset_base[0]),
            float(base_pos[1]) + float(offset_base[1]),
            float(base_pos[2]) + float(offset_base[2]),
        ),
        base_rot,
    )


__all__ = [
    "canonical_tool0_to_semantic_frame",
    "fk_model_to_base_link",
    "fk_tool0_to_ee_base_link",
]
