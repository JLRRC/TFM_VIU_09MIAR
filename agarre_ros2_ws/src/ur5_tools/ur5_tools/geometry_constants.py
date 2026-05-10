#!/usr/bin/env python3
"""Constantes geométricas únicas del stack UR5+RG2.

F2 (auditoría 2026-05-10): centraliza los valores numéricos de geometría
que estaban duplicados en múltiples archivos. Cualquier consumidor debe
importar desde aquí en lugar de hardcodear el valor.

Fuente autoritativa de cada constante: el comentario indica el archivo
de origen físico (URDF / SDF / config). Si la fuente cambia, esta
constante DEBE actualizarse y todos los consumidores recibirán el valor
nuevo automáticamente.
"""
from __future__ import annotations

from typing import Tuple

#: Origen del frame ``base_link`` del UR5 expresado en el frame ``world``.
#:
#: Fuente autoritativa: ``src/ur5_description/urdf/ur5.urdf.xacro``
#: línea 47 — ``<origin xyz="-0.85 0 0.850" rpy="0 0 0"/>`` del joint
#: ``world_to_base_link`` (sin rotación). Cualquier conversión
#: world↔base_link en código Python debe usar esta constante.
#:
#: Fórmulas:
#:   pos_base = pos_world - BASE_LINK_IN_WORLD
#:   pos_world = pos_base + BASE_LINK_IN_WORLD
BASE_LINK_IN_WORLD: Tuple[float, float, float] = (-0.85, 0.0, 0.850)


def world_to_base(
    pos_world: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Convierte una posición ``(x, y, z)`` de ``world`` a ``base_link``.

    Asume que el robot está fijo en el mundo (sin pose dinámica del
    base_link). Equivalente a un ``tf2`` lookup estático pero sin
    necesidad de tener un buffer activo — útil en goal builders puros.
    """
    return (
        pos_world[0] - BASE_LINK_IN_WORLD[0],
        pos_world[1] - BASE_LINK_IN_WORLD[1],
        pos_world[2] - BASE_LINK_IN_WORLD[2],
    )


def base_to_world(
    pos_base: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Convierte una posición ``(x, y, z)`` de ``base_link`` a ``world``."""
    return (
        pos_base[0] + BASE_LINK_IN_WORLD[0],
        pos_base[1] + BASE_LINK_IN_WORLD[1],
        pos_base[2] + BASE_LINK_IN_WORLD[2],
    )


#: Pose default de la cesta de drop en frame ``world`` (m).
#:
#: Punto de soltado por defecto cuando un goal ``/pick_place`` no
#: especifica ``drop_xyz_world``. El valor histórico vivía en
#: ``ur5_qt_panel/panel_config.py`` como ``BASKET_DROP``; F2-followup
#: (auditoría 2026-05-10) lo mueve a este módulo neutral para que el
#: orchestrator pueda referenciarlo sin depender del paquete del panel.
#: ``panel_config.BASKET_DROP`` se mantiene como re-export.
BASKET_DROP_WORLD: Tuple[float, float, float] = (-1.30, 0.00, 0.82)


__all__ = [
    "BASE_LINK_IN_WORLD",
    "BASKET_DROP_WORLD",
    "world_to_base",
    "base_to_world",
]
