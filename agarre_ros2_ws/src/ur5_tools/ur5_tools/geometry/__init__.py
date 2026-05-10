"""F11 (auditoría 2026-05-10): namespace de geometría del stack UR5.

Subpaquete que re-exporta los módulos del dominio geométrico:
  * Constantes únicas (``BASE_LINK_IN_WORLD``, ``BASKET_DROP_WORLD``,
    helpers ``world_to_base``/``base_to_world``).
  * Geometría del gripper RG2 (frames, offsets, contact center).
  * Lógica pura de TF (sin nodos).

Los archivos físicos viven todavía en ``ur5_tools/`` (top-level) por
compatibilidad con todos los imports existentes. F11 iter 2 (futuro)
moverá los archivos a este subpaquete y dejará shims top-level. Por
ahora, ``from ur5_tools.geometry import X`` es equivalente a
``from ur5_tools.X import X`` y NO debe romperse en la migración.
"""
from __future__ import annotations

from ..geometry_constants import (  # noqa: F401
    BASE_LINK_IN_WORLD,
    BASKET_DROP_WORLD,
    base_to_world,
    world_to_base,
)
from ..tf_geometry_logic import *  # noqa: F401,F403

__all__ = [
    "BASE_LINK_IN_WORLD",
    "BASKET_DROP_WORLD",
    "base_to_world",
    "world_to_base",
]
