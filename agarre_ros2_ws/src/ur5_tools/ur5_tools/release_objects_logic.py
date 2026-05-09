#!/usr/bin/env python3
"""F5 audit-v4 (2026-05-08): puro release_objects_service helpers.

Lógica testeable extraída del LifecycleNode ``ReleaseObjectsService``:
- ``compute_missing_required(names, expected, anchor)`` — qué entidades
  faltan en pose/info (sin lifecycle, sin ROS).
- ``find_drop_anchor_duplicates(names, anchor)`` — detecta nombres
  ``anchor(N)`` que indican duplicación involuntaria.
- ``parse_world_name_from_sdf(text)`` — extrae el nombre del world de
  un SDF como string XML (sin filesystem).

Sin dependencias ROS. Tests offline puros.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Iterable, List, Sequence


def compute_missing_required(
    pose_names: Sequence[str],
    expected_names: Sequence[str],
    anchor_name: str,
) -> List[str]:
    """Devuelve lista de nombres requeridos no presentes en pose/info.

    Required = [anchor_name] + expected_names. Mantiene orden de aparición
    en required (no se reordena alfabéticamente).
    """
    pose_set = set(pose_names)
    required: List[str] = [anchor_name] + list(expected_names)
    return [name for name in required if name not in pose_set]


def find_drop_anchor_duplicates(
    pose_names: Iterable[str],
    anchor_name: str,
) -> List[str]:
    """Devuelve duplicados ``anchor_name(N)`` (Gazebo añade índice).

    Si pose_info reporta varios objetos llamados ``drop_anchor(2)``,
    significa que Gazebo no los limpió. La función NO incluye el
    anchor base (sin paréntesis).
    """
    prefix = f"{anchor_name}("
    return [n for n in pose_names if n.startswith(prefix)]


def parse_world_name_from_sdf(sdf_text: str) -> str:
    """Extrae ``<world name="...">`` del primer elemento ``<world>``.

    Devuelve cadena vacía si el SDF no tiene world o el atributo name.
    """
    if not sdf_text or not sdf_text.strip():
        return ""
    try:
        root = ET.fromstring(sdf_text)
    except ET.ParseError:
        return ""
    if root.tag == "world":
        return str(root.get("name") or "")
    world_elem = root.find("world")
    if world_elem is None:
        return ""
    return str(world_elem.get("name") or "")
