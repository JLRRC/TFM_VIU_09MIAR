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
from typing import Iterable, List, Optional, Sequence, Tuple


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


def drop_anchor_cleanup_targets(
    anchor_name: str,
    *,
    include_primary: bool,
) -> List[Tuple[str, str]]:
    """Genera la lista de targets a limpiar para el drop_anchor.

    F8 (auditoría 2026-05-10): extraído de ``ReleaseObjectsService.
    _drop_anchor_cleanup_targets``. Para cada model_name considerado
    (primary opcional + ``anchor_name(1)`` y ``anchor_name(2)``) se
    devuelven 3 entries: el modelo en sí (``MODEL``) y dos formas de
    su link (``LINK``, separador ``::`` y ``/``).

    Returns:
        Lista de tuplas ``(target_name, target_type)`` donde
        ``target_type`` es ``"MODEL"`` o ``"LINK"``.
    """
    model_names: List[str] = []
    if include_primary:
        model_names.append(anchor_name)
    for idx in (1, 2):
        model_names.append(f"{anchor_name}({idx})")
    targets: List[Tuple[str, str]] = []
    for model_name in model_names:
        targets.append((model_name, "MODEL"))
        targets.append((f"{model_name}::link", "LINK"))
        targets.append((f"{model_name}/link", "LINK"))
    return targets


def pick_gz_service(
    services: Sequence[str],
    world_name: str,
    suffixes: Tuple[str, ...],
) -> Optional[str]:
    """Selecciona un nombre de servicio Gazebo de la lista por sufijo.

    F8 (auditoría 2026-05-10): extraído de ``ReleaseObjectsService.
    _pick_gz_service``. Reglas:

      1. Filtra ``services`` por los que contienen ``/world/{world_name}/``.
      2. Para cada ``suffix`` en orden, devuelve el primer scoped que
         termina exactamente con ``/{suffix}``.
      3. Si ninguno cumple (2), devuelve el primer scoped que termina
         con cualquiera de los suffixes (sin barra).
      4. Si no hay scoped, devuelve None.
    """
    scoped = [s for s in services if f"/world/{world_name}/" in s]
    for suffix in suffixes:
        for name in scoped:
            if name.endswith(f"/{suffix}"):
                return name
    if scoped:
        for name in scoped:
            for suffix in suffixes:
                if name.endswith(suffix):
                    return name
    return None


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
