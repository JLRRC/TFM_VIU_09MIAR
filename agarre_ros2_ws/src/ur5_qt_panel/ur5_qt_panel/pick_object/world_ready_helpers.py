#!/usr/bin/env python3
"""V1.1 audit-v4 (2026-05-08): puros helpers de world_ready para pick_object.

Primer paso del split iter2 de panel_pick_object.py (3.814 LOC).
Extrae lógica que NO depende del objeto ``panel``.

Helpers:
- ``normalize_world_ready_scope(raw)`` — normaliza el setting raw del
  YAML a uno de {"all", "target"}.
- ``compute_default_tracked_names(all_drop_names, all_pick_demo_names)``
  — devuelve el set canónico de nombres trackeados en scope='all'.
- ``derive_tracked_names_target_scope(selected_name, fallback_name)``
  — lógica de selección para scope='target' (con fallbacks).
- ``check_object_on_table(xyz, table_z_min, table_z_max)`` — predicate
  puro.
"""
from __future__ import annotations

from typing import Iterable, List, Optional, Tuple


def normalize_world_ready_scope(raw: object) -> str:
    """Normaliza ``world_ready_scope`` (param raw) a {"all", "target"}.

    - "all" o "strict" → "all" (track todos los objetos en world).
    - cualquier otro → "target" (sólo el objeto seleccionado).

    Match histórico de _world_ready_scope() en panel_pick_object.
    """
    if raw in ("all", "strict"):
        return "all"
    return "target"


def compute_default_tracked_names(
    drop_names: Iterable[str],
    pick_demo_names: Iterable[str],
) -> List[str]:
    """En scope='all': sorted(set(DROP) | set(PICK_DEMO)).

    Args:
        drop_names: nombres de DROP_NAME_SET.
        pick_demo_names: nombres de PICK_DEMO_NAME_SET.

    Returns:
        Lista ordenada con duplicados eliminados.
    """
    return sorted(set(drop_names) | set(pick_demo_names))


def derive_tracked_names_target_scope(
    candidates: Iterable[Optional[str]],
    fallback_name: str = "pick_demo",
) -> List[str]:
    """En scope='target': itera candidatos, devuelve los no-vacíos en
    orden de aparición sin duplicados; si la lista resultante está
    vacía, devuelve ``[fallback_name]``.

    Args:
        candidates: iterable de strings (None/empty se ignoran).
        fallback_name: nombre canónico cuando no hay candidatos válidos.
    """
    tracked: List[str] = []
    for cand in candidates:
        if cand is None:
            continue
        s = str(cand).strip()
        if not s:
            continue
        if s in tracked:
            continue
        tracked.append(s)
    if tracked:
        return tracked
    return [fallback_name]


def check_object_on_table(
    xyz: Tuple[float, float, float],
    table_z_min: float,
    table_z_max: float,
) -> bool:
    """Predicate: ``table_z_min <= z <= table_z_max``."""
    z = float(xyz[2])
    return float(table_z_min) <= z <= float(table_z_max)
