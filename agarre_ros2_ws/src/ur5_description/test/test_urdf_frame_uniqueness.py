"""Regresión arquitectónica: detectar pares de frames hermanos accidentales
con mismo `parent` + `origin`.

Audit 2026-05-10 (Action 8). Originalmente concebido para "cerrar" un
falso bug B2 (rg2_tcp + rg2_pinch_center coincidentes). El re-análisis
posterior verificó que ese par es **convención canónica vigilada por
los tests existentes**:

  - test_urdf_sdf_parity.py
  - test_gripper_geometry.py
  - test_gripper_geometry_contract.py

Esos tests garantizan offset 0.175 m exacto entre tool0 y ambos frames,
y paridad SDF↔URDF al sub-milímetro. La "duplicación aparente" es por
diseño: dos nombres semánticos (`rg2_tcp` planning/MoveIt, `rg2_pinch_center`
attach/contacto) para el mismo punto físico.

Este test conserva su valor para detectar **futuras** duplicaciones
no-declaradas: cualquier nuevo par con mismo parent+origin que NO esté
en `KNOWN_ALIAS_GROUPS` falla.
"""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import pytest

XACRO_PATH = (
    Path(__file__).resolve().parent.parent / "urdf" / "ur5.urdf.xacro"
)


# Pares conocidos de frames coincidentes por DISEÑO. Si añades aquí un par,
# DEBE estar también vigilado por un test de paridad URDF↔SDF dedicado.
# Cada entry es un frozenset de nombres de child-link que comparten el mismo
# parent + origin. El test los excluye de la detección de duplicados.
KNOWN_ALIAS_GROUPS: list[frozenset[str]] = [
    frozenset({"rg2_tcp", "rg2_pinch_center"}),  # vigilado por test_gripper_geometry_contract
]


def _resolve_xacro_property(text: str, key: str) -> str | None:
    """Best-effort lookup `<xacro:property name="key" value="..."/>`."""
    m = re.search(
        rf'<xacro:property\s+name="{re.escape(key)}"\s+value="([^"]*)"',
        text,
    )
    return m.group(1) if m else None


def _parse_origin(xyz: str | None) -> tuple[float, float, float]:
    if not xyz:
        return (0.0, 0.0, 0.0)
    parts = xyz.strip().split()
    if len(parts) != 3:
        return (0.0, 0.0, 0.0)
    try:
        return tuple(float(p) for p in parts)  # type: ignore[return-value]
    except ValueError:
        return (0.0, 0.0, 0.0)


def _xacro_substitute(value: str, properties: dict[str, str]) -> str:
    """Sustituye ${name} por su valor si está en properties."""
    def _repl(match: re.Match[str]) -> str:
        return properties.get(match.group(1), match.group(0))

    return re.sub(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}", _repl, value)


def _is_known_alias_group(children: list[str]) -> bool:
    """True si `children` (todos compartiendo parent+origin) coincide con
    algún grupo declarativo conocido."""
    children_set = frozenset(children)
    return any(
        children_set == group or children_set.issubset(group)
        for group in KNOWN_ALIAS_GROUPS
    )


def test_no_undeclared_duplicate_parent_origin_pairs() -> None:
    text = XACRO_PATH.read_text(encoding="utf-8")
    props = {
        m.group(1): m.group(2)
        for m in re.finditer(
            r'<xacro:property\s+name="([^"]+)"\s+value="([^"]*)"', text
        )
    }

    pattern = re.compile(
        r'<joint\s+name="([^"]+)"\s+type="fixed"[^>]*>\s*'
        r'<parent\s+link="([^"]+)"\s*/>\s*'
        r'<child\s+link="([^"]+)"\s*/>\s*'
        r'<origin\s+xyz="([^"]*)"',
        re.DOTALL,
    )

    pairs: dict[tuple[str, tuple[float, float, float]], list[str]] = defaultdict(list)
    for joint_name, parent, child, xyz_raw in pattern.findall(text):
        xyz_resolved = _xacro_substitute(xyz_raw, props)
        origin = _parse_origin(xyz_resolved)
        pairs[(parent, origin)].append(child)

    undeclared_duplicates = {
        key: children
        for key, children in pairs.items()
        if len(children) > 1 and not _is_known_alias_group(children)
    }
    assert not undeclared_duplicates, (
        "Frames hermanos con parent+origin idénticos NO declarados como "
        "alias en KNOWN_ALIAS_GROUPS. Si son alias intencionales, "
        "añádelos a la lista (con test de paridad dedicado). "
        f"Detalle: {undeclared_duplicates}"
    )


def test_critical_frames_present() -> None:
    """Mantiene la garantía de que los frames canónicos siguen existiendo."""
    text = XACRO_PATH.read_text(encoding="utf-8")
    for link_name in ("rg2_tcp", "rg2_pinch_center", "rg2_base_link"):
        assert re.search(rf'<link\s+name="{re.escape(link_name)}"', text), (
            f"link {link_name!r} no declarado en URDF"
        )


def test_known_alias_group_actually_coincident() -> None:
    """Los miembros de cada KNOWN_ALIAS_GROUPS DEBEN coincidir físicamente.

    Si alguien rompe esa garantía (modifica origin de uno solo del grupo),
    este test falla aunque el `test_no_undeclared_duplicate_parent_origin_pairs`
    no detecte el cambio.
    """
    text = XACRO_PATH.read_text(encoding="utf-8")
    props = {
        m.group(1): m.group(2)
        for m in re.finditer(
            r'<xacro:property\s+name="([^"]+)"\s+value="([^"]*)"', text
        )
    }
    pattern = re.compile(
        r'<joint\s+name="([^"]+)"\s+type="fixed"[^>]*>\s*'
        r'<parent\s+link="([^"]+)"\s*/>\s*'
        r'<child\s+link="([^"]+)"\s*/>\s*'
        r'<origin\s+xyz="([^"]*)"',
        re.DOTALL,
    )
    by_child: dict[str, tuple[str, tuple[float, float, float]]] = {}
    for _joint_name, parent, child, xyz_raw in pattern.findall(text):
        xyz_resolved = _xacro_substitute(xyz_raw, props)
        by_child[child] = (parent, _parse_origin(xyz_resolved))

    for group in KNOWN_ALIAS_GROUPS:
        ref: tuple[str, tuple[float, float, float]] | None = None
        for member in group:
            if member not in by_child:
                continue  # link sin joint fixed propio (no aplicable)
            if ref is None:
                ref = by_child[member]
            else:
                assert by_child[member] == ref, (
                    f"Alias group {set(group)} ya no coincide: "
                    f"{member} parent+origin = {by_child[member]} "
                    f"!= referencia {ref}. Si la divergencia es "
                    f"intencional, separa el par del KNOWN_ALIAS_GROUPS."
                )
