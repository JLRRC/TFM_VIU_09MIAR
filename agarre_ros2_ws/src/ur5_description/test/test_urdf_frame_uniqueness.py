"""Regresión arquitectónica: ningún par de frames del URDF/Xacro tiene
el mismo `parent` + `origin` (≈ frame redundante).

Audit 2026-05-10 (Action 8) — protege contra el bug B2 donde
`rg2_tcp` y `rg2_pinch_center` colgaban ambos de `tool0` con
origin idéntico (xyz=0 0 0.175), provocando ambigüedad semántica
en el código (algunos nodos usaban uno, otros el otro).

Cierre del bug B2: rg2_pinch_center pasa a alias explícito (parent
rg2_tcp, origin identidad) en commit 26b019e (2026-05-10).

Permite alias DECLARADOS explícitamente: si dos joints fixed apuntan
a frames con parent en cadena (A→B y A→C tal que B y C resuelven al
mismo punto físico), eso es DECLARATIVO. Lo que el test detecta es
el caso accidental donde dos hermanos del mismo padre comparten origin
sin documentación.
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


def test_no_duplicate_parent_origin_pairs() -> None:
    text = XACRO_PATH.read_text(encoding="utf-8")
    # Recolectar todas las xacro:property name=... value=... para resolver
    # plantillas en origin/xyz.
    props = {
        m.group(1): m.group(2)
        for m in re.finditer(
            r'<xacro:property\s+name="([^"]+)"\s+value="([^"]*)"', text
        )
    }

    # Las regex sobre xacro funcionan razonablemente bien para joint+origin
    # locales (no UR macro). Buscamos joints type="fixed" del paquete.
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

    duplicates = {
        key: children for key, children in pairs.items() if len(children) > 1
    }
    assert not duplicates, (
        "Frames hermanos con parent+origin idénticos detectados (B2 regresión). "
        "Si son alias intencionales, hazlos cuelgar uno del otro con "
        "transform identidad. Detalle: " + str(duplicates)
    )


def test_critical_frames_present() -> None:
    """Mantiene la garantía de que los frames canónicos siguen existiendo."""
    text = XACRO_PATH.read_text(encoding="utf-8")
    for link_name in ("rg2_tcp", "rg2_pinch_center", "rg2_base_link"):
        assert re.search(rf'<link\s+name="{re.escape(link_name)}"', text), (
            f"link {link_name!r} no declarado en URDF"
        )
