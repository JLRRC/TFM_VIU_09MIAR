#!/usr/bin/env python3
"""F2 (auditoría 2026-05-10): regresión sobre las constantes geométricas.

Garantiza que:
  1. ``BASE_LINK_IN_WORLD`` coincide con el valor del URDF (autoridad).
  2. Nadie reintroduce el literal ``(-0.85, 0.0, 0.850)`` (o variantes
     numéricas) hardcoded en código fuente productivo. Tests sí pueden
     replicarlo (independencia del módulo bajo test).
  3. Los helpers ``world_to_base`` y ``base_to_world`` son inversos.
"""
from __future__ import annotations

import pathlib
import re

import pytest

from ur5_tools.geometry_constants import (
    BASE_LINK_IN_WORLD,
    base_to_world,
    world_to_base,
)


WS_ROOT = pathlib.Path(__file__).resolve().parents[3]


def test_base_link_in_world_matches_urdf() -> None:
    """La constante debe coincidir con el origen del joint world→base_link."""
    urdf = WS_ROOT / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro"
    text = urdf.read_text(encoding="utf-8")
    # Buscar la línea: <origin xyz="-0.85 0 0.850" rpy="0 0 0"/>
    match = re.search(
        r'<origin\s+xyz="(-0?\.85)\s+(0(?:\.0+)?)\s+(0?\.85(?:0+)?)"\s+rpy="0 0 0"/>',
        text,
    )
    assert match is not None, (
        "URDF ur5.urdf.xacro ya no contiene el origin xyz=\"-0.85 0 0.850\". "
        "Si la geometría cambió, actualiza BASE_LINK_IN_WORLD en "
        "ur5_tools/geometry_constants.py para que coincida."
    )
    urdf_xyz = (float(match.group(1)), float(match.group(2)), float(match.group(3)))
    assert BASE_LINK_IN_WORLD == pytest.approx(urdf_xyz), (
        f"Mismatch URDF↔constante: URDF={urdf_xyz} vs "
        f"BASE_LINK_IN_WORLD={BASE_LINK_IN_WORLD}"
    )


def test_world_to_base_and_back_is_identity() -> None:
    pos_world = (0.5, -0.3, 0.05)
    assert base_to_world(world_to_base(pos_world)) == pytest.approx(pos_world)


def test_world_to_base_subtracts_base_offset() -> None:
    # Punto en world coincidente con el origen base_link → debe dar (0,0,0).
    assert world_to_base(BASE_LINK_IN_WORLD) == pytest.approx((0.0, 0.0, 0.0))


def test_no_hardcoded_base_link_offset_in_production_code() -> None:
    """Detecta nuevos hardcodes del literal (-0.85, 0, 0.85) en src/.

    Usa AST para inspeccionar sólo expresiones reales (no docstrings ni
    comentarios). Una tupla de 3 floats que coincida con
    BASE_LINK_IN_WORLD (con tolerancia 1e-6) cuenta como hardcode.

    Excepciones permitidas:
      - El propio módulo geometry_constants.py (es la fuente única).
      - Tests bajo test/ (replican la constante para independencia).
    """
    import ast

    src_root = WS_ROOT / "src"
    target = BASE_LINK_IN_WORLD
    offenders = []

    def _is_target_tuple(node: ast.AST) -> bool:
        if not isinstance(node, ast.Tuple) or len(node.elts) != 3:
            return False
        vals = []
        for elt in node.elts:
            # Soporta números positivos y negativos (UnaryOp(USub, Constant)).
            if isinstance(elt, ast.Constant) and isinstance(elt.value, (int, float)):
                vals.append(float(elt.value))
            elif (
                isinstance(elt, ast.UnaryOp)
                and isinstance(elt.op, ast.USub)
                and isinstance(elt.operand, ast.Constant)
                and isinstance(elt.operand.value, (int, float))
            ):
                vals.append(-float(elt.operand.value))
            else:
                return False
        return all(abs(v - t) < 1e-6 for v, t in zip(vals, target))

    for py_file in src_root.rglob("*.py"):
        rel = py_file.relative_to(src_root)
        if "test" in rel.parts:
            continue
        if py_file.name == "geometry_constants.py":
            continue
        try:
            tree = ast.parse(py_file.read_text(encoding="utf-8", errors="replace"))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if _is_target_tuple(node):
                offenders.append(f"{rel}:{getattr(node, 'lineno', '?')}")

    assert not offenders, (
        "Hardcodes de la tupla (-0.85, 0.0, 0.85) detectados en código "
        "productivo. Importa BASE_LINK_IN_WORLD desde "
        "ur5_tools.geometry_constants:\n  " + "\n  ".join(offenders)
    )
