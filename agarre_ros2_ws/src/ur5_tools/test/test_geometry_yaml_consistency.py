#!/usr/bin/env python3
# Ruta/archivo: src/ur5_tools/test/test_geometry_yaml_consistency.py
# Contenido: Test de consistencia entre constantes Python y geometry.yaml.
# Uso breve: pytest src/ur5_tools/test/test_geometry_yaml_consistency.py
"""F2.3 audit (2026-05-10): garantía de coherencia geometry constants.

El YAML ``ur5_description/config/geometry.yaml`` y el módulo
``ur5_tools.geometry_constants`` son la fuente de verdad declarativa
y la fuente de verdad runtime, respectivamente. Este test exige que
ambos estén alineados — protege contra refactors que muten una sin
la otra.
"""
from __future__ import annotations

import os
from pathlib import Path

import pytest


def _resolve_geometry_yaml() -> Path:
    """Resuelve geometry.yaml desde el source tree (sin install ROS).

    Necesario porque el test corre en CI offline sin
    ``ament_index_python`` ni install share path.
    """
    here = Path(__file__).resolve().parent
    candidates = [
        here.parent.parent / "ur5_description" / "config" / "geometry.yaml",
        here.parent.parent.parent / "src" / "ur5_description" / "config" / "geometry.yaml",
    ]
    for cand in candidates:
        if cand.is_file():
            return cand
    pytest.skip(f"geometry.yaml not found in: {candidates}")


def test_geometry_yaml_exists() -> None:
    """El YAML declarativo debe existir en ur5_description/config/."""
    yaml_path = _resolve_geometry_yaml()
    assert yaml_path.exists(), f"missing {yaml_path}"
    assert yaml_path.stat().st_size > 0, "geometry.yaml is empty"


def test_geometry_yaml_consistent_with_python_constants() -> None:
    """``geometry_constants`` debe coincidir con ``geometry.yaml``."""
    yaml_path = _resolve_geometry_yaml()
    from ur5_tools.geometry_constants import assert_consistent_with_yaml

    # No debe lanzar — si lanza, hay drift entre YAML y constantes Python.
    assert_consistent_with_yaml(str(yaml_path))


def test_urdf_offsets_match_yaml() -> None:
    """El URDF y el YAML deben coincidir en offsets críticos.

    Comparación textual ligera: el URDF no se parsea con xacro aquí
    (no instalado en el venv); buscamos las strings esperadas.
    """
    here = Path(__file__).resolve().parent
    urdf_candidates = [
        here.parent.parent / "ur5_description" / "urdf" / "ur5.urdf.xacro",
        here.parent.parent.parent / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro",
    ]
    urdf_path = next((p for p in urdf_candidates if p.is_file()), None)
    if urdf_path is None:
        pytest.skip("ur5.urdf.xacro not found")
    text = urdf_path.read_text(encoding="utf-8")
    # ur5_base_in_world: -0.85, 0, 0.850
    assert 'xyz="-0.85 0 0.850"' in text, "URDF base offset drift vs geometry.yaml"
    # rg2_tcp_offset: 0 0 0.175
    assert 'value="0 0 0.175"' in text, "URDF rg2_contact_tcp_xyz drift"


def test_panel_settings_no_longer_owns_ur5_base_z() -> None:
    """``panel_settings.yaml`` no debe duplicar ``ur5_base_z`` (P-03).

    F2.2 movió la fuente de verdad a geometry.yaml. Si alguien la
    reintroduce, este test falla.
    """
    here = Path(__file__).resolve().parent
    candidates = [
        here.parent.parent / "ur5_qt_panel" / "config" / "panel_settings.yaml",
        here.parent.parent.parent / "src" / "ur5_qt_panel" / "config" / "panel_settings.yaml",
    ]
    yaml_path = next((p for p in candidates if p.is_file()), None)
    if yaml_path is None:
        pytest.skip("panel_settings.yaml not found")
    text = yaml_path.read_text(encoding="utf-8")
    # El comentario informativo está permitido; la key real no.
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        assert not stripped.startswith("ur5_base_z:"), (
            f"P-03 regression: panel_settings.yaml reintroduce ur5_base_z "
            f"línea: {stripped!r}"
        )
