#!/usr/bin/env python3
"""F9 audit (2026-05-10): parity entre fragmentos bridges/ y archivo compuesto.

F5 audit (2026-05-10): paths actualizados — fragments y target viven
en ``src/ur5_gazebo/config/`` (antes en ``scripts/`` fuera de paquete).

El runtime consume ``src/ur5_gazebo/config/ros_gz_bridge.yaml`` (compuesto).
Los fragmentos en ``src/ur5_gazebo/config/bridges/[0-9]*.yaml`` son la
fuente editable. Cualquier divergencia entre ambos es un drift.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

WS = Path(__file__).resolve().parents[3]
COMPOSED = WS / "src" / "ur5_gazebo" / "config" / "ros_gz_bridge.yaml"
FRAGMENTS_DIR = WS / "src" / "ur5_gazebo" / "config" / "bridges"


def _load_yaml_list(path: Path) -> list:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or []


def test_compose_script_exists() -> None:
    assert (FRAGMENTS_DIR / "compose.py").is_file()


def test_fragments_dir_has_expected_files() -> None:
    fragments = sorted(FRAGMENTS_DIR.glob("[0-9]*.yaml"))
    assert len(fragments) >= 4, (
        "F9 espera ≥4 fragmentos: 01_core, 02_cameras, 03_drop_anchors, "
        "04_gripper_anchors"
    )
    names = {p.name for p in fragments}
    expected = {
        "01_core.yaml",
        "02_cameras.yaml",
        "03_drop_anchors.yaml",
        "04_gripper_anchors.yaml",
    }
    assert expected <= names, f"Faltan fragmentos: {expected - names}"


def test_composed_total_matches_fragments_concat() -> None:
    composed = _load_yaml_list(COMPOSED)
    expected = []
    for path in sorted(FRAGMENTS_DIR.glob("[0-9]*.yaml")):
        expected.extend(_load_yaml_list(path))
    assert len(composed) == len(expected), (
        f"Compuesto tiene {len(composed)} entries, fragmentos suman "
        f"{len(expected)} — corre `python3 src/ur5_gazebo/config/bridges/compose.py`"
    )


def test_compose_script_reproduces_composed_file() -> None:
    """Idempotencia: ejecutar compose.py sobre los fragmentos = archivo en disco."""
    sys.path.insert(0, str(FRAGMENTS_DIR))
    try:
        # Forzar recarga si ya estaba importado (otros tests).
        if "compose" in sys.modules:
            del sys.modules["compose"]
        import compose  # type: ignore[import-not-found]
        generated = compose.compose()
    finally:
        sys.path.pop(0)
    on_disk = COMPOSED.read_text(encoding="utf-8")
    assert generated == on_disk, (
        "El archivo ros_gz_bridge.yaml no coincide con la composición de "
        "fragmentos. Ejecuta `python3 src/ur5_gazebo/config/bridges/compose.py` y "
        "vuelve a comitear."
    )


@pytest.mark.parametrize("required_topic,fragment", [
    ("/clock", "01_core.yaml"),
    ("/world/ur5_mesa_objetos/pose/info", "01_core.yaml"),
    ("/camera_overhead/image", "02_cameras.yaml"),
])
def test_critical_bridges_in_correct_fragment(required_topic: str, fragment: str) -> None:
    entries = _load_yaml_list(FRAGMENTS_DIR / fragment)
    found = any(
        e["ros_topic_name"] == required_topic
        or e["gz_topic_name"] == required_topic
        for e in entries
    )
    assert found, f"Topic {required_topic!r} ausente del fragmento {fragment}"
