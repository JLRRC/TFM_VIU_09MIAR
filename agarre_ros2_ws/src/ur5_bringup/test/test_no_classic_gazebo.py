#!/usr/bin/env python3
"""F4 audit (2026-05-10): garantizar que no se cuela Gazebo Classic.

El stack TFM usa Gazebo Sim moderno (Harmonic) + ros_gz. Cualquier
referencia a las APIs de Gazebo Classic (gazebo_msgs, gazebo_ros,
libgazebo_, gazebo_ros2_control) sería un anti-patrón regresivo.

Este test recorre todos los .py, .xml, .yaml, .sdf y .launch del
workspace y falla si encuentra cualquier import o uso de Classic.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

WS = Path(__file__).resolve().parents[3]

# Nada de estos patrones debería aparecer en código productivo.
FORBIDDEN_PATTERNS = [
    re.compile(r"\bfrom\s+gazebo_msgs\b"),
    re.compile(r"\bimport\s+gazebo_msgs\b"),
    re.compile(r"\bfrom\s+gazebo_ros\b"),
    re.compile(r"\bimport\s+gazebo_ros\b"),
    re.compile(r"\blibgazebo_\w+"),
    re.compile(r"\bgazebo_ros2_control\b"),
]

# Carpetas que no deben analizarse (cache, build, docs históricos).
EXCLUDED_DIRS = {
    "build",
    "install",
    "log",
    "historico",
    "auditoria",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    "__pycache__",
    "reports",
    ".coverage",
}

# Extensiones de archivos productivos a chequear.
SCANNED_SUFFIXES = {".py", ".yaml", ".yml", ".xml", ".sdf", ".xacro", ".urdf"}


def _iter_files() -> list[Path]:
    files: list[Path] = []
    for path in WS.rglob("*"):
        if not path.is_file():
            continue
        if any(part in EXCLUDED_DIRS for part in path.parts):
            continue
        # Permitir excepciones documentadas en este propio test.
        if path.name == Path(__file__).name:
            continue
        if path.suffix.lower() in SCANNED_SUFFIXES:
            files.append(path)
    return files


@pytest.mark.parametrize("pattern", FORBIDDEN_PATTERNS, ids=lambda p: p.pattern)
def test_no_gazebo_classic_reference(pattern: re.Pattern) -> None:
    offenders: list[tuple[Path, int, str]] = []
    for path in _iter_files():
        try:
            with path.open("r", encoding="utf-8", errors="ignore") as fh:
                for lineno, line in enumerate(fh, start=1):
                    if pattern.search(line):
                        offenders.append((path, lineno, line.rstrip()))
        except OSError:
            continue
    assert not offenders, (
        f"Patrón Gazebo Classic '{pattern.pattern}' encontrado:\n"
        + "\n".join(f"  {p}:{n}: {ln}" for p, n, ln in offenders[:20])
    )


def test_gz_ros2_control_plugin_is_modern() -> None:
    """gz_ros2_control (no gazebo_ros2_control) en URDF/SDF."""
    urdf = WS / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro"
    sdf = WS / "models" / "ur5_rg2" / "model.sdf"
    txt_urdf = urdf.read_text(encoding="utf-8")
    txt_sdf = sdf.read_text(encoding="utf-8")
    assert "gz_ros2_control" in txt_urdf, "URDF no usa plugin gz_ros2_control"
    assert "gazebo_ros2_control" not in txt_urdf, "URDF usa plugin Classic"
    assert (
        "gz_ros2_control" in txt_sdf or "GazeboSimROS2ControlPlugin" in txt_sdf
    ), "SDF no usa plugin moderno"
    assert "gazebo_ros2_control" not in txt_sdf, "SDF usa plugin Classic"
