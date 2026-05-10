#!/usr/bin/env python3
"""F12 audit (2026-05-10): tests del kinematics.yaml.

Verifica que la caché IK persistente está habilitada (acelera ciclos
repetitivos) y que el solver usado es CachedKDLKinematicsPlugin.
"""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
KIN_YAML = PKG / "config" / "kinematics.yaml"


def _load() -> dict:
    return yaml.safe_load(KIN_YAML.read_text(encoding="utf-8"))


def test_kinematics_yaml_exists() -> None:
    assert KIN_YAML.is_file()


def test_manipulator_uses_cached_kdl() -> None:
    data = _load()
    assert data["manipulator"]["kinematics_solver"] == (
        "cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin"
    )


def test_cached_ik_path_is_persistent() -> None:
    """F12: cached_ik_path no puede estar vacío (perdería caché entre runs)."""
    data = _load()
    cache_path = data["manipulator"]["cached_ik_kinematics"]["cached_ik_path"]
    assert cache_path, (
        "cached_ik_path vacío — caché IK no persiste entre runs. "
        "F12 audit recomienda 'log/cached_ik.bin' o ruta absoluta."
    )


@pytest.mark.parametrize("key,minimum,maximum", [
    ("max_cache_size", 1000, 50000),
    ("min_pose_distance", 0.001, 0.05),
    ("min_joint_config_distance", 0.01, 0.5),
])
def test_cached_ik_params_in_range(key: str, minimum: float, maximum: float) -> None:
    """Sanity bounds para tunables de caché IK."""
    data = _load()
    val = data["manipulator"]["cached_ik_kinematics"][key]
    assert minimum <= val <= maximum, (
        f"cached_ik_kinematics.{key}={val} fuera del rango razonable "
        f"[{minimum}, {maximum}]"
    )
