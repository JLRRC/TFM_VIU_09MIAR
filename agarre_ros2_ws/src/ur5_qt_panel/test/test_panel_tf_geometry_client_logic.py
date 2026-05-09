#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_tf_geometry_client_logic.py
# Contenido: F16-step3 (2026-05-02) — tests offline del cliente tf_geometry.
"""Tests offline estructurales del módulo ``panel_tf_geometry_client``."""

from __future__ import annotations

import importlib.util
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"
_MODULE_PATH = _SRC_DIR / "panel_tf_geometry_client.py"


def test_module_exists():
    assert _MODULE_PATH.is_file()


def test_module_exports_canonical_names():
    src = _MODULE_PATH.read_text(encoding="utf-8")
    assert "class TfGeometryClient" in src
    assert "def is_available" in src
    assert "DEFAULT_WORLD_TO_BASE" in src
    assert "DEFAULT_APPROACH_POSE" in src


def test_module_canonical_service_names():
    src = _MODULE_PATH.read_text(encoding="utf-8")
    assert '"/tf_geometry/world_to_base"' in src
    assert '"/tf_geometry/compute_approach_pose"' in src


@pytest.mark.parametrize(
    "method",
    ("__init__", "wait_for_services", "world_to_base",
     "compute_approach_pose", "destroy"),
)
def test_class_defines_method(method):
    src = _MODULE_PATH.read_text(encoding="utf-8")
    pat = rf"def\s+{re.escape(method)}\s*\("
    assert re.search(pat, src), f"falta método '{method}'"


def test_world_to_base_returns_tuple_with_reason():
    src = _MODULE_PATH.read_text(encoding="utf-8")
    assert 'return None, "service_unavailable"' in src
    assert 'return None, "timeout"' in src
    assert 'return None, "no_response"' in src
    assert ', "ok"' in src


def test_handles_optional_imports_gracefully():
    src = _MODULE_PATH.read_text(encoding="utf-8")
    assert "try:\n    import rclpy" in src
    assert "try:\n    from geometry_msgs.msg import" in src
    assert "try:\n    from ur5_panel_interfaces.srv import" in src


def test_is_available_function_offline():
    spec = importlib.util.spec_from_file_location("_test_tfgc", _MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as e:
        pytest.skip(f"módulo no cargable sin ROS: {e}")
    assert callable(mod.is_available)
    result = mod.is_available()
    assert isinstance(result, bool)
