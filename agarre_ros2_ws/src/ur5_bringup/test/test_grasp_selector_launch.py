#!/usr/bin/env python3
"""F6 audit-v4 (2026-05-08): test parseable de grasp_selector.launch.py."""
from __future__ import annotations

from pathlib import Path

import pytest


WS_DIR = Path(__file__).resolve().parents[3]
LAUNCH_FILE = WS_DIR / "src/ur5_bringup/launch/grasp_selector.launch.py"


def test_launch_file_exists() -> None:
    assert LAUNCH_FILE.is_file(), f"missing {LAUNCH_FILE}"


def test_launch_file_has_generate_launch_description() -> None:
    txt = LAUNCH_FILE.read_text(encoding="utf-8")
    assert "def generate_launch_description" in txt
    assert "tfm_grasping" in txt
    assert "grasp_selector_node" in txt


def test_launch_file_declares_canonical_args() -> None:
    txt = LAUNCH_FILE.read_text(encoding="utf-8")
    expected_args = (
        "ws_radius_m",
        "ws_min_radius_m",
        "tcp_down_tolerance_rad",
        "require_tcp_down",
    )
    for arg in expected_args:
        assert arg in txt, f"launch arg {arg!r} missing"


def test_launch_file_parses() -> None:
    """Importa y ejecuta generate_launch_description() sin raise."""
    try:
        from launch import LaunchDescription  # noqa: F401
    except ImportError:
        pytest.skip("launch package not installed in this env")
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "grasp_selector_launch", str(LAUNCH_FILE)
    )
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    assert ld is not None
    actions = list(ld.entities)
    # 4 DeclareLaunchArgument + 1 Node = 5
    assert len(actions) >= 5, f"expected >= 5 actions, got {len(actions)}"
