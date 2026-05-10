"""Smoke import test: every package module imports without error.

Prevents regressions where a refactor breaks `from tfm_grasping import X`
without surfacing immediately. Runs offline (no rclpy init).

Audit 2026-05-10 (Action 7).
"""
from __future__ import annotations

import importlib

import pytest

MODULES = [
    "tfm_grasping",
    "tfm_grasping.config",
    "tfm_grasping.geometry",
    "tfm_grasping.grasp_inference",
    "tfm_grasping.grasp_module",
    "tfm_grasping.grasp_selector",
    "tfm_grasping.grasp_selector_node",
    "tfm_grasping.model",
    "tfm_grasping.perception",
    "tfm_grasping.ros_interface",
]


@pytest.mark.parametrize("module_name", MODULES)
def test_module_imports(module_name: str) -> None:
    importlib.import_module(module_name)
