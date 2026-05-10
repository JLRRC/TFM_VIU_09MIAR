"""Smoke import test: every package module imports without error.

Prevents regressions where a refactor breaks `from tfm_grasping import X`
without surfacing immediately. Runs offline (no rclpy init).

Audit 2026-05-10 (Action 7).
"""
from __future__ import annotations

import importlib

import pytest

# Módulos que SÍ deben importar offline (lógica pura, sin ROS).
PURE_MODULES = [
    "tfm_grasping",
    "tfm_grasping.config",
    "tfm_grasping.geometry",
    "tfm_grasping.grasp_module",
    "tfm_grasping.grasp_selector",
    "tfm_grasping.model",
    "tfm_grasping.perception",
    "tfm_grasping.ros_interface",
]

# Módulos que requieren rclpy. En entorno offline se permite que el import
# falle con error controlado (sys.exit / RuntimeError / SystemExit) — el test
# en su lugar verifica que el fichero existe y es parseable.
ROS_DEPENDENT_MODULES = [
    "tfm_grasping.grasp_inference",
    "tfm_grasping.grasp_selector_node",
]


@pytest.mark.parametrize("module_name", PURE_MODULES)
def test_pure_module_imports(module_name: str) -> None:
    importlib.import_module(module_name)


@pytest.mark.parametrize("module_name", ROS_DEPENDENT_MODULES)
def test_ros_module_parses_offline(module_name: str) -> None:
    """Verifica AST parseable sin import (rclpy puede no estar)."""
    import ast
    import importlib.util

    spec = importlib.util.find_spec(module_name)
    assert spec is not None and spec.origin, (
        f"módulo {module_name} no localizable"
    )
    with open(spec.origin, "r", encoding="utf-8") as fh:
        ast.parse(fh.read())
