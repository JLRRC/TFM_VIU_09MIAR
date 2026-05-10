"""Smoke import test: every module in the package imports without error.

Walks ur5_qt_panel auto-discovering submodules (incluye pick_demo/).
Skips módulos cuyo import falla con `ModuleNotFoundError` para deps
opcionales runtime (rclpy, PyQt5) o no disponibles offline.

Audit 2026-05-10 (Action 7).
"""
from __future__ import annotations

import importlib
import pkgutil

import pytest

import ur5_qt_panel

OPTIONAL_DEPS = {
    "rclpy",
    "PyQt5",
    "cv_bridge",
    "tf2_ros",
    "tf2_geometry_msgs",
    "ros_gz_interfaces",
    "controller_manager_msgs",
    "moveit_msgs",
    "ur5_panel_interfaces",
    "tfm_grasping",
}


def _discover_modules() -> list[str]:
    names: list[str] = []
    for info in pkgutil.walk_packages(ur5_qt_panel.__path__, "ur5_qt_panel."):
        names.append(info.name)
    return sorted(names)


@pytest.mark.parametrize("module_name", _discover_modules())
def test_module_imports(module_name: str) -> None:
    try:
        importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        missing = exc.name or ""
        if any(missing == dep or missing.startswith(dep + ".") for dep in OPTIONAL_DEPS):
            pytest.skip(f"optional runtime dep missing: {missing}")
        raise
