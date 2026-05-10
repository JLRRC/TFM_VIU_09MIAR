"""Smoke import test: every module in the package imports without error.

Walks ur5_tools auto-discovering submodules. Skips modules whose import
fails with `ModuleNotFoundError` for known optional runtime deps (rclpy,
ros_gz_interfaces) so the test runs offline.

Audit 2026-05-10 (Action 7).
"""
from __future__ import annotations

import importlib
import pkgutil

import pytest

import ur5_tools

OPTIONAL_DEPS = {
    "rclpy",
    "ros_gz_interfaces",
    "controller_manager_msgs",
    "moveit_msgs",
    "ur5_panel_interfaces",
    "tf2_ros",
    "tf2_geometry_msgs",
}


def _discover_modules() -> list[str]:
    names: list[str] = []
    for info in pkgutil.iter_modules(ur5_tools.__path__, "ur5_tools."):
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
