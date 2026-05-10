"""Smoke import test: every module in the package imports without error.

Walks ur5_tools auto-discovering submodules. Skips módulos cuyo import
falla por dep opcional runtime (rclpy, ros_gz_*, etc.).

Audit 2026-05-10 (Action 7).
"""
from __future__ import annotations

import importlib
import pkgutil

import pytest

import ur5_tools

OPTIONAL_DEP_KEYWORDS = (
    "rclpy",
    "ros_gz_interfaces",
    "controller_manager_msgs",
    "moveit_msgs",
    "ur5_panel_interfaces",
    "tf2_ros",
    "tf2_geometry_msgs",
    "ament_index",
    "rosgraph_msgs",
    "sensor_msgs",
    "geometry_msgs",
    "trajectory_msgs",
    "std_srvs",
    "std_msgs",
    "rcl_interfaces",
    "lifecycle_msgs",
    "rosidl",
    "control_msgs",
    "action_msgs",
)


def _discover_modules() -> list[str]:
    names: list[str] = []
    for info in pkgutil.iter_modules(ur5_tools.__path__, "ur5_tools."):
        names.append(info.name)
    return sorted(names)


def _is_optional_dep_error(exc: BaseException) -> bool:
    if isinstance(exc, ModuleNotFoundError):
        missing = exc.name or ""
        if any(missing == k or missing.startswith(k + ".") for k in OPTIONAL_DEP_KEYWORDS):
            return True
    msg = str(exc)
    return any(k in msg for k in OPTIONAL_DEP_KEYWORDS)


@pytest.mark.parametrize("module_name", _discover_modules())
def test_module_imports(module_name: str) -> None:
    try:
        importlib.import_module(module_name)
    except (ModuleNotFoundError, ImportError, RuntimeError, SystemExit) as exc:
        if _is_optional_dep_error(exc):
            pytest.skip(f"optional runtime dep missing: {exc}")
        raise
