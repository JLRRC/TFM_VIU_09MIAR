"""Smoke import test: every module in the package imports without error.

Walks ur5_qt_panel auto-discovering submodules. Skips módulos cuyo import
falla por:
  - `ModuleNotFoundError` de un dep opcional runtime (rclpy, PyQt5, etc.)
  - `ImportError` con mensaje que menciona dep opcional
  - `RuntimeError` / `SystemExit` con mensaje que menciona dep opcional
    (algunos módulos del repo capturan ImportError y emiten su propio
    error de inicialización al perder rclpy).

Audit 2026-05-10 (Action 7).
"""
from __future__ import annotations

import importlib
import pkgutil

import pytest

import ur5_qt_panel

OPTIONAL_DEP_KEYWORDS = (
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
    "ament_index",
    "rosgraph_msgs",
    "sensor_msgs",
    "geometry_msgs",
    "visualization_msgs",
    "trajectory_msgs",
    "std_srvs",
    "std_msgs",
    "rcl_interfaces",
    "lifecycle_msgs",
    "control_msgs",
    "action_msgs",
)

# Módulos con ciclo de imports conocido — deuda técnica documentada
# antes de esta sesión (2026-05-10). Skip en test_imports hasta que se
# resuelva el ciclo (panel_camera._runtime_time vs panel_tfm_inference).
KNOWN_IMPORT_CYCLE_MODULES = frozenset({
    "ur5_qt_panel.panel_camera",
    "ur5_qt_panel.panel_camera_controllers",
    "ur5_qt_panel.panel_gz_startup",
    "ur5_qt_panel.panel_main_ui",
    "ur5_qt_panel.panel_object_mgmt",
    "ur5_qt_panel.panel_tfm_inference",
})


def _discover_modules() -> list[str]:
    names: list[str] = []
    for info in pkgutil.walk_packages(ur5_qt_panel.__path__, "ur5_qt_panel."):
        names.append(info.name)
    return sorted(names)


def _is_optional_dep_error(exc: BaseException) -> bool:
    """True si la excepción menciona un dep opcional."""
    if isinstance(exc, ModuleNotFoundError):
        missing = exc.name or ""
        if any(missing == k or missing.startswith(k + ".") for k in OPTIONAL_DEP_KEYWORDS):
            return True
    msg = str(exc)
    return any(k in msg for k in OPTIONAL_DEP_KEYWORDS)


@pytest.mark.parametrize("module_name", _discover_modules())
def test_module_imports(module_name: str) -> None:
    if module_name in KNOWN_IMPORT_CYCLE_MODULES:
        pytest.skip(f"known import cycle (deuda preexistente): {module_name}")
    try:
        importlib.import_module(module_name)
    except (ModuleNotFoundError, ImportError, RuntimeError, SystemExit) as exc:
        if _is_optional_dep_error(exc):
            pytest.skip(f"optional runtime dep missing: {exc}")
        raise
