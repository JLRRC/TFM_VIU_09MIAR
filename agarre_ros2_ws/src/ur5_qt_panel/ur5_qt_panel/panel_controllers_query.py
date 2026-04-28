#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_controllers_query.py
# Contenido: Helpers de query de controllers / controller_manager extraidos de panel_utils.
"""Helpers de query de controller_manager extraidos de panel_utils.py (refactor B.3).

Extraido de ``panel_utils.py`` (lineas 629-757 originales). Funciones que
consultan estado de controladores ros2_control + descubren la ruta del
controller_manager. Sin estado funcional propio (caches privados con TTL
para minimizar llamadas IPC).

Reexportado desde panel_utils para mantener compatibilidad con
``panel_gz_startup``, ``panel_controllers``, ``panel_camera_controllers``.

Funciones publicas:
- gripper_controller_defined
- resolve_controller_manager
- list_controllers_state
- list_active_controllers
- ros2_control_running

Helpers privados:
- _discover_controller_manager
- _controller_manager_path
"""

from __future__ import annotations

import os
import time
from typing import Dict, Optional, Set, Tuple

try:
    from controller_manager_msgs.srv import ListControllers
except Exception:  # pragma: no cover
    ListControllers = None  # type: ignore

try:
    from .panel_config import (
        ROS_AVAILABLE,
        UR5_CONTROLLERS_YAML,
        rclpy,
    )
except Exception:  # pragma: no cover
    from .panel_config import (  # type: ignore
        ROS_AVAILABLE,
        UR5_CONTROLLERS_YAML,
        rclpy,
    )


def _create_graph_node(name: str):
    """Late-import wrapper para evitar circular import con panel_utils."""

    from .panel_utils import _create_graph_node as _impl
    return _impl(name)

# Caches privados (compartidos entre llamadas en el mismo proceso).
_GRIPPER_CTRL_CACHE: Tuple[Optional[float], Optional[bool]] = (None, None)
_CM_CACHE: Tuple[float, str] = (0.0, "/controller_manager")
_CM_CACHE_TTL_SEC = 2.0


def _log_exception(context: str, exc: Exception) -> None:
    """Inline de panel_utils._log_exception (evita circular import)."""

    debug_enabled = os.environ.get("PANEL_DEBUG_EXCEPTIONS", "").strip() in (
        "1", "true", "True"
    )
    if debug_enabled:
        import traceback
        traceback.print_exc()


# Las funciones extraidas se apenden aqui via sed.
def gripper_controller_defined() -> bool:
    global _GRIPPER_CTRL_CACHE
    try:
        if not UR5_CONTROLLERS_YAML:
            return False
        try:
            mtime = os.path.getmtime(UR5_CONTROLLERS_YAML)
        except FileNotFoundError:
            mtime = None
        cached_mtime, cached_value = _GRIPPER_CTRL_CACHE
        if cached_mtime == mtime and cached_value is not None:
            return cached_value
        if mtime is None:
            _GRIPPER_CTRL_CACHE = (mtime, False)
            return False
        with open(UR5_CONTROLLERS_YAML, "r", encoding="utf-8", errors="ignore") as f:
            value = "gripper_controller:" in f.read()
        _GRIPPER_CTRL_CACHE = (mtime, value)
        return value
    except Exception as exc:
        _log_exception("gripper_controller_defined", exc)
        return False


def _discover_controller_manager(node) -> str:
    try:
        services = node.get_service_names_and_types()
    except Exception as exc:
        _log_exception("discover controller_manager", exc)
        return ""
    for name, _types in services:
        if name.endswith("/controller_manager/list_controllers"):
            return name.rsplit("/list_controllers", 1)[0]
    return ""


def _controller_manager_path(preferred: str = "") -> str:
    if preferred:
        return preferred
    env_path = os.environ.get("PANEL_CONTROLLER_MANAGER", "").strip()
    if env_path:
        return env_path
    return "/controller_manager"


def resolve_controller_manager(node=None, preferred: str = "") -> str:
    """Return controller_manager namespace, discovering when possible."""
    global _CM_CACHE
    cm_path = _controller_manager_path(preferred)
    if cm_path != "/controller_manager":
        return cm_path
    now = time.monotonic()
    if (now - _CM_CACHE[0]) <= _CM_CACHE_TTL_SEC:
        return _CM_CACHE[1]
    created = False
    if node is None:
        node = _create_graph_node("panel_cm_discover")
        created = node is not None
    if node is None:
        return cm_path
    try:
        discovered = _discover_controller_manager(node)
        resolved = discovered or cm_path
        _CM_CACHE = (now, resolved)
        return resolved
    finally:
        if created:
            try:
                node.destroy_node()
            except Exception as exc:
                _log_exception("destroy cm discover node", exc)


def list_controllers_state(
    controller_manager: str = "",
) -> Tuple[Optional[Dict[str, str]], Optional[str]]:
    if not ROS_AVAILABLE or ListControllers is None:
        return None, "ROS no disponible"
    node = _create_graph_node("panel_ctrl_probe")
    if node is None:
        return None, "node unavailable"
    cm_path = resolve_controller_manager(node, preferred=controller_manager)
    client = node.create_client(ListControllers, f"{cm_path}/list_controllers")
    if not client.wait_for_service(timeout_sec=0.5):
        node.destroy_node()
        return None, "service unavailable"
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=0.6)
    if not future.done():
        node.destroy_node()
        return None, "timeout"
    result = future.result()
    states: Dict[str, str] = {}
    if result and hasattr(result, "controller"):
        for ctrl in result.controller:
            states[str(ctrl.name)] = str(ctrl.state)
    node.destroy_node()
    return states, None


def list_active_controllers(
    controller_manager: str = "",
) -> Tuple[Optional[Set[str]], Optional[str]]:
    states, err = list_controllers_state(controller_manager=controller_manager)
    if err or states is None:
        return None, err
    active = {name for name, st in states.items() if st == "active"}
    return active, None


def ros2_control_running(controller_manager: str = "") -> bool:
    node = _create_graph_node("panel_ctrl_check")
    if node is None:
        return False
    try:
        services = node.get_service_names_and_types()
        if controller_manager:
            target = f"{controller_manager}/list_controllers"
            return any(name == target for name, _ in services)
        return any(name.endswith("/controller_manager/list_controllers") for name, _ in services)
    except Exception as exc:
        _log_exception("ros2_control_running", exc)
        return False
    finally:
        try:
            node.destroy_node()
        except Exception as exc:
            _log_exception("destroy ctrl check node", exc)

