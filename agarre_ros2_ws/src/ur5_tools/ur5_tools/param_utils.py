# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/param_utils.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Parameter helpers for validation and defaults."""

from __future__ import annotations

from typing import List, Optional


def _warn_invalid(node, name: str, value, default) -> None:
    try:
        node.get_logger().warn(
            f"Parametro '{name}' invalido ({value!r}); usando {default!r}"
        )
    except Exception:
        pass


def read_str_param(node, name: str, default: str = "") -> str:
    try:
        value = node.get_parameter(name).value
    except Exception:
        value = None
    if value is None:
        return default
    try:
        text = str(value)
    except Exception:
        _warn_invalid(node, name, value, default)
        return default
    return text


def read_str_list_param(
    node, name: str, default: Optional[List[str]] = None
) -> List[str]:
    if default is None:
        default = []
    try:
        value = node.get_parameter(name).value
    except Exception:
        value = None
    if value is None:
        return list(default)
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    if isinstance(value, str):
        return [value]
    _warn_invalid(node, name, value, default)
    return list(default)


def read_float_param(
    node,
    name: str,
    default: float = 0.0,
    min_value: Optional[float] = None,
    max_value: Optional[float] = None,
) -> float:
    try:
        value = node.get_parameter(name).value
    except Exception:
        value = None
    try:
        number = float(value)
    except Exception:
        _warn_invalid(node, name, value, default)
        number = float(default)
    if min_value is not None and number < min_value:
        _warn_invalid(node, name, number, min_value)
        number = float(min_value)
    if max_value is not None and number > max_value:
        _warn_invalid(node, name, number, max_value)
        number = float(max_value)
    return number


def read_int_param(
    node,
    name: str,
    default: int = 0,
    min_value: Optional[int] = None,
) -> int:
    try:
        value = node.get_parameter(name).value
    except Exception:
        value = None
    try:
        number = int(value)
    except Exception:
        _warn_invalid(node, name, value, default)
        number = int(default)
    if min_value is not None and number < min_value:
        _warn_invalid(node, name, number, min_value)
        number = int(min_value)
    return number
