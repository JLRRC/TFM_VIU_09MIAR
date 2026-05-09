#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_ros_msg_helpers.py
"""Tests para panel_ros_msg_helpers (F3 step 2)."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from ur5_qt_panel.panel_ros_msg_helpers import (
    extract_grasp_rect,
    resolve_ros_message_class,
)


# ---------------------------------------------------------------------------
# resolve_ros_message_class
# ---------------------------------------------------------------------------


def test_resolve_empty_returns_none():
    assert resolve_ros_message_class("") is None
    assert resolve_ros_message_class(None) is None  # type: ignore[arg-type]


def test_resolve_unknown_returns_none():
    """Tipo claramente inexistente devuelve None (no lanza)."""
    assert resolve_ros_message_class("nonexistent_pkg/msg/NonExistent") is None


def test_resolve_known_message_or_skip():
    """Si rosidl_runtime_py está disponible, std_msgs/msg/String resuelve."""
    cls = resolve_ros_message_class("std_msgs/msg/String")
    if cls is None:
        pytest.skip("rosidl_runtime_py no disponible (sin ROS sourceado)")
    # Verificamos que es una clase importable, sin instanciar.
    assert isinstance(cls, type) or callable(cls)


# ---------------------------------------------------------------------------
# extract_grasp_rect — 3 contratos
# ---------------------------------------------------------------------------


def _bbox2d(cx: float, cy: float, w: float, h: float, theta: float = 0.0):
    """Construye un mock de vision_msgs/BoundingBox2D."""
    position = SimpleNamespace(x=cx, y=cy)
    center = SimpleNamespace(position=position, theta=theta)
    return SimpleNamespace(center=center, size_x=w, size_y=h)


def test_bbox2d_basic():
    msg = _bbox2d(100.0, 200.0, 50.0, 30.0, math.pi / 4)
    rect = extract_grasp_rect(msg)
    assert rect is not None
    assert rect["cx"] == 100.0
    assert rect["cy"] == 200.0
    assert rect["w"] == 50.0
    assert rect["h"] == 30.0
    assert rect["theta_rad"] == pytest.approx(math.pi / 4)
    assert rect["angle_deg"] == pytest.approx(45.0)


def test_bbox2d_default_theta_zero():
    msg = _bbox2d(0, 0, 10, 10)
    rect = extract_grasp_rect(msg)
    assert rect is not None
    assert rect["theta_rad"] == 0.0


def test_custom_message_contract():
    msg = SimpleNamespace(c_x=5.0, c_y=10.0, w=2.0, h=4.0, theta=math.pi)
    rect = extract_grasp_rect(msg)
    assert rect is not None
    assert rect["cx"] == 5.0
    assert rect["cy"] == 10.0
    assert rect["w"] == 2.0
    assert rect["h"] == 4.0
    assert rect["theta_rad"] == pytest.approx(math.pi)
    assert rect["angle_deg"] == pytest.approx(180.0)


def test_float_array_contract():
    msg = SimpleNamespace(data=[1.0, 2.0, 3.0, 4.0, 0.5])
    rect = extract_grasp_rect(msg)
    assert rect is not None
    assert rect["cx"] == 1.0
    assert rect["cy"] == 2.0
    assert rect["w"] == 3.0
    assert rect["h"] == 4.0
    assert rect["theta_rad"] == 0.5


def test_float_array_too_short_returns_none():
    msg = SimpleNamespace(data=[1.0, 2.0])
    assert extract_grasp_rect(msg) is None


def test_unknown_message_returns_none():
    msg = SimpleNamespace(unrelated_attr=42)
    assert extract_grasp_rect(msg) is None


def test_none_message_returns_none():
    assert extract_grasp_rect(None) is None


def test_bbox2d_takes_precedence_over_custom():
    """Si un mensaje tiene ambos contratos (raro), el de vision_msgs gana."""
    position = SimpleNamespace(x=999.0, y=999.0)
    center = SimpleNamespace(position=position, theta=0.0)
    msg = SimpleNamespace(
        center=center, size_x=999.0, size_y=999.0,
        c_x=1.0, c_y=2.0, w=3.0, h=4.0, theta=0.5,
    )
    rect = extract_grasp_rect(msg)
    assert rect is not None
    # vision_msgs/BoundingBox2D va primero
    assert rect["cx"] == 999.0
