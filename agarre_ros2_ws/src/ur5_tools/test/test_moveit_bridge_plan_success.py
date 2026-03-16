#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_moveit_bridge_plan_success.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Unit tests for MoveIt bridge plan success parsing helpers."""

import pytest

pytest.importorskip("rclpy")

from ur5_tools.ur5_moveit_bridge import UR5MoveItBridge  # noqa: E402


class _CodeObj:
    def __init__(self, val):
        self.val = val


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        (True, 1),
        (False, 0),
        (1, 1),
        (0, 0),
        (_CodeObj(1), 1),
        (_CodeObj(-1), -1),
    ],
)
def test_plan_success_code(raw, expected):
    assert UR5MoveItBridge._plan_success_code(raw) == expected


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        (True, True),
        (False, False),
        (1, True),
        (0, False),
        (_CodeObj(1), True),
        (_CodeObj(-1), False),
        ("SUCCESS", True),
        ("ABORTED", False),
        (None, True),
    ],
)
def test_plan_success_ok(raw, expected):
    assert UR5MoveItBridge._plan_success_ok(raw) is expected
