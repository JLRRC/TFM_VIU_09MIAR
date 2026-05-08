#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_env_helpers.py
# Contenido: F2 — unit tests de los helpers panel_env.py (audit-v4).
"""F2 — Tests de helpers panel_env.

Verifica el comportamiento de los helpers añadidos en F2-step3..4 que
consolidan env reads scattered:

  * get_gz_partition / get_gz_ip / get_panel_ros_timeout (F2-step3)
  * get_panel_max_fps / is_panel_ros2_only / is_panel_single_cam (F2-step4)

Cada helper tiene contrato: retorna default si env no está, parsea
robustamente si env está, no crashea con valores inválidos.
"""
from __future__ import annotations

import os

import pytest

from ur5_qt_panel.panel_env import (
    get_gz_ip,
    get_gz_partition,
    get_gz_transport_ip,
    get_panel_max_fps,
    get_panel_ros_timeout,
    is_panel_ros2_only,
    is_panel_single_cam,
)


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Garantiza env limpio antes de cada test."""
    for key in (
        "GZ_PARTITION",
        "GZ_IP",
        "GZ_TRANSPORT_IP",
        "PANEL_ROS_TIMEOUT",
        "PANEL_MAX_FPS",
        "PANEL_ROS2_ONLY",
        "PANEL_SINGLE_CAM",
    ):
        monkeypatch.delenv(key, raising=False)


# ---------------------------------------------------------------------------
# get_gz_partition / get_gz_ip
# ---------------------------------------------------------------------------


def test_get_gz_partition_default_empty() -> None:
    assert get_gz_partition() == ""


def test_get_gz_partition_strips_whitespace(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GZ_PARTITION", "  my_partition  ")
    assert get_gz_partition() == "my_partition"


def test_get_gz_partition_custom_default(monkeypatch: pytest.MonkeyPatch) -> None:
    assert get_gz_partition(default="fallback") == "fallback"


def test_get_gz_ip_default_empty() -> None:
    assert get_gz_ip() == ""


def test_get_gz_ip_strips(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GZ_IP", "  192.168.1.1  ")
    assert get_gz_ip() == "192.168.1.1"


# ---------------------------------------------------------------------------
# get_gz_transport_ip
# ---------------------------------------------------------------------------


def test_get_gz_transport_ip_default_localhost() -> None:
    assert get_gz_transport_ip() == "127.0.0.1"


def test_get_gz_transport_ip_empty_default(monkeypatch: pytest.MonkeyPatch) -> None:
    assert get_gz_transport_ip(default="") == ""


def test_get_gz_transport_ip_env_overrides(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GZ_TRANSPORT_IP", "10.0.0.5")
    assert get_gz_transport_ip() == "10.0.0.5"


# ---------------------------------------------------------------------------
# get_panel_ros_timeout
# ---------------------------------------------------------------------------


def test_get_panel_ros_timeout_default() -> None:
    assert get_panel_ros_timeout() == pytest.approx(1.5)


def test_get_panel_ros_timeout_custom_default() -> None:
    assert get_panel_ros_timeout(default=3.0) == pytest.approx(3.0)


def test_get_panel_ros_timeout_env_int(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PANEL_ROS_TIMEOUT", "5")
    assert get_panel_ros_timeout() == pytest.approx(5.0)


def test_get_panel_ros_timeout_env_float(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PANEL_ROS_TIMEOUT", "2.5")
    assert get_panel_ros_timeout() == pytest.approx(2.5)


def test_get_panel_ros_timeout_env_invalid_falls_back(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PANEL_ROS_TIMEOUT", "not-a-number")
    assert get_panel_ros_timeout() == pytest.approx(1.5)


def test_get_panel_ros_timeout_env_empty_falls_back(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PANEL_ROS_TIMEOUT", "")
    assert get_panel_ros_timeout() == pytest.approx(1.5)


# ---------------------------------------------------------------------------
# get_panel_max_fps
# ---------------------------------------------------------------------------


def test_get_panel_max_fps_default() -> None:
    assert get_panel_max_fps() == pytest.approx(12.0)


def test_get_panel_max_fps_env_valid(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PANEL_MAX_FPS", "30")
    assert get_panel_max_fps() == pytest.approx(30.0)


def test_get_panel_max_fps_zero_clamps_to_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Cap inferior: <= 0 → default."""
    monkeypatch.setenv("PANEL_MAX_FPS", "0")
    assert get_panel_max_fps() == pytest.approx(12.0)


def test_get_panel_max_fps_negative_clamps_to_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PANEL_MAX_FPS", "-5")
    assert get_panel_max_fps() == pytest.approx(12.0)


def test_get_panel_max_fps_invalid_falls_back(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PANEL_MAX_FPS", "garbage")
    assert get_panel_max_fps() == pytest.approx(12.0)


def test_get_panel_max_fps_custom_default(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PANEL_MAX_FPS", "")
    assert get_panel_max_fps(default=24.0) == pytest.approx(24.0)


# ---------------------------------------------------------------------------
# is_panel_ros2_only / is_panel_single_cam
# ---------------------------------------------------------------------------


def test_is_panel_ros2_only_default_false() -> None:
    assert is_panel_ros2_only() is False


def test_is_panel_ros2_only_set_to_one(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PANEL_ROS2_ONLY", "1")
    assert is_panel_ros2_only() is True


def test_is_panel_ros2_only_set_to_zero(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PANEL_ROS2_ONLY", "0")
    assert is_panel_ros2_only() is False


def test_is_panel_ros2_only_arbitrary_string_is_false(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PANEL_ROS2_ONLY", "true")
    # Sólo "1" cuenta — convención del proyecto.
    assert is_panel_ros2_only() is False


def test_is_panel_single_cam_default_false_when_no_ros2_only() -> None:
    """Sin PANEL_ROS2_ONLY ni PANEL_SINGLE_CAM, single_cam es False."""
    assert is_panel_single_cam() is False


def test_is_panel_single_cam_default_true_when_ros2_only(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Si PANEL_ROS2_ONLY=1 y PANEL_SINGLE_CAM no set, default es True."""
    monkeypatch.setenv("PANEL_ROS2_ONLY", "1")
    assert is_panel_single_cam(default_when_ros2_only=True) is True


def test_is_panel_single_cam_explicit_set_overrides(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """PANEL_SINGLE_CAM=0 anula el default-when-ros2-only."""
    monkeypatch.setenv("PANEL_ROS2_ONLY", "1")
    monkeypatch.setenv("PANEL_SINGLE_CAM", "0")
    assert is_panel_single_cam() is False


def test_is_panel_single_cam_explicit_one(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PANEL_SINGLE_CAM", "1")
    assert is_panel_single_cam() is True
