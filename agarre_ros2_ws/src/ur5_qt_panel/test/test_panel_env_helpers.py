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
    is_strict_physics_mode,
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
        "PANEL_STRICT_PHYSICS_MODE",
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


# ---------------------------------------------------------------------------
# is_strict_physics_mode
# ---------------------------------------------------------------------------


def test_is_strict_physics_mode_default_false() -> None:
    assert is_strict_physics_mode() is False


@pytest.mark.parametrize("val", ["1", "true", "True", "TRUE", "yes", "YES", "on", "On"])
def test_is_strict_physics_mode_truthy_values(
    monkeypatch: pytest.MonkeyPatch, val: str
) -> None:
    """Valores permisivos de truthy: 1/true/yes/on (case-insensitive)."""
    monkeypatch.setenv("PANEL_STRICT_PHYSICS_MODE", val)
    assert is_strict_physics_mode() is True


@pytest.mark.parametrize("val", ["0", "false", "no", "off", "", "garbage", "2"])
def test_is_strict_physics_mode_falsy_values(
    monkeypatch: pytest.MonkeyPatch, val: str
) -> None:
    """Cualquier valor que no sea {1,true,yes,on} es false."""
    monkeypatch.setenv("PANEL_STRICT_PHYSICS_MODE", val)
    assert is_strict_physics_mode() is False


def test_is_strict_physics_mode_strips_whitespace(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PANEL_STRICT_PHYSICS_MODE", "  TRUE  ")
    assert is_strict_physics_mode() is True


# ---------------------------------------------------------------------------
# F2-step5 (2026-05-08) — helpers tipo-genéricos: env_str/int/float/bool +
# optionals. Sustituyen el patrón os.environ.get + cast + fallback.
# ---------------------------------------------------------------------------


from ur5_qt_panel.panel_env import (  # noqa: E402
    env_bool,
    env_float,
    env_int,
    env_optional_bool,
    env_optional_float,
    env_optional_int,
    env_optional_str,
    env_str,
)

_GENERIC_VAR = "_TEST_PANEL_ENV_GENERIC_VAR"


def test_env_str_default_when_not_set(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(_GENERIC_VAR, raising=False)
    assert env_str(_GENERIC_VAR, "default") == "default"


def test_env_str_returns_value(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "hello")
    assert env_str(_GENERIC_VAR, "default") == "hello"


def test_env_int_default_when_not_set(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(_GENERIC_VAR, raising=False)
    assert env_int(_GENERIC_VAR, 42) == 42


def test_env_int_valid(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "100")
    assert env_int(_GENERIC_VAR, 42) == 100


def test_env_int_invalid_falls_back(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "not_an_int")
    assert env_int(_GENERIC_VAR, 42) == 42


def test_env_float_default(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(_GENERIC_VAR, raising=False)
    assert env_float(_GENERIC_VAR, 1.5) == pytest.approx(1.5)


def test_env_float_valid(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "3.14")
    assert env_float(_GENERIC_VAR, 1.5) == pytest.approx(3.14)


def test_env_float_invalid_falls_back(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "not_a_float")
    assert env_float(_GENERIC_VAR, 1.5) == pytest.approx(1.5)


@pytest.mark.parametrize(
    "truthy", ["1", "true", "TRUE", "yes", "YES", "on", "On"]
)
def test_env_bool_truthy(monkeypatch: pytest.MonkeyPatch, truthy: str) -> None:
    monkeypatch.setenv(_GENERIC_VAR, truthy)
    assert env_bool(_GENERIC_VAR, default=False) is True


@pytest.mark.parametrize(
    "falsy", ["0", "false", "FALSE", "no", "off", "", "any"]
)
def test_env_bool_falsy(monkeypatch: pytest.MonkeyPatch, falsy: str) -> None:
    monkeypatch.setenv(_GENERIC_VAR, falsy)
    assert env_bool(_GENERIC_VAR, default=True) is False


def test_env_bool_default_when_not_set(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(_GENERIC_VAR, raising=False)
    assert env_bool(_GENERIC_VAR, default=True) is True
    assert env_bool(_GENERIC_VAR, default=False) is False


def test_env_optional_str_none(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(_GENERIC_VAR, raising=False)
    assert env_optional_str(_GENERIC_VAR) is None


def test_env_optional_str_value(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "hello")
    assert env_optional_str(_GENERIC_VAR) == "hello"


def test_env_optional_int_invalid_returns_none(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "abc")
    assert env_optional_int(_GENERIC_VAR) is None


def test_env_optional_int_valid(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "7")
    assert env_optional_int(_GENERIC_VAR) == 7


def test_env_optional_float_invalid(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "abc")
    assert env_optional_float(_GENERIC_VAR) is None


def test_env_optional_float_valid(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "2.5")
    assert env_optional_float(_GENERIC_VAR) == pytest.approx(2.5)


def test_env_optional_bool_none(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(_GENERIC_VAR, raising=False)
    assert env_optional_bool(_GENERIC_VAR) is None


def test_env_optional_bool_truthy(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "yes")
    assert env_optional_bool(_GENERIC_VAR) is True


def test_env_optional_bool_falsy(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_GENERIC_VAR, "no")
    assert env_optional_bool(_GENERIC_VAR) is False
