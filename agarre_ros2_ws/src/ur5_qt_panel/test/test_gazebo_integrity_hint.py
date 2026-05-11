# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_gazebo_integrity_hint.py
# Resumen: tests del espejo cliente del gate Gazebo en pick_demo_dispatcher.
"""Pruebas del helper _gazebo_integrity_status (sin ROS, sin Qt)."""

from __future__ import annotations

from pathlib import Path

import pytest

from ur5_qt_panel.pick_demo_dispatcher import (
    _gazebo_integrity_status,
    _GAZEBO_INTEGRITY_FLAG_DEFAULT,
)


def test_default_flag_path_is_well_known() -> None:
    assert _GAZEBO_INTEGRITY_FLAG_DEFAULT == "/tmp/gazebo_model_integrity_ok"


def test_blocked_without_flag_or_override(tmp_path: Path) -> None:
    missing = tmp_path / "absent"
    ok, reason = _gazebo_integrity_status(env={}, flag_path=str(missing))
    assert ok is False
    assert "missing_flag" in reason
    assert str(missing) in reason


def test_allowed_when_flag_file_exists(tmp_path: Path) -> None:
    flag = tmp_path / "ok"
    flag.write_text("GAZEBO_INTEGRITY_OK_AT=...\n")
    ok, reason = _gazebo_integrity_status(env={}, flag_path=str(flag))
    assert ok is True
    assert "flag" in reason
    assert str(flag) in reason


@pytest.mark.parametrize("value", ["1", "true", "yes", "on", "True", "TRUE"])
def test_override_truthy_allows(value: str, tmp_path: Path) -> None:
    ok, reason = _gazebo_integrity_status(
        env={"ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY": value},
        flag_path=str(tmp_path / "missing"),
    )
    assert ok is True
    assert "override" in reason


@pytest.mark.parametrize("value", ["0", "false", "no", "off", "", " "])
def test_override_falsy_keeps_block(value: str, tmp_path: Path) -> None:
    ok, _ = _gazebo_integrity_status(
        env={"ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY": value},
        flag_path=str(tmp_path / "missing"),
    )
    assert ok is False


def test_env_var_overrides_default_path(tmp_path: Path) -> None:
    custom = tmp_path / "custom"
    custom.write_text("x")
    ok, reason = _gazebo_integrity_status(
        env={"GAZEBO_INTEGRITY_FLAG": str(custom)},
        flag_path=None,
    )
    assert ok is True
    assert str(custom) in reason
