# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_gazebo_integrity_gate.py
# Resumen: tests offline del gate puro _check_gazebo_integrity_gate.
"""Pruebas del gate de integridad Gazebo (puro, sin rclpy/ROS)."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from tfm_orchestrator.pick_orchestrator_lifecycle_node import (
    _check_gazebo_integrity_gate,
    _DEFAULT_INTEGRITY_FLAG,
)


def test_blocked_when_no_flag_and_no_override(tmp_path: Path) -> None:
    missing = tmp_path / "does_not_exist"
    ok, reason = _check_gazebo_integrity_gate(env={}, flag_path=str(missing))
    assert ok is False
    assert "missing_flag" in reason
    assert str(missing) in reason


def test_allowed_when_flag_present(tmp_path: Path) -> None:
    flag = tmp_path / "ok"
    flag.write_text("GAZEBO_INTEGRITY_OK_AT=2026-05-11T12:00:00Z\n")
    ok, reason = _check_gazebo_integrity_gate(env={}, flag_path=str(flag))
    assert ok is True
    assert "flag" in reason
    assert str(flag) in reason


@pytest.mark.parametrize("value", ["1", "true", "yes", "on", "True", "TRUE"])
def test_override_truthy_values_allow(value: str, tmp_path: Path) -> None:
    missing = tmp_path / "still_missing"
    ok, reason = _check_gazebo_integrity_gate(
        env={"ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY": value},
        flag_path=str(missing),
    )
    assert ok is True
    assert "override" in reason


@pytest.mark.parametrize("value", ["0", "false", "no", "off", "", " "])
def test_override_falsy_keeps_gate(value: str, tmp_path: Path) -> None:
    missing = tmp_path / "missing"
    ok, reason = _check_gazebo_integrity_gate(
        env={"ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY": value},
        flag_path=str(missing),
    )
    assert ok is False
    assert "missing_flag" in reason


def test_env_var_overrides_flag_path(tmp_path: Path) -> None:
    custom = tmp_path / "custom_flag"
    custom.write_text("ok")
    ok, reason = _check_gazebo_integrity_gate(
        env={"GAZEBO_INTEGRITY_FLAG": str(custom)},
        flag_path=None,
    )
    assert ok is True
    assert str(custom) in reason


def test_default_flag_constant_well_known() -> None:
    assert _DEFAULT_INTEGRITY_FLAG == "/tmp/gazebo_model_integrity_ok"


def test_real_environment_does_not_leak_into_test(tmp_path: Path) -> None:
    # Aseguramos que pasar env={} desliga del os.environ real.
    saved = os.environ.pop("ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY", None)
    try:
        os.environ["ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY"] = "1"
        # En proceso real estaria allowed; pasamos env={} para forzar gate cerrado.
        ok, _ = _check_gazebo_integrity_gate(env={}, flag_path=str(tmp_path / "x"))
        assert ok is False
    finally:
        if saved is None:
            os.environ.pop("ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY", None)
        else:
            os.environ["ALLOW_PICK_WITHOUT_GAZEBO_INTEGRITY"] = saved
