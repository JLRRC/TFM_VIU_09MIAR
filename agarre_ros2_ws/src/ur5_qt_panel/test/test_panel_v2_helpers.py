#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_helpers.py
# Contenido: F14 (2026-05-01) — tests puros de panel_v2_helpers.
"""Tests offline de ``ur5_qt_panel.panel_v2_helpers``.

F14 extrajo helpers puros desde ``panel_v2.py`` para que sean
testeables sin Qt, sin ROS y sin importar el panel completo.

Estos tests verifican el contrato de cada helper y un caso
representativo de cada rama interesante (default, edge cases,
formato canónico).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from ur5_qt_panel.panel_v2_helpers import (
    camera_required_label,
    env_flag,
    env_float,
    normalize_joint_name,
    proto_time_to_seconds,
    rot_to_rpy,
    runtime_time,
)


# ---------------------------------------------------------------------------
# runtime_time
# ---------------------------------------------------------------------------


def test_runtime_time_monotonic():
    t1 = runtime_time()
    t2 = runtime_time()
    assert t2 >= t1, "runtime_time debe ser monotónico no-decreciente"


# ---------------------------------------------------------------------------
# camera_required_label
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value,expected",
    [
        (None, "unset"),
        (True, "true"),
        (False, "false"),
    ],
)
def test_camera_required_label(value, expected):
    assert camera_required_label(value) == expected


# ---------------------------------------------------------------------------
# env_float / env_flag
# ---------------------------------------------------------------------------


def test_env_float_default_when_absent(monkeypatch):
    monkeypatch.delenv("PANEL_TEST_FOO_FLOAT", raising=False)
    assert env_float("PANEL_TEST_FOO_FLOAT", 3.14) == 3.14


def test_env_float_reads_when_present(monkeypatch):
    monkeypatch.setenv("PANEL_TEST_FOO_FLOAT", "2.71")
    assert env_float("PANEL_TEST_FOO_FLOAT", 0.0) == 2.71


def test_env_float_default_when_invalid(monkeypatch):
    monkeypatch.setenv("PANEL_TEST_FOO_FLOAT", "no-es-float")
    assert env_float("PANEL_TEST_FOO_FLOAT", 9.0) == 9.0


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("1", True),
        ("true", True),
        ("True", True),
        ("yes", True),
        ("on", True),
        ("0", False),
        ("false", False),
        ("no", False),
        ("FOO", False),
    ],
)
def test_env_flag_truthy_values(monkeypatch, raw, expected):
    monkeypatch.setenv("PANEL_TEST_FOO_FLAG", raw)
    assert env_flag("PANEL_TEST_FOO_FLAG", default=not expected) == expected


def test_env_flag_default_when_absent(monkeypatch):
    monkeypatch.delenv("PANEL_TEST_FOO_FLAG", raising=False)
    assert env_flag("PANEL_TEST_FOO_FLAG", True) is True
    assert env_flag("PANEL_TEST_FOO_FLAG", False) is False


# ---------------------------------------------------------------------------
# proto_time_to_seconds
# ---------------------------------------------------------------------------


def test_proto_time_full():
    assert proto_time_to_seconds({"sec": 5, "nsec": 500_000_000}) == 5.5


def test_proto_time_only_sec():
    assert proto_time_to_seconds({"sec": 3}) == 3.0


def test_proto_time_only_nsec():
    assert proto_time_to_seconds({"nsec": 250_000_000}) == 0.25


def test_proto_time_none_dict():
    assert proto_time_to_seconds(None) == 0.0


def test_proto_time_non_dict():
    assert proto_time_to_seconds("hello") == 0.0


def test_proto_time_with_none_values():
    assert proto_time_to_seconds({"sec": None, "nsec": None}) == 0.0


# ---------------------------------------------------------------------------
# normalize_joint_name
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name,expected",
    [
        ("ur5_rg2::shoulder_pan_joint", "shoulder_pan_joint"),
        ("/joint/elbow", "elbow"),
        ("wrist_3_joint", "wrist_3_joint"),
        ("  shoulder_lift_joint  ", "shoulder_lift_joint"),
        ("ns::ns2::tip", "tip"),
    ],
)
def test_normalize_joint_name(name, expected):
    assert normalize_joint_name(name) == expected


# ---------------------------------------------------------------------------
# rot_to_rpy
# ---------------------------------------------------------------------------


def test_rot_to_rpy_identity():
    """La matriz identidad debe dar rpy = (0, 0, 0)."""
    rot = np.eye(3)
    roll, pitch, yaw = rot_to_rpy(rot)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_rot_to_rpy_90_z():
    """Rotación 90° alrededor de Z debe dar yaw = pi/2."""
    rot = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    roll, pitch, yaw = rot_to_rpy(rot)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(math.pi / 2)


def test_rot_to_rpy_singular_handles_gimbal():
    """En el caso singular (cos(pitch)≈0) la función no lanza."""
    # pitch = pi/2 → cos(pitch) = 0 (gimbal lock)
    rot = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    roll, pitch, yaw = rot_to_rpy(rot)
    # Debería devolver pitch=pi/2 y yaw=0 (rama singular)
    assert pitch == pytest.approx(math.pi / 2, abs=1e-3)
    assert yaw == pytest.approx(0.0)
