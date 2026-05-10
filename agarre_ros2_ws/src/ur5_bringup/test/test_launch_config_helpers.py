#!/usr/bin/env python3
"""F14 (auditoría 2026-05-10): tests offline de helpers config en launch_helpers."""
from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

# launch_helpers vive en src/ur5_bringup/launch/. El conftest del paquete
# ya añade launch/ al sys.path; replicamos para robustez si se ejecuta
# directamente.
_LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))

from launch_helpers import (  # noqa: E402
    coerce_env_flag,
    coerce_env_float,
    resolve_config_value,
    select_qt_platform,
)


# ---- resolve_config_value ----


def test_resolve_uses_env_when_set():
    out = resolve_config_value(
        "FOO", "default_val",
        env={"FOO": "from_env"},
        runtime_defaults={"FOO": "from_yaml"},
    )
    assert out == "from_env"


def test_resolve_falls_to_yaml_when_env_missing():
    out = resolve_config_value(
        "FOO", "default_val",
        env={},
        runtime_defaults={"FOO": "from_yaml"},
    )
    assert out == "from_yaml"


def test_resolve_falls_to_default_when_both_missing():
    out = resolve_config_value(
        "FOO", "default_val",
        env={},
        runtime_defaults={},
    )
    assert out == "default_val"


def test_resolve_treats_empty_env_as_missing():
    out = resolve_config_value(
        "FOO", "default_val",
        env={"FOO": "   "},  # whitespace only
        runtime_defaults={"FOO": "from_yaml"},
    )
    assert out == "from_yaml"


def test_resolve_treats_empty_yaml_as_missing():
    out = resolve_config_value(
        "FOO", "default_val",
        env={},
        runtime_defaults={"FOO": "  "},
    )
    assert out == "default_val"


def test_resolve_no_runtime_defaults():
    out = resolve_config_value("FOO", "default_val", env={"FOO": "x"})
    assert out == "x"


def test_resolve_uses_os_environ_when_no_env_passed():
    """Si env=None, usa os.environ (verificable con monkeypatch)."""
    os.environ["__F14_TEST__"] = "live"
    try:
        out = resolve_config_value("__F14_TEST__", "default_val")
        assert out == "live"
    finally:
        del os.environ["__F14_TEST__"]


# ---- coerce_env_flag ----


@pytest.mark.parametrize(
    "raw,default,expected",
    [
        ("1", False, True),
        ("0", True, False),
        ("true", False, True),
        ("TRUE", False, True),
        ("False", True, False),
        ("yes", False, True),
        ("no", True, False),
        ("on", False, True),
        ("off", True, False),
        ("", True, True),  # vacío → default
        ("", False, False),
        ("garbage", True, True),  # no parseable → default
        ("garbage", False, False),
        (None, True, True),
        (None, False, False),
        ("  TRUE  ", False, True),  # whitespace trimmed
    ],
)
def test_coerce_env_flag(raw, default, expected):
    assert coerce_env_flag(raw, default) is expected


# ---- coerce_env_float ----


def test_coerce_float_valid():
    assert coerce_env_float("3.14", 0.0) == 3.14


def test_coerce_float_negative():
    assert coerce_env_float("-1.5", 0.0) == -1.5


def test_coerce_float_invalid_returns_default():
    assert coerce_env_float("abc", 99.0) == 99.0


def test_coerce_float_empty_returns_default():
    assert coerce_env_float("", 99.0) == 99.0


def test_coerce_float_none_returns_default():
    assert coerce_env_float(None, 99.0) == 99.0


def test_coerce_float_strips_whitespace():
    assert coerce_env_float("  2.5  ", 0.0) == 2.5


# ---- select_qt_platform ----


def test_qt_platform_explicit_panel_qt_platform():
    out = select_qt_platform(env={"PANEL_QT_PLATFORM": "wayland"})
    assert out == "wayland"


def test_qt_platform_explicit_qt_qpa_platform_when_panel_empty():
    out = select_qt_platform(env={"QT_QPA_PLATFORM": "minimal"})
    assert out == "minimal"


def test_qt_platform_panel_takes_priority_over_qt_qpa():
    out = select_qt_platform(env={
        "PANEL_QT_PLATFORM": "wayland",
        "QT_QPA_PLATFORM": "xcb",
    })
    assert out == "wayland"


def test_qt_platform_force_offscreen():
    out = select_qt_platform(env={"PANEL_FORCE_OFFSCREEN": "1", "DISPLAY": ":0"})
    assert out == "offscreen"


def test_qt_platform_no_display_falls_to_offscreen():
    out = select_qt_platform(env={})
    assert out == "offscreen"


def test_qt_platform_with_display_uses_default_xcb():
    out = select_qt_platform(env={"DISPLAY": ":0"})
    assert out == "xcb"


def test_qt_platform_custom_default():
    out = select_qt_platform(env={"DISPLAY": ":0"}, default="wayland")
    assert out == "wayland"


def test_qt_platform_force_offscreen_zero_does_not_force():
    out = select_qt_platform(env={"PANEL_FORCE_OFFSCREEN": "0", "DISPLAY": ":0"})
    assert out == "xcb"
