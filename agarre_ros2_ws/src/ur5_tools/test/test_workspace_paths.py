#!/usr/bin/env python3
"""F2 audit (2026-05-10): tests del helper workspace_paths."""
from __future__ import annotations

from unittest import mock

from ur5_tools.workspace_paths import (
    get_gz_ip,
    get_gz_partition,
    get_strict_physics_mode,
    get_strict_self_collision,
    get_vision_dir,
    get_ws_dir,
    get_ws_dir_path,
)


def test_ws_dir_uses_env_var_when_set() -> None:
    with mock.patch.dict("os.environ", {"WS_DIR": "/tmp/foo"}, clear=False):
        assert get_ws_dir() == "/tmp/foo"


def test_ws_dir_falls_back_to_default() -> None:
    """F1.3: default expandido desde ``~/TFM/agarre_ros2_ws`` (portable)."""
    import os as _os
    env = {k: v for k, v in __import__("os").environ.items() if k != "WS_DIR"}
    with mock.patch.dict("os.environ", env, clear=True):
        assert get_ws_dir() == _os.path.expanduser("~/TFM/agarre_ros2_ws")
        # Garantía explícita: nunca empieza por path absoluto hardcoded.
        assert get_ws_dir() != "/home/laboratorio/TFM/agarre_ros2_ws" or "laboratorio" in _os.path.expanduser("~")


def test_ws_dir_caller_default_overrides_built_in() -> None:
    env = {k: v for k, v in __import__("os").environ.items() if k != "WS_DIR"}
    with mock.patch.dict("os.environ", env, clear=True):
        assert get_ws_dir(default="/custom/ws") == "/custom/ws"


def test_ws_dir_path_returns_path_object() -> None:
    with mock.patch.dict("os.environ", {"WS_DIR": "/tmp/foo"}, clear=False):
        p = get_ws_dir_path()
        assert str(p) == "/tmp/foo"
        assert hasattr(p, "is_dir")


def test_gz_partition_strips_whitespace() -> None:
    with mock.patch.dict("os.environ", {"GZ_PARTITION": "  alpha  "}, clear=False):
        assert get_gz_partition() == "alpha"


def test_gz_partition_default_empty() -> None:
    env = {k: v for k, v in __import__("os").environ.items() if k != "GZ_PARTITION"}
    with mock.patch.dict("os.environ", env, clear=True):
        assert get_gz_partition() == ""


def test_gz_ip_uses_env_var() -> None:
    with mock.patch.dict("os.environ", {"GZ_IP": "10.0.0.1"}, clear=False):
        assert get_gz_ip() == "10.0.0.1"


def test_vision_dir_expands_tilde() -> None:
    env = {k: v for k, v in __import__("os").environ.items() if k != "VISION_DIR"}
    with mock.patch.dict("os.environ", env, clear=True):
        result = get_vision_dir()
        assert "~" not in result
        assert result.endswith("/TFM/agarre_inteligente")


def test_strict_physics_mode_permissive_parsing() -> None:
    for raw in ("1", "true", "yes", "on", "TRUE"):
        with mock.patch.dict("os.environ", {"STRICT_PHYSICS_MODE": raw}, clear=False):
            assert get_strict_physics_mode() is True
    for raw in ("0", "false", "no", "off"):
        with mock.patch.dict("os.environ", {"STRICT_PHYSICS_MODE": raw}, clear=False):
            assert get_strict_physics_mode() is False


def test_strict_self_collision_permissive_parsing() -> None:
    for raw in ("1", "true", "yes", "on"):
        with mock.patch.dict("os.environ", {"STRICT_SELF_COLLISION": raw}, clear=False):
            assert get_strict_self_collision() is True
