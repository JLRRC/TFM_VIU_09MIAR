#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_logging_utils.py
"""Unit tests for logging_utils — pure stdlib, no ROS/Qt needed."""
from __future__ import annotations

from ur5_qt_panel.logging_utils import _PanelLogger, timestamped_line


def test_timestamped_line_contains_message() -> None:
    result = timestamped_line("hello world")
    assert "hello world" in result


def test_timestamped_line_starts_with_bracket() -> None:
    result = timestamped_line("test")
    assert result.startswith("[")


def test_timestamped_line_contains_timestamp_format() -> None:
    result = timestamped_line("msg")
    assert "T" in result or "-" in result


def test_panel_logger_info_calls_panel_log() -> None:
    class _FakePanel:
        def __init__(self):
            self.logged: list[str] = []
        def _log(self, msg: str) -> None:
            self.logged.append(msg)

    panel = _FakePanel()
    logger = _PanelLogger(panel)
    logger.info("test message")
    assert panel.logged == ["test message"]


def test_panel_logger_stores_panel_reference() -> None:
    class _Stub:
        def _log(self, msg): pass

    stub = _Stub()
    logger = _PanelLogger(stub)
    assert logger._panel is stub
