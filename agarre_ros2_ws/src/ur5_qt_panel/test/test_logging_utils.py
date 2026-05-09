#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_logging_utils.py
"""Unit tests for logging_utils — pure stdlib, no ROS/Qt needed."""
from __future__ import annotations

import io

from ur5_qt_panel.logging_utils import _PanelLogger, emit_log_line, timestamped_line


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


def test_emit_log_line_appends_newline_if_missing() -> None:
    buf = io.StringIO()
    emit_log_line("hello", stream=buf)
    assert buf.getvalue() == "hello\n"


def test_emit_log_line_does_not_double_newline() -> None:
    buf = io.StringIO()
    emit_log_line("hello\n", stream=buf)
    assert buf.getvalue() == "hello\n"


def test_emit_log_line_writes_to_provided_stream() -> None:
    target = io.StringIO()
    emit_log_line("[TAG][WARN] something failed", stream=target)
    assert target.getvalue().startswith("[TAG][WARN]")


def test_emit_log_line_default_stream_is_stdout(capsys) -> None:
    emit_log_line("on stdout please")
    captured = capsys.readouterr()
    assert "on stdout please" in captured.out
    assert captured.err == ""


def test_emit_log_line_stderr_routing(capsys) -> None:
    import sys as _sys
    emit_log_line("on stderr please", stream=_sys.stderr)
    captured = capsys.readouterr()
    assert "on stderr please" in captured.err
    assert captured.out == ""


class _FlushTrackingStream(io.StringIO):
    def __init__(self) -> None:
        super().__init__()
        self.flush_calls = 0

    def flush(self) -> None:  # type: ignore[override]
        self.flush_calls += 1
        super().flush()


def test_emit_log_line_flushes_by_default() -> None:
    s = _FlushTrackingStream()
    emit_log_line("flush me", stream=s)
    assert s.flush_calls == 1


def test_emit_log_line_skips_flush_when_disabled() -> None:
    s = _FlushTrackingStream()
    emit_log_line("no flush", stream=s, flush=False)
    assert s.flush_calls == 0
