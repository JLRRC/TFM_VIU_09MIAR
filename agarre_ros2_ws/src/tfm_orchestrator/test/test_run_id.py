#!/usr/bin/env python3
"""F5 (auditoría 2026-05-10): tests del helper PICK_RUN_ID.

Cubre:
  - generate_run_id: longitud, charset, unicidad razonable.
  - is_valid_run_id: aceptación/rechazo correcto.
  - format_log_line: formato canónico, status mapping, extras quoting.
  - parse_log_line: round-trip básico.
"""
from __future__ import annotations

import pytest

from tfm_orchestrator.run_id import (
    RUN_ID_LENGTH,
    VALID_STATUS,
    format_log_line,
    generate_run_id,
    is_valid_run_id,
    parse_log_line,
    short_status,
)


# -----------------------------------------------------------------------------
# generate_run_id
# -----------------------------------------------------------------------------
def test_generate_run_id_length():
    assert len(generate_run_id()) == RUN_ID_LENGTH == 8


def test_generate_run_id_charset():
    rid = generate_run_id()
    assert all(c in "0123456789abcdef" for c in rid)


def test_generate_run_id_uniqueness_in_sample():
    # 1000 IDs en 32 bits → probabilidad de colisión < 0.01% por
    # paradoja del cumpleaños. Si vemos colisión aquí algo falla.
    ids = {generate_run_id() for _ in range(1000)}
    assert len(ids) == 1000


# -----------------------------------------------------------------------------
# is_valid_run_id
# -----------------------------------------------------------------------------
@pytest.mark.parametrize(
    "value,expected",
    [
        ("a3b8c9d1", True),
        ("00ff00ff", True),
        ("ABCDEF12", False),  # mayúsculas no válidas
        ("a3b8c9d", False),  # 7 chars
        ("a3b8c9d12", False),  # 9 chars
        ("ghijklmn", False),  # chars no hex
        ("", False),
        (None, False),
        (12345678, False),
        (("a3b8c9d1",), False),
    ],
)
def test_is_valid_run_id(value, expected):
    assert is_valid_run_id(value) is expected


# -----------------------------------------------------------------------------
# format_log_line
# -----------------------------------------------------------------------------
def test_format_log_line_basic():
    line = format_log_line("a3b8c9d1", "APPROACH", "orch", "STARTED", "dispatch")
    assert line == (
        "[PICK_RUN_ID=a3b8c9d1][PHASE=APPROACH][NODE=orch][STATUS=S] dispatch"
    )


def test_format_log_line_with_extras():
    line = format_log_line(
        "a3b8c9d1", "GRASP", "attach", "WARN", "gate_failed",
        tcp_obj_dist=0.847,
        gate_max=1.0,
    )
    assert line == (
        "[PICK_RUN_ID=a3b8c9d1][PHASE=GRASP][NODE=attach][STATUS=W] "
        "gate_failed tcp_obj_dist=0.847 gate_max=1.0"
    )


def test_format_log_line_quotes_values_with_spaces():
    line = format_log_line(
        "deadbeef", "RELEASE", "orch", "FAILED", "release_failed",
        reason="Timeout waiting for service",
    )
    assert "reason=\"Timeout waiting for service\"" in line


def test_format_log_line_quotes_values_with_brackets():
    line = format_log_line(
        "deadbeef", "TRANSPORT", "plan", "INFO", "retrying",
        attempt="[1/2]",
    )
    assert 'attempt="[1/2]"' in line


def test_format_log_line_status_mapping():
    cases = {
        "STARTED": "S",
        "FINISHED": "F",
        "FAILED": "F",
        "WARN": "W",
        "INFO": "I",
        "DEBUG": "D",
    }
    for status, expected_char in cases.items():
        line = format_log_line("aaaabbbb", "X", "n", status, "msg")
        assert f"[STATUS={expected_char}]" in line, (status, line)


def test_format_log_line_unknown_status_uses_first_letter():
    line = format_log_line("aaaabbbb", "X", "n", "QUARANTINED", "msg")
    assert "[STATUS=Q]" in line


def test_format_log_line_no_extras_no_trailing_space():
    line = format_log_line("aaaabbbb", "X", "n", "INFO", "msg")
    assert not line.endswith(" ")


def test_short_status():
    assert short_status("STARTED") == "S"
    assert short_status("info") == "I"
    assert short_status("") == "?"


def test_valid_status_set_complete():
    expected = {"STARTED", "FINISHED", "FAILED", "WARN", "INFO", "DEBUG"}
    assert VALID_STATUS == expected


# -----------------------------------------------------------------------------
# parse_log_line
# -----------------------------------------------------------------------------
def test_parse_log_line_round_trip_simple():
    line = format_log_line("a3b8c9d1", "APPROACH", "orch", "STARTED", "dispatch")
    parsed = parse_log_line(line)
    assert parsed == {
        "run_id": "a3b8c9d1",
        "phase": "APPROACH",
        "node": "orch",
        "status": "S",
        "msg": "dispatch",
    }


def test_parse_log_line_with_extras_msg_keeps_kv():
    line = format_log_line(
        "a3b8c9d1", "GRASP", "attach", "WARN", "gate_failed",
        tcp_obj_dist=0.847,
    )
    parsed = parse_log_line(line)
    assert parsed is not None
    assert parsed["msg"] == "gate_failed tcp_obj_dist=0.847"


def test_parse_log_line_returns_none_on_garbage():
    assert parse_log_line("not a log line at all") is None
    assert parse_log_line("") is None
    assert parse_log_line("[PICK_RUN_ID=abc]") is None  # incompleto
