#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_object_wait_helpers.py
# Contenido: F3 — tests offline de pick_object/wait_helpers.
"""Tests offline de wait_helpers."""

from __future__ import annotations

import pytest

from ur5_qt_panel.pick_object.wait_helpers import (
    clamp_grace_window,
    compute_wait_chunk_sec,
    format_wait_state_log,
)


# ---------------------------------------------------------------------------
# compute_wait_chunk_sec
# ---------------------------------------------------------------------------


def test_chunk_returns_max_when_remaining_large():
    assert compute_wait_chunk_sec(deadline=100.0, now=0.0) == 1.0


def test_chunk_returns_remaining_when_in_range():
    assert compute_wait_chunk_sec(deadline=10.5, now=10.0) == pytest.approx(0.5)


def test_chunk_returns_min_when_remaining_small():
    assert compute_wait_chunk_sec(deadline=10.05, now=10.0) == pytest.approx(0.2)


def test_chunk_returns_min_when_deadline_passed():
    assert compute_wait_chunk_sec(deadline=5.0, now=10.0) == 0.2


def test_chunk_custom_min_max():
    assert compute_wait_chunk_sec(
        deadline=100.0, now=0.0, min_chunk=0.5, max_chunk=2.0
    ) == 2.0
    assert compute_wait_chunk_sec(
        deadline=10.3, now=10.0, min_chunk=0.5, max_chunk=2.0
    ) == 0.5


# ---------------------------------------------------------------------------
# clamp_grace_window
# ---------------------------------------------------------------------------


def test_clamp_grace_above_floor():
    assert clamp_grace_window(50.0, floor=10.0) == 50.0


def test_clamp_grace_below_floor_clamps():
    assert clamp_grace_window(5.0, floor=10.0) == 10.0


def test_clamp_grace_invalid_uses_floor_as_default():
    assert clamp_grace_window("not_a_number", floor=10.0) == 10.0


def test_clamp_grace_invalid_uses_explicit_default():
    assert clamp_grace_window("invalid", floor=10.0, default=90.0) == 90.0


def test_clamp_grace_default_clamped_to_floor():
    """Si default < floor, sigue aplicando clamp a floor."""
    assert clamp_grace_window("invalid", floor=10.0, default=2.0) == 10.0


def test_clamp_grace_none_uses_default():
    assert clamp_grace_window(None, floor=1.0, default=5.0) == 5.0


# ---------------------------------------------------------------------------
# format_wait_state_log
# ---------------------------------------------------------------------------


def test_wait_state_log_full():
    s = format_wait_state_log(
        state="enter",
        label="GRASP",
        elapsed_sec=1.234,
        expected_id=42,
        expected_uuid="uu",
        last_seen_id=-1,
        last_seen_uuid="",
    )
    assert "[PICK_OBJ][WAIT_RESULT]" in s
    assert "state=enter" in s
    assert "label=GRASP" in s
    assert "elapsed=1.2s" in s
    assert "expected_id=42" in s
    assert "expected_uuid=uu" in s
    assert "last_seen_id=-1" in s
    assert "last_seen_uuid=n/a" in s


def test_wait_state_log_no_uuids():
    s = format_wait_state_log(
        state="exit",
        label="X",
        elapsed_sec=0.0,
        expected_id=-1,
        expected_uuid="",
        last_seen_id=-1,
        last_seen_uuid="",
    )
    assert "expected_uuid=n/a" in s
    assert "last_seen_uuid=n/a" in s
