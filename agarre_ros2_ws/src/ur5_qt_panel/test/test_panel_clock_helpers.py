#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_clock_helpers.py
# Contenido: F15 (2026-05-01) — tests offline de panel_clock_helpers.
"""Tests offline de ``panel_clock_helpers``.

F15 consolida los helpers de timestamp/freshness duplicados en
``panel_v2._runtime_time`` y ``panel_ros._steady_time`` a un único
módulo puro. Este test verifica el contrato.
"""

from __future__ import annotations

import pytest

from ur5_qt_panel.panel_clock_helpers import (
    clock_age_threshold_exceeded,
    format_clock_age,
    steady_time,
)


def test_steady_time_is_monotonic_non_decreasing():
    t1 = steady_time()
    t2 = steady_time()
    assert t2 >= t1


def test_threshold_disabled_when_zero_or_negative():
    assert clock_age_threshold_exceeded(1.0, 0.0, now=10.0) is False
    assert clock_age_threshold_exceeded(1.0, -1.0, now=10.0) is False


def test_threshold_exceeded_when_old():
    """now=10, ts=1, threshold=5 → age=9 > 5 → exceeded."""
    assert clock_age_threshold_exceeded(1.0, 5.0, now=10.0) is True


def test_threshold_not_exceeded_when_fresh():
    """now=10, ts=8, threshold=5 → age=2 < 5 → fresh."""
    assert clock_age_threshold_exceeded(8.0, 5.0, now=10.0) is False


def test_threshold_none_msg_means_stale():
    assert clock_age_threshold_exceeded(None, 5.0, now=10.0) is True


def test_threshold_invalid_msg_means_stale():
    assert clock_age_threshold_exceeded("foo", 5.0, now=10.0) is True
    assert clock_age_threshold_exceeded(0.0, 5.0, now=10.0) is True
    assert clock_age_threshold_exceeded(-3.0, 5.0, now=10.0) is True


def test_threshold_invalid_threshold_means_stale():
    assert clock_age_threshold_exceeded(1.0, "foo", now=10.0) is True


def test_format_clock_age_none_is_fresh():
    assert format_clock_age(None) == "fresh"


def test_format_clock_age_negative_is_fresh():
    assert format_clock_age(-1.0) == "fresh"


def test_format_clock_age_large_is_stale():
    assert format_clock_age(1000.0) == "stale"
    assert format_clock_age(9999.99) == "stale"


def test_format_clock_age_normal():
    assert format_clock_age(0.0) == "0.00 s"
    assert format_clock_age(1.234) == "1.23 s"
    assert format_clock_age(12.5) == "12.50 s"


def test_format_clock_age_invalid_returns_fresh():
    assert format_clock_age("foo") == "fresh"
