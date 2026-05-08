#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_time_conversion.py
"""F3-step41b (2026-05-08) — Tests offline para time_conversion."""

from __future__ import annotations

import pytest

from ur5_tools.moveit_bridge.time_conversion import (
    NANOS_PER_SEC,
    is_negative_or_zero,
    seconds_to_sec_nsec,
)


# ---------------------------------------------------------------------------
# seconds_to_sec_nsec
# ---------------------------------------------------------------------------


def test_zero():
    assert seconds_to_sec_nsec(0.0) == (0, 0)


def test_one_second_exact():
    assert seconds_to_sec_nsec(1.0) == (1, 0)


def test_one_and_half_seconds():
    sec, nsec = seconds_to_sec_nsec(1.5)
    assert sec == 1
    assert nsec == 500_000_000


def test_fractional_below_one():
    sec, nsec = seconds_to_sec_nsec(0.25)
    assert sec == 0
    assert nsec == 250_000_000


def test_negative_clamps_to_zero():
    """Valores negativos se clampean a (0, 0)."""
    assert seconds_to_sec_nsec(-1.0) == (0, 0)
    assert seconds_to_sec_nsec(-0.5) == (0, 0)


def test_overflow_handles_carry():
    """0.9999999995s podría redondear a nsec=1_000_000_000 → carry al sec."""
    sec, nsec = seconds_to_sec_nsec(0.9999999995)
    # Después del carry: (1, 0) (o cerca: depende del round())
    assert sec == 1
    assert nsec == 0


def test_large_value():
    sec, nsec = seconds_to_sec_nsec(120.5)
    assert sec == 120
    assert nsec == 500_000_000


def test_small_fractional_precision():
    """1ms = 1_000_000 ns."""
    sec, nsec = seconds_to_sec_nsec(0.001)
    assert sec == 0
    assert nsec == 1_000_000


def test_microseconds_precision():
    """1µs = 1_000 ns."""
    sec, nsec = seconds_to_sec_nsec(0.000_001)
    assert sec == 0
    assert nsec == 1_000


def test_nanos_within_range():
    """Para cualquier seconds valido, 0 <= nsec < 1e9."""
    for s in [0.0, 0.5, 1.0, 1.5, 99.999, 0.1, 0.001, 0.123456789]:
        _sec, nsec = seconds_to_sec_nsec(s)
        assert 0 <= nsec < NANOS_PER_SEC, f"nsec out of range for {s}: {nsec}"


# ---------------------------------------------------------------------------
# is_negative_or_zero
# ---------------------------------------------------------------------------


def test_negative_or_zero_true():
    assert is_negative_or_zero(0.0) is True
    assert is_negative_or_zero(-1.0) is True
    assert is_negative_or_zero(-0.001) is True


def test_negative_or_zero_false_for_positive():
    assert is_negative_or_zero(0.001) is False
    assert is_negative_or_zero(1.5) is False
    assert is_negative_or_zero(120.0) is False


def test_negative_or_zero_handles_invalid():
    """Inválido → True (sentinel "no aplicar")."""
    assert is_negative_or_zero("invalid") is True  # type: ignore[arg-type]
    assert is_negative_or_zero(None) is True  # type: ignore[arg-type]
