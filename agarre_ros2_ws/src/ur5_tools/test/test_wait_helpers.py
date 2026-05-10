#!/usr/bin/env python3
"""F3 audit (2026-05-10): tests del helper wait_helpers."""
from __future__ import annotations

from itertools import count

import pytest

from ur5_tools.wait_helpers import backoff_iter, wait_for_value, wait_until


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def monotonic(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt


@pytest.fixture
def fake_clock(monkeypatch):
    clk = FakeClock()
    monkeypatch.setattr("ur5_tools.wait_helpers.time.monotonic", clk.monotonic)
    return clk


def test_wait_until_returns_true_when_predicate_true_first_call(fake_clock):
    assert wait_until(lambda: True, timeout_sec=1.0, sleep_fn=fake_clock.sleep) is True


def test_wait_until_returns_true_when_predicate_becomes_true(fake_clock):
    counter = count()
    assert wait_until(
        lambda: next(counter) >= 3,
        timeout_sec=1.0,
        poll_dt=0.05,
        sleep_fn=fake_clock.sleep,
    ) is True


def test_wait_until_returns_false_on_timeout(fake_clock):
    assert wait_until(
        lambda: False,
        timeout_sec=0.5,
        poll_dt=0.1,
        sleep_fn=fake_clock.sleep,
    ) is False


def test_wait_until_zero_poll_dt_raises():
    with pytest.raises(ValueError):
        wait_until(lambda: True, timeout_sec=1.0, poll_dt=0.0)


def test_wait_for_value_returns_value(fake_clock):
    counter = count()
    val = wait_for_value(
        lambda: 42 if next(counter) >= 2 else None,
        timeout_sec=1.0,
        poll_dt=0.05,
        sleep_fn=fake_clock.sleep,
    )
    assert val == 42


def test_wait_for_value_returns_none_on_timeout(fake_clock):
    assert wait_for_value(
        lambda: None,
        timeout_sec=0.5,
        poll_dt=0.1,
        sleep_fn=fake_clock.sleep,
    ) is None


def test_backoff_iter_basic_growth():
    seq = list(backoff_iter(1.0, factor=2.0, max_attempts=4))
    assert seq == [1.0, 2.0, 4.0, 8.0]


def test_backoff_iter_capped_by_max_sec():
    seq = list(backoff_iter(1.0, factor=2.0, max_sec=3.5, max_attempts=5))
    assert seq == [1.0, 2.0, 3.5, 3.5, 3.5]


def test_backoff_iter_factor_less_than_one_raises():
    with pytest.raises(ValueError):
        list(backoff_iter(1.0, factor=0.5))


def test_backoff_iter_negative_initial_raises():
    with pytest.raises(ValueError):
        list(backoff_iter(-1.0))
