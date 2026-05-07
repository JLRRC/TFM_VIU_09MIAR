#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_retry.py
# Contenido: B-iter9 (2026-05-03) — tests retry_with_backoff.
"""Tests para tfm_orchestrator.retry."""

from __future__ import annotations

from types import SimpleNamespace


from tfm_orchestrator.retry import retry_with_backoff


def _ok(reason="ok"):
    return SimpleNamespace(success=True, reason=reason)


def _fail(reason="fail"):
    return SimpleNamespace(success=False, reason=reason)


# ---------------------------------------------------------------------------
# happy paths
# ---------------------------------------------------------------------------


def test_retry_succeeds_on_first_attempt_no_sleep():
    sleeps = []
    result, attempts = retry_with_backoff(
        lambda: _ok(),
        sleep_fn=sleeps.append,
    )
    assert result.success is True
    assert attempts == 1
    assert sleeps == []  # NO hubo back-off (success al primer intento)


def test_retry_succeeds_on_second_attempt_one_backoff():
    calls = {"n": 0}
    def call():
        calls["n"] += 1
        return _ok() if calls["n"] == 2 else _fail("transient")

    sleeps = []
    result, attempts = retry_with_backoff(
        call,
        max_attempts=3,
        initial_backoff_sec=1.0,
        backoff_factor=2.0,
        sleep_fn=sleeps.append,
    )
    assert result.success is True
    assert attempts == 2
    assert sleeps == [1.0]  # back-off antes del 2º intento


def test_retry_succeeds_on_third_attempt_two_backoffs_with_factor():
    calls = {"n": 0}
    def call():
        calls["n"] += 1
        return _ok() if calls["n"] == 3 else _fail()

    sleeps = []
    result, attempts = retry_with_backoff(
        call,
        max_attempts=5,
        initial_backoff_sec=2.0,
        backoff_factor=3.0,
        sleep_fn=sleeps.append,
    )
    assert result.success is True
    assert attempts == 3
    assert sleeps == [2.0, 6.0]  # 2.0 → 2.0*3.0


# ---------------------------------------------------------------------------
# fail paths
# ---------------------------------------------------------------------------


def test_retry_exhausts_all_attempts_when_always_fails():
    calls = {"n": 0}
    def call():
        calls["n"] += 1
        return _fail("persistent")

    sleeps = []
    result, attempts = retry_with_backoff(
        call,
        max_attempts=3,
        sleep_fn=sleeps.append,
    )
    assert result.success is False
    assert result.reason == "persistent"
    assert attempts == 3
    assert calls["n"] == 3
    # back-offs antes del 2º y 3º intento, NO antes del 1º ni después del 3º.
    assert len(sleeps) == 2


def test_retry_default_max_attempts_is_two():
    calls = {"n": 0}
    def call():
        calls["n"] += 1
        return _fail()
    result, attempts = retry_with_backoff(call, sleep_fn=lambda _: None)
    assert attempts == 2
    assert calls["n"] == 2


def test_retry_clamps_max_attempts_to_min_one():
    calls = {"n": 0}
    def call():
        calls["n"] += 1
        return _ok()
    result, attempts = retry_with_backoff(
        call, max_attempts=0, sleep_fn=lambda _: None,
    )
    assert attempts == 1


# ---------------------------------------------------------------------------
# excepciones
# ---------------------------------------------------------------------------


def test_retry_treats_exception_as_failed_attempt():
    calls = {"n": 0}
    def call():
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("transient_crash")
        return _ok()
    result, attempts = retry_with_backoff(
        call,
        max_attempts=2,
        sleep_fn=lambda _: None,
    )
    assert result.success is True
    assert attempts == 2


def test_retry_returns_exception_placeholder_when_all_throw():
    def call():
        raise ValueError("permanently_broken")

    result, attempts = retry_with_backoff(
        call,
        max_attempts=3,
        sleep_fn=lambda _: None,
    )
    assert result.success is False
    assert "exception" in result.reason
    assert "ValueError" in result.reason
    assert "permanently_broken" in result.reason
    assert attempts == 3


def test_retry_swallows_sleep_exceptions():
    """Si sleep_fn lanza, retry no debe romper — sigue al siguiente intento."""
    def bad_sleep(_):
        raise RuntimeError("clock_unavailable")

    calls = {"n": 0}
    def call():
        calls["n"] += 1
        return _ok() if calls["n"] == 2 else _fail()

    result, attempts = retry_with_backoff(
        call,
        max_attempts=2,
        sleep_fn=bad_sleep,
    )
    assert result.success is True
    assert attempts == 2


# ---------------------------------------------------------------------------
# parámetros / clamps
# ---------------------------------------------------------------------------


def test_retry_negative_initial_backoff_is_clamped_to_zero():
    sleeps = []
    def call():
        return _fail()
    result, attempts = retry_with_backoff(
        call,
        max_attempts=2,
        initial_backoff_sec=-5.0,
        sleep_fn=sleeps.append,
    )
    assert sleeps == [0.0]


def test_retry_backoff_factor_below_one_is_clamped_to_one():
    sleeps = []
    def call():
        return _fail()
    result, attempts = retry_with_backoff(
        call,
        max_attempts=3,
        initial_backoff_sec=1.0,
        backoff_factor=0.5,  # se clampea a 1.0
        sleep_fn=sleeps.append,
    )
    assert sleeps == [1.0, 1.0]  # sin decrecer


def test_retry_call_fn_returning_none_treated_as_failure():
    calls = {"n": 0}
    def call():
        calls["n"] += 1
        return None
    result, attempts = retry_with_backoff(
        call, max_attempts=2, sleep_fn=lambda _: None,
    )
    assert attempts == 2
    assert result is None
