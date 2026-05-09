#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_perf_helpers.py
# Contenido: F19 (2026-05-01) — tests offline de helpers de performance.
"""Tests offline de los helpers F19 ``perf_helpers``.

F19 cierra la fase de optimización con helpers analíticos para
medir antes/después de un cambio sin necesidad de ROS vivo.
"""

from __future__ import annotations

import pytest

from ur5_tools.perf_helpers import (
    classify_topic_health,
    compute_percentiles,
    detect_periodic_oscillation,
    summarize_performance_run,
)


# ---------------------------------------------------------------------------
# compute_percentiles
# ---------------------------------------------------------------------------


def test_percentiles_basic():
    out = compute_percentiles(list(range(1, 101)))  # 1..100
    # Para 100 valores 1..100, idx = round(p * 99). p50 cae en ~50 (50/51).
    assert out["p50"] in (50, 51)
    assert out["p95"] in (95, 96)
    assert out["p99"] in (99, 100)


def test_percentiles_single_value():
    out = compute_percentiles([42.0])
    assert out["p50"] == 42.0
    assert out["p95"] == 42.0
    assert out["p99"] == 42.0


def test_percentiles_empty_returns_none():
    assert compute_percentiles([]) is None


def test_percentiles_filters_invalid():
    out = compute_percentiles([1.0, "foo", 3.0, None, 5.0])
    # solo 1, 3, 5 son válidos → p50=3
    assert out["p50"] == 3.0


def test_percentiles_custom_set():
    out = compute_percentiles(
        list(range(1, 11)), percentiles=(0.10, 0.50, 0.90)
    )
    assert "p10" in out
    assert "p50" in out
    assert "p90" in out


# ---------------------------------------------------------------------------
# detect_periodic_oscillation
# ---------------------------------------------------------------------------


def test_periodic_within_tolerance_returns_false():
    """Período 1.0s con tol 0.02s y muestras estables → no oscila."""
    samples = [0.0, 1.0, 2.0, 3.0, 4.0]
    assert detect_periodic_oscillation(
        samples, expected_period_sec=1.0, tol_sec=0.02
    ) is False


def test_periodic_systematic_oscillation_returns_true():
    """Período esperado 1s pero observado 0.5/1.5 alternando → oscila."""
    samples = [0.0, 0.5, 2.0, 2.5, 4.0]  # diffs = 0.5, 1.5, 0.5, 1.5
    assert detect_periodic_oscillation(
        samples, expected_period_sec=1.0, tol_sec=0.02
    ) is True


def test_periodic_too_few_samples_returns_false():
    assert detect_periodic_oscillation(
        [1.0], expected_period_sec=1.0
    ) is False
    assert detect_periodic_oscillation(
        [], expected_period_sec=1.0
    ) is False


def test_periodic_invalid_period_returns_false():
    samples = [0.0, 1.0, 2.0]
    assert detect_periodic_oscillation(samples, expected_period_sec=0.0) is False
    assert detect_periodic_oscillation(samples, expected_period_sec=-1.0) is False
    assert detect_periodic_oscillation(samples, expected_period_sec="foo") is False


# ---------------------------------------------------------------------------
# classify_topic_health
# ---------------------------------------------------------------------------


def test_health_ok_in_range():
    assert classify_topic_health(10.0, expected_hz=10.0) == "ok"
    # tol 30% por defecto → 7-13 Hz aceptables
    assert classify_topic_health(7.5, expected_hz=10.0) == "ok"
    assert classify_topic_health(12.5, expected_hz=10.0) == "ok"


def test_health_slow():
    assert classify_topic_health(5.0, expected_hz=10.0) == "slow"


def test_health_fast():
    assert classify_topic_health(20.0, expected_hz=10.0) == "fast"


def test_health_missing():
    assert classify_topic_health(None, expected_hz=10.0) == "missing"
    assert classify_topic_health(0.0, expected_hz=10.0) == "missing"
    assert classify_topic_health(-1.0, expected_hz=10.0) == "missing"


def test_health_zero_expected_returns_ok():
    """Si no hay expectativa concreta, cualquier rate positivo es ok."""
    assert classify_topic_health(5.0, expected_hz=0.0) == "ok"


def test_health_custom_tolerance():
    """tol 5% → fuera de [9.5, 10.5] es slow/fast."""
    assert classify_topic_health(9.0, expected_hz=10.0, tol_pct=0.05) == "slow"
    assert classify_topic_health(11.0, expected_hz=10.0, tol_pct=0.05) == "fast"


def test_health_invalid_rate_is_missing():
    assert classify_topic_health("foo", expected_hz=10.0) == "missing"


# ---------------------------------------------------------------------------
# summarize_performance_run
# ---------------------------------------------------------------------------


def test_summary_basic():
    out = summarize_performance_run({
        "latency_pick": [0.1, 0.2, 0.3, 0.4, 0.5],
        "latency_release": [1.0, 1.5, 2.0],
    })
    assert out["latency_pick"]["n"] == 5
    assert out["latency_pick"]["mean"] == pytest.approx(0.3)
    assert out["latency_pick"]["min"] == pytest.approx(0.1)
    assert out["latency_pick"]["max"] == pytest.approx(0.5)
    assert "p50" in out["latency_pick"]
    assert out["latency_release"]["n"] == 3


def test_summary_empty_metric():
    out = summarize_performance_run({"x": []})
    assert out["x"] == {"n": 0}


def test_summary_filters_invalid_samples():
    out = summarize_performance_run({"y": [1.0, "foo", 3.0]})
    assert out["y"]["n"] == 2
    assert out["y"]["mean"] == pytest.approx(2.0)


def test_summary_empty_input():
    assert summarize_performance_run({}) == {}
