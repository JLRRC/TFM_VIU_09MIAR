#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_evidence_telemetry_f18.py
# Contenido: F18 (2026-05-01) — tests offline de telemetría/observabilidad extendida.
"""Tests offline de los helpers F18 extendidos en ``evidence_helpers``.

F18 añade tres helpers puros para enriquecer ``metrics.json`` y
producir reportes Markdown defendibles académicamente:

* ``compute_inter_event_latencies`` — pair-wise latency entre
  ``from_kind`` → siguiente ``to_kind``.
* ``compute_event_rates`` — tasa global eventos/segundo por kind.
* ``generate_latency_report_md`` — render Markdown del agregado.

Tests sin rclpy ni ROS, sólo el módulo puro.
"""

from __future__ import annotations

import pytest

from ur5_tools.evidence_helpers import (
    compute_event_rates,
    compute_inter_event_latencies,
    generate_latency_report_md,
)


def _ev(kind: str, ts_mono: float, **data) -> dict:
    return {"kind": kind, "ts_mono": ts_mono, "data": data}


# ---------------------------------------------------------------------------
# compute_inter_event_latencies
# ---------------------------------------------------------------------------


def test_inter_event_latencies_basic():
    events = [
        _ev("system_state", 1.0),
        _ev("grasp_result", 1.5),  # 0.5s diff
        _ev("system_state", 2.0),
        _ev("grasp_result", 2.8),  # 0.8s diff
    ]
    result = compute_inter_event_latencies(
        events, from_kind="system_state", to_kind="grasp_result"
    )
    assert result["samples"] == 2
    assert result["mean_sec"] == pytest.approx(0.65, abs=1e-3)
    assert result["min_sec"] == pytest.approx(0.5)
    assert result["max_sec"] == pytest.approx(0.8)
    assert result["unmatched_from"] == 0
    assert result["from_kind"] == "system_state"
    assert result["to_kind"] == "grasp_result"


def test_inter_event_latencies_no_match():
    events = [_ev("system_state", 1.0), _ev("system_state", 2.0)]
    result = compute_inter_event_latencies(
        events, from_kind="system_state", to_kind="grasp_result"
    )
    assert result["samples"] == 0
    assert result["mean_sec"] is None
    assert result["min_sec"] is None
    assert result["max_sec"] is None
    assert result["p95_sec"] is None
    assert result["unmatched_from"] >= 1


def test_inter_event_latencies_unmatched_chain():
    """Si llegan 2 from seguidos, el primero queda unmatched."""
    events = [
        _ev("system_state", 1.0),
        _ev("system_state", 2.0),
        _ev("grasp_result", 2.5),
    ]
    result = compute_inter_event_latencies(
        events, from_kind="system_state", to_kind="grasp_result"
    )
    assert result["samples"] == 1
    assert result["mean_sec"] == pytest.approx(0.5)
    assert result["unmatched_from"] == 1


def test_inter_event_latencies_negative_diff_skipped():
    events = [
        _ev("system_state", 5.0),
        _ev("grasp_result", 4.0),  # negative diff → skip
    ]
    result = compute_inter_event_latencies(
        events, from_kind="system_state", to_kind="grasp_result"
    )
    assert result["samples"] == 0


def test_inter_event_latencies_p95_with_many_samples():
    events = []
    for i, lat in enumerate([0.1 * (j + 1) for j in range(20)]):
        events.append(_ev("system_state", float(i) * 10.0))
        events.append(_ev("grasp_result", float(i) * 10.0 + lat))
    result = compute_inter_event_latencies(
        events, from_kind="system_state", to_kind="grasp_result"
    )
    assert result["samples"] == 20
    assert result["min_sec"] == pytest.approx(0.1)
    assert result["max_sec"] == pytest.approx(2.0)
    assert result["p95_sec"] >= 1.8


def test_inter_event_latencies_invalid_events_filtered():
    events = [
        None,
        {"kind": "system_state"},  # sin ts_mono → ignorado
        _ev("system_state", 1.0),
        _ev("grasp_result", 1.5),
    ]
    result = compute_inter_event_latencies(
        events, from_kind="system_state", to_kind="grasp_result"
    )
    assert result["samples"] == 1


# ---------------------------------------------------------------------------
# compute_event_rates
# ---------------------------------------------------------------------------


def test_event_rates_basic():
    events = [
        _ev("system_diag", 0.0),
        _ev("system_diag", 1.0),
        _ev("system_diag", 2.0),
        _ev("grasp_result", 1.5),
    ]
    rates = compute_event_rates(events)
    assert "system_diag" in rates
    assert rates["system_diag"]["count"] == 3
    assert rates["system_diag"]["span_sec"] == pytest.approx(2.0)
    assert rates["system_diag"]["rate_hz"] == pytest.approx(1.5)
    assert rates["grasp_result"]["count"] == 1
    assert rates["grasp_result"]["rate_hz"] is None


def test_event_rates_empty():
    assert compute_event_rates([]) == {}


def test_event_rates_filters_invalid():
    events = [
        None,
        "string",
        {"kind": "ok", "ts_mono": 1.0},
        {"kind": "ok", "ts_mono": 2.0},
        {"kind": "no_ts"},
    ]
    rates = compute_event_rates(events)
    assert rates["ok"]["count"] == 2
    # 2 eventos en span 1.0s → 2.0 Hz.
    assert rates["ok"]["rate_hz"] == pytest.approx(2.0)
    assert rates["no_ts"]["count"] == 0
    assert rates["no_ts"]["rate_hz"] is None


# ---------------------------------------------------------------------------
# generate_latency_report_md
# ---------------------------------------------------------------------------


def test_latency_report_renders_basic():
    metrics = {
        "total_events": 100,
        "duration_sec": 60.5,
        "grasp_success": 8,
        "grasp_failure": 2,
        "grasp_success_rate": 0.8,
        "by_kind": {"grasp_result": 10, "system_diag": 90},
        "event_rates": {
            "grasp_result": {
                "count": 10,
                "span_sec": 50.0,
                "rate_hz": 0.2,
            },
        },
        "inter_event_latencies": [
            {
                "from_kind": "system_state",
                "to_kind": "grasp_result",
                "samples": 5,
                "mean_sec": 0.5,
                "min_sec": 0.1,
                "max_sec": 1.2,
                "p95_sec": 1.0,
            }
        ],
    }
    md = generate_latency_report_md(metrics=metrics)
    assert md.startswith("# Reporte de telemetría F18")
    assert "100" in md
    assert "80.0%" in md
    assert "system_state" in md
    assert "0.500" in md
    assert "0.20" in md
    assert "| Kind | Count |" in md
    assert "| `grasp_result` | 10 |" in md


def test_latency_report_handles_missing_optionals():
    metrics = {
        "total_events": 5,
        "duration_sec": None,
        "grasp_success": 0,
        "grasp_failure": 0,
        "grasp_success_rate": None,
        "by_kind": {},
    }
    md = generate_latency_report_md(metrics=metrics)
    assert "n/d" in md
    assert "## Latencias entre eventos" not in md
    assert "## Tasa por kind" not in md


def test_latency_report_custom_title():
    md = generate_latency_report_md(metrics={}, title="Sesión 7")
    assert md.startswith("# Sesión 7")
