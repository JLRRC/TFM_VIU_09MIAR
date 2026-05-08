#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_session_metrics.py
# Contenido: F4 — tests de compute_session_metrics + aggregate_phase_timings.
"""F4 — Tests de helpers puros del evidence_logger.

Cubre las funciones de agregación NO ya cubiertas por
``test_evidence_telemetry_f18.py``:

  * ``compute_session_metrics``: agregación de eventos JSONL de una sesión
    (events totales, by_kind, grasp success/failure, attach/detach, etc.).
  * ``aggregate_phase_timings``: agregación cross-session de phase timings.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List

import pytest

from ur5_tools.evidence_helpers import (
    aggregate_phase_timings,
    compute_session_metrics,
)


# ---------------------------------------------------------------------------
# compute_session_metrics
# ---------------------------------------------------------------------------


def test_compute_session_metrics_empty_input() -> None:
    m = compute_session_metrics([])
    assert m["total_events"] == 0
    assert m["by_kind"] == {}
    assert m["grasp_success"] == 0
    assert m["grasp_failure"] == 0
    assert m["grasp_success_rate"] is None
    assert m["attach_count"] == 0
    assert m["detach_count"] == 0
    assert m["objects_attached"] == []
    assert m["session_started_iso"] is None
    assert m["session_finished_iso"] is None
    assert m["duration_sec"] is None


def test_compute_session_metrics_counts_by_kind() -> None:
    events = [
        {"kind": "session_started", "ts_iso": "2026-05-08T00:00:00Z", "ts_mono": 100.0, "data": {}},
        {"kind": "grasp_result", "ts_mono": 110.0, "data": {"success": True}},
        {"kind": "grasp_result", "ts_mono": 120.0, "data": {"success": False}},
        {"kind": "session_finished", "ts_iso": "2026-05-08T00:01:00Z", "ts_mono": 160.0, "data": {}},
    ]
    m = compute_session_metrics(events)
    assert m["total_events"] == 4
    assert m["by_kind"]["grasp_result"] == 2
    assert m["by_kind"]["session_started"] == 1
    assert m["by_kind"]["session_finished"] == 1


def test_compute_session_metrics_grasp_success_rate() -> None:
    events: List[Dict[str, Any]] = [
        {"kind": "grasp_result", "ts_mono": 1.0, "data": {"success": True}},
        {"kind": "grasp_result", "ts_mono": 2.0, "data": {"success": True}},
        {"kind": "grasp_result", "ts_mono": 3.0, "data": {"success": False}},
    ]
    m = compute_session_metrics(events)
    assert m["grasp_success"] == 2
    assert m["grasp_failure"] == 1
    assert m["grasp_success_rate"] == pytest.approx(2.0 / 3.0)


def test_compute_session_metrics_success_rate_none_when_no_grasp() -> None:
    events = [
        {"kind": "session_started", "ts_iso": "...", "ts_mono": 0.0, "data": {}},
    ]
    m = compute_session_metrics(events)
    assert m["grasp_success_rate"] is None


def test_compute_session_metrics_attach_detach_tracking() -> None:
    events = [
        {"kind": "gripper_state", "ts_mono": 1.0, "data": {"object": "box_red", "attached": True}},
        {"kind": "gripper_state", "ts_mono": 2.0, "data": {"object": "box_blue", "attached": True}},
        {"kind": "gripper_state", "ts_mono": 3.0, "data": {"object": "box_red", "attached": False}},
        {"kind": "gripper_state", "ts_mono": 4.0, "data": {"object": "box_red", "attached": True}},
    ]
    m = compute_session_metrics(events)
    assert m["attach_count"] == 3
    assert m["detach_count"] == 1
    # objects_attached: lista de nombres únicos en orden de primera aparición.
    assert m["objects_attached"] == ["box_red", "box_blue"]


def test_compute_session_metrics_duration() -> None:
    events = [
        {"kind": "x", "ts_mono": 100.5, "data": {}},
        {"kind": "x", "ts_mono": 105.0, "data": {}},
        {"kind": "x", "ts_mono": 110.7, "data": {}},
    ]
    m = compute_session_metrics(events)
    assert m["duration_sec"] == pytest.approx(10.2, abs=0.01)


def test_compute_session_metrics_duration_none_with_one_event() -> None:
    events = [{"kind": "x", "ts_mono": 100.0, "data": {}}]
    m = compute_session_metrics(events)
    assert m["duration_sec"] is None


def test_compute_session_metrics_skips_non_dict_events() -> None:
    events: List[Any] = [
        {"kind": "valid", "ts_mono": 1.0, "data": {}},
        "not a dict",
        12345,
        None,
    ]
    m = compute_session_metrics(events)
    assert m["total_events"] == 1


def test_compute_session_metrics_handles_missing_ts_mono() -> None:
    """Eventos sin ts_mono no rompen el cálculo de duration."""
    events = [
        {"kind": "x", "data": {}},  # sin ts_mono
        {"kind": "x", "ts_mono": 5.0, "data": {}},
        {"kind": "x", "ts_mono": "not_a_number", "data": {}},
    ]
    m = compute_session_metrics(events)
    # Solo 1 ts_mono válido → duration None (necesita >= 2).
    assert m["duration_sec"] is None


def test_compute_session_metrics_grasp_unknown_when_data_missing() -> None:
    events = [
        {"kind": "grasp_result", "ts_mono": 1.0, "data": {}},  # sin "success"
        {"kind": "grasp_result", "ts_mono": 2.0, "data": "not-a-dict"},
    ]
    m = compute_session_metrics(events)
    assert m["grasp_unknown"] == 2
    assert m["grasp_success"] == 0
    assert m["grasp_failure"] == 0


# ---------------------------------------------------------------------------
# aggregate_phase_timings
# ---------------------------------------------------------------------------


def test_aggregate_phase_timings_empty() -> None:
    result = aggregate_phase_timings([])
    # Empty input → no sessions, no phases.
    assert result["total_sessions"] == 0
    assert result["successful_sessions"] == 0
    assert result["per_phase"] == {}


def test_aggregate_phase_timings_basic_one_session() -> None:
    snapshot = {
        "phases": {
            "APPROACH": {"duration_sec": 2.0, "success": True},
            "GRASP": {"duration_sec": 1.5, "success": True},
        },
        "phase_order": ["APPROACH", "GRASP"],
        "success": True,
    }
    result = aggregate_phase_timings([snapshot])
    assert result["total_sessions"] == 1
    assert result["successful_sessions"] == 1
    assert "APPROACH" in result["per_phase"]
    approach = result["per_phase"]["APPROACH"]
    assert approach["samples"] == 1
    assert approach["mean_sec"] == pytest.approx(2.0)
    assert approach["min_sec"] == pytest.approx(2.0)
    assert approach["max_sec"] == pytest.approx(2.0)


def test_aggregate_phase_timings_multi_session() -> None:
    snapshots = [
        {
            "phases": {"APPROACH": {"duration_sec": 1.0, "success": True}},
            "phases_failed": 0,
        },
        {
            "phases": {"APPROACH": {"duration_sec": 2.0, "success": True}},
            "phases_failed": 0,
        },
        {
            "phases": {"APPROACH": {"duration_sec": 3.0, "success": False}},
            "phases_failed": 1,
        },
    ]
    result = aggregate_phase_timings(snapshots)
    assert result["total_sessions"] == 3
    assert result["successful_sessions"] == 2
    approach = result["per_phase"]["APPROACH"]
    assert approach["samples"] == 3
    assert approach["mean_sec"] == pytest.approx(2.0)
    assert approach["min_sec"] == pytest.approx(1.0)
    assert approach["max_sec"] == pytest.approx(3.0)
    assert approach["success_rate"] == pytest.approx(2.0 / 3.0)


def test_aggregate_phase_timings_skips_non_dict_input() -> None:
    """Inputs no-dict son ignorados — no incrementan total_sessions."""
    snapshots: List[Any] = [
        "not-a-dict",
        12345,
        None,
        # Valid one — debería contar.
        {
            "phases": {"X": {"duration_sec": 1.0, "success": True}},
            "phases_failed": 0,
        },
    ]
    result = aggregate_phase_timings(snapshots)
    assert result["total_sessions"] == 1
    assert result["per_phase"]["X"]["samples"] == 1


def test_aggregate_phase_timings_avg_total_duration() -> None:
    """avg_total_duration_sec se computa de los snapshots con total_duration_sec."""
    snapshots = [
        {"phases": {}, "phases_failed": 0, "total_duration_sec": 10.0},
        {"phases": {}, "phases_failed": 0, "total_duration_sec": 20.0},
        {"phases": {}, "phases_failed": 0},  # sin total_duration → ignorado
    ]
    result = aggregate_phase_timings(snapshots)
    assert result["avg_total_duration_sec"] == pytest.approx(15.0)


def test_aggregate_phase_timings_no_durations_returns_none() -> None:
    """Si ningún snapshot tiene total_duration_sec, avg es None."""
    snapshots = [
        {"phases": {}, "phases_failed": 0},
    ]
    result = aggregate_phase_timings(snapshots)
    assert result["avg_total_duration_sec"] is None
