#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_phase_timings.py
# Contenido: F8 — tests offline de PhaseTimings.
"""Tests offline de la instrumentación PhaseTimings."""

from __future__ import annotations

import pytest

from tfm_orchestrator.phase_timings import PhaseTimings


# ---------------------------------------------------------------------------
# Basic mark/end
# ---------------------------------------------------------------------------


def test_empty_snapshot():
    pt = PhaseTimings()
    snap = pt.snapshot()
    assert snap["phases"] == {}
    assert snap["phase_order"] == []
    assert snap["total_duration_sec"] is None
    assert snap["phases_completed"] == 0
    assert snap["phases_failed"] == 0


def test_mark_start_then_end():
    pt = PhaseTimings()
    pt.mark_start("APPROACH", clock_now=10.0)
    pt.mark_end("APPROACH", clock_now=11.5, success=True)
    snap = pt.snapshot()
    assert "APPROACH" in snap["phases"]
    assert snap["phases"]["APPROACH"]["duration_sec"] == pytest.approx(1.5)
    assert snap["phases"]["APPROACH"]["success"] is True
    assert snap["phases_completed"] == 1
    assert snap["phases_failed"] == 0


def test_mark_end_with_detail():
    pt = PhaseTimings()
    pt.mark_start("GRASP", clock_now=0.0)
    pt.mark_end("GRASP", clock_now=2.0, success=False, detail="grasp_close:timeout")
    snap = pt.snapshot()
    assert snap["phases"]["GRASP"]["success"] is False
    assert snap["phases"]["GRASP"]["detail"] == "grasp_close:timeout"
    assert snap["phases_failed"] == 1


def test_phase_order_preserved():
    pt = PhaseTimings()
    pt.mark_start("SELECT_OBJECT", clock_now=0.0)
    pt.mark_start("APPROACH", clock_now=1.0)
    pt.mark_start("GRASP", clock_now=2.0)
    snap = pt.snapshot()
    assert snap["phase_order"] == ["SELECT_OBJECT", "APPROACH", "GRASP"]


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------


def test_mark_end_without_start_creates_entry():
    pt = PhaseTimings()
    pt.mark_end("PHASE_X", clock_now=5.0, success=True)
    snap = pt.snapshot()
    assert "PHASE_X" in snap["phases"]
    assert snap["phases"]["PHASE_X"]["start_mono"] is None
    assert snap["phases"]["PHASE_X"]["end_mono"] == 5.0
    assert snap["phases"]["PHASE_X"]["duration_sec"] is None


def test_negative_duration_clamped_to_zero():
    """Si end < start (clock skew), duration se clamp a 0."""
    pt = PhaseTimings()
    pt.mark_start("X", clock_now=10.0)
    pt.mark_end("X", clock_now=9.5, success=True)
    snap = pt.snapshot()
    assert snap["phases"]["X"]["duration_sec"] == pytest.approx(0.0)


def test_re_marking_overwrites_previous():
    """Llamar mark_start dos veces sobrescribe."""
    pt = PhaseTimings()
    pt.mark_start("X", clock_now=10.0)
    pt.mark_start("X", clock_now=20.0)  # retry
    pt.mark_end("X", clock_now=21.0, success=True)
    snap = pt.snapshot()
    assert snap["phases"]["X"]["start_mono"] == 20.0
    assert snap["phases"]["X"]["duration_sec"] == pytest.approx(1.0)


def test_phase_order_only_appended_once():
    """Re-marcar la misma fase no duplica el order."""
    pt = PhaseTimings()
    pt.mark_start("X", clock_now=0.0)
    pt.mark_start("X", clock_now=10.0)
    pt.mark_end("X", clock_now=11.0)
    assert pt.snapshot()["phase_order"] == ["X"]


# ---------------------------------------------------------------------------
# Session-level timing
# ---------------------------------------------------------------------------


def test_session_total_duration_explicit():
    pt = PhaseTimings()
    pt.mark_session_start(clock_now=0.0)
    pt.mark_start("X", clock_now=1.0)
    pt.mark_end("X", clock_now=2.0, success=True)
    pt.mark_session_end(clock_now=3.0)
    assert pt.total_duration_sec() == pytest.approx(3.0)


def test_session_total_duration_inferred_from_phases():
    """Sin mark_session_*, infiere desde min(start)/max(end)."""
    pt = PhaseTimings()
    pt.mark_start("A", clock_now=10.0)
    pt.mark_end("A", clock_now=11.0, success=True)
    pt.mark_start("B", clock_now=12.0)
    pt.mark_end("B", clock_now=15.0, success=True)
    assert pt.total_duration_sec() == pytest.approx(5.0)


def test_session_total_duration_none_when_empty():
    pt = PhaseTimings()
    assert pt.total_duration_sec() is None


def test_session_total_duration_partial_data_returns_none():
    """Sólo start, sin end → None."""
    pt = PhaseTimings()
    pt.mark_start("X", clock_now=10.0)
    assert pt.total_duration_sec() is None


# ---------------------------------------------------------------------------
# Snapshot completeness
# ---------------------------------------------------------------------------


def test_snapshot_counts_completed_and_failed():
    pt = PhaseTimings()
    pt.mark_start("A", clock_now=0.0)
    pt.mark_end("A", clock_now=1.0, success=True)
    pt.mark_start("B", clock_now=1.0)
    pt.mark_end("B", clock_now=2.0, success=False)
    pt.mark_start("C", clock_now=2.0)
    pt.mark_end("C", clock_now=3.0, success=True)
    snap = pt.snapshot()
    assert snap["phases_completed"] == 2
    assert snap["phases_failed"] == 1


def test_snapshot_serializable_to_json():
    """Snapshot debe ser JSON-serializable (sólo tipos primitivos)."""
    import json
    pt = PhaseTimings()
    pt.mark_session_start(clock_now=0.0)
    pt.mark_start("A", clock_now=0.5, detail="hello")
    pt.mark_end("A", clock_now=1.0, success=True)
    pt.mark_session_end(clock_now=2.0)
    snap = pt.snapshot()
    serialized = json.dumps(snap)
    decoded = json.loads(serialized)
    assert decoded["phases"]["A"]["duration_sec"] == 0.5
    assert decoded["phases"]["A"]["detail"] == "hello"


# ---------------------------------------------------------------------------
# Critical path
# ---------------------------------------------------------------------------


def test_critical_path_preserves_order():
    pt = PhaseTimings()
    pt.mark_start("APPROACH", clock_now=0.0)
    pt.mark_end("APPROACH", clock_now=2.0, success=True)
    pt.mark_start("GRASP", clock_now=2.0)
    pt.mark_end("GRASP", clock_now=2.5, success=True)
    pt.mark_start("LIFT", clock_now=2.5)
    pt.mark_end("LIFT", clock_now=3.0, success=True)
    cp = pt.critical_path()
    assert [p["phase"] for p in cp] == ["APPROACH", "GRASP", "LIFT"]
    assert cp[0]["duration_sec"] == pytest.approx(2.0)
    assert cp[1]["duration_sec"] == pytest.approx(0.5)


def test_critical_path_empty():
    pt = PhaseTimings()
    assert pt.critical_path() == []


def test_critical_path_includes_failures():
    pt = PhaseTimings()
    pt.mark_start("X", clock_now=0.0)
    pt.mark_end("X", clock_now=1.0, success=False)
    cp = pt.critical_path()
    assert cp[0]["success"] is False
