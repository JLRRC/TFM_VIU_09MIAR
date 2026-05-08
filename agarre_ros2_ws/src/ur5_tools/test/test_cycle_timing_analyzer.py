#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_cycle_timing_analyzer.py
"""F8 / #20 (2026-05-08) — Tests offline para cycle_timing_analyzer."""

from __future__ import annotations

import pytest

from ur5_tools.cycle_timing_analyzer import (
    CycleTiming,
    PhaseTiming,
    analyze_cycle,
    format_cycle_summary,
    parse_phase_events,
)


# ---------------------------------------------------------------------------
# parse_phase_events
# ---------------------------------------------------------------------------


def test_parse_phase_events_empty():
    starts, ends = parse_phase_events([])
    assert starts == {}
    assert ends == {}


def test_parse_phase_events_single_start_no_end():
    lines = [
        "[INFO] [1715175123.000000] [orchestrator]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
    ]
    starts, ends = parse_phase_events(lines)
    assert starts == {"HOME_INITIAL": pytest.approx(1715175123.0)}
    assert ends == {}


def test_parse_phase_events_full_phase():
    lines = [
        "[INFO] [1715175100.000000] [orchestrator]: [ORCHESTRATOR_LC][APPROACH] start",
        "[INFO] [1715175125.500000] [orchestrator]: [ORCHESTRATOR_LC][APPROACH] end success=true",
    ]
    starts, ends = parse_phase_events(lines)
    assert starts["APPROACH"] == pytest.approx(1715175100.0)
    assert ends["APPROACH"][0] == pytest.approx(1715175125.5)
    assert ends["APPROACH"][1] is True


def test_parse_phase_events_failed_phase():
    lines = [
        "[INFO] [1715175100.000000] [...]: [ORCHESTRATOR_LC][TRANSPORT] start",
        "[INFO] [1715175340.000000] [...]: [ORCHESTRATOR_LC][TRANSPORT] end success=false",
    ]
    _starts, ends = parse_phase_events(lines)
    assert ends["TRANSPORT"][1] is False


def test_parse_phase_events_takes_first_start_only():
    """Si hay 2 starts del mismo phase, se conserva el primero."""
    lines = [
        "[INFO] [1715175100.000000] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
        "[INFO] [1715175200.000000] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
    ]
    starts, _ends = parse_phase_events(lines)
    assert starts["HOME_INITIAL"] == pytest.approx(1715175100.0)


def test_parse_phase_events_multiple_phases():
    lines = [
        "[INFO] [1715175000.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
        "[INFO] [1715175005.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] end success=true",
        "[INFO] [1715175005.0] [...]: [ORCHESTRATOR_LC][APPROACH] start",
        "[INFO] [1715175030.0] [...]: [ORCHESTRATOR_LC][APPROACH] end success=true",
    ]
    starts, ends = parse_phase_events(lines)
    assert len(starts) == 2
    assert len(ends) == 2


# ---------------------------------------------------------------------------
# analyze_cycle
# ---------------------------------------------------------------------------


def test_analyze_cycle_empty_log():
    cycle = analyze_cycle([])
    assert cycle.n_phases == 0
    assert cycle.success is False
    assert cycle.total_duration_sec is None


def test_analyze_cycle_legacy_pass_marker():
    lines = [
        "[INFO] [1715175000.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
        "[INFO] [1715175005.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] end success=true",
        "[INFO] [1715175170.0] [...]: SECUENCIA COMPLETADA EXITOSAMENTE",
    ]
    cycle = analyze_cycle(lines)
    assert cycle.success is True
    assert cycle.n_phases == 1
    assert cycle.total_duration_sec is not None


def test_analyze_cycle_orchestrator_pass_marker():
    lines = [
        "[INFO] [1715175000.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
        "[INFO] [1715175005.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] end success=true",
        "[INFO] [1715175170.0] [...]: [ORCHESTRATOR_LC] result success=True",
    ]
    cycle = analyze_cycle(lines)
    assert cycle.success is True


def test_analyze_cycle_full_pipeline():
    """Caso real: 7 fases, total ~175s (referencia run 2026-05-08)."""
    lines = [
        "[INFO] [1715175000.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
        "[INFO] [1715175005.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] end success=true",
        "[INFO] [1715175005.0] [...]: [ORCHESTRATOR_LC][APPROACH] start",
        "[INFO] [1715175040.0] [...]: [ORCHESTRATOR_LC][APPROACH] end success=true",
        "[INFO] [1715175040.0] [...]: [ORCHESTRATOR_LC][GRASP_DOWN] start",
        "[INFO] [1715175055.0] [...]: [ORCHESTRATOR_LC][GRASP_DOWN] end success=true",
        "[INFO] [1715175055.0] [...]: [ORCHESTRATOR_LC][GRASP] start",
        "[INFO] [1715175062.0] [...]: [ORCHESTRATOR_LC][GRASP] end success=true",
        "[INFO] [1715175062.0] [...]: [ORCHESTRATOR_LC][LIFT] start",
        "[INFO] [1715175080.0] [...]: [ORCHESTRATOR_LC][LIFT] end success=true",
        "[INFO] [1715175080.0] [...]: [ORCHESTRATOR_LC][TRANSPORT] start",
        "[INFO] [1715175165.0] [...]: [ORCHESTRATOR_LC][TRANSPORT] end success=true",
        "[INFO] [1715175165.0] [...]: [ORCHESTRATOR_LC][RELEASE] start",
        "[INFO] [1715175175.0] [...]: [ORCHESTRATOR_LC][RELEASE] end success=true",
        "[INFO] [1715175175.0] [...]: [ORCHESTRATOR_LC] result success=True",
    ]
    cycle = analyze_cycle(lines)
    assert cycle.success is True
    assert cycle.n_phases == 7
    assert cycle.total_duration_sec == pytest.approx(175.0)
    # Verify TRANSPORT es el bottleneck (~85s)
    transport = next(p for p in cycle.phases if p.phase == "TRANSPORT")
    assert transport.duration_sec == pytest.approx(85.0)


def test_analyze_cycle_phases_sorted_by_start():
    lines = [
        "[INFO] [1715175030.0] [...]: [ORCHESTRATOR_LC][APPROACH] start",
        "[INFO] [1715175040.0] [...]: [ORCHESTRATOR_LC][APPROACH] end success=true",
        "[INFO] [1715175000.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
        "[INFO] [1715175005.0] [...]: [ORCHESTRATOR_LC][HOME_INITIAL] end success=true",
    ]
    cycle = analyze_cycle(lines)
    assert cycle.phases[0].phase == "HOME_INITIAL"
    assert cycle.phases[1].phase == "APPROACH"


# ---------------------------------------------------------------------------
# format_cycle_summary
# ---------------------------------------------------------------------------


def test_format_summary_empty():
    cycle = CycleTiming(phases=[], total_duration_sec=None, success=False, n_phases=0)
    assert "no phases detected" in format_cycle_summary(cycle)


def test_format_summary_basic():
    phase = PhaseTiming(
        phase="HOME_INITIAL", start_sec=0.0, end_sec=5.0,
        duration_sec=5.0, success=True,
    )
    cycle = CycleTiming(phases=[phase], total_duration_sec=5.0, success=True, n_phases=1)
    summary = format_cycle_summary(cycle)
    assert "HOME_INITIAL" in summary
    assert "5.0s" in summary
    assert "OK" in summary
    assert "success=True" in summary


def test_format_summary_failed_phase():
    phase = PhaseTiming(
        phase="TRANSPORT", start_sec=0.0, end_sec=240.0,
        duration_sec=240.0, success=False,
    )
    cycle = CycleTiming(phases=[phase], total_duration_sec=240.0, success=False, n_phases=1)
    summary = format_cycle_summary(cycle)
    assert "TRANSPORT" in summary
    assert "FAIL" in summary


def test_format_summary_incomplete_phase():
    phase = PhaseTiming(
        phase="LIFT", start_sec=0.0, end_sec=None,
        duration_sec=None, success=None,
    )
    cycle = CycleTiming(phases=[phase], total_duration_sec=None, success=False, n_phases=1)
    summary = format_cycle_summary(cycle)
    assert "incomplete" in summary
