#!/usr/bin/env python3
"""F8 audit-v4: tests offline para cycle_timing_aggregator."""
from __future__ import annotations

import json
from typing import List

import pytest

from ur5_tools.cycle_timing_aggregator import (
    aggregate_cycle_totals,
    aggregate_phase_samples,
    render_baseline_markdown,
    render_json,
)
from ur5_tools.cycle_timing_analyzer import CycleTiming, PhaseTiming


def _phase(name: str, dur: float, success: bool = True) -> PhaseTiming:
    return PhaseTiming(
        phase=name, start_sec=0.0, end_sec=dur, duration_sec=dur, success=success
    )


def _cycle(phases: List[PhaseTiming], total: float = None, success: bool = True) -> CycleTiming:
    if total is None:
        total = sum(p.duration_sec for p in phases if p.duration_sec is not None)
    return CycleTiming(
        phases=phases, total_duration_sec=total, success=success, n_phases=len(phases)
    )


def test_aggregate_phase_samples_basic():
    cycles = [
        _cycle([_phase("APPROACH", 5.0), _phase("GRASP", 2.0)]),
        _cycle([_phase("APPROACH", 7.0), _phase("GRASP", 3.0)]),
    ]
    s = aggregate_phase_samples(cycles)
    assert s == {"APPROACH": [5.0, 7.0], "GRASP": [2.0, 3.0]}


def test_aggregate_phase_samples_skips_failures():
    cycles = [
        _cycle([_phase("APPROACH", 5.0, success=True),
                _phase("GRASP", 2.0, success=False)]),
    ]
    s = aggregate_phase_samples(cycles)
    assert s == {"APPROACH": [5.0]}


def test_aggregate_phase_samples_skips_none_duration():
    p_none = PhaseTiming(
        phase="APPROACH", start_sec=0.0, end_sec=None, duration_sec=None, success=True
    )
    cycles = [_cycle([p_none])]
    s = aggregate_phase_samples(cycles)
    assert s == {}


def test_aggregate_cycle_totals_skips_failures():
    cycles = [
        _cycle([_phase("X", 1.0)], total=10.0, success=True),
        _cycle([_phase("X", 1.0)], total=20.0, success=False),
        _cycle([_phase("X", 1.0)], total=15.0, success=True),
    ]
    assert aggregate_cycle_totals(cycles) == [10.0, 15.0]


def test_render_baseline_markdown_includes_percentiles():
    cycles = [
        _cycle([_phase("APPROACH", 4.0)], total=4.0),
        _cycle([_phase("APPROACH", 6.0)], total=6.0),
        _cycle([_phase("APPROACH", 5.0)], total=5.0),
    ]
    md = render_baseline_markdown(cycles, title="Test")
    assert "# Test" in md
    assert "Cycle total duration" in md
    assert "Per-phase duration" in md
    assert "APPROACH" in md
    assert "p50" in md
    assert "p95" in md
    assert "p99" in md


def test_render_baseline_markdown_handles_empty():
    md = render_baseline_markdown([], title="Empty")
    assert "# Empty" in md
    assert "Cycles parseados: **0**" in md


def test_render_json_structure():
    cycles = [_cycle([_phase("APPROACH", 4.0), _phase("GRASP", 2.0)])]
    out = render_json(cycles)
    data = json.loads(out)
    assert data["n_cycles"] == 1
    assert data["n_success"] == 1
    assert "APPROACH" in data["per_phase"]
    assert data["per_phase"]["APPROACH"]["n"] == 1
    assert data["cycle_total_sec"]["n"] == 1
