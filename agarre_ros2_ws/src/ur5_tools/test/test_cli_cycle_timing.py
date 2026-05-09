#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_cli_cycle_timing.py
"""F8-step1 (2026-05-08) — Tests offline para cli_cycle_timing."""

from __future__ import annotations

import json
from pathlib import Path
from typing import List

import pytest

from ur5_tools.cli_cycle_timing import (
    cycle_to_json,
    find_bottleneck,
    main,
)
from ur5_tools.cycle_timing_analyzer import (
    CycleTiming,
    PhaseTiming,
    analyze_cycle,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def fake_log_lines() -> List[str]:
    """Log típico con 3 fases: HOME_INITIAL/APPROACH/TRANSPORT (TRANSPORT bottleneck)."""
    return [
        "[INFO] [1715175000.0] [orchestrator]: [ORCHESTRATOR_LC][HOME_INITIAL] start",
        "[INFO] [1715175005.0] [orchestrator]: [ORCHESTRATOR_LC][HOME_INITIAL] end success=true",
        "[INFO] [1715175005.0] [orchestrator]: [ORCHESTRATOR_LC][APPROACH] start",
        "[INFO] [1715175040.0] [orchestrator]: [ORCHESTRATOR_LC][APPROACH] end success=true",
        "[INFO] [1715175040.0] [orchestrator]: [ORCHESTRATOR_LC][TRANSPORT] start",
        "[INFO] [1715175125.0] [orchestrator]: [ORCHESTRATOR_LC][TRANSPORT] end success=true",
        "[INFO] [1715175126.0] [orchestrator]: [ORCHESTRATOR_LC] result success=True",
    ]


# ---------------------------------------------------------------------------
# cycle_to_json
# ---------------------------------------------------------------------------


def test_cycle_to_json_empty():
    cycle = CycleTiming(phases=[], total_duration_sec=None, success=False, n_phases=0)
    js = cycle_to_json(cycle)
    parsed = json.loads(js)
    assert parsed["success"] is False
    assert parsed["n_phases"] == 0
    assert parsed["phases"] == []


def test_cycle_to_json_full(fake_log_lines):
    cycle = analyze_cycle(fake_log_lines)
    js = cycle_to_json(cycle)
    parsed = json.loads(js)
    assert parsed["success"] is True
    assert parsed["n_phases"] == 3
    assert parsed["total_duration_sec"] == pytest.approx(125.0)
    # Phases ordered, TRANSPORT last
    assert parsed["phases"][-1]["phase"] == "TRANSPORT"
    assert parsed["phases"][-1]["duration_sec"] == pytest.approx(85.0)


# ---------------------------------------------------------------------------
# find_bottleneck
# ---------------------------------------------------------------------------


def test_find_bottleneck_empty():
    cycle = CycleTiming(phases=[], total_duration_sec=None, success=False, n_phases=0)
    assert "no phases" in find_bottleneck(cycle)


def test_find_bottleneck_with_phases(fake_log_lines):
    cycle = analyze_cycle(fake_log_lines)
    result = find_bottleneck(cycle)
    assert "TRANSPORT" in result
    assert "85.0s" in result
    assert "%" in result


def test_find_bottleneck_no_completed_phases():
    """Si todas las fases están sin end, no hay bottleneck calculable."""
    phase = PhaseTiming(
        phase="STUCK", start_sec=0.0, end_sec=None,
        duration_sec=None, success=None,
    )
    cycle = CycleTiming(phases=[phase], total_duration_sec=None, success=False, n_phases=1)
    assert "no completed" in find_bottleneck(cycle)


def test_find_bottleneck_picks_max_duration():
    phases = [
        PhaseTiming("HOME", 0.0, 5.0, 5.0, True),
        PhaseTiming("APPROACH", 5.0, 30.0, 25.0, True),
        PhaseTiming("TRANSPORT", 30.0, 100.0, 70.0, True),
        PhaseTiming("RELEASE", 100.0, 110.0, 10.0, True),
    ]
    cycle = CycleTiming(phases=phases, total_duration_sec=110.0, success=True, n_phases=4)
    result = find_bottleneck(cycle)
    assert "TRANSPORT" in result
    assert "70.0s" in result
    # 70/110 ≈ 63.6%
    assert "63.6%" in result or "63.7%" in result


# ---------------------------------------------------------------------------
# main (CLI)
# ---------------------------------------------------------------------------


def test_main_missing_file_returns_1(capsys):
    rc = main(["/nonexistent/path/log.log"])
    assert rc == 1
    captured = capsys.readouterr()
    assert "no existe" in captured.err


def test_main_summary_default(tmp_path: Path, fake_log_lines, capsys):
    log = tmp_path / "test.log"
    log.write_text("\n".join(fake_log_lines), encoding="utf-8")
    rc = main([str(log)])
    assert rc == 0
    captured = capsys.readouterr()
    assert "TRANSPORT" in captured.out
    assert "success=True" in captured.out


def test_main_json(tmp_path: Path, fake_log_lines, capsys):
    log = tmp_path / "test.log"
    log.write_text("\n".join(fake_log_lines), encoding="utf-8")
    rc = main([str(log), "--json"])
    assert rc == 0
    captured = capsys.readouterr()
    parsed = json.loads(captured.out)
    assert parsed["n_phases"] == 3
    assert parsed["success"] is True


def test_main_bottleneck_flag(tmp_path: Path, fake_log_lines, capsys):
    log = tmp_path / "test.log"
    log.write_text("\n".join(fake_log_lines), encoding="utf-8")
    rc = main([str(log), "--bottleneck"])
    assert rc == 0
    captured = capsys.readouterr()
    assert "BOTTLENECK" in captured.out
    assert "TRANSPORT" in captured.out
