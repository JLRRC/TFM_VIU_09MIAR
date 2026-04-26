#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_cycle_logger.py
"""Unit tests for cycle_logger.py — no ROS needed."""

import json
import time

import pytest

from ur5_tools.cycle_logger import CycleLogger, _next_cycle_id


# ---------------------------------------------------------------------------
# _next_cycle_id
# ---------------------------------------------------------------------------

def test_next_cycle_id_format():
    cid = _next_cycle_id()
    # YYYYMMDD_HHMMSS_NNN
    parts = cid.split("_")
    assert len(parts) == 3
    assert parts[0].isdigit() and len(parts[0]) == 8
    assert parts[1].isdigit() and len(parts[1]) == 6
    assert parts[2].isdigit() and len(parts[2]) == 3


def test_next_cycle_id_monotone():
    ids = [_next_cycle_id() for _ in range(5)]
    counters = [int(cid.split("_")[2]) for cid in ids]
    assert counters == sorted(counters)
    assert len(set(counters)) == 5


# ---------------------------------------------------------------------------
# begin_cycle / end_cycle basics
# ---------------------------------------------------------------------------

def test_begin_cycle_returns_id(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle(backend="DIRECTO")
    assert cid
    assert logger.cycle_id == cid
    assert logger.active


def test_end_cycle_writes_file(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle(backend="DIRECTO")
    path = logger.end_cycle(outcome="SUCCESS")
    assert path is not None
    assert (tmp_path / f"{cid}.json").exists()


def test_end_cycle_json_structure(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle(backend="MOVEIT")
    path = logger.end_cycle(outcome="FAILED")
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert doc["cycle_id"] == cid
    assert doc["backend"] == "MOVEIT"
    assert doc["outcome"] == "FAILED"
    assert "start_iso" in doc
    assert "end_iso" in doc
    assert isinstance(doc["duration_sec"], float)
    assert isinstance(doc["phases"], list)
    assert isinstance(doc["metrics"], dict)
    assert isinstance(doc["errors"], list)


def test_outcome_uppercased(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    path = logger.end_cycle(outcome="success")
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert doc["outcome"] == "SUCCESS"


def test_duration_positive(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    logger.begin_cycle()
    time.sleep(0.02)
    path = logger.end_cycle(outcome="SUCCESS")
    doc = json.loads((tmp_path / f"{path.rsplit('/', 1)[-1]}").read_text())
    assert doc["duration_sec"] >= 0.01


# ---------------------------------------------------------------------------
# Phases
# ---------------------------------------------------------------------------

def test_phases_recorded(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    logger.begin_phase("APPROACH_COARSE")
    logger.end_phase(ok=True)
    logger.begin_phase("GRASP_DOWN")
    logger.end_phase(ok=False, note="timeout")
    path = logger.end_cycle(outcome="FAILED")
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    phases = doc["phases"]
    assert len(phases) == 2
    assert phases[0]["name"] == "APPROACH_COARSE"
    assert phases[0]["ok"] is True
    assert phases[1]["name"] == "GRASP_DOWN"
    assert phases[1]["ok"] is False
    assert phases[1]["note"] == "timeout"


def test_phase_has_timing(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    logger.begin_phase("APPROACH_COARSE")
    time.sleep(0.02)
    logger.end_phase(ok=True)
    path = logger.end_cycle()
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    p = doc["phases"][0]
    assert p["start_sec"] >= 0.0
    assert p["end_sec"] > p["start_sec"]
    assert p["duration_sec"] > 0.0


def test_implicit_phase_end_when_new_begins(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    logger.begin_phase("PHASE_A")
    logger.begin_phase("PHASE_B")  # implicitly ends PHASE_A
    logger.end_phase(ok=True)
    path = logger.end_cycle()
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert len(doc["phases"]) == 2
    assert doc["phases"][0]["name"] == "PHASE_A"
    assert doc["phases"][1]["name"] == "PHASE_B"


def test_open_phase_closed_on_end_cycle(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    logger.begin_phase("UNFINISHED")
    path = logger.end_cycle(outcome="ABORTED")
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert len(doc["phases"]) == 1
    assert doc["phases"][0]["name"] == "UNFINISHED"


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def test_metrics_phase_count(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    for name in ("A", "B", "C"):
        logger.begin_phase(name)
        logger.end_phase(ok=True)
    path = logger.end_cycle(outcome="SUCCESS")
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert doc["metrics"]["phase_count"] == 3
    assert doc["metrics"]["failed_phases"] == 0


def test_metrics_failed_phases(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    logger.begin_phase("OK"); logger.end_phase(ok=True)
    logger.begin_phase("FAIL1"); logger.end_phase(ok=False)
    logger.begin_phase("FAIL2"); logger.end_phase(ok=False)
    path = logger.end_cycle()
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert doc["metrics"]["failed_phases"] == 2


def test_add_metric(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    logger.add_metric("ik_attempts", 3)
    logger.add_metric("ee_error_m", 0.0045)
    path = logger.end_cycle()
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert doc["metrics"]["ik_attempts"] == 3
    assert doc["metrics"]["ee_error_m"] == pytest.approx(0.0045)


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------

def test_add_error(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    logger.add_error("TF lookup timed out")
    logger.add_error("IK failed after 4 attempts")
    path = logger.end_cycle(outcome="FAILED")
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert len(doc["errors"]) == 2
    assert "TF lookup timed out" in doc["errors"]


# ---------------------------------------------------------------------------
# Reuse across cycles
# ---------------------------------------------------------------------------

def test_logger_reusable(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    id1 = logger.begin_cycle()
    logger.begin_phase("A"); logger.end_phase(ok=True)
    logger.end_cycle(outcome="SUCCESS")

    id2 = logger.begin_cycle()
    logger.begin_phase("B"); logger.end_phase(ok=False)
    logger.end_cycle(outcome="FAILED")

    assert id1 != id2
    doc1 = json.loads((tmp_path / f"{id1}.json").read_text())
    doc2 = json.loads((tmp_path / f"{id2}.json").read_text())
    assert doc1["outcome"] == "SUCCESS"
    assert doc2["outcome"] == "FAILED"
    assert len(doc1["phases"]) == 1
    assert len(doc2["phases"]) == 1


def test_second_cycle_has_no_first_cycle_phases(tmp_path):
    logger = CycleLogger(log_root=str(tmp_path))
    logger.begin_cycle()
    logger.begin_phase("PHASE_FROM_CYCLE_1"); logger.end_phase(ok=True)
    logger.end_cycle(outcome="SUCCESS")

    id2 = logger.begin_cycle()
    logger.end_cycle(outcome="ABORTED")
    doc2 = json.loads((tmp_path / f"{id2}.json").read_text())
    assert doc2["phases"] == []


# ---------------------------------------------------------------------------
# Log dir creation
# ---------------------------------------------------------------------------

def test_creates_log_root_if_missing(tmp_path):
    root = str(tmp_path / "deeply" / "nested" / "cycles")
    logger = CycleLogger(log_root=root)
    logger.begin_cycle()
    path = logger.end_cycle(outcome="SUCCESS")
    assert path is not None
    import os
    assert os.path.isdir(root)


# ---------------------------------------------------------------------------
# Edge paths: flush with no phase, write failure
# ---------------------------------------------------------------------------

def test_end_phase_without_begin_phase_is_noop(tmp_path):
    # _flush_current_phase when _current_phase is None → early return (line 180)
    logger = CycleLogger(log_root=str(tmp_path))
    cid = logger.begin_cycle()
    logger.end_phase(ok=True)  # no begin_phase → flush is a no-op
    path = logger.end_cycle(outcome="SUCCESS")
    doc = json.loads((tmp_path / f"{cid}.json").read_text())
    assert doc["phases"] == []  # nothing was recorded


def test_write_failure_returns_none(tmp_path):
    # _write raises → except Exception: return None (lines 200-201)
    logger = CycleLogger(log_root="/proc/no_such_dir_ever/cannot_write")
    logger.begin_cycle()
    path = logger.end_cycle(outcome="SUCCESS")
    assert path is None
