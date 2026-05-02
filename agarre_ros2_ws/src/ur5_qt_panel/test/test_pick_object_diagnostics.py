#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_object_diagnostics.py
# Contenido: F15 (2026-05-01) — tests offline de pick_object/diagnostics.
"""Tests offline de ``pick_object.diagnostics``.

F15 extrae helpers puros de logs estructurados de ``run_pick_object``
para que sean testeables sin Qt/ROS.
"""

from __future__ import annotations

import pytest

from ur5_qt_panel.pick_object.diagnostics import (
    classify_phase_outcome,
    filter_known_metrics,
    format_phase_marker,
    format_phase_metrics,
    format_target_pose,
)


# ---------------------------------------------------------------------------
# format_phase_marker
# ---------------------------------------------------------------------------


def test_phase_marker_default_status():
    assert format_phase_marker("approach") == "[PICK_OBJECT][APPROACH] status=ok"


def test_phase_marker_with_detail():
    out = format_phase_marker("grasp", status="fail", detail="ik_no_solution")
    assert out == "[PICK_OBJECT][GRASP] status=fail detail=ik_no_solution"


def test_phase_marker_empty_phase_uses_unknown():
    assert "UNKNOWN" in format_phase_marker("")


def test_phase_marker_strips_whitespace():
    out = format_phase_marker("  lift  ", status="  ok  ", detail="  done  ")
    assert "[PICK_OBJECT][LIFT]" in out
    assert "status=ok" in out
    assert "detail=done" in out


# ---------------------------------------------------------------------------
# format_phase_metrics
# ---------------------------------------------------------------------------


def test_metrics_floats_4_decimals():
    out = format_phase_metrics("approach", {"x": 0.5, "y": 0.0, "z": 0.3})
    assert "x=0.5000" in out
    assert "y=0.0000" in out
    assert "z=0.3000" in out


def test_metrics_ints_as_int():
    out = format_phase_metrics("retry", {"attempts": 3})
    assert "attempts=3" in out
    assert "attempts=3.0000" not in out


def test_metrics_bool_as_string():
    out = format_phase_metrics("gate", {"ok": True, "fail": False})
    assert "ok=true" in out
    assert "fail=false" in out


def test_metrics_keys_sorted():
    out = format_phase_metrics("any", {"z": 1, "a": 2, "m": 3})
    # En el orden alfabético: a, m, z
    a_idx = out.index("a=")
    m_idx = out.index("m=")
    z_idx = out.index("z=")
    assert a_idx < m_idx < z_idx


def test_metrics_empty_dict_just_marker():
    assert format_phase_metrics("test", {}) == "[PICK_OBJECT][TEST]"


def test_metrics_non_dict_just_marker():
    assert format_phase_metrics("test", None) == "[PICK_OBJECT][TEST]"


# ---------------------------------------------------------------------------
# format_target_pose
# ---------------------------------------------------------------------------


def test_target_pose_default_world():
    out = format_target_pose((0.5, 0.0, 0.05))
    assert "0.500" in out
    assert "0.000" in out
    assert "0.050" in out
    assert "@world" in out


def test_target_pose_custom_frame():
    out = format_target_pose((1.0, 2.0, 3.0), frame="base_link")
    assert "@base_link" in out


def test_target_pose_invalid():
    assert format_target_pose(None) == "target=invalid"
    assert format_target_pose("foo") == "target=invalid"
    assert format_target_pose((1.0, 2.0)) == "target=invalid"  # solo 2 elementos
    assert format_target_pose((1.0, "foo", 3.0)) == "target=invalid"


def test_target_pose_truncates_extra():
    """Si pasan más de 3 elementos, solo toma los 3 primeros."""
    out = format_target_pose((0.1, 0.2, 0.3, 0.4, 0.5))
    assert "0.100" in out
    assert "0.300" in out
    assert "0.4" not in out


# ---------------------------------------------------------------------------
# classify_phase_outcome
# ---------------------------------------------------------------------------


def test_outcome_ok():
    assert classify_phase_outcome(True, "exec_ok") == "ok"
    assert classify_phase_outcome(True, "") == "ok"


def test_outcome_failed():
    assert classify_phase_outcome(False, "plan_failed") == "failed"
    assert classify_phase_outcome(False, "") == "failed"


def test_outcome_timeout():
    assert classify_phase_outcome(False, "request_timeout") == "timeout"
    assert classify_phase_outcome(False, "exec_timed_out:30s") == "timeout"


def test_outcome_unknown():
    assert classify_phase_outcome(None, "anything") == "unknown"


# ---------------------------------------------------------------------------
# filter_known_metrics
# ---------------------------------------------------------------------------


def test_filter_known():
    metrics = {"x": 1, "y": 2, "debug_extra": 99}
    out = filter_known_metrics(metrics, allowed_keys=("x", "y"))
    assert out == {"x": 1, "y": 2}


def test_filter_empty():
    assert filter_known_metrics({}, allowed_keys=("x",)) == {}


def test_filter_non_dict():
    assert filter_known_metrics(None, allowed_keys=("x",)) == {}
