#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_evidence_logger_offline.py
# Contenido: F4 — tests offline de helpers puros del evidence_logger.
"""Tests offline de los helpers puros de ``ur5_tools.evidence_logger``.

NO instancian la clase ``EvidenceLogger`` (que requiere rclpy.Node y
QoS de ROS 2). Sólo verifican:

* ``_now_iso()`` produce timestamps ISO 8601 razonables.
* ``_safe_unique_dir(root)`` crea directorios distintos en colisión.

Si rclpy no está disponible, se importan los helpers desde el módulo
con un try/except — si falla se skipea con razón clara.
"""

from __future__ import annotations

import re
from datetime import datetime
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def helpers():
    """Importa los helpers desde evidence_helpers (sin rclpy)."""
    from ur5_tools.evidence_helpers import now_iso, safe_unique_dir
    return now_iso, safe_unique_dir


# ---------------------------------------------------------------------------
# _now_iso
# ---------------------------------------------------------------------------


def test_now_iso_returns_string(helpers):
    _now_iso, _ = helpers
    s = _now_iso()
    assert isinstance(s, str)
    assert s.endswith("Z"), f"timestamp debe acabar en Z (UTC), got {s!r}"


def test_now_iso_parseable(helpers):
    _now_iso, _ = helpers
    s = _now_iso()
    # Formato esperado: 2026-04-30T22:01:02.345678Z
    assert re.match(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d+Z$", s), (
        f"timestamp no cumple formato esperado: {s!r}"
    )
    # Parseable como datetime ISO 8601 (sustituyendo Z por +00:00)
    dt = datetime.fromisoformat(s.replace("Z", "+00:00"))
    assert dt.year >= 2025  # sanity check del reloj


def test_now_iso_monotonic_or_equal(helpers):
    _now_iso, _ = helpers
    a = _now_iso()
    b = _now_iso()
    assert a <= b, f"timestamps consecutivos deben ser monotónicos: {a} > {b}"


# ---------------------------------------------------------------------------
# _safe_unique_dir
# ---------------------------------------------------------------------------


def test_safe_unique_dir_creates_directory(helpers, tmp_path):
    _, _safe_unique_dir = helpers
    out = _safe_unique_dir(tmp_path)
    assert out.exists() and out.is_dir()
    assert out.parent == tmp_path


def test_safe_unique_dir_handles_collision(helpers, tmp_path, monkeypatch):
    _, _safe_unique_dir = helpers
    # Forzar colisión: pre-crear un dir con el timestamp actual.
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    pre = tmp_path / stamp
    pre.mkdir()
    out = _safe_unique_dir(tmp_path)
    assert out != pre, "no debe sobrescribir un directorio existente"
    assert out.exists() and out.is_dir()
    assert out.name.startswith(stamp), (
        f"el sufijo debe basarse en el stamp original: got {out.name!r}"
    )


def test_safe_unique_dir_returns_path(helpers, tmp_path):
    _, _safe_unique_dir = helpers
    out = _safe_unique_dir(tmp_path)
    assert isinstance(out, Path)


# ---------------------------------------------------------------------------
# F10: parse_grasp_result
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def parse_result():
    from ur5_tools.evidence_helpers import parse_grasp_result
    return parse_grasp_result


def test_parse_grasp_result_success_true(parse_result):
    ok, reason = parse_result("success=true reason=exec_ok request_uuid=abc")
    assert ok is True
    assert reason == "exec_ok"


def test_parse_grasp_result_success_false(parse_result):
    ok, reason = parse_result("success=false reason=plan_failed:joint_limits")
    assert ok is False
    assert reason == "plan_failed:joint_limits"


def test_parse_grasp_result_alias_truthy(parse_result):
    assert parse_result("success=1 reason=ok")[0] is True
    assert parse_result("success=yes reason=ok")[0] is True


def test_parse_grasp_result_alias_falsy(parse_result):
    assert parse_result("success=0 reason=fail")[0] is False
    assert parse_result("success=no reason=fail")[0] is False


def test_parse_grasp_result_empty(parse_result):
    ok, reason = parse_result("")
    assert ok is None
    assert reason == ""


def test_parse_grasp_result_no_kv(parse_result):
    """Texto sin pares key=value: success None, reason = texto entero."""
    ok, reason = parse_result("just some unstructured text")
    assert ok is None
    assert reason == "just some unstructured text"


def test_parse_grasp_result_only_reason(parse_result):
    ok, reason = parse_result("reason=some_failure")
    assert ok is None
    assert reason == "some_failure"


def test_parse_grasp_result_unknown_success_value(parse_result):
    """success=maybe → success None pero reason sí se lee."""
    ok, reason = parse_result("success=maybe reason=odd")
    assert ok is None
    assert reason == "odd"


# ---------------------------------------------------------------------------
# F10: compute_session_metrics
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def metrics_fn():
    from ur5_tools.evidence_helpers import compute_session_metrics
    return compute_session_metrics


def test_metrics_empty_session(metrics_fn):
    m = metrics_fn([])
    assert m["total_events"] == 0
    assert m["by_kind"] == {}
    assert m["grasp_success"] == 0
    assert m["grasp_failure"] == 0
    assert m["grasp_success_rate"] is None
    assert m["duration_sec"] is None


def test_metrics_aggregates_by_kind(metrics_fn):
    events = [
        {"kind": "session_started", "ts_iso": "2026-05-01T00:00:00Z", "ts_mono": 100.0},
        {"kind": "grasp_result", "data": {"success": True}, "ts_mono": 110.0},
        {"kind": "grasp_result", "data": {"success": True}, "ts_mono": 120.0},
        {"kind": "grasp_result", "data": {"success": False}, "ts_mono": 130.0},
        {"kind": "session_finished", "ts_iso": "2026-05-01T00:01:00Z", "ts_mono": 140.0},
    ]
    m = metrics_fn(events)
    assert m["total_events"] == 5
    assert m["by_kind"]["grasp_result"] == 3
    assert m["by_kind"]["session_started"] == 1
    assert m["grasp_success"] == 2
    assert m["grasp_failure"] == 1
    assert m["grasp_success_rate"] == pytest.approx(2 / 3)
    assert m["duration_sec"] == pytest.approx(40.0)
    assert m["session_started_iso"] == "2026-05-01T00:00:00Z"
    assert m["session_finished_iso"] == "2026-05-01T00:01:00Z"


def test_metrics_attach_detach_count(metrics_fn):
    events = [
        {"kind": "gripper_state", "data": {"object": "box_red", "attached": True}},
        {"kind": "gripper_state", "data": {"object": "box_red", "attached": False}},
        {"kind": "gripper_state", "data": {"object": "cyl_blue", "attached": True}},
        {"kind": "gripper_state", "data": {"object": "box_red", "attached": True}},
    ]
    m = metrics_fn(events)
    assert m["attach_count"] == 3
    assert m["detach_count"] == 1
    # objects_attached lleva nombres únicos en orden de primera vista
    assert m["objects_attached"] == ["box_red", "cyl_blue"]


def test_metrics_grasp_unknown_does_not_count_in_rate(metrics_fn):
    events = [
        {"kind": "grasp_result", "data": {"success": True}},
        {"kind": "grasp_result", "data": {"success": None}},
        {"kind": "grasp_result", "data": {}},
    ]
    m = metrics_fn(events)
    assert m["grasp_success"] == 1
    assert m["grasp_failure"] == 0
    assert m["grasp_unknown"] == 2
    # success_rate solo cuenta clasificados
    assert m["grasp_success_rate"] == pytest.approx(1.0)


def test_metrics_handles_invalid_entries(metrics_fn):
    events = [
        "not a dict",
        None,
        {"kind": "grasp_result", "data": {"success": True}},
    ]
    m = metrics_fn(events)
    assert m["total_events"] == 1  # solo el dict cuenta
    assert m["grasp_success"] == 1


def test_metrics_duration_requires_two_monos(metrics_fn):
    events = [{"kind": "x", "ts_mono": 100.0}]
    m = metrics_fn(events)
    assert m["duration_sec"] is None
