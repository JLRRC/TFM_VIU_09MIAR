#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_place_client_logic.py
# Contenido: F6.4 — tests de la lógica pura del cliente PickPlace.
"""Tests offline de ``ur5_qt_panel.pick_place_client_logic``.

NO requieren ROS — sólo verifican validación, conversión y feature
flag.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from ur5_qt_panel.pick_place_client_logic import (
    PickPlaceGoalRequest,
    build_goal_request,
    feedback_to_panel_event,
    result_to_panel_event,
    should_use_orchestrator,
)


# ---------------------------------------------------------------------------
# build_goal_request
# ---------------------------------------------------------------------------


def test_build_goal_request_valid():
    req, reason = build_goal_request("box_red", (0.5, 0.0, 0.05))
    assert isinstance(req, PickPlaceGoalRequest)
    assert reason == ""
    assert req.object_name == "box_red"
    assert req.drop_xyz_world == (0.5, 0.0, 0.05)


def test_build_goal_request_strips_object_name():
    req, _ = build_goal_request("  box_blue  ", (0.0, 0.0, 0.0))
    assert req.object_name == "box_blue"


def test_build_goal_request_empty_name_fails():
    req, reason = build_goal_request("", (0.0, 0.0, 0.0))
    assert req is None
    assert reason == "object_name_empty"


def test_build_goal_request_whitespace_name_fails():
    req, reason = build_goal_request("   ", (0.0, 0.0, 0.0))
    assert req is None
    assert reason == "object_name_empty"


def test_build_goal_request_none_xyz_fails():
    req, reason = build_goal_request("box", None)
    assert req is None
    assert reason.startswith("drop_xyz_world_invalid")


def test_build_goal_request_short_xyz_fails():
    req, reason = build_goal_request("box", (1.0, 2.0))
    assert req is None
    assert reason.startswith("drop_xyz_world_invalid")


def test_build_goal_request_nan_xyz_fails():
    req, reason = build_goal_request("box", (0.0, float("nan"), 0.0))
    assert req is None
    assert "non_finite" in reason


def test_build_goal_request_inf_xyz_fails():
    req, reason = build_goal_request("box", (0.0, float("inf"), 0.0))
    assert req is None
    assert "non_finite" in reason


def test_build_goal_request_accepts_list_xyz():
    req, _ = build_goal_request("box", [0.1, 0.2, 0.3])
    assert req is not None
    assert req.drop_xyz_world == (0.1, 0.2, 0.3)


def test_build_goal_request_truncates_extra_xyz():
    """Si pasan 4+ elementos, solo toma los 3 primeros."""
    req, _ = build_goal_request("box", (0.1, 0.2, 0.3, 0.4, 0.5))
    assert req is not None
    assert req.drop_xyz_world == (0.1, 0.2, 0.3)


# ---------------------------------------------------------------------------
# feedback_to_panel_event
# ---------------------------------------------------------------------------


def test_feedback_to_panel_event_full():
    fb = SimpleNamespace(
        current_phase="GRASP",
        progress=0.42,
        phase_index=3,
        detail="grasp_ok",
    )
    ev = feedback_to_panel_event(fb)
    assert ev == {
        "current_phase": "GRASP",
        "progress": 0.42,
        "phase_index": 3,
        "detail": "grasp_ok",
    }


def test_feedback_to_panel_event_none():
    ev = feedback_to_panel_event(None)
    assert ev["current_phase"] == ""
    assert ev["progress"] == 0.0
    assert ev["phase_index"] == -1
    assert ev["detail"] == ""


def test_feedback_to_panel_event_missing_fields():
    fb = SimpleNamespace()  # sin atributos
    ev = feedback_to_panel_event(fb)
    assert ev["current_phase"] == ""
    assert ev["progress"] == 0.0
    assert ev["phase_index"] == -1


def test_feedback_to_panel_event_coerces_types():
    fb = SimpleNamespace(
        current_phase=42,  # int → str
        progress="0.5",  # str → float
        phase_index=2.7,  # float → int
        detail=None,
    )
    ev = feedback_to_panel_event(fb)
    assert ev["current_phase"] == "42"
    assert ev["progress"] == 0.5
    assert ev["phase_index"] == 2
    assert ev["detail"] == ""


# ---------------------------------------------------------------------------
# result_to_panel_event
# ---------------------------------------------------------------------------


def test_result_to_panel_event_full():
    r = SimpleNamespace(
        success=True,
        reason="ok",
        duration_sec=12.34,
        cycles_completed=1,
    )
    ev = result_to_panel_event(r)
    assert ev == {
        "success": True,
        "reason": "ok",
        "duration_sec": 12.34,
        "cycles_completed": 1,
    }


def test_result_to_panel_event_none():
    ev = result_to_panel_event(None)
    assert ev["success"] is False
    assert ev["reason"] == "no_result"
    assert ev["duration_sec"] == 0.0
    assert ev["cycles_completed"] == 0


def test_result_to_panel_event_missing_fields():
    r = SimpleNamespace()
    ev = result_to_panel_event(r)
    assert ev["success"] is False
    assert ev["reason"] == ""


# ---------------------------------------------------------------------------
# should_use_orchestrator
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("v", ["1", "true", "True", "yes", "on", "ON"])
def test_should_use_orchestrator_truthy(v):
    assert should_use_orchestrator(v) is True


@pytest.mark.parametrize("v", ["0", "false", "no", "off"])
def test_should_use_orchestrator_falsy_explicit(v):
    """Falsy explícito desactiva orchestrator (F12: solo strings reservadas)."""
    assert should_use_orchestrator(v) is False


def test_should_use_orchestrator_none_default_orchestrator_20260508():
    """F5-legacy-removed (2026-05-08): legacy borrado físicamente
    (run_pick_demo eliminado de panel_pick_demo.py). Default = orchestrator
    (era legacy en F1.7 / 2026-05-07; revertido tras borrado del legacy).
    Para recuperar legacy: git checkout audit-pre-borrar-legacy-20260508."""
    assert should_use_orchestrator(None) is True


@pytest.mark.parametrize("v", ["FOO", "weird"])
def test_should_use_orchestrator_unknown_defaults_to_orchestrator_20260508(v):
    """F5-legacy-removed: cualquier valor no falsy explícito → orchestrator."""
    assert should_use_orchestrator(v) is True


@pytest.mark.parametrize("v", ["1", "true", "yes", "on"])
def test_should_use_orchestrator_legacy_override_forces_legacy_sentinel(v):
    """USE_LEGACY_PICK_DEMO=1 → False (sentinel "legacy_removed").
    Tras F5-legacy-removed el dispatcher ya no ejecuta el legacy aunque
    el flag esté truthy — emite log y no hace nada."""
    assert should_use_orchestrator(None, legacy_env_value=v) is False
    assert should_use_orchestrator("1", legacy_env_value=v) is False


@pytest.mark.parametrize("v", ["0", "false", "no", "off", None])
def test_should_use_orchestrator_legacy_override_inactive_20260508(v):
    """F5-legacy-removed: USE_LEGACY falsy + env_value None → orchestrator
    (default invertido tras borrado físico del legacy)."""
    assert should_use_orchestrator(None, legacy_env_value=v) is True


@pytest.mark.parametrize("v", ["1", "true", "yes", "on"])
def test_should_use_orchestrator_explicit_truthy_forces_orchestrator(v):
    """F5-legacy-removed: PANEL_PICK_DEMO_USE_ORCHESTRATOR=1 explícito
    redundante (default ya es orchestrator). Test mantiene compat."""
    assert should_use_orchestrator(v) is True


# ---------------------------------------------------------------------------
# F5-step2: object_pose_world_hint en build_goal_request
# ---------------------------------------------------------------------------


def test_build_goal_request_default_hint_is_none():
    req, _ = build_goal_request("box", (0.5, 0.0, 0.05))
    assert req is not None
    assert req.object_pose_world_hint is None


def test_build_goal_request_accepts_hint_tuple3_completes_with_identity():
    req, reason = build_goal_request(
        "box", (0.5, 0.0, 0.05),
        object_pose_world_hint=(0.4, 0.1, 0.03),
    )
    assert reason == ""
    assert req is not None
    assert req.object_pose_world_hint == (0.4, 0.1, 0.03, 0.0, 0.0, 0.0, 1.0)


def test_build_goal_request_accepts_hint_tuple7_passthrough():
    hint7 = (0.4, 0.1, 0.03, 0.0, 0.0, 0.7071, 0.7071)
    req, reason = build_goal_request(
        "box", (0.5, 0.0, 0.05), object_pose_world_hint=hint7,
    )
    assert reason == ""
    assert req.object_pose_world_hint == hint7


def test_build_goal_request_accepts_hint_list():
    req, _ = build_goal_request(
        "box", (0.5, 0.0, 0.05),
        object_pose_world_hint=[0.4, 0.1, 0.03],
    )
    assert req.object_pose_world_hint == (0.4, 0.1, 0.03, 0.0, 0.0, 0.0, 1.0)


def test_build_goal_request_rejects_hint_invalid_length():
    req, reason = build_goal_request(
        "box", (0.5, 0.0, 0.05),
        object_pose_world_hint=(1.0, 2.0),
    )
    assert req is None
    assert "object_pose_world_hint_invalid" in reason
    assert "len=2" in reason


def test_build_goal_request_rejects_hint_with_non_finite():
    req, reason = build_goal_request(
        "box", (0.5, 0.0, 0.05),
        object_pose_world_hint=(0.1, float("nan"), 0.3),
    )
    assert req is None
    assert "non_finite" in reason


def test_build_goal_request_rejects_hint_non_numeric():
    req, reason = build_goal_request(
        "box", (0.5, 0.0, 0.05),
        object_pose_world_hint=("a", "b", "c"),
    )
    assert req is None
    assert "object_pose_world_hint_invalid" in reason


def test_build_goal_request_hint_inf_rejected():
    req, reason = build_goal_request(
        "box", (0.5, 0.0, 0.05),
        object_pose_world_hint=(0.0, float("inf"), 0.0, 0.0, 0.0, 0.0, 1.0),
    )
    assert req is None
    assert "non_finite" in reason
