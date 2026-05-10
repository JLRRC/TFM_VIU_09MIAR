#!/usr/bin/env python3
"""F13 (auditoría 2026-05-10): tests offline de moveit_precheck."""
from __future__ import annotations

import pytest

from tfm_orchestrator.moveit_precheck import (
    MOVE_GROUP_ACTION_DEFAULT,
    MOVEIT_REQUIRED_SERVICES_DEFAULT,
    diagnose_missing_moveit,
    moveit_ready_from_graph,
    select_precheck_timeout,
)


# ---------------- moveit_ready_from_graph ----------------


def test_ready_when_action_and_services_present():
    ok, reason = moveit_ready_from_graph(
        action_names=["/move_action", "/other"],
        service_names=["/get_planning_scene", "/compute_ik", "/foo"],
    )
    assert ok is True
    assert reason == "ok"


def test_not_ready_when_move_action_missing():
    ok, reason = moveit_ready_from_graph(
        action_names=["/joint_trajectory_controller/follow_joint_trajectory"],
        service_names=["/get_planning_scene", "/compute_ik"],
    )
    assert ok is False
    assert reason == "action_missing:/move_action"


def test_not_ready_when_service_missing():
    ok, reason = moveit_ready_from_graph(
        action_names=["/move_action"],
        service_names=["/get_planning_scene"],  # falta /compute_ik
    )
    assert ok is False
    assert reason == "service_missing:/compute_ik"


def test_not_ready_when_all_missing():
    """Devuelve solo el primer fallo (action prevalece)."""
    ok, reason = moveit_ready_from_graph(
        action_names=[],
        service_names=[],
    )
    assert ok is False
    assert reason == "action_missing:/move_action"


def test_custom_action_name():
    ok, reason = moveit_ready_from_graph(
        action_names=["/custom_move"],
        service_names=["/get_planning_scene", "/compute_ik"],
        move_group_action="/custom_move",
    )
    assert ok is True


def test_custom_required_services():
    ok, reason = moveit_ready_from_graph(
        action_names=["/move_action"],
        service_names=["/foo"],
        required_services=("/foo",),
    )
    assert ok is True


# ---------------- select_precheck_timeout ----------------


def test_timeout_full_when_moveit_known_running():
    assert select_precheck_timeout(
        base_timeout_sec=60.0, moveit_known_running=True
    ) == 60.0


def test_timeout_capped_when_moveit_unknown():
    assert select_precheck_timeout(
        base_timeout_sec=60.0, moveit_known_running=False
    ) == 5.0


def test_timeout_keeps_smaller_base_when_unknown():
    """Si la base ya es pequeña, no la subimos."""
    assert select_precheck_timeout(
        base_timeout_sec=2.0, moveit_known_running=False
    ) == 2.0


# ---------------- diagnose_missing_moveit ----------------


def test_diagnose_all_present_returns_empty():
    out = diagnose_missing_moveit(
        action_names=["/move_action"],
        service_names=list(MOVEIT_REQUIRED_SERVICES_DEFAULT),
    )
    assert out == []


def test_diagnose_action_missing():
    out = diagnose_missing_moveit(
        action_names=[],
        service_names=list(MOVEIT_REQUIRED_SERVICES_DEFAULT),
    )
    assert out == ["action:/move_action"]


def test_diagnose_lists_all_missing_services():
    """A diferencia de moveit_ready_from_graph, enumera todos los faltantes."""
    out = diagnose_missing_moveit(
        action_names=["/move_action"],
        service_names=[],
    )
    assert "service:/get_planning_scene" in out
    assert "service:/compute_ik" in out
    assert len(out) == 2


def test_diagnose_action_and_services_missing():
    out = diagnose_missing_moveit(
        action_names=[],
        service_names=[],
    )
    assert out[0] == "action:/move_action"
    assert "service:/get_planning_scene" in out
    assert "service:/compute_ik" in out


def test_constants_are_strings():
    assert isinstance(MOVE_GROUP_ACTION_DEFAULT, str)
    assert all(isinstance(s, str) for s in MOVEIT_REQUIRED_SERVICES_DEFAULT)
