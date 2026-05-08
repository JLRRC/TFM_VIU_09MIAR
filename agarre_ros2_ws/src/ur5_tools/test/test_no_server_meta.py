#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_no_server_meta.py
"""F3-step41a (2026-05-08) — Tests offline para no_server_meta."""

from __future__ import annotations

from ur5_tools.moveit_bridge.no_server_meta import (
    build_no_server_meta,
    build_no_server_reason,
)


def test_meta_basic_with_available_and_candidates():
    meta = build_no_server_meta(
        expected_action="/joint_trajectory_controller/follow_joint_trajectory",
        available_actions=["/jtc/cancel_goal", "/jtc/get_result"],
        candidates=["/joint_trajectory_controller/follow_joint_trajectory", "/foo/follow"],
    )
    assert meta["action"] == "/joint_trajectory_controller/follow_joint_trajectory"
    assert meta["status_text"] == "NO_SERVER"
    assert "checked=" in meta["error_string"]
    assert "available=" in meta["error_string"]


def test_meta_empty_available():
    meta = build_no_server_meta(
        expected_action="/expected",
        available_actions=[],
        candidates=["/foo"],
    )
    assert "available=none" in meta["error_string"]
    assert "checked=/foo" in meta["error_string"]


def test_meta_empty_candidates():
    meta = build_no_server_meta(
        expected_action="/expected",
        available_actions=["/something"],
        candidates=[],
    )
    assert "checked=none" in meta["error_string"]


def test_meta_empty_both():
    meta = build_no_server_meta(
        expected_action="/expected",
        available_actions=[],
        candidates=[],
    )
    assert "checked=none available=none" == meta["error_string"]


def test_meta_custom_status_text():
    meta = build_no_server_meta(
        expected_action="/expected",
        available_actions=["/x"],
        candidates=["/y"],
        status_text="CUSTOM_STATE",
    )
    assert meta["status_text"] == "CUSTOM_STATE"


def test_meta_available_actions_sorted():
    """Sortear los available actions para tener output determinista."""
    meta = build_no_server_meta(
        expected_action="/expected",
        available_actions=["/zzz", "/aaa", "/mmm"],
        candidates=[],
    )
    # Debe estar ordenado alfabéticamente
    assert "available=/aaa,/mmm,/zzz" in meta["error_string"]


def test_reason_format_is_canonical():
    """Format: fjt_no_action_server:expected_action=X;checked_candidates=Y;available_actions=Z."""
    reason = build_no_server_reason(
        expected_action="/jtc/follow",
        available_actions=["/x"],
        candidates=["/jtc/follow", "/legacy/follow"],
    )
    assert reason.startswith("fjt_no_action_server:")
    assert "expected_action=/jtc/follow" in reason
    assert "checked_candidates=/jtc/follow,/legacy/follow" in reason
    assert "available_actions=/x" in reason


def test_reason_empty_lists():
    reason = build_no_server_reason(
        expected_action="/jtc/follow",
        available_actions=[],
        candidates=[],
    )
    assert "checked_candidates=none" in reason
    assert "available_actions=none" in reason
