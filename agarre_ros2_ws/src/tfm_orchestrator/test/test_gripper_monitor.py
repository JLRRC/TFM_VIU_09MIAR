#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_gripper_monitor.py
# Contenido: B-iter12 (2026-05-03) — tests gripper_monitor.
"""Tests para tfm_orchestrator.gripper_monitor."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from tfm_orchestrator.gripper_monitor import (
    GRIPPER_RG2_JOINT_NAMES,
    extract_gripper_opening_sum,
    is_gripper_closed,
)


def _make_js(joints_dict):
    """Construye JointState mock desde dict {name: position}."""
    names = list(joints_dict.keys())
    positions = list(joints_dict.values())
    return SimpleNamespace(name=names, position=positions)


# ---------------------------------------------------------------------------
# extract_gripper_opening_sum
# ---------------------------------------------------------------------------


def test_extract_opening_sum_with_default_rg2_joints():
    js = _make_js({
        "shoulder_pan_joint": 0.0,
        "rg2_finger_joint_left": 0.030,
        "rg2_finger_joint_right": 0.025,
    })
    assert extract_gripper_opening_sum(js) == pytest.approx(0.055)


def test_extract_opening_sum_returns_none_when_msg_none():
    assert extract_gripper_opening_sum(None) is None


def test_extract_opening_sum_returns_none_when_joint_missing():
    js = _make_js({
        "rg2_finger_joint_left": 0.030,
        # right falta
    })
    assert extract_gripper_opening_sum(js) is None


def test_extract_opening_sum_returns_none_on_length_mismatch():
    js = SimpleNamespace(
        name=["rg2_finger_joint_left", "rg2_finger_joint_right"],
        position=[0.030],  # mismatch
    )
    assert extract_gripper_opening_sum(js) is None


def test_extract_opening_sum_custom_joint_names():
    js = _make_js({
        "custom_left": 0.010,
        "custom_right": 0.015,
    })
    s = extract_gripper_opening_sum(
        js, gripper_joint_names=("custom_left", "custom_right"),
    )
    assert s == pytest.approx(0.025)


def test_extract_opening_sum_zero_when_fully_closed():
    js = _make_js({
        "rg2_finger_joint_left": 0.0,
        "rg2_finger_joint_right": 0.0,
    })
    assert extract_gripper_opening_sum(js) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# is_gripper_closed
# ---------------------------------------------------------------------------


def test_is_closed_true_below_default_threshold():
    js = _make_js({
        "rg2_finger_joint_left": 0.005,
        "rg2_finger_joint_right": 0.005,
    })
    assert is_gripper_closed(js) is True


def test_is_closed_false_above_default_threshold():
    js = _make_js({
        "rg2_finger_joint_left": 0.030,
        "rg2_finger_joint_right": 0.030,
    })
    assert is_gripper_closed(js) is False


def test_is_closed_at_exact_threshold_is_true():
    js = _make_js({
        "rg2_finger_joint_left": 0.010,
        "rg2_finger_joint_right": 0.010,
    })
    assert is_gripper_closed(js) is True  # opening=0.020 == threshold


def test_is_closed_returns_none_when_msg_none():
    assert is_gripper_closed(None) is None


def test_is_closed_returns_none_when_joint_missing():
    js = _make_js({"rg2_finger_joint_left": 0.0})
    assert is_gripper_closed(js) is None


def test_is_closed_custom_threshold():
    js = _make_js({
        "rg2_finger_joint_left": 0.040,
        "rg2_finger_joint_right": 0.040,
    })
    # Default threshold 0.020 → opening 0.080 → False (abierto)
    assert is_gripper_closed(js) is False
    # Custom threshold 0.10 → True (considerado cerrado)
    assert is_gripper_closed(js, closed_threshold_sum=0.10) is True


def test_default_rg2_joint_names_are_two():
    assert len(GRIPPER_RG2_JOINT_NAMES) == 2
    assert "rg2_finger_joint_left" in GRIPPER_RG2_JOINT_NAMES
    assert "rg2_finger_joint_right" in GRIPPER_RG2_JOINT_NAMES
