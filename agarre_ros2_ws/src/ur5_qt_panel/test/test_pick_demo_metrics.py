#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_demo_metrics.py
# Contenido: F3 — tests offline de pick_demo/metrics.py.
"""Tests offline de las funciones puras de métricas pick_demo."""

from __future__ import annotations

import math

import pytest

from ur5_qt_panel.pick_demo.metrics import (
    alignment_metrics_base,
    grasp_down_runtime_metrics,
    joint_delta_metrics,
    joint_error_metrics,
)


UR5_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


# ---------------------------------------------------------------------------
# joint_delta_metrics
# ---------------------------------------------------------------------------


def test_joint_delta_empty_reference():
    out = joint_delta_metrics(None, [0.0] * 6, UR5_NAMES)
    assert out == {
        "joint_delta_abs": {},
        "max_joint_delta": None,
        "sum_joint_delta": None,
        "critical_joints_delta": {},
    }


def test_joint_delta_empty_final():
    out = joint_delta_metrics([0.0] * 6, [], UR5_NAMES)
    assert out["joint_delta_abs"] == {}
    assert out["max_joint_delta"] is None


def test_joint_delta_zero_when_identical():
    joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    out = joint_delta_metrics(joints, joints, UR5_NAMES)
    assert out["max_joint_delta"] == pytest.approx(0.0)
    assert out["sum_joint_delta"] == pytest.approx(0.0)
    assert all(v == pytest.approx(0.0) for v in out["joint_delta_abs"].values())


def test_joint_delta_critical_subset():
    ref = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    final = [0.0, 0.5, 0.3, 0.1, 0.0, 0.0]
    out = joint_delta_metrics(ref, final, UR5_NAMES)
    crit = out["critical_joints_delta"]
    assert "shoulder_lift_joint" in crit
    assert "elbow_joint" in crit
    assert "wrist_1_joint" in crit
    assert crit["shoulder_lift_joint"] == pytest.approx(0.5)
    assert crit["elbow_joint"] == pytest.approx(0.3)


def test_joint_delta_uses_shortest_path():
    """Diff entre 0.1 y 2*pi-0.1 ≈ 2*0.1 (no 2*pi-0.2)."""
    ref = [0.1] + [0.0] * 5
    final = [2 * math.pi - 0.1] + [0.0] * 5
    out = joint_delta_metrics(ref, final, UR5_NAMES)
    assert out["max_joint_delta"] == pytest.approx(0.2, abs=1e-6)


# ---------------------------------------------------------------------------
# joint_error_metrics
# ---------------------------------------------------------------------------


def test_joint_error_no_current_positions():
    out = joint_error_metrics([0.0] * 6, UR5_NAMES, {})
    assert out["available"] is False
    assert out["max_diff_rad"] is None


def test_joint_error_partial_positions():
    """Sólo 2 joints en current positions — calcula sobre ellos."""
    current = {"shoulder_pan_joint": 0.1, "elbow_joint": 0.5}
    out = joint_error_metrics([0.2, 0.0, 1.0, 0.0, 0.0, 0.0], UR5_NAMES, current)
    assert out["available"] is True
    # diff[shoulder_pan] = 0.1, diff[elbow] = 0.5
    assert out["max_diff_rad"] == pytest.approx(0.5)
    assert out["sum_diff_rad"] == pytest.approx(0.6)
    # shoulder_max sólo se llena para idx 0/1; elbow es idx 2 → no shoulder
    assert out["shoulder_max_diff_rad"] == pytest.approx(0.1)


def test_joint_error_shoulder_max_zero_if_no_shoulder():
    current = {"elbow_joint": 0.0}
    out = joint_error_metrics([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], UR5_NAMES, current)
    assert out["available"] is True
    assert out["shoulder_max_diff_rad"] == 0.0


def test_joint_error_clamps_when_joints_shorter_than_names():
    current = {n: 0.0 for n in UR5_NAMES}
    out = joint_error_metrics([0.1, 0.2], UR5_NAMES, current)
    assert out["available"] is True
    assert out["sum_diff_rad"] == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# alignment_metrics_base
# ---------------------------------------------------------------------------


def _trivial_consistency_ok():
    return {"sources_ok": True}


def _trivial_consistency_bad():
    return {"sources_ok": False}


def test_alignment_pose_unavailable_obj_none():
    out = alignment_metrics_base(
        obj_base=None,
        tcp_base=(0.0, 0.0, 0.5),
        grasp_z_target=0.0,
        xy_tol=0.01,
        z_tol=0.01,
        pose_consistency=_trivial_consistency_ok(),
    )
    assert out["ok"] is False
    assert out["reason"] == "pose_unavailable"
    assert out["xy_dist"] is None


def test_alignment_pose_unavailable_tcp_none():
    out = alignment_metrics_base(
        obj_base=(0.0, 0.0, 0.0),
        tcp_base=None,
        grasp_z_target=0.0,
        xy_tol=0.01,
        z_tol=0.01,
        pose_consistency=_trivial_consistency_ok(),
    )
    assert out["ok"] is False


def test_alignment_ok_when_within_tol():
    out = alignment_metrics_base(
        obj_base=(0.5, 0.5, 0.0),
        tcp_base=(0.5005, 0.5, 0.020),  # xy=0.0005, z_gap=0.020
        grasp_z_target=0.020,
        xy_tol=0.005,
        z_tol=0.005,
        pose_consistency=_trivial_consistency_ok(),
    )
    assert out["ok"] is True
    assert out["geometry_ok"] is True
    assert out["reason"] == "ok"
    assert out["xy_dist"] == pytest.approx(0.0005)
    assert out["z_gap"] == pytest.approx(0.020)
    assert out["z_error"] == pytest.approx(0.0, abs=1e-9)


def test_alignment_fail_pose_source_mismatch():
    out = alignment_metrics_base(
        obj_base=(0.0, 0.0, 0.0),
        tcp_base=(0.0, 0.0, 0.0),
        grasp_z_target=0.0,
        xy_tol=0.01,
        z_tol=0.01,
        pose_consistency=_trivial_consistency_bad(),
    )
    assert out["ok"] is False
    assert out["reason"] == "pose_source_mismatch"
    assert out["pose_source_ok"] is False


def test_alignment_fail_out_of_tolerance_xy():
    out = alignment_metrics_base(
        obj_base=(0.0, 0.0, 0.0),
        tcp_base=(0.05, 0.0, 0.0),
        grasp_z_target=0.0,
        xy_tol=0.01,
        z_tol=0.01,
        pose_consistency=_trivial_consistency_ok(),
    )
    assert out["ok"] is False
    assert out["reason"] == "alignment_out_of_tolerance"
    assert out["xy_dist"] == pytest.approx(0.05)


def test_alignment_z_error_against_target():
    """z_error = |z_gap - grasp_z_target|."""
    out = alignment_metrics_base(
        obj_base=(0.0, 0.0, 0.0),
        tcp_base=(0.0, 0.0, 0.030),  # z_gap=0.030
        grasp_z_target=0.020,
        xy_tol=0.01,
        z_tol=0.005,
        pose_consistency=_trivial_consistency_ok(),
    )
    assert out["z_error"] == pytest.approx(0.010)
    assert out["ok"] is False  # z_error 0.010 > 0.005


# ---------------------------------------------------------------------------
# grasp_down_runtime_metrics
# ---------------------------------------------------------------------------


def test_grasp_down_all_none():
    out = grasp_down_runtime_metrics(target_base=None, tcp_base=None, obj_base=None)
    assert out["xy_err_target"] is None
    assert out["xy_err_object"] is None
    assert out["target_dist"] is None


def test_grasp_down_target_only():
    out = grasp_down_runtime_metrics(
        target_base=(0.5, 0.5, 0.1),
        tcp_base=(0.5, 0.5, 0.15),
    )
    assert out["xy_err_target"] == pytest.approx(0.0, abs=1e-9)
    assert out["z_err_target"] == pytest.approx(0.05)
    assert out["target_dist"] == pytest.approx(0.05)
    assert out["xy_err_object"] is None
    assert out["z_gap_object"] is None


def test_grasp_down_full():
    out = grasp_down_runtime_metrics(
        target_base=(0.5, 0.5, 0.1),
        tcp_base=(0.5, 0.5, 0.15),
        obj_base=(0.5, 0.5, 0.0),
    )
    assert out["xy_err_object"] == pytest.approx(0.0, abs=1e-9)
    assert out["z_gap_object"] == pytest.approx(0.15)


def test_grasp_down_xy_only_offset():
    out = grasp_down_runtime_metrics(
        target_base=(0.5, 0.5, 0.1),
        tcp_base=(0.503, 0.504, 0.1),
    )
    assert out["xy_err_target"] == pytest.approx(0.005, abs=1e-3)
    assert out["z_err_target"] == pytest.approx(0.0, abs=1e-9)
