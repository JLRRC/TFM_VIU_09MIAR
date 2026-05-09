#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_demo_phase_checks.py
# Contenido: F3 — tests offline de pick_demo/phase_checks.py.
"""Tests offline del phase check APPROACH_COARSE."""

from __future__ import annotations

import pytest

from ur5_qt_panel.pick_demo.phase_checks import (
    build_approach_coarse_phase_check,
)


def _baseline(**overrides) -> dict:
    """Caso base 'todo OK estricto' parametrizable con overrides."""
    base = dict(
        target_base=(0.5, 0.5, 0.05),
        gate_metrics={
            "tcp_base": (0.5, 0.5, 0.05),
            "object_base": (0.5, 0.5, 0.0),
            "ok": True,
        },
        fallback_tcp=None,
        fallback_obj=None,
        handoff_dist_tol=0.10,
        handoff_dz_tol=0.10,
        relaxed_handoff_dist_tol=0.15,
        relaxed_handoff_dz_tol=0.15,
        gate_xy_err=0.001,
        gate_pose_ok=True,
        relaxed_xy_cap_m=0.020,
        relaxed_skip_pose_ok=False,
    )
    base.update(overrides)
    return base


# ---------------------------------------------------------------------------
# Result OK strict
# ---------------------------------------------------------------------------


def test_strict_handoff_ok():
    out = build_approach_coarse_phase_check(**_baseline())
    assert out["result"] == "OK"
    assert out["gate_decision"] == "handoff_ready"
    assert out["gate_ok"] is True
    assert out["handoff_dist_ok"] is True
    assert out["handoff_dz_ok"] is True
    assert out["block_reasons"] == []


def test_strict_distances_calculated():
    """tcp=(0.5, 0.5, 0.05), obj=(0.5, 0.5, 0.0) → tcp_obj_dist=0.05, dz=0.05."""
    out = build_approach_coarse_phase_check(**_baseline())
    assert out["tcp_obj_dist"] == pytest.approx(0.05)
    assert out["dz_obj"] == pytest.approx(0.05)
    assert out["actual_z"] == pytest.approx(0.05)
    assert out["object_z"] == pytest.approx(0.0)
    assert out["target_z"] == pytest.approx(0.05)


# ---------------------------------------------------------------------------
# Fallback tcp/obj when gate doesn't supply
# ---------------------------------------------------------------------------


def test_uses_fallback_tcp_when_gate_missing():
    out = build_approach_coarse_phase_check(
        **_baseline(
            gate_metrics={"ok": True},
            fallback_tcp=(0.5, 0.5, 0.05),
            fallback_obj=(0.5, 0.5, 0.0),
        )
    )
    assert out["tcp_base"] == (0.5, 0.5, 0.05)
    assert out["object_base"] == (0.5, 0.5, 0.0)


def test_no_tcp_no_obj_means_no_distances():
    out = build_approach_coarse_phase_check(
        **_baseline(gate_metrics={"ok": True}, fallback_tcp=None, fallback_obj=None)
    )
    assert out["tcp_obj_dist"] is None
    assert out["dz_obj"] is None
    assert out["handoff_dist_ok"] is False
    assert out["handoff_dz_ok"] is False
    assert out["result"] == "NO"


# ---------------------------------------------------------------------------
# Strict failures
# ---------------------------------------------------------------------------


def test_strict_fails_when_gate_not_ok():
    """gate_ok=False y relaxed también desactivado por gate_pose_ok=False."""
    out = build_approach_coarse_phase_check(
        **_baseline(
            gate_metrics={
                "tcp_base": (0.5, 0.5, 0.05),
                "object_base": (0.5, 0.5, 0.0),
                "ok": False,
            },
            gate_pose_ok=False,
            relaxed_skip_pose_ok=False,
        )
    )
    assert out["gate_ok"] is False
    assert out["result"] == "NO"
    assert "phase_gate_not_ready" in out["block_reasons"]


def test_strict_fails_dist_exceeded():
    """strict falla por dist y relaxed también (relaxed_dist_tol < dist)."""
    out = build_approach_coarse_phase_check(
        **_baseline(handoff_dist_tol=0.01, relaxed_handoff_dist_tol=0.005)
    )
    # dist=0.05 > 0.01 estricto, > 0.005 relaxed
    assert out["handoff_dist_ok"] is False
    assert out["relaxed_handoff_ok"] is False
    assert out["result"] == "NO"
    assert "tcp_obj_dist_exceeded" in out["block_reasons"]


def test_strict_fails_dz_exceeded():
    out = build_approach_coarse_phase_check(
        **_baseline(
            handoff_dz_tol=0.01,
            gate_metrics={
                "tcp_base": (0.5, 0.5, 0.20),
                "object_base": (0.5, 0.5, 0.0),
                "ok": True,
            },
        )
    )
    # dz=0.20 > 0.01
    assert out["handoff_dz_ok"] is False
    assert "dz_obj_exceeded" in out["block_reasons"]


# ---------------------------------------------------------------------------
# Relaxed corridor
# ---------------------------------------------------------------------------


def test_relaxed_corridor_passes_when_strict_fails():
    out = build_approach_coarse_phase_check(
        **_baseline(
            handoff_dist_tol=0.01,  # strict fails
            relaxed_handoff_dist_tol=0.10,  # relaxed allows
            gate_xy_err=0.005,
        )
    )
    assert out["handoff_dist_ok"] is False
    assert out["relaxed_handoff_ok"] is True
    assert out["result"] == "OK"
    assert out["gate_decision"] == "handoff_ready_relaxed_corridor"


def test_relaxed_fails_when_xy_cap_exceeded():
    out = build_approach_coarse_phase_check(
        **_baseline(
            handoff_dist_tol=0.01,
            relaxed_handoff_dist_tol=0.10,
            gate_xy_err=0.030,  # > 0.020 cap
        )
    )
    assert out["relaxed_handoff_ok"] is False


def test_relaxed_fails_when_pose_not_ok_and_no_skip():
    out = build_approach_coarse_phase_check(
        **_baseline(
            handoff_dist_tol=0.01,
            relaxed_handoff_dist_tol=0.10,
            gate_pose_ok=False,
            relaxed_skip_pose_ok=False,
        )
    )
    assert out["relaxed_handoff_ok"] is False


def test_relaxed_passes_when_pose_not_ok_but_skip_enabled():
    out = build_approach_coarse_phase_check(
        **_baseline(
            handoff_dist_tol=0.01,
            relaxed_handoff_dist_tol=0.10,
            gate_pose_ok=False,
            relaxed_skip_pose_ok=True,
        )
    )
    assert out["relaxed_handoff_ok"] is True


def test_relaxed_requires_dz_non_negative():
    """relaxed corridor requires dz_obj >= 0 (TCP above object)."""
    out = build_approach_coarse_phase_check(
        **_baseline(
            handoff_dist_tol=0.01,
            relaxed_handoff_dist_tol=0.10,
            gate_metrics={
                "tcp_base": (0.5, 0.5, -0.01),  # below object → dz < 0
                "object_base": (0.5, 0.5, 0.0),
                "ok": False,
            },
            gate_xy_err=0.005,
        )
    )
    # dz = -0.01 < 0 → relaxed fails
    assert out["relaxed_handoff_ok"] is False


def test_relaxed_caps_dz_max():
    out = build_approach_coarse_phase_check(
        **_baseline(
            handoff_dz_tol=0.01,
            relaxed_handoff_dz_tol=0.05,
            gate_metrics={
                "tcp_base": (0.5, 0.5, 0.10),  # dz=0.10 > relaxed_dz=0.05
                "object_base": (0.5, 0.5, 0.0),
                "ok": True,
            },
        )
    )
    assert out["relaxed_handoff_ok"] is False


# ---------------------------------------------------------------------------
# Block reasons compounding
# ---------------------------------------------------------------------------


def test_multiple_block_reasons_when_all_fail():
    out = build_approach_coarse_phase_check(
        **_baseline(
            handoff_dist_tol=0.01,
            handoff_dz_tol=0.01,
            relaxed_handoff_dist_tol=0.005,  # also fail relaxed
            gate_metrics={
                "tcp_base": (0.5, 0.5, 0.05),
                "object_base": (0.5, 0.5, 0.0),
                "ok": False,
            },
        )
    )
    assert "phase_gate_not_ready" in out["block_reasons"]
    assert "tcp_obj_dist_exceeded" in out["block_reasons"]
    assert "dz_obj_exceeded" in out["block_reasons"]
