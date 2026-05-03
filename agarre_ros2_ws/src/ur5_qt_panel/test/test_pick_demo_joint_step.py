"""F3-step3e: tests smoke del run_joint_step extraído."""

from __future__ import annotations

import inspect
from unittest.mock import MagicMock

import pytest

from ur5_qt_panel.pick_demo.joint_step import (
    JointStepContext,
    run_joint_step,
)


def test_joint_step_context_constructs_with_all_required_fields():
    panel = MagicMock()
    ctx = JointStepContext(
        panel=panel,
        live_tcp_base=lambda: (0.4, 0.0, 0.10),
        dist=lambda a, b: 0.0,
        append_trace=MagicMock(),
        move_sec=2.0,
        tuple3=lambda v: tuple(float(c) for c in v[:3]) if v is not None else None,
        fmt_vec=lambda v: "(0,0,0)",
        fmt_scalar=lambda v, *, digits=3: "0.000",
        get_pick_demo_params=MagicMock(),
        direct_runtime_target_tol_m=lambda label: 0.005,
        iso_now=lambda: "2026-05-03T12:00:00Z",
        ur5_joint_names=("a", "b", "c", "d", "e", "f"),
        angle_shortest_diff_rad=lambda a, b: 0.0,
        get_global_step_timeout_extra=lambda: 0.0,
        should_apply_global_step_timeout_extra=lambda label: False,
    )
    assert ctx.panel is panel
    assert callable(ctx.live_tcp_base)
    assert ctx.move_sec == 2.0


def test_run_joint_step_is_callable():
    assert callable(run_joint_step)


def test_run_joint_step_signature_matches_legacy():
    """Firma debe ser idéntica al wrapper para preservar 15 callsites."""
    sig = inspect.signature(run_joint_step)
    expected = [
        "ctx", "label", "joints", "timeout_sec", "tol_rad",
        "runtime_target_base", "runtime_target_tol_m", "force_send",
        "move_sec_override", "apply_step_timeout_extra",
    ]
    assert list(sig.parameters.keys()) == expected
