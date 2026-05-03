#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_phase_progress.py
# Contenido: B-iter7 (2026-05-03) — tests de interpolación de progress intra-fase.
"""Tests para tfm_orchestrator.phase_progress."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from tfm_orchestrator.phase_progress import (
    fjt_progress_from_feedback,
    interpolate_phase_progress,
    moveit_progress_from_state,
)
from tfm_orchestrator.pick_fsm import PickPhase, happy_path


# ---------------------------------------------------------------------------
# interpolate_phase_progress
# ---------------------------------------------------------------------------


def test_interpolate_idle_is_zero():
    assert interpolate_phase_progress(PickPhase.IDLE, 0.0) == 0.0
    assert interpolate_phase_progress(PickPhase.IDLE, 1.0) == 0.0


def test_interpolate_done_is_one():
    assert interpolate_phase_progress(PickPhase.DONE, 0.0) == 1.0
    assert interpolate_phase_progress(PickPhase.DONE, 0.5) == 1.0


def test_interpolate_intermediate_phase_at_zero_matches_phase_index():
    # APPROACH es índice 4 en happy path de 9 phases (8 transiciones).
    p = interpolate_phase_progress(PickPhase.APPROACH, 0.0)
    expected = 4.0 / 9.0  # ≈0.444 (10 fases en happy path → 9 transiciones)
    assert p == pytest.approx(expected)


def test_interpolate_intermediate_phase_at_one_matches_next_phase_index():
    # APPROACH (idx 4) sub=1.0 debe igualar GRASP (idx 5) sub=0.0.
    p_approach_full = interpolate_phase_progress(PickPhase.APPROACH, 1.0)
    p_grasp_zero = interpolate_phase_progress(PickPhase.GRASP, 0.0)
    assert p_approach_full == pytest.approx(p_grasp_zero)
    assert p_approach_full == pytest.approx(5.0 / 9.0)


def test_interpolate_intermediate_phase_at_half():
    # APPROACH idx 4, sub=0.5 → entre 4/9 y 5/9 → 0.5
    p = interpolate_phase_progress(PickPhase.APPROACH, 0.5)
    assert p == pytest.approx(0.5)


def test_interpolate_clamps_sub_progress_high():
    p = interpolate_phase_progress(PickPhase.APPROACH, 5.0)
    # >1.0 se clampea a 1.0
    assert p == pytest.approx(5.0 / 9.0)


def test_interpolate_clamps_sub_progress_low():
    p = interpolate_phase_progress(PickPhase.APPROACH, -0.5)
    assert p == pytest.approx(4.0 / 9.0)


def test_interpolate_unknown_phase_returns_zero():
    p = interpolate_phase_progress("UNKNOWN_PHASE_XYZ", 0.5)
    assert p == 0.0


def test_interpolate_invalid_sub_progress_returns_phase_index():
    # NaN/string como sub_progress → cae a 0.0 → progress = phase_index/8.
    p = interpolate_phase_progress(PickPhase.LIFT, "not_a_number")
    assert p == pytest.approx(6.0 / 9.0)


def test_interpolate_failed_phase_falls_back_to_zero():
    # FAILED no está en happy path → fallback 0.
    p = interpolate_phase_progress(PickPhase.FAILED, 0.5)
    assert p == 0.0


# ---------------------------------------------------------------------------
# fjt_progress_from_feedback
# ---------------------------------------------------------------------------


def _make_fjt_feedback(desired, actual):
    return SimpleNamespace(
        feedback=SimpleNamespace(
            desired=SimpleNamespace(positions=list(desired)),
            actual=SimpleNamespace(positions=list(actual)),
        ),
    )


def test_fjt_progress_zero_when_no_feedback():
    assert fjt_progress_from_feedback(None) == 0.0


def test_fjt_progress_one_when_actual_matches_desired():
    fb = _make_fjt_feedback([0.1, 0.2, 0.3], [0.1, 0.2, 0.3])
    assert fjt_progress_from_feedback(fb) == 1.0


def test_fjt_progress_decreases_with_max_error():
    # max_error pequeño → progress alto
    fb_close = _make_fjt_feedback([1.0, 1.0, 1.0], [0.95, 0.99, 1.0])
    p_close = fjt_progress_from_feedback(fb_close)
    # max_error grande → progress bajo
    fb_far = _make_fjt_feedback([1.0, 1.0, 1.0], [0.0, 0.0, 0.0])
    p_far = fjt_progress_from_feedback(fb_far)
    assert p_close > p_far
    assert p_close > 0.9  # casi 1.0 con error de 0.05
    assert p_far < 0.4  # error 1.0 / 1.57 ≈ 0.36 progress


def test_fjt_progress_handles_empty_arrays():
    fb = _make_fjt_feedback([], [])
    assert fjt_progress_from_feedback(fb) == 0.0


def test_fjt_progress_handles_missing_attrs():
    fb = SimpleNamespace(feedback=None)
    assert fjt_progress_from_feedback(fb) == 0.0


# ---------------------------------------------------------------------------
# moveit_progress_from_state
# ---------------------------------------------------------------------------


def test_moveit_progress_planning():
    assert moveit_progress_from_state("PLANNING") == 0.30


def test_moveit_progress_monitor():
    assert moveit_progress_from_state("MONITOR") == 0.70


def test_moveit_progress_idle():
    assert moveit_progress_from_state("IDLE") == 1.0


def test_moveit_progress_unknown_state():
    assert moveit_progress_from_state("WHATEVER") == 0.10


def test_moveit_progress_none_returns_zero():
    assert moveit_progress_from_state(None) == 0.0


def test_moveit_progress_accepts_feedback_wrapper():
    fb = SimpleNamespace(feedback=SimpleNamespace(state="MONITOR"))
    assert moveit_progress_from_state(fb) == 0.70


def test_moveit_progress_accepts_state_attr_object():
    msg = SimpleNamespace(state="PLANNING")
    assert moveit_progress_from_state(msg) == 0.30
