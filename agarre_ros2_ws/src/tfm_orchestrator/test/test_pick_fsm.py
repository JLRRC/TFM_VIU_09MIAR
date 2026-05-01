#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_pick_fsm.py
# Contenido: F5 — tests unitarios del state machine pick & place.
"""Tests del FSM puro (sin ROS).

Cubre:

* Happy path completo IDLE → ... → DONE.
* Transiciones permitidas / no permitidas.
* Fallo desde cada fase no-terminal.
* Cancelación.
* Snapshot de feedback (mapeable a PickPlace.action.Feedback).
* Progreso normalizado.
* Estados terminales no transicionan.
"""

from __future__ import annotations

import pytest

from tfm_orchestrator.pick_fsm import (
    PickContext,
    PickPhase,
    TERMINAL_PHASES,
    allowed_transitions,
    can_transition,
    happy_path,
    phase_index,
    progress_fraction,
)


# ---------------------------------------------------------------------------
# happy path
# ---------------------------------------------------------------------------

def test_happy_path_order():
    path = happy_path()
    assert path[0] == PickPhase.IDLE
    assert path[-1] == PickPhase.DONE
    assert PickPhase.SELECT_OBJECT in path
    assert PickPhase.APPROACH in path
    assert PickPhase.GRASP in path
    assert PickPhase.LIFT in path
    assert PickPhase.TRANSPORT in path
    assert PickPhase.RELEASE in path


def test_happy_path_full_advance():
    ctx = PickContext()
    for dst in happy_path()[1:]:
        ctx.advance(dst)
    assert ctx.current_phase == PickPhase.DONE
    assert ctx.is_terminal()
    assert len(ctx.history) == len(happy_path()) - 1


# ---------------------------------------------------------------------------
# transiciones
# ---------------------------------------------------------------------------

def test_can_transition_to_next_in_happy_path():
    path = happy_path()
    for src, dst in zip(path[:-1], path[1:]):
        assert can_transition(src, dst), f"happy: {src} → {dst} debe ser permitida"


def test_cannot_skip_phases():
    # IDLE no puede saltar a APPROACH directamente.
    assert not can_transition(PickPhase.IDLE, PickPhase.APPROACH)
    # SELECT_OBJECT no puede saltar a GRASP.
    assert not can_transition(PickPhase.SELECT_OBJECT, PickPhase.GRASP)


def test_can_transition_to_failed_from_any_non_terminal():
    for phase in PickPhase:
        if phase in TERMINAL_PHASES:
            continue
        assert can_transition(phase, PickPhase.FAILED), (
            f"{phase} → FAILED debe ser permitida"
        )


def test_can_transition_to_aborted_from_any_non_terminal():
    for phase in PickPhase:
        if phase in TERMINAL_PHASES:
            continue
        assert can_transition(phase, PickPhase.ABORTED), (
            f"{phase} → ABORTED debe ser permitida"
        )


def test_terminal_phases_do_not_transition():
    for phase in TERMINAL_PHASES:
        assert allowed_transitions(phase) == frozenset(), (
            f"{phase} terminal: no debe permitir transiciones"
        )


def test_cannot_go_backwards():
    assert not can_transition(PickPhase.GRASP, PickPhase.APPROACH)
    assert not can_transition(PickPhase.LIFT, PickPhase.IDLE)


# ---------------------------------------------------------------------------
# PickContext
# ---------------------------------------------------------------------------

def test_context_initial_state():
    ctx = PickContext()
    assert ctx.current_phase == PickPhase.IDLE
    assert not ctx.is_terminal()
    assert ctx.history == []


def test_context_advance_records_history():
    ctx = PickContext()
    ctx.advance(PickPhase.SELECT_OBJECT, detail="picked from store")
    assert ctx.current_phase == PickPhase.SELECT_OBJECT
    assert ctx.history == [PickPhase.IDLE]
    assert ctx.detail == "picked from store"


def test_context_invalid_transition_raises():
    ctx = PickContext()
    with pytest.raises(ValueError, match="transición no permitida"):
        ctx.advance(PickPhase.LIFT)


def test_context_fail_shortcut():
    ctx = PickContext()
    ctx.advance(PickPhase.SELECT_OBJECT)
    ctx.fail("object not visible")
    assert ctx.current_phase == PickPhase.FAILED
    assert ctx.is_terminal()
    assert ctx.detail == "object not visible"


def test_context_abort_shortcut():
    ctx = PickContext()
    ctx.advance(PickPhase.SELECT_OBJECT)
    ctx.advance(PickPhase.APPROACH)
    ctx.abort()
    assert ctx.current_phase == PickPhase.ABORTED
    assert ctx.is_terminal()
    assert ctx.detail == "client_canceled"


def test_context_after_terminal_cannot_advance():
    ctx = PickContext()
    ctx.fail("test")
    with pytest.raises(ValueError):
        ctx.advance(PickPhase.SELECT_OBJECT)


# ---------------------------------------------------------------------------
# phase_index + progress_fraction
# ---------------------------------------------------------------------------

def test_phase_index_idle_zero():
    assert phase_index(PickPhase.IDLE) == 0


def test_phase_index_done_last():
    assert phase_index(PickPhase.DONE) == len(happy_path()) - 1


def test_phase_index_failed_aborted_minus_one():
    assert phase_index(PickPhase.FAILED) == -1
    assert phase_index(PickPhase.ABORTED) == -1


def test_progress_idle_zero():
    assert progress_fraction(PickPhase.IDLE) == 0.0


def test_progress_done_one():
    assert progress_fraction(PickPhase.DONE) == 1.0


def test_progress_failed_aborted_zero():
    assert progress_fraction(PickPhase.FAILED) == 0.0
    assert progress_fraction(PickPhase.ABORTED) == 0.0


def test_progress_monotonic_in_happy_path():
    last = -1.0
    for phase in happy_path():
        p = progress_fraction(phase)
        assert p > last, f"progress not monotonic at {phase}"
        last = p


# ---------------------------------------------------------------------------
# feedback snapshot
# ---------------------------------------------------------------------------

def test_feedback_snapshot_shape():
    ctx = PickContext(object_name="box_red")
    ctx.advance(PickPhase.SELECT_OBJECT, detail="ok")
    snap = ctx.feedback_snapshot()
    assert set(snap.keys()) == {"current_phase", "progress", "phase_index", "detail"}
    assert snap["current_phase"] == "SELECT_OBJECT"
    assert isinstance(snap["progress"], float)
    assert isinstance(snap["phase_index"], int)
    assert snap["detail"] == "ok"


def test_feedback_snapshot_at_done():
    ctx = PickContext()
    for dst in happy_path()[1:]:
        ctx.advance(dst, detail=dst.value)
    snap = ctx.feedback_snapshot()
    assert snap["current_phase"] == "DONE"
    assert snap["progress"] == 1.0
    assert snap["phase_index"] == len(happy_path()) - 1
