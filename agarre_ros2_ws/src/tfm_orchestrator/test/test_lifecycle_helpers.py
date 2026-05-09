#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_lifecycle_helpers.py
# Contenido: F9 — tests offline de lifecycle_helpers.
"""Tests offline de lifecycle_helpers."""

from __future__ import annotations

import pytest

from tfm_orchestrator.lifecycle_helpers import (
    LifecycleState,
    LifecycleTransition,
    OrchestratorLifecycleResources,
    allowed_transitions,
    can_transition,
    next_state,
    reject_reason_for_state,
)


# ---------------------------------------------------------------------------
# allowed_transitions
# ---------------------------------------------------------------------------


def test_unconfigured_allows_configure_and_shutdown():
    allowed = allowed_transitions(LifecycleState.UNCONFIGURED)
    assert allowed == {LifecycleTransition.CONFIGURE, LifecycleTransition.SHUTDOWN}


def test_inactive_allows_activate_cleanup_shutdown():
    allowed = allowed_transitions(LifecycleState.INACTIVE)
    assert allowed == {
        LifecycleTransition.ACTIVATE,
        LifecycleTransition.CLEANUP,
        LifecycleTransition.SHUTDOWN,
    }


def test_active_allows_deactivate_and_shutdown():
    allowed = allowed_transitions(LifecycleState.ACTIVE)
    assert allowed == {LifecycleTransition.DEACTIVATE, LifecycleTransition.SHUTDOWN}


def test_finalized_allows_no_transitions():
    assert allowed_transitions(LifecycleState.FINALIZED) == set()


# ---------------------------------------------------------------------------
# can_transition
# ---------------------------------------------------------------------------


def test_can_transition_valid():
    assert can_transition(LifecycleState.UNCONFIGURED, LifecycleTransition.CONFIGURE)


def test_can_transition_invalid():
    assert not can_transition(
        LifecycleState.UNCONFIGURED, LifecycleTransition.ACTIVATE
    )


def test_can_transition_finalized_anything_false():
    for t in LifecycleTransition:
        assert not can_transition(LifecycleState.FINALIZED, t)


# ---------------------------------------------------------------------------
# next_state
# ---------------------------------------------------------------------------


def test_next_state_configure():
    assert (
        next_state(LifecycleState.UNCONFIGURED, LifecycleTransition.CONFIGURE)
        == LifecycleState.INACTIVE
    )


def test_next_state_activate():
    assert (
        next_state(LifecycleState.INACTIVE, LifecycleTransition.ACTIVATE)
        == LifecycleState.ACTIVE
    )


def test_next_state_deactivate():
    assert (
        next_state(LifecycleState.ACTIVE, LifecycleTransition.DEACTIVATE)
        == LifecycleState.INACTIVE
    )


def test_next_state_cleanup():
    assert (
        next_state(LifecycleState.INACTIVE, LifecycleTransition.CLEANUP)
        == LifecycleState.UNCONFIGURED
    )


def test_next_state_shutdown_from_any_managed():
    for state in (
        LifecycleState.UNCONFIGURED,
        LifecycleState.INACTIVE,
        LifecycleState.ACTIVE,
    ):
        assert (
            next_state(state, LifecycleTransition.SHUTDOWN) == LifecycleState.FINALIZED
        )


def test_next_state_invalid_raises():
    with pytest.raises(ValueError) as exc:
        next_state(LifecycleState.UNCONFIGURED, LifecycleTransition.ACTIVATE)
    assert "transition_not_allowed" in str(exc.value)
    assert "from=unconfigured" in str(exc.value)
    assert "via=activate" in str(exc.value)


def test_next_state_finalized_any_raises():
    with pytest.raises(ValueError):
        next_state(LifecycleState.FINALIZED, LifecycleTransition.CONFIGURE)


# ---------------------------------------------------------------------------
# OrchestratorLifecycleResources
# ---------------------------------------------------------------------------


def test_resources_default_empty():
    r = OrchestratorLifecycleResources()
    assert not r.is_configured()
    assert not r.is_activated()


def test_resources_is_configured_after_setup():
    r = OrchestratorLifecycleResources(
        has_action_server=True,
        has_service_map=True,
        params_loaded=True,
    )
    assert r.is_configured()
    assert not r.is_activated()  # still need accepts_goals


def test_resources_is_activated_after_full_setup():
    r = OrchestratorLifecycleResources(
        has_action_server=True,
        has_service_map=True,
        params_loaded=True,
        accepts_goals=True,
    )
    assert r.is_activated()


def test_configure_invariant_missing_action_server():
    r = OrchestratorLifecycleResources(has_service_map=True, params_loaded=True)
    assert r.configure_invariant() == "action_server_not_created"


def test_configure_invariant_missing_service_map():
    r = OrchestratorLifecycleResources(has_action_server=True, params_loaded=True)
    assert r.configure_invariant() == "service_map_not_initialized"


def test_configure_invariant_missing_params():
    r = OrchestratorLifecycleResources(has_action_server=True, has_service_map=True)
    assert r.configure_invariant() == "params_not_loaded"


def test_configure_invariant_should_not_accept_yet():
    r = OrchestratorLifecycleResources(
        has_action_server=True,
        has_service_map=True,
        params_loaded=True,
        accepts_goals=True,
    )
    assert r.configure_invariant() == "should_not_accept_goals_yet"


def test_configure_invariant_ok_when_complete():
    r = OrchestratorLifecycleResources(
        has_action_server=True,
        has_service_map=True,
        params_loaded=True,
        accepts_goals=False,
    )
    assert r.configure_invariant() is None


def test_activate_invariant_not_configured():
    r = OrchestratorLifecycleResources(accepts_goals=True)
    assert r.activate_invariant() == "not_configured"


def test_activate_invariant_accepts_goals_disabled():
    r = OrchestratorLifecycleResources(
        has_action_server=True,
        has_service_map=True,
        params_loaded=True,
        accepts_goals=False,
    )
    assert r.activate_invariant() == "accepts_goals_not_enabled"


def test_activate_invariant_ok():
    r = OrchestratorLifecycleResources(
        has_action_server=True,
        has_service_map=True,
        params_loaded=True,
        accepts_goals=True,
    )
    assert r.activate_invariant() is None


def test_deactivate_invariant_still_accepting():
    r = OrchestratorLifecycleResources(
        has_action_server=True,
        has_service_map=True,
        params_loaded=True,
        accepts_goals=True,
    )
    assert r.deactivate_invariant() == "still_accepting_goals"


def test_deactivate_invariant_ok():
    r = OrchestratorLifecycleResources(
        has_action_server=True,
        has_service_map=True,
        params_loaded=True,
        accepts_goals=False,
    )
    assert r.deactivate_invariant() is None


def test_cleanup_invariant_still_present():
    r = OrchestratorLifecycleResources(has_action_server=True)
    assert r.cleanup_invariant() == "action_server_still_present"


def test_cleanup_invariant_ok_when_empty():
    r = OrchestratorLifecycleResources()
    assert r.cleanup_invariant() is None


# ---------------------------------------------------------------------------
# reject_reason_for_state
# ---------------------------------------------------------------------------


def test_reject_reason_active_returns_none():
    assert reject_reason_for_state(LifecycleState.ACTIVE) is None


def test_reject_reason_inactive():
    assert reject_reason_for_state(LifecycleState.INACTIVE) == "node_not_active:inactive"


def test_reject_reason_unconfigured():
    assert (
        reject_reason_for_state(LifecycleState.UNCONFIGURED)
        == "node_not_active:unconfigured"
    )


def test_reject_reason_finalized():
    assert (
        reject_reason_for_state(LifecycleState.FINALIZED)
        == "node_not_active:finalized"
    )
