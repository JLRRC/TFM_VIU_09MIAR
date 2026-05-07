#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_pick_gates.py
# Contenido: B-iter8 (2026-05-03) — tests gates legacy migrados.
"""Tests para tfm_orchestrator.pick_gates."""

from __future__ import annotations


from tfm_orchestrator.pick_gates import (
    DEFAULT_MAX_ATTACH_DIST_M,
    DEFAULT_RELEASE_MIN_OPEN_SUM,
    evaluate_attach_distance_gate,
    evaluate_close_delta_gate,
    evaluate_release_open_gate,
)


# ---------------------------------------------------------------------------
# evaluate_attach_distance_gate
# ---------------------------------------------------------------------------


def test_attach_gate_ok_within_default_limit():
    ok, reason = evaluate_attach_distance_gate(0.030)
    assert ok is True
    assert "ok" in reason
    assert "0.0300" in reason


def test_attach_gate_ok_at_exact_limit():
    ok, reason = evaluate_attach_distance_gate(DEFAULT_MAX_ATTACH_DIST_M)
    assert ok is True


def test_attach_gate_fails_when_too_far():
    ok, reason = evaluate_attach_distance_gate(0.100)
    assert ok is False
    assert "too_far" in reason
    assert "0.1000" in reason


def test_attach_gate_custom_limit():
    ok, reason = evaluate_attach_distance_gate(0.030, max_dist_m=0.020)
    assert ok is False
    assert "too_far" in reason


def test_attach_gate_none_distance_fails():
    ok, reason = evaluate_attach_distance_gate(None)
    assert ok is False
    assert reason == "attach_distance:unmeasured"


def test_attach_gate_invalid_value_fails():
    ok, reason = evaluate_attach_distance_gate("not_a_number")  # type: ignore[arg-type]
    assert ok is False
    assert "invalid_value" in reason


def test_attach_gate_simulates_panel_drop_anchor_placebo():
    """Caso real del bug: backend retorna success=True pero TCP a >1m del objeto.
    El gate detecta el placebo."""
    ok, reason = evaluate_attach_distance_gate(1.093)  # del log live
    assert ok is False
    assert "too_far" in reason


# ---------------------------------------------------------------------------
# evaluate_close_delta_gate
# ---------------------------------------------------------------------------


def test_close_gate_ok_when_delta_above_min():
    ok, reason = evaluate_close_delta_gate(0.050, 0.040)
    # delta = 0.010 > 0.005 (min)
    assert ok is True
    assert "delta=0.0100" in reason


def test_close_gate_ok_via_fallback_when_no_before():
    ok, reason = evaluate_close_delta_gate(None, 0.015)
    # opening_after 0.015 <= 0.020 (fallback_max) → ok
    assert ok is True
    assert "fallback" in reason


def test_close_gate_ok_via_fallback_with_before():
    """Si el delta es pequeño pero opening_after está bajo el fallback,
    también pasa (doble criterio)."""
    ok, reason = evaluate_close_delta_gate(0.018, 0.015)
    # delta=0.003 < 0.005 fail, fallback opening=0.015 <= 0.020 ok
    assert ok is True
    assert "fallback" in reason


def test_close_gate_fail_when_both_criteria_fail():
    ok, reason = evaluate_close_delta_gate(0.060, 0.058)
    # delta=0.002 < 0.005, opening_after=0.058 > 0.020 → fail
    assert ok is False
    assert "fail" in reason
    assert "0.0020" in reason  # delta value


def test_close_gate_fail_no_before_and_opening_too_high():
    ok, reason = evaluate_close_delta_gate(None, 0.050)
    # No before, opening > fallback → fail
    assert ok is False
    assert "no_before" in reason


def test_close_gate_fail_no_after():
    ok, reason = evaluate_close_delta_gate(0.050, None)
    assert ok is False
    assert reason == "close_delta:no_after_measurement"


def test_close_gate_invalid_after():
    ok, reason = evaluate_close_delta_gate(0.050, "bad")  # type: ignore[arg-type]
    assert ok is False
    assert "invalid_after" in reason


def test_close_gate_custom_min_delta():
    ok, reason = evaluate_close_delta_gate(
        0.050, 0.040, min_delta_sum=0.020,
    )
    # delta=0.010 < 0.020 fail, opening=0.040 > fallback default 0.020 fail
    assert ok is False


# ---------------------------------------------------------------------------
# evaluate_release_open_gate
# ---------------------------------------------------------------------------


def test_release_gate_ok_when_open_enough():
    ok, reason = evaluate_release_open_gate(0.050)
    assert ok is True
    assert "ok" in reason


def test_release_gate_ok_at_exact_limit():
    ok, reason = evaluate_release_open_gate(DEFAULT_RELEASE_MIN_OPEN_SUM)
    assert ok is True


def test_release_gate_fail_when_not_open_enough():
    ok, reason = evaluate_release_open_gate(0.010)
    assert ok is False
    assert "fail" in reason
    assert "0.0100" in reason


def test_release_gate_none_fails():
    ok, reason = evaluate_release_open_gate(None)
    assert ok is False
    assert reason == "release_open:no_measurement"


def test_release_gate_invalid_value():
    ok, reason = evaluate_release_open_gate("bad")  # type: ignore[arg-type]
    assert ok is False
    assert "invalid_value" in reason


def test_release_gate_custom_min_open():
    ok, reason = evaluate_release_open_gate(0.020, min_open_sum=0.015)
    assert ok is True
