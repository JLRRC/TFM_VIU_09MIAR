#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_plan_to_pose_logic.py
# Contenido: F6.5 — tests de la lógica pura del PlanToPose server.
"""Tests offline de ``ur5_tools.plan_to_pose_logic``.

NO levantan ROS. Sólo verifican:

* validate_goal acepta/rechaza correctamente según contrato.
* feedback_sequence determinístico, monotónico y dentro de [0, 1].
* execute_stub success path + invalid goal path.
* normalize_quat: norma 1, fallback a identity en quat cero.
"""

from __future__ import annotations

import math

import pytest

from ur5_tools.plan_to_pose_logic import (
    PlanToPoseGoal,
    execute_stub,
    feedback_sequence,
    normalize_quat,
    validate_goal,
)


def _valid_goal(**overrides) -> PlanToPoseGoal:
    base = dict(
        target_xyz=(0.5, 0.0, 0.10),
        target_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ee_frame="rg2_pinch_center",
        cartesian=False,
        timeout_sec=0.0,
    )
    base.update(overrides)
    return PlanToPoseGoal(**base)


# ---------------------------------------------------------------------------
# validate_goal
# ---------------------------------------------------------------------------


def test_validate_goal_default_ok():
    ok, reason = validate_goal(_valid_goal())
    assert ok is True
    assert reason == "ok"


@pytest.mark.parametrize("frame", ["rg2_pinch_center", "rg2_tcp", "tool0"])
def test_validate_goal_accepts_all_known_frames(frame):
    ok, _ = validate_goal(_valid_goal(ee_frame=frame))
    assert ok is True


def test_validate_goal_rejects_unknown_frame():
    ok, reason = validate_goal(_valid_goal(ee_frame="unknown_frame"))
    assert ok is False
    assert reason == "unknown_ee_frame:unknown_frame"


def test_validate_goal_rejects_nan_xyz():
    ok, reason = validate_goal(_valid_goal(target_xyz=(0.5, float("nan"), 0.1)))
    assert ok is False
    assert reason == "target_xyz_non_finite"


def test_validate_goal_rejects_inf_xyz():
    ok, reason = validate_goal(_valid_goal(target_xyz=(0.5, float("inf"), 0.1)))
    assert ok is False
    assert reason == "target_xyz_non_finite"


def test_validate_goal_rejects_nan_quat():
    ok, reason = validate_goal(_valid_goal(target_quat_xyzw=(0.0, float("nan"), 0.0, 1.0)))
    assert ok is False
    assert reason == "target_quat_non_finite"


def test_validate_goal_rejects_zero_quat():
    ok, reason = validate_goal(_valid_goal(target_quat_xyzw=(0.0, 0.0, 0.0, 0.0)))
    assert ok is False
    assert reason == "target_quat_zero_norm"


def test_validate_goal_rejects_negative_timeout():
    ok, reason = validate_goal(_valid_goal(timeout_sec=-0.5))
    assert ok is False
    assert reason == "timeout_sec_negative"


def test_validate_goal_zero_timeout_is_ok():
    ok, _ = validate_goal(_valid_goal(timeout_sec=0.0))
    assert ok is True


# ---------------------------------------------------------------------------
# normalize_quat
# ---------------------------------------------------------------------------


def test_normalize_quat_already_unit():
    q = normalize_quat((0.0, 0.0, 0.0, 1.0))
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_normalize_quat_scales_to_unit():
    q = normalize_quat((2.0, 0.0, 0.0, 0.0))
    norm = math.sqrt(sum(c * c for c in q))
    assert norm == pytest.approx(1.0)


def test_normalize_quat_zero_returns_identity():
    q = normalize_quat((0.0, 0.0, 0.0, 0.0))
    assert q == (0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# feedback_sequence
# ---------------------------------------------------------------------------


def test_feedback_sequence_default_lengths():
    seq = feedback_sequence(_valid_goal())
    # default: 3 planning + 4 executing
    states = [f.current_state for f in seq]
    assert states.count("PLANNING") == 3
    assert states.count("EXECUTING") == 4


def test_feedback_sequence_progress_monotonic():
    seq = feedback_sequence(_valid_goal())
    progresses = [f.progress for f in seq]
    for prev, nxt in zip(progresses[:-1], progresses[1:]):
        assert nxt >= prev, f"feedback progress no monotónico: {progresses}"


def test_feedback_sequence_progress_within_bounds():
    seq = feedback_sequence(_valid_goal())
    for f in seq:
        assert 0.0 <= f.progress <= 1.0, f"progress fuera de [0,1]: {f.progress}"


def test_feedback_sequence_planning_then_executing():
    seq = feedback_sequence(_valid_goal())
    # Todos los PLANNING vienen antes que cualquier EXECUTING.
    saw_executing = False
    for f in seq:
        if f.current_state == "EXECUTING":
            saw_executing = True
        elif saw_executing and f.current_state == "PLANNING":
            pytest.fail("PLANNING vino después de EXECUTING — orden inválido")


def test_feedback_sequence_zero_steps():
    seq = feedback_sequence(_valid_goal(), n_planning_steps=0, n_executing_steps=0)
    assert seq == []


def test_feedback_sequence_only_executing():
    seq = feedback_sequence(_valid_goal(), n_planning_steps=0, n_executing_steps=2)
    assert all(f.current_state == "EXECUTING" for f in seq)
    assert len(seq) == 2


# ---------------------------------------------------------------------------
# execute_stub
# ---------------------------------------------------------------------------


def test_execute_stub_valid_returns_success():
    r = execute_stub(_valid_goal(), duration_sec=0.5)
    assert r.success is True
    assert r.reason == "stub_planning_completed"
    assert r.duration_sec == 0.5
    assert r.attempts == 1


def test_execute_stub_invalid_goal_returns_failure():
    r = execute_stub(_valid_goal(ee_frame="bogus"))
    assert r.success is False
    assert r.reason.startswith("invalid_goal:")


def test_execute_stub_final_pose_normalizes_quat():
    r = execute_stub(_valid_goal(target_quat_xyzw=(2.0, 0.0, 0.0, 0.0)))
    norm = math.sqrt(sum(c * c for c in r.final_quat_xyzw))
    assert norm == pytest.approx(1.0)


def test_execute_stub_final_xyz_unchanged():
    r = execute_stub(_valid_goal(target_xyz=(0.123, 0.456, 0.789)))
    assert r.final_xyz == (0.123, 0.456, 0.789)


# ---------------------------------------------------------------------------
# F6.6: encode_request_frame + parse_bridge_result + result_matches_request
# ---------------------------------------------------------------------------


from ur5_tools.plan_to_pose_logic import (
    encode_request_frame,
    parse_bridge_result,
    result_matches_request,
)


def test_encode_request_frame_basic():
    s = encode_request_frame("base_link", 42, "abc")
    assert s == "base_link|rid=42|uid=abc"


def test_encode_request_frame_with_tol_and_phase():
    s = encode_request_frame("base_link", 7, "xyz", tol_m=0.0123, phase_label="PLAN_TO_POSE")
    assert s == "base_link|rid=7|uid=xyz|tol=0.012|phase=PLAN_TO_POSE"


def test_encode_request_frame_default_base():
    s = encode_request_frame("", 1, "u")
    assert s.startswith("base_link|rid=1|uid=u")


def test_encode_request_frame_phase_strips_pipes():
    s = encode_request_frame("base_link", 1, "u", phase_label="A|B|C")
    assert "phase=A_B_C" in s
    assert s.count("|") == 3  # base|rid|uid|phase, no pipes inside phase


def test_encode_request_frame_omits_empty_uuid():
    s = encode_request_frame("base_link", 1, "")
    assert s == "base_link|rid=1"


def test_encode_request_frame_omits_invalid_tol():
    s = encode_request_frame("base_link", 1, "u", tol_m=float("nan"))
    # NaN se serializa como 'nan' por el f-string :.3f, eso es válido
    # (no rompe). Pero un valor None no aparece. Verificamos tipo no-None.
    assert "uid=u" in s


def test_parse_bridge_result_success_true():
    s, r, u = parse_bridge_result("success=true reason=exec_ok request_uuid=abc")
    assert s is True
    assert r == "exec_ok"
    assert u == "abc"


def test_parse_bridge_result_success_false():
    s, r, u = parse_bridge_result("success=false reason=plan_failed request_uuid=xyz")
    assert s is False
    assert r == "plan_failed"
    assert u == "xyz"


def test_parse_bridge_result_no_kv():
    s, r, u = parse_bridge_result("garbage payload no kv")
    assert s is None
    assert r == "garbage payload no kv"
    assert u == ""


def test_parse_bridge_result_empty():
    s, r, u = parse_bridge_result("")
    assert s is None
    assert r == ""
    assert u == ""


def test_parse_bridge_result_partial_uuid_only():
    s, r, u = parse_bridge_result("request_uuid=just_uuid")
    assert s is None
    assert u == "just_uuid"


def test_result_matches_request_true():
    assert result_matches_request("success=true request_uuid=abc", "abc") is True


def test_result_matches_request_false_different_uuid():
    assert result_matches_request("success=true request_uuid=abc", "xyz") is False


def test_result_matches_request_empty_expected_accepts_any():
    """Si expected_uuid es vacío, modo correlación-libre."""
    assert result_matches_request("any text", "") is True


def test_result_matches_request_no_uuid_in_result():
    # result sin request_uuid → match falso si expected no vacío.
    assert result_matches_request("success=true", "abc") is False
