#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_directo_gate_evaluator.py
# Contenido: Tests unitarios de directo_gate_evaluator.py (puras, sin ROS/Qt).
# Uso breve: pytest test/test_directo_gate_evaluator.py
"""Unit tests for directo_gate_evaluator pure helpers."""
from __future__ import annotations

import math

import pytest

from ur5_qt_panel.directo_gate_evaluator import (
    _build_transport_seed_candidates,
    _coerce_ur5_joint_vector,
    _direct_pregrasp_gate_caps,
    _evaluate_transport_stage_postcheck,
    _evaluate_transport_stage_preexec_model_guard,
    _normalize_joint_goal_for_execution,
    _normalize_joint_goal_near_seed,
    _resolve_joint_goal_normalization_seed,
    _should_apply_global_step_timeout_extra,
    _should_transport_prep_failure_jump_to_replan,
    _transport_prep_failure_policy,
)

_SIX_ZEROS = [0.0] * 6
_SIX_ONES = [1.0] * 6


# ---------------------------------------------------------------------------
# _coerce_ur5_joint_vector
# ---------------------------------------------------------------------------

class TestCoerceUr5JointVector:
    def test_valid_six_floats(self):
        result = _coerce_ur5_joint_vector([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        assert result == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    def test_valid_six_ints(self):
        result = _coerce_ur5_joint_vector([0, 0, 0, 0, 0, 0])
        assert result == pytest.approx(_SIX_ZEROS)
        assert all(isinstance(v, float) for v in result)

    def test_too_few_joints_returns_none(self):
        assert _coerce_ur5_joint_vector([1.0, 2.0, 3.0]) is None

    def test_too_many_joints_returns_none(self):
        assert _coerce_ur5_joint_vector([0.0] * 7) is None

    def test_non_numeric_returns_none(self):
        assert _coerce_ur5_joint_vector(["a"] * 6) is None

    def test_nan_returns_none(self):
        vals = [0.0] * 5 + [float("nan")]
        assert _coerce_ur5_joint_vector(vals) is None

    def test_inf_returns_none(self):
        vals = [0.0] * 5 + [float("inf")]
        assert _coerce_ur5_joint_vector(vals) is None


# ---------------------------------------------------------------------------
# _normalize_joint_goal_near_seed
# ---------------------------------------------------------------------------

class TestNormalizeJointGoalNearSeed:
    def test_already_near_seed(self):
        goal = [0.1, 0.2, 0.3, -0.1, -0.2, -0.3]
        seed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        result = _normalize_joint_goal_near_seed(goal, seed)
        assert result == pytest.approx(goal)

    def test_wraps_positive_by_2pi(self):
        two_pi = 2 * math.pi
        goal = [two_pi + 0.5] + [0.0] * 5
        seed = [0.0] * 6
        result = _normalize_joint_goal_near_seed(goal, seed)
        assert result[0] == pytest.approx(0.5)

    def test_wraps_negative_by_2pi(self):
        two_pi = 2 * math.pi
        goal = [-(two_pi) + 0.5] + [0.0] * 5
        seed = [0.0] * 6
        result = _normalize_joint_goal_near_seed(goal, seed)
        assert result[0] == pytest.approx(0.5)

    def test_length_preserved(self):
        result = _normalize_joint_goal_near_seed(_SIX_ONES, _SIX_ZEROS)
        assert len(result) == 6


# ---------------------------------------------------------------------------
# _resolve_joint_goal_normalization_seed
# ---------------------------------------------------------------------------

class TestResolveJointGoalNormalizationSeed:
    def test_non_transport_prefers_fallback(self):
        fallback = [0.1] * 6
        retry = [0.9] * 6
        result = _resolve_joint_goal_normalization_seed(
            label="APPROACH_COARSE",
            fallback_seed=fallback,
            retry_seed=retry,
        )
        assert result == pytest.approx(fallback)

    def test_transport_prefers_retry_seed(self):
        fallback = [0.1] * 6
        retry = [0.9] * 6
        result = _resolve_joint_goal_normalization_seed(
            label="CESTA_STAGE_1",
            fallback_seed=fallback,
            retry_seed=retry,
        )
        assert result == pytest.approx(retry)

    def test_transport_falls_back_when_no_retry(self):
        fallback = [0.2] * 6
        result = _resolve_joint_goal_normalization_seed(
            label="CESTA_STAGE_2",
            fallback_seed=fallback,
            retry_seed=None,
        )
        assert result == pytest.approx(fallback)

    def test_raises_when_no_seed_available(self):
        with pytest.raises(ValueError, match="unavailable"):
            _resolve_joint_goal_normalization_seed(
                label="GRASP_DOWN_JOINT",
                fallback_seed=None,
                retry_seed=None,
            )


# ---------------------------------------------------------------------------
# _normalize_joint_goal_for_execution
# ---------------------------------------------------------------------------

class TestNormalizeJointGoalForExecution:
    def test_returns_normalized_goal(self):
        goal = [2 * math.pi + 0.1] + [0.0] * 5
        seed = [0.0] * 6
        result = _normalize_joint_goal_for_execution(
            label="APPROACH_COARSE",
            joint_goal=goal,
            fallback_seed=seed,
        )
        assert result[0] == pytest.approx(0.1)


# ---------------------------------------------------------------------------
# _should_apply_global_step_timeout_extra
# ---------------------------------------------------------------------------

class TestShouldApplyGlobalStepTimeoutExtra:
    def test_false_when_not_requested(self):
        assert _should_apply_global_step_timeout_extra("APPROACH_COARSE", requested=False) is False

    def test_false_for_transport_stage(self):
        assert _should_apply_global_step_timeout_extra("CESTA_STAGE_1", requested=True) is False

    def test_false_for_cesta_release(self):
        assert _should_apply_global_step_timeout_extra("CESTA_RELEASE", requested=True) is False

    def test_true_for_non_transport(self):
        assert _should_apply_global_step_timeout_extra("APPROACH_COARSE", requested=True) is True

    def test_true_for_grasp_down(self):
        assert _should_apply_global_step_timeout_extra("GRASP_DOWN_JOINT", requested=True) is True


# ---------------------------------------------------------------------------
# _build_transport_seed_candidates
# ---------------------------------------------------------------------------

class TestBuildTransportSeedCandidates:
    def test_base_seed_always_first(self):
        base = [0.1] * 6
        result = _build_transport_seed_candidates(base_seed=base)
        assert result[0][0] == pytest.approx(base)
        assert result[0][1] == "base_seed"

    def test_duplicates_deduplicated(self):
        base = [0.1] * 6
        result = _build_transport_seed_candidates(
            base_seed=base,
            live_seed=base,
            last_transport_joint_goal=base,
        )
        sources = [r[1] for r in result]
        assert sources.count("base_seed") == 1

    def test_invalid_live_seed_skipped(self):
        base = [0.0] * 6
        result = _build_transport_seed_candidates(
            base_seed=base,
            live_seed=["bad"] * 6,
        )
        sources = [r[1] for r in result]
        assert "live_joints" not in sources

    def test_returns_list_of_tuples(self):
        result = _build_transport_seed_candidates(base_seed=_SIX_ZEROS)
        assert isinstance(result, list)
        for joints, source in result:
            assert isinstance(joints, list)
            assert isinstance(source, str)


# ---------------------------------------------------------------------------
# _evaluate_transport_stage_preexec_model_guard
# ---------------------------------------------------------------------------

class TestEvaluateTransportStagePreexecModelGuard:
    def _fk_identity(self, joints):
        return joints[:3], None

    def test_non_transport_stage_skipped(self):
        result = _evaluate_transport_stage_preexec_model_guard(
            label="APPROACH_COARSE",
            target_ik=[0.0, 0.0, 0.0],
            joint_goal=_SIX_ZEROS,
            tol_m=0.010,
        )
        assert result["ok"] is True
        assert result["reason"] == "skipped_non_transport_stage"

    def test_transport_ok_when_fk_matches_target(self):
        target = [0.5, 0.3, 0.2]
        # fk returns the first 3 joints as xyz — craft joints matching target
        joints = target + [0.0, 0.0, 0.0]
        result = _evaluate_transport_stage_preexec_model_guard(
            label="CESTA_STAGE_1",
            target_ik=target,
            joint_goal=joints,
            tol_m=0.010,
            fk_fn=self._fk_identity,
        )
        assert result["ok"] is True

    def test_transport_fail_when_fk_far_from_target(self):
        target = [0.5, 0.3, 0.2]
        joints = [10.0, 10.0, 10.0, 0.0, 0.0, 0.0]
        result = _evaluate_transport_stage_preexec_model_guard(
            label="CESTA_STAGE_2",
            target_ik=target,
            joint_goal=joints,
            tol_m=0.010,
            fk_fn=self._fk_identity,
        )
        assert result["ok"] is False

    def test_invalid_joint_goal_returns_false(self):
        result = _evaluate_transport_stage_preexec_model_guard(
            label="CESTA_STAGE_1",
            target_ik=[0.0, 0.0, 0.0],
            joint_goal=["bad"] * 6,
            tol_m=0.010,
        )
        assert result["ok"] is False
        assert "invalid_input" in result["reason"]

    def test_fk_exception_returns_false(self):
        def bad_fk(joints):
            raise RuntimeError("fk_broke")

        result = _evaluate_transport_stage_preexec_model_guard(
            label="CESTA_STAGE_1",
            target_ik=[0.0, 0.0, 0.0],
            joint_goal=_SIX_ZEROS,
            tol_m=0.010,
            fk_fn=bad_fk,
        )
        assert result["ok"] is False
        assert "fk_error" in result["reason"]


# ---------------------------------------------------------------------------
# _direct_pregrasp_gate_caps
# ---------------------------------------------------------------------------

class TestDirectPregraspGateCaps:
    def test_approach_coarse_returns_dict(self):
        result = _direct_pregrasp_gate_caps("APPROACH_COARSE")
        assert isinstance(result, dict)
        assert "source_tol_m" in result
        assert "phase_jump_tol_m" in result

    def test_grasp_down_joint_returns_dict(self):
        result = _direct_pregrasp_gate_caps("GRASP_DOWN_JOINT")
        assert isinstance(result, dict)

    def test_other_phase_returns_none(self):
        assert _direct_pregrasp_gate_caps("CESTA_STAGE_1") is None
        assert _direct_pregrasp_gate_caps(None) is None

    def test_caps_are_positive(self):
        caps = _direct_pregrasp_gate_caps("APPROACH_COARSE")
        for key, val in caps.items():
            assert val > 0.0, f"{key} must be positive"


# ---------------------------------------------------------------------------
# _should_transport_prep_failure_jump_to_replan
# ---------------------------------------------------------------------------

class TestShouldTransportPrepFailureJumpToReplan:
    def test_false_when_residuals_none(self):
        assert _should_transport_prep_failure_jump_to_replan(
            failed_segment_index=5,
            total_segments=5,
            max_joint_residual_rad=None,
            shoulder_joint_residual_rad=None,
            min_failed_segment_fraction=0.5,
            max_joint_residual_threshold_rad=0.1,
            shoulder_joint_residual_threshold_rad=0.05,
        ) is False

    def test_true_when_all_conditions_met(self):
        assert _should_transport_prep_failure_jump_to_replan(
            failed_segment_index=5,
            total_segments=5,
            max_joint_residual_rad=0.5,
            shoulder_joint_residual_rad=0.2,
            min_failed_segment_fraction=0.5,
            max_joint_residual_threshold_rad=0.1,
            shoulder_joint_residual_threshold_rad=0.05,
        ) is True

    def test_false_when_failed_fraction_below_threshold(self):
        assert _should_transport_prep_failure_jump_to_replan(
            failed_segment_index=1,
            total_segments=10,
            max_joint_residual_rad=0.5,
            shoulder_joint_residual_rad=0.5,
            min_failed_segment_fraction=0.8,
            max_joint_residual_threshold_rad=0.1,
            shoulder_joint_residual_threshold_rad=0.05,
        ) is False

    def test_false_when_residuals_below_threshold(self):
        assert _should_transport_prep_failure_jump_to_replan(
            failed_segment_index=5,
            total_segments=5,
            max_joint_residual_rad=0.001,
            shoulder_joint_residual_rad=0.001,
            min_failed_segment_fraction=0.5,
            max_joint_residual_threshold_rad=0.1,
            shoulder_joint_residual_threshold_rad=0.05,
        ) is False


# ---------------------------------------------------------------------------
# _evaluate_transport_stage_postcheck
# ---------------------------------------------------------------------------

class TestEvaluateTransportStagePostcheck:
    def test_ok_when_runtime_target_ok(self):
        result = _evaluate_transport_stage_postcheck(
            label="CESTA_STAGE_1",
            runtime_target_ok=True,
            runtime_target_dist_m=0.005,
            runtime_target_tol_m=0.020,
            model_target_err_m=0.005,
            model_target_tol_m=0.020,
        )
        assert result["ok"] is True

    def test_fail_when_runtime_target_false(self):
        result = _evaluate_transport_stage_postcheck(
            label="CESTA_STAGE_1",
            runtime_target_ok=False,
            runtime_target_dist_m=0.050,
            runtime_target_tol_m=0.020,
            model_target_err_m=0.005,
            model_target_tol_m=0.020,
        )
        assert result["ok"] is False
        assert "runtime_target_dist" in result["reason"]

    def test_fail_when_model_error_high(self):
        result = _evaluate_transport_stage_postcheck(
            label="CESTA_STAGE_1",
            runtime_target_ok=True,
            runtime_target_dist_m=0.005,
            runtime_target_tol_m=0.020,
            model_target_err_m=0.100,
            model_target_tol_m=0.020,
        )
        assert result["ok"] is False
        assert "model_target_err" in result["reason"]

    def test_recover_stage_bypasses_model_guard(self):
        result = _evaluate_transport_stage_postcheck(
            label="CESTA_STAGE_1_RECOVER_2",
            runtime_target_ok=True,
            runtime_target_dist_m=0.005,
            runtime_target_tol_m=0.020,
            model_target_err_m=0.100,
            model_target_tol_m=0.020,
            joint_accept_source="runtime_target",
        )
        assert result["ok"] is True
        assert result["model_target_bypassed"] is True

    def test_unconfirmed_runtime_target_reported(self):
        result = _evaluate_transport_stage_postcheck(
            label="CESTA_STAGE_1",
            runtime_target_ok=False,
            runtime_target_dist_m=None,
            runtime_target_tol_m=0.020,
            model_target_err_m=0.005,
            model_target_tol_m=0.020,
        )
        assert result["ok"] is False
        assert "unconfirmed" in result["reason"]


# ---------------------------------------------------------------------------
# _transport_prep_failure_policy
# ---------------------------------------------------------------------------

class TestTransportPrepFailurePolicy:
    def test_strict_returns_raise(self):
        assert _transport_prep_failure_policy(strict_mode=True) == "raise"

    def test_non_strict_returns_continue(self):
        assert _transport_prep_failure_policy(strict_mode=False) == "continue_final_stage"


# ---------------------------------------------------------------------------
# Additional coverage: uncovered branches
# ---------------------------------------------------------------------------

class TestResolveJointGoalNormalizationSeedFallback:
    def test_retry_used_when_fallback_none_and_non_transport(self):
        # fallback=None, retry=valid, non-transport label → retry_seed_list returned (line 68)
        retry = [0.5] * 6
        result = _resolve_joint_goal_normalization_seed(
            label="GRASP_DOWN_JOINT",
            fallback_seed=None,
            retry_seed=retry,
        )
        assert result == pytest.approx(retry)


class TestBuildTransportSeedCandidatesAllNone:
    def test_all_none_seeds_returns_early_with_empty_list(self):
        # All coerce() calls return None → corridor_seed is None → return candidates (line 119)
        result = _build_transport_seed_candidates(base_seed=None)
        assert result == []


class TestEvaluateTransportStagePreexecModelGuardFkInvalid:
    def test_fk_returns_none_target_is_fk_invalid(self):
        # fk_fn returns (None, None) → _pick_demo_tuple3(None) = None → fk_invalid (line 176)
        result = _evaluate_transport_stage_preexec_model_guard(
            label="CESTA_STAGE_1",
            target_ik=[0.0, 0.0, 0.0],
            joint_goal=_SIX_ZEROS,
            tol_m=0.010,
            fk_fn=lambda joints: (None, None),
        )
        assert result["ok"] is False
        assert result["reason"] == "fk_invalid"
