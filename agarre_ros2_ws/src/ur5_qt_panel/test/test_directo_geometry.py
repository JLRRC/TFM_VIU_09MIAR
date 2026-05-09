#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_directo_geometry.py
# Contenido: Tests unitarios de directo_geometry.py (puras, sin ROS/Qt).
# Uso breve: pytest test/test_directo_geometry.py
"""Unit tests for directo_geometry pure helpers."""
from __future__ import annotations

import math

import pytest

from ur5_qt_panel.directo_geometry import (
    _compute_demo_basket_targets,
    _compute_demo_joint_prep_waypoint,
    _compute_demo_joint_prep_waypoints,
    _compute_demo_linear_stage_targets,
    _compute_demo_stage_count_for_distance,
    _compute_demo_transport_micro_recovery_target,
    _compute_demo_transport_prep_joint_tol,
    _compute_demo_transport_recovery_stage_targets,
    _direct_runtime_target_tol_m,
    _effective_direct_grasp_z,
    _is_demo_basket_transport_motion,
    _is_demo_basket_transport_stage,
    _joint_step_wait_timeout,
    _pick_demo_env_flag,
    _pick_demo_env_float,
    _pick_demo_env_int,
    _pick_demo_fmt_scalar,
    _pick_demo_tuple3,
    angle_shortest_diff_rad,
    pick_demo_target_semantics,
)


# ---------------------------------------------------------------------------
# angle_shortest_diff_rad
# ---------------------------------------------------------------------------

class TestAngleShortestDiffRad:
    def test_zero_diff(self):
        assert angle_shortest_diff_rad(0.0, 0.0) == pytest.approx(0.0)

    def test_positive_diff(self):
        result = angle_shortest_diff_rad(math.pi / 4, 0.0)
        assert result == pytest.approx(math.pi / 4)

    def test_negative_diff(self):
        result = angle_shortest_diff_rad(0.0, math.pi / 4)
        assert result == pytest.approx(-math.pi / 4)

    def test_wrap_around_positive(self):
        # 350° - 10° = 340° but shortest is -20°
        result = angle_shortest_diff_rad(math.radians(350), math.radians(10))
        assert result == pytest.approx(math.radians(-20), abs=1e-9)

    def test_wrap_around_negative(self):
        # 10° - 350° = -340° but shortest is +20°
        result = angle_shortest_diff_rad(math.radians(10), math.radians(350))
        assert result == pytest.approx(math.radians(20), abs=1e-9)

    def test_pi_boundary(self):
        result = angle_shortest_diff_rad(math.pi, 0.0)
        assert abs(result) == pytest.approx(math.pi)

    def test_two_pi_wraps_to_zero(self):
        result = angle_shortest_diff_rad(2 * math.pi, 0.0)
        assert result == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# _pick_demo_tuple3
# ---------------------------------------------------------------------------

class TestPickDemoTuple3:
    def test_list_input(self):
        assert _pick_demo_tuple3([1.0, 2.0, 3.0]) == (1.0, 2.0, 3.0)

    def test_tuple_input(self):
        assert _pick_demo_tuple3((0.5, -0.5, 0.0)) == (0.5, -0.5, 0.0)

    def test_int_elements(self):
        result = _pick_demo_tuple3([1, 2, 3])
        assert result == (1.0, 2.0, 3.0)
        assert all(isinstance(v, float) for v in result)

    def test_none_returns_none(self):
        assert _pick_demo_tuple3(None) is None

    def test_too_short_returns_none(self):
        assert _pick_demo_tuple3([1.0, 2.0]) is None

    def test_non_numeric_returns_none(self):
        assert _pick_demo_tuple3(["a", "b", "c"]) is None


# ---------------------------------------------------------------------------
# _pick_demo_fmt_scalar
# ---------------------------------------------------------------------------

class TestPickDemoFmtScalar:
    def test_default_3_digits(self):
        assert _pick_demo_fmt_scalar(1.23456) == "1.235"

    def test_custom_digits(self):
        assert _pick_demo_fmt_scalar(1.5, digits=1) == "1.5"

    def test_none_returns_none_string(self):
        assert _pick_demo_fmt_scalar(None) == "none"

    def test_zero(self):
        assert _pick_demo_fmt_scalar(0.0) == "0.000"


# ---------------------------------------------------------------------------
# _pick_demo_env_float / _pick_demo_env_int / _pick_demo_env_flag
# ---------------------------------------------------------------------------

class TestPickDemoEnvFloat:
    def test_default_when_unset(self, monkeypatch):
        monkeypatch.delenv("TEST_ENVFLOAT", raising=False)
        assert _pick_demo_env_float("TEST_ENVFLOAT", 1.5) == pytest.approx(1.5)

    def test_reads_env_var(self, monkeypatch):
        monkeypatch.setenv("TEST_ENVFLOAT", "3.14")
        assert _pick_demo_env_float("TEST_ENVFLOAT", 1.0) == pytest.approx(3.14)

    def test_minimum_clamp(self, monkeypatch):
        monkeypatch.setenv("TEST_ENVFLOAT", "-5.0")
        assert _pick_demo_env_float("TEST_ENVFLOAT", 1.0, minimum=0.0) == pytest.approx(0.0)

    def test_maximum_clamp(self, monkeypatch):
        monkeypatch.setenv("TEST_ENVFLOAT", "100.0")
        assert _pick_demo_env_float("TEST_ENVFLOAT", 1.0, maximum=10.0) == pytest.approx(10.0)

    def test_invalid_string_falls_back_to_default(self, monkeypatch):
        monkeypatch.setenv("TEST_ENVFLOAT", "not_a_number")
        assert _pick_demo_env_float("TEST_ENVFLOAT", 2.5) == pytest.approx(2.5)


class TestPickDemoEnvInt:
    def test_default_when_unset(self, monkeypatch):
        monkeypatch.delenv("TEST_ENVINT", raising=False)
        assert _pick_demo_env_int("TEST_ENVINT", 5) == 5

    def test_reads_env_var(self, monkeypatch):
        monkeypatch.setenv("TEST_ENVINT", "7")
        assert _pick_demo_env_int("TEST_ENVINT", 0) == 7

    def test_minimum_clamp(self, monkeypatch):
        monkeypatch.setenv("TEST_ENVINT", "-3")
        assert _pick_demo_env_int("TEST_ENVINT", 0, minimum=0) == 0

    def test_float_string_truncated(self, monkeypatch):
        monkeypatch.setenv("TEST_ENVINT", "4.9")
        assert _pick_demo_env_int("TEST_ENVINT", 0) == 4


class TestPickDemoEnvFlag:
    def test_default_when_unset_true(self, monkeypatch):
        monkeypatch.delenv("TEST_ENVFLAG", raising=False)
        assert _pick_demo_env_flag("TEST_ENVFLAG", True) is True

    def test_default_when_unset_false(self, monkeypatch):
        monkeypatch.delenv("TEST_ENVFLAG", raising=False)
        assert _pick_demo_env_flag("TEST_ENVFLAG", False) is False

    @pytest.mark.parametrize("val", ["1", "true", "yes", "on", "True", "YES"])
    def test_truthy_values(self, monkeypatch, val):
        monkeypatch.setenv("TEST_ENVFLAG", val)
        assert _pick_demo_env_flag("TEST_ENVFLAG", False) is True

    @pytest.mark.parametrize("val", ["0", "false", "no", "off", "", "False"])
    def test_falsy_values(self, monkeypatch, val):
        monkeypatch.setenv("TEST_ENVFLAG", val)
        assert _pick_demo_env_flag("TEST_ENVFLAG", True) is False


# ---------------------------------------------------------------------------
# _effective_direct_grasp_z
# ---------------------------------------------------------------------------

class TestEffectiveDirectGraspZ:
    def test_pinch_center_returns_zero(self):
        assert _effective_direct_grasp_z("rg2_pinch_center", 0.050) == pytest.approx(0.0)

    def test_other_frame_returns_offset(self):
        assert _effective_direct_grasp_z("tool0", 0.050) == pytest.approx(0.050)

    def test_rg2_tcp_returns_offset(self):
        assert _effective_direct_grasp_z("rg2_tcp", 0.025) == pytest.approx(0.025)


# ---------------------------------------------------------------------------
# _direct_runtime_target_tol_m
# ---------------------------------------------------------------------------

class TestDirectRuntimeTargetTolM:
    @pytest.fixture(autouse=True)
    def _reset_params_cache(self):
        """Invalida el singleton de PickDemoParams para que cada test
        re-lea env vars frescos (el helper get_pick_demo_params cachea)."""
        from ur5_qt_panel.panel_pick_demo_params import reset_pick_demo_params_cache
        reset_pick_demo_params_cache()
        yield
        reset_pick_demo_params_cache()

    def test_approach_coarse_default(self, monkeypatch):
        monkeypatch.delenv("PANEL_PICK_DEMO_APPROACH_COARSE_TCP_TOL_M", raising=False)
        result = _direct_runtime_target_tol_m("APPROACH_COARSE")
        assert result == pytest.approx(0.015)

    def test_approach_coarse_xy_corr(self, monkeypatch):
        monkeypatch.delenv("PANEL_PICK_DEMO_APPROACH_COARSE_TCP_TOL_M", raising=False)
        assert _direct_runtime_target_tol_m("APPROACH_COARSE_XY_CORR") == pytest.approx(0.015)

    def test_grasp_down_joint_default(self, monkeypatch):
        monkeypatch.delenv("PANEL_PICK_DEMO_GRASP_DOWN_TCP_TOL_M", raising=False)
        assert _direct_runtime_target_tol_m("GRASP_DOWN_JOINT") == pytest.approx(0.020)

    def test_generic_default(self, monkeypatch):
        monkeypatch.delenv("PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M", raising=False)
        result = _direct_runtime_target_tol_m("UNKNOWN_STAGE")
        assert result == pytest.approx(0.040)

    def test_minimum_floor_applied(self, monkeypatch):
        monkeypatch.setenv("PANEL_PICK_DEMO_APPROACH_COARSE_TCP_TOL_M", "0.001")
        result = _direct_runtime_target_tol_m("APPROACH_COARSE")
        assert result == pytest.approx(0.006)


# ---------------------------------------------------------------------------
# pick_demo_target_semantics
# ---------------------------------------------------------------------------

class TestPickDemoTargetSemantics:
    def test_approach_coarse(self):
        kind, _ = pick_demo_target_semantics("APPROACH_COARSE")
        assert kind == "OBJETO_MAS_CLEARANCE"

    def test_grasp_down_joint(self):
        kind, _ = pick_demo_target_semantics("GRASP_DOWN_JOINT")
        assert kind == "CONTACTO_GRASP"

    def test_grasp_align_ik(self):
        kind, _ = pick_demo_target_semantics("GRASP_ALIGN_IK")
        assert kind == "OBJETO_EXACTO"

    def test_pre_close(self):
        kind, _ = pick_demo_target_semantics("PRE_CLOSE")
        assert kind == "EXEC_REAL"

    def test_mesa_ready(self):
        kind, _ = pick_demo_target_semantics("MESA_READY")
        assert kind == "CACHE"

    def test_unknown_falls_back_to_cache(self):
        kind, _ = pick_demo_target_semantics("SOME_UNKNOWN_PHASE")
        assert kind == "CACHE"

    def test_case_insensitive(self):
        kind1, _ = pick_demo_target_semantics("approach_coarse")
        kind2, _ = pick_demo_target_semantics("APPROACH_COARSE")
        assert kind1 == kind2


# ---------------------------------------------------------------------------
# _is_demo_basket_transport_stage / _is_demo_basket_transport_motion
# ---------------------------------------------------------------------------

class TestTransportStageMotion:
    @pytest.mark.parametrize("label", [
        "CESTA_STAGE_1", "CESTA_STAGE_2", "CESTA_STAGE_10",
    ])
    def test_transport_stage_true(self, label):
        assert _is_demo_basket_transport_stage(label) is True

    @pytest.mark.parametrize("label", [
        "GRASP_DOWN_JOINT", "APPROACH_COARSE", "CESTA_RELEASE",
    ])
    def test_transport_stage_false(self, label):
        assert _is_demo_basket_transport_stage(label) is False

    def test_transport_motion_includes_stage(self):
        assert _is_demo_basket_transport_motion("CESTA_STAGE_3") is True

    def test_transport_motion_includes_release(self):
        assert _is_demo_basket_transport_motion("CESTA_RELEASE") is True

    def test_transport_motion_excludes_grasp(self):
        assert _is_demo_basket_transport_motion("GRASP_DOWN_JOINT") is False


# ---------------------------------------------------------------------------
# _compute_demo_basket_targets
# ---------------------------------------------------------------------------

class TestComputeDemoBasketTargets:
    def test_returns_expected_keys(self):
        result = _compute_demo_basket_targets(
            (0.5, 0.0, 0.3),
            transport_z_offset=0.10,
            release_z_offset=0.05,
        )
        assert "basket_base" in result
        assert "transport_target_base" in result
        assert "release_target_base" in result

    def test_transport_z_is_base_plus_offset(self):
        base = (0.5, 0.0, 0.3)
        result = _compute_demo_basket_targets(
            base,
            transport_z_offset=0.10,
            release_z_offset=0.05,
        )
        assert result["transport_target_base"][2] == pytest.approx(base[2] + 0.10)

    def test_release_z_is_base_plus_offset(self):
        base = (0.5, 0.0, 0.3)
        result = _compute_demo_basket_targets(
            base,
            transport_z_offset=0.10,
            release_z_offset=0.05,
        )
        assert result["release_target_base"][2] == pytest.approx(base[2] + 0.05)

    def test_xy_preserved(self):
        base = (0.42, -0.15, 0.30)
        result = _compute_demo_basket_targets(
            base,
            transport_z_offset=0.0,
            release_z_offset=0.0,
        )
        assert result["transport_target_base"][0] == pytest.approx(base[0])
        assert result["transport_target_base"][1] == pytest.approx(base[1])


# ---------------------------------------------------------------------------
# _compute_demo_stage_count_for_distance
# ---------------------------------------------------------------------------

class TestComputeDemoStageCountForDistance:
    def test_short_distance_returns_min_stages(self):
        count = _compute_demo_stage_count_for_distance(
            (0.0, 0.0, 0.0),
            (0.01, 0.0, 0.0),
            min_stages=2,
            max_stage_dist_m=0.10,
            max_stages=10,
        )
        assert count == 2

    def test_long_distance_increases_stage_count(self):
        count = _compute_demo_stage_count_for_distance(
            (0.0, 0.0, 0.0),
            (0.80, 0.0, 0.0),
            min_stages=2,
            max_stage_dist_m=0.10,
            max_stages=10,
        )
        assert count > 2

    def test_does_not_exceed_max_stages(self):
        count = _compute_demo_stage_count_for_distance(
            (0.0, 0.0, 0.0),
            (100.0, 0.0, 0.0),
            min_stages=2,
            max_stage_dist_m=0.10,
            max_stages=5,
        )
        assert count <= 5


# ---------------------------------------------------------------------------
# _compute_demo_linear_stage_targets
# ---------------------------------------------------------------------------

class TestComputeDemoLinearStageTargets:
    def test_single_stage_is_endpoint(self):
        result = _compute_demo_linear_stage_targets(
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            stages=1,
        )
        assert len(result) == 1
        assert result[0][0] == pytest.approx(1.0)

    def test_two_stages_midpoint_and_endpoint(self):
        result = _compute_demo_linear_stage_targets(
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            stages=2,
        )
        assert len(result) == 2
        assert result[0][0] == pytest.approx(1.0)
        assert result[1][0] == pytest.approx(2.0)

    def test_all_stages_lie_on_line(self):
        start = (0.0, 0.0, 0.0)
        end = (3.0, 6.0, 0.0)
        result = _compute_demo_linear_stage_targets(start, end, stages=3)
        for pt in result:
            assert pt[1] == pytest.approx(2.0 * pt[0])


# ---------------------------------------------------------------------------
# _compute_demo_joint_prep_waypoint
# ---------------------------------------------------------------------------

class TestComputeDemoJointPrepWaypoint:
    def test_blend_clamped_to_minimum(self):
        # blend is clamped to [0.05, 0.95]; blend=0.0 → effective 0.05
        seed = [0.0] * 6
        target = [1.0] * 6
        result = _compute_demo_joint_prep_waypoint(seed, target, blend=0.0)
        # result should be close to seed (5% blend towards target)
        assert result == pytest.approx([0.05] * 6)

    def test_blend_clamped_to_maximum(self):
        # blend=1.0 → effective 0.95
        seed = [0.0] * 6
        target = [1.0] * 6
        result = _compute_demo_joint_prep_waypoint(seed, target, blend=1.0)
        assert result == pytest.approx([0.95] * 6)

    def test_blend_half_moves_halfway(self):
        seed = [0.0] * 6
        target = [2.0] * 6
        result = _compute_demo_joint_prep_waypoint(seed, target, blend=0.5)
        assert result == pytest.approx([1.0] * 6)

    def test_result_length_matches_input(self):
        seed = [0.0] * 6
        target = [1.0] * 6
        result = _compute_demo_joint_prep_waypoint(seed, target, blend=0.5)
        assert len(result) == 6


# ---------------------------------------------------------------------------
# _compute_demo_transport_prep_joint_tol
# ---------------------------------------------------------------------------

class TestComputeDemoTransportPrepJointTol:
    def test_returns_at_least_minimum(self):
        start = [0.0] * 6
        target = [0.001] * 6
        tol = _compute_demo_transport_prep_joint_tol(
            start, target,
            configured_tol_rad=0.05,
            minimum_tol_rad=0.02,
        )
        assert tol >= 0.02

    def test_does_not_exceed_fraction_of_motion(self):
        start = [0.0] * 6
        target = [1.0] * 6
        tol = _compute_demo_transport_prep_joint_tol(
            start, target,
            configured_tol_rad=10.0,
            minimum_tol_rad=0.01,
            max_fraction=0.45,
        )
        max_joint_delta = max(abs(t - s) for t, s in zip(target, start))
        assert tol <= max_joint_delta * 0.45 + 1e-9

    def test_returns_float(self):
        result = _compute_demo_transport_prep_joint_tol(
            [0.0] * 6, [0.5] * 6,
            configured_tol_rad=0.1,
        )
        assert isinstance(result, float)


# ---------------------------------------------------------------------------
# _joint_step_wait_timeout
# ---------------------------------------------------------------------------

class TestJointStepWaitTimeout:
    def test_no_extra_is_just_timeout(self):
        result = _joint_step_wait_timeout(
            10.0,
            effective_move_sec=5.0,
            step_timeout_extra_sec=3.0,
            apply_step_timeout_extra=False,
        )
        assert result == pytest.approx(10.0)

    def test_with_extra_increases_timeout(self):
        result = _joint_step_wait_timeout(
            10.0,
            effective_move_sec=5.0,
            step_timeout_extra_sec=3.0,
            apply_step_timeout_extra=True,
        )
        assert result > 10.0

    def test_result_is_positive(self):
        result = _joint_step_wait_timeout(
            0.1,
            effective_move_sec=0.0,
            step_timeout_extra_sec=0.0,
        )
        assert result > 0.0


# ---------------------------------------------------------------------------
# Additional error-path and branch coverage
# ---------------------------------------------------------------------------

class TestPickDemoEnvIntInvalidFallback:
    def test_invalid_string_falls_back_to_default(self, monkeypatch):
        monkeypatch.setenv("TEST_ENVINT_BAD", "not_an_int_at_all")
        assert _pick_demo_env_int("TEST_ENVINT_BAD", 42) == 42


class TestDirectRuntimeTargetTolMExtraPaths:
    def test_approach_coarse_refine_default(self, monkeypatch):
        # 2026-05-04: default subido 0.006→0.020 para que COARSE_REFINE no
        # rechace por sesgo determinista FK panel↔TF (~10mm).
        monkeypatch.delenv("PANEL_PICK_DEMO_APPROACH_COARSE_REFINE_TCP_TOL_M", raising=False)
        result = _direct_runtime_target_tol_m("APPROACH_COARSE_REFINE")
        assert result == pytest.approx(0.020)

    def test_grasp_align_ik_default(self, monkeypatch):
        monkeypatch.delenv("PANEL_PICK_DEMO_GRASP_ALIGN_TCP_TOL_M", raising=False)
        result = _direct_runtime_target_tol_m("GRASP_ALIGN_IK")
        assert result == pytest.approx(0.015)


class TestComputeDemoBasketTargetsError:
    def test_none_base_raises_value_error(self):
        with pytest.raises(ValueError, match="basket_base_unavailable"):
            _compute_demo_basket_targets(None, transport_z_offset=0.1, release_z_offset=0.05)


class TestComputeDemoLinearStageTargetsError:
    def test_none_start_raises_value_error(self):
        with pytest.raises(ValueError, match="stage_target_unavailable"):
            _compute_demo_linear_stage_targets(None, (1.0, 0.0, 0.0), stages=2)

    def test_none_end_raises_value_error(self):
        with pytest.raises(ValueError, match="stage_target_unavailable"):
            _compute_demo_linear_stage_targets((0.0, 0.0, 0.0), None, stages=2)


class TestComputeDemoStageCountForDistanceError:
    def test_none_start_raises_value_error(self):
        with pytest.raises(ValueError, match="stage_count_target_unavailable"):
            _compute_demo_stage_count_for_distance(
                None, (1.0, 0.0, 0.0),
                min_stages=1, max_stage_dist_m=0.1, max_stages=5,
            )


class TestComputeDemoJointPrepWaypointError:
    def test_length_mismatch_raises(self):
        with pytest.raises(ValueError, match="joint_prep_length_mismatch"):
            _compute_demo_joint_prep_waypoint([0.0] * 6, [0.0] * 5, blend=0.5)


# ---------------------------------------------------------------------------
# _compute_demo_transport_recovery_stage_targets
# ---------------------------------------------------------------------------

class TestComputeDemoTransportRecoveryStageTargets:
    def test_none_current_raises_value_error(self):
        with pytest.raises(ValueError, match="transport_recovery_target_unavailable"):
            _compute_demo_transport_recovery_stage_targets(
                None, (1.0, 0.0, 0.5),
                min_remaining_dist_m=0.01, min_stages=2,
                max_stage_dist_m=0.10, max_stages=8,
            )

    def test_within_min_remaining_returns_empty(self):
        result = _compute_demo_transport_recovery_stage_targets(
            (0.5, 0.0, 0.5), (0.501, 0.0, 0.5),
            min_remaining_dist_m=0.05,
            min_stages=2,
            max_stage_dist_m=0.10,
            max_stages=8,
        )
        assert result == []

    def test_short_distance_single_stage_returns_empty(self):
        # distance 0.02 m with max_stage_dist_m=0.10 → adaptive_count=1 → stage_count≤1 → []
        result = _compute_demo_transport_recovery_stage_targets(
            (0.0, 0.0, 0.0), (0.02, 0.0, 0.0),
            min_remaining_dist_m=0.0,
            min_stages=1,
            max_stage_dist_m=0.10,
            max_stages=1,
        )
        assert result == []

    def test_far_target_returns_multiple_waypoints(self):
        result = _compute_demo_transport_recovery_stage_targets(
            (0.0, 0.0, 0.5), (0.80, 0.0, 0.5),
            min_remaining_dist_m=0.01,
            min_stages=2,
            max_stage_dist_m=0.10,
            max_stages=10,
        )
        assert len(result) >= 2
        # Last waypoint should be the target
        assert result[-1][0] == pytest.approx(0.80)

    def test_waypoints_are_tuples_of_three_floats(self):
        result = _compute_demo_transport_recovery_stage_targets(
            (0.0, 0.0, 0.5), (0.50, 0.0, 0.5),
            min_remaining_dist_m=0.001,
            min_stages=2,
            max_stage_dist_m=0.10,
            max_stages=8,
        )
        assert len(result) > 0
        for pt in result:
            assert len(pt) == 3
            assert all(isinstance(v, float) for v in pt)


# ---------------------------------------------------------------------------
# _compute_demo_transport_micro_recovery_target
# ---------------------------------------------------------------------------

class TestComputeDemoTransportMicroRecoveryTarget:
    def test_none_current_raises_value_error(self):
        with pytest.raises(ValueError, match="transport_micro_recovery_target_unavailable"):
            _compute_demo_transport_micro_recovery_target(
                None, (0.5, 0.0, 0.5),
                step_m=0.05, minimum_remaining_dist_m=0.01,
            )

    def test_within_step_plus_min_returns_none(self):
        # remaining = 0.02 m, step=0.05, min_remaining=0.01 → 0.02 ≤ 0.06 → None
        result = _compute_demo_transport_micro_recovery_target(
            (0.0, 0.0, 0.5), (0.02, 0.0, 0.5),
            step_m=0.05, minimum_remaining_dist_m=0.01,
        )
        assert result is None

    def test_far_target_returns_step_sized_advance(self):
        result = _compute_demo_transport_micro_recovery_target(
            (0.0, 0.0, 0.5), (1.0, 0.0, 0.5),
            step_m=0.05, minimum_remaining_dist_m=0.01,
        )
        assert result is not None
        dist = math.sqrt(sum((r - s) ** 2 for r, s in zip(result, (0.0, 0.0, 0.5))))
        assert dist == pytest.approx(0.05, abs=1e-6)

    def test_returns_tuple_of_three(self):
        result = _compute_demo_transport_micro_recovery_target(
            (0.0, 0.0, 0.0), (0.5, 0.0, 0.0),
            step_m=0.05, minimum_remaining_dist_m=0.001,
        )
        assert result is not None
        assert len(result) == 3


# ---------------------------------------------------------------------------
# _compute_demo_joint_prep_waypoints (plural)
# ---------------------------------------------------------------------------

class TestComputeDemoJointPrepWaypoints:
    def test_length_mismatch_raises(self):
        with pytest.raises(ValueError, match="joint_prep_length_mismatch"):
            _compute_demo_joint_prep_waypoints(
                [0.0] * 6, [0.0] * 5,
                blend=0.5, max_joint_delta_rad=0.3,
                max_sum_delta_rad=1.0, max_steps=10,
            )

    def test_empty_joints_returns_empty(self):
        result = _compute_demo_joint_prep_waypoints(
            [], [],
            blend=0.5, max_joint_delta_rad=0.3,
            max_sum_delta_rad=1.0, max_steps=10,
        )
        assert result == []

    def test_max_steps_one_returns_empty(self):
        # segment_count capped to max_steps=1 → return []
        seed = [0.0] * 6
        target = [1.0] * 6
        result = _compute_demo_joint_prep_waypoints(
            seed, target,
            blend=0.5, max_joint_delta_rad=0.3,
            max_sum_delta_rad=1.0, max_steps=1,
        )
        assert result == []

    def test_large_delta_produces_multiple_waypoints(self):
        seed = [0.0] * 6
        target = [2.0] * 6  # 2 rad per joint, 12 rad total
        result = _compute_demo_joint_prep_waypoints(
            seed, target,
            blend=0.5, max_joint_delta_rad=0.5,
            max_sum_delta_rad=2.0, max_steps=20,
        )
        assert len(result) >= 2
        # Each waypoint has 6 joints
        for wp in result:
            assert len(wp) == 6

    def test_max_steps_limits_waypoints(self):
        seed = [0.0] * 6
        target = [3.0] * 6
        result = _compute_demo_joint_prep_waypoints(
            seed, target,
            blend=0.5, max_joint_delta_rad=0.1,
            max_sum_delta_rad=0.5, max_steps=3,
        )
        # segment_count ≤ max_steps=3 → at most 2 intermediate waypoints (range(1, 3))
        assert len(result) <= 2

    def test_max_shoulder_delta_rad_increases_segments(self):
        # With a tight shoulder limit, more segments should be produced
        seed = [0.0] * 6
        target = [1.0] * 6
        result_no_shoulder = _compute_demo_joint_prep_waypoints(
            seed, target,
            blend=0.5, max_joint_delta_rad=2.0,
            max_sum_delta_rad=10.0, max_steps=20,
        )
        result_with_shoulder = _compute_demo_joint_prep_waypoints(
            seed, target,
            blend=0.5, max_joint_delta_rad=2.0,
            max_sum_delta_rad=10.0, max_steps=20,
            max_shoulder_delta_rad=0.1,
        )
        assert len(result_with_shoulder) >= len(result_no_shoulder)

    def test_seed_equals_target_returns_empty(self):
        # All deltas zero → _segment_count_for_limit(0.0, ...) = 1 → []
        same = [0.5] * 6
        result = _compute_demo_joint_prep_waypoints(
            same, same,
            blend=0.5, max_joint_delta_rad=0.3,
            max_sum_delta_rad=1.0, max_steps=1,
        )
        assert result == []

    def test_waypoints_monotonically_approach_target(self):
        seed = [0.0] * 6
        target = [1.0] * 6
        result = _compute_demo_joint_prep_waypoints(
            seed, target,
            blend=0.5, max_joint_delta_rad=0.4,
            max_sum_delta_rad=1.5, max_steps=10,
        )
        if len(result) >= 2:
            # First waypoint should be closer to seed than last
            dist_first = sum(abs(v) for v in result[0])
            dist_last = sum(abs(v) for v in result[-1])
            assert dist_last >= dist_first


# ---------------------------------------------------------------------------
# _compute_demo_transport_prep_joint_tol — extra paths
# ---------------------------------------------------------------------------

class TestComputeDemoTransportPrepJointTolExtraPaths:
    def test_length_mismatch_raises(self):
        with pytest.raises(ValueError, match="joint_prep_tol_length_mismatch"):
            _compute_demo_transport_prep_joint_tol(
                [0.0] * 6, [0.0] * 5,
                configured_tol_rad=0.1,
            )

    def test_empty_lists_returns_configured_or_minimum(self):
        result = _compute_demo_transport_prep_joint_tol(
            [], [],
            configured_tol_rad=0.15,
            minimum_tol_rad=0.02,
        )
        assert result >= 0.02


# ---------------------------------------------------------------------------
# V1.1 audit-v4 (2026-05-08): compute_distance_3d (replaces closure _dist3)
# ---------------------------------------------------------------------------


def test_compute_distance_3d_basic():
    from ur5_qt_panel.directo_geometry import compute_distance_3d
    d = compute_distance_3d((0.0, 0.0, 0.0), (3.0, 4.0, 0.0))
    assert d is not None
    assert abs(d - 5.0) < 1e-9


def test_compute_distance_3d_full_3d():
    from ur5_qt_panel.directo_geometry import compute_distance_3d
    d = compute_distance_3d((1.0, 2.0, 3.0), (4.0, 6.0, 3.0))
    assert d is not None
    assert abs(d - 5.0) < 1e-9


def test_compute_distance_3d_zero():
    from ur5_qt_panel.directo_geometry import compute_distance_3d
    assert compute_distance_3d((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)) == 0.0


def test_compute_distance_3d_none_input():
    from ur5_qt_panel.directo_geometry import compute_distance_3d
    assert compute_distance_3d(None, (0, 0, 0)) is None
    assert compute_distance_3d((0, 0, 0), None) is None
    assert compute_distance_3d(None, None) is None


def test_compute_distance_3d_invalid_types():
    """Inputs no convertibles a 3-tuple → None."""
    from ur5_qt_panel.directo_geometry import compute_distance_3d
    assert compute_distance_3d("abc", (0, 0, 0)) is None
    assert compute_distance_3d((1, 2), (0, 0, 0)) is None  # 2-tuple


def test_compute_distance_3d_lists():
    """Acepta lists además de tuples."""
    from ur5_qt_panel.directo_geometry import compute_distance_3d
    d = compute_distance_3d([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
    assert d is not None
    assert abs(d - 1.0) < 1e-9


def test_compute_distance_3d_negative():
    from ur5_qt_panel.directo_geometry import compute_distance_3d
    d = compute_distance_3d((0.0, 0.0, 0.0), (-3.0, -4.0, 0.0))
    assert d is not None
    assert abs(d - 5.0) < 1e-9
