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
    _compute_demo_linear_stage_targets,
    _compute_demo_stage_count_for_distance,
    _compute_demo_transport_prep_joint_tol,
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
