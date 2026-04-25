#!/usr/bin/env python3
"""Pruebas unitarias para el fail-fast de transporte de DIRECTO."""

from __future__ import annotations

import math

import pytest

from ur5_qt_panel import panel_pick_demo as pick_demo


class _FakePanel:
    def __init__(self) -> None:
        self._logs: list[str] = []

    def _emit_log(self, line: str) -> None:
        self._logs.append(line)


class _TransportHarness:
    def __init__(self, samples: list[dict]) -> None:
        self._samples = list(samples)
        self._index = 0
        self.now = 0.0

    def _sample(self) -> dict:
        return self._samples[min(self._index, len(self._samples) - 1)]

    def live_object_world(self):
        return self._sample()["obj_world"]

    def live_object_base(self):
        return self._sample()["obj_base"]

    def live_tcp_base(self):
        return self._sample()["tcp_base"]

    def clock(self) -> float:
        return self.now

    def sleep(self, sec: float) -> None:
        self.now += float(sec)
        self._index += 1


class _RuntimeTargetHarness:
    def __init__(self, tcp_samples: list[tuple[float, float, float] | None]) -> None:
        self._samples = list(tcp_samples)
        self._index = 0
        self.now = 0.0

    def live_tcp_base(self):
        return self._samples[min(self._index, len(self._samples) - 1)]

    def clock(self) -> float:
        return self.now

    def sleep(self, sec: float) -> None:
        self.now += float(sec)
        self._index += 1


def test_validate_demo_transport_follow_accepts_object_following_tcp() -> None:
    panel = _FakePanel()
    harness = _TransportHarness(
        [
            {
                "obj_world": (-1.10, -0.05, 1.18),
                "obj_base": (-1.10, -0.05, 1.18),
                "tcp_base": (-1.09, -0.05, 1.18),
            },
            {
                "obj_world": (-1.12, -0.04, 1.15),
                "obj_base": (-1.12, -0.04, 1.15),
                "tcp_base": (-1.11, -0.04, 1.15),
            },
        ]
    )

    result = pick_demo._validate_demo_transport_follow(
        panel,
        phase="basket_transport",
        timeout_sec=0.5,
        max_tcp_dist_m=0.05,
        min_obj_world_z=0.72,
        live_object_world_fn=harness.live_object_world,
        live_object_base_fn=harness.live_object_base,
        live_tcp_base_fn=harness.live_tcp_base,
        clock_fn=harness.clock,
        sleep_fn=harness.sleep,
    )

    assert result["ok"] is True
    assert result["phase"] == "basket_transport"
    assert any("phase=basket_transport ok" in line for line in panel._logs)


def test_validate_demo_transport_follow_fails_when_object_leaves_tcp_window() -> None:
    panel = _FakePanel()
    harness = _TransportHarness(
        [
            {
                "obj_world": (-1.10, -0.05, 1.18),
                "obj_base": (-1.10, -0.05, 1.18),
                "tcp_base": (-0.70, 0.10, 1.18),
            },
            {
                "obj_world": (-1.10, -0.05, 1.18),
                "obj_base": (-1.10, -0.05, 1.18),
                "tcp_base": (-0.70, 0.10, 1.18),
            },
        ]
    )

    with pytest.raises(RuntimeError, match="demo_transport_follow_failed .*tcp_dist_above_max"):
        pick_demo._validate_demo_transport_follow(
            panel,
            phase="basket_transport",
            timeout_sec=0.35,
            max_tcp_dist_m=0.05,
            min_obj_world_z=0.72,
            live_object_world_fn=harness.live_object_world,
            live_object_base_fn=harness.live_object_base,
            live_tcp_base_fn=harness.live_tcp_base,
            clock_fn=harness.clock,
            sleep_fn=harness.sleep,
        )

    assert any("phase=basket_transport failed" in line for line in panel._logs)


def test_validate_demo_transport_follow_fails_when_object_drops_too_low() -> None:
    panel = _FakePanel()
    harness = _TransportHarness(
        [
            {
                "obj_world": (-1.10, -0.05, 0.40),
                "obj_base": (-1.10, -0.05, 0.40),
                "tcp_base": (-1.11, -0.05, 0.42),
            },
            {
                "obj_world": (-1.10, -0.05, 0.40),
                "obj_base": (-1.10, -0.05, 0.40),
                "tcp_base": (-1.11, -0.05, 0.42),
            },
        ]
    )

    with pytest.raises(RuntimeError, match="demo_transport_follow_failed .*obj_world_z_below_min"):
        pick_demo._validate_demo_transport_follow(
            panel,
            phase="basket_release_pose",
            timeout_sec=0.35,
            max_tcp_dist_m=0.05,
            min_obj_world_z=0.72,
            live_object_world_fn=harness.live_object_world,
            live_object_base_fn=harness.live_object_base,
            live_tcp_base_fn=harness.live_tcp_base,
            clock_fn=harness.clock,
            sleep_fn=harness.sleep,
        )

    assert any("phase=basket_release_pose failed" in line for line in panel._logs)


def test_compute_demo_basket_targets_uses_single_basket_reference() -> None:
    targets = pick_demo._compute_demo_basket_targets(
        (0.45, 0.10, -0.03),
        transport_z_offset=0.28,
        release_z_offset=0.05,
    )

    assert targets["basket_base"] == pytest.approx((0.45, 0.10, -0.03))
    assert targets["transport_target_base"] == pytest.approx((0.45, 0.10, 0.25))
    assert targets["release_target_base"] == pytest.approx((0.45, 0.10, 0.02))


def test_compute_demo_linear_stage_targets_interpolates_xyz() -> None:
    targets = pick_demo._compute_demo_linear_stage_targets(
        (0.0, 0.3, 1.0),
        (-0.45, 0.0, 0.25),
        stages=3,
    )

    assert targets == pytest.approx(
        [
            (-0.15, 0.2, 0.75),
            (-0.30, 0.1, 0.50),
            (-0.45, 0.0, 0.25),
        ]
    )


def test_compute_demo_stage_count_for_distance_uses_adaptive_floor() -> None:
    count = pick_demo._compute_demo_stage_count_for_distance(
        (0.018, 0.196, 1.001),
        (-0.45, 0.0, 0.25),
        min_stages=5,
        max_stage_dist_m=0.10,
        max_stages=16,
    )

    assert count == 10


def test_compute_demo_stage_count_for_distance_supports_finer_transport_segments() -> None:
    count = pick_demo._compute_demo_stage_count_for_distance(
        (0.018, 0.196, 1.001),
        (-0.45, 0.0, 0.25),
        min_stages=5,
        max_stage_dist_m=0.06,
        max_stages=24,
    )

    assert count == 16


def test_compute_demo_transport_recovery_stage_targets_splits_remaining_runtime_path() -> None:
    targets = pick_demo._compute_demo_transport_recovery_stage_targets(
        (0.207, 0.197, 0.929),
        (-0.013, 0.184, 0.954),
        min_remaining_dist_m=0.060,
        min_stages=2,
        max_stage_dist_m=0.050,
        max_stages=8,
    )

    assert len(targets) == 5
    assert targets[0] == pytest.approx((0.163, 0.1944, 0.934))
    assert targets[-1] == pytest.approx((-0.013, 0.184, 0.954))


def test_compute_demo_transport_recovery_stage_targets_skips_short_residuals() -> None:
    targets = pick_demo._compute_demo_transport_recovery_stage_targets(
        (0.010, 0.190, 0.950),
        (-0.013, 0.184, 0.954),
        min_remaining_dist_m=0.060,
        min_stages=2,
        max_stage_dist_m=0.050,
        max_stages=8,
    )

    assert targets == []


def test_normalize_joint_goal_for_execution_prefers_retry_seed_for_basket_transport() -> None:
    joint_goal = [0.10, -1.50, 0.0, -1.60, 0.0, 0.0]
    fallback_seed = [0.10, -1.50, 0.0, -1.60, 0.0, 0.0]
    retry_seed = [0.10 + (2.0 * math.pi), -1.50, 0.0, -1.60, 0.0, 0.0]

    normalized = pick_demo._normalize_joint_goal_for_execution(
        label="CESTA_STAGE_1_RECOVER_1",
        joint_goal=joint_goal,
        fallback_seed=fallback_seed,
        retry_seed=retry_seed,
    )

    assert normalized[0] == pytest.approx(retry_seed[0])


def test_normalize_joint_goal_for_execution_keeps_fallback_seed_for_non_transport() -> None:
    joint_goal = [0.10, -1.50, 0.0, -1.60, 0.0, 0.0]
    fallback_seed = [0.10, -1.50, 0.0, -1.60, 0.0, 0.0]
    retry_seed = [0.10 + (2.0 * math.pi), -1.50, 0.0, -1.60, 0.0, 0.0]

    normalized = pick_demo._normalize_joint_goal_for_execution(
        label="HOME",
        joint_goal=joint_goal,
        fallback_seed=fallback_seed,
        retry_seed=retry_seed,
    )

    assert normalized[0] == pytest.approx(fallback_seed[0])


def test_compute_demo_transport_micro_recovery_target_steps_along_residual() -> None:
    target = pick_demo._compute_demo_transport_micro_recovery_target(
        (0.261, 0.329, 0.885),
        (0.133, 0.336, 0.912),
        step_m=0.015,
        minimum_remaining_dist_m=0.040,
    )

    assert target is not None
    assert target == pytest.approx((0.246343, 0.329801, 0.888091), abs=1e-6)


def test_compute_demo_transport_micro_recovery_target_skips_short_residual() -> None:
    target = pick_demo._compute_demo_transport_micro_recovery_target(
        (0.252, 0.330, 0.891),
        (0.202, 0.334, 0.894),
        step_m=0.015,
        minimum_remaining_dist_m=0.040,
    )

    assert target is None


def test_compute_demo_joint_prep_waypoint_blends_shortest_joint_deltas() -> None:
    prep = pick_demo._compute_demo_joint_prep_waypoint(
        [0.0, -1.50, 0.0],
        [0.20, -1.90, 0.60],
        blend=0.5,
    )

    assert prep == pytest.approx([0.10, -1.70, 0.30])


def test_compute_demo_joint_prep_waypoints_split_large_joint_jump_into_segments() -> None:
    prep_waypoints = pick_demo._compute_demo_joint_prep_waypoints(
        [0.0, -1.50, 0.0],
        [0.30, -2.10, 0.60],
        blend=0.55,
        max_joint_delta_rad=0.20,
        max_sum_delta_rad=0.60,
        max_steps=6,
    )

    assert len(prep_waypoints) == 2
    assert prep_waypoints[0] == pytest.approx([0.10, -1.70, 0.20])
    assert prep_waypoints[1] == pytest.approx([0.20, -1.90, 0.40])


def test_compute_demo_joint_prep_waypoints_bias_more_segments_for_shoulders() -> None:
    prep_waypoints = pick_demo._compute_demo_joint_prep_waypoints(
        [0.0, -1.50, 0.0],
        [0.18, -1.82, 0.28],
        blend=0.55,
        max_joint_delta_rad=0.20,
        max_sum_delta_rad=0.60,
        max_steps=6,
        max_shoulder_delta_rad=0.10,
    )

    assert len(prep_waypoints) == 3
    assert prep_waypoints[0] == pytest.approx([0.045, -1.58, 0.07])
    assert prep_waypoints[1] == pytest.approx([0.09, -1.66, 0.14])
    assert prep_waypoints[2] == pytest.approx([0.135, -1.74, 0.21])


def test_compute_demo_transport_prep_joint_tol_tightens_small_segments() -> None:
    tol = pick_demo._compute_demo_transport_prep_joint_tol(
        [0.0, -1.50, 0.0],
        [0.02, -1.56, 0.07],
        configured_tol_rad=0.10,
        minimum_tol_rad=0.02,
    )

    assert tol == pytest.approx(0.0315)


def test_compute_demo_transport_prep_joint_tol_keeps_floor_for_tiny_segments() -> None:
    tol = pick_demo._compute_demo_transport_prep_joint_tol(
        [0.0, -1.50, 0.0],
        [0.001, -1.505, 0.008],
        configured_tol_rad=0.10,
        minimum_tol_rad=0.02,
    )

    assert tol == pytest.approx(0.02)


def test_joint_step_wait_timeout_can_skip_global_extra_for_transport_prep() -> None:
    assert pick_demo._joint_step_wait_timeout(
        10.0,
        effective_move_sec=4.0,
        step_timeout_extra_sec=60.0,
        apply_step_timeout_extra=False,
    ) == pytest.approx(10.0)

    assert pick_demo._joint_step_wait_timeout(
        None,
        effective_move_sec=4.0,
        step_timeout_extra_sec=60.0,
        apply_step_timeout_extra=True,
    ) == pytest.approx(66.0)


def test_should_apply_global_step_timeout_extra_skips_basket_transport_motion() -> None:
    assert (
        pick_demo._should_apply_global_step_timeout_extra(
            "CESTA_STAGE_1",
            requested=True,
        )
        is False
    )
    assert (
        pick_demo._should_apply_global_step_timeout_extra(
            "CESTA_STAGE_1_RECOVER_1",
            requested=True,
        )
        is False
    )
    assert (
        pick_demo._should_apply_global_step_timeout_extra(
            "CESTA_RELEASE",
            requested=True,
        )
        is False
    )
    assert (
        pick_demo._should_apply_global_step_timeout_extra(
            "HOME",
            requested=True,
        )
        is True
    )
    assert (
        pick_demo._should_apply_global_step_timeout_extra(
            "MESA",
            requested=False,
        )
        is False
    )


def test_wait_for_demo_runtime_target_progress_accepts_reaching_target(monkeypatch) -> None:
    panel = _FakePanel()
    harness = _RuntimeTargetHarness(
        [
            (0.220, 0.000, 0.000),
            (0.160, 0.000, 0.000),
            (0.090, 0.000, 0.000),
            (0.030, 0.000, 0.000),
        ]
    )
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_PROGRESS_POLL_SEC", "0.05")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_PROGRESS_ARM_SEC", "0.10")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_STALL_TIMEOUT_SEC", "0.20")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_MIN_PROGRESS_M", "0.01")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_WRONG_DIRECTION_TOL_M", "0.02")

    result = pick_demo._wait_for_demo_runtime_target_progress(
        panel,
        label="CESTA_STAGE_1",
        target_xyz=(0.0, 0.0, 0.0),
        timeout_sec=2.0,
        tol_xyz_m=0.04,
        live_tcp_base_fn=harness.live_tcp_base,
        clock_fn=harness.clock,
        sleep_fn=harness.sleep,
    )

    assert result["ok"] is True
    assert result["reason"] == "target_reached"
    assert result["dist_m"] == pytest.approx(0.03)


def test_wait_for_demo_runtime_target_progress_fails_fast_on_no_progress(monkeypatch) -> None:
    panel = _FakePanel()
    harness = _RuntimeTargetHarness(
        [
            (0.221, 0.197, 0.929),
            (0.221, 0.197, 0.929),
            (0.221, 0.197, 0.929),
            (0.221, 0.197, 0.929),
            (0.221, 0.197, 0.929),
        ]
    )
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_PROGRESS_POLL_SEC", "0.05")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_PROGRESS_ARM_SEC", "0.10")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_STALL_TIMEOUT_SEC", "0.20")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_MIN_PROGRESS_M", "0.01")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_WRONG_DIRECTION_TOL_M", "0.02")

    result = pick_demo._wait_for_demo_runtime_target_progress(
        panel,
        label="CESTA_STAGE_1",
        target_xyz=(0.0, 0.0, 0.0),
        timeout_sec=2.0,
        tol_xyz_m=0.04,
        live_tcp_base_fn=harness.live_tcp_base,
        clock_fn=harness.clock,
        sleep_fn=harness.sleep,
    )

    assert result["ok"] is False
    assert result["reason"] == "no_progress"
    assert result["dist_m"] == pytest.approx((0.221**2 + 0.197**2 + 0.929**2) ** 0.5)
    assert result["elapsed_sec"] == pytest.approx(0.20, abs=0.05)


def test_wait_for_demo_runtime_target_progress_allows_initial_detour_before_converging(monkeypatch) -> None:
    panel = _FakePanel()
    harness = _RuntimeTargetHarness(
        [
            (0.100, 0.000, 0.000),
            (0.115, 0.000, 0.000),
            (0.131, 0.000, 0.000),
            (0.090, 0.000, 0.000),
            (0.030, 0.000, 0.000),
        ]
    )
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_PROGRESS_POLL_SEC", "0.05")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_PROGRESS_ARM_SEC", "0.10")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_STALL_TIMEOUT_SEC", "0.30")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_MIN_PROGRESS_M", "0.01")
    monkeypatch.setenv("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_WRONG_DIRECTION_TOL_M", "0.02")

    result = pick_demo._wait_for_demo_runtime_target_progress(
        panel,
        label="CESTA_STAGE_1",
        target_xyz=(0.0, 0.0, 0.0),
        timeout_sec=2.0,
        tol_xyz_m=0.04,
        live_tcp_base_fn=harness.live_tcp_base,
        clock_fn=harness.clock,
        sleep_fn=harness.sleep,
    )

    assert result["ok"] is True
    assert result["reason"] == "target_reached"
    assert result["dist_m"] == pytest.approx(0.03)


def test_transport_prep_failure_policy_defaults_to_soft_continue() -> None:
    assert (
        pick_demo._transport_prep_failure_policy(strict_mode=False)
        == "continue_final_stage"
    )
    assert pick_demo._transport_prep_failure_policy(strict_mode=True) == "raise"


def test_should_transport_prep_failure_jump_to_replan_for_late_large_residual() -> None:
    assert pick_demo._should_transport_prep_failure_jump_to_replan(
        failed_segment_index=5,
        total_segments=6,
        max_joint_residual_rad=0.145,
        shoulder_joint_residual_rad=0.145,
        min_failed_segment_fraction=0.70,
        max_joint_residual_threshold_rad=0.12,
        shoulder_joint_residual_threshold_rad=0.10,
    ) is True


def test_should_transport_prep_failure_jump_to_replan_ignores_early_failure() -> None:
    assert pick_demo._should_transport_prep_failure_jump_to_replan(
        failed_segment_index=2,
        total_segments=6,
        max_joint_residual_rad=0.145,
        shoulder_joint_residual_rad=0.145,
        min_failed_segment_fraction=0.70,
        max_joint_residual_threshold_rad=0.12,
        shoulder_joint_residual_threshold_rad=0.10,
    ) is False


def test_evaluate_transport_stage_postcheck_accepts_runtime_and_model_in_tol() -> None:
    result = pick_demo._evaluate_transport_stage_postcheck(
        label="CESTA_STAGE_1",
        runtime_target_ok=True,
        runtime_target_dist_m=0.031,
        runtime_target_tol_m=0.040,
        model_target_err_m=0.018,
        model_target_tol_m=0.040,
    )

    assert result == {
        "ok": True,
        "reason": "ok",
        "runtime_target_tol_m": pytest.approx(0.040),
        "model_target_tol_m": pytest.approx(0.040),
        "model_target_bypassed": False,
    }


def test_evaluate_transport_stage_postcheck_flags_runtime_and_model_miss() -> None:
    result = pick_demo._evaluate_transport_stage_postcheck(
        label="CESTA_STAGE_1",
        runtime_target_ok=False,
        runtime_target_dist_m=0.048,
        runtime_target_tol_m=0.040,
        model_target_err_m=0.056,
        model_target_tol_m=0.040,
    )

    assert result["ok"] is False
    assert result["reason"] == "runtime_target_dist=0.048/0.040 model_target_err=0.056/0.040"


def test_evaluate_transport_stage_postcheck_flags_unconfirmed_runtime_target() -> None:
    result = pick_demo._evaluate_transport_stage_postcheck(
        label="CESTA_STAGE_1",
        runtime_target_ok=False,
        runtime_target_dist_m=None,
        runtime_target_tol_m=0.040,
        model_target_err_m=0.018,
        model_target_tol_m=0.040,
    )

    assert result["ok"] is False
    assert result["reason"] == "runtime_target=unconfirmed"


def test_build_transport_seed_candidates_adds_live_last_ok_prep_and_corridor_variants() -> None:
    base_seed = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    live_seed = [0.1, -1.40, 0.2, -1.30, -0.2, 0.1]
    last_ok = [0.2, -1.30, 0.4, -1.10, -0.3, 0.2]
    prep_seed = [0.3, -1.20, 0.5, -1.00, -0.4, 0.3]

    candidates = pick_demo._build_transport_seed_candidates(
        base_seed=base_seed,
        live_seed=live_seed,
        last_transport_joint_goal=last_ok,
        prep_reference_seed=prep_seed,
    )

    labels = [label for _seed, label in candidates]
    assert labels[:4] == [
        "base_seed",
        "live_joints",
        "last_transport_ok",
        "prep_reference",
    ]
    assert "corridor_wrist2" in labels
    assert "corridor_shoulder_elbow" in labels


def test_evaluate_transport_stage_preexec_model_guard_accepts_in_tolerance() -> None:
    result = pick_demo._evaluate_transport_stage_preexec_model_guard(
        label="CESTA_STAGE_1",
        target_ik=(0.10, 0.20, 0.30),
        joint_goal=[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        tol_m=0.02,
        fk_fn=lambda _q: ((0.11, 0.19, 0.30), None),
    )

    assert result["ok"] is True
    assert result["reason"] == "ok"
    assert result["model_target_err_m"] == pytest.approx((0.01**2 + 0.01**2) ** 0.5)


def test_evaluate_transport_stage_preexec_model_guard_rejects_bad_model_target() -> None:
    result = pick_demo._evaluate_transport_stage_preexec_model_guard(
        label="CESTA_STAGE_1_RECOVER_1",
        target_ik=(0.10, 0.20, 0.30),
        joint_goal=[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        tol_m=0.02,
        fk_fn=lambda _q: ((0.16, 0.24, 0.30), None),
    )

    assert result["ok"] is False
    assert result["reason"] == "model_target_err=0.072/0.020"
