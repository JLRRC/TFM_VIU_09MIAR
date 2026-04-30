#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_pick_demo_params.py
# Contenido: Tests del loader de parámetros runtime de pick_demo (F2 parcial).
"""Tests unitarios para panel_pick_demo_params.load_pick_demo_params().

Cubren:
- defaults del dataclass coinciden con los defaults históricos de los env
- prioridad env > YAML > default
- coerción de tipos (str -> float, str -> Optional[float])
- robustez: YAML inexistente, valores inválidos, sin pyyaml.
"""

from __future__ import annotations

import os
import textwrap
from pathlib import Path

import pytest

from ur5_qt_panel.panel_pick_demo_params import (
    ENV_VAR_BY_FIELD,
    PickDemoParams,
    load_pick_demo_params,
    reset_pick_demo_params_cache,
)


@pytest.fixture(autouse=True)
def _isolate_env(monkeypatch):
    """Ningún env de la familia PANEL_PICK_DEMO_ debe filtrarse al test."""
    for env_name in ENV_VAR_BY_FIELD.values():
        monkeypatch.delenv(env_name, raising=False)
    reset_pick_demo_params_cache()
    yield
    reset_pick_demo_params_cache()


def test_defaults_match_dataclass():
    p = load_pick_demo_params(yaml_path=Path("/nonexistent.yaml"))
    base = PickDemoParams()
    assert p == base


def test_defaults_have_expected_values():
    """Paridad con defaults históricos de panel_pick_demo (no tocar sin auditoría)."""
    p = PickDemoParams()
    assert p.ik_seed_joints == ""
    assert p.grasp_down_util_z_err_tol_m == 0.025
    assert p.close_z_err_tol_m == 0.012
    assert p.close_xy_tol_m == 0.012
    assert p.approach_coarse_keep_xy_tol_m == 0.020
    assert p.grasp_tcp_z_offset_m == 0.0
    assert p.gripper_closed_opening_thr_m == 0.020
    assert p.move_sec is None
    assert p.grasp_down_ik_err_tol == 0.080
    assert p.grasp_down_ik_seed_weight == 0.65
    assert p.grasp_down_rot_weight == 0.10
    # Tolerancias TCP por fase (consumidas en directo_geometry).
    assert p.approach_coarse_tcp_tol_m == 0.015
    assert p.approach_coarse_refine_tcp_tol_m == 0.006
    assert p.grasp_down_tcp_tol_m == 0.020
    assert p.grasp_align_tcp_tol_m == 0.015
    assert p.basket_transport_tcp_tol_m == 0.060
    assert p.direct_ik_tcp_tol_m == 0.040
    # GRASP_DOWN: branch guard.
    assert p.grasp_down_branch_guard_xy_tol_m == 0.010
    assert p.grasp_down_branch_guard_z_min_m == 0.015
    assert p.grasp_down_branch_guard_max_dev_rad == 0.35
    assert p.grasp_down_branch_guard_sum_dev_rad == 0.75
    # GRASP_DOWN: branch deltas.
    assert p.grasp_down_branch_max_delta_rad == 0.95
    assert p.grasp_down_branch_sum_delta_rad == 1.80
    assert p.grasp_down_branch_shoulder_lift_delta_rad == 0.80
    assert p.grasp_down_branch_elbow_delta_rad == 0.85
    assert p.grasp_down_branch_wrist1_delta_rad == 0.85
    # GRASP_DOWN: phase deltas.
    assert p.grasp_down_phase_max_delta_rad == 2.35
    assert p.grasp_down_phase_sum_delta_rad == 6.20
    assert p.grasp_down_phase_critical_sum_delta_rad == 2.85
    assert p.grasp_down_phase_shoulder_lift_delta_rad == 1.20
    assert p.grasp_down_phase_elbow_delta_rad == 1.15
    assert p.grasp_down_phase_wrist1_delta_rad == 1.10
    # GRASP_DOWN: strict + segment.
    assert p.grasp_down_strict_xy_tol_m == 0.012
    assert p.grasp_down_strict_z_tol_m == 0.025
    assert p.grasp_down_strict_dist_tol_m == 0.025
    assert p.grasp_down_segment_xy_step_m == 0.020
    assert p.grasp_down_segment_z_step_m == 0.005
    # F2 final batch: 94 tunables estáticos (orden alfabético).
    assert p.ac_phase_check_settle_sec == 3.0
    assert p.ac_phase_check_stable_samples == 3
    assert p.ac_phase_check_threshold_m == 0.004
    assert p.align_exit_xy_tol_m == 0.006
    assert p.align_exit_z_tol_m == 0.01
    assert p.align_ik_err_tol == 0.08
    assert p.align_ik_seed_weight == 0.5
    assert p.align_no_effect_tol_m == 0.002
    assert p.align_rot_weight == 0.1
    assert p.align_xy_lock_factor == 2.0
    assert p.align_z_bias_cap_m == 0.03
    assert p.align_z_bias_gain == 0.7
    assert p.align_z_improve_min_m == 0.006
    assert p.align_z_residual_tol_m == 0.015
    assert p.approach_coarse_gate_poll_sec == 0.1
    assert p.approach_coarse_gate_settle_sec == 0.8
    assert p.approach_coarse_gate_stable_samples == 2
    assert p.approach_coarse_gate_z_tol_m == 0.008
    assert p.approach_coarse_handoff_dist_tol_m == 0.015
    assert p.approach_coarse_handoff_dz_tol_m == 0.015
    assert p.approach_coarse_max_skip_m == 0.06
    assert p.approach_coarse_relaxed_handoff_xy_tol_m == 0.01
    assert p.approach_coarse_relaxed_skip_pose_ok is False
    assert p.approach_coarse_skip_xy_tol_m == 0.03
    assert p.approach_coarse_skip_z_tol_m == 0.04
    assert p.approach_coarse_util_dist_tol_m == 0.26
    assert p.approach_coarse_util_xy_tol_m == 0.18
    assert p.approach_coarse_util_z_err_tol_m == 0.18
    assert p.approach_coarse_xy_corr_max_m == 0.04
    assert p.approach_coarse_xy_corr_tol_m == 0.015
    assert p.approach_coarse_z_corr_tol_m == 0.02
    assert p.attach_follow_max_tcp_dist_m == 0.04
    assert p.attach_max_rel_drift_m == 0.012
    assert p.attach_max_tf_visual_gap_m == 0.02
    assert p.attach_min_stable_samples == 5
    assert p.attach_settle_sec == 1.8
    assert p.attach_stable_window_sec == 0.35
    assert p.attach_xy_tol_m == 0.012
    assert p.attach_z_tol_m == 0.015
    assert p.carry_home_max_tcp_dist_m == 0.2
    assert p.carry_settle_sec == 3.0
    assert p.close_confirm_timeout_sec == 1.8
    assert p.close_fallback_opening_sum == 0.4
    assert p.close_min_delta_sum == 0.08
    assert p.debug_pause_grasp_align_ik is False
    assert p.direct_ik_joint_tol_rad == 0.03
    assert p.direct_ik_runtime_attempts == 5
    assert p.direct_ik_runtime_settle_delta_m == 0.003
    assert p.direct_ik_runtime_settle_poll_sec == 0.1
    assert p.direct_ik_runtime_settle_samples == 3
    assert p.direct_ik_runtime_settle_sec == 2.5
    assert p.direct_ik_seed_weight == 0.035
    assert p.direct_ik_tcp_timeout_sec == 4.0
    assert p.extra_grasp_down_m == 0.0
    assert p.fallback_preset_max_dist_m == 0.1
    assert p.fallback_preset_max_xy_m == 0.05
    assert p.grasp_align_joint_tol_rad == 0.01
    assert p.grasp_align_max_attempts == 3
    assert p.grasp_down_disable_permissive_fallback is False
    assert p.grasp_down_extra_z_m == 0.0
    assert p.grasp_down_gate_poll_sec == 0.1
    assert p.grasp_down_gate_settle_sec == 0.8
    assert p.grasp_down_gate_stable_samples == 2
    assert p.grasp_down_max_attempts == 4
    assert p.grasp_down_permissive_ik_err_tol == 0.015
    assert p.grasp_down_permissive_rot_weight == 0.35
    assert p.grasp_down_permissive_seed_weight == 0.65
    assert p.grasp_down_use_moveit_cartesian is True
    assert p.grasp_down_util_dist_tol_m == 0.22
    assert p.gripper_confirm_max_state_age_sec == 0.35
    assert p.gripper_confirm_stable_samples == 2
    assert p.gripper_target_tol_m == 0.035
    assert p.handoff_target_jump_tol_m == 0.005
    assert p.manual_like_attach_max_tcp_dist_m == 0.14
    assert p.manual_like_attach_wait_sec == 0.9
    assert p.manual_like_attach_xy_tol_m == 0.06
    assert p.manual_like_attach_z_tol_m == 0.06
    assert p.manual_ref_stale_xy_tol_m == 0.08
    assert p.manual_ref_stale_z_below_tol_m == 0.005
    assert p.post_align_settle_sec == 0.2
    assert p.post_attach_hold_sec == 0.9
    assert p.post_close_hold_sec == 0.45
    assert p.post_close_mode == 'basket'
    assert p.pre_close_consecutive == 3
    assert p.pre_close_realign_retries == 2
    assert p.pre_close_wait_sec == 1.2
    assert p.release_open_confirm_timeout_sec == 1.8
    assert p.release_wait_sec == 1.6
    assert p.route_mode == 'direct_ik_hybrid'
    assert p.selected_base_stale_tol_m == 0.08
    assert p.short_lift_m == 0.12
    assert p.short_release_only is False
    assert p.skip_align_if_reachable is True
    assert p.step_timeout_extra_sec == 0.0


def test_env_overrides_default(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_GRASP_TCP_Z_OFFSET_M", "0.05")
    p = load_pick_demo_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.grasp_tcp_z_offset_m == 0.05


def test_yaml_overrides_default(tmp_path):
    yml = tmp_path / "pick_demo_runtime.yaml"
    yml.write_text(textwrap.dedent("""
        close_z_err_tol_m: 0.030
        close_xy_tol_m: 0.020
    """).strip())
    p = load_pick_demo_params(yaml_path=yml)
    assert p.close_z_err_tol_m == 0.030
    assert p.close_xy_tol_m == 0.020
    # los demás siguen al default
    assert p.grasp_tcp_z_offset_m == 0.0


def test_env_beats_yaml(tmp_path, monkeypatch):
    yml = tmp_path / "pick_demo_runtime.yaml"
    yml.write_text("close_z_err_tol_m: 0.999\n")
    monkeypatch.setenv("PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M", "0.005")
    p = load_pick_demo_params(yaml_path=yml)
    # env gana sobre YAML
    assert p.close_z_err_tol_m == 0.005


def test_invalid_value_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M", "not-a-number")
    p = load_pick_demo_params(yaml_path=Path("/nonexistent.yaml"))
    # coerción falla -> default
    assert p.close_z_err_tol_m == 0.012


def test_optional_move_sec_handles_empty_string(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_MOVE_SEC", "")
    p = load_pick_demo_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.move_sec is None


def test_optional_move_sec_accepts_number(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_MOVE_SEC", "5.5")
    p = load_pick_demo_params(yaml_path=Path("/nonexistent.yaml"))
    assert p.move_sec == 5.5


def test_dataclass_is_frozen():
    p = PickDemoParams()
    with pytest.raises(Exception):
        p.close_z_err_tol_m = 0.999  # type: ignore[misc]


def test_env_var_mapping_covers_all_fields():
    """Cada campo del dataclass debe tener una env var asociada."""
    base = PickDemoParams()
    field_names = {f.name for f in base.__dataclass_fields__.values()}
    mapped = set(ENV_VAR_BY_FIELD.keys())
    missing = field_names - mapped
    extra = mapped - field_names
    assert not missing, f"campos sin env var: {missing}"
    assert not extra, f"env vars sin campo: {extra}"
