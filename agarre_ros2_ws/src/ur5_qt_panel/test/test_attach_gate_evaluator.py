#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_attach_gate_evaluator.py
"""Unit tests for attach_gate_evaluator — no ROS needed."""
from __future__ import annotations

import pytest

from ur5_qt_panel.attach_gate_evaluator import (
    AttachGateConfig,
    AttachGateEvaluator,
    AttachGateResult,
    PoseSample,
    _dist3,
    _fmt3,
    _fmts,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _nolog(_: str) -> None:
    pass


def _fast_cfg(**kwargs) -> AttachGateConfig:
    """Config with short timeout and few samples for fast tests."""
    defaults = dict(
        timeout_sec=0.05,
        min_stable_samples=2,
        stable_window_sec=0.10,
        sample_interval_sec=0.0,
    )
    defaults.update(kwargs)
    return AttachGateConfig(**defaults)


def _make_ev(
    *,
    attach_ok: bool = True,
    tcp_base=(0.0, 0.0, 0.5),
    obj_base=(0.0, 0.0, 0.5),
    gripper_opening: float | None = 0.010,
    cfg: AttachGateConfig | None = None,
    logs: list | None = None,
) -> AttachGateEvaluator:
    def _log(msg: str) -> None:
        if logs is not None:
            logs.append(msg)

    return AttachGateEvaluator(
        config=cfg or _fast_cfg(),
        emit_log=_log,
        tcp_tf_world_fn=lambda: tcp_base,
        tcp_tf_base_fn=lambda: tcp_base,
        object_world_fn=lambda: obj_base,
        object_base_fn=lambda: obj_base,
        gripper_opening_fn=(lambda: gripper_opening) if gripper_opening is not None else None,
        attach_backend_ok=attach_ok,
    )


# ---------------------------------------------------------------------------
# _dist3 / _fmt3 / _fmts
# ---------------------------------------------------------------------------

def test_dist3_zero_distance():
    assert _dist3((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)) == pytest.approx(0.0)


def test_dist3_unit_vector():
    assert _dist3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)) == pytest.approx(1.0)


def test_dist3_none_returns_none():
    assert _dist3(None, (1.0, 0.0, 0.0)) is None
    assert _dist3((1.0, 0.0, 0.0), None) is None


def test_fmt3_none():
    assert _fmt3(None) == "(none)"


def test_fmt3_value():
    s = _fmt3((1.0, 2.0, 3.0))
    assert "1.000" in s and "2.000" in s


def test_fmts_none():
    assert _fmts(None) == "none"


def test_fmts_value():
    assert "0.0100" in _fmts(0.01)


# ---------------------------------------------------------------------------
# PoseSample.rel_vec_base
# ---------------------------------------------------------------------------

def test_pose_sample_rel_vec_base_computed():
    s = PoseSample(
        timestamp=0.0,
        tcp_tf_world=None,
        tcp_tf_base=(0.1, 0.2, 0.3),
        object_world=None,
        object_base=(0.4, 0.5, 0.6),
    )
    rv = s.rel_vec_base
    assert rv == pytest.approx((0.3, 0.3, 0.3))


def test_pose_sample_rel_vec_base_none_when_missing():
    s = PoseSample(
        timestamp=0.0,
        tcp_tf_world=None,
        tcp_tf_base=None,
        object_world=None,
        object_base=(0.1, 0.2, 0.3),
    )
    assert s.rel_vec_base is None


# ---------------------------------------------------------------------------
# AttachGateConfig defaults
# ---------------------------------------------------------------------------

def test_default_config_reasonable_values():
    cfg = AttachGateConfig()
    assert cfg.max_tcp_obj_dist_m > 0
    assert cfg.max_rel_drift_m > 0
    assert cfg.timeout_sec > 0
    assert cfg.min_stable_samples >= 1


# ---------------------------------------------------------------------------
# AttachGateEvaluator — fail fast: attach_backend_not_ok
# ---------------------------------------------------------------------------

def test_evaluator_fails_immediately_if_backend_not_ok():
    logs: list[str] = []
    ev = _make_ev(attach_ok=False, logs=logs)
    result = ev.evaluate()
    assert result.ok is False
    assert result.reason == "attach_backend_not_ok"
    assert any("attach_backend_not_ok" in m for m in logs)


# ---------------------------------------------------------------------------
# AttachGateEvaluator — happy path
# ---------------------------------------------------------------------------

def test_evaluator_passes_when_close_and_stable():
    cfg = _fast_cfg(
        timeout_sec=1.0,
        min_stable_samples=2,
        max_tcp_obj_dist_m=0.05,
        max_rel_drift_m=0.02,
        gripper_closed_threshold_m=0.015,
    )
    ev = _make_ev(
        tcp_base=(0.0, 0.0, 0.5),
        obj_base=(0.0, 0.0, 0.5),
        gripper_opening=0.010,
        cfg=cfg,
    )
    result = ev.evaluate()
    assert result.ok is True
    assert result.reason == "ok"


def test_evaluator_result_has_tcp_obj_dist_zero_when_coincident():
    cfg = _fast_cfg(timeout_sec=1.0, min_stable_samples=2)
    ev = _make_ev(tcp_base=(0.3, 0.0, 0.5), obj_base=(0.3, 0.0, 0.5), cfg=cfg)
    result = ev.evaluate()
    assert result.ok is True
    assert result.tcp_obj_dist == pytest.approx(0.0, abs=1e-9)


def test_evaluator_gripper_closed_flag_set_when_below_threshold():
    cfg = _fast_cfg(timeout_sec=1.0, min_stable_samples=2, gripper_closed_threshold_m=0.020)
    ev = _make_ev(gripper_opening=0.005, cfg=cfg)
    result = ev.evaluate()
    assert result.ok is True
    assert result.gripper_closed is True


def test_evaluator_gripper_not_closed_does_not_block_pass():
    # gripper_not_closed is a warning, not a failure
    cfg = _fast_cfg(timeout_sec=1.0, min_stable_samples=2, gripper_closed_threshold_m=0.005)
    ev = _make_ev(gripper_opening=0.030, cfg=cfg)
    result = ev.evaluate()
    assert result.ok is True
    assert result.gripper_closed is False
    assert any("gripper_not_closed" in w for w in result.warnings)


# ---------------------------------------------------------------------------
# AttachGateEvaluator — failures
# ---------------------------------------------------------------------------

def test_evaluator_fails_when_tcp_too_far():
    cfg = _fast_cfg(
        timeout_sec=0.10,
        min_stable_samples=2,
        max_tcp_obj_dist_m=0.01,
    )
    ev = _make_ev(tcp_base=(0.0, 0.0, 0.0), obj_base=(1.0, 0.0, 0.0), cfg=cfg)
    result = ev.evaluate()
    assert result.ok is False
    assert "too_far" in result.reason or "timeout" in result.reason


def test_evaluator_timeout_when_drift_uncomputable():
    # object_base=None means rel_vec_base is always None → drift never computable
    # → every sample triggers sleep + continue until timeout
    cfg = _fast_cfg(
        timeout_sec=0.08,
        min_stable_samples=2,
        sample_interval_sec=0.05,
    )
    logs: list[str] = []
    ev = AttachGateEvaluator(
        config=cfg,
        emit_log=lambda m: logs.append(m),
        tcp_tf_world_fn=lambda: (0.0, 0.0, 0.5),
        tcp_tf_base_fn=lambda: (0.0, 0.0, 0.5),
        object_world_fn=lambda: None,
        object_base_fn=lambda: None,
        attach_backend_ok=True,
    )
    result = ev.evaluate()
    assert result.ok is False
    assert "timeout" in result.reason


def test_evaluator_logs_pass_message_on_success():
    logs: list[str] = []
    cfg = _fast_cfg(timeout_sec=1.0, min_stable_samples=2)
    ev = _make_ev(cfg=cfg, logs=logs)
    result = ev.evaluate()
    assert result.ok is True
    assert any("[ATTACH_GATE][PASS]" in m for m in logs)


def test_evaluator_logs_fail_message_on_backend_not_ok():
    logs: list[str] = []
    ev = _make_ev(attach_ok=False, logs=logs)
    ev.evaluate()
    assert any("[ATTACH_GATE][FAIL]" in m for m in logs)


# ---------------------------------------------------------------------------
# AttachGateResult defaults
# ---------------------------------------------------------------------------

def test_attach_gate_result_default_not_ok():
    r = AttachGateResult()
    assert r.ok is False
    assert r.reason == "not_evaluated"
    assert r.warnings == []


# ---------------------------------------------------------------------------
# Gripper opening function raises → exception swallowed (lines 194-195)
# ---------------------------------------------------------------------------

def test_evaluator_gripper_fn_exception_swallowed_still_passes():
    def _bad_gripper():
        raise RuntimeError("gripper_hw_fault")

    cfg = _fast_cfg(timeout_sec=1.0, min_stable_samples=2)
    ev = AttachGateEvaluator(
        config=cfg,
        emit_log=_nolog,
        tcp_tf_world_fn=lambda: (0.0, 0.0, 0.5),
        tcp_tf_base_fn=lambda: (0.0, 0.0, 0.5),
        object_world_fn=lambda: (0.0, 0.0, 0.5),
        object_base_fn=lambda: (0.0, 0.0, 0.5),
        gripper_opening_fn=_bad_gripper,
        attach_backend_ok=True,
    )
    result = ev.evaluate()
    # exception is swallowed; gripper_opening_m is None so no blocking
    assert result.ok is True
    assert result.gripper_opening_m is None


# ---------------------------------------------------------------------------
# Window pruning fires (line 208) with very short stable_window_sec
# ---------------------------------------------------------------------------

def test_evaluator_window_prunes_old_samples():
    # With stable_window_sec extremely small, old samples are pruned each iteration.
    # The evaluator still passes because we only need min_stable_samples=1 (immediate).
    cfg = _fast_cfg(
        timeout_sec=1.0,
        min_stable_samples=1,
        stable_window_sec=0.0001,
        sample_interval_sec=0.0,
    )
    ev = _make_ev(cfg=cfg)
    result = ev.evaluate()
    # stable_window_sec is so small that window gets pruned each loop,
    # but min_stable_samples=1 still allows pass once 1 sample accumulates
    assert result.ok is True


# ---------------------------------------------------------------------------
# strict_pose_source — degraded source (lines 278-285 + line 418)
# ---------------------------------------------------------------------------

def test_evaluator_strict_pose_source_degraded_times_out():
    # object_pose_source="strict_fresh_gazebo" + both object poses None → samples rejected
    cfg = _fast_cfg(timeout_sec=0.05, sample_interval_sec=0.0)
    ev = AttachGateEvaluator(
        config=cfg,
        emit_log=_nolog,
        tcp_tf_world_fn=lambda: (0.0, 0.0, 0.5),
        tcp_tf_base_fn=lambda: (0.0, 0.0, 0.5),
        object_world_fn=lambda: None,
        object_base_fn=lambda: None,
        attach_backend_ok=True,
        object_pose_source="strict_fresh_gazebo",
    )
    result = ev.evaluate()
    assert result.ok is False
    assert result.reason == "degraded_pose_source"


# ---------------------------------------------------------------------------
# tf_visual_mismatch blocks approval (lines 308-317)
# ---------------------------------------------------------------------------

def test_evaluator_tf_visual_mismatch_added_to_warnings():
    # tcp_visual_world far from live tcp_tf_world → tf_visual_gap > threshold → warning
    logs: list[str] = []
    cfg = _fast_cfg(
        timeout_sec=0.05,
        min_stable_samples=2,
        max_tf_visual_gap_m=0.001,  # very strict: any real gap blocks
        sample_interval_sec=0.0,
    )
    ev = AttachGateEvaluator(
        config=cfg,
        emit_log=lambda m: logs.append(m),
        tcp_tf_world_fn=lambda: (1.0, 0.0, 0.5),   # live TCP world
        tcp_tf_base_fn=lambda: (0.0, 0.0, 0.5),
        object_world_fn=lambda: (0.0, 0.0, 0.5),
        object_base_fn=lambda: (0.0, 0.0, 0.5),
        attach_backend_ok=True,
        tcp_visual_world=(0.0, 0.0, 0.5),           # reference: 1 m away → gap = 1 m
    )
    result = ev.evaluate()
    assert result.ok is False
    assert any("tf_visual_mismatch" in w for w in result.warnings)
    assert any("tf_visual_mismatch" in m for m in logs)


# ---------------------------------------------------------------------------
# drift too high in loop (lines 348-354) → timeout_rel_drift_too_high (line 424)
# ---------------------------------------------------------------------------

def test_evaluator_timeout_rel_drift_too_high():
    counter = [0]

    def _jitter_obj():
        counter[0] += 1
        # Alternates between two positions 0.05 m apart → drift = 0.05 >> max_rel_drift_m
        return (0.05 * (counter[0] % 2), 0.0, 0.5)

    cfg = _fast_cfg(
        timeout_sec=0.05,
        min_stable_samples=2,
        max_rel_drift_m=0.005,    # strict: 5 mm
        max_tcp_obj_dist_m=0.10,  # permissive: 10 cm
        stable_window_sec=5.0,    # keep all samples in window
        sample_interval_sec=0.0,
    )
    ev = AttachGateEvaluator(
        config=cfg,
        emit_log=_nolog,
        tcp_tf_world_fn=lambda: (0.0, 0.0, 0.5),
        tcp_tf_base_fn=lambda: (0.0, 0.0, 0.5),
        object_world_fn=lambda: (0.0, 0.0, 0.5),
        object_base_fn=_jitter_obj,
        attach_backend_ok=True,
    )
    result = ev.evaluate()
    assert result.ok is False
    assert result.reason == "timeout_rel_drift_too_high"


# ---------------------------------------------------------------------------
# timeout_insufficient_samples (line 422)
# ---------------------------------------------------------------------------

def test_evaluator_timeout_insufficient_samples():
    # object_base=None → rel_vec_base=None → drift always None → loop continues
    # stable_window_sec tiny → window always pruned → len(window) stays low
    cfg = _fast_cfg(
        timeout_sec=0.05,
        min_stable_samples=50,    # very high: impossible to accumulate
        stable_window_sec=0.000001,  # 1 µs: prunes almost all previous samples
        sample_interval_sec=0.0,
    )
    ev = AttachGateEvaluator(
        config=cfg,
        emit_log=_nolog,
        tcp_tf_world_fn=lambda: (0.0, 0.0, 0.5),
        tcp_tf_base_fn=lambda: (0.0, 0.0, 0.5),
        object_world_fn=lambda: (0.0, 0.0, 0.5),
        object_base_fn=lambda: None,   # rel_vec_base=None → drift not computable
        attach_backend_ok=True,
    )
    result = ev.evaluate()
    assert result.ok is False
    assert result.reason == "timeout_insufficient_samples"
