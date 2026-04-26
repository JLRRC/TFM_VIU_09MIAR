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
