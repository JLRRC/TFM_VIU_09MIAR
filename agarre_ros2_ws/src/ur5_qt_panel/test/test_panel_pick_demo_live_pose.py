#!/usr/bin/env python3
"""Unit tests for live object pose hardening in pick_demo."""

from types import SimpleNamespace

from ur5_qt_panel.panel_objects import ObjectLogicalState
from ur5_qt_panel.panel_pick_demo import (
    _resolve_live_object_base,
    _resolve_live_object_world,
    _select_pick_demo_cycle_object_reference,
)


class _FakeRosWorker:
    def __init__(self, pose_map=None, pose_ts=None, ready=True):
        self._pose_map = pose_map or {}
        self._pose_ts = pose_ts
        self._ready = ready

    def node_ready(self):
        return self._ready

    def pose_snapshot(self):
        return dict(self._pose_map), self._pose_ts


class _FakePanel:
    def __init__(self, ros_worker=None):
        self.ros_worker = ros_worker
        self._ros_worker_started = ros_worker is not None

    def _world_frame_last_first(self, fallback=None):
        return fallback or "world"

    def _business_base_frame(self):
        return "base_link"


def test_live_object_world_accepts_fresh_snapshot(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_SNAPSHOT_AGE_SEC", "0.10")
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_STABLE_CACHE_AGE_SEC", "0.20")
    monkeypatch.setenv("PANEL_PICK_DEMO_ALLOW_CORRELATED_STABLE_FALLBACK", "1")
    logs = []
    panel = _FakePanel(
        _FakeRosWorker(
            pose_map={"demo": (0.10, 0.20, 0.30)},
            pose_ts=9.95,
        )
    )
    state = SimpleNamespace(position=(0.10, 0.20, 0.30), last_update_ts=9.96)

    result = _resolve_live_object_world(
        panel,
        "demo",
        trace_fn=logs.append,
        now_fn=lambda: 10.0,
        get_positions_fn=lambda: {"demo": (0.10, 0.20, 0.30)},
        get_state_fn=lambda _name: state,
    )

    assert result["world"] == (0.10, 0.20, 0.30)
    assert result["source"] == "snapshot_pose_info"
    assert result["reason"] == "snapshot_fresh"
    assert any("[LIVE_OBJ][FINAL]" in line and "source=snapshot_pose_info" in line for line in logs)


def test_live_object_world_rejects_stale_snapshot(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_SNAPSHOT_AGE_SEC", "0.10")
    monkeypatch.setenv("PANEL_PICK_DEMO_ALLOW_CORRELATED_STABLE_FALLBACK", "0")
    logs = []
    panel = _FakePanel(
        _FakeRosWorker(
            pose_map={"demo": (0.10, 0.20, 0.30)},
            pose_ts=8.0,
        )
    )

    result = _resolve_live_object_world(
        panel,
        "demo",
        trace_fn=logs.append,
        now_fn=lambda: 10.0,
        get_positions_fn=lambda: {},
        get_state_fn=lambda _name: None,
    )

    assert result["world"] is None
    assert result["source"] == "none"
    assert result["reason"] == "no_fresh_live_object_pose"
    assert any("[LIVE_OBJ][REJECT]" in line and "source=snapshot" in line for line in logs)


def test_live_object_world_uses_correlated_fallback_only_if_allowed(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_SNAPSHOT_AGE_SEC", "0.10")
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_STABLE_CACHE_AGE_SEC", "0.20")
    panel = _FakePanel(
        _FakeRosWorker(
            pose_map={"demo": (0.10, 0.20, 0.30)},
            pose_ts=8.0,
        )
    )
    state = SimpleNamespace(position=(0.40, 0.50, 0.60), last_update_ts=9.95)

    monkeypatch.setenv("PANEL_PICK_DEMO_ALLOW_CORRELATED_STABLE_FALLBACK", "0")
    rejected = _resolve_live_object_world(
        panel,
        "demo",
        trace_fn=lambda _line: None,
        now_fn=lambda: 10.0,
        get_positions_fn=lambda: {"demo": (0.40, 0.50, 0.60)},
        get_state_fn=lambda _name: state,
    )
    assert rejected["world"] is None
    assert rejected["reason"] == "correlated_fallback_disabled"

    monkeypatch.setenv("PANEL_PICK_DEMO_ALLOW_CORRELATED_STABLE_FALLBACK", "1")
    accepted = _resolve_live_object_world(
        panel,
        "demo",
        trace_fn=lambda _line: None,
        now_fn=lambda: 10.0,
        get_positions_fn=lambda: {"demo": (0.40, 0.50, 0.60)},
        get_state_fn=lambda _name: state,
    )
    assert accepted["world"] == (0.40, 0.50, 0.60)
    assert accepted["source"] == "stable_cache_correlated"
    assert accepted["reason"] == "snapshot_not_usable_correlated_fallback"


def test_live_object_base_rejects_static_fallback_when_disabled(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_ALLOW_STATIC_WORLD_TO_BASE_FALLBACK", "0")
    logs = []
    panel = _FakePanel(
        _FakeRosWorker(
            pose_map={"demo": (1.0, 2.0, 3.0)},
            pose_ts=9.95,
        )
    )
    world_result = {
        "world": (1.0, 2.0, 3.0),
        "source": "snapshot_pose_info",
        "reason": "snapshot_fresh",
    }

    result = _resolve_live_object_base(
        panel,
        "demo",
        world_result=world_result,
        trace_fn=logs.append,
        transform_fn=lambda *_args, **_kwargs: (None, None),
        static_world_to_base_fn=lambda *_args: (9.0, 9.0, 9.0),
    )

    assert result["base"] is None
    assert result["base_source"] == "none"
    assert result["base_reason"] == "tf_transform_unavailable_static_fallback_disabled"
    assert any("[LIVE_OBJ][STATIC_FALLBACK]" in line and "enabled=false" in line for line in logs)


def test_cycle_ref_selects_snapshot_when_available(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_PROMOTED_STABLE_AGE_SEC", "0.35")
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_SELECTED_STABLE_DIVERGENCE_M", "0.080")
    monkeypatch.setenv("PANEL_PICK_DEMO_REQUIRE_OBJECT_ON_TABLE_FOR_PROMOTION", "1")
    logs = []
    panel = _FakePanel()

    def _resolve_world(_panel, _name, *, trace_fn=None):
        _ = trace_fn
        return {
            "world": (0.40, -0.01, 0.05),
            "source": "snapshot_pose_info",
            "reason": "snapshot_fresh",
            "snapshot_age_sec": 0.02,
            "stable_world": (0.401, -0.01, 0.05),
            "stable_age_sec": 0.10,
        }

    def _resolve_base(_panel, _name, *, world_result=None, trace_fn=None):
        _ = trace_fn
        if (world_result or {}).get("world") == (0.40, -0.01, 0.05):
            return {"base": (0.44, 0.00, 0.05)}
        return {"base": (0.44, 0.00, 0.05)}

    selected = _select_pick_demo_cycle_object_reference(
        panel,
        "demo",
        selected_base_anchor=(0.44, 0.00, 0.05),
        trace_fn=logs.append,
        resolve_world_fn=_resolve_world,
        resolve_base_fn=_resolve_base,
        get_state_fn=lambda _name: None,
        is_on_table_fn=lambda _name: True,
    )

    assert selected["ok"] is True
    assert selected["source"] == "snapshot_pose_info"
    assert selected["base"] == (0.44, 0.00, 0.05)
    assert selected["promoted_stable"] is False
    assert any("[DIRECT][CYCLE_REF][SELECT]" in line and "source=snapshot_pose_info" in line for line in logs)


def test_cycle_ref_promotes_stable_when_snapshot_not_usable(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_PROMOTED_STABLE_AGE_SEC", "0.35")
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_SELECTED_STABLE_DIVERGENCE_M", "0.080")
    monkeypatch.setenv("PANEL_PICK_DEMO_REQUIRE_OBJECT_ON_TABLE_FOR_PROMOTION", "1")
    logs = []
    panel = _FakePanel()
    state = SimpleNamespace(logical_state=ObjectLogicalState.ON_TABLE)

    def _resolve_world(_panel, _name, *, trace_fn=None):
        _ = trace_fn
        return {
            "world": None,
            "source": "none",
            "reason": "no_fresh_live_object_pose",
            "snapshot_age_sec": 0.30,
            "stable_world": (0.442, 0.000, 0.050),
            "stable_age_sec": 0.08,
        }

    def _resolve_base(_panel, _name, *, world_result=None, trace_fn=None):
        _ = trace_fn
        world = (world_result or {}).get("world")
        if world == (0.442, 0.000, 0.050):
            return {"base": (0.442, 0.000, 0.050)}
        return {"base": None}

    selected = _select_pick_demo_cycle_object_reference(
        panel,
        "demo",
        selected_base_anchor=(0.441, 0.000, 0.049),
        trace_fn=logs.append,
        resolve_world_fn=_resolve_world,
        resolve_base_fn=_resolve_base,
        get_state_fn=lambda _name: state,
        is_on_table_fn=lambda _name: True,
    )

    assert selected["ok"] is True
    assert selected["source"] == "stable_cache_promoted_cycle"
    assert selected["promoted_stable"] is True
    assert selected["base"] == (0.442, 0.000, 0.050)
    assert any("[DIRECT][CYCLE_REF][PROMOTE_STABLE]" in line for line in logs)


def test_cycle_ref_rejects_unstable_state_for_promotion(monkeypatch):
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_PROMOTED_STABLE_AGE_SEC", "0.35")
    monkeypatch.setenv("PANEL_PICK_DEMO_MAX_SELECTED_STABLE_DIVERGENCE_M", "0.080")
    monkeypatch.setenv("PANEL_PICK_DEMO_REQUIRE_OBJECT_ON_TABLE_FOR_PROMOTION", "1")
    logs = []
    panel = _FakePanel()
    state = SimpleNamespace(logical_state=ObjectLogicalState.SPAWNED)

    def _resolve_world(_panel, _name, *, trace_fn=None):
        _ = trace_fn
        return {
            "world": None,
            "source": "none",
            "reason": "no_fresh_live_object_pose",
            "snapshot_age_sec": 0.40,
            "stable_world": (0.442, 0.000, 0.050),
            "stable_age_sec": 0.05,
        }

    def _resolve_base(_panel, _name, *, world_result=None, trace_fn=None):
        _ = trace_fn
        world = (world_result or {}).get("world")
        if world == (0.442, 0.000, 0.050):
            return {"base": (0.442, 0.000, 0.050)}
        return {"base": None}

    selected = _select_pick_demo_cycle_object_reference(
        panel,
        "demo",
        selected_base_anchor=(0.441, 0.000, 0.049),
        trace_fn=logs.append,
        resolve_world_fn=_resolve_world,
        resolve_base_fn=_resolve_base,
        get_state_fn=lambda _name: state,
        is_on_table_fn=lambda _name: True,
    )

    assert selected["ok"] is False
    assert selected["source"] == "none"
    assert "state_not_stable_for_promotion" in (selected.get("reject_reasons") or [])
    assert any("[DIRECT][CYCLE_REF][REJECT]" in line and "state_not_stable_for_promotion" in line for line in logs)