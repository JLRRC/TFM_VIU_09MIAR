#!/usr/bin/env python3
"""Tests for TFM apply/reset gating and ROI selection."""

import numpy as np

from ur5_qt_panel import panel_tfm
from ur5_qt_panel.panel_tfm import tfm_infer


class _FakePanel:
    def __init__(self, *, experiment_ready: bool, infer_ready: bool, infer_reason: str):
        self._tfm_infer_inflight = False
        self.tfm_module = object()
        self._experiment_ready = experiment_ready
        self._infer_ready = infer_ready
        self._infer_reason = infer_reason
        self._selected_object = ""
        self._selected_px = None
        self.status_calls = []
        self.logs = []
        self.audit = []
        self.infer_ready_calls = 0

    def _tfm_experiment_ready_status(self):
        if self._experiment_ready:
            return True, ""
        return False, "aplica un experimento primero"

    def _tfm_infer_ready_status(self):
        self.infer_ready_calls += 1
        return self._infer_ready, self._infer_reason

    def _set_status(self, text, error=False):
        self.status_calls.append((text, error))

    def _emit_log(self, text):
        self.logs.append(text)

    def _audit_append(self, path, text):
        self.audit.append((path, text))


def test_tfm_infer_requires_applied_experiment_before_readiness_checks():
    panel = _FakePanel(
        experiment_ready=False,
        infer_ready=False,
        infer_reason="sin frame de cámara",
    )

    ok, reason = tfm_infer(panel)

    assert not ok
    assert reason == "aplica un experimento primero"
    assert panel.infer_ready_calls == 0
    assert panel.status_calls[-1] == ("TFM bloqueado: aplica un experimento primero", True)


def test_tfm_infer_keeps_original_readiness_flow_after_apply():
    panel = _FakePanel(
        experiment_ready=True,
        infer_ready=False,
        infer_reason="sin frame de cámara",
    )

    ok, reason = tfm_infer(panel)

    assert not ok
    assert reason == "sin frame de cámara"
    assert panel.infer_ready_calls == 1
    assert panel.status_calls[-1] == ("TFM en espera: sin frame de cámara", True)


def test_resolve_infer_roi_uses_selected_object_by_default(monkeypatch):
    monkeypatch.delenv("PANEL_TFM_INFER_USE_ROI", raising=False)
    panel = _FakePanel(
        experiment_ready=True,
        infer_ready=True,
        infer_reason="",
    )
    panel._selected_object = "box_red"
    panel._selected_px = (124, 124)

    roi = panel_tfm._resolve_infer_roi(panel)

    assert roi == (124, 124, 96)


def test_resolve_infer_roi_allows_explicit_full_frame(monkeypatch):
    monkeypatch.setenv("PANEL_TFM_INFER_USE_ROI", "0")
    panel = _FakePanel(
        experiment_ready=True,
        infer_ready=True,
        infer_reason="",
    )
    panel._selected_object = "box_red"
    panel._selected_px = (124, 124)

    roi = panel_tfm._resolve_infer_roi(panel)

    assert roi is None


def test_reconcile_inferred_grasp_size_uses_reference_when_center_is_close():
    pred = {"cx": 156.6, "cy": 180.7, "w": 7.35, "h": 5.54, "angle_deg": -12.9}
    ref = {"cx": 160.0, "cy": 175.4, "w": 17.78, "h": 19.25, "angle_deg": 0.0}

    adjusted, changed = panel_tfm.reconcile_inferred_grasp_size(pred, ref, roi=(159, 175, 96))

    assert changed is True
    assert adjusted is not None
    assert adjusted["w"] == ref["w"]
    assert adjusted["h"] == ref["h"]
    assert adjusted["cx"] == pred["cx"]
    assert adjusted["cy"] == pred["cy"]


def test_reconcile_inferred_grasp_size_keeps_model_size_when_center_is_far():
    pred = {"cx": 161.6, "cy": 109.5, "w": 25.39, "h": 13.42, "angle_deg": -6.7}
    ref = {"cx": 160.0, "cy": 175.4, "w": 17.78, "h": 19.25, "angle_deg": 0.0}

    adjusted, changed = panel_tfm.reconcile_inferred_grasp_size(pred, ref, roi=(159, 175, 96))

    assert changed is False
    assert adjusted == pred


def test_reconcile_inferred_grasp_center_blends_towards_reference_when_close():
    pred = {"cx": 153.57, "cy": 187.85, "w": 17.78, "h": 19.25, "angle_deg": -14.2}
    ref = {"cx": 160.0, "cy": 175.4, "w": 17.78, "h": 19.25, "angle_deg": 0.0}

    adjusted, changed = panel_tfm.reconcile_inferred_grasp_center(pred, ref, roi=(159, 175, 96))

    assert changed is True
    assert adjusted is not None
    assert adjusted["cx"] == 157.428
    assert adjusted["cy"] == 180.38
    assert adjusted["w"] == pred["w"]
    assert adjusted["h"] == pred["h"]


def test_reconcile_inferred_grasp_center_keeps_model_center_when_far():
    pred = {"cx": 151.5, "cy": 199.9, "w": 6.3, "h": 5.01, "angle_deg": -8.5}
    ref = {"cx": 160.0, "cy": 175.4, "w": 17.78, "h": 19.25, "angle_deg": 0.0}

    adjusted, changed = panel_tfm.reconcile_inferred_grasp_center(pred, ref, roi=(159, 175, 96))

    assert changed is False
    assert adjusted == pred


def test_reconcile_inferred_grasp_angle_snaps_circle_to_reference():
    pred = {"cx": 157.4, "cy": 180.3, "w": 17.78, "h": 19.25, "angle_deg": -14.27}
    ref = {"cx": 160.0, "cy": 175.4, "w": 17.78, "h": 19.25, "angle_deg": 0.0}

    adjusted, changed = panel_tfm.reconcile_inferred_grasp_angle(
        pred,
        ref,
        roi=(159, 175, 96),
        object_shape="circle",
    )

    assert changed is True
    assert adjusted is not None
    assert adjusted["angle_deg"] == 0.0
    assert adjusted["cx"] == pred["cx"]
    assert adjusted["cy"] == pred["cy"]


def test_reconcile_inferred_grasp_angle_keeps_rectangular_objects():
    pred = {"cx": 180.2, "cy": 66.3, "w": 31.49, "h": 10.01, "angle_deg": -23.87}
    ref = {"cx": 182.17, "cy": 68.06, "w": 31.49, "h": 10.01, "angle_deg": 0.0}

    adjusted, changed = panel_tfm.reconcile_inferred_grasp_angle(
        pred,
        ref,
        roi=(182, 68, 96),
        object_shape="cross",
    )

    assert changed is False
    assert adjusted == pred


def test_cached_preprocessed_input_rejects_stale_channel_layout():
    panel = _FakePanel(
        experiment_ready=True,
        infer_ready=True,
        infer_reason="",
    )
    panel._tfm_preprocessed_cache = (
        123.0,
        np.zeros((3, 224, 224), dtype=np.float32),
        3,
    )

    cached = panel_tfm._get_cached_preprocessed_input(
        panel,
        frame_ts=123.0,
        in_channels=4,
        roi=None,
    )

    assert cached is None


def test_cached_preprocessed_input_accepts_matching_channel_layout():
    panel = _FakePanel(
        experiment_ready=True,
        infer_ready=True,
        infer_reason="",
    )
    expected = np.zeros((4, 224, 224), dtype=np.float32)
    panel_tfm._store_preprocessed_cache(
        panel,
        frame_ts=123.0,
        preprocessed=expected,
        in_channels=4,
    )

    cached = panel_tfm._get_cached_preprocessed_input(
        panel,
        frame_ts=123.0,
        in_channels=4,
        roi=None,
    )

    assert cached is expected