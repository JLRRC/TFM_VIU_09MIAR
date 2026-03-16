import json
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent / "src" / "ur5_qt_panel"))

from ur5_qt_panel import panel_tfm


class _FakeImage:
    def __init__(self):
        self.saved_paths = []

    def save(self, path):
        Path(path).write_text("fake-image", encoding="utf-8")
        self.saved_paths.append(path)
        return True


class _UiSignal:
    def emit(self, callback):
        callback()


def test_tfm_infer_module_writes_artifacts(tmp_path, monkeypatch):
    captured = {}
    qimg = _FakeImage()
    tfm_module = SimpleNamespace(
        is_model_loaded=lambda: True,
        set_input_image=lambda *args, **kwargs: None,
        infer_grasp_params=lambda: {
            "cx": 10.0,
            "cy": 20.0,
            "w": 30.0,
            "h": 40.0,
            "angle_deg": 5.0,
        },
        last_error=lambda: "",
    )
    panel = SimpleNamespace(
        _tfm_infer_inflight=False,
        _safety=SimpleNamespace(require_ready_basic=lambda action: True),
        _last_camera_frame=(qimg, 320, 240, 123.0),
        _selected_object="box_lightblue",
        _selected_px=None,
        _selected_world=(-0.1, 0.2, 0.88),
        _selected_base=(0.6, 0.1, 0.1),
        _selected_base_frame="base_link",
        _selection_timestamp=99.0,
        _tfm_get_ckpt_path=lambda: "/tmp/model.pth",
        tfm_module=tfm_module,
        _tfm_ckpt_selected="/tmp/model.pth",
        _tfm_preprocessed_cache=None,
        _infer_session_id="session",
        camera_topic="/camera_overhead/image",
        _audit_append=lambda *args, **kwargs: None,
        _set_status=lambda *args, **kwargs: None,
        signal_run_ui=_UiSignal(),
        _run_async=lambda worker, name="": worker(),
        _handle_infer_result=lambda result: captured.setdefault("result", result),
    )
    monkeypatch.setattr(panel_tfm, "LOG_DIR", str(tmp_path))
    monkeypatch.setattr(panel_tfm, "INFER_ROI_SIZE", 0)
    monkeypatch.setattr(panel_tfm.os.path, "isfile", lambda path: True)
    monkeypatch.setattr(panel_tfm, "build_tfm_preprocessed_input", lambda *args, **kwargs: None)

    panel_tfm.tfm_infer(panel)

    result = captured["result"]
    assert result["ok"] is True
    assert result["image_path"]
    assert result["out_path"]
    assert result["selection_snapshot"]["name"] == "box_lightblue"
    assert result["selection_snapshot"]["world"] == (-0.1, 0.2, 0.88)
    assert Path(result["image_path"]).is_file()
    assert Path(result["out_path"]).is_file()
    assert json.loads(Path(result["out_path"]).read_text(encoding="utf-8"))["cx"] == 10.0


def test_handle_infer_result_restores_selection_from_snapshot(monkeypatch):
    calls = []
    ControlPanelV2 = __import__("ur5_qt_panel.panel_v2", fromlist=["ControlPanelV2"]).ControlPanelV2
    panel = SimpleNamespace(
        _tfm_infer_inflight=True,
        _set_status=lambda *args, **kwargs: None,
        _audit_append=lambda *args, **kwargs: calls.append(("audit", args)),
        _audit_write_json=lambda *args, **kwargs: calls.append(("json", args)),
        _infer_session_id="session",
        camera_topic="/camera_overhead/image",
        _selected_object=None,
        _selected_px=None,
        _selected_world=None,
        _selected_base=None,
        _selected_base_frame="",
        _selection_timestamp=0.0,
        _selection_last_user_name="",
        _selection_last_user_ts=0.0,
        _last_cornell=None,
        _last_cornell_reason="",
        _perf_infer_hist=[],
        _perf_total_hist=[],
        _push_history=lambda values, value, max_len=20: values.append(value),
        _compute_world_grasp=lambda w, h: None,
        _ensure_base_coords=lambda coords, frame, timeout_sec=0.35: None,
        _world_frame_config_first=lambda: "world",
        _business_base_frame=lambda: "base_link",
        _refresh_cornell_metrics=lambda w, h: calls.append(("cornell", panel._selected_object, panel._selected_world, panel._selected_base)),
        _sync_tfm_module_grasp_state=lambda: None,
        _refresh_grasp_overlay_now=lambda: None,
        _refresh_science_ui=lambda: None,
        _emit_log=lambda msg: calls.append(("log", msg)),
    )
    panel._restore_infer_selection_snapshot = lambda snapshot: ControlPanelV2._restore_infer_selection_snapshot(panel, snapshot)

    ControlPanelV2._handle_infer_result(
        panel,
        {
            "ok": True,
            "pred": {"cx": 10.0, "cy": 20.0, "w": 30.0, "h": 40.0, "angle_deg": 5.0},
            "infer_ms": 1.0,
            "total_ms": 2.0,
            "frame_w": 320,
            "frame_h": 240,
            "frame_ts": 123.0,
            "image_path": "/tmp/frame.png",
            "out_path": "/tmp/grasp.json",
            "selection_snapshot": {
                "name": "box_lightblue",
                "px": (-1, -1),
                "world": (-0.1, 0.2, 0.88),
                "base": (0.6, 0.1, 0.1),
                "base_frame": "base_link",
                "timestamp": 99.0,
            },
        },
    )

    assert panel._selected_object == "box_lightblue"
    assert panel._selected_world == (-0.1, 0.2, 0.88)
    assert panel._selected_base == (0.6, 0.1, 0.1)
    assert any(item[0] == "cornell" and item[1] == "box_lightblue" for item in calls)