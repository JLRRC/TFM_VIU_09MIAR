import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent / "src" / "ur5_qt_panel"))

from ur5_qt_panel.panel_ros import RosWorker
from ur5_qt_panel.panel_v2 import ControlPanelV2
from ur5_qt_panel import panel_objects


def test_ros_worker_emits_remote_object_selection():
    worker = RosWorker()
    captured = []
    worker.object_select_request.connect(lambda name, source: captured.append((name, source)))

    worker._on_select_object_topic(SimpleNamespace(data="box_lightblue"))

    assert captured == [("box_lightblue", "topic:/panel/select_object")]


def test_ros_worker_select_object_service_waits_for_panel_ack():
    worker = RosWorker()

    def fake_emit(name, source):
        request_id = source.rsplit("#", 1)[-1]
        worker.complete_object_select_request(request_id, True, f"selected:{name}")

    worker._emit_object_select_request = fake_emit
    request = SimpleNamespace(name="box_lightblue")
    response = SimpleNamespace(success=False, message="")

    result = worker._on_select_object_service(request, response)

    assert result.success is True
    assert result.message == "selected:box_lightblue"


def test_remote_object_selection_clear_resets_selection(monkeypatch):
    calls = []
    selected_state = object()
    panel = SimpleNamespace(
        _selected_object="box_lightblue",
        _selected_px=(10, 20),
        _selected_world=(1.0, 2.0, 3.0),
        _selected_base=(0.1, 0.2, 0.3),
        _selection_timestamp=12.0,
        _selection_last_user_name="box_lightblue",
        _selection_last_user_ts=12.0,
        _last_cornell={"success": True},
        _last_cornell_reason="ok",
        _emit_log=lambda msg: calls.append(("log", msg)),
        _refresh_science_ui=lambda: calls.append(("science", None)),
        _update_objects=lambda: calls.append(("objects", None)),
        _set_status=lambda msg, error=False: calls.append(("status", msg, error)),
        _on_object_clicked=lambda name: calls.append(("select", name)),
    )
    monkeypatch.setattr(
        "ur5_qt_panel.panel_v2.get_object_state",
        lambda _name: SimpleNamespace(logical_state=selected_state),
    )
    monkeypatch.setattr(
        "ur5_qt_panel.panel_v2.ObjectLogicalState",
        SimpleNamespace(SELECTED=selected_state, ON_TABLE="ON_TABLE"),
    )
    monkeypatch.setattr("ur5_qt_panel.panel_v2.update_object_state", lambda *args, **kwargs: calls.append(("update", args, kwargs)))

    ControlPanelV2._on_remote_object_select_request(panel, "clear", "topic:/panel/select_object")

    assert panel._selected_object is None
    assert panel._selected_px is None
    assert panel._selected_world is None
    assert panel._selected_base is None
    assert panel._last_cornell is None
    assert panel._last_cornell_reason == "Selección limpiada remotamente"
    assert any(item[0] == "update" for item in calls)


def test_remote_object_selection_forwards_name_to_existing_selection_flow():
    calls = []
    panel = SimpleNamespace(
        _emit_log=lambda msg: calls.append(("log", msg)),
        _on_object_clicked=lambda name: calls.append(("select", name)),
        _ros_worker_started=False,
    )

    ControlPanelV2._on_remote_object_select_request(panel, "box_lightblue", "topic:/panel/select_object")

    assert ("select", "box_lightblue") in calls


def test_remote_object_selection_prefers_live_pose_info(monkeypatch):
    calls = []
    panel = SimpleNamespace(
        _emit_log=lambda msg: calls.append(("log", msg)),
        _on_object_clicked=lambda name: calls.append(("fallback", name)),
        _objects_settled=False,
        _ros_worker_started=True,
        ros_worker=SimpleNamespace(pose_snapshot=lambda: ({"box_lightblue": (-0.1, 0.2, 0.85)}, 1.0)),
        camera_view=SimpleNamespace(_img_width=0, _img_height=0),
        _camera_stream_ok=False,
        _pose_info_ok=True,
        _select_object=lambda name, px, py, x, y, z, source="unknown": calls.append(("select_object", name, px, py, x, y, z, source)),
    )
    monkeypatch.setattr("ur5_qt_panel.panel_v2.world_xyz_to_pixel", lambda *args, **kwargs: None)
    monkeypatch.setattr("ur5_qt_panel.panel_v2.table_xy_to_pixel", lambda *args, **kwargs: None)

    ControlPanelV2._on_remote_object_select_request(panel, "box_lightblue", "topic:/panel/select_object")

    assert any(item[0] == "select_object" and item[-1] == "remote_pose" for item in calls)
    assert not any(item[0] == "fallback" for item in calls)


def test_remote_object_selection_service_ack_reports_success(monkeypatch):
    calls = []
    panel = SimpleNamespace(
        _emit_log=lambda msg: calls.append(("log", msg)),
        _on_object_clicked=lambda name: calls.append(("fallback", name)),
        _objects_settled=False,
        _ros_worker_started=True,
        ros_worker=SimpleNamespace(
            pose_snapshot=lambda: ({"box_lightblue": (-0.1, 0.2, 0.85)}, 1.0),
            complete_object_select_request=lambda request_id, success, message: calls.append(("ack", request_id, success, message)),
        ),
        camera_view=SimpleNamespace(_img_width=0, _img_height=0),
        _camera_stream_ok=False,
        _pose_info_ok=True,
        _selected_object=None,
        _select_object=lambda name, px, py, x, y, z, source="unknown": calls.append(("select_object", name, px, py, x, y, z, source)) or setattr(panel, "_selected_object", name),
    )
    monkeypatch.setattr("ur5_qt_panel.panel_v2.world_xyz_to_pixel", lambda *args, **kwargs: None)
    monkeypatch.setattr("ur5_qt_panel.panel_v2.table_xy_to_pixel", lambda *args, **kwargs: None)

    ControlPanelV2._on_remote_object_select_request(panel, "box_lightblue", "service:/panel/select_object#req-1")

    assert ("ack", "req-1", True, "selected") in calls


def test_remote_object_selection_syncs_store_from_live_pose_when_settled(monkeypatch):
    calls = []
    panel = SimpleNamespace(
        _emit_log=lambda msg: calls.append(("log", msg)),
        _on_object_clicked=lambda name: calls.append(("fallback", name)),
        _objects_settled=True,
        _ros_worker_started=True,
        ros_worker=SimpleNamespace(pose_snapshot=lambda: ({"box_lightblue": (-0.1, 0.2, 0.85)}, 1.0)),
        camera_view=SimpleNamespace(_img_width=0, _img_height=0),
        _camera_stream_ok=False,
        _pose_info_ok=True,
        _select_object=lambda name, px, py, x, y, z, source="unknown": calls.append(("select_object", name, px, py, x, y, z, source)),
    )
    monkeypatch.setattr("ur5_qt_panel.panel_v2.world_xyz_to_pixel", lambda *args, **kwargs: None)
    monkeypatch.setattr("ur5_qt_panel.panel_v2.table_xy_to_pixel", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        "ur5_qt_panel.panel_v2.bulk_update_object_positions",
        lambda updates, source="unknown", read_only=False, objects_stable=False: calls.append(
            ("bulk_update", updates, source, read_only, objects_stable)
        ),
    )
    monkeypatch.setattr(
        "ur5_qt_panel.panel_v2.recalc_object_states",
        lambda reason="recalc": calls.append(("recalc", reason)),
    )

    ControlPanelV2._on_remote_object_select_request(panel, "box_lightblue", "topic:/panel/select_object")

    assert any(
        item[0] == "bulk_update"
        and item[1] == {"box_lightblue": (-0.1, 0.2, 0.85)}
        and item[2] == "remote_pose_select"
        and item[4] is True
        for item in calls
    )
    assert ("recalc", "remote_select_pose_sync") in calls
    assert any(item[0] == "select_object" and item[-1] == "remote_pose" for item in calls)


def test_remote_object_selection_syncs_store_when_live_pose_is_on_table(monkeypatch):
    calls = []
    panel = SimpleNamespace(
        _emit_log=lambda msg: calls.append(("log", msg)),
        _on_object_clicked=lambda name: calls.append(("fallback", name)),
        _objects_settled=False,
        _ros_worker_started=True,
        ros_worker=SimpleNamespace(pose_snapshot=lambda: ({"box_lightblue": (-0.1, 0.2, 0.88)}, 1.0)),
        camera_view=SimpleNamespace(_img_width=0, _img_height=0),
        _camera_stream_ok=False,
        _pose_info_ok=True,
        _select_object=lambda name, px, py, x, y, z, source="unknown": calls.append(("select_object", name, px, py, x, y, z, source)),
    )
    monkeypatch.setattr("ur5_qt_panel.panel_v2.world_xyz_to_pixel", lambda *args, **kwargs: None)
    monkeypatch.setattr("ur5_qt_panel.panel_v2.table_xy_to_pixel", lambda *args, **kwargs: None)
    monkeypatch.setattr("ur5_qt_panel.panel_v2.is_on_table", lambda pose: True)
    monkeypatch.setattr(
        "ur5_qt_panel.panel_v2.bulk_update_object_positions",
        lambda updates, source="unknown", read_only=False, objects_stable=False: calls.append(
            ("bulk_update", updates, source, read_only, objects_stable)
        ),
    )
    monkeypatch.setattr(
        "ur5_qt_panel.panel_v2.recalc_object_states",
        lambda reason="recalc": calls.append(("recalc", reason)),
    )

    ControlPanelV2._on_remote_object_select_request(panel, "box_lightblue", "topic:/panel/select_object")

    assert any(
        item[0] == "bulk_update"
        and item[1] == {"box_lightblue": (-0.1, 0.2, 0.88)}
        and item[2] == "remote_pose_select"
        and item[4] is True
        for item in calls
    )
    assert ("recalc", "remote_select_pose_sync") in calls
    assert any(item[0] == "select_object" and item[-1] == "remote_pose" for item in calls)


def test_handle_objects_settled_refreshes_positions_before_recalc(monkeypatch):
    calls = []
    panel = SimpleNamespace(
        _refresh_objects_from_gz=lambda: calls.append("refresh_objects"),
        _refresh_controls=lambda: calls.append("refresh_controls"),
        _bridge_running=True,
        _calibration_ready=False,
        signal_calibration_check=SimpleNamespace(emit=lambda: calls.append("emit_calibration")),
    )
    monkeypatch.setattr(
        "ur5_qt_panel.panel_v2.recalc_object_states",
        lambda reason="recalc": calls.append(("recalc", reason)),
    )

    ControlPanelV2._handle_objects_settled(panel)

    assert calls[0] == "refresh_objects"
    assert calls[1] == ("recalc", "settled")
    assert "refresh_controls" in calls
    assert "emit_calibration" in calls


def test_update_object_state_allows_spawned_to_selected_when_on_table(monkeypatch):
    obj = panel_objects.ObjectState(
        name="pick_demo",
        logical_state=panel_objects.ObjectLogicalState.SPAWNED,
        owner=panel_objects.ObjectOwner.NONE,
        attached=False,
        position=(0.0, 0.0, 0.88),
        last_update_ts=0.0,
    )
    monkeypatch.setattr(panel_objects, "_ensure_loaded", lambda: None)
    monkeypatch.setattr(panel_objects, "OBJECT_POSITIONS", {"pick_demo": (0.0, 0.0, 0.88)})
    monkeypatch.setattr(panel_objects, "_ensure_object_state", lambda name, position: obj)
    monkeypatch.setattr(panel_objects, "_on_table_geometry", lambda position: True)

    ok = panel_objects.update_object_state(
        "pick_demo",
        logical_state=panel_objects.ObjectLogicalState.SELECTED,
        owner=panel_objects.ObjectOwner.NONE,
        attached=False,
        reason="test_spawned_to_selected",
    )

    assert ok is True
    assert obj.logical_state == panel_objects.ObjectLogicalState.SELECTED


def test_build_reference_grasp_prefers_live_pose_snapshot(monkeypatch):
    pose_calls = []
    pixel_calls = []
    panel = SimpleNamespace(
        _selected_object="box_lightblue",
        _selected_world=(9.0, 9.0, 9.0),
        _selected_base=None,
        _selected_px=(-1, -1),
        _ros_worker_started=True,
        ros_worker=SimpleNamespace(pose_snapshot=lambda: ({"box_lightblue": (-0.1, 0.2, 0.88)}, 1.0)),
        _sdf_model_cache={"box_lightblue": {"type": "box", "size": (0.04, 0.06, 0.03)}},
        _load_sdf_geometry_cache=lambda: None,
        _resolve_table_top_z=lambda: 0.85,
        _ensure_base_coords=lambda coords, frame, timeout_sec=0.35: pose_calls.append((coords, frame, timeout_sec)) or (0.6, 0.1, 0.1),
        _world_frame_last_first=lambda: "world",
        _business_base_frame=lambda: "base_link",
        _audit_append=lambda *args, **kwargs: None,
    )

    def fake_world_to_pixel_diag(x, y, z, w, h):
        pixel_calls.append((round(x, 3), round(y, 3), round(z, 3), w, h))
        if abs(x + 0.1) < 1e-6 and abs(y - 0.2) < 1e-6:
            return (160, 120), (160.0, 120.0), "world_xyz"
        if abs(x + 0.12) < 1e-6 and abs(y - 0.2) < 1e-6:
            return (150, 120), (150.0, 120.0), "world_xyz"
        if abs(x + 0.08) < 1e-6 and abs(y - 0.2) < 1e-6:
            return (170, 120), (170.0, 120.0), "world_xyz"
        if abs(x + 0.1) < 1e-6 and abs(y - 0.17) < 1e-6:
            return (160, 105), (160.0, 105.0), "world_xyz"
        if abs(x + 0.1) < 1e-6 and abs(y - 0.23) < 1e-6:
            return (160, 135), (160.0, 135.0), "world_xyz"
        return None, None, "none"

    panel._world_to_pixel_diag = fake_world_to_pixel_diag

    ref = ControlPanelV2._build_reference_grasp(panel, 320, 240)

    assert ref == {"cx": 160.0, "cy": 120.0, "w": 20.0, "h": 30.0, "angle_deg": 0.0}
    assert panel._selected_world == (-0.1, 0.2, 0.88)
    assert panel._selected_base == (0.6, 0.1, 0.1)
    assert panel._selected_px == (160, 120)
    assert pose_calls == [((-0.1, 0.2, 0.88), "world", 0.35)]
    assert pixel_calls[0][:3] == (-0.1, 0.2, 0.88)


def test_build_reference_grasp_logs_pose_source_when_projection_fails():
    calls = []
    panel = SimpleNamespace(
        _selected_object="box_lightblue",
        _selected_world=None,
        _selected_base=None,
        _selected_px=None,
        _ros_worker_started=True,
        ros_worker=SimpleNamespace(pose_snapshot=lambda: ({"box_lightblue": (-0.1, 0.2, 0.88)}, 1.0)),
        _sdf_model_cache={"box_lightblue": {"type": "box", "size": (0.04, 0.06, 0.03)}},
        _load_sdf_geometry_cache=lambda: None,
        _resolve_table_top_z=lambda: 0.85,
        _ensure_base_coords=lambda coords, frame, timeout_sec=0.35: None,
        _world_frame_last_first=lambda: "world",
        _business_base_frame=lambda: "base_link",
        _world_to_pixel_diag=lambda *args, **kwargs: (None, None, "none"),
        _audit_append=lambda *args: calls.append(args),
    )

    ref = ControlPanelV2._build_reference_grasp(panel, 320, 240)

    assert ref is None
    assert any("pose_source=pose_snapshot" in entry[1] for entry in calls)