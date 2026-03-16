import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "src" / "ur5_qt_panel"))

from ur5_qt_panel.panel_v2 import CameraController


class _Label:
    def __init__(self):
        self.text = ""
        self.style = ""

    def setText(self, value):
        self.text = value

    def setStyleSheet(self, value):
        self.style = value


class _Signal:
    def __init__(self):
        self.calls = 0

    def emit(self):
        self.calls += 1


class _PanelStub:
    def __init__(self):
        self._camera_required = True
        self._bridge_running = True
        self._camera_stream_ok = False
        self._camera_reconnect_scheduled = False
        self._camera_reconnect_attempts = 0
        self._camera_reconnect_last_reason = ""
        self._camera_reconnect_base_delay_ms = 1500
        self._camera_reconnect_max_delay_ms = 6000
        self._camera_subscribed = True
        self._camera_frame_count = 0
        self._camera_depth_frame_count = 0
        self._last_camera_depth_frame_ts = 0.0
        self._camera_fault_age_sec = 4.0
        self._camera_fault_since = 0.0
        self._camera_fault_active = False
        self._camera_fault_reason = ""
        self._debug_logs_enabled = False
        self._last_camera_diag_log = 0.0
        self.camera_topic = "/camera_overhead/image"
        self.camera_info = _Label()
        self.signal_connect_camera = _Signal()
        self.logs = []
        self.throttled = []

    def _camera_runtime_flags(self, _now):
        return False, False, False, float("inf"), False

    def _camera_depth_expectation(self):
        return False, ""

    def _emit_log(self, msg):
        self.logs.append(msg)

    def _emit_log_throttled(self, key, msg, min_interval=0.0):
        self.throttled.append((key, msg, min_interval))


def test_health_check_programs_reconnect_when_subscribed_without_frames(monkeypatch):
    panel = _PanelStub()
    ctrl = CameraController(panel)
    ctrl.auto_connect = lambda: panel.signal_connect_camera.emit()
    scheduled = {}

    def fake_single_shot(delay_ms, callback):
        scheduled["delay_ms"] = delay_ms
        scheduled["callback"] = callback

    monkeypatch.setattr("ur5_qt_panel.panel_v2.QTimer.singleShot", fake_single_shot)

    ctrl.health_check()

    assert panel.camera_info.text == "Sin imágenes"
    assert panel._camera_reconnect_scheduled is True
    assert scheduled["delay_ms"] == 1500

    scheduled["callback"]()

    assert panel.signal_connect_camera.calls == 1
    assert panel._camera_reconnect_attempts == 1
    assert panel._camera_reconnect_last_reason == "camera_no_frames"


def test_schedule_reconnect_uses_backoff_and_stops_when_stream_recovers(monkeypatch):
    panel = _PanelStub()
    ctrl = CameraController(panel)
    ctrl.auto_connect = lambda: panel.logs.append("auto_connect")
    delays = []
    callbacks = []

    def fake_single_shot(delay_ms, callback):
        delays.append(delay_ms)
        callbacks.append(callback)

    monkeypatch.setattr("ur5_qt_panel.panel_v2.QTimer.singleShot", fake_single_shot)

    ctrl.schedule_reconnect("camera_fault")
    assert delays == [1500]

    callbacks.pop(0)()
    assert panel.logs[-1] == "auto_connect"
    assert panel._camera_reconnect_attempts == 1

    ctrl.schedule_reconnect("camera_fault")
    assert delays[-1] == 3000

    panel._camera_stream_ok = True
    callbacks.pop(0)()
    assert panel.logs.count("auto_connect") == 1