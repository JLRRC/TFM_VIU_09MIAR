from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
SRC_DIR = ROOT / "src" / "ur5_qt_panel"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from ur5_qt_panel.panel_state import SystemState
from ur5_qt_panel.panel_ui_state import apply_ui_state


class DummyWidget:
    def __init__(self):
        self.enabled = True
        self.tooltip = ""

    def setEnabled(self, value):
        self.enabled = bool(value)

    def setToolTip(self, value):
        self.tooltip = value


class DummyEvent:
    def __init__(self):
        self.was_set = False

    def set(self):
        self.was_set = True


class DummyPanel:
    def __init__(self, has_grasp):
        self._managed_mode = False
        self._bridge_running = True
        self._gz_launching = False
        self._bridge_launching = False
        self._moveit_launching = False
        self._moveit_bridge_launching = False
        self._star_inflight = False
        self._script_motion_active = False
        self._camera_stream_ok = True
        self._robot_test_done = True
        self._robot_test_disabled = False
        self._controllers_ok = True
        self._tf_ready_state = True
        self._ee_frame_effective = "rg2_tcp"
        self._pick_demo_executed = False
        self._pose_info_ok = True
        self._selected_object = "box_blue"
        self._joint_limits_ok = True
        self._joint_limits_err = ""
        self._objects_release_done = True
        self._detach_inflight = False
        self._external_motion_block_reason = None
        self._calibration_topic_allowed = lambda: True
        self._last_grasp_px = (100.0, 120.0, 40.0, 18.0, 12.0) if has_grasp else None
        self._pick_block_reason = None
        self._state_event = DummyEvent()
        self.tfm_module = object()
        self.gz_proc = object()
        self.bridge_proc = object()
        self.bag_proc = None
        self.moveit_proc = object()
        self.moveit_bridge_proc = None

        widget_names = [
            "btn_gz_start",
            "btn_gz_stop",
            "btn_debug_joints",
            "bridge_presets",
            "bridge_edit",
            "btn_bridge_browse",
            "btn_bridge_start",
            "btn_bridge_stop",
            "bag_name",
            "bag_topics",
            "btn_bag_start",
            "btn_bag_stop",
            "btn_moveit_start",
            "btn_moveit_stop",
            "btn_moveit_bridge_start",
            "btn_moveit_bridge_stop",
            "btn_star",
            "btn_kill_hard",
            "camera_topic_combo",
            "btn_camera_refresh",
            "btn_camera_connect",
            "btn_calibrate",
            "btn_release_objects",
            "chk_trace_freeze",
            "btn_trace_diag",
            "btn_copy_trace",
            "btn_save_episode",
            "btn_send_joints",
            "joint_time",
            "chk_auto_joints",
            "btn_test_robot",
            "btn_home",
            "btn_table",
            "btn_basket",
            "btn_gripper",
            "btn_pick_demo",
            "btn_pick_object",
            "combo_tfm_experiment",
            "btn_tfm_apply",
            "btn_tfm_infer",
            "btn_tfm_visualize",
            "btn_tfm_publish",
            "btn_tfm_reset",
            "obj_panel",
            "camera_view",
            "world_combo",
            "mode_combo",
            "btn_world_browse",
        ]
        for name in widget_names:
            setattr(self, name, DummyWidget())
        self.joint_sliders = [DummyWidget() for _ in range(6)]
        self.ros_worker = type("RosWorker", (), {"node_namespace": lambda _self: ""})()

    def _gazebo_state(self):
        return "GAZEBO_READY"

    def _pose_info_active(self):
        return True

    def _proc_alive(self, proc):
        return proc is not None

    def _set_launching_style(self, *_args, **_kwargs):
        return None

    def _moveit_ready(self):
        return True

    def _moveit_bridge_detected(self):
        return False

    def _system_running(self):
        return True

    def _manual_control_status(self):
        return True, ""

    def _select_traj_topic(self):
        return "/joint_trajectory_controller/joint_trajectory"

    def _external_publishers_for_topic(self, _topic):
        return []

    def _set_btn_state(self, btn, enabled, tooltip):
        btn.setEnabled(enabled)
        btn.setToolTip(tooltip)

    def _set_status(self, *_args, **_kwargs):
        return None

    def _emit_log(self, *_args, **_kwargs):
        return None

    def _tf_not_ready_reason(self):
        return ""

    def _set_robot_test_blocked(self, *_args, **_kwargs):
        return None

    def _schedule_controller_check(self):
        return None

    def _moveit_control_status(self):
        return True, ""

    def _update_panel_flow_state(self, **_kwargs):
        return None


def test_tfm_buttons_require_real_grasp_before_visualize_or_execute():
    panel = DummyPanel(has_grasp=False)

    apply_ui_state(panel, SystemState.READY_MOVEIT, "Sistema listo")

    assert panel.btn_tfm_apply.enabled is True
    assert panel.btn_tfm_infer.enabled is True
    assert panel.btn_tfm_visualize.enabled is False
    assert panel.btn_tfm_publish.enabled is False
    assert panel.btn_tfm_reset.enabled is True
    assert panel.btn_tfm_visualize.tooltip == "Primero infiere o recibe un grasp"
    assert panel.btn_tfm_publish.tooltip == "Primero infiere o recibe un grasp"
    assert panel._state_event.was_set is True


def test_tfm_buttons_enable_visualize_and_execute_when_grasp_exists():
    panel = DummyPanel(has_grasp=True)

    apply_ui_state(panel, SystemState.READY_MOVEIT, "Sistema listo")

    assert panel.btn_tfm_apply.enabled is True
    assert panel.btn_tfm_infer.enabled is True
    assert panel.btn_tfm_visualize.enabled is True
    assert panel.btn_tfm_publish.enabled is True
    assert panel.btn_tfm_reset.enabled is True
    assert panel.btn_tfm_visualize.tooltip == ""
    assert panel.btn_tfm_publish.tooltip == ""