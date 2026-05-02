#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""
Panel V2 minimal: barra de control con los botones mostrados, sin logs ni dependencias del panel existente.
Ejecutar con: python -m ur5_qt_panel.panel_v2
"""
from __future__ import annotations

import os
import math
import signal
import sys
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from ur5_tools.gripper_geometry import (
    tool0_offset_for_frame,
)

try:
    import psutil  # type: ignore
except Exception:
    psutil = None
import numpy as np
from PyQt5.QtCore import QTimer, pyqtSignal, QThread
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QGroupBox,
    QLabel,
    QMainWindow,
    QPushButton,
    QTableWidget,
)
try:
    import yaml  # type: ignore
except Exception:
    yaml = None

from .panel_config import (
    DROP_OBJECTS,
    WORLD_FRAME,
    WS_DIR,
    GZ_LAUNCH_TIMEOUT_SEC,
    TRACE_PRINT_PERIOD_SEC,
    PANEL_MANAGED,
    PANEL_MOVEIT_REQUIRED,
    CAMERA_READY_FRAMES,
    CAMERA_INIT_GRACE_SEC,
    CAMERA_READY_MAX_AGE_SEC,
    CAMERA_REQUIRED,
    CRITICAL_CLOCK_TIMEOUT_SEC,
    GZ_SERVICE_CHECK_SEC,
    CRITICAL_CLOCK_TIMEOUT_SEC,
    AUTO_CALIB_FROM_CAMERA,
    DEBUG_LOGS_TO_STDOUT,
    DEBUG_JOINTS_TO_STDOUT,
    PANEL_JOINT_STATES_TOPIC,
    PANEL_CAMERA_TOPIC,
    USE_SIM_TIME,
    VISION_DIR,
    INFER_CKPT,
    PICK_DEMO_SPAWN_POSE,
)
from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params
from .panel_utils import (
    CmdRunner,
    RosWorker,
    get_object_positions,
    load_object_positions,
    rclpy,
)
from .panel_state import (
    MoveItState,
    PanelStateEvaluator,
    SystemState,
)
from .panel_physics import PanelPhysics
from .panel_tf_monitor import TFMonitor
from .panel_moveit_publishers import init_moveit_publishers
from .panel_tfm import (
    tfm_infer_grasp,
    tfm_canonical_use_pick_object,
    complete_pending_tfm_infer_request,
    complete_pending_tfm_execute_request,
    complete_pending_pick_demo_request,
    build_tfm_pick_object_override,
    tfm_canonical_state_reset, tfm_canonical_phase_update,
    tfm_canonical_finish, restore_infer_selection_snapshot,
    latest_camera_frame_snapshot, ensure_selected_object_in_store,
    handle_infer_result, sync_tfm_module_grasp_state, tfm_visualize_grasp,
    wait_tfm_moveit_result, execute_tfm_world_grasp,
    on_tfm_grasp_object_clicked, tfm_publish_grasp,
)
from .tf_pose_utils import (
    get_transform as tf_get_transform,
)
from .panel_safety import PanelSafety
from .panel_state_machine import PanelStateMachine
from .panel_watchdog import PanelWatchdog
from .panel_env import (
    collect_env_diagnostics,
    format_env_diagnostics,
    validate_env,
)
from .logging_utils import _PanelLogger, timestamped_line
from .panel_camera import CameraController
from .panel_trace_ui import build_trace_group, build_science_group
from . import panel_remote_callbacks as _rc
from . import panel_trace_callbacks as _tc
from . import panel_draw_overlays as _do
from . import panel_object_mgmt as _om
from . import panel_calib_selection as _cs
from . import panel_tfm_science as _ts
from . import panel_status_mgmt as _sm
from . import panel_gz_startup as _gs
from . import panel_helpers as _ph
from . import panel_state_methods as _stm
from . import panel_motion_control as _mc
from . import panel_calib_actions as _ca
from . import panel_shutdown as _sd

from rclpy.node import Node
try:
    from tfm_grasping import TFMGraspModule
except Exception:
    TFMGraspModule = None
try:
    from rclpy.parameter import Parameter
except Exception:
    Parameter = None
from std_msgs.msg import Float32MultiArray
try:
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point
    from builtin_interfaces.msg import Duration
except Exception:
    Marker = None
    Point = None
    Duration = None
try:
    from controller_manager_msgs.srv import ListControllers
except Exception:
    ListControllers = None
try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity as GzEntity
except Exception:
    SetEntityPose = None
    GzEntity = None
try:
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.action import MoveGroup
except Exception:
    ActionClient = None
    FollowJointTrajectory = None
    MoveGroup = None

CAMERA_TOPIC_PREFIX = "/camera"
# Contract: panel publishes PoseStamped on these topics in BASE_FRAME business coordinates.
MOVEIT_POSE_TOPIC = "/desired_grasp"
MOVEIT_CARTESIAN_POSE_TOPIC = "/desired_grasp_cartesian"
GLOBAL_FRAME_EFFECTIVE = "base_link"
GRASP_RECT_TOPIC = _get_panel_ui_params().grasp_rect_topic
TEST_CORNER_OVERLAY = _get_panel_ui_params().test_corner_overlay
TCP_POSE_OVERLAY = _get_panel_ui_params().tcp_pose_overlay
TCP_POSE_TEXT_OVERLAY = _get_panel_ui_params().tcp_pose_text_overlay
FAR_FRONT_CAMERA_TOPIC_CANDIDATES = (
    "/camera_west/image",
    "/camera_south/image",
    "/camera_north/image",
    "/camera_east/image",
)
TOP_CAMERA_TOPIC_CANDIDATES = (
    "/camera_debug_top/image",
)
WRIST_CAMERA_TOPIC_CANDIDATES = (
    "/camera_wrist/image",
)

_DEBUG_EXCEPTIONS = _get_panel_ui_params().debug_exceptions


def _log_exception(context: str, exc: Exception) -> None:
    if not _DEBUG_EXCEPTIONS:
        return
    print(timestamped_line(f"[PANEL_V2][WARN] {context}: {exc}"), file=sys.stderr, flush=True)


# F14 (2026-05-01): helpers puros extraídos a panel_v2_helpers.
# Re-exportados con sus nombres legacy con underscore para no romper
# el resto de panel_v2.py.
from .panel_v2_helpers import (  # noqa: E402,F401
    camera_required_label as _camera_required_label,
    env_flag as _env_flag,
    env_float as _env_float,
    runtime_time as _runtime_time,
)


# TFM Grasp geometry helpers (compartidos con panel_tfm via panel_tfm_geometry).
from .panel_tfm_geometry import (  # noqa: E402,F401
    _compute_minor_axis_from_grasp_rect,
    _compute_rg2_preopen_from_minor_width,
    _tfm_clamp,
    _tfm_normalize_angle,
)


SETTLE_MANUAL = {"pick_demo"}
CONTROLLER_CHECK_INTERVAL_SEC = 3.0
CONTROLLER_LIST_RETRY_WINDOW_SEC = max(
    0.0, _env_float("PANEL_CTRL_LIST_RETRY_WINDOW_SEC", 5.0)
)
CONTROLLER_LAST_OK_GRACE_SEC = max(
    0.0, _env_float("PANEL_CTRL_LAST_OK_GRACE_SEC", 5.0)
)
CONTROLLER_LIST_RETRY_STEP_SEC = 0.25


from .panel_v2_helpers import (  # noqa: E402,F401
    proto_time_to_seconds as _proto_time_to_seconds,
)


def _load_cornell_metrics(vision_dir: str):
    if vision_dir:
        if vision_dir not in sys.path:
            sys.path.append(vision_dir)
        vision_src = os.path.join(vision_dir, "src")
        if os.path.isdir(vision_src) and vision_src not in sys.path:
            sys.path.append(vision_src)
    try:
        from graspnet.utils.metrics import (  # type: ignore
            angle_diff_deg,
            compute_grasp_success,
            grasp_iou,
        )
    except Exception as exc:
        try:
            from src.training.metrics import (  # type: ignore
                angle_error_deg as _angle_error_batch,
                cornell_success as _cornell_success_batch,
                iou_axis_aligned_boxes as _iou_batch,
            )

            def grasp_iou(pred, ref) -> float:
                p = np.asarray(pred, dtype=np.float64).reshape(1, -1)[:, :5]
                g = np.asarray(ref, dtype=np.float64).reshape(1, -1)[:, :5]
                return float(_iou_batch(p, g)[0])

            def angle_diff_deg(pred_angle: float, ref_angle: float) -> float:
                p = np.asarray([pred_angle], dtype=np.float64)
                g = np.asarray([ref_angle], dtype=np.float64)
                return float(_angle_error_batch(p, g)[0])

            def compute_grasp_success(
                pred,
                ref,
                *,
                iou_thresh: float = 0.25,
                angle_thresh: float = 30.0,
            ) -> bool:
                p = np.asarray(pred, dtype=np.float64).reshape(1, -1)[:, :5]
                g = np.asarray(ref, dtype=np.float64).reshape(1, -1)[:, :5]
                return bool(
                    _cornell_success_batch(p, g, iou_thr=iou_thresh, angle_thr=angle_thresh)[0]
                )
        except Exception as fallback_exc:
            return None, f"{exc} | fallback src.training.metrics: {fallback_exc}"
    return {
        "angle_diff_deg": angle_diff_deg,
        "compute_grasp_success": compute_grasp_success,
        "grasp_iou": grasp_iou,
    }, ""


from .calibration_service import CalibrationService


def _canonical_tool0_to_semantic_frame(
    frame_name: str,
) -> tuple[float, float, float] | None:
    frame = str(frame_name or "").strip()
    if frame not in {"rg2_pinch_center", "rg2_tcp"}:
        return None
    return tool0_offset_for_frame(frame)


def _fk_model_to_base_link(
    pos_model: tuple[float, float, float] | list[float] | np.ndarray,
    rot_model: np.ndarray,
) -> tuple[tuple[float, float, float], np.ndarray]:
    """Convert UR5 FK output from base_link_inertia into runtime base_link."""
    rz_pi = np.array(
        [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    base_pos = (
        -float(pos_model[0]),
        -float(pos_model[1]),
        float(pos_model[2]),
    )
    base_rot = rz_pi @ np.asarray(rot_model, dtype=float)
    return base_pos, base_rot


def _fk_tool0_to_ee_base_link(
    pos_model: tuple[float, float, float] | list[float] | np.ndarray,
    rot_model: np.ndarray,
    ee_frame: str,
) -> tuple[tuple[float, float, float], np.ndarray]:
    base_pos, base_rot = _fk_model_to_base_link(pos_model, rot_model)
    frame_name = str(ee_frame or "").strip()
    if not frame_name or frame_name == "tool0":
        return base_pos, base_rot

    local_offset = None
    tf_tool0_ee, _tf_reason = tf_get_transform("tool0", frame_name, timeout=0.05, logger=None)
    if tf_tool0_ee is not None:
        translation = tf_tool0_ee.transform.translation
        local_offset = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )
    else:
        local_offset = _canonical_tool0_to_semantic_frame(frame_name)

    if local_offset is None:
        return base_pos, base_rot

    offset_base = tuple((np.asarray(base_rot, dtype=float) @ np.asarray(local_offset, dtype=float)).tolist())
    return (
        (
            float(base_pos[0]) + float(offset_base[0]),
            float(base_pos[1]) + float(offset_base[1]),
            float(base_pos[2]) + float(offset_base[2]),
        ),
        base_rot,
    )


from .panel_v2_publisher_mixin import PanelV2PublisherMixin  # noqa: E402
from .panel_v2_base_pose_mixin import PanelV2BasePoseMixin  # noqa: E402
from .panel_v2_gripper_attach_mixin import PanelV2GripperAttachMixin  # noqa: E402
from .panel_v2_motion_mixin import PanelV2MotionMixin  # noqa: E402
from .panel_v2_traj_settle_mixin import PanelV2TrajSettleMixin  # noqa: E402
from .panel_v2_system_state_mixin import PanelV2SystemStateMixin  # noqa: E402
from .panel_v2_step_debug_mixin import PanelV2StepDebugMixin  # noqa: E402
from .panel_v2_runtime_diagnostics_mixin import (  # noqa: E402
    PanelV2RuntimeDiagnosticsMixin,
)


class ControlPanelV2(
    PanelV2PublisherMixin,
    PanelV2BasePoseMixin,
    PanelV2GripperAttachMixin,
    PanelV2MotionMixin,
    PanelV2TrajSettleMixin,
    PanelV2SystemStateMixin,
    PanelV2StepDebugMixin,
    PanelV2RuntimeDiagnosticsMixin,
    QMainWindow,
):
    retry_send_joints = pyqtSignal()
    status_updated = pyqtSignal(bool, bool, bool, bool, bool, bool, bool)
    signal_status = pyqtSignal(str, bool)
    signal_refresh_controls = pyqtSignal()
    signal_set_led = pyqtSignal(object, str)
    signal_update_objects = pyqtSignal()
    signal_start_objects_settle_watch = pyqtSignal()
    signal_handle_objects_settled = pyqtSignal()
    signal_schedule_camera_health_check = pyqtSignal(int)
    signal_update_camera_topics = pyqtSignal(object)
    signal_connect_camera = pyqtSignal()
    signal_request_auto_bridge_start = pyqtSignal()
    signal_bridge_ready = pyqtSignal()
    signal_calibration_check = pyqtSignal()
    signal_trace_ready = pyqtSignal()
    signal_schedule_home_offset = pyqtSignal(int, int)
    signal_close_panel = pyqtSignal()
    signal_tf_ready = pyqtSignal(bool)
    signal_calib_ready = pyqtSignal(bool)
    signal_controllers_ready = pyqtSignal(bool)
    signal_error = pyqtSignal(str)
    signal_moveit_state = pyqtSignal(str, str)
    signal_run_ui = pyqtSignal(object)
    signal_run_ui_delayed = pyqtSignal(object, int)
    def _ui_set_status(self, text: str, error: bool = False) -> None:
        self.signal_status.emit(text, error)

    def _camera_health_check(self):
        """Chequeo periódico: si no llegan imágenes, log throttled y alerta visual."""
        self._camera_ctrl.health_check()

    def get_health_report(self, *args, **kwargs):
        return _sm.get_health_report(self, *args, **kwargs)


    def __init__(self):
        super().__init__()
        # F14-step2: PanelV2PublisherMixin lee este flag en lugar de la
        # constante global USE_SIM_TIME para no introducir un import
        # circular en el mixin.
        self._use_sim_time_flag = bool(USE_SIM_TIME)
        self._state_event = threading.Event()
        self._debug_motion_lock = threading.Lock()
        self._debug_motion_continue_event = threading.Event()
        self._step_gate_lock = threading.Lock()
        self._step_wait_event = threading.Event()
        self._direct_wait_for_approach_event = threading.Event()
        self._direct_flow_state = ""
        self._direct_flow_request_id = ""
        self._debug_motion_wait_active = False
        self._debug_motion_wait_reason = ""
        self._step_mode = "AUTO"
        self._step_wait_active = False
        self._step_pending_phase = ""
        self._step_pending_flow = ""
        self._step_current_phase = ""
        self._step_next_phase = ""
        self._step_running_phase = ""
        self._step_decision = ""
        self._step_phase_position: Optional[Tuple[float, float, float]] = None
        self._step_object_position: Optional[Tuple[float, float, float]] = None
        self._step_start_pose_base: Optional[Tuple[float, float, float]] = None
        self._step_start_pose_rpy_deg: Optional[Tuple[float, float, float]] = None
        self._step_start_trigger = ""
        self._step_history_flow = ""
        self._step_history_rows: List[Dict[str, object]] = []
        self._step_history_target_xy_tol_m = max(0.001, _env_float("PANEL_STEP_HISTORY_TARGET_XY_TOL_M", 0.02))
        self._step_history_target_z_tol_m = max(0.001, _env_float("PANEL_STEP_HISTORY_TARGET_Z_TOL_M", 0.02))
        self._step_mode_combo_syncing = False
        self._step_window = None
        self._step_pipeline_flow = ""
        self._step_pipeline_table = None
        self._step_pipeline_help_label = None
        self._step_pipeline_row_index: Dict[str, int] = {}
        self._step_pipeline_buttons: Dict[str, QPushButton] = {}
        self._step_phase_label = None
        self._step_mode_label = None
        self._step_current_label = None
        self._step_next_label = None
        self._step_intent_label = None
        self._step_decision_label = None
        self._step_target_label = None
        self._step_live_operational_label = None
        self._step_live_visual_label = None
        self._step_gripper_expected_label = None
        self._step_live_gripper_label = None
        self._step_object_label = None
        self._step_start_pose_label = None
        self._step_runtime_section = None
        self._step_runtime_help_label = None
        self._step_runtime_block_labels: Dict[str, Dict[str, QLabel]] = {}
        self._step_history_table = None
        self._step_continue_btn = None
        self._step_continue_help_label = None
        self._step_cart_debug_window = None
        self._step_cart_debug_pose_label = None
        self._step_cart_debug_tool0_label = None
        self._step_cart_debug_frame_label = None
        self._step_cart_debug_time_label = None
        self._step_cart_debug_status_label = None
        self._step_cart_debug_step_combo = None
        self._step_cart_debug_timer = None
        self._step_cart_debug_move_inflight = False
        self._step_cart_debug_last_sample_wall = 0.0
        self._debug_motion_pause_alcance_enabled = _get_panel_ui_params().debug_pause_alcance
        self._debug_motion_pause_timeout_sec = max(
            0.0,
            _get_panel_ui_params().debug_pause_timeout_sec,
        )
        self.setWindowTitle("Panel V2")
        self.setMinimumWidth(1200)
        load_object_positions()
        self.ws_dir = WS_DIR
        self.gz_proc = None
        self.gz_gui_proc = None
        self.bridge_proc = None
        self.bag_proc = None
        self.moveit_proc = None
        self.moveit_bridge_proc = None
        self.release_service_proc = None
        self.world_tf_proc = None
        self.gz_pose_proc = None
        self.rsp_proc = None
        self.gz_partition = ""
        self._status_check_inflight = False
        self._gz_running = False
        self._gz_state = "GAZEBO_OFF"
        self._gz_state_pending = ""
        self._gz_state_change_ts = 0.0
        self._last_clock_ok_ts = 0.0
        self._gz_clock_stall_since = 0.0
        self._gz_orphan_since = 0.0
        self._gz_root_pid = 0
        self._gz_real_pid = 0
        self._gz_pgid = 0
        self._gz_health_freeze_sec = max(2.0, _env_float("PANEL_GZ_HEALTH_FREEZE_SEC", 4.0))
        self._gz_world_name = None
        self._bridge_running = False
        self._moveit_running = False
        self._moveit_bridge_running = False
        self._gz_launching = False
        self._gz_launch_start = 0.0
        self._bridge_launching = False
        self._bridge_launch_start = 0.0
        self._moveit_launching = False
        self._moveit_launch_start = 0.0
        self._moveit_bridge_launching = False
        self._moveit_bridge_launch_start = 0.0
        self._moveit_bridge_stop_grace_until = 0.0
        self._star_inflight = False
        self._controller_spawn_last_start = 0.0
        self._robot_test_done = True
        self._robot_test_disabled = True
        self._robot_test_substate = "IDLE"
        self._robot_test_last_failure = ""
        # FASE 3: Mutex para evitar goals MoveIt concurrentes.
        self._motion_in_progress = False
        self._motion_in_progress_lock = threading.Lock()
        self._panel_flow_state = "BOOT"
        self._panel_flow_reason = "startup"
        self._tf_ready_timer: Optional[QTimer] = None
        self._tf_ready_last_notice = 0.0
        self._tf_ready_state = False
        self._tf_ever_ok = False
        self._last_tf_ok_monotonic = 0.0
        self._tf_not_ready_logged = False
        self._trace_ready = False
        self._bridge_ready = False
        self._trace_print_period = max(0.5, TRACE_PRINT_PERIOD_SEC)
        self._ee_warn_period = 5.0
        self._trace_debug_logged = False
        self._panel_start_ts = time.time()
        self._perf_start_monotonic = time.monotonic()
        self._metrics_enabled = _get_panel_ui_params().metrics
        self._diagnostic_mode = _get_panel_ui_params().diag_mode
        self._fatal_stops_all = _get_panel_ui_params().fatal_stops_all
        self._perf_marks: Dict[str, float] = {}
        self._debug_logs_enabled = bool(DEBUG_LOGS_TO_STDOUT)
        self._panel_logger = _PanelLogger(self)
        self._camera_ctrl = CameraController(self)
        self._tf_monitor = TFMonitor(self)
        self._state_evaluator = PanelStateEvaluator()
        self._physics = PanelPhysics(self)
        self._joint_limits_ok = False
        self._joint_limits_err = ""
        self._tfm_ckpt_meta: Dict[str, Dict[str, object]] = {}
        self._tfm_ckpt_options = self._discover_tfm_checkpoints()
        self._tfm_ckpt_selected = self._pick_default_tfm_checkpoint(preferred=INFER_CKPT)
        self.tfm_module = (
            TFMGraspModule(logger=self._emit_log, model_path=self._tfm_ckpt_selected or "")
            if TFMGraspModule
            else None
        )
        if not self.tfm_module:
            self._emit_log("[TFM] WARN: modulo TFM no disponible (import fallido).")
        else:
            err = self.tfm_module.last_error()
            if err:
                self._emit_log(f"[TFM] WARN: modulo disponible pero sin modelo ({err}).")
            else:
                self._emit_log(
                    f"[TFM] Modulo listo (modelo_cargado={self.tfm_module.is_model_loaded()})."
                )
        self._load_joint_limits()
        self._env_diag = collect_env_diagnostics(use_sim_time=bool(USE_SIM_TIME))
        if CAMERA_REQUIRED is None:
            self._camera_required = True
        else:
            self._camera_required = bool(CAMERA_REQUIRED)
            if not self._camera_required:
                self._emit_log(
                    "[STARTUP] CAMERA_REQUIRED=false; auto-connect y health-check de cámara deshabilitados."
                )
        # Override: when running offscreen (no real display), camera frames are
        # structurally impossible — disable the requirement regardless of env var.
        if self._camera_required and _get_panel_ui_params().force_offscreen:
            self._camera_required = False
            self._emit_log(
                "[STARTUP] camera_required overridden → False (PANEL_FORCE_OFFSCREEN=1)"
            )
        self._system_state = SystemState.BOOT
        self._system_state_reason = "boot"
        self._fatal_latched = False
        self._system_error_reason = ""
        now = time.monotonic()
        self._system_state_deadline = now + max(0.1, GZ_LAUNCH_TIMEOUT_SEC)
        self._critical_clock_deadline = 0.0
        self._critical_pose_deadline = 0.0
        self._critical_tf_deadline = 0.0
        self._critical_camera_deadline = 0.0
        self._clock_ever_ok = False
        try:
            self._tf_drop_grace_sec = max(
                0.0,
                _get_panel_ui_params().tf_drop_grace_sec,
            )
        except Exception:
            self._tf_drop_grace_sec = 4.0
        self._managed_mode = PANEL_MANAGED
        self._moveit_required = PANEL_MOVEIT_REQUIRED
        if self._managed_mode:
            self._critical_clock_deadline = now + max(0.1, CRITICAL_CLOCK_TIMEOUT_SEC)
            self._critical_pose_deadline = 0.0
            self._critical_tf_deadline = 0.0
            self._critical_camera_deadline = 0.0
        self._tf_invalid = False
        self._moveit_state = MoveItState.OFF
        self._moveit_state_reason = "manual"
        self._moveit_block_reason: Optional[str] = None
        self.signal_status.connect(self._set_status_async)
        self.signal_refresh_controls.connect(self._refresh_controls)
        self.signal_set_led.connect(self._set_led_async)
        self.signal_update_objects.connect(self._update_objects)
        self.signal_start_objects_settle_watch.connect(self._start_objects_settle_watch)
        self.signal_handle_objects_settled.connect(self._handle_objects_settled)
        self.signal_schedule_camera_health_check.connect(self._camera_ctrl.schedule_health_check)
        self.signal_update_camera_topics.connect(self._camera_ctrl.update_topics_async)
        self.signal_connect_camera.connect(self._camera_ctrl.connect)
        self.signal_request_auto_bridge_start.connect(self._request_auto_bridge_start)
        self.signal_bridge_ready.connect(self._on_bridge_ready)
        self.signal_calibration_check.connect(self._on_calibration_check)
        self.signal_trace_ready.connect(self._on_trace_ready)
        self.signal_schedule_home_offset.connect(self._schedule_home_offset_retry)
        self.signal_close_panel.connect(self.close)
        self._emit_log(
            f"[STARTUP] camera_required={self._camera_required} "
            f"PANEL_CAMERA_REQUIRED={_camera_required_label(CAMERA_REQUIRED)} "
            f"debug_logs_enabled={self._debug_logs_enabled}"
        )
        self._emit_log(f"[STARTUP] env: {format_env_diagnostics(self._env_diag)}")
        if self._diagnostic_mode:
            self._emit_log("[STARTUP] PANEL_DIAG_MODE=1 (no stop_all on ERROR_FATAL)")
        if not self._fatal_stops_all:
            self._emit_log("[STARTUP] PANEL_FATAL_STOPS_ALL=0 (ERROR_FATAL no hace stop_all)")
        self._cornell_metrics, self._cornell_metrics_err = _load_cornell_metrics(VISION_DIR)
        if not self._cornell_metrics and self._cornell_metrics_err:
            self._emit_log(f"[TFM] WARN: métricas Cornell no disponibles ({self._cornell_metrics_err})")
        self.signal_tf_ready.connect(self._on_tf_ready_signal)
        self.signal_calib_ready.connect(self._on_calib_ready_signal)
        self.signal_controllers_ready.connect(self._on_controllers_ready_signal)
        self.signal_error.connect(self._on_error_signal)
        self.signal_moveit_state.connect(self._on_moveit_state_signal)
        self.signal_run_ui.connect(self._run_ui_callable)
        self.signal_run_ui_delayed.connect(self._run_ui_delayed)
        self._watchdog_timer = QTimer(self)
        self._watchdog_timer.setInterval(400)
        self._watchdog_timer.timeout.connect(self._check_critical_timeouts)
        self._watchdog_timer.start()
        self._moveit_node: Optional[Node] = None
        self._moveit_pose_pub = None
        self._moveit_pose_pub_cartesian = None
        self._grasp_rect_pub = None
        self._gripper_pub = None
        self._gripper_topic = ""
        self.ActionClient = ActionClient
        self.MoveGroup = MoveGroup
        self._attach_pubs: Dict[str, object] = {}
        self._traj_pub = None
        self._traj_topic = ""
        self._traj_action_client = None
        self._traj_action_name = ""
        self._traj_action_inflight = False
        self._traj_fallback_last_ts = 0.0
        self._traj_publish_inflight = False
        self._moveit_action_client = None
        self._controller_client = None
        self._controller_client_name = ""
        self._objects_settled = False
        self._objects_seen_fall = False
        self._settle_worker_active = False
        self._settle_thread: Optional[QThread] = None
        self._async_threads: List[QThread] = []
        self._objects_release_done = False
        # Activable por entorno para forzar escena canonical lista tras bridge.
        self._auto_release_drop_objects = _env_flag(
            "PANEL_AUTO_RELEASE_DROP_OBJECTS", True
        )
        self._drop_nudge_done = False
        self._drop_hold_last_ts = 0.0
        self._drop_hold_inflight = False
        self._drop_hold_warned = False
        self._drop_hold_log_ts = 0.0
        self._drop_hold_gz_warned = False
        self._drop_hold_gz_available: Optional[bool] = None
        self._drop_hold_enabled = False
        self._drop_hold_gz_checked = False
        self._drop_hold_gz_last_check = 0.0
        self._drop_hold_gz_check_interval = float(GZ_SERVICE_CHECK_SEC)
        self._drop_hold_gz_service_name: Optional[str] = None
        self._drop_anchor_attached = False
        self._release_retry_count = 0
        self._set_pose_service_name: Optional[str] = None
        self._pose_info_save_logged = False
        self._pose_info_ok = False
        self._pose_info_ever_ok = False
        self._pose_info_msg_count = 0
        self._pose_info_last_age = float("inf")
        self._pose_info_last_log = 0.0
        self._pose_info_timer: Optional[QTimer] = None
        self._physics_runtime_check_scheduled = False
        self._pose_info_resub_ts = 0.0
        self._pose_info_diag_logged = False
        self._pose_cli_last_ts = 0.0
        self._pose_cli_warn_ts = 0.0
        self._pose_info_active_cache = False
        self._pose_info_active_ts = 0.0
        self._tf_no_msgs_logged = False
        self._pick_block_reason: Optional[str] = None
        self._external_motion_block_reason: Optional[str] = None
        self._robot_test_block_reason: Optional[str] = None
        self._robot_test_cleanup_pending = False
        self._robot_test_cleanup_retries = 0
        self._robot_test_cleanup_topic = ""
        self._fall_test_last_log = 0.0
        self._detach_feature_checked = False
        self._detach_feature_available = False
        self._detach_feature_logged = False
        self._started_gazebo = False
        self._started_bridge = False
        self._started_moveit = False
        self._started_moveit_bridge = False
        self._started_release_service = False
        self._moveit_bridge_detected_cache = False
        self._moveit_bridge_detected_ts = 0.0
        self._started_world_tf = False
        self._started_rsp = False
        self._started_bag = False
        self._detach_inflight = False
        self._detach_attempted = False
        self._detach_auto_disabled = False
        self._detach_backoff_until = 0.0
        self._trace_transform_warn_last: Dict[str, float] = {}
        self._trace_transform_warn_count: Dict[str, int] = {}
        self._trace_transform_warn_period = 5.0
        self._pick_disable_warn_ts = 0.0
        self._pick_tf_inflight = False
        self._pick_log_last_sig = ""
        self._pick_log_last_ts = 0.0
        self._pick_demo_executed = False  # Flag para UI state (FASE 4)
        self._pick_demo_result_ready = False
        self._pick_demo_result_success = False
        self._pick_demo_result_reason = ""
        self._pick_target_lock_active = False
        self._pick_target_lock_name: str = ""
        self._pick_target_lock_ts: float = 0.0
        self._pick_target_lock_id: str = ""
        self._pick_target_lock_source: str = ""
        self._pick_target_lock_reason: str = ""
        self._fall_test_active = False
        self._settle_log_once_done = False
        self._settle_log_snapshot_next = False
        self._settle_log_snapshot_active = False
        self._bag_running = False
        self._reset_trace_throttle("init")
        self._timers_started = False
        self._gripper_closed = False
        self._manual_inflight = False
        self._manual_pending = False
        self._script_motion_active = False
        self._allow_camera_while_script_motion = _get_panel_ui_params().allow_camera_while_motion
        self._allow_gripper_while_script_motion = _get_panel_ui_params().allow_gripper_while_motion
        self._manual_controls_always_enabled = _get_panel_ui_params().manual_controls_always_enabled
        self._closing = False
        self._shutdown_complete = False
        self._last_tcp_world = None
        self._last_tcp_base = None
        self._last_tcp_world_tf = None
        self._last_tcp_base_z = None
        self._last_tcp_rpy_deg = None
        self._last_tcp_fk_ts: float = 0.0
        self._last_trace_tcp_rpy_deg: Optional[Tuple[float, float, float]] = None
        self._last_trace_object_age_sec: Optional[float] = None
        self._last_tcp_mismatch_warn_ts = 0.0
        self._last_debug_tcp_base: Optional[Tuple[float, float, float]] = None
        self._last_debug_tcp_ts: float = 0.0
        self._last_trace_tcp_base: Optional[Tuple[float, float, float]] = None
        self._last_trace_tcp_ts: float = 0.0
        # Sim-time stamp (nanoseconds) from the TF message header when _last_trace_tcp_base
        # was last updated.  Used to compute true bridge latency (sim_now - tf_stamp).
        self._last_trace_tcp_tf_stamp_ns: int = 0
        self._last_panel_trace_audit_ts: float = 0.0
        self._tf_chain_logged: bool = False
        self._debug_joints_to_stdout = bool(DEBUG_JOINTS_TO_STDOUT)
        self.joint_topic = PANEL_JOINT_STATES_TOPIC
        self._joint_subscribed = False
        self._last_joint_positions: Dict[str, float] = {}
        self._last_joint_time: float = 0.0
        self._last_joint_stamp: float = 0.0
        self._joint_current_topic = ""
        self._joint_active = False
        self._joint_names_warned = False
        self.dof_pos_labels: Dict[str, QLabel] = {}
        self.dof_vel_labels: Dict[str, QLabel] = {}
        self.gripper_labels: Dict[str, QLabel] = {}
        self.gripper_total_lbl: Optional[QLabel] = None
        self.tcp_xyz_lbl: Optional[QLabel] = None
        self.tcp_rpy_lbl: Optional[QLabel] = None
        self.tcp_live_xyz_lbl: Optional[QLabel] = None
        self.tcp_live_rpy_lbl: Optional[QLabel] = None
        self.vel_norm_lbl: Optional[QLabel] = None
        self.vel_max_lbl: Optional[QLabel] = None
        self.eff_max_lbl: Optional[QLabel] = None
        self.joint_sliders = []
        self.joint_value_labels = []
        self._last_slider_values = {}  # Track cambios en sliders para debug
        self._slider_update_blocked_until = 0.0  # Bloquea actualizaciones de sliders por gestos manuales
        self._updating_sliders_from_joint_state = False  # Flag para evitar loops
        self.camera_topic = PANEL_CAMERA_TOPIC
        self._camera_subscribed = False
        self._camera_stream_ok = False
        self._camera_ever_ok = False
        self._camera_topic_hz = 0.0
        self._camera_topic_check_inflight = False
        self._camera_health_retry_scheduled = False
        self._camera_reconnect_scheduled = False
        self._camera_reconnect_attempts = 0
        self._camera_reconnect_last_reason = ""
        self._camera_frame_count = 0
        self._camera_depth_frame_count = 0
        self._camera_subscribe_ts = 0.0
        self._last_camera_depth_frame_ts = 0.0
        self._camera_depth_topic = ""
        self._camera_depth_required_env = _get_panel_ui_params().camera_require_depth
        self._camera_fault_since = 0.0
        self._camera_fault_active = False
        self._camera_fault_reason = ""
        self._camera_fault_age_sec = max(CAMERA_READY_MAX_AGE_SEC + 0.5, _env_float("PANEL_CAMERA_FAULT_AGE_SEC", 4.0))
        self._camera_fault_persist_sec = max(2.0, _env_float("PANEL_CAMERA_FAULT_PERSIST_SEC", 10.0))
        self._camera_warmup_grace_sec = max(CAMERA_INIT_GRACE_SEC, _env_float("PANEL_CAMERA_WARMUP_GRACE_SEC", CAMERA_INIT_GRACE_SEC))
        self._camera_reconnect_base_delay_ms = max(500, int(_env_float("PANEL_CAMERA_RECONNECT_BASE_DELAY_MS", 1500.0)))
        self._camera_reconnect_max_delay_ms = max(
            self._camera_reconnect_base_delay_ms,
            int(_env_float("PANEL_CAMERA_RECONNECT_MAX_DELAY_MS", 6000.0)),
        )
        self._camera_ready_frames = max(1, CAMERA_READY_FRAMES)
        self._required_ee_frame = (
            str(
                _get_panel_ui_params().required_ee_frame
                or "rg2_pinch_center"
            ).strip()
            or "rg2_pinch_center"
        )
        self._auto_calib_from_camera = bool(AUTO_CALIB_FROM_CAMERA)
        self._calibrating = False
        self._calib_points = []  # Lista de (px, py, wx, wy)
        self._calib_grid_until = 0.0
        self._auto_joint2_move_done = False
        self._auto_pick_demo_enabled = _get_panel_ui_params().auto_run_pick_demo
        try:
            self._auto_pick_demo_attempts = max(
                1, _get_panel_ui_params().auto_run_pick_demo_attempts
            )
        except Exception:
            self._auto_pick_demo_attempts = 1
        self._auto_pick_demo_done = 0
        self._auto_pick_demo_last_try_ts = 0.0
        self._selected_object = None  # Objeto seleccionado para pick
        self._reach_overlay_enabled = False
        self._reach_overlay_points: List[Tuple[int, int]] = []
        self._reach_overlay_size: Tuple[int, int] = (0, 0)
        self._pickable_map_cache: Optional[Dict[str, bool]] = None
        self._sdf_model_cache: Dict[str, Dict[str, object]] = {}
        self._table_top_z: Optional[float] = None
        self._last_camera_frame: Optional[Tuple[object, int, int, float]] = None
        self._camera_last_fps: float = 0.0
        self._perf_fps_hist: List[float] = []
        self._perf_fps_avg: float = 0.0
        self._perf_infer_hist: List[float] = []
        self._perf_total_hist: List[float] = []
        self._perf_infer_ms: Optional[float] = None
        self._perf_total_ms: Optional[float] = None
        self._perf_ui_last_ts: float = 0.0
        self._last_grasp_px: Optional[Dict[str, float]] = None
        self._last_grasp_world: Optional[Dict[str, float]] = None
        self._last_grasp_base: Optional[Dict[str, float]] = None
        self._last_cornell: Optional[Dict[str, object]] = None
        self._last_cornell_ref: Optional[Dict[str, float]] = None
        self._last_cornell_reason: str = "Inferir y seleccionar un objeto"
        self._tfm_visual_compare_enabled = False
        self._tfm_overlay_focus_active = False
        self._last_grasp_frame: str = ""
        self._last_grasp_source: str = ""
        self._grasp_rect_topic: str = GRASP_RECT_TOPIC
        self._grasp_rect_subscribed = False
        self._last_grasp_update_ts: float = 0.0
        self._last_grasp_selection_name: str = ""
        self._last_infer_selection_snapshot: Dict[str, object] = {}
        self._last_infer_image_path: str = ""
        self._last_infer_output_path: str = ""
        self._last_infer_frame_ts: float = 0.0
        self._last_infer_overlay_path: str = ""
        self._tfm_preprocessed_cache: Optional[Tuple[float, object]] = None
        self._infer_session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._safety = PanelSafety(self)
        self._state_machine = PanelStateMachine()
        self._watchdog = PanelWatchdog(self)
        self._exp_info: Dict[str, object] = {}
        self._tfm_experiment_applied = False
        self._tfm_infer_inflight = False
        self._tfm_execute_inflight = False
        self._pick_demo_pending_request_id: str = ""
        self._tfm_infer_pending_request_id: str = ""
        self._tfm_execute_pending_request_id: str = ""
        self._tfm_canonical_ctx: Optional[Dict[str, object]] = None
        self._pick_object_grasp_override: Optional[Dict[str, object]] = None
        self._pick_object_worker_started = False
        self._marker_pub = None
        self._controller_check_inflight = False
        self._controllers_ok = False
        self._controllers_reason = "controladores no verificados"
        self._controllers_state = "STARTING"
        self._last_controller_check = 0.0
        self._controllers_last_ok_ts = 0.0
        self._controller_state_map: Dict[str, str] = {}
        self._controller_state_source = ""
        self._controller_state_ts = 0.0
        self._controller_spawn_inflight = False
        self._controller_spawn_done = False
        self._selected_px = None  # Píxel seleccionado (px, py)
        self._selected_world = None  # Solo para overlay/cámara (no negocio)
        self._selected_base = None  # Coordenadas en base_link (x, y, z)
        self._selected_base_frame = "base_link"
        self._last_tf_status: Optional[Dict[str, object]] = None
        self._last_selection_frame: Optional[str] = WORLD_FRAME or "world"
        self._base_frame_effective: Optional[str] = "base_link"
        self._selection_timestamp: float = 0.0
        self._selection_last_user_name: str = ""
        self._selection_last_user_ts: float = 0.0
        self._ee_frame_effective: Optional[str] = None
        self._last_selected_world_pose: Optional[Tuple[float, float, float, str]] = None
        self._last_selected_base_pose: Optional[Tuple[float, float, float, str]] = None
        self._last_ee_warn_ts: float = 0.0
        self._last_ee_diag_ts: float = 0.0
        self._trace_timer: Optional[QTimer] = None
        self.trace_group: Optional[QGroupBox] = None
        self.trace_table: Optional[QTableWidget] = None
        self.chk_trace_freeze: Optional[QCheckBox] = None
        self.science_group: Optional[QGroupBox] = None
        self._spawn_positions_snapshot = get_object_positions()
        self._drop_spawn_positions = dict(DROP_OBJECTS)
        self._pick_demo_spawn_pose = tuple(float(v) for v in PICK_DEMO_SPAWN_POSE)
        self._pick_demo_recover_inflight = False
        self._pick_demo_recover_last_ts = 0.0
        self._pick_demo_recover_gz_cli: str = ""
        self._pick_demo_recover_sdf: str = ""
        self._pick_demo_recover_world_sdf: str = ""
        self._external_state: Optional[str] = None
        self._external_state_reason: str = ""
        self._external_state_last: float = 0.0
        self._emit_log(
            f"[STARTUP] panel_v2 argv0={os.path.abspath(sys.argv[0])} file={os.path.abspath(__file__)}"
        )
        self.lbl_trace_frames: Optional[QLabel] = None
        self.lbl_trace_error_base: Optional[QLabel] = None
        self.lbl_trace_error_world: Optional[QLabel] = None
        self.lbl_trace_tf_translation: Optional[QLabel] = None
        self.lbl_trace_tf_yaw: Optional[QLabel] = None
        self.btn_copy_trace: Optional[QPushButton] = None
        self.lbl_cornell_iou: Optional[QLabel] = None
        self.lbl_cornell_theta: Optional[QLabel] = None
        self.lbl_cornell_success: Optional[QLabel] = None
        self.lbl_cornell_note: Optional[QLabel] = None
        self.lbl_exp_model: Optional[QLabel] = None
        self.lbl_exp_modality: Optional[QLabel] = None
        self.lbl_exp_name: Optional[QLabel] = None
        self.lbl_exp_seed: Optional[QLabel] = None
        self.lbl_exp_epoch: Optional[QLabel] = None
        self.lbl_exp_success: Optional[QLabel] = None
        self.lbl_exp_iou: Optional[QLabel] = None
        self.lbl_exp_weights: Optional[QLabel] = None
        self.lbl_perf_infer: Optional[QLabel] = None
        self.lbl_perf_total: Optional[QLabel] = None
        self.lbl_perf_fps: Optional[QLabel] = None
        self.lbl_grasp_img: Optional[QLabel] = None
        self.lbl_grasp_world: Optional[QLabel] = None
        self.lbl_grasp_frame: Optional[QLabel] = None
        self.btn_save_episode: Optional[QPushButton] = None
        self.lbl_moveit_status: Optional[QLabel] = None
        self.lbl_moveit_bridge_status: Optional[QLabel] = None
        self._trace_cached_text = ""
        self._trace_diag_inflight = False
        self._pose_stream_proc = {}  # Slot para proceso de stream de poses
        self._fatal_shutdown_started = False
        self._pose_debug_timer: Optional[QTimer] = None
        self._auto_bridge_attempts = 0
        self._auto_bridge_timer_scheduled = False
        self._bridge_start_ts = 0.0
        self.runner = CmdRunner()
        self.runner.line.connect(lambda msg: self._log(msg))
        self._emit_log("[STARTUP] Limpieza de procesos fantasma")
        self._cleanup_stray_processes()
        self._emit_log("[STARTUP] Limpieza de cache Python")
        self._clean_cache_dirs()
        self._emit_log("[STARTUP] Creando CalibrationService")
        self.calib_service = CalibrationService(log_fn=self._log)
        # Asegurar que rclpy está inicializado antes de crear el RosWorker y el TfHelper.
        # Sin este bloque el error queda silenciado (debug_logs_enabled=False o bridge off).
        if rclpy is not None:
            try:
                if not rclpy.ok():
                    rclpy.init(args=None)
                self._emit_log("[STARTUP] rclpy.init OK")
            except Exception as exc:
                self._emit_log(f"[STARTUP] WARN rclpy.init: {exc}")
        self._emit_log("[STARTUP] Creando RosWorker")
        # FIX-SIM-TIME: force_realtime=False allows use_sim_time to follow USE_SIM_TIME env.
        # Previously hardcoded True caused panel_superpro to run with use_sim_time=False
        # even in simulation, breaking watchdogs and clock-sensitive subscribers.
        self.ros_worker = RosWorker(force_realtime=False)
        self.ros_worker.image.connect(self._camera_ctrl.on_image)
        self.ros_worker.joint_state.connect(self._on_joint_state)
        self.ros_worker.grasp_rect.connect(self._on_grasp_rect)
        self.ros_worker.log.connect(self._log_ros_message)
        self.ros_worker.system_state.connect(self._on_system_state_update)
        self.ros_worker.camera_connect_request.connect(self._on_remote_camera_connect_request)
        self.ros_worker.camera_disconnect_request.connect(self._on_remote_camera_disconnect_request)
        self.ros_worker.recover_request.connect(self._on_remote_recover_request)
        self.ros_worker.tfm_infer_request.connect(self._on_remote_tfm_infer_request)
        self.ros_worker.tfm_execute_request.connect(self._on_remote_tfm_execute_request)
        self.ros_worker.pick_demo_request.connect(self._on_remote_pick_demo_request)
        self.ros_worker.pick_object_request.connect(self._on_remote_pick_object_request)
        self.ros_worker.object_select_request.connect(self._on_remote_object_select_request)
        self._ros_worker_started = False
        self._ensure_ros_worker_started()
        self._calibration_ready = False
        self._last_calib_block_log = 0.0
        self._emit_log("[STARTUP] Inicializando publisher MoveIt")
        self._init_moveit_publisher()
    
        # Conectar señal de retry para movimiento manual (thread-safe)
        self.retry_send_joints.connect(self._send_joints_retry)
    
        self._emit_log("[STARTUP] Construyendo UI")
        self._build_ui()
        self._load_experiment_info()
        self._refresh_science_ui()
        for warning in validate_env(self._effective_mode(), self._env_diag):
            self._emit_log(f"[STARTUP][WARN] {warning}")
        self._emit_log("[STARTUP] UI lista")
    
        # Timer para actualizar objetos
        self.objects_timer = QTimer(self)
        self.objects_timer.timeout.connect(self._update_objects)
        self.objects_timer.start(1000)  # Actualizar cada segundo
    
        # Timer para auto-conectar cámara (activado tras lanzar el bridge)
        # (no se programa inmediatamente para evitar logs antes de que el usuario arranque Gazebo/bridge)
        # Suscribir joint_states con un temporizador de reintento corto
        self.joint_timer = QTimer(self)
        self.joint_timer.timeout.connect(self._auto_subscribe_joints)
        self.joint_timer.start(800)

        # Timer de hold para objetos DROP (frozen hasta Soltar/Obj).
        self._drop_hold_timer = QTimer(self)
        self._drop_hold_timer.timeout.connect(self._drop_hold_tick)
        self._drop_hold_timer.start(250)
    
        # Inicializar estado UI (arranque manual por botones del panel)
        # (self._update_ui_state() se llama al final de _build_ui)
        self._emit_log("[STARTUP] UI state inicial aplicado")
    
        # Forzar estado inicial en OFF para LEDs y permitir arranque manual
        self._emit_log("[STARTUP] LEDs forzados a OFF")

        # Chequeo de estado asíncrono tras 1s
        QTimer.singleShot(1000, self._refresh_status_async)
        if not self._managed_mode:
            self._request_auto_bridge_start()
    
        # Test inicial de logging
        self._emit_log("[PANEL-V2] Panel iniciado - logging activo")
        self._emit_log(f"[PANEL-V2] Debug logs enabled: {self._debug_logs_enabled}")
        QTimer.singleShot(1200, self._ensure_grasp_rect_subscription)

    def get_logger(self):
        return self._panel_logger

    # F14-step2: _init_moveit_publisher / _ensure_moveit_node /
    # _publish_current_grasp_rect heredados de PanelV2PublisherMixin.

    # F14-step3: _expected_world_frame, _business_base_frame,
    # _base_frame_candidates, ensure_base_pose, _ensure_base_coords,
    # get_tcp_base, get_tcp_pose_base, transform_pose_to_base,
    # log_pose_base, log_pose, get_pose_in_base — heredados de
    # PanelV2BasePoseMixin.

    # F14-step2: _moveit_publish_context, _request_auto_bridge_start,
    # _auto_bridge_tick, _get_*_publisher heredados de PanelV2PublisherMixin.

    # F14-step4: _normalize_attach_name, _find_attach_candidate,
    # _attempt_attach, _schedule_attach_attempt, _command_gripper,
    # _command_gripper_preopen — heredados de PanelV2GripperAttachMixin.

    # F14-step5: _traj_action_target, _resolve_traj_action_name,
    # _get_action_client, _wait_action_server, _format_action_error,
    # _joint_motion_since, _wait_for_joint_target, _wait_for_tcp_base_z,
    # _wait_for_tcp_base_target — heredados de PanelV2MotionMixin.

    # F14-step6: _send_joint_trajectory_action, _schedule_traj_action_fallback,
    # _clamp_joint_positions, _log_traj_action_fallback,
    # _start_objects_settle_watch, _invalidate_settle, _run_fall_test_async,
    # _objects_settle_worker, _handle_objects_settled, _log_calib_blocked,
    # _log_settle_snapshot, _request_settle_snapshot, wait_for_objects_to_settle
    # — heredados de PanelV2TrajSettleMixin.

    # F14-step2: _publish_joint_trajectory, _publish_moveit_pose heredados
    # de PanelV2PublisherMixin.

    # F14-step7: _build_ui, _debounced_btn_action, showEvent,
    # _set_status*, _set_led_async, _on_*_signal*, _on_trace_ready,
    # _on_calibration_check, _set_system_state, _effective_system_state,
    # _trigger_fatal, _resolve_system_state, _build_state_snapshot,
    # _evaluate_system_state, _update_system_state,
    # _check_critical_timeouts, _resolve_critical_fault,
    # _state_ready_*, _manual_control_ready, _calibration_topic_allowed,
    # _overhead_camera_active — heredados de PanelV2SystemStateMixin.



    def _moveit_not_ready_reason(self, *args, **kwargs):
        return _ph._moveit_not_ready_reason(self, *args, **kwargs)

    def _set_moveit_wait_status(self, *args, **kwargs):
        return _ph._set_moveit_wait_status(self, *args, **kwargs)

    def _controllers_not_ready_reason(self, *args, **kwargs):
        return _ph._controllers_not_ready_reason(self, *args, **kwargs)

    def _ros_node_not_ready_reason(self, *args, **kwargs):
        return _ph._ros_node_not_ready_reason(*args, **kwargs)

    def _controller_manager_not_ready_reason(self, *args, **kwargs):
        return _ph._controller_manager_not_ready_reason(*args, **kwargs)

    def _list_controllers_not_ready_reason(self, *args, **kwargs):
        return _ph._list_controllers_not_ready_reason(*args, **kwargs)

    def _camera_not_ready_reason(self, *args, **kwargs):
        return _ph._camera_not_ready_reason(self, *args, **kwargs)

    def _tfm_experiment_ready_status(self, *args, **kwargs):
        return _ph._tfm_experiment_ready_status(self, *args, **kwargs)

    def _tfm_infer_ready_status(self, *args, **kwargs):
        return _ph._tfm_infer_ready_status(self, *args, **kwargs)

    def _tfm_infer_waitable_reason(self, *args, **kwargs):
        return _ph._tfm_infer_waitable_reason(*args, **kwargs)

    def _current_grasp_status(self, *args, **kwargs):
        return _ph._current_grasp_status(self, *args, **kwargs)

    def _restore_execute_selection_context(self, *args, **kwargs):
        return _ph._restore_execute_selection_context(self, *args, **kwargs)

    def _calibration_action_status(self, *args, **kwargs):
        return _ph._calibration_action_status(self, *args, **kwargs)

    def _pose_info_not_ready_reason(self, *args, **kwargs):
        return _ph._pose_info_not_ready_reason(*args, **kwargs)

    def _tf_not_ready_reason(self, *args, **kwargs):
        return _ph._tf_not_ready_reason(self, *args, **kwargs)

    def _moveit_control_status(self, *args, **kwargs):
        return _ph._moveit_control_status(self, *args, **kwargs)

    def _manual_control_status(self, *args, **kwargs):
        return _ph._manual_control_status(self, *args, **kwargs)

    def _external_publishers_for_topic(self, *args, **kwargs):
        return _ph._external_publishers_for_topic(self, *args, **kwargs)

    def _bridge_publishers_only(self, *args, **kwargs):
        return _ph._bridge_publishers_only(self, *args, **kwargs)

    def _set_robot_test_blocked(self, *args, **kwargs):
        return _ph._set_robot_test_blocked(self, *args, **kwargs)

    def _await_external_publishers_clear(self, *args, **kwargs):
        return _ph._await_external_publishers_clear(self, *args, **kwargs)

    def _schedule_robot_test_cleanup_check(self, *args, **kwargs):
        return _ph._schedule_robot_test_cleanup_check(self, *args, **kwargs)

    def _update_camera_topics_async(self, *args, **kwargs):
        return _ph._update_camera_topics_async(self, *args, **kwargs)

    def _emit_log(self, *args, **kwargs):
        return _ph._emit_log(self, *args, **kwargs)

    def _metric_mark(self, *args, **kwargs):
        return _ph._metric_mark(self, *args, **kwargs)

    def _audit_root(self, *args, **kwargs):
        return _ph._audit_root(self, *args, **kwargs)

    def _audit_append(self, *args, **kwargs):
        return _ph._audit_append(self, *args, **kwargs)

    def _audit_write_json(self, *args, **kwargs):
        return _ph._audit_write_json(self, *args, **kwargs)

    def _sha256_file(self, *args, **kwargs):
        return _ph._sha256_file(self, *args, **kwargs)

    def _should_emit_log(self, *args, **kwargs):
        return _ph._should_emit_log(self, *args, **kwargs)

    def _set_motion_lock(self, *args, **kwargs):
        return _ph._set_motion_lock(self, *args, **kwargs)

    def _set_btn_state(self, *args, **kwargs):
        return _ph._set_btn_state(self, *args, **kwargs)

    def _set_launching_style(self, *args, **kwargs):
        return _ph._set_launching_style(self, *args, **kwargs)

    def _clear_launching_if_timeout(self, *args, **kwargs):
        return _ph._clear_launching_if_timeout(self, *args, **kwargs)

    def _controller_drop_grace_active(self, *args, **kwargs):
        return _ph._controller_drop_grace_active(self, *args, **kwargs)

    def _require_ready_basic(self, *args, **kwargs):
        return _ph._require_ready_basic(self, *args, **kwargs)

    def _basic_ready_status(self, *args, **kwargs):
        return _ph._basic_ready_status(self, *args, **kwargs)

    def _pick_demo_remote_ready_status(self, *args, **kwargs):
        return _ph._pick_demo_remote_ready_status(self, *args, **kwargs)

    def _auto_release_drop_objects_when_ready(self, *args, **kwargs):
        return _ph._auto_release_drop_objects_when_ready(self, *args, **kwargs)

    def _require_ready_vision(self, *args, **kwargs):
        return _ph._require_ready_vision(self, *args, **kwargs)

    def _require_manual_ready(self, *args, **kwargs):
        return _ph._require_manual_ready(self, *args, **kwargs)

    def _log(self, *args, **kwargs):
        return _ph._log(self, *args, **kwargs)

    def _emit_log_throttled(self, *args, **kwargs):
        return _ph._emit_log_throttled(self, *args, **kwargs)

    def _block_if_managed(self, *args, **kwargs):
        return _ph._block_if_managed(self, *args, **kwargs)

    # F14-step9: 97 wrappers logs/async/external_state/status/pose_info/
    # gazebo/controllers/camera/bridge_presets/start/stop heredados de
    # PanelV2RuntimeDiagnosticsMixin. Ver panel_v2_runtime_diagnostics_mixin.py.

    def _log_trace(self, *args, **kwargs):
        return _sm._log_trace(self, *args, **kwargs)

    def _apply_debug_button_style(self, *args, **kwargs):
        return _sm._apply_debug_button_style(self, *args, **kwargs)

    def _print_pose_snapshot(self, *args, **kwargs):
        return _sm._print_pose_snapshot(self, *args, **kwargs)

    def _drop_detach_supported(self, *args, **kwargs):
        return _sm._drop_detach_supported(self, *args, **kwargs)

    def _release_objects(self, *args, **kwargs):
        return _sm._release_objects(self)

    def _schedule_release_retry(self, *args, **kwargs):
        return _sm._schedule_release_retry(self, *args, **kwargs)

    def _attach_drop_objects(self, *args, **kwargs):
        return _sm._attach_drop_objects(self, *args, **kwargs)

    def _maybe_nudge_drop_objects(self, *args, **kwargs):
        return _sm._maybe_nudge_drop_objects(self, *args, **kwargs)

    def _resolve_set_pose_service(self, *args, **kwargs):
        return _sm._resolve_set_pose_service(self, *args, **kwargs)

    def _resolve_gz_cli(self, *args, **kwargs):
        return _sm._resolve_gz_cli(self, *args, **kwargs)

    def _resolve_world_sdf_path(self, *args, **kwargs):
        return _sm._resolve_world_sdf_path(self, *args, **kwargs)

    def _load_pick_demo_recover_sdf(self, *args, **kwargs):
        return _sm._load_pick_demo_recover_sdf(self, *args, **kwargs)

    def _run_gz_service_cli(self, *args, **kwargs):
        return _sm._run_gz_service_cli(self, *args, **kwargs)

    def _recover_pick_demo_to_table_gz(self, *args, **kwargs):
        return _sm._recover_pick_demo_to_table_gz(self, *args, **kwargs)

    def _maybe_hold_drop_objects(self, *args, **kwargs):
        return _sm._maybe_hold_drop_objects(self, *args, **kwargs)

    def _maybe_recover_pick_demo(self, *args, **kwargs):
        return _sm._maybe_recover_pick_demo(self, *args, **kwargs)

    def _recover_pick_demo_to_table(self, *args, **kwargs):
        return _sm._recover_pick_demo_to_table(self, *args, **kwargs)

    def _hold_drop_objects(self, *args, **kwargs):
        return _sm._hold_drop_objects(self, *args, **kwargs)

    def _hold_drop_objects_gz(self, *args, **kwargs):
        return _sm._hold_drop_objects_gz(self, *args, **kwargs)

    def _drop_hold_tick(self, *args, **kwargs):
        return _sm._drop_hold_tick(self, *args, **kwargs)

    def _nudge_drop_objects(self, *args, **kwargs):
        return _sm._nudge_drop_objects(self, *args, **kwargs)

    def _start_release_service(self, *args, **kwargs):
        return _sm._start_release_service(self, *args, **kwargs)

    def _start_world_tf_publisher(self, *args, **kwargs):
        return _sm._start_world_tf_publisher(self, *args, **kwargs)

    def _stop_world_tf_publisher(self, *args, **kwargs):
        return _sm._stop_world_tf_publisher(self, *args, **kwargs)

    def _stop_gazebo(self, *args, **kwargs):
        return _sm._stop_gazebo(self)

    def _start_robot_state_publisher(self, *args, **kwargs):
        return _sm._start_robot_state_publisher(self, *args, **kwargs)

    def _start_bridge(self, *args, **kwargs):
        return _sm._start_bridge(self, *args, **kwargs)

    def _spawn_controllers_async(self, *args, **kwargs):
        return _sm._spawn_controllers_async(self, *args, **kwargs)

    def _stop_bridge(self, *args, **kwargs):
        return _sm._stop_bridge(self, *args, **kwargs)

    def _start_moveit(self, *args, **kwargs):
        return _sm._start_moveit(self, *args, **kwargs)

    def _stop_moveit(self, *args, **kwargs):
        return _sm._stop_moveit(self, *args, **kwargs)

    def _wait_for_moveit_ready(self, *args, **kwargs):
        return _sm._wait_for_moveit_ready(self, *args, **kwargs)

    def _start_moveit_bridge(self, *args, **kwargs):
        return _sm._start_moveit_bridge(self, *args, **kwargs)

    def _stop_moveit_bridge(self, *args, **kwargs):
        return _sm._stop_moveit_bridge(self, *args, **kwargs)

    def _clear_moveit_bridge_launching(self, *args, **kwargs):
        return _sm._clear_moveit_bridge_launching(self, *args, **kwargs)

    def _kill_proc(self, *args, **kwargs):
        return _sm._kill_proc(self, *args, **kwargs)

    def _proc_alive(self, *args, **kwargs):
        return _sm._proc_alive(self, *args, **kwargs)

    def _save_home_from_sliders(self, *args, **kwargs):
        return _sm._save_home_from_sliders(self, *args, **kwargs)

    def _rosbag_running(self, *args, **kwargs):
        return _sm._rosbag_running(self, *args, **kwargs)

    def _start_bag(self, *args, **kwargs):
        return _sm._start_bag(self, *args, **kwargs)

    def _stop_bag(self, *args, **kwargs):
        return _sm._stop_bag(self, *args, **kwargs)

    def _refresh_status_sync(self, *args, **kwargs):
        return _sm._refresh_status_sync(self, *args, **kwargs)

    def _refresh_status_async(self, *args, **kwargs):
        return _sm._refresh_status_async(self, *args, **kwargs)

    def _apply_status(self, *args, **kwargs):
        return _sm._apply_status(self, *args, **kwargs)

    def _moveit_topics_ready(self, *args, **kwargs):
        return _sm._moveit_topics_ready(self, *args, **kwargs)

    def _moveit_status_ready(self, *args, **kwargs):
        return _sm._moveit_status_ready(self, *args, **kwargs)

    def _moveit_action_ready(self, *args, **kwargs):
        return _sm._moveit_action_ready(self, *args, **kwargs)

    def _list_topic_names(self, *args, **kwargs):
        return _sm._list_topic_names(self, *args, **kwargs)

    def _list_action_names(self, *args, **kwargs):
        return _sm._list_action_names(self, *args, **kwargs)

    def _topic_has_any_publishers(self, *args, **kwargs):
        return _sm._topic_has_any_publishers(self, *args, **kwargs)

    def _world_frame_last_first(self, *args, **kwargs):
        return _sm._world_frame_last_first(self, *args, **kwargs)

    def _world_frame_config_first(self, *args, **kwargs):
        return _sm._world_frame_config_first(self, *args, **kwargs)

    def _follow_joint_traj_ready(self, *args, **kwargs):
        return _sm._follow_joint_traj_ready(self, *args, **kwargs)

    def _moveit_bridge_detected(self, *args, **kwargs):
        return _sm._moveit_bridge_detected(self, *args, **kwargs)

    def _move_group_startup_ready(self, *args, **kwargs):
        return _sm._move_group_startup_ready(self, *args, **kwargs)

    def _move_group_ready(self, *args, **kwargs):
        return _sm._move_group_ready(self, *args, **kwargs)

    def _moveit_ready(self, *args, **kwargs):
        return _sm._moveit_ready(self, *args, **kwargs)

    def _update_moveit_status_label(self, *args, **kwargs):
        return _sm._update_moveit_status_label(self, *args, **kwargs)

    def _update_system_stats(self, *args, **kwargs):
        return _sm._update_system_stats(self, *args, **kwargs)

    def _set_stat_label(self, *args, **kwargs):
        return _sm._set_stat_label(self, *args, **kwargs)

    def _known_process_pids(self, *args, **kwargs):
        return _sm._known_process_pids(self, *args, **kwargs)

    def _list_stale_processes(self, *args, **kwargs):
        return _sm._list_stale_processes(self, *args, **kwargs)

    def _detect_stale_processes(self, *args, **kwargs):
        return _sm._detect_stale_processes(self, *args, **kwargs)

    def _refresh_controls(self, *args, **kwargs):
        return _sm._refresh_controls(self, *args, **kwargs)

    def _maybe_auto_run_pick_demo(self, *args, **kwargs):
        return _sm._maybe_auto_run_pick_demo(self, *args, **kwargs)

    def _wait_for_state_change(self, *args, **kwargs):
        return _sm._wait_for_state_change(self, *args, **kwargs)

    def _schedule_controller_check(self, *args, **kwargs):
        return _sm._schedule_controller_check(self, *args, **kwargs)


    def _update_ui_state(self, *args, **kwargs):
        return _mc._update_ui_state(self, *args, **kwargs)

    def _effective_mode(self, *args, **kwargs):
        return _mc._effective_mode(self, *args, **kwargs)

    def _apply_home_joint2_offset(self, *args, **kwargs):
        return _mc._apply_home_joint2_offset(self, *args, **kwargs)

    def _schedule_home_offset_retry(self, *args, **kwargs):
        return _mc._schedule_home_offset_retry(self, *args, **kwargs)

    def _get_home_joint_pose(self, *args, **kwargs):
        return _mc._get_home_joint_pose(self, *args, **kwargs)

    def _get_gripper_force(self, *args, **kwargs):
        return _mc._get_gripper_force(self, *args, **kwargs)

    def _wait_for_joint_convergence(self, *args, **kwargs):
        return _mc._wait_for_joint_convergence(self, *args, **kwargs)

    def _run_baseline_motion(self, *args, **kwargs):
        return _mc._run_baseline_motion(self, *args, **kwargs)

    def _go_home(self, *args, **kwargs):
        return _mc._go_home(self)

    def _set_test_failed(self, *args, **kwargs):
        return _mc._set_test_failed(self, *args, **kwargs)

    def _set_robot_test_done(self, *args, **kwargs):
        return _mc._set_robot_test_done(self, *args, **kwargs)

    def _set_panel_flow_state(self, *args, **kwargs):
        return _mc._set_panel_flow_state(self, *args, **kwargs)

    def _update_panel_flow_state(self, *args, **kwargs):
        return _mc._update_panel_flow_state(self, *args, **kwargs)

    def _compute_reach_overlay_points(self, *args, **kwargs):
        return _mc._compute_reach_overlay_points(self, *args, **kwargs)

    # F14-step8: _step_joint heredado de PanelV2StepDebugMixin.

    def _maybe_send_auto(self, *args, **kwargs):
        return _mc._maybe_send_auto(self, *args, **kwargs)

    def _send_joints(self, *args, **kwargs):
        return _mc._send_joints(self)

    def _send_joints_retry(self, *args, **kwargs):
        return _mc._send_joints_retry(self, *args, **kwargs)


    def _tfm_infer_grasp(self, *args, **kwargs):
        return tfm_infer_grasp(self)

    def _tfm_canonical_use_pick_object(self, *args, **kwargs):
        return tfm_canonical_use_pick_object(self, *args, **kwargs)

    def _complete_pending_tfm_infer_request(self, *args, **kwargs):
        return complete_pending_tfm_infer_request(self, *args, **kwargs)

    def _complete_pending_tfm_execute_request(self, *args, **kwargs):
        return complete_pending_tfm_execute_request(self, *args, **kwargs)

    def _complete_pending_pick_demo_request(self, *args, **kwargs):
        return complete_pending_pick_demo_request(self, *args, **kwargs)

    def _build_tfm_pick_object_override(self, *args, **kwargs):
        return build_tfm_pick_object_override(self, *args, **kwargs)

    def _tfm_canonical_state_reset(self, *args, **kwargs):
        return tfm_canonical_state_reset(self, *args, **kwargs)

    def _tfm_canonical_phase_update(self, *args, **kwargs):
        return tfm_canonical_phase_update(self, *args, **kwargs)

    def _tfm_canonical_finish(self, *args, **kwargs):
        return tfm_canonical_finish(self, *args, **kwargs)

    def _restore_infer_selection_snapshot(self, *args, **kwargs):
        return restore_infer_selection_snapshot(self, *args, **kwargs)

    def _latest_camera_frame_snapshot(self, *args, **kwargs):
        return latest_camera_frame_snapshot(self, *args, **kwargs)

    def _ensure_selected_object_in_store(self, *args, **kwargs):
        return ensure_selected_object_in_store(self, *args, **kwargs)

    def _handle_infer_result(self, *args, **kwargs):
        return handle_infer_result(self, *args, **kwargs)

    def _sync_tfm_module_grasp_state(self, *args, **kwargs):
        return sync_tfm_module_grasp_state(self, *args, **kwargs)

    def _tfm_visualize_grasp(self, *args, **kwargs):
        return tfm_visualize_grasp(self)

    def _wait_tfm_moveit_result(self, *args, **kwargs):
        return wait_tfm_moveit_result(self, *args, **kwargs)

    def _execute_tfm_world_grasp(self, *args, **kwargs):
        return execute_tfm_world_grasp(self, *args, **kwargs)

    def _on_tfm_grasp_object_clicked(self, *args, **kwargs):
        return on_tfm_grasp_object_clicked(self)

    def _tfm_publish_grasp(self, *args, **kwargs):
        return tfm_publish_grasp(self, *args, **kwargs)

    def _on_remote_camera_connect_request(self, source: str) -> None:
        _rc._on_remote_camera_connect_request(self, source)

    def _on_remote_camera_disconnect_request(self, source: str) -> None:
        _rc._on_remote_camera_disconnect_request(self, source)

    def _on_remote_recover_request(self, source: str) -> None:
        _rc._on_remote_recover_request(self, source)

    def _on_remote_tfm_infer_request(self, source: str) -> None:
        _rc._on_remote_tfm_infer_request(self, source)

    def _on_remote_tfm_execute_request(self, source: str) -> None:
        _rc._on_remote_tfm_execute_request(self, source)

    def _on_remote_pick_demo_request(self, source: str) -> None:
        _rc._on_remote_pick_demo_request(self, source)

    def _on_remote_pick_object_request(self, source: str) -> None:
        _rc._on_remote_pick_object_request(self, source)

    def _on_remote_object_select_request(self, name: str, source: str) -> None:
        _rc._on_remote_object_select_request(self, name, source)

    def _publish_calib_grid_marker(self, *args, **kwargs):
        return _ca._publish_calib_grid_marker(self, *args, **kwargs)

    def _start_calibration(self, *args, **kwargs):
        return _ca._start_calibration(self)

    def _set_calibrate_button_text(self, *args, **kwargs):
        return _ca._set_calibrate_button_text(self, *args, **kwargs)

    def _capture_calibration_frame(self, *args, **kwargs):
        return _ca._capture_calibration_frame(self, *args, **kwargs)

    def _auto_calibrate_from_camera(self, *args, **kwargs):
        return _ca._auto_calibrate_from_camera(self, *args, **kwargs)

    def _pick_confirm_dialog(self, *args, **kwargs):
        return _ca._pick_confirm_dialog(*args, **kwargs)

    def _run_pick_demo(self, *args, **kwargs):
        return _ca._run_pick_demo(self)

    def _run_pick_object(self, *args, **kwargs):
        return _ca._run_pick_object(self)

    def _get_object_world_position(self, *args, **kwargs):
        return _ca._get_object_world_position(self, *args, **kwargs)

    def _on_slider_change(self, *args, **kwargs):
        return _ca._on_slider_change(self, *args, **kwargs)

    def _slider_to_deg(self, *args, **kwargs):
        return _ca._slider_to_deg(self, *args, **kwargs)

    def _current_joint_positions_rad(self, *args, **kwargs):
        return _ca._current_joint_positions_rad(self, *args, **kwargs)

    def _resolve_table_top_z(self, *args, **kwargs):
        return _om._resolve_table_top_z(self, *args, **kwargs)

    def _resolve_safety_surface_z(self, *args, **kwargs):
        return _om._resolve_safety_surface_z(self, *args, **kwargs)

    def _check_robot_above_table(self, *args, **kwargs):
        return _om._check_robot_above_table(self, *args, **kwargs)

    def _load_sdf_geometry_cache(self, *args, **kwargs):
        return _om._load_sdf_geometry_cache(self, *args, **kwargs)

    def _load_joint_limits(self, *args, **kwargs):
        return _om._load_joint_limits(self, *args, **kwargs)

    def _post_calibration_pipeline(self, *args, **kwargs):
        return _om._post_calibration_pipeline(self, *args, **kwargs)

    def _build_object_report(self, *args, **kwargs):
        return _om._build_object_report(self, *args, **kwargs)

    def _is_pickable(self, *args, **kwargs):
        return _om._is_pickable(self, *args, **kwargs)

    def _log_object_report(self, *args, **kwargs):
        return _om._log_object_report(self, *args, **kwargs)

    def _go_table(self, *args, **kwargs):
        return _om._go_table(self)

    def _go_basket(self, *args, **kwargs):
        return _om._go_basket(self)

    def _toggle_gripper_button(self, *args, **kwargs):
        return _om._toggle_gripper_button(self, *args, **kwargs)

    def _on_camera_click(self, *args, **kwargs):
        return _om._on_camera_click(self, *args, **kwargs)

    def _load_table_calibration(self, *args, **kwargs):
        return _om._load_table_calibration(self, *args, **kwargs)

    def _refresh_objects_from_gz_async(self, *args, **kwargs):
        return _om._refresh_objects_from_gz_async(self, *args, **kwargs)

    def _resolve_pose_object_name(self, *args, **kwargs):
        return _om._resolve_pose_object_name(self, *args, **kwargs)

    def _extract_pose_updates(self, *args, **kwargs):
        return _om._extract_pose_updates(self, *args, **kwargs)

    def _refresh_objects_from_gz(self, *args, **kwargs):
        return _om._refresh_objects_from_gz(self, *args, **kwargs)

    def _settle_targets(self, *args, **kwargs):
        return _om._settle_targets(self, *args, **kwargs)

    def _read_world_pose_info(self, *args, **kwargs):
        return _om._read_world_pose_info(self, *args, **kwargs)



    def _compute_homography(self, *args, **kwargs):
        return _do._compute_homography(*args, **kwargs)

    def _draw_calib_overlay(self, *args, **kwargs):
        return _do._draw_calib_overlay(self, *args, **kwargs)

    def _draw_selection_overlay(self, *args, **kwargs):
        return _do._draw_selection_overlay(self, *args, **kwargs)

    def _draw_grasp_overlay(self, *args, **kwargs):
        return _do._draw_grasp_overlay(self, *args, **kwargs)

    def _tfm_overlay_focus_enabled(self, *args, **kwargs):
        return _do._tfm_overlay_focus_enabled(self, *args, **kwargs)

    def _should_draw_reach_overlay(self, *args, **kwargs):
        return _do._should_draw_reach_overlay(self, *args, **kwargs)

    def _should_draw_selection_overlay(self, *args, **kwargs):
        return _do._should_draw_selection_overlay(self, *args, **kwargs)

    def _should_draw_test_corner_overlay(self, *args, **kwargs):
        return _do._should_draw_test_corner_overlay(self, *args, **kwargs)

    def _should_draw_tcp_pose_overlay(self, *args, **kwargs):
        return _do._should_draw_tcp_pose_overlay(self, *args, **kwargs)

    def _base_to_world_coords(self, *args, **kwargs):
        return _do._base_to_world_coords(self, *args, **kwargs)

    def _compute_test_corner_base_points(self, *args, **kwargs):
        return _do._compute_test_corner_base_points(self, *args, **kwargs)

    def _compute_test_corner_world_points(self, *args, **kwargs):
        return _do._compute_test_corner_world_points(self, *args, **kwargs)

    def _draw_test_corner_overlay(self, *args, **kwargs):
        return _do._draw_test_corner_overlay(self, *args, **kwargs)

    def _draw_tcp_pose_overlay(self, *args, **kwargs):
        return _do._draw_tcp_pose_overlay(self, *args, **kwargs)

    def _save_overhead_frame_with_overlays(self, *args, **kwargs):
        return _do._save_overhead_frame_with_overlays(self, *args, **kwargs)

    def _save_grasp_overlay(self, *args, **kwargs):
        return _do._save_grasp_overlay(self, *args, **kwargs)

    def _refresh_grasp_overlay_now(self, *args, **kwargs):
        return _do._refresh_grasp_overlay_now(self, *args, **kwargs)

    def _draw_reach_overlay(self, *args, **kwargs):
        return _do._draw_reach_overlay(self, *args, **kwargs)


    def _handle_calibration_click(self, *args, **kwargs):
        return _cs._handle_calibration_click(self, *args, **kwargs)

    def _finish_calibration(self, *args, **kwargs):
        return _cs._finish_calibration(self, *args, **kwargs)

    def _handle_object_selection_click(self, *args, **kwargs):
        return _cs._handle_object_selection_click(self, *args, **kwargs)

    def _select_object(self, *args, **kwargs):
        return _cs._select_object(self, *args, **kwargs)

    def _log_selection_tf(self, *args, **kwargs):
        return _cs._log_selection_tf(self, *args, **kwargs)

    def _start_tf_diagnose_async(self, *args, **kwargs):
        return _cs._start_tf_diagnose_async(self, *args, **kwargs)

    def _start_pick_tf_resolve(self, *args, **kwargs):
        return _cs._start_pick_tf_resolve(self, *args, **kwargs)

    def _selection_candidate(self, *args, **kwargs):
        return _cs._selection_candidate(self, *args, **kwargs)

    def _selection_to_base(self, *args, **kwargs):
        return _cs._selection_to_base(self, *args, **kwargs)

    def _on_object_clicked(self, *args, **kwargs):
        return _cs._on_object_clicked(self, *args, **kwargs)

    def _update_objects(self, *args, **kwargs):
        return _cs._update_objects(self, *args, **kwargs)

    def _push_history(self, *args, **kwargs):
        return _cs._push_history(self, *args, **kwargs)

    def _mean_history(self, *args, **kwargs):
        return _cs._mean_history(self, *args, **kwargs)

    def _update_fps_stats(self, *args, **kwargs):
        return _cs._update_fps_stats(self, *args, **kwargs)

    def _tfm_repro_profile_env(self, *args, **kwargs):
        return _ts._tfm_repro_profile_env(self, *args, **kwargs)

    def _tfm_repro_profile(self, *args, **kwargs):
        return _ts._tfm_repro_profile(self, *args, **kwargs)

    def _tfm_raw_output_env_enabled(self, *args, **kwargs):
        return _ts._tfm_raw_output_env_enabled(self, *args, **kwargs)

    def _tfm_postprocess_enabled(self, *args, **kwargs):
        return _ts._tfm_postprocess_enabled(self, *args, **kwargs)

    def _tfm_postprocess_policy_label(self, *args, **kwargs):
        return _ts._tfm_postprocess_policy_label(self, *args, **kwargs)

    def _on_tfm_repro_mode_changed(self, *args, **kwargs):
        return _ts._on_tfm_repro_mode_changed(self, *args, **kwargs)

    def _on_tfm_postprocess_mode_changed(self, *args, **kwargs):
        return _ts._on_tfm_postprocess_mode_changed(self, *args, **kwargs)

    def _on_tfm_checkpoint_selection_changed(self, *args, **kwargs):
        return _ts._on_tfm_checkpoint_selection_changed(self, *args, **kwargs)

    def _tfm_apply_memoria_case(self, *args, **kwargs):
        return _ts._tfm_apply_memoria_case(self)

    def _tfm_repro_checkpoint(self, *args, **kwargs):
        return _ts._tfm_repro_checkpoint(self, *args, **kwargs)

    def _tfm_repro_checkpoint_meta(self, *args, **kwargs):
        return _ts._tfm_repro_checkpoint_meta(self, *args, **kwargs)

    def _tfm_is_aux_experiment(self, *args, **kwargs):
        return _ts._tfm_is_aux_experiment(self, *args, **kwargs)

    def _tfm_select_seed_from_summary(self, *args, **kwargs):
        return _ts._tfm_select_seed_from_summary(self, *args, **kwargs)

    def _discover_tfm_checkpoints(self, *args, **kwargs):
        return _ts._discover_tfm_checkpoints(self, *args, **kwargs)

    def _pick_default_tfm_checkpoint(self, *args, **kwargs):
        return _ts._pick_default_tfm_checkpoint(self, *args, **kwargs)

    def _refresh_tfm_checkpoint_options(self, *args, **kwargs):
        return _ts._refresh_tfm_checkpoint_options(self, *args, **kwargs)

    def _format_ckpt_label(self, *args, **kwargs):
        return _ts._format_ckpt_label(self, *args, **kwargs)

    def _tfm_get_ckpt_path(self, *args, **kwargs):
        return _ts._tfm_get_ckpt_path(self, *args, **kwargs)

    def _tfm_apply_experiment(self, *args, **kwargs):
        return _ts._tfm_apply_experiment(self)

    def _tfm_reset_grasp(self, *args, **kwargs):
        return _ts._tfm_reset_grasp(self)

    def _load_experiment_info(self, *args, **kwargs):
        return _ts._load_experiment_info(self, *args, **kwargs)

    def _format_value(self, *args, **kwargs):
        return _ts._format_value(self, *args, **kwargs)

    def _refresh_science_ui(self, *args, **kwargs):
        return _ts._refresh_science_ui(self, *args, **kwargs)

    def _world_to_pixel(self, *args, **kwargs):
        return _ts._world_to_pixel(self, *args, **kwargs)

    def _world_to_pixel_diag(self, *args, **kwargs):
        return _ts._world_to_pixel_diag(self, *args, **kwargs)

    def _build_reference_grasp(self, *args, **kwargs):
        return _ts._build_reference_grasp(self, *args, **kwargs)

    def _grasp_projection_z_target(self, *args, **kwargs):
        return _ts._grasp_projection_z_target(self, *args, **kwargs)

    def _compute_world_grasp(self, *args, **kwargs):
        return _ts._compute_world_grasp(self, *args, **kwargs)

    def _world_grasp_to_base(self, *args, **kwargs):
        return _ts._world_grasp_to_base(self, *args, **kwargs)

    def _update_cornell_metrics(self, *args, **kwargs):
        return _ts._update_cornell_metrics(self, *args, **kwargs)

    def _refresh_cornell_metrics(self, *args, **kwargs):
        return _ts._refresh_cornell_metrics(self, *args, **kwargs)

    def _save_episode(self, *args, **kwargs):
        return _ts._save_episode(self, *args, **kwargs)

    def _build_science_group(self) -> QGroupBox:
        return build_science_group(self)

    def _build_trace_group(self) -> QGroupBox:
        return build_trace_group(self)

    def _start_trace_timer(self):
        return _tc._start_trace_timer(self)

    def _resolve_trace_frames(self, world_frame: str) -> Tuple[str, Optional[str]]:
        return _tc._resolve_trace_frames(self, world_frame)

    def _refresh_trace_data(self):
        return _tc._refresh_trace_data(self)

    def _log_trace_transform_warning(self, message: str) -> None:
        _tc._log_trace_transform_warning(self, message)

    def _maybe_log_tf_not_ready(self):
        return _tc._maybe_log_tf_not_ready(self)

    def _maybe_log_trace(self, now: float):
        _tc._maybe_log_trace(self, now)

    def _reset_trace_throttle(self, reason: str):
        _tc._reset_trace_throttle(self, reason)

    def _run_trace_diag_once(self):
        return _tc._run_trace_diag_once(self)

    def _tf_sanity_check(self) -> Tuple[bool, str]:
        return _tc._tf_sanity_check(self)

    def _run_self_check_once(self) -> None:
        _tc._run_self_check_once(self)

    def _self_check_worker(self) -> None:
        _tc._self_check_worker(self)

    def _trace_diag_worker(self) -> None:
        _tc._trace_diag_worker(self)

    def _try_mark_tf_ready(self):
        return _tc._try_mark_tf_ready(self)

    def _start_tf_ready_timer(self):
        return _tc._start_tf_ready_timer(self)

    def _wait_for_tf_ready(self, world_frame: str, helper) -> Optional[str]:
        return _tc._wait_for_tf_ready(self, world_frame, helper)

    def _tf_world_base_valid(self, helper, base_frame: str, world_frame: str) -> bool:
        return _tc._tf_world_base_valid(self, helper, base_frame, world_frame)

    def _stop_tf_ready_timer(self):
        return _tc._stop_tf_ready_timer(self)

    def _log_tf_chain_once(self, world_frame: str, base_frame: str, ee_frame) -> None:
        _tc._log_tf_chain_once(self, world_frame, base_frame, ee_frame)

    def _check_tcp_source_mismatch(self, now_mono: float) -> None:
        _tc._check_tcp_source_mismatch(self, now_mono)

    def _build_trace_text(self, world_frame, base_frame, ee_frame, object_world,
                          object_base, tcp_world, tcp_base, base_error, world_error,
                          tf_transform, *, object_source, tcp_source):
        return _tc._build_trace_text(self, world_frame, base_frame, ee_frame,
                                     object_world, object_base, tcp_world, tcp_base,
                                     base_error, world_error, tf_transform,
                                     object_source=object_source, tcp_source=tcp_source)

    def _set_trace_row(self, row, world_data, base_data, world_frame, base_frame):
        _tc._set_trace_row(self, row, world_data, base_data, world_frame, base_frame)

    def _value_from_pose(self, data, key: str):
        return _tc._value_from_pose(self, data, key)

    def _set_trace_item(self, row: int, col: int, text: str):
        _tc._set_trace_item(self, row, col, text)

    def _format_dual_value(self, first, second) -> str:
        return _tc._format_dual_value(self, first, second)

    def _pose_dict(self, position, orientation, frame: str):
        return _tc._pose_dict(self, position, orientation, frame)

    def _compute_error(self, source, target):
        return _tc._compute_error(self, source, target)

    def _format_error_text(self, error) -> str:
        return _tc._format_error_text(self, error)

    def _format_error_tuple(self, error) -> str:
        return _tc._format_error_tuple(self, error)

    def _format_pose_summary(self, label: str, data) -> str:
        return _tc._format_pose_summary(self, label, data)

    def _copy_trace_text(self):
        return _tc._copy_trace_text(self)

    def closeEvent(self, event):
        _sd.closeEvent(self, event)


    def _force_cleanup_leftovers(self, *args, **kwargs):
        return _gs._force_cleanup_leftovers(self, *args, **kwargs)

# F14: extraídos a panel_v2_helpers; re-exports legacy preservados.
from .panel_v2_helpers import (  # noqa: E402,F401
    normalize_joint_name as _normalize_joint_name,
    rot_to_rpy as _rot_to_rpy,
)


def main():
    app = QApplication(sys.argv)
    panel = ControlPanelV2()
    panel.show()
    def _handle_signal(_signum, _frame):
        try:
            QTimer.singleShot(0, panel.close)
        except Exception as exc:
            _log_exception("handle signal", exc)
    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, _handle_signal)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
