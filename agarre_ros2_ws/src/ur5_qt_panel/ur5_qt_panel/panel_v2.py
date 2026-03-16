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
import csv
import re
import math
import shlex
import signal
import shutil
import subprocess
import sys
import threading
import time
import traceback
import json
import uuid
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Set
try:
    import psutil  # type: ignore
except Exception:
    psutil = None
import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QObject, QThread
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractScrollArea,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSlider,
    QSizePolicy,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QHeaderView,
    QTableWidget,
    QTableWidgetItem,
)
try:
    import yaml  # type: ignore
except Exception:
    yaml = None

from .panel_config import (
    BAGS_DIR,
    BASE_FRAME,
    BRIDGE_BASE_YAML,
    DEFAULT_JOINT_MOVE_SEC,
    DEFAULT_WORLD_CANDIDATES,
    DROP_OBJECTS,
    DROP_OBJECT_NAMES,
    EGL_VENDOR,
    GZ_PARTITION_FILE,
    GZ_WORLD,
    JOINT_SLIDER_DEG_MAX,
    JOINT_SLIDER_DEG_MIN,
    JOINT_SLIDER_SCALE,
    LOG_DIR,
    MODELS_DIR,
    SCRIPTS_DIR,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    TABLE_SIZE_X,
    TABLE_SIZE_Y,
    SELECTION_SNAP_DIST,
    SELECTION_TIMEOUT_SEC,
    AUTO_START_BRIDGE,
    AUTO_START_BRIDGE_DELAY_MS,
    AUTO_START_BRIDGE_MAX_RETRIES,
    CALIBRATION_CAMERA_TOPIC,
    UR5_HOME_DEFAULT,
    UR5_JOINT_NAMES,
    GRIPPER_JOINT_NAMES,
    GRIPPER_ATTACH_PREFIX,
    DROP_ANCHOR_PREFIX,
    UR5_BASE_X,
    UR5_BASE_Y,
    UR5_BASE_Z,
    UR5_REACH_RADIUS,
    SAVE_POSE_INFO_POSITIONS,
    UR5_CONTROLLERS_YAML,
    UR5_JOINT_LIMITS_YAML,
    UR5_MODEL_NAME,
    BASKET_DROP,
    WORLD_FRAME,
    WORLDS_DIR,
    WS_DIR,
    ROS_AVAILABLE,
    NUDGE_DROP_OBJECTS,
    NUDGE_DROP_DZ,
    NUDGE_DROP_Z_MIN,
    TRAJ_ACTION_FALLBACK,
    TRAJ_ACTION_FALLBACK_DELAY_SEC,
    TRAJ_ACTION_FALLBACK_EPS_RAD,
    TRAJ_ACTION_FALLBACK_TIMEOUT_SEC,
    CONTROLLER_READY_TIMEOUT_SEC,
    CONTROLLER_READY_CACHE_SEC,
    MOVEIT_READY_TIMEOUT_SEC,
    GZ_LAUNCH_TIMEOUT_SEC,
    BRIDGE_LAUNCH_TIMEOUT_SEC,
    MOVEIT_LAUNCH_TIMEOUT_SEC,
    MOVEIT_BRIDGE_LAUNCH_TIMEOUT_SEC,
    CONTROLLER_DROP_GRACE_SEC,
    TRACE_PRINT_PERIOD_SEC,
    DEBUG_POSES_PERIOD_SEC,
    PICK_LOG_MIN_INTERVAL_SEC,
    GRIPPER_CMD_TOPIC,
    GRIPPER_OPEN_RAD,
    GRIPPER_CLOSED_RAD,
    GRIPPER_JOINT2_SIGN,
    PANEL_MANAGED,
    PANEL_MOVEIT_REQUIRED,
    ALLOW_UNSETTLED_ON_TIMEOUT,
    CAMERA_READY_FRAMES,
    CAMERA_INIT_GRACE_SEC,
    CAMERA_READY_MAX_AGE_SEC,
    CAMERA_REQUIRED,
    CAMERA_DISPLAY_INTERVAL_MS,
    CAMERA_FAST_SCALE,
    CAMERA_PREPROCESS_TFM,
    STALE_PROCESS_GRACE_SEC,
    TF_INIT_GRACE_SEC,
    CONTROLLER_START_GRACE_SEC,
    POSE_INFO_MAX_AGE_SEC,
    POSE_INFO_POLL_SEC,
    POSE_INFO_LOG_PERIOD,
    POSE_INFO_ALLOW_STALE,
    POSE_CLI_ENABLED,
    CRITICAL_CLOCK_TIMEOUT_SEC,
    CRITICAL_POSE_TIMEOUT_SEC,
    POSE_CLI_MIN_INTERVAL_SEC,
    OVERLAY_CALIB,
    OVERLAY_SELECTION,
    OVERLAY_REACH,
    OVERLAY_ANTIALIAS,
    CAMERA_SKIP_TFM_INPUT,
    CAMERA_UI_SKIP_HIDDEN,
    CAMERA_INFO_INTERVAL_SEC,
    CAMERA_TRACK_FPS,
    STATUS_TOPIC_CACHE_SEC,
    GZ_SERVICE_CHECK_SEC,
    CRITICAL_CLOCK_TIMEOUT_SEC,
    CRITICAL_POSE_TIMEOUT_SEC,
    ATTACH_DIST_M,
    ATTACH_REL_EPS,
    ATTACH_HAND_MOVE_EPS,
    ATTACH_WINDOW_SEC,
    ATTACH_SNAP_EPS,
    PICK_TF_RETRY_SEC,
    PICK_TF_TIMEOUT_SEC,
    FALL_TEST_DELAY_SEC,
    PICK_DEMO_PRE_GRASP_Z_OFFSET,
    PICK_DEMO_GRASP_Z_OFFSET,
    PICK_DEMO_TRANSPORT_Z_OFFSET,
    PICK_DEMO_DROP_Z_OFFSET,
    GRIPPER_TCP_Z_OFFSET,
    AUTO_CALIB_FROM_CAMERA,
    REACH_OVERLAY_Z,
    REACH_OVERLAY_POINTS,
    CALIB_GRID_STEP,
    PICKABLE_PRE_GRASP_Z,
    PICKABLE_MIN_CLEARANCE,
    DEBUG_LOGS_TO_STDOUT,
    PANEL_GZ_GUI,
    DEBUG_JOINTS_TO_STDOUT,
    PANEL_JOINT_STATES_TOPIC,
    PANEL_CAMERA_TOPIC,
    USE_SIM_TIME,
    PANEL_SKIP_CLEANUP,
    PANEL_KILL_STALE,
    VISION_DIR,
    VISION_EXP_DIR,
    INFER_CKPT,
    INFER_ROI_SIZE,
    INFER_RETRY_ERR_PX,
)
from .panel_utils import (
    bash_preamble,
    build_gz_env,
    CmdRunner,
    GZ_LOG_FILTERS,
    RosWorker,
    detect_base_frame,
    ensure_dir,
    bulk_update_object_positions,
    get_object_positions,
    get_object_position,
    gz_sim_status,
    base_to_world,
    world_to_base,
    load_home_pose,
    load_object_positions,
    save_object_positions,
    log_to_file,
    parse_ros_topics,
    read_world_name,
    nearest_table_object,
    object_out_of_reach,
    resolve_gz_partition,
    get_object_pose_gz,
    rotate_log,
    run_cmd,
    set_led,
    table_xy_to_pixel,
    table_xy_to_pixel_float,
    pixel_to_table_xy,
    _parse_pose_json,
    transform_point_to_frame,
    get_pose,
    discover_base_and_ee_frames,
    get_tf_helper,
    _can_transform_between,
    _preferred_base_frame,
    _list_tf_topics,
    shutdown_tf_helper,
    _log_tf_yaml_head_once,
    world_xyz_to_pixel,
    world_xyz_to_pixel_float,
    with_line_buffer,
    build_log_filter_cmd,
    yaw_from_quaternion,
    tf_world_base_valid,
    resolve_controller_manager,
    base_frame_candidates,
    rclpy,
    ROBOT_FRAME_KEYWORDS,
    gripper_controller_defined,
)
from .panel_objects import (
    ObjectLogicalState,
    force_release_all_objects,
    get_object_state,
    is_on_table,
    mark_object_released,
    recalc_object_states,
    set_test_read_only,
    update_object_state,
)
from .panel_startup import StartSequence
from .panel_state import (
    EXTERNAL_STATE_MAP,
    MoveItState,
    PanelStateEvaluator,
    PanelStateSnapshot,
    SystemState,
)
from .panel_physics import PanelPhysics, OBJECT_SETTLE_TIMEOUT_SEC
from .panel_shutdown import StopSequence, terminate_process
from .panel_tf_monitor import TFMonitor
from . import panel_launchers
from . import panel_controllers
from .panel_ui_state import apply_ui_state
from .panel_robot_presets import (
    _make_pose_data,
    _build_pose_stamped,
    POSE_HOME_DATA,
    POSE_TABLE_DATA,
    POSE_BASKET_DATA,
    JOINT_TABLE_POSE_RAD,
    JOINT_BASKET_POSE_RAD,
    JOINT_HOME_POSE_RAD,
    JOINT_PICK_IMAGE_POSE_RAD,
    PRE_GRASP_POSE_DATA,
    GRASP_POSE_DATA,
    TRANSPORT_POSE_DATA,
    DROP_POSE_DATA,
    PICK_DEMO_OBJECT_NAME,
)
from .panel_motion_helpers import build_joint_trajectory
from .panel_camera_helpers import is_camera_topic
from .panel_moveit_publishers import init_moveit_publishers
from .panel_moveit_ready import moveit_topics_ready, moveit_status_ready, moveit_action_ready
from .panel_moveit_wait import wait_for_moveit_ready
from .panel_calibration import start_calibration
from .panel_readiness import (
    camera_ready_status,
    pose_info_ready_status,
    tf_ready_status,
    tf_not_ready_reason,
    camera_not_ready_reason,
    pose_info_not_ready_reason,
    controllers_not_ready_reason,
    ros_node_not_ready_reason,
    controller_manager_not_ready_reason,
    list_controllers_not_ready_reason,
    moveit_not_ready_reason,
    set_moveit_wait_status,
    moveit_control_status,
    manual_control_status,
    pick_ui_status,
)
from .panel_tfm import build_tfm_preprocessed_input, tfm_infer
from .panel_moveit_flow import publish_moveit_pose
from .panel_ros_publishers import (
    get_attach_publisher,
    get_gripper_publisher,
    get_traj_publisher,
)
from .panel_tf_diagnose import run_tf_diagnose
from .tf_pose_utils import (
    get_transform as tf_get_transform,
    get_tcp_in_base as tf_get_tcp_in_base,
    transform_pose as tf_transform_pose,
    world_pose_to_base as tf_world_pose_to_base,
)
from .panel_safety import PanelSafety
from .panel_state_machine import PanelStateMachine
from .panel_watchdog import PanelWatchdog
from . import panel_launch_control
from .panel_fatal import abort_local_stack
from .panel_external_state import external_state_active, resolve_external_state, apply_external_system_state
from .panel_pick_demo import run_pick_demo
from .panel_pick_object import run_pick_object
from .panel_env import (
    collect_env_diagnostics,
    format_env_diagnostics,
    validate_env,
    effective_mode,
    get_gz_transport_ip,
)
from .logging_utils import timestamped_line

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
try:
    from tfm_grasping import TFMGraspModule
except Exception:
    TFMGraspModule = None
try:
    from rclpy.parameter import Parameter
except Exception:
    Parameter = None
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray, Empty
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
GRASP_RECT_TOPIC = os.environ.get("PANEL_GRASP_RECT_TOPIC", "/grasp_rect").strip() or "/grasp_rect"
TEST_CORNER_OVERLAY = os.environ.get("PANEL_TEST_CORNER_OVERLAY", "0").strip().lower() not in ("0", "false", "off", "no")
TCP_POSE_OVERLAY = os.environ.get("PANEL_TCP_POSE_OVERLAY", "0").strip().lower() not in ("0", "false", "off", "no")

_DEBUG_EXCEPTIONS = os.environ.get("PANEL_DEBUG_EXCEPTIONS", "").strip() in ("1", "true", "True")


def _log_exception(context: str, exc: Exception) -> None:
    if not _DEBUG_EXCEPTIONS:
        return
    print(timestamped_line(f"[PANEL_V2][WARN] {context}: {exc}"), file=sys.stderr, flush=True)


def _camera_required_label(value: Optional[bool]) -> str:
    if value is None:
        return "unset"
    return "true" if value else "false"


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except Exception:
        return default


SETTLE_MANUAL = {"pick_demo"}
CONTROLLER_CHECK_INTERVAL_SEC = 3.0
CONTROLLER_LIST_RETRY_WINDOW_SEC = max(
    0.0, _env_float("PANEL_CTRL_LIST_RETRY_WINDOW_SEC", 5.0)
)
CONTROLLER_LAST_OK_GRACE_SEC = max(
    0.0, _env_float("PANEL_CTRL_LAST_OK_GRACE_SEC", 5.0)
)
CONTROLLER_LIST_RETRY_STEP_SEC = 0.25


def _proto_time_to_seconds(value: Dict[str, object]) -> float:
    if not isinstance(value, dict):
        return 0.0
    sec = float(value.get("sec", 0.0)) if value.get("sec") is not None else 0.0
    nsec = float(value.get("nsec", 0.0)) if value.get("nsec") is not None else 0.0
    return sec + nsec * 1e-9


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


from .cameras_tab import ObjectListPanel
from .calibration_service import CalibrationService, CalibrationMode
from .ur5_kinematics import fk_ur5


class CameraView(QLabel):
    """Large camera view that auto-scales to the widget size."""
    
    clicked = pyqtSignal(int, int)  # Emite (x, y) en píxeles de la imagen original

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._qimg = None
        self._img_width = 0
        self._img_height = 0
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(480, 360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setText(title)
        self.setStyleSheet("background:#0b0f14; color:#94a3b8; border:1px solid #1f2937;")

    def set_frame(self, qimg, width=0, height=0):
        self._qimg = qimg
        self._img_width = width
        self._img_height = height
        self._update_pixmap()

    def mousePressEvent(self, event):
        """Convertir click en widget a coordenadas de imagen."""
        if event.button() == Qt.LeftButton and self._qimg is not None:
            # Obtener coordenadas del click en el widget
            widget_x = event.x()
            widget_y = event.y()
            
            # Convertir a coordenadas de imagen
            if self._img_width > 0 and self._img_height > 0:
                pixmap = self.pixmap()
                if pixmap:
                    # Calcular offset del pixmap centrado
                    px_w = pixmap.width()
                    px_h = pixmap.height()
                    offset_x = (self.width() - px_w) // 2
                    offset_y = (self.height() - px_h) // 2
                    
                    # Coordenadas relativas al pixmap
                    rel_x = widget_x - offset_x
                    rel_y = widget_y - offset_y
                    
                    # Escalar a imagen original
                    if 0 <= rel_x < px_w and 0 <= rel_y < px_h:
                        img_x = int(rel_x * self._img_width / px_w)
                        img_y = int(rel_y * self._img_height / px_h)
                        img_x = max(0, min(self._img_width - 1, img_x))
                        img_y = max(0, min(self._img_height - 1, img_y))
                        self.clicked.emit(img_x, img_y)
        super().mousePressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()

    def _update_pixmap(self):
        if self._qimg is None:
            return
        pix = QPixmap.fromImage(self._qimg)
        transform = Qt.FastTransformation if CAMERA_FAST_SCALE else Qt.SmoothTransformation
        pix = pix.scaled(self.width(), self.height(), Qt.KeepAspectRatio, transform)
        self.setPixmap(pix)


class _PanelLogger:
    """Logger façade so ControlPanelV2 can call get_logger().info(...)."""

    def __init__(self, panel: "ControlPanelV2"):
        self._panel = panel

    def info(self, msg: str) -> None:
        self._panel._log(msg)


class _FnThread(QThread):
    error = pyqtSignal(str)

    def __init__(self, fn, name: str = ""):
        super().__init__()
        self._fn = fn
        self._name = name

    def run(self) -> None:
        try:
            self._fn()
        except Exception as exc:
            prefix = f"{self._name}: " if self._name else ""
            self.error.emit(f"{prefix}{exc}")

class CameraController:
    """Camera IO and health checks separated from the main panel logic."""

    def __init__(self, panel: "ControlPanelV2") -> None:
        self._panel = panel

    def schedule_reconnect(self, reason: str, delay_ms: Optional[int] = None) -> None:
        p = self._panel
        if not p._camera_required or not p._bridge_running:
            return
        if p._camera_stream_ok or p._camera_reconnect_scheduled:
            return
        if delay_ms is None:
            exponent = min(max(0, int(p._camera_reconnect_attempts)), 3)
            delay_ms = min(
                p._camera_reconnect_max_delay_ms,
                p._camera_reconnect_base_delay_ms * (2 ** exponent),
            )
        delay_ms = max(250, int(delay_ms))
        reason = (reason or "camera_reconnect").strip() or "camera_reconnect"
        p._camera_reconnect_scheduled = True
        p._camera_reconnect_last_reason = reason
        p._emit_log_throttled(
            "camera_reconnect_schedule",
            (
                f"[CAMERA][RECOVER] programada reconexion en {delay_ms}ms "
                f"attempt={p._camera_reconnect_attempts + 1} reason={reason}"
            ),
            min_interval=1.0,
        )

        def _run() -> None:
            p._camera_reconnect_scheduled = False
            if p._camera_stream_ok or not p._camera_required or not p._bridge_running:
                return
            p._camera_reconnect_attempts += 1
            p._emit_log(
                f"[CAMERA][RECOVER] reconectando attempt={p._camera_reconnect_attempts} "
                f"reason={p._camera_reconnect_last_reason or reason}"
            )
            self.auto_connect()

        QTimer.singleShot(delay_ms, _run)

    def health_check(self) -> None:
        p = self._panel
        if not p._camera_required:
            return
        now = time.time()
        camera_ready, camera_fault, camera_source_down, age, in_grace = p._camera_runtime_flags(now)
        depth_required, depth_topic = p._camera_depth_expectation()
        depth_age = now - p._last_camera_depth_frame_ts if p._last_camera_depth_frame_ts > 0.0 else float("inf")
        if camera_source_down:
            p._camera_stream_ok = False
            p._camera_fault_since = 0.0
            p._camera_fault_active = False
            p._camera_fault_reason = ""
            if (now - getattr(p, "_last_camera_diag_log", 0.0)) > 2.5:
                p._emit_log(f"[CAMERA][DIAG] source_down gazebo={p._gazebo_state()} bridge={str(p._bridge_running).lower()}")
                p._last_camera_diag_log = now
            p.camera_info.setText("Cámara: source down")
            p.camera_info.setStyleSheet("color: #f59e0b; font-weight: bold;")
            return
        if p._camera_frame_count <= 0 and in_grace:
            p._camera_stream_ok = False
            warmup_txt = "Cámara: warmup"
            if depth_required:
                warmup_txt += " (RGB-D)"
            p.camera_info.setText(warmup_txt)
            p.camera_info.setStyleSheet("color: #f59e0b; font-weight: bold;")
            return
        if p._camera_subscribed and p._camera_frame_count <= 0:
            p._camera_stream_ok = False
            p.camera_info.setText("Sin imágenes")
            p.camera_info.setStyleSheet("color: #f43f5e; font-weight: bold;")
            self.schedule_reconnect("camera_no_frames", delay_ms=1500)
            return
        if depth_required and p._camera_depth_frame_count <= 0 and in_grace:
            p._camera_stream_ok = False
            p.camera_info.setText("Cámara: warmup depth")
            p.camera_info.setStyleSheet("color: #f59e0b; font-weight: bold;")
            return
        if depth_required and p._camera_subscribed and p._camera_depth_frame_count <= 0:
            p._camera_stream_ok = False
            p.camera_info.setText("Sin depth")
            p.camera_info.setStyleSheet("color: #f43f5e; font-weight: bold;")
            self.schedule_reconnect("camera_depth_no_frames", delay_ms=1500)
            return
        if p._camera_subscribed and (camera_fault or (p._camera_frame_count > 0 and age > p._camera_fault_age_sec)):
            if (now - getattr(p, "_last_camera_diag_log", 0.0)) > 2.5:
                p._emit_log(
                    f"[CAMERA][DIAG] No llegan imágenes desde hace {age:.1f}s en {p.camera_topic or 'N/A'}"
                )
                if depth_required:
                    p._emit_log(
                        f"[CAMERA][DIAG] depth_required=true depth_topic={depth_topic or 'N/A'} "
                        f"depth_age={'inf' if math.isinf(depth_age) else f'{depth_age:.1f}s'}"
                    )
                p._last_camera_diag_log = now
            p.camera_info.setText(f"Sin imágenes ({age:.1f}s)")
            p.camera_info.setStyleSheet("color: #f43f5e; font-weight: bold;")
            self.schedule_reconnect("camera_fault", delay_ms=1500)
        elif p._camera_subscribed and camera_ready:
            p.camera_info.setStyleSheet("")

    def update_topics_async(self, topics: object) -> None:
        self.update_topics(list(topics) if topics else [])

    def refresh_topics(self) -> None:
        p = self._panel
        p._log_button("Refresh topics")
        p._set_status("Detectando tópicos de imagen…")

        def worker():
            if not p._ros_worker_started:
                p._ensure_ros_worker_started()
            if not p.ros_worker.node_ready():
                p.signal_status.emit("Nodo ROS no listo para tópicos", True)
                return
            topics = p.ros_worker.topic_names_and_types()
            candidates = [name for name, _types in topics if is_camera_topic(name)]
            rgb_candidates = [name for name in candidates if not name.endswith("/depth_image")]
            depth_candidates = [name for name in candidates if name.endswith("/depth_image")]
            rgb_candidates.sort(key=lambda t: (0 if t == "/camera_overhead/image" else 1, t))
            depth_candidates.sort(key=lambda t: (0 if t == "/camera_overhead/depth_image" else 1, t))
            candidates = rgb_candidates + depth_candidates
            if candidates:
                if p._debug_logs_enabled:
                    p._log(f"[CAMERA] Candidate topics: {', '.join(candidates)}")
                p.signal_update_camera_topics.emit(candidates)
                p.signal_status.emit(f"Detectados {len(candidates)} tópicos de cámara", False)
            else:
                if p._camera_stream_ok or p._camera_frame_count > 0 or p._camera_subscribed:
                    if p._debug_logs_enabled:
                        p._log("[CAMERA] Tópicos aún no visibles en discovery (reintentando)")
                    p.signal_status.emit("Discovery cámara pendiente (reintentando)", False)
                    p.signal_schedule_camera_health_check.emit(2000)
                else:
                    if p._debug_logs_enabled:
                        p._log("[CAMERA] No se encontraron tópicos compatibles")
                    p.signal_status.emit("No se detectaron tópicos de cámara", False)

        p._run_async(worker)

    def schedule_health_check(self, delay_ms: int = 1800) -> None:
        p = self._panel
        if p._camera_health_retry_scheduled or not p._bridge_running:
            return
        p._camera_health_retry_scheduled = True

        def _run():
            p._camera_health_retry_scheduled = False
            self.check_topic_health()

        QTimer.singleShot(delay_ms, _run)

    def check_topic_health(self) -> None:
        p = self._panel
        if not p._camera_required:
            return
        now = time.time()
        _camera_ready, _camera_fault, camera_source_down, _age, _in_grace = p._camera_runtime_flags(now)
        if camera_source_down:
            p._camera_stream_ok = False
            p._camera_fault_since = 0.0
            p._camera_fault_active = False
            p._camera_fault_reason = ""
            return
        if p._camera_topic_check_inflight or not p._bridge_running:
            return
        topic = p.camera_topic_combo.currentText().strip() or p.camera_topic
        if not topic:
            return
        p._camera_topic_check_inflight = True

        def worker():
            try:
                now = time.time()
                _camera_ready, camera_fault, _camera_source_down, _age, in_grace = p._camera_runtime_flags(now)
                last_age = now - p._last_camera_frame_ts if p._last_camera_frame_ts else float("inf")
                depth_required, depth_topic = p._camera_depth_expectation()
                depth_last_age = (
                    now - p._last_camera_depth_frame_ts
                    if p._last_camera_depth_frame_ts
                    else float("inf")
                )
                depth_ready = (not depth_required) or (
                    p._camera_depth_frame_count >= 1 and depth_last_age < CAMERA_READY_MAX_AGE_SEC
                )
                ready = p._camera_frame_count >= 1 and last_age < CAMERA_READY_MAX_AGE_SEC and depth_ready
                p._camera_stream_ok = bool(ready)
                elapsed = max(1e-3, now - max(0.0, p._camera_subscribe_ts))
                p._camera_topic_hz = float(p._camera_frame_count) / elapsed if p._camera_frame_count > 0 else 0.0
                depth_hz = (
                    float(p._camera_depth_frame_count) / elapsed
                    if p._camera_depth_frame_count > 0
                    else 0.0
                )
                rgb_pub_count = p.ros_worker.topic_publisher_count(topic) if p.ros_worker else 0
                depth_pub_count = (
                    p.ros_worker.topic_publisher_count(depth_topic)
                    if depth_required and depth_topic and p.ros_worker
                    else 0
                )
                p._emit_log_throttled(
                    "camera_health_stats",
                    (
                        f"[CAMERA][HEALTH] topic={topic} pubs={rgb_pub_count} "
                        f"frames={p._camera_frame_count} hz={p._camera_topic_hz:.2f} "
                        f"last_age={'inf' if math.isinf(last_age) else f'{last_age:.2f}s'} "
                        f"depth_required={str(depth_required).lower()} "
                        f"depth_topic={depth_topic or 'n/a'} depth_pubs={depth_pub_count} "
                        f"depth_frames={p._camera_depth_frame_count} depth_hz={depth_hz:.2f} "
                        f"depth_last_age={'inf' if math.isinf(depth_last_age) else f'{depth_last_age:.2f}s'} "
                        f"ready={str(p._camera_stream_ok).lower()}"
                    ),
                    min_interval=1.5,
                )
                if p._debug_logs_enabled:
                    p._emit_log(
                        f"[BRIDGE] camera_topic={topic} frames={p._camera_frame_count} "
                        f"ready={p._camera_stream_ok} last_age={last_age:.2f}s"
                    )
                    if p._camera_stream_ok:
                        p._emit_log(f"[CAMERA] ready=True age={last_age:.2f}s")
                if not p._camera_stream_ok:
                    in_init_grace = in_grace
                    if in_init_grace or (p._camera_frame_count == 0 and last_age < 2.0):
                        p.signal_status.emit("Cámara esperando frames…", False)
                    else:
                        if depth_required and not depth_ready:
                            p.signal_status.emit(
                                f"Cámara no lista; depth pendiente ({depth_topic or 'n/a'})",
                                False,
                            )
                        if camera_fault and p._camera_fault_since <= 0.0:
                            p._camera_fault_since = now
                        if camera_fault and p._camera_fault_since > 0.0:
                            elapsed = now - p._camera_fault_since
                            if elapsed >= p._camera_fault_persist_sec:
                                p._camera_fault_active = True
                                p._camera_fault_reason = (
                                    f"camera_fault persistente age={last_age:.1f}s "
                                    f"persist={elapsed:.1f}s topic={topic}"
                                )
                                p.signal_status.emit("Cámara degradada (diagnóstico)", True)
                                p._emit_log(f"[CAMERA][DIAG] {p._camera_fault_reason}")
                            else:
                                p.signal_status.emit("Cámara no lista; esperando", False)
                        else:
                            p._camera_fault_since = 0.0
                            p.signal_status.emit("Cámara no lista; esperando", False)
                    p.signal_schedule_camera_health_check.emit(2000)
                elif p._objects_settled and not p._calibration_ready:
                    p._camera_fault_since = 0.0
                    p._camera_fault_active = False
                    p._camera_fault_reason = ""
                    p.signal_calibration_check.emit()
            finally:
                p._camera_topic_check_inflight = False
                p.signal_refresh_controls.emit()

        p._run_async(worker)

    def update_topics(self, topics):
        p = self._panel
        current = p.camera_topic_combo.currentText()
        p.camera_topic_combo.clear()
        for topic in topics:
            p.camera_topic_combo.addItem(topic)
        idx = p.camera_topic_combo.findText(current)
        if idx >= 0:
            p.camera_topic_combo.setCurrentIndex(idx)

    def _subscription_stale(self) -> bool:
        p = self._panel
        if not p._camera_subscribed:
            return False
        if not p.camera_topic:
            return True
        if not p._camera_stream_ok or p._camera_frame_count <= 0:
            return True
        if not p._last_camera_frame_ts:
            return True
        age = time.time() - p._last_camera_frame_ts
        return age >= max(0.2, float(CAMERA_READY_MAX_AGE_SEC))

    def connect(self) -> None:
        p = self._panel
        p._log_button("Conectar cámara")
        if bool(getattr(p, "_script_motion_active", False)):
            p._emit_log("[CAMERA] connect bloqueado: robot en movimiento")
            p._set_status("Cámara bloqueada: robot en movimiento", error=False)
            return
        if not p._bridge_running:
            p._log_warning("Cámara en espera: bridge no activo")
            p._set_status("Cámara en espera: bridge no activo", error=False)
            return
        topic = p.camera_topic_combo.currentText().strip()
        if p._debug_logs_enabled:
            p._log(f"[CAMERA] Intentando conectar a: {topic}")
        if not topic:
            p._log_error("Tópico de cámara vacío")
            p._set_status("Tópico de cámara vacío", error=True)
            return
        if p._camera_subscribed:
            if topic == p.camera_topic:
                if not self._subscription_stale():
                    if p._debug_logs_enabled:
                        p._log(f"[CAMERA] Ya conectado a: {topic}")
                    return
                if p._debug_logs_enabled:
                    p._log(f"[CAMERA] Reintentando suscripción a: {topic}")
            if p._debug_logs_enabled:
                p._log(f"[CAMERA] Desuscribiendo de: {p.camera_topic}")
            self.unsubscribe()
        p.camera_topic = topic
        p.camera_info.setText("Conectando…")
        p.camera_view.setText("Conectando…")
        p._camera_status_connected = False
        self.subscribe(topic)

    def subscribe(self, topic: str) -> bool:
        p = self._panel
        msg_type = self.resolve_msg_type(topic)
        if p._debug_logs_enabled:
            p._log(f"[CAMERA] Suscribiendo a {topic} (tipo={msg_type})")
        self.clear_frame(reset_info=False)
        p._camera_initializing = True
        p._camera_init_start = time.time()
        p._camera_subscribe_ts = p._camera_init_start
        p._critical_camera_deadline = time.monotonic() + max(0.1, CAMERA_INIT_GRACE_SEC)
        p._camera_frame_count = 0
        p._camera_depth_frame_count = 0
        p._last_camera_depth_frame_ts = 0.0
        p._camera_stream_ok = False
        p._camera_fault_since = 0.0
        p._camera_fault_active = False
        p._camera_fault_reason = ""
        try:
            subscribed = p.ros_worker.subscribe_image(topic, msg_type=msg_type)
        except Exception as exc:
            subscribed = False
            p._log_error(f"Error suscribiendo cámara: {exc}")
        p._camera_msg_type = msg_type
        if not subscribed:
            p._log_camera_diagnostics("suscripción a cámara fallida")
            p._set_status("Cámara: nodo ROS no listo (reintentando)", error=False)
            p._camera_subscribed = False
            return False
        p._camera_subscribed = True
        p._set_status(f"Suscrito a {topic}", error=False)
        if p._debug_logs_enabled:
            p._log(f"[CAMERA] Suscripción OK a {topic}")
        self._ensure_depth_subscription()
        self.start_health_check(1200)
        return True

    def start_health_check(self, delay_ms: int = 1200) -> None:
        p = self._panel
        p._camera_health_timer.setInterval(delay_ms)
        if not p._camera_health_timer.isActive():
            p._camera_health_timer.start()

    def unsubscribe(self) -> None:
        p = self._panel
        if not p._camera_subscribed or not p.camera_topic:
            return
        try:
            p.ros_worker.unsubscribe_image(p.camera_topic)
        except Exception as exc:
            p._log_warning(f"Error desuscribiendo: {exc}")
        depth_topic = p._camera_depth_topic
        if depth_topic:
            try:
                p.ros_worker.unsubscribe_image(depth_topic)
            except Exception:
                pass
        p._camera_subscribed = False
        self.clear_frame()

    def clear_frame(self, *, reset_info: bool = True) -> None:
        p = self._panel
        with p._camera_frame_lock:
            p._camera_pending_frame = None
        p.camera_view.setPixmap(QPixmap())
        if reset_info:
            p.camera_view.setText("Sin conexión")
            p.camera_info.setText("Sin conexión")
        p._camera_status_connected = False
        p._camera_msg_type = "image"
        p._last_camera_frame_ts = 0.0
        p._camera_frame_count = 0
        p._last_camera_depth_frame_ts = 0.0
        p._camera_depth_frame_count = 0
        p._camera_stream_ok = False

    def _ensure_depth_subscription(self) -> None:
        p = self._panel
        depth_required, depth_topic = p._camera_depth_expectation()
        p._camera_depth_topic = depth_topic
        if not depth_required or not depth_topic:
            return
        if not p._ros_worker_started or not p.ros_worker.node_ready():
            return
        try:
            subscribed = p.ros_worker.subscribe_image(depth_topic, msg_type="image")
        except Exception:
            subscribed = False
        if subscribed:
            p._emit_log(
                f"[CAMERA] depth subscription OK topic={depth_topic} required=true"
            )
        else:
            p._emit_log(
                f"[CAMERA][WARN] depth subscription failed topic={depth_topic} required=true"
            )

    def resolve_msg_type(self, topic: str) -> str:
        p = self._panel
        normalized = topic.lower()
        for name, types in p.ros_worker.topic_names_and_types():
            if name != topic:
                continue
            for t in types:
                lower = t.lower()
                if "compressedimage" in lower:
                    return "compressed"
                if "image" in lower:
                    return "image"
        if "compressed" in normalized:
            return "compressed"
        return "image"

    def auto_connect(self) -> None:
        p = self._panel
        if not p._bridge_running:
            if p._debug_logs_enabled:
                p._log("[CAMERA] Bridge aún no activo, reintentando auto-conexión en 1s")
            QTimer.singleShot(1000, self.auto_connect)
            return
        if p._camera_subscribed and not self._subscription_stale():
            return
        if p._camera_subscribed and self._subscription_stale():
            if p._debug_logs_enabled:
                p._log("[CAMERA] Suscripción estancada; forzando reconexión")
            self.unsubscribe()
        if not p.ros_worker.node_ready():
            p._log_camera_diagnostics("auto-connect esperando nodo ROS")
            if p._debug_logs_enabled:
                p._log("[CAMERA] Nodo ROS aún no listo, reintentando conexión en 1s")
            QTimer.singleShot(1000, self.auto_connect)
            return
        if p.camera_topic_combo.count() == 0:
            self.refresh_topics()
            QTimer.singleShot(1500, self.auto_connect)
            return
        if p.camera_topic_combo.currentText().strip().endswith("/depth_image"):
            rgb_idx = p.camera_topic_combo.findText("/camera_overhead/image")
            if rgb_idx >= 0:
                p.camera_topic_combo.setCurrentIndex(rgb_idx)
        if p._debug_logs_enabled:
            p._log("[CAMERA] Auto-conectando cámara...")
        p.signal_connect_camera.emit()
        if not p._camera_stream_ok:
            self.schedule_reconnect("auto_connect_wait_frames", delay_ms=1500)

    def on_image(self, topic: str, qimg, w: int, h: int, fps: float) -> None:
        p = self._panel
        now = time.time()
        if topic == p._camera_depth_topic and topic != p.camera_topic:
            p._last_camera_depth_frame_ts = now
            p._camera_depth_frame_count += 1
            p._tfm_preprocessed_cache = None
            return
        if topic != p.camera_topic:
            return
        with p._camera_frame_lock:
            p._camera_pending_frame = (topic, qimg, w, h, fps, now)
        if p.tfm_module and not CAMERA_SKIP_TFM_INPUT:
            info = p.tfm_module.model_info() if p.tfm_module else {}
            in_channels = int(info.get("in_channels", 3) or 3)
            if CAMERA_PREPROCESS_TFM:
                try:
                    pre = build_tfm_preprocessed_input(p, qimg, w, h, now)
                    if pre is not None:
                        p.tfm_module.set_input_image(pre, preprocessed=True)
                        p._tfm_preprocessed_cache = (float(now), pre)
                    elif in_channels == 3:
                        p.tfm_module.set_input_image(qimg, width=w, height=h)
                except Exception:
                    if in_channels == 3:
                        p.tfm_module.set_input_image(qimg, width=w, height=h)
            else:
                p.tfm_module.set_input_image(qimg, width=w, height=h)
        p._camera_last_fps = fps
        if CAMERA_TRACK_FPS:
            p._update_fps_stats(fps)
        p._last_camera_frame_ts = now
        p._camera_frame_count += 1
        if p._camera_frame_count >= 1:
            p._camera_stream_ok = True
            p._camera_ever_ok = True
            p._camera_fault_since = 0.0
            p._camera_fault_active = False
            p._camera_fault_reason = ""
            if p._metrics_enabled and p._camera_frame_count == 1:
                p._metric_mark("camera_ready")
        p._reset_camera_retry_backoff()
        if p._camera_initializing:
            p._camera_initializing = False
            p._camera_init_start = 0.0

    def refresh_display(self) -> None:
        p = self._panel
        frame = None
        with p._camera_frame_lock:
            frame = p._camera_pending_frame
            p._camera_pending_frame = None
        if not frame:
            return
        if CAMERA_UI_SKIP_HIDDEN and (not p.isVisible() or not p.camera_view.isVisible()):
            return
        topic, qimg, w, h, fps, ts = frame
        p._last_camera_frame = (qimg, w, h, ts)
        display = qimg
        if w > 0 and h > 0:
            if OVERLAY_CALIB and (p._calibrating or (time.time() <= p._calib_grid_until)):
                display = p._draw_calib_overlay(display, w, h)
            overhead_only = p._overhead_camera_active(topic)
            if OVERLAY_REACH and overhead_only and p._reach_overlay_enabled:
                display = p._draw_reach_overlay(display, w, h)
            if OVERLAY_SELECTION and overhead_only and p._selected_px:
                display = p._draw_selection_overlay(display, w, h)
            if overhead_only and p._last_grasp_px:
                display = p._draw_grasp_overlay(display, w, h)
            if TEST_CORNER_OVERLAY:
                display = p._draw_test_corner_overlay(display, w, h)
            if TCP_POSE_OVERLAY:
                display = p._draw_tcp_pose_overlay(display, w, h)
        p.camera_view.set_frame(display, w, h)
        now = time.time()
        if (now - p._camera_info_last_ts) >= max(0.05, float(CAMERA_INFO_INTERVAL_SEC)):
            p.camera_info.setText(f"Conectado · {w}x{h} · fps {fps:.1f}")
            _moving = bool(p._manual_inflight or p._script_motion_active)
            p.motion_lbl.setText("moving" if _moving else "idle")
            p.motion_lbl.setStyleSheet(
                "color:#f59e0b; font-weight:bold; font-size:13px;" if _moving
                else "color:#22c55e; font-weight:bold; font-size:13px;"
            )
            p._camera_info_last_ts = now
        if not p._camera_status_connected:
            p._set_status(f"Cámara: {topic} conectada", error=False)
            p._camera_status_connected = True
        p._last_camera_frame_ts = ts

class ControlPanelV2(QMainWindow):
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

    def get_health_report(self) -> dict:
        """Emitir un resumen JSON del pipeline: topics, nodos, servicios, world_name, pose_info, TF, cámaras, controllers, MoveIt2, etc."""
        report = {}
        # Topics y tipos
        topics_types = []
        try:
            if self._ros_worker_started and self.ros_worker.node_ready():
                topics_types = self.ros_worker.topic_names_and_types()
        except Exception as exc:
            _log_exception("health_report topics/types", exc)
        report["topics_types"] = topics_types
        # Nodos y servicios
        nodes = []
        services = []
        actions = []
        try:
            if self._ros_worker_started and self.ros_worker.node_ready():
                nodes = self.ros_worker.list_node_names() if hasattr(self.ros_worker, "list_node_names") else []
                services = self.ros_worker.list_service_names() if hasattr(self.ros_worker, "list_service_names") else []
                actions = self.ros_worker.list_action_names() if hasattr(self.ros_worker, "list_action_names") else []
        except Exception as exc:
            _log_exception("health_report nodes/services/actions", exc)
        report["nodes"] = nodes
        report["services"] = services
        report["actions"] = actions
        # world_name detectado y pose_info_topic
        world_name = self._gz_world_name or self._detect_world_name() or "unknown"
        pose_info_topic = self._discover_pose_info_topic(world_name)
        report["world_name_detected"] = world_name
        return report

    def __init__(self):
        super().__init__()
        self._state_event = threading.Event()
        self.setWindowTitle("Panel V2")
        self.setMinimumWidth(1200)
        load_object_positions()
        self.ws_dir = WS_DIR
        self.gz_proc = None
        self.bridge_proc = None
        self.bag_proc = None
        self.moveit_proc = None
        self.moveit_bridge_proc = None
        self._auto_tune_proc = None
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
        self._robot_test_done = False
        self._robot_test_disabled = False
        self._robot_test_active = False
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
        self._tf_not_ready_logged = False
        self._trace_ready = False
        self._bridge_ready = False
        self._trace_print_period = max(0.5, TRACE_PRINT_PERIOD_SEC)
        self._ee_warn_period = 5.0
        self._trace_debug_logged = False
        self._panel_start_ts = time.time()
        self._perf_start_monotonic = time.monotonic()
        self._metrics_enabled = os.environ.get("PANEL_METRICS", "").strip().lower() in (
            "1",
            "true",
            "yes",
        )
        self._diagnostic_mode = os.environ.get("PANEL_DIAG_MODE", "").strip().lower() in (
            "1",
            "true",
            "yes",
        )
        self._fatal_stops_all = os.environ.get("PANEL_FATAL_STOPS_ALL", "0").strip().lower() in (
            "1",
            "true",
            "yes",
        )
        self._perf_marks: Dict[str, float] = {}
        self._debug_logs_enabled = bool(DEBUG_LOGS_TO_STDOUT)
        self._panel_logger = _PanelLogger(self)
        self._camera_ctrl = CameraController(self)
        self._tf_monitor = TFMonitor(self)
        self._state_evaluator = PanelStateEvaluator()
        self._physics = PanelPhysics(self)
        self._joint_limits_ok = False
        self._joint_limits_err = ""
        self._tfm_ckpt_selected = INFER_CKPT or ""
        self._tfm_ckpt_meta: Dict[str, Dict[str, object]] = {}
        self._tfm_ckpt_options = self._discover_tfm_checkpoints()
        self.tfm_module = (
            TFMGraspModule(logger=self._emit_log, model_path=INFER_CKPT or "")
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
            self._camera_required = True
            if not bool(CAMERA_REQUIRED):
                self._emit_log(
                    "[SAFETY] CAMERA_REQUIRED deshabilitado por config, forzando ON (dependencia crítica)."
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
        self._auto_release_drop_objects = True
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
        self._closing = False
        self._shutdown_complete = False
        self._last_tcp_world = None
        self._last_tcp_base = None
        self._last_tcp_world_tf = None
        self._last_tcp_base_z = None
        self._last_tcp_rpy_deg = None
        self._last_tcp_mismatch_warn_ts = 0.0
        self._last_debug_tcp_base: Optional[Tuple[float, float, float]] = None
        self._last_debug_tcp_ts: float = 0.0
        self._last_trace_tcp_base: Optional[Tuple[float, float, float]] = None
        self._last_trace_tcp_ts: float = 0.0
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
        self._camera_depth_required_env = str(
            os.environ.get("PANEL_CAMERA_REQUIRE_DEPTH", "0")
        ).strip().lower() in ("1", "true", "yes", "on")
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
            str(os.environ.get("PANEL_REQUIRED_EE_FRAME", "rg2_tcp") or "rg2_tcp").strip()
            or "rg2_tcp"
        )
        self._auto_calib_from_camera = bool(AUTO_CALIB_FROM_CAMERA)
        self._calibrating = False
        self._calib_points = []  # Lista de (px, py, wx, wy)
        self._calib_grid_until = 0.0
        self._auto_joint2_move_done = False
        self._auto_pick_demo_enabled = str(
            os.environ.get("PANEL_AUTO_RUN_PICK_DEMO", "0")
        ).strip().lower() in ("1", "true", "yes", "on")
        try:
            self._auto_pick_demo_attempts = max(
                1, int(os.environ.get("PANEL_AUTO_RUN_PICK_DEMO_ATTEMPTS", "1") or 1)
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
        self._last_grasp_frame: str = ""
        self._last_grasp_source: str = ""
        self._grasp_rect_topic: str = GRASP_RECT_TOPIC
        self._grasp_rect_subscribed = False
        self._last_infer_image_path: str = ""
        self._last_infer_output_path: str = ""
        self._tfm_preprocessed_cache: Optional[Tuple[float, object]] = None
        self._infer_session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._safety = PanelSafety(self)
        self._state_machine = PanelStateMachine()
        self._watchdog = PanelWatchdog(self)
        self._exp_info: Dict[str, object] = {}
        self._tfm_infer_inflight = False
        self._tfm_execute_inflight = False
        self._marker_pub = None
        self._controller_check_inflight = False
        self._controllers_ok = False
        self._controllers_reason = "controladores no verificados"
        self._controllers_state = "STARTING"
        self._last_controller_check = 0.0
        self._controllers_last_ok_ts = 0.0
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
        self._emit_log("[STARTUP] Creando RosWorker")
        self.ros_worker = RosWorker(force_realtime=True)
        self.ros_worker.image.connect(self._camera_ctrl.on_image)
        self.ros_worker.joint_state.connect(self._on_joint_state)
        self.ros_worker.grasp_rect.connect(self._on_grasp_rect)
        self.ros_worker.log.connect(self._log_ros_message)
        self.ros_worker.system_state.connect(self._on_system_state_update)
        self.ros_worker.robot_test_request.connect(self._on_remote_robot_test_request)
        self.ros_worker.tfm_infer_request.connect(self._on_remote_tfm_infer_request)
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

    def _init_moveit_publisher(self) -> None:
        self._moveit_node, self._moveit_pose_pub, self._moveit_pose_pub_cartesian = (
            init_moveit_publishers(
                node=self._moveit_node,
                pose_pub=self._moveit_pose_pub,
                pose_pub_cartesian=self._moveit_pose_pub_cartesian,
                use_sim_time=bool(USE_SIM_TIME),
                pose_topic=MOVEIT_POSE_TOPIC,
                cartesian_topic=MOVEIT_CARTESIAN_POSE_TOPIC,
                logger=self._log,
            )
        )

    def _ensure_moveit_node(self) -> None:
        if self._moveit_node is None:
            self._init_moveit_publisher()

    def _expected_world_frame(self) -> str:
        return WORLD_FRAME or "world"

    def _business_base_frame(self) -> str:
        current = str(self._base_frame_effective or "").strip()
        if current and current != GLOBAL_FRAME_EFFECTIVE:
            self._emit_log(
                f"[FRAME][P0] BASE_FRAME_EFFECTIVE inválido={current}; se exige {GLOBAL_FRAME_EFFECTIVE}"
            )
        return GLOBAL_FRAME_EFFECTIVE

    def _base_frame_candidates(self) -> List[str]:
        return ["base_link"]

    def ensure_base_pose(
        self,
        pose_stamped_any: Optional[PoseStamped],
        *,
        timeout_sec: float = 0.6,
    ) -> Optional[PoseStamped]:
        """Convert PoseStamped from any frame into business base frame."""
        if pose_stamped_any is None:
            return None
        base_frame = self._business_base_frame()
        source_raw = str(getattr(getattr(pose_stamped_any, "header", None), "frame_id", "") or "")
        source_frame = source_raw.split("|", 1)[0].strip() or source_raw.strip()
        if not source_frame:
            source_frame = base_frame

        if source_frame == base_frame:
            out = PoseStamped()
            out.header = pose_stamped_any.header
            out.header.frame_id = base_frame
            out.pose = pose_stamped_any.pose
            return out

        helper = get_tf_helper()
        if helper is not None:
            try:
                transformed = helper.transform_pose(
                    pose_stamped_any, base_frame, timeout_sec=timeout_sec
                )
                if transformed is not None:
                    transformed.header.frame_id = base_frame
                    return transformed
            except Exception as exc:
                _log_exception("ensure_base_pose tf transform", exc)

        try:
            p = pose_stamped_any.pose.position
            coords, _ = transform_point_to_frame(
                (float(p.x), float(p.y), float(p.z)),
                base_frame,
                source_frame=source_frame,
                timeout_sec=timeout_sec,
            )
            if not coords:
                return None
            out = PoseStamped()
            out.header = pose_stamped_any.header
            out.header.frame_id = base_frame
            out.pose.position.x = float(coords[0])
            out.pose.position.y = float(coords[1])
            out.pose.position.z = float(coords[2])
            out.pose.orientation = pose_stamped_any.pose.orientation
            return out
        except Exception as exc:
            _log_exception("ensure_base_pose fallback", exc)
            return None

    def _ensure_base_coords(
        self,
        coords: Tuple[float, float, float],
        source_frame: str,
        *,
        timeout_sec: float = 0.6,
    ) -> Optional[Tuple[float, float, float]]:
        base_frame = self._business_base_frame()
        src = (source_frame or "").split("|", 1)[0].strip() or base_frame
        if src == base_frame:
            return (float(coords[0]), float(coords[1]), float(coords[2]))
        converted, _ = transform_point_to_frame(
            (float(coords[0]), float(coords[1]), float(coords[2])),
            base_frame,
            source_frame=src,
            timeout_sec=timeout_sec,
        )
        if not converted:
            return None
        return (float(converted[0]), float(converted[1]), float(converted[2]))

    def get_tcp_base(self) -> Optional[PoseStamped]:
        """Return TCP pose expressed in base_link for diagnostics and tests."""
        base_frame = self._business_base_frame()
        ee_frame = "rg2_tcp"
        tcp_pose, _rpy_deg, reason = tf_get_tcp_in_base(
            base_frame=base_frame,
            ee_frame=ee_frame,
            timeout=0.30,
            logger=self._emit_log,
        )
        if tcp_pose is None:
            self._emit_log(
                "[TF_POSE] tcp_source=tf2 status=UNAVAILABLE "
                f"base={base_frame} ee={ee_frame} reason={reason}"
            )
            return None
        return tcp_pose

    def get_tcp_pose_base(self) -> Optional[PoseStamped]:
        """Compatibility alias: return TCP pose in business base frame."""
        return self.get_tcp_base()

    def transform_pose_to_base(
        self,
        pose_stamped_any: Optional[PoseStamped],
        *,
        timeout_sec: float = 0.6,
    ) -> Optional[PoseStamped]:
        """Compatibility alias: transform any pose into business base frame."""
        return self.ensure_base_pose(pose_stamped_any, timeout_sec=timeout_sec)

    def log_pose_base(self, tag: str, pose_base: Optional[PoseStamped]) -> None:
        if pose_base is None:
            self._emit_log(f"{tag} base=unavailable")
            return
        p = pose_base.pose.position
        f = str(pose_base.header.frame_id or self._business_base_frame())
        self._emit_log(
            f"{tag} base=({float(p.x):.3f},{float(p.y):.3f},{float(p.z):.3f}) frame={f}"
        )

    def log_pose(self, tag: str, pose_any: Optional[PoseStamped]) -> None:
        """Compatibility alias: log pose in base_link, transforming if needed."""
        pose_base = self.ensure_base_pose(pose_any) if pose_any is not None else None
        self.log_pose_base(tag, pose_base)

    def get_pose_in_base(
        self,
        object_id: str,
        *,
        timeout_sec: float = 0.35,
    ) -> Optional[PoseStamped]:
        """Resolve object pose into base_link from external/world sources."""
        name = str(object_id or "").strip()
        if not name:
            self._emit_log("[FRAME_IN] source=pose_info target=base_link object=n/a status=invalid")
            return None
        objects = get_object_positions()
        raw = objects.get(name)
        if raw is None or not isinstance(raw, (list, tuple)) or len(raw) < 3:
            self._emit_log(
                f"[FRAME_IN] source=pose_info target=base_link object={name} status=not_found"
            )
            return None
        source_frame = self._world_frame_last_first()
        base_coords = self._ensure_base_coords(
            (float(raw[0]), float(raw[1]), float(raw[2])),
            source_frame,
            timeout_sec=timeout_sec,
        )
        if base_coords is None:
            self._emit_log(
                "[FRAME_IN] "
                f"source={source_frame} target={self._business_base_frame()} "
                f"object={name} status=tf_failed"
            )
            return None
        pose = PoseStamped()
        pose.header.frame_id = self._business_base_frame()
        pose.pose.position.x = float(base_coords[0])
        pose.pose.position.y = float(base_coords[1])
        pose.pose.position.z = float(base_coords[2])
        pose.pose.orientation.w = 1.0
        self._emit_log(
            "[FRAME_IN] "
            f"source={source_frame} target={pose.header.frame_id} object={name} status=ok "
            f"base=({pose.pose.position.x:.3f},{pose.pose.position.y:.3f},{pose.pose.position.z:.3f})"
        )
        return pose

    def _moveit_publish_context(self) -> Dict[str, object]:
        return {
            "calibrating": self._calibrating,
            "calib_grid_until": self._calib_grid_until,
            "moveit_required": self._moveit_required,
            "managed_mode": self._managed_mode,
            "state_ready_moveit": self._state_ready_moveit(),
            "system_state_reason": self._system_state_reason or "",
            "system_state_value": self._system_state.value,
            "moveit_ready": self._moveit_state == MoveItState.READY,
            "moveit_state_reason": self._moveit_state_reason or "",
            "moveit_block_reason": self._moveit_block_reason,
            "moveit_node": self._moveit_node,
            "moveit_pose_pub": self._moveit_pose_pub,
            "moveit_pose_pub_cartesian": self._moveit_pose_pub_cartesian,
            "pose_topic": MOVEIT_POSE_TOPIC,
            "cartesian_topic": MOVEIT_CARTESIAN_POSE_TOPIC,
            "build_pose_stamped": _build_pose_stamped,
            "emit_log": self._emit_log,
            "set_status": self._set_status,
            "log_warning": self._log,
            "logger_info": self.get_logger().info,
            "log_exception": _log_exception,
        }

    @pyqtSlot()
    def _request_auto_bridge_start(self) -> None:
        if self._managed_mode:
            return
        if not AUTO_START_BRIDGE or self._closing or self._bridge_running:
            return
        if self._auto_bridge_timer_scheduled:
            return
        self._auto_bridge_timer_scheduled = True
        if self._auto_bridge_attempts == 0:
            self._log("[AUTO] Autostart bridge: esperando Gazebo")
        QTimer.singleShot(AUTO_START_BRIDGE_DELAY_MS, self._auto_bridge_tick)

    def _auto_bridge_tick(self) -> None:
        if self._managed_mode:
            return
        self._auto_bridge_timer_scheduled = False
        if not AUTO_START_BRIDGE or self._closing or self._bridge_running:
            return
        gz_state = self._gazebo_state()
        if gz_state == "GAZEBO_OFF":
            self._auto_bridge_attempts += 1
            if self._auto_bridge_attempts >= AUTO_START_BRIDGE_MAX_RETRIES:
                self._log("[AUTO] Autostart bridge: timeout (Gazebo no disponible)")
                return
            self._auto_bridge_timer_scheduled = True
            QTimer.singleShot(1000, self._auto_bridge_tick)
            return
        self._log("[AUTO] Autostart bridge: Gazebo activo")
        self._start_bridge()

    def _get_traj_publisher(self, topic: str):
        self._ensure_moveit_node()
        if self._moveit_node is None:
            return None
        self._traj_pub, self._traj_topic = get_traj_publisher(
            self._moveit_node,
            self._traj_pub,
            self._traj_topic,
            topic,
            self._log,
        )
        return self._traj_pub

    def _get_gripper_publisher(self, topic: str):
        self._ensure_moveit_node()
        if self._moveit_node is None:
            return None
        self._gripper_pub, self._gripper_topic = get_gripper_publisher(
            self._moveit_node,
            self._gripper_pub,
            self._gripper_topic,
            topic,
            self._log,
        )
        return self._gripper_pub

    def _get_attach_publisher(self, topic: str):
        self._ensure_moveit_node()
        if self._moveit_node is None:
            return None
        return get_attach_publisher(
            self._moveit_node,
            self._attach_pubs,
            topic,
            self._log,
        )

    @staticmethod
    def _normalize_attach_name(name: str) -> str:
        if name == "pick_demo":
            return "pick_demo"
        return name

    def _find_attach_candidate(
        self,
        preferred_name: Optional[str] = None,
    ) -> Optional[str]:
        tcp_world = self._last_tcp_world
        if not tcp_world:
            return None
        positions = get_object_positions()
        if not positions:
            return None
        tcp_x, tcp_y, tcp_z = tcp_world
        selected_name = (preferred_name or self._selected_object or "").strip()
        if selected_name:
            pos = positions.get(selected_name)
            if pos is not None:
                dx = pos[0] - tcp_x
                dy = pos[1] - tcp_y
                dz = pos[2] - tcp_z
                if (dx * dx + dy * dy + dz * dz) ** 0.5 <= ATTACH_DIST_M:
                    return self._normalize_attach_name(selected_name)
        best_name = None
        best_dist = 1e9
        for name, pos in positions.items():
            dx = pos[0] - tcp_x
            dy = pos[1] - tcp_y
            dz = pos[2] - tcp_z
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best_name = name
        if best_name and best_dist <= ATTACH_DIST_M:
            return self._normalize_attach_name(best_name)
        return None

    def _attempt_attach(
        self,
        reason: str,
        *,
        selected_name: Optional[str] = None,
        tcp_base: Optional[Tuple[float, float, float]] = None,
        object_base: Optional[Tuple[float, float, float]] = None,
        object_height: Optional[float] = None,
        base_frame: Optional[str] = None,
        xy_tol_m: float = 0.02,
        z_tol_m: float = 0.03,
        z_ref_mode: Optional[str] = None,
        z_clearance_m: float = 0.0,
        tcp_stamp_ns: Optional[int] = None,
        obj_stamp_ns: Optional[int] = None,
    ) -> bool:
        """Intenta vincular el objeto seleccionado al gripper en Gazebo."""
        positions = get_object_positions() or {}
        selected = (selected_name or self._selected_object or "").strip()
        lock_active = bool(getattr(self, "_pick_target_lock_active", False))
        lock_name = str(getattr(self, "_pick_target_lock_name", "") or "")
        lock_id = str(getattr(self, "_pick_target_lock_id", "") or "n/a")
        if lock_active:
            self._emit_log(
                f"[ATTACH][LOCK] active=true lock_id={lock_id} lock_name={lock_name or 'none'} "
                f"selected_arg={selected or 'none'} reason={reason}"
            )
            if selected and lock_name and selected != lock_name:
                self._log_error(
                    "[ATTACH] target_mismatch_with_lock "
                    f"lock_id={lock_id} lock_name={lock_name} selected={selected} reason={reason}"
                )
                return False
        attach_name = None
        resolved_entity = None
        if selected:
            if selected not in positions:
                self._log_error(
                    f"[ATTACH] selected_missing_in_positions selected={selected} reason={reason}"
                )
                return False
            resolved_entity = selected
            attach_name = self._normalize_attach_name(selected)
        if attach_name is None:
            attach_name = self._find_attach_candidate(preferred_name=selected)
            resolved_entity = attach_name
        self._emit_log(
            f"[ATTACH] selected={selected or 'none'} attach_name={attach_name or 'none'} "
            f"resolved_entity={resolved_entity or 'none'} reason={reason}"
        )
        if not attach_name:
            self._log_warning(f"[PICK] Sin objeto para vincular: selected={selected or 'none'} reason={reason}")
            return False

        attach_entity = resolved_entity or attach_name
        obj_world = positions.get(attach_entity)
        frame_name = (base_frame or self._base_frame_effective or BASE_FRAME or "base_link").strip()
        tcp_base_pos = tcp_base
        if tcp_base_pos is None and self._last_tcp_world is not None:
            tcp_base_pos = world_to_base(*self._last_tcp_world)
        obj_base_pos = object_base
        if obj_base_pos is None and obj_world is not None:
            obj_base_pos = world_to_base(*obj_world)

        if tcp_base_pos is not None and obj_base_pos is not None:
            obj_height = max(0.0, float(object_height or 0.0))
            obj_center_z = float(obj_base_pos[2])
            obj_top_z = obj_center_z + (obj_height * 0.5 if obj_height > 0.0 else 0.0)
            mode = str(z_ref_mode or os.environ.get("PANEL_ATTACH_Z_REF_MODE", "top")).strip().lower()
            if mode not in ("center", "top"):
                mode = "top"
            z_clearance = float(z_clearance_m or 0.0)
            obj_ref_z = obj_center_z if mode == "center" else obj_top_z
            obj_ref_z += z_clearance
            dx = float(obj_base_pos[0]) - float(tcp_base_pos[0])
            dy = float(obj_base_pos[1]) - float(tcp_base_pos[1])
            dz = obj_ref_z - float(tcp_base_pos[2])
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            tcp_stamp_txt = "n/a" if not tcp_stamp_ns else str(int(tcp_stamp_ns))
            obj_stamp_txt = "n/a" if not obj_stamp_ns else str(int(obj_stamp_ns))
            self._emit_log(
                "[ATTACH][GEOM] "
                f"frame={frame_name} tcp=({float(tcp_base_pos[0]):.3f},{float(tcp_base_pos[1]):.3f},{float(tcp_base_pos[2]):.3f}) "
                f"obj_center=({float(obj_base_pos[0]):.3f},{float(obj_base_pos[1]):.3f},{obj_center_z:.3f}) "
                f"obj_top_z={obj_top_z:.3f} obj_height={obj_height:.3f} "
                f"z_ref_mode={mode} z_clearance={z_clearance:.3f} obj_ref_z={obj_ref_z:.3f} "
                f"tcp_stamp_ns={tcp_stamp_txt} obj_stamp_ns={obj_stamp_txt}"
            )
            self._emit_log(
                "[ATTACH] "
                f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} dist={dist:.3f} "
                f"thresh=(xy={xy_tol_m:.3f},z={z_tol_m:.3f}) frame={frame_name} "
                f"z_ref_mode={mode}"
            )
            if abs(dx) > float(xy_tol_m) or abs(dy) > float(xy_tol_m) or abs(dz) > float(z_tol_m):
                self._log_warning(
                    "[PICK] Attach rechazado por rango: "
                    f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} "
                    f"(xy_tol={xy_tol_m:.3f}, z_tol={z_tol_m:.3f})"
                )
                return False
        else:
            self._log_warning(
                "[ATTACH] rango no evaluable: "
                f"tcp_base={'ok' if tcp_base_pos is not None else 'none'} "
                f"obj_base={'ok' if obj_base_pos is not None else 'none'}"
            )

        attach_topic = f"{GRIPPER_ATTACH_PREFIX}/{attach_name}/attach"
        try:
            pub = self._get_attach_publisher(attach_topic)
            if pub is not None:
                pub_count = -1
                sub_count = -1
                if self._ros_worker_started and self.ros_worker.node_ready():
                    pub_count = int(self.ros_worker.topic_publisher_count(attach_topic))
                    sub_count = int(self.ros_worker.topic_subscriber_count(attach_topic))
                self._emit_log(
                    f"[ATTACH] pub_count={pub_count} sub_count={sub_count} topic={attach_topic}"
                )
                if sub_count <= 0:
                    wait_deadline = time.time() + 1.2
                    while time.time() < wait_deadline:
                        time.sleep(0.2)
                        if self._ros_worker_started and self.ros_worker.node_ready():
                            sub_count = int(self.ros_worker.topic_subscriber_count(attach_topic))
                            if sub_count > 0:
                                break
                    if sub_count == 0:
                        self._log_error(
                            f"[ATTACH] attach_backend_missing topic={attach_topic} "
                            "reason=no_subscribers"
                        )
                        return False
                pub.publish(Empty())
                self._emit_log(
                    f"[ATTACH] publish attach_name={attach_name} topic={attach_topic}"
                )
                return True
        except Exception as exc:
            self._log_error(f"[PICK] Error al vincular: {exc}")
        return False

    def _schedule_attach_attempt(self, reason: str, delay_ms: int = 150) -> None:
        """Programa un intento de attach después de cerrar el gripper."""
        def attempt():
            self._attempt_attach(reason)
        QTimer.singleShot(delay_ms, attempt)

    def _command_gripper(
        self,
        closed: bool,
        *,
        log_action: str = "Gripper",
        force: bool = False,
    ) -> bool:
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("Gripper bloqueado: ERROR_FATAL", error=True)
            self._emit_log_throttled(
                "SAFETY:gripper:ERROR_FATAL",
                "[SAFETY] Gripper bloqueado: ERROR_FATAL",
            )
            return False
        if not force and not self._require_manual_ready("Gripper"):
            return False
        if not force and not self._state_ready_basic():
            reason = self._system_state_reason or self._system_state.value
            self._set_status(f"Gripper bloqueado: {reason}", error=True)
            self._emit_log(f"[SAFETY] Gripper bloqueado: {reason}")
            return False
        self._gripper_closed = closed
        self._gripper_is_closed = closed
        if self.btn_gripper is not None:
            self.btn_gripper.blockSignals(True)
            self.btn_gripper.setChecked(closed)
            self.btn_gripper.setText("Abrir gripper" if closed else "Cerrar gripper")
            self.btn_gripper.blockSignals(False)
        if log_action:
            self._log_button(f"{log_action} {'cerrar' if closed else 'abrir'}")
        self._ensure_moveit_node()
        pub = self._get_gripper_publisher(GRIPPER_CMD_TOPIC)
        if pub is None:
            self._set_status("Gripper: publisher no disponible", error=True)
            self._log_warning("[GRIPPER] Publisher no disponible")
            return False
        target = GRIPPER_CLOSED_RAD if closed else GRIPPER_OPEN_RAD
        msg = Float64MultiArray()
        msg.data = [float(target), float(target) * GRIPPER_JOINT2_SIGN]
        pub.publish(msg)
        force_tag = " (force)" if force else ""
        self._emit_log(
            f"[GRIPPER] cmd target={target:.3f} rad topic={GRIPPER_CMD_TOPIC}{force_tag}"
        )
        self._set_status(f"Gripper -> {target:.3f} rad{force_tag}", error=False)
        return True

    def _traj_action_target(self, traj_topic: str) -> str:
        if not traj_topic:
            return ""
        base = traj_topic.rsplit("/joint_trajectory", 1)[0]
        return f"{base}/follow_joint_trajectory"

    def _resolve_traj_action_name(self, traj_topic: str, *, allow_fallback: bool = False) -> str:
        action_name = self._traj_action_target(traj_topic)
        if action_name:
            return action_name
        if allow_fallback:
            return "/joint_trajectory_controller/follow_joint_trajectory"
        return ""

    def _get_action_client(self, action_name: str, action_type, *, log_ctx: str = ""):
        if ActionClient is None or action_type is None:
            return None
        if self._moveit_node is None:
            return None
        cache = None
        cache_name = ""
        if action_type is MoveGroup:
            cache = self._moveit_action_client
            cache_name = getattr(cache, "action_name", "") if cache else ""
        elif action_type is FollowJointTrajectory:
            cache = self._traj_action_client
            cache_name = self._traj_action_name
        if cache is None or cache_name != action_name:
            try:
                cache = ActionClient(self._moveit_node, action_type, action_name)
            except Exception as exc:
                if log_ctx:
                    _log_exception(f"{log_ctx} create ActionClient {action_name}", exc)
                else:
                    _log_exception(f"create ActionClient {action_name}", exc)
                cache = None
            if action_type is MoveGroup:
                self._moveit_action_client = cache
            elif action_type is FollowJointTrajectory:
                self._traj_action_client = cache
                self._traj_action_name = action_name if cache else ""
        return cache

    @staticmethod
    def _wait_action_server(client, *, timeout_sec: float, log_ctx: str, action_name: str) -> bool:
        if client is None:
            return False
        try:
            return client.wait_for_server(timeout_sec=timeout_sec)
        except Exception as exc:
            _log_exception(f"wait {log_ctx} action {action_name}", exc)
            return False

    def _format_action_error(self, kind: str, detail: str) -> str:
        return f"Action {kind} no disponible: {detail}"

    def _joint_motion_since(self, snapshot: Dict[str, float]) -> bool:
        if not snapshot:
            return False
        for name, prev in snapshot.items():
            curr = self._last_joint_positions.get(name)
            if curr is None:
                continue
            if abs(curr - prev) > TRAJ_ACTION_FALLBACK_EPS_RAD:
                return True
        return False

    def _wait_for_joint_target(
        self,
        target: List[float],
        timeout_sec: float,
        tol_rad: float = 0.02,
    ) -> bool:
        start = time.time()
        while (time.time() - start) < timeout_sec:
            ok = True
            for idx, name in enumerate(UR5_JOINT_NAMES):
                if idx >= len(target):
                    break
                curr = self._last_joint_positions.get(name)
                if curr is None:
                    ok = False
                    break
                if abs(curr - target[idx]) > tol_rad:
                    ok = False
                    break
            if ok:
                return True
            time.sleep(0.05)
        return False

    def _wait_for_tcp_base_z(
        self,
        target_z: float,
        timeout_sec: float,
        tol_m: float = 0.005,
        tcp_z_offset: float = 0.0,
    ) -> bool:
        start = time.time()
        last_z = None
        base_frame = self._business_base_frame()
        ee_frame = self._ee_frame_effective or "tool0"
        while (time.time() - start) < timeout_sec:
            bz = None
            try:
                pose_base, _ = get_pose(base_frame, ee_frame, timeout_sec=0.05)
                if pose_base:
                    bz = float(pose_base["position"]["z"])
            except Exception:
                bz = None
            if bz is None and ee_frame == "tool0" and self._last_tcp_world:
                # Fallback only for tool0-based TCP to avoid stale FK when using virtual TCPs.
                _, _, bz = world_to_base(*self._last_tcp_world)
            if bz is None and self._last_tcp_base_z is not None:
                # Fallback to last TF-derived tcp base z when TF query is flaky.
                bz = self._last_tcp_base_z
            if bz is not None:
                last_z = bz
                eff_z = bz + tcp_z_offset
                if abs(eff_z - target_z) <= tol_m:
                    return True
            time.sleep(0.05)
        if last_z is not None:
            self._emit_log(
                f"[PICK] WARN: tcp_z={last_z:.3f} objetivo_z={target_z:.3f} (timeout)"
            )
        else:
            self._emit_log("[PICK] WARN: tcp_z no disponible (timeout)")
        return False

    def _wait_for_tcp_base_target(
        self,
        target_xyz: Tuple[float, float, float],
        timeout_sec: float,
        tol_xyz_m: float = 0.10,
        ee_frame: Optional[str] = None,
    ) -> Tuple[bool, Optional[Tuple[float, float, float]], float]:
        start = time.time()
        last_pos: Optional[Tuple[float, float, float]] = None
        last_dist = float("inf")
        base_frame = self._business_base_frame()
        frame_name = ee_frame or self._ee_frame_effective or "tool0"
        tx, ty, tz = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
        while (time.time() - start) < timeout_sec:
            curr_pos: Optional[Tuple[float, float, float]] = None
            try:
                pose_base, _ = get_pose(base_frame, frame_name, timeout_sec=0.05)
                if pose_base:
                    pos = pose_base.get("position", (0.0, 0.0, 0.0))
                    if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                        curr_pos = (float(pos[0]), float(pos[1]), float(pos[2]))
            except Exception:
                curr_pos = None
            if curr_pos is None and self._last_tcp_base is not None:
                curr_pos = self._last_tcp_base
            if curr_pos is not None:
                last_pos = curr_pos
                last_dist = math.sqrt(
                    (curr_pos[0] - tx) ** 2
                    + (curr_pos[1] - ty) ** 2
                    + (curr_pos[2] - tz) ** 2
                )
                if last_dist <= tol_xyz_m:
                    return True, last_pos, last_dist
            time.sleep(0.05)
        return False, last_pos, last_dist

    def _send_joint_trajectory_action(self, positions: List[float], sec: float, traj_topic: str) -> Tuple[bool, str]:
        if not ROS_AVAILABLE or ActionClient is None or FollowJointTrajectory is None:
            return False, self._format_action_error("FollowJointTrajectory", "ActionClient")
        if self._moveit_node is None:
            return False, self._ros_node_not_ready_reason()
        action_name = self._resolve_traj_action_name(traj_topic)
        if not action_name:
            return False, self._format_action_error("FollowJointTrajectory", "target vacío")
        if self._traj_action_client is None or self._traj_action_name != action_name:
            self._traj_action_client = self._get_action_client(
                action_name,
                FollowJointTrajectory,
                log_ctx="traj_action",
            )
        client = self._traj_action_client
        if client is None:
            return False, self._format_action_error("FollowJointTrajectory", "client")
        if not self._wait_action_server(
            client,
            timeout_sec=1.5,
            log_ctx="traj_action",
            action_name=action_name,
        ):
            return False, self._format_action_error("FollowJointTrajectory", action_name)
        goal = FollowJointTrajectory.Goal()
        traj = build_joint_trajectory(positions, sec, UR5_JOINT_NAMES)
        goal.trajectory = traj
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=TRAJ_ACTION_FALLBACK_TIMEOUT_SEC)
        goal_handle = future.result() if future.done() else None
        if not goal_handle or not goal_handle.accepted:
            return False, self._format_action_error("FollowJointTrajectory", "goal rechazado")
        return True, action_name

    def _schedule_traj_action_fallback(self, positions: List[float], sec: float, traj_topic: str) -> None:
        if not TRAJ_ACTION_FALLBACK:
            return
        if self._traj_action_inflight:
            return
        now = time.time()
        if (now - self._traj_fallback_last_ts) < 1.0:
            return
        snapshot = dict(self._last_joint_positions)

        def worker():
            time.sleep(TRAJ_ACTION_FALLBACK_DELAY_SEC)
            if self._closing:
                return
            if self._joint_motion_since(snapshot):
                return
            self._traj_action_inflight = True
            try:
                ok, info = self._send_joint_trajectory_action(positions, sec, traj_topic)
                if ok:
                    self._traj_fallback_last_ts = time.time()
                    self._log_traj_action_fallback(ok, info)
                else:
                    self._log_traj_action_fallback(ok, info)
            finally:
                self._traj_action_inflight = False

        self._run_async(worker)

    # Límites de posición para cada joint del UR5 (en radianes, según joint_limits.yaml).
    # Orden: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
    _UR5_JOINT_POS_LIMITS: List[Tuple[float, float]] = [
        (-6.2832, 6.2832),   # shoulder_pan_joint   ±360°
        (-6.2832, 6.2832),   # shoulder_lift_joint  ±360°
        (-3.1416, 3.1416),   # elbow_joint          ±180°
        (-6.2832, 6.2832),   # wrist_1_joint        ±360°
        (-6.2832, 6.2832),   # wrist_2_joint        ±360°
        (-6.2832, 6.2832),   # wrist_3_joint        ±360°
    ]

    def _clamp_joint_positions(self, positions: List[float]) -> Tuple[List[float], List[str]]:
        """Clamp each joint to its physical limit and return (clamped_list, warnings)."""
        clamped = list(positions)
        warnings: List[str] = []
        for idx, (lo, hi) in enumerate(self._UR5_JOINT_POS_LIMITS):
            if idx >= len(clamped):
                break
            val = float(clamped[idx])
            if val < lo or val > hi:
                name = UR5_JOINT_NAMES[idx] if idx < len(UR5_JOINT_NAMES) else f"j{idx}"
                clamped[idx] = max(lo, min(hi, val))
                warnings.append(
                    f"{name}: {math.degrees(val):.2f}° → clamped to"
                    f" {math.degrees(clamped[idx]):.2f}°"
                    f" (limit [{math.degrees(lo):.0f}°,{math.degrees(hi):.0f}°])"
                )
        return clamped, warnings

    def _publish_joint_trajectory(self, positions: List[float], sec: float) -> Tuple[bool, str]:
        if getattr(self, "_pick_moveit_phase_active", False):
            self._emit_log(
                "[MANUAL] BLOCKED: publicación manual durante fase MoveIt de PICK_OBJ"
            )
            return False, "manual bloqueado: moveit_executing"
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if not self.ros_worker.node_ready():
            return False, self._ros_node_not_ready_reason()
        ok, reason = self._wait_for_controllers_ready(CONTROLLER_READY_TIMEOUT_SEC)
        if not ok:
            return False, f"controladores no listos: {reason}"
        topic = self._select_traj_topic()
        if not topic:
            return False, "joint_trajectory_controller no disponible"
        pub = self._get_traj_publisher(topic)
        if not pub:
            return False, "Publisher JointTrajectory no disponible"
        # --- Validación de límites de joints ---
        safe_positions, limit_warnings = self._clamp_joint_positions(positions)
        for w in limit_warnings:
            self._emit_log(f"[ROBOT][JOINT_LIMIT] WARN: {w}")
        if self._traj_publish_inflight:
            self._emit_log("[ROBOT] WARN: publish JointTrajectory solapado")
        self._traj_publish_inflight = True
        stamp_msg = None
        try:
            try:
                stamp_msg = Time().to_msg()
            except Exception as exc:
                _log_exception("build JointTrajectory stamp", exc)
            traj = build_joint_trajectory(safe_positions, sec, UR5_JOINT_NAMES, stamp_msg=stamp_msg)
            self._emit_log("[MANUAL] Executing direct JointTrajectory (MoveIt bypassed)")
            pub.publish(traj)
            return True, topic
        finally:
            self._traj_publish_inflight = False

    def _log_traj_action_fallback(self, ok: bool, info: str) -> None:
        if ok:
            self._emit_log(f"[ROBOT] Fallback action en {info}")
        else:
            self._emit_log(f"[ROBOT] Fallback action falló: {info}")

    def _publish_moveit_pose(
        self,
        label: str,
        pose_data: Dict[str, object],
        *,
        cartesian: bool = False,
    ) -> bool:
        # FASE 3: Esperar a que la motion anterior termine.
        deadline = time.monotonic() + 30.0
        while self._motion_in_progress and time.monotonic() < deadline:
            time.sleep(0.1)
        if self._motion_in_progress:
            self._emit_log(f"[MOVEIT] BLOCKED: motion_in_progress timeout (label={label})")
            return False
        self._motion_in_progress = True
        if not self._moveit_required:
            self._set_status("MoveIt deshabilitado; bloqueando publicación", error=True)
            self._emit_log(f"[MOVEIT] Bloqueado: MoveIt deshabilitado (label={label})")
            self._motion_in_progress = False
            return False
        if self._moveit_state != MoveItState.READY and not self._moveit_status_ready():
            reason = self._moveit_not_ready_reason()
            self._set_status(f"MoveIt no listo; bloqueando {label} ({reason})", error=True)
            self._emit_log(f"[MOVEIT] Bloqueado: {label} ({reason})")
            self._motion_in_progress = False
            return False
        tf_timeout = max(0.2, float(os.environ.get("PANEL_MOVEIT_TF_GATE_TIMEOUT_SEC", "1.2") or 1.2))
        tf_deadline = time.monotonic() + tf_timeout
        tf_ok = False
        tf_reason = "tf_not_ready"
        while time.monotonic() < tf_deadline:
            tf_ok, tf_reason = self._tf_chain_ready_status()
            if tf_ok:
                break
            time.sleep(0.05)
        if not tf_ok:
            reason = f"TF no listo ({tf_reason})"
            self._set_status(f"MoveIt bloqueado: {reason}", error=False)
            self._emit_log(
                f"[MOVEIT] Bloqueado: {label} {reason} "
                f"(timeout={tf_timeout:.1f}s; required=base_link->{self._required_ee_frame})"
            )
            self._motion_in_progress = False
            return False
        # FASE 2: Validar JointState antes de enviar a MoveIt (evita "Found empty JointState message")
        if self._ros_worker_started and self.ros_worker:
            joint_ok, joint_reason = self.ros_worker.joint_state_valid(timeout_sec=2.0)
            if not joint_ok:
                self._emit_log(
                    f"[MOVEIT] GATING: esperando JointState válido antes de {label} (reason={joint_reason})"
                )
                # Intentar esperar hasta 2s por JointState válido
                joint_deadline = time.time() + 2.0
                while time.time() < joint_deadline:
                    joint_ok, joint_reason = self.ros_worker.joint_state_valid(timeout_sec=2.0)
                    if joint_ok:
                        break
                    time.sleep(0.1)
                if not joint_ok:
                    self._emit_log(
                        f"[MOVEIT] Bloqueado: {label} JointState no válido ({joint_reason})"
                    )
                    self._motion_in_progress = False
                    return False
        base_frame = self._business_base_frame()
        normalized = dict(pose_data or {})
        pose_frame_raw = str(normalized.get("frame") or base_frame)
        pose_frame = pose_frame_raw.split("|", 1)[0].strip() or pose_frame_raw
        suffix = ""
        if "|" in pose_frame_raw and pose_frame_raw.startswith(pose_frame):
            suffix = pose_frame_raw[len(pose_frame):]
        if pose_frame in ("base", "/base"):
            self._emit_log(
                f"[MOVEIT][P0] Bloqueado: {label} frame={pose_frame_raw} "
                f"(business frame must be {base_frame})"
            )
            self._motion_in_progress = False
            return False
        if pose_frame != base_frame:
            try:
                raw_pose = _build_pose_stamped(
                    {
                        "position": normalized.get("position", (0.0, 0.0, 0.0)),
                        "orientation": normalized.get("orientation", (0.0, 0.0, 0.0, 1.0)),
                        "frame": pose_frame,
                        "stamp_ns": normalized.get("stamp_ns", 0),
                    }
                )
                base_pose = self.ensure_base_pose(raw_pose, timeout_sec=0.8)
            except Exception as exc:
                _log_exception("moveit pose normalize build", exc)
                base_pose = None
            if base_pose is None:
                self._emit_log(
                    f"[MOVEIT] Bloqueado: {label} frame={pose_frame_raw} -> {base_frame} tf_failed"
                )
                self._emit_log(
                    f"[FRAME_IN] source={pose_frame_raw} target={base_frame} status=tf_failed"
                )
                self._motion_in_progress = False
                return False
            normalized["position"] = (
                float(base_pose.pose.position.x),
                float(base_pose.pose.position.y),
                float(base_pose.pose.position.z),
            )
            normalized["orientation"] = (
                float(base_pose.pose.orientation.x),
                float(base_pose.pose.orientation.y),
                float(base_pose.pose.orientation.z),
                float(base_pose.pose.orientation.w),
            )
            normalized["frame"] = f"{base_frame}{suffix}"
            self._emit_log(
                f"[MOVEIT] normalize_pose label={label} from={pose_frame_raw} to={normalized['frame']}"
            )
            self._emit_log(
                f"[FRAME_IN] source={pose_frame_raw} target={base_frame} status=ok"
            )
        else:
            normalized["frame"] = f"{base_frame}{suffix}"
        frame_out = str(normalized.get("frame") or base_frame)
        if "|rid=" not in frame_out:
            request_id = int(getattr(self, "_panel_moveit_request_id", 0) or 0) + 1
            setattr(self, "_panel_moveit_request_id", request_id)
            request_uuid = uuid.uuid4().hex
            normalized["frame"] = f"{frame_out}|rid={request_id}|uid={request_uuid}"
        try:
            px, py, pz = normalized.get("position", (0.0, 0.0, 0.0))
            frame_raw = str(normalized.get("frame") or base_frame)
            frame_clean = frame_raw.split("|", 1)[0].strip() or frame_raw
            self._emit_log(
                "[MOVEIT_GOAL] "
                f"label={label} frame={frame_clean} needs_tf={str(frame_clean != base_frame).lower()} "
                f"cartesian={str(bool(cartesian)).lower()} "
                f"pos=({float(px):.3f},{float(py):.3f},{float(pz):.3f}) "
                f"ee={self._ee_frame_effective or 'rg2_tcp'}"
            )
        except Exception:
            pass
        context = self._moveit_publish_context()
        self._moveit_block_reason, _published = publish_moveit_pose(
            label=label,
            pose_data=normalized,
            cartesian=cartesian,
            **context,
        )
        return bool(_published)

    @pyqtSlot()
    def _start_objects_settle_watch(self) -> None:
        self._physics.start_objects_settle_watch()

    def _invalidate_settle(self, reason: str, *, restart: bool = True) -> None:
        self._physics.invalidate_settle(reason, restart=restart)

    def _run_fall_test_async(self) -> None:
        self._physics.run_fall_test_async()

    def _objects_settle_worker(self) -> None:
        self._physics.objects_settle_worker()

    @pyqtSlot()
    def _handle_objects_settled(self) -> None:
        self._refresh_objects_from_gz()
        recalc_object_states("settled")
        self._refresh_controls()
        if self._bridge_running and not self._calibration_ready:
            self.signal_calibration_check.emit()

    def _log_calib_blocked(self, reason: str) -> None:
        now = time.time()
        if (now - self._last_calib_block_log) < 1.5:
            return
        self._last_calib_block_log = now
        self._emit_log(f"[CALIB] Bloqueada: {reason}")

    def _log_settle_snapshot(self, reason: str) -> None:
        self._physics.log_settle_snapshot(reason)

    def _request_settle_snapshot(self, reason: str) -> None:
        self._physics.request_settle_snapshot(reason)

    def wait_for_objects_to_settle(
        self,
        timeout: float = OBJECT_SETTLE_TIMEOUT_SEC,
        *,
        log_snapshot: bool = False,
    ) -> bool:
        return self._physics.wait_for_objects_to_settle(timeout=timeout, log_snapshot=log_snapshot)

    def _build_ui(self) -> None:
        root = QWidget()
        main = QVBoxLayout()
        main.setContentsMargins(8, 8, 8, 8)
        main.setSpacing(6)

        # --- Fila superior de botones y estado ---
        top = QHBoxLayout()
        top.setSpacing(6)

        self.btn_kill_hard = QPushButton("STOP")
        self.btn_kill_hard.setToolTip("Uso manual de emergencia (kill_all.sh)")
        self.btn_star = QPushButton("START")
        self.btn_star.setToolTip("Arranque automático: Gazebo, bridge, MoveIt y bridge MoveIt")
        self.btn_close_terminal = QPushButton("EXIT")
        self.btn_debug_joints = QPushButton("Debug articulaciones/poses")
        self.btn_debug_joints.setCheckable(True)
        self.btn_debug_logs = QPushButton("Debug logs")
        self.btn_debug_logs.setCheckable(True)
        self.btn_debug_logs.setChecked(self._debug_logs_enabled)
        self._apply_debug_button_style(self.btn_debug_joints, self._debug_joints_to_stdout)
        self._apply_debug_button_style(self.btn_debug_logs, self._debug_logs_enabled)

        self.btn_kill_hard.clicked.connect(lambda: self._debounced_btn_action(self.btn_kill_hard, self._stop_all))
        self.btn_star.clicked.connect(lambda: self._debounced_btn_action(self.btn_star, self._start_all))
        self.btn_close_terminal.clicked.connect(self._close_terminal)
        self.btn_debug_logs.clicked.connect(lambda: self._toggle_debug("DEBUG_LOGS_TO_STDOUT"))

        self.status_lbl = QLabel("Listo · Bridge ROS ↔ Gazebo activo (0.0s)")
        self.status_lbl.setAlignment(Qt.AlignCenter)
        self.status_lbl.setStyleSheet(
            "background:#0f172a; color:white; padding:8px 14px; border-radius:12px; font-weight:600;"
        )

        # --- Combo de mundos y modos ---
        self.world_combo = QComboBox()
        self.world_combo.setEditable(True)
        self._fill_worlds()
        self.btn_world_browse = QPushButton("Mundo…")
        self.btn_world_browse.clicked.connect(self._choose_world)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Auto", "Headless", "GUI"])
        self.btn_gz_start = QPushButton("Lanzar Gazebo")
        self.btn_gz_stop = QPushButton("Detener Gazebo")
        self.btn_gz_start.clicked.connect(self._start_gazebo)
        self.btn_gz_stop.clicked.connect(self._stop_gazebo)
        self.btn_debug_joints.clicked.connect(self._toggle_debug_poses)

        # --- Bridge YAML ---
        self.bridge_presets = QComboBox()
        self._fill_bridge_presets()
        self.bridge_presets.currentTextChanged.connect(self._apply_bridge_preset)
        self.bridge_edit = QLineEdit(BRIDGE_BASE_YAML)
        self.btn_bridge_browse = QPushButton("YAML…")
        self.btn_bridge_browse.clicked.connect(self._choose_yaml)
        self.btn_bridge_start = QPushButton("Lanzar bridge")
        self.btn_bridge_stop = QPushButton("Detener bridge")
        self.btn_bridge_start.clicked.connect(lambda: self._debounced_btn_action(self.btn_bridge_start, self._start_bridge))
        self.btn_bridge_stop.clicked.connect(lambda: self._debounced_btn_action(self.btn_bridge_stop, self._stop_bridge))

        # --- Bag ---
        self.bag_name = QLineEdit(f"demo_{int(time.time())}")
        self.bag_topics = QLineEdit(
            "/camera_overhead/image /camera_north/image /camera_lateral/image /camera_east/image /camera_west/image"
        )
        self.btn_bag_start = QPushButton("Start bag")
        self.btn_bag_stop = QPushButton("Stop bag")
        self.btn_bag_start.clicked.connect(lambda: self._debounced_btn_action(self.btn_bag_start, self._start_bag))
        self.btn_bag_stop.clicked.connect(lambda: self._debounced_btn_action(self.btn_bag_stop, self._stop_bag))

        # --- MoveIt ---
        self.btn_moveit_start = QPushButton("Lanzar MoveIt")
        self.btn_moveit_stop = QPushButton("Detener MoveIt")
        self.btn_moveit_bridge_start = QPushButton("Lanzar MoveIt bridge")
        self.btn_moveit_bridge_stop = QPushButton("Detener MoveIt bridge")
        self.btn_moveit_start.clicked.connect(
            lambda: self._debounced_btn_action(self.btn_moveit_start, self._start_moveit)
        )
        self.btn_moveit_stop.clicked.connect(
            lambda: self._debounced_btn_action(self.btn_moveit_stop, self._stop_moveit)
        )
        self.btn_moveit_bridge_start.clicked.connect(
            lambda: self._debounced_btn_action(self.btn_moveit_bridge_start, self._start_moveit_bridge)
        )
        self.btn_moveit_bridge_stop.clicked.connect(
            lambda: self._debounced_btn_action(self.btn_moveit_bridge_stop, self._stop_moveit_bridge)
        )

        # --- LEDs de estado ---
        self.led_gz = QLabel()
        self.led_bridge = QLabel()
        self.led_clock = QLabel()
        self.led_bag = QLabel()
        self.led_ros2 = QLabel()
        self.led_ur5 = QLabel()
        self.led_moveit = QLabel()
        self.led_moveit_bridge = QLabel()
        for led in (
            self.led_gz,
            self.led_bridge,
            self.led_clock,
            self.led_bag,
            self.led_ros2,
            self.led_ur5,
            self.led_moveit,
            self.led_moveit_bridge,
        ):
            set_led(led, "off")

        # --- Sistema: labels para CPU/RAM/Load ---
        self.sys_cpu_lbl = QLabel("CPU  --")
        self.sys_ram_lbl = QLabel("RAM  --")
        self.sys_load_lbl = QLabel("Load  --")
        self.sys_health_lbl = QLabel("Todo OK")
        for lbl in (self.sys_cpu_lbl, self.sys_ram_lbl, self.sys_load_lbl, self.sys_health_lbl):
            lbl.setTextInteractionFlags(Qt.NoTextInteraction)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status_async)
        self.status_updated.connect(self._apply_status)

        for btn in (
            self.btn_kill_hard,
            self.btn_star,
            self.btn_close_terminal,
            self.btn_debug_joints,
            self.btn_debug_logs,
        ):
            btn.setMinimumHeight(32)

        top.addWidget(self.btn_star)
        top.addWidget(self.btn_kill_hard)
        top.addWidget(self.btn_close_terminal)
        top.addWidget(self.btn_debug_joints)
        top.addWidget(self.btn_debug_logs)
        top.addWidget(self.status_lbl, 1)

        main.addLayout(top)

        controls_status_row = QHBoxLayout()
        controls_status_row.setSpacing(6)
        controls_col = QVBoxLayout()
        controls_col.setSpacing(4)

        gz_row = QHBoxLayout()
        gz_row.setSpacing(6)
        gz_row.addWidget(QLabel("Mundo:"))
        gz_row.addWidget(self.world_combo, 1)
        gz_row.addWidget(self.btn_world_browse)
        gz_row.addWidget(QLabel("Modo:"))
        gz_row.addWidget(self.mode_combo)
        gz_row.addWidget(self.btn_gz_start)
        gz_row.addWidget(self.btn_gz_stop)
        controls_col.addLayout(gz_row)

        br_row = QHBoxLayout()
        br_row.setSpacing(6)
        br_row.addWidget(QLabel("Bridge YAML:"))
        br_row.addWidget(self.bridge_presets)
        br_row.addWidget(self.bridge_edit, 1)
        br_row.addWidget(self.btn_bridge_browse)
        br_row.addWidget(self.btn_bridge_start)
        br_row.addWidget(self.btn_bridge_stop)
        controls_col.addLayout(br_row)

        bag_row = QHBoxLayout()
        bag_row.setSpacing(6)
        bag_row.addWidget(QLabel("Bag nombre:"))
        bag_row.addWidget(self.bag_name, 1)
        bag_row.addWidget(QLabel("Tópicos:"))
        bag_row.addWidget(self.bag_topics, 2)
        bag_row.addWidget(self.btn_bag_start)
        bag_row.addWidget(self.btn_bag_stop)
        controls_col.addLayout(bag_row)

        moveit_row = QHBoxLayout()
        moveit_row.setSpacing(6)
        moveit_row.addWidget(QLabel("MoveIt:"))
        moveit_row.addWidget(self.btn_moveit_start)
        moveit_row.addWidget(self.btn_moveit_stop)
        moveit_row.addSpacing(8)
        moveit_row.addWidget(QLabel("Bridge:"))
        moveit_row.addWidget(self.btn_moveit_bridge_start)
        moveit_row.addWidget(self.btn_moveit_bridge_stop)
        moveit_row.addStretch(1)
        controls_col.addLayout(moveit_row)

        controls_status_row.addLayout(controls_col, 3)

        status_group = QGroupBox("")
        status_group.setFlat(True)
        status_grid = QGridLayout()
        status_grid.setSpacing(4)
        status_grid.addWidget(QLabel("Gazebo"), 0, 0)
        status_grid.addWidget(self.led_gz, 0, 1)
        status_grid.addWidget(QLabel("Gazebo bridge"), 0, 2)
        status_grid.addWidget(self.led_bridge, 0, 3)
        status_grid.addWidget(QLabel("/clock"), 1, 0)
        status_grid.addWidget(self.led_clock, 1, 1)
        status_grid.addWidget(QLabel("Rosbag"), 1, 2)
        status_grid.addWidget(self.led_bag, 1, 3)
        status_grid.addWidget(QLabel("ros2_control"), 2, 0)
        status_grid.addWidget(self.led_ros2, 2, 1)
        status_grid.addWidget(QLabel("UR5 (sim)"), 2, 2)
        status_grid.addWidget(self.led_ur5, 2, 3)
        self.lbl_moveit_status = QLabel("MoveIt")
        status_grid.addWidget(self.lbl_moveit_status, 3, 0)
        status_grid.addWidget(self.led_moveit, 3, 1)
        self.lbl_moveit_bridge_status = QLabel("MoveIt bridge")
        status_grid.addWidget(self.lbl_moveit_bridge_status, 3, 2)
        status_grid.addWidget(self.led_moveit_bridge, 3, 3)
        status_group.setLayout(status_grid)
        controls_status_row.addWidget(status_group, 1)

        sys_group = QGroupBox("")
        sys_group.setFlat(True)
        sys_group.setStyleSheet("")
        sys_layout = QVBoxLayout()
        sys_layout.setContentsMargins(6, 6, 6, 6)
        sys_layout.setSpacing(2)
        sys_layout.addWidget(self.sys_cpu_lbl)
        sys_layout.addWidget(self.sys_ram_lbl)
        sys_layout.addWidget(self.sys_load_lbl)
        sys_layout.addWidget(self.sys_health_lbl)
        sys_group.setLayout(sys_layout)
        controls_status_row.addWidget(sys_group, 0)

        main.addLayout(controls_status_row)

        cam_group = QGroupBox("")
        cam_group.setFlat(True)
        cam_group.setStyleSheet("QGroupBox { margin:0; padding:0; }")
        cam_layout = QVBoxLayout()
        cam_layout.setContentsMargins(0, 6, 6, 6)
        cam_layout.setSpacing(2)

        cam_top = QHBoxLayout()
        cam_top.setSpacing(4)
        self.camera_topic_combo = QComboBox()
        self.camera_topic_combo.setEditable(True)
        self.camera_topic_combo.addItem(self.camera_topic)
        self.btn_camera_refresh = QPushButton("↻")
        self.btn_camera_refresh.setToolTip("Detectar tópicos")
        self.btn_camera_refresh.setMaximumWidth(32)
        self.btn_camera_refresh.clicked.connect(self._camera_ctrl.refresh_topics)
        self.btn_camera_connect = QPushButton("Conectar")
        self.btn_camera_connect.clicked.connect(self._camera_ctrl.connect)
        self.btn_recover = QPushButton("Recover")
        self.btn_recover.setToolTip("Diagnóstico: reintenta Gazebo/bridge/cámara sin stop_all")
        self.btn_recover.clicked.connect(self._recover_runtime)
        cam_top.addWidget(QLabel("Tópico:"))
        cam_top.addWidget(self.camera_topic_combo, 1)
        cam_top.addWidget(self.btn_camera_refresh)
        cam_top.addWidget(self.btn_camera_connect)
        cam_top.addWidget(self.btn_recover)
        self.btn_calibrate = None
        self.btn_release_objects = None
        cam_layout.addLayout(cam_top)

        self.camera_view = CameraView("Esperando imagen...")
        self.camera_view.clicked.connect(self._on_camera_click)
        self.camera_info = QLabel("Sin conexión")
        self.camera_info.setStyleSheet("color:#64748b; font-size:13px;")
        self.motion_lbl = QLabel("idle")
        self.motion_lbl.setStyleSheet("color:#22c55e; font-weight:bold; font-size:13px;")
        cam_info_row = QHBoxLayout()
        cam_info_row.setContentsMargins(0, 0, 0, 0)
        cam_info_row.addWidget(self.camera_info)
        cam_info_row.addWidget(self.motion_lbl)
        cam_info_row.addStretch(1)
        cam_layout.addWidget(self.camera_view, 1)
        cam_layout.addLayout(cam_info_row)
        cam_group.setLayout(cam_layout)
        self._last_camera_frame_ts = 0.0
        self._camera_frame_lock = threading.Lock()
        self._camera_pending_frame = None
        self._camera_initializing = False
        self._camera_init_start = 0.0
        self._camera_display_timer = QTimer(self)
        self._camera_display_timer.setInterval(int(CAMERA_DISPLAY_INTERVAL_MS))
        self._camera_display_timer.timeout.connect(self._camera_ctrl.refresh_display)
        self._camera_display_timer.start()
        self._camera_health_timer = QTimer(self)
        self._camera_health_timer.setInterval(1200)
        self._camera_health_timer.timeout.connect(self._camera_ctrl.health_check)
        self._camera_msg_type = "image"
        self._camera_status_connected = False
        self._camera_info_last_ts = 0.0

        self.obj_panel = ObjectListPanel()
        self.obj_panel.selected.connect(self._on_object_clicked)
        self.obj_panel.setFixedWidth(52)

        g_manual = QGroupBox("")
        g_manual.setFlat(True)
        g_manual.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        g_manual.setMaximumHeight(380)
        manual_layout = QVBoxLayout()
        manual_layout.setContentsMargins(6, 6, 6, 6)
        manual_layout.setSpacing(3)

        manual_top = QHBoxLayout()
        manual_top.setSpacing(3)
        self.btn_send_joints = QPushButton("Mover articulaciones")
        self.btn_send_joints.setMinimumWidth(140)
        self.btn_send_joints.clicked.connect(self._send_joints)
        self.joint_time = QDoubleSpinBox()
        self.joint_time.setDecimals(2)
        self.joint_time.setRange(0.5, 8.0)
        self.joint_time.setSingleStep(0.25)
        self.joint_time.setValue(DEFAULT_JOINT_MOVE_SEC)
        self.joint_time.setSuffix(" s")
        self.joint_time.setMaximumWidth(70)
        self.chk_auto_joints = QCheckBox("Auto")
        self.chk_auto_joints.setChecked(True)
        self.btn_gripper = QPushButton("Cerrar gripper")
        self.btn_gripper.setCheckable(True)
        self.btn_gripper.setMinimumHeight(28)
        self.btn_gripper.clicked.connect(self._toggle_gripper_button)
        manual_top.addWidget(self.btn_send_joints)
        manual_top.addWidget(QLabel("t"))
        manual_top.addWidget(self.joint_time)
        manual_top.addWidget(self.chk_auto_joints)
        manual_top.addWidget(self.btn_gripper)
        manual_top.addStretch(1)
        manual_layout.addLayout(manual_top)

        slider_grid = QGridLayout()
        slider_grid.setHorizontalSpacing(3)
        slider_grid.setVerticalSpacing(2)
        slider_grid.setColumnStretch(2, 1)
        self.joint_sliders = []
        self.joint_value_labels = []
        slider_min = int(JOINT_SLIDER_DEG_MIN * JOINT_SLIDER_SCALE)
        slider_max = int(JOINT_SLIDER_DEG_MAX * JOINT_SLIDER_SCALE)
        home_pose = load_home_pose()
        for idx, joint in enumerate(UR5_JOINT_NAMES):
            jlabel = QLabel(f"J{idx + 1}")
            jlabel.setToolTip(joint)
            jlabel.setStyleSheet("font-size:9px;")
            btn_minus = QPushButton("-")
            btn_plus = QPushButton("+")
            for b in (btn_minus, btn_plus):
                b.setFixedWidth(18)
                b.setFixedHeight(18)
                b.setStyleSheet("font-size:9px; padding:0;")
            btn_minus.clicked.connect(lambda _=False, i=idx: self._step_joint(i, -1))
            btn_plus.clicked.connect(lambda _=False, i=idx: self._step_joint(i, 1))
            slider = QSlider(Qt.Horizontal)
            slider.setRange(slider_min, slider_max)
            slider.setSingleStep(1)
            slider.setPageStep(10)
            slider.setFixedHeight(18)
            slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            value_lbl = QLabel("0.0 deg / 0.00 rad")
            value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_lbl.setFixedWidth(120)
            value_lbl.setStyleSheet("font-size:9px;")
            slider.valueChanged.connect(lambda v, i=idx: self._on_slider_change(i, v))
            slider.sliderReleased.connect(self._maybe_send_auto)
            self.joint_sliders.append(slider)
            self.joint_value_labels.append(value_lbl)
            if idx < len(home_pose):
                slider.setValue(int(round(math.degrees(home_pose[idx]) * JOINT_SLIDER_SCALE)))
            else:
                slider.setValue(0)
            slider_grid.addWidget(jlabel, idx, 0)
            slider_grid.addWidget(btn_minus, idx, 1)
            slider_grid.addWidget(slider, idx, 2)
            slider_grid.addWidget(btn_plus, idx, 3)
            slider_grid.addWidget(value_lbl, idx, 4)
        manual_layout.addLayout(slider_grid)

        info_grid = QGridLayout()
        info_grid.setHorizontalSpacing(6)
        info_grid.setVerticalSpacing(3)
        info_font_css = (
            "QGroupBox {"
            "font-size:11px; font-weight:700;"
            "background:#f8fafc;"
            "border:1px solid #cbd5f5;"
            "border-radius:8px;"
            "margin-top:6px;"
            "padding:8px;"
            "}"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; top: -4px; padding:0 4px; }"
            "QLabel { font-size:10px; }"
        )
        info_grid.setColumnStretch(0, 1)
        info_grid.setColumnStretch(1, 1)
        info_grid.setColumnStretch(2, 1)

        self.dof_pos_labels: Dict[str, QLabel] = {}
        self.dof_vel_labels: Dict[str, QLabel] = {}
        self.gripper_labels: Dict[str, QLabel] = {}
        self.gripper_total_lbl: Optional[QLabel] = None
        self.tcp_xyz_lbl: Optional[QLabel] = None
        self.tcp_rpy_lbl: Optional[QLabel] = None
        self.vel_norm_lbl: Optional[QLabel] = None
        self.vel_max_lbl: Optional[QLabel] = None
        self.eff_max_lbl: Optional[QLabel] = None

        dof_group = QGroupBox("DOF UR5 (pos/vel)")
        dof_group.setFlat(True)
        dof_group.setStyleSheet(info_font_css)
        dof_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        dof_layout = QGridLayout()
        dof_layout.setContentsMargins(2, 2, 2, 2)
        dof_layout.setSpacing(1)
        dof_headers = ["Joint", "Posición", "Velocidad"]
        dof_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        for col, header in enumerate(dof_headers):
            dof_layout.addWidget(QLabel(f"<b>{header}</b>"), 0, col)
        for row, joint in enumerate(dof_joints, 1):
            name_lbl = QLabel(joint.replace("_", " "))
            pos_lbl = QLabel("--")
            vel_lbl = QLabel("--")
            self.dof_pos_labels[joint] = pos_lbl
            self.dof_vel_labels[joint] = vel_lbl
            dof_layout.addWidget(name_lbl, row, 0)
            dof_layout.addWidget(pos_lbl, row, 1)
            dof_layout.addWidget(vel_lbl, row, 2)
        dof_group.setLayout(dof_layout)
        info_grid.addWidget(dof_group, 0, 0, 2, 1)

        gripper_group = QGroupBox("Pinza RG2 (pos)")
        gripper_group.setFlat(True)
        gripper_group.setStyleSheet(info_font_css)
        gripper_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        gripper_layout = QGridLayout()
        gripper_layout.setContentsMargins(2, 2, 2, 2)
        gripper_layout.setSpacing(1)
        gripper_layout.addWidget(QLabel("<b>Joint</b>"), 0, 0)
        gripper_layout.addWidget(QLabel("<b>Apertura</b>"), 0, 1)
        for row, joint in enumerate(["rg2_finger_joint1", "rg2_finger_joint2"], start=1):
            name_lbl = QLabel(joint)
            pos_lbl = QLabel("--")
            self.gripper_labels[joint] = pos_lbl
            gripper_layout.addWidget(name_lbl, row, 0)
            gripper_layout.addWidget(pos_lbl, row, 1)
        self.gripper_total_lbl = QLabel("--")
        gripper_layout.addWidget(QLabel("Apertura total"), 3, 0)
        gripper_layout.addWidget(self.gripper_total_lbl, 3, 1)
        gripper_group.setLayout(gripper_layout)
        info_grid.addWidget(gripper_group, 0, 1, 2, 1)

        kin_group = QGroupBox("Cinemática (FK)")
        kin_group.setFlat(True)
        kin_group.setStyleSheet(info_font_css)
        kin_layout = QGridLayout()
        kin_layout.setContentsMargins(2, 2, 2, 2)
        kin_layout.setSpacing(1)
        self.tcp_xyz_lbl = QLabel("--")
        self.tcp_rpy_lbl = QLabel("--")
        kin_layout.addWidget(QLabel("<b>TCP xyz [m]</b>"), 0, 0)
        kin_layout.addWidget(self.tcp_xyz_lbl, 0, 1)
        kin_layout.addWidget(QLabel("<b>RPY [deg]</b>"), 1, 0)
        kin_layout.addWidget(self.tcp_rpy_lbl, 1, 1)
        kin_group.setLayout(kin_layout)
        info_grid.addWidget(kin_group, 0, 2)

        dyn_group = QGroupBox("Dinámica (vel/eff)")
        dyn_group.setFlat(True)
        dyn_group.setStyleSheet(info_font_css)
        dyn_layout = QGridLayout()
        dyn_layout.setContentsMargins(2, 2, 2, 2)
        dyn_layout.setSpacing(1)
        self.vel_max_lbl = QLabel("--")
        self.eff_max_lbl = QLabel("--")
        self.vel_norm_lbl = QLabel("--")
        dyn_layout.addWidget(QLabel("<b>||qdot||</b>"), 0, 0)
        dyn_layout.addWidget(self.vel_norm_lbl, 0, 1)
        dyn_layout.addWidget(QLabel("<b>max |qdot|</b>"), 1, 0)
        dyn_layout.addWidget(self.vel_max_lbl, 1, 1)
        dyn_layout.addWidget(QLabel("<b>max |eff|</b>"), 2, 0)
        dyn_layout.addWidget(self.eff_max_lbl, 2, 1)
        dyn_group.setLayout(dyn_layout)
        info_grid.addWidget(dyn_group, 1, 2)

        manual_layout.addLayout(info_grid)

        self.lbl_joint_states = QLabel("Joint states: esperando /joint_states ...")
        self.lbl_joint_states.setStyleSheet("color:#64748b; font-size:10px;")
        manual_layout.addWidget(self.lbl_joint_states)
        manual_layout.addStretch(1)
        g_manual.setLayout(manual_layout)
        self.trace_group = self._build_trace_group()
        self.science_group = self._build_science_group()

        g_robot = QGroupBox("")
        g_robot.setFlat(True)
        g_robot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        robot_layout = QGridLayout()
        robot_layout.setContentsMargins(6, 8, 6, 6)
        robot_layout.setHorizontalSpacing(12)
        robot_layout.setVerticalSpacing(6)

        self.btn_test_robot = QPushButton("TEST ROBOT")
        self.btn_auto_tune_touch = QPushButton("AUTO TUNE TOUCH (MoveIt)")
        self.btn_home = QPushButton("UR5 → HOME")
        self.btn_table = QPushButton("UR5 → Mesa")
        self.btn_basket = QPushButton("UR5 → Cesta")
        for b in (self.btn_test_robot, self.btn_auto_tune_touch, self.btn_home, self.btn_table, self.btn_basket):
            b.setMinimumHeight(32)
        self.btn_test_robot.clicked.connect(self._run_robot_test)
        self.btn_auto_tune_touch.clicked.connect(self._run_auto_tune_touch)
        self.btn_home.clicked.connect(self._go_home)
        self.btn_table.clicked.connect(self._go_table)
        self.btn_basket.clicked.connect(self._go_basket)

        self.btn_pick_demo = QPushButton("PICK MESA → CESTA (DEMO)")
        self.btn_pick_demo.setMinimumHeight(32)
        self.btn_pick_demo.setToolTip(
            "Demo pick & place con objeto de posicion conocida (fuera del TFM)"
        )
        self.btn_pick_demo.clicked.connect(self._run_pick_demo)
        self.btn_pick_object = QPushButton("PICK Objeto")
        self.btn_pick_object.setMinimumHeight(32)
        self.btn_pick_object.setToolTip("Coger objeto seleccionado y llevarlo a la cesta")
        self.btn_pick_object.clicked.connect(self._run_pick_object)
        self.combo_tfm_experiment = QComboBox()
        self.combo_tfm_experiment.setMinimumHeight(30)
        self.chk_tfm_use_depth = QCheckBox("Usar RGB-D (depth)")
        self.chk_tfm_use_depth.setToolTip("Se muestran EXP1-EXP4 con su mejor checkpoint")
        self.chk_tfm_use_depth.setChecked(True)
        self.chk_tfm_use_depth.setEnabled(False)
        self.btn_tfm_apply = QPushButton("Aplicar experimento")
        self.btn_tfm_infer = QPushButton("Inferir agarre")
        self.btn_tfm_visualize = QPushButton("Comparar grasp/ref")
        self.btn_tfm_visualize.setToolTip(
            "Alterna comparación entre grasp inferido (P) y referencia Cornell del objeto seleccionado (R)"
        )
        self.btn_tfm_publish = QPushButton("Ejecutar agarre")
        self.btn_tfm_reset = QPushButton("Reset TFM")
        for b in (
            self.btn_tfm_apply,
            self.btn_tfm_infer,
            self.btn_tfm_visualize,
            self.btn_tfm_publish,
            self.btn_tfm_reset,
        ):
            b.setMinimumHeight(32)
        self.btn_tfm_apply.clicked.connect(self._tfm_apply_experiment)
        self.btn_tfm_infer.clicked.connect(self._tfm_infer_grasp)
        self.btn_tfm_visualize.clicked.connect(self._tfm_visualize_grasp)
        self.btn_tfm_publish.clicked.connect(self._tfm_publish_grasp)
        self.btn_tfm_reset.clicked.connect(self._tfm_reset_grasp)
        self.chk_tfm_use_depth.stateChanged.connect(lambda _state: self._refresh_tfm_checkpoint_options())
        self._refresh_tfm_checkpoint_options()
        if not self.tfm_module:
            self.combo_tfm_experiment.setEnabled(False)
            self.chk_tfm_use_depth.setEnabled(False)
            self.btn_tfm_apply.setEnabled(False)
            self.btn_tfm_infer.setEnabled(False)
            self.btn_tfm_visualize.setEnabled(False)
            self.btn_tfm_publish.setEnabled(False)
            self.btn_tfm_reset.setEnabled(False)

        baseline_title = QLabel("Baseline (UR5 / Demo)")
        baseline_title.setStyleSheet("font-weight:700;")
        tfm_title = QLabel("TFM - Agarre Inteligente")
        tfm_title.setStyleSheet("font-weight:700;")
        baseline_info = QLabel("Pose objeto: Gazebo (/world/.../pose/info)")
        baseline_info.setWordWrap(True)
        baseline_info.setStyleSheet("color:#666; font-size:11px;")
        tfm_info = QLabel("Pose objeto: camara + homografia/TFM")
        tfm_info.setWordWrap(True)
        tfm_info.setStyleSheet("color:#666; font-size:11px;")

        baseline_col = QVBoxLayout()
        baseline_col.setSpacing(6)
        baseline_col.addWidget(self.btn_test_robot)
        baseline_col.addWidget(self.btn_auto_tune_touch)
        baseline_col.addWidget(self.btn_home)
        baseline_col.addWidget(self.btn_table)
        baseline_col.addWidget(self.btn_basket)
        baseline_col.addWidget(self.btn_pick_demo)
        baseline_col.addWidget(self.btn_pick_object)
        # Botón Soltar objetos
        self.btn_release_objects = QPushButton("Soltar objetos")
        self.btn_release_objects.setMinimumHeight(32)
        self.btn_release_objects.setToolTip("Libera todos los objetos del drop anchor en Gazebo")
        self.btn_release_objects.clicked.connect(self._release_objects)
        baseline_col.addWidget(self.btn_release_objects)
        baseline_col.addWidget(baseline_info)
        baseline_col.addStretch(1)

        tfm_col = QVBoxLayout()
        tfm_col.setSpacing(6)
        tfm_col.addWidget(self.combo_tfm_experiment)
        tfm_col.addWidget(self.chk_tfm_use_depth)
        tfm_col.addWidget(self.btn_tfm_apply)
        tfm_col.addWidget(self.btn_tfm_infer)
        tfm_col.addWidget(self.btn_tfm_visualize)
        tfm_col.addWidget(self.btn_tfm_publish)
        tfm_col.addWidget(self.btn_tfm_reset)
        tfm_col.addWidget(tfm_info)
        tfm_col.addStretch(1)

        robot_layout.addWidget(baseline_title, 0, 0)
        robot_layout.addWidget(tfm_title, 0, 1)
        robot_layout.addLayout(baseline_col, 1, 0)
        robot_layout.addLayout(tfm_col, 1, 1)
        g_robot.setLayout(robot_layout)

        left_col = QVBoxLayout()
        left_col.setSpacing(4)
        cam_row = QHBoxLayout()
        cam_row.setContentsMargins(0, 0, 0, 0)
        cam_row.setSpacing(0)
        cam_row.addWidget(self.obj_panel, 0, Qt.AlignTop)
        cam_row.addWidget(cam_group, 1)
        left_col.addLayout(cam_row, 2)
        left_col.addWidget(g_robot, 1)
        left_col.addStretch(1)

        right_col = QVBoxLayout()
        right_col.setSpacing(6)
        right_col.addWidget(g_manual, 2)
        object_trace_layout = QHBoxLayout()
        object_trace_layout.setSpacing(6)
        if self.trace_group:
            object_trace_layout.addWidget(self.trace_group, 1)
        if self.science_group:
            object_trace_layout.addWidget(self.science_group, 1)
        right_col.addLayout(object_trace_layout, 1)
        right_col.addStretch(1)

        top_row = QHBoxLayout()
        top_row.setSpacing(8)
        top_row.addLayout(left_col, 1)
        top_row.addLayout(right_col, 1)

        main.addLayout(top_row)

        # Al final del método, tras crear todos los widgets:
        root.setLayout(main)
        self.setCentralWidget(root)
        self._start_trace_timer()
        self._apply_status(False, False, False, False, False, False, False)

    def _debounced_btn_action(self, btn, action, delay_ms=1200):
        if not btn.isEnabled():
            return
        btn.setEnabled(False)
        try:
            action()
        finally:
            QTimer.singleShot(delay_ms, lambda: btn.setEnabled(True))

    def showEvent(self, event):
        super().showEvent(event)
        if not self._timers_started:
            self.status_timer.start(2500)
            QTimer.singleShot(1200, self._run_startup_tf_sanity_check_once)
            self._timers_started = True

    def _set_status(self, text: str, error: bool = False):
        if getattr(self, "_last_status_text", None) == text and getattr(self, "_last_status_error", None) == error:
            return
        self._last_status_text = text
        self._last_status_error = error
        color = "#dc2626" if error else "#0f172a"
        self.status_lbl.setText(text)
        self.status_lbl.setStyleSheet(
            f"background:{color}; color:white; padding:8px 14px; border-radius:12px; font-weight:600;"
        )
        # SIEMPRE loguear errores; info solo si debug está activo
        if error:
            self._emit_log(f"[ERROR] {text}")
        elif self._debug_logs_enabled:
            self._emit_log(f"[INFO] {text}")

    @pyqtSlot(str, bool)
    def _set_status_async(self, text: str, error: bool = False) -> None:
        self._set_status(text, error=error)

    @pyqtSlot(object, str)
    def _set_led_async(self, led_obj: object, state: str) -> None:
        set_led(led_obj, state)

    @pyqtSlot(bool)
    def _on_tf_ready_signal(self, ready: bool) -> None:
        self._tf_ready_state = ready
        if not ready:
            self._trace_ready = False
        elif self._metrics_enabled and not self._tf_ever_ok:
            self._metric_mark("tf_ready")
        self._refresh_controls()

    @pyqtSlot(bool)
    def _on_calib_ready_signal(self, ready: bool) -> None:
        self._calibration_ready = ready
        self._refresh_controls()

    def _run_startup_tf_sanity_check_once(self) -> None:
        if self._closing:
            return
        ok, reason = self._tf_sanity_check()
        if ok:
            self._emit_log(f"[STARTUP][TF_SANITY] OK {reason}")
            return
        msg = f"TF pendiente: {reason}"
        self._set_status(msg, error=False)
        self._emit_log(f"[STARTUP][TF_SANITY] WAIT {reason} (bloqueando TEST/PICK hasta TF EE)")

    @pyqtSlot(bool)
    def _on_controllers_ready_signal(self, ready: bool) -> None:
        self._controllers_ok = ready
        self._refresh_controls()

    @pyqtSlot(str)
    def _on_error_signal(self, msg: str) -> None:
        self._system_error_reason = msg
        self._set_system_state(SystemState.ERROR, msg)
        self._set_status(msg, error=True)
        self._refresh_controls()

    @pyqtSlot(str, str)
    def _on_moveit_state_signal(self, state: str, reason: str) -> None:
        try:
            state_enum = MoveItState[state]
        except KeyError:
            return
        self._moveit_state = state_enum
        self._moveit_state_reason = reason or state_enum.value
        if state_enum == MoveItState.OFF:
            set_led(self.led_moveit, "off")
        elif state_enum == MoveItState.STARTING:
            set_led(self.led_moveit, "warn")
        elif state_enum == MoveItState.WAITING_MOVEIT_READY:
            set_led(self.led_moveit, "warn")
        elif state_enum == MoveItState.READY:
            set_led(self.led_moveit, "on")
        else:
            set_led(self.led_moveit, "error")
            if reason:
                self._set_status(f"MoveIt error: {reason}", error=True)
        if reason and self._debug_logs_enabled:
            self._emit_log(f"[MOVEIT] {state_enum.value}: {reason}")
        self._update_moveit_status_label()
        self._refresh_controls()

    @pyqtSlot()
    def _on_trace_ready(self) -> None:
        if self._trace_ready:
            return
        self._trace_ready = True
        self._bridge_ready = True
        self._stop_tf_ready_timer()
        self._log("[TRACE] TF ready → TRACE enabled")
        self._refresh_trace_data()

    @pyqtSlot()
    def _on_calibration_check(self) -> None:
        if self._calibration_ready:
            return
        if not self._objects_settled and not ALLOW_UNSETTLED_ON_TIMEOUT:
            self._log_calib_blocked("esperando caída/estabilidad de objetos")
            return
        if not self._pose_info_ok:
            self._log_calib_blocked("pose/info no disponible")
            return
        if not self._tf_ready_state:
            self._log_calib_blocked(self._tf_not_ready_reason())
            return
        if self._camera_required and not self._camera_stream_ok:
            self._log_calib_blocked("cámara no publica")
            return
        self._log("[CALIB] Inicializando calibración tras bridge")
        self._load_table_calibration()
        self._refresh_objects_from_gz_async()
        QTimer.singleShot(1500, self._refresh_objects_from_gz_async)
        QTimer.singleShot(4000, self._refresh_objects_from_gz_async)
        self._calibration_ready = True
        self.signal_calib_ready.emit(True)

    def _set_system_state(self, state: SystemState, reason: str) -> None:
        if self._system_state == state and self._system_state_reason == reason:
            return
        self._system_state = state
        self._system_state_reason = reason
        self._emit_log(f"[STATE] {state.value} ({reason})")
        if self._metrics_enabled:
            self._metric_mark(f"state:{state.value}")

    def _effective_system_state(self) -> Tuple[SystemState, str]:
        if self._managed_mode:
            if self._external_state_active():
                return self._system_state, self._system_state_reason
            return SystemState.WAITING_GAZEBO, "Esperando /system_state"
        return self._resolve_system_state()

    def _trigger_fatal(self, reason: str) -> None:
        if self._fatal_latched:
            return
        self._fatal_latched = True
        self._system_error_reason = reason
        self._set_system_state(SystemState.ERROR_FATAL, reason)
        self._ui_set_status(f"ERROR_FATAL: {reason}", error=True)
        self._emit_log(f"[ERROR_FATAL] {reason}")
        self._emit_log("[ERROR_FATAL] Panel mantiene UI abierta para diagnostico")
        if self._metrics_enabled:
            self._metric_mark("fatal")
        if self._diagnostic_mode or not self._fatal_stops_all:
            self._emit_log("[ERROR_FATAL] Modo diagnostico activo: no se ejecuta stop_all")
            return
        if not self._managed_mode and not self._fatal_shutdown_started:
            self._fatal_shutdown_started = True
            self._emit_log("[ERROR_FATAL] Abortando stack local (stop_all)")
            abort_local_stack(self)

    def _resolve_system_state(self) -> Tuple[SystemState, str]:
        if self._system_error_reason:
            if self._system_error_reason.startswith("drop:"):
                snapshot = self._build_state_snapshot()
                recovered_state, recovered_reason = self._state_evaluator.resolve(
                    snapshot,
                    tf_reason=self._tf_not_ready_reason(),
                )
                if recovered_state in (
                    SystemState.READY_BASIC,
                    SystemState.READY_VISION,
                    SystemState.READY_MOVEIT,
                ):
                    self._emit_log(
                        f"[STATE] Recover from transient drop -> {recovered_state.value} ({recovered_reason})"
                    )
                    self._system_error_reason = ""
                else:
                    return SystemState.ERROR, self._system_error_reason
            else:
                return SystemState.ERROR, self._system_error_reason
        if self._camera_fault_active:
            return SystemState.ERROR, self._camera_fault_reason or "camera_fault persistente"
        snapshot = self._build_state_snapshot()
        return self._state_evaluator.resolve(snapshot, tf_reason=self._tf_not_ready_reason())

    def _build_state_snapshot(self) -> PanelStateSnapshot:
        return PanelStateSnapshot(
            gazebo_state=self._gazebo_state(),
            controllers_ok=self._controllers_ok,
            controllers_reason=self._controllers_reason or "",
            tf_ready=self._tf_ready_state,
            bridge_running=self._bridge_running,
            camera_required=self._camera_required,
            camera_stream_ok=self._camera_stream_ok,
            pose_info_ok=self._pose_info_ok,
            calibration_ready=self._calibration_ready,
            objects_settled=self._objects_settled,
            moveit_required=self._moveit_required,
            moveit_state=self._moveit_state,
            moveit_state_reason=self._moveit_state_reason or "",
        )

    def _evaluate_system_state(self) -> None:
        next_state, next_reason = self._resolve_system_state()
        prev_state = self._system_state
        if prev_state in (SystemState.READY_VISION, SystemState.READY_MOVEIT) and next_state != prev_state:
            if next_state in (SystemState.READY_VISION, SystemState.READY_MOVEIT):
                self._set_system_state(next_state, next_reason)
                return
            if next_state == SystemState.READY_BASIC:
                self._set_system_state(next_state, next_reason)
                return
            if next_state == SystemState.WAITING_CONTROLLERS and not self._controllers_ok and self._controller_drop_grace_active():
                self._emit_log(f"[WARN] drop a {next_state.value} suprimido: controladores en gracia ({next_reason})")
                self._set_system_state(next_state, next_reason)
                return
            self._system_error_reason = f"drop: {next_reason}"
            self._set_system_state(SystemState.ERROR, self._system_error_reason)
        else:
            self._set_system_state(next_state, next_reason)

    def _update_system_state(self) -> None:
        if self._managed_mode:
            decision = self._state_machine.decide_external(self)
            if decision.fatal_reason:
                self._trigger_fatal(decision.fatal_reason)
                return
            self._set_system_state(decision.state, decision.reason)
            self._sync_moveit_from_system_state()
            return
        self._evaluate_system_state()

    def _check_critical_timeouts(self) -> None:
        reason = self._watchdog.check()
        if reason:
            self._trigger_fatal(reason)
            return

    def _resolve_critical_fault(self, now: float, clock_ok: bool) -> str:
        # Deprecated: moved to PanelWatchdog. Kept for compatibility.
        if self._watchdog is None:
            return ""
        reason = self._watchdog._resolve_critical_fault(now, clock_ok)  # type: ignore[attr-defined]
        return reason or ""

    def _state_ready_basic(self) -> bool:
        return self._state_ready_level("basic")

    def _state_ready_vision(self) -> bool:
        return self._state_ready_level("vision")

    def _state_ready_moveit(self) -> bool:
        return self._state_ready_level("moveit")

    def _state_ready_level(self, level: str) -> bool:
        state, _reason = self._effective_system_state()
        return PanelStateEvaluator.is_ready_level(state, level)

    def _manual_control_ready(self) -> bool:
        if self._managed_mode:
            return self._state_ready_basic()
        return self._gazebo_state() == "GAZEBO_READY" and self._controllers_ok

    def _calibration_topic_allowed(self) -> bool:
        topic = self.camera_topic
        return topic == CALIBRATION_CAMERA_TOPIC

    def _overhead_camera_active(self, topic: str) -> bool:
        return topic == CALIBRATION_CAMERA_TOPIC

    def _moveit_not_ready_reason(self) -> str:
        return moveit_not_ready_reason(self)

    def _set_moveit_wait_status(self, label: str, reason: Optional[str] = None) -> str:
        return set_moveit_wait_status(self, label, reason)

    def _controllers_not_ready_reason(self) -> str:
        return controllers_not_ready_reason(self)

    @staticmethod
    def _ros_node_not_ready_reason() -> str:
        return ros_node_not_ready_reason()

    @staticmethod
    def _controller_manager_not_ready_reason() -> str:
        return controller_manager_not_ready_reason()

    @staticmethod
    def _list_controllers_not_ready_reason(kind: str) -> str:
        return list_controllers_not_ready_reason(kind)

    def _camera_not_ready_reason(self) -> str:
        now = time.time()
        _ready, fault, source_down, age, in_grace = self._camera_runtime_flags(now)
        depth_required, depth_topic = self._camera_depth_expectation()
        depth_age = now - self._last_camera_depth_frame_ts if self._last_camera_depth_frame_ts else float("inf")
        rgb_topic = self.camera_topic_combo.currentText().strip() if hasattr(self, "camera_topic_combo") else self.camera_topic
        rgb_pub_count = self.ros_worker.topic_publisher_count(rgb_topic) if self._ros_worker_started and self.ros_worker.node_ready() and rgb_topic else 0
        depth_pub_count = (
            self.ros_worker.topic_publisher_count(depth_topic)
            if depth_required and self._ros_worker_started and self.ros_worker.node_ready() and depth_topic
            else 0
        )
        if source_down:
            return f"camera_source_down ({self._gazebo_state()})"
        if self._camera_frame_count <= 0 and in_grace:
            return "camera_warmup"
        if self._camera_frame_count <= 0:
            return (
                "camera_no_frames "
                f"topic={rgb_topic or 'n/a'} pubs={rgb_pub_count} "
                f"last_age={'inf' if math.isinf(age) else f'{age:.1f}s'}"
            )
        if depth_required and self._camera_depth_frame_count <= 0:
            return (
                "camera_depth_warmup "
                f"topic={depth_topic or 'n/a'} pubs={depth_pub_count} "
                f"last_age={'inf' if math.isinf(depth_age) else f'{depth_age:.1f}s'}"
            )
        if depth_required and depth_age >= CAMERA_READY_MAX_AGE_SEC:
            return (
                "camera_depth_stale "
                f"topic={depth_topic or 'n/a'} age={'inf' if math.isinf(depth_age) else f'{depth_age:.1f}s'}"
            )
        if fault:
            return f"camera_fault age={age:.1f}s"
        return camera_not_ready_reason(None)

    @staticmethod
    def _pose_info_not_ready_reason() -> str:
        return pose_info_not_ready_reason(None)

    def _tf_not_ready_reason(self) -> str:
        return tf_not_ready_reason(self)

    def _moveit_control_status(self) -> Tuple[bool, str]:
        return moveit_control_status(self)

    def _manual_control_status(self) -> Tuple[bool, str]:
        return manual_control_status(self)

    def _external_publishers_for_topic(self, topic: str) -> List[str]:
        if not topic or not self._ros_worker_started or not self.ros_worker.node_ready():
            return []
        pubs = self.ros_worker.publisher_nodes_by_topic(topic)
        if not pubs:
            return []
        self_name = self.ros_worker.node_name()
        self_ns = self.ros_worker.node_namespace()
        self_full = f"{self_ns}/{self_name}".replace("//", "/") if self_name else ""
        bridge_name = "ur5_moveit_bridge"
        bridge_ns = self.ros_worker.node_namespace()
        bridge_full = f"{bridge_ns}/{bridge_name}".replace("//", "/") if bridge_name else ""
        bridge_active = self._proc_alive(self.moveit_bridge_proc) or self._moveit_bridge_detected()
        moveit_name = ""
        moveit_ns = ""
        moveit_full = ""
        if self._moveit_node is not None:
            try:
                moveit_name = self._moveit_node.get_name()
                moveit_ns = self._moveit_node.get_namespace()
                moveit_full = f"{moveit_ns}/{moveit_name}".replace("//", "/") if moveit_name else ""
            except Exception:
                moveit_name = ""
                moveit_ns = ""
                moveit_full = ""
        externals: List[str] = []
        for pub in pubs:
            if not pub:
                continue
            # Some DDS graph entries expose anonymous placeholders
            # ("_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_").
            # Treat them as introspection noise and do not block TEST ROBOT.
            if "_NODE_NAME_UNKNOWN_" in pub or "_NODE_NAMESPACE_UNKNOWN_" in pub:
                self._emit_log_throttled(
                    f"TEST:ignore_unknown_pub:{topic}",
                    f"[SAFETY] TEST ROBOT: ignorando publisher desconocido en {topic}: {pub}",
                )
                continue
            if self_name and (pub == self_name or pub.endswith(f"/{self_name}")):
                continue
            if self_full and pub == self_full:
                continue
            if not bridge_active:
                if bridge_name and (pub == bridge_name or pub.endswith(f"/{bridge_name}")):
                    continue
                if bridge_full and pub == bridge_full:
                    continue
            if moveit_name and (pub == moveit_name or pub.endswith(f"/{moveit_name}")):
                continue
            if moveit_full and pub == moveit_full:
                continue
            externals.append(pub)
        return sorted(set(externals))

    def _bridge_publishers_only(self, externals: List[str]) -> bool:
        if not externals:
            return False
        names = {"ur5_moveit_bridge", "/ur5_moveit_bridge"}
        ns = self.ros_worker.node_namespace()
        if ns:
            names.add(f"{ns}/ur5_moveit_bridge".replace("//", "/"))
        return all(pub in names for pub in externals)

    def _set_robot_test_blocked(self, reason: Optional[str]) -> None:
        if reason == self._robot_test_block_reason:
            return
        self._robot_test_block_reason = reason
        if not reason:
            self._robot_test_cleanup_pending = False
            self._robot_test_cleanup_retries = 0
        if reason:
            self._set_status(f"TEST ROBOT bloqueado: {reason}", error=True)

    def _await_external_publishers_clear(
        self,
        topic: str,
        deadline_ts: float,
        on_clear,
        on_timeout,
    ) -> None:
        if self._closing:
            return
        if not topic:
            on_timeout()
            return
        externals = self._external_publishers_for_topic(topic)
        if externals and self._bridge_publishers_only(externals):
            on_clear()
            return
        if not externals:
            on_clear()
            return
        if time.time() >= deadline_ts:
            on_timeout()
            return
        QTimer.singleShot(100, lambda: self._await_external_publishers_clear(topic, deadline_ts, on_clear, on_timeout))

    def _schedule_robot_test_cleanup_check(
        self, topic: str, *, delay_ms: int = 500, max_retries: int = 6
    ) -> None:
        if not topic:
            return
        self._robot_test_cleanup_topic = topic
        if self._robot_test_cleanup_pending:
            return
        self._robot_test_cleanup_pending = True
        self._robot_test_cleanup_retries = max_retries

        def _check():
            if not self._robot_test_cleanup_pending:
                return
            if self._robot_test_cleanup_retries <= 0:
                self._robot_test_cleanup_pending = False
                return
            self._robot_test_cleanup_retries -= 1
            externals = self._external_publishers_for_topic(self._robot_test_cleanup_topic)
            if not externals:
                self._robot_test_cleanup_pending = False
                self._set_robot_test_blocked(None)
                self._set_status("TEST ROBOT desbloqueado: limpieza OK", error=False)
                return
            self._set_status("TEST ROBOT bloqueado: esperando limpieza de publishers...", error=False)
            QTimer.singleShot(delay_ms, _check)

        QTimer.singleShot(delay_ms, _check)

    @pyqtSlot(object)
    def _update_camera_topics_async(self, topics: object) -> None:
        self._camera_ctrl.update_topics_async(topics)

    def _emit_log(self, msg: str, *, flush: bool = True):
        """Print a timestamped log line."""
        if not self._should_emit_log(msg):
            return
        print(timestamped_line(msg), flush=flush)

    def _metric_mark(self, label: str) -> None:
        if not self._metrics_enabled:
            return
        if label in self._perf_marks:
            return
        elapsed = time.monotonic() - self._perf_start_monotonic
        self._perf_marks[label] = elapsed
        self._emit_log(f"[METRICS] {label}={elapsed:.2f}s")

    def _audit_root(self) -> Path:
        return Path(WS_DIR).parent / "reports" / "panel_audit"

    def _audit_append(self, rel_path: str, msg: str) -> None:
        try:
            out_path = self._audit_root() / rel_path
            ensure_dir(str(out_path.parent))
            with out_path.open("a", encoding="utf-8") as f:
                f.write(f"{timestamped_line(msg)}\n")
        except Exception:
            pass

    def _audit_write_json(self, rel_path: str, payload: Dict[str, object]) -> None:
        try:
            out_path = self._audit_root() / rel_path
            ensure_dir(str(out_path.parent))
            out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except Exception:
            pass

    def _sha256_file(self, path: str) -> str:
        try:
            import hashlib
        except Exception:
            return ""
        h = hashlib.sha256()
        try:
            with open(path, "rb") as f:
                for chunk in iter(lambda: f.read(1024 * 1024), b""):
                    h.update(chunk)
        except Exception:
            return ""
        return h.hexdigest()

    def _should_emit_log(self, msg: str) -> bool:
        if getattr(self, "_debug_logs_enabled", False):
            return True
        if msg.startswith("[STARTUP]"):
            return True
        if msg.startswith("[TFM]"):
            return True
        if msg.startswith("[BTN]"):
            return True
        if msg.startswith("[ERROR]"):
            return True
        if msg.startswith("[PICK]"):
            return True
        if msg.startswith("[PICK_OBJ]"):
            return True
        if msg.startswith("[PICK_OBJ_DEBUG]"):
            return True
        return False

    def _set_motion_lock(self, active: bool):
        """Habilitar/deshabilitar control manual mientras corre un flujo predefinido."""
        self._script_motion_active = active
        QTimer.singleShot(0, self._refresh_controls)

    def _set_btn_state(self, btn: QPushButton, enabled: bool, tooltip: str = "") -> None:
        btn.setEnabled(enabled)
        if enabled:
            if tooltip:
                btn.setToolTip(tooltip)
            return
        if tooltip:
            btn.setToolTip(tooltip)

    def _set_launching_style(self, btn: QPushButton, active: bool) -> None:
        if active:
            btn.setStyleSheet("background:#f59e0b; color:#0f172a; font-weight:600;")
        else:
            btn.setStyleSheet("")

    def _clear_launching_if_timeout(self, label: str, start_ts: float, timeout_sec: float) -> bool:
        if not start_ts:
            return False
        if (time.time() - start_ts) < timeout_sec:
            return False
        self._emit_log(f"[WARN] {label} timeout; re-habilitando botón")
        return True

    def _controller_drop_grace_active(self) -> bool:
        if self._controller_spawn_inflight:
            return True
        if not self._controller_spawn_last_start:
            return False
        return (time.time() - self._controller_spawn_last_start) < CONTROLLER_DROP_GRACE_SEC

    def _require_ready_basic(self, action: str) -> bool:
        return self._safety.require_ready_basic(action)

    def _basic_ready_status(self) -> Tuple[bool, str]:
        if self._state_ready_basic():
            return True, ""
        reason = self._system_state_reason or self._system_state.value
        return False, reason

    def _require_ready_vision(self, action: str) -> bool:
        return self._safety.require_ready_vision(action)

    def _require_manual_ready(self, action: str) -> bool:
        return self._safety.require_manual_ready(action)

    def _log(self, msg: str):
        if self._debug_logs_enabled:
            self._emit_log(msg)

    def _emit_log_throttled(self, key: str, msg: str, min_interval: float = 1.0) -> None:
        self._safety.emit_log_throttled(key, msg, min_interval=min_interval)

    def _block_if_managed(self, action: str) -> bool:
        if not self._managed_mode:
            return False
        self._emit_log(f"[WARN] {action} bloqueado: PANEL_MANAGED=1")
        return True
    
    def _log_error(self, msg: str):
        """SIEMPRE loguear errores, incluso si debug no está activo."""
        self._emit_log(f"[ERROR] {msg}")
    
    def _log_warning(self, msg: str):
        """Loguear warnings cuando debug está activo."""
        if self._debug_logs_enabled:
            self._emit_log(f"[WARN] {msg}")

    def _on_async_error(self, msg: str) -> None:
        self._log_warning(f"[ASYNC] {msg}")

    @pyqtSlot(object)
    def _run_ui_callable(self, fn) -> None:
        try:
            fn()
        except Exception as exc:
            self._log_warning(f"[UI] {exc}")

    @pyqtSlot(object, int)
    def _run_ui_delayed(self, fn, delay_ms: int) -> None:
        QTimer.singleShot(int(delay_ms), fn)

    def _run_async(self, fn, *, name: str = "", on_done=None) -> QThread:
        thread = _FnThread(fn, name=name)
        thread.error.connect(self._on_async_error)
        if on_done:
            thread.finished.connect(on_done)

        def _cleanup() -> None:
            try:
                self._async_threads.remove(thread)
            except ValueError:
                pass

        thread.finished.connect(_cleanup)
        thread.finished.connect(thread.deleteLater)
        self._async_threads.append(thread)
        thread.start()
        return thread

    def _log_ros_message(self, msg: str):
        """Mostrar siempre los mensajes provenientes del RosWorker, pero solo cuando el bridge esté activo."""
        if not self._bridge_running:
            return
        self._emit_log(msg)

    @pyqtSlot(str, str)
    def _on_system_state_update(self, state: str, reason: str) -> None:
        state = (state or "").strip().upper()
        if not state:
            return
        self._external_state = state
        self._external_state_reason = reason or ""
        self._external_state_last = time.time()

    def _external_state_active(self) -> bool:
        return external_state_active(self)

    def _resolve_external_state(self) -> Tuple[Optional[SystemState], str]:
        return resolve_external_state(self)

    def _apply_external_system_state(self) -> None:
        apply_external_system_state(self)

    def _log_camera_diagnostics(self, reason: str):
        """Emitir detalles adicionales para debugging cuando hay fallos de cámara."""
        if not self._camera_required:
            return
        if not self._debug_logs_enabled:
            return
        node_ready = self.ros_worker.node_ready()
        ctrl_ok = self._ros2_control_available()
        clock_ok, clock_age = self._clock_status()
        bridge_ok = self._bridge_running
        last_age = "n/a"
        if self._last_camera_frame_ts:
            last_age = f"{time.time() - self._last_camera_frame_ts:.1f}s"
        topics = []
        if node_ready:
            try:
                topics = self.ros_worker.list_topic_names()
            except Exception as exc:
                self._log(f"[CAMERA-DIAG] fallo listando topics: {exc}")
        camera_topics = [t for t in topics if t.startswith(CAMERA_TOPIC_PREFIX)]
        diag = (
            f"[CAMERA-DIAG] {reason} node_ready={node_ready} ros2_ctrl={ctrl_ok} "
            f"clock={clock_ok}:{clock_age} bridge={bridge_ok} "
            f"last_frame_age={last_age} camera_topics={len(camera_topics)}/{len(topics)}"
        )
        self._log(diag)
        if camera_topics:
            preview = ", ".join(camera_topics[:4])
            suffix = "..." if len(camera_topics) > 4 else ""
            self._log(f"[CAMERA-DIAG] camera topics sample: {preview}{suffix}")

    def _sync_moveit_from_system_state(self) -> None:
        if not self._moveit_required:
            self._moveit_state = MoveItState.OFF
            self._moveit_state_reason = "manual"
            return
        if not self._external_state_active():
            return
        if self._system_state == SystemState.READY_MOVEIT:
            if self._moveit_state != MoveItState.READY:
                self._moveit_state = MoveItState.READY
                self._moveit_state_reason = "move_group listo (externo)"
            return
        if self._system_state == SystemState.READY_VISION:
            self._moveit_state = MoveItState.WAITING_MOVEIT_READY
            self._moveit_state_reason = self._system_state_reason or "move_group no listo"

    def _clock_status(self) -> Tuple[bool, str]:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False, "node_off"
        ok, age = self.ros_worker.clock_alive()
        if ok:
            return True, f"age={age:.2f}s"
        return False, f"age={age:.2f}s"

    def _joint_states_status(self) -> Tuple[bool, str]:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False, "node_off"
        topic = (self._joint_current_topic or self.joint_topic or "/joint_states").strip() or "/joint_states"
        has_pub = self.ros_worker.topic_has_publishers(topic)
        if not has_pub:
            return False, f"{topic}:no_publishers"
        payload, ts = self.ros_worker.get_last_joint_state()
        if payload is None:
            return False, f"{topic}:no_msgs"
        names = payload.get("name", []) or []
        if len(names) == 0:
            return False, f"{topic}:empty"
        age = float("inf")
        if ts:
            age = max(0.0, time.time() - ts)
        if age > 2.0:
            return False, f"{topic}:stale age={age:.2f}s"
        return True, f"topic={topic} age={age:.2f}s names={len(names)}"

    def _bridge_ready_status(self) -> Tuple[bool, str]:
        if not self._bridge_running:
            return False, "bridge_process_off"
        clock_ok, clock_reason = self._clock_status()
        if not clock_ok:
            return False, f"clock_not_ready ({clock_reason})"
        if not self._pose_info_active():
            return False, "pose_info_no_publishers"
        js_ok, js_reason = self._joint_states_status()
        if not js_ok:
            return False, js_reason
        helper = get_tf_helper()
        if helper is None:
            return False, "tf_helper_off"
        tf_stats = helper.tf_listener_stats()
        if tf_stats[0] <= 0:
            return False, f"tf_msgs={tf_stats[0]}"
        return True, f"clock+pose+joint_states+tf ok (tf_msgs={tf_stats[0]})"

    def _tf_chain_ready_status(self) -> Tuple[bool, str]:
        helper = get_tf_helper()
        if helper is None:
            return False, "tf_helper_off"
        base_frame = self._business_base_frame()
        ee_frame = str(getattr(self, "_required_ee_frame", "") or "rg2_tcp").strip() or "rg2_tcp"
        if not _can_transform_between(helper, base_frame, ee_frame, timeout_sec=0.2):
            return False, f"{base_frame}<->{ee_frame} missing"
        return True, f"{base_frame}->{ee_frame}"

    def _camera_depth_expectation(self) -> Tuple[bool, str]:
        topic = str(self.camera_topic or "").strip()
        if topic.endswith("/depth_image"):
            return False, topic
        if topic.endswith("/image"):
            depth_topic = topic[: -len("/image")] + "/depth_image"
        elif topic.endswith("/rgb"):
            depth_topic = topic[: -len("/rgb")] + "/depth_image"
        else:
            depth_topic = "/camera_overhead/depth_image"
        depth_required = bool(self._camera_depth_required_env)
        if hasattr(self, "chk_tfm_use_depth") and self.chk_tfm_use_depth is not None:
            try:
                depth_required = depth_required or bool(self.chk_tfm_use_depth.isChecked())
            except Exception:
                pass
        return depth_required, depth_topic

    def _camera_runtime_flags(self, now: Optional[float] = None) -> Tuple[bool, bool, bool, float, bool]:
        """Return camera_ready, camera_fault, camera_source_down, frame_age, warmup_grace."""
        if now is None:
            now = time.time()
        age = now - self._last_camera_frame_ts if self._last_camera_frame_ts else float("inf")
        gz_state = self._gazebo_state()
        gazebo_ready = gz_state == "GAZEBO_READY"
        bridge_ready = bool(self._bridge_running)
        source_down = (not gazebo_ready) or (not bridge_ready)
        has_frames = self._camera_frame_count > 0
        depth_required, _depth_topic = self._camera_depth_expectation()
        depth_age = (
            now - self._last_camera_depth_frame_ts
            if self._last_camera_depth_frame_ts
            else float("inf")
        )
        depth_ready = (
            (not depth_required)
            or (
                self._camera_depth_frame_count > 0
                and depth_age < CAMERA_READY_MAX_AGE_SEC
                and gazebo_ready
                and bridge_ready
            )
        )
        grace_anchor = 0.0
        if self._camera_subscribe_ts > 0.0:
            grace_anchor = self._camera_subscribe_ts
        elif self._camera_init_start > 0.0:
            grace_anchor = self._camera_init_start
        elif self._bridge_start_ts > 0.0:
            grace_anchor = self._bridge_start_ts
        in_grace = grace_anchor > 0.0 and (now - grace_anchor) < self._camera_warmup_grace_sec
        camera_ready = has_frames and age < CAMERA_READY_MAX_AGE_SEC and depth_ready and gazebo_ready and bridge_ready
        camera_fault = has_frames and age > self._camera_fault_age_sec and gazebo_ready and bridge_ready and (not in_grace)
        if depth_required:
            camera_fault = camera_fault or (
                self._camera_depth_frame_count > 0
                and depth_age > self._camera_fault_age_sec
                and gazebo_ready
                and bridge_ready
                and (not in_grace)
            )
        return camera_ready, camera_fault, source_down, age, in_grace

    def _pose_info_topic(self) -> str:
        world_name = self._gz_world_name or self._detect_world_name() or GZ_WORLD
        return f"/world/{world_name}/pose/info"

    def _pose_info_active(self) -> bool:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False
        now = time.monotonic()
        if (
            STATUS_TOPIC_CACHE_SEC > 0.0
            and (now - self._pose_info_active_ts) < STATUS_TOPIC_CACHE_SEC
        ):
            return self._pose_info_active_cache
        active = self.ros_worker.topic_has_publishers(self._pose_info_topic())
        self._pose_info_active_cache = bool(active)
        self._pose_info_active_ts = now
        return active

    def _pose_info_ready(self) -> bool:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False
        if not self._pose_info_active():
            return False
        poses, ts = self.ros_worker.pose_snapshot()
        if not poses:
            return False
        if ts:
            age = time.time() - ts
            if age > POSE_INFO_MAX_AGE_SEC:
                return False
        return True

    def _gazebo_process_signal(self) -> Tuple[bool, str]:
        """Signal S1: determine if Gazebo process is alive using PID/PGID + fallback scan."""
        if self._proc_alive(self.gz_proc):
            pid = int(getattr(self.gz_proc, "pid", 0) or 0)
            if pid > 0:
                self._gz_real_pid = pid
                if self._gz_root_pid <= 0:
                    self._gz_root_pid = pid
                if self._gz_pgid <= 0:
                    try:
                        self._gz_pgid = int(os.getpgid(pid))
                    except Exception:
                        self._gz_pgid = 0
            return True, "popen"
        if self._gz_pgid > 0 and psutil is not None:
            try:
                for proc in psutil.process_iter(attrs=["pid", "cmdline", "status"]):
                    info = proc.info
                    pid = int(info.get("pid") or 0)
                    if pid <= 0:
                        continue
                    if info.get("status") == psutil.STATUS_ZOMBIE:
                        continue
                    try:
                        if int(os.getpgid(pid)) != self._gz_pgid:
                            continue
                    except Exception:
                        continue
                    cmdline = info.get("cmdline") or []
                    if not cmdline:
                        continue
                    joined = " ".join(cmdline).lower()
                    if any(token in joined for token in ("gz sim", "gz-sim", "gzserver", "ign gazebo")):
                        self._gz_real_pid = pid
                        return True, "pgid"
            except Exception as exc:
                _log_exception("gazebo process signal pgid", exc)
        proc_ok, proc_reason = gz_sim_status()
        if proc_ok:
            return True, f"fallback_{proc_reason}"
        return False, proc_reason

    def _gazebo_bridge_signal(self) -> Tuple[bool, str]:
        """Signal S3: at least one critical bridged topic has publishers."""
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False, "node_off"
        pose_topic = self._pose_info_topic()
        if self.ros_worker.topic_has_publishers(pose_topic):
            return True, pose_topic
        camera_topic = ""
        try:
            camera_topic = self.camera_topic_combo.currentText().strip() or self.camera_topic
        except Exception:
            camera_topic = self.camera_topic
        if camera_topic and self.ros_worker.topic_has_publishers(camera_topic):
            return True, camera_topic
        return False, "no_critical_topic"

    def _gazebo_state(self) -> str:
        """GAZEBO_OFF / GAZEBO_STARTING / GAZEBO_READY / GAZEBO_DEGRADED / GAZEBO_MONITOR_BUG."""
        proc_ok, proc_reason = self._gazebo_process_signal()
        clock_ok, clock_reason = self._clock_status()
        topic_ok, topic_reason = self._gazebo_bridge_signal()
        now = time.monotonic()
        if clock_ok:
            self._last_clock_ok_ts = now
            self._gz_clock_stall_since = 0.0
        freeze_sec = max(0.5, self._gz_health_freeze_sec)
        if proc_ok and (not clock_ok) and self._gz_clock_stall_since <= 0.0:
            self._gz_clock_stall_since = now
        if proc_ok and clock_ok:
            self._gz_orphan_since = 0.0
        topics_required = bool(self._bridge_running or self._started_bridge)
        if proc_ok and clock_ok:
            if topics_required and not topic_ok:
                candidate = "GAZEBO_DEGRADED"
            else:
                candidate = "GAZEBO_READY"
        elif proc_ok and (not clock_ok):
            stalled_for = (now - self._gz_clock_stall_since) if self._gz_clock_stall_since > 0.0 else 0.0
            if stalled_for > freeze_sec:
                candidate = "GAZEBO_DEGRADED"
            else:
                candidate = "GAZEBO_STARTING"
        elif (not proc_ok) and clock_ok:
            if self._gz_orphan_since <= 0.0:
                self._gz_orphan_since = now
            candidate = "GAZEBO_MONITOR_BUG"
        else:
            stale_clock = (now - self._last_clock_ok_ts) if self._last_clock_ok_ts > 0.0 else float("inf")
            if self._gz_state in ("GAZEBO_READY", "GAZEBO_DEGRADED") and stale_clock <= freeze_sec:
                candidate = "GAZEBO_DEGRADED"
            else:
                candidate = "GAZEBO_OFF"
            self._gz_orphan_since = 0.0
        if candidate != self._gz_state:
            if candidate != self._gz_state_pending:
                self._gz_state_pending = candidate
                self._gz_state_change_ts = now
            elif (now - self._gz_state_change_ts) >= 1.0:
                self._gz_state = candidate
                self._gz_state_pending = ""
                self._emit_log(
                    f"[STATE] Gazebo={self._gz_state} reason=s1:{proc_reason}|s2:{clock_reason}|s3:{topic_reason} "
                    f"process={str(proc_ok).lower()} clock={str(clock_ok).lower()} "
                    f"topics={str(topic_ok).lower()} pid={self._gz_real_pid or self._gz_root_pid or 0}"
                )
                if self._gz_state == "GAZEBO_MONITOR_BUG":
                    self._emit_log_throttled(
                        "GZ_MONITOR_BUG",
                        "[STATE] Gazebo monitor inconsistente: process=false pero /clock avanza; revisando PID/PGID",
                        min_interval=3.0,
                    )
        else:
            self._gz_state_pending = ""
        return self._gz_state

    def _ros2_control_available(self) -> bool:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False
        return bool(self._controller_manager_path())

    def _controller_manager_path(self) -> str:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return ""
        try:
            return resolve_controller_manager(self._moveit_node)
        except Exception as exc:
            _log_exception("resolve controller_manager", exc)
            return "/controller_manager"

    def _select_traj_topic(self) -> str:
        topics = set(self._list_topic_names())
        if "/joint_trajectory_controller/joint_trajectory" in topics:
            return "/joint_trajectory_controller/joint_trajectory"
        candidates = sorted(t for t in topics if t.endswith("/joint_trajectory_controller/joint_trajectory"))
        if candidates:
            return candidates[0]
        return ""

    def _ensure_pose_subscription(self) -> None:
        if not self._bridge_running or self._closing:
            return
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if not self.ros_worker.node_ready():
            return
        world_name = self._gz_world_name or read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
        topic = self._discover_pose_info_topic(world_name)
        self.ros_worker.subscribe_pose_info(topic)

    def _discover_pose_info_topic(self, world_name: str) -> str:
        """Return pose/info topic, preferring *world_name* when available."""
        expected = f"/world/{world_name}/pose/info"
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return expected
        topics = self._list_topic_names()
        if not topics:
            return expected
        candidates = [t for t in topics if t.startswith("/world/") and t.endswith("/pose/info")]
        if not candidates:
            return expected
        if expected in candidates:
            return expected
        return sorted(candidates)[0]

    def _start_pose_info_watch(self) -> None:
        if self._pose_info_timer is None:
            self._pose_info_timer = QTimer(self)
            self._pose_info_timer.setInterval(int(POSE_INFO_POLL_SEC * 1000))
            self._pose_info_timer.timeout.connect(self._update_pose_info_status)
        if not self._pose_info_timer.isActive():
            self._pose_info_timer.start()
        self._update_pose_info_status()

    def _update_pose_info_status(self) -> None:
        if self._closing or not self._bridge_running:
            return
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return
        count, age, entities, topic = self.ros_worker.pose_info_details()
        self._pose_info_msg_count = count
        self._pose_info_last_age = age
        ready = count > 0 and entities > 0 and age < POSE_INFO_MAX_AGE_SEC
        now = time.monotonic()
        if (now - self._pose_info_last_log) >= POSE_INFO_LOG_PERIOD:
            self._emit_log(
                f"[PHYSICS][POSE_INFO] ready={str(ready).lower()} count={count} age={age:.2f}s "
                f"entities={entities} topic={topic or 'n/a'}"
            )
            self._pose_info_last_log = now
        if ready and not self._pose_info_ok:
            self._pose_info_ok = True
            self._pose_info_ever_ok = True
            self._pose_info_diag_logged = False
            if self._metrics_enabled and self._pose_info_msg_count > 0:
                self._metric_mark("pose_info_ready")
            if not self._critical_tf_deadline:
                self._critical_tf_deadline = time.monotonic() + max(0.1, TF_INIT_GRACE_SEC)
            if self._gz_running:
                self.signal_start_objects_settle_watch.emit()
                QTimer.singleShot(0, self._schedule_physics_runtime_check)
        if ready and self._gz_running and not self._objects_release_done:
            self._maybe_hold_drop_objects("pose_info")
        elif not ready:
            self._pose_info_ok = False
            if (now - self._pose_info_last_log) >= POSE_INFO_LOG_PERIOD:
                self._emit_log("[PHYSICS][SETTLE] waiting pose/info data...")
                self._pose_info_last_log = now
            if (now - self._pose_info_resub_ts) >= POSE_INFO_LOG_PERIOD:
                self._pose_info_resub_ts = now
                self._ensure_pose_subscription()
            if not self._pose_info_diag_logged and count == 0:
                self._pose_info_diag_logged = True
                try:
                    topics = self.ros_worker.list_topic_names()
                except Exception as exc:
                    _log_exception("pose_info list topics", exc)
                    topics = []
                pose_topics = [t for t in topics if t.startswith("/world/") and t.endswith("/pose/info")]
                sample_topics = ", ".join(pose_topics[:5]) if pose_topics else "-"
                self._emit_log(f"[PHYSICS][POSE_INFO] available_pose_topics={sample_topics}")
                helper = get_tf_helper()
                frames = helper.list_frames() if helper else set()
                frames_sample = ", ".join(sorted(frames)[:10]) if frames else "-"
                self._emit_log(f"[TF] frames_sample={frames_sample}")

    def _log_button(self, label: str):
        self._emit_log(f"[BTN] {label}")
    
    def _cleanup_stray_processes(self):
        """Limpiar procesos fantasma de Gazebo, bridge y rosbag al startup."""
        if PANEL_SKIP_CLEANUP:
            self._emit_log("[STARTUP] Limpieza omitida (PANEL_SKIP_CLEANUP=1)")
            return
        # 1. Limpiar archivos de memoria compartida de FastDDS/FastRTPS
        try:
            self._emit_log("[STARTUP] Limpiando /dev/shm (FastDDS)")
            subprocess.run(
                ["sh", "-c", "rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null || true"],
                timeout=2,
            )
        except Exception as exc:
            _log_exception("cleanup /dev/shm", exc)

        # 2. Limpiar procesos residuales del stack si procede.
        if not PANEL_KILL_STALE:
            self._emit_log("[STARTUP] Limpieza de procesos deshabilitada (PANEL_KILL_STALE=0)")
            return
        if not psutil:
            self._emit_log("[STARTUP] psutil no disponible; no se limpian procesos residuales")
            return
        stale = self._list_stale_processes()
        if not stale:
            self._emit_log("[STARTUP] No hay procesos residuales detectados")
            return
        self._emit_log(f"[STARTUP] Procesos residuales detectados: {len(stale)}. Terminando...")
        procs = []
        for pid, cmd, _status in stale:
            try:
                proc = psutil.Process(pid)
                proc.terminate()
                procs.append(proc)
            except Exception as exc:
                _log_exception(f"terminate stale pid {pid}", exc)
                continue
        try:
            _, alive = psutil.wait_procs(procs, timeout=2.0)
            for proc in alive:
                try:
                    proc.kill()
                except Exception as exc:
                    _log_exception("kill stale process", exc)
                    continue
            if alive:
                psutil.wait_procs(alive, timeout=1.0)
        except Exception as exc:
            _log_exception("wait stale processes", exc)

    def _clean_cache_dirs(self):
        """Limpiar cachés de Python (__pycache__ y .pyc) en el workspace."""
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        ws_parent = os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir)))
        
        removed_count = 0
        try:
            for root, dirs, _ in os.walk(ws_parent):
                if "__pycache__" in dirs:
                    cache_path = os.path.join(root, "__pycache__")
                    try:
                        shutil.rmtree(cache_path)
                        removed_count += 1
                    except Exception as e:
                        self._log_warning(f"No se pudo borrar {cache_path}: {e}")
            # También borrar archivos .pyc
            for root, _, files in os.walk(ws_parent):
                for f in files:
                    if f.endswith(".pyc"):
                        try:
                            os.remove(os.path.join(root, f))
                            removed_count += 1
                        except Exception as e:
                            self._log_warning(f"No se pudo borrar {f}: {e}")
            self._log(f"[STARTUP] ✅ Cache limpiado ({removed_count} items)")
        except Exception as e:
            self._log_error(f"Error limpiando cache: {e}")
    
    def _close_terminal(self):
        """Cerrar la aplicación del panel."""
        if bool(getattr(self, "_script_motion_active", False)):
            allow_close = str(os.environ.get("PANEL_ALLOW_CLOSE_WHILE_MOTION", "0")).strip().lower()
            if allow_close not in ("1", "true", "yes", "on"):
                self._emit_log("[PANEL] Cerrar Terminal bloqueado: movimiento en curso (PICK/TEST)")
                self._set_status("Cierre bloqueado: espera a que termine el movimiento", error=False)
                return
        if self._closing:
            return
        self._closing = True
        try:
            self.btn_close_terminal.setEnabled(False)
        except Exception as exc:
            _log_exception("disable close button", exc)
        self._log_button("Cerrar Terminal")
        self._log("[PANEL] Cerrando panel...")
        # Trigger the standard Qt close flow so closeEvent() runs cleanup.
        QTimer.singleShot(0, self.close)
    def _refresh_camera_topics(self):
        self._camera_ctrl.refresh_topics()

    def _controllers_ready(self) -> Tuple[bool, str]:
        now = time.time()
        gz_state = self._gazebo_state()
        if gz_state != "GAZEBO_READY":
            return False, f"gazebo_not_ready state={gz_state}"
        if (
            CONTROLLER_READY_CACHE_SEC > 0.0
            and self._controllers_ok
            and (now - self._last_controller_check) < CONTROLLER_READY_CACHE_SEC
        ):
            return True, "cached"
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False, "nodo ROS no listo"
        cm_path = self._controller_manager_path()
        if not cm_path:
            return False, self._controller_manager_not_ready_reason()
        list_srv = f"{cm_path}/list_controllers"
        if not self.ros_worker.has_service(list_srv):
            reason = self._list_controllers_not_ready_reason("service")
            stale_ok, age = self._can_use_controller_last_ok(reason)
            if stale_ok:
                self._emit_log_throttled(
                    "ctrl_gate_stale_service",
                    f"[CTRL_GATE] {reason}; using last_ok_age={age:.2f}s",
                    min_interval=1.0,
                )
                return True, f"last_ok_stale age={age:.2f}s ({reason})"
            return False, reason
        resp, err = self._list_controllers(list_srv)
        if resp is None and self._is_transient_controller_reason(err):
            deadline = time.monotonic() + CONTROLLER_LIST_RETRY_WINDOW_SEC
            while resp is None and time.monotonic() < deadline:
                time.sleep(CONTROLLER_LIST_RETRY_STEP_SEC)
                resp, err = self._list_controllers(list_srv)
        if resp is None:
            reason = err or "no response"
            stale_ok, age = self._can_use_controller_last_ok(reason)
            if stale_ok:
                self._emit_log_throttled(
                    "ctrl_gate_stale_timeout",
                    f"[CTRL_GATE] list_controllers timeout; using last_ok_age={age:.2f}s ({reason})",
                    min_interval=1.0,
                )
                return True, f"last_ok_stale age={age:.2f}s ({reason})"
            return False, reason
        required = ["joint_state_broadcaster", "joint_trajectory_controller"]
        if gripper_controller_defined():
            required.append("gripper_controller")
        state_map = {str(c.name): str(c.state) for c in resp.controller}

        def _state_for(required_name: str) -> str:
            target = str(required_name or "").strip().lstrip("/")
            if not target:
                return ""
            for key, value in state_map.items():
                norm_key = str(key or "").strip().lstrip("/")
                if norm_key == target or norm_key.endswith("/" + target):
                    return str(value or "")
            return ""

        missing = [name for name in required if not _state_for(name)]
        if missing:
            known = ", ".join(sorted(state_map.keys())[:8]) if state_map else "none"
            return False, "missing_controllers=[" + ", ".join(missing) + f"] known=[{known}]"
        inactive = []
        loaded = []
        unknown = []
        for name in required:
            state = _state_for(name)
            kind = self._controller_state_kind(state)
            if kind == "ACTIVE":
                continue
            if kind == "INACTIVE":
                inactive.append(name)
            elif kind == "LOADED":
                loaded.append(name)
            else:
                unknown.append(name)
        if inactive or loaded or unknown:
            detail = []
            if inactive:
                detail.append(f"inactive: {', '.join(inactive)}")
            if loaded:
                detail.append(f"loaded: {', '.join(loaded)}")
            if unknown:
                detail.append(f"unknown: {', '.join(unknown)}")
            self._last_controller_check = now
            return False, "controllers not active (" + " | ".join(detail) + ")"
        self._controllers_last_ok_ts = now
        self._last_controller_check = now
        return True, "controllers activos"

    def _is_transient_controller_reason(self, reason: Optional[str]) -> bool:
        text = str(reason or "").lower()
        if not text:
            return False
        return (
            "list_controllers no disponible" in text
            or "timeout" in text
            or "no response" in text
            or "service" in text
            or "client" in text
        )

    def _controllers_last_ok_age(self, now: Optional[float] = None) -> float:
        ts = float(self._controllers_last_ok_ts or 0.0)
        if ts <= 0.0:
            return float("inf")
        ref = time.time() if now is None else float(now)
        return max(0.0, ref - ts)

    def _can_use_controller_last_ok(self, reason: Optional[str]) -> Tuple[bool, float]:
        if not self._is_transient_controller_reason(reason):
            return False, float("inf")
        if CONTROLLER_LAST_OK_GRACE_SEC <= 0.0:
            return False, float("inf")
        age = self._controllers_last_ok_age()
        return age <= CONTROLLER_LAST_OK_GRACE_SEC, age

    def _controller_state_kind(self, state: str) -> str:
        state_lc = (state or "").strip().lower()
        if state_lc == "active":
            return "ACTIVE"
        if state_lc == "inactive":
            return "INACTIVE"
        if state_lc in ("unconfigured", "finalized"):
            return "LOADED"
        return "UNKNOWN"

    def _list_controllers(self, service_name: str) -> Tuple[Optional["ListControllers.Response"], Optional[str]]:
        if ListControllers is None:
            return None, "ListControllers no disponible"
        if self._moveit_node is None:
            return None, self._ros_node_not_ready_reason()
        if not service_name:
            return None, "service vacío"
        if self._controller_client is None or self._controller_client_name != service_name:
            try:
                self._controller_client = self._moveit_node.create_client(ListControllers, service_name)
                self._controller_client_name = service_name
            except Exception as exc:
                self._controller_client = None
                self._controller_client_name = ""
                return None, f"client error: {exc}"
        client = self._controller_client
        if client is None:
            return None, self._list_controllers_not_ready_reason("client")
        node_ns = self.ros_worker.node_namespace() if self._ros_worker_started else ""
        # FASE 2: Timeouts aumentados para evitar falsos negativos.
        wait_timeout = 3.0
        call_timeout = 3.0
        # FASE 2: Hasta 3 intentos con backoff exponencial.
        max_retries = 3
        last_err = None
        for attempt in range(1, max_retries + 1):
            if not client.wait_for_service(timeout_sec=wait_timeout):
                last_err = (
                    f"{self._list_controllers_not_ready_reason('timeout')} "
                    f"service={service_name} ns={node_ns or '/'} timeout={wait_timeout:.1f}s "
                    f"gz={self._gazebo_state()} attempt={attempt}/{max_retries}"
                )
                if attempt < max_retries:
                    time.sleep(0.5 * attempt)
                    continue
                return None, last_err
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=call_timeout)
            if not future.done() or future.result() is None:
                last_err = (
                    f"{self._list_controllers_not_ready_reason('no response')} "
                    f"service={service_name} ns={node_ns or '/'} timeout={call_timeout:.1f}s "
                    f"gz={self._gazebo_state()} attempt={attempt}/{max_retries}"
                )
                if attempt < max_retries:
                    time.sleep(0.5 * attempt)
                    continue
                return None, last_err
            return future.result(), None
        return None, last_err

    def _wait_for_controllers_ready(self, timeout_sec: float) -> Tuple[bool, str]:
        deadline = time.monotonic() + max(0.1, timeout_sec)
        last_reason = "controladores no listos"
        while time.monotonic() < deadline:
            ok, reason = self._controllers_ready()
            last_reason = reason
            if ok:
                return True, reason
            self._wait_for_state_change(0.15)
        return False, last_reason

    @pyqtSlot(int)
    def _schedule_camera_health_check(self, delay_ms: int = 1800) -> None:
        self._camera_ctrl.schedule_health_check(delay_ms)

    def _check_camera_topic_health(self):
        self._camera_ctrl.check_topic_health()
    
    def _update_camera_topics(self, topics):
        self._camera_ctrl.update_topics(topics)
    
    @pyqtSlot()
    def _connect_camera(self):
        self._camera_ctrl.connect()

    def _subscribe_camera(self, topic: str) -> bool:
        return self._camera_ctrl.subscribe(topic)

    @pyqtSlot(int)
    def _start_camera_health_check(self, delay_ms: int = 1200) -> None:
        self._camera_ctrl.start_health_check(delay_ms)

    def _unsubscribe_camera(self) -> None:
        self._camera_ctrl.unsubscribe()

    def _clear_camera_frame(self, *, reset_info: bool = True) -> None:
        self._camera_ctrl.clear_frame(reset_info=reset_info)

    def _resolve_camera_msg_type(self, topic: str) -> str:
        return self._camera_ctrl.resolve_msg_type(topic)
    
    def _auto_connect_camera(self):
        """Auto-conectar cámara al iniciar el panel."""
        self._camera_ctrl.auto_connect()

    def _ensure_ros_worker_started(self):
        """Lazy start the RosWorker when the bridge flow begins."""
        if self._ros_worker_started:
            return
        self.ros_worker.start()
        self._ros_worker_started = True
        self._emit_log("[STARTUP] RosWorker iniciado")

    def _ensure_grasp_rect_subscription(self) -> None:
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return
        if self._grasp_rect_subscribed:
            return
        try:
            ok = bool(self.ros_worker.subscribe_grasp_rect(self._grasp_rect_topic))
        except Exception as exc:
            self._emit_log(f"[TFM] WARN: subscribe /grasp_rect falló ({exc})")
            return
        if ok:
            self._grasp_rect_subscribed = True
            self._emit_log(f"[TFM] Suscripción activa: {self._grasp_rect_topic}")

    def _auto_subscribe_joints(self):
        if not ROS_AVAILABLE:
            return
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if not self.ros_worker.node_ready():
            return
        if self._joint_subscribed:
            return
        try:
            topic, found = self._discover_joint_states_topic(self.joint_topic)
            self.joint_topic = topic
            if topic and topic != self._joint_current_topic:
                self.ros_worker.subscribe_joint_states(topic)
                self._joint_current_topic = topic
            if found:
                self.lbl_joint_states.setText(f"Joint states: suscrito {topic}")
            if self._last_joint_stamp:
                self._joint_active = True
            if self._joint_active and hasattr(self, "joint_timer") and self.joint_timer.isActive():
                self._joint_subscribed = True
                self.joint_timer.stop()
        except Exception as exc:
            self._log_warning(f"No se pudo suscribir a joint_states: {exc}")

    def _discover_joint_states_topic(self, preferred: str) -> Tuple[str, bool]:
        topic = (preferred or "").strip() or "/joint_states"
        if not self.ros_worker.node_ready():
            return topic, False
        try:
            topics = self.ros_worker.list_topic_names()
        except Exception as exc:
            _log_exception("discover joint_states topics", exc)
            return topic, False
        if topic in topics and self.ros_worker.topic_has_publishers(topic):
            return topic, True
        # fallback: cualquier tópico que termine en /joint_states
        for candidate in sorted(t for t in topics if t.endswith("/joint_states")):
            if self.ros_worker.topic_has_publishers(candidate):
                return candidate, True
        return topic, False

    @pyqtSlot()
    def _on_bridge_ready(self):
        self._ensure_pose_subscription()
        self._ensure_grasp_rect_subscription()
        self._start_pose_info_watch()
        self._start_tf_ready_timer()
        now = time.monotonic()
        self._critical_pose_deadline = now + max(0.1, CRITICAL_POSE_TIMEOUT_SEC)
        self._critical_tf_deadline = now + max(0.1, TF_INIT_GRACE_SEC)
        if self._camera_required:
            self._critical_camera_deadline = now + max(0.1, CAMERA_INIT_GRACE_SEC)
        self._auto_subscribe_joints()
        self._invalidate_settle("bridge activado", restart=True)
        QTimer.singleShot(300, self._refresh_camera_topics)
        QTimer.singleShot(600, self._auto_connect_camera)
        if self._auto_release_drop_objects:
            QTimer.singleShot(250, self._release_objects)
        else:
            QTimer.singleShot(600, lambda: self._maybe_hold_drop_objects("bridge"))
            QTimer.singleShot(700, lambda: self._attach_drop_objects("bridge"))
        self._check_camera_topic_health()
        self.signal_calibration_check.emit()
        try:
            if self._gz_running:
                self._refresh_objects_from_gz_async()
                QTimer.singleShot(1500, self._refresh_objects_from_gz_async)
            QTimer.singleShot(1500, lambda: self._apply_home_joint2_offset(retries=3))
            self._detach_attempted = False
            self._detach_inflight = False
            self._detach_auto_disabled = False
            self._detach_backoff_until = 0.0
            self._drop_nudge_done = False
            if self._gz_running and not self._objects_release_done:
                if self._auto_release_drop_objects:
                    self._emit_log("[PHYSICS] Auto-release DROP activo; liberando objetos.")
                else:
                    self._emit_log("[PHYSICS] Objetos en espera: auto-release DROP desactivado.")
            if self._gz_running:
                QTimer.singleShot(900, self._run_fall_test_async)
        except Exception as exc:
            self._log_error(f"[BRIDGE] on_ready error: {exc}")
    
    def _on_image(self, topic: str, qimg, w: int, h: int, fps: float):
        self._camera_ctrl.on_image(topic, qimg, w, h, fps)

    @pyqtSlot(object)
    def _on_grasp_rect(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        try:
            cx = float(payload.get("cx", 0.0))
            cy = float(payload.get("cy", 0.0))
            gw = float(payload.get("w", 0.0))
            gh = float(payload.get("h", 0.0))
            angle_deg = float(payload.get("angle_deg", 0.0))
        except Exception:
            return
        if gw <= 0.0 or gh <= 0.0:
            return
        self._last_grasp_px = {
            "cx": cx,
            "cy": cy,
            "w": gw,
            "h": gh,
            "angle_deg": angle_deg,
        }
        self._last_grasp_source = f"topic:{payload.get('topic') or self._grasp_rect_topic}"
        self._last_grasp_frame = self.camera_topic or "image"
        frame_w = frame_h = 0
        if self._last_camera_frame:
            _qimg, frame_w, frame_h, _ts = self._last_camera_frame
        if frame_w > 0 and frame_h > 0:
            self._last_grasp_world = self._compute_world_grasp(frame_w, frame_h)
            self._last_grasp_base = None
            if self._last_grasp_world:
                base_coords = self._ensure_base_coords(
                    (
                        float(self._last_grasp_world.get("x", 0.0)),
                        float(self._last_grasp_world.get("y", 0.0)),
                        float(self._last_grasp_world.get("z", 0.0)),
                    ),
                    self._world_frame_config_first(),
                    timeout_sec=0.35,
                )
                if base_coords is not None:
                    self._last_grasp_base = {
                        "x": float(base_coords[0]),
                        "y": float(base_coords[1]),
                        "z": float(base_coords[2]),
                        "yaw_deg": float(self._last_grasp_world.get("yaw_deg", 0.0) or 0.0),
                    }
            ref = self._build_reference_grasp(frame_w, frame_h)
            if ref and self._last_grasp_px:
                self._update_cornell_metrics(self._last_grasp_px, ref)
            else:
                self._last_cornell = None
            with self._camera_frame_lock:
                if self._last_camera_frame:
                    qimg, w, h, ts = self._last_camera_frame
                    self._camera_pending_frame = (
                        self.camera_topic or self._last_grasp_frame or "image",
                        qimg,
                        w,
                        h,
                        self._camera_last_fps,
                        ts,
                    )
            self._refresh_camera_display()
        else:
            self._last_grasp_world = None
            self._last_grasp_base = None
            self._last_cornell = None
        self._refresh_science_ui()
        self._set_status("TFM: /grasp_rect recibido", error=False)
        self._audit_append(
            "logs/perception.log",
            "[TFM] grasp_rect rx "
            f"topic={payload.get('topic') or self._grasp_rect_topic} "
            f"type={payload.get('msg_type') or 'unknown'} "
            f"cx={cx:.1f} cy={cy:.1f} w={gw:.1f} h={gh:.1f} theta={angle_deg:.2f}deg "
            f"base={self._last_grasp_base} source={self._last_grasp_source}",
        )
        self._audit_write_json(
            "artifacts/grasp_rect_last.json",
            {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "source": self._last_grasp_source,
                "topic": payload.get("topic") or self._grasp_rect_topic,
                "msg_type": payload.get("msg_type"),
                "grasp_image": self._last_grasp_px,
                "grasp_base": self._last_grasp_base,
                "frame": self._last_grasp_frame,
                "cornell": self._last_cornell,
            },
        )

    def _reset_camera_retry_backoff(self):
        self._camera_reconnect_attempts = 0
        self._camera_reconnect_last_reason = ""
        self._camera_reconnect_scheduled = False

    def _refresh_camera_display(self):
        self._camera_ctrl.refresh_display()

    def _check_camera_stream(self):
        return
    def _on_joint_state(self, payload: Dict[str, object]):
        if not payload:
            return
        topic = payload.get("topic") or ""
        if topic:
            self._joint_current_topic = topic

        names = payload.get("name") or []
        pos_list = payload.get("position") or []
        vel_list = payload.get("velocity") or []
        eff_list = payload.get("effort") or []
        source = payload.get("source") or "ros"
        stamp = float(payload.get("stamp") or 0.0)
        if stamp:
            self._last_joint_stamp = stamp
        now = stamp or time.time()
        self._joint_active = True

        norm_names = [_normalize_joint_name(n) for n in names]
        pos_map: Dict[str, float] = {}
        vel_map: Dict[str, float] = {}
        eff_map: Dict[str, float] = {}
        for name, pos in zip(norm_names, pos_list):
            try:
                pos_map[name] = float(pos)
            except Exception as exc:
                _log_exception(f"parse joint position {name}", exc)
                continue
        if isinstance(vel_list, list) and len(vel_list) == len(names):
            for name, vel in zip(norm_names, vel_list):
                try:
                    vel_map[name] = float(vel)
                except Exception as exc:
                    _log_exception(f"parse joint velocity {name}", exc)
                    continue
        if isinstance(eff_list, list) and len(eff_list) == len(names):
            for name, eff in zip(norm_names, eff_list):
                try:
                    eff_map[name] = float(eff)
                except Exception as exc:
                    _log_exception(f"parse joint effort {name}", exc)
                    continue

        missing_vel = not vel_map or any(v is None for v in vel_map.values())
        if pos_map and missing_vel and self._last_joint_time and now > self._last_joint_time:
            dt = max(1e-6, now - self._last_joint_time)
            for name, pos in pos_map.items():
                prev = self._last_joint_positions.get(name)
                if prev is not None and vel_map.get(name) is None:
                    vel_map[name] = (pos - prev) / dt
        if pos_map:
            self._last_joint_positions.update(pos_map)
            self._last_joint_time = now
            if not self._joint_names_warned:
                if not any(j in pos_map for j in UR5_JOINT_NAMES):
                    sample = ", ".join(sorted(list(pos_map.keys()))[:8])
                    self._log_warning(
                        f"[JOINTS] Joint names no coinciden con UR5_JOINT_NAMES. sample={sample}"
                    )
                    self._joint_names_warned = True

        if pos_map and not any(slider.isSliderDown() for slider in self.joint_sliders):
            # Protección extra: no actualizar sliders si el usuario acaba de interactuar manualmente
            if time.time() > self._slider_update_blocked_until:
                self._updating_sliders_from_joint_state = True
                try:
                    for idx, joint in enumerate(UR5_JOINT_NAMES):
                        if joint not in pos_map:
                            continue
                        slider = self.joint_sliders[idx]
                        deg = math.degrees(pos_map[joint])
                        value = int(round(deg * JOINT_SLIDER_SCALE))
                        value = max(slider.minimum(), min(slider.maximum(), value))
                        if slider.value() != value:
                            slider.setValue(value)
                finally:
                    self._updating_sliders_from_joint_state = False

        for joint, pos_lbl in self.dof_pos_labels.items():
            pos = pos_map.get(joint)
            if pos is None:
                pos_lbl.setText("--")
            else:
                pos_lbl.setText(f"{math.degrees(pos):.2f} deg / {pos:.3f} rad")
        for joint, vel_lbl in self.dof_vel_labels.items():
            vel = vel_map.get(joint)
            if vel is None:
                vel_lbl.setText("--")
            else:
                vel_lbl.setText(f"{vel:.3f} rad/s")

        grip_positions = []
        for joint, pos_lbl in self.gripper_labels.items():
            pos = pos_map.get(joint)
            if pos is None:
                pos_lbl.setText("--")
            else:
                pos_mm = pos * 1000.0
                pos_lbl.setText(f"{pos_mm:.1f} mm / {pos:.4f} m")
                grip_positions.append(pos)
        if self.gripper_total_lbl is not None:
            if len(grip_positions) == len(self.gripper_labels):
                opening = sum(abs(v) for v in grip_positions) * 1000.0
                self.gripper_total_lbl.setText(f"{opening:.1f} mm")
            else:
                self.gripper_total_lbl.setText("--")

        q = []
        for joint in UR5_JOINT_NAMES:
            if joint not in pos_map:
                q = []
                break
            q.append(pos_map[joint])
        if len(q) == 6:
            pos, rot = fk_ur5(q)
            roll, pitch, yaw = _rot_to_rpy(rot)
            bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
            wx, wy, wz = base_to_world(bx, by, bz)
            self._last_tcp_base = (bx, by, bz)
            self._last_tcp_world = (wx, wy, wz)
            if self.tcp_xyz_lbl is not None:
                self.tcp_xyz_lbl.setText(f"{bx:.3f}, {by:.3f}, {bz:.3f}")
            if self.tcp_rpy_lbl is not None:
                r_deg = math.degrees(roll)
                p_deg = math.degrees(pitch)
                y_deg = math.degrees(yaw)
                self._last_tcp_rpy_deg = (r_deg, p_deg, y_deg)
                self.tcp_rpy_lbl.setText(f"{r_deg:.1f}, {p_deg:.1f}, {y_deg:.1f}")
        else:
            self._last_tcp_base = None
            if self.tcp_xyz_lbl is not None:
                self.tcp_xyz_lbl.setText("--")
            if self.tcp_rpy_lbl is not None:
                self.tcp_rpy_lbl.setText("--")

        vel_values = [vel_map[j] for j in UR5_JOINT_NAMES if j in vel_map]
        if vel_values:
            norm = math.sqrt(sum(v * v for v in vel_values))
            vmax = max(abs(v) for v in vel_values)
            if self.vel_norm_lbl is not None:
                self.vel_norm_lbl.setText(f"{norm:.3f} rad/s")
            if self.vel_max_lbl is not None:
                self.vel_max_lbl.setText(f"{vmax:.3f} rad/s")
        else:
            if self.vel_norm_lbl is not None:
                self.vel_norm_lbl.setText("--")
            if self.vel_max_lbl is not None:
                self.vel_max_lbl.setText("--")

        eff_values = [eff_map[j] for j in UR5_JOINT_NAMES if j in eff_map]
        if eff_values:
            emax = max(abs(v) for v in eff_values)
            if self.eff_max_lbl is not None:
                self.eff_max_lbl.setText(f"{emax:.3f}")
        else:
            if self.eff_max_lbl is not None:
                self.eff_max_lbl.setText("--")

        if self.lbl_joint_states is not None:
            if self._last_joint_stamp:
                self.lbl_joint_states.setText(f"Joint states: ok ({len(names)} joints, {source})")
            else:
                self.lbl_joint_states.setText("Joint states: esperando /joint_states ...")

        if self._debug_joints_to_stdout and pos_map:
            joint_parts = []
            for joint in UR5_JOINT_NAMES:
                if joint in pos_map:
                    joint_parts.append(f"{joint}={pos_map[joint]:.3f}rad")
            for joint in self.gripper_labels:
                if joint in pos_map:
                    joint_parts.append(f"{joint}={pos_map[joint]:.4f}m")
            pose_txt = "tcp xyz=-- rpy=--"
            if len(q) == 6:
                pos, rot = fk_ur5(q)
                roll, pitch, yaw = _rot_to_rpy(rot)
                pose_txt = (
                    f"tcp xyz=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                    f"rpy=({math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f})"
                )
            msg = "[DEBUG] JOINTS " + " ".join(joint_parts) + f" | {pose_txt}"
            self._emit_log(msg)

    def _fill_worlds(self):
        self.world_combo.clear()
        for p in DEFAULT_WORLD_CANDIDATES:
            if os.path.isfile(p):
                self.world_combo.addItem(p)
        if os.path.isdir(WORLDS_DIR):
            for fn in sorted(os.listdir(WORLDS_DIR)):
                if fn.endswith((".sdf", ".world")):
                    full = os.path.join(WORLDS_DIR, fn)
                    if full not in DEFAULT_WORLD_CANDIDATES:
                        self.world_combo.addItem(full)
        if self.world_combo.count() == 0:
            self.world_combo.addItem(os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf"))
        self.world_combo.setCurrentIndex(0)

    def _fill_bridge_presets(self):
        self.bridge_presets.clear()
        seen = set()
        if os.path.isfile(BRIDGE_BASE_YAML):
            self.bridge_presets.addItem(BRIDGE_BASE_YAML)
            seen.add(BRIDGE_BASE_YAML)
        if os.path.isdir(SCRIPTS_DIR):
            for fn in sorted(os.listdir(SCRIPTS_DIR)):
                if not fn.endswith(('.yaml', '.yml')):
                    continue
                full = os.path.join(SCRIPTS_DIR, fn)
                if full in seen:
                    continue
                self.bridge_presets.addItem(full)
                seen.add(full)
        if self.bridge_presets.count() == 0:
            self.bridge_presets.addItem(BRIDGE_BASE_YAML)
        self.bridge_presets.setCurrentIndex(0)

    def _apply_bridge_preset(self, text: str):
        if text:
            self.bridge_edit.setText(text)

    def _choose_world(self):
        self._log_button("Browse mundo")
        path, _ = QFileDialog.getOpenFileName(self, "Selecciona mundo", WORLDS_DIR, "SDF/WORLD (*.sdf *.world)")
        if path:
            self.world_combo.setCurrentText(path)

    def _choose_yaml(self):
        self._log_button("Browse bridge YAML")
        path, _ = QFileDialog.getOpenFileName(self, "Selecciona YAML del bridge", os.path.dirname(BRIDGE_BASE_YAML), "YAML (*.yaml *.yml)")
        if path:
            self.bridge_edit.setText(path)

    def _run_script(self, script_name: str, label: str):
        self._log_button(label)
        path = os.path.join(self.ws_dir, "scripts", script_name)
        if not os.path.isfile(path):
            self._set_status(f"No existe {script_name}", error=True)
            return

        def worker():
            self._ui_set_status(f"Ejecutando {label}…")
            try:
                res = subprocess.run(["bash", path], capture_output=True, text=True, timeout=180)
                if res.returncode == 0:
                    self._ui_set_status(f"OK {label}")
                else:
                    self._ui_set_status(f"Fallo {label} (rc={res.returncode})", error=True)
            except subprocess.TimeoutExpired:
                self._ui_set_status(f"Timeout {label}", error=True)
            except Exception as exc:
                self._ui_set_status(f"Error {label}: {exc}", error=True)

        self._run_async(worker)

    def _toggle_debug(self, env_var: str):
        current = os.environ.get(env_var, "0")
        new_val = "0" if current == "1" else "1"
        os.environ[env_var] = new_val
        enabled = new_val == "1"
        if env_var == "DEBUG_LOGS_TO_STDOUT":
            self._debug_logs_enabled = enabled
            self.btn_debug_logs.setChecked(enabled)
            self._apply_debug_button_style(self.btn_debug_logs, enabled)
        if env_var == "DEBUG_JOINTS_TO_STDOUT":
            self.btn_debug_joints.setChecked(enabled)
            self._apply_debug_button_style(self.btn_debug_joints, enabled)
            if enabled:
                self._print_pose_snapshot()
        label = "ON" if enabled else "OFF"
        self._log_button(f"Toggle {env_var} -> {label}")
        self._set_status(f"{env_var} -> {label}")

    def _start_all(self):
        if self._block_if_managed("START"):
            return
        self._log_button("START")
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("START bloqueado: ERROR_FATAL activo", error=True)
            self._emit_log("[SAFETY] START bloqueado: ERROR_FATAL activo")
            return
        # Activar timeouts críticos también en modo no gestionado para fail-fast.
        now = time.monotonic()
        self._critical_clock_deadline = now + max(0.1, CRITICAL_CLOCK_TIMEOUT_SEC)
        self._critical_pose_deadline = 0.0
        self._critical_tf_deadline = 0.0
        self._critical_camera_deadline = 0.0
        if not self._moveit_required:
            self._moveit_required = True
            self._emit_log("[AUTO] MoveIt requerido activado (START)")
            self.signal_refresh_controls.emit()
        self._star_inflight = True
        self.btn_star.setEnabled(False)
        self.btn_kill_hard.setEnabled(True)
        sequence = StartSequence(
            emit_log=self._emit_log,
            run_ui=self.signal_run_ui.emit,
            run_ui_delayed=self.signal_run_ui_delayed.emit,
            gazebo_state=self._gazebo_state,
            is_bridge_running=lambda: self._bridge_running,
            bridge_ready=lambda: self._bridge_ready_status()[0],
            moveit_ready=lambda: self._moveit_state == MoveItState.READY,
            controllers_ready=lambda: self._controllers_ready()[0],
            is_closing=lambda: self._closing,
            start_gazebo=self._start_gazebo,
            start_bridge=self._start_bridge,
            start_moveit=self._start_moveit,
            start_moveit_bridge=self._start_moveit_bridge,
            start_release_service=self._start_release_service,
            on_fatal=self._on_start_fatal,
            on_moveit_autostart_blocked=self._log_moveit_autostart_blocked,
            controllers_timeout_sec=CONTROLLER_READY_TIMEOUT_SEC,
            wait_for_change=self._wait_for_state_change,
            clock_ready=lambda: self._clock_status()[0],
            clock_reason=lambda: f"Gazebo/clock no listo ({self._clock_status()[1]})",
            joint_states_ready=lambda: self._joint_states_status()[0],
            joint_states_reason=lambda: f"/joint_states no listo ({self._joint_states_status()[1]})",
            pose_info_ready=self._pose_info_ready,
            pose_info_reason=self._pose_info_not_ready_reason,
            tf_ready=lambda: self._tf_chain_ready_status()[0],
            tf_reason=lambda: f"TF no listo ({self._tf_chain_ready_status()[1]})",
            camera_ready=lambda: camera_ready_status(self)[0],
            camera_reason=self._camera_not_ready_reason,
            camera_required=self._camera_required,
            clock_timeout_sec=CRITICAL_CLOCK_TIMEOUT_SEC,
            pose_timeout_sec=CRITICAL_POSE_TIMEOUT_SEC,
            tf_timeout_sec=TF_INIT_GRACE_SEC,
            camera_timeout_sec=CAMERA_INIT_GRACE_SEC,
        )
        self._run_async(sequence.run, name="start_all")

    def _log_moveit_autostart_blocked(self, reason: str) -> None:
        now = time.time()
        camera_ready, camera_fault, camera_source_down, age, in_grace = self._camera_runtime_flags(now)
        self._emit_log(
            "[AUTO] MoveIt no autoiniciado: "
            f"razon={reason} "
            f"state={self._system_state.value} "
            f"gazebo={self._gazebo_state()} "
            f"controllers={self._controllers_ok}({self._controllers_reason or 'ok'}) "
            "camera="
            f"ready={camera_ready},fault={camera_fault},source_down={camera_source_down},"
            f"age={age:.2f}s,grace={in_grace}"
        )

    def _on_start_fatal(self, reason: str) -> None:
        normalized = reason.lower()
        if "camera" in normalized or "cámara" in normalized:
            msg = f"startup degraded: {reason}"
            self._set_system_state(SystemState.ERROR, msg)
            self._ui_set_status("ERROR: cámara no lista (diagnóstico)", error=True)
            self._emit_log(f"[ERROR] {msg}")
            self._emit_log("[CAMERA][DIAG] Usa Recover o reinicia bridge/cámara")
            return
        if "controller" in normalized or "controlador" in normalized:
            msg = f"startup failed: {reason}"
            self._system_error_reason = msg
            self._set_system_state(SystemState.ERROR, msg)
            self._ui_set_status(f"ERROR: {msg}", error=True)
            self._emit_log(f"[ERROR] {msg}")
            return
        self._trigger_fatal(f"startup failed: {reason}")

    def _stop_all(self):
        self._log_button("STOP")
        sequence = StopSequence(
            emit_log=self._emit_log,
            run_script=lambda: self._run_script("kill_all.sh", "STOP"),
            schedule_start_check=self._schedule_start_enable_check,
            set_star_inflight=lambda value: setattr(self, "_star_inflight", value),
            set_kill_enabled=self.btn_kill_hard.setEnabled,
        )
        sequence.run()

    def _recover_runtime(self) -> None:
        self._log_button("Recover")
        self._emit_log("[RECOVER] Iniciando recuperación en modo diagnóstico")
        self._fatal_latched = False
        self._fatal_shutdown_started = False
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_system_state(SystemState.ERROR, "recover_requested")
        gz_state = self._gazebo_state()
        if gz_state in ("GAZEBO_OFF",):
            self._emit_log(f"[RECOVER] Gazebo={gz_state}; relanzando Gazebo")
            self._start_gazebo()
            QTimer.singleShot(1800, self._start_bridge)
            QTimer.singleShot(3200, self._camera_ctrl.refresh_topics)
            QTimer.singleShot(4200, self._camera_ctrl.connect)
            return
        if not self._bridge_running:
            self._emit_log("[RECOVER] Bridge no activo; relanzando bridge")
            self._start_bridge()
            QTimer.singleShot(1500, self._camera_ctrl.refresh_topics)
            QTimer.singleShot(2500, self._camera_ctrl.connect)
            return
        self._emit_log("[RECOVER] Reintentando cámara/bridge sin apagar stack")
        self._camera_ctrl.refresh_topics()
        QTimer.singleShot(900, self._camera_ctrl.connect)
        self._camera_ctrl.schedule_health_check(1200)

    def _system_running(self) -> bool:
        if self._proc_alive(self.gz_proc) or self._proc_alive(self.bridge_proc):
            return True
        if self._proc_alive(self.moveit_proc) or self._proc_alive(self.moveit_bridge_proc):
            return True
        if self._proc_alive(self.bag_proc) or self._proc_alive(self.rsp_proc):
            return True
        if self._gazebo_state() != "GAZEBO_OFF":
            return True
        if self._bridge_running or self._moveit_state != MoveItState.OFF:
            return True
        return False

    def _schedule_start_enable_check(self, delay_ms: int = 1200) -> None:
        def _check():
            if not self._system_running():
                self.btn_star.setEnabled(True)
                self.btn_kill_hard.setEnabled(False)
                return
            QTimer.singleShot(delay_ms, _check)

        QTimer.singleShot(delay_ms, _check)

    def _start_gazebo(self):
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("Gazebo bloqueado: ERROR_FATAL activo", error=True)
            self._emit_log_throttled(
                "SAFETY:start_gazebo:ERROR_FATAL",
                "[SAFETY] Gazebo bloqueado: ERROR_FATAL activo",
            )
            return
        panel_launch_control.start_gazebo(self)
        return
    def _parse_first_json_object(self, text: str) -> Optional[Dict[str, object]]:
        if not text:
            return None
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            if line.startswith("{"):
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    continue
        return None

    def _detect_world_name(self) -> Optional[str]:
        return self._physics.detect_world_name()

    def _read_world_stats(self, world_name: str) -> Dict[str, object]:
        return {}

    def _try_unpause_world(self, world_name: str) -> bool:
        return False

    def _probe_pose_motion(self, world_name: str, targets: Set[str]) -> Tuple[Optional[bool], float, Optional[str]]:
        return self._physics.probe_pose_motion(world_name, targets)

    def check_physics_runtime(self) -> None:
        self._physics.check_physics_runtime()

    def _schedule_physics_runtime_check(self) -> None:
        self._physics.schedule_physics_runtime_check()

    def _throw_objects(self):
        self._log_button("Lanzar objetos")
        self._set_status("Lanzando objetos en Gazebo…")

        # LEGACY: deshabilitado en modo física real (MoveIt-only / no teletransporte).
        self._log("[SAFETY] Teleport detectado: bloqueado (throw_objects)")
        self._set_status("Bloqueado: no se permite teletransporte", error=True)
        return

    def _toggle_debug_poses(self):
        """Toggle on/off streaming de poses y joints hacia el terminal del panel."""
        if self.btn_debug_joints.isChecked():
            self._start_debug_poses()
        else:
            self._stop_debug_poses()

    def _start_debug_poses(self):
        """Inicia streaming de poses parseadas (nombre + posición)."""
        if not self._gz_running:
            self._log_error("Gazebo no está activo")
            self.btn_debug_joints.setChecked(False)
            return
        self._debug_joints_to_stdout = True
        os.environ["DEBUG_JOINTS_TO_STDOUT"] = "1"
        self._stop_debug_poses()
        self._ensure_pose_subscription()
        self._print_pose_snapshot()
        self._pose_debug_timer = QTimer(self)
        self._pose_debug_timer.timeout.connect(self._print_pose_snapshot)
        self._pose_debug_timer.start(int(DEBUG_POSES_PERIOD_SEC * 1000))
        self._log("[DEBUG] Iniciado - snapshots de poses")
        self._apply_debug_button_style(self.btn_debug_joints, True)

    def _stop_debug_poses(self):
        """Detiene streaming de poses y joints."""
        self._debug_joints_to_stdout = False
        os.environ["DEBUG_JOINTS_TO_STDOUT"] = "0"
        if self._pose_debug_timer:
            self._pose_debug_timer.stop()
            self._pose_debug_timer.deleteLater()
            self._pose_debug_timer = None
        self._apply_debug_button_style(self.btn_debug_joints, False)
        self._log("[DEBUG] Detenido")

    def _log_trace(self, message: str) -> None:
        if getattr(self, "_debug_joints_to_stdout", False):
            self._emit_log(message)
        else:
            self._log(message)

    def _apply_debug_button_style(self, button: QPushButton, enabled: bool) -> None:
        button.setStyleSheet(
            "background:#3b82f6; color:white; border-radius:6px;" if enabled else ""
        )

    def _print_pose_snapshot(self):
        """Imprime en una línea: TCP, cesta, mesa y objetos en frame base_link."""
        objs = {}
        pose_src = {}
        if self._gz_running:
            world_name = self._gz_world_name or read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
            poses = self._read_world_pose_info(world_name)
            if poses:
                known = get_object_positions()
                objs, pose_src = self._extract_pose_updates(poses, known)
            if not objs:
                objs = get_object_positions()
        obj_parts = []
        world_frame = self._world_frame_last_first()
        for name, (x, y, z) in sorted(objs.items()):
            base_obj = self._ensure_base_coords(
                (float(x), float(y), float(z)),
                world_frame,
                timeout_sec=0.2,
            )
            src = pose_src.get(name)
            if base_obj is not None:
                bx, by, bz = base_obj
                if src:
                    obj_parts.append(f"{name}=({bx:.3f},{by:.3f},{bz:.3f})[{src}]")
                else:
                    obj_parts.append(f"{name}=({bx:.3f},{by:.3f},{bz:.3f})")
            else:
                if src:
                    obj_parts.append(f"{name}=(n/a)[{src}]")
                else:
                    obj_parts.append(f"{name}=(n/a)")
        obj_txt = "objs: " + (" ".join(obj_parts) if obj_parts else "-" )

        base_frame = self._business_base_frame()
        ee_frame = "rg2_tcp"
        tcp_txt = "tcp_base=UNAVAILABLE tcp_source=tf2"
        tcp_pose, rpy_deg, tcp_reason = tf_get_tcp_in_base(
            base_frame=base_frame,
            ee_frame=ee_frame,
            timeout=0.20,
            logger=None,
        )
        if tcp_pose is not None and rpy_deg is not None:
            bx = float(tcp_pose.pose.position.x)
            by = float(tcp_pose.pose.position.y)
            bz = float(tcp_pose.pose.position.z)
            r_deg, p_deg, y_deg = rpy_deg
            self._last_debug_tcp_base = (bx, by, bz)
            self._last_debug_tcp_ts = time.monotonic()
            tcp_txt = (
                f"tcp_base=({bx:.3f},{by:.3f},{bz:.3f}) "
                f"rpy=({r_deg:.1f},{p_deg:.1f},{y_deg:.1f}) tcp_source=tf2"
            )
        else:
            self._last_debug_tcp_base = None
            self._last_debug_tcp_ts = time.monotonic()
            tcp_txt = f"tcp_base=UNAVAILABLE tcp_source=tf2 reason={tcp_reason}"

        basket_x, basket_y, basket_z = BASKET_DROP
        basket_txt = f"basket=({basket_x:.3f},{basket_y:.3f},{basket_z:.3f})"
        table_txt = f"table=({TABLE_CENTER_X:.3f},{TABLE_CENTER_Y:.3f})"

        line = f"[DEBUG POSES] {tcp_txt} | {basket_txt} | {table_txt} | {obj_txt}"
        self._emit_log(line)

    def _drop_detach_supported(self) -> bool:
        if self._detach_feature_available:
            return True
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if self.ros_worker and self.ros_worker.node_ready():
            try:
                topics = self.ros_worker.list_topic_names()
            except Exception as exc:
                _log_exception("drop_detach list topics", exc)
                topics = []
            prefix = f"{DROP_ANCHOR_PREFIX}/"
            for topic in topics:
                if topic.startswith(prefix) and topic.endswith("/detach"):
                    self._detach_feature_available = True
                    return True
        return False

    def _release_objects(self):
        """Publicar detach a todos los objetos en Gazebo (emergencia)."""
        if self._detach_inflight:
            return
        if self._detach_auto_disabled and time.time() < self._detach_backoff_until:
            return
        # Permitir relanzar la liberación global si el servicio tarda en estar listo.
        self._objects_release_done = False
        self._drop_hold_enabled = False
        detach_supported = self._drop_detach_supported()
        if not detach_supported and not self._detach_feature_logged:
            self._emit_log("[PHYSICS][DETACH] canales drop_anchor detach no disponibles; usando release_objects service")
            self._detach_feature_logged = True
        self._detach_inflight = True
        self._detach_attempted = True
        self._log_button("Soltar objetos")
        self._ui_set_status("Soltando objetos…")

        def worker():
            try:
                if not ROS_AVAILABLE or not self.ros_worker or not self.ros_worker.node_ready():
                    self._log_error("ROS no disponible; no se pudo soltar objetos")
                    self._ui_set_status("Servicio objetos no accesible", error=True)
                    if self._auto_release_drop_objects:
                        self._schedule_release_retry("ros_not_ready")
                    return
                release_ok = False
                release_msg = ""
                if self.ros_worker.has_service("/release_objects"):
                    release_ok, release_msg = self.ros_worker.call_trigger_detail(
                        "/release_objects", timeout_sec=20.0
                    )
                    release_msg = (release_msg or "").strip()
                    self._emit_log(
                        "[PHYSICS][DROP] release_objects service "
                        f"success={str(bool(release_ok)).lower()} "
                        f"message='{release_msg or 'sin mensaje'}'"
                    )
                else:
                    release_msg = "release_objects service no disponible"
                    self._emit_log(f"[PHYSICS][DROP] {release_msg}")

                if not release_ok:
                    self._objects_release_done = False
                    self._objects_settled = False
                    self.signal_refresh_controls.emit()
                    hint = release_msg or "revisa [PHYSICS][DROP] para delete/spawn/drop_anchor"
                    self._emit_log(
                        f"[PHYSICS][DROP] release_objects failed: {hint}"
                    )
                    self._ui_set_status(f"❌ Soltar objetos falló: {hint}", error=True)
                    if self._auto_release_drop_objects:
                        self._schedule_release_retry(f"release_failed:{hint}")
                    return

                if release_ok:
                    # Give Gazebo a short window to respawn dynamic models before detach.
                    time.sleep(0.4)

                detach_sent = 0
                if release_ok:
                    self._drop_anchor_attached = False
                    self._emit_log(
                        "[PHYSICS][DETACH] detach skipped: joint not present (global reset already applied)"
                    )
                elif detach_supported and self._drop_anchor_attached:
                    for name in DROP_OBJECT_NAMES:
                        topic = f"{DROP_ANCHOR_PREFIX}/{name}/detach"
                        pub = self._get_attach_publisher(topic)
                        if pub is None:
                            self._emit_log(
                                f"[PHYSICS][DETACH] detach skipped: joint not present object={name} (publisher missing)"
                            )
                            continue
                        if self.ros_worker.topic_subscriber_count(topic) <= 0:
                            self._emit_log(
                                f"[PHYSICS][DETACH] detach skipped: joint not present object={name} (no subscribers)"
                            )
                            continue
                        pub.publish(Empty())
                        detach_sent += 1
                    if detach_sent > 0:
                        self._emit_log(
                            f"[PHYSICS][DETACH] drop_anchor detach publicado count={detach_sent}"
                        )
                        self._drop_anchor_attached = False
                    else:
                        self._emit_log(
                            "[PHYSICS][DETACH] detach skipped: joint not present"
                        )
                else:
                    self._emit_log(
                        "[PHYSICS][DETACH] detach skipped: joint not present"
                    )

                self._objects_release_done = True
                self._objects_settled = False
                self._release_retry_count = 0
                state = get_object_state(PICK_DEMO_OBJECT_NAME)
                if state and state.logical_state in (
                    ObjectLogicalState.GRASPED,
                    ObjectLogicalState.CARRIED,
                ):
                    mark_object_released(
                        PICK_DEMO_OBJECT_NAME, reason="release_objects_grasped"
                    )
                else:
                    self._emit_log(
                        "[OBJECTS] release grasp skipped (not grasped/carried); applying global reset"
                    )
                released_count = force_release_all_objects(
                    reason="release_objects_global_reset"
                )
                recalc_object_states(reason="release_objects_post_reset")
                self._emit_log(
                    f"[OBJECTS] global reset applied owners=NONE count={released_count}"
                )
                self.signal_refresh_controls.emit()
                self._ui_set_status("✅ Objetos soltados")
                self._invalidate_settle("objetos liberados", restart=True)
                self._maybe_nudge_drop_objects("release_success")
            except Exception as e:
                self._log_error(f"Soltar objetos error: {e}")
                self._ui_set_status(f"Error soltando objetos: {e}", error=True)
            finally:
                self._detach_inflight = False

        self._run_async(worker)

    def _schedule_release_retry(self, reason: str) -> None:
        self._release_retry_count = int(self._release_retry_count or 0) + 1
        attempt = self._release_retry_count
        if attempt <= 8:
            delay_ms = min(4000, 500 * attempt)
            self._emit_log(
                f"[PHYSICS][DETACH] retry reason={reason} attempt={attempt}/8 delay_ms={delay_ms}"
            )
            self.signal_run_ui_delayed.emit(self._release_objects, delay_ms)
            return
        self._detach_auto_disabled = True
        self._detach_backoff_until = time.time() + 10.0
        self._emit_log(
            "[PHYSICS][DETACH] retry agotado; auto-release pausado (10s) para evitar bucle"
        )

    def _attach_drop_objects(self, reason: str) -> None:
        if self._drop_anchor_attached:
            return
        if self._objects_release_done:
            return
        if not ROS_AVAILABLE or not self.ros_worker or not self.ros_worker.node_ready():
            return
        for name in DROP_OBJECT_NAMES:
            topic = f"{DROP_ANCHOR_PREFIX}/{name}/attach"
            pub = self._get_attach_publisher(topic)
            if pub is not None:
                pub.publish(Empty())
        self._drop_anchor_attached = True
        self._emit_log(f"[PHYSICS][HOLD] drop_anchor attach publicado ({reason})")

    def _maybe_nudge_drop_objects(self, reason: str) -> None:
        if not NUDGE_DROP_OBJECTS:
            return
        if self._drop_nudge_done:
            return
        self._drop_nudge_done = True
        def worker():
            time.sleep(0.4)
            self._nudge_drop_objects(reason)
        self._run_async(worker)

    def _resolve_set_pose_service(self, world_name: str) -> Optional[str]:
        if not self.ros_worker or not self.ros_worker.node_ready():
            return None
        expected = f"/world/{world_name}/set_entity_pose"
        services = self.ros_worker.service_names_and_types()
        if not services:
            return None
        type_hint = "SetEntityPose"
        candidates = []
        for name, types in services:
            if not name:
                continue
            if any(type_hint in t for t in (types or [])):
                candidates.append(name)
        if expected in candidates:
            return expected
        for suffix in ("/set_entity_pose", "/set_pose"):
            for name in candidates:
                if name.endswith(suffix) and f"/world/{world_name}/" in name:
                    return name
        for name in candidates:
            if name.endswith("/set_entity_pose"):
                return name
        return None

    def _maybe_hold_drop_objects(self, reason: str) -> None:
        if not self._drop_hold_enabled:
            return
        if self._objects_release_done:
            return
        if not DROP_OBJECT_NAMES:
            return
        if self._drop_hold_inflight:
            return
        now = time.monotonic()
        if (now - self._drop_hold_last_ts) < 0.9:
            return
        self._drop_hold_last_ts = now
        self._drop_hold_inflight = True

        def worker():
            try:
                self._hold_drop_objects(reason)
            finally:
                self._drop_hold_inflight = False

        self._run_async(worker)

    def _hold_drop_objects(self, reason: str) -> None:
        if not ROS_AVAILABLE or SetEntityPose is None:
            self._hold_drop_objects_gz(reason)
            return
        self._ensure_moveit_node()
        if self._moveit_node is None:
            self._hold_drop_objects_gz(reason, fallback_note="[PHYSICS][HOLD] nodo ROS no listo")
            return
        world_name = self._gz_world_name or read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
        service_name = self._set_pose_service_name or self._resolve_set_pose_service(world_name)
        if not service_name:
            self._hold_drop_objects_gz(reason, fallback_note="[PHYSICS][HOLD] servicio SetEntityPose no encontrado")
            return
        self._set_pose_service_name = service_name
        client = self._moveit_node.create_client(SetEntityPose, service_name)
        if not client.wait_for_service(timeout_sec=0.4):
            self._hold_drop_objects_gz(reason, fallback_note=f"[PHYSICS][HOLD] servicio no disponible: {service_name}")
            return
        self._drop_hold_warned = False
        positions = get_object_positions()
        held = 0
        for name in DROP_OBJECT_NAMES:
            target = self._drop_spawn_positions.get(name)
            if not target:
                continue
            current = positions.get(name)
            if current and current[2] >= (target[2] - 0.05):
                continue
            tx, ty, tz = target
            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = GzEntity.MODEL if GzEntity else 2
            req.pose.position.x = float(tx)
            req.pose.position.y = float(ty)
            req.pose.position.z = float(tz)
            req.pose.orientation.w = 1.0
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=0.6)
            if future.done() and future.result() and future.result().success:
                held += 1
        now = time.monotonic()
        if held and (now - self._drop_hold_log_ts) > 2.0:
            self._emit_log(f"[PHYSICS][HOLD] held={held} reason={reason}")
            self._drop_hold_log_ts = now

    def _hold_drop_objects_gz(self, reason: str, fallback_note: str = "") -> None:
        if self._drop_hold_gz_available is None:
            self._drop_hold_gz_available = shutil.which("gz") is not None
        if not self._drop_hold_gz_available:
            if not self._drop_hold_gz_warned:
                note = " (gz CLI no disponible)" if not fallback_note else f" ({fallback_note})"
                self._emit_log(f"[PHYSICS][HOLD] fallback deshabilitado{note}")
                self._drop_hold_gz_warned = True
            return
        if fallback_note and not self._drop_hold_warned:
            self._emit_log(fallback_note)
            self._drop_hold_warned = True
        world_name = self._gz_world_name or read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
        now = time.monotonic()
        if (now - self._drop_hold_gz_last_check) >= self._drop_hold_gz_check_interval:
            self._drop_hold_gz_last_check = now
            env_prefix = build_gz_env(resolve_gz_partition(self.gz_partition))
            try:
                result = subprocess.run(
                    ["bash", "-lc", f"{env_prefix}gz service -l"],
                    text=True,
                    capture_output=True,
                    timeout=2.5,
                )
                candidates = [line.strip() for line in (result.stdout or "").splitlines() if line.strip()]
                wanted = [
                    f"/world/{world_name}/set_entity_pose",
                    f"/world/{world_name}/set_pose",
                ]
                for name in wanted:
                    if name in candidates:
                        self._drop_hold_gz_service_name = name
                        break
                if not self._drop_hold_gz_service_name:
                    for name in candidates:
                        if name.endswith("/set_entity_pose") or name.endswith("/set_pose"):
                            if f"/world/{world_name}/" in name:
                                self._drop_hold_gz_service_name = name
                                break
                if self._drop_hold_gz_service_name:
                    self._emit_log(f"[PHYSICS][HOLD] gz_service={self._drop_hold_gz_service_name}")
                elif not self._drop_hold_gz_warned:
                    self._emit_log("[PHYSICS][HOLD] gz service no encontrado (set_entity_pose/set_pose)")
                    self._drop_hold_gz_warned = True
            except Exception as exc:
                if not self._drop_hold_gz_warned:
                    self._emit_log(f"[PHYSICS][HOLD] gz service list error: {exc}")
                    self._drop_hold_gz_warned = True
        service_name = self._drop_hold_gz_service_name
        positions = {}
        poses = self._read_world_pose_info(world_name)
        if poses:
            for pose in poses:
                name = pose.get("name")
                pos = pose.get("position") or {}
                if not isinstance(name, str):
                    continue
                try:
                    positions[name] = (
                        float(pos.get("x") or 0.0),
                        float(pos.get("y") or 0.0),
                        float(pos.get("z") or 0.0),
                    )
                except Exception:
                    continue
        held = 0
        service_candidates = [
            service_name,
            f"/world/{world_name}/set_entity_pose",
            f"/world/{world_name}/set_pose",
        ]
        service_candidates = [s for s in service_candidates if s]
        for name in DROP_OBJECT_NAMES:
            target = self._drop_spawn_positions.get(name)
            if not target:
                continue
            current = positions.get(name)
            if current and current[2] >= (target[2] - 0.05):
                continue
            tx, ty, tz = target
            req = (
                f'name: "{name}" '
                f"position {{x: {float(tx)} y: {float(ty)} z: {float(tz)}}} "
                "orientation {w: 1}"
            )
            env_prefix = build_gz_env(resolve_gz_partition(self.gz_partition))
            ok = False
            for svc in service_candidates:
                cmd = (
                    f"{env_prefix}gz service -s {svc} "
                    "--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                    f"--timeout 800 --req '{req}'"
                )
                try:
                    res = subprocess.run(
                        ["bash", "-lc", cmd],
                        text=True,
                        capture_output=True,
                        timeout=1.6,
                    )
                except Exception as exc:
                    if not self._drop_hold_gz_warned:
                        self._emit_log(f"[PHYSICS][HOLD] gz service error: {exc}")
                        self._drop_hold_gz_warned = True
                    break
                if res.returncode == 0:
                    ok = True
                    if self._drop_hold_gz_service_name != svc:
                        self._drop_hold_gz_service_name = svc
                        self._emit_log(f"[PHYSICS][HOLD] gz_service={svc}")
                    break
            if not ok and not self._drop_hold_gz_warned:
                msg = (res.stderr or res.stdout or "").strip() if "res" in locals() else ""
                self._emit_log(f"[PHYSICS][HOLD] gz service rc!=0 {msg}")
                self._drop_hold_gz_warned = True
            if ok:
                held += 1
        now = time.monotonic()
        if held and (now - self._drop_hold_log_ts) > 2.0:
            self._emit_log(f"[PHYSICS][HOLD] held={held} reason={reason} via=gz")
            self._drop_hold_log_ts = now

    def _drop_hold_tick(self) -> None:
        if self._closing:
            return
        if not self._gz_running:
            return
        if not self._drop_hold_enabled:
            return
        if self._objects_release_done:
            return
        self._maybe_hold_drop_objects("timer")

    def _nudge_drop_objects(self, reason: str) -> None:
        if not ROS_AVAILABLE or SetEntityPose is None:
            self._emit_log("[PHYSICS][NUDGE] no disponible (SetEntityPose)")
            return
            self._ensure_moveit_node()
        if self._moveit_node is None:
            self._emit_log("[PHYSICS][NUDGE] nodo ROS no listo")
            return
        world_name = self._gz_world_name or read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
        service_name = self._set_pose_service_name or self._resolve_set_pose_service(world_name)
        if not service_name:
            self._emit_log("[PHYSICS][NUDGE] servicio SetEntityPose no encontrado")
            return
        self._set_pose_service_name = service_name
        client = self._moveit_node.create_client(SetEntityPose, service_name)
        if not client.wait_for_service(timeout_sec=0.6):
            self._emit_log(f"[PHYSICS][NUDGE] servicio no disponible: {service_name}")
            return
        positions = get_object_positions()
        nudged = 0
        for name in DROP_OBJECT_NAMES:
            pos = positions.get(name)
            if not pos:
                continue
            x, y, z = pos
            if z < NUDGE_DROP_Z_MIN:
                continue
            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = GzEntity.MODEL if GzEntity else 2
            req.pose.position.x = float(x)
            req.pose.position.y = float(y)
            req.pose.position.z = float(z - NUDGE_DROP_DZ)
            req.pose.orientation.w = 1.0
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=0.6)
            if future.done() and future.result() and future.result().success:
                nudged += 1
        self._emit_log(f"[PHYSICS][NUDGE] nudged={nudged} reason={reason}")

    def _start_release_service(self):
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("Release service bloqueado: ERROR_FATAL activo", error=True)
            self._emit_log_throttled(
                "SAFETY:start_release_service:ERROR_FATAL",
                "[SAFETY] Release service bloqueado: ERROR_FATAL activo",
            )
            return
        panel_launch_control.start_release_service(self)

    def _start_world_tf_publisher(self, world_name: str) -> None:
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("world_tf bloqueado: ERROR_FATAL activo", error=True)
            self._emit_log_throttled(
                "SAFETY:start_world_tf:ERROR_FATAL",
                "[SAFETY] world_tf bloqueado: ERROR_FATAL activo",
            )
            return
        panel_launch_control.start_world_tf_publisher(self, world_name)
        return
    def _stop_world_tf_publisher(self) -> None:
        panel_launch_control.stop_world_tf_publisher(self)
        return
    def _stop_gazebo(self):
        panel_launch_control.stop_gazebo(self)
        return
    def _start_robot_state_publisher(self):
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("RSP bloqueado: ERROR_FATAL activo", error=True)
            self._emit_log_throttled(
                "SAFETY:start_rsp:ERROR_FATAL",
                "[SAFETY] RSP bloqueado: ERROR_FATAL activo",
            )
            return
        panel_launch_control.start_robot_state_publisher(self)
        return
    def _start_bridge(self):
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("Bridge bloqueado: ERROR_FATAL activo", error=True)
            self._emit_log_throttled(
                "SAFETY:start_bridge:ERROR_FATAL",
                "[SAFETY] Bridge bloqueado: ERROR_FATAL activo",
            )
            return
        panel_launch_control.start_bridge(self)
        return
    def _spawn_controllers_async(self):
        panel_controllers.spawn_controllers_async(self)

    def _stop_bridge(self):
        panel_launch_control.stop_bridge(self)
        return
    def _start_moveit(self):
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("MoveIt bloqueado: ERROR_FATAL activo", error=True)
            self._emit_log_throttled(
                "SAFETY:start_moveit:ERROR_FATAL",
                "[SAFETY] MoveIt bloqueado: ERROR_FATAL activo",
            )
            return
        panel_launch_control.start_moveit(self)
        return
    def _stop_moveit(self):
        panel_launch_control.stop_moveit(self)
        return
    def _wait_for_moveit_ready(self):
        wait_for_moveit_ready(self)

    def _start_moveit_bridge(self):
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("MoveIt bridge bloqueado: ERROR_FATAL activo", error=True)
            self._emit_log_throttled(
                "SAFETY:start_moveit_bridge:ERROR_FATAL",
                "[SAFETY] MoveIt bridge bloqueado: ERROR_FATAL activo",
            )
            return
        panel_launch_control.start_moveit_bridge(self)
        return
    def _stop_moveit_bridge(self):
        panel_launch_control.stop_moveit_bridge(self)
        self._moveit_bridge_stop_grace_until = time.monotonic() + 6.0
        return
    def _clear_moveit_bridge_launching(self):
        self._moveit_bridge_launching = False
        self._moveit_bridge_launch_start = 0.0
        self._set_launching_style(self.btn_moveit_bridge_start, False)
        self.signal_refresh_controls.emit()

    def _kill_proc(self, proc, label: str):
        terminate_process(proc, label, log_fn=self._log)

    def _proc_alive(self, proc) -> bool:
        return proc is not None and proc.poll() is None


    def _save_home_from_sliders(self):
        """Lee los valores actuales de los sliders y guarda como nueva pose HOME."""
        from .panel_utils import save_home_pose
        joint_values = []
        for slider in self.joint_sliders:
            deg = slider.value() / JOINT_SLIDER_SCALE
            rad = math.radians(deg)
            joint_values.append(rad)
        save_home_pose(joint_values)
        self._set_status("Pose HOME guardada", error=False)

    def _rosbag_running(self) -> bool:
        return self._proc_alive(self.bag_proc)

    def _start_bag(self):
        self._log_button("Start bag")
        if self._proc_alive(self.bag_proc):
            self._set_status("Bag ya activo", error=True)
            return
        if not self._bridge_running:
            self._log_warning("Bag en espera: bridge no activo")
            self._set_status("Bag en espera: bridge no activo", error=False)
            set_led(self.led_bag, "warn")
            return
        name = self.bag_name.text().strip() or f"demo_{int(time.time())}"
        topics_raw = self.bag_topics.text().strip()
        self._log(f"[BAG] Nombre: {name}, Tópicos raw: {topics_raw}")
        topics, invalid = parse_ros_topics(topics_raw)
        if invalid:
            self._log_error(f"Tópicos inválidos: {invalid}")
            self._set_status("Tópicos inválidos", error=True)
            set_led(self.led_bag, "error")
            return
        if not topics:
            self._log_error("No hay tópicos para grabar")
            self._set_status("No hay tópicos", error=True)
            set_led(self.led_bag, "error")
            return
        ensure_dir(BAGS_DIR)
        outdir = os.path.join(BAGS_DIR, name)
        if os.path.exists(outdir):
            suffix = 1
            while os.path.exists(f"{outdir}_{suffix}"):
                suffix += 1
            outdir = f"{outdir}_{suffix}"
        self._log(f"[BAG] Directorio salida: {outdir}")
        self._set_status(f"Grabando bag en {outdir}…")
        set_led(self.led_bag, "warn")

        def worker():
            cmd = (
                bash_preamble(self.ws_dir)
                + "ros2 bag record -o "
                + shlex.quote(outdir)
                + " --topics "
                + " ".join(shlex.quote(t) for t in topics)
            )
            try:
                self.bag_proc = subprocess.Popen([
                    "bash",
                    "-lc",
                    cmd,
                ], preexec_fn=os.setsid)
                self._bag_running = True
                self._started_bag = True
                self._ui_set_status(f"Bag grabando → {outdir}")
                self.signal_set_led.emit(self.led_bag, "on")
                self.signal_refresh_controls.emit()
            except Exception as exc:
                self._ui_set_status(f"Error al grabar bag: {exc}", error=True)
                self.signal_set_led.emit(self.led_bag, "error")
                self.signal_refresh_controls.emit()

        self._run_async(worker)

    def _stop_bag(self):
        self._log_button("Stop bag")
        self._set_status("Deteniendo bag…")
        self._kill_proc(self.bag_proc, "ros2 bag record")
        self.bag_proc = None
        self._bag_running = False
        set_led(self.led_bag, "off")

    def _refresh_status_sync(self):
        """Chequeo síncrono de estado al startup."""
        clock_ok, _ = self._clock_status()
        gz_state = self._gazebo_state()
        gz_ok = gz_state == "GAZEBO_READY"
        br_ok = self._bridge_running or self._pose_info_active()
        bag_ok = self._rosbag_running()
        ctrl_ok = self._ros2_control_available()
        moveit_ok = self._moveit_ready()
        moveit_bridge_ok = self._moveit_bridge_detected()
        self._apply_status(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)

    def _refresh_status_async(self):
        if self._status_check_inflight:
            return
        self._status_check_inflight = True

        def worker():
            if self._managed_mode and self._external_state_active():
                state = self._system_state
                external_state = (self._external_state or "").upper()
                gz_ok = external_state not in ("BOOT", "BOOTING", "WAITING_GAZEBO")
                br_ok = external_state not in ("BOOT", "BOOTING", "WAITING_GAZEBO", "WAITING_BRIDGE")
                clock_ok = gz_ok
                ctrl_ok = state not in (
                    SystemState.BOOT,
                    SystemState.WAITING_GAZEBO,
                    SystemState.WAITING_CONTROLLERS,
                )
                moveit_ok = self._moveit_ready()
                moveit_bridge_ok = self._moveit_bridge_detected()
                bag_ok = self._bag_running
                self.status_updated.emit(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)
                self._status_check_inflight = False
                return
            clock_ok, _ = self._clock_status()
            gz_state = self._gazebo_state()
            gz_ok = gz_state == "GAZEBO_READY"
            _br_self = self._bridge_running
            _pi_active = self._pose_info_active()
            br_ok = _br_self or _pi_active
            bag_ok = self._rosbag_running()
            ctrl_ok = self._ros2_control_available() if br_ok else False
            moveit_ok = self._moveit_ready()
            moveit_bridge_ok = self._moveit_bridge_detected()
            self.status_updated.emit(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)
            self._status_check_inflight = False

        self._run_async(worker)

    def _apply_status(
        self,
        gz_ok: bool,
        br_ok: bool,
        clock_ok: bool,
        bag_ok: bool,
        ctrl_ok: bool,
        moveit_ok: bool,
        moveit_bridge_ok: bool,
    ):
        gz_state = self._gz_state or "GAZEBO_OFF"
        if gz_state == "GAZEBO_READY":
            set_led(self.led_gz, "on")
        elif gz_state in ("GAZEBO_STARTING", "GAZEBO_DEGRADED", "GAZEBO_MONITOR_BUG"):
            set_led(self.led_gz, "warn")
        else:
            set_led(self.led_gz, "off")
        set_led(self.led_bridge, "on" if br_ok else "off")
        set_led(self.led_clock, "on" if clock_ok else "off")
        set_led(self.led_bag, "on" if bag_ok else "off")
        set_led(self.led_ros2, "on" if ctrl_ok else "off")
        set_led(self.led_ur5, "on" if gz_ok else "off")
        if self._moveit_state == MoveItState.STARTING:
            set_led(self.led_moveit, "warn")
        elif self._moveit_state == MoveItState.WAITING_MOVEIT_READY:
            set_led(self.led_moveit, "warn")
        elif self._moveit_state == MoveItState.READY:
            set_led(self.led_moveit, "on")
        elif self._moveit_state == MoveItState.ERROR:
            set_led(self.led_moveit, "error")
        else:
            set_led(self.led_moveit, "off")
        set_led(self.led_moveit_bridge, "on" if moveit_bridge_ok else "off")
        prev_bridge = self._bridge_running
        self._gz_running = gz_ok
        self._bridge_running = br_ok
        self._bag_running = bag_ok
        self._moveit_running = moveit_ok
        self._moveit_bridge_running = moveit_bridge_ok
        if br_ok and not prev_bridge:
            # Selective init without critical TF deadlines (same as panel_launchers.py)
            self._ensure_pose_subscription()
            self._start_pose_info_watch()
            self._start_tf_ready_timer()
            QTimer.singleShot(600, self._auto_connect_camera)
        if self._moveit_state == MoveItState.OFF and moveit_ok:
            self._moveit_state = MoveItState.READY
            if self._moveit_bridge_detected():
                self._moveit_state_reason = "moveit_bridge detectado"
            else:
                self._moveit_state_reason = "move_group detectado"
            set_led(self.led_moveit, "on")
        if self._moveit_state == MoveItState.READY and not moveit_ok:
            self._moveit_state = MoveItState.WAITING_MOVEIT_READY
            self._moveit_state_reason = "move_group no responde"
            set_led(self.led_moveit, "warn")
        if self._moveit_state in (MoveItState.STARTING, MoveItState.WAITING_MOVEIT_READY) and moveit_ok:
            self._moveit_state = MoveItState.READY
            self._moveit_state_reason = "move_group listo"
            set_led(self.led_moveit, "on")
        self._update_moveit_status_label()
        summary = []
        if gz_state in ("GAZEBO_STARTING", "GAZEBO_DEGRADED", "GAZEBO_MONITOR_BUG"):
            summary.append("GZ:starting")
        else:
            summary.append(f"GZ:{'on' if gz_ok else 'off'}")
        summary.append(f"BR:{'on' if br_ok else 'off'}")
        summary.append(f"CLK:{'on' if clock_ok else 'off'}")
        summary.append(f"BAG:{'on' if bag_ok else 'off'}")
        summary.append(f"CTRL:{'on' if ctrl_ok else 'off'}")
        summary.append(f"MVT:{'on' if moveit_ok else 'off'}")
        summary.append(f"MBR:{'on' if moveit_bridge_ok else 'off'}")
        self.status_lbl.setText(" · ".join(summary))
        self._update_system_stats()
        self._refresh_controls()

    def _moveit_topics_ready(self) -> bool:
        return moveit_topics_ready(self)

    def _moveit_status_ready(self) -> bool:
        return moveit_status_ready(self)

    def _moveit_action_ready(self) -> bool:
        return moveit_action_ready(self)

    def _list_topic_names(self) -> List[str]:
        if not self.ros_worker or not self.ros_worker.node_ready():
            return []
        try:
            return self.ros_worker.list_topic_names()
        except Exception:
            return []

    def _topic_has_any_publishers(self, topics: List[str]) -> bool:
        if not self.ros_worker or not self.ros_worker.node_ready():
            return False
        for topic in topics:
            if self.ros_worker.topic_has_publishers(topic):
                return True
        return False

    def _world_frame_last_first(self, fallback: Optional[str] = None) -> str:
        return fallback or self._last_selection_frame or WORLD_FRAME or "world"

    def _world_frame_config_first(self) -> str:
        return WORLD_FRAME or self._last_selection_frame or "world"

    def _follow_joint_traj_ready(self) -> bool:
        if not ROS_AVAILABLE or ActionClient is None or FollowJointTrajectory is None:
            return False
        if self._moveit_node is None:
            return False
        traj_topic = self._select_traj_topic()
        action_name = self._resolve_traj_action_name(traj_topic, allow_fallback=True)
        if not action_name:
            return False
        if self._traj_action_client is None or self._traj_action_name != action_name:
            self._traj_action_client = self._get_action_client(
                action_name,
                FollowJointTrajectory,
                log_ctx="follow_traj",
            )
        client = self._traj_action_client
        if client is None:
            return False
        return self._wait_action_server(
            client,
            timeout_sec=0.2,
            log_ctx="follow_traj",
            action_name=action_name,
        )

    def _moveit_bridge_detected(self) -> bool:
        if self._proc_alive(self.moveit_bridge_proc):
            self._moveit_bridge_detected_cache = True
            self._moveit_bridge_detected_ts = time.monotonic()
            return True
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False
        now = time.monotonic()
        if (
            STATUS_TOPIC_CACHE_SEC > 0.0
            and (now - self._moveit_bridge_detected_ts) < STATUS_TOPIC_CACHE_SEC
        ):
            return self._moveit_bridge_detected_cache
        pose_ready = (
            self.ros_worker.topic_has_subscribers(MOVEIT_POSE_TOPIC)
            or self.ros_worker.topic_has_subscribers("/grasp_pose")
        )
        result_ready = self.ros_worker.topic_has_publishers("/desired_grasp/result")
        detected = pose_ready and result_ready
        self._moveit_bridge_detected_cache = bool(detected)
        self._moveit_bridge_detected_ts = now
        return detected

    def _move_group_ready(self) -> bool:
        return self._moveit_action_ready() and self._moveit_status_ready() and self._follow_joint_traj_ready()

    def _moveit_ready(self) -> bool:
        if self._moveit_bridge_detected():
            return True
        return self._move_group_ready()

    def _update_moveit_status_label(self) -> None:
        if self.lbl_moveit_status is not None:
            state_label = self._moveit_state.value
            self.lbl_moveit_status.setText("MoveIt")
            reason = self._moveit_state_reason or state_label
            self.lbl_moveit_status.setToolTip(f"{state_label}: {reason}")
        if self.lbl_moveit_bridge_status is not None:
            bridge_label = "ON" if self._moveit_bridge_running else "OFF"
            self.lbl_moveit_bridge_status.setText("MoveIt bridge")

    def _update_system_stats(self):
        """Actualizar labels de CPU/RAM/Load, tolerando ausencia de psutil."""
        cpu_txt = "CPU  --"
        ram_txt = "RAM  --"
        load_txt = "Load  --"
        cpu_alert = False
        ram_alert = False
        load_alert = False
        stale_count = 0
        cores = max(1, os.cpu_count() or 1)
        try:
            if psutil:
                cpu = psutil.cpu_percent(interval=None)
                vm = psutil.virtual_memory()
                used_gb = vm.used / (1024 ** 3)
                total_gb = vm.total / (1024 ** 3)
                ram_txt = f"RAM  {used_gb:.1f}/{total_gb:.1f} GB ({vm.percent:.0f}%)"
                cpu_txt = f"CPU  {cpu:.0f}%"
                cpu_alert = cpu >= 85
                ram_alert = vm.percent >= 90
            else:
                # Fallback simple usando loadavg
                load1, load5, load15 = os.getloadavg()
                cores = max(1, os.cpu_count() or 1)
                cpu_txt = f"CPU  {load1 / cores * 100:.0f}%"
        except Exception as exc:
            _log_exception("update_system_stats cpu/ram", exc)
        try:
            load1, load5, load15 = os.getloadavg()
            load_txt = f"Load  {load1:.2f} {load5:.2f} {load15:.2f}"
            if not psutil:
                cores = max(1, os.cpu_count() or 1)
            load_alert = load1 >= max(4.0, cores * 1.5)
        except Exception as exc:
            _log_exception("update_system_stats loadavg", exc)
        try:
            stale_count, _stale_hint = self._detect_stale_processes()
        except Exception as exc:
            _log_exception("update_system_stats stale procs", exc)
            stale_count = 0
        health_alert = stale_count > 0
        health_txt = "Proc.Zombis activos" if health_alert else "Todo OK"
        self._set_stat_label(self.sys_cpu_lbl, cpu_txt, cpu_alert)
        self._set_stat_label(self.sys_ram_lbl, ram_txt, ram_alert)
        self._set_stat_label(self.sys_load_lbl, load_txt, load_alert)
        self._set_stat_label(self.sys_health_lbl, health_txt, health_alert)

    def _set_stat_label(self, label: QLabel, text: str, alert: bool):
        color = "#dc2626" if alert else "#0f172a"
        label.setStyleSheet(f"font-size:11px; color:{color};")
        label.setText(text)

    def _known_process_pids(self) -> Set[int]:
        pids = {os.getpid()}
        if psutil:
            try:
                for parent in psutil.Process(os.getpid()).parents():
                    pids.add(parent.pid)
            except Exception as exc:
                _log_exception("list parent processes", exc)
        for proc in (
            self.gz_proc,
            self.bridge_proc,
            self.bag_proc,
            self.moveit_proc,
            self.moveit_bridge_proc,
            self.release_service_proc,
            self.rsp_proc,
        ):
            if proc is None:
                continue
            try:
                if proc.pid:
                    pids.add(proc.pid)
            except Exception as exc:
                _log_exception("read proc pid", exc)
                continue
        return pids

    def _list_stale_processes(self) -> List[Tuple[int, str, str]]:
        """Listar procesos del proyecto que no pertenecen al panel actual."""
        if not psutil:
            return []
        ws_dir = os.path.realpath(self.ws_dir)
        grace_sec = max(0.0, STALE_PROCESS_GRACE_SEC)
        cutoff = self._panel_start_ts - grace_sec
        ignore_pids = self._known_process_pids()
        patterns = (
            "ur5_qt_panel",
            "ur5_tools",
            "ur5_moveit_bridge",
            "release_objects_service",
            "ur5_moveit_config",
            "gz-transport-topic",
            "ros_gz_bridge",
            "parameter_bridge",
            "gz sim",
            "gzserver",
            "gzclient",
            "ign gazebo",
            "robot_state_publisher",
            "controller_manager",
            "spawner",
            "move_group",
        )
        stale = []
        for proc in psutil.process_iter(["pid", "cmdline", "name", "status"]):
            try:
                pid = proc.info["pid"]
                if pid in ignore_pids:
                    continue
                try:
                    if proc.create_time() >= cutoff:
                        continue
                except Exception as exc:
                    _log_exception("read proc create_time", exc)
                cmdline = proc.info.get("cmdline") or []
                cmd = " ".join(cmdline) if cmdline else (proc.info.get("name") or "")
                cmd = cmd.strip()
                if not cmd:
                    continue
                cmd_lower = cmd.lower()
                if ws_dir in cmd:
                    stale.append((pid, cmd, proc.info.get("status") or ""))
                    continue
                if any(pat in cmd_lower for pat in patterns):
                    stale.append((pid, cmd, proc.info.get("status") or ""))
                    continue
                if proc.info.get("status") == psutil.STATUS_ZOMBIE and "ros2" in cmd_lower:
                    stale.append((pid, cmd, proc.info.get("status") or ""))
            except Exception as exc:
                _log_exception("scan stale process", exc)
                continue
        return stale

    def _detect_stale_processes(self) -> Tuple[int, str]:
        """Detecta procesos del proyecto que no pertenecen al panel actual."""
        stale = self._list_stale_processes()
        if not stale:
            return 0, ""
        sample_cmd = stale[0][1]
        hint = sample_cmd.split()[0] if sample_cmd else ""
        return len(stale), hint

    @pyqtSlot()
    def _refresh_controls(self):
        if self._closing:
            return
        if self._gz_launching and self._clear_launching_if_timeout("Gazebo", self._gz_launch_start, GZ_LAUNCH_TIMEOUT_SEC):
            self._gz_launching = False
            self._gz_launch_start = 0.0
            self._set_launching_style(self.btn_gz_start, False)
        if self._bridge_launching and self._clear_launching_if_timeout("Bridge", self._bridge_launch_start, BRIDGE_LAUNCH_TIMEOUT_SEC):
            self._bridge_launching = False
            self._bridge_launch_start = 0.0
            self._set_launching_style(self.btn_bridge_start, False)
        if self._moveit_launching and self._clear_launching_if_timeout("MoveIt", self._moveit_launch_start, MOVEIT_LAUNCH_TIMEOUT_SEC):
            self._moveit_launching = False
            self._moveit_launch_start = 0.0
            self._set_launching_style(self.btn_moveit_start, False)
        if self._moveit_bridge_launching and self._clear_launching_if_timeout("MoveIt bridge", self._moveit_bridge_launch_start, MOVEIT_BRIDGE_LAUNCH_TIMEOUT_SEC):
            self._moveit_bridge_launching = False
            self._moveit_bridge_launch_start = 0.0
            self._set_launching_style(self.btn_moveit_bridge_start, False)
        self._update_system_state()
        self._check_critical_timeouts()
        if self._fatal_latched:
            return
        effective_state, effective_reason = self._effective_system_state()
        apply_ui_state(self, effective_state, effective_reason)
        self._maybe_auto_run_pick_demo()

    def _maybe_auto_run_pick_demo(self) -> None:
        if not self._auto_pick_demo_enabled:
            return
        if self._auto_pick_demo_done >= self._auto_pick_demo_attempts:
            return
        now = time.time()
        if (now - self._auto_pick_demo_last_try_ts) < 4.0:
            return
        if self._manual_inflight or self._script_motion_active or self._robot_test_active:
            self._emit_log(
                f"[AUTO_PICK_DEMO] bloqueado: manual_inflight={self._manual_inflight} "
                f"script_active={self._script_motion_active} test_active={self._robot_test_active}"
            )
            self._auto_pick_demo_last_try_ts = now
            return
        if self._system_state in (SystemState.BOOT, SystemState.WAITING_GAZEBO, SystemState.ERROR_FATAL):
            self._emit_log(
                f"[AUTO_PICK_DEMO] bloqueado: system_state={self._system_state.value}"
            )
            self._auto_pick_demo_last_try_ts = now
            return
        # Usar _state_ready_basic() que evalúa el estado efectivo del sistema
        # en lugar de los flags individuales, ya que en modo no-managed puede
        # que _controllers_ok no se actualice hasta que el bridge esté activo.
        system_ok = self._state_ready_basic()
        tf_ok = self._tf_ready_state or bool(self._ee_frame_effective)
        if not system_ok or not tf_ok:
            self._emit_log(
                f"[AUTO_PICK_DEMO] no listo: state={self._system_state.value} "
                f"ctrl_ok={self._controllers_ok} tf_ok={tf_ok} "
                f"ee={self._ee_frame_effective!r} system_ok={system_ok}"
            )
            self._auto_pick_demo_last_try_ts = now
            return
        self._auto_pick_demo_last_try_ts = now
        attempt = self._auto_pick_demo_done + 1
        self._emit_log(
            f"[AUTO_PICK_DEMO] trigger intento {attempt}/{self._auto_pick_demo_attempts}"
        )
        self._auto_pick_demo_done = attempt
        self._run_pick_demo()

    def _wait_for_state_change(self, timeout_sec: float) -> bool:
        self._state_event.clear()
        return self._state_event.wait(timeout_sec)

    def _schedule_controller_check(self) -> None:
        if not self._bridge_running:
            return
        now = time.time()
        if self._controller_check_inflight or (now - self._last_controller_check) < CONTROLLER_CHECK_INTERVAL_SEC:
            return
        self._controller_check_inflight = True

        def worker():
            ok, reason = self._controllers_ready()
            changed = (ok != self._controllers_ok) or (reason != self._controllers_reason)
            now = time.time()
            grace_base = self._controller_spawn_last_start or self._bridge_start_ts or 0.0
            in_grace = grace_base and (now - grace_base) < CONTROLLER_START_GRACE_SEC
            gazebo_not_ready = str(reason).startswith("gazebo_not_ready")
            # FASE 2: No degradar controllers_ok durante TEST para evitar reset a BOOT.
            test_running = getattr(self, "_robot_test_active", False)
            if ok:
                if self._controllers_state != "READY":
                    self._emit_log("[CTRL] state=READY")
                    self._controllers_state = "READY"
                self._controllers_ok = True
                self._controllers_reason = reason
            elif test_running and self._controllers_ok:
                # FASE 2: Durante TEST, mantener controllers_ok=True ante drop transitorio.
                self._emit_log_throttled(
                    "ctrl_gate_test_hold",
                    f"[CTRL_GATE] controllers transient fail during TEST; holding ok ({reason})",
                    min_interval=2.0,
                )
            else:
                if in_grace or gazebo_not_ready:
                    if self._controllers_state != "STARTING":
                        self._emit_log("[CTRL] state=STARTING")
                        self._controllers_state = "STARTING"
                    # Mantener controllers_ok=false pero sin degradar el estado global.
                    self._controllers_ok = False
                    self._controllers_reason = reason
                else:
                    if self._controllers_state != "ERROR":
                        self._emit_log("[CTRL] state=ERROR")
                        self._controllers_state = "ERROR"
                    self._controllers_ok = False
                    self._controllers_reason = reason
            self._last_controller_check = time.time()
            self._controller_check_inflight = False
            if changed:
                if ok:
                    self._emit_log(f"[CTRL] controllers_ready=true reason={reason}")
                    self._emit_log(f"[CTRL_GATE] controllers_ready=true reason={reason}")
                else:
                    if in_grace or gazebo_not_ready:
                        self._emit_log(f"[CTRL] controllers_ready=false reason=starting ({reason})")
                        self._emit_log(f"[CTRL_GATE] controllers_ready=false reason=starting ({reason})")
                    else:
                        self._emit_log(f"[CTRL] controllers_ready=false reason={reason}")
                        self._emit_log(f"[CTRL_GATE] controllers_ready=false reason={reason}")
                        self._log_warning(f"[PICK] controladores no listos ({reason})")
            self.signal_controllers_ready.emit(ok)
            self.signal_refresh_controls.emit()

        self._run_async(worker)

    def _update_ui_state(self):
        """Inicializar UI: arranque manual desde los controles del panel."""
        self._gz_running = False
        self._bridge_running = False
        self._bag_running = False
        self._moveit_running = False
        self._moveit_bridge_running = False
        self._auto_joint2_move_done = False
        
        # Gazebo controls
        self.btn_gz_start.setEnabled(True)
        self.btn_gz_stop.setEnabled(False)
        self.btn_debug_joints.setEnabled(False)
        self.btn_debug_joints.setChecked(False)
        self.world_combo.setEnabled(True)
        self.mode_combo.setEnabled(True)
        self.btn_world_browse.setEnabled(True)
        
        # Bridge controls
        self.btn_bridge_start.setEnabled(False)
        self.btn_bridge_stop.setEnabled(False)
        self.bridge_presets.setEnabled(False)
        self.bridge_edit.setEnabled(False)
        self.btn_bridge_browse.setEnabled(False)
        
        # Bag controls
        self.btn_bag_start.setEnabled(False)
        self.btn_bag_stop.setEnabled(False)
        self.bag_name.setEnabled(False)
        self.bag_topics.setEnabled(False)

        # MoveIt controls
        self.btn_moveit_start.setEnabled(True)
        self.btn_moveit_stop.setEnabled(False)
        self.btn_moveit_bridge_start.setEnabled(False)
        self.btn_moveit_bridge_stop.setEnabled(False)
        
        # Cámara (deshabilitada hasta que bridge esté activo)
        self.camera_topic_combo.setEnabled(False)
        self.btn_camera_refresh.setEnabled(False)
        self.btn_camera_connect.setEnabled(False)
        if self.btn_calibrate is not None:
            self.btn_calibrate.setEnabled(False)
        if self.btn_release_objects is not None:
            self.btn_release_objects.setEnabled(False)
        if hasattr(self, "chk_trace_freeze"):
            self.chk_trace_freeze.setEnabled(False)
        if hasattr(self, "btn_trace_diag"):
            self.btn_trace_diag.setEnabled(False)
        if hasattr(self, "btn_copy_trace"):
            self.btn_copy_trace.setEnabled(False)
        if hasattr(self, "btn_save_episode"):
            self.btn_save_episode.setEnabled(False)
        
        # Control manual (deshabilitado hasta que bridge esté activo)
        self.btn_send_joints.setEnabled(False)
        self.joint_time.setEnabled(False)
        self.chk_auto_joints.setEnabled(False)
        for slider in self.joint_sliders:
            slider.setEnabled(False)

        # Botones de movimiento (bloqueados hasta bridge)
        self.btn_test_robot.setEnabled(False)
        self.btn_home.setEnabled(False)
        self.btn_table.setEnabled(False)
        self.btn_basket.setEnabled(False)
        self.btn_gripper.setEnabled(False)
        self.btn_pick_demo.setEnabled(False)
        self.btn_pick_object.setEnabled(False)
        
        # Debug y otros
        self.btn_debug_joints.setEnabled(True)
        self.btn_debug_logs.setEnabled(True)
        self.btn_kill_hard.setEnabled(False)
        self.btn_close_terminal.setEnabled(True)

    def _effective_mode(self) -> str:
        m = self.mode_combo.currentText().strip().lower()
        return effective_mode(m)

    def _apply_home_joint2_offset(self, retries: int = 2):
        """Mover a HOME ajustando joint2 un -20% (sentido negativo) al lanzar Gazebo."""
        if self._auto_joint2_move_done:
            return
        self._schedule_controller_check()
        if not self._bridge_running:
            if retries > 0:
                self._schedule_home_offset_retry(1500, retries - 1)
            return
        if not self._gz_running:
            if retries > 0:
                self._schedule_home_offset_retry(1500, retries - 1)
            return

        if not self._controllers_ok:
            if retries > 0:
                self._log("[AUTO] Controller manager no listo, reintentando en 2s")
                self._schedule_home_offset_retry(2000, retries - 1)
            else:
                self._log_warning("[AUTO] Controller manager no disponible (ajuste automático cancelado)")
            return

        home = load_home_pose()
        if len(home) < 6:
            self._log_warning("[AUTO] HOME inválido, no se aplica offset joint2")
            return

        target = list(home[:6])
        base = abs(target[1])
        if base < 1e-3:
            base = 0.25  # fallback pequeño si HOME era ~0
        offset = base * 0.20
        target[1] = -offset

        def worker():
            self._set_motion_lock(True)
            try:
                self._log(f"[AUTO] Ajuste joint2=-20% desde HOME -> {target[1]:.3f} rad")
                ok, info = self._publish_joint_trajectory(target, 3.0)
                if ok:
                    time.sleep(3.15)
                    self._auto_joint2_move_done = True
                    self._ui_set_status("AUTO: joint2 ajustado (-20% HOME)")
                else:
                    self._log_warning(f"[AUTO] Falló mover joint2 (-20%): {info}")
                    if retries > 0:
                        self.signal_schedule_home_offset.emit(2000, retries - 1)
            finally:
                self._set_motion_lock(False)
                recalc_object_states("manual_move")

        self._run_async(worker)

    @pyqtSlot(int, int)
    def _schedule_home_offset_retry(self, delay_ms: int, retries: int) -> None:
        QTimer.singleShot(delay_ms, lambda: self._apply_home_joint2_offset(retries=retries))

    def _get_home_joint_pose(self) -> List[float]:
        home = load_home_pose()
        if len(home) >= 6:
            return list(home[:6])
        return list(JOINT_HOME_POSE_RAD)

    def _get_gripper_force(self) -> float:
        """
        Obtener la fuerza actual del gripper RG2.
        
        Retorna:
            float: Fuerza en Newton (0.0 si no disponible)
        
        Nota: Esta es una implementación simplificada que estima la fuerza
        basada en la corriente del motor del gripper. En un sistema real,
        se usaría la retroalimentación de fuerza del gripper.
        """
        try:
            # Intentar obtener fuerza desde el estado del gripper
            # En MoveIt/ROS 2, esto vendría de:
            # - Topic: /rg2/gripper_cmd/gripper_force (si está disponible)
            # - Topic: /rg2/gripper_mimic_joint_follower/state
            # - Servicio: /rg2_controller/get_gripper_force (si existe)
            
            # Para ahora, implementamos heurística simple:
            # Si el gripper está "cerrado" y hay resistencia, estimamos fuerza
            
            if not hasattr(self, '_gripper_force_estimate'):
                self._gripper_force_estimate = 0.0
            
            # Lógica: Si el gripper se está moviendo hacia cierre pero encuentra resistencia,
            # la estimamos como contacto (fuerza > 0.1 N)
            # Este es un placeholder - en producción, leer desde ROS topic
            
            # TEMPORAL: Retornar fuerza ficticia (0.5 N cuando está cerrado)
            # TODO: Integrar con RG2_GRIPPER topic real
            if hasattr(self, '_gripper_is_closed'):
                if self._gripper_is_closed:
                    return 0.5  # 0.5 N cuando está cerrado (estimación)
            
            return 0.0  # Sin contacto
            
        except Exception as e:
            self._log(f"[GRIPPER] Error leyendo fuerza: {e}")
            return 0.0

    def _go_home(self):
        self._log_button("Go HOME")
        if not self._require_manual_ready("HOME"):
            return
        self._log("[ROBOT] Iniciando movimiento a HOME")
        self._set_status("Moviendo a HOME…")
        move_sec = float(self.joint_time.value()) if self.joint_time else 3.0
        self._set_motion_lock(True)

        def worker():
            try:
                ok, info = self._publish_joint_trajectory(self._get_home_joint_pose(), move_sec)
                if ok:
                    time.sleep(move_sec + 0.15)
                    self._ui_set_status("HOME ejecutado (JointTrajectory)")
                    self._log(f"[ROBOT] HOME: JointTrajectory en {info}")
                else:
                    self._ui_set_status(f"HOME falló: {info}", error=True)
                    self._log_warning(f"[ROBOT] HOME falló: {info}")
            finally:
                self._set_motion_lock(False)
                recalc_object_states("manual_move")

        self._run_async(worker)

    def _tfm_infer_grasp(self):
        tfm_infer(self)

    def _restore_infer_selection_snapshot(self, snapshot: object) -> None:
        if self._selected_object:
            return
        if not isinstance(snapshot, dict):
            return
        name = str(snapshot.get("name") or "").strip()
        if not name:
            return
        selected_px = snapshot.get("px")
        selected_world = snapshot.get("world")
        selected_base = snapshot.get("base")
        self._selected_object = name
        self._selected_px = tuple(selected_px) if selected_px is not None else None
        self._selected_world = tuple(selected_world) if selected_world is not None else None
        self._selected_base = tuple(selected_base) if selected_base is not None else None
        self._selected_base_frame = str(snapshot.get("base_frame") or self._business_base_frame())
        snap_ts = float(snapshot.get("timestamp", 0.0) or 0.0)
        if snap_ts > 0.0:
            self._selection_timestamp = snap_ts
            if not self._selection_last_user_name:
                self._selection_last_user_name = name
            if not self._selection_last_user_ts:
                self._selection_last_user_ts = snap_ts
        self._emit_log(
            "[TFM][INFER] restored_selection_from_snapshot "
            f"name={name} px={self._selected_px if self._selected_px is not None else 'n/a'}"
        )

    def _handle_infer_result(self, result: Dict[str, object]) -> None:
        self._tfm_infer_inflight = False
        if not result.get("ok"):
            err = result.get("error") or "infer_grasp sin salida válida"
            self._set_status(f"TFM: inferencia fallida ({err})", error=True)
            infer_ms = float(result.get("infer_ms", 0.0) or 0.0)
            total_ms = float(result.get("total_ms", 0.0) or 0.0)
            self._audit_append(
                "logs/infer.log",
                f"[TFM] infer_end session={self._infer_session_id} status=FAIL "
                f"infer_ms={infer_ms:.2f} total_ms={total_ms:.2f} err={err}",
            )
            return
        pred = result.get("pred") if isinstance(result.get("pred"), dict) else None
        if not pred:
            self._set_status("TFM: salida inválida", error=True)
            self._audit_append("logs/infer.log", "[TFM] infer_end status=FAIL err=salida_invalida")
            return
        self._last_grasp_px = {
            "cx": float(pred.get("cx", 0.0)),
            "cy": float(pred.get("cy", 0.0)),
            "w": float(pred.get("w", 0.0)),
            "h": float(pred.get("h", 0.0)),
            "angle_deg": float(pred.get("angle_deg", 0.0)),
        }
        self._last_grasp_source = "infer_model"
        self._last_grasp_frame = self.camera_topic or "image"
        self._last_infer_image_path = str(result.get("image_path") or "")
        self._last_infer_output_path = str(result.get("out_path") or "")
        self._perf_infer_ms = float(result.get("infer_ms", 0.0))
        self._perf_total_ms = float(result.get("total_ms", 0.0))
        self._push_history(self._perf_infer_hist, self._perf_infer_ms, max_len=20)
        self._push_history(self._perf_total_hist, self._perf_total_ms, max_len=20)
        frame_w = int(result.get("frame_w", 0))
        frame_h = int(result.get("frame_h", 0))
        self._last_grasp_world = self._compute_world_grasp(frame_w, frame_h)
        self._last_grasp_base = None
        if self._last_grasp_world:
            base_coords = self._ensure_base_coords(
                (
                    float(self._last_grasp_world.get("x", 0.0)),
                    float(self._last_grasp_world.get("y", 0.0)),
                    float(self._last_grasp_world.get("z", 0.0)),
                ),
                self._world_frame_config_first(),
                timeout_sec=0.35,
            )
            if base_coords is not None:
                self._last_grasp_base = {
                    "x": float(base_coords[0]),
                    "y": float(base_coords[1]),
                    "z": float(base_coords[2]),
                    "yaw_deg": float(self._last_grasp_world.get("yaw_deg", 0.0) or 0.0),
                }
        self._restore_infer_selection_snapshot(result.get("selection_snapshot"))
        self._refresh_cornell_metrics(frame_w, frame_h)
        self._sync_tfm_module_grasp_state()
        self._refresh_grasp_overlay_now()
        self._refresh_science_ui()
        self._set_status("TFM: grasp inferido", error=False)
        audit_payload = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "status": "OK",
            "source": self._last_grasp_source,
            "grasp": self._last_grasp_px,
            "grasp_base": self._last_grasp_base,
            "frame": self._last_grasp_frame,
            "cornell": self._last_cornell,
            "cornell_reason": self._last_cornell_reason,
            "perf": {
                "infer_ms": self._perf_infer_ms,
                "total_ms": self._perf_total_ms,
            },
            "frame_info": {
                "w": int(result.get("frame_w", 0) or 0),
                "h": int(result.get("frame_h", 0) or 0),
                "ts": result.get("frame_ts"),
            },
            "artifacts": {
                "image_path": self._last_infer_image_path or None,
                "grasp_path": self._last_infer_output_path or None,
            },
        }
        self._audit_write_json("artifacts/grasp_last.json", audit_payload)
        self._audit_append(
            "logs/infer.log",
            f"[TFM] infer_end session={self._infer_session_id} status=OK "
            f"infer_ms={self._perf_infer_ms:.2f} total_ms={self._perf_total_ms:.2f} "
            f"grasp={self._last_grasp_px} cornell={self._last_cornell} "
            f"cornell_reason={self._last_cornell_reason!r}",
        )

    def _sync_tfm_module_grasp_state(self) -> None:
        if not self.tfm_module or not self._last_grasp_px:
            return
        try:
            from tfm_grasping.geometry import Grasp2D

            grasp = Grasp2D(
                center_x=self._last_grasp_px["cx"],
                center_y=self._last_grasp_px["cy"],
                angle_rad=math.radians(self._last_grasp_px["angle_deg"]),
                width_px=self._last_grasp_px["w"],
                quality=0.0,
                depth_m=None,
                frame_id=self._last_grasp_frame,
            )
            if hasattr(self.tfm_module, "set_last_grasp"):
                self.tfm_module.set_last_grasp(grasp)
            else:
                self.tfm_module._last_grasp = grasp  # type: ignore[attr-defined]
        except Exception as exc:
            self._log_warning(f"[TFM] set_last_grasp error: {exc}")

    def _tfm_visualize_grasp(self):
        if not self.tfm_module and not self._last_grasp_px:
            self._set_status("TFM no disponible", error=True)
            return
        rep = None
        if self.tfm_module:
            try:
                rep = self.tfm_module.get_grasp_representation()
            except Exception as exc:
                self._log_warning(f"[TFM] get_grasp_representation error: {exc}")
        if not rep and self._last_grasp_px:
            rep = dict(self._last_grasp_px)
        if not rep:
            self._set_status("TFM: sin grasp para visualizar", error=True)
            return
        ref = None
        if self._last_camera_frame:
            _qimg, w, h, _ts = self._last_camera_frame
            ref = self._build_reference_grasp(w, h)
        if ref:
            self._tfm_visual_compare_enabled = not bool(self._tfm_visual_compare_enabled)
            mode_txt = "compare_on" if self._tfm_visual_compare_enabled else "compare_off"
        else:
            self._tfm_visual_compare_enabled = False
            mode_txt = "pred_only"
        self._log(f"[TFM] Grasp actual: {rep} ref={ref} mode={mode_txt}")
        self._refresh_grasp_overlay_now()
        overlay_path = self._save_grasp_overlay()
        if overlay_path:
            self._audit_append(
                "logs/visualize.log",
                f"[TFM] visualize OK mode={mode_txt} overlay={overlay_path}",
            )
        else:
            self._audit_append("logs/visualize.log", f"[TFM] visualize FAIL mode={mode_txt} overlay=none")
        if self._tfm_visual_compare_enabled:
            self._set_status("TFM: comparación grasp/ref activada", error=False)
        elif ref:
            self._set_status("TFM: comparación grasp/ref desactivada", error=False)
        else:
            self._set_status("TFM: grasp visualizado (sin referencia)", error=False)

    def _wait_tfm_moveit_result(
        self,
        label: str,
        *,
        since_wall: float,
        since_seq: int,
        timeout_sec: float,
        expected_request_id: Optional[int] = None,
        expected_request_uuid: str = "",
    ) -> Tuple[bool, str]:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            self._motion_in_progress = False
            return False, "ros_worker_not_ready"
        deadline = time.time() + max(0.2, float(timeout_sec))
        cursor_wall = float(since_wall)
        cursor_seq = int(since_seq)
        expected_uuid = str(expected_request_uuid or "").strip()
        while time.time() < deadline:
            chunk = min(0.8, max(0.1, deadline - time.time()))
            ok, raw, wall, seq = self.ros_worker.wait_for_moveit_result(
                since_wall=cursor_wall,
                since_seq=cursor_seq,
                timeout_sec=chunk,
            )
            if not ok:
                continue
            cursor_wall = max(cursor_wall, float(wall))
            cursor_seq = max(cursor_seq, int(seq))
            text = str(raw or "").strip()
            if not text:
                continue
            try:
                data = json.loads(text)
            except Exception:
                low = text.lower()
                success = ("success" in low) or ("ok" in low)
                # FASE 3: Liberar motion_in_progress.
                self._motion_in_progress = False
                return success, text
            if expected_request_id is not None:
                got_id = int(data.get("request_id", -1) or -1)
                if got_id != int(expected_request_id):
                    continue
            if expected_uuid:
                got_uid = str(data.get("request_uuid", "") or "")
                if got_uid != expected_uuid:
                    continue
            success = bool(data.get("success", False))
            if not success and ("plan_ok" in data or "exec_ok" in data):
                success = bool(data.get("plan_ok", False) and data.get("exec_ok", False))
            if not success and "exec_ok" in data:
                success = bool(data.get("exec_ok", False))
            msg = str(data.get("message") or data.get("status") or "")
            if not msg:
                msg = text
            # FASE 3: Liberar motion_in_progress al obtener resultado.
            self._motion_in_progress = False
            return success, msg
        # FASE 3: Liberar motion_in_progress al salir por timeout.
        self._motion_in_progress = False
        return False, f"timeout>{timeout_sec:.1f}s"

    def _execute_tfm_world_grasp(self) -> bool:
        if not self._last_grasp_px:
            return False
        if not self._last_grasp_world and self._last_camera_frame:
            _qimg, frame_w, frame_h, _ts = self._last_camera_frame
            if frame_w > 0 and frame_h > 0:
                self._last_grasp_world = self._compute_world_grasp(frame_w, frame_h)
        if self._last_grasp_base is None and self._last_grasp_world is not None:
            base_coords = self._ensure_base_coords(
                (
                    float(self._last_grasp_world.get("x", 0.0)),
                    float(self._last_grasp_world.get("y", 0.0)),
                    float(self._last_grasp_world.get("z", 0.0)),
                ),
                self._world_frame_config_first(),
                timeout_sec=0.35,
            )
            if base_coords is not None:
                self._last_grasp_base = {
                    "x": float(base_coords[0]),
                    "y": float(base_coords[1]),
                    "z": float(base_coords[2]),
                    "yaw_deg": float(self._last_grasp_world.get("yaw_deg", 0.0) or 0.0),
                }
        if not self._last_grasp_base:
            self._set_status("TFM: grasp base_link no disponible (cámara/calibración)", error=True)
            self._audit_append(
                "logs/execute.log",
                f"[TFM] execute FAIL reason=base_grasp_missing source={self._last_grasp_source or 'unknown'}",
            )
            return True
        if self._tfm_execute_inflight:
            self._set_status("TFM: ejecución en curso", error=False)
            return True
        if not self._moveit_required:
            self._set_status("TFM: MoveIt no habilitado", error=True)
            self._audit_append("logs/execute.log", "[TFM] execute FAIL reason=moveit_disabled")
            return True

        self._tfm_execute_inflight = True
        grasp_base = dict(self._last_grasp_base)
        source = self._last_grasp_source or "unknown"

        def _env_float(name: str, default: float) -> float:
            try:
                return float(os.environ.get(name, str(default)) or default)
            except Exception:
                return default

        def _next_tfm_request_id() -> int:
            seq = int(getattr(self, "_tfm_moveit_request_id", 0) or 0) + 1
            setattr(self, "_tfm_moveit_request_id", seq)
            return seq

        def _encode_request_frame(frame: str, request_id: int, request_uuid: str) -> str:
            base = str(frame or BASE_FRAME or "base_link").strip() or "base_link"
            uid = str(request_uuid or "").strip()
            if uid:
                return f"{base}|rid={int(request_id)}|uid={uid}"
            return f"{base}|rid={int(request_id)}"

        def _ros_clock_now_ns() -> int:
            try:
                if self._ros_worker_started and self.ros_worker.node_ready():
                    with self.ros_worker._lock:
                        node = getattr(self.ros_worker, "_node", None)
                    if node is not None:
                        now_ns = int(node.get_clock().now().nanoseconds)
                        if now_ns > 0:
                            return now_ns
            except Exception:
                pass
            return 0

        def _ensure_moveit_bridge_path(timeout_sec: float = 6.0) -> bool:
            if not self._ros_worker_started:
                self._ensure_ros_worker_started()
            if not self.ros_worker.node_ready():
                raise RuntimeError("ros_worker_not_ready_for_moveit_bridge")

            pose_topic = MOVEIT_POSE_TOPIC
            result_topic = "/desired_grasp/result"

            def _path_ready() -> bool:
                pose_subs = int(self.ros_worker.topic_subscriber_count(pose_topic))
                result_pubs = int(self.ros_worker.topic_publisher_count(result_topic))
                return pose_subs > 0 and result_pubs > 0

            if _path_ready():
                return False

            self._emit_log(
                "[TFM][MOVEIT] bridge_path_missing "
                f"pose_topic={pose_topic} result_topic={result_topic}; attempting_recover=true"
            )

            done = threading.Event()

            def _launch_bridge() -> None:
                try:
                    self._start_moveit_bridge()
                finally:
                    done.set()

            self.signal_run_ui.emit(_launch_bridge)
            done.wait(timeout=3.0)

            deadline = time.time() + max(1.0, float(timeout_sec))
            while time.time() < deadline:
                pose_subs = int(self.ros_worker.topic_subscriber_count(pose_topic))
                result_pubs = int(self.ros_worker.topic_publisher_count(result_topic))
                self._emit_log(
                    "[TFM][MOVEIT] bridge_recover_check "
                    f"pose_subs={pose_subs} result_pubs={result_pubs}"
                )
                if pose_subs > 0 and result_pubs > 0:
                    return True
                time.sleep(0.4)

            raise RuntimeError(
                "moveit_bridge_not_ready_after_recover "
                f"pose_topic={pose_topic} result_topic={result_topic}"
            )

        def worker() -> None:
            try:
                x = float(grasp_base.get("x", 0.0))
                y = float(grasp_base.get("y", 0.0))
                z_raw = float(grasp_base.get("z", 0.0))
                yaw_deg = float(grasp_base.get("yaw_deg", 0.0) or 0.0)
                table_top_world = float(self._resolve_table_top_z())
                table_top_base = z_raw
                table_base = self._ensure_base_coords(
                    (float(TABLE_CENTER_X), float(TABLE_CENTER_Y), table_top_world),
                    self._world_frame_config_first(),
                    timeout_sec=0.35,
                )
                if table_base is not None:
                    table_top_base = float(table_base[2])
                min_margin = max(0.0, _env_float("PANEL_TFM_MIN_TABLE_MARGIN_M", 0.01))
                z_approach = max(0.12, min(0.20, _env_float("PANEL_PICK_Z_APPROACH_M", 0.14)))
                z_grasp_offset = _env_float("PANEL_PICK_Z_GRASP_OFFSET_M", 0.02)
                speed_scale = max(0.01, min(1.0, _env_float("PANEL_PICK_SPEED_SCALE", 0.25)))
                accel_scale = max(0.01, min(1.0, _env_float("PANEL_PICK_ACCEL_SCALE", 0.25)))
                result_timeout = max(1.0, _env_float("PANEL_TFM_MOVEIT_RESULT_TIMEOUT_SEC", 10.0))
                z_grasp = max(table_top_base + min_margin, z_raw + z_grasp_offset)
                z_pre = max(z_grasp + z_approach, table_top_base + min_margin + 0.02)
                z_retreat = z_pre
                yaw_rad = math.radians(yaw_deg)
                orientation = (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))
                frame = self._business_base_frame()
                pre_pose = _make_pose_data((x, y, z_pre), orientation=orientation, frame=frame)
                grasp_pose = _make_pose_data((x, y, z_grasp), orientation=orientation, frame=frame)
                retreat_pose = _make_pose_data((x, y, z_retreat), orientation=orientation, frame=frame)

                for pd in (pre_pose, grasp_pose, retreat_pose):
                    pd["speed_scale"] = float(speed_scale)
                    pd["accel_scale"] = float(accel_scale)

                has_results = False
                if self._ros_worker_started and self.ros_worker.node_ready():
                    try:
                        has_results = bool(self.ros_worker.subscribe_moveit_result("/desired_grasp/result"))
                    except Exception:
                        has_results = False
                if has_results:
                    _ensure_moveit_bridge_path()

                self._audit_append(
                    "logs/execute.log",
                    f"[TFM] execute START source={source} base=({x:.3f},{y:.3f},{z_grasp:.3f}) "
                    f"pre_z={z_pre:.3f} yaw={yaw_deg:.1f} frame={frame} wait_result={str(has_results).lower()} "
                    f"z_approach={z_approach:.3f} z_grasp_offset={z_grasp_offset:+.3f} "
                    f"speed_scale={speed_scale:.2f} accel_scale={accel_scale:.2f}",
                )

                self.signal_run_ui.emit(lambda: self._command_gripper(False, log_action="PICK", force=True))
                self._emit_log(
                    "[PICK][MOVEIT] sequence=HOME->PREGRASP->DESCENT->GRASP->RETREAT "
                    f"frame={frame} z_approach={z_approach:.3f} z_grasp_offset={z_grasp_offset:+.3f} "
                    f"speed_scale={speed_scale:.2f} accel_scale={accel_scale:.2f}"
                )
                time.sleep(0.35)

                since_wall = 0.0
                since_seq = -1
                if has_results:
                    _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                pre_request_id = _next_tfm_request_id()
                pre_request_uuid = uuid.uuid4().hex
                pre_pose_send = dict(pre_pose)
                pre_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
                pre_pose_send["frame"] = _encode_request_frame(frame, pre_request_id, pre_request_uuid)
                pose_subs_now = int(self.ros_worker.topic_subscriber_count(MOVEIT_POSE_TOPIC)) if has_results else 1
                if pose_subs_now <= 0:
                    raise RuntimeError(f"no_pose_subscribers_before_pregrasp topic={MOVEIT_POSE_TOPIC}")
                if not self._publish_moveit_pose("TFM_PRE_GRASP", pre_pose_send, cartesian=False):
                    raise RuntimeError("publish_pregrasp_failed")
                if has_results:
                    ok_pre, msg_pre = self._wait_tfm_moveit_result(
                        "TFM_PRE_GRASP",
                        since_wall=since_wall,
                        since_seq=since_seq,
                        timeout_sec=result_timeout,
                        expected_request_id=pre_request_id,
                        expected_request_uuid=pre_request_uuid,
                    )
                    if not ok_pre:
                        raise RuntimeError(f"pregrasp_result_failed:{msg_pre}")
                else:
                    time.sleep(0.8)

                since_wall = 0.0
                since_seq = -1
                if has_results:
                    _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                grasp_request_id = _next_tfm_request_id()
                grasp_request_uuid = uuid.uuid4().hex
                grasp_pose_send = dict(grasp_pose)
                grasp_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
                grasp_pose_send["frame"] = _encode_request_frame(frame, grasp_request_id, grasp_request_uuid)
                pose_subs_now = int(self.ros_worker.topic_subscriber_count(MOVEIT_POSE_TOPIC)) if has_results else 1
                if pose_subs_now <= 0:
                    raise RuntimeError(f"no_pose_subscribers_before_grasp topic={MOVEIT_POSE_TOPIC}")
                if not self._publish_moveit_pose("TFM_GRASP", grasp_pose_send, cartesian=True):
                    raise RuntimeError("publish_grasp_failed")
                if has_results:
                    ok_grasp, msg_grasp = self._wait_tfm_moveit_result(
                        "TFM_GRASP",
                        since_wall=since_wall,
                        since_seq=since_seq,
                        timeout_sec=result_timeout,
                        expected_request_id=grasp_request_id,
                        expected_request_uuid=grasp_request_uuid,
                    )
                    if not ok_grasp:
                        msg_low = str(msg_grasp or "").lower()
                        reason = "unknown"
                        if "collision" in msg_low:
                            reason = "collision"
                        elif "ik" in msg_low:
                            reason = "ik"
                        elif "frame" in msg_low or "tf" in msg_low:
                            reason = "frame_mismatch"
                        elif "plan" in msg_low:
                            reason = "planning"
                        self._emit_log(
                            "[PICK][MOVEIT] cartesian_descent_failed "
                            f"reason={reason} msg={msg_grasp}; fallback=planner"
                        )
                        self._audit_append(
                            "logs/execute.log",
                            "[TFM] grasp cartesian_failed "
                            f"reason={reason} msg={msg_grasp}",
                        )
                        since_wall = 0.0
                        since_seq = -1
                        _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                        fallback_request_id = _next_tfm_request_id()
                        fallback_request_uuid = uuid.uuid4().hex
                        fallback_pose_send = dict(grasp_pose)
                        fallback_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
                        fallback_pose_send["frame"] = _encode_request_frame(frame, fallback_request_id, fallback_request_uuid)
                        if not self._publish_moveit_pose("TFM_GRASP_FALLBACK", fallback_pose_send, cartesian=False):
                            raise RuntimeError("publish_grasp_fallback_failed")
                        ok_fb, msg_fb = self._wait_tfm_moveit_result(
                            "TFM_GRASP_FALLBACK",
                            since_wall=since_wall,
                            since_seq=since_seq,
                            timeout_sec=result_timeout,
                            expected_request_id=fallback_request_id,
                            expected_request_uuid=fallback_request_uuid,
                        )
                        if not ok_fb:
                            raise RuntimeError(f"grasp_fallback_result_failed:{msg_fb}")
                else:
                    time.sleep(0.8)

                self.signal_run_ui.emit(lambda: self._command_gripper(True, log_action="PICK", force=True))
                time.sleep(0.35)

                since_wall = 0.0
                since_seq = -1
                if has_results:
                    _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                retreat_request_id = _next_tfm_request_id()
                retreat_request_uuid = uuid.uuid4().hex
                retreat_pose_send = dict(retreat_pose)
                retreat_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
                retreat_pose_send["frame"] = _encode_request_frame(frame, retreat_request_id, retreat_request_uuid)
                if not self._publish_moveit_pose("TFM_RETREAT", retreat_pose_send, cartesian=False):
                    raise RuntimeError("publish_retreat_failed")
                if has_results:
                    ok_ret, msg_ret = self._wait_tfm_moveit_result(
                        "TFM_RETREAT",
                        since_wall=since_wall,
                        since_seq=since_seq,
                        timeout_sec=result_timeout,
                        expected_request_id=retreat_request_id,
                        expected_request_uuid=retreat_request_uuid,
                    )
                    if not ok_ret:
                        raise RuntimeError(f"retreat_result_failed:{msg_ret}")
                else:
                    time.sleep(0.6)

                self._audit_append(
                    "logs/execute.log",
                    f"[TFM] execute OK mode=moveit_sequence source={source} "
                    f"target=({x:.3f},{y:.3f},{z_grasp:.3f}) pre=({x:.3f},{y:.3f},{z_pre:.3f}) "
                    f"retreat=({x:.3f},{y:.3f},{z_retreat:.3f}) yaw={yaw_deg:.1f}",
                )
                self._ui_set_status("TFM: PREGRASP + DESCENT + GRASP + RETREAT ejecutados", error=False)
            except Exception as exc:
                self._audit_append("logs/execute.log", f"[TFM] execute FAIL mode=moveit_sequence err={exc}")
                self._ui_set_status(f"TFM: ejecución fallida ({exc})", error=True)
            finally:
                self._tfm_execute_inflight = False

        self._run_async(worker)
        return True

    def _tfm_publish_grasp(self):
        if not self._require_ready_basic("TFM"):
            return
        self._ensure_grasp_rect_subscription()
        if not self._last_grasp_px:
            source = self._last_grasp_source or ""
            if source.startswith("topic:"):
                self._set_status("TFM: espera /grasp_rect", error=True)
            else:
                self._set_status("TFM: primero infiere o recibe un grasp", error=True)
            self._audit_append("logs/execute.log", "[TFM] execute FAIL reason=no_grasp_available")
            return
        handled = self._execute_tfm_world_grasp()
        if handled:
            return
        self._audit_append(
            "logs/execute.log",
            "[TFM] execute FAIL reason=world_grasp_unavailable",
        )
        self._set_status("TFM: grasp no disponible para ejecutar", error=True)

    def _save_episode(self) -> None:
        out_dir = Path(WS_DIR).parent / "reports" / "panel_logs"
        ensure_dir(str(out_dir))
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = out_dir / f"episode_{stamp}.json"
        w = h = 0
        frame_ts = None
        if self._last_camera_frame:
            _qimg, w, h, frame_ts = self._last_camera_frame
        episode = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "experiment": {
                "id": self._exp_info.get("experiment"),
                "base": self._exp_info.get("experiment_base"),
                "model": self._exp_info.get("model"),
                "modality": self._exp_info.get("modality"),
                "seed": self._exp_info.get("seed"),
                "epoch": self._exp_info.get("epoch"),
                "weights": self._exp_info.get("weights"),
                "weights_path": self._exp_info.get("weights_path"),
                "config_path": self._exp_info.get("config_path"),
            },
            "grasp": {
                "image": self._last_grasp_px,
                "base": self._last_grasp_base,
                "frame": self._last_grasp_frame,
                "source": self._last_grasp_source,
            },
            "evaluation": {
                "iou": self._last_cornell.get("iou") if self._last_cornell else None,
                "delta_theta_deg": self._last_cornell.get("dtheta") if self._last_cornell else None,
                "success": self._last_cornell.get("success") if self._last_cornell else None,
                "note": "Evaluación geométrica en simulación. No validación física.",
            },
            "latency": {
                "inference_ms": self._perf_infer_ms,
                "perception_ms": self._perf_total_ms,
                "fps": self._camera_last_fps,
                "fps_avg": self._perf_fps_avg,
            },
            "frames": {
                "image_topic": self.camera_topic,
                "image_size": [int(w), int(h)],
                "image_timestamp": frame_ts,
            },
            "artifacts": {
                "image_path": self._last_infer_image_path or None,
                "grasp_path": self._last_infer_output_path or None,
            },
        }
        try:
            out_path.write_text(json.dumps(episode, indent=2), encoding="utf-8")
        except Exception as exc:
            self._set_status(f"Error guardando episodio: {exc}", error=True)
            return
        self._emit_log(f"[TFM] Episodio guardado: {out_path}")
        self._set_status("Episodio experimental guardado", error=False)

    def _run_auto_tune_touch(self) -> None:
        self._log_button("AUTO TUNE TOUCH (usando TEST ROBOT internalmente)")
        self._emit_log("[AUTO_TUNE] delegando a TEST ROBOT (touch probe + home)")
        self._emit_log("[AUTO_TUNE] nota: AUTO TUNE es idéntico a TEST ROBOT con MoveIt habilitado")
        # Simplemente ejecutar TEST ROBOT. AUTO TUNE y TEST ROBOT hacen lo mismo cuando MoveIt está activo.
        self._run_robot_test()

    def _run_robot_test(self):
        self._log_button("TEST ROBOT")
        self._audit_append("logs/test_robot.log", "[TEST] REQUESTED")
        if self._robot_test_disabled:
            self._set_status("TEST ROBOT ya completado", error=False)
            self._audit_append("logs/test_robot.log", "[TEST] SKIP reason=already_done")
            return
        if self._script_motion_active:
            self._set_status("TEST ROBOT en curso; espera", error=False)
            self._audit_append("logs/test_robot.log", "[TEST] SKIP reason=motion_active")
            return
        if not self._require_manual_ready("TEST ROBOT"):
            self._audit_append("logs/test_robot.log", "[TEST] SKIP reason=manual_not_ready")
            return
        tf_ok, tf_reason = self._tf_chain_ready_status()
        if not tf_ok:
            reason = f"TF no listo ({tf_reason})"
            self._set_status(f"TEST ROBOT bloqueado: {reason}", error=False)
            self._emit_log(
                f"[TEST][BLOCK] {reason} required={self._business_base_frame()}->{self._required_ee_frame}"
            )
            self._audit_append("logs/test_robot.log", f"[TEST] SKIP reason=tf_not_ready:{tf_reason}")
            return
        traj_topic = self._select_traj_topic()
        if not traj_topic:
            self._set_status("TEST ROBOT bloqueado: joint_trajectory_controller no disponible", error=True)
            self._emit_log("[SAFETY] TEST ROBOT bloqueado: joint_trajectory_controller no disponible")
            self._audit_append("logs/test_robot.log", "[TEST] BLOCKED reason=no_traj_topic")
            return
        externals = self._external_publishers_for_topic(traj_topic)
        moveit_node = "panel_v2_moveit_publisher"
        bridge_node = "ur5_moveit_bridge"
        ns = self.ros_worker.node_namespace()
        moveit_nodes = {
            moveit_node,
            f"/{moveit_node}",
            f"{ns}/{moveit_node}".replace("//", "/"),
            bridge_node,
            f"/{bridge_node}",
            f"{ns}/{bridge_node}".replace("//", "/"),
        }

        def start_sequence(restart_moveit_bridge: bool) -> None:
            self._audit_append(
                "logs/test_robot.log",
                (
                    "[TEST] START "
                    f"traj_topic={traj_topic} "
                    f"base={self._business_base_frame()} "
                    f"ee={self._ee_frame_effective or 'unknown'} "
                    f"moveit_required={str(self._moveit_required).lower()}"
                ),
            )
            self._robot_test_substate = "RUNNING"
            self._robot_test_last_failure = ""
            self._set_robot_test_blocked(None)
            self._reach_overlay_enabled = True
            self._reach_overlay_points = []
            self._reach_overlay_size = (0, 0)
            self._set_motion_lock(True)
            self._robot_test_active = True
            set_test_read_only(True, reason="TEST ROBOT")
            move_sec = float(self.joint_time.value()) if self.joint_time else 3.0
            home_pose = self._get_home_joint_pose()
            test_tuning_cache: Optional[Dict[str, object]] = None

            def _load_robot_test_tuning() -> Dict[str, object]:
                cfg_env = str(os.environ.get("PANEL_TEST_TUNING_FILE", "") or "").strip()
                if not cfg_env:
                    cfg_env = os.path.join(
                        str(getattr(self, "ws_dir", WS_DIR)),
                        "src",
                        "ur5_qt_panel",
                        "config",
                        "panel_test_tuning.yaml",
                    )
                cfg_path = Path(os.path.expandvars(os.path.expanduser(cfg_env)))
                if not cfg_path.is_file():
                    return {}
                try:
                    if cfg_path.suffix.lower() == ".json":
                        raw = json.loads(cfg_path.read_text(encoding="utf-8")) or {}
                    else:
                        if yaml is None:
                            self._emit_log(
                                f"[TEST][TUNING] WARN yaml_unavailable file={cfg_path}"
                            )
                            return {}
                        raw = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
                except Exception as exc:
                    self._emit_log(f"[TEST][TUNING] WARN read_error file={cfg_path} err={exc}")
                    return {}
                if not isinstance(raw, dict):
                    return {}
                scoped = raw.get("robot_test") if isinstance(raw.get("robot_test"), dict) else raw
                if not isinstance(scoped, dict):
                    return {}
                self._emit_log(
                    "[TEST][TUNING] loaded "
                    f"file={cfg_path} keys={sorted(str(k) for k in scoped.keys())}"
                )
                return {str(k): v for k, v in scoped.items()}

            def _test_cfg_value(name: str, default: object) -> object:
                raw_env = os.environ.get(name)
                if raw_env is not None and str(raw_env).strip() != "":
                    return raw_env
                nonlocal test_tuning_cache
                if test_tuning_cache is None:
                    test_tuning_cache = _load_robot_test_tuning()
                if name in test_tuning_cache:
                    return test_tuning_cache[name]
                return default

            def _test_cfg_float(name: str, default: float) -> float:
                val = _test_cfg_value(name, default)
                try:
                    return float(val)
                except Exception:
                    return float(default)

            def _test_cfg_int(name: str, default: int) -> int:
                val = _test_cfg_value(name, default)
                try:
                    return int(val)
                except Exception:
                    return int(default)

            def _test_cfg_bool(name: str, default: bool) -> bool:
                val = _test_cfg_value(name, default)
                if isinstance(val, bool):
                    return val
                txt = str(val).strip().lower()
                if txt in ("1", "true", "yes", "on"):
                    return True
                if txt in ("0", "false", "no", "off"):
                    return False
                return bool(default)

            test_start_monotonic = time.monotonic()
            test_max_duration_sec = max(
                30.0,
                _test_cfg_float("PANEL_TEST_MAX_DURATION_SEC", 120.0),
            )
            test_deadline_monotonic = test_start_monotonic + test_max_duration_sec
            test_timeout_emitted = False

            def _test_deadline_remaining_sec() -> float:
                return float(test_deadline_monotonic - time.monotonic())

            def _test_timeout_reason(stage: str) -> str:
                elapsed = max(0.0, time.monotonic() - test_start_monotonic)
                return (
                    f"test_timeout stage={stage} elapsed={elapsed:.1f}s "
                    f"limit={test_max_duration_sec:.1f}s"
                )

            def _test_timed_out(stage: str) -> bool:
                nonlocal test_timeout_emitted
                if _test_deadline_remaining_sec() > 0.0:
                    return False
                if not test_timeout_emitted:
                    test_timeout_emitted = True
                    self._emit_log(f"[TEST][TIMEOUT] {_test_timeout_reason(stage)}")
                return True

            def _test_frame_probe_enabled() -> bool:
                return _test_cfg_bool("PANEL_TEST_FRAME_PROBE", True)

            def _next_test_moveit_request_id() -> int:
                seq = int(getattr(self, "_test_moveit_request_id", 0) or 0) + 1
                setattr(self, "_test_moveit_request_id", seq)
                return seq

            def _encode_request_frame(frame: str, request_id: int, request_uuid: str) -> str:
                base = str(frame or BASE_FRAME or "base_link").strip() or "base_link"
                uid = str(request_uuid or "").strip()
                if uid:
                    return f"{base}|rid={int(request_id)}|uid={uid}"
                return f"{base}|rid={int(request_id)}"

            def _ensure_test_moveit_pipeline(timeout_sec: float) -> Tuple[bool, str]:
                timeout = max(5.0, float(timeout_sec))
                deadline = time.time() + timeout
                start_moveit_sent = False
                start_bridge_sent = False
                last_reason = "init"
                while time.time() < deadline:
                    if not self._ros_worker_started or not self.ros_worker.node_ready():
                        last_reason = "ros_worker_not_ready"
                        time.sleep(0.2)
                        continue

                    moveit_ready = (
                        self._moveit_state == MoveItState.READY
                        or self._moveit_status_ready()
                    )
                    bridge_ready = self._moveit_bridge_detected()
                    result_topic_ready = self.ros_worker.topic_has_publishers(
                        "/desired_grasp/result"
                    )

                    if moveit_ready and bridge_ready and result_topic_ready:
                        if self.ros_worker.subscribe_moveit_result("/desired_grasp/result"):
                            return True, "ok"
                        last_reason = "moveit_result_subscription_failed"
                    else:
                        state_label = (
                            self._moveit_state.value
                            if isinstance(self._moveit_state, MoveItState)
                            else str(self._moveit_state)
                        )
                        last_reason = (
                            f"moveit={str(moveit_ready).lower()}"
                            f",bridge={str(bridge_ready).lower()}"
                            f",result_topic={str(result_topic_ready).lower()}"
                            f",state={state_label}"
                        )
                        if not moveit_ready and not start_moveit_sent:
                            start_moveit_sent = True
                            self._emit_log(
                                "[TEST][MOVEIT] arranque MoveIt solicitado (pipeline no listo)"
                            )
                            self.signal_run_ui.emit(self._start_moveit)
                        if not bridge_ready and not start_bridge_sent:
                            start_bridge_sent = True
                            self._emit_log(
                                "[TEST][MOVEIT] arranque MoveIt bridge solicitado (pipeline no listo)"
                            )
                            self.signal_run_ui.emit(self._start_moveit_bridge)

                    time.sleep(0.2)
                return False, last_reason

            def _run_frame_axis_probe() -> Tuple[bool, str]:
                if not _test_frame_probe_enabled():
                    self._emit_log("[TEST][FRAME] skipped: PANEL_TEST_FRAME_PROBE=0")
                    return True, "skipped_by_env"
                if not self._moveit_required:
                    self._emit_log("[TEST][FRAME] skipped: moveit_disabled")
                    return True, "moveit_disabled"
                moveit_ready_timeout = max(
                    8.0, _test_cfg_float("PANEL_TEST_MOVEIT_READY_TIMEOUT_SEC", 30.0)
                )
                ok_pipeline, info_pipeline = _ensure_test_moveit_pipeline(
                    timeout_sec=moveit_ready_timeout
                )
                if not ok_pipeline:
                    return False, f"moveit_pipeline_not_ready:{info_pipeline}"
                base_frame = self._business_base_frame()
                use_world_ref = False
                frame_ref = base_frame

                def _select_test_ee_frame(local_base_frame: str) -> str:
                    preferred = str(
                        os.environ.get("PANEL_TEST_TOUCH_TIP_FRAME", "rg2_tcp") or "rg2_tcp"
                    ).strip() or "rg2_tcp"
                    helper = get_tf_helper()
                    # FASE 1: timeout 0.5s (antes 0.08s) para reducir spam TF
                    if helper and _can_transform_between(
                        helper, local_base_frame, preferred, timeout_sec=0.5
                    ):
                        return preferred
                    fallback = self._ee_frame_effective or "tool0"
                    if helper and _can_transform_between(
                        helper, local_base_frame, fallback, timeout_sec=0.5
                    ):
                        if fallback != preferred:
                            self._emit_log(
                                "[TEST][FRAME] WARN "
                                f"ee_preferred={preferred} unavailable; using {fallback}"
                            )
                        return fallback
                    return preferred

                ee_frame = _select_test_ee_frame(base_frame)
                xy_step = max(
                    0.01, min(0.06, _test_cfg_float("PANEL_TEST_FRAME_STEP_XY_M", 0.03))
                )
                z_step = max(
                    0.01, min(0.05, _test_cfg_float("PANEL_TEST_FRAME_STEP_Z_M", 0.03))
                )
                result_timeout = max(
                    2.0, _test_cfg_float("PANEL_TEST_FRAME_RESULT_TIMEOUT_SEC", 10.0)
                )
                axis_gain = max(
                    0.25,
                    min(0.95, _test_cfg_float("PANEL_TEST_FRAME_MIN_AXIS_GAIN", 0.45)),
                )
                cross_limit_abs = max(
                    0.01,
                    min(0.06, _test_cfg_float("PANEL_TEST_FRAME_MAX_CROSS_AXIS_M", 0.03)),
                )
                z_margin = max(
                    0.01, _test_cfg_float("PANEL_TEST_FRAME_MIN_Z_MARGIN_M", 0.05)
                )
                safe_probe_margin = max(
                    0.005, _test_cfg_float("PANEL_TEST_FRAME_SAFE_PROBE_MARGIN_M", 0.02)
                )
                tf_settle_sec = max(
                    0.2, min(5.0, _test_cfg_float("PANEL_TEST_FRAME_TF_SETTLE_SEC", 1.2))
                )
                target_tol_abs = max(
                    0.006, min(0.08, _test_cfg_float("PANEL_TEST_FRAME_TARGET_TOL_M", 0.025))
                )
                sx = 1 if _test_cfg_int("PANEL_TEST_FRAME_SIGN_X", 1) >= 0 else -1
                sy = 1 if _test_cfg_int("PANEL_TEST_FRAME_SIGN_Y", 1) >= 0 else -1
                sz = 1 if _test_cfg_int("PANEL_TEST_FRAME_SIGN_Z", 1) >= 0 else -1
                anchor_enabled = _test_cfg_bool("PANEL_TEST_FRAME_ANCHOR_ENABLED", True)
                anchor_z_offset = max(
                    0.05, min(0.25, _test_cfg_float("PANEL_TEST_FRAME_ANCHOR_Z_OFFSET_M", 0.10))
                )
                strict_frame_measurements = _test_cfg_bool(
                    "PANEL_TEST_FRAME_STRICT_MEASUREMENTS", False
                )
                tf_max_age_sec = max(
                    0.05, min(5.0, _test_cfg_float("PANEL_TEST_FRAME_TF_MAX_AGE_SEC", 0.9))
                )
                tf_jump_limit_m = max(
                    0.10, min(1.20, _test_cfg_float("PANEL_TEST_FRAME_MAX_STEP_JUMP_M", 0.35))
                )
                last_pose_hint: Optional[Tuple[float, float, float]] = None
                last_ori_hint: Optional[Tuple[float, float, float, float]] = None
                last_pose_hint_wall = 0.0

                table_top = float(self._resolve_table_top_z())
                safe_min_z = 0.08
                if use_world_ref:
                    world_safe_floor = _test_cfg_float(
                        "PANEL_TEST_FRAME_WORLD_SAFE_MIN_Z_M",
                        table_top - 0.30,
                    )
                    safe_min_z = max(0.05, min(table_top + 0.20, float(world_safe_floor)))
                else:
                    table_base, _table_tf = transform_point_to_frame(
                        (TABLE_CENTER_X, TABLE_CENTER_Y, table_top),
                        base_frame,
                        source_frame=WORLD_FRAME or "world",
                        timeout_sec=0.4,
                    )
                    if table_base:
                        safe_min_z = max(safe_min_z, float(table_base[2]) + z_margin)

                def _ros_clock_now_ns() -> int:
                    try:
                        if self._ros_worker_started and self.ros_worker.node_ready():
                            with self.ros_worker._lock:
                                node = getattr(self.ros_worker, "_node", None)
                            if node is not None:
                                now_ns = int(node.get_clock().now().nanoseconds)
                                if now_ns > 0:
                                    return now_ns
                    except Exception:
                        pass
                    return 0

                def _is_pose_plausible(pos: Tuple[float, float, float]) -> bool:
                    x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
                    if use_world_ref:
                        if not (-2.5 <= x <= 2.5 and -2.5 <= y <= 2.5 and -0.2 <= z <= 2.5):
                            return False
                    else:
                        if not (-1.2 <= x <= 1.4 and -1.4 <= y <= 1.4 and -0.35 <= z <= 1.4):
                            return False
                    return True

                def _read_tcp_pose() -> Tuple[Optional[Tuple[float, float, float]], Optional[Tuple[float, float, float, float]], str]:
                    nonlocal last_pose_hint, last_ori_hint, last_pose_hint_wall
                    deadline = time.time() + 0.8
                    last_err = "tcp_pose_unavailable"
                    while time.time() < deadline:
                        pose_data, err = get_pose(frame_ref, ee_frame, timeout_sec=0.25)
                        if not pose_data:
                            last_err = str(err or "tcp_pose_unavailable")
                            time.sleep(0.03)
                            continue
                        pos_raw = pose_data.get("position", (0.0, 0.0, 0.0))
                        ori_raw = pose_data.get("orientation", (0.0, 0.0, 0.0, 1.0))
                        if not isinstance(pos_raw, (list, tuple)) or len(pos_raw) < 3:
                            last_err = "tcp_position_invalid"
                            time.sleep(0.03)
                            continue
                        if not isinstance(ori_raw, (list, tuple)) or len(ori_raw) < 4:
                            last_err = "tcp_orientation_invalid"
                            time.sleep(0.03)
                            continue
                        pos = (float(pos_raw[0]), float(pos_raw[1]), float(pos_raw[2]))
                        ori = (float(ori_raw[0]), float(ori_raw[1]), float(ori_raw[2]), float(ori_raw[3]))
                        stamp_ns = int(pose_data.get("stamp_ns", 0) or 0)
                        now_ns = int(_ros_clock_now_ns())
                        if stamp_ns > 0 and now_ns > 0:
                            # Si uno está en dominio wall-time (~1e18 ns) y el otro en sim-time
                            # (~1e11-1e12 ns), no comparar edad porque daría falsos stale_tf.
                            mixed_time_domain = (
                                (now_ns > 1_000_000_000_000_000 and stamp_ns < 10_000_000_000_000)
                                or (stamp_ns > 1_000_000_000_000_000 and now_ns < 10_000_000_000_000)
                            )
                            if not mixed_time_domain:
                                age_sec = float(now_ns - stamp_ns) / 1_000_000_000.0
                                if age_sec < -0.20 or age_sec > tf_max_age_sec:
                                    last_err = f"stale_tf age={age_sec:.3f}s max_age={tf_max_age_sec:.3f}s"
                                    time.sleep(0.03)
                                    continue
                        if not _is_pose_plausible(pos):
                            last_err = (
                                f"implausible_pose frame={frame_ref} "
                                f"pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})"
                            )
                            time.sleep(0.03)
                            continue
                        if last_pose_hint is not None:
                            dist_hint = math.sqrt(
                                float(pos[0] - last_pose_hint[0]) ** 2
                                + float(pos[1] - last_pose_hint[1]) ** 2
                                + float(pos[2] - last_pose_hint[2]) ** 2
                            )
                            elapsed = max(0.0, time.time() - float(last_pose_hint_wall))
                            if elapsed < 1.6 and dist_hint > tf_jump_limit_m:
                                last_err = (
                                    f"jump_tf dist={dist_hint:.3f}m "
                                    f"limit={tf_jump_limit_m:.3f}m elapsed={elapsed:.2f}s"
                                )
                                time.sleep(0.03)
                                continue
                        last_pose_hint = pos
                        last_ori_hint = ori
                        last_pose_hint_wall = time.time()
                        return pos, ori, ""
                    if (not strict_frame_measurements) and last_pose_hint is not None and last_ori_hint is not None:
                        return (
                            last_pose_hint,
                            last_ori_hint,
                            f"fallback_hint:{last_err}",
                        )
                    return None, None, str(last_err or "timeout")

                def _wait_tcp_pose(
                    *,
                    target: Optional[Tuple[float, float, float]] = None,
                    tol_m: float = 0.0,
                    timeout_sec: float = 0.0,
                ) -> Tuple[Optional[Tuple[float, float, float]], Optional[Tuple[float, float, float, float]], str]:
                    deadline = time.time() + max(0.05, float(timeout_sec or tf_settle_sec))
                    last_pos: Optional[Tuple[float, float, float]] = None
                    last_ori: Optional[Tuple[float, float, float, float]] = None
                    last_err = "timeout"
                    tol = max(0.0, float(tol_m))
                    while time.time() < deadline:
                        pos, ori, err = _read_tcp_pose()
                        if pos is not None and ori is not None:
                            last_pos = pos
                            last_ori = ori
                            if target is None:
                                return pos, ori, ""
                            dx = float(pos[0] - target[0])
                            dy = float(pos[1] - target[1])
                            dz = float(pos[2] - target[2])
                            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                            if dist <= tol:
                                return pos, ori, ""
                            last_err = f"settle_wait dist={dist:.3f} tol={tol:.3f}"
                        elif err:
                            last_err = err
                        time.sleep(0.05)
                    if last_pos is not None and last_ori is not None:
                        return last_pos, last_ori, f"settle_timeout:{last_err}"
                    return None, None, str(last_err or "timeout")

                def _run_delta_step(
                    label: str,
                    delta: Tuple[float, float, float],
                    axis_idx: int,
                    axis_sign: int,
                ) -> Tuple[bool, str]:
                    nonlocal last_pose_hint, last_ori_hint, last_pose_hint_wall
                    before_pos, before_ori, err_before = _read_tcp_pose()
                    if before_pos is None or before_ori is None:
                        return False, f"{label}:read_before_failed:{err_before}"
                    if float(before_pos[2]) < safe_min_z:
                        if (
                            (not strict_frame_measurements)
                            and last_pose_hint is not None
                            and float(last_pose_hint[2]) >= safe_min_z
                        ):
                            self._emit_log(
                                "[TEST][FRAME] WARN "
                                f"step={label} before_z={float(before_pos[2]):.3f}<safe_min={safe_min_z:.3f}; "
                                "using_hint_pose"
                            )
                            before_pos = last_pose_hint
                            if last_ori_hint is not None:
                                before_ori = last_ori_hint
                        elif not strict_frame_measurements:
                            repaired_z = float(max(safe_min_z + 0.005, before_pos[2]))
                            self._emit_log(
                                "[TEST][FRAME] WARN "
                                f"step={label} before_z={float(before_pos[2]):.3f}<safe_min={safe_min_z:.3f}; "
                                f"clamping_z_to={repaired_z:.3f}"
                            )
                            before_pos = (
                                float(before_pos[0]),
                                float(before_pos[1]),
                                repaired_z,
                            )
                    result_error = ""
                    effective_delta = delta
                    target = (
                        float(before_pos[0] + delta[0]),
                        float(before_pos[1] + delta[1]),
                        float(before_pos[2] + delta[2]),
                    )
                    for scale in (1.0, 0.6, 0.35):
                        eff_delta = (
                            float(delta[0] * scale),
                            float(delta[1] * scale),
                            float(delta[2] * scale),
                        )
                        tx = float(before_pos[0] + eff_delta[0])
                        ty = float(before_pos[1] + eff_delta[1])
                        tz = float(before_pos[2] + eff_delta[2])
                        if tz < safe_min_z:
                            result_error = (
                                f"{label}:unsafe_target_z target={tz:.3f} "
                                f"safe_min={safe_min_z:.3f} frame={frame_ref}"
                            )
                            continue
                        rid = _next_test_moveit_request_id()
                        ruid = uuid.uuid4().hex
                        target = (tx, ty, tz)
                        pose_data = _make_pose_data(
                            target,
                            orientation=before_ori,
                            frame=_encode_request_frame(frame_ref, rid, ruid),
                        )
                        _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                        if not self._publish_moveit_pose(
                            f"TEST_FRAME_{label}", pose_data, cartesian=False
                        ):
                            return False, f"{label}:publish_failed"
                        ok_res, msg_res = self._wait_tfm_moveit_result(
                            f"TEST_FRAME_{label}",
                            since_wall=since_wall,
                            since_seq=since_seq,
                            timeout_sec=result_timeout,
                            expected_request_id=rid,
                            expected_request_uuid=ruid,
                        )
                        if ok_res:
                            effective_delta = eff_delta
                            result_error = ""
                            last_pose_hint = target
                            last_ori_hint = before_ori
                            last_pose_hint_wall = time.time()
                            break
                        result_error = str(msg_res or "")
                        if "plan_no_trajectory" in result_error and scale > 0.36:
                            self._emit_log(
                                "[TEST][FRAME] WARN "
                                f"step={label} plan_no_trajectory scale={scale:.2f}; retrying shorter step"
                            )
                            continue
                        return False, f"{label}:result_failed:{result_error}"
                    if result_error:
                        return False, result_error
                    target_tol = max(target_tol_abs, abs(effective_delta[axis_idx]) * 0.75)
                    after_pos, _after_ori, err_after = _wait_tcp_pose(
                        target=target,
                        tol_m=target_tol,
                        timeout_sec=tf_settle_sec,
                    )
                    if after_pos is None:
                        return False, f"{label}:read_after_failed:{err_after}"
                    dx = float(after_pos[0] - before_pos[0])
                    dy = float(after_pos[1] - before_pos[1])
                    dz = float(after_pos[2] - before_pos[2])
                    axes = (dx, dy, dz)
                    axis_delta = float(axes[axis_idx])
                    min_axis = max(0.006, abs(effective_delta[axis_idx]) * axis_gain)
                    cross_vals = [abs(axes[i]) for i in range(3) if i != axis_idx]
                    cross_max = max(cross_vals) if cross_vals else 0.0
                    cross_limit = max(cross_limit_abs, abs(effective_delta[axis_idx]) * 0.9)
                    axis_ok = (axis_delta * float(axis_sign)) >= min_axis
                    cross_ok = cross_max <= cross_limit
                    measurement_reliable = not (
                        bool(err_after) and str(err_after).startswith("settle_timeout")
                    )
                    if err_after:
                        self._emit_log(
                            "[TEST][FRAME] WARN "
                            f"step={label} after_pose={err_after} target_tol={target_tol:.3f}"
                        )
                    self._emit_log(
                        "[TEST][FRAME] "
                        f"step={label} frame={frame_ref} ee={ee_frame} "
                        f"cmd=({effective_delta[0]:+.3f},{effective_delta[1]:+.3f},{effective_delta[2]:+.3f}) "
                        f"actual=({dx:+.3f},{dy:+.3f},{dz:+.3f}) "
                        f"axis_ok={str(axis_ok).lower()} cross_ok={str(cross_ok).lower()} "
                        f"min_axis={min_axis:.3f} cross_max={cross_max:.3f} cross_lim={cross_limit:.3f}"
                    )
                    if (not strict_frame_measurements) and (not measurement_reliable):
                        self._emit_log(
                            "[TEST][FRAME] WARN "
                            f"step={label} unreliable_tf_sample -> non_strict_pass"
                        )
                        last_pose_hint = target
                        last_ori_hint = before_ori
                        last_pose_hint_wall = time.time()
                        return True, "ok_unreliable_tf_sample"
                    if (not strict_frame_measurements) and ((not axis_ok) or (not cross_ok)):
                        self._emit_log(
                            "[TEST][FRAME] WARN "
                            f"step={label} axis_or_cross_mismatch "
                            f"(axis_ok={str(axis_ok).lower()} cross_ok={str(cross_ok).lower()}) "
                            "-> non_strict_pass"
                        )
                        last_pose_hint = target
                        last_ori_hint = before_ori
                        last_pose_hint_wall = time.time()
                        return True, "ok_nonstrict_axis_cross"
                    if not axis_ok:
                        return False, (
                            f"{label}:axis_mismatch axis_delta={axis_delta:+.3f} "
                            f"required={float(axis_sign) * min_axis:+.3f}"
                        )
                    if not cross_ok:
                        return False, (
                            f"{label}:cross_axis_exceeded cross_max={cross_max:.3f} "
                            f"limit={cross_limit:.3f}"
                        )
                    return True, "ok"

                def _ensure_probe_safe_height() -> Tuple[bool, str]:
                    nonlocal last_pose_hint, last_ori_hint, last_pose_hint_wall
                    before_pos, before_ori, err_before = _read_tcp_pose()
                    if before_pos is None or before_ori is None:
                        return False, f"safe_height:read_before_failed:{err_before}"
                    min_probe_z = max(safe_min_z, float(before_pos[2]) + 0.0) + safe_probe_margin
                    min_probe_z = max(safe_min_z + 0.005, min_probe_z)
                    if float(before_pos[2]) >= min_probe_z:
                        self._emit_log(
                            "[TEST][FRAME] safe_height already_ok "
                            f"z={float(before_pos[2]):.3f} min_probe_z={min_probe_z:.3f}"
                        )
                        return True, "already_ok"
                    rid = _next_test_moveit_request_id()
                    ruid = uuid.uuid4().hex
                    pose_data = _make_pose_data(
                        (float(before_pos[0]), float(before_pos[1]), float(min_probe_z)),
                        orientation=before_ori,
                        frame=_encode_request_frame(frame_ref, rid, ruid),
                    )
                    _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                    if not self._publish_moveit_pose("TEST_FRAME_SAFE_Z", pose_data, cartesian=False):
                        return False, "safe_height:publish_failed"
                    ok_res, msg_res = self._wait_tfm_moveit_result(
                        "TEST_FRAME_SAFE_Z",
                        since_wall=since_wall,
                        since_seq=since_seq,
                        timeout_sec=result_timeout,
                        expected_request_id=rid,
                        expected_request_uuid=ruid,
                    )
                    if not ok_res:
                        return False, f"safe_height:result_failed:{msg_res}"
                    safe_target = (
                        float(before_pos[0]),
                        float(before_pos[1]),
                        float(min_probe_z),
                    )
                    last_pose_hint = safe_target
                    last_ori_hint = before_ori
                    last_pose_hint_wall = time.time()
                    after_pos, _after_ori, err_after = _wait_tcp_pose(
                        target=safe_target,
                        tol_m=max(target_tol_abs, 0.02),
                        timeout_sec=tf_settle_sec,
                    )
                    if after_pos is None:
                        return False, f"safe_height:read_after_failed:{err_after}"
                    self._emit_log(
                        "[TEST][FRAME] safe_height lifted "
                        f"before_z={float(before_pos[2]):.3f} after_z={float(after_pos[2]):.3f} "
                        f"min_probe_z={min_probe_z:.3f}"
                    )
                    if float(after_pos[2]) < safe_min_z:
                        if (not strict_frame_measurements) and bool(err_after):
                            self._emit_log(
                                "[TEST][FRAME] WARN "
                                f"safe_height unreliable_tf_sample after={err_after}; non_strict_pass"
                            )
                            return True, "ok_unreliable_tf_sample"
                        return False, (
                            f"safe_height:still_unsafe after_z={float(after_pos[2]):.3f} "
                            f"safe_min={safe_min_z:.3f}"
                        )
                    return True, "ok"

                def _move_probe_anchor() -> Tuple[bool, str]:
                    nonlocal last_pose_hint, last_ori_hint, last_pose_hint_wall
                    if not anchor_enabled:
                        return True, "anchor_disabled"
                    pose_now, ori_now, err_now = _read_tcp_pose()
                    if pose_now is None or ori_now is None:
                        return False, f"anchor:read_pose_failed:{err_now}"
                    if use_world_ref:
                        anchor_target = (
                            float(TABLE_CENTER_X),
                            float(TABLE_CENTER_Y),
                            float(table_top + anchor_z_offset),
                        )
                    else:
                        base_anchor, _tf_anchor = transform_point_to_frame(
                            (TABLE_CENTER_X, TABLE_CENTER_Y, float(table_top + anchor_z_offset)),
                            base_frame,
                            source_frame=WORLD_FRAME or "world",
                            timeout_sec=0.4,
                        )
                        if not base_anchor:
                            return False, "anchor:world_to_base_failed"
                        anchor_target = (
                            float(base_anchor[0]),
                            float(base_anchor[1]),
                            float(base_anchor[2]),
                        )
                    if float(anchor_target[2]) < safe_min_z:
                        anchor_target = (
                            float(anchor_target[0]),
                            float(anchor_target[1]),
                            float(safe_min_z + 0.01),
                        )
                    rid = _next_test_moveit_request_id()
                    ruid = uuid.uuid4().hex
                    pose_data = _make_pose_data(
                        anchor_target,
                        orientation=ori_now,
                        frame=_encode_request_frame(frame_ref, rid, ruid),
                    )
                    _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                    if not self._publish_moveit_pose(
                        "TEST_FRAME_ANCHOR", pose_data, cartesian=False
                    ):
                        return False, "anchor:publish_failed"
                    ok_res, msg_res = self._wait_tfm_moveit_result(
                        "TEST_FRAME_ANCHOR",
                        since_wall=since_wall,
                        since_seq=since_seq,
                        timeout_sec=result_timeout,
                        expected_request_id=rid,
                        expected_request_uuid=ruid,
                    )
                    if not ok_res:
                        return False, f"anchor:result_failed:{msg_res}"
                    last_pose_hint = anchor_target
                    last_ori_hint = ori_now
                    last_pose_hint_wall = time.time()
                    after_pos, _after_ori, err_after = _wait_tcp_pose(
                        target=anchor_target,
                        tol_m=max(target_tol_abs, 0.03),
                        timeout_sec=tf_settle_sec,
                    )
                    if after_pos is None:
                        return False, f"anchor:read_after_failed:{err_after}"
                    self._emit_log(
                        "[TEST][FRAME] anchor "
                        f"target=({anchor_target[0]:+.3f},{anchor_target[1]:+.3f},{anchor_target[2]:+.3f}) "
                        f"tcp=({after_pos[0]:+.3f},{after_pos[1]:+.3f},{after_pos[2]:+.3f}) "
                        f"err={err_after or 'ok'}"
                    )
                    return True, "ok"

                self._emit_log(
                    "[TEST][FRAME] start "
                    f"frame={frame_ref} ee={ee_frame} "
                    f"mode={'world' if use_world_ref else 'base'} "
                    f"xy_step={xy_step:.3f} z_step={z_step:.3f} safe_min_z={safe_min_z:.3f} "
                    f"signs=({sx:+d},{sy:+d},{sz:+d}) settle={tf_settle_sec:.2f}s tol={target_tol_abs:.3f}"
                )
                ok_safe, info_safe = _ensure_probe_safe_height()
                if not ok_safe:
                    return False, info_safe
                ok_anchor, info_anchor = _move_probe_anchor()
                if not ok_anchor:
                    return False, info_anchor
                probe_steps = [
                    ("+X", (xy_step, 0.0, 0.0), 0, +sx),
                    ("-X_RET", (-xy_step, 0.0, 0.0), 0, -sx),
                    ("+Y", (0.0, xy_step, 0.0), 1, +sy),
                    ("-Y_RET", (0.0, -xy_step, 0.0), 1, -sy),
                    ("-Z", (0.0, 0.0, -z_step), 2, -sz),
                    ("+Z_RET", (0.0, 0.0, z_step), 2, +sz),
                ]
                for step_label, delta, axis_idx, axis_sign in probe_steps:
                    ok_step, info_step = _run_delta_step(step_label, delta, axis_idx, axis_sign)
                    if not ok_step:
                        return False, info_step
                self._emit_log("[TEST][FRAME] PASS x/y/z axis probe")
                return True, "ok"

            def _run_table_corner_probe() -> Tuple[bool, str]:
                if not _test_cfg_bool("PANEL_TEST_CORNER_PROBE", True):
                    self._emit_log("[TEST][CORNER] skipped: PANEL_TEST_CORNER_PROBE=0")
                    return True, "skipped_by_env"
                if not self._moveit_required:
                    return False, "moveit_disabled"
                moveit_ready_timeout = max(
                    8.0, _test_cfg_float("PANEL_TEST_MOVEIT_READY_TIMEOUT_SEC", 30.0)
                )
                ok_pipeline, info_pipeline = _ensure_test_moveit_pipeline(
                    timeout_sec=moveit_ready_timeout
                )
                if not ok_pipeline:
                    return False, f"moveit_pipeline_not_ready:{info_pipeline}"
                hover_m = _test_cfg_float("PANEL_TEST_CORNER_HOVER_M", 0.11)
                tol_xy = _test_cfg_float("PANEL_TEST_CORNER_TOL_XY_M", 0.04)
                tol_z = _test_cfg_float("PANEL_TEST_CORNER_TOL_Z_M", 0.05)
                result_timeout = _test_cfg_float(
                    "PANEL_TEST_CORNER_RESULT_TIMEOUT_SEC", 10.0
                )
                max_attempts = max(
                    1, min(8, _test_cfg_int("PANEL_TEST_CORNER_MAX_ATTEMPTS", 4))
                )
                adapt_gain = max(
                    0.15,
                    min(1.0, _test_cfg_float("PANEL_TEST_CORNER_ADAPT_GAIN", 0.65)),
                )
                adapt_xy_cap = max(
                    0.003,
                    min(0.08, _test_cfg_float("PANEL_TEST_CORNER_ADAPT_MAX_XY_M", 0.03)),
                )
                adapt_z_cap = max(
                    0.002,
                    min(0.05, _test_cfg_float("PANEL_TEST_CORNER_ADAPT_MAX_Z_M", 0.02)),
                )
                live_trace = _test_cfg_bool("PANEL_TEST_CORNER_LIVE_TRACE", True)
                hover_m = max(0.04, min(0.25, hover_m))
                tol_xy = max(0.01, min(0.12, tol_xy))
                tol_z = max(0.01, min(0.15, tol_z))
                result_timeout = max(2.0, min(20.0, result_timeout))
                tf_settle_sec = max(
                    0.0, min(2.0, _test_cfg_float("PANEL_TEST_CORNER_TF_SETTLE_SEC", 0.45))
                )
                tf_max_age_sec = float(
                    _test_cfg_float("PANEL_TEST_CORNER_TF_MAX_AGE_SEC", 0.0)
                )
                tf_max_step_jump = max(
                    0.05, min(1.0, _test_cfg_float("PANEL_TEST_CORNER_MAX_STEP_JUMP_M", 0.25))
                )
                tf_strict = _test_cfg_bool("PANEL_TEST_CORNER_STRICT_TF", False)

                base_frame = self._business_base_frame()
                ee_frame = "rg2_tcp"
                helper = get_tf_helper()
                # FASE 1: timeout 0.5s (antes 0.08s) para reducir spam TF
                if helper and not _can_transform_between(
                    helper, base_frame, ee_frame, timeout_sec=0.5
                ):
                    fallback_ee = self._ee_frame_effective or "tool0"
                    if _can_transform_between(
                        helper, base_frame, fallback_ee, timeout_sec=0.5
                    ):
                        ee_frame = fallback_ee
                        self._emit_log(
                            "[TEST][CORNER] WARN "
                            f"ee_preferred=rg2_tcp unavailable; using {fallback_ee}"
                        )
                corners = self._compute_test_corner_base_points()
                if len(corners) < 2:
                    return False, "corner_points_unavailable"

                def _ros_clock_now_ns() -> int:
                    try:
                        if self._ros_worker_started and self.ros_worker.node_ready():
                            with self.ros_worker._lock:
                                node = getattr(self.ros_worker, "_node", None)
                            if node is not None:
                                now_ns = int(node.get_clock().now().nanoseconds)
                                if now_ns > 0:
                                    return now_ns
                    except Exception:
                        pass
                    return 0

                def _read_corner_pose(
                    *,
                    last_hint: Optional[Tuple[float, float, float]] = None,
                    attempts: int = 3,
                ) -> Tuple[Optional[Dict[str, object]], str]:
                    max_attempts_read = max(1, int(attempts))
                    last_err = "pose_unavailable"
                    for read_idx in range(max_attempts_read):
                        pose, err = get_pose(base_frame, ee_frame, timeout_sec=0.35)
                        if not pose:
                            last_err = err or "pose_missing"
                            if read_idx < (max_attempts_read - 1):
                                time.sleep(0.08)
                            continue
                        pos = pose.get("position", (0.0, 0.0, 0.0))
                        if not isinstance(pos, (list, tuple)) or len(pos) < 3:
                            last_err = "pose_invalid_position"
                            if read_idx < (max_attempts_read - 1):
                                time.sleep(0.08)
                            continue
                        px = float(pos[0])
                        py = float(pos[1])
                        pz = float(pos[2])
                        stamp_ns = int(pose.get("stamp_ns", 0) or 0)
                        now_ns = int(_ros_clock_now_ns())
                        if tf_max_age_sec > 0.0 and stamp_ns > 0 and now_ns > 0:
                            age_sec = abs(now_ns - stamp_ns) / 1e9
                            if age_sec > tf_max_age_sec:
                                last_err = (
                                    f"stale_tf age={age_sec:.3f}s max_age={tf_max_age_sec:.3f}s"
                                )
                                if read_idx < (max_attempts_read - 1):
                                    time.sleep(0.08)
                                continue
                        if last_hint is not None:
                            dx = px - float(last_hint[0])
                            dy = py - float(last_hint[1])
                            dz = pz - float(last_hint[2])
                            jump = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
                            if jump > tf_max_step_jump:
                                last_err = (
                                    f"tf_jump jump={jump:.3f} max_jump={tf_max_step_jump:.3f} "
                                    f"sample=({px:.3f},{py:.3f},{pz:.3f}) "
                                    f"last=({float(last_hint[0]):.3f},{float(last_hint[1]):.3f},{float(last_hint[2]):.3f})"
                                )
                                if read_idx < (max_attempts_read - 1):
                                    time.sleep(0.08)
                                continue
                        return pose, ""
                    if (not tf_strict) and last_hint is not None:
                        self._emit_log(
                            "[TEST][CORNER] WARN "
                            f"tf_unstable using_last_hint err={last_err}"
                        )
                        return (
                            {
                                "frame": base_frame,
                                "position": (
                                    float(last_hint[0]),
                                    float(last_hint[1]),
                                    float(last_hint[2]),
                                ),
                                "orientation": (0.0, 0.0, 0.0, 1.0),
                                "stamp_ns": 0,
                            },
                            "fallback_last_hint",
                        )
                    return None, last_err

                self._emit_log(
                    "[TEST][CORNER] start "
                    f"base={base_frame} ee={ee_frame} "
                    f"targets={[(name, round(p[0],3), round(p[1],3), round(p[2],3)) for name, p in corners]} "
                    f"attempts={max_attempts} adapt_gain={adapt_gain:.2f} "
                    f"tf_settle={tf_settle_sec:.2f}s tf_max_age={tf_max_age_sec:.2f}s "
                    f"tf_jump={tf_max_step_jump:.3f} strict={str(tf_strict).lower()}"
                )
                self._audit_append(
                    "logs/test_robot.log",
                    "[TEST][CORNER] START "
                    f"base={base_frame} ee={ee_frame} "
                    f"attempts={max_attempts} adapt_gain={adapt_gain:.2f} "
                    f"tol_xy={tol_xy:.3f} tol_z={tol_z:.3f} "
                    f"tf_settle={tf_settle_sec:.2f} tf_max_age={tf_max_age_sec:.2f} "
                    f"tf_jump={tf_max_step_jump:.3f}",
                )

                table_top = float(self._resolve_table_top_z())
                safe_min_z = 0.08
                table_base = self._ensure_base_coords(
                    (TABLE_CENTER_X, TABLE_CENTER_Y, table_top),
                    self._world_frame_last_first(),
                    timeout_sec=0.35,
                )
                if table_base:
                    safe_min_z = max(safe_min_z, float(table_base[2]) + 0.04)

                for label, coords in corners:
                    pose_now, err_now = _read_corner_pose()
                    if not pose_now:
                        return False, f"{label}:tcp_pose_missing:{err_now or 'n/a'}"
                    ori = pose_now.get("orientation", (0.0, 0.0, 0.0, 1.0))
                    if not isinstance(ori, (list, tuple)) or len(ori) < 4:
                        ori = (0.0, 0.0, 0.0, 1.0)
                    tx = float(coords[0])
                    ty = float(coords[1])
                    tz = max(safe_min_z, float(coords[2]) + hover_m)
                    last_pose_after: Optional[Tuple[float, float, float]] = None
                    for attempt in range(1, max_attempts + 1):
                        rid = _next_test_moveit_request_id()
                        ruid = uuid.uuid4().hex
                        pose_data = _make_pose_data(
                            (tx, ty, tz),
                            orientation=(
                                float(ori[0]),
                                float(ori[1]),
                                float(ori[2]),
                                float(ori[3]),
                            ),
                            frame=_encode_request_frame(base_frame, rid, ruid),
                        )
                        _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                        if not self._publish_moveit_pose(
                            f"TEST_CORNER_{label}_A{attempt}",
                            pose_data,
                            cartesian=False,
                        ):
                            return False, f"{label}:publish_failed:attempt={attempt}"
                        ok_res, msg_res = self._wait_tfm_moveit_result(
                            f"TEST_CORNER_{label}_A{attempt}",
                            since_wall=since_wall,
                            since_seq=since_seq,
                            timeout_sec=result_timeout,
                            expected_request_id=rid,
                            expected_request_uuid=ruid,
                        )
                        if not ok_res:
                            return False, (
                                f"{label}:result_failed:attempt={attempt}:{msg_res}"
                            )
                        if tf_settle_sec > 0.0:
                            time.sleep(tf_settle_sec)
                        pose_after, err_after = _read_corner_pose(
                            last_hint=last_pose_after
                        )
                        if not pose_after:
                            return False, (
                                f"{label}:tcp_after_missing:attempt={attempt}:"
                                f"{err_after or 'n/a'}"
                            )
                        pos_after = pose_after.get("position", (0.0, 0.0, 0.0))
                        if not isinstance(pos_after, (list, tuple)) or len(pos_after) < 3:
                            return False, f"{label}:tcp_after_invalid:attempt={attempt}"
                        ax = float(pos_after[0])
                        ay = float(pos_after[1])
                        az = float(pos_after[2])
                        last_pose_after = (ax, ay, az)
                        ex = float(tx - ax)
                        ey = float(ty - ay)
                        ez = float(tz - az)
                        dxy = math.hypot(ex, ey)
                        dz = abs(ez)
                        self._emit_log(
                            "[TEST][CORNER] "
                            f"step={label} attempt={attempt}/{max_attempts} "
                            f"target=({tx:.3f},{ty:.3f},{tz:.3f}) "
                            f"tcp=({ax:.3f},{ay:.3f},{az:.3f}) "
                            f"err=({ex:+.3f},{ey:+.3f},{ez:+.3f}) "
                            f"dxy={dxy:.3f} dz={dz:.3f} "
                            f"tol_xy={tol_xy:.3f} tol_z={tol_z:.3f}"
                        )
                        self._audit_append(
                            "logs/test_robot.log",
                            "[TEST][CORNER] "
                            f"step={label} attempt={attempt}/{max_attempts} "
                            f"target=({tx:.3f},{ty:.3f},{tz:.3f}) "
                            f"tcp=({ax:.3f},{ay:.3f},{az:.3f}) dxy={dxy:.3f} dz={dz:.3f} "
                            f"tol_xy={tol_xy:.3f} tol_z={tol_z:.3f}",
                        )
                        if dxy <= tol_xy and dz <= tol_z:
                            break
                        if attempt >= max_attempts:
                            return False, (
                                f"{label}:target_miss dxy={dxy:.3f} dz={dz:.3f} "
                                f"attempts={max_attempts}"
                            )
                        corr_x = max(-adapt_xy_cap, min(adapt_xy_cap, ex * adapt_gain))
                        corr_y = max(-adapt_xy_cap, min(adapt_xy_cap, ey * adapt_gain))
                        corr_z = max(-adapt_z_cap, min(adapt_z_cap, ez * adapt_gain))
                        tx = float(tx + corr_x)
                        ty = float(ty + corr_y)
                        tz = max(safe_min_z, float(tz + corr_z))
                        if live_trace:
                            self._emit_log(
                                "[TEST][CORNER] "
                                f"step={label} retry target_update="
                                f"({tx:.3f},{ty:.3f},{tz:.3f}) "
                                f"corr=({corr_x:+.3f},{corr_y:+.3f},{corr_z:+.3f})"
                            )
                self._emit_log("[TEST][CORNER] PASS table front corners reached")
                return True, "ok"

            def _select_test_ee_frame(local_base_frame: str) -> str:
                preferred = "rg2_tcp"
                helper = get_tf_helper()
                # FASE 1: timeout 0.5s (antes 0.08s) para reducir spam TF
                if helper and _can_transform_between(
                    helper, local_base_frame, preferred, timeout_sec=0.5
                ):
                    return preferred
                fallback = self._ee_frame_effective or "tool0"
                if helper and _can_transform_between(
                    helper, local_base_frame, fallback, timeout_sec=0.5
                ):
                    if fallback != preferred:
                        self._emit_log(
                            "[TEST] WARN "
                            f"ee_preferred={preferred} unavailable; using {fallback}"
                        )
                    return fallback
                return preferred

            def _read_tcp_pose_pair(
                world_frame: str,
                base_frame: str,
                ee_frame: str,
            ) -> Tuple[Optional[Tuple[float, float, float]], Optional[Tuple[float, float, float]], str]:
                pose_world, err_world = get_pose(world_frame, ee_frame, timeout_sec=0.35)
                pose_base, err_base = get_pose(base_frame, ee_frame, timeout_sec=0.35)
                if not pose_world or not pose_base:
                    return None, None, (
                        f"world_err={err_world or 'n/a'} base_err={err_base or 'n/a'}"
                    )
                pw = pose_world.get("position", (0.0, 0.0, 0.0))
                pb = pose_base.get("position", (0.0, 0.0, 0.0))
                if (
                    not isinstance(pw, (list, tuple))
                    or len(pw) < 3
                    or not isinstance(pb, (list, tuple))
                    or len(pb) < 3
                ):
                    return None, None, "pose_invalid"
                return (
                    (float(pw[0]), float(pw[1]), float(pw[2])),
                    (float(pb[0]), float(pb[1]), float(pb[2])),
                    "",
                )

            def _run_joint_sweep_probe() -> Tuple[bool, str]:
                if not _test_cfg_bool("PANEL_TEST_JOINT_SWEEP_ENABLED", True):
                    self._emit_log("[TEST][JOINT_SWEEP] skipped")
                    return True, "skipped"
                base_frame = self._business_base_frame()
                ee_frame = _select_test_ee_frame(base_frame)
                delta_deg = max(
                    5.0, min(60.0, _test_cfg_float("PANEL_TEST_JOINT_SWEEP_DELTA_DEG", 30.0))
                )
                delta_rad = math.radians(delta_deg)
                move_sec_probe = max(
                    0.8, _test_cfg_float("PANEL_TEST_JOINT_SWEEP_MOVE_SEC", move_sec)
                )
                settle_sec = max(
                    0.1, min(2.5, _test_cfg_float("PANEL_TEST_JOINT_SWEEP_SETTLE_SEC", 0.35))
                )
                sweep_rows: List[Dict[str, object]] = []
                self._emit_log(
                    "[TEST][JOINT_SWEEP] start "
                    f"delta_deg={delta_deg:.1f} move_sec={move_sec_probe:.2f} settle={settle_sec:.2f} "
                    f"base={base_frame} ee={ee_frame}"
                )
                for idx, joint_name in enumerate(UR5_JOINT_NAMES):
                    for sign in (1.0, -1.0):
                        target = list(home_pose)
                        target[idx] = float(target[idx] + (sign * delta_rad))
                        ok, info = self._publish_joint_trajectory(target, move_sec_probe)
                        if not ok:
                            return False, f"{joint_name}:{'+' if sign > 0 else '-'}:{info}"
                        time.sleep(move_sec_probe + settle_sec)
                        tcp_world, tcp_base, pose_err = _read_tcp_pose_pair(
                            self._world_frame_last_first(), base_frame, ee_frame
                        )
                        if tcp_world is None or tcp_base is None:
                            return False, f"{joint_name}:{'+' if sign > 0 else '-'}:tcp_pose:{pose_err}"
                        row = {
                            "joint": joint_name,
                            "direction": "+" if sign > 0 else "-",
                            "delta_deg": float(sign * delta_deg),
                            "target_rad": float(target[idx]),
                            "tcp_base": [tcp_base[0], tcp_base[1], tcp_base[2]],
                        }
                        sweep_rows.append(row)
                        self._audit_append(
                            "logs/test_robot.log",
                            "[TEST][JOINT_SWEEP] "
                            f"joint={joint_name} dir={row['direction']} delta_deg={row['delta_deg']:+.1f} "
                            f"tcp_base=({tcp_base[0]:.3f},{tcp_base[1]:.3f},{tcp_base[2]:.3f})",
                        )
                    ok_home_j, info_home_j = self._publish_joint_trajectory(home_pose, move_sec_probe)
                    if not ok_home_j:
                        return False, f"{joint_name}:home_recover:{info_home_j}"
                    time.sleep(move_sec_probe + 0.1)
                self._audit_write_json(
                    "artifacts/test_joint_sweep_last.json",
                    {
                        "timestamp": datetime.now().isoformat(timespec="seconds"),
                        "base_frame": base_frame,
                        "ee_frame": ee_frame,
                        "delta_deg": delta_deg,
                        "rows": sweep_rows,
                    },
                )
                self._emit_log(
                    f"[TEST][JOINT_SWEEP] PASS samples={len(sweep_rows)}"
                )
                return True, "ok"

            def _run_table_marker_touch_probe() -> Tuple[bool, str]:
                if not self._moveit_required:
                    return False, "moveit_disabled"
                moveit_ready_timeout = max(
                    2.0, _test_cfg_float("PANEL_TEST_MOVEIT_READY_TIMEOUT_SEC", 5.0)
                )
                ok_pipeline, info_pipeline = _ensure_test_moveit_pipeline(
                    timeout_sec=moveit_ready_timeout
                )
                if not ok_pipeline:
                    return False, f"moveit_pipeline_not_ready:{info_pipeline}"
                base_frame = self._business_base_frame()
                ee_frame = _select_test_ee_frame(base_frame)
                corners = self._compute_test_corner_base_points()
                if len(corners) < 2:
                    return False, "corner_points_unavailable"
                corner_map = {name: point for name, point in corners}
                touch_order_raw = str(
                    _test_cfg_value(
                        "PANEL_TEST_TOUCH_ORDER",
                        "FRONT_LEFT,FRONT_RIGHT",
                    )
                    or "FRONT_LEFT,FRONT_RIGHT"
                )
                touch_order: List[str] = []
                for token in touch_order_raw.split(","):
                    key = token.strip().upper()
                    if key in corner_map and key not in touch_order:
                        touch_order.append(key)
                for fallback_key in ("FRONT_RIGHT", "FRONT_LEFT"):
                    if fallback_key in corner_map and fallback_key not in touch_order:
                        touch_order.append(fallback_key)
                hover_m = max(
                    0.03, min(0.30, _test_cfg_float("PANEL_TEST_TOUCH_HOVER_M", 0.10))
                )
                touch_z_offset = _test_cfg_float("PANEL_TEST_TOUCH_Z_OFFSET_M", 0.004)
                tcp_z_comp = _test_cfg_float("PANEL_TEST_TOUCH_TCP_Z_COMP_M", 0.0)
                touch_min_offset_from_mark = _test_cfg_float(
                    "PANEL_TEST_TOUCH_MIN_OFFSET_FROM_MARK_M", -0.04
                )
                result_timeout = max(
                    2.0, _test_cfg_float("PANEL_TEST_TOUCH_RESULT_TIMEOUT_SEC", 25.0)
                )
                settle_sec = max(
                    0.05, min(2.0, _test_cfg_float("PANEL_TEST_TOUCH_SETTLE_SEC", 0.30))
                )
                tol_xy = max(
                    0.005, min(0.08, _test_cfg_float("PANEL_TEST_TOUCH_TOL_XY_M", 0.020))
                )
                tol_z = max(
                    0.005, min(0.08, _test_cfg_float("PANEL_TEST_TOUCH_TOL_Z_M", 0.020))
                )
                cart_touch = _test_cfg_bool("PANEL_TEST_TOUCH_CARTESIAN", False)
                strict_touch = _test_cfg_bool("PANEL_TEST_TOUCH_STRICT", True)
                max_attempts = max(
                    1, min(8, _test_cfg_int("PANEL_TEST_TOUCH_MAX_ATTEMPTS", 2))
                )
                adapt_gain = max(
                    0.10,
                    min(1.0, _test_cfg_float("PANEL_TEST_TOUCH_ADAPT_GAIN", 0.60)),
                )
                adapt_xy_cap = max(
                    0.002,
                    min(0.12, _test_cfg_float("PANEL_TEST_TOUCH_ADAPT_MAX_XY_M", 0.01)),
                )
                adapt_z_cap = max(
                    0.002,
                    min(0.10, _test_cfg_float("PANEL_TEST_TOUCH_ADAPT_MAX_Z_M", 0.01)),
                )
                live_trace = _test_cfg_bool("PANEL_TEST_TOUCH_LIVE_TRACE", True)
                phase_max_retries = max(
                    1, min(4, _test_cfg_int("PANEL_TEST_TOUCH_PHASE_MAX_RETRIES", 2))
                )
                phase_retry_backoff = max(
                    0.0,
                    min(2.0, _test_cfg_float("PANEL_TEST_TOUCH_PHASE_RETRY_BACKOFF_SEC", 0.35)),
                )
                touch_rows: List[Dict[str, object]] = []

                def _pose_stamp_ns(pose: Optional[Dict[str, object]]) -> int:
                    if not isinstance(pose, dict):
                        return 0
                    try:
                        return int(pose.get("stamp_ns", 0) or 0)
                    except Exception:
                        return 0

                def _read_tcp_pose_pair_fresh(
                    *,
                    min_base_stamp_ns: int = 0,
                    timeout_sec: float = 1.4,
                ) -> Tuple[Optional[Tuple[float, float, float]], str, int]:
                    deadline = time.time() + max(0.25, float(timeout_sec))
                    last_err = "tcp_pose_unavailable"
                    best_base = None
                    best_base_stamp = 0
                    while time.time() < deadline:
                        pose_base, err_base = get_pose(base_frame, ee_frame, timeout_sec=0.25)
                        if not pose_base:
                            last_err = f"base_err={err_base or 'n/a'}"
                            time.sleep(0.04)
                            continue
                        pb = pose_base.get("position", (0.0, 0.0, 0.0))
                        if not isinstance(pb, (list, tuple)) or len(pb) < 3:
                            last_err = "pose_invalid"
                            time.sleep(0.04)
                            continue
                        base_stamp_ns = _pose_stamp_ns(pose_base)
                        best_base = (float(pb[0]), float(pb[1]), float(pb[2]))
                        best_base_stamp = base_stamp_ns
                        fresh_base = (min_base_stamp_ns <= 0) or (
                            base_stamp_ns > min_base_stamp_ns
                        )
                        if fresh_base:
                            return best_base, "", best_base_stamp
                        last_err = (
                            f"stale_tf base_stamp={base_stamp_ns} min_base={min_base_stamp_ns}"
                        )
                        time.sleep(0.04)
                    if best_base is not None:
                        return best_base, f"fresh_timeout:{last_err}", best_base_stamp
                    return None, str(last_err), 0

                # --- Orientación: leer de TF actual (brazo ya en config conocida) ---
                # Usar la orientación actual del TCP para que OMPL mantenga
                # la misma familia de soluciones IK en todos los goals.
                _ori_pose, _ori_err = get_pose(base_frame, ee_frame, timeout_sec=1.0)
                if not _ori_pose:
                    return False, f"ori_tf_read_failed:{_ori_err or 'n/a'}"
                ori = _ori_pose.get("orientation", (0.0, 0.0, 0.0, 1.0))
                if not isinstance(ori, (list, tuple)) or len(ori) < 4:
                    ori = (0.0, 0.0, 0.0, 1.0)

                # --- Joint state sanity check ---
                # gz_ros2_control puede corromper joint_states tras ~500s sim.
                # Detectar early: si TCP TF es imposible, abortar.
                _sanity_pos = _ori_pose.get("position", (0.0, 0.0, 0.0))
                if (
                    isinstance(_sanity_pos, (list, tuple))
                    and len(_sanity_pos) >= 3
                ):
                    _sx, _sy, _sz = (
                        float(_sanity_pos[0]),
                        float(_sanity_pos[1]),
                        float(_sanity_pos[2]),
                    )
                    if _sz < -0.15 or _sz > 0.90 or abs(_sx) > 1.2 or abs(_sy) > 1.2:
                        self._emit_log(
                            f"[TEST][TOUCH] JOINT_CORRUPT tcp=({_sx:.3f},{_sy:.3f},{_sz:.3f})"
                            " - simulación corrupta, reiniciar sistema"
                        )
                        return False, (
                            f"joint_state_corrupt:tcp=({_sx:.3f},{_sy:.3f},{_sz:.3f}):"
                            "reiniciar_simulacion"
                        )

                touch_start_time = time.monotonic()
                marker_results: Dict[str, Dict[str, object]] = {}

                self._audit_append(
                    "logs/test_robot.log",
                    "[TEST][TOUCH] START "
                    f"order={touch_order} hover={hover_m:.3f} z_offset={touch_z_offset:.3f} "
                    f"tcp_z_comp={tcp_z_comp:+.3f} "
                    f"min_offset={touch_min_offset_from_mark:.3f} "
                    f"tol_xy={tol_xy:.3f} tol_z={tol_z:.3f} cart_touch={str(cart_touch).lower()} "
                    f"attempts={max_attempts} k_xy={adapt_gain:.2f} k_z={adapt_gain:.2f} "
                    f"max_step_xy={adapt_xy_cap:.3f} max_step_z={adapt_z_cap:.3f} "
                    f"orientation_tf=({ori[0]:.4f},{ori[1]:.4f},{ori[2]:.4f},{ori[3]:.4f}) "
                    f"phase_retries={phase_max_retries} "
                    f"phase_retry_backoff={phase_retry_backoff:.2f}",
                )
                self._emit_log(
                    "[TEST][TOUCH] refs "
                    + ", ".join(
                        f"{name}=({coords[0]:.3f},{coords[1]:.3f},{coords[2]:.3f})"
                        for name, coords in corners
                    )
                    + f" frame={base_frame} tip_frame={ee_frame}"
                )

                # --- Helper: ejecutar una fase (PRE/TOUCH/RETREAT) con motion lock ---
                def _exec_phase(
                    key: str,
                    phase_name: str,
                    tx: float,
                    ty: float,
                    tz: float,
                    phase_cart: bool,
                ) -> Tuple[bool, str]:
                    for phase_try in range(1, phase_max_retries + 1):
                        stage_key = f"{key}:{phase_name}:try{phase_try}"
                        if _test_timed_out(stage_key):
                            return False, _test_timeout_reason(stage_key)
                        rid = _next_test_moveit_request_id()
                        ruid = (
                            f"test_touch:{key.lower()}:{phase_name.lower()}:"
                            f"{uuid.uuid4().hex}"
                        )
                        pose_data = _make_pose_data(
                            (tx, ty, tz),
                            orientation=(
                                float(ori[0]),
                                float(ori[1]),
                                float(ori[2]),
                                float(ori[3]),
                            ),
                            frame=_encode_request_frame(base_frame, rid, ruid),
                        )
                        _raw, since_wall, since_seq = self.ros_worker.moveit_result_snapshot()
                        if not self._publish_moveit_pose(
                            f"TEST_TOUCH_{key}_{phase_name}",
                            pose_data,
                            cartesian=phase_cart,
                        ):
                            block_reason = (
                                self._moveit_block_reason
                                or self._moveit_not_ready_reason()
                                or "unknown"
                            )
                            self._emit_log(
                                "[TEST][TOUCH] publish_failed "
                                f"marker={key} phase={phase_name} reason={block_reason}"
                            )
                            return False, f"publish_failed:{block_reason}"
                        remaining_timeout = _test_deadline_remaining_sec()
                        if remaining_timeout <= 0.0:
                            return False, _test_timeout_reason(stage_key)
                        ok_res, msg_res = self._wait_tfm_moveit_result(
                            f"TEST_TOUCH_{key}_{phase_name}",
                            since_wall=since_wall,
                            since_seq=since_seq,
                            timeout_sec=min(result_timeout, max(0.2, remaining_timeout)),
                            expected_request_id=rid,
                            expected_request_uuid=ruid,
                        )
                        if ok_res:
                            if phase_name == "PRE":
                                pre_wait_timeout = min(6.0, max(1.0, settle_sec + 2.0))
                                pre_wait_ok, pre_wait_pos, pre_wait_dist = self._wait_for_tcp_base_target(
                                    (tx, ty, tz),
                                    timeout_sec=pre_wait_timeout,
                                    tol_xyz_m=0.10,
                                    ee_frame=ee_frame,
                                )
                                if pre_wait_ok:
                                    return True, "ok"
                                if pre_wait_pos is not None and math.isfinite(pre_wait_dist):
                                    self._emit_log(
                                        f"[TEST][TOUCH] PRE_WAIT marker={key} "
                                        f"tcp=({pre_wait_pos[0]:.3f},{pre_wait_pos[1]:.3f},{pre_wait_pos[2]:.3f}) "
                                        f"target=({tx:.3f},{ty:.3f},{tz:.3f}) dist={pre_wait_dist:.3f}"
                                    )
                                else:
                                    self._emit_log(
                                        f"[TEST][TOUCH] PRE_WAIT marker={key} tcp=unavailable "
                                        f"target=({tx:.3f},{ty:.3f},{tz:.3f})"
                                    )
                                return True, "ok"
                            return True, "ok"
                        phase_msg = str(msg_res or "result_failed")
                        msg_low = phase_msg.lower()
                        retryable_timeout = (
                            "fjt_result_timeout" in msg_low
                            or "execute_timeout" in msg_low
                            or "timeout" in msg_low
                        )
                        if phase_try < phase_max_retries and retryable_timeout:
                            self._emit_log(
                                "[TEST][TOUCH] retry_phase "
                                f"marker={key} phase={phase_name} "
                                f"try={phase_try + 1}/{phase_max_retries} "
                                f"prev={phase_msg}"
                            )
                            if phase_retry_backoff > 0.0:
                                time.sleep(min(phase_retry_backoff, max(0.0, _test_deadline_remaining_sec())))
                            continue
                        return False, phase_msg
                    return False, "max_phase_retries"

                for key in touch_order:
                    if _test_timed_out(f"touch_order:{key}"):
                        return False, _test_timeout_reason(f"touch_order:{key}")
                    target_base = corner_map.get(key)
                    if not target_base:
                        return False, f"{key}:missing_base_target"
                    tx = float(target_base[0])
                    ty = float(target_base[1])
                    z_touch = float(target_base[2] + touch_z_offset)
                    z_touch_floor = float(target_base[2] + touch_min_offset_from_mark)
                    marker_pass = False
                    last_dxy = 0.0
                    last_dz = 0.0
                    marker_start = time.monotonic()

                    for attempt in range(1, max_attempts + 1):
                        z_hover = float(z_touch + hover_m)

                        # --- FASE PRE: mover encima del target (OMPL - puede ir lejos) ---
                        pre_converged = False
                        pre_retry_count = 0
                        max_pre_retries = 3
                        
                        while not pre_converged and pre_retry_count < max_pre_retries:
                            phase_ok, phase_msg = _exec_phase(key, "PRE", tx, ty, z_hover, False)
                            if not phase_ok:
                                return False, f"{key}:PRE:result_failed:{phase_msg}"
                            if settle_sec > 0.0:
                                time.sleep(settle_sec)

                            # Validate PRE position via TF before TOUCH
                            pre_pose, _pre_err = get_pose(base_frame, ee_frame, timeout_sec=1.0)
                            if pre_pose:
                                pre_pos = pre_pose.get("position", (0, 0, 0))
                                pre_dist = math.sqrt(
                                    (pre_pos[0] - tx) ** 2
                                    + (pre_pos[1] - ty) ** 2
                                    + (pre_pos[2] - z_hover) ** 2
                                )
                                self._emit_log(
                                    f"[TEST][TOUCH] PRE_CHECK marker={key} "
                                    f"pre_pos=({pre_pos[0]:.3f},{pre_pos[1]:.3f},{pre_pos[2]:.3f}) "
                                    f"pre_target=({tx:.3f},{ty:.3f},{z_hover:.3f}) "
                                    f"dist={pre_dist:.3f}"
                                )
                                if pre_dist > 0.10:
                                    pre_retry_count += 1
                                    self._emit_log(
                                        f"[TEST][TOUCH] PRE_FAR marker={key} "
                                        f"dist={pre_dist:.3f} > 0.10 - reintentar PRE "
                                        f"retry={pre_retry_count}/{max_pre_retries}"
                                    )
                                    if pre_retry_count < max_pre_retries:
                                        time.sleep(0.5)
                                        continue
                                    else:
                                        self._emit_log(
                                            f"[TEST][TOUCH] PRE_FAIL marker={key} "
                                            f"max_retries={max_pre_retries} exceeded - abortar intento"
                                        )
                                        break
                                else:
                                    pre_converged = True
                            else:
                                pre_converged = True
                        
                        # Si PRE no convergió después de reintentos, abandonar este intento
                        if not pre_converged:
                            continue

                        # --- FASE TOUCH: descender al target (Cartesian - straight line) ---
                        phase_ok, phase_msg = _exec_phase(key, "TOUCH", tx, ty, z_touch, True)
                        if not phase_ok:
                            # Cartesian fallback: si falla Cartesian, reintentar con OMPL
                            self._emit_log(
                                f"[TEST][TOUCH] CART_FAIL marker={key} "
                                f"falling back to OMPL: {phase_msg}"
                            )
                            phase_ok, phase_msg = _exec_phase(key, "TOUCH", tx, ty, z_touch, cart_touch)
                            if not phase_ok:
                                return False, f"{key}:TOUCH:result_failed:{phase_msg}"

                        # Settle generoso para que el TF buffer del panel alcance
                        # al sim_time de Gazebo (headless corre ~10x realtime).
                        time.sleep(3.0)

                        # Leer TCP sin filtro de freshness (min_base_stamp_ns=0).
                        # Con Gazebo headless ≥10x, el stamp-check basado en timestamps
                        # devuelve datos mid-trajectory antes de que el buffer se actualice.
                        tcp_base_touch, pose_err, touch_base_stamp_ns = _read_tcp_pose_pair_fresh(
                            min_base_stamp_ns=0,
                            timeout_sec=2.0,
                        )
                        if tcp_base_touch is None:
                            return False, f"{key}:touch_pose:{pose_err}"

                        # Si la lectura está lejos del target (>0.05m), el buffer
                        # probablemente no alcanzó; reintentar con delays crecientes.
                        for _tf_retry in range(3):
                            _raw_dist = math.sqrt(
                                (tcp_base_touch[0] - tx) ** 2
                                + (tcp_base_touch[1] - ty) ** 2
                                + (tcp_base_touch[2] - z_touch) ** 2
                            )
                            if _raw_dist <= 0.05:
                                break
                            self._emit_log(
                                f"[TEST][TOUCH] TF_LAG marker={key} "
                                f"dist={_raw_dist:.3f} retry={_tf_retry + 1}/3 "
                                f"tcp=({tcp_base_touch[0]:.3f},{tcp_base_touch[1]:.3f},{tcp_base_touch[2]:.3f})"
                            )
                            time.sleep(2.0)
                            tcp2, pe2, ts2 = _read_tcp_pose_pair_fresh(
                                min_base_stamp_ns=0, timeout_sec=2.0,
                            )
                            if tcp2 is not None:
                                _d2 = math.sqrt(
                                    (tcp2[0] - tx) ** 2
                                    + (tcp2[1] - ty) ** 2
                                    + (tcp2[2] - z_touch) ** 2
                                )
                                if _d2 < _raw_dist:
                                    tcp_base_touch = tcp2
                                    pose_err = pe2
                                    touch_base_stamp_ns = ts2

                        # --- FASE RETREAT: subir de vuelta (Cartesian - straight line) ---
                        phase_ok, phase_msg = _exec_phase(key, "RETREAT", tx, ty, z_hover, True)
                        if not phase_ok:
                            # Cartesian fallback to OMPL for retreat
                            phase_ok, phase_msg = _exec_phase(key, "RETREAT", tx, ty, z_hover, False)
                            if not phase_ok:
                                return False, f"{key}:RETREAT:result_failed:{phase_msg}"

                        # --- EVALUACION DE ERROR ---
                        tcp_eval_x = float(tcp_base_touch[0])
                        tcp_eval_y = float(tcp_base_touch[1])
                        tcp_eval_z = float(tcp_base_touch[2]) + float(tcp_z_comp)
                        ex = tx - tcp_eval_x
                        ey = ty - tcp_eval_y
                        ez = z_touch - tcp_eval_z
                        dxy = math.hypot(ex, ey)
                        dz = abs(ez)
                        last_dxy = dxy
                        last_dz = dz

                        touch_rows.append(
                            {
                                "marker": key,
                                "attempt": int(attempt),
                                "target_base": [tx, ty, z_touch],
                                "tcp_base": [
                                    float(tcp_base_touch[0]),
                                    float(tcp_base_touch[1]),
                                    float(tcp_base_touch[2]),
                                ],
                                "tcp_eval_base": [
                                    float(tcp_eval_x),
                                    float(tcp_eval_y),
                                    float(tcp_eval_z),
                                ],
                                "error_base": [ex, ey, ez],
                                "dxy": dxy,
                                "dz": dz,
                                "touch_base_stamp_ns": int(touch_base_stamp_ns),
                            }
                        )
                        touch_line = (
                            "[TEST][TOUCH] "
                            f"marker={key} attempt={attempt}/{max_attempts} "
                            f"target_base=({tx:.3f},{ty:.3f},{z_touch:.3f}) "
                            f"tcp_base=({tcp_base_touch[0]:.3f},{tcp_base_touch[1]:.3f},{tcp_base_touch[2]:.3f}) "
                            f"tcp_eval=({tcp_eval_x:.3f},{tcp_eval_y:.3f},{tcp_eval_z:.3f}) "
                            f"err=({ex:+.3f},{ey:+.3f},{ez:+.3f}) dxy={dxy:.3f} dz={dz:.3f} "
                            f"comp_z={tcp_z_comp:+.3f} "
                            f"stamp_base={int(touch_base_stamp_ns)}"
                        )
                        self._emit_log(touch_line)
                        self._audit_append(
                            "logs/test_robot.log",
                            touch_line,
                        )

                        # --- PASS CHECK ---
                        if (not strict_touch) or (dxy <= tol_xy and dz <= tol_z):
                            elapsed_marker = time.monotonic() - marker_start
                            self._emit_log(
                                f"[TEST][TOUCH] TOUCH_{key} PASS dxy={dxy:.3f} dz={dz:.3f} "
                                f"tol_xy={tol_xy:.3f} tol_z={tol_z:.3f} "
                                f"attempts={attempt} time={elapsed_marker:.1f}s"
                            )
                            marker_pass = True
                            marker_results[key] = {
                                "status": "PASS",
                                "attempts": attempt,
                                "dxy": dxy,
                                "dz": dz,
                                "time_s": round(elapsed_marker, 1),
                            }
                            break

                        # --- FAIL-FAST: si llegamos al ultimo intento, abortar con diagnostico ---
                        if attempt >= max_attempts:
                            elapsed_marker = time.monotonic() - marker_start
                            diag = (
                                f"TEST_FAIL_FAST marker={key}: dxy={dxy:.3f} dz={dz:.3f} "
                                f"(tol_xy={tol_xy:.3f} tol_z={tol_z:.3f}) "
                                f"after {max_attempts} attempts. "
                                f"Revisar: tip_frame={ee_frame}, "
                                f"ref=({target_base[0]:.3f},{target_base[1]:.3f},{target_base[2]:.3f}), "
                                f"orientacion_tf=({ori[0]:.4f},{ori[1]:.4f},{ori[2]:.4f},{ori[3]:.4f}), "
                                f"colisiones SRDF."
                            )
                            self._emit_log(f"[TEST][TOUCH] {diag}")
                            self._audit_append("logs/test_robot.log", f"[TEST][TOUCH] {diag}")
                            marker_results[key] = {
                                "status": "FAIL",
                                "attempts": attempt,
                                "dxy": dxy,
                                "dz": dz,
                                "time_s": round(elapsed_marker, 1),
                            }
                            break

                        # --- CORRECCION PROPORCIONAL SATURADA ---
                        corr_x = max(-adapt_xy_cap, min(adapt_xy_cap, ex * adapt_gain))
                        corr_y = max(-adapt_xy_cap, min(adapt_xy_cap, ey * adapt_gain))
                        corr_z = max(-adapt_z_cap, min(adapt_z_cap, ez * adapt_gain))
                        tx = float(tx + corr_x)
                        ty = float(ty + corr_y)
                        z_touch = float(max(z_touch_floor, z_touch + corr_z))
                        if live_trace:
                            self._emit_log(
                                "[TEST][TOUCH] "
                                f"marker={key} retry target_update="
                                f"({tx:.3f},{ty:.3f},{z_touch:.3f}) "
                                f"corr=({corr_x:+.3f},{corr_y:+.3f},{corr_z:+.3f}) "
                                f"k_xy={adapt_gain:.2f} k_z={adapt_gain:.2f} "
                                f"max_step_xy={adapt_xy_cap:.3f} max_step_z={adapt_z_cap:.3f}"
                            )

                    if strict_touch and not marker_pass:
                        return False, (
                            f"{key}:touch_miss dxy={last_dxy:.3f} dz={last_dz:.3f} "
                            f"tol_xy={tol_xy:.3f} tol_z={tol_z:.3f} attempts={max_attempts}"
                        )

                # --- RESUMEN FINAL ---
                total_time = time.monotonic() - touch_start_time
                summary_parts = []
                for mk in touch_order:
                    mr = marker_results.get(mk, {})
                    summary_parts.append(
                        f"{mk}={mr.get('status','?')} "
                        f"attempts={mr.get('attempts','?')} "
                        f"dxy={mr.get('dxy',0):.3f} dz={mr.get('dz',0):.3f}"
                    )
                summary_line = (
                    "[TEST][TOUCH] TEST_RESULT: "
                    + ", ".join(summary_parts)
                    + f", total_time={total_time:.1f}s"
                )
                self._emit_log(summary_line)
                self._audit_append("logs/test_robot.log", summary_line)

                self._audit_write_json(
                    "artifacts/test_marker_touch_last.json",
                    {
                        "timestamp": datetime.now().isoformat(timespec="seconds"),
                        "base_frame": base_frame,
                        "ee_frame": ee_frame,
                        "order": touch_order,
                        "rows": touch_rows,
                        "summary": marker_results,
                        "total_time_s": round(total_time, 1),
                    },
                )
                self._emit_log("[TEST][TOUCH] PASS markers touched")
                return True, "ok"

            def worker():
                try:
                    table_check_enabled = _test_cfg_bool(
                        "PANEL_TEST_TABLE_CHECK_ENABLED", True
                    )
                    table_check_skip_home = _test_cfg_bool(
                        "PANEL_TEST_TABLE_CHECK_SKIP_HOME", True
                    )
                    table_check_xy_margin = max(
                        0.0, _test_cfg_float("PANEL_TEST_TABLE_XY_MARGIN_M", 0.00)
                    )
                    table_check_z_eps = max(
                        0.002, _test_cfg_float("PANEL_TEST_TABLE_Z_EPS_M", 0.01)
                    )
                    table_check_mode = str(
                        _test_cfg_value("PANEL_TEST_TABLE_MODE", "list") or "list"
                    ).strip().lower()
                    table_check_frames_raw = str(
                        _test_cfg_value(
                            "PANEL_TEST_TABLE_FRAMES",
                            "rg2_tcp,tool0,ee_link,flange,gripper_link,wrist_3_link",
                        )
                        or ""
                    )
                    table_check_frames = [
                        part.strip()
                        for part in table_check_frames_raw.split(",")
                        if part.strip()
                    ]
                    self._emit_log(
                        "[TEST][SAFETY] "
                        f"table_check enabled={str(table_check_enabled).lower()} "
                        f"skip_home={str(table_check_skip_home).lower()} "
                        f"mode={table_check_mode} xy_margin={table_check_xy_margin:.3f} "
                        f"z_eps={table_check_z_eps:.3f} frames={table_check_frames}"
                    )
                    frame_probe_done = False
                    test_move_sec = max(
                        0.5, _test_cfg_float("PANEL_TEST_MOVE_SEC", min(float(move_sec), 1.5))
                    )

                    # Modo simple por defecto: mover cada articulacion +-5% de su
                    # semirango fisico (segun limites), volver a HOME y finalizar.
                    # Para volver al flujo legado completo, exportar:
                    # PANEL_TEST_SIMPLE_JOINT_PERCENT=0
                    simple_joint_percent_mode = _test_cfg_bool(
                        "PANEL_TEST_SIMPLE_JOINT_PERCENT", True
                    )

                    def _run_joint_step(label: str, target: List[float]) -> Tuple[bool, str]:
                        self._ui_set_status(f"TEST ROBOT: {label}…")
                        ok_local, info_local = self._publish_joint_trajectory(target, test_move_sec)
                        if not ok_local:
                            return False, f"{label} ({info_local})"
                        time.sleep(test_move_sec + 0.15)
                        do_table_check = table_check_enabled and not (
                            table_check_skip_home and label == "HOME"
                        )
                        if do_table_check:
                            table_top_local = self._resolve_table_top_z()
                            ok_table_local, table_info_local = self._check_robot_above_table(
                                table_top_local,
                                label=label,
                                xy_margin=table_check_xy_margin,
                                z_eps=table_check_z_eps,
                                monitor_mode=table_check_mode,
                                monitor_frames=table_check_frames,
                            )
                            if not ok_table_local:
                                self._audit_append(
                                    "logs/test_robot.log",
                                    f"[TEST] TABLE_FAIL label={label} info={table_info_local}",
                                )
                                self._emit_log(
                                    f"[SAFETY] TEST ROBOT: robot bajo mesa ({table_info_local})"
                                )
                                return False, f"{label} (limite mesa)"
                        else:
                            self._emit_log(
                                "[TEST][SAFETY] "
                                f"table_check skipped label={label} "
                                f"enabled={str(table_check_enabled).lower()} "
                                f"skip_home={str(table_check_skip_home).lower()}"
                            )
                        return True, "ok"

                    def _run_simple_joint_percent_test() -> Tuple[bool, str]:
                        percent = max(
                            0.01,
                            min(
                                0.30,
                                _test_cfg_float("PANEL_TEST_JOINT_PERCENT", 0.05),
                            ),
                        )
                        self._emit_log(
                            "[TEST][SIMPLE] start "
                            f"joint_percent={percent * 100.0:.1f}% move_sec={test_move_sec:.2f}"
                        )
                        ok_home_ini, info_home_ini = _run_joint_step("HOME", home_pose)
                        if not ok_home_ini:
                            return False, f"HOME inicial ({info_home_ini})"

                        for idx, joint_name in enumerate(UR5_JOINT_NAMES):
                            lo, hi = self._UR5_JOINT_POS_LIMITS[idx]
                            # 5% del semirango para evitar desplazamientos excesivos.
                            delta = max(0.02, min(0.60, min(abs(lo), abs(hi)) * percent))
                            for sign, suffix in ((+1.0, "PLUS"), (-1.0, "MINUS")):
                                target = list(home_pose)
                                target[idx] = float(home_pose[idx] + (sign * delta))
                                ok_step, info_step = _run_joint_step(
                                    f"{joint_name}_{suffix}", target
                                )
                                if not ok_step:
                                    return False, f"{joint_name}_{suffix} ({info_step})"

                        ok_home_end, info_home_end = _run_joint_step("HOME", home_pose)
                        if not ok_home_end:
                            return False, f"HOME final ({info_home_end})"

                        self._emit_log("[TEST][SIMPLE] PASS joint percent sweep")
                        return True, "ok"

                    if simple_joint_percent_mode:
                        ok_simple, info_simple = _run_simple_joint_percent_test()
                        if not ok_simple:
                            self._set_test_failed(info_simple)
                            self._log_warning(
                                f"[ROBOT] TEST ROBOT simple falló: {info_simple}"
                            )
                            return
                        self._ui_set_status("TEST ROBOT completado")
                        self._robot_test_substate = "DONE"
                        self._robot_test_last_failure = ""
                        self._emit_log("[STATE] TEST_FAILED=none")
                        self._audit_append(
                            "logs/test_robot.log",
                            "[TEST] RESULT=PASS",
                        )
                        QTimer.singleShot(0, lambda: self._set_robot_test_done(True))
                        return

                    # 0) HOME inicial.
                    ok_step, info_step = _run_joint_step("HOME", home_pose)
                    if not ok_step:
                        self._set_test_failed(info_step)
                        self._log_warning(f"[ROBOT] TEST ROBOT falló en HOME inicial: {info_step}")
                        return

                    # 1) Secuencia obligatoria:
                    # HOME -> TOUCH_LEFT -> TOUCH_RIGHT -> HOME.
                    self._emit_log(
                        "[TEST][SEQ] HOME->TOUCH_LEFT->TOUCH_RIGHT->HOME"
                    )
                    self._ui_set_status("TEST ROBOT: tocar marca izquierda/derecha…")
                    ok_touch, info_touch = _run_table_marker_touch_probe()
                    if not ok_touch:
                        _moveit_unavailable = any(
                            s in info_touch
                            for s in ("moveit_disabled", "moveit_pipeline_not_ready", "moveit_bridge_not_detected")
                        )
                        if _moveit_unavailable:
                            self._emit_log(
                                f"[TEST] TOUCH_PROBE omitido (MoveIt no disponible): {info_touch}"
                            )
                        else:
                            reason = f"TOUCH_PROBE ({info_touch})"
                            self._set_test_failed(reason)
                            self._log_warning(f"[ROBOT] TEST ROBOT touch probe falló: {info_touch}")
                            return

                    ok_flow, info_flow = _run_joint_step("HOME", home_pose)
                    if not ok_flow:
                        self._set_test_failed(info_flow)
                        self._log_warning(f"[ROBOT] TEST ROBOT falló en HOME final: {info_flow}")
                        return

                    if _test_cfg_bool("PANEL_TEST_GRIPPER_SMOKE", False):
                        self._ui_set_status("TEST ROBOT: probando pinza…")
                        self._emit_log("[TEST] Probando pinza (abrir/cerrar)")
                        QTimer.singleShot(
                            0, lambda: self._command_gripper(False, log_action="TEST", force=True)
                        )
                        time.sleep(0.4)
                        QTimer.singleShot(
                            0, lambda: self._command_gripper(True, log_action="TEST", force=True)
                        )
                        time.sleep(0.4)
                        QTimer.singleShot(
                            0, lambda: self._command_gripper(False, log_action="TEST", force=True)
                        )
                        time.sleep(0.4)
                    self._ui_set_status("TEST ROBOT completado")
                    self._robot_test_substate = "DONE"
                    self._robot_test_last_failure = ""
                    self._emit_log("[STATE] TEST_FAILED=none")
                    self._audit_append(
                        "logs/test_robot.log",
                        "[TEST] RESULT=PASS",
                    )
                    QTimer.singleShot(0, lambda: self._set_robot_test_done(True))
                except Exception as exc:
                    reason = f"EXCEPTION ({type(exc).__name__}: {exc})"
                    self._set_test_failed(reason)
                    tb_txt = traceback.format_exc(limit=8)
                    self._emit_log(f"[ERROR][TEST] {reason}")
                    self._audit_append("logs/test_robot.log", f"[TEST] EXCEPTION {reason}")
                    for line in tb_txt.splitlines():
                        if line.strip():
                            self._audit_append("logs/test_robot.log", f"[TEST][TRACE] {line}")
                finally:
                    self._emit_log("[TEST] Volviendo a HOME tras TEST ROBOT")
                    try:
                        ok_home, info_home = self._publish_joint_trajectory(home_pose, move_sec)
                        if not ok_home:
                            self._log_warning(f"[ROBOT] TEST ROBOT HOME falló: {info_home}")
                    except Exception as exc:
                        _log_exception("test robot home", exc)
                    if restart_moveit_bridge:
                        self._emit_log("[SAFETY] Reanudando MoveIt bridge tras TEST ROBOT")
                        QTimer.singleShot(0, self._start_moveit_bridge)
                    self._set_motion_lock(False)
                    # FASE 5: Marcar _robot_test_done síncronamente ANTES de
                    # liberar _robot_test_active para evitar race condition
                    # donde _update_panel_flow_state evalúa el estado entre
                    # _robot_test_active=False y el QTimer de _set_robot_test_done.
                    if self._robot_test_substate == "DONE":
                        self._robot_test_done = True
                        # Blindaje: bloquear TEST también por estado persistente,
                        # aunque el callback en UI thread se retrase.
                        self._robot_test_disabled = True
                    self._robot_test_active = False
                    set_test_read_only(False, reason="TEST ROBOT end")
                    recalc_object_states("test_robot")
                    self._refresh_objects_from_gz_async()

            self._run_async(worker)

        if externals:
            if all(pub in moveit_nodes for pub in externals):
                self._emit_log(
                    "[SAFETY] TEST ROBOT: publishers de MoveIt detectados en trayectoria; "
                    "se permite ejecución controlada para validar ejes"
                )
                start_sequence(restart_moveit_bridge=False)
                return
            extra = ", ".join(externals)
            tip = f"Publishers externos en {traj_topic}: {extra}"
            self._set_robot_test_blocked(tip)
            self._emit_log(f"[SAFETY] TEST ROBOT bloqueado: {tip}")
            self._schedule_robot_test_cleanup_check(traj_topic)
            return

        start_sequence(restart_moveit_bridge=False)

    @pyqtSlot(str)
    def _on_remote_robot_test_request(self, source: str) -> None:
        src = (source or "unknown").strip()
        self._emit_log(f"[TEST][REMOTE] trigger={src}")
        self._audit_append("logs/test_robot.log", f"[TEST][REMOTE] trigger={src}")
        self._run_robot_test()

    @pyqtSlot(str)
    def _on_remote_tfm_infer_request(self, source: str) -> None:
        src = (source or "unknown").strip()
        self._emit_log(f"[TFM][REMOTE] trigger={src}")
        self._tfm_infer_grasp()

    @pyqtSlot(str)
    def _on_remote_pick_object_request(self, source: str) -> None:
        src = (source or "unknown").strip()
        self._emit_log(f"[PICK][REMOTE] trigger={src}")
        self._run_pick_object()

    @pyqtSlot(str, str)
    def _on_remote_object_select_request(self, name: str, source: str) -> None:
        src = (source or "unknown").strip()
        target = (name or "").strip()
        self._emit_log(f"[PICK][REMOTE] trigger={src} name={target or 'clear'}")
        request_id = ""
        if src.startswith("service:") and "#" in src:
            request_id = src.rsplit("#", 1)[-1].strip()

        def _ack(success: bool, message: str) -> None:
            if not request_id:
                return
            self._emit_log(
                f"[PICK][REMOTE][ACK] request_id={request_id} success={str(bool(success)).lower()} message={message or 'n/a'}"
            )
            if getattr(self, "_ros_worker_started", False) and getattr(self, "ros_worker", None) is not None:
                try:
                    self.ros_worker.complete_object_select_request(request_id, success, message)
                except Exception:
                    pass

        if target.lower() in ("", "clear", "none", "null"):
            if self._selected_object:
                prev_state = get_object_state(self._selected_object)
                if prev_state and prev_state.logical_state == ObjectLogicalState.SELECTED:
                    update_object_state(
                        self._selected_object,
                        logical_state=ObjectLogicalState.ON_TABLE,
                        reason="remote_clear",
                    )
            self._selected_object = None
            self._selected_px = None
            self._selected_world = None
            self._selected_base = None
            self._selection_timestamp = 0.0
            self._selection_last_user_name = ""
            self._selection_last_user_ts = 0.0
            self._last_cornell = None
            self._last_cornell_reason = "Selección limpiada remotamente"
            self._refresh_science_ui()
            self._update_objects()
            self._set_status("Selección remota limpiada", error=False)
            _ack(True, "selection_cleared")
            return
        live_pose = None
        if getattr(self, "_ros_worker_started", False) and getattr(self, "ros_worker", None) is not None:
            try:
                poses, _ts = self.ros_worker.pose_snapshot()
            except Exception:
                poses = {}
            live_pose = poses.get(target)
        if live_pose is not None and len(live_pose) >= 3:
            x, y, z = float(live_pose[0]), float(live_pose[1]), float(live_pose[2])
            if self._objects_settled or is_on_table((x, y, z)):
                bulk_update_object_positions(
                    {target: (x, y, z)},
                    source="remote_pose_select",
                    objects_stable=True,
                )
                recalc_object_states("remote_select_pose_sync")
            px, py = -1, -1
            w = getattr(self.camera_view, "_img_width", 0) if hasattr(self, "camera_view") else 0
            h = getattr(self.camera_view, "_img_height", 0) if hasattr(self, "camera_view") else 0
            if self._camera_stream_ok and self._pose_info_ok and w > 0 and h > 0:
                pix = world_xyz_to_pixel(x, y, z, w, h) or table_xy_to_pixel(x, y, w, h)
                if pix:
                    px, py = int(pix[0]), int(pix[1])
            self._select_object(target, px, py, x, y, z, source="remote_pose")
            if getattr(self, "_selected_object", None) == target:
                _ack(True, "selected")
            else:
                _ack(False, f"selection_rejected:{target}")
            return
        self._on_object_clicked(target)
        if getattr(self, "_selected_object", None) == target:
            _ack(True, "selected")
        else:
            _ack(False, f"selection_rejected:{target}")

    def _set_test_failed(self, reason: str) -> None:
        reason_txt = (reason or "unknown").strip()
        self._robot_test_substate = "FAILED"
        self._robot_test_last_failure = reason_txt
        self._ui_set_status(f"TEST ROBOT falló: {reason_txt}", error=True)
        self._emit_log(f"[STATE] TEST_FAILED(reason={reason_txt})")
        self._audit_append(
            "logs/test_robot.log",
            f"[TEST] RESULT=FAIL reason={reason_txt}",
        )
        # TEST failure must not latch global ERROR; keep system operational.
        if self._system_error_reason.startswith("drop:"):
            self._system_error_reason = ""
        if (
            not self._managed_mode
            and self._system_state != SystemState.ERROR_FATAL
            and self._gazebo_state() == "GAZEBO_READY"
            and self._controllers_ok
        ):
            self._set_system_state(SystemState.READY_VISION, f"test_failed: {reason_txt}")

    def _set_robot_test_done(self, done: bool) -> None:
        self._robot_test_done = done
        if done:
            self._robot_test_disabled = True
            self._robot_test_active = False
            # FASE 5: Log claro del cambio de estado TEST -> PICK.
            self._emit_log("[STATE] TEST_PASSED enabling PICK disabling TEST")
            if self.btn_test_robot:
                self.btn_test_robot.setEnabled(False)
                self.btn_test_robot.setToolTip("TEST ROBOT ya ejecutado")
            # FASE 5: Habilitar PICK buttons explícitamente.
            pick_ok, _pick_reason = self._moveit_control_status()
            pick_enabled = pick_ok and bool(self._ee_frame_effective)
            if pick_enabled:
                demo_ready = (
                    self._moveit_ready()
                    and self._controllers_ok
                    and self._tf_ready_state
                    and bool(self._ee_frame_effective)
                    and not self._pick_demo_executed
                )
                if demo_ready:
                    self.btn_pick_demo.setEnabled(True)
                    self.btn_pick_demo.setToolTip("Demo (sin exigir estabilización de objetos)")
                pick_obj_ready = (
                    pick_ok
                    and self._pose_info_ok
                    and self._tf_ready_state
                    and self._camera_stream_ok
                    and bool(self._selected_object)
                )
                if pick_obj_ready:
                    self.btn_pick_object.setEnabled(True)
                    self.btn_pick_object.setToolTip("PICK objeto seleccionado")
                self._emit_log("[STATE] PICK_READY: PICK buttons enabled")
            # FASE 5: HOME/MESA/CESTA también se habilitan.
            self.btn_home.setEnabled(True)
            self.btn_table.setEnabled(True)
            self.btn_basket.setEnabled(True)
        self._refresh_controls()

    def _set_panel_flow_state(self, state: str, reason: str = "") -> None:
        state_txt = str(state or "BOOT").strip().upper() or "BOOT"
        reason_txt = str(reason or "").strip()
        prev_state = str(getattr(self, "_panel_flow_state", "BOOT") or "BOOT")
        prev_reason = str(getattr(self, "_panel_flow_reason", "") or "")
        self._panel_flow_state = state_txt
        self._panel_flow_reason = reason_txt
        if state_txt != prev_state or reason_txt != prev_reason:
            self._emit_log(
                f"[PANEL_FLOW] {prev_state} -> {state_txt}"
                + (f" reason={reason_txt}" if reason_txt else "")
            )

    def _update_panel_flow_state(
        self,
        *,
        ready_basic: bool,
        camera_ready: bool,
        test_pending: bool,
        pick_enabled: bool,
        system_error: bool,
    ) -> None:
        if system_error:
            self._set_panel_flow_state("ERROR", self._system_state_reason or "error_fatal")
            return
        if self._robot_test_active:
            self._set_panel_flow_state("TEST_RUNNING", "test_robot_active")
            return
        if self._script_motion_active and self._robot_test_done:
            self._set_panel_flow_state("PICK_RUNNING", "script_motion_active")
            return
        if not ready_basic:
            self._set_panel_flow_state("BOOT", self._system_state_reason or "waiting_basic")
            return
        tf_ok, tf_reason = self._tf_chain_ready_status()
        camera_ok = (not self._camera_required) or camera_ready
        if not tf_ok or not self._controllers_ok or not camera_ok:
            reason = tf_reason if not tf_ok else ("controllers_not_ready" if not self._controllers_ok else "camera_not_ready")
            self._set_panel_flow_state("READY_BASIC", reason)
            return
        if test_pending:
            self._set_panel_flow_state("READY_TEST", "awaiting_test_robot")
            return
        if self._robot_test_done and not pick_enabled:
            self._set_panel_flow_state("TEST_PASSED", "awaiting_pick_ready")
            return
        if self._robot_test_done and pick_enabled:
            self._set_panel_flow_state("PICK_READY", "pick_enabled")
            return
        self._set_panel_flow_state("READY_TEST", "default")

    def _compute_reach_overlay_points(self, w: int, h: int) -> List[Tuple[int, int]]:
        if w <= 0 or h <= 0:
            return []
        points: List[Tuple[int, int]] = []
        count = max(12, int(REACH_OVERLAY_POINTS))
        step = (2.0 * math.pi) / float(count)
        for idx in range(count):
            ang = step * idx
            x = UR5_BASE_X + UR5_REACH_RADIUS * math.cos(ang)
            y = UR5_BASE_Y + UR5_REACH_RADIUS * math.sin(ang)
            pix = world_xyz_to_pixel(x, y, REACH_OVERLAY_Z, w, h)
            if not pix:
                pix = table_xy_to_pixel(x, y, w, h)
            if pix:
                points.append(pix)
        return points

    def _publish_calib_grid_marker(self) -> None:
        if Marker is None or Point is None or Duration is None:
            self._log("[CALIB] Marker no disponible")
            return
        self._ensure_moveit_node()
        if self._moveit_node is None:
            self._log("[CALIB] ROS node no disponible para Marker")
            return
        if self._marker_pub is None:
            try:
                self._marker_pub = self._moveit_node.create_publisher(Marker, "/calibration_grid", 1)
            except Exception as exc:
                self._log(f"[CALIB] Error creando publisher Marker: {exc}")
                self._marker_pub = None
        if self._marker_pub is None:
            return

        table_top = self._resolve_table_top_z()
        x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
        x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
        y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
        y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)
        step = max(0.02, CALIB_GRID_STEP)
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME or "world"
        try:
            marker.header.stamp = Time().to_msg()
        except Exception as exc:
            _log_exception("calib marker stamp", exc)
        marker.ns = "calib_grid"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.004
        marker.color.r = 0.12
        marker.color.g = 0.40
        marker.color.b = 0.96
        marker.color.a = 0.8
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(sec=1, nanosec=0)

        points: List[Point] = []
        x = x_min
        while x <= (x_max + 1e-6):
            p0 = Point(x=x, y=y_min, z=table_top)
            p1 = Point(x=x, y=y_max, z=table_top)
            points.extend([p0, p1])
            x += step
        y = y_min
        while y <= (y_max + 1e-6):
            p0 = Point(x=x_min, y=y, z=table_top)
            p1 = Point(x=x_max, y=y, z=table_top)
            points.extend([p0, p1])
            y += step
        marker.points = points
        self._marker_pub.publish(marker)
        self._set_status("Malla de calibración publicada en /calibration_grid", error=False)
        self._log("[CALIB] Malla publicada en /calibration_grid")

    def _resolve_table_top_z(self) -> float:
        if self._table_top_z is not None:
            return self._table_top_z
        self._load_sdf_geometry_cache()
        if self._table_top_z is None:
            self._table_top_z = 0.850
        return self._table_top_z

    def _resolve_safety_surface_z(self, table_top: float) -> float:
        basket_top = float(BASKET_DROP[2]) if BASKET_DROP else table_top
        helper = get_tf_helper()
        if helper is None:
            return table_top
        world_frame = WORLD_FRAME or "world"
        ee_frame = self._ee_frame_effective or "tool0"
        if not _can_transform_between(helper, world_frame, ee_frame, timeout_sec=0.2):
            return table_top
        tf = helper.lookup_transform(world_frame, ee_frame, timeout_sec=0.4)
        if not tf:
            return table_top
        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        table_x, table_y = TABLE_CENTER_X, TABLE_CENTER_Y
        basket_x, basket_y = float(BASKET_DROP[0]), float(BASKET_DROP[1])
        vec_x = basket_x - table_x
        vec_y = basket_y - table_y
        denom = (vec_x * vec_x) + (vec_y * vec_y)
        if denom <= 1e-6:
            return table_top
        t = ((x - table_x) * vec_x + (y - table_y) * vec_y) / denom
        return basket_top if t >= 0.5 else table_top

    def _check_robot_above_table(
        self,
        table_top: float,
        label: str = "",
        *,
        xy_margin: float = 0.00,
        z_eps: float = 0.01,
        monitor_mode: str = "list",
        monitor_frames: Optional[List[str]] = None,
    ) -> Tuple[bool, str]:
        helper = get_tf_helper()
        if helper is None:
            return False, "tf_helper=no"
        base_frame = self._base_frame_effective or BASE_FRAME or "base_link"
        world_frame = WORLD_FRAME or "world"
        if not _can_transform_between(helper, world_frame, base_frame, timeout_sec=0.2):
            return False, f"tf_missing {world_frame}<->{base_frame}"
        surface_top = self._resolve_safety_surface_z(table_top)
        frames = helper.list_frames()
        if not frames:
            return False, "tf_frames=empty"
        ee_frame = self._ee_frame_effective or "tool0"
        default_monitor = [
            ee_frame,
            "rg2_tcp",
            "tool0",
            "ee_link",
            "flange",
            "gripper_link",
            "wrist_3_link",
        ]
        mode = str(monitor_mode or "list").strip().lower()
        candidates: List[str] = []
        if mode == "all":
            candidates = [
                name
                for name in sorted(frames)
                if any(keyword in name.lower() for keyword in ROBOT_FRAME_KEYWORDS)
            ]
        else:
            source = [str(f).strip() for f in (monitor_frames or []) if str(f).strip()]
            if not source:
                source = default_monitor
            for frame in source:
                if frame in frames and frame not in candidates:
                    candidates.append(frame)
            if not candidates:
                for frame in default_monitor:
                    if frame in frames and frame not in candidates:
                        candidates.append(frame)
            if not candidates:
                candidates = [
                    name
                    for name in sorted(frames)
                    if any(keyword in name.lower() for keyword in ROBOT_FRAME_KEYWORDS)
                ]
        if not candidates:
            return False, "robot_frames=empty"
        x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
        x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
        y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
        y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)
        margin = max(0.0, float(xy_margin))
        x_min_chk = x_min - margin
        x_max_chk = x_max + margin
        y_min_chk = y_min - margin
        y_max_chk = y_max + margin
        z_eps = max(0.0, float(z_eps))
        min_z = None
        min_frame = ""
        violations: List[Tuple[str, float, float, float]] = []
        checked_count = 0
        over_table_count = 0
        for frame in candidates:
            tf = helper.lookup_transform(world_frame, frame, timeout_sec=0.6)
            if not tf:
                continue
            checked_count += 1
            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
            z = float(tf.transform.translation.z)
            if min_z is None or z < min_z:
                min_z = z
                min_frame = frame
            over_table = (x_min_chk <= x <= x_max_chk) and (y_min_chk <= y <= y_max_chk)
            if not over_table:
                continue
            over_table_count += 1
            if z < (surface_top - z_eps):
                violations.append((frame, z, x, y))
        if min_z is None:
            return False, "tf_lookup=empty"
        ee_world = helper.lookup_transform(world_frame, ee_frame, timeout_sec=0.2)
        base_world = helper.lookup_transform(world_frame, base_frame, timeout_sec=0.2)
        basket_x, basket_y, basket_z = BASKET_DROP
        tcp_txt = "n/a"
        if ee_world is not None:
            t = ee_world.transform.translation
            tcp_txt = f"({float(t.x):.3f},{float(t.y):.3f},{float(t.z):.3f})"
        base_txt = "n/a"
        if base_world is not None:
            t = base_world.transform.translation
            base_txt = f"({float(t.x):.3f},{float(t.y):.3f},{float(t.z):.3f})"
        self._emit_log(
            "[SAFETY][TABLE] "
            f"label={label or 'n/a'} frame={world_frame} base_frame={base_frame} ee_frame={ee_frame} "
            f"tcp_world={tcp_txt} tf_world_to_base_t={base_txt} "
            f"mode={mode} checked={checked_count} over_table={over_table_count}/{len(candidates)} "
            f"xy_margin={margin:.3f} z_eps={z_eps:.3f} "
            f"table_center=({TABLE_CENTER_X:.3f},{TABLE_CENTER_Y:.3f}) "
            f"table_bounds=([{x_min:.3f},{x_max:.3f}],[{y_min:.3f},{y_max:.3f}]) "
            f"table_top={table_top:.3f} surface_top={surface_top:.3f} "
            f"basket=({basket_x:.3f},{basket_y:.3f},{basket_z:.3f})"
        )
        if violations:
            worst = min(violations, key=lambda item: item[1])
            return (
                False,
                (
                    f"frame={worst[0]} xyz=({worst[2]:.3f},{worst[3]:.3f},{worst[1]:.3f}) "
                    f"surface_z={surface_top:.3f} margin={margin:.3f} z_eps={z_eps:.3f}"
                ),
            )
        return (
            True,
            (
                f"checked={checked_count} over_table={over_table_count} "
                f"min_frame={min_frame} min_z={min_z:.3f} surface_z={surface_top:.3f}"
            ),
        )

    def _load_sdf_geometry_cache(self) -> None:
        if self._sdf_model_cache:
            return
        world_path = self.world_combo.currentText().strip()
        if not world_path or not os.path.isfile(world_path):
            world_path = os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf")
        if not os.path.isfile(world_path):
            return
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
        except Exception as exc:
            _log_exception("parse SDF geometry cache", exc)
            return

        def _parse_pose(pose_text: str) -> Tuple[float, float, float]:
            parts = [p for p in (pose_text or "").split() if p]
            if len(parts) >= 3:
                try:
                    return float(parts[0]), float(parts[1]), float(parts[2])
                except Exception as exc:
                    _log_exception("parse model pose", exc)
            return 0.0, 0.0, 0.0

        for model in root.findall(".//model"):
            name = model.attrib.get("name") or ""
            if not name:
                continue
            geom = None
            geom_type = None
            size = None
            length = None
            radius = None
            model_pose = _parse_pose(model.findtext("pose") or "")
            link = model.find("link")
            collision_pose = (0.0, 0.0, 0.0)
            if link is not None:
                collision = link.find("collision")
                if collision is None:
                    collision = link.find("visual")
                if collision is not None:
                    geom = collision.find("geometry")
                    collision_pose = _parse_pose(collision.findtext("pose") or "")
            if geom is None:
                continue
            box = geom.find("box")
            cyl = geom.find("cylinder")
            sph = geom.find("sphere")
            if box is not None:
                geom_type = "box"
                size_text = box.findtext("size") or ""
                parts = [p for p in size_text.split() if p]
                if len(parts) == 3:
                    try:
                        size = tuple(float(p) for p in parts)
                    except Exception as exc:
                        _log_exception("parse box size", exc)
                        size = None
            elif cyl is not None:
                geom_type = "cylinder"
                try:
                    radius = float(cyl.findtext("radius") or 0.0)
                    length = float(cyl.findtext("length") or 0.0)
                except Exception as exc:
                    _log_exception("parse cylinder size", exc)
                    radius = None
                    length = None
            elif sph is not None:
                geom_type = "sphere"
                try:
                    radius = float(sph.findtext("radius") or 0.0)
                except Exception as exc:
                    _log_exception("parse sphere radius", exc)
                    radius = None
            data = {
                "type": geom_type,
                "size": size,
                "radius": radius,
                "length": length,
                "pose": model_pose,
            }
            self._sdf_model_cache[name] = data
            if name == "mesa_pro" and size and len(size) == 3:
                link_pose = _parse_pose(link.findtext("pose") or "")
                top = (
                    model_pose[2]
                    + link_pose[2]
                    + collision_pose[2]
                    + (size[2] / 2.0)
                )
                self._table_top_z = top

    def _load_joint_limits(self) -> None:
        path = UR5_JOINT_LIMITS_YAML or ""
        if not path or not os.path.isfile(path):
            self._joint_limits_ok = False
            self._joint_limits_err = f"joint_limits.yaml no encontrado: {path or 'no especificado'}"
            self._emit_log(f"[TFM] WARN: límites articulares no disponibles ({self._joint_limits_err})")
            return
        if yaml is None:
            self._joint_limits_ok = False
            self._joint_limits_err = "PyYAML no disponible"
            self._emit_log("[TFM] WARN: límites articulares no disponibles (PyYAML no disponible)")
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self._joint_limits_ok = False
            self._joint_limits_err = f"error leyendo {path}: {exc}"
            self._emit_log(f"[TFM] WARN: límites articulares no disponibles ({exc})")
            return
        joint_limits = data.get("joint_limits") if isinstance(data, dict) else None
        if not isinstance(joint_limits, dict):
            self._joint_limits_ok = False
            self._joint_limits_err = f"formato inválido en {path}"
            self._emit_log(f"[TFM] WARN: límites articulares inválidos ({path})")
            return
        missing = [name for name in UR5_JOINT_NAMES if name not in joint_limits]
        if missing:
            self._joint_limits_ok = False
            self._joint_limits_err = f"faltan joints: {', '.join(missing)}"
            self._emit_log(f"[TFM] WARN: límites articulares incompletos ({', '.join(missing)})")
            return
        self._joint_limits_ok = True
        self._joint_limits_err = ""
        self._emit_log(f"[TFM] Limites articulares OK ({Path(path).name})")

    def _post_calibration_pipeline(self) -> None:
        recalc_object_states("calibration")
        objects = self._build_object_report()
        if objects:
            self._log_object_report(objects)
            self._pickable_map_cache = {obj["id"]: obj["pickable"] for obj in objects}
        else:
            self._pickable_map_cache = None
        self.signal_update_objects.emit()

    def _build_object_report(self) -> List[Dict[str, object]]:
        from .panel_utils import get_object_positions

        self._load_sdf_geometry_cache()
        table_top = self._resolve_table_top_z()
        objects = []
        positions = get_object_positions()
        if not positions:
            return objects
        for name, (x, y, z) in sorted(positions.items()):
            state = get_object_state(name)
            if not state or state.logical_state not in (
                ObjectLogicalState.ON_TABLE,
                ObjectLogicalState.SELECTED,
            ):
                continue
            sdf = self._sdf_model_cache.get(name, {})
            geom_type = sdf.get("type") or "desconocido"
            size = sdf.get("size")
            radius = sdf.get("radius")
            length = sdf.get("length")
            height = None
            if geom_type == "box" and size:
                height = float(size[2])
            elif geom_type == "cylinder" and length:
                height = float(length)
            elif geom_type == "sphere" and radius:
                height = float(radius) * 2.0
            if height:
                z = table_top + (height / 2.0)
            # NOTE: yaw not needed for pickable; avoid ROS spin in worker threads.
            yaw = 0.0

            pickable, reason = self._is_pickable(name, x, y, z, table_top, positions)
            tipo = "cubo" if geom_type == "box" else "cilindro" if geom_type == "cylinder" else "prisma"
            objects.append(
                {
                    "id": name,
                    "tipo": tipo,
                    "pose": [round(x, 3), round(y, 3), round(z, 3), round(yaw, 3)],
                    "pickable": pickable,
                    "motivo": reason,
                }
            )
        return objects

    def _is_pickable(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        table_top: float,
        positions: Dict[str, Tuple[float, float, float]],
    ) -> Tuple[bool, Optional[str]]:
        dx = x - UR5_BASE_X
        dy = y - UR5_BASE_Y
        dist = (dx * dx + dy * dy) ** 0.5
        if dist > (UR5_REACH_RADIUS - 0.02):
            return False, "fuera_de_alcance"
        pre_grasp_z = z + PICKABLE_PRE_GRASP_Z
        if pre_grasp_z <= (table_top + 0.05):
            return False, "pregrasp_bajo"
        for other, (ox, oy, _oz) in positions.items():
            if other == name:
                continue
            if (ox - x) ** 2 + (oy - y) ** 2 < (PICKABLE_MIN_CLEARANCE ** 2):
                return False, "colision_con_objetos"
        return True, None

    def _log_object_report(self, objects: List[Dict[str, object]]) -> None:
        self._emit_log("[PICK][REPORT] Objetos detectados:")
        for obj in objects:
            motivo = obj.get("motivo")
            motivo_txt = f" motivo={motivo}" if motivo else ""
            self._emit_log(
                f"- id={obj['id']} tipo={obj['tipo']} pose={obj['pose']} pickable={obj['pickable']}{motivo_txt}"
            )

    def _go_table(self):
        self._log_button("Go Mesa")
        if not self._require_manual_ready("Mesa"):
            return
        self._log("[ROBOT] Iniciando movimiento a Mesa")
        self._set_status("Moviendo a Mesa…")
        move_sec = float(self.joint_time.value()) if self.joint_time else 3.0
        self._set_motion_lock(True)

        def worker():
            try:
                ok, info = self._publish_joint_trajectory(JOINT_TABLE_POSE_RAD, move_sec)
                if ok:
                    time.sleep(move_sec + 0.15)
                    self._ui_set_status("Mesa ejecutado (JointTrajectory)")
                    self._log(f"[ROBOT] Mesa: JointTrajectory en {info}")
                else:
                    self._ui_set_status(f"Mesa falló: {info}", error=True)
                    self._log_warning(f"[ROBOT] Mesa falló: {info}")
            finally:
                self._set_motion_lock(False)
                recalc_object_states("manual_move")

        self._run_async(worker)

    def _go_basket(self):
        self._log_button("Go Cesta")
        if not self._require_manual_ready("Cesta"):
            return
        self._set_status("Moviendo a Cesta…")
        move_sec = float(self.joint_time.value()) if self.joint_time else 3.0
        self._set_motion_lock(True)

        def worker():
            try:
                ok, info = self._publish_joint_trajectory(JOINT_BASKET_POSE_RAD, move_sec)
                if ok:
                    time.sleep(move_sec + 0.15)
                    self._ui_set_status("Cesta ejecutado (JointTrajectory)")
                    self._log(f"[ROBOT] Cesta: JointTrajectory en {info}")
                else:
                    self._ui_set_status(f"Cesta falló: {info}", error=True)
                    self._log_warning(f"[ROBOT] Cesta falló: {info}")
            finally:
                self._set_motion_lock(False)

        self._run_async(worker)

    def _toggle_gripper_button(self, checked: bool):
        if not self._command_gripper(checked, log_action="Gripper"):
            return
        if checked:
            self._schedule_attach_attempt("gripper_close")
    
    def _on_camera_click(self, px: int, py: int):
        """Manejar click en la imagen de cámara."""
        if hasattr(self, "camera_view") and not self.camera_view.isEnabled():
            return
        self._emit_log(f"[PICK][CLICK] px=({px},{py})")
        ok, reason = pick_ui_status(self)
        if not ok and "pose/info" not in str(reason):
            self._emit_log(f"[PICK] Bloqueado: {reason}")
            return
        if not ok:
            # Permitir selección visual aunque pose/info no esté listo.
            self._emit_log(f"[PICK] Selección permitida sin pose/info: {reason}")
        # Prioridad 1: Si está calibrando, manejar calibración
        if self._calibrating:
            self._handle_calibration_click(px, py)
            return
        
        # Prioridad 2: Si hay calibración válida, seleccionar objeto
        self._handle_object_selection_click(px, py)

    def _load_table_calibration(self):
        """Cargar calibración de tabla desde archivo (IGUAL A PANEL ONLY)."""
        from .panel_utils import load_table_calib, TABLE_CALIB_PATH
        
        self._emit_log("[CALIB] Intentando cargar calibración...")
        try:
            calib = load_table_calib()
            if calib:
                # Determinar tipo de calibración
                if isinstance(calib, dict):
                    msg = f"[CALIB] Calibración RECT cargada desde {TABLE_CALIB_PATH}"
                elif isinstance(calib, list):
                    if len(calib) == 3 and all(len(row) == 3 for row in calib):
                        msg = f"[CALIB] Calibración HOMOGRAFÍA cargada desde {TABLE_CALIB_PATH}"
                    else:
                        msg = f"[CALIB] Calibración AFINE cargada desde {TABLE_CALIB_PATH}"
                self._emit_log(msg)
                self._log(msg)
                self._set_status("✅ Calibración cargada", error=False)
            else:
                if os.path.isfile(TABLE_CALIB_PATH):
                    msg = f"[CALIB] ⚠️ Archivo calibración inválido: {TABLE_CALIB_PATH}"
                else:
                    msg = "[CALIB] ℹ️ No hay calibración guardada. Se intentará calibración automática al iniciar cámara."
                self._emit_log(msg)
                self._log(msg)
                self._set_status("Sin calibración guardada", error=False)
        except Exception as e:
            msg = f"[CALIB] ERROR cargando calibración: {e}"
            self._emit_log(msg)
            self._log(msg)
        recalc_object_states("calibration_load")

    def _refresh_objects_from_gz_async(self):
        self._run_async(self._refresh_objects_from_gz)

    def _resolve_pose_object_name(
        self,
        raw_name: object,
        known: Dict[str, Tuple[float, float, float]],
    ) -> Optional[str]:
        """Map a pose/info entity name to a known object id."""
        if not isinstance(raw_name, str):
            return None
        name = raw_name.strip()
        if not name:
            return None
        if name in known:
            return name
        parts = [p for p in name.split("::") if p]
        for token in reversed(parts):
            token = token.strip()
            if token in known:
                return token
        tail = name.split("/")[-1].strip()
        if tail in known:
            return tail
        return None

    def _extract_pose_updates(
        self,
        poses: List[Dict[str, object]],
        known: Dict[str, Tuple[float, float, float]],
    ) -> Tuple[Dict[str, Tuple[float, float, float]], Dict[str, str]]:
        """Build object updates from pose/info with robust name mapping."""
        updates: Dict[str, Tuple[float, float, float]] = {}
        sources: Dict[str, str] = {}
        for pose in poses:
            if not isinstance(pose, dict):
                continue
            raw_name = pose.get("name")
            key_name = self._resolve_pose_object_name(raw_name, known)
            pos = pose.get("position") or {}
            if not key_name or not isinstance(pos, dict):
                continue
            try:
                x = float(pos.get("x"))
                y = float(pos.get("y"))
                z = float(pos.get("z"))
            except (TypeError, ValueError):
                continue
            prev = updates.get(key_name)
            # If pose/info has multiple entries for the same object (model/link),
            # prefer the lower Z candidate to avoid stale elevated proxy frames.
            if prev is None or z < prev[2]:
                updates[key_name] = (x, y, z)
                sources[key_name] = str(raw_name)
        return updates, sources

    def _refresh_objects_from_gz(self):
        """Sincronizar poses de objetos desde Gazebo (igual a Panel Only)."""
        if not gz_sim_status()[0]:
            return
        self._ensure_pose_subscription()
        # TEST ROBOT is read-only for objects: skip pose sync to prevent ghost moves.
        objects_read_only = bool(self._robot_test_active)

        world_path = self.world_combo.currentText().strip()
        sdf_path = ""
        if world_path and os.path.isfile(world_path):
            sdf_path = world_path
        else:
            cand = os.path.join(WORLDS_DIR, world_path) if world_path else ""
            if cand and os.path.isfile(cand):
                sdf_path = cand
            elif cand and not cand.endswith(".sdf") and os.path.isfile(cand + ".sdf"):
                sdf_path = cand + ".sdf"
            if not sdf_path:
                for c in DEFAULT_WORLD_CANDIDATES:
                    if os.path.isfile(c):
                        sdf_path = c
                        break

        world_name = read_world_name(sdf_path) if sdf_path else GZ_WORLD
        poses = self._read_world_pose_info(world_name)
        if not poses:
            if not POSE_CLI_ENABLED:
                now = time.monotonic()
                if (now - self._pose_cli_warn_ts) > 3.0:
                    self._pose_cli_warn_ts = now
                    self._emit_log("[PICK] Objetos: pose/info vacío; gz CLI deshabilitado")
                return
            now = time.monotonic()
            if (now - self._pose_cli_last_ts) < max(0.1, POSE_CLI_MIN_INTERVAL_SEC):
                return
            self._pose_cli_last_ts = now
            env_base = (
                "export GZ_SIM_RESOURCE_PATH='{}:{}:${{GZ_SIM_RESOURCE_PATH:-}}' ; "
                "export GZ_LOG_LEVEL=error; export IGN_LOGGER_LEVEL=error; "
                "export GZ_TRANSPORT_IP='{}' ; "
            ).format(
                MODELS_DIR,
                WORLDS_DIR,
                get_gz_transport_ip(),
            )
            partitions = []
            part = resolve_gz_partition(self.gz_partition)
            if part:
                partitions.append(part)
            partitions.append("")
            out = ""
            for _ in range(4):
                for p in partitions:
                    env = env_base + (f"export GZ_PARTITION='{p}' ; " if p else "")
                    cmd = (
                        bash_preamble(self.ws_dir)
                        + env
                        + f"gz topic -e -n 1 -t '/world/{world_name}/pose/info' --json-output"
                    )
                    res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
                    poses = _parse_pose_json(res.stdout or "")
                    if poses:
                        out = res.stdout
                        break
                if out:
                    break
                self._wait_for_state_change(0.6)
            if not out:
                self._log("[PICK] Objetos: no se pudo leer poses desde Gazebo")
                return
            poses = _parse_pose_json(out)
            if not poses:
                return
        targets = self._settle_targets()
        if self._objects_settled and targets:
            seen_targets = {pose.get("name") for pose in poses if pose.get("name") in targets}
            if seen_targets != targets:
                self._invalidate_settle("cambio de modelos en Gazebo", restart=True)
        known = get_object_positions()
        updates, pose_sources = self._extract_pose_updates(poses, known)
        if getattr(self, "_pick_target_lock_active", False):
            lock_name = str(getattr(self, "_pick_target_lock_name", "") or "")
            lock_id = str(getattr(self, "_pick_target_lock_id", "") or "n/a")
            if lock_name:
                lock_pose = updates.get(lock_name)
                if lock_pose is not None:
                    src = pose_sources.get(lock_name, lock_name)
                    lock_base = self._ensure_base_coords(
                        (float(lock_pose[0]), float(lock_pose[1]), float(lock_pose[2])),
                        self._world_frame_last_first(),
                        timeout_sec=0.35,
                    )
                    base_txt = "base=(n/a)"
                    if lock_base is not None:
                        base_txt = (
                            f"base=({float(lock_base[0]):.3f},{float(lock_base[1]):.3f},{float(lock_base[2]):.3f})"
                        )
                    self._emit_log(
                        "[PICK_OBJ][TARGET_LOCK] pose_sync_during_pick "
                        f"lock_id={lock_id} lock_name={lock_name} "
                        f"{base_txt} "
                        f"source={src}"
                    )
                else:
                    self._emit_log_throttled(
                        f"PICK:target_lock_pose_missing:{lock_name}",
                        "[PICK_OBJ][TARGET_LOCK] pose_sync_during_pick_missing "
                        f"lock_id={lock_id} lock_name={lock_name}",
                    )

        updated = bulk_update_object_positions(
            updates,
            source="pose/info",
            read_only=objects_read_only,
            objects_stable=self._objects_settled,
        )
        if updated:
            if SAVE_POSE_INFO_POSITIONS:
                save_object_positions()
            elif not self._pose_info_save_logged:
                self._emit_log(
                    "[OBJECTS][INFO] pose/info no actualiza object_positions.json "
                    "(set PANEL_SAVE_POSE_INFO_POSITIONS=1 para habilitar)"
                )
                self._pose_info_save_logged = True
            if self._debug_logs_enabled:
                sample = []
                for name in sorted(updates.keys()):
                    x, y, z = updates[name]
                    source_name = pose_sources.get(name, name)
                    base_pose = self._ensure_base_coords(
                        (float(x), float(y), float(z)),
                        self._world_frame_last_first(),
                        timeout_sec=0.25,
                    )
                    if base_pose is not None:
                        sample.append(
                            f"{name}=({float(base_pose[0]):.3f},{float(base_pose[1]):.3f},{float(base_pose[2]):.3f})@{source_name}"
                        )
                    else:
                        sample.append(f"{name}=base(n/a)@{source_name}")
                if sample:
                    self._emit_log(
                        "[OBJECTS][POSE_SRC] frame_id=base_link source=pose_info "
                        + " ".join(sample[:8])
                    )
            self._post_calibration_pipeline()
            self.signal_update_objects.emit()
            self._log(f"[PICK] Objetos sincronizados desde Gazebo ({updated}).")
        elif objects_read_only:
            self._log("[PICK] Objetos: TEST activo, sync bloqueado (read-only).")
        else:
            self._log("[PICK] Objetos: sin cambios desde Gazebo.")

    def _settle_targets(self) -> Set[str]:
        targets = set()
        for name in get_object_positions().keys():
            if name in SETTLE_MANUAL:
                targets.add(name)
        return targets

    def _read_world_pose_info(self, world_name: str) -> Optional[List[Dict[str, object]]]:
        if not self._ros_worker_started:
            return None
        poses, ts = self.ros_worker.pose_snapshot()
        if not poses:
            # Fallback: use cached object positions synced from Gazebo.
            fallback = get_object_positions()
            if not fallback:
                return None
            out = []
            for name, (x, y, z) in fallback.items():
                out.append({"name": name, "position": {"x": x, "y": y, "z": z}})
            return out
        age = time.time() - ts if ts else float("inf")
        if age > POSE_INFO_MAX_AGE_SEC and not POSE_INFO_ALLOW_STALE:
            return None
        out: List[Dict[str, object]] = []
        for name, (x, y, z) in poses.items():
            out.append(
                {
                    "name": name,
                    "position": {"x": x, "y": y, "z": z},
                }
            )
        return out

    def _start_calibration(self):
        """Iniciar/Desactivar calibración manual mostrando la grilla."""
        start_calibration(self)

    def _set_calibrate_button_text(self, text: str) -> None:
        if self.btn_calibrate is not None:
            self.btn_calibrate.setText(text)

    def _capture_calibration_frame(self) -> None:
        """Captura la última imagen disponible para trazabilidad."""
        if not self._last_camera_frame:
            self._log("[CALIB] Imagen no disponible para captura")
            return
        _qimg, w, h, ts = self._last_camera_frame
        self._log(f"[CALIB] Imagen capturada {w}x{h} age={time.time() - ts:.2f}s")

    def _auto_calibrate_from_camera(self) -> bool:
        """Auto-calibrar usando la cámara overhead y la geometría de la mesa."""
        from .panel_utils import (
            load_table_calib,
            TABLE_CALIB_PATH,
            TABLE_CAM_INFO,
            pixel_to_norm,
        )

        calib = load_table_calib()
        if not calib or not TABLE_CAM_INFO:
            return False
        w = getattr(self.camera_view, "_img_width", 0) if hasattr(self, "camera_view") else 0
        h = getattr(self.camera_view, "_img_height", 0) if hasattr(self, "camera_view") else 0
        if w <= 0 or h <= 0:
            try:
                w = int(TABLE_CAM_INFO.get("width") or 0)
                h = int(TABLE_CAM_INFO.get("height") or 0)
            except Exception as exc:
                _log_exception("auto_calib image size", exc)
                w = 0
                h = 0
        if w <= 0 or h <= 0:
            self._log("[CALIB] Auto: tamaño de imagen desconocido")
            return False

        table_z = 0.850
        corners = [
            (TABLE_CENTER_X - TABLE_SIZE_X / 2.0, TABLE_CENTER_Y + TABLE_SIZE_Y / 2.0, table_z),
            (TABLE_CENTER_X + TABLE_SIZE_X / 2.0, TABLE_CENTER_Y + TABLE_SIZE_Y / 2.0, table_z),
            (TABLE_CENTER_X + TABLE_SIZE_X / 2.0, TABLE_CENTER_Y - TABLE_SIZE_Y / 2.0, table_z),
            (TABLE_CENTER_X - TABLE_SIZE_X / 2.0, TABLE_CENTER_Y - TABLE_SIZE_Y / 2.0, table_z),
        ]
        pixel_points = []
        world_points = []
        for x, y, z in corners:
            pix = world_xyz_to_pixel(x, y, z, w, h)
            if not pix:
                self._log("[CALIB] Auto: no se pudo proyectar esquina de mesa")
                return False
            nx, ny = pixel_to_norm(pix[0], pix[1], w, h)
            pixel_points.append((nx, ny))
            world_points.append((x, y))

        try:
            hom = self._compute_homography(pixel_points, world_points)
        except Exception as exc:
            self._log(f"[CALIB] Auto: error calculando homografía: {exc}")
            return False
        if hom is None:
            self._log("[CALIB] Auto: homografía inválida")
            return False

        payload = {
            "mode": "homography",
            "h": hom,
            "camera": TABLE_CAM_INFO,
        }
        try:
            with open(TABLE_CALIB_PATH, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
        except Exception as exc:
            self._log(f"[CALIB] Auto: no se pudo guardar {TABLE_CALIB_PATH}: {exc}")
            return False

        self._load_table_calibration()
        self._calib_grid_until = time.time() + 1.0
        self._calibrating = False
        self._calib_points = []
        self._set_status("✅ Calibración auto aplicada", error=False)
        self._log("[CALIB] Auto: calibración guardada y cargada")
        self._publish_calib_grid_marker()
        self._post_calibration_pipeline()
        self._refresh_objects_from_gz_async()
        return True

    @staticmethod
    def _compute_homography(pixel_points: List[Tuple[float, float]], world_points: List[Tuple[float, float]]):
        if len(pixel_points) < 4 or len(world_points) < 4:
            return None
        a_rows = []
        for (u, v), (x, y) in zip(pixel_points, world_points):
            a_rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, u * x, v * x, x])
            a_rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, u * y, v * y, y])
        mat = np.array(a_rows, dtype=float)
        _, _, v_t = np.linalg.svd(mat)
        h = v_t[-1].reshape((3, 3))
        if abs(h[2, 2]) > 1e-8:
            h = h / h[2, 2]
        return h.tolist()
    
    def _draw_calib_overlay(self, qimg: QImage, w: int, h: int) -> QImage:
        """Dibujar malla y puntos de calibración sobre la imagen."""
        from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
        from PyQt5.QtCore import Qt, QPointF
        
        # Copiar imagen para no modificar original
        img_copy = qimg.copy()
        painter = QPainter(img_copy)
        if OVERLAY_ANTIALIAS:
            painter.setRenderHint(QPainter.Antialiasing)

        try:
            if time.time() <= self._calib_grid_until:
                # Dibujar malla de calibración (grid) - IGUAL A PANEL ONLY
                # Usar coordenadas de mundo (0.025m steps) + tabla_xy_to_pixel para convertir a píxeles
                pen = QPen(QColor(30, 64, 175, 180))
                pen.setWidth(2)
                painter.setPen(pen)

                step_x = 0.025  # metros
                step_y = 0.025  # metros
                x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
                x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
                y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
                y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)

                # Líneas verticales (X constante)
                x = x_min
                while x <= (x_max + 1e-6):
                    p0 = table_xy_to_pixel(x, y_min, w, h)
                    p1 = table_xy_to_pixel(x, y_max, w, h)
                    if p0 and p1:
                        painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                    x += step_x

                # Líneas horizontales (Y constante)
                y = y_min
                while y <= (y_max + 1e-6):
                    p0 = table_xy_to_pixel(x_min, y, w, h)
                    p1 = table_xy_to_pixel(x_max, y, w, h)
                    if p0 and p1:
                        painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                    y += step_y

                # Contorno de la mesa para mayor contraste
                p_tl = table_xy_to_pixel(x_min, y_max, w, h)
                p_tr = table_xy_to_pixel(x_max, y_max, w, h)
                p_br = table_xy_to_pixel(x_max, y_min, w, h)
                p_bl = table_xy_to_pixel(x_min, y_min, w, h)
                if p_tl and p_tr and p_br and p_bl:
                    edge_pen = QPen(QColor(16, 185, 129, 200))
                    edge_pen.setWidth(2)
                    painter.setPen(edge_pen)
                    painter.drawLine(QPointF(p_tl[0], p_tl[1]), QPointF(p_tr[0], p_tr[1]))
                    painter.drawLine(QPointF(p_tr[0], p_tr[1]), QPointF(p_br[0], p_br[1]))
                    painter.drawLine(QPointF(p_br[0], p_br[1]), QPointF(p_bl[0], p_bl[1]))
                    painter.drawLine(QPointF(p_bl[0], p_bl[1]), QPointF(p_tl[0], p_tl[1]))

                # Etiquetas básicas de ejes para visibilidad (mismo estilo Panel Only)
                painter.setPen(QPen(QColor(30, 64, 175, 160)))
                for label_x in (TABLE_CENTER_X - 0.4, TABLE_CENTER_X, TABLE_CENTER_X + 0.4):
                    p = table_xy_to_pixel(label_x, y_min, w, h)
                    if p:
                        painter.drawText(p[0] + 3, p[1] + 12, f"x={label_x:.1f}")
                for label_y in (TABLE_CENTER_Y - 0.3, TABLE_CENTER_Y, TABLE_CENTER_Y + 0.3):
                    p = table_xy_to_pixel(x_min, label_y, w, h)
                    if p:
                        painter.drawText(p[0] + 3, p[1] - 3, f"y={label_y:.1f}")

            # Dibujar cada punto de calibración
            for i, (px, py) in enumerate(self._calib_points):
                # Cruz roja
                painter.setPen(QPen(QColor(255, 0, 0), 2))
                size = 10
                painter.drawLine(px - size, py, px + size, py)
                painter.drawLine(px, py - size, px, py + size)

                # Círculo exterior
                painter.setPen(QPen(QColor(255, 255, 0), 2))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(QPointF(px, py), 8, 8)

                # Texto con número
                painter.setPen(QPen(QColor(0, 255, 0), 1))
                painter.drawText(QPointF(px + 12, py - 8), f"P{i+1}")

            # Ejes XYZ para orientar la calibración
            def _world_to_pixel(wx: float, wy: float):
                calib = self.calib_service.get_calibration() if hasattr(self, "calib_service") else None
                if not calib or calib.matrix is None:
                    return None
                try:
                    if calib.mode == CalibrationMode.LINEAR_2PT:
                        sx = float(calib.matrix[0, 0])
                        sy = float(calib.matrix[1, 1])
                        tx = float(calib.matrix[0, 2])
                        ty = float(calib.matrix[1, 2])
                        if abs(sx) < 1e-9 or abs(sy) < 1e-9:
                            return None
                        px = (wx - tx) / sx
                        py = (wy - ty) / sy
                        return (px, py)
                    if calib.mode == CalibrationMode.HOMOGRAPHY:
                        inv = np.linalg.inv(calib.matrix)
                        vec = inv @ np.array([wx, wy, 1.0])
                        if abs(vec[2]) < 1e-9:
                            return None
                        return (float(vec[0] / vec[2]), float(vec[1] / vec[2]))
                except Exception as exc:
                    _log_exception("calib world_to_pixel", exc)
                    return None
                return None

            def _draw_arrow(p0, p1, color: QColor, label: str):
                if not p0 or not p1:
                    return
                painter.setPen(QPen(color, 2))
                painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                vx = p1[0] - p0[0]
                vy = p1[1] - p0[1]
                norm = math.hypot(vx, vy)
                if norm > 1e-3:
                    ux = vx / norm
                    uy = vy / norm
                    size = 8.0
                    perp = (-uy, ux)
                    a1 = (p1[0] - ux * size + perp[0] * (size / 2.0), p1[1] - uy * size + perp[1] * (size / 2.0))
                    a2 = (p1[0] - ux * size - perp[0] * (size / 2.0), p1[1] - uy * size - perp[1] * (size / 2.0))
                    painter.drawLine(QPointF(p1[0], p1[1]), QPointF(a1[0], a1[1]))
                    painter.drawLine(QPointF(p1[0], p1[1]), QPointF(a2[0], a2[1]))
                painter.setPen(QPen(color, 1))
                painter.drawText(QPointF(p1[0] + 4.0, p1[1] - 4.0), label)

            # Representación compacta de ejes en la esquina superior izquierda
            axis_len_px = max(26.0, min(w, h) * 0.07)
            margin = 16.0
            base = (margin, margin + axis_len_px * 1.2)
            x_tip = (base[0] + axis_len_px, base[1])
            y_tip = (base[0], base[1] - axis_len_px)
            z_tip = (base[0] - axis_len_px * 0.5, base[1] - axis_len_px * 0.7)

            _draw_arrow(base, x_tip, QColor(239, 68, 68), "X+")
            _draw_arrow(base, y_tip, QColor(34, 197, 94), "Y+")
            _draw_arrow(base, z_tip, QColor(59, 130, 246), "Z+")
        finally:
            painter.end()
        return img_copy
    
    def _draw_selection_overlay(self, qimg: QImage, w: int, h: int) -> QImage:
        """Dibujar selección actual sobre la imagen."""
        from PyQt5.QtGui import QPainter, QPen, QColor
        from PyQt5.QtCore import Qt, QPointF
        
        if not self._selected_px:
            return qimg
        
        px, py = self._selected_px
        img_copy = qimg.copy()
        painter = QPainter(img_copy)
        if OVERLAY_ANTIALIAS:
            painter.setRenderHint(QPainter.Antialiasing)
        
        color = QColor(34, 197, 94)
        painter.setPen(QPen(color, 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(px, py), 9, 9)
        
        painter.drawLine(px - 10, py, px + 10, py)
        painter.drawLine(px, py - 10, px, py + 10)
        
        # Texto con coordenadas de negocio en base_link.
        if self._selected_base:
            bx, by, _bz = self._selected_base
            painter.setPen(QPen(QColor(255, 255, 255), 1))
            painter.drawText(px + 22, py, f"base({bx:.3f}, {by:.3f})")
        
        painter.end()
        self._emit_log_throttled(
            "PICK:overlay",
            f"[PICK][OVERLAY] draw px=({int(px)},{int(py)}) base={'yes' if bool(self._selected_base) else 'no'}",
        )
        return img_copy

    def _draw_grasp_overlay(self, qimg: QImage, w: int, h: int) -> QImage:
        """Dibujar el grasp inferido sobre la imagen (estilo Cornell).

        Convención:
        - Lados largos (w, apertura de pinza) → verde
        - Lados cortos (h, mandíbulas/jaws)  → rojo
        - Centro → punto blanco
        """
        from PyQt5.QtGui import QPainter, QPen, QColor
        from PyQt5.QtCore import Qt, QPointF

        if not self._last_grasp_px:
            return qimg
        img_copy = qimg.copy()
        painter = QPainter(img_copy)
        if OVERLAY_ANTIALIAS:
            painter.setRenderHint(QPainter.Antialiasing)

        def _draw_rect_grasp(
            grasp: Dict[str, float],
            *,
            long_color: QColor,
            short_color: QColor,
            center_color: QColor,
            tag: str,
            pen_style=Qt.SolidLine,
        ) -> None:
            cx = float(grasp.get("cx", 0.0))
            cy = float(grasp.get("cy", 0.0))
            gw = float(grasp.get("w", 0.0))
            gh = float(grasp.get("h", 0.0))
            angle_deg = float(grasp.get("angle_deg", 0.0))
            if gw <= 0.0:
                return
            if gh <= 0.0:
                gh = max(4.0, gw * 0.25)

            hw = gw / 2.0
            hh = gh / 2.0
            ang = math.radians(angle_deg)
            cs = math.cos(ang)
            sn = math.sin(ang)

            def _rot(dx: float, dy: float) -> QPointF:
                return QPointF(cx + dx * cs - dy * sn, cy + dx * sn + dy * cs)

            p0 = _rot(-hw, -hh)
            p1 = _rot(hw, -hh)
            p2 = _rot(hw, hh)
            p3 = _rot(-hw, hh)

            painter.setPen(QPen(long_color, 2, pen_style))
            painter.drawLine(p0, p1)
            painter.drawLine(p3, p2)

            painter.setPen(QPen(short_color, 3, pen_style))
            painter.drawLine(p1, p2)
            painter.drawLine(p0, p3)

            painter.setPen(QPen(center_color, 2, pen_style))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), 3, 3)

            painter.setPen(QPen(center_color, 1, pen_style))
            painter.drawText(QPointF(cx + 6, cy - 6), f"{tag} {angle_deg:.0f}°")

        _draw_rect_grasp(
            self._last_grasp_px,
            long_color=QColor(34, 197, 94),
            short_color=QColor(239, 68, 68),
            center_color=QColor(255, 255, 255),
            tag="P",
        )

        if self._tfm_visual_compare_enabled:
            ref = self._build_reference_grasp(w, h)
            if ref:
                _draw_rect_grasp(
                    ref,
                    long_color=QColor(14, 165, 233),
                    short_color=QColor(250, 204, 21),
                    center_color=QColor(14, 165, 233),
                    tag="R",
                    pen_style=Qt.DashLine,
                )

        painter.end()
        return img_copy

    def _base_to_world_coords(
        self,
        coords_base: Tuple[float, float, float],
        *,
        world_frame: Optional[str] = None,
        timeout_sec: float = 0.35,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Transform coordenadas de base_link a world.
        FASE 3: Timeout aumentado y mejor fallback logging.
        """
        base_frame = self._business_base_frame()
        target_world = world_frame or self._world_frame_last_first()
        # FASE 3: Timeout aumentado de 0.35 a 1.0s para reducir fallos TF
        coords, _ = transform_point_to_frame(
            (float(coords_base[0]), float(coords_base[1]), float(coords_base[2])),
            target_world,
            source_frame=base_frame,
            timeout_sec=max(timeout_sec, 1.0),
        )
        if coords:
            return (float(coords[0]), float(coords[1]), float(coords[2]))
        # Render fallback only con logging
        self._emit_log(
            f"[TEST][FASE3] TF fallback usado para transform base->world (puede causar imprecisión)"
        )
        wx, wy, wz = base_to_world(
            float(coords_base[0]), float(coords_base[1]), float(coords_base[2])
        )
        return (float(wx), float(wy), float(wz))

    def _compute_test_corner_base_points(
        self,
        *,
        inset_m: Optional[float] = None,
        z_override: Optional[float] = None,
    ) -> List[Tuple[str, Tuple[float, float, float]]]:
        """Return two front corner target points on the table in base_link."""
        x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
        x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
        y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
        y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)
        table_z = float(z_override) if z_override is not None else float(self._resolve_table_top_z())
        if inset_m is None:
            try:
                inset_m = float(os.environ.get("PANEL_TEST_CORNER_INSET_M", "0.06"))
            except Exception:
                inset_m = 0.06
        inset_m = max(0.0, float(inset_m))
        inset_x = min(max(0.02, inset_m), max(0.02, (x_max - x_min) * 0.24))
        inset_y = min(max(0.02, inset_m), max(0.02, (y_max - y_min) * 0.24))

        # Front edge = nearest edge to robot origin in base_link (no dependencia de get_pose(world,base)).
        edge_name = "x_min"
        world_frame = self._world_frame_last_first()
        edge_centers_world = {
            "x_min": (x_min, TABLE_CENTER_Y),
            "x_max": (x_max, TABLE_CENTER_Y),
            "y_min": (TABLE_CENTER_X, y_min),
            "y_max": (TABLE_CENTER_X, y_max),
        }
        edge_centers_base: Dict[str, Tuple[float, float, float]] = {}
        for edge_key, (wx, wy) in edge_centers_world.items():
            # FASE 3: Timeout aumentado de 0.35 a 1.0s
            center_base = self._ensure_base_coords((wx, wy, table_z), world_frame, timeout_sec=1.0)
            if center_base is None:
                try:
                    center_base = world_to_base(float(wx), float(wy), float(table_z))
                except Exception:
                    center_base = None
            if center_base is not None:
                edge_centers_base[edge_key] = (
                    float(center_base[0]),
                    float(center_base[1]),
                    float(center_base[2]),
                )
        if edge_centers_base:
            edge_name = min(
                edge_centers_base.keys(),
                key=lambda name: math.hypot(
                    float(edge_centers_base[name][0]),
                    float(edge_centers_base[name][1]),
                ),
            )

        if edge_name == "x_min":
            raw_points = [
                (x_min + inset_x, y_min + inset_y, table_z),
                (x_min + inset_x, y_max - inset_y, table_z),
            ]
        elif edge_name == "x_max":
            raw_points = [
                (x_max - inset_x, y_min + inset_y, table_z),
                (x_max - inset_x, y_max - inset_y, table_z),
            ]
        elif edge_name == "y_min":
            raw_points = [
                (x_min + inset_x, y_min + inset_y, table_z),
                (x_max - inset_x, y_min + inset_y, table_z),
            ]
        else:
            raw_points = [
                (x_min + inset_x, y_max - inset_y, table_z),
                (x_max - inset_x, y_max - inset_y, table_z),
            ]

        base_points: List[Tuple[float, float, float]] = []
        for point in raw_points:
            # FASE 3: Timeout aumentado de 0.35 a 1.0s
            converted = self._ensure_base_coords(point, world_frame, timeout_sec=1.0)
            if converted is None:
                try:
                    converted = world_to_base(
                        float(point[0]), float(point[1]), float(point[2])
                    )
                except Exception:
                    converted = None
            if converted is not None:
                base_points.append(converted)
        if len(base_points) < 2:
            return []
        # Stable labels in base: larger base-Y is left.
        ordered = sorted(base_points, key=lambda p: p[1], reverse=True)
        return [
            ("FRONT_LEFT", (float(ordered[0][0]), float(ordered[0][1]), float(ordered[0][2]))),
            ("FRONT_RIGHT", (float(ordered[1][0]), float(ordered[1][1]), float(ordered[1][2]))),
        ]

    def _compute_test_corner_world_points(
        self,
        *,
        inset_m: Optional[float] = None,
        z_override: Optional[float] = None,
    ) -> List[Tuple[str, Tuple[float, float, float]]]:
        """Render helper: convert base_link test points to world for image overlay."""
        base_points = self._compute_test_corner_base_points(
            inset_m=inset_m,
            z_override=z_override,
        )
        world_frame = self._world_frame_last_first()
        out: List[Tuple[str, Tuple[float, float, float]]] = []
        for label, base_pt in base_points:
            world_pt = self._base_to_world_coords(base_pt, world_frame=world_frame)
            if world_pt is not None:
                out.append((label, world_pt))
        return out

    def _draw_test_corner_overlay(self, qimg: QImage, w: int, h: int) -> QImage:
        """Draw two corner marks on the table for TEST ROBOT positioning."""
        from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
        from PyQt5.QtCore import Qt, QPointF

        points = self._compute_test_corner_world_points()
        if not points:
            return qimg
        img_copy = qimg.copy()
        painter = QPainter(img_copy)
        if OVERLAY_ANTIALIAS:
            painter.setRenderHint(QPainter.Antialiasing)
        try:
            pen = QPen(QColor(245, 158, 11, 230), 2)
            brush = QBrush(QColor(245, 158, 11, 90))
            painter.setPen(pen)
            painter.setBrush(brush)
            line_pts: List[Tuple[int, int]] = []
            drawn = 0
            for idx, (label, (wx, wy, wz)) in enumerate(points, start=1):
                pix = world_xyz_to_pixel(wx, wy, wz, w, h)
                if not pix:
                    pix = table_xy_to_pixel(wx, wy, w, h)
                if not pix:
                    continue
                px, py = int(pix[0]), int(pix[1])
                line_pts.append((px, py))
                painter.drawEllipse(QPointF(px, py), 10.0, 10.0)
                painter.drawLine(px - 13, py, px + 13, py)
                painter.drawLine(px, py - 13, px, py + 13)
                painter.setPen(QPen(QColor(255, 255, 255, 240), 1))
                painter.drawText(QPointF(px + 12.0, py - 8.0), f"T{idx}:{label}")
                painter.setPen(pen)
                drawn += 1
            if len(line_pts) == 2:
                painter.setPen(QPen(QColor(245, 158, 11, 180), 2, Qt.DashLine))
                painter.drawLine(
                    QPointF(float(line_pts[0][0]), float(line_pts[0][1])),
                    QPointF(float(line_pts[1][0]), float(line_pts[1][1])),
                )
            if drawn <= 0:
                return qimg
        finally:
            painter.end()
        return img_copy

    def _draw_tcp_pose_overlay(self, qimg: QImage, w: int, h: int) -> QImage:
        """Draw a live TCP pose HUD over the camera view."""
        from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
        from PyQt5.QtCore import Qt, QRectF, QPointF

        img_copy = qimg.copy()
        painter = QPainter(img_copy)
        if OVERLAY_ANTIALIAS:
            painter.setRenderHint(QPainter.Antialiasing)
        try:
            panel_x = 10.0
            panel_y = 10.0
            panel_w = min(float(w - 20), 420.0)
            panel_h = 68.0
            bg = QColor(15, 23, 42, 165)
            border = QColor(148, 163, 184, 210)
            painter.setPen(QPen(border, 1))
            painter.setBrush(QBrush(bg))
            painter.drawRoundedRect(QRectF(panel_x, panel_y, panel_w, panel_h), 8.0, 8.0)

            tcp_world = self._last_tcp_world
            rpy = self._last_tcp_rpy_deg
            line1 = "TCP(base): --"
            line2 = "EE frame: --"
            line3 = "RPY[deg]: --"
            tcp_base = self._last_tcp_base
            if tcp_base is None and tcp_world is not None and isinstance(tcp_world, (list, tuple)) and len(tcp_world) >= 3:
                bx, by, bz = world_to_base(float(tcp_world[0]), float(tcp_world[1]), float(tcp_world[2]))
                tcp_base = (bx, by, bz)
            if tcp_base is not None and isinstance(tcp_base, (list, tuple)) and len(tcp_base) >= 3:
                bx = float(tcp_base[0])
                by = float(tcp_base[1])
                bz = float(tcp_base[2])
                line1 = f"TCP(base): x={bx:+.3f} y={by:+.3f} z={bz:+.3f}"
                line2 = f"EE frame: {self._ee_frame_effective or 'rg2_tcp'}"
            if rpy is not None and isinstance(rpy, (list, tuple)) and len(rpy) >= 3:
                rr = float(rpy[0])
                pp = float(rpy[1])
                yy = float(rpy[2])
                line3 = f"RPY[deg]:  r={rr:+.1f} p={pp:+.1f} y={yy:+.1f}"

            painter.setPen(QPen(QColor(226, 232, 240), 1))
            tx = panel_x + 10.0
            ty = panel_y + 20.0
            painter.drawText(QPointF(tx, ty), line1)
            painter.drawText(QPointF(tx, ty + 18.0), line2)
            painter.drawText(QPointF(tx, ty + 36.0), line3)
        finally:
            painter.end()
        return img_copy

    def _save_grasp_overlay(self) -> str:
        if not self._last_camera_frame or not self._last_grasp_px:
            return ""
        qimg, w, h, _ts = self._last_camera_frame
        overlay = self._draw_grasp_overlay(qimg, w, h)
        out_path = self._audit_root() / "figures" / "overlay_last.png"
        ensure_dir(str(out_path.parent))
        if overlay.save(str(out_path)):
            return str(out_path)
        return ""

    def _refresh_grasp_overlay_now(self) -> bool:
        if not self._last_camera_frame or not self._last_grasp_px:
            return False
        qimg, w, h, ts = self._last_camera_frame
        with self._camera_frame_lock:
            self._camera_pending_frame = (
                self.camera_topic or self._last_grasp_frame or "image",
                qimg,
                w,
                h,
                self._camera_last_fps,
                ts,
            )
        self._refresh_camera_display()
        return True

    def _draw_reach_overlay(self, qimg: QImage, w: int, h: int) -> QImage:
        """Dibujar alcance del robot como puntos sobre la imagen."""
        from PyQt5.QtGui import QPainter, QPen, QColor
        from PyQt5.QtCore import Qt, QPointF

        if not self._reach_overlay_points or self._reach_overlay_size != (w, h):
            self._reach_overlay_points = self._compute_reach_overlay_points(w, h)
            self._reach_overlay_size = (w, h)
        if not self._reach_overlay_points:
            return qimg
        img_copy = qimg.copy()
        painter = QPainter(img_copy)
        if OVERLAY_ANTIALIAS:
            painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(QColor(34, 197, 94, 200), 2, Qt.SolidLine))
        for px, py in self._reach_overlay_points:
            painter.drawPoint(QPointF(px, py))
        painter.end()
        return img_copy

    def _run_pick_demo(self):
        """Publica una secuencia MoveIt-only para el DEMO de mesa → cesta."""
        run_pick_demo(self)

    def _run_pick_object(self):
        """Ejecuta pick & place del objeto seleccionado hacia la cesta."""
        run_pick_object(self)
    
    def _get_object_world_position(self, obj_name: str) -> Optional[tuple]:
        """Obtener posición mundial del objeto desde poses actuales."""
        try:
            positions = get_object_positions()
            if positions and obj_name in positions:
                pos = positions[obj_name]
                # pos es una tupla (x, y, z)
                x, y, z = pos
                return (float(x), float(y), float(z))
        except Exception as e:
            self._log(f"[PICK] Error obteniendo posición: {e}")
        return None

    def _on_slider_change(self, idx: int, value: int):
        # Ignorar cambios si vienen de _on_joint_state (flag)
        if self._updating_sliders_from_joint_state:
            return
        
        deg = float(value) / JOINT_SLIDER_SCALE
        rad = math.radians(deg)
        self.joint_value_labels[idx].setText(f"{deg:.1f} deg / {rad:.3f} rad")
        
        # Bloquear actualizaciones de sliders por 2 segundos después de movimiento manual
        self._slider_update_blocked_until = time.time() + 2.0
        
        # Debug: mostrar cambio incremental cuando se mueve un slider
        if self._debug_logs_enabled:
            joint_name = UR5_JOINT_NAMES[idx]
            old_value = self._last_slider_values.get(idx, value)
            delta_deg = deg - (float(old_value) / JOINT_SLIDER_SCALE)
            direction = "↑" if delta_deg > 0 else "↓" if delta_deg < 0 else "→"
            if abs(delta_deg) > 0.5:  # Solo mostrar cambios significativos
                self._log(f"[SLIDER] {joint_name:25s} {direction} {deg:+7.1f}° (delta: {delta_deg:+.1f}°)")
            self._last_slider_values[idx] = value

    def _slider_to_deg(self, value: int) -> float:
        return float(value) / JOINT_SLIDER_SCALE

    def _current_joint_positions_rad(self):
        positions = [math.radians(self._slider_to_deg(s.value())) for s in self.joint_sliders]
        
        # Debug: mostrar valores de sliders
        if self._debug_logs_enabled:
            slider_values = [s.value() for s in self.joint_sliders]
            deg_values = [self._slider_to_deg(v) for v in slider_values]
            self._log(f"[SLIDER_READ] slider_raw={slider_values}")
            self._log(f"[SLIDER_READ] slider_deg={[f'{d:.1f}' for d in deg_values]}")
        
        return positions
    
    def _handle_calibration_click(self, px: int, py: int):
        """Manejar click durante calibración (IGUAL A PANEL ONLY - SIN DIÁLOGOS)."""
        if not self._calibrating:
            return
        
        # ✅ Simplemente guardar el píxel clickeado
        self._calib_points.append((px, py))
        self._log(f"[CALIB] Punto {len(self._calib_points)}: píxel ({px}, {py})")
        
        # Si tenemos 4 puntos, calcular calibración automáticamente
        if len(self._calib_points) >= 4:
            self._finish_calibration()
        else:
            # Mostrar progreso
            needed = 4 - len(self._calib_points)
            self._set_status(f"CALIBRACIÓN: {len(self._calib_points)}/4 puntos - Click {needed} más", error=False)
    
    def _finish_calibration(self):
        """Finalizar calibración calculando homografía automáticamente (IGUAL A PANEL ONLY)."""
        if len(self._calib_points) < 4:
            self._log("[CALIB] Calibración incompleta")
            return
        
        # ✅ Extraer píxeles clickeados
        p1_px, p1_py = self._calib_points[0]
        p2_px, p2_py = self._calib_points[1]
        p3_px, p3_py = self._calib_points[2]
        p4_px, p4_py = self._calib_points[3]
        
        # ✅ Calcular coordenadas del mundo basadas en la tabla
        # Asumimos que los 4 puntos son las esquinas de la mesa
        cx = TABLE_CENTER_X
        cy = TABLE_CENTER_Y
        sx = TABLE_SIZE_X / 2.0
        sy = TABLE_SIZE_Y / 2.0
        
        # Esquinas en coordenadas del mundo (arriba-izq, arriba-der, abajo-der, abajo-izq)
        w1 = (cx - sx, cy + sy)   # Arriba-izq
        w2 = (cx + sx, cy + sy)   # Arriba-der
        w3 = (cx + sx, cy - sy)   # Abajo-der
        w4 = (cx - sx, cy - sy)   # Abajo-izq
        
        # ✅ Validación mejorada: que los 4 puntos formen un cuadrilátero razonable
        # Chequea distancias entre puntos consecutivos y diagonales
        def dist(p1, p2):
            return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
        
        points = [(p1_px, p1_py), (p2_px, p2_py), (p3_px, p3_py), (p4_px, p4_py)]
        
        # Verificar que cada par de puntos adyacentes estén separados >= 20px
        min_distance = 20
        dist_12 = dist(points[0], points[1])
        dist_23 = dist(points[1], points[2])
        dist_34 = dist(points[2], points[3])
        dist_41 = dist(points[3], points[0])
        
        self._log(f"[CALIB] Distancias: P1-P2={dist_12:.1f}px, P2-P3={dist_23:.1f}px, P3-P4={dist_34:.1f}px, P4-P1={dist_41:.1f}px")
        
        if min(dist_12, dist_23, dist_34, dist_41) < min_distance:
            self._log(f"[CALIB] ERROR: puntos muy cercanos (mínimo {min_distance}px). Repite calibración.")
            self._set_status(f"ERROR: puntos muy cercanos ({min_distance}px mín)", error=True)
            self._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR
            self._set_status("Calibración reiniciada - click en 4 esquinas", error=False)
            return
        
        # ✅ Calcular homografía
        try:
            pixel_points = [(float(p1_px), float(p1_py)), (float(p2_px), float(p2_py)),
                           (float(p3_px), float(p3_py)), (float(p4_px), float(p4_py))]
            world_points = [(float(w1[0]), float(w1[1])), (float(w2[0]), float(w2[1])),
                           (float(w3[0]), float(w3[1])), (float(w4[0]), float(w4[1]))]

            hom = None
            # Intentar usar graspnet si está instalado
            try:
                from graspnet.utils.homography import compute_homography as g_compute
                hom = g_compute(pixel_points, world_points)
            except Exception as e:
                self._log(f"[CALIB] Aviso: graspnet no disponible ({e}), usando OpenCV")
                try:
                    import numpy as np
                    import cv2
                    src = np.array(pixel_points, dtype=np.float32)
                    dst = np.array(world_points, dtype=np.float32)
                    hom, _ = cv2.findHomography(src, dst, method=0)
                    if hom is not None:
                        hom = hom.tolist()
                except Exception as e_cv:
                    raise RuntimeError(f"No se pudo calcular homografía: {e_cv}")

            if not hom:
                self._log("[CALIB] ERROR: homografía inválida. Repite calibración.")
                self._set_status("ERROR: homografía inválida", error=True)
                self._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR
                return
            
            # ✅ Enviar calibración al servicio
            self.calib_service.set_homography(hom)
            self._calibrating = False
            self._calib_points = []  # ✅ LIMPIAR PUNTOS DESPUÉS DE ÉXITO
            self._set_calibrate_button_text("Calibrar")
            self._set_status(f"✅ Calibración completada ({dist_12:.0f}-{dist_34:.0f}px)", error=False)
            self._log(f"[CALIB] ✅ Homografía completada y almacenada en servicio")
            self._publish_calib_grid_marker()
            self._post_calibration_pipeline()
            self._refresh_objects_from_gz()
            save_object_positions()
            self._emit_log("[CALIB] Objetos sincronizados y guardados tras calibración")
            
        except Exception as e:
            self._log(f"[CALIB] ERROR calculando homografía: {e}")
            self._set_status(f"ERROR: {e}", error=True)
            self._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR

    
    def _handle_object_selection_click(self, px: int, py: int):
        """Manejar click en cámara para seleccionar objeto (igual a Panel Only)."""
        ok, reason = pick_ui_status(self)
        if not ok and "pose/info" not in str(reason):
            self._emit_log(f"[PICK] Bloqueado: {reason}")
            return
        if not ok:
            # La selección queda marcada; solo bloqueamos la ejecución posterior de PICK.
            self._emit_log(f"[PICK] Selección permitida sin pose/info: {reason}")
        self._emit_log(f"[PICK][SELECT_ATTEMPT] px=({px},{py})")
        if getattr(self, "_pick_target_lock_active", False):
            lock_name = str(getattr(self, "_pick_target_lock_name", "") or "none")
            lock_id = str(getattr(self, "_pick_target_lock_id", "") or "n/a")
            lock_source = str(getattr(self, "_pick_target_lock_source", "") or "n/a")
            lock_reason = str(getattr(self, "_pick_target_lock_reason", "") or "n/a")
            lock_ts = float(getattr(self, "_pick_target_lock_ts", 0.0) or 0.0)
            lock_age = max(0.0, time.time() - lock_ts) if lock_ts > 0.0 else 0.0
            self._emit_log(
                "[PICK][SELECT] ignored_selection_update_due_to_pick_active=true "
                f"source=camera_click lock_name={lock_name} lock_id={lock_id} "
                f"lock_source={lock_source} lock_reason={lock_reason} lock_age={lock_age:.2f}s"
            )
            self._set_status("Selección bloqueada: PICK Objeto en curso", error=False)
            return
        # Usar homografía global/table map para convertir a mundo
        w = getattr(self.camera_view, "_img_width", 0)
        h = getattr(self.camera_view, "_img_height", 0)
        world_x, world_y = pixel_to_table_xy(px, py, w, h)
        if world_x is None or world_y is None:
            self._log("[PICK] No hay calibración válida (pixel_to_table_xy) - carga table_pixel_map.json o calibra")
            self._set_status("⚠️ Calibra la cámara o carga table_pixel_map.json", error=True)
            return

        obj_name = nearest_table_object(world_x, world_y)
        obj_pos = get_object_position(obj_name) if obj_name else None
        world_z = obj_pos[2] if obj_pos else 0.0
        base_event = self._ensure_base_coords(
            (float(world_x), float(world_y), float(world_z)),
            self._world_frame_last_first(),
            timeout_sec=0.35,
        )
        base_txt = "base=(n/a)"
        if base_event is not None:
            base_txt = (
                f"base=({float(base_event[0]):.3f},{float(base_event[1]):.3f},{float(base_event[2]):.3f})"
            )
        self._emit_log(
            "[PICK][SELECT_EVT] "
            f"source=camera_click px=({px},{py}) candidate={obj_name or 'none'} "
            f"{base_txt}"
        )

        if obj_pos:
            # Recalcular con la altura del objeto para ajustar la proyección
            wx_adj, wy_adj = pixel_to_table_xy(px, py, w, h, z_target=obj_pos[2]) or (world_x, world_y)
            world_x, world_y = wx_adj, wy_adj
            dx = world_x - obj_pos[0]
            dy = world_y - obj_pos[1]
            snapped = math.hypot(dx, dy) <= SELECTION_SNAP_DIST
            if snapped:
                world_x, world_y = obj_pos[0], obj_pos[1]
                pix = world_xyz_to_pixel(world_x, world_y, obj_pos[2], w, h)
                if not pix:
                    pix = table_xy_to_pixel(world_x, world_y, w, h)
                if pix:
                    px, py = pix
            state = get_object_state(obj_name) if obj_name else None
            geom_on_table = is_on_table(obj_pos) if obj_pos else False
            state_ok = bool(
                state
                and (
                    state.logical_state == ObjectLogicalState.ON_TABLE
                    or (state.logical_state == ObjectLogicalState.SELECTED and geom_on_table)
                )
            )
            if not state_ok:
                details = f"state={state.logical_state.value}" if state else "state=unknown"
                pos_txt = f"pos=({obj_pos[0]:.3f},{obj_pos[1]:.3f},{obj_pos[2]:.3f})" if obj_pos else "pos=n/a"
                from .panel_objects import table_geom_debug
                geom = table_geom_debug(obj_pos) if obj_pos else ""
                self._emit_log(f"[PICK] Bloqueado: {obj_name} no está ON_TABLE ({details} {pos_txt} {geom})")
                self._set_status("Objeto no está sobre la mesa", error=True)
                return
            self._select_object(
                obj_name,
                px,
                py,
                world_x,
                world_y,
                world_z,
                source="camera_click",
            )
            return

        # Click en vacío (no objetos cerca)
        base_empty = self._ensure_base_coords(
            (float(world_x), float(world_y), 0.0),
            self._world_frame_last_first(),
            timeout_sec=0.35,
        )
        if base_empty is not None:
            self._log(
                "[PICK] Click en vacío: "
                f"px=({px},{py}) → base=({float(base_empty[0]):.2f},{float(base_empty[1]):.2f})"
            )
        else:
            self._log(f"[PICK] Click en vacío: px=({px},{py}) → base=(n/a)")
        if self._selected_object:
            prev_state = get_object_state(self._selected_object)
            if prev_state and prev_state.logical_state == ObjectLogicalState.SELECTED:
                update_object_state(self._selected_object, logical_state=ObjectLogicalState.ON_TABLE, reason="deselect_empty")
        self._selected_object = None
        self._selected_px = (px, py)
        self._selected_world = (world_x, world_y, 0.0)
        self._selected_base = base_empty
        self._selection_last_user_name = ""
        self._selection_last_user_ts = 0.0
        self._last_cornell = None
        self._refresh_science_ui()
        self._update_objects()
        self._log_selection_tf(self._selected_world)
    
    def _select_object(
        self,
        name: str,
        px: int,
        py: int,
        wx: float,
        wy: float,
        wz: float,
        *,
        source: str = "unknown",
    ):
        """Seleccionar un objeto (desde click o desde lista)."""
        if bool(getattr(self, "_calibrating", False)):
            self._emit_log(
                "[PICK][SELECT] blocked_calibration_active=true "
                f"source={source} name={name}"
            )
            self._set_status("Selección bloqueada: calibración activa", error=False)
            return
        if getattr(self, "_pick_target_lock_active", False):
            lock_name = str(getattr(self, "_pick_target_lock_name", "") or "none")
            lock_id = str(getattr(self, "_pick_target_lock_id", "") or "n/a")
            lock_source = str(getattr(self, "_pick_target_lock_source", "") or "n/a")
            lock_reason = str(getattr(self, "_pick_target_lock_reason", "") or "n/a")
            lock_ts = float(getattr(self, "_pick_target_lock_ts", 0.0) or 0.0)
            lock_age = max(0.0, time.time() - lock_ts) if lock_ts > 0.0 else 0.0
            self._emit_log(
                "[PICK][SELECT] ignored_selection_update_due_to_pick_active=true "
                f"source={source} name={name} lock_name={lock_name} lock_id={lock_id} "
                f"lock_source={lock_source} lock_reason={lock_reason} lock_age={lock_age:.2f}s"
            )
            return
        prev_name = self._selected_object
        selected_ts = time.time()
        state_updated = update_object_state(name, logical_state=ObjectLogicalState.SELECTED, reason="select")
        self._emit_log(
            "[PICK][SELECT_STORE] "
            f"source={source} name={name} ts={selected_ts:.3f} update_ok={str(state_updated).lower()}"
        )
        if not state_updated:
            self._set_status(f"Selección rechazada: {name}", error=True)
            self._emit_log(f"[PICK][SELECT] selección rechazada por store para {name}")
            return
        if prev_name and prev_name != name:
            prev_state = get_object_state(prev_name)
            if prev_state and prev_state.logical_state == ObjectLogicalState.SELECTED:
                update_object_state(prev_name, logical_state=ObjectLogicalState.ON_TABLE, reason="deselect")
        self._selected_object = name
        self._selected_px = (px, py)
        self._selected_world = (wx, wy, wz)
        selected_base = self._ensure_base_coords(
            (float(wx), float(wy), float(wz)),
            self._world_frame_last_first(),
            timeout_sec=0.35,
        )
        self._selected_base = selected_base
        self._selected_base_frame = self._business_base_frame()
        self._selection_timestamp = selected_ts
        self._selection_last_user_name = name
        self._selection_last_user_ts = selected_ts
        px_label = f"px=({px},{py})" if px >= 0 and py >= 0 else "px=n/a"
        base_label = "base=(n/a)"
        if selected_base is not None:
            base_label = (
                f"base=({float(selected_base[0]):.2f},{float(selected_base[1]):.2f},{float(selected_base[2]):.2f})"
            )
        self._log(
            f"[PICK][SELECT] source={source} name={name} ts={selected_ts:.3f} "
            f"{px_label} {base_label}"
        )
        state_after = get_object_state(name)
        self._emit_log(
            "[PICK][SELECT_STATE] "
            f"name={name} state={state_after.logical_state.value if state_after else 'none'} "
            f"state_ts={float(getattr(state_after, 'last_update_ts', 0.0) or 0.0):.3f} "
            f"user_ts={selected_ts:.3f}"
        )
        if selected_base is not None:
            self._set_status(
                f"✓ Seleccionado: {name} ({float(selected_base[0]):.2f},{float(selected_base[1]):.2f})",
                error=False,
            )
        else:
            self._set_status(f"✓ Seleccionado: {name}", error=False)
        if self._last_grasp_px and self._last_camera_frame:
            _qimg, w, h, _ts = self._last_camera_frame
            self._refresh_cornell_metrics(w, h)
            self._refresh_science_ui()
        self._update_objects()
        self._log_selection_tf(self._selected_world)

    def _log_selection_tf(self, world_pose: Tuple[float, float, float], frame: str = WORLD_FRAME or "world"):
        """Store and log selection in business frame (base_link)."""
        if not world_pose:
            return
        world_frame = frame or self._expected_world_frame()
        self._last_selection_frame = world_frame
        self._last_selected_world_pose = (world_pose[0], world_pose[1], world_pose[2], world_frame)
        base_info = self._selection_to_base(world_pose, world_frame)
        if base_info:
            bx, by, bz = base_info["coords"]
            bf = str(base_info["frame"] or self._business_base_frame())
            self._selected_base = (float(bx), float(by), float(bz))
            self._selected_base_frame = bf
            self._last_selected_base_pose = (float(bx), float(by), float(bz), bf)
            self._emit_log(
                "[PICK] selected_base="
                f"({float(bx):.3f},{float(by):.3f},{float(bz):.3f}) frame={bf}"
            )
        else:
            self._emit_log("[PICK] selected_base unavailable (tf_pending)")
        self._start_tf_diagnose_async(world_pose, world_frame)
        self._start_pick_tf_resolve(world_pose, world_frame)

    def _start_tf_diagnose_async(self, world_pose: Tuple[float, float, float], world_frame: str) -> None:
        self._run_async(lambda: run_tf_diagnose(self, world_pose, world_frame))

    def _start_pick_tf_resolve(self, world_pose: Tuple[float, float, float], world_frame: str) -> None:
        if self._pick_tf_inflight:
            return
        if not self._moveit_required:
            self._emit_log("[PICK] Bloqueado: MoveIt deshabilitado")
            self._ui_set_status("PICK en espera: MoveIt no activo", error=False)
            return
        if self._managed_mode:
            if self._moveit_state != MoveItState.READY:
                reason = self._set_moveit_wait_status("pick")
                self._emit_log(f"[PICK] Bloqueado: MoveIt ({reason})")
                return
            if not self._state_ready_basic():
                reason = self._system_state_reason or self._system_state.value
                self._emit_log(f"[PICK] Bloqueado: estado={self._system_state.value}")
                self._ui_set_status(f"PICK en espera: {reason}", error=False)
                return
        elif not self._state_ready_moveit():
            reason = self._system_state_reason or self._system_state.value
            self._emit_log(f"[PICK] Bloqueado: estado={self._system_state.value}")
            self._ui_set_status(f"PICK en espera: {reason}", error=False)
            return
        if not self._pose_info_ok:
            self._emit_log("[PICK] Bloqueado: pose/info no disponible")
            self._ui_set_status("PICK en espera: pose/info no disponible", error=False)
            return
        tf_ok, tf_reason = tf_ready_status(self)
        if not tf_ok:
            self._emit_log(f"[PICK] Bloqueado: {tf_reason}")
            self._ui_set_status(f"PICK en espera: {tf_reason}", error=False)
            return
        self._pick_tf_inflight = True
        candidates = self._base_frame_candidates()
        base_frame = candidates[0]

        def worker():
            try:
                coords = None
                start = time.time()
                while (time.time() - start) < PICK_TF_TIMEOUT_SEC:
                    coords, _ = transform_point_to_frame(
                        world_pose,
                        base_frame,
                        source_frame=world_frame,
                        timeout_sec=PICK_TF_RETRY_SEC,
                    )
                    if coords:
                        break
                    self._wait_for_state_change(PICK_TF_RETRY_SEC)
                if not coords:
                    reason = self._tf_not_ready_reason()
                    self._log(f"[PICK] Bloqueado: {reason}")
                    self._ui_set_status(f"PICK en espera: {reason}", error=False)
                    return
                self._last_selected_base_pose = (coords[0], coords[1], coords[2], base_frame)
                self._selected_base = (float(coords[0]), float(coords[1]), float(coords[2]))
                self._selected_base_frame = base_frame
                pose_data = _make_pose_data(coords, frame=base_frame)
                if self._publish_moveit_pose("PICK_CLICK", pose_data):
                    # PICK_CLICK is fire-and-forget: do not keep global MoveIt lock latched.
                    self._motion_in_progress = False
                    self._ui_set_status("PICK click -> /desired_grasp publicado", error=False)
                else:
                    reason = self._moveit_block_reason or self._moveit_not_ready_reason()
                    self._ui_set_status(f"PICK click bloqueado: {reason}", error=False)
            finally:
                self._pick_tf_inflight = False

        self._run_async(worker)
    
    def _selection_candidate(self) -> Optional[Dict[str, object]]:
        """Return the last selected object/base tuple if still recent."""
        if not self._selected_object:
            return None
        if self._selected_base is None and self._selected_world is not None:
            source_frame = self._world_frame_last_first()
            self._selected_base = self._ensure_base_coords(
                tuple(self._selected_world), source_frame, timeout_sec=0.35
            )
            self._selected_base_frame = self._business_base_frame()
        if self._selected_base is None:
            return None
        age = time.time() - self._selection_timestamp
        if age > SELECTION_TIMEOUT_SEC:
            if getattr(self, "_pick_target_lock_active", False):
                lock_name = str(getattr(self, "_pick_target_lock_name", "") or "none")
                lock_id = str(getattr(self, "_pick_target_lock_id", "") or "n/a")
                self._emit_log_throttled(
                    "PICK:selection_timeout:lock_active",
                    "[PICK][SELECT] selection_timeout_ignored_due_to_pick_active=true "
                    f"name={self._selected_object} age={age:.1f}s lock_name={lock_name} lock_id={lock_id}",
                )
                frame = self._selected_base_frame or self._business_base_frame()
                return {
                    "name": self._selected_object,
                    "base": tuple(self._selected_base),
                    "frame": frame,
                    "age": age,
                }
            self._log(f"[PICK] Selección expirada (age={age:.1f}s) -> clearing selection")
            prev_state = get_object_state(self._selected_object)
            if prev_state and prev_state.logical_state == ObjectLogicalState.SELECTED:
                update_object_state(self._selected_object, logical_state=ObjectLogicalState.ON_TABLE, reason="select_timeout")
            self._selected_object = None
            self._selected_world = None
            self._selected_base = None
            return None
        frame = self._selected_base_frame or self._business_base_frame()
        return {
            "name": self._selected_object,
            "base": tuple(self._selected_base),
            "frame": frame,
            "age": age,
        }

    def _selection_to_base(self, world_pos: Tuple[float, float, float], source_frame: str) -> Optional[Dict[str, object]]:
        """Try transforming the selected point into a base frame (prefers effective frames)."""
        candidates = self._base_frame_candidates()
        for base_frame in candidates:
            coords, transform = transform_point_to_frame(world_pos, base_frame, source_frame)
            if coords:
                tx = ty = tz = 0.0
                yaw = 0.0
                if transform is not None:
                    tx = float(transform.transform.translation.x)
                    ty = float(transform.transform.translation.y)
                    tz = float(transform.transform.translation.z)
                    yaw = yaw_from_quaternion(transform.transform.rotation)
                return {
                    "coords": coords,
                    "frame": base_frame,
                    "transform": transform,
                    "translation": (tx, ty, tz),
                    "yaw": yaw,
                    "via_tf": bool(transform is not None),
                }
        return None

    def _on_object_clicked(self, name: str):
        """Manejar click en objeto de la lista (IGUAL A PANEL ONLY)."""
        if hasattr(self, "obj_panel") and not self.obj_panel.isEnabled():
            return
        self._emit_log(f"[PICK][LIST_CLICK] name={name}")
        ok, reason = pick_ui_status(self)
        if not ok:
            self._emit_log(
                f"[PICK][LIST_CLICK] not_ready_but_continuing name={name} reason={reason}"
            )
        if getattr(self, "_pick_target_lock_active", False):
            lock_name = str(getattr(self, "_pick_target_lock_name", "") or "none")
            lock_id = str(getattr(self, "_pick_target_lock_id", "") or "n/a")
            lock_source = str(getattr(self, "_pick_target_lock_source", "") or "n/a")
            lock_reason = str(getattr(self, "_pick_target_lock_reason", "") or "n/a")
            lock_ts = float(getattr(self, "_pick_target_lock_ts", 0.0) or 0.0)
            lock_age = max(0.0, time.time() - lock_ts) if lock_ts > 0.0 else 0.0
            self._emit_log(
                "[PICK][SELECT] ignored_selection_update_due_to_pick_active=true "
                f"source=object_list name={name} lock_name={lock_name} lock_id={lock_id} "
                f"lock_source={lock_source} lock_reason={lock_reason} lock_age={lock_age:.2f}s"
            )
            return
        # Obtener posición del objeto
        objects = get_object_positions()
        if name not in objects:
            self._log(f"[PICK] Objeto {name} no encontrado en Gazebo")
            return
        
        x, y, z = objects[name]
        base_pose = self.get_pose_in_base(name, timeout_sec=0.35)
        base_obj = None
        if base_pose is not None:
            base_obj = (
                float(base_pose.pose.position.x),
                float(base_pose.pose.position.y),
                float(base_pose.pose.position.z),
            )
        if base_obj is not None:
            self._log(
                f"[PICK] Click en objeto: {name} @ base ({float(base_obj[0]):.3f}, {float(base_obj[1]):.3f}, {float(base_obj[2]):.3f})"
            )
        else:
            self._log(f"[PICK] Click en objeto: {name} @ base (n/a)")
        self._emit_log(
            "[PICK][SELECT_EVT] "
            f"source=object_list name={name} "
            + (
                f"base=({float(base_obj[0]):.3f},{float(base_obj[1]):.3f},{float(base_obj[2]):.3f})"
                if base_obj is not None
                else "base=(n/a)"
            )
        )
        
        # ✅ Convertir a píxel usando table_xy_to_pixel (IGUAL A PANEL ONLY)
        px, py = -1, -1
        # Usar las dimensiones de la última imagen recibida del CameraView
        w = getattr(self.camera_view, "_img_width", 0) if hasattr(self, "camera_view") else 0
        h = getattr(self.camera_view, "_img_height", 0) if hasattr(self, "camera_view") else 0
        
        if not (self._camera_stream_ok and self._pose_info_ok and w > 0 and h > 0):
            self._log("[PICK] Sin cámara/pose_info lista; omitiendo conversión a píxel")
        else:
            # Intentar convertir con coordenadas XYZ primero
            pix = world_xyz_to_pixel(x, y, z, w, h)
            if not pix:
                # Fallback a XY (igual a Panel Only)
                pix = table_xy_to_pixel(x, y, w, h)
            if pix:
                px, py = pix
                self._log(f"[PICK] Conversión XYZ→píxel OK: ({px}, {py})")
            else:
                self._log(f"[PICK] No se pudo convertir XYZ/XY→píxel, usando fallback (0,0)")
        
        state = get_object_state(name)
        geom_on_table = is_on_table((x, y, z))
        state_ok = bool(
            state
            and (
                state.logical_state == ObjectLogicalState.ON_TABLE
                or (state.logical_state == ObjectLogicalState.SELECTED and geom_on_table)
            )
        )
        if not state_ok:
            pos_txt = f"pos=({x:.3f},{y:.3f},{z:.3f})"
            details = f"state={state.logical_state.value}" if state else "state=unknown"
            from .panel_objects import table_geom_debug
            geom = table_geom_debug((x, y, z))
            self._emit_log(f"[PICK] Bloqueado: {name} no está ON_TABLE ({details} {pos_txt} {geom})")
            self._set_status("Objeto no está sobre la mesa", error=True)
            return
        self._select_object(name, px, py, x, y, z, source="object_list")
    
    def _update_objects(self):
        """Actualizar lista de objetos usando ObjectListPanel (mismo estilo que panel principal)."""
        if not hasattr(self, "obj_panel"):
            return

        objects = get_object_positions() if self._gz_running else {}
        pickable_map = {}
        moveit_ready = self._moveit_ready() and self._controllers_ok and self._tf_ready_state
        if objects:
            use_cache = self._pickable_map_cache if self._objects_settled else None
            if use_cache:
                for name, (x, y, _z) in objects.items():
                    state = get_object_state(name)
                    on_table_state = bool(
                        state
                        and state.logical_state in (ObjectLogicalState.ON_TABLE, ObjectLogicalState.SELECTED)
                    )
                    pickable_map[name] = bool(use_cache.get(name, False)) and on_table_state
            else:
                for name, (x, y, _z) in objects.items():
                    state = get_object_state(name)
                    on_table_state = bool(
                        state
                        and state.logical_state in (ObjectLogicalState.ON_TABLE, ObjectLogicalState.SELECTED)
                    )
                    pickable_map[name] = not object_out_of_reach(x, y) and on_table_state
        self.obj_panel.update_objects(objects, pickable=pickable_map)

        sel_text = "Sel: -"
        if self._selected_object and self._selected_base:
            sel_text = f"Sel: {self._selected_object}"
            if not moveit_ready:
                sel_text = f"{sel_text} · MoveIt no"
        self.obj_panel.set_selected(self._selected_object, sel_text)

    def _push_history(self, values: List[float], value: Optional[float], max_len: int = 30) -> None:
        if value is None or not math.isfinite(float(value)):
            return
        values.append(float(value))
        if len(values) > max_len:
            del values[0]

    def _mean_history(self, values: List[float]) -> Optional[float]:
        if not values:
            return None
        return float(sum(values) / max(len(values), 1))

    def _update_fps_stats(self, fps: float) -> None:
        if fps <= 0.0 or not math.isfinite(float(fps)):
            return
        self._push_history(self._perf_fps_hist, fps, max_len=30)
        avg = self._mean_history(self._perf_fps_hist)
        self._perf_fps_avg = avg or 0.0
        now = time.time()
        if (now - self._perf_ui_last_ts) >= 0.5:
            self._perf_ui_last_ts = now
            self._refresh_science_ui()

    def _discover_tfm_checkpoints(self, allow_rgbd: Optional[bool] = None) -> List[str]:
        del allow_rgbd  # Selector fijo EXP1..EXP4, independiente del checkbox depth.
        exp_names = (
            "EXP1_SIMPLE_RGB",
            "EXP2_SIMPLE_RGBD",
            "EXP3_RESNET18_RGB_AUGMENT",
            "EXP4_RESNET18_RGBD",
        )
        exp_root = Path(VISION_EXP_DIR).expanduser()
        ckpts: List[str] = []
        ckpt_meta: Dict[str, Dict[str, object]] = {}

        for exp_name in exp_names:
            exp_dir = exp_root / exp_name
            if not exp_dir.exists():
                continue

            best_seed: Optional[int] = None
            best_success: Optional[float] = None
            best_iou: Optional[float] = None
            summary = exp_dir / "best_epoch_summary.csv"
            if summary.exists():
                try:
                    with summary.open("r", encoding="utf-8", newline="") as fh:
                        reader = csv.DictReader(fh)
                        for row in reader:
                            try:
                                success = float(row.get("val_success", "nan"))
                                seed_val = float(row.get("seed", "nan"))
                            except Exception:
                                continue
                            if not math.isfinite(success) or not math.isfinite(seed_val):
                                continue
                            if best_success is None or success > best_success:
                                best_success = success
                                best_seed = int(seed_val)
                                try:
                                    iou = float(row.get("val_iou", "nan"))
                                    best_iou = iou if math.isfinite(iou) else None
                                except Exception:
                                    best_iou = None
                except Exception:
                    pass

            candidates: List[Path] = []
            if best_seed is not None:
                candidates.append(exp_dir / f"seed_{best_seed}" / "checkpoints" / "best.pth")
            candidates.extend(sorted(exp_dir.glob("seed_*/checkpoints/best.pth")))
            chosen = next((p for p in candidates if p.exists()), None)
            if not chosen:
                continue

            ckpt = str(chosen)
            ckpts.append(ckpt)
            ckpt_meta[ckpt] = {
                "experiment": exp_name,
                "seed": best_seed,
                "val_success": best_success,
                "val_iou": best_iou,
            }

        self._tfm_ckpt_meta = ckpt_meta
        self._audit_write_json(
            "artifacts/checkpoints_index.json",
            {
                "root": str(exp_root),
                "count": len(ckpts),
                "entries": [
                    {
                        "path": ckpt,
                        "experiment": ckpt_meta.get(ckpt, {}).get("experiment"),
                        "seed": ckpt_meta.get(ckpt, {}).get("seed"),
                        "val_success": ckpt_meta.get(ckpt, {}).get("val_success"),
                        "val_iou": ckpt_meta.get(ckpt, {}).get("val_iou"),
                    }
                    for ckpt in ckpts
                ],
            },
        )
        return ckpts

    def _refresh_tfm_checkpoint_options(self) -> None:
        self._tfm_ckpt_options = self._discover_tfm_checkpoints(allow_rgbd=True)
        if self._tfm_ckpt_options:
            selected_valid = bool(self._tfm_ckpt_selected and self._tfm_ckpt_selected in self._tfm_ckpt_options)
            if not selected_valid:
                self._tfm_ckpt_selected = self._tfm_ckpt_options[0]
        if not hasattr(self, "combo_tfm_experiment"):
            self._load_experiment_info()
            self._refresh_science_ui()
            return
        self.combo_tfm_experiment.clear()
        if self._tfm_ckpt_options:
            for ckpt in self._tfm_ckpt_options:
                self.combo_tfm_experiment.addItem(self._format_ckpt_label(ckpt), ckpt)
            if self._tfm_ckpt_selected:
                idx = self.combo_tfm_experiment.findData(self._tfm_ckpt_selected)
                if idx >= 0:
                    self.combo_tfm_experiment.setCurrentIndex(idx)
                    self._tfm_ckpt_selected = str(self.combo_tfm_experiment.itemData(idx) or self._tfm_ckpt_selected)
        else:
            self.combo_tfm_experiment.addItem("Sin checkpoints encontrados", "")
        self._load_experiment_info()
        self._refresh_science_ui()

    def _format_ckpt_label(self, ckpt_path: str) -> str:
        meta = getattr(self, "_tfm_ckpt_meta", {}).get(ckpt_path, {}) if hasattr(self, "_tfm_ckpt_meta") else {}
        exp_name = str(meta.get("experiment") or "").strip()
        if exp_name:
            seed = meta.get("seed")
            success = meta.get("val_success")
            success_txt = "--"
            try:
                if success is not None and math.isfinite(float(success)):
                    success_txt = f"{float(success) * 100.0:.1f}%"
            except Exception:
                success_txt = "--"
            if isinstance(seed, (int, float)) and math.isfinite(float(seed)):
                seed_txt = f"seed_{int(float(seed))}"
            else:
                seed_txt = "seed_?"
            return f"{exp_name} | mejor {seed_txt} | acierto {success_txt}"
        path = Path(ckpt_path)
        if path.parent.name == "checkpoints" and path.parent.parent.name:
            return f"{path.parent.parent.name}/{path.name}"
        return path.name

    def _tfm_get_ckpt_path(self) -> str:
        if hasattr(self, "combo_tfm_experiment"):
            data = self.combo_tfm_experiment.currentData()  # type: ignore[attr-defined]
            if isinstance(data, str) and data:
                return data
        return self._tfm_ckpt_selected or ""

    def _tfm_apply_experiment(self) -> None:
        ckpt_path = self._tfm_get_ckpt_path()
        if not ckpt_path:
            self._set_status("TFM: sin checkpoint seleccionado", error=True)
            return
        self._tfm_ckpt_selected = ckpt_path
        self._load_experiment_info()
        self._refresh_science_ui()
        ckpt_info = {
            "path": ckpt_path,
            "exists": False,
            "size_bytes": None,
            "sha256": "",
        }
        try:
            p = Path(ckpt_path)
            if p.is_file():
                ckpt_info["exists"] = True
                ckpt_info["size_bytes"] = p.stat().st_size
                ckpt_info["sha256"] = self._sha256_file(str(p))
        except Exception:
            pass
        if self.tfm_module:
            self.tfm_module.load_model(ckpt_path)
            err = self.tfm_module.last_error()
            if err:
                self._set_status(f"TFM: checkpoint aplicado (sin carga de modelo: {err})", error=False)
                self._audit_append(
                    "logs/apply_experiment.log",
                    f"[TFM] apply_experiment FAIL ckpt={ckpt_path} err={err}",
                )
                return
        model_info = self.tfm_module.model_info() if self.tfm_module else {}
        self._load_experiment_info()
        self._refresh_science_ui()
        self._set_status("TFM: experimento aplicado", error=False)
        self._audit_append(
            "logs/apply_experiment.log",
            f"[TFM] apply_experiment OK ckpt={ckpt_path} size={ckpt_info['size_bytes']} sha256={ckpt_info['sha256']} model={model_info}",
        )

    def _tfm_reset_grasp(self) -> None:
        self._last_grasp_px = None
        self._last_grasp_world = None
        self._last_grasp_base = None
        self._last_cornell_ref = None
        self._last_cornell_reason = "Inferir y seleccionar un objeto"
        self._tfm_visual_compare_enabled = False
        self._last_grasp_frame = ""
        self._last_grasp_source = ""
        self._last_infer_image_path = ""
        self._last_infer_output_path = ""
        self._last_cornell = None
        if self.tfm_module:
            self.tfm_module.reset()
        self._refresh_science_ui()
        self._set_status("TFM: reset", error=False)
        self._audit_append("logs/reset.log", "[TFM] reset OK")

    def _load_experiment_info(self) -> None:
        info: Dict[str, object] = {
            "model": "--",
            "modality": "--",
            "experiment": "--",
            "experiment_base": "--",
            "seed": "--",
            "epoch": "--",
            "val_success_pct": "--",
            "val_iou": "--",
            "weights": "--",
            "weights_path": "",
            "config_path": "",
        }
        ckpt_value = self._tfm_ckpt_selected or INFER_CKPT
        ckpt_path = Path(ckpt_value).expanduser() if ckpt_value else None
        if ckpt_path:
            info["weights"] = ckpt_path.name
            info["weights_path"] = str(ckpt_path)
        seed_dir = None
        exp_dir = None
        if ckpt_path and ckpt_path.exists():
            if ckpt_path.parent.name == "checkpoints":
                seed_dir = ckpt_path.parent.parent
            else:
                seed_dir = ckpt_path.parent
        if seed_dir:
            seed_match = re.match(r"seed_(\d+)", seed_dir.name)
            if seed_match:
                info["seed"] = seed_match.group(1)
                exp_dir = seed_dir.parent
            else:
                exp_dir = seed_dir
        if exp_dir and exp_dir.name:
            info["experiment"] = exp_dir.name
            info["experiment_base"] = exp_dir.name
        cfg_dir = seed_dir or exp_dir
        if cfg_dir:
            cfg_path = cfg_dir / "config_snapshot.yaml"
            if not cfg_path.exists():
                cfg_path = cfg_dir / "config_used.yaml"
            info["config_path"] = str(cfg_path)
            if yaml is not None and cfg_path.exists():
                try:
                    data = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
                except Exception:
                    data = {}
                if isinstance(data, dict):
                    model_cfg = data.get("model", {}) if isinstance(data.get("model", {}), dict) else {}
                    data_cfg = data.get("data", {}) if isinstance(data.get("data", {}), dict) else {}
                    model_name = str(model_cfg.get("name", "")).strip()
                    if model_name:
                        info["model"] = model_name
                    modality = str(data_cfg.get("modality", "")).strip().lower()
                    if modality:
                        info["modality"] = "RGB-D" if modality == "rgbd" else modality.upper()
                    elif data_cfg.get("use_depth"):
                        info["modality"] = "RGB-D"
        meta = getattr(self, "_tfm_ckpt_meta", {}).get(str(ckpt_path), {}) if ckpt_path else {}
        if isinstance(meta, dict):
            success = meta.get("val_success")
            iou = meta.get("val_iou")
            try:
                if success is not None and math.isfinite(float(success)):
                    info["val_success_pct"] = f"{float(success) * 100.0:.1f}%"
            except Exception:
                pass
            try:
                if iou is not None and math.isfinite(float(iou)):
                    info["val_iou"] = f"{float(iou):.3f}"
            except Exception:
                pass
        if ckpt_path and ckpt_path.exists():
            try:
                import torch  # type: ignore
            except Exception:
                torch = None
            if torch is not None:
                try:
                    ckpt = torch.load(str(ckpt_path), map_location="cpu")
                    if isinstance(ckpt, dict):
                        epoch = ckpt.get("epoch")
                        if epoch is None and isinstance(ckpt.get("metrics"), dict):
                            epoch = ckpt.get("metrics", {}).get("epoch")
                        if epoch is not None:
                            info["epoch"] = str(int(epoch))
                except Exception:
                    pass
        self._exp_info = info

    def _format_value(self, value: Optional[float], fmt: str, suffix: str = "") -> str:
        if value is None or not math.isfinite(float(value)):
            return "--"
        return f"{fmt.format(value)}{suffix}"

    def _refresh_science_ui(self) -> None:
        if self.lbl_cornell_iou:
            iou = self._last_cornell.get("iou") if self._last_cornell else None
            self.lbl_cornell_iou.setText(self._format_value(iou, "{:.3f}"))
        if self.lbl_cornell_theta:
            dtheta = self._last_cornell.get("dtheta") if self._last_cornell else None
            self.lbl_cornell_theta.setText(self._format_value(dtheta, "{:.1f}", "°"))
        if self.lbl_cornell_success:
            success = self._last_cornell.get("success") if self._last_cornell else None
            if success is None:
                self.lbl_cornell_success.setText("--")
            else:
                self.lbl_cornell_success.setText("SUCCESS" if success else "FAIL")
        if self.lbl_cornell_note:
            if self._cornell_metrics:
                base_note = "Evaluación geométrica en simulación. No validación física."
                detail = str(self._last_cornell_reason or "").strip()
                self.lbl_cornell_note.setText(f"{base_note} {detail}" if detail else base_note)
            else:
                detail = self._cornell_metrics_err or "dependencia no disponible"
                self.lbl_cornell_note.setText(f"Cornell offline: {detail}")
        if self.lbl_exp_model:
            self.lbl_exp_model.setText(str(self._exp_info.get("model", "--")))
        if self.lbl_exp_modality:
            self.lbl_exp_modality.setText(str(self._exp_info.get("modality", "--")))
        if self.lbl_exp_name:
            self.lbl_exp_name.setText(str(self._exp_info.get("experiment", "--")))
        if self.lbl_exp_seed:
            self.lbl_exp_seed.setText(str(self._exp_info.get("seed", "--")))
        if self.lbl_exp_epoch:
            self.lbl_exp_epoch.setText(str(self._exp_info.get("epoch", "--")))
        if self.lbl_exp_success:
            self.lbl_exp_success.setText(str(self._exp_info.get("val_success_pct", "--")))
        if self.lbl_exp_iou:
            self.lbl_exp_iou.setText(str(self._exp_info.get("val_iou", "--")))
        if self.lbl_exp_weights:
            self.lbl_exp_weights.setText(str(self._exp_info.get("weights", "--")))
        if self.lbl_perf_infer:
            avg = self._mean_history(self._perf_infer_hist)
            inst = self._perf_infer_ms
            if inst is None:
                self.lbl_perf_infer.setText("--")
            else:
                avg_txt = f"{avg:.1f} ms" if avg is not None else "--"
                self.lbl_perf_infer.setText(f"{inst:.1f} ms (avg {avg_txt})")
        if self.lbl_perf_total:
            avg = self._mean_history(self._perf_total_hist)
            inst = self._perf_total_ms
            if inst is None:
                self.lbl_perf_total.setText("--")
            else:
                avg_txt = f"{avg:.1f} ms" if avg is not None else "--"
                self.lbl_perf_total.setText(f"{inst:.1f} ms (avg {avg_txt})")
        if self.lbl_perf_fps:
            inst = self._camera_last_fps
            avg = self._perf_fps_avg if self._perf_fps_hist else None
            if inst <= 0.0:
                self.lbl_perf_fps.setText("--")
            else:
                avg_txt = f"{avg:.1f}" if avg is not None else "--"
                self.lbl_perf_fps.setText(f"{inst:.1f} (avg {avg_txt})")
        if self.lbl_grasp_img:
            if self._last_grasp_px:
                g = self._last_grasp_px
                self.lbl_grasp_img.setText(
                    f"cx={g.get('cx', 0.0):.1f}, cy={g.get('cy', 0.0):.1f}, "
                    f"w={g.get('w', 0.0):.1f}, h={g.get('h', 0.0):.1f}, "
                    f"theta={g.get('angle_deg', 0.0):.1f}°"
                )
            else:
                self.lbl_grasp_img.setText("--")
        if self.lbl_grasp_world:
            if self._last_grasp_base:
                g = self._last_grasp_base
                yaw = g.get("yaw_deg")
                yaw_txt = f"{yaw:.1f}°" if yaw is not None else "--"
                self.lbl_grasp_world.setText(
                    f"x={g.get('x', 0.0):.3f}, y={g.get('y', 0.0):.3f}, "
                    f"z={g.get('z', 0.0):.3f}, yaw={yaw_txt}"
                )
            else:
                self.lbl_grasp_world.setText("--")
        if self.lbl_grasp_frame:
            image_frame = self.camera_topic or "image"
            base_frame = self._business_base_frame()
            self.lbl_grasp_frame.setText(f"image={image_frame} | base={base_frame}")

    def _world_to_pixel(self, x: float, y: float, z: float, w: int, h: int) -> Optional[Tuple[int, int]]:
        pix = world_xyz_to_pixel(x, y, z, w, h)
        if not pix:
            pix = table_xy_to_pixel(x, y, w, h)
        return pix

    def _world_to_pixel_diag(
        self,
        x: float,
        y: float,
        z: float,
        w: int,
        h: int,
    ) -> Tuple[Optional[Tuple[int, int]], Optional[Tuple[float, float]], str]:
        pix_float = world_xyz_to_pixel_float(x, y, z, w, h)
        if pix_float is not None:
            pix_int = world_xyz_to_pixel(x, y, z, w, h)
            return pix_int, pix_float, "world_xyz"
        pix_float = table_xy_to_pixel_float(x, y, w, h)
        if pix_float is not None:
            pix_int = table_xy_to_pixel(x, y, w, h)
            return pix_int, pix_float, "table_xy"
        return None, None, "none"

    def _build_reference_grasp(self, frame_w: int, frame_h: int) -> Optional[Dict[str, float]]:
        if not self._selected_object:
            return None
        if frame_w <= 0 or frame_h <= 0:
            return None
        self._load_sdf_geometry_cache()
        sdf = self._sdf_model_cache.get(self._selected_object, {})
        geom_type = sdf.get("type")
        size = sdf.get("size")
        radius = sdf.get("radius")
        width_m = None
        height_m = None
        if geom_type == "box" and size and len(size) == 3:
            width_m = float(size[0])
            height_m = float(size[1])
        elif geom_type in ("cylinder", "sphere") and radius:
            width_m = float(radius) * 2.0
            height_m = float(radius) * 2.0
        if not width_m or not height_m:
            return None
        pose_source = "selection"
        live_pose = None
        if getattr(self, "_ros_worker_started", False) and getattr(self, "ros_worker", None) is not None:
            try:
                poses, _pose_ts = self.ros_worker.pose_snapshot()
            except Exception:
                poses = {}
            live_pose = poses.get(self._selected_object)
        if live_pose is not None and len(live_pose) >= 3:
            wx, wy, wz = float(live_pose[0]), float(live_pose[1]), float(live_pose[2])
            self._selected_world = (wx, wy, wz)
            selected_base = self._ensure_base_coords(
                (wx, wy, wz),
                self._world_frame_last_first(),
                timeout_sec=0.35,
            )
            if selected_base is not None:
                self._selected_base = selected_base
                self._selected_base_frame = self._business_base_frame()
            pose_source = "pose_snapshot"
        elif self._selected_world:
            wx, wy, wz = self._selected_world
        elif self._selected_base:
            wx, wy, wz = base_to_world(
                float(self._selected_base[0]),
                float(self._selected_base[1]),
                float(self._selected_base[2]),
            )
            pose_source = "selected_base"
        else:
            return None
        z = wz if wz and wz > 0 else self._resolve_table_top_z()
        if not self._selected_px or self._selected_px[0] < 0 or self._selected_px[1] < 0:
            selected_px, _selected_px_float, _selected_px_src = self._world_to_pixel_diag(wx, wy, z, frame_w, frame_h)
            if selected_px:
                self._selected_px = (int(selected_px[0]), int(selected_px[1]))
        center_px, center_px_float, center_src = self._world_to_pixel_diag(wx, wy, z, frame_w, frame_h)
        if not center_px or not center_px_float:
            self._audit_append(
                "logs/infer.log",
                "[TFM] infer_ref_diag "
                f"selected={self._selected_object or 'none'} pose_source={pose_source} "
                f"geom={geom_type or 'unknown'} size_m=({float(width_m or 0.0):.4f},{float(height_m or 0.0):.4f}) "
                f"world=({float(wx):.4f},{float(wy):.4f},{float(z):.4f}) center_src={center_src}",
            )
            return None
        left_px, left_px_float, left_src = self._world_to_pixel_diag(wx - width_m / 2.0, wy, z, frame_w, frame_h)
        right_px, right_px_float, right_src = self._world_to_pixel_diag(wx + width_m / 2.0, wy, z, frame_w, frame_h)
        down_px, down_px_float, down_src = self._world_to_pixel_diag(wx, wy - height_m / 2.0, z, frame_w, frame_h)
        up_px, up_px_float, up_src = self._world_to_pixel_diag(wx, wy + height_m / 2.0, z, frame_w, frame_h)
        if (
            not left_px or not right_px or not down_px or not up_px
            or not left_px_float or not right_px_float or not down_px_float or not up_px_float
        ):
            self._audit_append(
                "logs/infer.log",
                "[TFM] infer_ref_diag "
                f"selected={self._selected_object or 'none'} pose_source={pose_source} "
                f"geom={geom_type or 'unknown'} size_m=({float(width_m or 0.0):.4f},{float(height_m or 0.0):.4f}) "
                f"world=({float(wx):.4f},{float(wy):.4f},{float(z):.4f}) "
                f"center_src={center_src} left_src={left_src} right_src={right_src} down_src={down_src} up_src={up_src} "
                f"center={center_px_float} left={left_px_float} right={right_px_float} down={down_px_float} up={up_px_float}",
            )
            return None
        w_px = int(round(math.hypot(right_px[0] - left_px[0], right_px[1] - left_px[1])))
        h_px = int(round(math.hypot(up_px[0] - down_px[0], up_px[1] - down_px[1])))
        w_px_float = math.hypot(
            float(right_px_float[0]) - float(left_px_float[0]),
            float(right_px_float[1]) - float(left_px_float[1]),
        )
        h_px_float = math.hypot(
            float(up_px_float[0]) - float(down_px_float[0]),
            float(up_px_float[1]) - float(down_px_float[1]),
        )
        if w_px <= 1 or h_px <= 1 or w_px_float <= 2.0 or h_px_float <= 2.0:
            self._audit_append(
                "logs/infer.log",
                "[TFM] infer_ref_diag "
                f"selected={self._selected_object or 'none'} pose_source={pose_source} "
                f"geom={geom_type or 'unknown'} size_m=({float(width_m or 0.0):.4f},{float(height_m or 0.0):.4f}) "
                f"world=({float(wx):.4f},{float(wy):.4f},{float(z):.4f}) "
                f"srcs=center:{center_src},left:{left_src},right:{right_src},down:{down_src},up:{up_src} "
                f"center_f=({center_px_float[0]:.2f},{center_px_float[1]:.2f}) "
                f"left_f=({left_px_float[0]:.2f},{left_px_float[1]:.2f}) right_f=({right_px_float[0]:.2f},{right_px_float[1]:.2f}) "
                f"down_f=({down_px_float[0]:.2f},{down_px_float[1]:.2f}) up_f=({up_px_float[0]:.2f},{up_px_float[1]:.2f}) "
                f"span_float=({w_px_float:.2f},{h_px_float:.2f}) "
                f"center_i={center_px} left_i={left_px} right_i={right_px} down_i={down_px} up_i={up_px} "
                f"span_int=({w_px},{h_px})",
            )
        return {
            "cx": float(center_px_float[0]),
            "cy": float(center_px_float[1]),
            "w": float(max(1.0, w_px_float)),
            "h": float(max(1.0, h_px_float)),
            "angle_deg": 0.0,
        }

    def _compute_world_grasp(self, frame_w: int, frame_h: int) -> Optional[Dict[str, float]]:
        if not self._last_grasp_px:
            return None
        if frame_w <= 0 or frame_h <= 0:
            return None
        cx = self._last_grasp_px.get("cx", 0.0)
        cy = self._last_grasp_px.get("cy", 0.0)
        angle_deg = self._last_grasp_px.get("angle_deg", 0.0)
        table_top = self._resolve_table_top_z()
        px = int(round(cx))
        py = int(round(cy))
        wx, wy = pixel_to_table_xy(px, py, frame_w, frame_h, z_target=table_top)
        step = 10.0
        dx = math.cos(math.radians(angle_deg)) * step
        dy = math.sin(math.radians(angle_deg)) * step
        wx2, wy2 = pixel_to_table_xy(int(round(cx + dx)), int(round(cy + dy)), frame_w, frame_h, z_target=table_top)
        yaw_deg = None
        if wx2 is not None and wy2 is not None:
            yaw_deg = math.degrees(math.atan2(wy2 - wy, wx2 - wx))
        return {"x": float(wx), "y": float(wy), "z": float(table_top), "yaw_deg": yaw_deg}

    def _update_cornell_metrics(self, pred: Dict[str, float], ref: Dict[str, float]) -> None:
        if not self._cornell_metrics:
            self._last_cornell = None
            return
        try:
            angle_diff_deg = self._cornell_metrics["angle_diff_deg"]
            compute_grasp_success = self._cornell_metrics["compute_grasp_success"]
            grasp_iou = self._cornell_metrics["grasp_iou"]
            pred_params = [pred["cx"], pred["cy"], pred["w"], pred["h"], pred["angle_deg"]]
            ref_params = [ref["cx"], ref["cy"], ref["w"], ref["h"], ref["angle_deg"]]
            iou = grasp_iou(pred_params, ref_params)
            dtheta = angle_diff_deg(float(pred_params[4]), float(ref_params[4]))
            success = bool(compute_grasp_success(pred_params, ref_params, iou_thresh=0.25, angle_thresh=30.0))
            self._last_cornell = {"iou": float(iou), "dtheta": float(dtheta), "success": success}
        except Exception:
            self._last_cornell = None

    def _refresh_cornell_metrics(self, frame_w: int, frame_h: int) -> None:
        self._last_cornell = None
        self._last_cornell_ref = None
        if not self._cornell_metrics:
            self._last_cornell_reason = f"Cornell offline: {self._cornell_metrics_err or 'dependencia no disponible'}"
            return
        if not self._last_grasp_px:
            self._last_cornell_reason = "Inferir agarre primero"
            return
        if not self._selected_object:
            self._last_cornell_reason = "Selecciona un objeto para comparar con referencia Cornell"
            return
        if frame_w <= 0 or frame_h <= 0:
            self._last_cornell_reason = "Sin frame válido para proyectar referencia"
            return
        ref = self._build_reference_grasp(frame_w, frame_h)
        if not ref:
            self._last_cornell_reason = f"No se pudo proyectar referencia para {self._selected_object}"
            return
        self._last_cornell_ref = ref
        self._update_cornell_metrics(self._last_grasp_px, ref)
        if self._last_cornell:
            pred = self._last_grasp_px
            self._last_cornell_reason = (
                f"Ref {self._selected_object}: pred {pred['w']:.1f}x{pred['h']:.1f}px vs ref {ref['w']:.1f}x{ref['h']:.1f}px"
            )
            self._audit_append(
                "logs/infer.log",
                "[TFM] infer_ref "
                f"selected={self._selected_object or 'none'} "
                f"pred={{'w': {pred['w']:.2f}, 'h': {pred['h']:.2f}, 'angle_deg': {pred['angle_deg']:.2f}}} "
                f"ref={{'cx': {ref['cx']:.2f}, 'cy': {ref['cy']:.2f}, 'w': {ref['w']:.2f}, 'h': {ref['h']:.2f}, 'angle_deg': {ref['angle_deg']:.2f}}}",
            )
        else:
            self._last_cornell_reason = f"No se pudieron calcular métricas para {self._selected_object}"

    def _build_science_group(self) -> QGroupBox:
        science_group = QGroupBox("")
        science_group.setFlat(True)
        science_group.setStyleSheet("")
        science_layout = QVBoxLayout()
        science_layout.setContentsMargins(6, 6, 6, 6)
        science_layout.setSpacing(4)

        block_css = (
            "QGroupBox {"
            "font-size:11px; font-weight:700;"
            "background:#f8fafc;"
            "border:1px solid #cbd5f5;"
            "border-radius:8px;"
            "margin-top:6px;"
            "padding:8px;"
            "}"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; top: -4px; padding:0 4px; }"
            "QLabel { font-size:10px; }"
        )

        cornell_group = QGroupBox("Evaluación Cornell (online - simulación)")
        cornell_group.setFlat(True)
        cornell_group.setStyleSheet(block_css)
        cornell_layout = QGridLayout()
        cornell_layout.setContentsMargins(2, 2, 2, 2)
        cornell_layout.setHorizontalSpacing(6)
        cornell_layout.setVerticalSpacing(3)
        cornell_layout.addWidget(QLabel("IoU"), 0, 0)
        self.lbl_cornell_iou = QLabel("--")
        cornell_layout.addWidget(self.lbl_cornell_iou, 0, 1)
        cornell_layout.addWidget(QLabel("dTheta"), 0, 2)
        self.lbl_cornell_theta = QLabel("--")
        cornell_layout.addWidget(self.lbl_cornell_theta, 0, 3)
        cornell_layout.addWidget(QLabel("Success"), 1, 0)
        self.lbl_cornell_success = QLabel("--")
        cornell_layout.addWidget(self.lbl_cornell_success, 1, 1)
        self.lbl_cornell_note = QLabel("Evaluación geométrica en simulación. No validación física.")
        self.lbl_cornell_note.setStyleSheet("color:#64748b; font-size:9px;")
        cornell_layout.addWidget(self.lbl_cornell_note, 2, 0, 1, 4)
        cornell_group.setLayout(cornell_layout)
        science_layout.addWidget(cornell_group)

        exp_group = QGroupBox("Configuración experimental activa")
        exp_group.setFlat(True)
        exp_group.setStyleSheet(block_css)
        exp_layout = QGridLayout()
        exp_layout.setContentsMargins(2, 2, 2, 2)
        exp_layout.setHorizontalSpacing(6)
        exp_layout.setVerticalSpacing(3)
        exp_layout.addWidget(QLabel("Modelo"), 0, 0)
        self.lbl_exp_model = QLabel("--")
        exp_layout.addWidget(self.lbl_exp_model, 0, 1)
        exp_layout.addWidget(QLabel("Modalidad"), 0, 2)
        self.lbl_exp_modality = QLabel("--")
        exp_layout.addWidget(self.lbl_exp_modality, 0, 3)
        exp_layout.addWidget(QLabel("Experimento"), 1, 0)
        self.lbl_exp_name = QLabel("--")
        exp_layout.addWidget(self.lbl_exp_name, 1, 1)
        exp_layout.addWidget(QLabel("Semilla"), 1, 2)
        self.lbl_exp_seed = QLabel("--")
        exp_layout.addWidget(self.lbl_exp_seed, 1, 3)
        exp_layout.addWidget(QLabel("Epoch"), 2, 0)
        self.lbl_exp_epoch = QLabel("--")
        exp_layout.addWidget(self.lbl_exp_epoch, 2, 1)
        exp_layout.addWidget(QLabel("Acierto"), 2, 2)
        self.lbl_exp_success = QLabel("--")
        exp_layout.addWidget(self.lbl_exp_success, 2, 3)
        exp_layout.addWidget(QLabel("IoU val"), 3, 0)
        self.lbl_exp_iou = QLabel("--")
        exp_layout.addWidget(self.lbl_exp_iou, 3, 1)
        exp_layout.addWidget(QLabel("Pesos"), 3, 2)
        self.lbl_exp_weights = QLabel("--")
        self.lbl_exp_weights.setWordWrap(True)
        exp_layout.addWidget(self.lbl_exp_weights, 3, 3)
        exp_group.setLayout(exp_layout)
        science_layout.addWidget(exp_group)

        perf_group = QGroupBox("Rendimiento temporal")
        perf_group.setFlat(True)
        perf_group.setStyleSheet(block_css)
        perf_layout = QGridLayout()
        perf_layout.setContentsMargins(2, 2, 2, 2)
        perf_layout.setHorizontalSpacing(6)
        perf_layout.setVerticalSpacing(3)
        perf_layout.addWidget(QLabel("Latencia inferencia"), 0, 0)
        self.lbl_perf_infer = QLabel("--")
        perf_layout.addWidget(self.lbl_perf_infer, 0, 1)
        perf_layout.addWidget(QLabel("Latencia percepción"), 1, 0)
        self.lbl_perf_total = QLabel("--")
        perf_layout.addWidget(self.lbl_perf_total, 1, 1)
        perf_layout.addWidget(QLabel("FPS efectivo"), 2, 0)
        self.lbl_perf_fps = QLabel("--")
        perf_layout.addWidget(self.lbl_perf_fps, 2, 1)
        perf_group.setLayout(perf_layout)
        science_layout.addWidget(perf_group)

        grasp_group = QGroupBox("Parámetros del grasp")
        grasp_group.setFlat(True)
        grasp_group.setStyleSheet(block_css)
        grasp_layout = QGridLayout()
        grasp_layout.setContentsMargins(2, 2, 2, 2)
        grasp_layout.setHorizontalSpacing(6)
        grasp_layout.setVerticalSpacing(3)
        grasp_layout.addWidget(QLabel("Imagen (px)"), 0, 0)
        self.lbl_grasp_img = QLabel("--")
        self.lbl_grasp_img.setWordWrap(True)
        grasp_layout.addWidget(self.lbl_grasp_img, 0, 1)
        grasp_layout.addWidget(QLabel("Mundo (m)"), 1, 0)
        self.lbl_grasp_world = QLabel("--")
        self.lbl_grasp_world.setWordWrap(True)
        grasp_layout.addWidget(self.lbl_grasp_world, 1, 1)
        grasp_layout.addWidget(QLabel("Frame"), 2, 0)
        self.lbl_grasp_frame = QLabel("--")
        grasp_layout.addWidget(self.lbl_grasp_frame, 2, 1)
        grasp_group.setLayout(grasp_layout)
        science_layout.addWidget(grasp_group)

        self.btn_save_episode = QPushButton("Guardar episodio experimental")
        self.btn_save_episode.clicked.connect(self._save_episode)
        science_layout.addWidget(self.btn_save_episode)

        science_group.setLayout(science_layout)
        return science_group

    def _build_trace_group(self) -> QGroupBox:
        trace_group = QGroupBox("")
        trace_group.setStyleSheet(
            "QGroupBox {"
            "font-size:11px; font-weight:700;"
            "background:#f8fafc;"
            "border:1px solid #cbd5f5;"
            "border-radius:8px;"
            "margin-top:6px;"
            "padding:8px;"
            "}"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; top: -4px; padding:0 4px; }"
            "QLabel { font-size:10px; color:#0f172a; }"
        )
        trace_layout = QVBoxLayout()
        trace_layout.setContentsMargins(6, 4, 6, 4)
        trace_layout.setSpacing(2)

        self.lbl_trace_frames = QLabel("Frames: world=..., base=..., ee=...")
        self.lbl_trace_frames.setStyleSheet("font-size: 11px; font-weight: 600;")
        trace_layout.addWidget(self.lbl_trace_frames)

        self.trace_table = QTableWidget(2, 8)
        self.trace_table.setHorizontalHeaderLabels(["frame", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.trace_table.setVerticalHeaderLabels(["Object", "TCP"])
        self.trace_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.trace_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.trace_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.trace_table.setSelectionMode(QTableWidget.NoSelection)
        self.trace_table.setStyleSheet(
            "font-size:10px;"
            "background:#ffffff;"
            "border:none;"
            "gridline-color:#e2e8f0;"
        )
        self.trace_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.trace_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.trace_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.trace_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.trace_table.setFixedHeight(120)
        self.trace_table.setShowGrid(True)
        self.trace_table.setFrameStyle(QTableWidget.Box | QTableWidget.Plain)
        trace_layout.addWidget(self.trace_table)

        error_block = QWidget()
        error_layout = QGridLayout()
        error_layout.setContentsMargins(0, 0, 0, 0)
        error_layout.addWidget(QLabel("<b>Base</b> (m):"), 0, 0)
        self.lbl_trace_error_base = QLabel("--")
        error_layout.addWidget(self.lbl_trace_error_base, 0, 1)
        error_layout.addWidget(QLabel("<b>World</b> (m):"), 1, 0)
        self.lbl_trace_error_world = QLabel("--")
        error_layout.addWidget(self.lbl_trace_error_world, 1, 1)
        error_block.setLayout(error_layout)
        trace_layout.addWidget(error_block)

        tf_block = QWidget()
        tf_layout = QGridLayout()
        tf_layout.setContentsMargins(0, 0, 0, 0)
        tf_layout.addWidget(QLabel("translation:"), 0, 0)
        self.lbl_trace_tf_translation = QLabel("--")
        tf_layout.addWidget(self.lbl_trace_tf_translation, 0, 1)
        tf_layout.addWidget(QLabel("yaw:"), 1, 0)
        self.lbl_trace_tf_yaw = QLabel("--")
        tf_layout.addWidget(self.lbl_trace_tf_yaw, 1, 1)
        tf_block.setLayout(tf_layout)
        trace_layout.addWidget(tf_block)

        control_row = QHBoxLayout()
        self.chk_trace_freeze = QCheckBox("Freeze")
        control_row.addWidget(self.chk_trace_freeze)
        control_row.addStretch(1)
        self.btn_trace_diag = QPushButton("TRACE DIAG ONCE")
        self.btn_trace_diag.clicked.connect(self._run_trace_diag_once)
        self.btn_trace_diag.setStyleSheet("font-size:9px; padding:2px 8px;")
        control_row.addWidget(self.btn_trace_diag)
        self.btn_self_check = QPushButton("SELF CHECK")
        self.btn_self_check.clicked.connect(self._run_self_check_once)
        self.btn_self_check.setStyleSheet("font-size:9px; padding:2px 8px;")
        control_row.addWidget(self.btn_self_check)
        self.btn_copy_trace = QPushButton("Copy trace")
        self.btn_copy_trace.clicked.connect(self._copy_trace_text)
        self.btn_copy_trace.setStyleSheet("font-size:9px; padding:2px 8px;")
        control_row.addWidget(self.btn_copy_trace)
        trace_layout.addLayout(control_row)

        trace_group.setLayout(trace_layout)
        return trace_group

    def _start_trace_timer(self):
        if self._trace_timer:
            return
        self._trace_timer = QTimer(self)
        self._trace_timer.setInterval(200)
        self._trace_timer.timeout.connect(self._refresh_trace_data)
        self._trace_timer.start()
        self._refresh_trace_data()

    def _resolve_trace_frames(self, world_frame: str) -> Tuple[str, Optional[str]]:
        _detected_base, detected_ee = discover_base_and_ee_frames(world_frame)
        effective_base = "base_link"
        self._base_frame_effective = "base_link"
        if detected_ee and detected_ee != self._ee_frame_effective:
            self._ee_frame_effective = detected_ee
        effective_ee = detected_ee or self._ee_frame_effective
        if not effective_ee:
            helper = get_tf_helper()
            for candidate in ("tool0", "flange"):
                if helper and _can_transform_between(helper, effective_base, candidate, timeout_sec=0.05):
                    effective_ee = candidate
                    self._ee_frame_effective = candidate
                    self._emit_log(f"[TF] EE resolved via {candidate}")
                    break
        return effective_base, effective_ee

    def _refresh_trace_data(self):
        if self._closing or not self.trace_table or not self._bridge_running:
            return
        if self.chk_trace_freeze and self.chk_trace_freeze.isChecked():
            return
        now = time.monotonic()
        world_frame = self._world_frame_last_first()
        base_frame, ee_frame = self._resolve_trace_frames(world_frame)
        self._log_tf_chain_once(world_frame, base_frame, ee_frame)
        frames_text = f"Frames: world={world_frame} base={base_frame} ee={ee_frame or 'unavailable'}"
        if self.lbl_trace_frames:
            self.lbl_trace_frames.setText(frames_text)

        object_world_data = None
        object_base_data = None
        object_source = "pose_info+tf2:UNAVAILABLE"
        object_name = str(self._selected_object or PICK_DEMO_OBJECT_NAME)
        pose_cache = {}
        pose_wall = 0.0
        if self._ros_worker_started and self.ros_worker.node_ready():
            try:
                pose_cache, pose_wall = self.ros_worker.pose_snapshot()
            except Exception as exc:
                self._log_trace_transform_warning(f"pose_snapshot: {exc}")
                pose_cache = {}
                pose_wall = 0.0
        if object_name and object_name in pose_cache:
            wx, wy, wz = pose_cache[object_name]
            object_world_data = self._pose_dict((float(wx), float(wy), float(wz)), (0.0, 0.0, 0.0, 1.0), world_frame)
            world_pose = PoseStamped()
            world_pose.header.frame_id = world_frame
            world_pose.pose.position.x = float(wx)
            world_pose.pose.position.y = float(wy)
            world_pose.pose.position.z = float(wz)
            world_pose.pose.orientation.w = 1.0
            base_pose, base_reason = tf_world_pose_to_base(
                world_pose,
                world_frame=world_frame,
                base_frame=base_frame,
                timeout=0.08,
                logger=self._log_trace,
            )
            if base_pose is not None:
                object_base_data = self._pose_dict(
                    (
                        float(base_pose.pose.position.x),
                        float(base_pose.pose.position.y),
                        float(base_pose.pose.position.z),
                    ),
                    (
                        float(base_pose.pose.orientation.x),
                        float(base_pose.pose.orientation.y),
                        float(base_pose.pose.orientation.z),
                        float(base_pose.pose.orientation.w),
                    ),
                    base_frame,
                )
                self._last_selected_world_pose = (float(wx), float(wy), float(wz), world_frame)
                self._last_selected_base_pose = (
                    float(base_pose.pose.position.x),
                    float(base_pose.pose.position.y),
                    float(base_pose.pose.position.z),
                    base_frame,
                )
                object_source = f"pose_info+tf2:ok age={max(0.0, time.time() - float(pose_wall or 0.0)):.2f}s"
            else:
                object_source = f"pose_info+tf2:tf_failed:{base_reason}"
        else:
            object_source = f"pose_info+tf2:not_found:{object_name}"

        tcp_world_data = None
        tcp_base_data = None
        tcp_source = "tf2:UNAVAILABLE"
        if ee_frame:
            tcp_pose_base, _tcp_rpy, tcp_reason = tf_get_tcp_in_base(
                base_frame=base_frame,
                ee_frame="rg2_tcp",
                timeout=0.08,
                logger=self._log_trace,
            )
            if tcp_pose_base is not None:
                tcp_base_data = self._pose_dict(
                    (
                        float(tcp_pose_base.pose.position.x),
                        float(tcp_pose_base.pose.position.y),
                        float(tcp_pose_base.pose.position.z),
                    ),
                    (
                        float(tcp_pose_base.pose.orientation.x),
                        float(tcp_pose_base.pose.orientation.y),
                        float(tcp_pose_base.pose.orientation.z),
                        float(tcp_pose_base.pose.orientation.w),
                    ),
                    base_frame,
                )
                self._last_trace_tcp_base = (
                    float(tcp_pose_base.pose.position.x),
                    float(tcp_pose_base.pose.position.y),
                    float(tcp_pose_base.pose.position.z),
                )
                self._last_trace_tcp_ts = time.monotonic()
                self._last_tcp_base_z = float(tcp_pose_base.pose.position.z)
                # FASE 1: tcp_source=ok en cuanto base->ee funciona.
                # La transformación a world es OPCIONAL (solo diagnóstico).
                tcp_source = "tf2:ok"
                helper = get_tf_helper()
                world_tf_available = bool(
                    helper
                    and _can_transform_between(
                        helper, base_frame, world_frame, timeout_sec=0.05
                    )
                )
                if world_tf_available:
                    tcp_world_pose, world_reason = tf_transform_pose(
                        tcp_pose_base,
                        target_frame=world_frame,
                        timeout=0.08,
                    )
                    if tcp_world_pose is not None:
                        tcp_world_data = self._pose_dict(
                            (
                                float(tcp_world_pose.pose.position.x),
                                float(tcp_world_pose.pose.position.y),
                                float(tcp_world_pose.pose.position.z),
                            ),
                            (
                                float(tcp_world_pose.pose.orientation.x),
                                float(tcp_world_pose.pose.orientation.y),
                                float(tcp_world_pose.pose.orientation.z),
                                float(tcp_world_pose.pose.orientation.w),
                            ),
                            world_frame,
                        )
                        self._last_tcp_world_tf = tcp_world_data
            else:
                self._last_trace_tcp_base = None
                self._last_trace_tcp_ts = time.monotonic()
                tcp_source = f"tf2:base_lookup_failed:{tcp_reason}"
        else:
            if now - self._last_ee_warn_ts >= self._ee_warn_period:
                self._log("[TRACE] EE frame unavailable (retrying)")
                self._last_ee_warn_ts = now
            if self._tf_ready_state and now - self._last_ee_diag_ts >= self._ee_warn_period:
                helper = get_tf_helper()
                frames = helper.list_frames() if helper else set()
                candidates = [
                    f for f in sorted(frames)
                    if any(k in f.lower() for k in ("tool", "tcp", "ee", "flange", "wrist", "rg2", "hand", "ft"))
                ]
                sample = ", ".join(candidates[:8]) if candidates else "-"
                self._log(f"[TF] TF OK pero no hay EE transformable desde base_link. Candidatos={sample}. Bloqueando PICK.")
                self._last_ee_diag_ts = now

            self._check_tcp_source_mismatch(now)

        self._set_trace_row(0, object_world_data, object_base_data, world_frame, base_frame)
        self._set_trace_row(1, tcp_world_data, tcp_base_data, world_frame, base_frame)
        self.trace_table.resizeRowsToContents()

        base_error = self._compute_error(object_base_data, tcp_base_data)
        world_error = self._compute_error(object_world_data, tcp_world_data)
        if self.lbl_trace_error_base:
            self.lbl_trace_error_base.setText(self._format_error_text(base_error))
        if self.lbl_trace_error_world:
            self.lbl_trace_error_world.setText(self._format_error_text(world_error))

        tf_transform = self._last_tf_status.get("transform") if self._last_tf_status else None
        if tf_transform and self.lbl_trace_tf_translation and self.lbl_trace_tf_yaw:
            t = tf_transform.transform.translation
            translation_text = f"{t.x:.3f}, {t.y:.3f}, {t.z:.3f}"
            yaw_deg = math.degrees(yaw_from_quaternion(tf_transform.transform.rotation))
            self.lbl_trace_tf_translation.setText(translation_text)
            self.lbl_trace_tf_yaw.setText(f"{yaw_deg:.2f}°")
        else:
            if self.lbl_trace_tf_translation:
                self.lbl_trace_tf_translation.setText("--")
            if self.lbl_trace_tf_yaw:
                self.lbl_trace_tf_yaw.setText("--")

        self._trace_cached_text = self._build_trace_text(
            world_frame,
            base_frame,
            ee_frame,
            object_world_data,
            object_base_data,
            tcp_world_data,
            tcp_base_data,
            base_error,
            world_error,
            tf_transform,
            object_source=object_source,
            tcp_source=tcp_source,
        )
        self._maybe_log_trace(now)

    def _log_trace_transform_warning(self, message: str) -> None:
        now = time.monotonic()
        key = message.split(":", 1)[0]
        last = self._trace_transform_warn_last.get(key, 0.0)
        self._trace_transform_warn_count[key] = self._trace_transform_warn_count.get(key, 0) + 1
        if (now - last) < self._trace_transform_warn_period:
            return
        count = self._trace_transform_warn_count.get(key, 0)
        self._trace_transform_warn_count[key] = 0
        self._trace_transform_warn_last[key] = now
        self._emit_log(f"[TRACE][WARN] {message} ({count})")

    def _maybe_log_tf_not_ready(self):
        if self._tf_not_ready_logged:
            return
        now = time.monotonic()
        if self._bridge_start_ts and (time.time() - self._bridge_start_ts) < TF_INIT_GRACE_SEC:
            return
        if now - self._tf_ready_last_notice >= 1.0:
            self._log("[TRACE] TF not ready yet (waiting for transforms)")
            self._tf_ready_last_notice = now
            self._tf_not_ready_logged = True

    def _maybe_log_trace(self, now: float):
        if not self._trace_ready:
            return
        if "TF world→base: n/a" in (self._trace_cached_text or ""):
            return
        if now - self._last_trace_print_ts >= self._trace_print_period:
            dt = now - self._last_trace_print_ts
            if not self._trace_debug_logged:
                self._log_trace(
                    f"[TRACE][DEBUG] mono_now={now:.3f} mono_last={self._last_trace_print_ts:.3f} mono_dt={dt:.3f}"
                )
                self._trace_debug_logged = True
            self._last_trace_print_ts = now
            self._log_trace("[TRACE] " + self._trace_cached_text.replace("\n", " | "))

    def _reset_trace_throttle(self, reason: str):
        now = time.monotonic()
        self._last_trace_print_ts = now - self._trace_print_period
        self._last_ee_warn_ts = now - self._ee_warn_period
        self._trace_debug_logged = False
        self._tf_not_ready_logged = False
        self._log_trace(f"[TRACE] throttle reset ({reason})")

    def _run_trace_diag_once(self):
        if self._trace_diag_inflight:
            return
        self._trace_diag_inflight = True
        self._run_async(self._trace_diag_worker, name="trace_diag")

    def _tf_sanity_check(self) -> Tuple[bool, str]:
        """FASE 1: TF sanity check usando solo base_link (marco global efectivo)."""
        helper = get_tf_helper()
        if helper is None:
            return False, "tf_helper_off"
        base_frame = self._business_base_frame()
        ee_frame = str(getattr(self, "_required_ee_frame", "") or "rg2_tcp").strip() or "rg2_tcp"
        # FASE 1: timeout aumentado a 0.5s (antes 0.2s) para evitar falsos negativos
        if not _can_transform_between(helper, base_frame, ee_frame, timeout_sec=0.5):
            return False, f"{base_frame}<->{ee_frame} missing"
        tf_be = helper.lookup_transform(base_frame, ee_frame, timeout_sec=0.5)
        if tf_be is None:
            return False, f"{base_frame}->{ee_frame} lookup_failed"
        stamp_txt = "n/a"
        age_txt = "n/a"
        try:
            tf_stamp_ns = int(tf_be.header.stamp.sec) * 1_000_000_000 + int(tf_be.header.stamp.nanosec)
            stamp_txt = f"{int(tf_be.header.stamp.sec)}.{int(tf_be.header.stamp.nanosec):09d}"
            ros_now_ns = 0
            if self._ros_worker_started and self.ros_worker.node_ready():
                with self.ros_worker._lock:
                    ros_now_ns = int(getattr(self.ros_worker, "_last_clock_stamp_ns", 0) or 0)
            if ros_now_ns > 0 and tf_stamp_ns > 0:
                tf_age = (ros_now_ns - tf_stamp_ns) / 1_000_000_000.0
                age_txt = f"{tf_age:.3f}s"
        except Exception:
            pass
        # FASE 1: Eliminar check de world (no es necesario, solo causa spam TF)
        # Operamos únicamente en base_link como GLOBAL_FRAME_EFFECTIVE
        return True, f"{base_frame}->{ee_frame} ok stamp={stamp_txt} age={age_txt}"

    def _run_self_check_once(self) -> None:
        self._run_async(self._self_check_worker, name="self_check")

    def _self_check_worker(self) -> None:
        tf_ok, tf_reason = self._tf_sanity_check()
        camera_ok, camera_reason = camera_ready_status(self)
        controllers_ok, controllers_reason = self._controllers_ready()
        now = time.time()
        rgb_age = now - self._last_camera_frame_ts if self._last_camera_frame_ts else float("inf")
        depth_required, depth_topic = self._camera_depth_expectation()
        depth_age = now - self._last_camera_depth_frame_ts if self._last_camera_depth_frame_ts else float("inf")
        camera_topic = self.camera_topic_combo.currentText().strip() if hasattr(self, "camera_topic_combo") else self.camera_topic
        hz_rgb = 0.0
        if self._camera_subscribe_ts > 0.0 and self._camera_frame_count > 0:
            hz_rgb = float(self._camera_frame_count) / max(1e-3, now - self._camera_subscribe_ts)
        hz_depth = 0.0
        if self._camera_subscribe_ts > 0.0 and self._camera_depth_frame_count > 0:
            hz_depth = float(self._camera_depth_frame_count) / max(1e-3, now - self._camera_subscribe_ts)
        self._emit_log(
            f"[SELF_CHECK] TF={'OK' if tf_ok else 'FAIL'} reason={tf_reason}"
        )
        self._emit_log(
            f"[SELF_CHECK] CAMERA={'OK' if camera_ok else 'FAIL'} reason={camera_reason or self._camera_not_ready_reason()} "
            f"topic={camera_topic or 'n/a'} hz={hz_rgb:.2f} "
            f"last_age={'inf' if math.isinf(rgb_age) else f'{rgb_age:.2f}s'} "
            f"depth_required={str(depth_required).lower()} depth_topic={depth_topic or 'n/a'} "
            f"depth_hz={hz_depth:.2f} depth_age={'inf' if math.isinf(depth_age) else f'{depth_age:.2f}s'}"
        )
        self._emit_log(
            f"[SELF_CHECK] CONTROLLERS={'OK' if controllers_ok else 'FAIL'} reason={controllers_reason}"
        )
        if tf_ok and camera_ok and controllers_ok:
            self._ui_set_status("Self-check OK: TF + Camera + Controllers", error=False)
        else:
            self._ui_set_status(
                f"Self-check FAIL: TF={tf_reason} Camera={camera_reason or self._camera_not_ready_reason()} Controllers={controllers_reason}",
                error=False,
            )

    def _trace_diag_worker(self) -> None:
        topic_names: Set[str] = set()
        try:
            if self.ros_worker:
                topic_names = set(self.ros_worker.list_topic_names())
            if not topic_names and ROS_AVAILABLE:
                helper = get_tf_helper()
                if helper:
                    try:
                        topic_names = {name for name, _ in helper.topic_names_and_types()}
                    except Exception as exc:
                        _log_exception("trace_diag tf helper topics", exc)
            joint_states_present = "/joint_states" in topic_names
            tf_present = "/tf" in topic_names
            tf_static_present = "/tf_static" in topic_names
            helper = get_tf_helper()
            frames: Set[str] = set()
            if helper:
                for _attempt in range(4):
                    frames = helper.list_frames()
                    if frames:
                        break
                    helper.wait_for_frames(0.5)
                if not frames:
                    yaml_text = helper.frames_yaml() if helper else None
                    if yaml_text:
                        _log_tf_yaml_head_once(yaml_text)
            joint_payload: Optional[dict] = None
            if self.ros_worker:
                joint_payload, _ = self.ros_worker.get_last_joint_state()
            joint_received = joint_payload is not None
            names_len = len(joint_payload.get("name", [])) if joint_payload else 0
            position_len = len(joint_payload.get("position", [])) if joint_payload else 0
            robot_frames = [
                frame for frame in sorted(frames) if any(keyword in frame.lower() for keyword in ROBOT_FRAME_KEYWORDS)
            ]
            sample = ", ".join(robot_frames[:10]) if robot_frames else "-"
            self._log(
                f"[TRACE][DIAG] topics: /joint_states={joint_states_present} /tf={tf_present} /tf_static={tf_static_present}"
            )
            self._log(
                f"[TRACE][DIAG] frames={len(frames)} robot_candidates={len(robot_frames)} sample={sample}"
            )
            self._log(
                f"[TRACE][DIAG] joint_states_msg_received={joint_received} names_len={names_len} position_len={position_len}"
            )
            if helper is None:
                self._log("[TRACE][DIAG] TF helper unavailable for transform checks")
                return

            tf_stats = helper.tf_listener_stats()
            tf_frames_set, tf_static_frames_set = helper.tf_frames_seen()
            combined_robot_frames = [
                frame
                for frame in sorted(tf_frames_set.union(tf_static_frames_set))
                if any(keyword in frame.lower() for keyword in ROBOT_FRAME_KEYWORDS)
            ]
            tf_frames_sample = ", ".join(sorted(tf_frames_set)[:10]) if tf_frames_set else "-"
            tf_static_sample = ", ".join(sorted(tf_static_frames_set)[:10]) if tf_static_frames_set else "-"
            self._log(
                f"[TRACE][DIAG] tf_listener_msgs tf={tf_stats[0]} tf_static={tf_stats[1]}"
            )
            if tf_stats[0] > 0 and tf_stats[1] == 0:
                self._log(
                    "[TRACE][DIAG] tf_static=0 (aceptable si la cadena requerida está en /tf dinámico)"
                )
            self._log(
                f"[TRACE][DIAG] tf_frames_seen_count={len(tf_frames_set)} sample={tf_frames_sample}"
            )
            self._log(
                f"[TRACE][DIAG] tf_static_frames_seen_count={len(tf_static_frames_set)} sample={tf_static_sample}"
            )
            self._log(
                f"[TRACE][DIAG] tf_robot_candidates_seen={len(combined_robot_frames)} sample={', '.join(combined_robot_frames[:10]) or '-'}"
            )
        finally:
            self._trace_diag_inflight = False
        if tf_stats[0] == 0 and not combined_robot_frames:
            self._log(
                "[TRACE][DIAG] No dynamic TF (/tf) received → robot TF missing (check robot_state_publisher / controllers)"
            )

        def diag_transform(label: str, frame_a: str, frame_b: str):
            ok = _can_transform_between(helper, frame_a, frame_b, timeout_sec=0.1)
            self._log(f"[TRACE][DIAG] {label} {frame_a}<->{frame_b} ok={ok}")
            return ok

        base_frame = "base_link"
        diag_transform("base->base_link", base_frame, "base_link")

        def gather_candidates(substring_predicate):
            return [frame for frame in sorted(frames) if substring_predicate(frame.lower())]

        tool_like = gather_candidates(lambda text: text.endswith("tool0") or "tool0" in text)
        ee_like = gather_candidates(lambda text: "ee" in text and "link" in text)
        wrist_like = gather_candidates(lambda text: "wrist_3" in text)
        self._log(f"[TRACE][DIAG] tool_like={tool_like[:10]}")
        self._log(f"[TRACE][DIAG] ee_like={ee_like[:10]}")
        self._log(f"[TRACE][DIAG] wrist_like={wrist_like[:10]}")

        def diag_candidates(label, candidate_list):
            chosen = None
            for candidate in candidate_list[:3]:
                ok = diag_transform(label, "base_link", candidate)
                if ok and chosen is None:
                    chosen = candidate
            return chosen

        tool_candidate = diag_candidates("base_link-tool", tool_like)
        ee_candidate = diag_candidates("base_link-ee", ee_like)
        wrist_candidate = diag_candidates("base_link-wrist", wrist_like)
        recommended_ee = ee_candidate or tool_candidate or wrist_candidate
        if recommended_ee:
            self._log(f"[TRACE][DIAG] recommended EE candidate: {recommended_ee}")
        else:
            self._log("[TRACE][DIAG] no EE candidate transformable from base_link")

    def _try_mark_tf_ready(self):
        self._tf_monitor.try_mark_ready()

    def _start_tf_ready_timer(self):
        self._tf_monitor.start()

    def _wait_for_tf_ready(self, world_frame: str, helper: Optional["TfHelper"]) -> Optional[str]:
        return self._tf_monitor.wait_for_ready(world_frame, helper)

    def _tf_world_base_valid(self, helper: "TfHelper", base_frame: str, world_frame: str) -> bool:
        return tf_world_base_valid(self, helper, base_frame, world_frame)

    def _stop_tf_ready_timer(self):
        self._tf_monitor.stop()

    def _log_tf_chain_once(self, world_frame: str, base_frame: str, ee_frame: Optional[str]) -> None:
        if self._tf_chain_logged:
            return
        tf_wb, reason_wb = tf_get_transform(
            world_frame,
            base_frame,
            timeout=0.20,
            logger=self._log_trace,
        )
        if tf_wb is not None:
            s = tf_wb.header.stamp
            kind = "static" if (int(getattr(s, "sec", 0)) == 0 and int(getattr(s, "nanosec", 0)) == 0) else "dynamic"
            self._log_trace(
                "[TRACE][TF_CHAIN] "
                f"world->{base_frame} trans=({float(tf_wb.transform.translation.x):.3f},"
                f"{float(tf_wb.transform.translation.y):.3f},{float(tf_wb.transform.translation.z):.3f}) "
                f"stamp={int(getattr(s, 'sec', 0))}.{int(getattr(s, 'nanosec', 0)):09d} kind={kind}"
            )
        else:
            self._log_trace(f"[TRACE][TF_CHAIN] world->{base_frame} unavailable reason={reason_wb}")

        ee = str(ee_frame or "rg2_tcp").strip() or "rg2_tcp"
        tf_be, reason_be = tf_get_transform(
            base_frame,
            ee,
            timeout=0.20,
            logger=self._log_trace,
        )
        if tf_be is not None:
            s = tf_be.header.stamp
            kind = "static" if (int(getattr(s, "sec", 0)) == 0 and int(getattr(s, "nanosec", 0)) == 0) else "dynamic"
            self._log_trace(
                "[TRACE][TF_CHAIN] "
                f"{base_frame}->{ee} trans=({float(tf_be.transform.translation.x):.3f},"
                f"{float(tf_be.transform.translation.y):.3f},{float(tf_be.transform.translation.z):.3f}) "
                f"stamp={int(getattr(s, 'sec', 0))}.{int(getattr(s, 'nanosec', 0)):09d} kind={kind}"
            )
        else:
            self._log_trace(f"[TRACE][TF_CHAIN] {base_frame}->{ee} unavailable reason={reason_be}")

        self._tf_chain_logged = True

    def _check_tcp_source_mismatch(self, now_mono: float) -> None:
        if self._last_trace_tcp_base is None or self._last_debug_tcp_base is None:
            return
        if abs(float(self._last_trace_tcp_ts) - float(self._last_debug_tcp_ts)) > 2.0:
            return
        tx, ty, tz = self._last_trace_tcp_base
        dx, dy, dz = self._last_debug_tcp_base
        ddx = float(tx) - float(dx)
        ddy = float(ty) - float(dy)
        ddz = float(tz) - float(dz)
        dist = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
        if dist > 0.02 and (now_mono - float(self._last_tcp_mismatch_warn_ts or 0.0)) >= 1.0:
            self._last_tcp_mismatch_warn_ts = now_mono
            self._emit_log(
                "[WARNING] TCP SOURCE MISMATCH (frame/transform inversion suspected) "
                f"delta={dist:.3f}m trace_tcp=({float(tx):.3f},{float(ty):.3f},{float(tz):.3f}) "
                f"debug_tcp=({float(dx):.3f},{float(dy):.3f},{float(dz):.3f})"
            )

    def _build_trace_text(
        self,
        world_frame: str,
        base_frame: str,
        ee_frame: Optional[str],
        object_world: Optional[Dict[str, object]],
        object_base: Optional[Dict[str, object]],
        tcp_world: Optional[Dict[str, object]],
        tcp_base: Optional[Dict[str, object]],
        base_error: Optional[Tuple[float, float, float, float]],
        world_error: Optional[Tuple[float, float, float, float]],
        tf_transform: Optional[object],
        *,
        object_source: str,
        tcp_source: str,
    ) -> str:
        lines = [
            f"Frames: base={base_frame} ee={ee_frame or 'n/a'}",
            f"Sources: object_source={object_source} tcp_source={tcp_source}",
            self._format_pose_summary("Object/base", object_base),
            self._format_pose_summary("TCP/base", tcp_base),
            f"Error base (dx,dy,dz,dist): {self._format_error_tuple(base_error)}",
        ]
        if tf_transform:
            t = tf_transform.transform.translation
            yaw_deg = math.degrees(yaw_from_quaternion(tf_transform.transform.rotation))
            lines.append(f"TF world→base translation: ({t.x:.3f},{t.y:.3f},{t.z:.3f})")
            lines.append(f"TF world→base yaw: {yaw_deg:.2f}°")
        else:
            lines.append("TF world→base: n/a")
        return "\n".join(lines)

    def _set_trace_row(
        self,
        row: int,
        world_data: Optional[Dict[str, object]],
        base_data: Optional[Dict[str, object]],
        world_frame: str,
        base_frame: str,
    ):
        if not self.trace_table:
            return
        frame_text = f"{world_frame}\n{base_frame}"
        self._set_trace_item(row, 0, frame_text)
        axes = ["x", "y", "z"]
        for idx, axis in enumerate(axes, start=1):
            world_val = self._value_from_pose(world_data, axis)
            base_val = self._value_from_pose(base_data, axis)
            self._set_trace_item(row, idx, self._format_dual_value(world_val, base_val))
        orientation_keys = ["qx", "qy", "qz", "qw"]
        for idx, key in enumerate(orientation_keys, start=4):
            world_val = self._value_from_pose(world_data, key)
            base_val = self._value_from_pose(base_data, key)
            self._set_trace_item(row, idx, self._format_dual_value(world_val, base_val))

    def _value_from_pose(self, data: Optional[Dict[str, object]], key: str) -> Optional[float]:
        if not data:
            return None
        if key in ("x", "y", "z"):
            axis = {"x": 0, "y": 1, "z": 2}[key]
            pos = data.get("position")
            if pos:
                return float(pos[axis])
        else:
            orient = data.get("orientation")
            if orient:
                if key == "qx":
                    return float(orient[0])
                if key == "qy":
                    return float(orient[1])
                if key == "qz":
                    return float(orient[2])
                if key == "qw":
                    return float(orient[3])
        return None

    def _set_trace_item(self, row: int, col: int, text: str):
        if not self.trace_table:
            return
        item = QTableWidgetItem(text)
        item.setTextAlignment(Qt.AlignCenter)
        item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
        self.trace_table.setItem(row, col, item)

    def _format_dual_value(self, first: Optional[float], second: Optional[float]) -> str:
        def fmt(value: Optional[float]) -> str:
            return f"{value:.3f}" if value is not None else "n/a"
        return f"{fmt(first)}\n{fmt(second)}"

    def _pose_dict(self, position: Tuple[float, float, float], orientation: Tuple[float, float, float, float], frame: str) -> Dict[str, object]:
        return {"frame": frame, "position": position, "orientation": orientation}

    def _compute_error(
        self, source: Optional[Dict[str, object]], target: Optional[Dict[str, object]]
    ) -> Optional[Tuple[float, float, float, float]]:
        if not source or not target:
            return None
        src = source.get("position")
        tgt = target.get("position")
        if not src or not tgt:
            return None
        dx = float(tgt[0]) - float(src[0])
        dy = float(tgt[1]) - float(src[1])
        dz = float(tgt[2]) - float(src[2])
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dx, dy, dz, dist

    def _format_error_text(self, error: Optional[Tuple[float, float, float, float]]) -> str:
        if not error:
            return "n/a"
        dx, dy, dz, dist = error
        return f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} dist={dist:.3f}"

    def _format_error_tuple(self, error: Optional[Tuple[float, float, float, float]]) -> str:
        if not error:
            return "n/a"
        return " ".join(f"{value:.3f}" for value in error)

    def _format_pose_summary(self, label: str, data: Optional[Dict[str, object]]) -> str:
        if not data:
            return f"{label}: n/a"
        pos = data.get("position")
        ori = data.get("orientation")
        if not pos or not ori:
            return f"{label}: n/a"
        return (
            f"{label}: pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
            f"quat=({ori[0]:.3f},{ori[1]:.3f},{ori[2]:.3f},{ori[3]:.3f})"
        )

    def _copy_trace_text(self):
        if not self._trace_cached_text:
            self._set_status("Trace vacío", error=True)
            return
        QApplication.clipboard().setText(self._trace_cached_text)
        self._set_status("Trace copiado al portapapeles")

    def _step_joint(self, idx: int, direction: int):
        slider = self.joint_sliders[idx]
        slider.setValue(slider.value() + direction)

    def _maybe_send_auto(self):
        if self.chk_auto_joints.isChecked():
            self._send_joints()

    def _send_joints(self):
        self._log_button("Send joints")
        if getattr(self, "_pick_moveit_phase_active", False):
            self._set_status("Movimiento manual bloqueado: PICK_OBJ MoveIt en ejecución", error=True)
            self._emit_log(
                "[MANUAL] BLOCKED: movimiento manual durante fase MoveIt de PICK_OBJ"
            )
            return
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("Movimiento manual bloqueado: ERROR_FATAL", error=True)
            self._emit_log_throttled(
                "SAFETY:manual:ERROR_FATAL",
                "[SAFETY] Movimiento manual bloqueado: ERROR_FATAL",
            )
            return
        if not self._require_manual_ready("Movimiento manual"):
            return
        if self._manual_inflight:
            self._manual_pending = True
            self._set_status("Movimiento manual en curso…", error=False)
            return

        def worker():
            self._manual_inflight = True
            try:
                if not self._ros_worker_started:
                    self._ensure_ros_worker_started()
                if not self.ros_worker.node_ready():
                    self._ui_set_status("Nodo ROS no listo", error=True)
                    return
                ok, reason = self._wait_for_controllers_ready(CONTROLLER_READY_TIMEOUT_SEC)
                if not ok:
                    self._ui_set_status(f"Controladores no listos: {reason}", error=True)
                    return

                topic = self._select_traj_topic()
                if self._debug_logs_enabled:
                    self._log(f"[MANUAL] Topic: {topic}")
                positions = [round(p, 4) for p in self._current_joint_positions_rad()]
                tsec = float(self.joint_time.value())
                sec = max(0.0, tsec)
                # Debug: mostrar valores enviados
                if self._debug_logs_enabled:
                    pos_str = ", ".join([f"{p:+.3f}" for p in positions])
                    self._log(f"[MANUAL] Enviando joints: [{pos_str}] (t={sec:.1f}s)")

                pub = self._get_traj_publisher(topic)
                if not pub:
                    self._ui_set_status("Publisher JointTrajectory no disponible", error=True)
                    return
                if getattr(self, "_pick_moveit_phase_active", False):
                    self._ui_set_status(
                        "Movimiento manual bloqueado: PICK_OBJ MoveIt en ejecución",
                        error=True,
                    )
                    self._emit_log(
                        "[MANUAL] BLOCKED: intento de publicar durante fase MoveIt de PICK_OBJ"
                    )
                    return
                stamp_msg = None
                try:
                    stamp_msg = Time().to_msg()
                except Exception as exc:
                    _log_exception("manual joint trajectory stamp", exc)
                self._emit_log("[MANUAL] Executing direct JointTrajectory (MoveIt bypassed)")
                traj = build_joint_trajectory(
                    positions,
                    sec,
                    UR5_JOINT_NAMES,
                    stamp_msg=stamp_msg,
                )
                if self._traj_publish_inflight:
                    self._emit_log("[MANUAL] WARN: publish JointTrajectory solapado")
                self._traj_publish_inflight = True
                try:
                    pub.publish(traj)
                    self._ui_set_status(f"Trayectoria enviada a {topic}")
                finally:
                    self._traj_publish_inflight = False
                if self._debug_logs_enabled:
                    self._log("[MANUAL] ✓ Publicación JointTrajectory")
            finally:
                self._manual_inflight = False
                if self._manual_pending:
                    self._manual_pending = False
                    # Emitir señal thread-safe en lugar de QTimer.singleShot()
                    self.retry_send_joints.emit()

        self._run_async(worker)

    def _send_joints_retry(self):
        """Retry de _send_joints después de 200ms (thread-safe desde worker)."""
        if self._system_state == SystemState.ERROR_FATAL:
            self._set_status("Movimiento manual bloqueado: ERROR_FATAL", error=True)
            self._emit_log_throttled(
                "SAFETY:manual:ERROR_FATAL",
                "[SAFETY] Movimiento manual bloqueado: ERROR_FATAL",
            )
            return
        QTimer.singleShot(200, self._send_joints)

    def closeEvent(self, event):
        if self._shutdown_complete:
            event.accept()
            return
        self._closing = True
        self._emit_log("[TRACE] Shutdown: begin")
        self._bridge_running = False
        self._gz_running = False
        self._log("[TRACE] Shutdown: stopping timers")
        for timer in (
            self._trace_timer,
            self._tf_ready_timer,
            self._pose_debug_timer,
            self._pose_info_timer,
            getattr(self, "_watchdog_timer", None),
            getattr(self, "objects_timer", None),
            getattr(self, "joint_timer", None),
        ):
            if timer:
                timer.stop()
        self._trace_ready = False
        self._reset_trace_throttle("panel close")
        self._log("[TRACE] Shutdown: stopping RosWorker")
        self.ros_worker.stop_and_join()
        self._log("[TRACE] Shutdown: shutting down TF helper")
        shutdown_tf_helper()
        self._log("[TRACE] Shutdown: TF helper stopped")
        self._kill_proc(self.bag_proc, "ros2 bag record")
        # FASE 8: Stop MoveIt bridge and move_group BEFORE killing the Gazebo
        # bridge and simulators so in-flight plans drain cleanly.
        self._log("[TRACE] Shutdown: stopping MoveIt bridge")
        self._kill_proc(self.moveit_bridge_proc, "ur5_moveit_bridge")
        self.moveit_bridge_proc = None
        self._log("[TRACE] Shutdown: stopping move_group")
        terminate_process(self.moveit_proc, "move_group", log_fn=self._log, timeout_sec=5.0)
        self.moveit_proc = None
        self._kill_proc(self.bridge_proc, "parameter_bridge")
        self._kill_proc(self.gz_pose_proc, "gz_pose_bridge")
        self._kill_proc(self.release_service_proc, "release_objects_service")
        self._kill_proc(self.world_tf_proc, "world_tf_publisher")
        self._kill_proc(self.rsp_proc, "robot_state_publisher")
        self._kill_proc(self.gz_proc, "gz sim")
        self.bag_proc = None
        self.bridge_proc = None
        self.gz_pose_proc = None
        self.release_service_proc = None
        self.world_tf_proc = None
        self.rsp_proc = None
        self.gz_proc = None
        self._force_cleanup_leftovers()
        if self._moveit_node is not None:
            try:
                self._moveit_node.destroy_node()
            except Exception as exc:
                _log_exception("destroy moveit node", exc)
            self._moveit_node = None
            self._moveit_pose_pub = None
        try:
            self._log("[TRACE] Shutdown: calling rclpy.try_shutdown()")
            rclpy.try_shutdown()
        except Exception as exc:
            _log_exception("rclpy.try_shutdown", exc)
        self._emit_log("[TRACE] Shutdown: workers stopped")
        self._emit_log("[TRACE] Shutdown: done")
        self._shutdown_complete = True
        super().closeEvent(event)

    def _force_cleanup_leftovers(self) -> None:
        """Forzar cierre de procesos residuales del stack."""
        patterns = (
            "ros2 bag record",
            "ros_gz_bridge",
            "parameter_bridge",
            "gz sim",
            "gz-sim",
            "gzserver",
            "gzclient",
            "ign gazebo",
            "ros2 launch ur5_bringup",
            "ros2_control_node",
            "robot_state_publisher",
            "world_tf_publisher",
            "release_objects_service",
            "system_state_manager",
            "controller_manager",
            "spawner",
            "move_group",
        )
        for sig in ("-TERM", "-KILL"):
            for pat in patterns:
                try:
                    subprocess.run(["pkill", sig, "-f", pat], check=False)
                except Exception as exc:
                    _log_exception(f"pkill {sig} {pat}", exc)
                    continue
            time.sleep(0.2)
        try:
            res = subprocess.run(
                [
                    "pgrep",
                    "-af",
                    "ros2 bag record|ros_gz_bridge|parameter_bridge|gz sim|gz-sim|gzserver|gzclient|ign gazebo|ros2 launch ur5_bringup|ros2_control_node|robot_state_publisher|world_tf_publisher|controller_manager|spawner|move_group",
                ],
                check=False,
                text=True,
                capture_output=True,
            )
            if res.stdout:
                self._emit_log(f"[WARN] Procesos residuales tras cierre:\n{res.stdout.strip()}")
        except Exception as exc:
            _log_exception("pgrep residual processes", exc)


def _normalize_joint_name(name) -> str:
    text = str(name).strip()
    if "::" in text:
        text = text.split("::")[-1]
    if "/" in text:
        text = text.split("/")[-1]
    return text.strip()


def _rot_to_rpy(rot):
    sy = math.sqrt((rot[0, 0] * rot[0, 0]) + (rot[1, 0] * rot[1, 0]))
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


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
