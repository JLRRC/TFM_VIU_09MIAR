#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_config.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Configuración compartida y constantes globales del panel SUPER PRO."""
import os
import sys
from typing import Dict, List, Optional, Tuple, Set

from ur5_tools.gripper_geometry import RG2_PINCH_CENTER_FRAME, contact_z_correction_for_frame

from .panel_env import env_float
from .panel_settings import PanelSettings
from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params
from .logging_utils import emit_log_line

# Disable FastDDS SHM early to avoid noisy startup errors.
os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")

SETTINGS = PanelSettings.from_env()

WS_DIR = SETTINGS.ws_dir
SCRIPTS_DIR = SETTINGS.scripts_dir
WORLDS_DIR = SETTINGS.worlds_dir
MODELS_DIR = SETTINGS.models_dir
LOG_DIR = SETTINGS.log_dir
BAGS_DIR = SETTINGS.bags_dir
FIG_DIR = SETTINGS.fig_dir
VISION_DIR = SETTINGS.vision_dir
VISION_EXP_DIR = SETTINGS.vision_exp_dir
VISION_PLOTS_DIR = SETTINGS.vision_plots_dir
VISION_SUMMARY = SETTINGS.vision_summary
VISION_FIG_DIR = SETTINGS.vision_fig_dir

TABLE_SIZE_X = SETTINGS.table_size_x
TABLE_SIZE_Y = SETTINGS.table_size_y
TABLE_CENTER_X = SETTINGS.table_center_x
TABLE_CENTER_Y = SETTINGS.table_center_y
TABLE_IMAGE_SWAP_XY = SETTINGS.table_image_swap_xy
TABLE_IMAGE_FLIP_X = SETTINGS.table_image_flip_x
TABLE_IMAGE_FLIP_Y = SETTINGS.table_image_flip_y
TABLE_CALIB_PATH = os.path.join(SCRIPTS_DIR, "table_pixel_map.json")
TABLE_PIXEL_AFFINE: Optional[List[List[float]]] = None
TABLE_PIXEL_RECT: Optional[Dict[str, Tuple[float, float]]] = None
TABLE_PIXEL_HOMOGRAPHY: Optional[List[List[float]]] = None
TABLE_CAM_INFO: Optional[Dict[str, object]] = None
TABLE_OBJECT_XY_MARGIN = SETTINGS.table_object_xy_margin
TABLE_OBJECT_Z_MIN = SETTINGS.table_object_z_min
TABLE_OBJECT_Z_MAX = SETTINGS.table_object_z_max
TABLE_TOP_Z = SETTINGS.reach_overlay_z
TABLE_OBJECT_WHITELIST: Optional[List[str]] = SETTINGS.table_object_whitelist
NUDGE_DROP_OBJECTS = SETTINGS.nudge_drop_objects
NUDGE_DROP_DZ = SETTINGS.nudge_drop_dz
NUDGE_DROP_Z_MIN = SETTINGS.nudge_drop_z_min
SELECTION_SNAP_DIST = SETTINGS.selection_snap_dist
OBJECT_POS_PATH = SETTINGS.object_pos_path
SAVE_POSE_INFO_POSITIONS = SETTINGS.save_pose_info_positions
UR5_BASE_X = SETTINGS.ur5_base_x
UR5_BASE_Y = SETTINGS.ur5_base_y
UR5_BASE_Z = SETTINGS.ur5_base_z
UR5_REACH_RADIUS = SETTINGS.ur5_reach_radius

# Pick demo objects: objetos de demostracion (definidos solo por nombre).
PICK_DEMO_SPAWN_POSE = (-0.42, 0.00, 0.875)
_PICK_DEMO_OBJECTS = {
    "pick_demo": PICK_DEMO_SPAWN_POSE,
}
ATTACHABLE_OBJECTS = ("pick_demo",)
PICK_DEMO_NAME_SET = set(ATTACHABLE_OBJECTS)
# Drop objects: racimo compacto suspendido a ~2 m hasta pulsar "Soltar objetos".
_DROP_AIR_OBJECTS = {
    "box_red": (-0.260, 0.100, 2.000),
    "box_blue": (-0.180, 0.100, 2.000),
    "box_green": (-0.100, 0.100, 2.000),
    "cyl_gray": (-0.020, 0.100, 2.000),
    "cyl_orange": (-0.260, 0.020, 2.000),
    "cyl_purple": (-0.180, 0.020, 2.000),
    "box_lightblue": (-0.100, 0.020, 2.000),
    "cyl_green": (-0.020, 0.020, 2.000),
    "box_yellow": (-0.220, -0.060, 2.000),
    "cross_cyan": (-0.080, -0.060, 2.000),
}
DROP_NAME_SET = {
    "box_blue",
    "box_green",
    "box_lightblue",
    "box_red",
    "box_yellow",
    "cross_cyan",
    "cyl_gray",
    "cyl_green",
    "cyl_orange",
    "cyl_purple",
}
UNKNOWN_NAME_SET: Set[str] = set()

OBJECT_POSITIONS = {**_PICK_DEMO_OBJECTS, **_DROP_AIR_OBJECTS}
DROP_OBJECTS: Dict[str, Tuple[float, float, float]] = {}
PICK_DEMO_OBJECTS: Dict[str, Tuple[float, float, float]] = {}
UNKNOWN_OBJECTS: Dict[str, Tuple[float, float, float]] = {}
DROP_OBJECT_NAMES: List[str] = []
PICK_DEMO_OBJECT_NAMES: List[str] = []
UNKNOWN_OBJECT_NAMES: List[str] = []
DYNAMIC_OBJECTS: Set[str] = set()
_UNKNOWN_WARNED: Set[str] = set()


def refresh_object_groups() -> None:
    """Recompute derived object groups from OBJECT_POSITIONS."""
    DROP_OBJECTS.clear()
    PICK_DEMO_OBJECTS.clear()
    UNKNOWN_OBJECTS.clear()
    UNKNOWN_NAME_SET.clear()
    for name, pos in OBJECT_POSITIONS.items():
        if name in DROP_NAME_SET:
            DROP_OBJECTS[name] = pos
        elif name in PICK_DEMO_NAME_SET:
            PICK_DEMO_OBJECTS[name] = pos
        else:
            UNKNOWN_OBJECTS[name] = pos
            UNKNOWN_NAME_SET.add(name)
            if name not in _UNKNOWN_WARNED:
                _UNKNOWN_WARNED.add(name)
                emit_log_line(f"[OBJECTS][WARN] object_type=UNKNOWN name={name}", stream=sys.stderr)
    DROP_OBJECT_NAMES[:] = list(DROP_OBJECTS.keys())
    PICK_DEMO_OBJECT_NAMES[:] = list(PICK_DEMO_OBJECTS.keys())
    UNKNOWN_OBJECT_NAMES[:] = list(UNKNOWN_OBJECTS.keys())
    DYNAMIC_OBJECTS.clear()
    DYNAMIC_OBJECTS.update(OBJECT_POSITIONS.keys())


refresh_object_groups()
OBJECT_SHAPES = {
    "pick_demo": "circle",
    "box_red": "square",
    "box_blue": "rect_h",
    "box_green": "rect_v",
    "cyl_gray": "circle",
    "cyl_orange": "circle",
    "cyl_purple": "circle",
    "box_lightblue": "square",
    "cyl_green": "circle",
    "box_yellow": "rect_h",
    "cross_cyan": "cross",
}
OBJECT_LABELS = {
    "pick_demo": "DEM",
    "box_red": "ROJ",
    "box_blue": "AZL",
    "box_green": "VER",
    "cyl_gray": "GRI",
    "cyl_orange": "NAR",
    "cyl_purple": "MOR",
    "box_lightblue": "CEL",
    "cyl_green": "CVE",
    "box_yellow": "AMA",
    "cross_cyan": "CRZ",
}
OBJECT_COLORS = {
    "pick_demo": "#f59e0b",
    "box_red": "#d94141",
    "box_blue": "#3b82f6",
    "box_green": "#22c55e",
    "cyl_gray": "#6b7280",
    "cyl_orange": "#f59e0b",
    "cyl_purple": "#a855f7",
    "box_lightblue": "#93c5fd",
    "cyl_green": "#34d399",
    "box_yellow": "#facc15",
    "cross_cyan": "#22d3ee",
}

# F2-followup (auditoría 2026-05-10): valor canónico vive en
# ur5_tools.geometry_constants para que el orchestrator también
# pueda usarlo sin depender del panel. Re-export por compat.
from ur5_tools.geometry_constants import BASKET_DROP_WORLD as BASKET_DROP  # noqa: E402,I100
GZ_WORLD = SETTINGS.gz_world
GRIPPER_ATTACH_PREFIX = SETTINGS.gripper_attach_prefix
DROP_ANCHOR_PREFIX = "/drop_anchor"
GZ_PARTITION_FILE = os.path.join(LOG_DIR, "gz_partition.txt")

INFER_SCRIPT = SETTINGS.infer_script
INFER_CKPT = SETTINGS.infer_ckpt
INFER_ROI_SIZE = SETTINGS.infer_roi_size
INFER_RETRY_ERR_PX = SETTINGS.infer_retry_err_px
FASTRTPS_PROFILES = SETTINGS.fastrtps_profiles
UR5_CONTROLLERS_YAML = SETTINGS.ur5_controllers_yaml
UR5_JOINT_LIMITS_YAML = SETTINGS.ur5_joint_limits_yaml

BRIDGE_BASE_YAML = SETTINGS.bridge_base_yaml
EGL_VENDOR = SETTINGS.egl_vendor
AUTO_START_BRIDGE = SETTINGS.auto_start_bridge
AUTO_START_BRIDGE_DELAY_MS = SETTINGS.auto_start_bridge_delay_ms
AUTO_START_BRIDGE_MAX_RETRIES = SETTINGS.auto_start_bridge_max_retries

DEFAULT_WORLD_CANDIDATES = SETTINGS.default_world_candidates

DEBUG_FRAME_LOG = SETTINGS.debug_frame_log

BASE_FRAME = SETTINGS.base_frame
WORLD_FRAME = SETTINGS.world_frame

# FASE 1: Marco global único para operaciones del panel.
# Todas las operaciones de control/planning se realizan en base_link.
# Esto elimina dependencias de TF world->base_link y previene spam de timeouts.
GLOBAL_FRAME_EFFECTIVE = "base_link"

ARM_TRAJ_TOPIC_DEFAULT = SETTINGS.arm_traj_topic_default
UR5_JOINT_NAMES = SETTINGS.ur5_joint_names
GRIPPER_JOINT_NAMES = SETTINGS.gripper_joint_names
UR5_HOME_ENV = SETTINGS.ur5_home_env
UR5_HOME_DEFAULT = SETTINGS.ur5_home_default
UR5_MODEL_NAME = SETTINGS.ur5_model_name
JOINT_SLIDER_DEG_MIN = SETTINGS.joint_slider_deg_min
JOINT_SLIDER_DEG_MAX = SETTINGS.joint_slider_deg_max
JOINT_SLIDER_SCALE = SETTINGS.joint_slider_scale
DEFAULT_JOINT_MOVE_SEC = SETTINGS.default_joint_move_sec
DEPTH_PCTL_REFRESH_FRAMES = SETTINGS.depth_pctl_refresh_frames
DEPTH_PCTL_STRIDE = SETTINGS.depth_pctl_stride
DEPTH_FAST = SETTINGS.depth_fast
DEBUG_LOGS_TO_STDOUT = SETTINGS.debug_logs_to_stdout
PANEL_GZ_GUI = SETTINGS.panel_gz_gui
DEBUG_JOINTS_TO_STDOUT = SETTINGS.debug_joints_to_stdout
PANEL_JOINT_STATES_TOPIC = SETTINGS.joint_states_topic
PANEL_CAMERA_TOPIC = SETTINGS.camera_topic
USE_SIM_TIME = SETTINGS.use_sim_time
PANEL_SKIP_CLEANUP = SETTINGS.skip_cleanup
PANEL_KILL_STALE = SETTINGS.kill_stale
SELECTION_TIMEOUT_SEC = SETTINGS.selection_timeout_sec
TRAJ_ACTION_FALLBACK = SETTINGS.traj_action_fallback
TRAJ_ACTION_FALLBACK_DELAY_SEC = SETTINGS.traj_action_fallback_delay_sec
TRAJ_ACTION_FALLBACK_EPS_RAD = SETTINGS.traj_action_fallback_eps_rad
TRAJ_ACTION_FALLBACK_TIMEOUT_SEC = SETTINGS.traj_action_fallback_timeout_sec
CONTROLLER_READY_TIMEOUT_SEC = SETTINGS.controller_ready_timeout_sec
CONTROLLER_READY_CACHE_SEC = SETTINGS.controller_ready_cache_sec
MOVEIT_READY_TIMEOUT_SEC = SETTINGS.moveit_ready_timeout_sec
GZ_LAUNCH_TIMEOUT_SEC = SETTINGS.gz_launch_timeout_sec
BRIDGE_LAUNCH_TIMEOUT_SEC = SETTINGS.bridge_launch_timeout_sec
MOVEIT_LAUNCH_TIMEOUT_SEC = SETTINGS.moveit_launch_timeout_sec
MOVEIT_BRIDGE_LAUNCH_TIMEOUT_SEC = SETTINGS.moveit_bridge_launch_timeout_sec
CONTROLLER_DROP_GRACE_SEC = SETTINGS.controller_drop_grace_sec
TRACE_PRINT_PERIOD_SEC = SETTINGS.trace_print_period_sec
DEBUG_POSES_PERIOD_SEC = SETTINGS.debug_poses_period_sec
PICK_LOG_MIN_INTERVAL_SEC = SETTINGS.pick_log_min_interval_sec
GRIPPER_CMD_TOPIC = SETTINGS.gripper_cmd_topic
GRIPPER_OPEN_RAD = SETTINGS.gripper_open_rad
GRIPPER_CLOSED_RAD = SETTINGS.gripper_closed_rad
GRIPPER_JOINT2_SIGN = SETTINGS.gripper_joint2_sign
PANEL_MANAGED = SETTINGS.panel_managed
PANEL_MOVEIT_REQUIRED = SETTINGS.panel_moveit_required
ALLOW_UNSETTLED_ON_TIMEOUT = SETTINGS.allow_unsettled_on_timeout
CAMERA_READY_FRAMES = SETTINGS.camera_ready_frames
CAMERA_INIT_GRACE_SEC = SETTINGS.camera_init_grace_sec
CAMERA_READY_MAX_AGE_SEC = SETTINGS.camera_ready_max_age_sec
CAMERA_REQUIRED = SETTINGS.camera_required
CAMERA_MAX_SIZE = SETTINGS.camera_max_size
CAMERA_USE_BGR = SETTINGS.camera_use_bgr
CAMERA_COPY_FRAME = SETTINGS.camera_copy_frame
CAMERA_DISPLAY_INTERVAL_MS = SETTINGS.camera_display_interval_ms
CAMERA_FAST_SCALE = SETTINGS.camera_fast_scale
OVERLAY_CALIB = SETTINGS.overlay_calib
OVERLAY_SELECTION = SETTINGS.overlay_selection
OVERLAY_REACH = SETTINGS.overlay_reach
CALIBRATION_CAMERA_TOPIC = "/camera_overhead/image"
OVERLAY_ANTIALIAS = SETTINGS.overlay_antialias
CAMERA_SKIP_TFM_INPUT = SETTINGS.camera_skip_tfm_input
CAMERA_PREPROCESS_TFM = SETTINGS.camera_preprocess_tfm
CAMERA_UI_SKIP_HIDDEN = SETTINGS.camera_ui_skip_hidden
CAMERA_INFO_INTERVAL_SEC = SETTINGS.camera_info_interval_sec
CAMERA_TRACK_FPS = SETTINGS.camera_track_fps
STATUS_TOPIC_CACHE_SEC = SETTINGS.status_topic_cache_sec
GZ_SERVICE_CHECK_SEC = SETTINGS.gz_service_check_sec
STALE_PROCESS_GRACE_SEC = SETTINGS.stale_process_grace_sec
TF_INIT_GRACE_SEC = SETTINGS.tf_init_grace_sec
CONTROLLER_START_GRACE_SEC = SETTINGS.controller_start_grace_sec
POSE_INFO_MAX_AGE_SEC = SETTINGS.pose_info_max_age_sec
POSE_INFO_POLL_SEC = SETTINGS.pose_info_poll_sec
POSE_INFO_LOG_PERIOD = SETTINGS.pose_info_log_period
POSE_INFO_ALLOW_STALE = SETTINGS.pose_info_allow_stale
POSE_CLI_ENABLED = SETTINGS.pose_cli_enabled
POSE_CLI_MIN_INTERVAL_SEC = SETTINGS.pose_cli_min_interval_sec
CRITICAL_CLOCK_TIMEOUT_SEC = SETTINGS.critical_clock_timeout_sec
CRITICAL_POSE_TIMEOUT_SEC = SETTINGS.critical_pose_timeout_sec
ATTACH_DIST_M = SETTINGS.attach_dist_m
ATTACH_REL_EPS = SETTINGS.attach_rel_eps
ATTACH_HAND_MOVE_EPS = SETTINGS.attach_hand_move_eps
ATTACH_WINDOW_SEC = SETTINGS.attach_window_sec
ATTACH_SNAP_EPS = SETTINGS.attach_snap_eps
PICK_TF_RETRY_SEC = SETTINGS.pick_tf_retry_sec
PICK_TF_TIMEOUT_SEC = SETTINGS.pick_tf_timeout_sec
FALL_TEST_DELAY_SEC = SETTINGS.fall_test_delay_sec
PICK_DEMO_PRE_GRASP_Z_OFFSET = SETTINGS.pick_demo_pre_grasp_z_offset
PICK_DEMO_GRASP_Z_OFFSET = SETTINGS.pick_demo_grasp_z_offset
PICK_DEMO_TRANSPORT_Z_OFFSET = SETTINGS.pick_demo_transport_z_offset
PICK_DEMO_DROP_Z_OFFSET = SETTINGS.pick_demo_drop_z_offset
# Constante legacy de compatibilidad: el agarre operativo deriva ahora la geometría
# de contacto desde el URDF canónico y `rg2_pinch_center`, así que este valor debe
# permanecer inerte.
GRIPPER_TCP_Z_OFFSET = contact_z_correction_for_frame(RG2_PINCH_CENTER_FRAME)
AUTO_CALIB_FROM_CAMERA = SETTINGS.auto_calib_from_camera
REACH_OVERLAY_Z = SETTINGS.reach_overlay_z
REACH_OVERLAY_POINTS = SETTINGS.reach_overlay_points
CALIB_GRID_STEP = SETTINGS.calib_grid_step
PICKABLE_PRE_GRASP_Z = SETTINGS.pickable_pre_grasp_z
PICKABLE_MIN_CLEARANCE = SETTINGS.pickable_min_clearance

ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data  # pragma: no cover

    ROS_AVAILABLE = True  # pragma: no cover
except Exception as exc:  # pragma: no cover
    rclpy = None  # type: ignore
    qos_profile_sensor_data = None  # type: ignore
    emit_log_line(f"[WARN] ROS 2 / OpenCV no disponible en el panel: {exc}", stream=sys.stderr)

# ── Topic / overlay constants (originally in panel_v2.py) ──────────────────
CAMERA_TOPIC_PREFIX = "/camera"
MOVEIT_POSE_TOPIC = "/desired_grasp"
MOVEIT_CARTESIAN_POSE_TOPIC = "/desired_grasp_cartesian"
# F-iter3 audit (2026-05-10): GLOBAL_FRAME_EFFECTIVE eliminada aquí —
# definida una sola vez arriba (línea 206). Era duplicación silenciosa.
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
TOP_CAMERA_TOPIC_CANDIDATES = ("/camera_debug_top/image",)
WRIST_CAMERA_TOPIC_CANDIDATES = ("/camera_wrist/image",)

# ── Controller polling constants ────────────────────────────────────────────
SETTLE_MANUAL: Set[str] = {"pick_demo"}
CONTROLLER_CHECK_INTERVAL_SEC = 3.0
CONTROLLER_LIST_RETRY_WINDOW_SEC = max(
    0.0, env_float("PANEL_CTRL_LIST_RETRY_WINDOW_SEC", 5.0)
)
CONTROLLER_LAST_OK_GRACE_SEC = max(
    0.0, env_float("PANEL_CTRL_LAST_OK_GRACE_SEC", 5.0)
)
CONTROLLER_LIST_RETRY_STEP_SEC = 0.25
