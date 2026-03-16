#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_settings.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Typed settings loader for the UR5 panel."""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except Exception:
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)))
    except Exception:
        return default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip() not in ("0", "false", "False", "")

def _env_str(name: str, default: str) -> str:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return str(raw)

def _env_optional_bool(name: str) -> Optional[bool]:
    raw = os.environ.get(name)
    if raw is None:
        return None
    return raw.strip().lower() not in ("0", "false", "no", "off", "")

def _load_yaml_overrides(path: str) -> Dict[str, object]:
    if not path:
        return {}
    path = os.path.expandvars(os.path.expanduser(path))
    if not os.path.isfile(path):
        return {}
    try:
        import yaml  # type: ignore
    except Exception:
        print(f"[PANEL][WARN] PyYAML no disponible; ignorando {path}", file=sys.stderr, flush=True)
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:
        print(f"[PANEL][WARN] Error leyendo YAML {path}: {exc}", file=sys.stderr, flush=True)
        return {}
    if isinstance(data, dict) and "panel_settings" in data:
        data = data.get("panel_settings") or {}
    if not isinstance(data, dict):
        return {}
    return {str(k): v for k, v in data.items()}

@dataclass(frozen=True)
class PanelSettings:
    ws_dir: str
    scripts_dir: str
    worlds_dir: str
    models_dir: str
    log_dir: str
    bags_dir: str
    fig_dir: str
    vision_dir: str
    vision_exp_dir: str
    vision_plots_dir: str
    vision_summary: str
    vision_fig_dir: str
    table_size_x: float = 0.768
    table_size_y: float = 0.80
    table_center_x: float = -0.17
    table_center_y: float = 0.0
    table_image_swap_xy: bool = True
    table_image_flip_x: bool = True
    table_image_flip_y: bool = True
    table_object_xy_margin: float = 0.09
    table_object_z_min: float = 0.6
    table_object_z_max: float = 1.55
    table_object_whitelist: Optional[List[str]] = None
    nudge_drop_objects: bool = False
    nudge_drop_dz: float = 0.08
    nudge_drop_z_min: float = 1.6
    selection_snap_dist: float = 0.0
    object_pos_path: str = ""
    save_pose_info_positions: bool = False
    ur5_base_x: float = -0.85
    ur5_base_y: float = 0.0
    ur5_base_z: float = 0.0
    ur5_reach_radius: float = 0.85
    gz_world: str = "ur5_mesa_objetos"
    gripper_attach_prefix: str = "/gripper"
    gz_partition_file: str = ""
    infer_script: str = ""
    infer_ckpt: str = ""
    infer_roi_size: int = 0
    infer_retry_err_px: float = 60.0
    fastrtps_profiles: str = ""
    ur5_controllers_yaml: str = ""
    ur5_joint_limits_yaml: str = ""
    bridge_base_yaml: str = ""
    egl_vendor: str = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    auto_start_bridge: bool = True
    auto_start_bridge_delay_ms: int = 1200
    auto_start_bridge_max_retries: int = 30
    default_world_candidates: List[str] = field(default_factory=list)
    debug_frame_log: bool = False
    base_frame: Optional[str] = None
    world_frame: Optional[str] = None
    arm_traj_topic_default: str = "/joint_trajectory_controller/joint_trajectory"
    ur5_joint_names: List[str] = field(default_factory=list)
    gripper_joint_names: List[str] = field(default_factory=list)
    ur5_home_env: str = ""
    ur5_home_default: List[float] = field(default_factory=list)
    ur5_model_name: str = "ur5_rg2"
    joint_slider_deg_min: float = -180.0
    joint_slider_deg_max: float = 180.0
    joint_slider_scale: float = 10.0
    default_joint_move_sec: float = 2.0
    depth_pctl_refresh_frames: int = 15
    depth_pctl_stride: int = 4
    depth_fast: bool = False
    debug_logs_to_stdout: bool = False
    panel_gz_gui: bool = False
    debug_joints_to_stdout: bool = False
    joint_states_topic: str = "/joint_states"
    camera_topic: str = "/camera_overhead/image"
    use_sim_time: bool = True
    skip_cleanup: bool = False
    kill_stale: bool = True
    selection_timeout_sec: float = 12.0
    traj_action_fallback: bool = True
    traj_action_fallback_delay_sec: float = 1.0
    traj_action_fallback_eps_rad: float = 0.002
    traj_action_fallback_timeout_sec: float = 2.0
    controller_ready_timeout_sec: float = 20.0
    controller_ready_cache_sec: float = 0.0
    moveit_ready_timeout_sec: float = 20.0
    gz_launch_timeout_sec: float = 20.0
    bridge_launch_timeout_sec: float = 12.0
    moveit_launch_timeout_sec: float = 25.0
    moveit_bridge_launch_timeout_sec: float = 15.0
    controller_drop_grace_sec: float = 3.0
    trace_print_period_sec: float = 3.0
    debug_poses_period_sec: float = 3.0
    pick_log_min_interval_sec: float = 2.0
    gripper_cmd_topic: str = "/gripper_controller/commands"
    gripper_open_rad: float = 1.0
    gripper_closed_rad: float = 0.0
    gripper_joint2_sign: float = 1.0
    panel_managed: bool = False
    panel_moveit_required: bool = True
    allow_unsettled_on_timeout: bool = False
    camera_ready_frames: int = 3
    camera_init_grace_sec: float = 4.0
    camera_ready_max_age_sec: float = 2.5
    camera_required: Optional[bool] = None
    camera_max_size: int = 960
    camera_use_bgr: bool = True
    camera_copy_frame: bool = True
    camera_display_interval_ms: int = 80
    camera_fast_scale: bool = True
    overlay_calib: bool = True
    overlay_selection: bool = True
    overlay_reach: bool = True
    overlay_antialias: bool = True
    camera_skip_tfm_input: bool = False
    camera_preprocess_tfm: bool = False
    camera_ui_skip_hidden: bool = True
    camera_info_interval_sec: float = 0.25
    camera_track_fps: bool = True
    status_topic_cache_sec: float = 0.5
    gz_service_check_sec: float = 2.5
    stale_process_grace_sec: float = 20.0
    tf_init_grace_sec: float = 8.0
    controller_start_grace_sec: float = 12.0
    pose_info_max_age_sec: float = 1.0
    pose_info_poll_sec: float = 0.5
    pose_info_log_period: float = 2.0
    pose_info_allow_stale: bool = False
    pose_cli_enabled: bool = True
    pose_cli_min_interval_sec: float = 2.0
    critical_clock_timeout_sec: float = 2.0
    critical_pose_timeout_sec: float = 20.0
    attach_dist_m: float = 0.02
    attach_rel_eps: float = 0.002
    attach_hand_move_eps: float = 0.005
    attach_window_sec: float = 0.6
    attach_snap_eps: float = 0.003
    pick_tf_retry_sec: float = 0.2
    pick_tf_timeout_sec: float = 3.0
    fall_test_delay_sec: float = 1.0
    pick_demo_pre_grasp_z_offset: float = 0.15
    pick_demo_grasp_z_offset: float = 0.02
    pick_demo_transport_z_offset: float = 0.28
    pick_demo_drop_z_offset: float = 0.05
    gripper_tcp_z_offset: float = 0.0
    auto_calib_from_camera: bool = True
    reach_overlay_z: float = 0.850
    reach_overlay_points: int = 72
    calib_grid_step: float = 0.05
    pickable_pre_grasp_z: float = 0.12
    pickable_min_clearance: float = 0.05

    @classmethod
    def from_env(cls) -> "PanelSettings":
        ws_dir = os.path.expanduser(os.environ.get("WS_DIR", "~/TFM/agarre_ros2_ws"))
        os.environ.setdefault("WS_DIR", ws_dir)
        os.environ.setdefault("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib")

        scripts_dir = os.path.join(ws_dir, "scripts")
        worlds_dir = os.path.join(ws_dir, "worlds")
        models_dir = os.path.join(ws_dir, "models")
        log_dir = os.path.join(ws_dir, "log")
        bags_dir = os.path.join(ws_dir, "bags")
        fig_dir = os.path.join(ws_dir, "experiments", "figures_memoria")
        vision_dir = os.path.expanduser(os.environ.get("VISION_DIR", "~/TFM/agarre_inteligente"))
        vision_exp_dir = os.path.join(vision_dir, "experiments")
        vision_plots_dir = os.path.join(vision_exp_dir, "plots")
        vision_summary = os.path.join(vision_exp_dir, "summary_base.csv")
        vision_fig_dir = os.path.join(vision_exp_dir, "figures_memoria")

        settings = cls(
            ws_dir=ws_dir,
            scripts_dir=scripts_dir,
            worlds_dir=worlds_dir,
            models_dir=models_dir,
            log_dir=log_dir,
            bags_dir=bags_dir,
            fig_dir=fig_dir,
            vision_dir=vision_dir,
            vision_exp_dir=vision_exp_dir,
            vision_plots_dir=vision_plots_dir,
            vision_summary=vision_summary,
            vision_fig_dir=vision_fig_dir,
            table_object_xy_margin=_env_float("TABLE_OBJECT_XY_MARGIN", 0.09),
            table_object_z_min=_env_float("TABLE_OBJECT_Z_MIN", 0.6),
            table_object_z_max=_env_float("TABLE_OBJECT_Z_MAX", 1.55),
            nudge_drop_objects=_env_bool("PANEL_NUDGE_DROP_OBJECTS", False),
            nudge_drop_dz=_env_float("PANEL_NUDGE_DROP_DZ", 0.08),
            nudge_drop_z_min=_env_float("PANEL_NUDGE_DROP_Z_MIN", 1.6),
            selection_snap_dist=_env_float("PANEL_SELECTION_SNAP_DIST", 0.0),
            object_pos_path=os.path.join(scripts_dir, "object_positions.json"),
            save_pose_info_positions=_env_bool("PANEL_SAVE_POSE_INFO_POSITIONS", False),
            infer_script=os.path.join(vision_dir, "scripts", "predict.py"),
            infer_ckpt=_env_str(
                "INFER_CKPT",
                os.path.join(
                    vision_dir,
                    "experiments",
                    "EXP1_SIMPLE_RGB",
                    "seed_0",
                    "checkpoints",
                    "best.pth",
                ),
            ),
            infer_roi_size=max(0, int(os.environ.get("INFER_ROI_SIZE", "0"))),
            infer_retry_err_px=_env_float("INFER_RETRY_ERR_PX", 60.0),
            fastrtps_profiles=os.path.join(scripts_dir, "fastdds_no_shm.xml"),
            ur5_controllers_yaml=os.path.join(
                ws_dir,
                "src",
                "ur5_description",
                "config",
                "ur5_controllers.yaml",
            ),
            ur5_joint_limits_yaml=_env_str(
                "PANEL_JOINT_LIMITS_YAML",
                os.path.join(ws_dir, "src", "ur5_moveit_config", "config", "joint_limits.yaml"),
            ),
            bridge_base_yaml=os.path.join(scripts_dir, "bridge_cameras.yaml"),
            auto_start_bridge=_env_bool("PANEL_AUTO_BRIDGE", True),
            auto_start_bridge_delay_ms=_env_int("PANEL_AUTO_BRIDGE_DELAY_MS", 1200),
            auto_start_bridge_max_retries=_env_int("PANEL_AUTO_BRIDGE_MAX_RETRIES", 30),
            default_world_candidates=[os.path.join(worlds_dir, "ur5_mesa_objetos.sdf")],
            debug_frame_log=_env_bool("PANEL_DEBUG_FRAMES", False),
            base_frame=os.environ.get("PANEL_BASE_FRAME"),
            world_frame=os.environ.get("PANEL_WORLD_FRAME"),
            arm_traj_topic_default=os.environ.get(
                "ARM_TRAJ_TOPIC", "/joint_trajectory_controller/joint_trajectory"
            ),
            ur5_joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            gripper_joint_names=[
                "rg2_finger_joint1",
                "rg2_finger_joint2",
            ],
            ur5_home_env=os.path.join(scripts_dir, "ur5_home_pose.env"),
            ur5_home_default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ur5_model_name=os.environ.get("UR5_MODEL_NAME", "ur5_rg2"),
            depth_pctl_refresh_frames=max(1, _env_int("PANEL_DEPTH_PCTL_REFRESH_FRAMES", 15)),
            depth_pctl_stride=max(1, _env_int("PANEL_DEPTH_PCTL_STRIDE", 4)),
            depth_fast=_env_bool("PANEL_DEPTH_FAST", False),
            debug_logs_to_stdout=_env_bool("DEBUG_LOGS_TO_STDOUT", False),
            panel_gz_gui=_env_bool("PANEL_GZ_GUI", False),
            debug_joints_to_stdout=_env_bool("DEBUG_JOINTS_TO_STDOUT", False),
            joint_states_topic=_env_str("PANEL_JOINT_STATES_TOPIC", "/joint_states"),
            camera_topic=_env_str("PANEL_CAMERA_TOPIC", "/camera_overhead/image"),
            use_sim_time=_env_bool("USE_SIM_TIME", True),
            # Por defecto no matar procesos en startup para evitar cortar el stack recién lanzado.
            skip_cleanup=_env_bool("PANEL_SKIP_CLEANUP", True),
            kill_stale=_env_bool("PANEL_KILL_STALE", True),
            selection_timeout_sec=_env_float("PANEL_SELECTION_TIMEOUT_SEC", 12.0),
            traj_action_fallback=_env_bool("PANEL_TRAJ_ACTION_FALLBACK", True),
            traj_action_fallback_delay_sec=_env_float("PANEL_TRAJ_ACTION_FALLBACK_DELAY_SEC", 1.0),
            traj_action_fallback_eps_rad=_env_float("PANEL_TRAJ_ACTION_FALLBACK_EPS_RAD", 0.002),
            traj_action_fallback_timeout_sec=_env_float("PANEL_TRAJ_ACTION_FALLBACK_TIMEOUT_SEC", 2.0),
            controller_ready_timeout_sec=_env_float("PANEL_CONTROLLER_READY_TIMEOUT_SEC", 20.0),
            controller_ready_cache_sec=_env_float("PANEL_CONTROLLER_READY_CACHE_SEC", 0.0),
            moveit_ready_timeout_sec=_env_float("PANEL_MOVEIT_READY_TIMEOUT_SEC", 20.0),
            gz_launch_timeout_sec=_env_float("PANEL_GZ_LAUNCH_TIMEOUT_SEC", 20.0),
            bridge_launch_timeout_sec=_env_float("PANEL_BRIDGE_LAUNCH_TIMEOUT_SEC", 12.0),
            moveit_launch_timeout_sec=_env_float("PANEL_MOVEIT_LAUNCH_TIMEOUT_SEC", 25.0),
            moveit_bridge_launch_timeout_sec=_env_float("PANEL_MOVEIT_BRIDGE_LAUNCH_TIMEOUT_SEC", 15.0),
            controller_drop_grace_sec=_env_float("PANEL_CTRL_DROP_GRACE_SEC", 3.0),
            trace_print_period_sec=_env_float("PANEL_TRACE_PRINT_PERIOD_SEC", 3.0),
            debug_poses_period_sec=_env_float("PANEL_DEBUG_POSES_PERIOD_SEC", 3.0),
            pick_log_min_interval_sec=_env_float("PANEL_PICK_LOG_MIN_INTERVAL_SEC", 2.0),
            gripper_cmd_topic=_env_str("PANEL_GRIPPER_CMD_TOPIC", "/gripper_controller/commands"),
            gripper_open_rad=_env_float("PANEL_GRIPPER_OPEN_RAD", 1.0),
            gripper_closed_rad=_env_float("PANEL_GRIPPER_CLOSED_RAD", 0.0),
            gripper_joint2_sign=_env_float("PANEL_GRIPPER_JOINT2_SIGN", 1.0),
            panel_managed=_env_bool("PANEL_MANAGED", False),
            panel_moveit_required=_env_bool("PANEL_MOVEIT_REQUIRED", True),
            allow_unsettled_on_timeout=_env_bool("PANEL_ALLOW_UNSETTLED_ON_TIMEOUT", False),
            camera_ready_frames=_env_int("PANEL_CAMERA_READY_FRAMES", 3),
            camera_init_grace_sec=_env_float("PANEL_CAMERA_INIT_GRACE_SEC", 4.0),
            camera_ready_max_age_sec=_env_float("PANEL_CAMERA_READY_MAX_AGE_SEC", 2.5),
            camera_required=_env_optional_bool("PANEL_CAMERA_REQUIRED"),
            camera_max_size=max(0, _env_int("PANEL_CAMERA_MAX_SIZE", 960)),
            camera_use_bgr=_env_bool("PANEL_CAMERA_USE_BGR", True),
            camera_copy_frame=_env_bool("PANEL_CAMERA_COPY_FRAME", True),
            camera_display_interval_ms=max(10, _env_int("PANEL_CAMERA_DISPLAY_MS", 80)),
            camera_fast_scale=_env_bool("PANEL_CAMERA_FAST_SCALE", True),
            overlay_calib=_env_bool("PANEL_OVERLAY_CALIB", True),
            overlay_selection=_env_bool("PANEL_OVERLAY_SELECTION", True),
            overlay_reach=_env_bool("PANEL_OVERLAY_REACH", True),
            overlay_antialias=_env_bool("PANEL_OVERLAY_ANTIALIAS", True),
            camera_skip_tfm_input=_env_bool("PANEL_CAMERA_SKIP_TFM_INPUT", False),
            camera_preprocess_tfm=_env_bool("PANEL_CAMERA_PREPROCESS_TFM", False),
            camera_ui_skip_hidden=_env_bool("PANEL_CAMERA_UI_SKIP_HIDDEN", True),
            camera_info_interval_sec=_env_float("PANEL_CAMERA_INFO_INTERVAL_SEC", 0.25),
            camera_track_fps=_env_bool("PANEL_CAMERA_TRACK_FPS", True),
            status_topic_cache_sec=_env_float("PANEL_STATUS_TOPIC_CACHE_SEC", 0.5),
            gz_service_check_sec=_env_float("PANEL_GZ_SERVICE_CHECK_SEC", 2.5),
            stale_process_grace_sec=_env_float("PANEL_STALE_GRACE_SEC", 20.0),
            tf_init_grace_sec=_env_float("PANEL_TF_INIT_GRACE_SEC", 8.0),
            controller_start_grace_sec=_env_float("PANEL_CTRL_START_GRACE_SEC", 12.0),
            pose_info_max_age_sec=_env_float("PANEL_POSE_INFO_MAX_AGE_SEC", 1.0),
            pose_info_poll_sec=_env_float("PANEL_POSE_INFO_POLL_SEC", 0.5),
            pose_info_log_period=_env_float("PANEL_POSE_INFO_LOG_PERIOD", 2.0),
            pose_info_allow_stale=_env_bool("PANEL_POSE_INFO_ALLOW_STALE", False),
            pose_cli_enabled=_env_bool("PANEL_POSE_CLI_ENABLED", True),
            pose_cli_min_interval_sec=_env_float("PANEL_POSE_CLI_MIN_INTERVAL_SEC", 2.0),
            critical_clock_timeout_sec=_env_float("PANEL_CRITICAL_CLOCK_TIMEOUT_SEC", 2.0),
            critical_pose_timeout_sec=_env_float("PANEL_CRITICAL_POSE_TIMEOUT_SEC", 20.0),
            attach_dist_m=_env_float("PANEL_ATTACH_DIST_M", 0.02),
            attach_rel_eps=_env_float("PANEL_ATTACH_REL_EPS", 0.002),
            attach_hand_move_eps=_env_float("PANEL_ATTACH_HAND_MOVE_EPS", 0.005),
            attach_window_sec=_env_float("PANEL_ATTACH_WINDOW_SEC", 0.6),
            attach_snap_eps=_env_float("PANEL_ATTACH_SNAP_EPS", 0.003),
            pick_tf_retry_sec=_env_float("PANEL_PICK_TF_RETRY_SEC", 0.2),
            pick_tf_timeout_sec=_env_float("PANEL_PICK_TF_TIMEOUT_SEC", 3.0),
            fall_test_delay_sec=_env_float("PANEL_FALL_TEST_DELAY_SEC", 1.0),
            pick_demo_pre_grasp_z_offset=_env_float("PANEL_PICK_DEMO_PRE_GRASP_Z", 0.15),
            pick_demo_grasp_z_offset=_env_float("PANEL_PICK_DEMO_GRASP_Z", 0.02),
            pick_demo_transport_z_offset=_env_float("PANEL_PICK_DEMO_TRANSPORT_Z", 0.28),
            pick_demo_drop_z_offset=_env_float("PANEL_PICK_DEMO_DROP_Z", 0.05),
            gripper_tcp_z_offset=_env_float("PANEL_GRIPPER_TCP_Z_OFFSET", 0.0),
            auto_calib_from_camera=_env_bool("PANEL_CALIB_AUTO", True),
            reach_overlay_z=_env_float("PANEL_REACH_OVERLAY_Z", 0.850),
            reach_overlay_points=_env_int("PANEL_REACH_OVERLAY_POINTS", 72),
            calib_grid_step=_env_float("PANEL_CALIB_GRID_STEP", 0.05),
            pickable_pre_grasp_z=_env_float("PANEL_PICKABLE_PRE_GRASP_Z", 0.12),
            pickable_min_clearance=_env_float("PANEL_PICKABLE_MIN_CLEARANCE", 0.05),
        )
        overrides = _load_yaml_overrides(os.environ.get("PANEL_SETTINGS_YAML", ""))
        if not overrides:
            return settings
        data = settings.__dict__.copy()
        for key, value in overrides.items():
            if key not in data:
                print(f"[PANEL][WARN] Clave desconocida en YAML: {key}", file=sys.stderr, flush=True)
                continue
            data[key] = value
        return cls(**data)
