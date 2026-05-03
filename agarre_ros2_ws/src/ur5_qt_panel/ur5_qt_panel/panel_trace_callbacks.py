#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_trace_callbacks.py
# Contenido: Trace/diagnostic callbacks extracted from ControlPanelV2 in panel_v2.py.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Trace and diagnostic callbacks for ControlPanelV2."""
from __future__ import annotations

import math
import os
import time
from typing import Dict, Optional, Tuple

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QTableWidgetItem

from .panel_config import (
    ROS_AVAILABLE,
    TF_INIT_GRACE_SEC,
)
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME
from .panel_utils import (
    discover_base_and_ee_frames,
    get_object_positions,
    yaw_from_quaternion,
    ROBOT_FRAME_KEYWORDS,
)
from .panel_readiness import camera_ready_status
from .panel_tf import get_tf_helper
from .panel_utils import tf_world_base_valid
from typing import Set
from .panel_camera import _runtime_time
from .panel_utils import _can_transform_between, _log_tf_yaml_head_once
from .panel_tf import TfHelper
from .logging_utils import emit_log_line
from .tf_pose_utils import (
    get_tcp_in_base as tf_get_tcp_in_base,
    get_transform as tf_get_transform,
    transform_pose as tf_transform_pose,
    world_pose_to_base as tf_world_pose_to_base,
)

try:
    from geometry_msgs.msg import PoseStamped
except Exception:
    PoseStamped = None  # type: ignore


def _log_exception(context: str, exc: Exception) -> None:
    emit_log_line(f"[TRACE_CB][ERROR][{context}] {exc}")


def _start_trace_timer(panel):
    if panel._trace_timer:
        return
    panel._trace_timer = QTimer(panel)
    panel._trace_timer.setInterval(200)
    panel._trace_timer.timeout.connect(panel._refresh_trace_data)
    panel._trace_timer.start()
    panel._refresh_trace_data()

def _resolve_trace_frames(panel, world_frame: str) -> Tuple[str, Optional[str]]:
    _detected_base, detected_ee = discover_base_and_ee_frames(world_frame)
    effective_base = "base_link"
    panel._base_frame_effective = "base_link"
    if detected_ee and detected_ee != panel._ee_frame_effective:
        panel._ee_frame_effective = detected_ee
    effective_ee = detected_ee or panel._ee_frame_effective
    if not effective_ee:
        helper = get_tf_helper()
        for candidate in ("rg2_tcp", "rg2_pinch_center", "tool0", "flange"):
            if helper and _can_transform_between(helper, effective_base, candidate, timeout_sec=0.05):
                effective_ee = candidate
                panel._ee_frame_effective = candidate
                panel._emit_log(f"[TF] EE resolved via {candidate}")
                break
    return effective_base, effective_ee

def _refresh_trace_data_resolve_tcp_pose(
    panel,
    *,
    ee_frame,
    base_frame: str,
    world_frame: str,
    now: float,
):
    """F3-step30c: resuelve TCP pose en base + world (~165 LOC).

    Branch 1: ee_frame disponible → tf_get_tcp_in_base + opcional transform a
    world. Actualiza tcp_live_xyz_lbl/rpy_lbl + panel._last_trace_tcp_*.
    Branch 2: ee_frame no disponible → log warn + diagnostic + reset labels.
    También calcula legacy_tcp_base/world_data (rg2_tcp frame separado).

    Devuelve dict con todos los pose_data + tcp_source.
    """
    tcp_world_data = None
    tcp_base_data = None
    tcp_source = "tf2:UNAVAILABLE"
    legacy_tcp_base_data = None
    pinch_tcp_base_data = None
    legacy_tcp_world_data = None
    pinch_tcp_world_data = None
    if ee_frame:
        tcp_pose_base, _tcp_rpy, tcp_reason = tf_get_tcp_in_base(
            base_frame=base_frame,
            ee_frame=ee_frame,
            timeout=0.08,
            logger=panel._log_trace,
        )
        if tcp_pose_base is not None:
            tcp_base_data = panel._pose_dict(
                (float(tcp_pose_base.pose.position.x), float(tcp_pose_base.pose.position.y), float(tcp_pose_base.pose.position.z)),
                (float(tcp_pose_base.pose.orientation.x), float(tcp_pose_base.pose.orientation.y), float(tcp_pose_base.pose.orientation.z), float(tcp_pose_base.pose.orientation.w)),
                base_frame,
            )
            panel._last_trace_tcp_base = (
                float(tcp_pose_base.pose.position.x),
                float(tcp_pose_base.pose.position.y),
                float(tcp_pose_base.pose.position.z),
            )
            panel._last_trace_tcp_ts = time.monotonic()
            try:
                _hdr = tcp_pose_base.header
                panel._last_trace_tcp_tf_stamp_ns = (
                    int(_hdr.stamp.sec) * 1_000_000_000 + int(_hdr.stamp.nanosec)
                )
            except Exception:
                panel._last_trace_tcp_tf_stamp_ns = 0
            panel._last_tcp_base_z = float(tcp_pose_base.pose.position.z)
            panel._last_trace_tcp_rpy_deg = (
                float(_tcp_rpy[0]),
                float(_tcp_rpy[1]),
                float(_tcp_rpy[2]),
            ) if _tcp_rpy is not None and len(_tcp_rpy) >= 3 else None
            if panel.tcp_live_xyz_lbl is not None:
                panel.tcp_live_xyz_lbl.setText(
                    f"{float(tcp_pose_base.pose.position.x):.3f}, "
                    f"{float(tcp_pose_base.pose.position.y):.3f}, "
                    f"{float(tcp_pose_base.pose.position.z):.3f}"
                )
            if panel.tcp_live_rpy_lbl is not None:
                if panel._last_trace_tcp_rpy_deg is not None:
                    panel.tcp_live_rpy_lbl.setText(
                        f"{float(panel._last_trace_tcp_rpy_deg[0]):.1f}, "
                        f"{float(panel._last_trace_tcp_rpy_deg[1]):.1f}, "
                        f"{float(panel._last_trace_tcp_rpy_deg[2]):.1f}"
                    )
                else:
                    panel.tcp_live_rpy_lbl.setText("--")
            pinch_tcp_base_data = tcp_base_data
            tcp_source = "tf2:ok"
            helper = get_tf_helper()
            world_tf_available = bool(
                helper
                and _can_transform_between(helper, base_frame, world_frame, timeout_sec=0.05)
            )
            if world_tf_available:
                tcp_world_pose, world_reason = tf_transform_pose(
                    tcp_pose_base,
                    target_frame=world_frame,
                    timeout=0.08,
                )
                if tcp_world_pose is not None:
                    tcp_world_data = panel._pose_dict(
                        (float(tcp_world_pose.pose.position.x), float(tcp_world_pose.pose.position.y), float(tcp_world_pose.pose.position.z)),
                        (float(tcp_world_pose.pose.orientation.x), float(tcp_world_pose.pose.orientation.y), float(tcp_world_pose.pose.orientation.z), float(tcp_world_pose.pose.orientation.w)),
                        world_frame,
                    )
                    panel._last_tcp_world_tf = tcp_world_data
                    pinch_tcp_world_data = tcp_world_data
        else:
            panel._last_trace_tcp_base = None
            panel._last_trace_tcp_ts = time.monotonic()
            panel._last_trace_tcp_rpy_deg = None
            if panel.tcp_live_xyz_lbl is not None:
                panel.tcp_live_xyz_lbl.setText("--")
            if panel.tcp_live_rpy_lbl is not None:
                panel.tcp_live_rpy_lbl.setText("--")
            tcp_source = f"tf2:base_lookup_failed:{tcp_reason}"
        legacy_pose_base, _legacy_rpy, _legacy_reason = tf_get_tcp_in_base(
            base_frame=base_frame,
            ee_frame="rg2_tcp",
            timeout=0.05,
            logger=None,
        )
        if legacy_pose_base is not None:
            legacy_tcp_base_data = panel._pose_dict(
                (float(legacy_pose_base.pose.position.x), float(legacy_pose_base.pose.position.y), float(legacy_pose_base.pose.position.z)),
                (float(legacy_pose_base.pose.orientation.x), float(legacy_pose_base.pose.orientation.y), float(legacy_pose_base.pose.orientation.z), float(legacy_pose_base.pose.orientation.w)),
                base_frame,
            )
            helper = get_tf_helper()
            if helper and _can_transform_between(helper, base_frame, world_frame, timeout_sec=0.05):
                legacy_world_pose, _legacy_world_reason = tf_transform_pose(
                    legacy_pose_base,
                    target_frame=world_frame,
                    timeout=0.08,
                )
                if legacy_world_pose is not None:
                    legacy_tcp_world_data = panel._pose_dict(
                        (float(legacy_world_pose.pose.position.x), float(legacy_world_pose.pose.position.y), float(legacy_world_pose.pose.position.z)),
                        (float(legacy_world_pose.pose.orientation.x), float(legacy_world_pose.pose.orientation.y), float(legacy_world_pose.pose.orientation.z), float(legacy_world_pose.pose.orientation.w)),
                        world_frame,
                    )
    else:
        if now - panel._last_ee_warn_ts >= panel._ee_warn_period:
            panel._log("[TRACE] EE frame unavailable (retrying)")
            panel._last_ee_warn_ts = now
        if panel._tf_ready_state and now - panel._last_ee_diag_ts >= panel._ee_warn_period:
            helper = get_tf_helper()
            frames = helper.list_frames() if helper else set()
            candidates = [
                f for f in sorted(frames)
                if any(k in f.lower() for k in ("tool", "tcp", "ee", "flange", "wrist", "rg2", "hand", "ft"))
            ]
            sample = ", ".join(candidates[:8]) if candidates else "-"
            panel._log(f"[TF] TF OK pero no hay EE transformable desde base_link. Candidatos={sample}. Bloqueando PICK.")
            panel._last_ee_diag_ts = now
        panel._last_trace_tcp_rpy_deg = None
        if panel.tcp_live_xyz_lbl is not None:
            panel.tcp_live_xyz_lbl.setText("--")
        if panel.tcp_live_rpy_lbl is not None:
            panel.tcp_live_rpy_lbl.setText("--")
    return {
        "tcp_world_data": tcp_world_data,
        "tcp_base_data": tcp_base_data,
        "tcp_source": tcp_source,
        "legacy_tcp_base_data": legacy_tcp_base_data,
        "pinch_tcp_base_data": pinch_tcp_base_data,
        "legacy_tcp_world_data": legacy_tcp_world_data,
        "pinch_tcp_world_data": pinch_tcp_world_data,
    }


def _refresh_trace_data_emit_audits(
    panel,
    *,
    now: float,
    world_frame: str,
    base_frame: str,
    ee_frame,
    object_name: str,
    object_source: str,
    tcp_source: str,
    object_world_data,
    object_base_data,
    tcp_world_data,
    tcp_base_data,
    legacy_tcp_base_data,
    pinch_tcp_base_data,
    legacy_tcp_world_data,
    pinch_tcp_world_data,
) -> None:
    """F3-step30a: emite logs PANEL_TRACE + RG2 AUDIT TCP/COMPARE (~80 LOC).

    Cada 1s emite [PICK][DIRECT][PANEL_TRACE] + [RG2][AUDIT][TCP] +
    [RG2][AUDIT][COMPARE] con resúmenes y deltas de distancia.
    """
    if (now - float(panel._last_panel_trace_audit_ts or 0.0)) < 1.0:
        return
    panel._last_panel_trace_audit_ts = now
    fk_tcp = panel._last_tcp_base
    live_tcp = panel._last_trace_tcp_base

    def _fmt_tuple(vec):
        if not isinstance(vec, (list, tuple)) or len(vec) < 3:
            return "--"
        try:
            return f"({float(vec[0]):.3f},{float(vec[1]):.3f},{float(vec[2]):.3f})"
        except Exception:
            return "--"

    def _fmt_scalar(value):
        if value is None:
            return "--"
        try:
            return f"{float(value):.3f}"
        except Exception:
            return "--"

    delta_txt = "--"
    dist_txt = "--"
    if fk_tcp is not None and live_tcp is not None:
        ddx = float(fk_tcp[0]) - float(live_tcp[0])
        ddy = float(fk_tcp[1]) - float(live_tcp[1])
        ddz = float(fk_tcp[2]) - float(live_tcp[2])
        dist = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
        delta_txt = f"({ddx:.3f},{ddy:.3f},{ddz:.3f})"
        dist_txt = f"{dist:.3f}"
    panel._emit_log(
        "[PICK][DIRECT][PANEL_TRACE] "
        f"world_frame={world_frame} base_frame={base_frame} ee_frame={ee_frame or 'none'} "
        f"selected_object={object_name or 'none'} "
        f"object_source={object_source} object_pose_base={panel._format_pose_summary('obj', object_base_data)} "
        f"tcp_live_source={tcp_source} tcp_live_base={panel._format_pose_summary('tcp_live', tcp_base_data)} "
        f"tcp_panel_fk_base={_fmt_tuple(panel._last_tcp_base)} "
        f"tcp_panel_fk_rpy_deg={_fmt_tuple(panel._last_tcp_rpy_deg)} "
        f"tcp_live_rpy_deg={_fmt_tuple(panel._last_trace_tcp_rpy_deg)} "
        f"panel_live_delta={delta_txt} panel_live_dist_m={dist_txt} "
        f"panel_fk_age_sec={max(0.0, now - float(panel._last_tcp_fk_ts or now)):.3f} "
        f"tcp_live_age_sec={max(0.0, now - float(panel._last_trace_tcp_ts or now)):.3f} "
        f"object_age_sec={_fmt_scalar(panel._last_trace_object_age_sec)}"
    )

    def _pose_xyz(data):
        if not isinstance(data, dict):
            return None
        pos = data.get("position")
        if not isinstance(pos, (list, tuple)) or len(pos) < 3:
            return None
        try:
            return (float(pos[0]), float(pos[1]), float(pos[2]))
        except Exception:
            return None

    obj_base_xyz = _pose_xyz(object_base_data)
    legacy_base_xyz = _pose_xyz(legacy_tcp_base_data)
    pinch_base_xyz = _pose_xyz(pinch_tcp_base_data)
    obj_world_xyz = _pose_xyz(object_world_data)
    legacy_world_xyz = _pose_xyz(legacy_tcp_world_data)
    pinch_world_xyz = _pose_xyz(pinch_tcp_world_data)

    def _dist(a, b):
        if a is None or b is None:
            return None
        dx = float(a[0]) - float(b[0])
        dy = float(a[1]) - float(b[1])
        dz = float(a[2]) - float(b[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    legacy_vs_obj = _dist(legacy_base_xyz, obj_base_xyz)
    pinch_vs_obj = _dist(pinch_base_xyz, obj_base_xyz)
    legacy_vs_pinch = _dist(legacy_base_xyz, pinch_base_xyz)
    panel._emit_log(
        "[RG2][AUDIT][TCP] "
        f"base_frame={base_frame} world_frame={world_frame} ee_effective={ee_frame or 'none'} "
        f"panel_fk_tool0_base={_fmt_tuple(panel._last_tcp_base)} "
        f"rg2_pinch_center_base={_fmt_tuple(pinch_base_xyz)} "
        f"rg2_tcp_base={_fmt_tuple(legacy_base_xyz)} "
        f"rg2_pinch_center_world={_fmt_tuple(pinch_world_xyz)} "
        f"rg2_tcp_world={_fmt_tuple(legacy_world_xyz)}"
    )
    panel._emit_log(
        "[RG2][AUDIT][COMPARE] "
        f"selected_object={object_name or 'none'} "
        f"object_base={_fmt_tuple(obj_base_xyz)} object_world={_fmt_tuple(obj_world_xyz)} "
        f"rg2_pinch_center_base={_fmt_tuple(pinch_base_xyz)} rg2_tcp_base={_fmt_tuple(legacy_base_xyz)} "
        f"dist_object_to_rg2_pinch_center_m={_fmt_scalar(pinch_vs_obj)} "
        f"dist_object_to_rg2_tcp_m={_fmt_scalar(legacy_vs_obj)} "
        f"dist_rg2_tcp_to_rg2_pinch_center_m={_fmt_scalar(legacy_vs_pinch)}"
    )


def _refresh_trace_data_resolve_object_pose(
    panel,
    *,
    object_name: str,
    world_frame: str,
    base_frame: str,
    pose_cache: dict,
    pose_wall: float,
    stable_world,
):
    """F3-step30b: resuelve object_world/base_data + object_source (~115 LOC).

    Tres ramas: pose_cache (snapshot fresh + divergence check) /
    stable_world only / not_found. Devuelve dict con {world_data, base_data,
    source}.
    """
    object_world_data = None
    object_base_data = None
    object_source = "pose_info+tf2:UNAVAILABLE"
    if object_name and object_name in pose_cache:
        wx, wy, wz = pose_cache[object_name]
        from .panel_pick_demo_params import get_pick_demo_params as _gpd
        divergence_tol_m = max(0.05, _gpd().object_source_divergence_tol_m)
        if stable_world is not None:
            dx = float(wx) - float(stable_world[0])
            dy = float(wy) - float(stable_world[1])
            dz = float(wz) - float(stable_world[2])
            div_m = math.sqrt(dx * dx + dy * dy + dz * dz)
            if div_m > divergence_tol_m:
                panel._emit_log_throttled(
                    "TRACE:object_pose_divergence",
                    "[PICK][DIRECT][DIVERGENCE] "
                    "kind=object_pose_snapshot_vs_cache "
                    f"object={object_name} "
                    f"snapshot_world=({float(wx):.3f},{float(wy):.3f},{float(wz):.3f}) "
                    f"stable_world=({float(stable_world[0]):.3f},{float(stable_world[1]):.3f},{float(stable_world[2]):.3f}) "
                    f"delta_m={div_m:.3f} tol_m={divergence_tol_m:.3f} "
                    "fallback=stable_object_cache",
                    min_interval=1.5,
                )
                wx, wy, wz = stable_world
        object_world_data = panel._pose_dict((float(wx), float(wy), float(wz)), (0.0, 0.0, 0.0, 1.0), world_frame)
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
            logger=panel._log_trace,
        )
        if base_pose is not None:
            object_base_data = panel._pose_dict(
                (float(base_pose.pose.position.x), float(base_pose.pose.position.y), float(base_pose.pose.position.z)),
                (float(base_pose.pose.orientation.x), float(base_pose.pose.orientation.y), float(base_pose.pose.orientation.z), float(base_pose.pose.orientation.w)),
                base_frame,
            )
            panel._last_selected_world_pose = (float(wx), float(wy), float(wz), world_frame)
            panel._last_selected_base_pose = (
                float(base_pose.pose.position.x),
                float(base_pose.pose.position.y),
                float(base_pose.pose.position.z),
                base_frame,
            )
            panel._last_trace_object_age_sec = max(0.0, _runtime_time() - float(pose_wall or 0.0))
            if stable_world is not None and (
                abs(float(wx) - float(stable_world[0])) > 1e-9
                or abs(float(wy) - float(stable_world[1])) > 1e-9
                or abs(float(wz) - float(stable_world[2])) > 1e-9
            ):
                object_source = f"stable_object_cache:fallback age={float(panel._last_trace_object_age_sec):.2f}s"
            else:
                object_source = f"pose_info+tf2:ok age={float(panel._last_trace_object_age_sec):.2f}s"
        else:
            panel._last_trace_object_age_sec = None
            object_source = f"pose_info+tf2:tf_failed:{base_reason}"
    elif object_name and stable_world is not None:
        wx, wy, wz = stable_world
        object_world_data = panel._pose_dict((float(wx), float(wy), float(wz)), (0.0, 0.0, 0.0, 1.0), world_frame)
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
            logger=panel._log_trace,
        )
        if base_pose is not None:
            object_base_data = panel._pose_dict(
                (float(base_pose.pose.position.x), float(base_pose.pose.position.y), float(base_pose.pose.position.z)),
                (float(base_pose.pose.orientation.x), float(base_pose.pose.orientation.y), float(base_pose.pose.orientation.z), float(base_pose.pose.orientation.w)),
                base_frame,
            )
            object_source = "stable_object_cache:no_pose_snapshot"
        else:
            object_source = f"stable_object_cache:tf_failed:{base_reason}"
    else:
        panel._last_trace_object_age_sec = None
        object_source = f"pose_info+tf2:not_found:{object_name}"
    return object_world_data, object_base_data, object_source


def _refresh_trace_data(panel):
    if panel._closing or not panel.trace_table or not panel._bridge_running:
        return
    if panel.chk_trace_freeze and panel.chk_trace_freeze.isChecked():
        return
    now = time.monotonic()
    world_frame = panel._world_frame_last_first()
    base_frame, ee_frame = panel._resolve_trace_frames(world_frame)
    panel._log_tf_chain_once(world_frame, base_frame, ee_frame)
    frames_text = f"Frames: world={world_frame} base={base_frame} ee={ee_frame or 'unavailable'}"
    if panel.lbl_trace_frames:
        panel.lbl_trace_frames.setText(frames_text)

    object_name = str(panel._selected_object or PICK_DEMO_OBJECT_NAME)
    pose_cache = {}
    pose_wall = 0.0
    stable_positions = get_object_positions() or {}
    stable_world = None
    if object_name:
        try:
            stable_world = tuple(float(v) for v in stable_positions.get(object_name, ())[:3]) if object_name in stable_positions else None
        except Exception:
            stable_world = None
    if panel._ros_worker_started and panel.ros_worker.node_ready():
        try:
            pose_cache, pose_wall = panel.ros_worker.pose_snapshot()
        except Exception as exc:
            panel._log_trace_transform_warning(f"pose_snapshot: {exc}")
            pose_cache = {}
            pose_wall = 0.0
    object_world_data, object_base_data, object_source = (
        _refresh_trace_data_resolve_object_pose(
            panel,
            object_name=object_name,
            world_frame=world_frame,
            base_frame=base_frame,
            pose_cache=pose_cache,
            pose_wall=pose_wall,
            stable_world=stable_world,
        )
    )

    tcp_data = _refresh_trace_data_resolve_tcp_pose(
        panel,
        ee_frame=ee_frame,
        base_frame=base_frame,
        world_frame=world_frame,
        now=now,
    )
    tcp_world_data = tcp_data["tcp_world_data"]
    tcp_base_data = tcp_data["tcp_base_data"]
    tcp_source = tcp_data["tcp_source"]
    legacy_tcp_base_data = tcp_data["legacy_tcp_base_data"]
    pinch_tcp_base_data = tcp_data["pinch_tcp_base_data"]
    legacy_tcp_world_data = tcp_data["legacy_tcp_world_data"]
    pinch_tcp_world_data = tcp_data["pinch_tcp_world_data"]

    panel._check_tcp_source_mismatch(now)

    panel._set_trace_row(0, object_world_data, object_base_data, world_frame, base_frame)
    panel._set_trace_row(1, tcp_world_data, tcp_base_data, world_frame, base_frame)
    panel.trace_table.resizeRowsToContents()

    base_error = panel._compute_error(object_base_data, tcp_base_data)
    world_error = panel._compute_error(object_world_data, tcp_world_data)
    if panel.lbl_trace_error_base:
        panel.lbl_trace_error_base.setText(panel._format_error_text(base_error))
    if panel.lbl_trace_error_world:
        panel.lbl_trace_error_world.setText(panel._format_error_text(world_error))

    tf_transform = panel._last_tf_status.get("transform") if panel._last_tf_status else None
    if tf_transform and panel.lbl_trace_tf_translation and panel.lbl_trace_tf_yaw:
        t = tf_transform.transform.translation
        translation_text = f"{t.x:.3f}, {t.y:.3f}, {t.z:.3f}"
        yaw_deg = math.degrees(yaw_from_quaternion(tf_transform.transform.rotation))
        panel.lbl_trace_tf_translation.setText(translation_text)
        panel.lbl_trace_tf_yaw.setText(f"{yaw_deg:.2f}°")
    else:
        if panel.lbl_trace_tf_translation:
            panel.lbl_trace_tf_translation.setText("--")
        if panel.lbl_trace_tf_yaw:
            panel.lbl_trace_tf_yaw.setText("--")

    panel._trace_cached_text = panel._build_trace_text(
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
    _refresh_trace_data_emit_audits(
        panel,
        now=now,
        world_frame=world_frame,
        base_frame=base_frame,
        ee_frame=ee_frame,
        object_name=object_name,
        object_source=object_source,
        tcp_source=tcp_source,
        object_world_data=object_world_data,
        object_base_data=object_base_data,
        tcp_world_data=tcp_world_data,
        tcp_base_data=tcp_base_data,
        legacy_tcp_base_data=legacy_tcp_base_data,
        pinch_tcp_base_data=pinch_tcp_base_data,
        legacy_tcp_world_data=legacy_tcp_world_data,
        pinch_tcp_world_data=pinch_tcp_world_data,
    )
    panel._maybe_log_trace(now)

def _log_trace_transform_warning(panel, message: str) -> None:
    now = time.monotonic()
    key = message.split(":", 1)[0]
    last = panel._trace_transform_warn_last.get(key, 0.0)
    panel._trace_transform_warn_count[key] = panel._trace_transform_warn_count.get(key, 0) + 1
    if (now - last) < panel._trace_transform_warn_period:
        return
    count = panel._trace_transform_warn_count.get(key, 0)
    panel._trace_transform_warn_count[key] = 0
    panel._trace_transform_warn_last[key] = now
    panel._emit_log(f"[TRACE][WARN] {message} ({count})")

def _maybe_log_tf_not_ready(panel):
    if panel._tf_not_ready_logged:
        return
    now = time.monotonic()
    if panel._bridge_start_ts and (now - panel._bridge_start_ts) < TF_INIT_GRACE_SEC:
        return
    if now - panel._tf_ready_last_notice >= 1.0:
        panel._log("[TRACE] TF not ready yet (waiting for transforms)")
        panel._tf_ready_last_notice = now
        panel._tf_not_ready_logged = True

def _maybe_log_trace(panel, now: float):
    if not panel._trace_ready:
        return
    if "TF world→base: n/a" in (panel._trace_cached_text or ""):
        return
    if now - panel._last_trace_print_ts >= panel._trace_print_period:
        dt = now - panel._last_trace_print_ts
        if not panel._trace_debug_logged:
            panel._log_trace(
                f"[TRACE][DEBUG] mono_now={now:.3f} mono_last={panel._last_trace_print_ts:.3f} mono_dt={dt:.3f}"
            )
            panel._trace_debug_logged = True
        panel._last_trace_print_ts = now
        panel._log_trace("[TRACE] " + panel._trace_cached_text.replace("\n", " | "))

def _reset_trace_throttle(panel, reason: str):
    now = time.monotonic()
    panel._last_trace_print_ts = now - panel._trace_print_period
    panel._last_ee_warn_ts = now - panel._ee_warn_period
    panel._trace_debug_logged = False
    panel._tf_not_ready_logged = False
    panel._log_trace(f"[TRACE] throttle reset ({reason})")

def _run_trace_diag_once(panel):
    if panel._trace_diag_inflight:
        return
    panel._trace_diag_inflight = True
    panel._run_async(panel._trace_diag_worker, name="trace_diag")

def _tf_sanity_check(panel) -> Tuple[bool, str]:
    """FASE 1: TF sanity check usando solo base_link (marco global efectivo)."""
    helper = get_tf_helper()
    if helper is None:
        return False, "tf_helper_off"
    base_frame = panel._business_base_frame()
    ee_frame = str(getattr(panel, "_required_ee_frame", "") or "rg2_pinch_center").strip() or "rg2_pinch_center"
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
        if panel._ros_worker_started and panel.ros_worker.node_ready():
            with panel.ros_worker._lock:
                ros_now_ns = int(getattr(panel.ros_worker, "_last_clock_stamp_ns", 0) or 0)
        if ros_now_ns > 0 and tf_stamp_ns > 0:
            tf_age = (ros_now_ns - tf_stamp_ns) / 1_000_000_000.0
            age_txt = f"{tf_age:.3f}s"
    except Exception:
        pass
    # FASE 1: Eliminar check de world (no es necesario, solo causa spam TF)
    # Operamos únicamente en base_link como GLOBAL_FRAME_EFFECTIVE
    return True, f"{base_frame}->{ee_frame} ok stamp={stamp_txt} age={age_txt}"

def _run_self_check_once(panel) -> None:
    panel._run_async(panel._self_check_worker, name="self_check")

def _self_check_worker(panel) -> None:
    tf_ok, tf_reason = panel._tf_sanity_check()
    camera_ok, camera_reason = camera_ready_status(panel)
    controllers_ok, controllers_reason = panel._controllers_ready()
    now = _runtime_time()
    rgb_age = now - panel._last_camera_frame_ts if panel._last_camera_frame_ts else float("inf")
    depth_required, depth_topic = panel._camera_depth_expectation()
    depth_age = now - panel._last_camera_depth_frame_ts if panel._last_camera_depth_frame_ts else float("inf")
    camera_topic = panel.camera_topic_combo.currentText().strip() if hasattr(panel, "camera_topic_combo") else panel.camera_topic
    hz_rgb = 0.0
    if panel._camera_subscribe_ts > 0.0 and panel._camera_frame_count > 0:
        hz_rgb = float(panel._camera_frame_count) / max(1e-3, now - panel._camera_subscribe_ts)
    hz_depth = 0.0
    if panel._camera_subscribe_ts > 0.0 and panel._camera_depth_frame_count > 0:
        hz_depth = float(panel._camera_depth_frame_count) / max(1e-3, now - panel._camera_subscribe_ts)
    panel._emit_log(
        f"[SELF_CHECK] TF={'OK' if tf_ok else 'FAIL'} reason={tf_reason}"
    )
    panel._emit_log(
        f"[SELF_CHECK] CAMERA={'OK' if camera_ok else 'FAIL'} reason={camera_reason or panel._camera_not_ready_reason()} "
        f"topic={camera_topic or 'n/a'} hz={hz_rgb:.2f} "
        f"last_age={'inf' if math.isinf(rgb_age) else f'{rgb_age:.2f}s'} "
        f"depth_required={str(depth_required).lower()} depth_topic={depth_topic or 'n/a'} "
        f"depth_hz={hz_depth:.2f} depth_age={'inf' if math.isinf(depth_age) else f'{depth_age:.2f}s'}"
    )
    panel._emit_log(
        f"[SELF_CHECK] CONTROLLERS={'OK' if controllers_ok else 'FAIL'} reason={controllers_reason}"
    )
    if tf_ok and camera_ok and controllers_ok:
        panel._ui_set_status("Self-check OK: TF + Camera + Controllers", error=False)
    else:
        panel._ui_set_status(
            f"Self-check FAIL: TF={tf_reason} Camera={camera_reason or panel._camera_not_ready_reason()} Controllers={controllers_reason}",
            error=False,
        )

def _trace_diag_worker(panel) -> None:
    topic_names: Set[str] = set()
    try:
        if panel.ros_worker:
            topic_names = set(panel.ros_worker.list_topic_names())
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
        if panel.ros_worker:
            joint_payload, _ = panel.ros_worker.get_last_joint_state()
        joint_received = joint_payload is not None
        names_len = len(joint_payload.get("name", [])) if joint_payload else 0
        position_len = len(joint_payload.get("position", [])) if joint_payload else 0
        robot_frames = [
            frame for frame in sorted(frames) if any(keyword in frame.lower() for keyword in ROBOT_FRAME_KEYWORDS)
        ]
        sample = ", ".join(robot_frames[:10]) if robot_frames else "-"
        panel._log(
            f"[TRACE][DIAG] topics: /joint_states={joint_states_present} /tf={tf_present} /tf_static={tf_static_present}"
        )
        panel._log(
            f"[TRACE][DIAG] frames={len(frames)} robot_candidates={len(robot_frames)} sample={sample}"
        )
        panel._log(
            f"[TRACE][DIAG] joint_states_msg_received={joint_received} names_len={names_len} position_len={position_len}"
        )
        if helper is None:
            panel._log("[TRACE][DIAG] TF helper unavailable for transform checks")
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
        panel._log(
            f"[TRACE][DIAG] tf_listener_msgs tf={tf_stats[0]} tf_static={tf_stats[1]}"
        )
        if tf_stats[0] > 0 and tf_stats[1] == 0:
            panel._log(
                "[TRACE][DIAG] tf_static=0 (aceptable si la cadena requerida está en /tf dinámico)"
            )
        panel._log(
            f"[TRACE][DIAG] tf_frames_seen_count={len(tf_frames_set)} sample={tf_frames_sample}"
        )
        panel._log(
            f"[TRACE][DIAG] tf_static_frames_seen_count={len(tf_static_frames_set)} sample={tf_static_sample}"
        )
        panel._log(
            f"[TRACE][DIAG] tf_robot_candidates_seen={len(combined_robot_frames)} sample={', '.join(combined_robot_frames[:10]) or '-'}"
        )
    finally:
        panel._trace_diag_inflight = False
    if tf_stats[0] == 0 and not combined_robot_frames:
        panel._log(
            "[TRACE][DIAG] No dynamic TF (/tf) received → robot TF missing (check robot_state_publisher / controllers)"
        )

    def diag_transform(label: str, frame_a: str, frame_b: str):
        ok = _can_transform_between(helper, frame_a, frame_b, timeout_sec=0.1)
        panel._log(f"[TRACE][DIAG] {label} {frame_a}<->{frame_b} ok={ok}")
        return ok

    base_frame = "base_link"
    diag_transform("base->base_link", base_frame, "base_link")

    def gather_candidates(substring_predicate):
        return [frame for frame in sorted(frames) if substring_predicate(frame.lower())]

    tool_like = gather_candidates(lambda text: text.endswith("tool0") or "tool0" in text)
    ee_like = gather_candidates(lambda text: "ee" in text and "link" in text)
    wrist_like = gather_candidates(lambda text: "wrist_3" in text)
    panel._log(f"[TRACE][DIAG] tool_like={tool_like[:10]}")
    panel._log(f"[TRACE][DIAG] ee_like={ee_like[:10]}")
    panel._log(f"[TRACE][DIAG] wrist_like={wrist_like[:10]}")

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
        panel._log(f"[TRACE][DIAG] recommended EE candidate: {recommended_ee}")
    else:
        panel._log("[TRACE][DIAG] no EE candidate transformable from base_link")

def _try_mark_tf_ready(panel):
    panel._tf_monitor.try_mark_ready()

def _start_tf_ready_timer(panel):
    panel._tf_monitor.start()

def _wait_for_tf_ready(panel, world_frame: str, helper: Optional["TfHelper"]) -> Optional[str]:
    return panel._tf_monitor.wait_for_ready(world_frame, helper)

def _tf_world_base_valid(panel, helper: "TfHelper", base_frame: str, world_frame: str) -> bool:
    return tf_world_base_valid(panel, helper, base_frame, world_frame)

def _stop_tf_ready_timer(panel):
    panel._tf_monitor.stop()

def _log_tf_chain_once(panel, world_frame: str, base_frame: str, ee_frame: Optional[str]) -> None:
    if panel._tf_chain_logged:
        return
    tf_wb, reason_wb = tf_get_transform(
        world_frame,
        base_frame,
        timeout=0.20,
        logger=panel._log_trace,
    )
    if tf_wb is not None:
        s = tf_wb.header.stamp
        kind = "static" if (int(getattr(s, "sec", 0)) == 0 and int(getattr(s, "nanosec", 0)) == 0) else "dynamic"
        panel._log_trace(
            "[TRACE][TF_CHAIN] "
            f"world->{base_frame} trans=({float(tf_wb.transform.translation.x):.3f},"
            f"{float(tf_wb.transform.translation.y):.3f},{float(tf_wb.transform.translation.z):.3f}) "
            f"stamp={int(getattr(s, 'sec', 0))}.{int(getattr(s, 'nanosec', 0)):09d} kind={kind}"
        )
    else:
        panel._log_trace(f"[TRACE][TF_CHAIN] world->{base_frame} unavailable reason={reason_wb}")

    ee = str(ee_frame or "rg2_pinch_center").strip() or "rg2_pinch_center"
    tf_be, reason_be = tf_get_transform(
        base_frame,
        ee,
        timeout=0.20,
        logger=panel._log_trace,
    )
    if tf_be is not None:
        s = tf_be.header.stamp
        kind = "static" if (int(getattr(s, "sec", 0)) == 0 and int(getattr(s, "nanosec", 0)) == 0) else "dynamic"
        panel._log_trace(
            "[TRACE][TF_CHAIN] "
            f"{base_frame}->{ee} trans=({float(tf_be.transform.translation.x):.3f},"
            f"{float(tf_be.transform.translation.y):.3f},{float(tf_be.transform.translation.z):.3f}) "
            f"stamp={int(getattr(s, 'sec', 0))}.{int(getattr(s, 'nanosec', 0)):09d} kind={kind}"
        )
    else:
        panel._log_trace(f"[TRACE][TF_CHAIN] {base_frame}->{ee} unavailable reason={reason_be}")

    panel._tf_chain_logged = True

def _check_tcp_source_mismatch(panel, now_mono: float) -> None:
    if panel._last_trace_tcp_base is None or panel._last_tcp_base is None:
        return
    if abs(float(panel._last_trace_tcp_ts) - float(panel._last_tcp_fk_ts)) > 2.0:
        return
    tx, ty, tz = panel._last_trace_tcp_base
    dx, dy, dz = panel._last_tcp_base
    ddx = float(tx) - float(dx)
    ddy = float(ty) - float(dy)
    ddz = float(tz) - float(dz)
    dist = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
    if dist > 0.02 and (now_mono - float(panel._last_tcp_mismatch_warn_ts or 0.0)) >= 1.0:
        panel._last_tcp_mismatch_warn_ts = now_mono
        panel._emit_log(
            "[PICK][DIRECT][DIVERGENCE] "
            "kind=panel_fk_vs_tf_live "
            f"delta_m={dist:.3f} "
            f"panel_fk_tcp=({float(dx):.3f},{float(dy):.3f},{float(dz):.3f}) "
            f"tf_live_tcp=({float(tx):.3f},{float(ty):.3f},{float(tz):.3f}) "
            f"delta_vec=({ddx:.3f},{ddy:.3f},{ddz:.3f}) "
            f"panel_fk_age_sec={max(0.0, now_mono - float(panel._last_tcp_fk_ts or now_mono)):.3f} "
            f"tf_live_age_sec={max(0.0, now_mono - float(panel._last_trace_tcp_ts or now_mono)):.3f} "
            "note=panel_fk_model_pose_not_same_as_live_rg2_tcp"
        )

def _build_trace_text(panel,
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
        panel._format_pose_summary("Object/base", object_base),
        panel._format_pose_summary("TCP/base", tcp_base),
        f"Error base (dx,dy,dz,dist): {panel._format_error_tuple(base_error)}",
    ]
    if tf_transform:
        t = tf_transform.transform.translation
        yaw_deg = math.degrees(yaw_from_quaternion(tf_transform.transform.rotation))
        lines.append(f"TF world→base translation: ({t.x:.3f},{t.y:.3f},{t.z:.3f})")
        lines.append(f"TF world→base yaw: {yaw_deg:.2f}°")
    else:
        lines.append("TF world→base: n/a")
    return "\n".join(lines)

def _set_trace_row(panel,
    row: int,
    world_data: Optional[Dict[str, object]],
    base_data: Optional[Dict[str, object]],
    world_frame: str,
    base_frame: str,
):
    if not panel.trace_table:
        return
    frame_text = f"{world_frame}\n{base_frame}"
    panel._set_trace_item(row, 0, frame_text)
    axes = ["x", "y", "z"]
    for idx, axis in enumerate(axes, start=1):
        world_val = panel._value_from_pose(world_data, axis)
        base_val = panel._value_from_pose(base_data, axis)
        panel._set_trace_item(row, idx, panel._format_dual_value(world_val, base_val))
    orientation_keys = ["qx", "qy", "qz", "qw"]
    for idx, key in enumerate(orientation_keys, start=4):
        world_val = panel._value_from_pose(world_data, key)
        base_val = panel._value_from_pose(base_data, key)
        panel._set_trace_item(row, idx, panel._format_dual_value(world_val, base_val))

def _value_from_pose(panel, data: Optional[Dict[str, object]], key: str) -> Optional[float]:
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

def _set_trace_item(panel, row: int, col: int, text: str):
    if not panel.trace_table:
        return
    item = QTableWidgetItem(text)
    item.setTextAlignment(Qt.AlignCenter)
    item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
    panel.trace_table.setItem(row, col, item)

def _format_dual_value(panel, first: Optional[float], second: Optional[float]) -> str:
    def fmt(value: Optional[float]) -> str:
        return f"{value:.3f}" if value is not None else "n/a"
    return f"{fmt(first)}\n{fmt(second)}"

def _pose_dict(panel, position: Tuple[float, float, float], orientation: Tuple[float, float, float, float], frame: str) -> Dict[str, object]:
    return {"frame": frame, "position": position, "orientation": orientation}

def _compute_error(panel, source: Optional[Dict[str, object]], target: Optional[Dict[str, object]]
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

def _format_error_text(panel, error: Optional[Tuple[float, float, float, float]]) -> str:
    if not error:
        return "n/a"
    dx, dy, dz, dist = error
    return f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} dist={dist:.3f}"

def _format_error_tuple(panel, error: Optional[Tuple[float, float, float, float]]) -> str:
    if not error:
        return "n/a"
    return " ".join(f"{value:.3f}" for value in error)

def _format_pose_summary(panel, label: str, data: Optional[Dict[str, object]]) -> str:
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

def _copy_trace_text(panel):
    if not panel._trace_cached_text:
        panel._set_status("Trace vacío", error=True)
        return
    QApplication.clipboard().setText(panel._trace_cached_text)
    panel._set_status("Trace copiado al portapapeles")
