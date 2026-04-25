#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tf_diagnose.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""TF diagnostics helper for selection logging."""
from __future__ import annotations

import math
import time
from typing import Tuple

from .panel_config import PICK_LOG_MIN_INTERVAL_SEC
from .panel_utils import effective_base_frame
from .panel_utils import (
    _list_tf_topics,
    debug_dump_tf,
    diagnose_tf_tree,
    yaw_from_quaternion,
)


def log_missing_ee_frames(panel, frames, now: float, interval_sec: float = 1.5) -> None:
    ee_candidates = [
        f for f in frames if any(k in f.lower() for k in ("tool", "tcp", "ee", "flange", "wrist", "rg2", "hand", "ft"))
    ]
    if ee_candidates:
        return
    if (now - getattr(panel, "_last_tf_diag_log", 0.0)) > interval_sec:
        panel._emit_log("[TF][DIAG] No hay EE frame transformable desde base. Bloqueando PICK.")
        panel._last_tf_diag_log = now


def run_tf_diagnose(panel, world_pose: Tuple[float, float, float], world_frame: str) -> None:
    """Populate panel TF diagnosis state and emit logs for a selected point."""
    tf_status = diagnose_tf_tree(world_pose, selection_frame=world_frame)
    panel._last_tf_status = tf_status
    base_frame_label = tf_status.get("base_frame") or effective_base_frame(panel, default="base_link")
    if base_frame_label != "base_link":
        panel._emit_log(f"[FRAME][P0] frame base inválido detectado={base_frame_label}; forzando base_link")
        base_frame_label = "base_link"
    world_frame_label = tf_status.get("world_frame") or world_frame
    panel._last_selection_frame = world_frame_label
    panel._base_frame_effective = base_frame_label
    selected_base = tf_status.get("selected_base")
    if selected_base:
        panel._last_selected_base_pose = (
            selected_base[0],
            selected_base[1],
            selected_base[2],
            base_frame_label,
        )
    else:
        panel._last_selected_base_pose = None
    tf_lookup_status = "ok" if tf_status.get("ok") else "fail"
    if tf_status.get("ok"):
        panel.signal_trace_ready.emit()
    selected_base = tf_status.get("selected_base")
    if selected_base:
        bx, by, bz = selected_base
        panel._last_selected_base_pose = (bx, by, bz, base_frame_label)
    else:
        panel._last_selected_base_pose = None
    transform = tf_status.get("transform")
    log_sig_parts = [
        "ok" if tf_status.get("ok") else "fail",
        base_frame_label,
        world_frame_label,
    ]
    if selected_base:
        log_sig_parts.append(
            f"sel={selected_base[0]:.3f},{selected_base[1]:.3f},{selected_base[2]:.3f}"
        )
    if transform:
        t = transform.transform.translation
        yaw_deg = math.degrees(yaw_from_quaternion(transform.transform.rotation))
        log_sig_parts.append(f"t={t.x:.3f},{t.y:.3f},{t.z:.3f},{yaw_deg:.2f}")
    log_signature = "|".join(log_sig_parts)
    now = time.time()
    log_pick = True
    if tf_status.get("ok"):
        if (
            log_signature == panel._pick_log_last_sig
            and (now - panel._pick_log_last_ts) < PICK_LOG_MIN_INTERVAL_SEC
        ):
            log_pick = False
        else:
            panel._pick_log_last_sig = log_signature
            panel._pick_log_last_ts = now
    else:
        panel._pick_log_last_sig = log_signature
        panel._pick_log_last_ts = now
    if log_pick or not tf_status.get("ok"):
        panel._log(f"[PICK] TF_DISCOVERY: base={base_frame_label} world={world_frame_label}")
        panel._log(
            f"[PICK] tf_lookup: {tf_lookup_status} "
            f"world={world_frame_label} base={base_frame_label} err={tf_status.get('error') or 'n/a'}"
        )
        if selected_base:
            panel._log(
                f"[PICK] selected_base=({bx:.3f},{by:.3f},{bz:.3f}) frame={base_frame_label}"
            )
        else:
            panel._log(f"[PICK] selected_base=None frame={base_frame_label}")
    if transform:
        if log_pick or not tf_status.get("ok"):
            t = transform.transform.translation
            yaw_deg = math.degrees(yaw_from_quaternion(transform.transform.rotation))
            panel._log(
                f"[PICK] tf_world_to_base: translation=({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
                f"yaw={yaw_deg:.2f}° frame={base_frame_label}"
            )
    tf_topics_list = tf_status.get("tf_topics") or []
    if not tf_topics_list and panel._trace_ready:
        tf_topics_list, _ = _list_tf_topics()
    tf_topics_str = ",".join(tf_topics_list) if tf_topics_list else "n/a"
    fallback_reason = tf_status.get("fallback") or "none"
    if panel._trace_ready:
        fallback_reason = "none"
    if (log_pick or not tf_status.get("ok")) and (panel._debug_logs_enabled or not tf_status.get("ok")):
        panel._log(f"[PICK] tf_topics={tf_topics_str} fallback={fallback_reason}")
        debug_tf, debug_err = debug_dump_tf(world_frame_label, base_frame_label)
        if debug_tf:
            tx, ty, tz = debug_tf["translation"]
            yaw_deg = math.degrees(debug_tf["yaw"])
            panel._log(
                f"[PICK] tf_world_to_base: translation=({tx:.3f},{ty:.3f},{tz:.3f}), "
                f"yaw={yaw_deg:.2f}°"
            )
        else:
            panel._log(f"[PICK] tf_world_to_base: error={debug_err or 'unknown'}")
