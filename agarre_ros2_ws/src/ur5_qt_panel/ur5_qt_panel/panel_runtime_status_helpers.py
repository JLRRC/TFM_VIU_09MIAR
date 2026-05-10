#!/usr/bin/env python3
"""F6 (auditoría 2026-05-10): runtime status helpers extraídos de panel_helpers.

Funciones de chequeo de estado runtime (clock, joint_states, bridge, TF
chain, cámara, pose_info, Gazebo proc) que el panel usa para evaluar
la disponibilidad de las distintas fuentes ROS/Gazebo. Cada función
recibe ``panel`` (instancia de ControlPanelV2) como primer argumento
y consulta atributos cacheados.

Extraídas de ``panel_helpers.py`` líneas ~1026–1316. ``panel_helpers``
re-exporta cada símbolo para preservar la API que usan los mixins
``panel_v2_*_mixin.py`` (que importan ``panel_helpers as _ph``).
"""
from __future__ import annotations

import os
import time
from typing import Optional, Tuple

try:
    import psutil
except ImportError:
    psutil = None  # type: ignore[assignment]

from .logging_utils import emit_log_line
# F6: import lazy de _runtime_time para evitar ciclo
# panel_camera → panel_tfm_inference → panel_camera (ya latente, expuesto
# al promover este módulo a top-level desde panel_helpers).
from .panel_config import (
    CAMERA_READY_MAX_AGE_SEC,
    DROP_OBJECT_NAMES,
    GZ_WORLD,
    POSE_INFO_MAX_AGE_SEC,
    STATUS_TOPIC_CACHE_SEC,
    UR5_JOINT_NAMES,
)
from .panel_objects import (
    bulk_update_object_positions,
    get_object_positions,
    is_on_table,
    recalc_object_states,
)
from .panel_tf import get_tf_helper
from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params
from .panel_utils import _can_transform_between, gz_sim_status


def _log_exception(context: str, exc: Exception) -> None:
    """Emite log uniforme para excepciones — copia local para evitar
    dependencia circular con ``panel_helpers``."""
    emit_log_line(f"[HELPERS][ERROR][{context}] {exc}")


def _normalize_joint_name(name) -> str:
    text = str(name).strip()
    if "::" in text:
        text = text.split("::")[-1]
    if "/" in text:
        text = text.split("/")[-1]
    return text.strip()


def _clock_status(panel) -> Tuple[bool, str]:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False, "node_off"
    ok, age = panel.ros_worker.clock_alive()
    if ok:
        return True, f"age={age:.2f}s"
    return False, f"age={age:.2f}s"


def _joint_states_status(panel) -> Tuple[bool, str]:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False, "node_off"
    topic = (panel._joint_current_topic or panel.joint_topic or "/joint_states").strip() or "/joint_states"
    has_pub = panel.ros_worker.topic_has_publishers(topic)
    if not has_pub:
        return False, f"{topic}:no_publishers"
    payload, ts = panel.ros_worker.get_last_joint_state()
    if payload is None:
        return False, f"{topic}:no_msgs"
    names = payload.get("name", []) or []
    if len(names) == 0:
        return False, f"{topic}:empty"
    strict_identity = _get_panel_ui_params().strict_joint_identity
    if strict_identity:
        normalized = {_normalize_joint_name(str(n)) for n in names if str(n).strip()}
        missing = [jn for jn in UR5_JOINT_NAMES if jn not in normalized]
        if missing:
            sample = ", ".join(sorted(list(normalized))[:8])
            return False, f"{topic}:joint_identity_mismatch missing={','.join(missing)} sample={sample}"
    from .panel_camera import _runtime_time  # lazy: ver nota del módulo
    age = float("inf")
    if ts:
        age = max(0.0, _runtime_time() - ts)
    if age > 2.0:
        return False, f"{topic}:stale age={age:.2f}s"
    return True, f"topic={topic} age={age:.2f}s names={len(names)}"


def _bridge_transport_detected(panel) -> bool:
    # A pose-only bridge (gz_pose_bridge) is not enough for DIRECTO; require
    # joint_states as well before treating the full ROS bridge as active.
    if panel._proc_alive(panel.bridge_proc):
        return True
    if not panel._pose_info_active():
        return False
    js_ok, _js_reason = panel._joint_states_status()
    return bool(js_ok)


def _bridge_ready_status(panel) -> Tuple[bool, str]:
    if not panel._bridge_running:
        return False, "bridge_process_off"
    clock_ok, clock_reason = panel._clock_status()
    if not clock_ok:
        return False, f"clock_not_ready ({clock_reason})"
    if not panel._pose_info_active():
        return False, "pose_info_no_publishers"
    js_ok, js_reason = panel._joint_states_status()
    if not js_ok:
        return False, js_reason
    helper = get_tf_helper()
    if helper is None:
        return False, "tf_helper_off"
    tf_stats = helper.tf_listener_stats()
    if tf_stats[0] <= 0:
        return False, f"tf_msgs={tf_stats[0]}"
    return True, f"clock+pose+joint_states+tf ok (tf_msgs={tf_stats[0]})"


def _tf_chain_ready_status(panel) -> Tuple[bool, str]:
    helper = get_tf_helper()
    if helper is None:
        return False, "tf_helper_off"
    base_frame = panel._business_base_frame()
    ee_frame = str(getattr(panel, "_required_ee_frame", "") or "rg2_pinch_center").strip() or "rg2_pinch_center"
    if not _can_transform_between(helper, base_frame, ee_frame, timeout_sec=0.2):
        return False, f"{base_frame}<->{ee_frame} missing"
    return True, f"{base_frame}->{ee_frame}"


def _camera_depth_expectation(panel) -> Tuple[bool, str]:
    topic = str(panel.camera_topic or "").strip()
    if topic.endswith("/depth_image"):
        return False, topic
    if topic.endswith("/image"):
        depth_topic = topic[: -len("/image")] + "/depth_image"
    elif topic.endswith("/rgb"):
        depth_topic = topic[: -len("/rgb")] + "/depth_image"
    else:
        depth_topic = "/camera_overhead/depth_image"
    # The runtime camera gate must follow the active TFM model modality.
    # Recent RGB checkpoints were being blocked by stale depth even though
    # preprocessing only consumes depth when in_channels == 4.
    depth_required = bool(panel._camera_depth_required_env)
    if not depth_required:
        in_channels = 0
        try:
            model_info = panel.tfm_module.model_info() if panel.tfm_module else {}
            in_channels = int((model_info or {}).get("in_channels", 0) or 0)
        except Exception:
            in_channels = 0
        if in_channels <= 0:
            try:
                modality = str(getattr(panel, "_exp_info", {}).get("modality", "") or "").strip().lower()
            except Exception:
                modality = ""
            if modality in ("rgbd", "rgb-d"):
                in_channels = 4
            elif modality == "rgb":
                in_channels = 3
        if in_channels > 0:
            depth_required = in_channels >= 4
        elif hasattr(panel, "chk_tfm_use_depth") and panel.chk_tfm_use_depth is not None:
            try:
                depth_required = bool(panel.chk_tfm_use_depth.isChecked())
            except Exception:
                pass
    return depth_required, depth_topic


def _camera_runtime_flags(panel, now: Optional[float] = None) -> Tuple[bool, bool, bool, float, bool]:
    """Return camera_ready, camera_fault, camera_source_down, frame_age, warmup_grace."""
    from .panel_camera import _runtime_time  # lazy: ver nota del módulo
    if now is None:
        now = _runtime_time()
    age = now - panel._last_camera_frame_ts if panel._last_camera_frame_ts else float("inf")
    gz_state = panel._gazebo_state()
    gazebo_ready = gz_state == "GAZEBO_READY"
    bridge_ready = bool(panel._bridge_running)
    source_down = (not gazebo_ready) or (not bridge_ready)
    has_frames = panel._camera_frame_count > 0
    depth_required, _depth_topic = panel._camera_depth_expectation()
    depth_age = (
        now - panel._last_camera_depth_frame_ts
        if panel._last_camera_depth_frame_ts
        else float("inf")
    )
    depth_ready = (
        (not depth_required)
        or (
            panel._camera_depth_frame_count > 0
            and depth_age < CAMERA_READY_MAX_AGE_SEC
            and gazebo_ready
            and bridge_ready
        )
    )
    grace_anchor = 0.0
    if panel._camera_subscribe_ts > 0.0:
        grace_anchor = panel._camera_subscribe_ts
    elif panel._camera_init_start > 0.0:
        grace_anchor = panel._camera_init_start
    elif panel._bridge_start_ts > 0.0:
        grace_anchor = panel._bridge_start_ts
    in_grace = grace_anchor > 0.0 and (now - grace_anchor) < panel._camera_warmup_grace_sec
    camera_ready = has_frames and age < CAMERA_READY_MAX_AGE_SEC and depth_ready and gazebo_ready and bridge_ready
    camera_fault = has_frames and age > panel._camera_fault_age_sec and gazebo_ready and bridge_ready and (not in_grace)
    if depth_required:
        camera_fault = camera_fault or (
            panel._camera_depth_frame_count > 0
            and depth_age > panel._camera_fault_age_sec
            and gazebo_ready
            and bridge_ready
            and (not in_grace)
        )
    return camera_ready, camera_fault, source_down, age, in_grace


def _sync_external_release_state(panel) -> bool:
    if panel._objects_release_done and panel._objects_settled:
        return True
    if not panel._pose_info_ok:
        return False
    positions = get_object_positions() or {}
    live_positions = {}
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            pose_map, _pose_ts = panel.ros_worker.pose_snapshot()
        except Exception:
            pose_map = {}
        for name in DROP_OBJECT_NAMES:
            pos = pose_map.get(name)
            if pos is None or len(pos) < 3:
                continue
            try:
                live_positions[name] = (float(pos[0]), float(pos[1]), float(pos[2]))
            except Exception:
                continue
        if live_positions:
            bulk_update_object_positions(
                live_positions,
                source="external_release_reconciled_live",
                objects_stable=True,
            )
            recalc_object_states(reason="external_release_reconciled_live")
            positions = get_object_positions() or {}
    if not DROP_OBJECT_NAMES:
        return False
    for name in DROP_OBJECT_NAMES:
        pos = positions.get(name)
        if pos is None or len(pos) < 3:
            return False
        try:
            xyz = (float(pos[0]), float(pos[1]), float(pos[2]))
        except Exception:
            return False
        if not is_on_table(xyz):
            return False
    if not panel._objects_release_done or not panel._objects_settled:
        panel._objects_release_done = True
        panel._objects_settled = True
        panel._emit_log(
            "[PHYSICS][DROP] external release reconciled scene=on_table_all "
            f"count={len(DROP_OBJECT_NAMES)}"
        )
        recalc_object_states(reason="external_release_reconciled")
        panel.signal_refresh_controls.emit()
    return True


def _pose_info_topic(panel) -> str:
    world_name = panel._gz_world_name or panel._detect_world_name() or GZ_WORLD
    return f"/world/{world_name}/pose/info"


def _pose_info_active(panel) -> bool:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False
    now = time.monotonic()
    if (
        STATUS_TOPIC_CACHE_SEC > 0.0
        and (now - panel._pose_info_active_ts) < STATUS_TOPIC_CACHE_SEC
    ):
        return panel._pose_info_active_cache
    active = panel.ros_worker.topic_has_publishers(panel._pose_info_topic())
    panel._pose_info_active_cache = bool(active)
    panel._pose_info_active_ts = now
    return active


def _pose_info_ready(panel) -> bool:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False
    if not panel._pose_info_active():
        return False
    from .panel_camera import _runtime_time  # lazy: ver nota del módulo
    poses, ts = panel.ros_worker.pose_snapshot()
    if not poses:
        return False
    if ts:
        age = _runtime_time() - ts
        if age > POSE_INFO_MAX_AGE_SEC:
            return False
    return True


def _gazebo_process_signal(panel) -> Tuple[bool, str]:
    """Signal S1: determine if Gazebo process is alive using PID/PGID + fallback scan."""
    if panel._proc_alive(panel.gz_proc):
        pid = int(getattr(panel.gz_proc, "pid", 0) or 0)
        if pid > 0:
            panel._gz_real_pid = pid
            if panel._gz_root_pid <= 0:
                panel._gz_root_pid = pid
            if panel._gz_pgid <= 0:
                try:
                    panel._gz_pgid = int(os.getpgid(pid))
                except Exception:
                    panel._gz_pgid = 0
        return True, "popen"
    if panel._gz_pgid > 0 and psutil is not None:
        try:
            for proc in psutil.process_iter(attrs=["pid", "cmdline", "status"]):
                info = proc.info
                pid = int(info.get("pid") or 0)
                if pid <= 0:
                    continue
                if info.get("status") == psutil.STATUS_ZOMBIE:
                    continue
                try:
                    if int(os.getpgid(pid)) != panel._gz_pgid:
                        continue
                except Exception:
                    continue
                cmdline = info.get("cmdline") or []
                if not cmdline:
                    continue
                joined = " ".join(cmdline).lower()
                if any(token in joined for token in ("gz sim", "gz-sim", "gzserver", "ign gazebo")):
                    panel._gz_real_pid = pid
                    return True, "pgid"
        except Exception as exc:
            _log_exception("gazebo process signal pgid", exc)
    proc_ok, proc_reason = gz_sim_status()
    if proc_ok:
        return True, f"fallback_{proc_reason}"
    return False, proc_reason


__all__ = [
    "_normalize_joint_name",
    "_clock_status",
    "_joint_states_status",
    "_bridge_transport_detected",
    "_bridge_ready_status",
    "_tf_chain_ready_status",
    "_camera_depth_expectation",
    "_camera_runtime_flags",
    "_sync_external_release_state",
    "_pose_info_topic",
    "_pose_info_active",
    "_pose_info_ready",
    "_gazebo_process_signal",
]
