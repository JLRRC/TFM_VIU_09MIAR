#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_state_methods.py
# Contenido: State management and TCP pose helper callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""State management, TCP pose, and system state callbacks for ControlPanelV2."""
from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

from .panel_config import (
    AUTO_START_BRIDGE_MAX_RETRIES,
    BASE_FRAME,
    ROS_AVAILABLE,
)
from .panel_physics import OBJECT_SETTLE_TIMEOUT_SEC
from .panel_state import (
    MoveItState,
    PanelStateEvaluator,
    PanelStateSnapshot,
    SystemState,
)
from .panel_fatal import abort_local_stack
from .tf_pose_utils import get_tcp_in_base as tf_get_tcp_in_base

MOVEIT_POSE_TOPIC = "/desired_grasp"
MOVEIT_CARTESIAN_POSE_TOPIC = "/desired_grasp_cartesian"
GLOBAL_FRAME_EFFECTIVE = "base_link"


def _log_exception(context: str, exc: Exception) -> None:
    print(f"[STATE_METHODS][ERROR][{context}] {exc}")


def _expected_world_frame(panel) -> str:
    return WORLD_FRAME or "world"

def _business_base_frame(panel) -> str:
    current = str(panel._base_frame_effective or "").strip()
    if current and current != GLOBAL_FRAME_EFFECTIVE:
        panel._emit_log(
            f"[FRAME][P0] BASE_FRAME_EFFECTIVE inválido={current}; se exige {GLOBAL_FRAME_EFFECTIVE}"
        )
    return GLOBAL_FRAME_EFFECTIVE

def _base_frame_candidates(panel) -> List[str]:
    return ["base_link"]

def ensure_base_pose(panel,
    pose_stamped_any: Optional[PoseStamped],
    *,
    timeout_sec: float = 0.6,
) -> Optional[PoseStamped]:
    """Convert PoseStamped from any frame into business base frame."""
    if pose_stamped_any is None:
        return None
    base_frame = panel._business_base_frame()
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

def _ensure_base_coords(panel,
    coords: Tuple[float, float, float],
    source_frame: str,
    *,
    timeout_sec: float = 0.6,
) -> Optional[Tuple[float, float, float]]:
    base_frame = panel._business_base_frame()
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

def get_tcp_base(panel) -> Optional[PoseStamped]:
    """Return TCP pose expressed in base_link for diagnostics and tests."""
    base_frame = panel._business_base_frame()
    ee_frame = str(getattr(panel, "_ee_frame_effective", "") or panel._required_ee_frame or "rg2_pinch_center").strip() or "rg2_pinch_center"
    tcp_pose, _rpy_deg, reason = tf_get_tcp_in_base(
        base_frame=base_frame,
        ee_frame=ee_frame,
        timeout=0.30,
        logger=panel._emit_log,
    )
    if tcp_pose is None:
        panel._emit_log(
            "[TF_POSE] tcp_source=tf2 status=UNAVAILABLE "
            f"base={base_frame} ee={ee_frame} reason={reason}"
        )
        return None
    return tcp_pose

def get_tcp_pose_base(panel) -> Optional[PoseStamped]:
    """Compatibility alias: return TCP pose in business base frame."""
    return panel.get_tcp_base()

def transform_pose_to_base(panel,
    pose_stamped_any: Optional[PoseStamped],
    *,
    timeout_sec: float = 0.6,
) -> Optional[PoseStamped]:
    """Compatibility alias: transform any pose into business base frame."""
    return panel.ensure_base_pose(pose_stamped_any, timeout_sec=timeout_sec)

def log_pose_base(panel, tag: str, pose_base: Optional[PoseStamped]) -> None:
    if pose_base is None:
        panel._emit_log(f"{tag} base=unavailable")
        return
    p = pose_base.pose.position
    f = str(pose_base.header.frame_id or panel._business_base_frame())
    panel._emit_log(
        f"{tag} base=({float(p.x):.3f},{float(p.y):.3f},{float(p.z):.3f}) frame={f}"
    )

def log_pose(panel, tag: str, pose_any: Optional[PoseStamped]) -> None:
    """Compatibility alias: log pose in base_link, transforming if needed."""
    pose_base = panel.ensure_base_pose(pose_any) if pose_any is not None else None
    panel.log_pose_base(tag, pose_base)

def get_pose_in_base(panel,
    object_id: str,
    *,
    timeout_sec: float = 0.35,
) -> Optional[PoseStamped]:
    """Resolve object pose into base_link from external/world sources."""
    name = str(object_id or "").strip()
    if not name:
        panel._emit_log("[FRAME_IN] source=pose_info target=base_link object=n/a status=invalid")
        return None
    objects = get_object_positions()
    raw = objects.get(name)
    if raw is None or not isinstance(raw, (list, tuple)) or len(raw) < 3:
        panel._emit_log(
            f"[FRAME_IN] source=pose_info target=base_link object={name} status=not_found"
        )
        return None
    source_frame = panel._world_frame_last_first()
    base_coords = panel._ensure_base_coords(
        (float(raw[0]), float(raw[1]), float(raw[2])),
        source_frame,
        timeout_sec=timeout_sec,
    )
    if base_coords is None:
        panel._emit_log(
            "[FRAME_IN] "
            f"source={source_frame} target={panel._business_base_frame()} "
            f"object={name} status=tf_failed"
        )
        return None
    pose = PoseStamped()
    pose.header.frame_id = panel._business_base_frame()
    pose.pose.position.x = float(base_coords[0])
    pose.pose.position.y = float(base_coords[1])
    pose.pose.position.z = float(base_coords[2])
    pose.pose.orientation.w = 1.0
    panel._emit_log(
        "[FRAME_IN] "
        f"source={source_frame} target={pose.header.frame_id} object={name} status=ok "
        f"base=({pose.pose.position.x:.3f},{pose.pose.position.y:.3f},{pose.pose.position.z:.3f})"
    )
    return pose

def _moveit_publish_context(panel) -> Dict[str, object]:
    return {
        "calibrating": panel._calibrating,
        "calib_grid_until": panel._calib_grid_until,
        "moveit_required": panel._moveit_required,
        "managed_mode": panel._managed_mode,
        "state_ready_moveit": panel._state_ready_moveit(),
        "system_state_reason": panel._system_state_reason or "",
        "system_state_value": panel._system_state.value,
        "moveit_ready": panel._moveit_state == MoveItState.READY,
        "moveit_state_reason": panel._moveit_state_reason or "",
        "moveit_block_reason": panel._moveit_block_reason,
        "moveit_node": panel._moveit_node,
        "moveit_pose_pub": panel._moveit_pose_pub,
        "moveit_pose_pub_cartesian": panel._moveit_pose_pub_cartesian,
        "pose_topic": MOVEIT_POSE_TOPIC,
        "cartesian_topic": MOVEIT_CARTESIAN_POSE_TOPIC,
        "build_pose_stamped": _build_pose_stamped,
        "emit_log": panel._emit_log,
        "set_status": panel._set_status,
        "log_warning": panel._log,
        "logger_info": panel.get_logger().info,
        "log_exception": _log_exception,
    }

def _request_auto_bridge_start(panel) -> None:
    if panel._managed_mode:
        return
    if not AUTO_START_BRIDGE or panel._closing or panel._bridge_running:
        return
    if panel._auto_bridge_timer_scheduled:
        return
    panel._auto_bridge_timer_scheduled = True
    if panel._auto_bridge_attempts == 0:
        panel._log("[AUTO] Autostart bridge: esperando Gazebo")
    QTimer.singleShot(AUTO_START_BRIDGE_DELAY_MS, panel._auto_bridge_tick)

def _auto_bridge_tick(panel) -> None:
    if panel._managed_mode:
        return
    panel._auto_bridge_timer_scheduled = False
    if not AUTO_START_BRIDGE or panel._closing or panel._bridge_running:
        return
    gz_state = panel._gazebo_state()
    if gz_state == "GAZEBO_OFF":
        panel._auto_bridge_attempts += 1
        if panel._auto_bridge_attempts >= AUTO_START_BRIDGE_MAX_RETRIES:
            panel._log("[AUTO] Autostart bridge: timeout (Gazebo no disponible)")
            return
        panel._auto_bridge_timer_scheduled = True
        QTimer.singleShot(1000, panel._auto_bridge_tick)
        return
    panel._log("[AUTO] Autostart bridge: Gazebo activo")
    panel._start_bridge()

def _get_traj_publisher(panel, topic: str):
    panel._ensure_moveit_node()
    if panel._moveit_node is None:
        return None
    panel._traj_pub, panel._traj_topic = get_traj_publisher(
        panel._moveit_node,
        panel._traj_pub,
        panel._traj_topic,
        topic,
        panel._log,
    )
    return panel._traj_pub

def _get_gripper_publisher(panel, topic: str):
    panel._ensure_moveit_node()
    if panel._moveit_node is None:
        return None
    panel._gripper_pub, panel._gripper_topic = get_gripper_publisher(
        panel._moveit_node,
        panel._gripper_pub,
        panel._gripper_topic,
        topic,
        panel._log,
    )
    return panel._gripper_pub

def _get_attach_publisher(panel, topic: str):
    panel._ensure_moveit_node()
    if panel._moveit_node is None:
        return None
    return get_attach_publisher(
        panel._moveit_node,
        panel._attach_pubs,
        topic,
        panel._log,
    )

@staticmethod
def _normalize_attach_name(name: str) -> str:
    if name == "pick_demo":
        return "pick_demo"
    return name

def _find_attach_candidate(panel, *args, **kwargs):
    return _gz._find_attach_candidate(panel, *args, **kwargs)

def _attempt_attach(panel, *args, **kwargs):
    return _gz._attempt_attach(panel, *args, **kwargs)

def _schedule_attach_attempt(panel, *args, **kwargs):
    _gz._schedule_attach_attempt(panel, *args, **kwargs)



def _command_gripper(panel, closed: bool, *, log_action: str = 'Gripper', force: bool = False) -> bool:
    return command_gripper(panel, closed, log_action=log_action, force=force)

def _command_gripper_preopen(panel, rad: float, *, log_action: str = 'Gripper') -> bool:
    return command_gripper_preopen(panel, rad, log_action=log_action)

def _traj_action_target(panel, traj_topic: str) -> str:
    return traj_action_target(panel, traj_topic)

def _resolve_traj_action_name(panel, traj_topic: str, *, allow_fallback: bool = False) -> str:
    return resolve_traj_action_name(panel, traj_topic, allow_fallback=allow_fallback)

def _get_action_client(panel, action_name: str, action_type, *, log_ctx: str = ''):
    return get_action_client(panel, action_name, action_type, log_ctx=log_ctx)

@staticmethod
def _wait_action_server(client, *, timeout_sec: float, log_ctx: str, action_name: str) -> bool:
    return wait_action_server(client, timeout_sec=timeout_sec, log_ctx=log_ctx, action_name=action_name)

def _format_action_error(panel, kind: str, detail: str) -> str:
    return format_action_error(panel, kind, detail)

def _joint_motion_since(panel, snapshot) -> bool:
    return joint_motion_since(panel, snapshot)

def _wait_for_joint_target(panel, *args, **kwargs):
    return wait_for_joint_target(panel, *args, **kwargs)

def _wait_for_tcp_base_z(panel, *args, **kwargs):
    return wait_for_tcp_base_z(panel, *args, **kwargs)

def _wait_for_tcp_base_target(panel, *args, **kwargs):
    return wait_for_tcp_base_target(panel, *args, **kwargs)

def _send_joint_trajectory_action(panel, positions, sec: float, traj_topic: str):
    return send_joint_trajectory_action(panel, positions, sec, traj_topic)

def _schedule_traj_action_fallback(panel, positions, sec: float, traj_topic: str) -> None:
    schedule_traj_action_fallback(panel, positions, sec, traj_topic)

def _clamp_joint_positions(panel, positions):
    return clamp_joint_positions(panel, positions)

def _publish_joint_trajectory(panel, *args, **kwargs):
    return publish_joint_trajectory(panel, *args, **kwargs)

def _log_traj_action_fallback(panel, ok: bool, info: str) -> None:
    log_traj_action_fallback(panel, ok, info)

def _publish_moveit_pose(panel, *args, **kwargs):
    return publish_moveit_pose(panel, *args, **kwargs)

def _start_objects_settle_watch(panel) -> None:
    panel._physics.start_objects_settle_watch()

def _invalidate_settle(panel, reason: str, *, restart: bool = True) -> None:
    panel._physics.invalidate_settle(reason, restart=restart)

def _run_fall_test_async(panel) -> None:
    panel._physics.run_fall_test_async()

def _objects_settle_worker(panel) -> None:
    panel._physics.objects_settle_worker()

def _handle_objects_settled(panel) -> None:
    panel._refresh_objects_from_gz()
    recalc_object_states("settled")
    panel._refresh_controls()
    if panel._bridge_running and not panel._calibration_ready:
        panel.signal_calibration_check.emit()

def _log_calib_blocked(panel, reason: str) -> None:
    now = time.time()
    if (now - panel._last_calib_block_log) < 1.5:
        return
    panel._last_calib_block_log = now
    panel._emit_log(f"[CALIB] Bloqueada: {reason}")

def _log_settle_snapshot(panel, reason: str) -> None:
    panel._physics.log_settle_snapshot(reason)

def _request_settle_snapshot(panel, reason: str) -> None:
    panel._physics.request_settle_snapshot(reason)

def wait_for_objects_to_settle(panel,
    timeout: float = OBJECT_SETTLE_TIMEOUT_SEC,
    *,
    log_snapshot: bool = False,
) -> bool:
    return panel._physics.wait_for_objects_to_settle(timeout=timeout, log_snapshot=log_snapshot)

def _build_ui(panel) -> None:
    build_main_ui(panel)

def _debounced_btn_action(panel, btn, action, delay_ms=1200):
    if not btn.isEnabled():
        return
    btn.setEnabled(False)
    try:
        action()
    finally:
        QTimer.singleShot(delay_ms, lambda: btn.setEnabled(True))

def showEvent(panel, event):
    super().showEvent(event)
    if not panel._timers_started:
        panel.status_timer.start(2500)
        QTimer.singleShot(1200, panel._run_startup_tf_sanity_check_once)
        panel._timers_started = True

def _set_status(panel, text: str, error: bool = False):
    if getattr(panel, "_last_status_text", None) == text and getattr(panel, "_last_status_error", None) == error:
        return
    panel._last_status_text = text
    panel._last_status_error = error
    color = "#dc2626" if error else "#0f172a"
    panel.status_lbl.setText(text)
    panel.status_lbl.setStyleSheet(
        f"background:{color}; color:white; padding:8px 14px; border-radius:12px; font-weight:600;"
    )
    # SIEMPRE loguear errores; info solo si debug está activo
    if error:
        panel._emit_log(f"[ERROR] {text}")
    elif panel._debug_logs_enabled:
        panel._emit_log(f"[INFO] {text}")

def _set_status_async(panel, text: str, error: bool = False) -> None:
    panel._set_status(text, error=error)

def _set_led_async(panel, led_obj: object, state: str) -> None:
    set_led(led_obj, state)

def _on_tf_ready_signal(panel, ready: bool) -> None:
    panel._tf_ready_state = ready
    if not ready:
        panel._trace_ready = False
    elif panel._metrics_enabled and not panel._tf_ever_ok:
        panel._metric_mark("tf_ready")
    panel._refresh_controls()

def _on_calib_ready_signal(panel, ready: bool) -> None:
    panel._calibration_ready = ready
    panel._refresh_controls()

def _run_startup_tf_sanity_check_once(panel) -> None:
    if panel._closing:
        return
    ok, reason = panel._tf_sanity_check()
    if ok:
        panel._tf_ready_state = True
        panel._tf_ever_ok = True
        panel._last_tf_ok_monotonic = time.monotonic()
        panel.signal_refresh_controls.emit()
        panel._emit_log(f"[STARTUP][TF_SANITY] OK {reason}")
        return
    msg = f"TF pendiente: {reason}"
    panel._set_status(msg, error=False)
    panel._emit_log(f"[STARTUP][TF_SANITY] WAIT {reason} (bloqueando TEST/PICK hasta TF EE)")

def _on_controllers_ready_signal(panel, ready: bool) -> None:
    panel._controllers_ok = ready
    panel._refresh_controls()

def _on_error_signal(panel, msg: str) -> None:
    panel._system_error_reason = msg
    panel._set_system_state(SystemState.ERROR, msg)
    panel._set_status(msg, error=True)
    panel._refresh_controls()

def _on_moveit_state_signal(panel, state: str, reason: str) -> None:
    try:
        state_enum = MoveItState[state]
    except KeyError:
        return
    panel._moveit_state = state_enum
    panel._moveit_state_reason = reason or state_enum.value
    if state_enum == MoveItState.OFF:
        set_led(panel.led_moveit, "off")
    elif state_enum == MoveItState.STARTING:
        set_led(panel.led_moveit, "warn")
    elif state_enum == MoveItState.WAITING_MOVEIT_READY:
        set_led(panel.led_moveit, "warn")
    elif state_enum == MoveItState.READY:
        set_led(panel.led_moveit, "on")
    else:
        set_led(panel.led_moveit, "error")
        if reason:
            panel._set_status(f"MoveIt error: {reason}", error=True)
    if reason and panel._debug_logs_enabled:
        panel._emit_log(f"[MOVEIT] {state_enum.value}: {reason}")
    panel._update_moveit_status_label()
    panel._refresh_controls()

def _on_trace_ready(panel) -> None:
    if panel._trace_ready:
        return
    panel._trace_ready = True
    panel._bridge_ready = True
    panel._stop_tf_ready_timer()
    panel._log("[TRACE] TF ready → TRACE enabled")
    panel._refresh_trace_data()

def _on_calibration_check(panel) -> None:
    if panel._calibration_ready:
        return
    if not panel._objects_settled and not ALLOW_UNSETTLED_ON_TIMEOUT:
        panel._log_calib_blocked("esperando caída/estabilidad de objetos")
        return
    if not panel._pose_info_ok:
        panel._log_calib_blocked("pose/info no disponible")
        return
    if not panel._tf_ready_state:
        panel._log_calib_blocked(panel._tf_not_ready_reason())
        return
    if panel._camera_required and not panel._camera_stream_ok:
        panel._log_calib_blocked("cámara no publica")
        return
    panel._log("[CALIB] Inicializando calibración tras bridge")
    panel._load_table_calibration()
    panel._refresh_objects_from_gz_async()
    QTimer.singleShot(1500, panel._refresh_objects_from_gz_async)
    QTimer.singleShot(4000, panel._refresh_objects_from_gz_async)
    panel._calibration_ready = True
    panel.signal_calib_ready.emit(True)

def _set_system_state(panel, state: SystemState, reason: str) -> None:
    if panel._system_state == state and panel._system_state_reason == reason:
        return
    panel._system_state = state
    panel._system_state_reason = reason
    panel._emit_log(f"[STATE] {state.value} ({reason})")
    if panel._metrics_enabled:
        panel._metric_mark(f"state:{state.value}")

def _effective_system_state(panel) -> Tuple[SystemState, str]:
    if panel._managed_mode:
        if panel._external_state_active():
            return panel._system_state, panel._system_state_reason
        return SystemState.WAITING_GAZEBO, "Esperando /system_state"
    return panel._resolve_system_state()

def _trigger_fatal(panel, reason: str) -> None:
    if panel._fatal_latched:
        return
    panel._fatal_latched = True
    panel._system_error_reason = reason
    panel._set_system_state(SystemState.ERROR_FATAL, reason)
    panel._ui_set_status(f"ERROR_FATAL: {reason}", error=True)
    panel._emit_log(f"[ERROR_FATAL] {reason}")
    panel._emit_log("[ERROR_FATAL] Panel mantiene UI abierta para diagnostico")
    if panel._metrics_enabled:
        panel._metric_mark("fatal")
    if panel._diagnostic_mode or not panel._fatal_stops_all:
        panel._emit_log("[ERROR_FATAL] Modo diagnostico activo: no se ejecuta stop_all")
        return
    if not panel._managed_mode and not panel._fatal_shutdown_started:
        panel._fatal_shutdown_started = True
        panel._emit_log("[ERROR_FATAL] Abortando stack local (stop_all)")
        abort_local_stack(panel)

def _resolve_system_state(panel) -> Tuple[SystemState, str]:
    if panel._system_error_reason:
        if panel._system_error_reason.startswith("drop:"):
            snapshot = panel._build_state_snapshot()
            recovered_state, recovered_reason = panel._state_evaluator.resolve(
                snapshot,
                tf_reason=panel._tf_not_ready_reason(),
            )
            if recovered_state in (
                SystemState.READY_BASIC,
                SystemState.READY_VISION,
                SystemState.READY_MOVEIT,
            ):
                panel._emit_log(
                    f"[STATE] Recover from transient drop -> {recovered_state.value} ({recovered_reason})"
            )
                panel._system_error_reason = ""
            else:
                return SystemState.ERROR, panel._system_error_reason
        else:
            return SystemState.ERROR, panel._system_error_reason
    if panel._camera_fault_active:
        return SystemState.ERROR, panel._camera_fault_reason or "camera_fault persistente"
    snapshot = panel._build_state_snapshot()
    return panel._state_evaluator.resolve(snapshot, tf_reason=panel._tf_not_ready_reason())

def _build_state_snapshot(panel) -> PanelStateSnapshot:
    return PanelStateSnapshot(
        gazebo_state=panel._gazebo_state(),
        controllers_ok=panel._controllers_ok,
        controllers_reason=panel._controllers_reason or "",
        tf_ready=panel._tf_ready_state,
        bridge_running=panel._bridge_running,
        camera_required=panel._camera_required,
        camera_stream_ok=panel._camera_stream_ok,
        pose_info_ok=panel._pose_info_ok,
        calibration_ready=panel._calibration_ready,
        objects_settled=panel._objects_settled,
        moveit_required=panel._moveit_required,
        moveit_state=panel._moveit_state,
        moveit_state_reason=panel._moveit_state_reason or "",
    )

def _evaluate_system_state(panel) -> None:
    next_state, next_reason = panel._resolve_system_state()
    prev_state = panel._system_state
    if prev_state in (SystemState.READY_VISION, SystemState.READY_MOVEIT) and next_state != prev_state:
        if next_state in (SystemState.READY_VISION, SystemState.READY_MOVEIT):
            panel._set_system_state(next_state, next_reason)
            return
        if next_state == SystemState.READY_BASIC:
            panel._set_system_state(next_state, next_reason)
            return
        if next_state == SystemState.WAITING_CONTROLLERS and not panel._controllers_ok and panel._controller_drop_grace_active():
            panel._emit_log(f"[WARN] drop a {next_state.value} suprimido: controladores en gracia ({next_reason})")
            panel._set_system_state(next_state, next_reason)
            return
        panel._system_error_reason = f"drop: {next_reason}"
        panel._set_system_state(SystemState.ERROR, panel._system_error_reason)
    else:
        panel._set_system_state(next_state, next_reason)

def _update_system_state(panel) -> None:
    if panel._managed_mode:
        decision = panel._state_machine.decide_external(panel)
        if decision.fatal_reason:
            panel._trigger_fatal(decision.fatal_reason)
            return
        panel._set_system_state(decision.state, decision.reason)
        panel._sync_moveit_from_system_state()
        return
    panel._evaluate_system_state()

def _check_critical_timeouts(panel) -> None:
    reason = panel._watchdog.check()
    if reason:
        panel._trigger_fatal(reason)
        return

def _resolve_critical_fault(panel, now: float, clock_ok: bool) -> str:
    # Deprecated: moved to PanelWatchdog. Kept for compatibility.
    if panel._watchdog is None:
        return ""
    reason = panel._watchdog._resolve_critical_fault(now, clock_ok)  # type: ignore[attr-defined]
    return reason or ""

def _state_ready_basic(panel) -> bool:
    return panel._state_ready_level("basic")

def _state_ready_vision(panel) -> bool:
    return panel._state_ready_level("vision")

def _state_ready_moveit(panel) -> bool:
    return panel._state_ready_level("moveit")

def _state_ready_level(panel, level: str) -> bool:
    state, _reason = panel._effective_system_state()
    return PanelStateEvaluator.is_ready_level(state, level)

def _manual_control_ready(panel) -> bool:
    if panel._managed_mode:
        return panel._state_ready_basic()
    return panel._gazebo_state() == "GAZEBO_READY" and panel._controllers_ok

def _calibration_topic_allowed(panel) -> bool:
    topic = panel.camera_topic
    return topic == CALIBRATION_CAMERA_TOPIC

def _overhead_camera_active(panel, topic: str) -> bool:
    return topic == CALIBRATION_CAMERA_TOPIC
