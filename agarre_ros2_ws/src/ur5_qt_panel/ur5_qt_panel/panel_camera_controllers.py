#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_camera_controllers.py
# Contenido: Camera and controller callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Camera, controller, and joint state callbacks for ControlPanelV2."""
from __future__ import annotations

import os
import time
from typing import Dict, List, Optional, Tuple

from .panel_config import (
    CAMERA_INIT_GRACE_SEC,
    CONTROLLER_READY_CACHE_SEC,
    CRITICAL_POSE_TIMEOUT_SEC,
    JOINT_SLIDER_SCALE,
    ROS_AVAILABLE,
    TF_INIT_GRACE_SEC,
    UR5_JOINT_NAMES,
)
from .panel_utils import (
    list_controllers_state,
    rclpy,
)


def _log_exception(context: str, exc: Exception) -> None:
    print(f"[CAM_CTRL][ERROR][{context}] {exc}")


def _controllers_ready(panel) -> Tuple[bool, str]:
    now = time.time()
    gz_state = panel._gazebo_state()
    proc_ok, proc_reason = panel._gazebo_process_signal()
    clock_ok, clock_reason = panel._clock_status()
    clock_graph_fallback = ""
    if not clock_ok:
        graph_clock_ok, graph_clock_reason = graph_clock_status()
        if graph_clock_ok and str(clock_reason).startswith("age="):
            clock_ok = True
            clock_graph_fallback = str(graph_clock_reason or "graph")
            panel._emit_log_throttled(
                "ctrl_gate_clock_graph_fallback",
                "[CTRL_GATE] worker /clock stale; accepting graph fallback "
                f"state={gz_state} worker={clock_reason} graph={clock_graph_fallback}",
                min_interval=2.0,
            )
    if not proc_ok:
        return False, f"gazebo_not_ready state={gz_state} process={proc_reason}"
    if not clock_ok:
        return False, f"gazebo_not_ready state={gz_state} clock={clock_reason}"
    gazebo_motion_degraded = gz_state == "GAZEBO_DEGRADED"
    if gz_state in ("GAZEBO_OFF", "GAZEBO_STARTING", "GAZEBO_MONITOR_BUG"):
        return False, f"gazebo_not_ready state={gz_state}"
    if (
        CONTROLLER_READY_CACHE_SEC > 0.0
        and panel._controllers_ok
        and (now - panel._last_controller_check) < CONTROLLER_READY_CACHE_SEC
    ):
        if gazebo_motion_degraded:
            return True, "cached gazebo_degraded_motion_ok"
        return True, "cached"
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False, "nodo ROS no listo"
    cm_path = panel._controller_manager_path()
    if not cm_path:
        return False, panel._controller_manager_not_ready_reason()
    list_srv = f"{cm_path}/list_controllers"
    if not panel.ros_worker.has_service(list_srv):
        reason = panel._list_controllers_not_ready_reason("service")
        stale_ok, age = panel._can_use_controller_last_ok(reason)
        if stale_ok:
            panel._emit_log_throttled(
                "ctrl_gate_stale_service",
                f"[CTRL_GATE] {reason}; using last_ok_age={age:.2f}s",
                min_interval=1.0,
            )
            return True, f"last_ok_stale age={age:.2f}s ({reason})"
        return False, reason
    resp, err = panel._list_controllers(list_srv)
    if resp is None and panel._is_transient_controller_reason(err):
        deadline = time.monotonic() + CONTROLLER_LIST_RETRY_WINDOW_SEC
        while resp is None and time.monotonic() < deadline:
            time.sleep(CONTROLLER_LIST_RETRY_STEP_SEC)
            resp, err = panel._list_controllers(list_srv)
    state_map: Dict[str, str] = {}
    controller_source = "client"
    if resp is None:
        fallback_states, fallback_err = list_controllers_state(controller_manager=cm_path)
        if fallback_states:
            state_map = {str(name): str(state) for name, state in fallback_states.items()}
            controller_source = "graph_probe"
            panel._emit_log_throttled(
                "ctrl_gate_graph_probe_ok",
                f"[CTRL_GATE] fallback list_controllers ok source={controller_source} count={len(state_map)}",
                min_interval=1.0,
            )
        else:
            reason = err or fallback_err or "no response"
            stale_ok, age = panel._can_use_controller_last_ok(reason)
            if stale_ok:
                panel._emit_log_throttled(
                    "ctrl_gate_stale_timeout",
                    f"[CTRL_GATE] list_controllers timeout; using last_ok_age={age:.2f}s ({reason})",
                    min_interval=1.0,
                )
                return True, f"last_ok_stale age={age:.2f}s ({reason})"
            return False, reason
    else:
        state_map = {str(c.name): str(c.state) for c in resp.controller}
    panel._controller_state_map = dict(state_map)
    panel._controller_state_source = controller_source
    panel._controller_state_ts = time.time()
    required = ["joint_state_broadcaster", "joint_trajectory_controller"]
    if gripper_controller_defined():
        required.append("gripper_controller")

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
        kind = panel._controller_state_kind(state)
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
        panel._last_controller_check = now
        return False, "controllers not active (" + " | ".join(detail) + ")"
    strict_action = str(os.environ.get("PANEL_STRICT_TRAJ_ACTION", "1")).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    if strict_action and not panel._follow_joint_traj_ready():
        expected = str(
            os.environ.get(
                "PANEL_EXPECTED_TRAJ_ACTION",
                "/joint_trajectory_controller/follow_joint_trajectory",
            )
        ).strip()
        panel._last_controller_check = now
        return False, f"follow_joint_traj_not_ready expected={expected}"
    panel._controllers_last_ok_ts = now
    panel._last_controller_check = now
    if gazebo_motion_degraded:
        panel._emit_log_throttled(
            "ctrl_gate_gazebo_degraded_motion_ok",
            "[CTRL_GATE] accepting motion readiness with Gazebo=GAZEBO_DEGRADED "
            f"(process={proc_reason} clock={clock_reason}"
            f"{' graph=' + clock_graph_fallback if clock_graph_fallback else ''})",
            min_interval=2.0,
        )
        return True, (
            f"controllers activos source={controller_source} "
            f"gazebo_degraded_motion_ok process={proc_reason} clock={clock_reason}"
            f"{' graph=' + clock_graph_fallback if clock_graph_fallback else ''}"
        )
    return (
        True,
        f"controllers activos source={controller_source}"
        f"{' clock_graph_fallback=' + clock_graph_fallback if clock_graph_fallback else ''}",
    )

def _is_transient_controller_reason(panel, reason: Optional[str]) -> bool:
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

def _controllers_last_ok_age(panel, now: Optional[float] = None) -> float:
    ts = float(panel._controllers_last_ok_ts or 0.0)
    if ts <= 0.0:
        return float("inf")
    ref = time.time() if now is None else float(now)
    return max(0.0, ref - ts)

def _can_use_controller_last_ok(panel, reason: Optional[str]) -> Tuple[bool, float]:
    if not panel._is_transient_controller_reason(reason):
        return False, float("inf")
    if CONTROLLER_LAST_OK_GRACE_SEC <= 0.0:
        return False, float("inf")
    age = panel._controllers_last_ok_age()
    return age <= CONTROLLER_LAST_OK_GRACE_SEC, age

def _controller_state_kind(panel, state: str) -> str:
    state_lc = (state or "").strip().lower()
    if state_lc == "active":
        return "ACTIVE"
    if state_lc == "inactive":
        return "INACTIVE"
    if state_lc in ("unconfigured", "finalized"):
        return "LOADED"
    return "UNKNOWN"

def _list_controllers(panel, service_name: str) -> Tuple[Optional["ListControllers.Response"], Optional[str]]:
    if ListControllers is None:
        return None, "ListControllers no disponible"
    if panel._moveit_node is None:
        return None, panel._ros_node_not_ready_reason()
    if not service_name:
        return None, "service vacío"
    # El gate canónico no debe depender del nodo del servicio remoto mientras
    # /panel/tfm_execute está bloqueado esperando el resultado del pick.
    client_node = panel._moveit_node
    client_node_id = id(client_node)
    if (
        panel._controller_client is None
        or panel._controller_client_name != service_name
        or getattr(panel, "_controller_client_node_id", None) != client_node_id
    ):
        try:
            panel._controller_client = client_node.create_client(ListControllers, service_name)
            panel._controller_client_name = service_name
            panel._controller_client_node_id = client_node_id
        except Exception as exc:
            panel._controller_client = None
            panel._controller_client_name = ""
            panel._controller_client_node_id = None
            return None, f"client error: {exc}"
    client = panel._controller_client
    if client is None:
        return None, panel._list_controllers_not_ready_reason("client")
    node_ns = panel.ros_worker.node_namespace() if panel._ros_worker_started else ""
    # FASE 2: Timeouts aumentados para evitar falsos negativos.
    wait_timeout = 3.0
    call_timeout = 3.0
    # FASE 2: Hasta 3 intentos con backoff exponencial.
    max_retries = 3
    last_err = None
    for attempt in range(1, max_retries + 1):
        if not client.wait_for_service(timeout_sec=wait_timeout):
            last_err = (
                f"{panel._list_controllers_not_ready_reason('timeout')} "
                f"service={service_name} ns={node_ns or '/'} timeout={wait_timeout:.1f}s "
                f"gz={panel._gazebo_state()} attempt={attempt}/{max_retries}"
            )
            if attempt < max_retries:
                time.sleep(0.5 * attempt)
                continue
            return None, last_err
        future = client.call_async(ListControllers.Request())
        if client_node is panel._moveit_node:
            try:
                rclpy.spin_until_future_complete(
                    panel._moveit_node,
                    future,
                    timeout_sec=call_timeout,
                )
            except RuntimeError as exc:
                if "already spinning" not in str(exc):
                    raise
                deadline = time.monotonic() + call_timeout
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
        else:
            deadline = time.monotonic() + call_timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
        if not future.done() or future.result() is None:
            last_err = (
                f"{panel._list_controllers_not_ready_reason('no response')} "
                f"service={service_name} ns={node_ns or '/'} timeout={call_timeout:.1f}s "
                f"gz={panel._gazebo_state()} attempt={attempt}/{max_retries}"
            )
            if attempt < max_retries:
                time.sleep(0.5 * attempt)
                continue
            return None, last_err
        return future.result(), None
    return None, last_err

def _wait_for_controllers_ready(panel, timeout_sec: float) -> Tuple[bool, str]:
    deadline = time.monotonic() + max(0.1, timeout_sec)
    last_reason = "controladores no listos"
    while time.monotonic() < deadline:
        ok, reason = panel._controllers_ready()
        last_reason = reason
        if ok:
            return True, reason
        panel._wait_for_state_change(0.15)
    return False, last_reason

def _schedule_camera_health_check(panel, delay_ms: int = 1800) -> None:
    panel._camera_ctrl.schedule_health_check(delay_ms)

def _check_camera_topic_health(panel):
    panel._camera_ctrl.check_topic_health()

def _update_camera_topics(panel, topics):
    panel._camera_ctrl.update_topics(topics)

def _connect_camera(panel):
    panel._camera_ctrl.connect()

def _switch_camera_topic(panel, topic_candidates: List[str], *, label: str) -> bool:
    if not topic_candidates:
        return False
    combo = getattr(panel, "camera_topic_combo", None)
    if combo is None:
        return False
    normalized = [str(t).strip() for t in topic_candidates if str(t).strip()]
    if not normalized:
        return False

    selected = ""
    selected_idx = -1
    for topic in normalized:
        idx = combo.findText(topic)
        if idx >= 0:
            selected = topic
            selected_idx = idx
            break
    if not selected:
        selected = normalized[0]

    if selected_idx >= 0:
        combo.setCurrentIndex(selected_idx)
    else:
        combo.setEditText(selected)

    panel.camera_topic = selected
    panel._emit_log(f"[CAMERA] preset={label} topic={selected}")
    panel._camera_ctrl.connect()
    return True

def _set_far_front_camera_view(panel) -> None:
    ok = panel._switch_camera_topic(
        list(FAR_FRONT_CAMERA_TOPIC_CANDIDATES),
        label="frontal_lejana",
    )
    if ok:
        panel._set_status("Cámara: vista frontal lejana", error=False)
    else:
        panel._set_status("No se pudo activar vista frontal", error=True)

def _set_top_camera_view(panel) -> None:
    ok = panel._switch_camera_topic(
        list(TOP_CAMERA_TOPIC_CANDIDATES),
        label="cenital",
    )
    if ok:
        panel._set_status("Cámara: vista cenital", error=False)
    else:
        panel._set_status("No se pudo activar vista cenital", error=True)

def _set_wrist_camera_view(panel) -> None:
    ok = panel._switch_camera_topic(
        list(WRIST_CAMERA_TOPIC_CANDIDATES),
        label="muñeca",
    )
    if ok:
        panel._set_status("Cámara: vista muñeca", error=False)
    else:
        panel._set_status("No se pudo activar vista muñeca", error=True)

def _subscribe_camera(panel, topic: str) -> bool:
    return panel._camera_ctrl.subscribe(topic)

def _start_camera_health_check(panel, delay_ms: int = 1200) -> None:
    panel._camera_ctrl.start_health_check(delay_ms)

def _unsubscribe_camera(panel) -> None:
    panel._camera_ctrl.unsubscribe()

def _clear_camera_frame(panel, *, reset_info: bool = True) -> None:
    panel._camera_ctrl.clear_frame(reset_info=reset_info)

def _resolve_camera_msg_type(panel, topic: str) -> str:
    return panel._camera_ctrl.resolve_msg_type(topic)

def _auto_connect_camera(panel):
    """Auto-conectar cámara al iniciar el panel."""
    if panel._closing:
        panel._emit_log("[CAMERA] auto_connect omitido: panel cerrando")
        return
    if not panel._camera_required:
        panel._emit_log("[CAMERA] auto_connect omitido: camera_required=false")
        return
    panel._camera_ctrl.auto_connect()

def _ensure_ros_worker_started(panel):
    """Lazy start the RosWorker when the bridge flow begins."""
    if panel._ros_worker_started:
        return
    panel.ros_worker.start()
    panel._ros_worker_started = True
    panel._emit_log("[STARTUP] RosWorker iniciado")

def _ensure_grasp_rect_subscription(panel) -> None:
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return
    if panel._grasp_rect_subscribed:
        return
    try:
        ok = bool(panel.ros_worker.subscribe_grasp_rect(panel._grasp_rect_topic))
    except Exception as exc:
        panel._emit_log(f"[TFM] WARN: subscribe /grasp_rect falló ({exc})")
        return
    if ok:
        panel._grasp_rect_subscribed = True
        panel._emit_log(f"[TFM] Suscripción activa: {panel._grasp_rect_topic}")

def _auto_subscribe_joints(panel):
    if not ROS_AVAILABLE:
        return
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    if not panel.ros_worker.node_ready():
        return
    if panel._joint_subscribed:
        return
    try:
        topic, found = panel._discover_joint_states_topic(panel.joint_topic)
        panel.joint_topic = topic
        if topic and topic != panel._joint_current_topic:
            panel.ros_worker.subscribe_joint_states(topic)
            panel._joint_current_topic = topic
        if found:
            panel.lbl_joint_states.setText(f"Joint states: suscrito {topic}")
        if panel._last_joint_stamp:
            panel._joint_active = True
        if panel._joint_active and hasattr(panel, "joint_timer") and panel.joint_timer.isActive():
            panel._joint_subscribed = True
            panel.joint_timer.stop()
    except Exception as exc:
        panel._log_warning(f"No se pudo suscribir a joint_states: {exc}")

def _discover_joint_states_topic(panel, preferred: str) -> Tuple[str, bool]:
    topic = (preferred or "").strip() or "/joint_states"
    if not panel.ros_worker.node_ready():
        return topic, False
    try:
        topics = panel.ros_worker.list_topic_names()
    except Exception as exc:
        _log_exception("discover joint_states topics", exc)
        return topic, False
    if topic in topics and panel.ros_worker.topic_has_publishers(topic):
        return topic, True
    # fallback: cualquier tópico que termine en /joint_states
    for candidate in sorted(t for t in topics if t.endswith("/joint_states")):
        if panel.ros_worker.topic_has_publishers(candidate):
            return candidate, True
    return topic, False

def _on_bridge_ready(panel):
    panel._ensure_pose_subscription()
    panel._ensure_grasp_rect_subscription()
    panel._start_pose_info_watch()
    panel._start_tf_ready_timer()
    now = time.monotonic()
    panel._critical_pose_deadline = now + max(0.1, CRITICAL_POSE_TIMEOUT_SEC)
    panel._critical_tf_deadline = now + max(0.1, TF_INIT_GRACE_SEC)
    if panel._camera_required:
        panel._critical_camera_deadline = now + max(0.1, CAMERA_INIT_GRACE_SEC)
    panel._auto_subscribe_joints()
    panel._invalidate_settle("bridge activado", restart=True)
    QTimer.singleShot(300, panel._refresh_camera_topics)
    QTimer.singleShot(600, panel._auto_connect_camera)
    if panel._auto_release_drop_objects:
        QTimer.singleShot(250, panel._auto_release_drop_objects_when_ready)
    else:
        QTimer.singleShot(600, lambda: panel._maybe_hold_drop_objects("bridge"))
        QTimer.singleShot(700, lambda: panel._attach_drop_objects("bridge"))
    panel._check_camera_topic_health()
    panel.signal_calibration_check.emit()
    try:
        if panel._gz_running:
            panel._refresh_objects_from_gz_async()
            QTimer.singleShot(1500, panel._refresh_objects_from_gz_async)
        QTimer.singleShot(1500, lambda: panel._apply_home_joint2_offset(retries=3))
        panel._detach_attempted = False
        panel._detach_inflight = False
        panel._detach_auto_disabled = False
        panel._detach_backoff_until = 0.0
        panel._drop_nudge_done = False
        if panel._gz_running and not panel._objects_release_done:
            if panel._auto_release_drop_objects:
                panel._emit_log("[PHYSICS] Auto-release DROP activo; liberando objetos.")
            else:
                panel._emit_log("[PHYSICS] Objetos en espera: auto-release DROP desactivado.")
        if panel._gz_running:
            QTimer.singleShot(900, panel._run_fall_test_async)
    except Exception as exc:
        panel._log_error(f"[BRIDGE] on_ready error: {exc}")

def _on_image(panel, topic: str, qimg, w: int, h: int, fps: float):
    panel._camera_ctrl.on_image(topic, qimg, w, h, fps)

def _on_grasp_rect(panel, payload: object) -> None:
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
    panel._last_grasp_px = {
        "cx": cx,
        "cy": cy,
        "w": gw,
        "h": gh,
        "angle_deg": angle_deg,
    }
    panel._last_grasp_source = f"topic:{payload.get('topic') or panel._grasp_rect_topic}"
    panel._last_grasp_frame = panel.camera_topic or "image"
    panel._last_grasp_update_ts = _runtime_time()
    panel._last_grasp_selection_name = str(getattr(panel, "_selected_object", "") or "").strip()
    frame_w = frame_h = 0
    if panel._last_camera_frame:
        _qimg, frame_w, frame_h, _ts = panel._last_camera_frame
    if frame_w > 0 and frame_h > 0:
        panel._last_grasp_world = panel._compute_world_grasp(frame_w, frame_h)
        panel._last_grasp_base = panel._world_grasp_to_base(panel._last_grasp_world)
        ref = panel._build_reference_grasp(frame_w, frame_h)
        if ref and panel._last_grasp_px:
            panel._update_cornell_metrics(panel._last_grasp_px, ref)
        else:
            panel._last_cornell = None
        with panel._camera_frame_lock:
            if panel._last_camera_frame:
                qimg, w, h, ts = panel._last_camera_frame
                panel._camera_pending_frame = (
                    panel.camera_topic or panel._last_grasp_frame or "image",
                    qimg,
                    w,
                    h,
                    panel._camera_last_fps,
                    ts,
            )
        panel._refresh_camera_display()
    else:
        panel._last_grasp_world = None
        panel._last_grasp_base = None
        panel._last_cornell = None
    panel._refresh_science_ui()
    panel._set_status("TFM: /grasp_rect recibido", error=False)
    panel._audit_append(
        "logs/perception.log",
        "[TFM] grasp_rect rx "
        f"topic={payload.get('topic') or panel._grasp_rect_topic} "
        f"type={payload.get('msg_type') or 'unknown'} "
        f"cx={cx:.1f} cy={cy:.1f} w={gw:.1f} h={gh:.1f} theta={angle_deg:.2f}deg "
        f"base={panel._last_grasp_base} source={panel._last_grasp_source}",
    )
    panel._audit_write_json(
        "artifacts/grasp_rect_last.json",
        {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "source": panel._last_grasp_source,
            "topic": payload.get("topic") or panel._grasp_rect_topic,
            "msg_type": payload.get("msg_type"),
            "grasp_image": panel._last_grasp_px,
            "grasp_base": panel._last_grasp_base,
            "frame": panel._last_grasp_frame,
            "cornell": panel._last_cornell,
        },
    )

def _reset_camera_retry_backoff(panel):
    panel._camera_reconnect_attempts = 0
    panel._camera_reconnect_last_reason = ""
    panel._camera_reconnect_scheduled = False

def _refresh_camera_display(panel):
    panel._camera_ctrl.refresh_display()

def _check_camera_stream(panel):
    return
def _on_joint_state(panel, payload: Dict[str, object]):
    if not payload:
        return
    topic = payload.get("topic") or ""
    if topic:
        panel._joint_current_topic = topic

    names = payload.get("name") or []
    pos_list = payload.get("position") or []
    vel_list = payload.get("velocity") or []
    eff_list = payload.get("effort") or []
    source = payload.get("source") or "ros"
    stamp = float(payload.get("stamp") or 0.0)
    if stamp:
        panel._last_joint_stamp = stamp
    now = stamp or time.time()
    panel._joint_active = True

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
    if pos_map and missing_vel and panel._last_joint_time and now > panel._last_joint_time:
        dt = max(1e-6, now - panel._last_joint_time)
        for name, pos in pos_map.items():
            prev = panel._last_joint_positions.get(name)
            if prev is not None and vel_map.get(name) is None:
                vel_map[name] = (pos - prev) / dt
    if pos_map:
        panel._last_joint_positions.update(pos_map)
        panel._last_joint_time = now
        if not panel._joint_names_warned:
            if not any(j in pos_map for j in UR5_JOINT_NAMES):
                sample = ", ".join(sorted(list(pos_map.keys()))[:8])
                panel._log_warning(
                    f"[JOINTS] Joint names no coinciden con UR5_JOINT_NAMES. sample={sample}"
            )
                panel._joint_names_warned = True

    if pos_map and not any(slider.isSliderDown() for slider in panel.joint_sliders):
        # Protección extra: no actualizar sliders si el usuario acaba de interactuar manualmente
        if time.time() > panel._slider_update_blocked_until:
            panel._updating_sliders_from_joint_state = True
            try:
                for idx, joint in enumerate(UR5_JOINT_NAMES):
                    if joint not in pos_map:
                        continue
                    slider = panel.joint_sliders[idx]
                    deg = math.degrees(pos_map[joint])
                    value = int(round(deg * JOINT_SLIDER_SCALE))
                    value = max(slider.minimum(), min(slider.maximum(), value))
                    if slider.value() != value:
                        slider.setValue(value)
            finally:
                panel._updating_sliders_from_joint_state = False

    for joint, pos_lbl in panel.dof_pos_labels.items():
        pos = pos_map.get(joint)
        if pos is None:
            pos_lbl.setText("--")
        else:
            pos_lbl.setText(f"{math.degrees(pos):.2f} deg / {pos:.3f} rad")
    for joint, vel_lbl in panel.dof_vel_labels.items():
        vel = vel_map.get(joint)
        if vel is None:
            vel_lbl.setText("--")
        else:
            vel_lbl.setText(f"{vel:.3f} rad/s")

    grip_positions = []
    for joint, pos_lbl in panel.gripper_labels.items():
        pos = pos_map.get(joint)
        if pos is None:
            pos_lbl.setText("--")
        else:
            pos_mm = pos * 1000.0
            pos_lbl.setText(f"{pos_mm:.1f} mm / {pos:.4f} m")
            grip_positions.append(pos)
    if panel.gripper_total_lbl is not None:
        if len(grip_positions) == len(panel.gripper_labels):
            opening = sum(abs(v) for v in grip_positions) * 1000.0
            panel.gripper_total_lbl.setText(f"{opening:.1f} mm")
        else:
            panel.gripper_total_lbl.setText("--")

    q = []
    for joint in UR5_JOINT_NAMES:
        if joint not in pos_map:
            q = []
            break
        q.append(pos_map[joint])
    if len(q) == 6:
        pos_model, rot_model = fk_ur5(q)
        ee_frame = str(
            getattr(panel, "_ee_frame_effective", "") or panel._required_ee_frame or "rg2_pinch_center"
        ).strip() or "rg2_pinch_center"
        (bx, by, bz), rot = _fk_tool0_to_ee_base_link(pos_model, rot_model, ee_frame)
        roll, pitch, yaw = _rot_to_rpy(rot)
        wx, wy, wz = base_to_world(bx, by, bz)
        panel._last_tcp_base = (bx, by, bz)
        panel._last_tcp_world = (wx, wy, wz)
        panel._last_tcp_fk_ts = time.monotonic()
        if panel.tcp_xyz_lbl is not None:
            panel.tcp_xyz_lbl.setText(f"{bx:.3f}, {by:.3f}, {bz:.3f}")
        if panel.tcp_rpy_lbl is not None:
            r_deg = math.degrees(roll)
            p_deg = math.degrees(pitch)
            y_deg = math.degrees(yaw)
            panel._last_tcp_rpy_deg = (r_deg, p_deg, y_deg)
            panel.tcp_rpy_lbl.setText(f"{r_deg:.1f}, {p_deg:.1f}, {y_deg:.1f}")
    else:
        panel._last_tcp_base = None
        panel._last_tcp_fk_ts = time.monotonic()
        if panel.tcp_xyz_lbl is not None:
            panel.tcp_xyz_lbl.setText("--")
        if panel.tcp_rpy_lbl is not None:
            panel.tcp_rpy_lbl.setText("--")

    vel_values = [vel_map[j] for j in UR5_JOINT_NAMES if j in vel_map]
    if vel_values:
        norm = math.sqrt(sum(v * v for v in vel_values))
        vmax = max(abs(v) for v in vel_values)
        if panel.vel_norm_lbl is not None:
            panel.vel_norm_lbl.setText(f"{norm:.3f} rad/s")
        if panel.vel_max_lbl is not None:
            panel.vel_max_lbl.setText(f"{vmax:.3f} rad/s")
    else:
        if panel.vel_norm_lbl is not None:
            panel.vel_norm_lbl.setText("--")
        if panel.vel_max_lbl is not None:
            panel.vel_max_lbl.setText("--")

    eff_values = [eff_map[j] for j in UR5_JOINT_NAMES if j in eff_map]
    if eff_values:
        emax = max(abs(v) for v in eff_values)
        if panel.eff_max_lbl is not None:
            panel.eff_max_lbl.setText(f"{emax:.3f}")
    else:
        if panel.eff_max_lbl is not None:
            panel.eff_max_lbl.setText("--")

    if panel.lbl_joint_states is not None:
        if panel._last_joint_stamp:
            panel.lbl_joint_states.setText(f"Joint states: ok ({len(names)} joints, {source})")
        else:
            panel.lbl_joint_states.setText("Joint states: esperando /joint_states ...")

    if panel._debug_joints_to_stdout and pos_map:
        joint_parts = []
        for joint in UR5_JOINT_NAMES:
            if joint in pos_map:
                joint_parts.append(f"{joint}={pos_map[joint]:.3f}rad")
        for joint in panel.gripper_labels:
            if joint in pos_map:
                joint_parts.append(f"{joint}={pos_map[joint]:.4f}m")
        pose_txt = "tcp xyz=-- rpy=--"
        if len(q) == 6:
            pos_model, rot_model = fk_ur5(q)
            ee_frame = str(
                getattr(panel, "_ee_frame_effective", "") or panel._required_ee_frame or "rg2_pinch_center"
            ).strip() or "rg2_pinch_center"
            pos, rot = _fk_tool0_to_ee_base_link(pos_model, rot_model, ee_frame)
            roll, pitch, yaw = _rot_to_rpy(rot)
            pose_txt = (
                f"tcp xyz=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                f"rpy=({math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f})"
            )
        msg = "[DEBUG] JOINTS " + " ".join(joint_parts) + f" | {pose_txt}"
        panel._emit_log(msg)
