#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_status_mgmt.py
# Contenido: Status, process, and UI management callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Status, process and UI management callbacks for ControlPanelV2."""
from __future__ import annotations

import math
import os
import shlex
import subprocess
import time
from typing import List, Optional, Set, Tuple
from .panel_ros_params import get_panel_ros_params as _get_panel_ros_params
from .panel_state import MoveItState, SystemState
from .panel_moveit_wait import wait_for_moveit_ready
from .panel_objects import get_object_positions
from .panel_process import bash_preamble
from .panel_moveit_ready import moveit_action_ready, moveit_status_ready, moveit_topics_ready
from .panel_shutdown import terminate_process
from .panel_ui_state import apply_ui_state
from .panel_utils import parse_ros_topics, read_world_name, set_led

try:
    import psutil
except ImportError:
    psutil = None

try:
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
except Exception:
    ActionClient = FollowJointTrajectory = None

from .panel_config import (
    BAGS_DIR,
    BASKET_DROP,
    BRIDGE_LAUNCH_TIMEOUT_SEC,
    CONTROLLER_START_GRACE_SEC,
    GZ_LAUNCH_TIMEOUT_SEC,
    GZ_WORLD,
    JOINT_SLIDER_SCALE,
    MOVEIT_BRIDGE_LAUNCH_TIMEOUT_SEC,
    MOVEIT_LAUNCH_TIMEOUT_SEC,
    ROS_AVAILABLE,
    STALE_PROCESS_GRACE_SEC,
    STATUS_TOPIC_CACHE_SEC,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    WORLD_FRAME,
)
from .panel_utils import ensure_dir
from . import panel_gz_objects as _gz
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QLabel, QPushButton
from .tf_pose_utils import get_tcp_in_base as tf_get_tcp_in_base
from . import panel_launch_control
from . import panel_controllers
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME
from .panel_config import (
    CONTROLLER_CHECK_INTERVAL_SEC,
    MOVEIT_POSE_TOPIC,
)
from .logging_utils import emit_log_line


def _log_exception(context: str, exc: Exception) -> None:
    emit_log_line(f"[STATUS_MGMT][ERROR][{context}] {exc}")


def _log_trace(panel, message: str) -> None:
    if getattr(panel, "_debug_joints_to_stdout", False):
        panel._emit_log(message)
    else:
        panel._log(message)

def _apply_debug_button_style(panel, button: QPushButton, enabled: bool) -> None:
    button.setStyleSheet(
        "background:#3b82f6; color:white; border-radius:6px;" if enabled else ""
    )

def _print_pose_snapshot(panel):
    """Imprime en una línea: TCP, cesta, mesa y objetos en frame base_link."""
    objs = {}
    pose_src = {}
    if panel._gz_running:
        world_name = panel._gz_world_name or read_world_name(panel.world_combo.currentText().strip()) or GZ_WORLD
        poses = panel._read_world_pose_info(world_name)
        if poses:
            known = get_object_positions()
            objs, pose_src = panel._extract_pose_updates(poses, known)
        if not objs:
            objs = get_object_positions()
    obj_parts = []
    world_frame = panel._world_frame_last_first()
    for name, (x, y, z) in sorted(objs.items()):
        base_obj = panel._ensure_base_coords(
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

    base_frame = panel._business_base_frame()
    ee_frame = str(getattr(panel, "_ee_frame_effective", "") or panel._required_ee_frame or "rg2_pinch_center").strip() or "rg2_pinch_center"
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
        panel._last_debug_tcp_base = (bx, by, bz)
        panel._last_debug_tcp_ts = time.monotonic()
        tcp_txt = (
            f"tcp_base=({bx:.3f},{by:.3f},{bz:.3f}) "
            f"rpy=({r_deg:.1f},{p_deg:.1f},{y_deg:.1f}) tcp_source=tf2"
        )
    else:
        panel._last_debug_tcp_base = None
        panel._last_debug_tcp_ts = time.monotonic()
        tcp_txt = f"tcp_base=UNAVAILABLE tcp_source=tf2 reason={tcp_reason}"

    basket_x, basket_y, basket_z = BASKET_DROP
    basket_txt = f"basket=({basket_x:.3f},{basket_y:.3f},{basket_z:.3f})"
    table_txt = f"table=({TABLE_CENTER_X:.3f},{TABLE_CENTER_Y:.3f})"

    line = f"[DEBUG POSES] {tcp_txt} | {basket_txt} | {table_txt} | {obj_txt}"
    panel._emit_log(line)

def _drop_detach_supported(panel, *args, **kwargs):
    return _gz._drop_detach_supported(panel, *args, **kwargs)

def _release_objects(panel, *args, **kwargs):
    _gz._release_objects(panel, *args, **kwargs)

def _schedule_release_retry(panel, *args, **kwargs):
    _gz._schedule_release_retry(panel, *args, **kwargs)

def _attach_drop_objects(panel, *args, **kwargs):
    _gz._attach_drop_objects(panel, *args, **kwargs)

def _maybe_nudge_drop_objects(panel, *args, **kwargs):
    _gz._maybe_nudge_drop_objects(panel, *args, **kwargs)

def _resolve_set_pose_service(panel, *args, **kwargs):
    return _gz._resolve_set_pose_service(panel, *args, **kwargs)

def _resolve_gz_cli(panel, *args, **kwargs):
    return _gz._resolve_gz_cli(panel, *args, **kwargs)

def _resolve_world_sdf_path(panel, *args, **kwargs):
    return _gz._resolve_world_sdf_path(panel, *args, **kwargs)

def _load_pick_demo_recover_sdf(panel, *args, **kwargs):
    return _gz._load_pick_demo_recover_sdf(panel, *args, **kwargs)

def _run_gz_service_cli(panel, *args, **kwargs):
    return _gz._run_gz_service_cli(panel, *args, **kwargs)

def _recover_pick_demo_to_table_gz(panel, *args, **kwargs):
    return _gz._recover_pick_demo_to_table_gz(panel, *args, **kwargs)

def _maybe_hold_drop_objects(panel, *args, **kwargs):
    _gz._maybe_hold_drop_objects(panel, *args, **kwargs)

def _maybe_recover_pick_demo(panel, *args, **kwargs):
    _gz._maybe_recover_pick_demo(panel, *args, **kwargs)

def _recover_pick_demo_to_table(panel, *args, **kwargs):
    _gz._recover_pick_demo_to_table(panel, *args, **kwargs)

def _hold_drop_objects(panel, *args, **kwargs):
    _gz._hold_drop_objects(panel, *args, **kwargs)

def _hold_drop_objects_gz(panel, *args, **kwargs):
    _gz._hold_drop_objects_gz(panel, *args, **kwargs)

def _drop_hold_tick(panel, *args, **kwargs):
    _gz._drop_hold_tick(panel, *args, **kwargs)

def _nudge_drop_objects(panel, *args, **kwargs):
    _gz._nudge_drop_objects(panel, *args, **kwargs)


def _start_release_service(panel):
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("Release service bloqueado: ERROR_FATAL activo", error=True)
        panel._emit_log_throttled(
            "SAFETY:start_release_service:ERROR_FATAL",
            "[SAFETY] Release service bloqueado: ERROR_FATAL activo",
        )
        return
    panel_launch_control.start_release_service(panel)

def _start_world_tf_publisher(panel, world_name: str) -> None:
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("world_tf bloqueado: ERROR_FATAL activo", error=True)
        panel._emit_log_throttled(
            "SAFETY:start_world_tf:ERROR_FATAL",
            "[SAFETY] world_tf bloqueado: ERROR_FATAL activo",
        )
        return
    panel_launch_control.start_world_tf_publisher(panel, world_name)
    return
def _stop_world_tf_publisher(panel) -> None:
    panel_launch_control.stop_world_tf_publisher(panel)
    return
def _stop_gazebo(panel):
    panel_launch_control.stop_gazebo(panel)
    return
def _start_robot_state_publisher(panel):
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("RSP bloqueado: ERROR_FATAL activo", error=True)
        panel._emit_log_throttled(
            "SAFETY:start_rsp:ERROR_FATAL",
            "[SAFETY] RSP bloqueado: ERROR_FATAL activo",
        )
        return
    panel_launch_control.start_robot_state_publisher(panel)
    return
def _start_bridge(panel):
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("Bridge bloqueado: ERROR_FATAL activo", error=True)
        panel._emit_log_throttled(
            "SAFETY:start_bridge:ERROR_FATAL",
            "[SAFETY] Bridge bloqueado: ERROR_FATAL activo",
        )
        return
    panel_launch_control.start_bridge(panel)
    return
def _spawn_controllers_async(panel):
    panel_controllers.spawn_controllers_async(panel)

def _stop_bridge(panel):
    panel_launch_control.stop_bridge(panel)
    return
def _start_moveit(panel):
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("MoveIt bloqueado: ERROR_FATAL activo", error=True)
        panel._emit_log_throttled(
            "SAFETY:start_moveit:ERROR_FATAL",
            "[SAFETY] MoveIt bloqueado: ERROR_FATAL activo",
        )
        return
    panel_launch_control.start_moveit(panel)
    return
def _stop_moveit(panel):
    panel_launch_control.stop_moveit(panel)
    return
def _wait_for_moveit_ready(panel):
    wait_for_moveit_ready(panel)

def _start_moveit_bridge(panel):
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("MoveIt bridge bloqueado: ERROR_FATAL activo", error=True)
        panel._emit_log_throttled(
            "SAFETY:start_moveit_bridge:ERROR_FATAL",
            "[SAFETY] MoveIt bridge bloqueado: ERROR_FATAL activo",
        )
        return
    panel_launch_control.start_moveit_bridge(panel)
    return
def _stop_moveit_bridge(panel):
    panel_launch_control.stop_moveit_bridge(panel)
    panel._moveit_bridge_stop_grace_until = time.monotonic() + 6.0
    return
def _clear_moveit_bridge_launching(panel):
    panel._moveit_bridge_launching = False
    panel._moveit_bridge_launch_start = 0.0
    panel._set_launching_style(panel.btn_moveit_bridge_start, False)
    panel.signal_refresh_controls.emit()

def _kill_proc(panel, proc, label: str):
    terminate_process(proc, label, log_fn=panel._log)

def _proc_alive(panel, proc) -> bool:
    return proc is not None and proc.poll() is None


def _save_home_from_sliders(panel):
    """Lee los valores actuales de los sliders y guarda como nueva pose HOME."""
    from .panel_utils import save_home_pose
    joint_values = []
    for slider in panel.joint_sliders:
        deg = slider.value() / JOINT_SLIDER_SCALE
        rad = math.radians(deg)
        joint_values.append(rad)
    save_home_pose(joint_values)
    panel._set_status("Pose HOME guardada", error=False)

def _rosbag_running(panel) -> bool:
    return panel._proc_alive(panel.bag_proc)

def _start_bag(panel):
    panel._log_button("Start bag")
    if panel._proc_alive(panel.bag_proc):
        panel._set_status("Bag ya activo", error=True)
        return
    if not panel._bridge_running:
        panel._log_warning("Bag en espera: bridge no activo")
        panel._set_status("Bag en espera: bridge no activo", error=False)
        set_led(panel.led_bag, "warn")
        return
    name = panel.bag_name.text().strip() or f"demo_{int(time.time())}"
    topics_raw = panel.bag_topics.text().strip()
    panel._log(f"[BAG] Nombre: {name}, Tópicos raw: {topics_raw}")
    topics, invalid = parse_ros_topics(topics_raw)
    if invalid:
        panel._log_error(f"Tópicos inválidos: {invalid}")
        panel._set_status("Tópicos inválidos", error=True)
        set_led(panel.led_bag, "error")
        return
    if not topics:
        panel._log_error("No hay tópicos para grabar")
        panel._set_status("No hay tópicos", error=True)
        set_led(panel.led_bag, "error")
        return
    ensure_dir(BAGS_DIR)
    outdir = os.path.join(BAGS_DIR, name)
    if os.path.exists(outdir):
        suffix = 1
        while os.path.exists(f"{outdir}_{suffix}"):
            suffix += 1
        outdir = f"{outdir}_{suffix}"
    panel._log(f"[BAG] Directorio salida: {outdir}")
    panel._set_status(f"Grabando bag en {outdir}…")
    set_led(panel.led_bag, "warn")

    def worker():
        cmd = (
            bash_preamble(panel.ws_dir)
            + "ros2 bag record -o "
            + shlex.quote(outdir)
            + " --topics "
            + " ".join(shlex.quote(t) for t in topics)
        )
        try:
            panel.bag_proc = subprocess.Popen([
                "bash",
                "-lc",
                cmd,
            ], preexec_fn=os.setsid)
            panel._bag_running = True
            panel._started_bag = True
            panel._ui_set_status(f"Bag grabando → {outdir}")
            panel.signal_set_led.emit(panel.led_bag, "on")
            panel.signal_refresh_controls.emit()
        except Exception as exc:
            panel._ui_set_status(f"Error al grabar bag: {exc}", error=True)
            panel.signal_set_led.emit(panel.led_bag, "error")
            panel.signal_refresh_controls.emit()

    panel._run_async(worker)

def _stop_bag(panel):
    panel._log_button("Stop bag")
    panel._set_status("Deteniendo bag…")
    panel._kill_proc(panel.bag_proc, "ros2 bag record")
    panel.bag_proc = None
    panel._bag_running = False
    set_led(panel.led_bag, "off")

def _refresh_status_sync(panel):
    """Chequeo síncrono de estado al startup."""
    clock_ok, _ = panel._clock_status()
    gz_state = panel._gazebo_state()
    gz_ok = gz_state == "GAZEBO_READY"
    br_ok = panel._bridge_transport_detected()
    bag_ok = panel._rosbag_running()
    ctrl_ok = panel._ros2_control_available()
    moveit_ok = panel._moveit_ready()
    moveit_bridge_ok = panel._moveit_bridge_detected()
    panel._apply_status(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)

def _refresh_status_async(panel):
    if panel._status_check_inflight:
        return
    panel._status_check_inflight = True

    def worker():
        if panel._managed_mode and panel._external_state_active():
            state = panel._system_state
            external_state = (panel._external_state or "").upper()
            gz_ok = external_state not in ("BOOT", "BOOTING", "WAITING_GAZEBO")
            br_ok = external_state not in ("BOOT", "BOOTING", "WAITING_GAZEBO", "WAITING_BRIDGE")
            clock_ok = gz_ok
            ctrl_ok = state not in (
                SystemState.BOOT,
                SystemState.WAITING_GAZEBO,
                SystemState.WAITING_CONTROLLERS,
            )
            moveit_ok = panel._moveit_ready()
            moveit_bridge_ok = panel._moveit_bridge_detected()
            bag_ok = panel._bag_running
            panel.status_updated.emit(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)
            panel._status_check_inflight = False
            return
        clock_ok, _ = panel._clock_status()
        gz_state = panel._gazebo_state()
        gz_ok = gz_state == "GAZEBO_READY"
        br_ok = panel._bridge_transport_detected()
        bag_ok = panel._rosbag_running()
        ctrl_ok = panel._ros2_control_available() if br_ok else False
        moveit_ok = panel._moveit_ready()
        moveit_bridge_ok = panel._moveit_bridge_detected()
        panel.status_updated.emit(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)
        panel._status_check_inflight = False

    panel._run_async(worker)

def _apply_status(panel,
    gz_ok: bool,
    br_ok: bool,
    clock_ok: bool,
    bag_ok: bool,
    ctrl_ok: bool,
    moveit_ok: bool,
    moveit_bridge_ok: bool,
):
    gz_state = panel._gz_state or "GAZEBO_OFF"
    if gz_state == "GAZEBO_READY":
        set_led(panel.led_gz, "on")
    elif gz_state in ("GAZEBO_STARTING", "GAZEBO_DEGRADED", "GAZEBO_MONITOR_BUG"):
        set_led(panel.led_gz, "warn")
    else:
        set_led(panel.led_gz, "off")
    set_led(panel.led_bridge, "on" if br_ok else "off")
    set_led(panel.led_clock, "on" if clock_ok else "off")
    set_led(panel.led_bag, "on" if bag_ok else "off")
    set_led(panel.led_ros2, "on" if ctrl_ok else "off")
    set_led(panel.led_ur5, "on" if gz_ok else "off")
    if panel._moveit_state == MoveItState.STARTING:
        set_led(panel.led_moveit, "warn")
    elif panel._moveit_state == MoveItState.WAITING_MOVEIT_READY:
        set_led(panel.led_moveit, "warn")
    elif panel._moveit_state == MoveItState.READY:
        set_led(panel.led_moveit, "on")
    elif panel._moveit_state == MoveItState.ERROR:
        set_led(panel.led_moveit, "error")
    else:
        set_led(panel.led_moveit, "off")
    set_led(panel.led_moveit_bridge, "on" if moveit_bridge_ok else "off")
    prev_bridge = panel._bridge_running
    prev_gz = panel._gz_running
    panel._gz_running = gz_ok
    panel._bridge_running = br_ok
    panel._bag_running = bag_ok
    panel._moveit_running = moveit_ok
    panel._moveit_bridge_running = moveit_bridge_ok
    if (not panel._moveit_required) and moveit_ok:
        panel._moveit_required = True
        panel._emit_log("[MOVEIT] move_group detectado; habilitando moveit_required automáticamente")
    if gz_ok and not prev_gz and not panel._objects_settled and not panel._settle_worker_active:
        panel._emit_log("[PHYSICS][SETTLE] Gazebo READY: triggering settle watch")
        panel.signal_start_objects_settle_watch.emit()
    if br_ok and not prev_bridge:
        # Selective init without critical TF deadlines (same as panel_launchers.py)
        panel._ensure_pose_subscription()
        panel._start_pose_info_watch()
        panel._start_tf_ready_timer()
        if panel._camera_required:
            QTimer.singleShot(600, panel._auto_connect_camera)
        panel._emit_log("[STATUS] Bridge externo detectado (transicion); inicializando suscripciones")
    # Programar auto-release directamente cuando el bridge esta detectado y no
    # se ha hecho. Cubre el caso de bridge externo (PANEL_AUTO_BRIDGE=0) en
    # el que ni start_bridge() ni signal_bridge_ready se invocan: sin este
    # bloque, _auto_release_drop_objects_when_ready nunca se programa y
    # validate_pick_3_cycles.sh queda con "release de objetos pendiente".
    if (
        br_ok
        and getattr(panel, "_auto_release_drop_objects", False)
        and not getattr(panel, "_objects_release_done", False)
        and not getattr(panel, "_auto_release_external_scheduled", False)
        and not getattr(panel, "_detach_inflight", False)
    ):
        panel._auto_release_external_scheduled = True
        panel._emit_log(
            "[PHYSICS] Auto-release DROP scheduled (bridge externo detectado)"
        )
        QTimer.singleShot(400, panel._auto_release_drop_objects_when_ready)
    if panel._moveit_state == MoveItState.OFF and moveit_ok:
        panel._moveit_state = MoveItState.READY
        if panel._moveit_bridge_detected():
            panel._moveit_state_reason = "moveit_bridge detectado"
        else:
            panel._moveit_state_reason = "move_group detectado"
        set_led(panel.led_moveit, "on")
    if panel._moveit_state == MoveItState.READY and not moveit_ok:
        panel._moveit_state = MoveItState.WAITING_MOVEIT_READY
        panel._moveit_state_reason = "move_group no responde"
        set_led(panel.led_moveit, "warn")
    if panel._moveit_state in (MoveItState.STARTING, MoveItState.WAITING_MOVEIT_READY) and moveit_ok:
        panel._moveit_state = MoveItState.READY
        panel._moveit_state_reason = "move_group listo"
        set_led(panel.led_moveit, "on")
    panel._update_moveit_status_label()
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
    panel.status_lbl.setText(" · ".join(summary))
    panel._update_system_stats()
    panel._refresh_controls()

def _moveit_topics_ready(panel) -> bool:
    return moveit_topics_ready(panel)

def _moveit_status_ready(panel) -> bool:
    return moveit_status_ready(panel)

def _moveit_action_ready(panel) -> bool:
    return moveit_action_ready(panel)

def _list_topic_names(panel) -> List[str]:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return []
    try:
        return panel.ros_worker.list_topic_names()
    except Exception:
        return []

def _list_action_names(panel) -> List[str]:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return []
    try:
        return panel.ros_worker.list_action_names()
    except Exception:
        return []

def _topic_has_any_publishers(panel, topics: List[str]) -> bool:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return False
    for topic in topics:
        if panel.ros_worker.topic_has_publishers(topic):
            return True
    return False

def _world_frame_last_first(panel, fallback: Optional[str] = None) -> str:
    frame = (
        fallback
        or WORLD_FRAME
        or panel._last_selection_frame
        or "world"
    )
    frame_norm = str(frame or "").split("|", 1)[0].strip() or "world"
    base_frame = str(panel._business_base_frame() or "base_link").strip() or "base_link"
    if frame_norm in {base_frame, "base", "tool0", "rg2_tcp", "rg2_pinch_center"}:
        # Guardrail: las poses de pose/info llegan en world, no en base_link.
        # Si este valor se contamina, los cálculos geométricos se desalinean.
        frame_norm = str(WORLD_FRAME or "world").strip() or "world"
        panel._emit_log_throttled(
            "FRAME:world_frame_guard",
            f"[FRAME] world_frame inválido ({frame}); usando {frame_norm}",
            min_interval=2.0,
        )
    return frame_norm

def _world_frame_config_first(panel) -> str:
    return panel._world_frame_last_first(WORLD_FRAME or "world")

def _follow_joint_traj_ready(panel) -> bool:
    if not ROS_AVAILABLE or ActionClient is None or FollowJointTrajectory is None:
        return False
    _ros_params = _get_panel_ros_params()
    strict_action = _ros_params.strict_traj_action
    expected_action = _ros_params.expected_traj_action.strip()
    action_names = set(panel._list_action_names())
    action_topics = set(panel._list_topic_names())

    def _action_graph_ready(action_name: str) -> bool:
        if not action_name:
            return False
        status_topic = f"{action_name}/_action/status"
        feedback_topic = f"{action_name}/_action/feedback"
        goal_topic = f"{action_name}/_action/send_goal"
        if status_topic in action_topics or feedback_topic in action_topics or goal_topic in action_topics:
            return True
        if not panel.ros_worker or not panel.ros_worker.node_ready():
            return False
        return bool(
            panel.ros_worker.topic_has_publishers(status_topic)
            or panel.ros_worker.topic_has_publishers(feedback_topic)
            or panel.ros_worker.topic_has_subscribers(goal_topic)
        )

    if strict_action and expected_action:
        if expected_action not in action_names:
            if not _action_graph_ready(expected_action):
                return False
        action_name = expected_action
    else:
        action_name = ""

    # Fallback de robustez: si el action server esperado ya existe en el grafo
    # ROS pero el nodo local de MoveIt aun no esta inicializado, aceptar READY.
    if panel._moveit_node is None:
        if strict_action and expected_action:
            return expected_action in action_names or _action_graph_ready(expected_action)
        return bool(action_names)

    traj_topic = panel._select_traj_topic()
    if not action_name:
        action_name = panel._resolve_traj_action_name(traj_topic, allow_fallback=True)
    if not action_name:
        return False
    if panel._traj_action_client is None or panel._traj_action_name != action_name:
        panel._traj_action_client = panel._get_action_client(
            action_name,
            FollowJointTrajectory,
            log_ctx="follow_traj",
        )
    client = panel._traj_action_client
    if client is None:
        return False
    return panel._wait_action_server(
        client,
        timeout_sec=0.2,
        log_ctx="follow_traj",
        action_name=action_name,
    )

def _moveit_bridge_detected(panel) -> bool:
    if panel._proc_alive(panel.moveit_bridge_proc):
        panel._moveit_bridge_detected_cache = True
        panel._moveit_bridge_detected_ts = time.monotonic()
        return True
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False
    now = time.monotonic()
    if (
        STATUS_TOPIC_CACHE_SEC > 0.0
        and (now - panel._moveit_bridge_detected_ts) < STATUS_TOPIC_CACHE_SEC
    ):
        return panel._moveit_bridge_detected_cache
    pose_ready = (
        panel.ros_worker.topic_has_subscribers(MOVEIT_POSE_TOPIC)
        or panel.ros_worker.topic_has_subscribers("/grasp_pose")
    )
    result_ready = panel.ros_worker.topic_has_publishers("/desired_grasp/result")
    detected = pose_ready and result_ready
    panel._moveit_bridge_detected_cache = bool(detected)
    panel._moveit_bridge_detected_ts = now
    return detected

def _move_group_startup_ready(panel) -> bool:
    status_ready = panel._moveit_status_ready()
    action_ready = panel._moveit_action_ready()
    ready = status_ready or action_ready
    ros_node_ready = bool(panel.ros_worker and panel.ros_worker.node_ready())
    panel._emit_log_throttled(
        "MOVEIT:startup_gate",
        "[MOVEIT2][STARTUP_GATE] "
        f"ready={str(bool(ready)).lower()} "
        f"status={str(bool(status_ready)).lower()} "
        f"action={str(bool(action_ready)).lower()} "
        f"moveit_proc={str(bool(panel._proc_alive(panel.moveit_proc))).lower()} "
        f"ros_worker_started={str(bool(panel._ros_worker_started)).lower()} "
        f"ros_node_ready={str(bool(ros_node_ready)).lower()} "
        f"moveit_state={panel._moveit_state.value}",
        min_interval=1.0,
    )
    return ready

def _move_group_ready(panel) -> bool:
    return panel._move_group_startup_ready() and panel._follow_joint_traj_ready()

def _moveit_ready(panel) -> bool:
    if panel._moveit_bridge_detected():
        return True
    return panel._move_group_startup_ready()

def _update_moveit_status_label(panel) -> None:
    if panel.lbl_moveit_status is not None:
        state_label = panel._moveit_state.value
        panel.lbl_moveit_status.setText("MoveIt")
        reason = panel._moveit_state_reason or state_label
        panel.lbl_moveit_status.setToolTip(f"{state_label}: {reason}")
    if panel.lbl_moveit_bridge_status is not None:
        bridge_label = "ON" if panel._moveit_bridge_running else "OFF"
        panel.lbl_moveit_bridge_status.setText("MoveIt bridge")

def _update_system_stats(panel):
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
        stale_count, _stale_hint = panel._detect_stale_processes()
    except Exception as exc:
        _log_exception("update_system_stats stale procs", exc)
        stale_count = 0
    health_alert = stale_count > 0
    health_txt = "Proc.Zombis activos" if health_alert else "Todo OK"
    panel._set_stat_label(panel.sys_cpu_lbl, cpu_txt, cpu_alert)
    panel._set_stat_label(panel.sys_ram_lbl, ram_txt, ram_alert)
    panel._set_stat_label(panel.sys_load_lbl, load_txt, load_alert)
    panel._set_stat_label(panel.sys_health_lbl, health_txt, health_alert)

def _set_stat_label(panel, label: QLabel, text: str, alert: bool):
    color = "#dc2626" if alert else "#0f172a"
    label.setStyleSheet(f"font-size:11px; color:{color};")
    label.setText(text)

def _known_process_pids(panel) -> Set[int]:
    pids = {os.getpid()}
    if psutil:
        try:
            for parent in psutil.Process(os.getpid()).parents():
                pids.add(parent.pid)
        except Exception as exc:
            _log_exception("list parent processes", exc)
    for proc in (
        panel.gz_proc,
        panel.gz_gui_proc,
        panel.bridge_proc,
        panel.bag_proc,
        panel.moveit_proc,
        panel.moveit_bridge_proc,
        panel.release_service_proc,
        panel.rsp_proc,
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

def _list_stale_processes(panel) -> List[Tuple[int, str, str]]:
    """Listar procesos del proyecto que no pertenecen al panel actual."""
    if not psutil:
        return []
    ws_dir = os.path.realpath(panel.ws_dir)
    grace_sec = max(0.0, STALE_PROCESS_GRACE_SEC)
    cutoff = panel._panel_start_ts - grace_sec
    ignore_pids = panel._known_process_pids()
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

def _detect_stale_processes(panel) -> Tuple[int, str]:
    """Detecta procesos del proyecto que no pertenecen al panel actual."""
    stale = panel._list_stale_processes()
    if not stale:
        return 0, ""
    sample_cmd = stale[0][1]
    hint = sample_cmd.split()[0] if sample_cmd else ""
    return len(stale), hint

def _refresh_controls(panel):
    if panel._closing:
        return
    if panel._gz_launching and panel._clear_launching_if_timeout("Gazebo", panel._gz_launch_start, GZ_LAUNCH_TIMEOUT_SEC):
        panel._gz_launching = False
        panel._gz_launch_start = 0.0
        panel._set_launching_style(panel.btn_gz_start, False)
    if panel._bridge_launching and panel._clear_launching_if_timeout("Bridge", panel._bridge_launch_start, BRIDGE_LAUNCH_TIMEOUT_SEC):
        panel._bridge_launching = False
        panel._bridge_launch_start = 0.0
        panel._set_launching_style(panel.btn_bridge_start, False)
    if panel._moveit_launching and panel._clear_launching_if_timeout("MoveIt", panel._moveit_launch_start, MOVEIT_LAUNCH_TIMEOUT_SEC):
        panel._moveit_launching = False
        panel._moveit_launch_start = 0.0
        panel._set_launching_style(panel.btn_moveit_start, False)
    if panel._moveit_bridge_launching and panel._clear_launching_if_timeout("MoveIt bridge", panel._moveit_bridge_launch_start, MOVEIT_BRIDGE_LAUNCH_TIMEOUT_SEC):
        panel._moveit_bridge_launching = False
        panel._moveit_bridge_launch_start = 0.0
        panel._set_launching_style(panel.btn_moveit_bridge_start, False)
    panel._update_system_state()
    panel._check_critical_timeouts()
    if panel._fatal_latched:
        return
    effective_state, effective_reason = panel._effective_system_state()
    apply_ui_state(panel, effective_state, effective_reason)
    if getattr(panel, "btn_pick_demo", None) is not None:
        _main_panel_can_release_approach = (
            panel._direct_waiting_for_approach_confirmation()
            and panel._step_mode != "STEP_BY_STEP"
        )
        if _main_panel_can_release_approach:
            panel.btn_pick_demo.setEnabled(False)
            panel.btn_pick_demo.setText("Agarre Objeto (Directo)")
            panel.btn_pick_demo.setToolTip(
                "Flujo Directo detenido en MESA. Usa el boton de iniciar APPROACH."
            )
            if getattr(panel, "btn_pick_demo_approach", None) is not None:
                panel.btn_pick_demo_approach.setVisible(True)
                panel.btn_pick_demo_approach.setEnabled(True)
                panel.btn_pick_demo_approach.setText("Iniciar APPROACH_COARSE")
                panel.btn_pick_demo_approach.setToolTip(
                    "Robot detenido en MESA. Pulsa para iniciar APPROACH_COARSE."
                )
        else:
            if panel._direct_waiting_for_approach_confirmation() and panel._step_mode == "STEP_BY_STEP":
                panel.btn_pick_demo.setToolTip(
                    "Flujo Directo detenido en MESA. Inicia APPROACH_COARSE desde el panel paso a paso."
                )
            panel.btn_pick_demo.setText("Agarre Objeto (Directo)")
            if not (panel._direct_waiting_for_approach_confirmation() and panel._step_mode == "STEP_BY_STEP"):
                panel.btn_pick_demo.setToolTip(
                    "Demo pick & place con objeto de posicion conocida (fuera del TFM)"
                )
            if getattr(panel, "btn_pick_demo_approach", None) is not None:
                panel.btn_pick_demo_approach.setVisible(False)
                panel.btn_pick_demo_approach.setEnabled(False)
    panel._maybe_auto_run_pick_demo()

def _maybe_auto_run_pick_demo(panel) -> None:
    if not panel._auto_pick_demo_enabled:
        return
    if panel._direct_waiting_for_approach_confirmation():
        panel._emit_log(
            "[AUTO_PICK_DEMO] en espera de confirmacion manual para APPROACH_COARSE"
        )
        return
    if panel._auto_pick_demo_done >= panel._auto_pick_demo_attempts:
        return
    now = time.time()
    if (now - panel._auto_pick_demo_last_try_ts) < 4.0:
        return
    if panel._manual_inflight or panel._script_motion_active:
        panel._emit_log(
            f"[AUTO_PICK_DEMO] bloqueado: manual_inflight={panel._manual_inflight} "
            f"script_active={panel._script_motion_active}"
        )
        panel._auto_pick_demo_last_try_ts = now
        return
    if panel._system_state in (SystemState.BOOT, SystemState.WAITING_GAZEBO, SystemState.ERROR_FATAL):
        panel._emit_log(
            f"[AUTO_PICK_DEMO] bloqueado: system_state={panel._system_state.value}"
        )
        panel._auto_pick_demo_last_try_ts = now
        return
    # Usar _state_ready_basic() que evalúa el estado efectivo del sistema
    # en lugar de los flags individuales, ya que en modo no-managed puede
    # que _controllers_ok no se actualice hasta que el bridge esté activo.
    system_ok = panel._state_ready_basic()
    tf_ok = panel._tf_ready_state or bool(panel._ee_frame_effective)
    if not system_ok or not tf_ok:
        panel._emit_log(
            f"[AUTO_PICK_DEMO] no listo: state={panel._system_state.value} "
            f"ctrl_ok={panel._controllers_ok} tf_ok={tf_ok} "
            f"ee={panel._ee_frame_effective!r} system_ok={system_ok}"
        )
        panel._auto_pick_demo_last_try_ts = now
        return
    panel._auto_pick_demo_last_try_ts = now
    attempt = panel._auto_pick_demo_done + 1
    panel._emit_log(
        f"[AUTO_PICK_DEMO] trigger intento {attempt}/{panel._auto_pick_demo_attempts}"
    )
    panel._auto_pick_demo_done = attempt
    # Auto-select pick_demo so the SYNC_GATE selection check passes.
    if not panel._selected_object:
        panel._selected_object = PICK_DEMO_OBJECT_NAME
    if not panel._selection_last_user_name:
        panel._selection_last_user_name = PICK_DEMO_OBJECT_NAME
    panel._run_pick_demo()

def _wait_for_state_change(panel, timeout_sec: float) -> bool:
    panel._state_event.clear()
    return panel._state_event.wait(timeout_sec)

def _schedule_controller_check(panel) -> None:
    if not panel._bridge_running:
        return
    now = time.time()
    if panel._controller_check_inflight or (now - panel._last_controller_check) < CONTROLLER_CHECK_INTERVAL_SEC:
        return
    panel._controller_check_inflight = True

    def worker():
        ok, reason = panel._controllers_ready()
        changed = (ok != panel._controllers_ok) or (reason != panel._controllers_reason)
        now = time.time()
        grace_base = panel._controller_spawn_last_start or panel._bridge_start_ts or 0.0
        in_grace = grace_base and (now - grace_base) < CONTROLLER_START_GRACE_SEC
        gazebo_not_ready = str(reason).startswith("gazebo_not_ready")
        if ok:
            if panel._controllers_state != "READY":
                panel._emit_log("[CTRL] state=READY")
                panel._controllers_state = "READY"
            panel._controllers_ok = True
            panel._controllers_reason = reason
        else:
            if in_grace or gazebo_not_ready:
                if panel._controllers_state != "STARTING":
                    panel._emit_log("[CTRL] state=STARTING")
                    panel._controllers_state = "STARTING"
                # Mantener controllers_ok=false pero sin degradar el estado global.
                panel._controllers_ok = False
                panel._controllers_reason = reason
            else:
                if panel._controllers_state != "ERROR":
                    panel._emit_log("[CTRL] state=ERROR")
                    panel._controllers_state = "ERROR"
                panel._controllers_ok = False
                panel._controllers_reason = reason
        panel._last_controller_check = time.time()
        panel._controller_check_inflight = False
        if changed:
            if ok:
                panel._emit_log(f"[CTRL] controllers_ready=true reason={reason}")
                panel._emit_log(f"[CTRL_GATE] controllers_ready=true reason={reason}")
            else:
                if in_grace or gazebo_not_ready:
                    panel._emit_log(f"[CTRL] controllers_ready=false reason=starting ({reason})")
                    panel._emit_log(f"[CTRL_GATE] controllers_ready=false reason=starting ({reason})")
                else:
                    panel._emit_log(f"[CTRL] controllers_ready=false reason={reason}")
                    panel._emit_log(f"[CTRL_GATE] controllers_ready=false reason={reason}")
                    panel._log_warning(f"[PICK] controladores no listos ({reason})")
        panel.signal_controllers_ready.emit(ok)
        panel.signal_refresh_controls.emit()

    panel._run_async(worker)

def get_health_report(panel) -> dict:
    """Emitir un resumen JSON del pipeline: topics, nodos, servicios, world_name, pose_info, TF, cámaras, controllers, MoveIt2, etc."""
    report = {}
    # Topics y tipos
    topics_types = []
    try:
        if panel._ros_worker_started and panel.ros_worker.node_ready():
            topics_types = panel.ros_worker.topic_names_and_types()
    except Exception as exc:
        _log_exception("health_report topics/types", exc)
    report["topics_types"] = topics_types
    # Nodos y servicios
    nodes = []
    services = []
    actions = []
    try:
        if panel._ros_worker_started and panel.ros_worker.node_ready():
            nodes = panel.ros_worker.list_node_names() if hasattr(panel.ros_worker, "list_node_names") else []
            services = panel.ros_worker.list_service_names() if hasattr(panel.ros_worker, "list_service_names") else []
            actions = panel.ros_worker.list_action_names() if hasattr(panel.ros_worker, "list_action_names") else []
    except Exception as exc:
        _log_exception("health_report nodes/services/actions", exc)
    report["nodes"] = nodes
    report["services"] = services
    report["actions"] = actions
    # world_name detectado y pose_info_topic
    world_name = panel._gz_world_name or panel._detect_world_name() or "unknown"
    pose_info_topic = panel._discover_pose_info_topic(world_name)
    report["world_name_detected"] = world_name
    return report
