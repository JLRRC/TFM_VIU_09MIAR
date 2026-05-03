#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_gz_startup.py
# Contenido: Gazebo startup, state, and control callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Gazebo startup, world/bridge control, and ROS2 control callbacks."""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from typing import Dict, Optional, Tuple
from .panel_readiness import camera_ready_status
from .panel_shutdown import StopSequence
from .panel_startup import StartSequence
from .panel_tf import get_tf_helper
from .panel_utils import resolve_controller_manager

try:
    import psutil
except ImportError:
    psutil = None  # type: ignore

from PyQt5.QtWidgets import QFileDialog

from .panel_config import (
    BRIDGE_BASE_YAML,
    CAMERA_INIT_GRACE_SEC,
    CONTROLLER_READY_TIMEOUT_SEC,
    CRITICAL_CLOCK_TIMEOUT_SEC,
    CRITICAL_POSE_TIMEOUT_SEC,
    DEBUG_POSES_PERIOD_SEC,
    DEFAULT_WORLD_CANDIDATES,
    GZ_WORLD,
    PANEL_KILL_STALE,
    PANEL_SKIP_CLEANUP,
    POSE_INFO_LOG_PERIOD,
    POSE_INFO_MAX_AGE_SEC,
    POSE_INFO_POLL_SEC,
    SCRIPTS_DIR,
    TF_INIT_GRACE_SEC,
    WORLDS_DIR,
)
from .panel_utils import read_world_name
from . import panel_camera_controllers as _cc
from typing import Set
from PyQt5.QtCore import QTimer
from .panel_state import MoveItState, SystemState
from .panel_camera import _runtime_time
from . import panel_launch_control
from .logging_utils import emit_log_line


def _log_exception(context: str, exc: Exception) -> None:
    emit_log_line(f"[GZ_STARTUP][ERROR][{context}] {exc}")


def _gazebo_bridge_signal(panel) -> Tuple[bool, str]:
    """Signal S3: at least one critical bridged topic has publishers."""
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False, "node_off"
    pose_topic = panel._pose_info_topic()
    if panel.ros_worker.topic_has_publishers(pose_topic):
        return True, pose_topic
    camera_topic = ""
    try:
        camera_topic = panel.camera_topic_combo.currentText().strip() or panel.camera_topic
    except Exception:
        camera_topic = panel.camera_topic
    if camera_topic and panel.ros_worker.topic_has_publishers(camera_topic):
        return True, camera_topic
    return False, "no_critical_topic"

def _gazebo_state(panel) -> str:
    """GAZEBO_OFF / GAZEBO_STARTING / GAZEBO_READY / GAZEBO_DEGRADED / GAZEBO_MONITOR_BUG."""
    proc_ok, proc_reason = panel._gazebo_process_signal()
    clock_ok, clock_reason = panel._clock_status()
    topic_ok, topic_reason = panel._gazebo_bridge_signal()
    now = time.monotonic()
    if clock_ok:
        panel._last_clock_ok_ts = now
        panel._gz_clock_stall_since = 0.0
    freeze_sec = max(0.5, panel._gz_health_freeze_sec)
    if proc_ok and (not clock_ok) and panel._gz_clock_stall_since <= 0.0:
        panel._gz_clock_stall_since = now
    if proc_ok and clock_ok:
        panel._gz_orphan_since = 0.0
    topics_required = bool(panel._bridge_running or panel._started_bridge)
    if proc_ok and clock_ok:
        if topics_required and not topic_ok:
            candidate = "GAZEBO_DEGRADED"
        else:
            candidate = "GAZEBO_READY"
    elif proc_ok and (not clock_ok):
        stalled_for = (now - panel._gz_clock_stall_since) if panel._gz_clock_stall_since > 0.0 else 0.0
        if stalled_for > freeze_sec:
            candidate = "GAZEBO_DEGRADED"
        else:
            candidate = "GAZEBO_STARTING"
    elif (not proc_ok) and clock_ok:
        if panel._gz_orphan_since <= 0.0:
            panel._gz_orphan_since = now
        candidate = "GAZEBO_MONITOR_BUG"
    else:
        stale_clock = (now - panel._last_clock_ok_ts) if panel._last_clock_ok_ts > 0.0 else float("inf")
        if panel._gz_state in ("GAZEBO_READY", "GAZEBO_DEGRADED") and stale_clock <= freeze_sec:
            candidate = "GAZEBO_DEGRADED"
        else:
            candidate = "GAZEBO_OFF"
        panel._gz_orphan_since = 0.0
    if candidate != panel._gz_state:
        if candidate != panel._gz_state_pending:
            panel._gz_state_pending = candidate
            panel._gz_state_change_ts = now
        elif (now - panel._gz_state_change_ts) >= 1.0:
            panel._gz_state = candidate
            panel._gz_state_pending = ""
            panel._emit_log(
                f"[STATE] Gazebo={panel._gz_state} reason=s1:{proc_reason}|s2:{clock_reason}|s3:{topic_reason} "
                f"process={str(proc_ok).lower()} clock={str(clock_ok).lower()} "
                f"topics={str(topic_ok).lower()} pid={panel._gz_real_pid or panel._gz_root_pid or 0}"
            )
            if panel._gz_state == "GAZEBO_MONITOR_BUG":
                panel._emit_log_throttled(
                    "GZ_MONITOR_BUG",
                    "[STATE] Gazebo monitor inconsistente: process=false pero /clock avanza; revisando PID/PGID",
                    min_interval=3.0,
            )
    else:
        panel._gz_state_pending = ""
    return panel._gz_state

def _ros2_control_available(panel) -> bool:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False
    return bool(panel._controller_manager_path())

def _controller_manager_path(panel) -> str:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return ""
    try:
        return resolve_controller_manager(panel._moveit_node)
    except Exception as exc:
        _log_exception("resolve controller_manager", exc)
        return "/controller_manager"

def _select_traj_topic(panel) -> str:
    topics = set(panel._list_topic_names())
    if "/joint_trajectory_controller/joint_trajectory" in topics:
        return "/joint_trajectory_controller/joint_trajectory"
    candidates = sorted(t for t in topics if t.endswith("/joint_trajectory_controller/joint_trajectory"))
    if candidates:
        return candidates[0]
    return ""

def _ensure_pose_subscription(panel) -> None:
    if not panel._bridge_running or panel._closing:
        return
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    if not panel.ros_worker.node_ready():
        return
    world_name = panel._gz_world_name or read_world_name(panel.world_combo.currentText().strip()) or GZ_WORLD
    topic = panel._discover_pose_info_topic(world_name)
    panel.ros_worker.subscribe_pose_info(topic)

def _discover_pose_info_topic(panel, world_name: str) -> str:
    """Return pose/info topic, preferring *world_name* when available."""
    expected = f"/world/{world_name}/pose/info"
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return expected
    topics = panel._list_topic_names()
    if not topics:
        return expected
    candidates = [t for t in topics if t.startswith("/world/") and t.endswith("/pose/info")]
    if not candidates:
        return expected
    if expected in candidates:
        return expected
    return sorted(candidates)[0]

def _start_pose_info_watch(panel) -> None:
    if panel._pose_info_timer is None:
        panel._pose_info_timer = QTimer(panel)
        panel._pose_info_timer.setInterval(int(POSE_INFO_POLL_SEC * 1000))
        panel._pose_info_timer.timeout.connect(panel._update_pose_info_status)
    if not panel._pose_info_timer.isActive():
        panel._pose_info_timer.start()
    panel._update_pose_info_status()

def _update_pose_info_status(panel) -> None:
    if panel._closing or not panel._bridge_running:
        return
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return
    count, age, entities, topic = panel.ros_worker.pose_info_details()
    panel._pose_info_msg_count = count
    panel._pose_info_last_age = age
    ready = count > 0 and entities > 0 and age < POSE_INFO_MAX_AGE_SEC
    now = time.monotonic()
    if (now - panel._pose_info_last_log) >= POSE_INFO_LOG_PERIOD:
        panel._emit_log(
            f"[PHYSICS][POSE_INFO] ready={str(ready).lower()} count={count} age={age:.2f}s "
            f"entities={entities} topic={topic or 'n/a'}"
        )
        panel._pose_info_last_log = now
    if ready and not panel._pose_info_ok:
        panel._pose_info_ok = True
        panel._pose_info_ever_ok = True
        panel._pose_info_diag_logged = False
        if panel._metrics_enabled and panel._pose_info_msg_count > 0:
            panel._metric_mark("pose_info_ready")
        if not panel._critical_tf_deadline:
            panel._critical_tf_deadline = time.monotonic() + max(0.1, TF_INIT_GRACE_SEC)
        if panel._gz_running:
            panel._drop_hold_enabled = True
            panel.signal_start_objects_settle_watch.emit()
            QTimer.singleShot(0, panel._schedule_physics_runtime_check)
    if ready and panel._gz_running and not panel._objects_release_done:
        panel._maybe_hold_drop_objects("pose_info")
    elif not ready:
        panel._pose_info_ok = False
        if (now - panel._pose_info_last_log) >= POSE_INFO_LOG_PERIOD:
            panel._emit_log("[PHYSICS][SETTLE] waiting pose/info data...")
            panel._pose_info_last_log = now
        if (now - panel._pose_info_resub_ts) >= POSE_INFO_LOG_PERIOD:
            panel._pose_info_resub_ts = now
            panel._ensure_pose_subscription()
        if not panel._pose_info_diag_logged and count == 0:
            panel._pose_info_diag_logged = True
            try:
                topics = panel.ros_worker.list_topic_names()
            except Exception as exc:
                _log_exception("pose_info list topics", exc)
                topics = []
            pose_topics = [t for t in topics if t.startswith("/world/") and t.endswith("/pose/info")]
            sample_topics = ", ".join(pose_topics[:5]) if pose_topics else "-"
            panel._emit_log(f"[PHYSICS][POSE_INFO] available_pose_topics={sample_topics}")
            helper = get_tf_helper()
            frames = helper.list_frames() if helper else set()
            frames_sample = ", ".join(sorted(frames)[:10]) if frames else "-"
            panel._emit_log(f"[TF] frames_sample={frames_sample}")

def _log_button(panel, label: str):
    panel._emit_log(f"[BTN] {label}")

def _cleanup_stray_processes(panel):
    """Limpiar procesos fantasma de Gazebo, bridge y rosbag al startup."""
    if PANEL_SKIP_CLEANUP:
        panel._emit_log("[STARTUP] Limpieza omitida (PANEL_SKIP_CLEANUP=1)")
        return
    # 1. Limpiar archivos de memoria compartida de FastDDS/FastRTPS
    try:
        panel._emit_log("[STARTUP] Limpiando /dev/shm (FastDDS)")
        subprocess.run(
            ["sh", "-c", "rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null || true"],
            timeout=2,
        )
    except Exception as exc:
        _log_exception("cleanup /dev/shm", exc)

    # 2. Limpiar procesos residuales del stack si procede.
    if not PANEL_KILL_STALE:
        panel._emit_log("[STARTUP] Limpieza de procesos deshabilitada (PANEL_KILL_STALE=0)")
        return
    if not psutil:
        panel._emit_log("[STARTUP] psutil no disponible; no se limpian procesos residuales")
        return
    stale = panel._list_stale_processes()
    if not stale:
        panel._emit_log("[STARTUP] No hay procesos residuales detectados")
        return
    panel._emit_log(f"[STARTUP] Procesos residuales detectados: {len(stale)}. Terminando...")
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

def _clean_cache_dirs(panel):
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
                    panel._log_warning(f"No se pudo borrar {cache_path}: {e}")
        # También borrar archivos .pyc
        for root, _, files in os.walk(ws_parent):
            for f in files:
                if f.endswith(".pyc"):
                    try:
                        os.remove(os.path.join(root, f))
                        removed_count += 1
                    except Exception as e:
                        panel._log_warning(f"No se pudo borrar {f}: {e}")
        panel._log(f"[STARTUP] ✅ Cache limpiado ({removed_count} items)")
    except Exception as e:
        panel._log_error(f"Error limpiando cache: {e}")

def _close_terminal(panel):
    """Cerrar la aplicación del panel."""
    if bool(getattr(panel, "_script_motion_active", False)):
        allow_close = str(os.environ.get("PANEL_ALLOW_CLOSE_WHILE_MOTION", "0")).strip().lower()
        if allow_close not in ("1", "true", "yes", "on"):
            panel._emit_log("[PANEL] Cerrar Terminal bloqueado: movimiento en curso (PICK/TEST)")
            panel._set_status("Cierre bloqueado: espera a que termine el movimiento", error=False)
            return
    if panel._closing:
        return
    panel._closing = True
    try:
        panel.btn_close_terminal.setEnabled(False)
    except Exception as exc:
        _log_exception("disable close button", exc)
    panel._log_button("Cerrar Terminal")
    panel._log("[PANEL] Cerrando panel...")
    # Trigger the standard Qt close flow so closeEvent() runs cleanup.
    QTimer.singleShot(0, panel.close)
def _refresh_camera_topics(panel):
    panel._camera_ctrl.refresh_topics()

def _controllers_ready(panel, *args, **kwargs):
    return _cc._controllers_ready(panel, *args, **kwargs)

def _is_transient_controller_reason(panel, *args, **kwargs):
    return _cc._is_transient_controller_reason(panel, *args, **kwargs)

def _controllers_last_ok_age(panel, *args, **kwargs):
    return _cc._controllers_last_ok_age(panel, *args, **kwargs)

def _can_use_controller_last_ok(panel, *args, **kwargs):
    return _cc._can_use_controller_last_ok(panel, *args, **kwargs)

def _controller_state_kind(panel, *args, **kwargs):
    return _cc._controller_state_kind(panel, *args, **kwargs)

def _list_controllers(panel, *args, **kwargs):
    return _cc._list_controllers(panel, *args, **kwargs)

def _wait_for_controllers_ready(panel, *args, **kwargs):
    return _cc._wait_for_controllers_ready(panel, *args, **kwargs)

def _schedule_camera_health_check(panel, *args, **kwargs):
    _cc._schedule_camera_health_check(panel, *args, **kwargs)

def _check_camera_topic_health(panel, *args, **kwargs):
    _cc._check_camera_topic_health(panel, *args, **kwargs)

def _update_camera_topics(panel, *args, **kwargs):
    _cc._update_camera_topics(panel, *args, **kwargs)

def _connect_camera(panel, *args, **kwargs):
    _cc._connect_camera(panel, *args, **kwargs)

def _switch_camera_topic(panel, *args, **kwargs):
    return _cc._switch_camera_topic(panel, *args, **kwargs)

def _set_far_front_camera_view(panel, *args, **kwargs):
    _cc._set_far_front_camera_view(panel, *args, **kwargs)

def _set_top_camera_view(panel, *args, **kwargs):
    _cc._set_top_camera_view(panel, *args, **kwargs)

def _set_wrist_camera_view(panel, *args, **kwargs):
    _cc._set_wrist_camera_view(panel, *args, **kwargs)

def _subscribe_camera(panel, *args, **kwargs):
    return _cc._subscribe_camera(panel, *args, **kwargs)

def _start_camera_health_check(panel, *args, **kwargs):
    _cc._start_camera_health_check(panel, *args, **kwargs)

def _unsubscribe_camera(panel, *args, **kwargs):
    _cc._unsubscribe_camera(panel, *args, **kwargs)

def _clear_camera_frame(panel, *args, **kwargs):
    _cc._clear_camera_frame(panel, *args, **kwargs)

def _resolve_camera_msg_type(panel, *args, **kwargs):
    return _cc._resolve_camera_msg_type(panel, *args, **kwargs)

def _auto_connect_camera(panel, *args, **kwargs):
    _cc._auto_connect_camera(panel, *args, **kwargs)

def _ensure_ros_worker_started(panel, *args, **kwargs):
    _cc._ensure_ros_worker_started(panel, *args, **kwargs)

def _ensure_grasp_rect_subscription(panel, *args, **kwargs):
    _cc._ensure_grasp_rect_subscription(panel, *args, **kwargs)

def _auto_subscribe_joints(panel, *args, **kwargs):
    _cc._auto_subscribe_joints(panel, *args, **kwargs)

def _discover_joint_states_topic(panel, *args, **kwargs):
    return _cc._discover_joint_states_topic(panel, *args, **kwargs)

def _on_bridge_ready(panel, *args, **kwargs):
    _cc._on_bridge_ready(panel, *args, **kwargs)

def _on_image(panel, *args, **kwargs):
    _cc._on_image(panel, *args, **kwargs)

def _on_grasp_rect(panel, *args, **kwargs):
    _cc._on_grasp_rect(panel, *args, **kwargs)

def _reset_camera_retry_backoff(panel, *args, **kwargs):
    _cc._reset_camera_retry_backoff(panel, *args, **kwargs)

def _refresh_camera_display(panel, *args, **kwargs):
    _cc._refresh_camera_display(panel, *args, **kwargs)

def _check_camera_stream(panel, *args, **kwargs):
    _cc._check_camera_stream(panel, *args, **kwargs)

def _on_joint_state(panel, *args, **kwargs):
    _cc._on_joint_state(panel, *args, **kwargs)


def _fill_worlds(panel):
    panel.world_combo.clear()
    for p in DEFAULT_WORLD_CANDIDATES:
        if os.path.isfile(p):
            panel.world_combo.addItem(p)
    if os.path.isdir(WORLDS_DIR):
        for fn in sorted(os.listdir(WORLDS_DIR)):
            if fn.endswith((".sdf", ".world")):
                full = os.path.join(WORLDS_DIR, fn)
                if full not in DEFAULT_WORLD_CANDIDATES:
                    panel.world_combo.addItem(full)
    if panel.world_combo.count() == 0:
        panel.world_combo.addItem(os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf"))
    panel.world_combo.setCurrentIndex(0)

def _fill_bridge_presets(panel):
    panel.bridge_presets.clear()
    seen = set()
    if os.path.isfile(BRIDGE_BASE_YAML):
        panel.bridge_presets.addItem(BRIDGE_BASE_YAML)
        seen.add(BRIDGE_BASE_YAML)
    if os.path.isdir(SCRIPTS_DIR):
        for fn in sorted(os.listdir(SCRIPTS_DIR)):
            if not fn.endswith(('.yaml', '.yml')):
                continue
            full = os.path.join(SCRIPTS_DIR, fn)
            if full in seen:
                continue
            panel.bridge_presets.addItem(full)
            seen.add(full)
    if panel.bridge_presets.count() == 0:
        panel.bridge_presets.addItem(BRIDGE_BASE_YAML)
    panel.bridge_presets.setCurrentIndex(0)

def _apply_bridge_preset(panel, text: str):
    if text:
        panel.bridge_edit.setText(text)

def _choose_world(panel):
    panel._log_button("Browse mundo")
    path, _ = QFileDialog.getOpenFileName(panel, "Selecciona mundo", WORLDS_DIR, "SDF/WORLD (*.sdf *.world)")
    if path:
        panel.world_combo.setCurrentText(path)

def _choose_yaml(panel):
    panel._log_button("Browse bridge YAML")
    path, _ = QFileDialog.getOpenFileName(panel, "Selecciona YAML del bridge", os.path.dirname(BRIDGE_BASE_YAML), "YAML (*.yaml *.yml)")
    if path:
        panel.bridge_edit.setText(path)

def _run_script(panel, script_name: str, label: str):
    panel._log_button(label)
    path = os.path.join(panel.ws_dir, "scripts", script_name)
    if not os.path.isfile(path):
        panel._set_status(f"No existe {script_name}", error=True)
        return

    def worker():
        panel._ui_set_status(f"Ejecutando {label}…")
        try:
            res = subprocess.run(["bash", path], capture_output=True, text=True, timeout=180)
            if res.returncode == 0:
                panel._ui_set_status(f"OK {label}")
            else:
                panel._ui_set_status(f"Fallo {label} (rc={res.returncode})", error=True)
        except subprocess.TimeoutExpired:
            panel._ui_set_status(f"Timeout {label}", error=True)
        except Exception as exc:
            panel._ui_set_status(f"Error {label}: {exc}", error=True)

    panel._run_async(worker)

def _toggle_debug(panel, env_var: str):
    current = os.environ.get(env_var, "0")
    new_val = "0" if current == "1" else "1"
    os.environ[env_var] = new_val
    enabled = new_val == "1"
    if env_var == "DEBUG_LOGS_TO_STDOUT":
        panel._debug_logs_enabled = enabled
        panel.btn_debug_logs.setChecked(enabled)
        panel._apply_debug_button_style(panel.btn_debug_logs, enabled)
    if env_var == "DEBUG_JOINTS_TO_STDOUT":
        panel.btn_debug_joints.setChecked(enabled)
        panel._apply_debug_button_style(panel.btn_debug_joints, enabled)
        if enabled:
            panel._print_pose_snapshot()
    label = "ON" if enabled else "OFF"
    panel._log_button(f"Toggle {env_var} -> {label}")
    panel._set_status(f"{env_var} -> {label}")

def _start_all(panel):
    if panel._block_if_managed("START"):
        return
    panel._log_button("START")
    if panel._star_inflight:
        panel._set_status("START ya en curso", error=False)
        panel._emit_log("[SAFETY] START duplicado ignorado: secuencia ya en curso")
        return
    if panel._system_running():
        panel._set_status("START ignorado: stack ya activo", error=False)
        panel._emit_log("[SAFETY] START duplicado ignorado: stack ya activo")
        return
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("START bloqueado: ERROR_FATAL activo", error=True)
        panel._emit_log("[SAFETY] START bloqueado: ERROR_FATAL activo")
        return
    # Activar timeouts críticos también en modo no gestionado para fail-fast.
    now = time.monotonic()
    panel._critical_clock_deadline = now + max(0.1, CRITICAL_CLOCK_TIMEOUT_SEC)
    panel._critical_pose_deadline = 0.0
    panel._critical_tf_deadline = 0.0
    panel._critical_camera_deadline = 0.0
    if not panel._moveit_required:
        panel._moveit_required = True
        panel._emit_log("[AUTO] MoveIt requerido activado (START)")
        panel.signal_refresh_controls.emit()
    panel._star_inflight = True
    panel.btn_star.setEnabled(False)
    panel.btn_kill_hard.setEnabled(True)
    sequence = StartSequence(
        emit_log=panel._emit_log,
        run_ui=panel.signal_run_ui.emit,
        run_ui_delayed=panel.signal_run_ui_delayed.emit,
        gazebo_state=panel._gazebo_state,
        is_bridge_running=lambda: panel._bridge_running,
        bridge_ready=lambda: panel._bridge_ready_status()[0],
        moveit_ready=panel._move_group_startup_ready,
        controllers_ready=lambda: panel._controllers_ready()[0],
        is_closing=lambda: panel._closing,
        start_gazebo=panel._start_gazebo,
        start_bridge=panel._start_bridge,
        start_moveit=panel._start_moveit,
        start_moveit_bridge=panel._start_moveit_bridge,
        start_release_service=panel._start_release_service,
        on_fatal=panel._on_start_fatal,
        on_moveit_autostart_blocked=panel._log_moveit_autostart_blocked,
        controllers_timeout_sec=CONTROLLER_READY_TIMEOUT_SEC,
        wait_for_change=panel._wait_for_state_change,
        clock_ready=lambda: panel._clock_status()[0],
        clock_reason=lambda: f"Gazebo/clock no listo ({panel._clock_status()[1]})",
        joint_states_ready=lambda: panel._joint_states_status()[0],
        joint_states_reason=lambda: f"/joint_states no listo ({panel._joint_states_status()[1]})",
        pose_info_ready=panel._pose_info_ready,
        pose_info_reason=panel._pose_info_not_ready_reason,
        tf_ready=lambda: panel._tf_chain_ready_status()[0],
        tf_reason=lambda: f"TF no listo ({panel._tf_chain_ready_status()[1]})",
        camera_ready=lambda: camera_ready_status(panel)[0],
        camera_reason=panel._camera_not_ready_reason,
        camera_required=panel._camera_required,
        clock_timeout_sec=CRITICAL_CLOCK_TIMEOUT_SEC,
        pose_timeout_sec=CRITICAL_POSE_TIMEOUT_SEC,
        tf_timeout_sec=TF_INIT_GRACE_SEC,
        camera_timeout_sec=CAMERA_INIT_GRACE_SEC,
    )
    panel._run_async(sequence.run, name="start_all")

def _log_moveit_autostart_blocked(panel, reason: str) -> None:
    now = _runtime_time()
    camera_ready, camera_fault, camera_source_down, age, in_grace = panel._camera_runtime_flags(now)
    panel._emit_log(
        "[AUTO] MoveIt no autoiniciado: "
        f"razon={reason} "
        f"state={panel._system_state.value} "
        f"gazebo={panel._gazebo_state()} "
        f"controllers={panel._controllers_ok}({panel._controllers_reason or 'ok'}) "
        "camera="
        f"ready={camera_ready},fault={camera_fault},source_down={camera_source_down},"
        f"age={age:.2f}s,grace={in_grace}"
    )

def _on_start_fatal(panel, reason: str) -> None:
    normalized = reason.lower()
    if "camera" in normalized or "cámara" in normalized:
        msg = f"startup degraded: {reason}"
        panel._set_system_state(SystemState.ERROR, msg)
        panel._ui_set_status("ERROR: cámara no lista (diagnóstico)", error=True)
        panel._emit_log(f"[ERROR] {msg}")
        panel._emit_log("[CAMERA][DIAG] Usa Recover o reinicia bridge/cámara")
        return
    if "controller" in normalized or "controlador" in normalized:
        msg = f"startup failed: {reason}"
        panel._system_error_reason = msg
        panel._set_system_state(SystemState.ERROR, msg)
        panel._ui_set_status(f"ERROR: {msg}", error=True)
        panel._emit_log(f"[ERROR] {msg}")
        return
    panel._trigger_fatal(f"startup failed: {reason}")

def _stop_all(panel):
    panel._log_button("STOP")
    sequence = StopSequence(
        emit_log=panel._emit_log,
        run_script=lambda: panel._run_script("kill_all.sh", "STOP"),
        schedule_start_check=panel._schedule_start_enable_check,
        set_star_inflight=lambda value: setattr(panel, "_star_inflight", value),
        set_kill_enabled=panel.btn_kill_hard.setEnabled,
    )
    sequence.run()

def _recover_runtime(panel) -> None:
    panel._log_button("Recover")
    panel._emit_log("[RECOVER] Iniciando recuperación en modo diagnóstico")
    if not panel._objects_release_done and not panel._detach_inflight:
        panel._emit_log("[RECOVER] release pendiente detectado; lanzando Soltar objetos")
        panel._release_objects()
    panel._fatal_latched = False
    panel._fatal_shutdown_started = False
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_system_state(SystemState.ERROR, "recover_requested")
    gz_state = panel._gazebo_state()
    if gz_state in ("GAZEBO_OFF",):
        panel._emit_log(f"[RECOVER] Gazebo={gz_state}; relanzando Gazebo")
        panel._start_gazebo()
        QTimer.singleShot(1800, panel._start_bridge)
        QTimer.singleShot(3200, panel._camera_ctrl.refresh_topics)
        QTimer.singleShot(4200, panel._camera_ctrl.connect)
        return
    if not panel._bridge_running:
        panel._emit_log("[RECOVER] Bridge no activo; relanzando bridge")
        panel._start_bridge()
        QTimer.singleShot(1500, panel._camera_ctrl.refresh_topics)
        QTimer.singleShot(2500, panel._camera_ctrl.connect)
        return
    panel._emit_log("[RECOVER] Reintentando cámara/bridge sin apagar stack")
    panel._camera_ctrl.refresh_topics()
    QTimer.singleShot(900, panel._camera_ctrl.connect)
    panel._camera_ctrl.schedule_health_check(1200)

def _system_running(panel) -> bool:
    if panel._proc_alive(panel.gz_proc) or panel._proc_alive(panel.bridge_proc):
        return True
    if panel._proc_alive(panel.moveit_proc) or panel._proc_alive(panel.moveit_bridge_proc):
        return True
    if panel._proc_alive(panel.bag_proc) or panel._proc_alive(panel.rsp_proc):
        return True
    if panel._gazebo_state() != "GAZEBO_OFF":
        return True
    if panel._bridge_running or panel._moveit_state != MoveItState.OFF:
        return True
    return False

def _schedule_start_enable_check(panel, delay_ms: int = 1200) -> None:
    def _check():
        if not panel._system_running():
            panel.btn_star.setEnabled(True)
            panel.btn_kill_hard.setEnabled(False)
            return
        QTimer.singleShot(delay_ms, _check)

    QTimer.singleShot(delay_ms, _check)

def _start_gazebo(panel):
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("Gazebo bloqueado: ERROR_FATAL activo", error=True)
        panel._emit_log_throttled(
            "SAFETY:start_gazebo:ERROR_FATAL",
            "[SAFETY] Gazebo bloqueado: ERROR_FATAL activo",
        )
        return
    panel_launch_control.start_gazebo(panel)
    return
def _parse_first_json_object(panel, text: str) -> Optional[Dict[str, object]]:
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

def _detect_world_name(panel) -> Optional[str]:
    return panel._physics.detect_world_name()

def _read_world_stats(panel, world_name: str) -> Dict[str, object]:
    return {}

def _try_unpause_world(panel, world_name: str) -> bool:
    return False

def _probe_pose_motion(panel, world_name: str, targets: Set[str]) -> Tuple[Optional[bool], float, Optional[str]]:
    return panel._physics.probe_pose_motion(world_name, targets)

def check_physics_runtime(panel) -> None:
    panel._physics.check_physics_runtime()

def _schedule_physics_runtime_check(panel) -> None:
    panel._physics.schedule_physics_runtime_check()

def _throw_objects(panel):
    panel._log_button("Lanzar objetos")
    panel._set_status("Lanzando objetos en Gazebo…")

    # LEGACY: deshabilitado en modo física real (MoveIt-only / no teletransporte).
    panel._log("[SAFETY] Teleport detectado: bloqueado (throw_objects)")
    panel._set_status("Bloqueado: no se permite teletransporte", error=True)
    return

def _toggle_debug_poses(panel):
    """Toggle on/off streaming de poses y joints hacia el terminal del panel."""
    if panel.btn_debug_joints.isChecked():
        panel._start_debug_poses()
    else:
        panel._stop_debug_poses()

def _start_debug_poses(panel):
    """Inicia streaming de poses parseadas (nombre + posición)."""
    if not panel._gz_running:
        panel._log_error("Gazebo no está activo")
        panel.btn_debug_joints.setChecked(False)
        return
    panel._debug_joints_to_stdout = True
    os.environ["DEBUG_JOINTS_TO_STDOUT"] = "1"
    panel._stop_debug_poses()
    panel._ensure_pose_subscription()
    panel._print_pose_snapshot()
    panel._pose_debug_timer = QTimer(panel)
    panel._pose_debug_timer.timeout.connect(panel._print_pose_snapshot)
    panel._pose_debug_timer.start(int(DEBUG_POSES_PERIOD_SEC * 1000))
    panel._log("[DEBUG] Iniciado - snapshots de poses")
    panel._apply_debug_button_style(panel.btn_debug_joints, True)

def _stop_debug_poses(panel):
    """Detiene streaming de poses y joints."""
    panel._debug_joints_to_stdout = False
    os.environ["DEBUG_JOINTS_TO_STDOUT"] = "0"
    if panel._pose_debug_timer:
        panel._pose_debug_timer.stop()
        panel._pose_debug_timer.deleteLater()
        panel._pose_debug_timer = None
    panel._apply_debug_button_style(panel.btn_debug_joints, False)
    panel._log("[DEBUG] Detenido")

# --- _force_cleanup_leftovers (extracted from panel_v2.py) ---
def _force_cleanup_leftovers(panel) -> None:
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
            panel._emit_log(f"[WARN] Procesos residuales tras cierre:\n{res.stdout.strip()}")
    except Exception as exc:
        _log_exception("pgrep residual processes", exc)
