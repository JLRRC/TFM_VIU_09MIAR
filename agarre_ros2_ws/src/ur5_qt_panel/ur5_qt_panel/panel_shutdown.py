#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_shutdown.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Shutdown helpers for the panel."""
from __future__ import annotations

import os
import signal
from dataclasses import dataclass
from typing import Callable, Optional

from .logging_utils import timestamped_line

_DEBUG_EXCEPTIONS = os.environ.get("PANEL_DEBUG_EXCEPTIONS", "").strip() in ("1", "true", "True")


def _log_exception(context: str, exc: Exception) -> None:
    if not _DEBUG_EXCEPTIONS:
        return
    print(timestamped_line(f"[SHUTDOWN][WARN] {context}: {exc}"), flush=True)


@dataclass
class StopSequence:
    emit_log: Callable[[str], None]
    run_script: Callable[[], None]
    schedule_start_check: Callable[[], None]
    set_star_inflight: Callable[[bool], None]
    set_kill_enabled: Callable[[bool], None]

    def run(self) -> None:
        self.emit_log("[STOP] sequence begin")
        self.set_star_inflight(False)
        self.set_kill_enabled(False)
        self.run_script()
        self.schedule_start_check()
        self.emit_log("[STOP] sequence end")


def terminate_process(proc, label: str, log_fn: Optional[Callable[[str], None]] = None, timeout_sec: float = 2.0) -> None:
    if proc is None:
        return
    try:
        proc.terminate()
    except Exception as exc:
        _log_exception("terminate process", exc)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception as exc:
        _log_exception("kill process group SIGTERM", exc)
    try:
        proc.wait(timeout=timeout_sec)
        return
    except Exception as exc:
        _log_exception("wait process", exc)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception as exc:
        _log_exception("kill process group SIGKILL", exc)
    if log_fn:
        log_fn(f"[{label}] proceso terminado")

# --- closeEvent (extracted from panel_v2.py) ---
def closeEvent(panel, event):
    if panel._shutdown_complete:
        event.accept()
        return
    panel._closing = True
    panel._emit_log("[SHUTDOWN][PANEL] begin")
    panel._set_step_mode("AUTO", emit_log=False)
    panel._emit_log("[SHUTDOWN][PANEL] stop_step_mode ok")
    panel._step_wait_event.set()
    if panel._step_window is not None:
        panel._step_window.close()
    if panel._step_cart_debug_window is not None:
        panel._step_cart_debug_window.close()
    panel._bridge_running = False
    panel._gz_running = False
    panel._log("[SHUTDOWN][PANEL] stop_timers begin")
    for timer in (
        panel._trace_timer,
        panel._tf_ready_timer,
        panel._pose_debug_timer,
        panel._pose_info_timer,
        getattr(panel, "_camera_health_timer", None),
        getattr(panel, "_drop_hold_timer", None),
        getattr(panel, "_watchdog_timer", None),
        getattr(panel, "objects_timer", None),
        getattr(panel, "joint_timer", None),
    ):
        if timer:
            timer.stop()
    panel._log("[SHUTDOWN][PANEL] stop_timers ok")
    panel._trace_ready = False
    panel._reset_trace_throttle("panel close")
    panel._log("[SHUTDOWN][PANEL] stop_workers begin")
    panel.ros_worker.stop_and_join()
    panel._log("[SHUTDOWN][PANEL] stop_workers ros_worker ok")
    panel._log("[SHUTDOWN][PANEL] stop_workers tf_helper begin")
    shutdown_tf_helper()
    panel._log("[SHUTDOWN][PANEL] stop_workers tf_helper ok")
    for thread_name, thread in (("settle_thread", getattr(panel, "_settle_thread", None)),):
        if thread is None:
            continue
        try:
            running = bool(thread.isRunning())
        except RuntimeError:
            panel._emit_log(
                f"[SHUTDOWN][PANEL] thread_join name={thread_name} ok=true timeout=0.0 note=already_deleted"
            )
            continue
        if running:
            thread.quit()
            joined = bool(thread.wait(1000))
            panel._emit_log(
                f"[SHUTDOWN][PANEL] thread_join name={thread_name} ok={str(joined).lower()} timeout=1.0"
            )
    for idx, thread in enumerate(list(getattr(panel, "_async_threads", []))):
        if thread is None:
            continue
        try:
            running = bool(thread.isRunning())
        except RuntimeError:
            panel._emit_log(
                f"[SHUTDOWN][PANEL] thread_join name=async_{idx} ok=true timeout=0.0 note=already_deleted"
            )
            continue
        if running:
            thread.quit()
            joined = bool(thread.wait(1000))
            panel._emit_log(
                f"[SHUTDOWN][PANEL] thread_join name=async_{idx} ok={str(joined).lower()} timeout=1.0"
            )
    panel._log("[SHUTDOWN][PANEL] stop_workers ok")
    panel._kill_proc(panel.bag_proc, "ros2 bag record")
    # FASE 8: Stop MoveIt bridge and move_group BEFORE killing the Gazebo
    # bridge and simulators so in-flight plans drain cleanly.
    panel._log("[TRACE] Shutdown: stopping MoveIt bridge")
    panel._kill_proc(panel.moveit_bridge_proc, "ur5_moveit_bridge")
    panel.moveit_bridge_proc = None
    panel._log("[TRACE] Shutdown: stopping move_group")
    terminate_process(panel.moveit_proc, "move_group", log_fn=panel._log, timeout_sec=5.0)
    panel.moveit_proc = None
    panel._kill_proc(panel.bridge_proc, "parameter_bridge")
    panel._kill_proc(panel.gz_pose_proc, "gz_pose_bridge")
    panel._kill_proc(panel.release_service_proc, "release_objects_service")
    panel._kill_proc(panel.world_tf_proc, "world_tf_publisher")
    panel._kill_proc(panel.rsp_proc, "robot_state_publisher")
    panel._kill_proc(panel.gz_gui_proc, "gz sim gui")
    panel._kill_proc(panel.gz_proc, "gz sim")
    panel.bag_proc = None
    panel.bridge_proc = None
    panel.gz_pose_proc = None
    panel.release_service_proc = None
    panel.world_tf_proc = None
    panel.rsp_proc = None
    panel.gz_gui_proc = None
    panel.gz_proc = None
    panel._force_cleanup_leftovers()
    if panel._moveit_node is not None:
        try:
            panel._moveit_node.destroy_node()
        except Exception as exc:
            _log_exception("destroy moveit node", exc)
        panel._moveit_node = None
        panel._moveit_pose_pub = None
    try:
        panel._log("[SHUTDOWN][PANEL] ros_shutdown begin")
        rclpy.try_shutdown()
        panel._log("[SHUTDOWN][PANEL] ros_shutdown ok")
    except Exception as exc:
        _log_exception("rclpy.try_shutdown", exc)
    panel._emit_log("[SHUTDOWN][PANEL] qt_close begin")
    panel._emit_log("[SHUTDOWN][PANEL] qt_close ok")
    panel._emit_log("[SHUTDOWN][PANEL] done")
    panel._shutdown_complete = True
    super().closeEvent(event)
