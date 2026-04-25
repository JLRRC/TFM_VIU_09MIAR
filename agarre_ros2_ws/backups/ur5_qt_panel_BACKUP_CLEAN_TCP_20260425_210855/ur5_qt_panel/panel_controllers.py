#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_controllers.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Controller spawn helpers for the panel."""
from __future__ import annotations

import subprocess
import time

from .panel_config import CONTROLLER_READY_TIMEOUT_SEC
from .panel_utils import bash_preamble, gripper_controller_defined, with_line_buffer

def _controller_bootstrap_cli(
    ws_dir: str,
    cm_path: str,
    controllers: list[str],
) -> str:
    controllers_yaml = "[" + ", ".join(controllers) + "]"
    cmd_core = (
        "ros2 run ur5_tools controller_bootstrap --ros-args "
        f"-p controller_manager:={cm_path} "
        f"-p required_controllers:=\"{controllers_yaml}\" "
    )
    return bash_preamble(ws_dir) + with_line_buffer(cmd_core)


def spawn_controllers_async(panel) -> None:
    def worker():
        if panel._controller_spawn_done and panel._controllers_ok:
            return
        if panel._controller_spawn_inflight:
            return
        panel._controller_spawn_last_start = time.time()
        panel._controller_spawn_inflight = True
        for _ in range(20):
            if panel._ros2_control_available():
                break
            time.sleep(0.25)
        if not panel._ros2_control_available():
            panel._log_error("controller_manager no disponible; no se pueden spawnear controladores")
            panel._controller_spawn_inflight = False
            return
        clock_ok = False
        if panel._ros_worker_started:
            clock_ok = panel.ros_worker.wait_for_clock(timeout_sec=30.0)
        if not clock_ok:
            panel._log_error("/clock no disponible; abortando spawners")
            panel._controller_spawn_inflight = False
            return

        try:
            panel._emit_log("[CTRL] Ajustando parámetros de gz_ros_control…")
            cmd_core = with_line_buffer(
                "ros2 run ur5_tools gz_ros_control_guard "
                "--ros-args -p use_sim_time:=true -p hold_joints:=false"
            )
            cmd = bash_preamble(panel.ws_dir) + cmd_core
            if panel._debug_logs_enabled:
                subprocess.run(["bash", "-lc", cmd], timeout=12)
            else:
                subprocess.run(
                    ["bash", "-lc", cmd],
                    timeout=12,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
        except Exception as exc:
            panel._log_warning(f"[CTRL] gz_ros_control_guard falló: {exc}")
        controllers = ["joint_state_broadcaster", "joint_trajectory_controller"]
        if gripper_controller_defined():
            controllers.append("gripper_controller")
        else:
            panel._emit_log("[CTRL] gripper_controller no definido; omitido.")

        cm_path = panel._controller_manager_path()
        cmd = _controller_bootstrap_cli(panel.ws_dir, cm_path, controllers)
        try:
            if panel._ros_worker_started and panel.ros_worker.has_service("/controller_bootstrap/run"):
                panel._emit_log("[CTRL] Servicio controller_bootstrap disponible; usando Trigger.")
                if panel.ros_worker.call_trigger("/controller_bootstrap/run", timeout_sec=20.0):
                    panel._emit_log("[CTRL] controller_bootstrap (servicio) completado.")
                    panel._controller_spawn_inflight = False
                    panel._controller_spawn_done = True
                    return
                panel._log_warning("[CTRL] controller_bootstrap (servicio) falló; esperando controladores.")
                ok, _reason = panel._wait_for_controllers_ready(CONTROLLER_READY_TIMEOUT_SEC)
                if ok:
                    panel._emit_log("[CTRL] controller_bootstrap completado (controladores listos).")
                    panel._controller_spawn_inflight = False
                    panel._controller_spawn_done = True
                    return
                panel._log_error("[CTRL] controller_bootstrap (servicio) falló; controladores no listos.")
                panel._controller_spawn_inflight = False
                panel._controller_spawn_done = False
                return
            panel._emit_log("[CTRL] Ejecutando controller_bootstrap…")
            if panel._debug_logs_enabled:
                res = subprocess.run(["bash", "-lc", cmd], timeout=40)
            else:
                res = subprocess.run(
                    ["bash", "-lc", cmd],
                    timeout=40,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            if res.returncode == 0:
                panel._emit_log("[CTRL] controller_bootstrap completado.")
                panel._controller_spawn_inflight = False
                panel._controller_spawn_done = True
                return
            panel._log_error(f"[CTRL] controller_bootstrap falló rc={res.returncode}")
        except Exception as exc:
            panel._log_error(f"[CTRL] controller_bootstrap falló: {exc}")
        panel._controller_spawn_inflight = False
        panel._controller_spawn_done = False
        return

    panel._run_async(worker)
