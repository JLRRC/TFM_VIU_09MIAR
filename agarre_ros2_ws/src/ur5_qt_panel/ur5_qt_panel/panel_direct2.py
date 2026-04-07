#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_direct2.py
# Contenido: Controlador comun para la secuencia DIRECTO2 del panel UR5.
# Uso breve: Lo usan tanto el boton de la GUI como el disparo remoto por ROS 2.
"""Shared DIRECTO2 controller for GUI and ROS 2 triggers."""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from .panel_objects import recalc_object_states
from .panel_robot_presets import (
    JOINT_BASKET_POSE_RAD,
    JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD,
    JOINT_TABLE_POSE_RAD,
    PICK_DEMO_OBJECT_NAME,
)


class Directo2Controller:
    """Owns the single execution entry point for DIRECTO2."""

    def __init__(self, panel) -> None:
        self._panel = panel
        self._state_lock = threading.Lock()
        self._inflight = False

    def request_run(
        self,
        *,
        source: str = "unknown",
        on_done: Optional[Callable[[bool, str], None]] = None,
    ) -> tuple[bool, str]:
        panel = self._panel
        if getattr(panel, "_pick_moveit_phase_active", False):
            panel._set_status("Directo2 bloqueado: PICK_OBJ MoveIt en ejecución", error=True)
            panel._emit_log("[DIRECT2] BLOCKED: MoveIt pick object en ejecución")
            self._notify(on_done, False, "blocked_moveit_active")
            return False, "blocked_moveit_active"
        if not panel._require_manual_ready("Directo2"):
            self._notify(on_done, False, "manual_not_ready")
            return False, "manual_not_ready"
        with self._state_lock:
            if self._inflight or getattr(panel, "_script_motion_active", False):
                panel._set_status("Directo2 en curso…", error=False)
                self._notify(on_done, False, "busy")
                return False, "busy"
            self._inflight = True

        move_sec = float(panel.joint_time.value()) if panel.joint_time else 2.0
        settle_sec = 0.35
        gripper_wait_sec = 1.0
        sequence = (
            ("MESA_1", JOINT_TABLE_POSE_RAD),
            ("POSE_BUENA", JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD),
            ("MESA_2", JOINT_TABLE_POSE_RAD),
            ("CESTA", JOINT_BASKET_POSE_RAD),
        )

        def _sleep_until_done(delay_sec: float) -> None:
            end = time.monotonic() + max(0.0, delay_sec)
            while time.monotonic() < end:
                if panel._closing:
                    break
                time.sleep(0.05)

        reported = False

        def _finish(success: bool, message: str) -> None:
            nonlocal reported
            if reported:
                return
            reported = True
            self._notify(on_done, success, message)

        def worker() -> None:
            panel._set_motion_lock(True)
            try:
                panel._emit_log(
                    "[DIRECT2] start "
                    f"source={source or 'unknown'} "
                    f"object={PICK_DEMO_OBJECT_NAME} move_sec={move_sec:.2f} "
                    f"gripper_wait_sec={gripper_wait_sec:.2f}"
                )
                for step_idx, (step_name, joints) in enumerate(sequence):
                    status_label = {
                        "MESA_1": "Directo2: yendo a Mesa…",
                        "POSE_BUENA": "Directo2: yendo a Pose Buena…",
                        "MESA_2": "Directo2: volviendo a Mesa…",
                        "CESTA": "Directo2: yendo a Cesta…",
                    }.get(step_name, "Directo2 ejecutando…")
                    panel._ui_set_status(status_label)
                    panel._set_direct2_visual_target(list(joints), step_name)
                    ok, info = panel._publish_joint_trajectory(list(joints), move_sec)
                    if not ok:
                        panel._ui_set_status(f"Directo2 falló en {step_name}: {info}", error=True)
                        panel._emit_log(f"[DIRECT2] fail step={step_name} reason={info}")
                        _finish(False, f"trajectory_failed:{step_name}:{info}")
                        return
                    panel._emit_log(
                        f"[DIRECT2] step={step_name} action=joint_trajectory topic={info}"
                    )
                    _sleep_until_done(move_sec + settle_sec)
                    next_idx = step_idx + 1
                    if next_idx < len(sequence):
                        next_step_name, next_joints = sequence[next_idx]
                        panel._set_direct2_visual_target(list(next_joints), next_step_name)
                    else:
                        panel._set_direct2_visual_target(None)
                    if step_name == "MESA_1":
                        panel._ui_set_status("Directo2: abriendo pinza…")
                        panel._command_gripper(False, log_action="DIRECT2", force=True)
                        panel._emit_log("[DIRECT2] step=MESA_1 action=gripper_open")
                        _sleep_until_done(gripper_wait_sec)
                    elif step_name == "POSE_BUENA":
                        panel._ui_set_status("Directo2: cerrando pinza…")
                        panel._command_gripper(True, log_action="DIRECT2", force=True)
                        panel._emit_log("[DIRECT2] step=POSE_BUENA action=gripper_close")
                        try:
                            import math as _math

                            from .panel_robot_presets import (
                                JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD as _JREF,
                            )
                            from .panel_utils import get_pose as _get_pose
                            from .ur5_kinematics import fk_ur5 as _fk_ur5

                            tcp_z = 0.175
                            fk_pose = _fk_ur5(list(_JREF))
                            tool0 = (-float(fk_pose[0][0]), -float(fk_pose[0][1]), float(fk_pose[0][2]))
                            tcp = (tool0[0], tool0[1], tool0[2] + tcp_z)
                            obj = None
                            try:
                                obj_pose = (
                                    _get_pose(
                                        panel.ros_worker,
                                        "base_link",
                                        PICK_DEMO_OBJECT_NAME,
                                        timeout_sec=0.2,
                                    )
                                    if getattr(panel, "ros_worker", None)
                                    else None
                                )
                                if obj_pose is not None:
                                    obj_pos = obj_pose.position
                                    obj = (float(obj_pos.x), float(obj_pos.y), float(obj_pos.z))
                            except Exception:
                                obj = None
                            xy = None
                            d3 = None
                            if obj is not None:
                                dx = tcp[0] - obj[0]
                                dy = tcp[1] - obj[1]
                                dz = tcp[2] - obj[2]
                                xy = _math.sqrt(dx * dx + dy * dy)
                                d3 = _math.sqrt(dx * dx + dy * dy + dz * dz)
                            panel._emit_log(
                                "[COMPARE][DIRECT2][GOOD_REF] "
                                f"step=POSE_BUENA "
                                f"preset=JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD "
                                f"tool0_base=({tool0[0]:.3f},{tool0[1]:.3f},{tool0[2]:.3f}) "
                                f"rg2_tcp_base=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) "
                                f"object_base={'({:.3f},{:.3f},{:.3f})'.format(*obj) if obj else 'N/A'} "
                                f"xy_dist_rg2tcp_obj={'{:.4f}'.format(xy) if xy is not None else 'N/A'} "
                                f"dist3d_rg2tcp_obj={'{:.4f}'.format(d3) if d3 is not None else 'N/A'} "
                                f"verdict={'PRESET_VALID' if xy is not None and xy < 0.05 else 'PRESET_POSITION_MISMATCH'}"
                            )
                        except Exception as exc:
                            panel._emit_log(f"[COMPARE][DIRECT2][GOOD_REF] error={exc}")
                        _sleep_until_done(gripper_wait_sec)
                    elif step_name == "CESTA":
                        panel._ui_set_status("Directo2: soltando en cesta…")
                        panel._command_gripper(False, log_action="DIRECT2", force=True)
                        panel._emit_log("[DIRECT2] step=CESTA action=gripper_open")
                        _sleep_until_done(gripper_wait_sec)
                panel._ui_set_status("Directo2 completado")
                panel._emit_log(
                    "[DIRECT2] completed route=mesa_open_pose_buena_close_mesa_cesta_open"
                )
                _finish(True, "completed")
            except Exception as exc:
                _finish(False, f"exception:{exc}")
                raise
            finally:
                panel._set_direct2_visual_target(None)
                panel._set_motion_lock(False)
                recalc_object_states("direct2_sequence")
                with self._state_lock:
                    self._inflight = False

        try:
            panel._run_async(worker, name="pick_demo_direct2")
        except Exception as exc:
            with self._state_lock:
                self._inflight = False
            self._notify(on_done, False, f"launch_exception:{exc}")
            return False, f"launch_exception:{exc}"
        return True, "started"

    @staticmethod
    def _notify(
        callback: Optional[Callable[[bool, str], None]],
        success: bool,
        message: str,
    ) -> None:
        if callback is None:
            return
        callback(bool(success), str(message or ""))
