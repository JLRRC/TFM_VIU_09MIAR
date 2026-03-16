#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_startup.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Startup orchestration helpers for the panel."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass(kw_only=True)
class StartSequence:
    emit_log: Callable[[str], None]
    run_ui: Callable[[Callable[[], None]], None]
    run_ui_delayed: Callable[[Callable[[], None], int], None]
    gazebo_state: Callable[[], str]
    is_bridge_running: Callable[[], bool]
    bridge_ready: Callable[[], bool]
    moveit_ready: Callable[[], bool]
    is_closing: Callable[[], bool]
    start_gazebo: Callable[[], None]
    start_bridge: Callable[[], None]
    start_moveit: Callable[[], None]
    start_moveit_bridge: Callable[[], None]
    controllers_ready: Optional[Callable[[], bool]] = None
    start_release_service: Optional[Callable[[], None]] = None
    on_fatal: Optional[Callable[[str], None]] = None
    on_moveit_autostart_blocked: Optional[Callable[[str], None]] = None
    wait_for_change: Optional[Callable[[float], bool]] = None
    clock_ready: Optional[Callable[[], bool]] = None
    pose_info_ready: Optional[Callable[[], bool]] = None
    tf_ready: Optional[Callable[[], bool]] = None
    joint_states_ready: Optional[Callable[[], bool]] = None
    camera_ready: Optional[Callable[[], bool]] = None
    clock_reason: Optional[Callable[[], str]] = None
    pose_info_reason: Optional[Callable[[], str]] = None
    tf_reason: Optional[Callable[[], str]] = None
    joint_states_reason: Optional[Callable[[], str]] = None
    camera_reason: Optional[Callable[[], str]] = None
    camera_required: bool = False
    gazebo_off_state: str = "GAZEBO_OFF"
    gazebo_timeout_sec: float = 35.0
    bridge_timeout_sec: float = 30.0
    controllers_timeout_sec: float = 20.0
    controllers_stable_sec: float = 2.0
    moveit_timeout_sec: float = 40.0
    clock_timeout_sec: float = 12.0
    pose_timeout_sec: float = 20.0
    tf_timeout_sec: float = 12.0
    camera_timeout_sec: float = 20.0
    poll_sec: float = 0.4

    def run(self) -> None:
        self.emit_log("[AUTO] START sequence begin")
        if self.gazebo_state() == self.gazebo_off_state:
            self.emit_log("[AUTO] START: launching Gazebo")
            self.run_ui(self.start_gazebo)
        if not self._wait_step(
            "gazebo_process",
            lambda: self.gazebo_state() != self.gazebo_off_state,
            self.gazebo_timeout_sec,
        ):
            msg = "[AUTO] START: Gazebo timeout"
            self.emit_log(msg)
            self._notify_moveit_autostart_blocked("gazebo_timeout")
            if self.on_fatal:
                self.run_ui(lambda: self.on_fatal(msg))
            return
        self.emit_log("[AUTO] START: launching bridge")
        self.run_ui(self.start_bridge)
        if not self._wait_step(
            "gazebo_ready",
            lambda: self.gazebo_state() == "GAZEBO_READY",
            self.gazebo_timeout_sec,
        ):
            msg = "[AUTO] START: Gazebo/clock timeout"
            self.emit_log(msg)
            self._notify_moveit_autostart_blocked("gazebo_not_ready")
            if self.on_fatal:
                self.run_ui(lambda: self.on_fatal(msg))
            return
        if self.start_release_service is not None:
            self.run_ui_delayed(self.start_release_service, 200)
        bridge_predicate = self.bridge_ready or self.is_bridge_running
        if not self._wait_step("bridge", bridge_predicate, self.bridge_timeout_sec):
            msg = "[AUTO] START: bridge timeout"
            self.emit_log(msg)
            self._notify_moveit_autostart_blocked("bridge_timeout")
            if self.on_fatal:
                self.run_ui(lambda: self.on_fatal(msg))
            return
        if self.clock_ready is not None:
            if not self._wait_step("clock", self.clock_ready, self.clock_timeout_sec):
                reason = self._resolve_reason(self.clock_reason, "Gazebo/clock no listo")
                msg = f"[AUTO] START: {reason}"
                self.emit_log(msg)
                self._notify_moveit_autostart_blocked("clock_not_ready")
                if self.on_fatal:
                    self.run_ui(lambda: self.on_fatal(reason))
                return
        if self.joint_states_ready is not None:
            if not self._wait_step("joint_states", self.joint_states_ready, self.pose_timeout_sec):
                reason = self._resolve_reason(
                    self.joint_states_reason,
                    "/joint_states no disponible",
                )
                msg = f"[AUTO] START: {reason}"
                self.emit_log(msg)
                self._notify_moveit_autostart_blocked("joint_states_not_ready")
                if self.on_fatal:
                    self.run_ui(lambda: self.on_fatal(reason))
                return
        if self.pose_info_ready is not None:
            if not self._wait_step("pose_info", self.pose_info_ready, self.pose_timeout_sec):
                reason = self._resolve_reason(self.pose_info_reason, "pose/info no disponible")
                msg = f"[AUTO] START: {reason}"
                self.emit_log(msg)
                self._notify_moveit_autostart_blocked("pose_info_not_ready")
                if self.on_fatal:
                    self.run_ui(lambda: self.on_fatal(reason))
                return
        if self.tf_ready is not None:
            if not self._wait_step("tf", self.tf_ready, self.tf_timeout_sec):
                reason = self._resolve_reason(self.tf_reason, "TF world->base no disponible")
                msg = f"[AUTO] START: {reason}"
                self.emit_log(msg)
                self._notify_moveit_autostart_blocked("tf_not_ready")
                if self.on_fatal:
                    self.run_ui(lambda: self.on_fatal(reason))
                return
        if self.camera_required and self.camera_ready is not None:
            if not self._wait_until(self.camera_ready, self.camera_timeout_sec):
                reason = self._resolve_reason(self.camera_reason, "Cámara no publica frames")
                msg = f"[AUTO] START: cámara en warmup/degradada; se permite MoveIt ({reason})"
                self.emit_log(msg)
        if self.controllers_ready is not None:
            if not self._wait_until_stable(
                self.controllers_ready,
                self.controllers_stable_sec,
                self.controllers_timeout_sec,
            ):
                msg = "[AUTO] START: controllers timeout"
                self.emit_log(msg)
                self._notify_moveit_autostart_blocked("controllers_timeout")
                # Dejar el sistema en modo manual en vez de abortar con error fatal.
                return
            if not self.controllers_ready():
                msg = "[AUTO] START: controllers inestables antes de MoveIt"
                self.emit_log(msg)
                self._notify_moveit_autostart_blocked("controllers_unstable")
                # Dejar el sistema en modo manual en vez de abortar con error fatal.
                return
        self.emit_log("[AUTO] START: launching MoveIt")
        self.run_ui(self.start_moveit)
        if not self._wait_step("moveit", self.moveit_ready, self.moveit_timeout_sec):
            msg = "[AUTO] START: MoveIt timeout"
            self.emit_log(msg)
            self._notify_moveit_autostart_blocked("moveit_timeout")
            if self.on_fatal:
                self.run_ui(lambda: self.on_fatal(msg))
            return
        self.emit_log("[AUTO] START: launching MoveIt bridge")
        self.run_ui(self.start_moveit_bridge)
        self.emit_log("[AUTO] START sequence end")

    def _wait_step(
        self,
        label: str,
        predicate: Callable[[], bool],
        timeout_sec: float,
    ) -> bool:
        start = time.monotonic()
        ok = self._wait_until(predicate, timeout_sec)
        elapsed = time.monotonic() - start
        if ok:
            self.emit_log(f"[AUTO][WAIT] {label}=OK t={elapsed:.2f}s")
        else:
            self.emit_log(f"[AUTO][WAIT] {label}=FAIL t={elapsed:.2f}s timeout={timeout_sec:.1f}s")
        return ok

    @staticmethod
    def _resolve_reason(reason_fn: Optional[Callable[[], str]], fallback: str) -> str:
        if reason_fn is None:
            return fallback
        try:
            return reason_fn() or fallback
        except Exception:
            return fallback

    def _wait_until(self, predicate: Callable[[], bool], timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.1, timeout_sec)
        while time.monotonic() < deadline:
            if self.is_closing():
                return False
            if predicate():
                return True
            if self.wait_for_change is not None:
                self.wait_for_change(self.poll_sec)
            else:
                time.sleep(self.poll_sec)
        return False

    def _wait_until_stable(
        self,
        predicate: Callable[[], bool],
        stable_sec: float,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + max(0.1, timeout_sec)
        stable_start: Optional[float] = None
        stable_required = max(0.1, stable_sec)
        while time.monotonic() < deadline:
            if self.is_closing():
                return False
            if predicate():
                if stable_start is None:
                    stable_start = time.monotonic()
                if (time.monotonic() - stable_start) >= stable_required:
                    return True
            else:
                stable_start = None
            if self.wait_for_change is not None:
                self.wait_for_change(self.poll_sec)
            else:
                time.sleep(self.poll_sec)
        return False

    def _notify_moveit_autostart_blocked(self, reason: str) -> None:
        if self.on_moveit_autostart_blocked is None:
            return
        self.run_ui(lambda: self.on_moveit_autostart_blocked(reason))
