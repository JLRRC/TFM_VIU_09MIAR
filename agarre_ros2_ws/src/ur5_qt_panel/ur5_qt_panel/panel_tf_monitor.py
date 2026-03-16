#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tf_monitor.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""TF readiness monitor for the panel."""
from __future__ import annotations

import os
import time
from typing import Optional, TYPE_CHECKING

from PyQt5.QtCore import QTimer

from .panel_utils import (
    _can_transform_between,
    _list_tf_topics,
    get_tf_helper,
)
from .panel_tf_diagnose import log_missing_ee_frames

if TYPE_CHECKING:
    from .panel_tf import TfHelper
    from .panel_v2 import ControlPanelV2


class TFMonitor:
    """TF readiness watchdog isolated from UI routines."""

    def __init__(self, panel: "ControlPanelV2") -> None:
        self._panel = panel

    def start(self) -> None:
        p = self._panel
        if p._tf_ready_timer is None:
            p._tf_ready_timer = QTimer(p)
            p._tf_ready_timer.setInterval(500)
            p._tf_ready_timer.timeout.connect(self.try_mark_ready)
        p._tf_ready_timer.start()

    def stop(self) -> None:
        p = self._panel
        if p._tf_ready_timer:
            p._tf_ready_timer.stop()

    def try_mark_ready(self) -> None:
        p = self._panel
        try:
            if p._closing or not p._bridge_running or p._trace_ready:
                self.stop()
                return
            prev_state = p._tf_ready_state
            helper = get_tf_helper()
            now = time.monotonic()
            if helper is None:
                if (now - getattr(p, "_last_tf_diag_log", 0.0)) > 1.5:
                    p._emit_log("[TF][DIAG] helper no disponible, tf_ready_state=False")
                    p._last_tf_diag_log = now
                p._tf_ready_state = False
                p._maybe_log_tf_not_ready()
                if prev_state:
                    p.signal_refresh_controls.emit()
                return
            tf_stats = helper.tf_listener_stats()
            if tf_stats[0] == 0 and tf_stats[1] == 0:
                if (now - getattr(p, "_last_tf_diag_log", 0.0)) > 1.5:
                    p._emit_log(
                        "[TF][DIAG] No llegan mensajes a /tf o /tf_static. Falta robot_state_publisher o la cadena de publish TF. Bloqueando pick."
                    )
                    p._last_tf_diag_log = now
                p._tf_no_msgs_logged = True
            elif tf_stats[0] > 0 or tf_stats[1] > 0:
                p._tf_no_msgs_logged = False
            world_frame = p._world_frame_last_first()
            final_base = self.wait_for_ready(world_frame, helper)
            if final_base:
                if not p._tf_ready_state:
                    p._log(f"[TRACE] TF ready (base={final_base})")
                p._tf_ready_state = True
                p._tf_ever_ok = True
                p._tf_not_ready_logged = False
                p._base_frame_effective = final_base
                p._bridge_ready = True
                p.signal_trace_ready.emit()
                if not prev_state:
                    p.signal_tf_ready.emit(True)
                    p.signal_refresh_controls.emit()
                return
            frames = helper.list_frames() if helper else set()
            log_missing_ee_frames(p, frames, now)
            p._tf_ready_state = False
            p._maybe_log_tf_not_ready()
            if prev_state:
                p.signal_tf_ready.emit(False)
                p.signal_refresh_controls.emit()
            elif p._system_error_reason:
                p.signal_refresh_controls.emit()
        except Exception as exc:
            p._log_error(f"[TRACE][ERROR] TF readiness error: {exc}")

    def wait_for_ready(self, world_frame: str, helper: Optional["TfHelper"]) -> Optional[str]:
        p = self._panel
        if helper is None:
            return None
        tf_topics, tf_static = _list_tf_topics()
        if not tf_topics and not tf_static:
            return None
        base_frame = "base_link"
        required_ee = (
            str(getattr(p, "_required_ee_frame", "") or "")
            or str(os.environ.get("PANEL_REQUIRED_EE_FRAME", "rg2_tcp") or "rg2_tcp")
        ).strip() or "rg2_tcp"
        frames = helper.list_frames()
        if "base" in frames and "base_link" in frames:
            base_rel = helper.lookup_transform("base_link", "base", timeout_sec=0.05)
            if base_rel is not None:
                t = base_rel.transform.translation
                if abs(float(t.x)) > 1e-4 or abs(float(t.y)) > 1e-4 or abs(float(t.z)) > 1e-4:
                    p._emit_log(
                        "[FRAME][P0] Transform base_link<-base no trivial; base queda prohibido como frame de negocio"
                    )
        if base_frame not in frames:
            now = time.monotonic()
            if (now - getattr(p, "_last_tf_diag_log", 0.0)) > 1.5:
                p._emit_log("[TF][DIAG] Falta frame base_link en TF; esperando robot_state_publisher")
                p._last_tf_diag_log = now
            return None
        if _can_transform_between(helper, base_frame, required_ee, timeout_sec=0.15):
            p._ee_frame_effective = required_ee
            return base_frame
        now = time.monotonic()
        if (now - getattr(p, "_last_tf_diag_log", 0.0)) > 1.5:
            p._emit_log(
                f"[TF][DIAG] Falta transform {base_frame}<->{required_ee}; esperando TF EE para habilitar TEST/PICK/MoveIt"
            )
            p._last_tf_diag_log = now
        return None
