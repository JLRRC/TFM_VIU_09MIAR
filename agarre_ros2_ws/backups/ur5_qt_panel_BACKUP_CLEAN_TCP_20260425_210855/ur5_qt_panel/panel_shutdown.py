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
