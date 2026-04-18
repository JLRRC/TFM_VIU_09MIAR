#!/usr/bin/env python3
"""Launch the real panel offscreen and trigger DIRECTO2 through the real UI path.

Analogous to run_directo_button_offscreen.py but for the DIRECTO2 button.
Patches _pick_confirm_dialog to auto-accept (no blocking modal) and calls
_direct2_controller.request_run() directly to bypass the dialog path.
"""

from __future__ import annotations

import os
import signal
import sys
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from ur5_qt_panel.panel_v2 import ControlPanelV2


CLICK_DELAY_MS = int(os.environ.get("DIRECTO2_CLICK_DELAY_MS", "5000") or "5000")
EXIT_AFTER_MS = int(os.environ.get("DIRECTO2_EXIT_AFTER_MS", "120000") or "120000")
RETRY_MS = int(os.environ.get("DIRECTO2_RETRY_MS", "1500") or "1500")
MAX_ATTEMPTS = int(os.environ.get("DIRECTO2_MAX_ATTEMPTS", "60") or "60")
AUTO_ACCEPT_CONFIRM = str(
    os.environ.get("DIRECTO2_AUTO_ACCEPT_CONFIRM", "1") or "1"
).strip().lower() in ("1", "true", "yes", "on")


def main() -> int:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("PANEL_FORCE_OFFSCREEN", "1")
    os.environ.setdefault("PANEL_START_STACK", "0")

    def _stamp() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%S")

    app = QApplication(sys.argv)
    panel = ControlPanelV2()

    if AUTO_ACCEPT_CONFIRM:
        def _auto_confirm(_parent, btn_label: str, frame: str, pose_str: str) -> bool:
            panel._emit_log(
                "[AUDIT][DIRECTO2] offscreen helper auto-accepting confirm dialog "
                f"button={btn_label} frame={frame} pose={pose_str}"
            )
            return True
        panel._pick_confirm_dialog = _auto_confirm

    panel.show()
    print(f"[{_stamp()}] [SHUTDOWN][HELPER] app_started", flush=True)

    retry_timer = QTimer(panel)
    retry_timer.setInterval(RETRY_MS)
    retry_timer.setSingleShot(False)

    state = {
        "attempts": 0,
        "triggered": False,
        "gz_requested": False,
        "bridge_requested": False,
    }

    def _trigger() -> None:
        if state["triggered"]:
            return
        state["attempts"] += 1
        system_running = False
        try:
            system_running = bool(panel._system_running())
        except Exception:
            system_running = False
        gz_state = "UNKNOWN"
        try:
            gz_state = str(panel._gazebo_state())
        except Exception:
            gz_state = "UNKNOWN"
        bridge_running = bool(getattr(panel, "_bridge_running", False))
        start_enabled = bool(getattr(panel, "btn_star", None) and panel.btn_star.isEnabled())
        gz_enabled = bool(getattr(panel, "btn_gz_start", None) and panel.btn_gz_start.isEnabled())
        bridge_enabled = bool(getattr(panel, "btn_bridge_start", None) and panel.btn_bridge_start.isEnabled())
        enabled = bool(getattr(panel, "btn_pick_demo2", None) and panel.btn_pick_demo2.isEnabled())
        ready, reason = panel._manual_control_status()
        print(
            f"[{_stamp()}] [AUDIT][DIRECTO2] attempt={state['attempts']} "
            f"start_enabled={str(start_enabled).lower()} "
            f"gz_enabled={str(gz_enabled).lower()} "
            f"bridge_enabled={str(bridge_enabled).lower()} "
            f"system_running={str(system_running).lower()} "
            f"gz_state={gz_state} "
            f"bridge_running={str(bridge_running).lower()} "
            f"btn_enabled={str(enabled).lower()} "
            f"manual_ready={str(bool(ready)).lower()} "
            f"reason={reason or 'ok'} "
            f"ros_worker={str(bool(getattr(panel, '_ros_worker_started', False))).lower()}",
            flush=True,
        )
        if not ready:
            if gz_state == "GAZEBO_OFF" and gz_enabled and not state["gz_requested"]:
                panel._emit_log("[AUDIT][DIRECTO2] offscreen helper clicking btn_gz_start")
                panel.btn_gz_start.click()
                state["gz_requested"] = True
            elif gz_state != "GAZEBO_OFF" and (not bridge_running) and bridge_enabled and not state["bridge_requested"]:
                panel._emit_log("[AUDIT][DIRECTO2] offscreen helper clicking btn_bridge_start")
                panel.btn_bridge_start.click()
                state["bridge_requested"] = True
            if state["attempts"] >= MAX_ATTEMPTS:
                retry_timer.stop()
            return
        try:
            before = bool(getattr(panel, "_script_motion_active", False))
            direct2 = getattr(panel, "_direct2_controller", None)
            if direct2 is not None:
                panel._emit_log("[AUDIT][DIRECTO2] offscreen helper invoking _direct2_controller.request_run")
                ok_run, msg_run = direct2.request_run(source="offscreen_helper")
            else:
                panel._emit_log("[AUDIT][DIRECTO2] offscreen helper invoking _run_pick_demo2 (no direct2 controller)")
                panel._run_pick_demo2()
                ok_run = True
                msg_run = "via_run_pick_demo2"
            app.processEvents()
            ok = bool(getattr(panel, "_script_motion_active", False)) and not before
            if not ok and direct2 is not None:
                ok = bool(getattr(direct2, "_inflight", False))
            message = "started" if ok else f"no_script_motion_active({msg_run})"
            print(
                f"[{_stamp()}] [AUDIT][DIRECTO2] request_run ok={str(bool(ok)).lower()} "
                f"message={message or 'n/a'} "
                f"controller_ok={str(bool(ok_run)).lower()} "
                f"script_active={str(bool(getattr(panel, '_script_motion_active', False))).lower()}",
                flush=True,
            )
            if ok:
                state["triggered"] = True
                retry_timer.stop()
            elif state["attempts"] >= MAX_ATTEMPTS:
                retry_timer.stop()
        except Exception as exc:
            print(f"[{_stamp()}] [AUDIT][DIRECTO2] invoke_error={exc}", flush=True)
            if state["attempts"] >= MAX_ATTEMPTS:
                retry_timer.stop()

    def _trigger_safe() -> None:
        try:
            _trigger()
        except Exception as exc:
            print(f"[{_stamp()}] [AUDIT][DIRECTO2] trigger_error={exc}", flush=True)

    def _close() -> None:
        retry_timer.stop()
        print(f"[{_stamp()}] [SHUTDOWN][HELPER] close_begin", flush=True)
        panel._emit_log("[AUDIT][DIRECTO2] offscreen helper closing panel")
        panel.close()
        QTimer.singleShot(250, app.quit)

    def _handle_signal(_signum, _frame) -> None:
        QTimer.singleShot(0, _close)

    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, _handle_signal)

    retry_timer.timeout.connect(_trigger_safe)
    QTimer.singleShot(CLICK_DELAY_MS, lambda: (retry_timer.start(), _trigger_safe()))
    QTimer.singleShot(EXIT_AFTER_MS, _close)
    rc = app.exec_()
    print(f"[{_stamp()}] [SHUTDOWN][HELPER] app_exec_end rc={rc}", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
