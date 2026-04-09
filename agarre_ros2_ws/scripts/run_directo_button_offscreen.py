#!/usr/bin/env python3
"""Launch the real panel offscreen and trigger DIRECTO through the real UI path.

Análogo a run_directo2_button_offscreen.py pero para el botón DIRECTO (btn_pick_demo).
Este helper es observacional/no invasivo: no parchea código de producción.
Si el stack no está arrancado, hace clic en "Start Gazebo" y "Start bridge" y
espera a que el sistema esté listo antes de disparar el botón de Agarre Directo.
"""

from __future__ import annotations

import os
import signal
import sys
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from ur5_qt_panel.panel_v2 import ControlPanelV2


CLICK_DELAY_MS = int(os.environ.get("DIRECTO_CLICK_DELAY_MS", "5000") or "5000")
EXIT_AFTER_MS = int(os.environ.get("DIRECTO_EXIT_AFTER_MS", "300000") or "300000")
RETRY_MS = int(os.environ.get("DIRECTO_RETRY_MS", "2000") or "2000")
MAX_ATTEMPTS = int(os.environ.get("DIRECTO_MAX_ATTEMPTS", "90") or "90")
# Minimum time (s) that _script_motion_active must stay True before we count the
# trigger as "accepted".  Aborts due to cycle_reference_unavailable happen in
# ~2 s; a real DIRECTO run stays active for tens of seconds.
TRIGGER_SETTLE_SEC = float(os.environ.get("DIRECTO_TRIGGER_SETTLE_SEC", "10") or "10")


def main() -> int:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("PANEL_FORCE_OFFSCREEN", "1")
    os.environ.setdefault("PANEL_START_STACK", "0")

    app = QApplication(sys.argv)
    panel = ControlPanelV2()
    panel.show()
    state = {
        "attempts": 0,
        "triggered": False,
        "confirmed": False,    # True once the run has been running > TRIGGER_SETTLE_SEC
        "trigger_ts": 0.0,     # monotonic time when the last ok=true was recorded
        "gz_requested": False,
        "gz_stop_requested": False,  # True after clicking btn_gz_stop to clear DEGRADED
        "gz_stop_attempt": 0,        # counts how many times we tried to stop Gazebo
        "bridge_requested": False,
    }

    def _stamp() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%S")

    def _check_trigger_confirmed() -> None:
        """Scheduled TRIGGER_SETTLE_SEC after ok=true.  If script_motion_active is
        still True → run is in progress, mark confirmed.  Otherwise DIRECTO aborted
        quickly → allow a new attempt."""
        if state["confirmed"]:
            return
        still_active = bool(getattr(panel, "_script_motion_active", False))
        demo_done = bool(getattr(panel, "_pick_demo_executed", False))
        print(
            f"[{_stamp()}] [AUDIT][DIRECTO] trigger_settle_check "
            f"still_active={str(still_active).lower()} demo_done={str(demo_done).lower()}",
            flush=True,
        )
        if still_active or demo_done:
            state["confirmed"] = True
            print(
                f"[{_stamp()}] [AUDIT][DIRECTO] trigger_confirmed "
                f"demo_done={str(demo_done).lower()}",
                flush=True,
            )
        else:
            # Quick abort — allow next attempt
            print(
                f"[{_stamp()}] [AUDIT][DIRECTO] trigger_aborted quickly; will retry",
                flush=True,
            )
            state["triggered"] = False
            if state["attempts"] < MAX_ATTEMPTS:
                QTimer.singleShot(RETRY_MS, _trigger)

    def _trigger() -> None:
        if state["triggered"]:
            return
        state["attempts"] += 1
        gz_state = "UNKNOWN"
        try:
            gz_state = str(panel._gazebo_state())
        except Exception:
            gz_state = "UNKNOWN"
        bridge_running = bool(getattr(panel, "_bridge_running", False))
        gz_enabled = bool(getattr(panel, "btn_gz_start", None) and panel.btn_gz_start.isEnabled())
        bridge_enabled = bool(getattr(panel, "btn_bridge_start", None) and panel.btn_bridge_start.isEnabled())
        btn_enabled = bool(getattr(panel, "btn_pick_demo", None) and panel.btn_pick_demo.isEnabled())
        ready, reason = panel._manual_control_status()
        selected = str(getattr(panel, "_selected_object", None) or "none")
        user_selected = str(getattr(panel, "_selection_last_user_name", None) or "none")
        print(
            f"[{_stamp()}] [AUDIT][DIRECTO] attempt={state['attempts']} "
            f"ready={str(bool(ready)).lower()} "
            f"reason={reason or 'ok'} "
            f"gz_state={gz_state} "
            f"bridge_running={str(bridge_running).lower()} "
            f"gz_enabled={str(gz_enabled).lower()} "
            f"bridge_enabled={str(bridge_enabled).lower()} "
            f"btn_enabled={str(btn_enabled).lower()} "
            f"selected={selected} "
            f"user_selected={user_selected}",
            flush=True,
        )
        gz_stop_enabled = bool(getattr(panel, "btn_gz_stop", None) and panel.btn_gz_stop.isEnabled())
        if not ready:
            if gz_state in ("GAZEBO_OFF", "GAZEBO_DEGRADED") and gz_enabled and not state["gz_requested"]:
                panel._emit_log(f"[AUDIT][DIRECTO] offscreen helper clicking btn_gz_start (gz_state={gz_state})")
                panel.btn_gz_start.click()
                state["gz_requested"] = True
            elif gz_state == "GAZEBO_DEGRADED" and not gz_enabled and (
                not state["gz_stop_requested"] or state["gz_stop_attempt"] > 0
            ):
                # DEGRADED + start disabled: Gazebo process still alive (or lingering).
                # Stop Gazebo first so the state transitions to GAZEBO_OFF and the start
                # button becomes re-enabled.  If stop button is also disabled, call the
                # underlying method directly to force the transition.
                # Allow re-trying stop every 5 retries in case first stop was a no-op.
                _do_stop = (not state["gz_stop_requested"]) or (
                    state["gz_stop_attempt"] > 0
                    and (state["attempts"] - state.get("_gz_stop_last_attempt", 0)) >= 5
                )
                if _do_stop:
                    state["_gz_stop_last_attempt"] = state["attempts"]
                    state["gz_stop_attempt"] += 1
                    print(
                        f"[{_stamp()}] [AUDIT][DIRECTO] gz_stop attempt={state['gz_stop_attempt']} "
                        f"gz_state={gz_state} gz_stop_enabled={str(gz_stop_enabled).lower()}",
                        flush=True,
                    )
                    if gz_stop_enabled:
                        panel.btn_gz_stop.click()
                    else:
                        try:
                            panel._stop_gazebo()
                        except Exception as _exc:
                            print(f"[{_stamp()}] [AUDIT][DIRECTO] _stop_gazebo error: {_exc}", flush=True)
                    state["gz_stop_requested"] = True
                    state["gz_requested"] = False  # allow re-requesting start after stop
            elif gz_state not in ("GAZEBO_OFF", "GAZEBO_STARTING", "GAZEBO_DEGRADED") and (not bridge_running) and bridge_enabled and not state["bridge_requested"]:
                panel._emit_log("[AUDIT][DIRECTO] offscreen helper clicking btn_bridge_start")
                panel.btn_bridge_start.click()
                state["bridge_requested"] = True
            if state["attempts"] < MAX_ATTEMPTS:
                QTimer.singleShot(RETRY_MS, _trigger)
            return
        try:
            # Simulate user selection of pick_demo (required by run_pick_demo guard).
            # A real user would click the object in the camera image; we set the
            # attributes directly since there is no camera in offscreen mode.
            panel._selected_object = "pick_demo"
            panel._selection_last_user_name = "pick_demo"
            panel._selection_last_user_ts = time.time()
            panel._emit_log("[AUDIT][DIRECTO] offscreen helper set selection: pick_demo")
            if btn_enabled:
                panel._emit_log("[AUDIT][DIRECTO] offscreen helper clicking btn_pick_demo")
            else:
                panel._emit_log("[AUDIT][DIRECTO] offscreen helper invoking _run_pick_demo (button disabled)")
            before = bool(getattr(panel, "_script_motion_active", False))
            if btn_enabled:
                panel.btn_pick_demo.click()
            else:
                panel._run_pick_demo()
            ok = bool(getattr(panel, "_script_motion_active", False)) and not before
            message = "started" if ok else "no_script_motion_active"
            print(
                f"[{_stamp()}] [AUDIT][DIRECTO] request_run ok={str(bool(ok)).lower()} "
                f"message={message} script_active={str(bool(getattr(panel, '_script_motion_active', False))).lower()}",
                flush=True,
            )
            if ok:
                state["triggered"] = True
                state["trigger_ts"] = time.monotonic()
                # Schedule a confirmation check: if DIRECTO aborts within TRIGGER_SETTLE_SEC
                # we will re-arm and try again.
                QTimer.singleShot(int(TRIGGER_SETTLE_SEC * 1000), _check_trigger_confirmed)
            elif state["attempts"] < MAX_ATTEMPTS:
                QTimer.singleShot(RETRY_MS, _trigger)
        except Exception as exc:
            print(f"[{_stamp()}] [AUDIT][DIRECTO] invoke_error={exc}", flush=True)
            if state["attempts"] < MAX_ATTEMPTS:
                QTimer.singleShot(RETRY_MS, _trigger)

    def _close() -> None:
        panel._emit_log("[AUDIT][DIRECTO] offscreen helper closing panel")
        panel.close()

    def _handle_signal(_signum, _frame) -> None:
        QTimer.singleShot(0, _close)

    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, _handle_signal)

    QTimer.singleShot(CLICK_DELAY_MS, _trigger)
    QTimer.singleShot(EXIT_AFTER_MS, _close)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
