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
from typing import Any, Dict, List, Optional, Tuple

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
FORCE_STEP_MODE = str(os.environ.get("DIRECTO_FORCE_STEP_MODE", "0") or "0").strip().lower() in (
    "1",
    "true",
    "yes",
    "on",
)
STEP_POLL_MS = int(os.environ.get("DIRECTO_STEP_POLL_MS", "120") or "120")
STEP_MAX_CONTINUES = int(os.environ.get("DIRECTO_STEP_MAX_CONTINUES", "1") or "1")


def _fmt_xyz(vec: Optional[Tuple[float, float, float]]) -> str:
    if vec is None:
        return "none"
    try:
        return f"({float(vec[0]):.3f},{float(vec[1]):.3f},{float(vec[2]):.3f})"
    except Exception:
        return "none"


def _snapshot_step_rows(panel: ControlPanelV2) -> List[Dict[str, Any]]:
    rows = []
    for row in list(getattr(panel, "_step_history_rows", []) or []):
        phase = str(row.get("phase") or "").strip().upper()
        target = row.get("target")
        actual = row.get("actual")
        reached = row.get("reached")
        rows.append(
            {
                "phase": phase,
                "target": target,
                "actual": actual,
                "reached": reached,
            }
        )
    return rows


def _find_row(rows: List[Dict[str, Any]], phase: str) -> Optional[Dict[str, Any]]:
    phase_u = str(phase or "").strip().upper()
    for row in rows:
        if str(row.get("phase") or "").strip().upper() == phase_u:
            return row
    return None


def _xyz_delta(a: Optional[Tuple[float, float, float]], b: Optional[Tuple[float, float, float]]) -> str:
    if a is None or b is None:
        return "none"
    try:
        dx = float(a[0]) - float(b[0])
        dy = float(a[1]) - float(b[1])
        dz = float(a[2]) - float(b[2])
        return f"({dx:.6f},{dy:.6f},{dz:.6f})"
    except Exception:
        return "none"


def main() -> int:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("PANEL_FORCE_OFFSCREEN", "1")
    os.environ.setdefault("PANEL_START_STACK", "0")

    def _stamp() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%S")

    app = QApplication(sys.argv)
    panel = ControlPanelV2()
    panel.show()
    print(f"[{_stamp()}] [SHUTDOWN][HELPER] app_started", flush=True)
    retry_timer = QTimer(panel)
    retry_timer.setInterval(RETRY_MS)
    retry_timer.setSingleShot(False)
    step_timer = QTimer(panel)
    step_timer.setInterval(STEP_POLL_MS)
    step_timer.setSingleShot(False)
    state = {
        "attempts": 0,
        "triggered": False,
        "confirmed": False,    # True once the run has been running > TRIGGER_SETTLE_SEC
        "trigger_ts": 0.0,     # monotonic time when the last ok=true was recorded
        "gz_requested": False,
        "gz_stop_requested": False,  # True after clicking btn_gz_stop to clear DEGRADED
        "gz_stop_attempt": 0,        # counts how many times we tried to stop Gazebo
        "bridge_requested": False,
        "release_requested": False,
        "selection_staged": False,
        "script_motion_guard_logged": False,
        "step_mode_forced": False,
        "step_inicio_captured": False,
        "step_post_captured": False,
        "step_close_scheduled": False,
        "step_continues_sent": 0,
        "step_last_continue_ts": 0.0,
        "step_inicio_actual": None,
        "step_inicio_target": None,
    }

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
            retry_timer.stop()
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

    def _trigger() -> None:
        if state["triggered"]:
            return
        # Guard against re-entrant calls: if the panel already has script_motion_active
        # from a concurrent trigger, avoid double-fire but do not latch the helper
        # as triggered unless we have actually observed request_run ok=true.
        if bool(getattr(panel, "_script_motion_active", False)):
            if not state["script_motion_guard_logged"]:
                print(
                    f"[{_stamp()}] [AUDIT][DIRECTO] script_motion_active_guard "
                    "helper_waiting_for_script_to_clear",
                    flush=True,
                )
                state["script_motion_guard_logged"] = True
            return
        state["script_motion_guard_logged"] = False
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
        try:
            panel._selected_object = "pick_demo"
            panel._selection_last_user_name = "pick_demo"
            panel._selection_last_user_ts = time.time()
            if not state["selection_staged"]:
                panel._emit_log("[AUDIT][DIRECTO] offscreen helper staged selection: pick_demo")
                state["selection_staged"] = True
        except Exception:
            pass
        ready, reason = panel._manual_control_status()
        remote_ready = bool(ready)
        remote_reason = reason or "ok"
        remote_waitable = False
        remote_ready_status = getattr(panel, "_pick_demo_remote_ready_status", None)
        if callable(remote_ready_status):
            try:
                remote_ready, remote_reason, remote_waitable = remote_ready_status()
            except Exception as exc:
                remote_ready = False
                remote_reason = f"remote_ready_error:{exc}"
                remote_waitable = False

        if FORCE_STEP_MODE and not state["step_mode_forced"]:
            try:
                panel._set_step_mode("STEP_BY_STEP", emit_log=True)
                panel._step_window_set_waiting()
                pending_mode = str(getattr(panel, "_step_mode", "") or "")
                step_window = getattr(panel, "_step_window", None)
                window_visible = bool(step_window is not None and step_window.isVisible())
                panel._emit_log(
                    "[AUDIT][STEP] forced_mode "
                    f"mode={pending_mode or 'unknown'} "
                    f"window_visible={str(window_visible).lower()}"
                )
                state["step_mode_forced"] = True
            except Exception as exc:
                print(f"[{_stamp()}] [AUDIT][STEP] force_mode_error={exc}", flush=True)
        remote_reason_norm = str(remote_reason or "").strip().lower()
        selected = str(getattr(panel, "_selected_object", None) or "none")
        user_selected = str(getattr(panel, "_selection_last_user_name", None) or "none")
        print(
            f"[{_stamp()}] [AUDIT][DIRECTO] attempt={state['attempts']} "
            f"ready={str(bool(ready)).lower()} "
            f"reason={reason or 'ok'} "
            f"remote_ready={str(bool(remote_ready)).lower()} "
            f"remote_reason={remote_reason or 'ok'} "
            f"remote_waitable={str(bool(remote_waitable)).lower()} "
            f"gz_state={gz_state} "
            f"bridge_running={str(bridge_running).lower()} "
            f"gz_enabled={str(gz_enabled).lower()} "
            f"bridge_enabled={str(bridge_enabled).lower()} "
            f"btn_enabled={str(btn_enabled).lower()} "
            f"selected={selected} "
            f"user_selected={user_selected}",
            flush=True,
        )
        if (not ready) and gz_state == "GAZEBO_READY" and str(reason or "").startswith("gazebo_not_ready"):
            print(
                f"[{_stamp()}] [AUDIT][DIRECTO] gate_mismatch "
                f"gz_state={gz_state} reason={reason} btn_enabled={str(btn_enabled).lower()}",
                flush=True,
            )
        gz_stop_enabled = bool(getattr(panel, "btn_gz_stop", None) and panel.btn_gz_stop.isEnabled())
        # If the button is enabled but remote_ready is False due to a waitable condition
        # (e.g. settle timeout, objects not stabilised) that has persisted for many
        # attempts, override remote_ready so we can still fire.  The real UI button is
        # enabled, meaning the panel's hard gate already allows clicking.
        _REMOTE_WAITABLE_OVERRIDE_ATTEMPTS = 6
        if (
            ready
            and btn_enabled
            and (not remote_ready)
            and remote_waitable
            and state["attempts"] >= _REMOTE_WAITABLE_OVERRIDE_ATTEMPTS
        ):
            print(
                f"[{_stamp()}] [AUDIT][DIRECTO] remote_waitable_override "
                f"attempts={state['attempts']} remote_reason={remote_reason} btn_enabled=true",
                flush=True,
            )
            remote_ready = True
        if (not ready) or (not remote_ready) or (not btn_enabled):
            if (
                ready
                and gz_state == "GAZEBO_READY"
                and bridge_running
                and remote_reason_norm == "release de objetos pendiente"
                and not bool(getattr(panel, "_detach_inflight", False))
                and not state["release_requested"]
            ):
                release_btn = getattr(panel, "btn_release_objects", None)
                if release_btn is not None and release_btn.isEnabled():
                    panel._emit_log("[AUDIT][DIRECTO] offscreen helper clicking btn_release_objects")
                    release_btn.click()
                    state["release_requested"] = True
                else:
                    release_fn = getattr(panel, "_release_objects", None)
                    if callable(release_fn):
                        panel._emit_log("[AUDIT][DIRECTO] offscreen helper invoking _release_objects")
                        release_fn()
                        state["release_requested"] = True
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
            if state["attempts"] >= MAX_ATTEMPTS:
                retry_timer.stop()
            return
        try:
            if btn_enabled:
                panel._emit_log("[AUDIT][DIRECTO] offscreen helper clicking btn_pick_demo")
            before = bool(getattr(panel, "_script_motion_active", False))
            panel.btn_pick_demo.click()
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
                retry_timer.stop()
                # Schedule a confirmation check: if DIRECTO aborts within TRIGGER_SETTLE_SEC
                # we will re-arm and try again.
                QTimer.singleShot(int(TRIGGER_SETTLE_SEC * 1000), _check_trigger_confirmed)
            elif state["attempts"] >= MAX_ATTEMPTS:
                retry_timer.stop()
        except Exception as exc:
            print(f"[{_stamp()}] [AUDIT][DIRECTO] invoke_error={exc}", flush=True)
            if state["attempts"] >= MAX_ATTEMPTS:
                retry_timer.stop()

    def _trigger_safe() -> None:
        try:
            _trigger()
        except Exception as exc:
            print(f"[{_stamp()}] [AUDIT][DIRECTO] trigger_error={exc}", flush=True)

    def _step_drive() -> None:
        if not FORCE_STEP_MODE:
            return
        try:
            pending = str(getattr(panel, "_step_pending_phase", "") or "").strip()
            mode = str(getattr(panel, "_step_mode", "") or "").strip()
            step_window = getattr(panel, "_step_window", None)
            window_visible = bool(step_window is not None and step_window.isVisible())

            # Captura obligatoria: INICIO visible antes de pulsar Siguiente.
            if pending.upper().endswith("INICIO") and not state["step_inicio_captured"]:
                rows = _snapshot_step_rows(panel)
                inicio = _find_row(rows, "INICIO")
                inicio_actual = inicio.get("actual") if inicio else None
                inicio_target = inicio.get("target") if inicio else None
                state["step_inicio_actual"] = inicio_actual
                state["step_inicio_target"] = inicio_target
                panel._emit_log(
                    "[AUDIT][STEP] inicio_before_continue "
                    f"mode={mode or 'unknown'} "
                    f"window_visible={str(window_visible).lower()} "
                    f"pending={pending or 'none'} "
                    f"rows={len(rows)} "
                    f"inicio_act={_fmt_xyz(inicio_actual)} "
                    f"inicio_obj={_fmt_xyz(inicio_target)}"
                )
                state["step_inicio_captured"] = True

            # Avance explícito con botón Siguiente.
            if (
                state["step_inicio_captured"]
                and state["step_continues_sent"] < STEP_MAX_CONTINUES
                and pending.upper().endswith("INICIO")
            ):
                now = time.monotonic()
                if now - float(state["step_last_continue_ts"] or 0.0) > 0.4:
                    state["step_last_continue_ts"] = now
                    state["step_continues_sent"] += 1
                    panel._emit_log(
                        "[AUDIT][STEP] continue_click "
                        f"index={state['step_continues_sent']} pending={pending or 'none'}"
                    )
                    panel._on_step_continue_clicked()

            # Captura post-avance: verificar no mutación de INICIO + fila siguiente.
            if (
                state["step_continues_sent"] >= 1
                and (pending != "")
                and (not pending.upper().endswith("INICIO"))
                and (not state["step_post_captured"])
            ):
                rows_after = _snapshot_step_rows(panel)
                inicio_after = _find_row(rows_after, "INICIO")
                next_row = None
                for row in rows_after:
                    if str(row.get("phase") or "").strip().upper() != "INICIO":
                        next_row = row
                        break
                inicio_after_actual = inicio_after.get("actual") if inicio_after else None
                inicio_after_target = inicio_after.get("target") if inicio_after else None
                next_phase = str(next_row.get("phase") or "--") if next_row else "--"
                next_actual = next_row.get("actual") if next_row else None
                next_target = next_row.get("target") if next_row else None

                pre_act = state.get("step_inicio_actual")
                pre_obj = state.get("step_inicio_target")
                same_act = bool(pre_act == inicio_after_actual)
                same_obj = bool(pre_obj == inicio_after_target)
                same_obj_next = bool(pre_obj == next_target)
                panel._emit_log(
                    "[AUDIT][STEP] post_continue "
                    f"pending={pending} "
                    f"rows={len(rows_after)} "
                    f"inicio_act={_fmt_xyz(inicio_after_actual)} "
                    f"inicio_obj={_fmt_xyz(inicio_after_target)} "
                    f"next_phase={next_phase} "
                    f"next_act={_fmt_xyz(next_actual)} "
                    f"next_obj={_fmt_xyz(next_target)}"
                )
                panel._emit_log(
                    "[AUDIT][STEP] inicio_mutation_check "
                    f"act_unchanged={str(same_act).lower()} "
                    f"obj_unchanged={str(same_obj).lower()} "
                    f"delta_act={_xyz_delta(inicio_after_actual, pre_act)} "
                    f"delta_obj={_xyz_delta(inicio_after_target, pre_obj)}"
                )
                panel._emit_log(
                    "[AUDIT][STEP] inicio_obj_vs_first_real_target "
                    f"match={str(same_obj_next).lower()} "
                    f"delta={_xyz_delta(pre_obj, next_target)}"
                )
                panel._emit_log(
                    "[AUDIT][STEP] next_phase_snapshot_check "
                    f"next_phase={next_phase} "
                    f"next_has_actual={str(next_actual is not None).lower()} "
                    f"inicio_row_preserved={str(same_act and same_obj).lower()}"
                )
                state["step_post_captured"] = True
                if not state["step_close_scheduled"]:
                    state["step_close_scheduled"] = True
                    QTimer.singleShot(1800, _close)
        except Exception as exc:
            print(f"[{_stamp()}] [AUDIT][STEP] step_drive_error={exc}", flush=True)

    def _start_polling() -> None:
        _trigger_safe()
        if not state["confirmed"] and state["attempts"] < MAX_ATTEMPTS and not retry_timer.isActive():
            retry_timer.start()

    def _close() -> None:
        retry_timer.stop()
        step_timer.stop()
        print(f"[{_stamp()}] [SHUTDOWN][HELPER] close_begin", flush=True)
        panel._emit_log("[AUDIT][DIRECTO] offscreen helper closing panel")
        panel.close()
        QTimer.singleShot(250, app.quit)

    def _about_to_quit() -> None:
        print(f"[{_stamp()}] [SHUTDOWN][HELPER] about_to_quit", flush=True)

    def _handle_signal(_signum, _frame) -> None:
        QTimer.singleShot(0, _close)

    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, _handle_signal)

    retry_timer.timeout.connect(_trigger_safe)
    step_timer.timeout.connect(_step_drive)
    app.aboutToQuit.connect(_about_to_quit)
    QTimer.singleShot(CLICK_DELAY_MS, _start_polling)
    if FORCE_STEP_MODE:
        step_timer.start()
    QTimer.singleShot(EXIT_AFTER_MS, _close)
    rc = app.exec_()
    print(f"[{_stamp()}] [SHUTDOWN][HELPER] app_exec_end rc={rc}", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
