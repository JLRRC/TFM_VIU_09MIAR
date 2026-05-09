#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/wait_runtime_tcp_stable.py
# Contenido: F3-step37 — wait_runtime_tcp_stable extraído del closure run_pick_demo.
"""Wait loop hasta que el TCP runtime se estabilice cerca de un target.

Función pura extraída del closure ``run_pick_demo._wait_runtime_tcp_stable``
(106 LOC). El wrapper en panel_pick_demo se reduce a ~12 LOC delegando aquí.

Estabilidad = 2+ muestras consecutivas con (target_dist <= target_tol_m
y motion_delta <= stable_delta_m).
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional


def wait_runtime_tcp_stable(
    panel: Any,
    *,
    label: str,
    target_tcp_runtime: Any,
    target_tol_m: float,
    timeout_sec: float,
    # Callables-as-args (closure-captured en run_pick_demo):
    tuple3: Callable[[Any], Optional[tuple]],
    live_tcp_base: Callable[[], Optional[tuple]],
    dist: Callable[[Optional[tuple], Optional[tuple]], Optional[float]],
    pose_consistency_metrics: Callable[..., dict],
    json_safe: Callable[[Any], Any],
    fmt_vec: Callable[[Any], str],
    fmt_scalar: Callable[[Any], str],
    # Params snapshot (caller invoca _get_pick_demo_params() y pasa los 3 valores):
    direct_ik_runtime_settle_poll_sec: float,
    direct_ik_runtime_settle_delta_m: float,
    direct_ik_runtime_settle_samples: int,
) -> dict:
    target_tcp_3 = tuple3(target_tcp_runtime)
    poll_sec = max(0.05, float(direct_ik_runtime_settle_poll_sec))
    stable_delta_m = max(0.001, float(direct_ik_runtime_settle_delta_m))
    stable_samples = max(2, int(direct_ik_runtime_settle_samples))
    start_mono = time.monotonic()
    deadline = start_mono + max(0.2, float(timeout_sec))
    prev_tcp = tuple3(live_tcp_base())
    last_tcp = prev_tcp
    last_target_dist = dist(prev_tcp, target_tcp_3)
    last_motion_delta = None
    last_metrics = pose_consistency_metrics(
        tcp_base=prev_tcp,
        target_base=target_tcp_3,
    )
    stable_count = 0
    stable_ok = False

    while True:
        curr_tcp = tuple3(live_tcp_base())
        if curr_tcp is not None:
            last_tcp = curr_tcp
            last_target_dist = dist(curr_tcp, target_tcp_3)
            last_motion_delta = dist(curr_tcp, prev_tcp) if prev_tcp is not None else None
            last_metrics = pose_consistency_metrics(
                tcp_base=curr_tcp,
                target_base=target_tcp_3,
            )
            target_ok = (
                target_tcp_3 is None
                or (
                    last_target_dist is not None
                    and float(last_target_dist) <= float(target_tol_m)
                )
            )
            motion_ok = (
                last_motion_delta is None
                or float(last_motion_delta) <= float(stable_delta_m)
            )
            # sources_ok reflects FK/trace lag vs TF-live, which is a transient
            # timing artefact (~0.5s after the controller settles). Physical
            # convergence (target_ok + motion_ok) is sufficient here; source
            # consistency is enforced by _wait_phase_gate_ready.
            if target_ok and motion_ok:
                stable_count += 1
                if stable_count >= stable_samples:
                    stable_ok = True
                    break
            else:
                stable_count = 0
            prev_tcp = curr_tcp
        if time.monotonic() >= deadline:
            break
        time.sleep(poll_sec)

    elapsed_sec = max(0.0, float(time.monotonic() - start_mono))
    result = {
        "ok": bool(stable_ok),
        "tcp_base": tuple3(last_tcp),
        "target_dist_m": (
            float(last_target_dist) if last_target_dist is not None else None
        ),
        "motion_delta_m": (
            float(last_motion_delta) if last_motion_delta is not None else None
        ),
        "stable_samples": int(stable_count),
        "required_samples": int(stable_samples),
        "elapsed_sec": float(elapsed_sec),
        "poll_sec": float(poll_sec),
        "stable_delta_m": float(stable_delta_m),
        "target_tol_m": float(target_tol_m),
        "pose_consistency": json_safe(last_metrics),
    }
    panel._emit_log(
        "[PICK][DIRECT][RUNTIME_SETTLE] "
        f"label={label} "
        f"target={fmt_vec(target_tcp_3)} "
        f"tcp={fmt_vec(result.get('tcp_base'))} "
        f"target_dist={fmt_scalar(result.get('target_dist_m'))}/{float(target_tol_m):.3f} "
        f"motion_delta={fmt_scalar(result.get('motion_delta_m'))}/{float(stable_delta_m):.3f} "
        f"samples={stable_count}/{stable_samples} "
        f"elapsed={elapsed_sec:.2f}s "
        f"sources_ok={str(bool((last_metrics or {}).get('sources_ok'))).lower()} "
        f"result={'OK' if stable_ok else 'NO'}"
    )
    return result
