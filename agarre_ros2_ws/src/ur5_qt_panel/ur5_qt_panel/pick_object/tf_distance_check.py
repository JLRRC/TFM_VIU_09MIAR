#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/tf_distance_check.py
# Contenido: F3-step4b — extracción de _tf_distance_check (120 LOC).
"""TF distance check extraído del closure run_pick_object.worker.

``tf_distance_check`` valida que el TCP esté dentro de ``tol_m`` del
target ``pose_data`` en el frame ``ee_frame``, consultando TF live.
Devuelve dict con diagnóstico (success, dist_m, ee_pose, age, etc.).

Antes de F3-step4b vivía como ``def`` anidada de 120 LOC dentro del
closure ``run_pick_object.worker``, capturando 3 deps reales
(panel + BASE_FRAME + WORLD_FRAME). 3 callsites en el closure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class TfDistanceCheckContext:
    """Captura de dependencias closure de tf_distance_check."""

    panel: Any
    base_frame: str = "base_link"
    world_frame: str = "world"


def tf_distance_check(
    ctx: TfDistanceCheckContext,
    *,
    label: str,
    pose_data: dict,
    tol_m: float,
    ee_frame: str,
) -> dict:
    """Comprueba distancia TCP→target en TF y devuelve dict diagnóstico."""
    frame_id = str(pose_data.get("frame", ctx.base_frame or "base_link"))
    position = pose_data.get("position", (0.0, 0.0, 0.0))
    target = (float(position[0]), float(position[1]), float(position[2]))
    timeout_pos_reach = max(
        2.0, float(getattr(ctx.panel, "_pick_tf_reach_timeout_sec", 8.0) or 8.0)
    )
    timeout_tf_fresh = 2.0
    max_tf_age_sec = 0.80
    deadline = time.time() + timeout_pos_reach
    fresh_deadline = time.time() + timeout_tf_fresh
    wait_log_ts = 0.0
    last_diag: Optional[dict] = None
    best_dist = float("inf")
    best_dist_ts = 0.0
    saw_fresh = False
    while time.time() <= deadline:
        tcp, tf_msg = _read_tcp_in_frame(frame_id, ee_frame)
        if tcp is None:
            time.sleep(0.08)
            continue
        dx = target[0] - tcp[0]
        dy = target[1] - tcp[1]
        dz = target[2] - tcp[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        tf_stamp = "n/a"
        tf_stamp_ns = 0
        try:
            stamp = tf_msg.header.stamp  # type: ignore[attr-defined]
            tf_stamp = f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"
            tf_stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except Exception:
            tf_stamp = "n/a"
        ros_now_ns = 0
        ros_now_txt = "n/a"
        ros_now_ns = _ros_clock_now_ns()
        if ros_now_ns > 0:
            ros_now_txt = f"{ros_now_ns // 1_000_000_000}.{ros_now_ns % 1_000_000_000:09d}"
        tf_age_sec = None
        if ros_now_ns > 0 and tf_stamp_ns > 0:
            tf_age_sec = (ros_now_ns - tf_stamp_ns) / 1_000_000_000.0
        tf_age_txt = "n/a" if tf_age_sec is None else f"{tf_age_sec:.3f}"
        tf_fresh = tf_age_sec is None or (-0.10 <= float(tf_age_sec) <= max_tf_age_sec)
        if tf_fresh:
            saw_fresh = True
        if dist < best_dist:
            best_dist = dist
            best_dist_ts = time.time()
        # Distancia manda; freshness solo diagnostica para evitar falsos abortos.
        tf_ok = bool(dist <= tol_m)
        last_diag = {
            "dist": dist,
            "best_dist": best_dist,
            "best_dist_age_sec": max(0.0, time.time() - best_dist_ts) if best_dist_ts > 0 else None,
            "target": target,
            "tcp": tcp,
            "tol": float(tol_m),
            "frame": frame_id,
            "ee_frame": ee_frame,
            "tf_stamp": tf_stamp,
            "tf_stamp_ns": tf_stamp_ns,
            "ros_now_ns": ros_now_ns,
            "tf_age_sec": tf_age_sec,
            "tf_fresh": tf_fresh,
            "saw_fresh": saw_fresh,
            "ok": tf_ok,
        }
        now_txt = ros_now_txt
        ctx.panel._emit_log(
            f"[TF_CHECK] label={label} frame={frame_id} ee_link={ee_frame} "
            f"dist={dist:.3f} best={best_dist:.3f} tol={tol_m:.3f} "
            f"tf_age={tf_age_txt}s ros_now={now_txt} tf_stamp={tf_stamp} "
            f"fresh={str(tf_fresh).lower()}"
        )
        if tf_ok:
            ctx.panel._emit_log(
                f"[PICK_OBJ][TF][CHECK] {label} target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) "
                f"tcp=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) dist={dist:.3f} "
                f"tol={tol_m:.3f} frame={frame_id} ee={ee_frame} "
                f"tf_stamp={tf_stamp} ros_now={ros_now_txt} tf_age={tf_age_txt}s "
                f"fresh={str(tf_fresh).lower()} ok=true"
            )
            return last_diag
        # If TF is stale, give it a short dedicated grace window before
        # deciding based on distance timeout.
        if (not tf_fresh) and (time.time() <= fresh_deadline):
            time.sleep(0.08)
            continue
        now = time.time()
        if (now - wait_log_ts) >= 0.6:
            ctx.panel._emit_log(
                f"[PICK_OBJ][TF][WAIT] {label} dist={dist:.3f} tol={tol_m:.3f} "
                f"best={best_dist:.3f} tf_age={tf_age_txt}s fresh={str(tf_fresh).lower()} "
                f"frame={frame_id} ee={ee_frame}"
            )
            wait_log_ts = now
        time.sleep(0.10)
    if last_diag is None:
        raise RuntimeError(f"TF sin datos para {frame_id}->{ee_frame}")
    last_tcp = last_diag["tcp"]
    tf_age_sec = last_diag.get("tf_age_sec")
    tf_age_txt = "n/a" if tf_age_sec is None else f"{float(tf_age_sec):.3f}"
    fail_reason = "pos_not_reached"
    last_diag["fail_reason"] = fail_reason
    ctx.panel._emit_log(
        f"[PICK_OBJ][TF][CHECK] {label} target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) "
        f"tcp=({last_tcp[0]:.3f},{last_tcp[1]:.3f},{last_tcp[2]:.3f}) dist={float(last_diag['dist']):.3f} "
        f"best={float(last_diag.get('best_dist', float(last_diag['dist']))):.3f} "
        f"tol={tol_m:.3f} frame={frame_id} ee={ee_frame} "
        f"tf_stamp={last_diag.get('tf_stamp', 'n/a')} tf_age={tf_age_txt}s "
        f"fresh={str(bool(last_diag.get('tf_fresh', False))).lower()} "
        f"reason={fail_reason} ok=false"
    )
    return last_diag
