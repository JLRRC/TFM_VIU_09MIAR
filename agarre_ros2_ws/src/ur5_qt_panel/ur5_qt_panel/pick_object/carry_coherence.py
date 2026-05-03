#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/carry_coherence.py
# Contenido: F3-step5bis-a — extracción de _assert_carry_coherence_after_lift (147 LOC).
"""Carry coherence gate extraído del closure run_pick_object.worker.

``assert_carry_coherence_after_lift`` valida tras LIFT que el objeto
sigue "agarrado" — distancia TCP↔objeto < ``max_dist_m`` durante al
menos ``min_consecutive`` muestras consecutivas. Si falla, lanza
RuntimeError ``carry_coherence_failed_<label>``.

Antes de F3-step5bis-a vivía como ``def`` anidada de 147 LOC dentro
del closure ``run_pick_object.worker``, capturando 6 deps reales
(panel + base/world/ee frames + obj_name + read_tcp_in_frame helper).
8 callsites en el closure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional


@dataclass
class CarryCoherenceContext:
    """Captura de dependencias closure de assert_carry_coherence_after_lift."""

    panel: Any
    read_tcp_in_frame: Callable[..., Any]
    base_frame: str
    world_frame: str
    measured_ee_frame: str
    obj_name: str
    get_pick_object_params: Callable[[], Any]
    log_moveit_panel_trace: Callable[..., Any]


def assert_carry_coherence_after_lift(
    ctx: CarryCoherenceContext,
    require_state_override: Optional[bool] = None,
    timeout_override: Optional[float] = None,
    max_dist_override: Optional[float] = None,
    min_consecutive_override: Optional[int] = None,
    gate_label: str = "carry_gate",
) -> None:
    """Valida coherencia TCP↔objeto tras LIFT, lanza RuntimeError si falla."""
    enabled = ctx.get_pick_object_params().carry_gate_enable
    if not enabled:
        return
    try:
        timeout_sec = ctx.get_pick_object_params().carry_gate_timeout_sec
    except Exception:
        timeout_sec = 1.4
    try:
        sample_dt = ctx.get_pick_object_params().carry_gate_sample_dt_sec
    except Exception:
        sample_dt = 0.08
    try:
        max_dist_m = ctx.get_pick_object_params().carry_gate_max_dist_m
    except Exception:
        max_dist_m = 0.18
    try:
        min_consecutive = ctx.get_pick_object_params().carry_gate_min_consecutive
    except Exception:
        min_consecutive = 2
    require_state = ctx.get_pick_object_params().carry_gate_require_state
    if require_state_override is not None:
        require_state = bool(require_state_override)
    if timeout_override is not None:
        timeout_sec = float(timeout_override)
    if max_dist_override is not None:
        max_dist_m = float(max_dist_override)
    if min_consecutive_override is not None:
        min_consecutive = int(min_consecutive_override)

    timeout_sec = max(0.2, timeout_sec)
    sample_dt = max(0.03, sample_dt)
    max_dist_m = max(0.02, max_dist_m)
    min_consecutive = max(1, min_consecutive)

    deadline = time.time() + timeout_sec
    consecutive_ok = 0
    best_dist = float("inf")
    last_dist = float("inf")
    last_state = "none"
    wait_log_ts = 0.0

    ctx.panel._emit_log(
        f"[PICK_OBJ][CARRY_GATE] start phase={gate_label} timeout={timeout_sec:.2f}s "
        f"max_dist={max_dist_m:.3f} min_ok={min_consecutive} "
        f"require_state={str(require_state).lower()}"
    )

    while time.time() < deadline:
        tcp_base, _tcp_tf = ctx.read_tcp_in_frame(
            ctx.base_frame,
            ctx.measured_ee_frame,
            timeout_sec=min(0.2, sample_dt * 1.5),
        )
        obj_state = get_object_state(ctx.obj_name)
        obj_world = None
        obj_source = "none"
        # During carry validation the logical object state can keep the
        # last table pose for a short time after attach. Prefer live
        # pose feeds and leave state_tracking as a last-resort fallback.
        if getattr(ctx.panel, "_ros_worker_started", False) and getattr(ctx.panel, "ros_worker", None) is not None:
            try:
                pose_map, _pose_ts = ctx.panel.ros_worker.pose_snapshot()
                live_pose = (pose_map or {}).get(ctx.obj_name)
                if live_pose is not None and len(live_pose) >= 3:
                    obj_world = (
                        float(live_pose[0]),
                        float(live_pose[1]),
                        float(live_pose[2]),
                    )
                    obj_source = "pose_snapshot"
            except Exception:
                pass
        if obj_world is None:
            obj_world = (get_object_positions() or {}).get(ctx.obj_name)
            if obj_world is not None:
                obj_source = "store"
        if obj_world is None:
            if obj_state and getattr(obj_state, "position", None):
                obj_world = tuple(obj_state.position)
                obj_source = "state_tracking"
        if tcp_base is None or obj_world is None:
            consecutive_ok = 0
            time.sleep(sample_dt)
            continue

        obj_base, _ = transform_point_to_frame(
            (float(obj_world[0]), float(obj_world[1]), float(obj_world[2])),
            ctx.base_frame,
            source_frame=ctx.world_frame,
            timeout_sec=min(0.2, sample_dt * 1.5),
        )
        if obj_base is None:
            consecutive_ok = 0
            time.sleep(sample_dt)
            continue

        dx = float(obj_base[0]) - float(tcp_base[0])
        dy = float(obj_base[1]) - float(tcp_base[1])
        dz = float(obj_base[2]) - float(tcp_base[2])
        dist = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        best_dist = min(best_dist, dist)
        last_dist = dist

        state_ok = True
        if obj_state is not None:
            last_state = str(getattr(obj_state, "logical_state", "none"))
            if require_state:
                state_ok = obj_state.logical_state in (
                    ObjectLogicalState.CARRIED,
                    ObjectLogicalState.GRASPED,
                )

        if dist <= max_dist_m and state_ok:
            consecutive_ok += 1
        else:
            consecutive_ok = 0

        now = time.time()
        if (now - wait_log_ts) >= 0.45:
            ctx.panel._emit_log(
                f"[PICK_OBJ][CARRY_GATE] phase={gate_label} dist={dist:.3f} best={best_dist:.3f} "
                f"max={max_dist_m:.3f} state={last_state} src={obj_source} ok_count={consecutive_ok}/{min_consecutive}"
            )
            wait_log_ts = now

        if consecutive_ok >= min_consecutive:
            ctx.panel._emit_log(
                f"[PICK_OBJ][CARRY_GATE] ok phase={gate_label} dist={dist:.3f} best={best_dist:.3f} state={last_state}"
            )
            return
        time.sleep(sample_dt)

    ctx.panel._emit_log(
        f"[PICK_OBJ][ABORT] carry_coherence_failed phase={gate_label} "
        f"dist={last_dist:.3f} best={best_dist:.3f} max={max_dist_m:.3f} "
        f"state={last_state}"
    )
    raise RuntimeError(
        f"carry_coherence_failed_{gate_label} dist={last_dist:.3f} best={best_dist:.3f} max={max_dist_m:.3f}"
    )
