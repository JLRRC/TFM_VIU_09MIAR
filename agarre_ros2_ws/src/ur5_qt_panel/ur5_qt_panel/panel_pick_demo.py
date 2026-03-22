#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Pick demo sequence helper for the panel."""
from __future__ import annotations

import os
import json
import math
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

try:
    from std_msgs.msg import Empty
except Exception:  # pragma: no cover - ROS not available in unit contexts
    Empty = None

from .panel_robot_presets import (
    JOINT_TABLE_POSE_RAD,
    JOINT_PICK_IMAGE_POSE_RAD,
    JOINT_GRASP_DOWN_POSE_RAD,
    JOINT_BASKET_POSE_RAD,
    JOINT_BASKET_DEMO_RELEASE_POSE_RAD,
)
from .panel_config import (
    BASKET_DROP,
    WORLD_FRAME,
    BASE_FRAME,
    GRIPPER_ATTACH_PREFIX,
    GRIPPER_CLOSED_RAD,
    GRIPPER_OPEN_RAD,
    GRIPPER_TCP_Z_OFFSET,
    UR5_JOINT_NAMES,
    GRIPPER_JOINT_NAMES,
)
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME
from .panel_objects import (
    mark_object_grasped,
    mark_object_attached,
    mark_object_released,
    get_object_state,
    is_on_table,
    update_object_state,
    ObjectOwner,
    ObjectLogicalState,
)
from .panel_readiness import tf_ready_status
from .panel_utils import transform_point_to_frame, world_to_base
from .ur5_kinematics import fk_ur5, ik_ur5


def _demo_object_in_basket(panel, timeout_sec: float = 4.0) -> bool:
    """Confirma por posicion que el objeto demo esta en la cesta."""
    start = time.monotonic()
    basket_world = tuple(float(v) for v in BASKET_DROP)
    base_frame = str(getattr(panel, "_base_frame_effective", "") or BASE_FRAME)
    basket_base, _ = transform_point_to_frame(
        basket_world,
        base_frame,
        source_frame=WORLD_FRAME,
    )
    xy_tol_world = 0.35
    z_tol_world = 0.35
    xy_tol_base = 0.30
    z_tol_base = 0.25
    while (time.monotonic() - start) <= timeout_sec:
        st = get_object_state(PICK_DEMO_OBJECT_NAME)
        if st is not None:
            xw, yw, zw = (float(st.position[0]), float(st.position[1]), float(st.position[2]))

            # Criterio en mundo (pose state suele almacenarse en WORLD_FRAME).
            dxw = xw - basket_world[0]
            dyw = yw - basket_world[1]
            dxy_world = (dxw * dxw + dyw * dyw) ** 0.5
            dz_world = abs(zw - basket_world[2])
            world_ok = dxy_world <= xy_tol_world and dz_world <= z_tol_world

            # Criterio alternativo en base_link para tolerar drift de origen.
            base_ok = False
            dxy_base = float("inf")
            dz_base = float("inf")
            if basket_base:
                obj_base, _ = transform_point_to_frame(
                    (xw, yw, zw),
                    base_frame,
                    source_frame=WORLD_FRAME,
                )
                if obj_base:
                    dxb = float(obj_base[0]) - float(basket_base[0])
                    dyb = float(obj_base[1]) - float(basket_base[1])
                    dxy_base = (dxb * dxb + dyb * dyb) ** 0.5
                    dz_base = abs(float(obj_base[2]) - float(basket_base[2]))
                    base_ok = dxy_base <= xy_tol_base and dz_base <= z_tol_base

            detached_ok = (not bool(st.attached)) and (st.owner == ObjectOwner.NONE)
            if detached_ok and (world_ok or base_ok):
                panel._emit_log(
                    "[PICK][DEMO] confirmacion cesta OK "
                    f"world_obj=({xw:.3f},{yw:.3f},{zw:.3f}) "
                    f"world_basket=({basket_world[0]:.3f},{basket_world[1]:.3f},{basket_world[2]:.3f}) "
                    f"dxy_w={dxy_world:.3f} dz_w={dz_world:.3f} "
                    f"dxy_b={dxy_base:.3f} dz_b={dz_base:.3f}"
                )
                return True
        time.sleep(0.2)
    panel._emit_log("[PICK][DEMO] confirmacion cesta NO alcanzada (timeout)")
    return False


def run_pick_demo(panel) -> None:
    panel._log_button("PICK MESA → CESTA")
    panel._emit_log("[DEMO] Inicio pick & place (mesa -> cesta)")
    if not panel._require_ready_basic("PICK DEMO"):
        return
    selected_name = str(getattr(panel, "_selected_object", "") or "").strip()
    user_selected = str(getattr(panel, "_selection_last_user_name", "") or "").strip()
    if selected_name != PICK_DEMO_OBJECT_NAME or user_selected != PICK_DEMO_OBJECT_NAME:
        panel._emit_log(
            "[PICK][DIRECT][ABORT] "
            f"selected={selected_name or 'none'} user_selected={user_selected or 'none'} "
            f"required={PICK_DEMO_OBJECT_NAME}"
        )
        panel._ui_set_status(
            "Directo: selecciona pick_demo antes de ejecutar",
            error=True,
        )
        return
    # Si _require_ready_basic pasó, el sistema está en READY_BASIC o superior.
    # Solo verificar que TF y EE frame estén disponibles (necesarios para pick).
    tf_ok, tf_reason = tf_ready_status(panel)
    if not tf_ok or not bool(panel._ee_frame_effective):
        panel._set_status(f"TF no listo; esperando pick ({tf_reason})", error=False)
        panel._emit_log(f"[PICK] Bloqueado: {panel._tf_not_ready_reason()}")
        return
    panel._emit_log(
        f"[PICK][DIRECT] selection_ok=true target={PICK_DEMO_OBJECT_NAME} route=direct_joint_only"
    )
    panel._emit_log("[PICK] Secuencia manual iniciada (ruta directa, MoveIt deshabilitado)")
    
    # Normalizar estado previo del objeto demo antes del grasp manual.
    obj_state = get_object_state(PICK_DEMO_OBJECT_NAME)
    if obj_state:
        if obj_state.logical_state in (ObjectLogicalState.GRASPED, ObjectLogicalState.CARRIED):
            panel._emit_log(f"[PICK] Limpiando estado anterior: {PICK_DEMO_OBJECT_NAME} era {obj_state.logical_state.value}")
            update_object_state(
                PICK_DEMO_OBJECT_NAME,
                logical_state=ObjectLogicalState.ON_TABLE,
                owner=ObjectOwner.NONE,
                attached=False,
                reason="pick_demo_cleanup"
            )
            panel._emit_log(f"[PICK] Estado limpiado: {PICK_DEMO_OBJECT_NAME} → ON_TABLE")
        elif obj_state.logical_state in (ObjectLogicalState.SPAWNED, ObjectLogicalState.RELEASED):
            panel._emit_log(
                f"[PICK] Normalizando estado previo: {PICK_DEMO_OBJECT_NAME} era {obj_state.logical_state.value} → ON_TABLE"
            )
            update_object_state(
                PICK_DEMO_OBJECT_NAME,
                logical_state=ObjectLogicalState.ON_TABLE,
                owner=ObjectOwner.NONE,
                attached=False,
                reason="pick_demo_on_table_normalize"
            )
    
    ready, reason = panel._controllers_ready()
    if not ready:
        panel._emit_log(f"[PICK] controladores no listos ({reason})")
        panel._set_status("Controladores no listos; esperando", error=False)
        return
    panel._emit_log("[PICK] target_source=selected_demo_object")
    panel._set_status("Pick demo: ejecutando secuencia manual…")
    panel._set_motion_lock(True)

    def worker():
        try:
            move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
            home_pose = panel._get_home_joint_pose()
            repo_root = Path(__file__).resolve().parents[4]
            debug_root = Path(
                os.environ.get("PANEL_DIRECT_DEBUG_ROOT")
                or (repo_root / "HISTORICO" / "panel_v2_20260321")
            )
            debug_root.mkdir(parents=True, exist_ok=True)
            trace_path = debug_root / "DIRECTO_DEBUG_TRACE.log"
            run_id = (
                datetime.now(timezone.utc)
                .astimezone()
                .strftime("%Y%m%d_%H%M%S")
                + "_"
                + uuid.uuid4().hex[:8]
            )
            snapshots_root = debug_root / "DIRECTO_DEBUG_SNAPSHOTS"
            run_snapshots_dir = snapshots_root / run_id
            run_snapshots_dir.mkdir(parents=True, exist_ok=True)
            phase_seq = {"value": 0}
            current_phase = {"data": None}
            last_target = {
                "phase": None,
                "target_world": None,
                "target_base": None,
                "frame": None,
                "offsets": None,
                "joint_goal": None,
                "ik_solution": None,
                "note": None,
            }
            demo_follow_confirmed = False

            def _iso_now() -> str:
                return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")

            def _json_safe(value):
                if value is None:
                    return None
                if isinstance(value, (str, int, float, bool)):
                    return value
                if isinstance(value, Path):
                    return str(value)
                if isinstance(value, (list, tuple)):
                    return [_json_safe(v) for v in value]
                if isinstance(value, dict):
                    return {str(k): _json_safe(v) for k, v in value.items()}
                try:
                    return float(value)
                except Exception:
                    return str(value)

            def _append_trace(line: str) -> None:
                stamped = f"{_iso_now()} {line}"
                try:
                    with trace_path.open("a", encoding="utf-8") as fh:
                        fh.write(stamped + "\n")
                except Exception:
                    pass
                panel._emit_log(line)

            def _tuple3(data):
                if data is None:
                    return None
                try:
                    return (
                        float(data[0]),
                        float(data[1]),
                        float(data[2]),
                    )
                except Exception:
                    return None

            def _fmt_vec(vec) -> str:
                vec3 = _tuple3(vec)
                if vec3 is None:
                    return "none"
                return f"({vec3[0]:.3f},{vec3[1]:.3f},{vec3[2]:.3f})"

            def _vector_minus(a, b):
                av = _tuple3(a)
                bv = _tuple3(b)
                if av is None or bv is None:
                    return None
                return (
                    float(av[0]) - float(bv[0]),
                    float(av[1]) - float(bv[1]),
                    float(av[2]) - float(bv[2]),
                )

            def _vec_norm(vec) -> float | None:
                v = _tuple3(vec)
                if v is None:
                    return None
                return math.sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]))

            def _joint_error_snapshot(joints):
                names = list(getattr(panel, "UR5_JOINT_NAMES", []) or [])
                if not names:
                    names = [
                        "shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint",
                    ]
                parts = []
                for idx, name in enumerate(names):
                    if idx >= len(joints):
                        break
                    curr = panel._last_joint_positions.get(name)
                    if curr is None:
                        parts.append(f"{name}=n/a")
                        continue
                    diff = abs(float(curr) - float(joints[idx]))
                    parts.append(f"{name}={diff:.3f}")
                return " ".join(parts)

            def _run_joint_step(label, joints, timeout_sec=None, tol_rad=0.02):
                panel._emit_log(f"[PICK] Paso joint: {label}")
                ok, info = panel._publish_joint_trajectory(joints, move_sec)
                if not ok:
                    raise RuntimeError(f"{label} fallo: {info}")
                wait_timeout = move_sec + 2.0 if timeout_sec is None else timeout_sec
                if panel._wait_for_joint_target(joints, wait_timeout, tol_rad=tol_rad):
                    return
                if label in {"HOME", "MESA", "PICK_IMAGE", "HOME_WITH_OBJECT", "CESTA", "CESTA_RELEASE"}:
                    panel._emit_log(
                        f"[PICK][RECOVERY] {label} no alcanzado; reintentando una vez diffs={_joint_error_snapshot(joints)}"
                    )
                    ok_retry, info_retry = panel._publish_joint_trajectory(joints, move_sec)
                    if not ok_retry:
                        raise RuntimeError(f"{label} retry fallo: {info_retry}")
                    retry_timeout = max(wait_timeout, move_sec + 4.0)
                    retry_tol = max(tol_rad, 0.06)
                    if panel._wait_for_joint_target(joints, retry_timeout, tol_rad=retry_tol):
                        panel._emit_log(f"[PICK][RECOVERY] {label} alcanzado tras reintento")
                        return
                raise RuntimeError(
                    f"{label} no alcanzado (timeout) diffs={_joint_error_snapshot(joints)}"
                )

            def _dist(a, b) -> float:
                dx = float(a[0]) - float(b[0])
                dy = float(a[1]) - float(b[1])
                dz = float(a[2]) - float(b[2])
                return math.sqrt(dx * dx + dy * dy + dz * dz)

            def _live_object_world():
                if panel._ros_worker_started and panel.ros_worker and panel.ros_worker.node_ready():
                    try:
                        pose_map, _pose_ts = panel.ros_worker.pose_snapshot()
                        pose = (pose_map or {}).get(PICK_DEMO_OBJECT_NAME)
                        if pose is not None and len(pose) >= 3:
                            return (float(pose[0]), float(pose[1]), float(pose[2]))
                    except Exception:
                        pass
                st = get_object_state(PICK_DEMO_OBJECT_NAME)
                if st is None:
                    return None
                return (
                    float(st.position[0]),
                    float(st.position[1]),
                    float(st.position[2]),
                )

            def _live_object_base():
                world_pos = _live_object_world()
                if world_pos is None:
                    return None
                try:
                    base_frame = str(panel._business_base_frame() or BASE_FRAME or "base_link")
                except Exception:
                    base_frame = str(BASE_FRAME or "base_link")
                obj_base, _ = transform_point_to_frame(
                    world_pos,
                    base_frame,
                    source_frame=WORLD_FRAME,
                )
                if not obj_base:
                    try:
                        return tuple(float(v) for v in world_to_base(*world_pos))
                    except Exception:
                        return None
                return (float(obj_base[0]), float(obj_base[1]), float(obj_base[2]))

            def _live_tcp_base():
                try:
                    tcp_pose = panel.get_tcp_base()
                except Exception:
                    tcp_pose = None
                if tcp_pose is None:
                    tcp_world = getattr(panel, "_last_tcp_world", None)
                    if tcp_world is not None:
                        try:
                            return tuple(float(v) for v in world_to_base(*tcp_world))
                        except Exception:
                            return None
                    return None
                pos = tcp_pose.pose.position
                return (float(pos.x), float(pos.y), float(pos.z))

            def _live_tcp_world():
                tcp_base = _live_tcp_base()
                if tcp_base is not None:
                    try:
                        world = panel._base_to_world_coords(tcp_base)
                    except Exception:
                        world = None
                    if world is not None:
                        return (float(world[0]), float(world[1]), float(world[2]))
                tcp_world = getattr(panel, "_last_tcp_world", None)
                return _tuple3(tcp_world)

            def _current_joint_seed():
                seed = []
                try:
                    snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
                except Exception:
                    snapshot = {}
                for joint_name in UR5_JOINT_NAMES:
                    if joint_name not in snapshot:
                        return list(JOINT_GRASP_DOWN_POSE_RAD)
                    seed.append(float(snapshot[joint_name]))
                return seed

            def _read_gripper_state():
                joint_snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
                positions = {}
                for joint_name in GRIPPER_JOINT_NAMES:
                    if joint_name in joint_snapshot:
                        positions[joint_name] = float(joint_snapshot[joint_name])
                target_mag = abs(
                    float(GRIPPER_CLOSED_RAD if getattr(panel, "_gripper_closed", False) else GRIPPER_OPEN_RAD)
                )
                tol_m = max(
                    0.005,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M",
                            "0.035",
                        )
                        or 0.035
                    ),
                )
                opening_sum = None
                max_abs_err = None
                measured_target_ok = None
                if positions:
                    magnitudes = [abs(float(pos)) for pos in positions.values()]
                    opening_sum = float(sum(magnitudes))
                    max_abs_err = float(max(abs(mag - target_mag) for mag in magnitudes))
                    measured_target_ok = bool(max_abs_err <= tol_m)
                joint_state_age_sec = None
                if panel._ros_worker_started and panel.ros_worker is not None:
                    try:
                        _payload, ts = panel.ros_worker.get_last_joint_state()
                    except Exception:
                        ts = 0.0
                    if ts:
                        try:
                            joint_state_age_sec = max(0.0, time.time() - float(ts))
                        except Exception:
                            joint_state_age_sec = None
                force_est = None
                try:
                    force_est = float(panel._get_gripper_force())
                except Exception:
                    force_est = None
                return {
                    "closed_flag": bool(getattr(panel, "_gripper_closed", False)),
                    "joint_positions": positions,
                    "target_mag": float(target_mag),
                    "opening_sum": opening_sum,
                    "max_abs_err": max_abs_err,
                    "measured_target_ok": measured_target_ok,
                    "joint_state_age_sec": joint_state_age_sec,
                    "force_estimate": force_est,
                }

            def _wait_for_gripper_target(
                closed: bool,
                *,
                timeout_sec: float = 1.8,
            ):
                required_hits = max(
                    1,
                    int(
                        os.environ.get(
                            "PANEL_PICK_DEMO_GRIPPER_CONFIRM_STABLE_SAMPLES",
                            "2",
                        )
                        or 2
                    ),
                )
                max_state_age_sec = max(
                    0.05,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_GRIPPER_CONFIRM_MAX_STATE_AGE_SEC",
                            "0.35",
                        )
                        or 0.35
                    ),
                )
                start = time.monotonic()
                stable_hits = 0
                last_state = _read_gripper_state()
                _append_trace(
                    "[PICK][DIRECT][GRIPPER] "
                    f"wait_start target={'closed' if closed else 'open'} "
                    f"target_mag={last_state.get('target_mag')} timeout={timeout_sec:.2f}s "
                    f"stable_hits={required_hits}"
                )
                while (time.monotonic() - start) <= timeout_sec:
                    state = _read_gripper_state()
                    last_state = state
                    measured_ok = bool(state.get("measured_target_ok"))
                    age_ok = state.get("joint_state_age_sec") is None or float(state.get("joint_state_age_sec")) <= max_state_age_sec
                    if measured_ok and age_ok:
                        stable_hits += 1
                        if stable_hits >= required_hits:
                            _append_trace(
                                "[PICK][DIRECT][GRIPPER] "
                                f"wait_ok target={'closed' if closed else 'open'} "
                                f"opening_sum={state.get('opening_sum')} max_abs_err={state.get('max_abs_err')} "
                                f"age={state.get('joint_state_age_sec')}"
                            )
                            return True, state
                    else:
                        stable_hits = 0
                    time.sleep(0.05)
                _append_trace(
                    "[PICK][DIRECT][GRIPPER] "
                    f"wait_timeout target={'closed' if closed else 'open'} "
                    f"opening_sum={last_state.get('opening_sum')} max_abs_err={last_state.get('max_abs_err')} "
                    f"age={last_state.get('joint_state_age_sec')}"
                )
                return False, last_state

            def _read_attach_state():
                st = get_object_state(PICK_DEMO_OBJECT_NAME)
                logical_state = None
                owner = None
                attached = None
                if st is not None:
                    logical_state = getattr(getattr(st, "logical_state", None), "value", None)
                    owner = getattr(getattr(st, "owner", None), "value", None)
                    attached = bool(getattr(st, "attached", False))
                return {
                    "logical_attached": attached,
                    "logical_state": logical_state,
                    "owner": owner,
                    "attach_topic_published": bool(demo_attach_published),
                    "follow_confirmed": bool(demo_follow_confirmed),
                }

            def _close_alignment_metrics():
                obj_base = _live_object_base()
                tcp_base = _live_tcp_base()
                if obj_base is None or tcp_base is None:
                    return {
                        "ok": False,
                        "reason": "pose_unavailable",
                        "xy_dist": None,
                        "z_gap": None,
                        "z_error": None,
                        "tcp_obj_dist": None,
                        "tcp_base": _tuple3(tcp_base),
                        "object_base": _tuple3(obj_base),
                    }
                xy_dist = math.hypot(
                    float(tcp_base[0]) - float(obj_base[0]),
                    float(tcp_base[1]) - float(obj_base[1]),
                )
                z_gap = float(tcp_base[2]) - float(obj_base[2])
                z_error = abs(z_gap - float(GRIPPER_TCP_Z_OFFSET))
                tcp_obj_dist = _dist(tcp_base, obj_base)
                xy_tol = max(
                    0.01,
                    float(os.environ.get("PANEL_PICK_DEMO_CLOSE_XY_TOL_M", "0.035") or 0.035),
                )
                z_tol = max(
                    0.01,
                    float(os.environ.get("PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M", "0.030") or 0.030),
                )
                gripper_state = _read_gripper_state()
                gripper_closed_measured = bool(gripper_state.get("measured_target_ok"))
                ok = gripper_closed_measured and xy_dist <= xy_tol and z_error <= z_tol
                return {
                    "ok": ok,
                    "reason": "ok" if ok else "alignment_out_of_tolerance",
                    "xy_dist": float(xy_dist),
                    "z_gap": float(z_gap),
                    "z_error": float(z_error),
                    "tcp_obj_dist": float(tcp_obj_dist),
                    "xy_tol": float(xy_tol),
                    "z_tol": float(z_tol),
                    "gripper_closed_measured": gripper_closed_measured,
                    "gripper_opening_sum": gripper_state.get("opening_sum"),
                    "gripper_max_abs_err": gripper_state.get("max_abs_err"),
                    "tcp_base": _tuple3(tcp_base),
                    "object_base": _tuple3(obj_base),
                }

            def _target_world_from_base(target_base):
                base_coords = _tuple3(target_base)
                if base_coords is None:
                    return None
                try:
                    world = panel._base_to_world_coords(base_coords)
                except Exception:
                    world = None
                return _tuple3(world)

            def _write_json_snapshot(path: Path, payload: dict) -> None:
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("w", encoding="utf-8") as fh:
                    json.dump(_json_safe(payload), fh, indent=2, ensure_ascii=False, sort_keys=True)

            def _phase_target_update(
                phase: str,
                *,
                target_world=None,
                target_base=None,
                frame_used: str | None = None,
                offsets=None,
                joint_goal=None,
                ik_solution=None,
                note: str | None = None,
            ) -> None:
                target_world_3 = _tuple3(target_world) or _target_world_from_base(target_base)
                target_base_3 = _tuple3(target_base)
                last_target.update(
                    {
                        "phase": phase,
                        "target_world": target_world_3,
                        "target_base": target_base_3,
                        "frame": frame_used,
                        "offsets": _json_safe(offsets),
                        "joint_goal": _json_safe(joint_goal),
                        "ik_solution": _json_safe(ik_solution),
                        "note": note,
                    }
                )
                _append_trace(
                    "[PICK][DIRECT][DEBUG] "
                    f"TARGET_UPDATE phase={phase} frame={frame_used or 'none'} "
                    f"target_world={_fmt_vec(target_world_3)} target_base={_fmt_vec(target_base_3)} "
                    f"offsets={json.dumps(_json_safe(offsets) or {}, ensure_ascii=False, sort_keys=True)} "
                    f"note={note or 'none'}"
                )

            def _phase_begin(
                phase: str,
                *,
                target_world=None,
                target_base=None,
                frame_used: str | None = None,
                offsets=None,
                joint_goal=None,
                ik_solution=None,
                note: str | None = None,
            ) -> None:
                phase_seq["value"] += 1
                _append_trace(f"[PICK][DIRECT][DEBUG] ENTER_PHASE {phase}")
                if any(v is not None for v in (target_world, target_base, frame_used, offsets, joint_goal, ik_solution, note)):
                    _phase_target_update(
                        phase,
                        target_world=target_world,
                        target_base=target_base,
                        frame_used=frame_used,
                        offsets=offsets,
                        joint_goal=joint_goal,
                        ik_solution=ik_solution,
                        note=note,
                    )
                current_phase["data"] = {
                    "seq": int(phase_seq["value"]),
                    "phase": phase,
                    "timestamp_enter": _iso_now(),
                    "object_name": PICK_DEMO_OBJECT_NAME,
                    "object_pose_world_before": _tuple3(_live_object_world()),
                    "tcp_pose_world_before": _tuple3(_live_tcp_world()),
                    "tcp_pose_base_before": _tuple3(_live_tcp_base()),
                    "target_pose_world": _tuple3(target_world) or _tuple3(last_target.get("target_world")),
                    "target_pose_base": _tuple3(target_base) or _tuple3(last_target.get("target_base")),
                    "frame_usado": frame_used or last_target.get("frame"),
                    "offsets_aplicados": _json_safe(offsets if offsets is not None else last_target.get("offsets")),
                    "joint_goal": _json_safe(joint_goal if joint_goal is not None else last_target.get("joint_goal")),
                    "ik_solution": _json_safe(ik_solution if ik_solution is not None else last_target.get("ik_solution")),
                    "note": note or last_target.get("note"),
                }

            def _phase_end(
                phase: str,
                *,
                target_world=None,
                target_base=None,
                frame_used: str | None = None,
                offsets=None,
                joint_goal=None,
                ik_solution=None,
                attach_state=None,
                note: str | None = None,
                result: str = "ok",
            ) -> dict:
                data = dict(current_phase.get("data") or {})
                if not data or data.get("phase") != phase:
                    data = {
                        "seq": int(phase_seq["value"]) + 1,
                        "phase": phase,
                        "timestamp_enter": _iso_now(),
                        "object_name": PICK_DEMO_OBJECT_NAME,
                    }
                target_world_3 = _tuple3(target_world) or _tuple3(data.get("target_pose_world")) or _tuple3(last_target.get("target_world"))
                target_base_3 = _tuple3(target_base) or _tuple3(data.get("target_pose_base")) or _tuple3(last_target.get("target_base"))
                tcp_world_after = _tuple3(_live_tcp_world())
                obj_world_after = _tuple3(_live_object_world())
                tcp_base_after = _tuple3(_live_tcp_base())
                obj_base_after = _tuple3(_live_object_base())
                target_minus_tcp_before = _vector_minus(target_world_3, data.get("tcp_pose_world_before"))
                target_minus_tcp_after = _vector_minus(target_world_3, tcp_world_after)
                dist_tcp_obj_before = None
                dist_tcp_obj_after = None
                dist_target_tcp_before = _vec_norm(target_minus_tcp_before)
                dist_target_tcp_after = _vec_norm(target_minus_tcp_after)
                if data.get("tcp_pose_world_before") is not None and data.get("object_pose_world_before") is not None:
                    dist_tcp_obj_before = _dist(data["tcp_pose_world_before"], data["object_pose_world_before"])
                if tcp_world_after is not None and obj_world_after is not None:
                    dist_tcp_obj_after = _dist(tcp_world_after, obj_world_after)
                attach_payload = _json_safe(attach_state if attach_state is not None else _read_attach_state())
                gripper_payload = _json_safe(_read_gripper_state())
                payload = {
                    **data,
                    "timestamp_exit": _iso_now(),
                    "result": result,
                    "object_pose_world_after": obj_world_after,
                    "object_pose_world": obj_world_after or data.get("object_pose_world_before"),
                    "tcp_pose_world_after": tcp_world_after,
                    "tcp_pose_base_after": tcp_base_after,
                    "object_pose_base_after": obj_base_after,
                    "target_pose_world": target_world_3,
                    "target_pose_base": target_base_3,
                    "target_minus_tcp_before": _json_safe(target_minus_tcp_before),
                    "target_minus_tcp_after": _json_safe(target_minus_tcp_after),
                    "frame_usado": frame_used or data.get("frame_usado") or last_target.get("frame"),
                    "offsets_aplicados": _json_safe(offsets if offsets is not None else data.get("offsets_aplicados")),
                    "joint_goal": _json_safe(joint_goal if joint_goal is not None else data.get("joint_goal")),
                    "ik_solution": _json_safe(ik_solution if ik_solution is not None else data.get("ik_solution")),
                    "gripper_state": gripper_payload,
                    "attach_state": attach_payload,
                    "dist_tcp_obj_before": dist_tcp_obj_before,
                    "dist_tcp_obj_after": dist_tcp_obj_after,
                    "dist_tcp_obj": dist_tcp_obj_after if dist_tcp_obj_after is not None else dist_tcp_obj_before,
                    "dist_target_tcp_before": dist_target_tcp_before,
                    "dist_target_tcp_after": dist_target_tcp_after,
                    "dist_target_tcp": dist_target_tcp_after if dist_target_tcp_after is not None else dist_target_tcp_before,
                    "note": note or data.get("note"),
                }
                snap_name = f"{int(payload['seq']):02d}_{phase}.json"
                _write_json_snapshot(run_snapshots_dir / snap_name, payload)
                _append_trace(
                    "[PICK][DIRECT][DEBUG] "
                    f"phase={phase} result={result} "
                    f"obj_world={_fmt_vec(payload.get('object_pose_world'))} "
                    f"target_world={_fmt_vec(target_world_3)} "
                    f"tcp_before={_fmt_vec(payload.get('tcp_pose_world_before'))} "
                    f"tcp_after={_fmt_vec(tcp_world_after)} "
                    f"d_obj_tcp={payload.get('dist_tcp_obj') if payload.get('dist_tcp_obj') is not None else float('nan'):.3f} "
                    f"d_target_tcp={payload.get('dist_target_tcp') if payload.get('dist_target_tcp') is not None else float('nan'):.3f} "
                    f"frame={payload.get('frame_usado') or 'none'} "
                    f"gripper_closed={bool(gripper_payload.get('closed_flag'))} "
                    f"gripper_measured={bool(gripper_payload.get('measured_target_ok'))} "
                    f"attach={json.dumps(attach_payload or {}, ensure_ascii=False, sort_keys=True)}"
                )
                _append_trace(f"[PICK][DIRECT][DEBUG] EXIT_PHASE {phase}")
                current_phase["data"] = None
                return payload

            def _grasp_failure_analysis(
                *,
                code: str,
                phase: str,
                note: str | None = None,
                metrics=None,
            ) -> Path:
                target_world = _tuple3(last_target.get("target_world"))
                target_base = _tuple3(last_target.get("target_base"))
                tcp_world = _tuple3(_live_tcp_world())
                tcp_base = _tuple3(_live_tcp_base())
                obj_world = _tuple3(_live_object_world())
                obj_base = _tuple3(_live_object_base())
                target_minus_tcp = _vector_minus(target_world, tcp_world)
                tcp_obj_dist = _dist(tcp_world, obj_world) if tcp_world is not None and obj_world is not None else None
                mode = "unknown"
                metric_dict = _json_safe(metrics) or {}
                xy_dist = metric_dict.get("xy_dist")
                z_error = metric_dict.get("z_error")
                if xy_dist is not None or z_error is not None:
                    if (xy_dist or 0.0) > float(metric_dict.get("xy_tol") or 0.0) or (z_error or 0.0) > float(metric_dict.get("z_tol") or 0.0):
                        mode = "close_outside_object"
                    else:
                        mode = "object_lost_after_close"
                payload = {
                    "timestamp": _iso_now(),
                    "failure_code": code,
                    "phase": phase,
                    "failure_mode": mode,
                    "note": note,
                    "object_name": PICK_DEMO_OBJECT_NAME,
                    "tcp_pose_world": tcp_world,
                    "tcp_pose_base": tcp_base,
                    "object_pose_world": obj_world,
                    "object_pose_base": obj_base,
                    "dist_tcp_obj": tcp_obj_dist,
                    "last_target_world": target_world,
                    "last_target_base": target_base,
                    "last_target_frame": last_target.get("frame"),
                    "last_offsets": _json_safe(last_target.get("offsets")),
                    "target_minus_tcp": _json_safe(target_minus_tcp),
                    "gripper_state": _json_safe(_read_gripper_state()),
                    "attach_state": _json_safe(_read_attach_state()),
                    "metrics": metric_dict,
                    "snapshot_dir": str(run_snapshots_dir),
                }
                analysis_path = run_snapshots_dir / f"analysis_{int(phase_seq['value']) + 1:02d}_{code}.json"
                _write_json_snapshot(analysis_path, payload)
                _append_trace(
                    "[PICK][DIRECT][ANALYSIS] "
                    f"code={code} phase={phase} mode={mode} "
                    f"tcp_world={_fmt_vec(tcp_world)} obj_world={_fmt_vec(obj_world)} "
                    f"dist_tcp_obj={tcp_obj_dist if tcp_obj_dist is not None else float('nan'):.3f} "
                    f"last_target_world={_fmt_vec(target_world)} "
                    f"metrics={json.dumps(metric_dict, ensure_ascii=False, sort_keys=True)}"
                )
                return analysis_path

            def _abort_grasp(
                *,
                code: str,
                phase: str,
                note: str | None = None,
                metrics=None,
            ) -> None:
                analysis_path = _grasp_failure_analysis(
                    code=code,
                    phase=phase,
                    note=note,
                    metrics=metrics,
                )
                raise RuntimeError(
                    f"{code} phase={phase} note={note or 'none'} analysis={analysis_path}"
                )

            def _move_tcp_direct(*, label: str, target_tcp_runtime, timeout_sec: float) -> None:
                tcp_base = _live_tcp_base()
                if tcp_base is None:
                    raise RuntimeError(f"{label.lower()}_tcp_pose_unavailable")
                seed = _current_joint_seed()
                seed_pos, target_rot = fk_ur5(seed)
                delta_runtime = (
                    float(target_tcp_runtime[0]) - float(tcp_base[0]),
                    float(target_tcp_runtime[1]) - float(tcp_base[1]),
                    float(target_tcp_runtime[2]) - float(tcp_base[2]),
                )
                # La FK/IK numerica trabaja en tool0 y en un frame numerico donde X/Y van
                # invertidos respecto al tcp_base(=rg2_tcp) del panel. Convertimos primero
                # el target real de rg2_tcp al frame numerico y luego restamos el offset
                # fijo tool0->rg2_tcp sobre el eje Z local del efector.
                tcp_offset_m = max(
                    0.0,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M",
                            "0.230",
                        )
                        or 0.230
                    ),
                )
                target_numeric_tcp = (
                    -float(target_tcp_runtime[0]),
                    -float(target_tcp_runtime[1]),
                    float(target_tcp_runtime[2]),
                )
                target_ik = (
                    float(target_numeric_tcp[0]) - float(target_rot[0, 2]) * tcp_offset_m,
                    float(target_numeric_tcp[1]) - float(target_rot[1, 2]) * tcp_offset_m,
                    float(target_numeric_tcp[2]) - float(target_rot[2, 2]) * tcp_offset_m,
                )
                solved_q, err_norm, ik_ok = ik_ur5(
                    target_ik,
                    target_rot,
                    seed,
                    max_iter=240,
                    pos_weight=1.0,
                    rot_weight=0.35,
                )
                panel._emit_log(
                    "[PICK][DIRECT][IK] "
                    f"label={label} "
                    f"tcp_now=({tcp_base[0]:.3f},{tcp_base[1]:.3f},{tcp_base[2]:.3f}) "
                    f"target_tcp=({target_tcp_runtime[0]:.3f},{target_tcp_runtime[1]:.3f},{target_tcp_runtime[2]:.3f}) "
                    f"delta_runtime=({delta_runtime[0]:.3f},{delta_runtime[1]:.3f},{delta_runtime[2]:.3f}) "
                    f"target_numeric_tcp=({target_numeric_tcp[0]:.3f},{target_numeric_tcp[1]:.3f},{target_numeric_tcp[2]:.3f}) "
                    f"target_ik=({target_ik[0]:.3f},{target_ik[1]:.3f},{target_ik[2]:.3f}) "
                    f"tcp_offset_m={tcp_offset_m:.3f} "
                    f"err_norm={float(err_norm):.4f} success={str(bool(ik_ok)).lower()}"
                )
                if (not ik_ok) or float(err_norm) > 0.035:
                    raise RuntimeError(
                        f"{label.lower()}_ik_failed err_norm={float(err_norm):.4f}"
                    )
                solved_q_list = [float(v) for v in solved_q.tolist()]
                _run_joint_step(
                    label,
                    solved_q_list,
                    timeout_sec=max(float(timeout_sec), move_sec + 2.0),
                    tol_rad=0.08,
                )
                tcp_after = _live_tcp_base()
                if tcp_after is not None:
                    obj_after = _live_object_base()
                    target_err = _dist(tcp_after, target_tcp_runtime)
                    obj_dist = _dist(tcp_after, obj_after) if obj_after is not None else float("nan")
                    panel._emit_log(
                        "[PICK][DIRECT][IK] "
                        f"label={label} "
                        f"tcp_after=({tcp_after[0]:.3f},{tcp_after[1]:.3f},{tcp_after[2]:.3f}) "
                        f"target_err={target_err:.3f} "
                        f"obj_dist={obj_dist:.3f}"
                    )
                return {
                    "label": label,
                    "seed": [float(v) for v in seed],
                    "target_tcp_runtime": _tuple3(target_tcp_runtime),
                    "target_numeric_tcp": _tuple3(target_numeric_tcp),
                    "target_ik": _tuple3(target_ik),
                    "tcp_offset_m": float(tcp_offset_m),
                    "ik_solution": solved_q_list,
                    "err_norm": float(err_norm),
                    "ik_ok": bool(ik_ok),
                }

            def _align_demo_grasp_direct() -> None:
                obj_base = _live_object_base()
                if obj_base is None:
                    raise RuntimeError("demo_object_pose_unavailable_before_align")
                target_tcp_runtime = (
                    float(obj_base[0]),
                    float(obj_base[1]),
                    float(obj_base[2]) + float(GRIPPER_TCP_Z_OFFSET),
                )
                return _move_tcp_direct(
                    label="GRASP_ALIGN_IK",
                    target_tcp_runtime=target_tcp_runtime,
                    timeout_sec=move_sec + 8.0,
                )

            def _wait_demo_attach_follow(
                *,
                timeout_sec: float,
                max_tcp_dist_m: float,
                min_consecutive: int = 3,
            ) -> None:
                deadline = time.time() + max(0.4, float(timeout_sec))
                consecutive_ok = 0
                best_tcp_dist = float("inf")
                last_obj_base = None
                last_tcp_base = None
                panel._emit_log(
                    "[PICK][DIRECT][ATTACH] "
                    f"waiting_follow timeout={float(timeout_sec):.2f}s "
                    f"max_tcp_dist={float(max_tcp_dist_m):.3f}"
                )
                while time.time() < deadline:
                    obj_base = _live_object_base()
                    tcp_base = _live_tcp_base()
                    last_obj_base = obj_base
                    last_tcp_base = tcp_base
                    if obj_base is None or tcp_base is None:
                        consecutive_ok = 0
                        time.sleep(0.08)
                        continue
                    tcp_dist = _dist(obj_base, tcp_base)
                    best_tcp_dist = min(best_tcp_dist, tcp_dist)
                    if tcp_dist <= float(max_tcp_dist_m):
                        consecutive_ok += 1
                        if consecutive_ok >= max(1, int(min_consecutive)):
                            panel._emit_log(
                                "[PICK][DIRECT][ATTACH] "
                                f"follow_confirmed tcp_dist={tcp_dist:.3f} "
                                f"consecutive={consecutive_ok}"
                            )
                            return
                    else:
                        consecutive_ok = 0
                    time.sleep(0.08)
                obj_txt = "none"
                tcp_txt = "none"
                if last_obj_base is not None:
                    obj_txt = (
                        f"({float(last_obj_base[0]):.3f},{float(last_obj_base[1]):.3f},"
                        f"{float(last_obj_base[2]):.3f})"
                    )
                if last_tcp_base is not None:
                    tcp_txt = (
                        f"({float(last_tcp_base[0]):.3f},{float(last_tcp_base[1]):.3f},"
                        f"{float(last_tcp_base[2]):.3f})"
                    )
                raise RuntimeError(
                    "demo_attach_follow_not_confirmed "
                    f"best_tcp_dist={best_tcp_dist:.3f} "
                    f"last_obj_base={obj_txt} last_tcp_base={tcp_txt}"
                )

            def _lift_demo_object_direct(lift_m: float) -> None:
                tcp_base = _live_tcp_base()
                if tcp_base is None:
                    raise RuntimeError("demo_tcp_pose_unavailable_before_lift")
                target_tcp_runtime = (
                    float(tcp_base[0]),
                    float(tcp_base[1]),
                    float(tcp_base[2]) + float(lift_m),
                )
                return _move_tcp_direct(
                    label="POST_GRASP_LIFT_IK",
                    target_tcp_runtime=target_tcp_runtime,
                    timeout_sec=move_sec + 8.0,
                )

            def _detach_demo_object(reason: str) -> None:
                detach_topic = f"{GRIPPER_ATTACH_PREFIX}/{PICK_DEMO_OBJECT_NAME}/detach"
                if Empty is None:
                    panel._emit_log(
                        f"[PICK][DIRECT][DETACH] skipped reason={reason} topic={detach_topic} empty_msg_unavailable"
                    )
                    return
                try:
                    detach_pub = panel._get_attach_publisher(detach_topic)
                    detach_subs = -1
                    if detach_pub is not None and panel._ros_worker_started and panel.ros_worker and panel.ros_worker.node_ready():
                        detach_subs = int(panel.ros_worker.topic_subscriber_count(detach_topic))
                    if detach_pub is None:
                        panel._emit_log(
                            f"[PICK][DIRECT][DETACH] skipped reason={reason} topic={detach_topic} publisher_unavailable"
                        )
                        return
                    if detach_subs == 0:
                        panel._emit_log(
                            f"[PICK][DIRECT][DETACH] skipped reason={reason} topic={detach_topic} no_subscribers"
                        )
                        return
                    detach_pub.publish(Empty())
                    panel._emit_log(
                        f"[PICK][DIRECT][DETACH] publish reason={reason} topic={detach_topic}"
                    )
                except Exception as exc:
                    panel._emit_log(
                        f"[PICK][DIRECT][DETACH] error reason={reason} topic={detach_topic} exc={exc}"
                    )

            def _validate_demo_carry(
                *,
                initial_obj_world,
                phase: str,
                timeout_sec: float,
                min_obj_move_m: float,
                min_lift_delta_m: float,
                max_tcp_dist_m: float,
                min_consecutive: int = 2,
            ) -> None:
                deadline = time.time() + max(0.3, float(timeout_sec))
                consecutive_ok = 0
                best_obj_move = 0.0
                best_lift = float("-inf")
                best_tcp_dist = float("inf")
                last_obj_world = None
                last_tcp_base = None
                panel._emit_log(
                    "[PICK][DIRECT][PHYSICS] "
                    f"phase={phase} start initial_obj_world=({initial_obj_world[0]:.3f},{initial_obj_world[1]:.3f},{initial_obj_world[2]:.3f}) "
                    f"min_obj_move={float(min_obj_move_m):.3f} min_lift_delta={float(min_lift_delta_m):.3f} "
                    f"max_tcp_dist={float(max_tcp_dist_m):.3f}"
                )
                while time.time() < deadline:
                    obj_world = _live_object_world()
                    obj_base = _live_object_base()
                    tcp_base = _live_tcp_base()
                    last_obj_world = obj_world
                    last_tcp_base = tcp_base
                    if obj_world is None or obj_base is None or tcp_base is None:
                        consecutive_ok = 0
                        time.sleep(0.08)
                        continue
                    obj_move = _dist(obj_world, initial_obj_world)
                    lift_delta = float(obj_world[2]) - float(initial_obj_world[2])
                    tcp_dist = _dist(obj_base, tcp_base)
                    best_obj_move = max(best_obj_move, obj_move)
                    best_lift = max(best_lift, lift_delta)
                    best_tcp_dist = min(best_tcp_dist, tcp_dist)
                    if (
                        obj_move >= float(min_obj_move_m)
                        and lift_delta >= float(min_lift_delta_m)
                        and tcp_dist <= float(max_tcp_dist_m)
                    ):
                        consecutive_ok += 1
                        if consecutive_ok >= max(1, int(min_consecutive)):
                            panel._emit_log(
                                "[PICK][DIRECT][PHYSICS] "
                                f"phase={phase} ok obj_move={obj_move:.3f} lift_delta={lift_delta:.3f} "
                                f"tcp_dist={tcp_dist:.3f} consecutive={consecutive_ok}"
                            )
                            return
                    else:
                        consecutive_ok = 0
                    time.sleep(0.08)
                obj_txt = "none"
                tcp_txt = "none"
                if last_obj_world is not None:
                    obj_txt = f"({last_obj_world[0]:.3f},{last_obj_world[1]:.3f},{last_obj_world[2]:.3f})"
                if last_tcp_base is not None:
                    tcp_txt = f"({last_tcp_base[0]:.3f},{last_tcp_base[1]:.3f},{last_tcp_base[2]:.3f})"
                raise RuntimeError(
                    "demo_carry_validation_failed "
                    f"phase={phase} best_obj_move={best_obj_move:.3f} best_lift_delta={best_lift:.3f} "
                    f"best_tcp_dist={best_tcp_dist:.3f} last_obj_world={obj_txt} last_tcp_base={tcp_txt}"
                )

            demo_attach_published = False
            demo_logical_attached = False
            _run_joint_step("HOME", home_pose)
            _run_joint_step("MESA", JOINT_TABLE_POSE_RAD)

            panel._emit_log("[DEMO] Abriendo pinza en posición MESA")
            panel.signal_run_ui.emit(lambda: panel._command_gripper(False, log_action="PICK", force=True))
            time.sleep(0.6)

            _run_joint_step("PICK_IMAGE", JOINT_PICK_IMAGE_POSE_RAD)

            panel._emit_log("[DEMO] Bajando a pose de grasp (joints)")
            obj_base_before_coarse = _live_object_base()
            target_base_coarse = None
            target_world_coarse = None
            if obj_base_before_coarse is not None:
                target_base_coarse = (
                    float(obj_base_before_coarse[0]),
                    float(obj_base_before_coarse[1]),
                    float(obj_base_before_coarse[2]) + float(GRIPPER_TCP_Z_OFFSET),
                )
                target_world_coarse = _target_world_from_base(target_base_coarse)
            _phase_begin(
                "APPROACH_COARSE",
                target_world=target_world_coarse,
                target_base=target_base_coarse,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "mode": "joint_preset_coarse",
                },
                joint_goal=[float(v) for v in JOINT_GRASP_DOWN_POSE_RAD],
                note="coarse target computed before GRASP_DOWN_JOINT",
            )
            _phase_end("APPROACH_COARSE", result="ok")
            _phase_begin(
                "GRASP_DOWN_JOINT",
                target_world=target_world_coarse,
                target_base=target_base_coarse,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "mode": "joint_preset",
                },
                joint_goal=[float(v) for v in JOINT_GRASP_DOWN_POSE_RAD],
                note="preset descent to grasp-down joint pose",
            )
            _run_joint_step(
                "GRASP_DOWN_JOINT",
                JOINT_GRASP_DOWN_POSE_RAD,
                timeout_sec=move_sec + 6.0,
                tol_rad=0.08,
            )
            tcp_after_joint = _live_tcp_base()
            obj_after_joint = _live_object_base()
            if tcp_after_joint is not None and obj_after_joint is not None:
                panel._emit_log(
                    "[PICK][DIRECT][PHASE] "
                    f"phase=GRASP_DOWN_JOINT frame=base_link "
                    f"tcp=({tcp_after_joint[0]:.3f},{tcp_after_joint[1]:.3f},{tcp_after_joint[2]:.3f}) "
                    f"obj=({obj_after_joint[0]:.3f},{obj_after_joint[1]:.3f},{obj_after_joint[2]:.3f}) "
                    f"tcp_obj_dist={_dist(tcp_after_joint, obj_after_joint):.3f}"
                )
            _phase_end("GRASP_DOWN_JOINT", result="ok")
            obj_base_align = _live_object_base()
            target_base_align = None
            target_world_align = None
            if obj_base_align is not None:
                target_base_align = (
                    float(obj_base_align[0]),
                    float(obj_base_align[1]),
                    float(obj_base_align[2]) + float(GRIPPER_TCP_Z_OFFSET),
                )
                target_world_align = _target_world_from_base(target_base_align)
            _phase_begin(
                "GRASP_ALIGN_IK",
                target_world=target_world_align,
                target_base=target_base_align,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "ik_mode": "direct_rg2_tcp_to_tool0_numeric",
                },
                note="fine alignment IK over live object pose",
            )
            align_debug = _align_demo_grasp_direct()
            _phase_end(
                "GRASP_ALIGN_IK",
                ik_solution=align_debug.get("ik_solution") if isinstance(align_debug, dict) else None,
                note="ik alignment completed",
                result="ok",
            )
            extra_down_m = max(
                0.0,
                float(os.environ.get("PANEL_PICK_DEMO_EXTRA_GRASP_DOWN_M", "0.0") or 0.0),
            )
            if extra_down_m > 1e-4:
                try:
                    tcp_pose = panel.get_tcp_base()
                except Exception:
                    tcp_pose = None
                if tcp_pose is not None:
                    base_frame = str(getattr(tcp_pose.header, "frame_id", "") or panel._business_base_frame())
                    tcp_pos = tcp_pose.pose.position
                    tcp_ori = tcp_pose.pose.orientation
                    target_z = float(tcp_pos.z) - float(extra_down_m)
                    request_id = int(getattr(panel, "_panel_moveit_request_id", 0) or 0) + 1
                    setattr(panel, "_panel_moveit_request_id", request_id)
                    request_uuid = uuid.uuid4().hex
                    pose_data = {
                        "position": (float(tcp_pos.x), float(tcp_pos.y), float(target_z)),
                        "orientation": (
                            float(tcp_ori.x),
                            float(tcp_ori.y),
                            float(tcp_ori.z),
                            float(tcp_ori.w),
                        ),
                        "frame": f"{base_frame}|rid={request_id}|uid={request_uuid}",
                    }
                    has_results = False
                    if panel._ros_worker_started and panel.ros_worker and panel.ros_worker.node_ready():
                        try:
                            has_results = bool(
                                panel.ros_worker.subscribe_moveit_result("/desired_grasp/result")
                            )
                        except Exception:
                            has_results = False
                    panel._emit_log(
                        f"[DEMO] GRASP_DOWN extra cartesian {extra_down_m:.3f} m "
                        f"target_z={target_z:.3f} frame={base_frame}"
                    )
                    since_wall = 0.0
                    since_seq = -1
                    if has_results and panel.ros_worker:
                        _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
                    if not panel._publish_moveit_pose("GRASP_DOWN_EXTRA", pose_data, cartesian=True):
                        raise RuntimeError("GRASP_DOWN_EXTRA publish_failed")
                    if has_results:
                        ok_extra, msg_extra = panel._wait_tfm_moveit_result(
                            "GRASP_DOWN_EXTRA",
                            since_wall=since_wall,
                            since_seq=since_seq,
                            timeout_sec=move_sec + 8.0,
                            expected_request_id=request_id,
                            expected_request_uuid=request_uuid,
                        )
                        if not ok_extra:
                            raise RuntimeError(f"GRASP_DOWN_EXTRA result_failed:{msg_extra}")
                    else:
                        time.sleep(0.8)
                        panel._motion_in_progress = False
                    if not panel._wait_for_tcp_base_z(target_z, timeout_sec=4.0, tol_m=0.015):
                        panel._emit_log(
                            f"[DEMO] WARN: GRASP_DOWN_EXTRA tcp_z no confirmado target_z={target_z:.3f}"
                        )
                else:
                    panel._emit_log("[DEMO] WARN: tcp_base no disponible; omitiendo extra_down")
            else:
                panel._emit_log("[PICK][DIRECT] moveit_extra_down=disabled route=direct_joint_only")
            time.sleep(3.0)
            initial_obj_world = _live_object_world()
            if initial_obj_world is None:
                raise RuntimeError("demo_object_pose_unavailable_before_close")
            obj_base_pre = _live_object_base()
            target_base_pre = None
            target_world_pre = None
            if obj_base_pre is not None:
                target_base_pre = (
                    float(obj_base_pre[0]),
                    float(obj_base_pre[1]),
                    float(obj_base_pre[2]) + float(GRIPPER_TCP_Z_OFFSET),
                )
                target_world_pre = _target_world_from_base(target_base_pre)
            _phase_begin(
                "PRE_CLOSE",
                target_world=target_world_pre,
                target_base=target_base_pre,
                frame_used="base_link",
                offsets={"tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET)},
                note="pre-close diagnostic snapshot",
            )
            tcp_base_pre_close = _live_tcp_base()
            obj_base_pre_close = _live_object_base()
            if tcp_base_pre_close is not None and obj_base_pre_close is not None:
                panel._emit_log(
                    "[PICK][DIRECT][PHASE] "
                    f"phase=PRE_CLOSE frame=base_link "
                    f"tcp=({tcp_base_pre_close[0]:.3f},{tcp_base_pre_close[1]:.3f},{tcp_base_pre_close[2]:.3f}) "
                    f"obj=({obj_base_pre_close[0]:.3f},{obj_base_pre_close[1]:.3f},{obj_base_pre_close[2]:.3f}) "
                    f"tcp_obj_dist={_dist(tcp_base_pre_close, obj_base_pre_close):.3f}"
                )
            _phase_end("PRE_CLOSE", result="ok")
            panel._emit_log("[DEMO] Cerrando pinza")
            def _close_only():
                panel._command_gripper(True, log_action="PICK", force=True)

            _phase_begin(
                "CLOSE",
                target_world=target_world_pre,
                target_base=target_base_pre,
                frame_used="base_link",
                offsets={"tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET)},
                note="closing gripper over target",
            )
            panel.signal_run_ui.emit(_close_only)
            time.sleep(0.1)
            close_confirmed, close_wait_state = _wait_for_gripper_target(
                True,
                timeout_sec=max(
                    0.8,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC",
                            "1.8",
                        )
                        or 1.8
                    ),
                ),
            )
            close_metrics = _close_alignment_metrics()
            close_metrics["close_confirmed"] = bool(close_confirmed)
            close_metrics["close_wait_state"] = _json_safe(close_wait_state)
            _phase_end(
                "CLOSE",
                attach_state=_read_attach_state(),
                note=json.dumps(_json_safe(close_metrics), ensure_ascii=False, sort_keys=True),
                result="ok" if bool(close_confirmed) and bool(close_metrics.get("ok")) else "failed",
            )
            if not bool(close_confirmed):
                _abort_grasp(
                    code="CLOSE_NOT_CONFIRMED",
                    phase="CLOSE",
                    note="gripper close command did not reach measured closed state before attach",
                    metrics=close_metrics,
                )
            if not bool(close_metrics.get("ok")):
                _abort_grasp(
                    code="CLOSE_WITHOUT_OBJECT",
                    phase="CLOSE",
                    note="gripper closed but object not geometrically acquired",
                    metrics=close_metrics,
                )
            tcp_base_grasp = _live_tcp_base()
            obj_base_grasp = _live_object_base()
            if tcp_base_grasp is None or obj_base_grasp is None:
                panel._emit_log(
                    "[PICK][DIRECT][GEOM] "
                    f"tcp_base={'ok' if tcp_base_grasp is not None else 'none'} "
                    f"obj_base={'ok' if obj_base_grasp is not None else 'none'}"
                )
                raise RuntimeError("demo_attach_geometry_unavailable")
            target_base_attach = (
                float(obj_base_grasp[0]),
                float(obj_base_grasp[1]),
                float(obj_base_grasp[2]) + float(GRIPPER_TCP_Z_OFFSET),
            )
            target_world_attach = _target_world_from_base(target_base_attach)
            _phase_begin(
                "ATTACH_GATE",
                target_world=target_world_attach,
                target_base=target_base_attach,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "attach_xy_tol_m": max(0.02, float(os.environ.get("PANEL_PICK_DEMO_ATTACH_XY_TOL_M", "0.080") or 0.080)),
                    "attach_z_tol_m": max(0.02, float(os.environ.get("PANEL_PICK_DEMO_ATTACH_Z_TOL_M", "0.080") or 0.080)),
                },
                note="attach geometry gate and follow confirmation",
            )
            panel._emit_log(
                "[PICK][DIRECT][PHASE] "
                f"phase=ATTACH_GATE frame=base_link "
                f"tcp=({tcp_base_grasp[0]:.3f},{tcp_base_grasp[1]:.3f},{tcp_base_grasp[2]:.3f}) "
                f"obj=({obj_base_grasp[0]:.3f},{obj_base_grasp[1]:.3f},{obj_base_grasp[2]:.3f}) "
                f"tcp_obj_dist={_dist(tcp_base_grasp, obj_base_grasp):.3f}"
            )
            attach_ok = panel._attempt_attach(
                "demo_grasp_physical",
                selected_name=PICK_DEMO_OBJECT_NAME,
                tcp_base=tcp_base_grasp,
                object_base=obj_base_grasp,
                base_frame=str(panel._business_base_frame() or BASE_FRAME or "base_link"),
                xy_tol_m=max(0.02, float(os.environ.get("PANEL_PICK_DEMO_ATTACH_XY_TOL_M", "0.080") or 0.080)),
                z_tol_m=max(0.02, float(os.environ.get("PANEL_PICK_DEMO_ATTACH_Z_TOL_M", "0.080") or 0.080)),
                z_ref_mode="center",
            )
            if not attach_ok:
                _phase_end("ATTACH_GATE", attach_state=_read_attach_state(), result="failed", note="attach gate returned false")
                raise RuntimeError("demo_attach_failed")
            demo_attach_published = True

            _wait_demo_attach_follow(
                timeout_sec=max(
                    1.2,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_ATTACH_SETTLE_SEC",
                            "1.8",
                        )
                        or 1.8
                    ),
                ),
                max_tcp_dist_m=max(
                    0.12,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M",
                            "0.160",
                        )
                        or 0.160
                    ),
                ),
            )
            demo_follow_confirmed = True
            attach_metrics = _close_alignment_metrics()
            _phase_end(
                "ATTACH_GATE",
                attach_state=_read_attach_state(),
                note=json.dumps(_json_safe(attach_metrics), ensure_ascii=False, sort_keys=True),
                result="ok" if bool(attach_metrics.get("ok")) and demo_attach_published and demo_follow_confirmed else "failed",
            )
            if not (bool(attach_metrics.get("ok")) and demo_attach_published and demo_follow_confirmed):
                _abort_grasp(
                    code="GRASP_NOT_ACQUIRED",
                    phase="ATTACH_GATE",
                    note="attach/follow did not confirm acquisition before lift",
                    metrics=attach_metrics,
                )
            _phase_begin(
                "LIFT",
                target_world=None,
                target_base=None,
                frame_used="base_link",
                offsets={
                    "lift_m": max(
                        0.04,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_SHORT_LIFT_M",
                                "0.120",
                            )
                            or 0.120
                        ),
                    )
                },
                note="post-grasp short lift",
            )
            lift_debug = _lift_demo_object_direct(
                max(
                    0.04,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_SHORT_LIFT_M",
                            "0.120",
                        )
                        or 0.120
                    ),
                )
            )
            _phase_end(
                "LIFT",
                target_base=lift_debug.get("target_tcp_runtime") if isinstance(lift_debug, dict) else None,
                target_world=_target_world_from_base(lift_debug.get("target_tcp_runtime")) if isinstance(lift_debug, dict) else None,
                ik_solution=lift_debug.get("ik_solution") if isinstance(lift_debug, dict) else None,
                attach_state=_read_attach_state(),
                note="short lift executed",
                result="ok",
            )
            _phase_begin(
                "CARRY",
                frame_used="world",
                offsets={
                    "min_obj_move_m": 0.030,
                    "min_lift_delta_m": 0.025,
                    "max_tcp_dist_m": 0.160,
                },
                note="carry validation after lift",
            )
            try:
                _validate_demo_carry(
                    initial_obj_world=initial_obj_world,
                    phase="post_grasp_lift",
                    timeout_sec=1.6,
                    min_obj_move_m=0.030,
                    min_lift_delta_m=0.025,
                    max_tcp_dist_m=0.160,
                )
                _phase_end("CARRY", attach_state=_read_attach_state(), result="ok")
            except Exception as carry_exc:
                _phase_end("CARRY", attach_state=_read_attach_state(), result="failed", note=str(carry_exc))
                _grasp_failure_analysis(
                    code="CARRY_NOT_ACQUIRED",
                    phase="CARRY",
                    note=str(carry_exc),
                    metrics={
                        "initial_obj_world": initial_obj_world,
                        "last_target_world": last_target.get("target_world"),
                    },
                )
                raise
            if not mark_object_grasped(PICK_DEMO_OBJECT_NAME, reason="demo_physical_lift_ok"):
                raise RuntimeError("demo_mark_grasped_failed")
            if not mark_object_attached(PICK_DEMO_OBJECT_NAME, reason="demo_physical_lift_ok"):
                raise RuntimeError("demo_mark_attached_failed")
            demo_logical_attached = True
            short_release_mode = str(
                os.environ.get("PANEL_PICK_DEMO_SHORT_RELEASE_ONLY", "1") or "1"
            ).strip().lower() not in {"0", "false", "no", "off"}
            if short_release_mode:
                panel._emit_log("[PICK][DEMO] short_release_mode=true")

                def _open_and_release_short():
                    panel._command_gripper(False, log_action="DROP", force=True)
                    mark_object_released(PICK_DEMO_OBJECT_NAME, reason="demo_short_release")

                _phase_begin(
                    "RELEASE",
                    frame_used="lift_pose",
                    offsets={
                        "drop_reason": "demo_short_release",
                        "mode": "short_lift_release",
                    },
                    note="open gripper and detach after validated short lift",
                )
                panel.signal_run_ui.emit(_open_and_release_short)
                time.sleep(0.4)
                _detach_demo_object("demo_short_release")
                demo_logical_attached = False
                time.sleep(0.8)
                _phase_end("RELEASE", attach_state=_read_attach_state(), result="ok")
                try:
                    _run_joint_step("HOME_FINAL", home_pose)
                except Exception as home_final_exc:
                    panel._emit_log(
                        f"[PICK][DEMO] HOME_FINAL warning after short_release: {home_final_exc}"
                    )
                panel._ui_set_status("Pick demo completado (lift + release)", error=False)
                panel._emit_log("[PICK][DEMO] Secuencia completada en modo short_release")
                return

            _run_joint_step(
                "HOME_WITH_OBJECT",
                home_pose,
                timeout_sec=move_sec + 8.0,
                tol_rad=0.10,
            )
            _validate_demo_carry(
                initial_obj_world=initial_obj_world,
                phase="home_with_object",
                timeout_sec=1.2,
                min_obj_move_m=0.080,
                min_lift_delta_m=0.060,
                max_tcp_dist_m=0.200,
            )
            _run_joint_step(
                "CESTA",
                JOINT_BASKET_POSE_RAD,
                timeout_sec=move_sec + 10.0,
                tol_rad=0.12,
            )
            _run_joint_step(
                "CESTA_RELEASE",
                JOINT_BASKET_DEMO_RELEASE_POSE_RAD,
                timeout_sec=move_sec + 8.0,
                tol_rad=0.08,
            )

            panel._emit_log("[DEMO] Abriendo pinza en cesta")
            def _open_and_release():
                panel._command_gripper(False, log_action="DROP", force=True)
                mark_object_released(PICK_DEMO_OBJECT_NAME, reason="demo_drop")

            _phase_begin(
                "RELEASE",
                frame_used="basket",
                offsets={"drop_reason": "demo_drop"},
                note="open gripper and logical release in basket",
            )
            panel.signal_run_ui.emit(_open_and_release)
            time.sleep(1.0)
            panel._emit_log("[DEMO] Cerrando pinza en cesta")
            panel.signal_run_ui.emit(lambda: panel._command_gripper(True, log_action="DROP", force=True))
            time.sleep(0.4)
            _phase_end("RELEASE", attach_state=_read_attach_state(), result="ok")

            _run_joint_step("HOME_FINAL", home_pose)

            panel._ui_set_status("Pick demo: verificando entrega en cesta…")
            panel._emit_log("[PICK] Secuencia PICK completada; validando entrega física.")
            
            # Marcar como exitoso y diferir confirmación de cesta para evitar contenciones del executor
            panel._pick_demo_executed = True
            panel._emit_log("[PICK][DEMO] Deferiendo confirmación de cesta...")
            
            def _deferred_basket_check():
                """Ejecuta verificación de cesta después de dar tiempo al executor"""
                time.sleep(1.0)  # Dar tiempo para que el executor se libere
                if _demo_object_in_basket(panel):
                    def _lock_pick_demo_button() -> None:
                        panel._pick_demo_executed = True
                        panel.btn_pick_demo.setEnabled(False)
                        panel.btn_pick_demo.setToolTip("Ya ejecutado: objeto demo confirmado en cesta")
                        panel._ui_set_status("Pick demo completado", error=False)
                        panel._emit_log("[PICK][DEMO] boton deshabilitado (objeto confirmado en cesta)")

                    panel.signal_run_ui.emit(_lock_pick_demo_button)
                else:
                    panel._emit_log("[PICK][DEMO] Cesta no confirmada pero secuencia completada")
                    def _disable_button_anyway() -> None:
                        panel.btn_pick_demo.setEnabled(False)
                        panel.btn_pick_demo.setToolTip("Secuencia completada (objeto en cesta no confirmado visualmente)")
                        panel._ui_set_status("Pick demo fallido: cesta no confirmada", error=True)
                    panel.signal_run_ui.emit(_disable_button_anyway)
            
            # Ejecutar verificación en thread separado para no bloquear
            panel._pick_demo_checker_thread = panel._run_async(_deferred_basket_check)
            
        except Exception as exc:
            active_phase = (current_phase.get("data") or {}).get("phase")
            if active_phase:
                try:
                    _phase_end(
                        active_phase,
                        attach_state=_read_attach_state(),
                        result="exception",
                        note=str(exc),
                    )
                except Exception:
                    pass
            if demo_attach_published:
                panel.signal_run_ui.emit(lambda: panel._command_gripper(False, log_action="PICK_FAIL", force=True))
                time.sleep(0.3)
                _detach_demo_object("error_recovery")
                if demo_logical_attached:
                    try:
                        mark_object_released(PICK_DEMO_OBJECT_NAME, reason="demo_error_recovery")
                    except Exception:
                        pass
            try:
                panel._emit_log("[PICK][RECOVERY] Error detectado; intentando HOME_SAFE")
                _run_joint_step("HOME_SAFE", home_pose, timeout_sec=move_sec + 3.0, tol_rad=0.08)
            except Exception as home_exc:
                panel._emit_log(f"[PICK][RECOVERY] HOME_SAFE falló: {home_exc}")
            panel._ui_set_status(f"Error en pick demo: {exc}", error=True)
            panel._emit_log(f"[PICK] ✗ Error: {exc}")
            # Marcar como ejecutado sin confirmación si falló
            panel._pick_demo_executed = False
        finally:
            panel._set_motion_lock(False)

    panel._run_async(worker)
