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
from .panel_utils import (
    transform_point_to_frame,
    world_to_base,
    world_xyz_to_pixel_float,
    table_xy_to_pixel_float,
)
from .ur5_kinematics import fk_ur5, ik_ur5

DIRECT_ROUTE_MODE = "direct_rg2_tcp"
DIRECT_SOURCE_FRAME = "rg2_tcp"
DIRECT_EXECUTION_FRAME = "tool0"
DIRECT_EXECUTION_IK_MODE = "formal_rg2_tcp_source_to_tool0_numeric"
DIRECT_TOOL0_TO_RG2_TCP_Z_M = 0.175


def _demo_object_in_basket(panel, timeout_sec: float = 4.0) -> bool:
    """Confirma por posicion que el objeto demo esta en la cesta."""
    start = time.monotonic()
    basket_world = tuple(float(v) for v in BASKET_DROP)
    world_frame = str(
        getattr(panel, "_world_frame_last_first", lambda fallback=None: WORLD_FRAME or "world")(
            WORLD_FRAME or "world"
        )
    ).strip() or "world"
    base_frame = str(getattr(panel, "_base_frame_effective", "") or BASE_FRAME)
    basket_base, _ = transform_point_to_frame(
        basket_world,
        base_frame,
        source_frame=world_frame,
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
                    source_frame=world_frame,
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

            def _fmt_scalar(value, *, digits: int = 3) -> str:
                try:
                    return f"{float(value):.{digits}f}"
                except Exception:
                    return "none"

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
                if label in {"HOME", "MESA", "PICK_IMAGE", "HOME_WITH_OBJECT", "CESTA", "CESTA_RELEASE", "HOME_FINAL"}:
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
                world_frame = str(
                    getattr(panel, "_world_frame_last_first", lambda fallback=None: WORLD_FRAME or "world")(
                        WORLD_FRAME or "world"
                    )
                ).strip() or "world"
                try:
                    base_frame = str(panel._business_base_frame() or BASE_FRAME or "base_link")
                except Exception:
                    base_frame = str(BASE_FRAME or "base_link")
                obj_base, _ = transform_point_to_frame(
                    world_pos,
                    base_frame,
                    source_frame=world_frame,
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

            def _read_gripper_state(*, expected_closed: Optional[bool] = None):
                joint_snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
                positions = {}
                for joint_name in GRIPPER_JOINT_NAMES:
                    if joint_name in joint_snapshot:
                        positions[joint_name] = float(joint_snapshot[joint_name])
                expected_closed_flag = (
                    bool(getattr(panel, "_gripper_closed", False))
                    if expected_closed is None
                    else bool(expected_closed)
                )
                target_mag = abs(
                    float(GRIPPER_CLOSED_RAD if expected_closed_flag else GRIPPER_OPEN_RAD)
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
                    "expected_closed": bool(expected_closed_flag),
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
                opening_ref_sum: Optional[float] = None,
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
                close_min_delta_sum = max(
                    0.02,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM",
                            "0.08",
                        )
                        or 0.08
                    ),
                )
                close_fallback_opening_sum = max(
                    0.02,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_CLOSE_FALLBACK_OPENING_SUM",
                            "0.40",
                        )
                        or 0.40
                    ),
                )
                start = time.monotonic()
                stable_hits = 0
                last_state = _read_gripper_state(expected_closed=closed)
                best_close_delta = float("-inf")
                last_debug_log_ts = 0.0
                _append_trace(
                    "[PICK][DIRECT][GRIPPER] "
                    f"wait_start target={'closed' if closed else 'open'} "
                    f"target_mag={last_state.get('target_mag')} timeout={timeout_sec:.2f}s stable_hits={required_hits} "
                    f"opening_ref_sum={opening_ref_sum}"
                )
                while (time.monotonic() - start) <= timeout_sec:
                    _monitor_alcance(trigger=f"GRIPPER_WAIT_{'CLOSE' if closed else 'OPEN'}")
                    state = _read_gripper_state(expected_closed=closed)
                    last_state = state
                    measured_ok = bool(state.get("measured_target_ok"))
                    age_ok = (
                        state.get("joint_state_age_sec") is None
                        or float(state.get("joint_state_age_sec")) <= max_state_age_sec
                    )
                    opening_sum = state.get("opening_sum")
                    close_delta = None
                    if (
                        opening_ref_sum is not None
                        and opening_sum is not None
                    ):
                        close_delta = float(opening_ref_sum) - float(opening_sum)
                        best_close_delta = max(best_close_delta, float(close_delta))
                    close_heuristic_ok = False
                    if closed:
                        if (
                            age_ok
                            and bool(state.get("closed_flag"))
                            and opening_sum is not None
                        ):
                            delta_ok = (
                                close_delta is not None
                                and float(close_delta) >= float(close_min_delta_sum)
                            )
                            fallback_ok = (
                                opening_ref_sum is None
                                and float(opening_sum) <= float(close_fallback_opening_sum)
                            )
                            close_heuristic_ok = bool(delta_ok or fallback_ok)
                    confirm_mode = "none"
                    confirm_ok = False
                    if measured_ok and age_ok:
                        confirm_ok = True
                        confirm_mode = "measured_target_ok"
                    elif close_heuristic_ok:
                        confirm_ok = True
                        confirm_mode = "closing_delta_ok"
                    now_ts = time.monotonic()
                    if now_ts >= (last_debug_log_ts + 0.20):
                        _append_trace(
                            "[PICK][DIRECT][GRIPPER] "
                            f"wait_sample target={'closed' if closed else 'open'} "
                            f"closed_flag={bool(state.get('closed_flag'))} measured_ok={measured_ok} age_ok={age_ok} "
                            f"opening_sum={state.get('opening_sum')} max_abs_err={state.get('max_abs_err')} "
                            f"close_delta={close_delta} min_delta={close_min_delta_sum} mode={confirm_mode}"
                        )
                        last_debug_log_ts = now_ts
                    if confirm_ok:
                        stable_hits += 1
                        if stable_hits >= required_hits:
                            done_state = dict(state)
                            done_state["confirm_mode"] = confirm_mode
                            done_state["close_delta_from_ref"] = close_delta
                            done_state["close_delta_best"] = (
                                None
                                if best_close_delta == float("-inf")
                                else float(best_close_delta)
                            )
                            _append_trace(
                                "[PICK][DIRECT][GRIPPER] "
                                f"wait_ok target={'closed' if closed else 'open'} "
                                f"opening_sum={state.get('opening_sum')} max_abs_err={state.get('max_abs_err')} "
                                f"age={state.get('joint_state_age_sec')} mode={confirm_mode} "
                                f"close_delta={close_delta}"
                            )
                            return True, done_state
                    else:
                        stable_hits = 0
                    time.sleep(0.05)
                timeout_state = dict(last_state or {})
                timeout_state["confirm_mode"] = "timeout"
                timeout_state["close_delta_best"] = (
                    None if best_close_delta == float("-inf") else float(best_close_delta)
                )
                _append_trace(
                    "[PICK][DIRECT][GRIPPER] "
                    f"wait_timeout target={'closed' if closed else 'open'} "
                    f"opening_sum={last_state.get('opening_sum')} max_abs_err={last_state.get('max_abs_err')} "
                    f"age={last_state.get('joint_state_age_sec')} "
                    f"closed_flag={bool(last_state.get('closed_flag'))} "
                    f"measured_ok={bool(last_state.get('measured_target_ok'))} "
                    f"opening_ref_sum={opening_ref_sum} "
                    f"close_delta_best={timeout_state.get('close_delta_best')}"
                )
                return False, timeout_state

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

            def _release_observation(*, tag: str, reason: str = ""):
                tcp_world = _tuple3(_live_tcp_world())
                obj_world = _tuple3(_live_object_world())
                tcp_base = _tuple3(_live_tcp_base())
                obj_base = _tuple3(_live_object_base())
                dist_world = _dist(tcp_world, obj_world) if tcp_world is not None and obj_world is not None else None
                dist_base = _dist(tcp_base, obj_base) if tcp_base is not None and obj_base is not None else None
                attach_state = _json_safe(_read_attach_state()) or {}
                gripper_open_state = _json_safe(_read_gripper_state(expected_closed=False)) or {}
                panel._emit_log(
                    "[PICK][DIRECT][RELEASE] "
                    f"tag={tag} reason={reason or 'none'} "
                    f"tcp_world={_fmt_vec(tcp_world)} obj_world={_fmt_vec(obj_world)} "
                    f"tcp_base={_fmt_vec(tcp_base)} obj_base={_fmt_vec(obj_base)} "
                    f"dist_world={_fmt_scalar(dist_world)} dist_base={_fmt_scalar(dist_base)} "
                    f"attach_state={json.dumps(attach_state, ensure_ascii=False, sort_keys=True)} "
                    f"gripper_open_ok={gripper_open_state.get('measured_target_ok')} "
                    f"gripper_opening_sum={_fmt_scalar(gripper_open_state.get('opening_sum'))} "
                    f"gripper_open_err={_fmt_scalar(gripper_open_state.get('max_abs_err'))}"
                )
                return {
                    "tcp_world": tcp_world,
                    "obj_world": obj_world,
                    "tcp_base": tcp_base,
                    "obj_base": obj_base,
                    "dist_world": dist_world,
                    "dist_base": dist_base,
                    "attach_state": attach_state,
                    "gripper_open_state": gripper_open_state,
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
                gripper_state = _read_gripper_state(expected_closed=True)
                gripper_closed_measured = bool(gripper_state.get("measured_target_ok"))
                geometry_ok = bool(xy_dist <= xy_tol and z_error <= z_tol)
                return {
                    "ok": geometry_ok,
                    "geometry_ok": geometry_ok,
                    "reason": "ok" if geometry_ok else "alignment_out_of_tolerance",
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

            def _pre_close_alignment_metrics():
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
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M",
                            os.environ.get("PANEL_PICK_DEMO_CLOSE_XY_TOL_M", "0.035"),
                        )
                        or 0.035
                    ),
                )
                z_tol = max(
                    0.01,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M",
                            os.environ.get("PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M", "0.030"),
                        )
                        or 0.030
                    ),
                )
                ok = xy_dist <= xy_tol and z_error <= z_tol
                return {
                    "ok": ok,
                    "reason": "ok" if ok else "alignment_out_of_tolerance",
                    "xy_dist": float(xy_dist),
                    "z_gap": float(z_gap),
                    "z_error": float(z_error),
                    "tcp_obj_dist": float(tcp_obj_dist),
                    "xy_tol": float(xy_tol),
                    "z_tol": float(z_tol),
                    "tcp_base": _tuple3(tcp_base),
                    "object_base": _tuple3(obj_base),
                    "gripper_state": _json_safe(_read_gripper_state()),
                }

            def _wait_pre_close_alignment(*, timeout_sec: float, min_consecutive: int = 3):
                deadline = time.time() + max(0.3, float(timeout_sec))
                consecutive_ok = 0
                best_xy_dist = None
                best_z_error = None
                last_metrics = None
                while time.time() < deadline:
                    _monitor_alcance(trigger="PRE_CLOSE_WAIT")
                    metrics = _pre_close_alignment_metrics()
                    last_metrics = metrics
                    xy_dist = metrics.get("xy_dist")
                    z_error = metrics.get("z_error")
                    if xy_dist is not None:
                        best_xy_dist = (
                            float(xy_dist)
                            if best_xy_dist is None
                            else min(float(best_xy_dist), float(xy_dist))
                        )
                    if z_error is not None:
                        best_z_error = (
                            float(z_error)
                            if best_z_error is None
                            else min(float(best_z_error), float(z_error))
                        )
                    if bool(metrics.get("ok")):
                        consecutive_ok += 1
                        if consecutive_ok >= max(1, int(min_consecutive)):
                            done_metrics = dict(metrics)
                            done_metrics["consecutive_ok"] = int(consecutive_ok)
                            done_metrics["best_xy_dist"] = best_xy_dist
                            done_metrics["best_z_error"] = best_z_error
                            return True, done_metrics
                    else:
                        consecutive_ok = 0
                    time.sleep(0.06)
                done_metrics = dict(last_metrics or {})
                done_metrics["consecutive_ok"] = int(consecutive_ok)
                done_metrics["best_xy_dist"] = best_xy_dist
                done_metrics["best_z_error"] = best_z_error
                return False, done_metrics

            def _emit_transition_decision(
                *,
                from_phase: str,
                to_phase: str,
                decision: str,
                reason: str,
                condition: str | None = None,
                metrics=None,
            ) -> None:
                metric_dict = _json_safe(metrics) or {}
                tcp_base = _tuple3(metric_dict.get("tcp_base")) or _tuple3(_live_tcp_base())
                obj_base = _tuple3(metric_dict.get("object_base")) or _tuple3(_live_object_base())
                tcp_obj_dist = metric_dict.get("tcp_obj_dist")
                if tcp_obj_dist is None and tcp_base is not None and obj_base is not None:
                    tcp_obj_dist = _dist(tcp_base, obj_base)
                gripper = _json_safe(_read_gripper_state()) or {}
                attach = _json_safe(_read_attach_state()) or {}
                panel._emit_log(
                    "[PICK][DIRECT][TRANSITION] "
                    f"from={from_phase} to={to_phase} decision={decision} reason={reason} "
                    f"condition={condition or 'none'} "
                    f"tcp={_fmt_vec(tcp_base)} obj={_fmt_vec(obj_base)} "
                    f"tcp_obj_dist={_fmt_scalar(tcp_obj_dist)} "
                    f"xy_dist={_fmt_scalar(metric_dict.get('xy_dist'))} "
                    f"z_error={_fmt_scalar(metric_dict.get('z_error'))} "
                    f"z_gap={_fmt_scalar(metric_dict.get('z_gap'))} "
                    f"xy_tol={_fmt_scalar(metric_dict.get('xy_tol'))} "
                    f"z_tol={_fmt_scalar(metric_dict.get('z_tol'))} "
                    f"gripper_closed={bool(gripper.get('closed_flag'))} "
                    f"gripper_measured={bool(gripper.get('measured_target_ok'))} "
                    f"opening_sum={_fmt_scalar(gripper.get('opening_sum'))} "
                    f"max_abs_err={_fmt_scalar(gripper.get('max_abs_err'))} "
                    f"attach_logical={attach.get('logical_attached')} "
                    f"attach_state={attach.get('logical_state')} "
                    f"attach_owner={attach.get('owner')}"
                )

            def _trace_phase_pose(
                *,
                phase: str,
                event: str,
                target_base=None,
                frame_used: str = "base_link",
                offsets=None,
                decision: str = "",
                preset_used: str | None = None,
                execution_type: str | None = None,
            ) -> None:
                obj_base = _tuple3(_live_object_base())
                tcp_base = _tuple3(_live_tcp_base())
                dx = dy = dz = dist = xy_dist = None
                if obj_base is not None and tcp_base is not None:
                    dx = float(tcp_base[0]) - float(obj_base[0])
                    dy = float(tcp_base[1]) - float(obj_base[1])
                    dz = float(tcp_base[2]) - float(obj_base[2])
                    dist = _dist(tcp_base, obj_base)
                    xy_dist = math.hypot(float(dx), float(dy))
                msg = (
                    "[PICK][DIRECT][POSE_TRACE] "
                    f"phase={phase} event={event} frame={frame_used} "
                    f"obj_base={_fmt_vec(obj_base)} tcp_base={_fmt_vec(tcp_base)} "
                    f"dx={_fmt_scalar(dx)} dy={_fmt_scalar(dy)} dz={_fmt_scalar(dz)} "
                    f"dist={_fmt_scalar(dist)} xy_dist={_fmt_scalar(xy_dist)} "
                    f"target_base={_fmt_vec(target_base)} "
                    f"offsets={json.dumps(_json_safe(offsets) or {}, ensure_ascii=False, sort_keys=True)} "
                    f"preset={preset_used or 'none'} "
                    f"execution_type={execution_type or 'none'} "
                    f"decision={decision or 'none'}"
                )
                panel._emit_log(msg)
                _append_trace(msg)
                _monitor_alcance(trigger=f"{phase}:{event}")

            def _execution_type_from_decision(decision: str | None) -> str:
                decision_txt = str(decision or "").strip().lower()
                if "fallback_joint_preset" in decision_txt or "target_unavailable" in decision_txt:
                    return "preset"
                if decision_txt in {"direct_ik_move", "direct_ik_move_refresh"}:
                    return "geometrico"
                if decision_txt:
                    return "hibrido"
                return "hibrido"

            def _phase_tcp_obj_metrics_base() -> dict:
                obj_base = _tuple3(_live_object_base())
                tcp_base = _tuple3(_live_tcp_base())
                if obj_base is None or tcp_base is None:
                    return {
                        "ok": False,
                        "obj_base": obj_base,
                        "tcp_base": tcp_base,
                        "dx": None,
                        "dy": None,
                        "dz": None,
                        "dist": None,
                        "xy_dist": None,
                        "z_error": None,
                        "tcp_obj_dist": None,
                    }
                dx = float(tcp_base[0]) - float(obj_base[0])
                dy = float(tcp_base[1]) - float(obj_base[1])
                dz = float(tcp_base[2]) - float(obj_base[2])
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                xy_dist = math.hypot(dx, dy)
                z_error = abs(float(dz) - float(GRIPPER_TCP_Z_OFFSET))
                return {
                    "ok": True,
                    "obj_base": obj_base,
                    "tcp_base": tcp_base,
                    "dx": dx,
                    "dy": dy,
                    "dz": dz,
                    "dist": dist,
                    "xy_dist": xy_dist,
                    "z_error": z_error,
                    "tcp_obj_dist": dist,
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

            visual_focus_phases = {"GRASP_ALIGN_IK", "PRE_CLOSE", "ATTACH_GATE"}

            def _camera_frame_size() -> tuple[int, int]:
                view = getattr(panel, "camera_view", None)
                if view is None:
                    return 0, 0
                try:
                    fw = int(getattr(view, "_img_width", 0) or 0)
                    fh = int(getattr(view, "_img_height", 0) or 0)
                except Exception:
                    fw, fh = 0, 0
                return fw, fh

            def _project_base_to_overhead(base_coords, frame_w: int, frame_h: int):
                base_3 = _tuple3(base_coords)
                if base_3 is None or frame_w <= 0 or frame_h <= 0:
                    return None, None, "none"
                world_3 = _target_world_from_base(base_3)
                if world_3 is None:
                    return None, None, "base_to_world_fail"
                px = world_xyz_to_pixel_float(
                    float(world_3[0]),
                    float(world_3[1]),
                    float(world_3[2]),
                    frame_w,
                    frame_h,
                )
                if px is not None:
                    return world_3, (float(px[0]), float(px[1])), "world_xyz"
                px = table_xy_to_pixel_float(
                    float(world_3[0]),
                    float(world_3[1]),
                    frame_w,
                    frame_h,
                )
                if px is not None:
                    return world_3, (float(px[0]), float(px[1])), "table_xy"
                return world_3, None, "projection_fail"

            def _save_visual_snapshot(phase: str, event: str) -> str | None:
                seq = int(phase_seq["value"])
                file_name = f"{seq:02d}_{phase}_{event}_overhead.png"
                out_path = run_snapshots_dir / file_name
                saver = getattr(panel, "_save_overhead_frame_with_overlays", None)
                if callable(saver):
                    try:
                        if saver(str(out_path)):
                            return str(out_path)
                    except Exception:
                        return None
                    return None
                frame = getattr(panel, "_last_camera_frame", None)
                if not frame:
                    return None
                try:
                    qimg, _w, _h, _ts = frame
                    if qimg is not None and qimg.save(str(out_path)):
                        return str(out_path)
                except Exception:
                    return None
                return None

            def _emit_visual_coherence(phase: str, *, event: str) -> None:
                frame_w, frame_h = _camera_frame_size()
                tcp_base = _tuple3(_live_tcp_base())
                obj_base = _tuple3(_live_object_base())
                tcp_world, tcp_px, tcp_src = _project_base_to_overhead(tcp_base, frame_w, frame_h)
                obj_world, obj_px, obj_src = _project_base_to_overhead(obj_base, frame_w, frame_h)
                def _fmt_px(px) -> str:
                    if px is None:
                        return "none"
                    try:
                        return f"({float(px[0]):.1f},{float(px[1]):.1f})"
                    except Exception:
                        return "none"
                px_dist = None
                if tcp_px is not None and obj_px is not None:
                    px_dist = math.hypot(
                        float(tcp_px[0]) - float(obj_px[0]),
                        float(tcp_px[1]) - float(obj_px[1]),
                    )
                snap_path = _save_visual_snapshot(phase, event)
                panel._emit_log(
                    "[PICK][DIRECT][VISUAL_FISICA] "
                    f"phase={phase} event={event} "
                    f"camera_topic={str(getattr(panel, 'camera_topic', '') or 'none')} "
                    f"frame={frame_w}x{frame_h} "
                    f"tcp_base={_fmt_vec(tcp_base)} obj_base={_fmt_vec(obj_base)} "
                    f"tcp_world={_fmt_vec(tcp_world)} obj_world={_fmt_vec(obj_world)} "
                    f"tcp_px={_fmt_px(tcp_px)} obj_px={_fmt_px(obj_px)} "
                    f"tcp_px_src={tcp_src} obj_px_src={obj_src} "
                    f"px_dist={_fmt_scalar(px_dist)} "
                    f"snapshot={snap_path or 'none'}"
                )

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
                if phase in visual_focus_phases:
                    _emit_visual_coherence(phase, event="enter")
                _monitor_alcance(trigger=f"{phase}:enter")

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
                tcp_obj_dx_base = tcp_obj_dy_base = tcp_obj_dz_base = tcp_obj_xy_dist_base = None
                if data.get("tcp_pose_world_before") is not None and data.get("object_pose_world_before") is not None:
                    dist_tcp_obj_before = _dist(data["tcp_pose_world_before"], data["object_pose_world_before"])
                if tcp_world_after is not None and obj_world_after is not None:
                    dist_tcp_obj_after = _dist(tcp_world_after, obj_world_after)
                if tcp_base_after is not None and obj_base_after is not None:
                    tcp_obj_dx_base = float(tcp_base_after[0]) - float(obj_base_after[0])
                    tcp_obj_dy_base = float(tcp_base_after[1]) - float(obj_base_after[1])
                    tcp_obj_dz_base = float(tcp_base_after[2]) - float(obj_base_after[2])
                    tcp_obj_xy_dist_base = math.hypot(float(tcp_obj_dx_base), float(tcp_obj_dy_base))
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
                    "tcp_obj_dx_base": tcp_obj_dx_base,
                    "tcp_obj_dy_base": tcp_obj_dy_base,
                    "tcp_obj_dz_base": tcp_obj_dz_base,
                    "tcp_obj_xy_dist_base": tcp_obj_xy_dist_base,
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
                    f"xy_obj_tcp={payload.get('tcp_obj_xy_dist_base') if payload.get('tcp_obj_xy_dist_base') is not None else float('nan'):.3f} "
                    f"d_target_tcp={payload.get('dist_target_tcp') if payload.get('dist_target_tcp') is not None else float('nan'):.3f} "
                    f"frame={payload.get('frame_usado') or 'none'} "
                    f"gripper_closed={bool(gripper_payload.get('closed_flag'))} "
                    f"gripper_measured={bool(gripper_payload.get('measured_target_ok'))} "
                    f"attach={json.dumps(attach_payload or {}, ensure_ascii=False, sort_keys=True)}"
                )
                if phase in visual_focus_phases:
                    _emit_visual_coherence(phase, event="exit")
                _monitor_alcance(trigger=f"{phase}:exit")
                _append_trace(f"[PICK][DIRECT][DEBUG] EXIT_PHASE {phase}")
                current_phase["data"] = None
                return payload

            def _final_phase_trace(
                phase: str,
                *,
                event: str,
                expected: str | None = None,
                received: str | None = None,
                timeout_sec=None,
                reason: str | None = None,
                logical_state: str | None = None,
                physical_state: str | None = None,
                request_state: str | None = None,
            ) -> None:
                panel._emit_log(
                    "[PICK][DIRECT][FINAL_TRACE] "
                    f"phase={phase} event={event} "
                    f"expected={expected or 'none'} received={received or 'none'} "
                    f"timeout={str(timeout_sec) if timeout_sec is not None else 'none'} "
                    f"request={request_state or 'none'} logical={logical_state or 'none'} "
                    f"physical={physical_state or 'none'} reason={reason or 'none'}"
                )

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

            def _move_tcp_direct(*, label: str, target_tcp_runtime, timeout_sec: float) -> dict:
                def _resolve_direct_execution_target(
                    tcp_target_base,
                    tool_rot,
                ) -> dict:
                    # DIRECT keeps rg2_tcp as the operational grasp semantics.
                    # Numeric UR5 IK still solves in tool0, so the only allowed
                    # conversion lives here as a fixed, traceable transform.
                    env_value = os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M", "")
                    tcp_offset_m = max(
                        0.0,
                        float(env_value or DIRECT_TOOL0_TO_RG2_TCP_Z_M),
                    )
                    offset_vector = (
                        float(tool_rot[0, 2]) * tcp_offset_m,
                        float(tool_rot[1, 2]) * tcp_offset_m,
                        float(tool_rot[2, 2]) * tcp_offset_m,
                    )
                    execution_target_tool0 = (
                        float(tcp_target_base[0]) - float(offset_vector[0]),
                        float(tcp_target_base[1]) - float(offset_vector[1]),
                        float(tcp_target_base[2]) - float(offset_vector[2]),
                    )
                    return {
                        "source_frame": DIRECT_SOURCE_FRAME,
                        "source_pose": _tuple3(tcp_target_base),
                        "execution_frame": DIRECT_EXECUTION_FRAME,
                        "execution_pose": _tuple3(execution_target_tool0),
                        "offset_vector": _tuple3(offset_vector),
                        "offset_m": float(tcp_offset_m),
                        "offset_source": (
                            "env:PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M"
                            if str(env_value).strip()
                            else "default:ur5.urdf.xacro/rg2_tcp_joint"
                        ),
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                    }

                tcp_base = _live_tcp_base()
                if tcp_base is None:
                    raise RuntimeError(f"{label.lower()}_tcp_pose_unavailable")
                seed = _current_joint_seed()
                _seed_pos, target_rot = fk_ur5(seed)
                delta_runtime = (
                    float(target_tcp_runtime[0]) - float(tcp_base[0]),
                    float(target_tcp_runtime[1]) - float(tcp_base[1]),
                    float(target_tcp_runtime[2]) - float(tcp_base[2]),
                )
                execution_semantics = _resolve_direct_execution_target(
                    target_tcp_runtime,
                    target_rot,
                )
                target_ik = execution_semantics["execution_pose"]
                offset_vector = execution_semantics["offset_vector"]
                tcp_offset_m = float(execution_semantics["offset_m"])
                panel._emit_log(
                    "[PICK][DIRECT][FRAME] "
                    f"label={label} route={DIRECT_ROUTE_MODE} "
                    f"source_frame={execution_semantics['source_frame']} "
                    f"execution_frame={execution_semantics['execution_frame']} "
                    f"ik_mode={execution_semantics['ik_mode']} "
                    f"source_pose={_fmt_vec(execution_semantics['source_pose'])} "
                    f"offset_vector={_fmt_vec(offset_vector)} "
                    f"offset_m={tcp_offset_m:.3f} "
                    f"offset_source={execution_semantics['offset_source']} "
                    f"execution_pose={_fmt_vec(target_ik)}"
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
                    f"source_frame={DIRECT_SOURCE_FRAME} execution_frame={DIRECT_EXECUTION_FRAME} "
                    f"offset_vector=({offset_vector[0]:.3f},{offset_vector[1]:.3f},{offset_vector[2]:.3f}) "
                    f"target_ik=({target_ik[0]:.3f},{target_ik[1]:.3f},{target_ik[2]:.3f}) "
                    f"tcp_offset_m={tcp_offset_m:.3f} "
                    f"err_norm={float(err_norm):.4f} success={str(bool(ik_ok)).lower()}"
                )
                if (not ik_ok) or float(err_norm) > 0.035:
                    raise RuntimeError(
                        f"{label.lower()}_ik_failed err_norm={float(err_norm):.4f}"
                    )
                solved_q_list = [float(v) for v in solved_q.tolist()]
                align_joint_tol_rad = max(
                    0.01,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_DIRECT_IK_JOINT_TOL_RAD",
                            "0.03",
                        )
                        or 0.03
                    ),
                )
                _run_joint_step(
                    label,
                    solved_q_list,
                    timeout_sec=max(float(timeout_sec), move_sec + 2.0),
                    tol_rad=align_joint_tol_rad,
                )
                tcp_after = _live_tcp_base()
                runtime_target_ok = None
                runtime_target_dist = None
                runtime_target_pos = None
                runtime_target_tol_m = max(
                    0.01,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M",
                            "0.040",
                        )
                        or 0.040
                    ),
                )
                runtime_target_timeout_sec = max(
                    0.5,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_DIRECT_IK_TCP_TIMEOUT_SEC",
                            "4.0",
                        )
                        or 4.0
                    ),
                )
                try:
                    wait_fn = getattr(panel, "_wait_for_tcp_base_target", None)
                    if callable(wait_fn):
                        runtime_target_ok, runtime_target_pos, runtime_target_dist = wait_fn(
                            target_tcp_runtime,
                            timeout_sec=runtime_target_timeout_sec,
                            tol_xyz_m=runtime_target_tol_m,
                            ee_frame="rg2_tcp",
                        )
                except Exception:
                    runtime_target_ok = None
                q_after = _current_joint_seed()
                fk_after_pos, _fk_after_rot = fk_ur5(q_after)
                model_target_err = math.sqrt(
                    (float(fk_after_pos[0]) - float(target_ik[0])) ** 2
                    + (float(fk_after_pos[1]) - float(target_ik[1])) ** 2
                    + (float(fk_after_pos[2]) - float(target_ik[2])) ** 2
                )
                model_live_delta = math.sqrt(
                    (float(fk_after_pos[0]) - float(_seed_pos[0])) ** 2
                    + (float(fk_after_pos[1]) - float(_seed_pos[1])) ** 2
                    + (float(fk_after_pos[2]) - float(_seed_pos[2])) ** 2
                )
                panel._emit_log(
                    "[PICK][DIRECT][IK_MODEL] "
                    f"label={label} "
                    f"seed_pos=({_seed_pos[0]:.3f},{_seed_pos[1]:.3f},{_seed_pos[2]:.3f}) "
                    f"fk_after=({fk_after_pos[0]:.3f},{fk_after_pos[1]:.3f},{fk_after_pos[2]:.3f}) "
                    f"target_ik=({target_ik[0]:.3f},{target_ik[1]:.3f},{target_ik[2]:.3f}) "
                    f"model_target_err={model_target_err:.3f} "
                    f"model_move_delta={model_live_delta:.3f} "
                    f"joint_tol_rad={align_joint_tol_rad:.3f} "
                    f"runtime_target_ok={runtime_target_ok} "
                    f"runtime_target_dist={_fmt_scalar(runtime_target_dist)} "
                    f"runtime_target_tol={runtime_target_tol_m:.3f} "
                    f"runtime_target_pos={_fmt_vec(runtime_target_pos)}"
                )
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
                    "target_source_frame": DIRECT_SOURCE_FRAME,
                    "target_execution_frame": DIRECT_EXECUTION_FRAME,
                    "target_ik": _tuple3(target_ik),
                    "offset_vector": _tuple3(offset_vector),
                    "tcp_offset_m": float(tcp_offset_m),
                    "ik_solution": solved_q_list,
                    "err_norm": float(err_norm),
                    "ik_ok": bool(ik_ok),
                    "runtime_target_ok": runtime_target_ok,
                    "runtime_target_dist": (
                        float(runtime_target_dist)
                        if runtime_target_dist is not None
                        else None
                    ),
                }

            def _align_demo_grasp_direct() -> None:
                nonlocal z_alineada_alert_emitted
                max_attempts = max(
                    1,
                    int(
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_ATTEMPTS",
                                "3",
                            )
                            or 3
                        )
                    ),
                )
                align_z_residual_tol = max(
                    0.003,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M",
                            "0.015",
                        )
                        or 0.015
                    ),
                )
                align_z_improve_min = max(
                    0.001,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_ALIGN_Z_IMPROVE_MIN_M",
                            "0.006",
                        )
                        or 0.006
                    ),
                )
                align_z_bias_gain = max(
                    0.0,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_ALIGN_Z_BIAS_GAIN",
                            "0.70",
                        )
                        or 0.70
                    ),
                )
                align_z_bias_cap_m = max(
                    0.0,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_ALIGN_Z_BIAS_CAP_M",
                            "0.030",
                        )
                        or 0.030
                    ),
                )
                last_metrics = None
                last_debug = None
                for attempt in range(1, max_attempts + 1):
                    obj_base = _live_object_base()
                    if obj_base is None:
                        raise RuntimeError("demo_object_pose_unavailable_before_align")
                    target_z_expected = float(obj_base[2]) + float(GRIPPER_TCP_Z_OFFSET)
                    target_tcp_runtime_raw = (
                        float(obj_base[0]),
                        float(obj_base[1]),
                        float(target_z_expected),
                    )
                    target_tcp_runtime = target_tcp_runtime_raw
                    tcp_before = _live_tcp_base()
                    pre_metrics = _pre_close_alignment_metrics()
                    xy_lock_factor = max(
                        1.0,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_ALIGN_XY_LOCK_FACTOR",
                                "2.0",
                            )
                            or 2.0
                        ),
                    )
                    xy_tol_pre = float(pre_metrics.get("xy_tol") or 0.035)
                    z_tol_pre = float(pre_metrics.get("z_tol") or 0.030)
                    xy_dist_pre = (
                        float(pre_metrics.get("xy_dist"))
                        if pre_metrics.get("xy_dist") is not None
                        else None
                    )
                    z_error_pre = (
                        float(pre_metrics.get("z_error"))
                        if pre_metrics.get("z_error") is not None
                        else None
                    )
                    z_residual_pre = None
                    if tcp_before is not None:
                        z_residual_pre = float(tcp_before[2]) - float(target_z_expected)
                    decision = "full_xy_z"
                    keep_xy = bool(
                        tcp_before is not None
                        and xy_dist_pre is not None
                        and z_error_pre is not None
                        and xy_dist_pre <= (xy_tol_pre * xy_lock_factor)
                        and z_error_pre > z_tol_pre
                    )
                    if keep_xy:
                        # Si el TCP ya esta encima en XY, evitar correccion lateral
                        # que pueda degradar la alineacion visual; solo corregir Z.
                        target_tcp_runtime = (
                            float(tcp_before[0]),
                            float(tcp_before[1]),
                            float(target_tcp_runtime_raw[2]),
                        )
                        decision = "z_only_keep_xy"
                    z_bias_cmd = 0.0
                    if (
                        attempt > 1
                        and z_residual_pre is not None
                        and abs(float(z_residual_pre)) > align_z_residual_tol
                    ):
                        z_bias_cmd = max(
                            -align_z_bias_cap_m,
                            min(align_z_bias_cap_m, float(z_residual_pre) * align_z_bias_gain),
                        )
                        target_z_bias = float(target_z_expected) - float(z_bias_cmd)
                        if keep_xy and tcp_before is not None:
                            target_tcp_runtime = (
                                float(tcp_before[0]),
                                float(tcp_before[1]),
                                float(target_z_bias),
                            )
                            decision = "z_only_keep_xy_bias"
                        else:
                            target_tcp_runtime = (
                                float(target_tcp_runtime_raw[0]),
                                float(target_tcp_runtime_raw[1]),
                                float(target_z_bias),
                            )
                            decision = "full_xy_z_bias"
                    delta_raw = None
                    delta_used = None
                    if tcp_before is not None:
                        delta_raw = (
                            float(target_tcp_runtime_raw[0]) - float(tcp_before[0]),
                            float(target_tcp_runtime_raw[1]) - float(tcp_before[1]),
                            float(target_tcp_runtime_raw[2]) - float(tcp_before[2]),
                        )
                        delta_used = (
                            float(target_tcp_runtime[0]) - float(tcp_before[0]),
                            float(target_tcp_runtime[1]) - float(tcp_before[1]),
                            float(target_tcp_runtime[2]) - float(tcp_before[2]),
                        )
                    panel._emit_log(
                        "[PICK][DIRECT][IK_GEOM] "
                        f"attempt={attempt}/{max_attempts} "
                        f"tcp_before={_fmt_vec(tcp_before)} "
                        f"obj_base={_fmt_vec(obj_base)} "
                        f"target_tcp_raw={_fmt_vec(target_tcp_runtime_raw)} "
                        f"target_tcp_used={_fmt_vec(target_tcp_runtime)} "
                        f"delta_raw={_fmt_vec(delta_raw)} "
                        f"delta_used={_fmt_vec(delta_used)} "
                        f"xy_dist_pre={_fmt_scalar(xy_dist_pre)} "
                        f"z_error_pre={_fmt_scalar(z_error_pre)} "
                        f"z_residual_pre={_fmt_scalar(z_residual_pre)} "
                        f"target_z_expected={target_z_expected:.3f} "
                        f"z_bias_cmd={z_bias_cmd:.3f} "
                        f"align_z_residual_tol={align_z_residual_tol:.3f} "
                        f"xy_tol_pre={xy_tol_pre:.3f} "
                        f"z_tol_pre={z_tol_pre:.3f} "
                        f"xy_lock_factor={xy_lock_factor:.2f} "
                        f"decision={decision}"
                    )
                    last_debug = _move_tcp_direct(
                        label="GRASP_ALIGN_IK",
                        target_tcp_runtime=target_tcp_runtime,
                        timeout_sec=move_sec + 8.0,
                    )
                    last_metrics = _pre_close_alignment_metrics()
                    runtime_ok = bool((last_debug or {}).get("runtime_target_ok"))
                    tcp_after = _live_tcp_base()
                    obj_after = _live_object_base()
                    z_error_after = None
                    target_tcp_used_z = float(target_tcp_runtime[2])
                    tcp_final_z = None
                    z_expected_after = None
                    if obj_after is not None:
                        z_expected_after = float(obj_after[2]) + float(GRIPPER_TCP_Z_OFFSET)
                    if tcp_after is not None:
                        tcp_final_z = float(tcp_after[2])
                    if z_expected_after is not None and tcp_final_z is not None:
                        z_error_after = abs(float(tcp_final_z) - float(z_expected_after))
                    xy_dist_after = (
                        float(last_metrics.get("xy_dist"))
                        if last_metrics.get("xy_dist") is not None
                        else None
                    )
                    z_error_after_metrics = (
                        float(last_metrics.get("z_error"))
                        if last_metrics.get("z_error") is not None
                        else None
                    )
                    z_error_after_cmp = (
                        float(z_error_after)
                        if z_error_after is not None
                        else z_error_after_metrics
                    )
                    xy_after_ok = bool(
                        xy_dist_after is not None
                        and xy_dist_after <= max(0.010, xy_tol_pre * xy_lock_factor)
                    )
                    z_after_ok = bool(
                        z_error_after_cmp is not None
                        and z_error_after_cmp <= align_z_residual_tol
                    )
                    convergence_ok = bool(runtime_ok and xy_after_ok and z_after_ok)
                    z_improved = bool(
                        z_error_pre is not None
                        and z_error_after_cmp is not None
                        and (float(z_error_pre) - float(z_error_after_cmp)) >= align_z_improve_min
                    )
                    z_trace_msg = (
                        "[PICK][DIRECT][IK_Z_TRACE] "
                        f"attempt={attempt}/{max_attempts} "
                        f"obj_base={_fmt_vec(obj_base)} "
                        f"tcp_before={_fmt_vec(tcp_before)} "
                        f"target_tcp_raw={_fmt_vec(target_tcp_runtime_raw)} "
                        f"target_tcp_used={_fmt_vec(target_tcp_runtime)} "
                        f"target_z_expected={_fmt_scalar(target_z_expected)} "
                        f"target_tcp_raw_z={_fmt_scalar(target_tcp_runtime_raw[2])} "
                        f"target_tcp_used_z={_fmt_scalar(target_tcp_used_z)} "
                        f"tcp_final_z={_fmt_scalar(tcp_final_z)} "
                        f"obj_expected_z={_fmt_scalar(z_expected_after)} "
                        f"z_error_before={_fmt_scalar(z_error_pre)} "
                        f"z_error_after={_fmt_scalar(z_error_after_cmp)} "
                        f"xy_dist_before={_fmt_scalar(xy_dist_pre)} "
                        f"xy_dist_after={_fmt_scalar(xy_dist_after)} "
                        f"retries_used={attempt - 1} "
                        f"convergence_criterion=runtime_ok&&xy<=xy_lock&&z<={align_z_residual_tol:.3f} "
                        f"convergence_ok={str(convergence_ok).lower()} "
                        f"decision={decision}"
                    )
                    panel._emit_log(z_trace_msg)
                    _append_trace(z_trace_msg)
                    panel._emit_log(
                        "[PICK][DIRECT][IK_RUNTIME] "
                        f"attempt={attempt}/{max_attempts} "
                        f"runtime_ok={runtime_ok} "
                        f"xy_dist={_fmt_scalar(last_metrics.get('xy_dist'))} "
                        f"z_error={_fmt_scalar(last_metrics.get('z_error'))} "
                        f"z_error_tight={_fmt_scalar(z_error_after_cmp)}/{align_z_residual_tol:.3f} "
                        f"tcp_obj_dist={_fmt_scalar(last_metrics.get('tcp_obj_dist'))} "
                        f"ok_pre_close={bool(last_metrics.get('ok'))} "
                        f"ok_align_z={str(convergence_ok).lower()}"
                    )
                    if convergence_ok and not z_alineada_alert_emitted and z_improved:
                        z_alineada_alert_emitted = True
                        z_msg = (
                            "[PICK][DIRECT] AVISO: Z ALINEADA "
                            f"attempt={attempt}/{max_attempts} "
                            f"z_error_before={_fmt_scalar(z_error_pre)} "
                            f"z_error_after={_fmt_scalar(z_error_after_cmp)} "
                            f"xy_dist_after={_fmt_scalar(xy_dist_after)} "
                            f"target_z_expected={_fmt_scalar(target_z_expected)} "
                            f"tcp_final_z={_fmt_scalar(tcp_final_z)}"
                        )
                        panel._emit_log(z_msg)
                        _append_trace(z_msg)
                    if convergence_ok:
                        return last_debug
                    # Mantener estabilidad: si en el ultimo intento cumple PRE_CLOSE,
                    # no bloquear la secuencia aunque no alcance la tolerancia Z estricta.
                    if attempt >= max_attempts and bool(last_metrics.get("ok")):
                        return last_debug
                raise RuntimeError(
                    "grasp_align_ik_runtime_not_aligned "
                    f"attempts={max_attempts} "
                    f"xy_dist={_fmt_scalar((last_metrics or {}).get('xy_dist'))} "
                    f"z_error={_fmt_scalar((last_metrics or {}).get('z_error'))}"
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
                expected_z_gap = float(GRIPPER_TCP_Z_OFFSET)
                next_sample_log_ts = 0.0
                panel._emit_log(
                    "[PICK][DIRECT][ATTACH] "
                    f"waiting_follow timeout={float(timeout_sec):.2f}s "
                    f"max_tcp_dist={float(max_tcp_dist_m):.3f} "
                    f"expected_z_gap={expected_z_gap:.3f}"
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
                    dx = float(obj_base[0]) - float(tcp_base[0])
                    dy = float(obj_base[1]) - float(tcp_base[1])
                    z_gap = float(tcp_base[2]) - float(obj_base[2])
                    tcp_dist = _dist(obj_base, tcp_base)
                    best_tcp_dist = min(best_tcp_dist, tcp_dist)
                    now_ts = time.time()
                    if now_ts >= next_sample_log_ts:
                        panel._emit_log(
                            "[PICK][DIRECT][ATTACH] "
                            f"sample tcp={_fmt_vec(tcp_base)} obj={_fmt_vec(obj_base)} "
                            f"dx={dx:.3f} dy={dy:.3f} z_gap={z_gap:.3f} "
                            f"tcp_dist={tcp_dist:.3f} within_follow={str(tcp_dist <= float(max_tcp_dist_m)).lower()}"
                        )
                        next_sample_log_ts = now_ts + 0.24
                    if tcp_dist <= float(max_tcp_dist_m):
                        consecutive_ok += 1
                        if consecutive_ok >= max(1, int(min_consecutive)):
                            panel._emit_log(
                                "[PICK][DIRECT][ATTACH] "
                                f"follow_confirmed tcp_dist={tcp_dist:.3f} z_gap={z_gap:.3f} "
                                f"dx={dx:.3f} dy={dy:.3f} consecutive={consecutive_ok}"
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
                last_z_gap_txt = "none"
                if last_obj_base is not None and last_tcp_base is not None:
                    last_z_gap_txt = f"{(float(last_tcp_base[2]) - float(last_obj_base[2])):.3f}"
                raise RuntimeError(
                    "demo_attach_follow_not_confirmed "
                    f"best_tcp_dist={best_tcp_dist:.3f} "
                    f"last_obj_base={obj_txt} last_tcp_base={tcp_txt} "
                    f"last_z_gap={last_z_gap_txt}"
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
                last_obj_base = None
                fail_reasons = {
                    "obj_move_below_min": 0,
                    "lift_delta_below_min": 0,
                    "tcp_dist_above_max": 0,
                }
                next_sample_log_ts = 0.0
                panel._emit_log(
                    "[PICK][DIRECT][PHYSICS] "
                    f"phase={phase} start initial_obj_world=({initial_obj_world[0]:.3f},{initial_obj_world[1]:.3f},{initial_obj_world[2]:.3f}) "
                    f"min_obj_move={float(min_obj_move_m):.3f} min_lift_delta={float(min_lift_delta_m):.3f} "
                    f"max_tcp_dist={float(max_tcp_dist_m):.3f} "
                    f"expected_tcp_obj_z_gap={float(GRIPPER_TCP_Z_OFFSET):.3f}"
                )
                while time.time() < deadline:
                    obj_world = _live_object_world()
                    obj_base = _live_object_base()
                    tcp_base = _live_tcp_base()
                    last_obj_world = obj_world
                    last_obj_base = obj_base
                    last_tcp_base = tcp_base
                    if obj_world is None or obj_base is None or tcp_base is None:
                        consecutive_ok = 0
                        time.sleep(0.08)
                        continue
                    obj_move = _dist(obj_world, initial_obj_world)
                    lift_delta = float(obj_world[2]) - float(initial_obj_world[2])
                    tcp_dist = _dist(obj_base, tcp_base)
                    z_gap = float(tcp_base[2]) - float(obj_base[2])
                    best_obj_move = max(best_obj_move, obj_move)
                    best_lift = max(best_lift, lift_delta)
                    best_tcp_dist = min(best_tcp_dist, tcp_dist)
                    cond_obj_move = obj_move >= float(min_obj_move_m)
                    cond_lift = lift_delta >= float(min_lift_delta_m)
                    cond_tcp = tcp_dist <= float(max_tcp_dist_m)
                    if not cond_obj_move:
                        fail_reasons["obj_move_below_min"] += 1
                    if not cond_lift:
                        fail_reasons["lift_delta_below_min"] += 1
                    if not cond_tcp:
                        fail_reasons["tcp_dist_above_max"] += 1
                    now_ts = time.time()
                    if now_ts >= next_sample_log_ts:
                        panel._emit_log(
                            "[PICK][DIRECT][PHYSICS] "
                            f"phase={phase} sample obj_world={_fmt_vec(obj_world)} obj_base={_fmt_vec(obj_base)} "
                            f"tcp_base={_fmt_vec(tcp_base)} obj_move={obj_move:.3f} lift_delta={lift_delta:.3f} "
                            f"tcp_dist={tcp_dist:.3f} z_gap={z_gap:.3f} "
                            f"cond_obj_move={str(cond_obj_move).lower()} "
                            f"cond_lift={str(cond_lift).lower()} cond_tcp={str(cond_tcp).lower()}"
                        )
                        next_sample_log_ts = now_ts + 0.24
                    if (
                        cond_obj_move
                        and cond_lift
                        and cond_tcp
                    ):
                        consecutive_ok += 1
                        if consecutive_ok >= max(1, int(min_consecutive)):
                            panel._emit_log(
                                "[PICK][DIRECT][PHYSICS] "
                                f"phase={phase} ok obj_move={obj_move:.3f} lift_delta={lift_delta:.3f} "
                                f"tcp_dist={tcp_dist:.3f} z_gap={z_gap:.3f} consecutive={consecutive_ok}"
                            )
                            return
                    else:
                        consecutive_ok = 0
                    time.sleep(0.08)
                obj_txt = "none"
                tcp_txt = "none"
                obj_base_txt = "none"
                if last_obj_world is not None:
                    obj_txt = f"({last_obj_world[0]:.3f},{last_obj_world[1]:.3f},{last_obj_world[2]:.3f})"
                if last_obj_base is not None:
                    obj_base_txt = f"({last_obj_base[0]:.3f},{last_obj_base[1]:.3f},{last_obj_base[2]:.3f})"
                if last_tcp_base is not None:
                    tcp_txt = f"({last_tcp_base[0]:.3f},{last_tcp_base[1]:.3f},{last_tcp_base[2]:.3f})"
                fail_reason_keys = [
                    key for key, count in fail_reasons.items() if int(count) > 0
                ] or ["unknown"]
                raise RuntimeError(
                    "demo_carry_validation_failed "
                    f"phase={phase} best_obj_move={best_obj_move:.3f} best_lift_delta={best_lift:.3f} "
                    f"best_tcp_dist={best_tcp_dist:.3f} fail_reasons={','.join(fail_reason_keys)} "
                    f"last_obj_world={obj_txt} last_obj_base={obj_base_txt} last_tcp_base={tcp_txt}"
                )

            demo_attach_published = False
            demo_logical_attached = False
            encima_alert_emitted = False
            listo_alert_emitted = False
            approach_coarse_util_alert_emitted = False
            grasp_down_util_alert_emitted = False
            z_alineada_alert_emitted = False
            alcance_pause_done = False
            alcance_monitor_last_log_ts = 0.0
            debug_pause_grasp_align_enabled = str(
                os.environ.get("PANEL_PICK_DEMO_DEBUG_PAUSE_GRASP_ALIGN_IK", "0") or "0"
            ).strip().lower() in {"1", "true", "yes", "on"}
            debug_pause_grasp_align_done = False

            def _monitor_alcance(*, trigger: str) -> dict:
                nonlocal encima_alert_emitted
                nonlocal listo_alert_emitted
                nonlocal alcance_pause_done
                nonlocal alcance_monitor_last_log_ts
                metrics = _pre_close_alignment_metrics()
                metric_dict = _json_safe(metrics) or {}
                xy_dist_raw = metric_dict.get("xy_dist")
                xy_ready_tol_raw = metric_dict.get("xy_tol")
                try:
                    xy_dist = float(xy_dist_raw) if xy_dist_raw is not None else float("inf")
                except Exception:
                    xy_dist = float("inf")
                try:
                    xy_ready_tol = float(xy_ready_tol_raw) if xy_ready_tol_raw is not None else 0.035
                except Exception:
                    xy_ready_tol = 0.035
                on_top_xy_tol = max(
                    0.01,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_ON_TOP_XY_TOL_M",
                            str(xy_ready_tol),
                        )
                        or xy_ready_tol
                    ),
                )
                on_top = bool(math.isfinite(xy_dist) and xy_dist <= on_top_xy_tol)
                ready = bool(metric_dict.get("ok"))
                now_ts = time.time()
                if now_ts >= (alcance_monitor_last_log_ts + 0.25):
                    monitor_msg = (
                        "[PICK][DIRECT][ALCANCE_MONITOR] "
                        f"trigger={trigger} on_top={str(on_top).lower()} ready={str(ready).lower()} "
                        f"tcp_obj_dist={_fmt_scalar(metric_dict.get('tcp_obj_dist'))} "
                        f"xy_dist={_fmt_scalar(metric_dict.get('xy_dist'))}/{_fmt_scalar(metric_dict.get('xy_tol'))} "
                        f"z_error={_fmt_scalar(metric_dict.get('z_error'))}/{_fmt_scalar(metric_dict.get('z_tol'))} "
                        f"z_gap={_fmt_scalar(metric_dict.get('z_gap'))}"
                    )
                    panel._emit_log(monitor_msg)
                    _append_trace(monitor_msg)
                    alcance_monitor_last_log_ts = now_ts
                if on_top and not encima_alert_emitted:
                    encima_alert_emitted = True
                    alert_on_top_msg = (
                        "[PICK][DIRECT] AVISO: ENCIMA DEL OBJETO "
                        f"trigger={trigger} "
                        f"tcp_obj_dist={_fmt_scalar(metric_dict.get('tcp_obj_dist'))} "
                        f"xy_dist={_fmt_scalar(metric_dict.get('xy_dist'))}/{_fmt_scalar(on_top_xy_tol)} "
                        f"z_gap={_fmt_scalar(metric_dict.get('z_gap'))}"
                    )
                    panel._emit_log(alert_on_top_msg)
                    _append_trace(alert_on_top_msg)
                if ready and not listo_alert_emitted:
                    listo_alert_emitted = True
                    alert_ready_msg = (
                        "[PICK][DIRECT] AVISO: LISTO PARA COGER "
                        f"trigger={trigger} "
                        f"tcp_obj_dist={_fmt_scalar(metric_dict.get('tcp_obj_dist'))} "
                        f"xy_dist={_fmt_scalar(metric_dict.get('xy_dist'))}/{_fmt_scalar(metric_dict.get('xy_tol'))} "
                        f"z_error={_fmt_scalar(metric_dict.get('z_error'))}/{_fmt_scalar(metric_dict.get('z_tol'))} "
                        f"z_gap={_fmt_scalar(metric_dict.get('z_gap'))}"
                    )
                    panel._emit_log(alert_ready_msg)
                    _append_trace(alert_ready_msg)
                    if not alcance_pause_done:
                        alcance_pause_done = True
                        pause_fn = getattr(panel, "_debug_motion_wait_for_continue", None)
                        if callable(pause_fn):
                            pause_reason = "ALCANCE_DIRECTO"
                            pause_req_msg = (
                                "[PICK][DIRECT][ALCANCE_DEBUG] "
                                f"pause_requested reason={pause_reason} trigger={trigger} "
                                "waiting_button=DEBUG_MOVIMIENTO"
                            )
                            panel._emit_log(pause_req_msg)
                            _append_trace(pause_req_msg)
                            try:
                                resumed = bool(pause_fn(reason=pause_reason))
                            except Exception as exc:
                                resumed = True
                                err_msg = (
                                    "[PICK][DIRECT][ALCANCE_DEBUG] "
                                    f"pause_error reason={pause_reason} exc={exc}"
                                )
                                panel._emit_log(err_msg)
                                _append_trace(err_msg)
                            resume_msg = (
                                "[PICK][DIRECT][ALCANCE_DEBUG] "
                                f"pause_resume reason={pause_reason} trigger={trigger} "
                                f"resumed={str(resumed).lower()}"
                            )
                            panel._emit_log(resume_msg)
                            _append_trace(resume_msg)
                return metric_dict

            def _emit_phase_utility_alert(
                *,
                phase: str,
                target_base=None,
                frame_used: str = "base_link",
                preset_used: str | None = None,
                decision: str = "",
            ) -> None:
                nonlocal approach_coarse_util_alert_emitted
                nonlocal grasp_down_util_alert_emitted
                metrics = _phase_tcp_obj_metrics_base()
                if not bool(metrics.get("ok")):
                    return
                xy_dist = float(metrics.get("xy_dist") or float("inf"))
                z_error = float(metrics.get("z_error") or float("inf"))
                dist = float(metrics.get("dist") or float("inf"))
                execution_type = _execution_type_from_decision(decision)
                if phase == "APPROACH_COARSE":
                    xy_tol = max(
                        0.05,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_XY_TOL_M",
                                "0.18",
                            )
                            or 0.18
                        ),
                    )
                    z_tol = max(
                        0.05,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_Z_ERR_TOL_M",
                                "0.18",
                            )
                            or 0.18
                        ),
                    )
                    dist_tol = max(
                        0.10,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_DIST_TOL_M",
                                "0.26",
                            )
                            or 0.26
                        ),
                    )
                    util = bool(xy_dist <= xy_tol and z_error <= z_tol and dist <= dist_tol)
                    if util and not approach_coarse_util_alert_emitted:
                        approach_coarse_util_alert_emitted = True
                        msg = (
                            "[PICK][DIRECT] AVISO: APPROACH GEOMETRICO UTIL "
                            f"phase={phase} frame={frame_used} preset={preset_used or 'none'} "
                            f"execution_type={execution_type} "
                            f"target_base={_fmt_vec(target_base)} "
                            f"dx={_fmt_scalar(metrics.get('dx'))} dy={_fmt_scalar(metrics.get('dy'))} "
                            f"dz={_fmt_scalar(metrics.get('dz'))} dist={_fmt_scalar(metrics.get('dist'))} "
                            f"xy_dist={_fmt_scalar(metrics.get('xy_dist'))}/{_fmt_scalar(xy_tol)} "
                            f"z_error={_fmt_scalar(metrics.get('z_error'))}/{_fmt_scalar(z_tol)} "
                            f"dist_tol={_fmt_scalar(dist_tol)} decision={decision or 'none'}"
                        )
                        panel._emit_log(msg)
                        _append_trace(msg)
                elif phase == "GRASP_DOWN_JOINT":
                    xy_tol = max(
                        0.01,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_XY_TOL_M",
                                "0.10",
                            )
                            or 0.10
                        ),
                    )
                    z_tol = max(
                        0.01,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M",
                                "0.10",
                            )
                            or 0.10
                        ),
                    )
                    dist_tol = max(
                        0.01,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_DIST_TOL_M",
                                "0.22",
                            )
                            or 0.22
                        ),
                    )
                    util = bool(xy_dist <= xy_tol and z_error <= z_tol and dist <= dist_tol)
                    if util and not grasp_down_util_alert_emitted:
                        grasp_down_util_alert_emitted = True
                        msg = (
                            "[PICK][DIRECT] AVISO: GRASP_DOWN GEOMETRICO UTIL "
                            f"phase={phase} frame={frame_used} preset={preset_used or 'none'} "
                            f"execution_type={execution_type} "
                            f"target_base={_fmt_vec(target_base)} "
                            f"dx={_fmt_scalar(metrics.get('dx'))} dy={_fmt_scalar(metrics.get('dy'))} "
                            f"dz={_fmt_scalar(metrics.get('dz'))} dist={_fmt_scalar(metrics.get('dist'))} "
                            f"xy_dist={_fmt_scalar(metrics.get('xy_dist'))}/{_fmt_scalar(xy_tol)} "
                            f"z_error={_fmt_scalar(metrics.get('z_error'))}/{_fmt_scalar(z_tol)} "
                            f"dist_tol={_fmt_scalar(dist_tol)} decision={decision or 'none'}"
                        )
                        panel._emit_log(msg)
                        _append_trace(msg)

            def _debug_pause_grasp_align_if_enabled(*, trigger: str) -> None:
                nonlocal debug_pause_grasp_align_done
                if debug_pause_grasp_align_done or not debug_pause_grasp_align_enabled:
                    return
                debug_pause_grasp_align_done = True
                metrics = _pre_close_alignment_metrics()
                metric_dict = _json_safe(metrics) or {}
                pause_msg = (
                    "[PICK][DIRECT][DEBUG_PAUSE] "
                    f"phase=GRASP_ALIGN_IK trigger={trigger} "
                    f"tcp={_fmt_vec(metric_dict.get('tcp_base'))} "
                    f"obj={_fmt_vec(metric_dict.get('object_base'))} "
                    f"tcp_obj_dist={_fmt_scalar(metric_dict.get('tcp_obj_dist'))} "
                    f"xy_dist={_fmt_scalar(metric_dict.get('xy_dist'))}/{_fmt_scalar(metric_dict.get('xy_tol'))} "
                    f"z_error={_fmt_scalar(metric_dict.get('z_error'))}/{_fmt_scalar(metric_dict.get('z_tol'))} "
                    f"z_gap={_fmt_scalar(metric_dict.get('z_gap'))}"
                )
                panel._emit_log(pause_msg)
                _append_trace(pause_msg)
                panel.signal_run_ui.emit(
                    lambda: panel._ui_set_status(
                        "DEBUG GRASP_ALIGN_IK: pausa activa, pulsa DEBUG MOVIMIENTO para continuar",
                        error=False,
                    )
                )
                pause_fn = getattr(panel, "_debug_motion_wait_for_continue", None)
                if callable(pause_fn):
                    try:
                        resumed = bool(pause_fn(reason="GRASP_ALIGN_IK"))
                    except Exception as exc:
                        resumed = True
                        err_msg = (
                            "[PICK][DIRECT][DEBUG_PAUSE] "
                            f"phase=GRASP_ALIGN_IK pause_error={exc}"
                        )
                        panel._emit_log(err_msg)
                        _append_trace(err_msg)
                else:
                    resumed = True
                resume_msg = (
                    "[PICK][DIRECT][DEBUG_PAUSE] "
                    f"phase=GRASP_ALIGN_IK resumed={str(resumed).lower()} trigger={trigger}"
                )
                panel._emit_log(resume_msg)
                _append_trace(resume_msg)

            _run_joint_step("HOME", home_pose)
            _run_joint_step("MESA", JOINT_TABLE_POSE_RAD)
            _monitor_alcance(trigger="DIRECT_PICK_START")

            panel._emit_log("[DEMO] Abriendo pinza en posición MESA")
            panel.signal_run_ui.emit(lambda: panel._command_gripper(False, log_action="PICK", force=True))
            time.sleep(0.6)

            _run_joint_step("PICK_IMAGE", JOINT_PICK_IMAGE_POSE_RAD)

            panel._emit_log("[DEMO] Bajando a pose de grasp (joints)")
            coarse_extra_z_m = max(
                0.0,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M",
                        "0.10",
                    )
                    or 0.10
                ),
            )
            grasp_down_extra_z_m = max(
                0.0,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_GRASP_DOWN_EXTRA_Z_M",
                        "0.00",
                    )
                    or 0.00
                ),
            )
            preset_approach = "direct_rg2_tcp_dynamic_coarse"
            preset_grasp_down = "direct_rg2_tcp_dynamic_down"
            obj_base_before_coarse = _live_object_base()
            target_base_coarse = None
            target_world_coarse = None
            target_base_grasp_down = None
            target_world_grasp_down = None
            if obj_base_before_coarse is not None:
                target_base_coarse = (
                    float(obj_base_before_coarse[0]),
                    float(obj_base_before_coarse[1]),
                    float(obj_base_before_coarse[2]) + float(GRIPPER_TCP_Z_OFFSET) + float(coarse_extra_z_m),
                )
                target_world_coarse = _target_world_from_base(target_base_coarse)
                target_base_grasp_down = (
                    float(obj_base_before_coarse[0]),
                    float(obj_base_before_coarse[1]),
                    float(obj_base_before_coarse[2]) + float(GRIPPER_TCP_Z_OFFSET) + float(grasp_down_extra_z_m),
                )
                target_world_grasp_down = _target_world_from_base(target_base_grasp_down)
            _phase_begin(
                "APPROACH_COARSE",
                target_world=target_world_coarse,
                target_base=target_base_coarse,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "coarse_extra_z_m": float(coarse_extra_z_m),
                    "mode": "direct_rg2_tcp",
                },
                joint_goal=[float(v) for v in JOINT_GRASP_DOWN_POSE_RAD],
                note="dynamic coarse approach over object before GRASP_DOWN_JOINT",
            )
            _trace_phase_pose(
                phase="APPROACH_COARSE",
                event="target_set",
                target_base=target_base_coarse,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "coarse_extra_z_m": float(coarse_extra_z_m),
                    "mode": "direct_rg2_tcp",
                    "execution_mode": "hybrid_geometric_with_preset_fallback",
                },
                preset_used=preset_approach,
                execution_type="hibrido",
                decision="phase_enter",
            )
            approach_debug = None
            approach_decision = "direct_ik_move"
            if target_base_coarse is not None:
                try:
                    approach_debug = _move_tcp_direct(
                        label="APPROACH_COARSE",
                        target_tcp_runtime=target_base_coarse,
                        timeout_sec=max(move_sec + 2.5, 4.0),
                    )
                except Exception as exc:
                    approach_decision = f"fallback_joint_preset:{exc}"
                    panel._emit_log(
                        "[PICK][DIRECT][WARN] "
                        f"phase=APPROACH_COARSE direct_ik_failed={exc} fallback=joint_preset"
                    )
                    _run_joint_step(
                        "APPROACH_COARSE_FALLBACK",
                        JOINT_GRASP_DOWN_POSE_RAD,
                        timeout_sec=move_sec + 5.0,
                        tol_rad=0.10,
                    )
            else:
                approach_decision = "target_unavailable_keep_pose"
                panel._emit_log(
                    "[PICK][DIRECT][WARN] phase=APPROACH_COARSE target_unavailable"
                )
            _phase_end(
                "APPROACH_COARSE",
                result="ok",
                joint_goal=(
                    approach_debug.get("ik_solution")
                    if isinstance(approach_debug, dict)
                    else [float(v) for v in JOINT_GRASP_DOWN_POSE_RAD]
                ),
                ik_solution=approach_debug,
                note=approach_decision,
            )
            _trace_phase_pose(
                phase="APPROACH_COARSE",
                event="phase_end",
                target_base=target_base_coarse,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "coarse_extra_z_m": float(coarse_extra_z_m),
                    "mode": "direct_rg2_tcp",
                    "execution_mode": "hybrid_geometric_with_preset_fallback",
                },
                preset_used=preset_approach,
                execution_type=_execution_type_from_decision(approach_decision),
                decision=approach_decision,
            )
            _emit_phase_utility_alert(
                phase="APPROACH_COARSE",
                target_base=target_base_coarse,
                frame_used="base_link",
                preset_used=preset_approach,
                decision=approach_decision,
            )

            # Refrescar target justo antes del descenso para seguir pick_demo vivo.
            obj_base_before_grasp_down = _live_object_base()
            if obj_base_before_grasp_down is None:
                obj_base_before_grasp_down = obj_base_before_coarse
            target_base_grasp_down = None
            target_world_grasp_down = None
            if obj_base_before_grasp_down is not None:
                target_base_grasp_down = (
                    float(obj_base_before_grasp_down[0]),
                    float(obj_base_before_grasp_down[1]),
                    float(obj_base_before_grasp_down[2]) + float(GRIPPER_TCP_Z_OFFSET) + float(grasp_down_extra_z_m),
                )
                target_world_grasp_down = _target_world_from_base(target_base_grasp_down)
            _phase_begin(
                "GRASP_DOWN_JOINT",
                target_world=target_world_grasp_down,
                target_base=target_base_grasp_down,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "grasp_down_extra_z_m": float(grasp_down_extra_z_m),
                    "mode": "direct_rg2_tcp",
                    "execution_mode": "hybrid_geometric_with_preset_fallback",
                },
                joint_goal=[float(v) for v in JOINT_GRASP_DOWN_POSE_RAD],
                note="dynamic descent near grasp before GRASP_ALIGN_IK",
            )
            _trace_phase_pose(
                phase="GRASP_DOWN_JOINT",
                event="target_set",
                target_base=target_base_grasp_down,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "grasp_down_extra_z_m": float(grasp_down_extra_z_m),
                    "mode": "direct_rg2_tcp",
                    "execution_mode": "hybrid_geometric_with_preset_fallback",
                },
                preset_used=preset_grasp_down,
                execution_type="hibrido",
                decision="phase_enter",
            )
            grasp_down_debug = None
            grasp_down_decision = "direct_ik_move"
            if target_base_grasp_down is not None:
                try:
                    grasp_down_debug = _move_tcp_direct(
                        label="GRASP_DOWN_JOINT",
                        target_tcp_runtime=target_base_grasp_down,
                        timeout_sec=max(move_sec + 3.0, 4.5),
                    )
                except Exception as exc:
                    grasp_down_decision = f"fallback_joint_preset:{exc}"
                    panel._emit_log(
                        "[PICK][DIRECT][WARN] "
                        f"phase=GRASP_DOWN_JOINT direct_ik_failed={exc} fallback=joint_preset"
                    )
                    _run_joint_step(
                        "GRASP_DOWN_JOINT",
                        JOINT_GRASP_DOWN_POSE_RAD,
                        timeout_sec=move_sec + 6.0,
                        tol_rad=0.08,
                    )
            else:
                grasp_down_decision = "target_unavailable_fallback_joint_preset"
                panel._emit_log(
                    "[PICK][DIRECT][WARN] phase=GRASP_DOWN_JOINT target_unavailable fallback=joint_preset"
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
                dx = float(tcp_after_joint[0]) - float(obj_after_joint[0])
                dy = float(tcp_after_joint[1]) - float(obj_after_joint[1])
                dz = float(tcp_after_joint[2]) - float(obj_after_joint[2])
                xy_dist = math.hypot(dx, dy)
                panel._emit_log(
                    "[PICK][DIRECT][PHASE] "
                    f"phase=GRASP_DOWN_JOINT frame=base_link "
                    f"tcp=({tcp_after_joint[0]:.3f},{tcp_after_joint[1]:.3f},{tcp_after_joint[2]:.3f}) "
                    f"obj=({obj_after_joint[0]:.3f},{obj_after_joint[1]:.3f},{obj_after_joint[2]:.3f}) "
                    f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} "
                    f"xy_dist={xy_dist:.3f} "
                    f"tcp_obj_dist={_dist(tcp_after_joint, obj_after_joint):.3f}"
                )
            _phase_end(
                "GRASP_DOWN_JOINT",
                result="ok",
                joint_goal=(
                    grasp_down_debug.get("ik_solution")
                    if isinstance(grasp_down_debug, dict)
                    else [float(v) for v in JOINT_GRASP_DOWN_POSE_RAD]
                ),
                ik_solution=grasp_down_debug,
                note=grasp_down_decision,
            )
            _trace_phase_pose(
                phase="GRASP_DOWN_JOINT",
                event="phase_end",
                target_base=target_base_grasp_down,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "grasp_down_extra_z_m": float(grasp_down_extra_z_m),
                    "mode": "direct_rg2_tcp",
                    "execution_mode": "hybrid_geometric_with_preset_fallback",
                },
                preset_used=preset_grasp_down,
                execution_type=_execution_type_from_decision(grasp_down_decision),
                decision=grasp_down_decision,
            )
            _emit_phase_utility_alert(
                phase="GRASP_DOWN_JOINT",
                target_base=target_base_grasp_down,
                frame_used="base_link",
                preset_used=preset_grasp_down,
                decision=grasp_down_decision,
            )
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
            skip_align_if_reachable = str(
                os.environ.get("PANEL_PICK_DEMO_SKIP_ALIGN_IF_REACHABLE", "1")
                or "1"
            ).strip().lower() in ("1", "true", "yes", "on")
            pre_align_metrics = _pre_close_alignment_metrics()
            if skip_align_if_reachable and bool(pre_align_metrics.get("ok")):
                _phase_begin(
                    "GRASP_ALIGN_IK",
                    target_world=target_world_align,
                    target_base=target_base_align,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "source_frame": DIRECT_SOURCE_FRAME,
                        "execution_frame": DIRECT_EXECUTION_FRAME,
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                    },
                    note="skip align: tcp already within pre-close tolerance",
                )
                _trace_phase_pose(
                    phase="GRASP_ALIGN_IK",
                    event="target_set",
                    target_base=target_base_align,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "source_frame": DIRECT_SOURCE_FRAME,
                        "execution_frame": DIRECT_EXECUTION_FRAME,
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                    },
                    preset_used=DIRECT_EXECUTION_IK_MODE,
                    decision="phase_enter_skip",
                )
                _debug_pause_grasp_align_if_enabled(trigger="phase_enter_skip")
                _phase_end(
                    "GRASP_ALIGN_IK",
                    note=json.dumps(_json_safe(pre_align_metrics), ensure_ascii=False, sort_keys=True),
                    result="skipped",
                )
                _trace_phase_pose(
                    phase="GRASP_ALIGN_IK",
                    event="phase_end",
                    target_base=target_base_align,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "source_frame": DIRECT_SOURCE_FRAME,
                        "execution_frame": DIRECT_EXECUTION_FRAME,
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                    },
                    preset_used=DIRECT_EXECUTION_IK_MODE,
                    decision="skip_align_preclose_ok",
                )
                panel._emit_log(
                    "[PICK][DIRECT] GRASP_ALIGN_IK skipped: close-window already satisfied "
                    f"xy_dist={float(pre_align_metrics.get('xy_dist') or 0.0):.3f} "
                    f"z_error={float(pre_align_metrics.get('z_error') or 0.0):.3f}"
                )
            else:
                _phase_begin(
                    "GRASP_ALIGN_IK",
                    target_world=target_world_align,
                    target_base=target_base_align,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "source_frame": DIRECT_SOURCE_FRAME,
                        "execution_frame": DIRECT_EXECUTION_FRAME,
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                    },
                    note="fine alignment IK over live object pose",
                )
                _trace_phase_pose(
                    phase="GRASP_ALIGN_IK",
                    event="target_set",
                    target_base=target_base_align,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "source_frame": DIRECT_SOURCE_FRAME,
                        "execution_frame": DIRECT_EXECUTION_FRAME,
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                    },
                    preset_used=DIRECT_EXECUTION_IK_MODE,
                    decision="phase_enter",
                )
                _debug_pause_grasp_align_if_enabled(trigger="phase_enter")
                align_retries = max(
                    1,
                    int(
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_GRASP_ALIGN_MAX_ATTEMPTS",
                                "3",
                            )
                            or 3
                        )
                    ),
                )
                align_debug = None
                align_metrics = {}
                for align_attempt in range(1, align_retries + 1):
                    align_debug = _align_demo_grasp_direct()
                    align_metrics = _pre_close_alignment_metrics()
                    _trace_phase_pose(
                        phase="GRASP_ALIGN_IK",
                        event=f"attempt_{align_attempt}",
                        target_base=target_base_align,
                        frame_used="base_link",
                        offsets={
                            "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                            "source_frame": DIRECT_SOURCE_FRAME,
                            "execution_frame": DIRECT_EXECUTION_FRAME,
                            "ik_mode": DIRECT_EXECUTION_IK_MODE,
                            "attempt": int(align_attempt),
                            "max_attempts": int(align_retries),
                        },
                        preset_used=DIRECT_EXECUTION_IK_MODE,
                        decision="align_retry",
                    )
                    if bool(align_metrics.get("ok")):
                        break
                    if align_attempt < align_retries:
                        panel._emit_log(
                            "[PICK][DIRECT][ALIGN] "
                            f"retry attempt={align_attempt + 1}/{align_retries} "
                            f"xy_dist={_fmt_scalar(align_metrics.get('xy_dist'))} "
                            f"z_error={_fmt_scalar(align_metrics.get('z_error'))}"
                        )
                _phase_end(
                    "GRASP_ALIGN_IK",
                    ik_solution=align_debug.get("ik_solution") if isinstance(align_debug, dict) else None,
                    note=json.dumps(_json_safe(align_metrics), ensure_ascii=False, sort_keys=True),
                    result="ok" if bool(align_metrics.get("ok")) else "partial",
                )
                _trace_phase_pose(
                    phase="GRASP_ALIGN_IK",
                    event="phase_end",
                    target_base=target_base_align,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "source_frame": DIRECT_SOURCE_FRAME,
                        "execution_frame": DIRECT_EXECUTION_FRAME,
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                    },
                    preset_used=DIRECT_EXECUTION_IK_MODE,
                    decision="phase_exit",
                )
            post_align_metrics = _pre_close_alignment_metrics()
            _emit_transition_decision(
                from_phase="GRASP_ALIGN_IK",
                to_phase="PRE_CLOSE",
                decision="gate_check",
                reason="grasp_align_completed",
                condition="prepare_pre_close_gate",
                metrics=post_align_metrics,
            )
            extra_down_m = max(
                0.0,
                float(os.environ.get("PANEL_PICK_DEMO_EXTRA_GRASP_DOWN_M", "0.0") or 0.0),
            )
            if extra_down_m > 1e-4:
                try:
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
                except Exception as extra_down_exc:
                    _emit_transition_decision(
                        from_phase="GRASP_ALIGN_IK",
                        to_phase="PRE_CLOSE",
                        decision="blocked",
                        reason=f"extra_down_failed:{extra_down_exc}",
                        condition="extra_down_completed",
                        metrics=_pre_close_alignment_metrics(),
                    )
                    raise
            else:
                panel._emit_log("[PICK][DIRECT] moveit_extra_down=disabled route=direct_joint_only")
            post_align_settle_sec = max(
                0.0,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_POST_ALIGN_SETTLE_SEC",
                        "0.20",
                    )
                    or 0.20
                ),
            )
            panel._emit_log(
                "[PICK][DIRECT][TRANSITION] "
                f"from=GRASP_ALIGN_IK to=PRE_CLOSE decision=settle_wait reason=post_align_settle "
                f"condition=post_align_settle_sec wait_sec={post_align_settle_sec:.2f}"
            )
            if post_align_settle_sec > 1e-4:
                time.sleep(post_align_settle_sec)
            initial_obj_world = _live_object_world()
            if initial_obj_world is None:
                fallback_initial_obj_world = (
                    _tuple3(target_world_align)
                    or _tuple3(last_target.get("target_world"))
                )
                if fallback_initial_obj_world is not None:
                    initial_obj_world = fallback_initial_obj_world
                    panel._emit_log(
                        "[PICK][DIRECT][WARN] "
                        f"initial_obj_world unavailable; using fallback={_fmt_vec(initial_obj_world)}"
                    )
                    _emit_transition_decision(
                        from_phase="GRASP_ALIGN_IK",
                        to_phase="PRE_CLOSE",
                        decision="enter",
                        reason="object_pose_unavailable_using_fallback",
                        condition="initial_obj_world_available",
                        metrics=_pre_close_alignment_metrics(),
                    )
                else:
                    _emit_transition_decision(
                        from_phase="GRASP_ALIGN_IK",
                        to_phase="PRE_CLOSE",
                        decision="blocked",
                        reason="object_pose_unavailable_before_pre_close",
                        condition="initial_obj_world_available",
                        metrics=_pre_close_alignment_metrics(),
                    )
                    raise RuntimeError("demo_object_pose_unavailable_before_close")
            else:
                _emit_transition_decision(
                    from_phase="GRASP_ALIGN_IK",
                    to_phase="PRE_CLOSE",
                    decision="enter",
                    reason="object_pose_available_before_pre_close",
                    condition="initial_obj_world_available",
                    metrics=_pre_close_alignment_metrics(),
                )
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
            pre_close_wait_sec = max(
                0.4,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_PRE_CLOSE_WAIT_SEC",
                        "1.2",
                    )
                    or 1.2
                ),
            )
            pre_close_min_consecutive = max(
                1,
                int(
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_PRE_CLOSE_CONSECUTIVE",
                            "3",
                        )
                        or 3
                    )
                ),
            )
            pre_close_realign_retries = max(
                0,
                int(
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES",
                            "1",
                        )
                        or 1
                    )
                ),
            )
            pre_close_ok = False
            pre_close_metrics = {}
            pre_close_attempt = 0
            _final_phase_trace(
                "PRE_CLOSE",
                event="wait_start",
                expected="pre_close_ok",
                received="pending",
                timeout_sec=f"{pre_close_wait_sec:.2f}",
                reason="wait_pre_close_alignment",
                request_state="alignment_gate",
            )
            while pre_close_attempt <= pre_close_realign_retries:
                panel._emit_log(
                    "[PICK][DIRECT][PRE_CLOSE_GATE] "
                    f"waiting_alignment attempt={pre_close_attempt + 1}/{pre_close_realign_retries + 1} "
                    f"timeout={pre_close_wait_sec:.2f}s "
                    f"min_consecutive={pre_close_min_consecutive}"
                )
                pre_close_ok, pre_close_metrics = _wait_pre_close_alignment(
                    timeout_sec=pre_close_wait_sec,
                    min_consecutive=pre_close_min_consecutive,
                )
                if bool(pre_close_ok):
                    break
                if pre_close_attempt >= pre_close_realign_retries:
                    break
                panel._emit_log(
                    "[PICK][DIRECT][PRE_CLOSE_GATE] "
                    "alignment_not_reached -> retrying fine align before close"
                )
                try:
                    _align_demo_grasp_direct()
                except Exception as realign_exc:
                    pre_close_metrics["realign_error"] = str(realign_exc)
                    break
                pre_close_attempt += 1
            pre_close_metrics["wait_timeout_sec"] = float(pre_close_wait_sec)
            pre_close_metrics["min_consecutive"] = int(pre_close_min_consecutive)
            pre_close_metrics["realign_retries"] = int(pre_close_realign_retries)
            pre_close_metrics["attempt_used"] = int(pre_close_attempt + 1)
            _final_phase_trace(
                "PRE_CLOSE",
                event="wait_done",
                expected="pre_close_ok",
                received=str(bool(pre_close_ok)).lower(),
                timeout_sec=f"{pre_close_wait_sec:.2f}",
                reason="gate_result",
                logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                physical_state=f"xy={_fmt_scalar(pre_close_metrics.get('xy_dist'))},z={_fmt_scalar(pre_close_metrics.get('z_error'))}",
            )
            _phase_end(
                "PRE_CLOSE",
                note=json.dumps(_json_safe(pre_close_metrics), ensure_ascii=False, sort_keys=True),
                result="ok" if bool(pre_close_ok) else "failed",
            )
            if not bool(pre_close_ok):
                _emit_transition_decision(
                    from_phase="PRE_CLOSE",
                    to_phase="CLOSE",
                    decision="blocked",
                    reason="pre_close_gate_not_satisfied",
                    condition="pre_close_ok",
                    metrics=pre_close_metrics,
                )
                _abort_grasp(
                    code="PRE_CLOSE_NOT_ALIGNED",
                    phase="PRE_CLOSE",
                    note="tcp not aligned with object before close; refusing to close away from target",
                    metrics=pre_close_metrics,
                )
            _emit_transition_decision(
                from_phase="PRE_CLOSE",
                to_phase="CLOSE",
                decision="enter",
                reason="pre_close_gate_satisfied",
                condition="pre_close_ok",
                metrics=pre_close_metrics,
            )
            _monitor_alcance(trigger="PRE_CLOSE_GATE_OK")
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
            close_state_pre_cmd = _read_gripper_state(expected_closed=False)
            panel._emit_log(
                "[PICK][DIRECT][CLOSE] "
                f"pre_cmd opening_sum={_fmt_scalar(close_state_pre_cmd.get('opening_sum'))} "
                f"max_abs_err={_fmt_scalar(close_state_pre_cmd.get('max_abs_err'))} "
                f"measured_ok={bool(close_state_pre_cmd.get('measured_target_ok'))} "
                f"age={_fmt_scalar(close_state_pre_cmd.get('joint_state_age_sec'))} "
                f"closed_flag={bool(close_state_pre_cmd.get('closed_flag'))}"
            )
            panel.signal_run_ui.emit(_close_only)
            time.sleep(0.1)
            close_confirm_timeout_sec = max(
                0.8,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC",
                        "1.8",
                    )
                    or 1.8
                ),
            )
            _final_phase_trace(
                "CLOSE",
                event="wait_start",
                expected="close_confirmed&&close_metrics_ok",
                received="pending",
                timeout_sec=f"{close_confirm_timeout_sec:.2f}",
                reason="wait_gripper_close_confirm",
                request_state="gripper_close",
            )
            close_confirmed, close_wait_state = _wait_for_gripper_target(
                True,
                timeout_sec=close_confirm_timeout_sec,
                opening_ref_sum=close_state_pre_cmd.get("opening_sum"),
            )
            panel._emit_log(
                "[PICK][DIRECT][CLOSE] "
                f"wait_done confirmed={bool(close_confirmed)} "
                f"mode={str((close_wait_state or {}).get('confirm_mode') or 'none')} "
                f"opening_sum={_fmt_scalar((close_wait_state or {}).get('opening_sum'))} "
                f"max_abs_err={_fmt_scalar((close_wait_state or {}).get('max_abs_err'))} "
                f"measured_ok={bool((close_wait_state or {}).get('measured_target_ok'))} "
                f"closed_flag={bool((close_wait_state or {}).get('closed_flag'))} "
                f"age={_fmt_scalar((close_wait_state or {}).get('joint_state_age_sec'))} "
                f"close_delta_best={_fmt_scalar((close_wait_state or {}).get('close_delta_best'))}"
            )
            close_metrics = _close_alignment_metrics()
            close_metrics["close_confirmed"] = bool(close_confirmed)
            close_metrics["close_wait_state"] = _json_safe(close_wait_state)
            _final_phase_trace(
                "CLOSE",
                event="wait_done",
                expected="close_confirmed&&close_metrics_ok",
                received=str(bool(close_confirmed) and bool(close_metrics.get("ok"))).lower(),
                timeout_sec=f"{close_confirm_timeout_sec:.2f}",
                reason="gate_result",
                logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                physical_state=f"xy={_fmt_scalar(close_metrics.get('xy_dist'))},z={_fmt_scalar(close_metrics.get('z_error'))}",
            )
            _phase_end(
                "CLOSE",
                attach_state=_read_attach_state(),
                note=json.dumps(_json_safe(close_metrics), ensure_ascii=False, sort_keys=True),
                result="ok" if bool(close_confirmed) and bool(close_metrics.get("ok")) else "failed",
            )
            if not bool(close_confirmed):
                _emit_transition_decision(
                    from_phase="CLOSE",
                    to_phase="ATTACH_GATE",
                    decision="blocked",
                    reason="close_not_confirmed",
                    condition="close_confirmed",
                    metrics=close_metrics,
                )
                _abort_grasp(
                    code="CLOSE_NOT_CONFIRMED",
                    phase="CLOSE",
                    note="gripper close command did not reach measured closed state before attach",
                    metrics=close_metrics,
                )
            if not bool(close_metrics.get("ok")):
                _emit_transition_decision(
                    from_phase="CLOSE",
                    to_phase="ATTACH_GATE",
                    decision="blocked",
                    reason="close_metrics_not_ok",
                    condition="close_metrics_ok",
                    metrics=close_metrics,
                )
                _abort_grasp(
                    code="CLOSE_WITHOUT_OBJECT",
                    phase="CLOSE",
                    note="gripper closed but object not geometrically acquired",
                    metrics=close_metrics,
                )
            _emit_transition_decision(
                from_phase="CLOSE",
                to_phase="ATTACH_GATE",
                decision="enter",
                reason="close_confirmed_and_metrics_ok",
                condition="close_confirmed&&close_metrics_ok",
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
            panel._emit_log(
                "[PICK][DIRECT][ATTACH] "
                f"attach_result={str(bool(attach_ok)).lower()} "
                f"tcp={_fmt_vec(tcp_base_grasp)} obj={_fmt_vec(obj_base_grasp)} "
                f"tcp_obj_dist={_fmt_scalar(_dist(tcp_base_grasp, obj_base_grasp))} "
                f"expected_z_gap={float(GRIPPER_TCP_Z_OFFSET):.3f}"
            )
            if not attach_ok:
                _final_phase_trace(
                    "ATTACH_GATE",
                    event="attach_call_done",
                    expected="attach_ok",
                    received="false",
                    reason="attempt_attach_returned_false",
                    logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                )
                _phase_end("ATTACH_GATE", attach_state=_read_attach_state(), result="failed", note="attach gate returned false")
                raise RuntimeError("demo_attach_failed")
            demo_attach_published = True

            attach_follow_timeout_sec = max(
                1.2,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_ATTACH_SETTLE_SEC",
                        "1.8",
                    )
                    or 1.8
                ),
            )
            attach_follow_max_tcp_dist_m = max(
                0.12,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M",
                        "0.160",
                    )
                    or 0.160
                ),
            )
            _final_phase_trace(
                "ATTACH_GATE",
                event="wait_start",
                expected="follow_confirmed",
                received="pending",
                timeout_sec=f"{attach_follow_timeout_sec:.2f}",
                reason="wait_demo_attach_follow",
                request_state="attach_follow",
            )

            _wait_demo_attach_follow(
                timeout_sec=attach_follow_timeout_sec,
                max_tcp_dist_m=attach_follow_max_tcp_dist_m,
            )
            demo_follow_confirmed = True
            attach_metrics = _close_alignment_metrics()
            _final_phase_trace(
                "ATTACH_GATE",
                event="wait_done",
                expected="attach_ok&&follow_confirmed&&attach_metrics_ok",
                received=str(bool(attach_metrics.get("ok")) and demo_attach_published and demo_follow_confirmed).lower(),
                timeout_sec=f"{attach_follow_timeout_sec:.2f}",
                reason="gate_result",
                logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                physical_state=f"xy={_fmt_scalar(attach_metrics.get('xy_dist'))},z={_fmt_scalar(attach_metrics.get('z_error'))}",
            )
            panel._emit_log(
                "[PICK][DIRECT][ATTACH] "
                f"attach_follow_result=ok attach_published={str(bool(demo_attach_published)).lower()} "
                f"follow_confirmed={str(bool(demo_follow_confirmed)).lower()} "
                f"attach_state={json.dumps(_json_safe(_read_attach_state()), ensure_ascii=False, sort_keys=True)}"
            )
            post_attach_hold_sec = max(
                0.0,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_POST_ATTACH_HOLD_SEC",
                        "0.90",
                    )
                    or 0.90
                ),
            )
            if post_attach_hold_sec > 1e-4:
                panel._emit_log(
                    "[PICK][DIRECT][ATTACH] "
                    f"post_attach_hold wait_sec={post_attach_hold_sec:.2f} "
                    "reason=allow_backend_follow_lock"
                )
                time.sleep(post_attach_hold_sec)
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
            _final_phase_trace(
                "LIFT",
                event="wait_start",
                expected="post_grasp_lift_ok",
                received="pending",
                timeout_sec=f"{(move_sec + 8.0):.2f}",
                reason="move_tcp_direct",
                request_state="post_grasp_lift",
            )
            tcp_base_before_lift = _live_tcp_base()
            obj_world_before_lift = _live_object_world()
            obj_base_before_lift = _live_object_base()
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
            tcp_base_after_lift = _live_tcp_base()
            obj_world_after_lift = _live_object_world()
            obj_base_after_lift = _live_object_base()
            tcp_lift_delta = None
            obj_lift_delta = None
            tcp_obj_dist_after_lift = None
            if tcp_base_before_lift is not None and tcp_base_after_lift is not None:
                tcp_lift_delta = float(tcp_base_after_lift[2]) - float(tcp_base_before_lift[2])
            if obj_world_before_lift is not None and obj_world_after_lift is not None:
                obj_lift_delta = float(obj_world_after_lift[2]) - float(obj_world_before_lift[2])
            if tcp_base_after_lift is not None and obj_base_after_lift is not None:
                tcp_obj_dist_after_lift = _dist(tcp_base_after_lift, obj_base_after_lift)
            panel._emit_log(
                "[PICK][DIRECT][LIFT] "
                f"tcp_before={_fmt_vec(tcp_base_before_lift)} tcp_after={_fmt_vec(tcp_base_after_lift)} "
                f"obj_world_before={_fmt_vec(obj_world_before_lift)} obj_world_after={_fmt_vec(obj_world_after_lift)} "
                f"obj_base_after={_fmt_vec(obj_base_after_lift)} "
                f"tcp_lift_delta={_fmt_scalar(tcp_lift_delta)} obj_lift_delta={_fmt_scalar(obj_lift_delta)} "
                f"tcp_obj_dist_after={_fmt_scalar(tcp_obj_dist_after_lift)} "
                f"expected_z_gap={float(GRIPPER_TCP_Z_OFFSET):.3f}"
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
            _final_phase_trace(
                "LIFT",
                event="wait_done",
                expected="post_grasp_lift_ok",
                received="true",
                timeout_sec=f"{(move_sec + 8.0):.2f}",
                reason="lift_completed",
                logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                physical_state=f"tcp_lift={_fmt_scalar(tcp_lift_delta)},obj_lift={_fmt_scalar(obj_lift_delta)}",
            )
            _phase_begin(
                "CARRY",
                frame_used="world",
                offsets={
                    "min_obj_move_m": 0.030,
                    "min_lift_delta_m": 0.025,
                    "max_tcp_dist_m": 0.080,  # FIX-CARRY-VALIDATION: was 0.160
                },
                note="carry validation after lift",
            )
            _final_phase_trace(
                "CARRY",
                event="wait_start",
                expected="carry_validation_ok",
                received="pending",
                timeout_sec="1.60",
                reason="validate_demo_carry",
                request_state="carry_validation",
            )
            try:
                # FIX-CARRY-VALIDATION: max_tcp_dist_m tightened from 0.160 to 0.080 m.
                # 16 cm allowed the gripper to be far from the object during carry,
                # permitting false-positive "carry confirmed" even without real contact.
                # 8 cm still tolerates small offsets during lift while requiring proximity.
                _validate_demo_carry(
                    initial_obj_world=initial_obj_world,
                    phase="post_grasp_lift",
                    timeout_sec=1.6,
                    min_obj_move_m=0.030,
                    min_lift_delta_m=0.025,
                    max_tcp_dist_m=0.080,
                )
                _phase_end("CARRY", attach_state=_read_attach_state(), result="ok")
                _final_phase_trace(
                    "CARRY",
                    event="wait_done",
                    expected="carry_validation_ok",
                    received="true",
                    timeout_sec="1.60",
                    reason="carry_validation_pass",
                    logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                )
            except Exception as carry_exc:
                _phase_end("CARRY", attach_state=_read_attach_state(), result="failed", note=str(carry_exc))
                _final_phase_trace(
                    "CARRY",
                    event="wait_done",
                    expected="carry_validation_ok",
                    received="false",
                    timeout_sec="1.60",
                    reason=str(carry_exc),
                    logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                )
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

                def _open_gripper_short():
                    panel._command_gripper(False, log_action="DROP", force=True)

                _phase_begin(
                    "RELEASE",
                    frame_used="lift_pose",
                    offsets={
                        "drop_reason": "demo_short_release",
                        "mode": "short_lift_release",
                    },
                    note="open gripper and detach after validated short lift",
                )
                release_pre = _release_observation(
                    tag="pre_open",
                    reason="before short release open command",
                )
                panel.signal_run_ui.emit(_open_gripper_short)
                time.sleep(0.4)
                release_mark_ok = bool(
                    mark_object_released(
                        PICK_DEMO_OBJECT_NAME,
                        reason="demo_short_release_worker",
                    )
                )
                panel._emit_log(
                    "[PICK][DIRECT][RELEASE] "
                    f"mark_object_released={str(release_mark_ok).lower()} "
                    "reason=demo_short_release_worker"
                )
                _detach_demo_object("demo_short_release")
                demo_logical_attached = False
                release_wait_timeout = max(
                    0.8,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_RELEASE_WAIT_SEC",
                            "1.6",
                        )
                        or 1.6
                    ),
                )
                _final_phase_trace(
                    "RELEASE",
                    event="wait_start",
                    expected="detached&&owner_none&&released_state",
                    received="pending",
                    timeout_sec=f"{release_wait_timeout:.2f}",
                    reason="wait_detach_confirmation",
                    request_state="release_detach",
                )
                release_wait_deadline = time.monotonic() + release_wait_timeout
                release_retry_sent = False
                release_wait_ok = False
                while time.monotonic() < release_wait_deadline:
                    wait_attach = _json_safe(_read_attach_state()) or {}
                    wait_detached = not bool(wait_attach.get("logical_attached"))
                    wait_owner_none = str(wait_attach.get("owner") or "").upper() in {"", "NONE"}
                    wait_state_ok = str(wait_attach.get("logical_state") or "").upper() in {
                        "RELEASED",
                        "ON_TABLE",
                        "SPAWNED",
                        "",
                    }
                    if wait_detached and wait_owner_none and wait_state_ok:
                        release_wait_ok = True
                        break
                    if (not release_retry_sent) and ((time.monotonic() + 0.45) >= release_wait_deadline):
                        _detach_demo_object("demo_short_release_retry")
                        mark_object_released(PICK_DEMO_OBJECT_NAME, reason="demo_short_release_retry")
                        release_retry_sent = True
                    time.sleep(0.08)
                release_post = _release_observation(
                    tag="post_detach",
                    reason="after short release detach wait",
                )
                release_attach = _json_safe(release_post.get("attach_state")) or {}
                release_detached = not bool(release_attach.get("logical_attached"))
                release_owner_none = str(release_attach.get("owner") or "").upper() in {"", "NONE"}
                release_state_ok = str(release_attach.get("logical_state") or "").upper() in {
                    "RELEASED",
                    "ON_TABLE",
                    "SPAWNED",
                    "",
                }
                release_ok = bool(release_detached and release_owner_none and release_state_ok)
                _final_phase_trace(
                    "RELEASE",
                    event="wait_done",
                    expected="detached&&owner_none&&released_state",
                    received=str(release_ok).lower(),
                    timeout_sec=f"{release_wait_timeout:.2f}",
                    reason="release_validation_result",
                    logical_state=str(release_attach.get("logical_state") or "none"),
                    physical_state=f"dist_world={_fmt_scalar(release_post.get('dist_world'))}",
                )
                panel._emit_log(
                    "[PICK][DIRECT][RELEASE] "
                    f"validation={str(release_ok).lower()} wait_ok={str(release_wait_ok).lower()} "
                    f"mark_ok={str(release_mark_ok).lower()} detached={str(release_detached).lower()} "
                    f"owner_none={str(release_owner_none).lower()} state_ok={str(release_state_ok).lower()} "
                    f"dist_world={_fmt_scalar(release_post.get('dist_world'))} "
                    f"dist_base={_fmt_scalar(release_post.get('dist_base'))}"
                )
                _phase_end(
                    "RELEASE",
                    attach_state=_read_attach_state(),
                    note=json.dumps(
                        _json_safe(
                            {
                                "release_ok": release_ok,
                                "release_pre": release_pre,
                                "release_post": release_post,
                            }
                        ),
                        ensure_ascii=False,
                        sort_keys=True,
                    ),
                    result="ok" if release_ok else "warning",
                )
                _phase_begin(
                    "HOME_FINAL",
                    frame_used="base_link",
                    note="return home after short_release",
                )
                _final_phase_trace(
                    "HOME_FINAL",
                    event="wait_start",
                    expected="home_joint_reached",
                    received="pending",
                    timeout_sec="auto(move_sec+2)",
                    reason="run_joint_step_HOME_FINAL",
                    request_state="home_final",
                )
                home_final_ok = True
                home_final_note = "ok"
                _release_observation(
                    tag="before_home_final",
                    reason="starting HOME_FINAL after release",
                )
                try:
                    _run_joint_step("HOME_FINAL", home_pose)
                except Exception as home_final_exc:
                    home_final_ok = False
                    home_final_note = str(home_final_exc)
                    panel._emit_log(
                        f"[PICK][DEMO] HOME_FINAL warning after short_release: {home_final_exc}"
                    )
                _release_observation(
                    tag="after_home_final",
                    reason="HOME_FINAL completed",
                )
                _phase_end(
                    "HOME_FINAL",
                    attach_state=_read_attach_state(),
                    note=home_final_note,
                    result="ok" if home_final_ok else "warning",
                )
                _final_phase_trace(
                    "HOME_FINAL",
                    event="wait_done",
                    expected="home_joint_reached",
                    received=str(home_final_ok).lower(),
                    timeout_sec="auto(move_sec+2)",
                    reason=home_final_note,
                    logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                )
                panel._emit_log("[PICK][DIRECT] AVISO: TRAMO FINAL COMPLETADO route=short_release")
                panel._emit_log("[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE route=short_release")
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
            _final_phase_trace(
                "RELEASE",
                event="wait_start",
                expected="basket_release_done",
                received="pending",
                timeout_sec="1.40",
                reason="open_wait_close_wait",
                request_state="basket_release",
            )
            panel.signal_run_ui.emit(_open_and_release)
            time.sleep(1.0)
            panel._emit_log("[DEMO] Cerrando pinza en cesta")
            panel.signal_run_ui.emit(lambda: panel._command_gripper(True, log_action="DROP", force=True))
            time.sleep(0.4)
            _phase_end("RELEASE", attach_state=_read_attach_state(), result="ok")
            _final_phase_trace(
                "RELEASE",
                event="wait_done",
                expected="basket_release_done",
                received="true",
                timeout_sec="1.40",
                reason="basket_release_completed",
                logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
            )
            _phase_begin(
                "HOME_FINAL",
                frame_used="base_link",
                note="return home after basket release",
            )
            _final_phase_trace(
                "HOME_FINAL",
                event="wait_start",
                expected="home_joint_reached",
                received="pending",
                timeout_sec="auto(move_sec+2)",
                reason="run_joint_step_HOME_FINAL",
                request_state="home_final",
            )
            try:
                _run_joint_step("HOME_FINAL", home_pose)
            except Exception as home_final_exc:
                _phase_end(
                    "HOME_FINAL",
                    attach_state=_read_attach_state(),
                    note=str(home_final_exc),
                    result="failed",
                )
                _final_phase_trace(
                    "HOME_FINAL",
                    event="wait_done",
                    expected="home_joint_reached",
                    received="false",
                    timeout_sec="auto(move_sec+2)",
                    reason=str(home_final_exc),
                    logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                )
                raise
            _phase_end("HOME_FINAL", attach_state=_read_attach_state(), result="ok")
            _final_phase_trace(
                "HOME_FINAL",
                event="wait_done",
                expected="home_joint_reached",
                received="true",
                timeout_sec="auto(move_sec+2)",
                reason="home_final_completed",
                logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
            )
            panel._emit_log("[PICK][DIRECT] AVISO: TRAMO FINAL COMPLETADO route=basket")
            panel._emit_log("[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE route=basket")

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
