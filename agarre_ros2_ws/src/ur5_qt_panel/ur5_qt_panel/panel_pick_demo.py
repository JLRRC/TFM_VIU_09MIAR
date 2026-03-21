#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Pick demo sequence helper for the panel."""
from __future__ import annotations

import os
import math
import time
import uuid

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
    GRIPPER_TCP_Z_OFFSET,
    UR5_JOINT_NAMES,
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
                # ur5_kinematics opera en un frame numerico distinto del tcp_base del panel.
                # Aplicamos la correccion en coordenadas relativas al pose semilla, que si es consistente.
                target_ik = (
                    float(seed_pos[0]) - float(delta_runtime[0]),
                    float(seed_pos[1]) - float(delta_runtime[1]),
                    float(seed_pos[2]) + float(delta_runtime[2]),
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
                    f"target_ik=({target_ik[0]:.3f},{target_ik[1]:.3f},{target_ik[2]:.3f}) "
                    f"err_norm={float(err_norm):.4f} success={str(bool(ik_ok)).lower()}"
                )
                if (not ik_ok) or float(err_norm) > 0.035:
                    raise RuntimeError(
                        f"{label.lower()}_ik_failed err_norm={float(err_norm):.4f}"
                    )
                _run_joint_step(
                    label,
                    [float(v) for v in solved_q.tolist()],
                    timeout_sec=max(float(timeout_sec), move_sec + 2.0),
                    tol_rad=0.08,
                )
                tcp_after = _live_tcp_base()
                if tcp_after is not None:
                    panel._emit_log(
                        "[PICK][DIRECT][IK] "
                        f"label={label} "
                        f"tcp_after=({tcp_after[0]:.3f},{tcp_after[1]:.3f},{tcp_after[2]:.3f})"
                    )

            def _align_demo_grasp_direct() -> None:
                obj_base = _live_object_base()
                if obj_base is None:
                    raise RuntimeError("demo_object_pose_unavailable_before_align")
                target_tcp_runtime = (
                    float(obj_base[0]),
                    float(obj_base[1]),
                    float(obj_base[2]) + float(GRIPPER_TCP_Z_OFFSET),
                )
                _move_tcp_direct(
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
                _move_tcp_direct(
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
            _run_joint_step(
                "GRASP_DOWN_JOINT",
                JOINT_GRASP_DOWN_POSE_RAD,
                timeout_sec=move_sec + 6.0,
                tol_rad=0.08,
            )
            _align_demo_grasp_direct()
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
            panel._emit_log("[DEMO] Cerrando pinza")
            def _close_only():
                panel._command_gripper(True, log_action="PICK", force=True)

            panel.signal_run_ui.emit(_close_only)
            time.sleep(0.7)
            tcp_base_grasp = _live_tcp_base()
            obj_base_grasp = _live_object_base()
            if tcp_base_grasp is None or obj_base_grasp is None:
                panel._emit_log(
                    "[PICK][DIRECT][GEOM] "
                    f"tcp_base={'ok' if tcp_base_grasp is not None else 'none'} "
                    f"obj_base={'ok' if obj_base_grasp is not None else 'none'}"
                )
                raise RuntimeError("demo_attach_geometry_unavailable")
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
            _lift_demo_object_direct(
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
            _validate_demo_carry(
                initial_obj_world=initial_obj_world,
                phase="post_grasp_lift",
                timeout_sec=1.6,
                min_obj_move_m=0.030,
                min_lift_delta_m=0.025,
                max_tcp_dist_m=0.160,
            )
            if not mark_object_grasped(PICK_DEMO_OBJECT_NAME, reason="demo_physical_lift_ok"):
                raise RuntimeError("demo_mark_grasped_failed")
            if not mark_object_attached(PICK_DEMO_OBJECT_NAME, reason="demo_physical_lift_ok"):
                raise RuntimeError("demo_mark_attached_failed")
            demo_logical_attached = True
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

            panel.signal_run_ui.emit(_open_and_release)
            time.sleep(1.0)
            panel._emit_log("[DEMO] Cerrando pinza en cesta")
            panel.signal_run_ui.emit(lambda: panel._command_gripper(True, log_action="DROP", force=True))
            time.sleep(0.4)

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
