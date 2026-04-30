#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_execute.py
# Contenido: F3 — TFM execute flow (visualize, wait, execute, on_click, publish).
"""TFM execute flow extraído de panel_tfm.py (F3).

Cubre:

* ``tfm_visualize_grasp(panel)`` — comparar grasp/ref.
* ``wait_tfm_moveit_result(panel, ...)`` — espera del bridge MoveIt.
* ``execute_tfm_world_grasp(panel)`` — flujo grande de ejecución.
* ``on_tfm_grasp_object_clicked(panel)`` — handler del botón Agarre.

Re-exportado desde panel_tfm para preservar la API pública.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Optional

from .panel_pick_object import run_pick_object
from .panel_tfm_canonical import (
    build_tfm_pick_object_override,
    complete_pending_pick_demo_request,
    complete_pending_tfm_execute_request,
    tfm_canonical_finish,
    tfm_canonical_phase_update,
    tfm_canonical_state_reset,
    tfm_canonical_use_pick_object,
)
from .panel_tfm_params import get_panel_tfm_params as _get_panel_tfm_params



def tfm_visualize_grasp(panel):
    panel._log_button("TFM Comparar grasp/ref")
    panel._emit_log(
        "[TFM][BUTTON] action=visualize "
        f"applied={str(bool(getattr(panel, '_tfm_experiment_applied', False))).lower()} "
        f"has_grasp={str(bool(getattr(panel, '_last_grasp_px', None))).lower()} "
        f"selected={str(getattr(panel, '_selected_object', '') or 'none')}"
    )
    experiment_ready, experiment_reason = panel._tfm_experiment_ready_status()
    if not experiment_ready:
        panel._set_status(f"TFM bloqueado: {experiment_reason}", error=True)
        panel._audit_append("logs/visualize.log", f"[TFM] visualize FAIL reason={experiment_reason}")
        return False
    if not panel.tfm_module and not panel._last_grasp_px:
        panel._set_status("TFM no disponible", error=True)
        return False
    rep = None
    if panel.tfm_module:
        try:
            rep = panel.tfm_module.get_grasp_representation()
        except Exception as exc:
            panel._log_warning(f"[TFM] get_grasp_representation error: {exc}")
    if not rep and panel._last_grasp_px:
        rep = dict(panel._last_grasp_px)
    if not rep:
        panel._set_status("TFM: sin grasp para visualizar", error=True)
        return False
    ref = None
    if panel._last_camera_frame:
        _qimg, w, h, _ts = panel._last_camera_frame
        ref = panel._build_reference_grasp(w, h)
    if ref:
        panel._tfm_visual_compare_enabled = not bool(panel._tfm_visual_compare_enabled)
        mode_txt = "compare_on" if panel._tfm_visual_compare_enabled else "compare_off"
    else:
        panel._tfm_visual_compare_enabled = False
        mode_txt = "pred_only"
    panel._log(f"[TFM] Grasp actual: {rep} ref={ref} mode={mode_txt}")
    panel._refresh_grasp_overlay_now()
    overlay_path = panel._save_grasp_overlay()
    if overlay_path:
        panel._audit_append(
            "logs/visualize.log",
            f"[TFM] visualize OK mode={mode_txt} overlay={overlay_path}",
        )
    else:
        panel._audit_append("logs/visualize.log", f"[TFM] visualize FAIL mode={mode_txt} overlay=none")
    if panel._tfm_visual_compare_enabled:
        panel._set_status("TFM: comparación grasp/ref activada", error=False)
    elif ref:
        panel._set_status("TFM: comparación grasp/ref desactivada", error=False)
    else:
        panel._set_status("TFM: grasp visualizado (sin referencia)", error=False)
    return True

def wait_tfm_moveit_result(panel,
    label: str,
    *,
    since_wall: float,
    since_seq: int,
    timeout_sec: float,
    expected_request_id: Optional[int] = None,
    expected_request_uuid: str = "",
) -> Tuple[bool, str]:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        panel._motion_in_progress = False
        return False, "ros_worker_not_ready"
    started = time.time()
    deadline = started + max(0.2, float(timeout_sec))
    cursor_wall = float(since_wall)
    cursor_seq = int(since_seq)
    expected_uuid = str(expected_request_uuid or "").strip()
    result_topic = "/desired_grasp/result"
    grace_applied = False
    try:
        active_request_grace_sec = _get_panel_tfm_params().moveit_active_request_grace_sec
    except Exception:
        active_request_grace_sec = 90.0
    active_request_grace_sec = max(10.0, active_request_grace_sec)
    try:
        hb_recent_window_sec = _get_panel_tfm_params().moveit_active_request_hb_sec
    except Exception:
        hb_recent_window_sec = 2.5
    hb_recent_window_sec = max(1.2, hb_recent_window_sec)
    while time.time() < deadline:
        chunk = min(0.8, max(0.1, deadline - time.time()))
        ok, raw, wall, seq = panel.ros_worker.wait_for_moveit_result(
            since_wall=cursor_wall,
            since_seq=cursor_seq,
            timeout_sec=chunk,
        )
        if not ok:
            now = time.time()
            if now >= deadline:
                result_pubs = int(panel.ros_worker.topic_publisher_count(result_topic))
                result_subs = int(panel.ros_worker.topic_subscriber_count(result_topic))
                bridge_alive = bool(panel._proc_alive(getattr(panel, "moveit_bridge_proc", None)))
                hb_age = panel.ros_worker.moveit_bridge_heartbeat_age()
                hb_recent = panel.ros_worker.has_recent_moveit_bridge_heartbeat(hb_recent_window_sec)
                hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
                if (
                    not grace_applied
                    and result_pubs > 0
                    and result_subs > 0
                    and bridge_alive
                    and (
                        bool(hb_recent)
                        or (not math.isinf(hb_age) and hb_age <= max(5.0, hb_recent_window_sec * 2.0))
                    )
                ):
                    old_timeout = max(0.0, deadline - started)
                    deadline = now + active_request_grace_sec
                    grace_applied = True
                    panel._emit_log(
                        f"[TFM][MOVEIT][WAIT] {label} extend_wait old_timeout={old_timeout:.1f}s "
                        f"new_timeout={max(0.0, deadline - started):.1f}s grace={active_request_grace_sec:.1f}s "
                        f"bridge_alive={str(bridge_alive).lower()} "
                        f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
                    )
                    continue
            continue
        cursor_wall = max(cursor_wall, float(wall))
        cursor_seq = max(cursor_seq, int(seq))
        text = str(raw or "").strip()
        if not text:
            continue
        try:
            data = json.loads(text)
        except Exception:
            low = text.lower()
            success = ("success" in low) or ("ok" in low)
            # FASE 3: Liberar motion_in_progress.
            panel._motion_in_progress = False
            return success, text
        if expected_request_id is not None:
            got_id = int(data.get("request_id", -1) or -1)
            if got_id != int(expected_request_id):
                continue
        if expected_uuid:
            got_uid = str(data.get("request_uuid", "") or "")
            if got_uid != expected_uuid:
                continue
        success = bool(data.get("success", False))
        if not success and ("plan_ok" in data or "exec_ok" in data):
            success = bool(data.get("plan_ok", False) and data.get("exec_ok", False))
        if not success and "exec_ok" in data:
            success = bool(data.get("exec_ok", False))
        msg = str(data.get("message") or data.get("status") or "")
        if not msg:
            msg = text
        # FASE 3: Liberar motion_in_progress al obtener resultado.
        panel._motion_in_progress = False
        return success, msg
    # FASE 3: Liberar motion_in_progress al salir por timeout.
    panel._motion_in_progress = False
    return False, f"timeout_active_request>{max(0.0, deadline - started):.1f}s"

def execute_tfm_world_grasp(panel) -> bool:
    if not panel._last_grasp_px:
        return False
    if not panel._last_grasp_world and panel._last_camera_frame:
        _qimg, frame_w, frame_h, _ts = panel._last_camera_frame
        if frame_w > 0 and frame_h > 0:
            panel._last_grasp_world = panel._compute_world_grasp(frame_w, frame_h)
    if panel._last_grasp_base is None and panel._last_grasp_world is not None:
        panel._last_grasp_base = panel._world_grasp_to_base(panel._last_grasp_world)
    if not panel._last_grasp_base:
        panel._set_status("TFM: grasp base_link no disponible (cámara/calibración)", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM] execute FAIL reason=base_grasp_missing source={panel._last_grasp_source or 'unknown'}",
        )
        return False
    if panel._tfm_execute_inflight:
        panel._set_status("TFM: ejecución en curso", error=False)
        return False
    if not panel._moveit_required:
        panel._set_status("TFM: MoveIt no habilitado", error=True)
        panel._audit_append("logs/execute.log", "[TFM] execute FAIL reason=moveit_disabled")
        return False

    grasp_base = dict(panel._last_grasp_base)
    source = panel._last_grasp_source or "unknown"
    if tfm_canonical_use_pick_object(panel):
        selected_name = str(
            getattr(panel, "_selected_object", "") or getattr(panel, "_last_grasp_selection_name", "") or ""
        ).strip()
        grasp_selection = str(getattr(panel, "_last_grasp_selection_name", "") or "").strip()
        if not selected_name:
            panel._set_status("TFM: selección de objeto no disponible", error=True)
            panel._audit_append(
                "logs/execute.log",
                "[TFM] execute FAIL reason=selected_object_missing_for_canonical_route",
            )
            return False
        if grasp_selection and selected_name != grasp_selection:
            panel._set_status(
                f"TFM: grasp no corresponde a la selección actual ({grasp_selection} -> {selected_name})",
                error=True,
            )
            panel._audit_append(
                "logs/execute.log",
                "[TFM] execute FAIL reason=selection_grasp_mismatch "
                f"selected={selected_name} grasp_selection={grasp_selection}",
            )
            return False
        panel._tfm_execute_inflight = True
        panel._pick_object_grasp_override = build_tfm_pick_object_override(panel, 
            grasp_base=grasp_base,
            selected_object=selected_name,
            source=source,
        )
        tfm_canonical_state_reset(panel, 
            selected_object=selected_name,
            grasp_base=grasp_base,
            source=source,
        )
        tfm_canonical_phase_update(panel, "READY", detail="preconditions_ok")
        tfm_canonical_phase_update(panel, "OBJECT_SELECTED", detail=f"name={selected_name}")
        tfm_canonical_phase_update(panel, 
            "GRASP_FRESH",
            detail=f"source={source} age_sec={max(0.0, _runtime_time() - float(panel._last_grasp_update_ts or 0.0)):.2f}",
        )
        tfm_canonical_phase_update(panel, 
            "VISUAL_GRASP_OK",
            detail=f"topic={panel._grasp_rect_topic or '/grasp_rect'}",
        )
        tfm_canonical_phase_update(panel, 
            "EXECUTABLE_GRASP_OK",
            detail=f"pose_topic={MOVEIT_POSE_TOPIC} cartesian_topic={MOVEIT_CARTESIAN_POSE_TOPIC}",
        )
        setattr(panel, "_pick_object_worker_started", False)
        run_pick_object(panel)
        if not bool(getattr(panel, "_pick_object_worker_started", False)):
            tfm_canonical_finish(panel, 
                False,
                "tfm_canonical_pick_object_not_started",
                final_state="FAIL_TERMINAL",
            )
            return False
        return True

    panel._tfm_execute_inflight = True
    _minor_yaw_override = getattr(panel, "_tfm_grasp_minor_yaw_deg", None)
    _preopen_rad_override = getattr(panel, "_tfm_grasp_preopen_rad", None)
    _grasp_orientation_tag = "minor_axis" if _minor_yaw_override is not None else "direct_angle"
    panel._tfm_grasp_minor_yaw_deg = None
    panel._tfm_grasp_preopen_rad = None

    def _env_float(name: str, default: float) -> float:
        try:
            return float(os.environ.get(name, str(default)) or default)
        except Exception:
            return default

    def _next_tfm_request_id() -> int:
        seq = int(getattr(panel, "_tfm_moveit_request_id", 0) or 0) + 1
        setattr(panel, "_tfm_moveit_request_id", seq)
        return seq

    def _encode_request_frame(frame: str, request_id: int, request_uuid: str) -> str:
        base = str(frame or BASE_FRAME or "base_link").strip() or "base_link"
        uid = str(request_uuid or "").strip()
        if uid:
            return f"{base}|rid={int(request_id)}|uid={uid}"
        return f"{base}|rid={int(request_id)}"

    def _ros_clock_now_ns() -> int:
        try:
            if panel._ros_worker_started and panel.ros_worker.node_ready():
                with panel.ros_worker._lock:
                    node = getattr(panel.ros_worker, "_node", None)
                if node is not None:
                    now_ns = int(node.get_clock().now().nanoseconds)
                    if now_ns > 0:
                        return now_ns
        except Exception:
            pass
        return 0

    def _ensure_moveit_bridge_path(timeout_sec: float = 6.0) -> bool:
        if not panel._ros_worker_started:
            panel._ensure_ros_worker_started()
        if not panel.ros_worker.node_ready():
            raise RuntimeError("ros_worker_not_ready_for_moveit_bridge")

        pose_topic = MOVEIT_POSE_TOPIC
        result_topic = "/desired_grasp/result"

        def _path_ready() -> bool:
            pose_subs = int(panel.ros_worker.topic_subscriber_count(pose_topic))
            result_pubs = int(panel.ros_worker.topic_publisher_count(result_topic))
            return pose_subs > 0 and result_pubs > 0

        if _path_ready():
            return False

        panel._emit_log(
            "[TFM][MOVEIT] bridge_path_missing "
            f"pose_topic={pose_topic} result_topic={result_topic}; attempting_recover=true"
        )

        done = threading.Event()

        def _launch_bridge() -> None:
            try:
                panel._start_moveit_bridge()
            finally:
                done.set()

        panel.signal_run_ui.emit(_launch_bridge)
        done.wait(timeout=3.0)

        deadline = time.time() + max(1.0, float(timeout_sec))
        while time.time() < deadline:
            pose_subs = int(panel.ros_worker.topic_subscriber_count(pose_topic))
            result_pubs = int(panel.ros_worker.topic_publisher_count(result_topic))
            panel._emit_log(
                "[TFM][MOVEIT] bridge_recover_check "
                f"pose_subs={pose_subs} result_pubs={result_pubs}"
            )
            if pose_subs > 0 and result_pubs > 0:
                return True
            time.sleep(0.4)

        raise RuntimeError(
            "moveit_bridge_not_ready_after_recover "
            f"pose_topic={pose_topic} result_topic={result_topic}"
        )

    def worker() -> None:
        try:
            x = float(grasp_base.get("x", 0.0))
            y = float(grasp_base.get("y", 0.0))
            z_raw = float(grasp_base.get("z", 0.0))
            yaw_deg = (
                float(_minor_yaw_override) if _minor_yaw_override is not None
                else float(grasp_base.get("yaw_deg", 0.0) or 0.0)
            )
            proj_z_source = str(grasp_base.get("proj_z_source", "unknown") or "unknown")
            grasp_semantics = str(
                grasp_base.get("grasp_semantics", "projection_surface") or "projection_surface"
            )
            pretable_enabled = _get_panel_tfm_params().execute_pretable
            if pretable_enabled:
                move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
                panel._emit_log(
                    "[TFM][PRETABLE] yendo a MESA antes de ejecutar grasp"
                )
                ok_table, info_table = panel._publish_joint_trajectory(JOINT_TABLE_POSE_RAD, move_sec)
                if not ok_table:
                    raise RuntimeError(f"pretable_publish_failed:{info_table}")
                table_reached = panel._wait_for_joint_target(
                    JOINT_TABLE_POSE_RAD,
                    timeout_sec=max(6.0, move_sec + 4.0),
                    tol_rad=0.08,
                )
                if not table_reached:
                    raise RuntimeError("pretable_target_not_reached")
                panel._emit_log(
                    "[TFM][PRETABLE] MESA alcanzada; continuando con ejecución MoveIt"
                )
            table_top_world = float(panel._resolve_table_top_z())
            table_top_base = z_raw
            table_base = panel._ensure_base_coords(
                (float(TABLE_CENTER_X), float(TABLE_CENTER_Y), table_top_world),
                panel._world_frame_config_first(),
                timeout_sec=0.35,
            )
            if table_base is not None:
                table_top_base = float(table_base[2])
            min_margin = max(0.0, _env_float("PANEL_TFM_MIN_TABLE_MARGIN_M", 0.01))
            z_approach = max(0.12, min(0.20, _env_float("PANEL_PICK_Z_APPROACH_M", 0.14)))
            z_grasp_offset = _env_float("PANEL_PICK_Z_GRASP_OFFSET_M", 0.02)
            grasp_contact_z_offset = float(GRIPPER_TCP_Z_OFFSET)
            speed_scale = max(0.01, min(1.0, _env_float("PANEL_PICK_SPEED_SCALE", 0.25)))
            accel_scale = max(0.01, min(1.0, _env_float("PANEL_PICK_ACCEL_SCALE", 0.25)))
            bridge_request_timeout = max(
                2.0,
                _env_float("PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC", 35.0),
            )
            result_timeout = max(
                    10.0,
                    _env_float(
                        "PANEL_TFM_MOVEIT_RESULT_TIMEOUT_SEC",
                        bridge_request_timeout,
                    ),
                    bridge_request_timeout,
            )
            z_grasp = max(table_top_base + min_margin, z_raw + z_grasp_offset + grasp_contact_z_offset)
            z_pre = max(z_grasp + z_approach, table_top_base + min_margin + 0.02)
            z_retreat = z_pre
            panel._emit_log(
                "[TFM][GRASP_SEMANTICS] "
                f"source={source} visual_topic={panel._grasp_rect_topic or '/grasp_rect'} "
                f"semantics={grasp_semantics} proj_z_source={proj_z_source} "
                f"exec_seed_z={z_raw:.3f} table_top_base={table_top_base:.3f} exec_z={z_grasp:.3f}"
            )
            yaw_rad = math.radians(yaw_deg)
            orientation = (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))
            frame = panel._business_base_frame()
            tfm_grasp_cartesian = _get_panel_tfm_params().grasp_cartesian
            grasp_mode = "cartesian" if tfm_grasp_cartesian else "pose"
            pre_pose = _make_pose_data((x, y, z_pre), orientation=orientation, frame=frame)
            grasp_pose = _make_pose_data((x, y, z_grasp), orientation=orientation, frame=frame)
            retreat_pose = _make_pose_data((x, y, z_retreat), orientation=orientation, frame=frame)

            for pd in (pre_pose, grasp_pose, retreat_pose):
                pd["speed_scale"] = float(speed_scale)
                pd["accel_scale"] = float(accel_scale)

            has_results = False
            if panel._ros_worker_started and panel.ros_worker.node_ready():
                try:
                    has_results = bool(panel.ros_worker.subscribe_moveit_result("/desired_grasp/result"))
                except Exception:
                    has_results = False
            if has_results:
                _ensure_moveit_bridge_path()

            panel._audit_append(
                "logs/execute.log",
                f"[TFM] execute START source={source} base=({x:.3f},{y:.3f},{z_grasp:.3f}) "
                f"pre_z={z_pre:.3f} yaw={yaw_deg:.1f} frame={frame} wait_result={str(has_results).lower()} "
                f"semantics={grasp_semantics} proj_z_source={proj_z_source} z_seed={z_raw:.3f} "
                f"table_top_base={table_top_base:.3f} "
                f"z_approach={z_approach:.3f} z_grasp_offset={z_grasp_offset:+.3f} "
                f"speed_scale={speed_scale:.2f} accel_scale={accel_scale:.2f} "
                f"result_timeout={result_timeout:.1f} grasp_mode={grasp_mode}",
            )

            if _preopen_rad_override is not None:
                _pr = float(_preopen_rad_override)
                panel.signal_run_ui.emit(
                    lambda _r=_pr: panel._command_gripper_preopen(_r, log_action="PICK")
                )
            else:
                panel.signal_run_ui.emit(lambda: panel._command_gripper(False, log_action="PICK", force=True))
            panel._emit_log(
                "[PICK][MOVEIT] sequence=HOME->PREGRASP->DESCENT->GRASP->RETREAT "
                f"frame={frame} z_approach={z_approach:.3f} z_grasp_offset={z_grasp_offset:+.3f} "
                f"speed_scale={speed_scale:.2f} accel_scale={accel_scale:.2f} "
                f"grasp_mode={grasp_mode}"
            )
            time.sleep(0.35)

            since_wall = 0.0
            since_seq = -1
            if has_results:
                _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
            pre_request_id = _next_tfm_request_id()
            pre_request_uuid = uuid.uuid4().hex
            pre_pose_send = dict(pre_pose)
            pre_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
            pre_pose_send["frame"] = _encode_request_frame(frame, pre_request_id, pre_request_uuid)
            pose_subs_now = int(panel.ros_worker.topic_subscriber_count(MOVEIT_POSE_TOPIC)) if has_results else 1
            if pose_subs_now <= 0:
                raise RuntimeError(f"no_pose_subscribers_before_pregrasp topic={MOVEIT_POSE_TOPIC}")
            if not panel._publish_moveit_pose("TFM_PRE_GRASP", pre_pose_send, cartesian=False):
                raise RuntimeError("publish_pregrasp_failed")
            if has_results:
                ok_pre, msg_pre = wait_tfm_moveit_result(panel, 
                    "TFM_PRE_GRASP",
                    since_wall=since_wall,
                    since_seq=since_seq,
                    timeout_sec=result_timeout,
                    expected_request_id=pre_request_id,
                    expected_request_uuid=pre_request_uuid,
            )
                if not ok_pre:
                    raise RuntimeError(f"pregrasp_result_failed:{msg_pre}")
            else:
                time.sleep(0.8)

            since_wall = 0.0
            since_seq = -1
            if has_results:
                _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
            grasp_request_id = _next_tfm_request_id()
            grasp_request_uuid = uuid.uuid4().hex
            grasp_pose_send = dict(grasp_pose)
            grasp_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
            grasp_pose_send["frame"] = _encode_request_frame(frame, grasp_request_id, grasp_request_uuid)
            pose_subs_now = int(panel.ros_worker.topic_subscriber_count(MOVEIT_POSE_TOPIC)) if has_results else 1
            if pose_subs_now <= 0:
                raise RuntimeError(f"no_pose_subscribers_before_grasp topic={MOVEIT_POSE_TOPIC}")
            if _grasp_orientation_tag == "minor_axis":
                panel._emit_log(
                    "[TFM_GRASP][MOVEIT] publish topic=/desired_grasp/request "
                    "mode=moveit_sequence source=infer_model step=grasp_descent"
                )
            if not panel._publish_moveit_pose("TFM_GRASP", grasp_pose_send, cartesian=tfm_grasp_cartesian):
                raise RuntimeError("publish_grasp_failed")
            if has_results:
                if _grasp_orientation_tag == "minor_axis":
                    panel._emit_log(
                        "[TFM_GRASP][MOVEIT] waiting topic=/desired_grasp/result "
                        f"step=grasp_descent timeout_sec={result_timeout:.1f}"
                    )
                ok_grasp, msg_grasp = wait_tfm_moveit_result(panel, 
                    "TFM_GRASP",
                    since_wall=since_wall,
                    since_seq=since_seq,
                    timeout_sec=result_timeout,
                    expected_request_id=grasp_request_id,
                    expected_request_uuid=grasp_request_uuid,
                )
                if _grasp_orientation_tag == "minor_axis":
                    panel._emit_log(
                        f"[TFM_GRASP][MOVEIT] result={'OK' if ok_grasp else 'FAIL'} "
                        f"step=grasp_descent reason={msg_grasp or 'ok'}"
                    )
                if not ok_grasp:
                    if not tfm_grasp_cartesian:
                        raise RuntimeError(f"grasp_result_failed:{msg_grasp}")
                    msg_low = str(msg_grasp or "").lower()
                    reason = "unknown"
                    if "collision" in msg_low:
                        reason = "collision"
                    elif "ik" in msg_low:
                        reason = "ik"
                    elif "frame" in msg_low or "tf" in msg_low:
                        reason = "frame_mismatch"
                    elif "plan" in msg_low:
                        reason = "planning"
                    panel._emit_log(
                        "[PICK][MOVEIT] cartesian_descent_failed "
                        f"reason={reason} msg={msg_grasp}; fallback=planner"
                    )
                    panel._audit_append(
                        "logs/execute.log",
                        "[TFM] grasp cartesian_failed "
                        f"reason={reason} msg={msg_grasp}",
                    )
                    since_wall = 0.0
                    since_seq = -1
                    _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
                    fallback_request_id = _next_tfm_request_id()
                    fallback_request_uuid = uuid.uuid4().hex
                    fallback_pose_send = dict(grasp_pose)
                    fallback_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
                    fallback_pose_send["frame"] = _encode_request_frame(frame, fallback_request_id, fallback_request_uuid)
                    if not panel._publish_moveit_pose("TFM_GRASP_FALLBACK", fallback_pose_send, cartesian=False):
                        raise RuntimeError("publish_grasp_fallback_failed")
                    ok_fb, msg_fb = wait_tfm_moveit_result(panel, 
                        "TFM_GRASP_FALLBACK",
                        since_wall=since_wall,
                        since_seq=since_seq,
                        timeout_sec=result_timeout,
                        expected_request_id=fallback_request_id,
                        expected_request_uuid=fallback_request_uuid,
                    )
                    if not ok_fb:
                        raise RuntimeError(f"grasp_fallback_result_failed:{msg_fb}")
            else:
                time.sleep(0.8)

            panel.signal_run_ui.emit(lambda: panel._command_gripper(True, log_action="PICK", force=True))
            time.sleep(0.35)

            since_wall = 0.0
            since_seq = -1
            if has_results:
                _raw, since_wall, since_seq = panel.ros_worker.moveit_result_snapshot()
            retreat_request_id = _next_tfm_request_id()
            retreat_request_uuid = uuid.uuid4().hex
            retreat_pose_send = dict(retreat_pose)
            retreat_pose_send["stamp_ns"] = int(_ros_clock_now_ns())
            retreat_pose_send["frame"] = _encode_request_frame(frame, retreat_request_id, retreat_request_uuid)
            if not panel._publish_moveit_pose("TFM_RETREAT", retreat_pose_send, cartesian=False):
                raise RuntimeError("publish_retreat_failed")
            if has_results:
                ok_ret, msg_ret = wait_tfm_moveit_result(panel, 
                    "TFM_RETREAT",
                    since_wall=since_wall,
                    since_seq=since_seq,
                    timeout_sec=result_timeout,
                    expected_request_id=retreat_request_id,
                    expected_request_uuid=retreat_request_uuid,
            )
                if not ok_ret:
                    raise RuntimeError(f"retreat_result_failed:{msg_ret}")
            else:
                time.sleep(0.6)

            panel._audit_append(
                "logs/execute.log",
                f"[TFM] execute OK mode=moveit_sequence source={source} "
                f"target=({x:.3f},{y:.3f},{z_grasp:.3f}) pre=({x:.3f},{y:.3f},{z_pre:.3f}) "
                f"retreat=({x:.3f},{y:.3f},{z_retreat:.3f}) yaw={yaw_deg:.1f}",
            )
            if _grasp_orientation_tag == "minor_axis":
                panel._emit_log(
                    "[TFM_GRASP][EXECUTE] status=OK mode=moveit_sequence "
                    "source=infer_model grasp_orientation=minor_axis"
                )
                panel._ui_set_status(
                    "TFM: agarre completado (MoveIt + inferencia)", error=False
                )
            else:
                panel._ui_set_status("TFM: PREGRASP + DESCENT + GRASP + RETREAT ejecutados", error=False)
            complete_pending_tfm_execute_request(panel, 
                True,
                "PREGRASP + DESCENT + GRASP + RETREAT ejecutados",
            )
        except Exception as exc:
            panel._audit_append("logs/execute.log", f"[TFM] execute FAIL mode=moveit_sequence err={exc}")
            if _grasp_orientation_tag == "minor_axis":
                panel._emit_log(
                    f"[TFM_GRASP][EXECUTE] status=FAIL mode=moveit_sequence "
                    f"source=infer_model grasp_orientation=minor_axis reason={exc}"
                )
            panel._ui_set_status(f"TFM: ejecución fallida ({exc})", error=True)
            complete_pending_tfm_execute_request(panel, False, f"ejecución fallida ({exc})")
        finally:
            panel._tfm_execute_inflight = False

    panel._run_async(worker)
    return True

