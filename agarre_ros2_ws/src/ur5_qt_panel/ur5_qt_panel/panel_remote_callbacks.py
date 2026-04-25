#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_remote_callbacks.py
# Contenido: Remote request callbacks extracted from ControlPanelV2 in panel_v2.py.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Remote service/topic request callbacks for ControlPanelV2."""
from __future__ import annotations

import time


def _on_remote_camera_connect_request(panel, source: str) -> None:
    src = (source or "unknown").strip()
    panel._emit_log(f"[CAMERA][REMOTE] connect_trigger={src}")
    panel.signal_connect_camera.emit()

def _on_remote_camera_disconnect_request(panel, source: str) -> None:
    src = (source or "unknown").strip()
    panel._emit_log(f"[CAMERA][REMOTE] disconnect_trigger={src}")
    panel._unsubscribe_camera()
    panel._clear_camera_frame(reset_info=True)
    panel._set_status("Cámara desconectada (diagnóstico)", error=False)

def _on_remote_recover_request(panel, source: str) -> None:
    src = (source or "unknown").strip()
    panel._emit_log(f"[RECOVER][REMOTE] trigger={src}")
    panel._recover_runtime()

def _on_remote_tfm_infer_request(panel, source: str) -> None:
    src = (source or "unknown").strip()
    panel._emit_log(f"[TFM][REMOTE] trigger={src}")
    request_id = ""
    if src.startswith("service:") and "#" in src:
        request_id = src.rsplit("#", 1)[-1].strip()

    def _ack(success: bool, message: str) -> None:
        if not request_id:
            return
        panel._emit_log(
            f"[TFM][REMOTE][INFER_ACK] request_id={request_id} success={str(bool(success)).lower()} message={message or 'n/a'}"
        )
        if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
            try:
                panel.ros_worker.complete_tfm_infer_request(request_id, success, message)
            except Exception:
                pass

    infer_ready, infer_reason = panel._tfm_infer_ready_status()
    if (
        not infer_ready
        and str(infer_reason or "").strip().lower() == "release de objetos pendiente"
        and not bool(getattr(panel, "_detach_inflight", False))
    ):
        panel._emit_log("[TFM][REMOTE] release pendiente; lanzando Soltar objetos")
        panel._release_objects()
    if request_id and not infer_ready and panel._tfm_infer_waitable_reason(infer_reason):
        wait_sec = max(
            2.0,
            float(os.environ.get("PANEL_TFM_REMOTE_INFER_READY_WAIT_SEC", "20.0") or 20.0),
        )
        poll_sec = max(
            0.1,
            float(os.environ.get("PANEL_TFM_REMOTE_INFER_READY_POLL_SEC", "0.25") or 0.25),
        )
        panel._tfm_infer_pending_request_id = request_id
        panel._emit_log(
            "[TFM][REMOTE][INFER_ACK] "
            f"request_id={request_id} deferred=true waiting_ready=true "
            f"reason={infer_reason or 'n/a'} wait_sec={wait_sec:.1f}"
        )

        def _resume_infer_when_ready() -> None:
            deadline = time.time() + wait_sec
            last_reason = infer_reason
            while time.time() < deadline:
                ready, reason = panel._tfm_infer_ready_status()
                last_reason = reason
                if ready:
                    break
                if not panel._tfm_infer_waitable_reason(reason):
                    panel.signal_run_ui.emit(
                        lambda reason=reason: panel._complete_pending_tfm_infer_request(
                            False, str(reason or "n/a")
                        )
                    )
                    return
                time.sleep(poll_sec)

            def _run_infer_now() -> None:
                ready_now, reason_now = panel._tfm_infer_ready_status()
                if not ready_now:
                    panel._complete_pending_tfm_infer_request(
                        False,
                        str(reason_now or last_reason or "n/a"),
                    )
                    return
                ok_now, msg_now = panel._tfm_infer_grasp()
                if not bool(ok_now) or not bool(getattr(panel, "_tfm_infer_inflight", False)):
                    panel._complete_pending_tfm_infer_request(bool(ok_now), str(msg_now or "n/a"))

            panel.signal_run_ui.emit(_run_infer_now)

        panel._run_async(_resume_infer_when_ready, name="remote_tfm_infer_wait")
        return

    ok, message = panel._tfm_infer_grasp()
    if bool(ok) and request_id and bool(getattr(panel, "_tfm_infer_inflight", False)):
        panel._tfm_infer_pending_request_id = request_id
        panel._emit_log(
            f"[TFM][REMOTE][INFER_ACK] request_id={request_id} deferred=true pending_result=true"
        )
        return
    _ack(bool(ok), str(message or "n/a"))

def _on_remote_tfm_execute_request(panel, source: str) -> None:
    src = (source or "unknown").strip()
    panel._emit_log(f"[TFM][REMOTE] execute_trigger={src}")
    request_id = ""
    if src.startswith("service:") and "#" in src:
        request_id = src.rsplit("#", 1)[-1].strip()

    def _ack(success: bool, message: str) -> None:
        if not request_id:
            return
        panel._emit_log(
            f"[TFM][REMOTE][EXEC_ACK] request_id={request_id} success={str(bool(success)).lower()} message={message or 'n/a'}"
        )
        if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
            try:
                panel.ros_worker.complete_tfm_execute_request(request_id, success, message)
            except Exception:
                pass

    basic_ok, basic_reason = panel._basic_ready_status()
    if request_id and not basic_ok:
        wait_sec = max(
            2.0,
            float(os.environ.get("PANEL_TFM_REMOTE_EXECUTE_READY_WAIT_SEC", "90.0") or 90.0),
        )
        poll_sec = max(
            0.5,
            float(os.environ.get("PANEL_TFM_REMOTE_EXECUTE_READY_POLL_SEC", "1.0") or 1.0),
        )
        panel._emit_log(
            "[TFM][REMOTE][EXEC_ACK] "
            f"request_id={request_id} deferred=true waiting_basic=true "
            f"reason={basic_reason or 'n/a'} wait_sec={wait_sec:.1f}"
        )
        panel._tfm_execute_pending_request_id = request_id

        def _resume_execute_when_ready() -> None:
            deadline = time.time() + wait_sec
            last_reason = basic_reason
            while time.time() < deadline:
                ok, reason = panel._basic_ready_status()
                last_reason = reason
                if ok:
                    break
                time.sleep(poll_sec)

            def _run_execute_now() -> None:
                ok_now, msg_now = panel._tfm_publish_grasp()
                if bool(ok_now) and bool(getattr(panel, "_tfm_execute_inflight", False)):
                    return
                if getattr(panel, "_tfm_execute_pending_request_id", "") == request_id:
                    panel._tfm_execute_pending_request_id = ""
                _ack(bool(ok_now), str(msg_now or last_reason or "n/a"))

            panel.signal_run_ui.emit(_run_execute_now)

        panel._run_async(_resume_execute_when_ready, name="remote_tfm_execute_wait")
        return

    ok, message = panel._tfm_publish_grasp()
    if bool(ok) and request_id and bool(getattr(panel, "_tfm_execute_inflight", False)):
        panel._tfm_execute_pending_request_id = request_id
        panel._emit_log(
            f"[TFM][REMOTE][EXEC_ACK] request_id={request_id} deferred=true pending_result=true"
        )
        return
    _ack(bool(ok), str(message or "n/a"))

def _on_remote_pick_demo_request(panel, source: str) -> None:
    src = (source or "unknown").strip()
    panel._emit_log(f"[PICK][REMOTE][DIRECT] trigger={src}")
    request_id = ""
    if src.startswith("service:") and "#" in src:
        request_id = src.rsplit("#", 1)[-1].strip()

    def _trace_remote_pick(stage: str, *, ready: bool, reason: str, waitable: bool) -> None:
        manual_ok, manual_reason = panel._manual_control_status()
        panel._emit_log(
            "[PICK][REMOTE][TRACE] "
            f"stage={stage} "
            f"ready={str(bool(ready)).lower()} "
            f"reason={reason or 'ok'} "
            f"waitable={str(bool(waitable)).lower()} "
            f"script_active={str(bool(getattr(panel, '_script_motion_active', False))).lower()} "
            f"manual_inflight={str(bool(getattr(panel, '_manual_inflight', False))).lower()} "
            f"manual_ok={str(bool(manual_ok)).lower()} "
            f"manual_reason={manual_reason or 'ok'}"
        )

    def _ack(success: bool, message: str) -> None:
        if not request_id:
            return
        panel._emit_log(
            f"[PICK][REMOTE][DIRECT_ACK] request_id={request_id} success={str(bool(success)).lower()} message={message or 'n/a'}"
        )
        if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
            try:
                panel.ros_worker.complete_pick_demo_request(request_id, success, message)
            except Exception:
                pass

    ready, reason, waitable = panel._pick_demo_remote_ready_status()
    _trace_remote_pick("entry", ready=ready, reason=str(reason or ""), waitable=waitable)
    if request_id and not ready and waitable:
        wait_sec = max(
            2.0,
            float(os.environ.get("PANEL_PICK_DEMO_REMOTE_READY_WAIT_SEC", "90.0") or 90.0),
        )
        poll_sec = max(
            0.1,
            float(os.environ.get("PANEL_PICK_DEMO_REMOTE_READY_POLL_SEC", "0.5") or 0.5),
        )
        panel._pick_demo_pending_request_id = request_id
        panel._emit_log(
            "[PICK][REMOTE][DIRECT_ACK] "
            f"request_id={request_id} deferred=true waiting_ready=true "
            f"reason={reason or 'n/a'} wait_sec={wait_sec:.1f}"
        )

        def _resume_pick_demo_when_ready() -> None:
            deadline = time.time() + wait_sec
            last_reason = reason
            while time.time() < deadline:
                ready_now, reason_now, waitable_now = panel._pick_demo_remote_ready_status()
                _trace_remote_pick(
                    "wait_loop",
                    ready=ready_now,
                    reason=str(reason_now or ""),
                    waitable=waitable_now,
                )
                last_reason = reason_now
                if ready_now:
                    break
                if not waitable_now:
                    panel.signal_run_ui.emit(
                        lambda reason=reason_now: panel._complete_pending_pick_demo_request(
                            False, str(reason or "n/a")
                        )
                    )
                    return
                time.sleep(poll_sec)

            def _run_pick_demo_now() -> None:
                ready_now, reason_now, _waitable_now = panel._pick_demo_remote_ready_status()
                _trace_remote_pick(
                    "before_run_pick_demo",
                    ready=ready_now,
                    reason=str(reason_now or ""),
                    waitable=_waitable_now,
                )
                if not ready_now:
                    panel._complete_pending_pick_demo_request(
                        False,
                        str(reason_now or last_reason or "n/a"),
                    )
                    return
                # Skip confirm dialog for remote invocations (headless context)
                panel._emit_log("[PICK][REMOTE] Ejecutando pick_demo sin diálogo de confirmación")
                run_pick_demo(panel)
                ready_after, reason_after, waitable_after = panel._pick_demo_remote_ready_status()
                _trace_remote_pick(
                    "after_run_pick_demo",
                    ready=ready_after,
                    reason=str(reason_after or ""),
                    waitable=waitable_after,
                )
                if bool(getattr(panel, "_script_motion_active", False)):
                    panel._complete_pending_pick_demo_request(True, "pick_demo_started")
                    return
                panel._complete_pending_pick_demo_request(
                    False,
                    str(reason_after or "pick_demo_no_iniciado"),
                )

            panel.signal_run_ui.emit(_run_pick_demo_now)

        panel._run_async(_resume_pick_demo_when_ready, name="remote_pick_demo_wait")
        return

    if request_id and not ready:
        _trace_remote_pick("immediate_reject", ready=ready, reason=str(reason or ""), waitable=waitable)
        _ack(False, str(reason or "n/a"))
        return

    # Skip confirm dialog for remote invocations (headless context)
    panel._emit_log("[PICK][REMOTE] Ejecutando pick_demo sin diálogo de confirmación")
    run_pick_demo(panel)
    ready_after, reason_after, waitable_after = panel._pick_demo_remote_ready_status()
    _trace_remote_pick(
        "after_direct_run_pick_demo",
        ready=ready_after,
        reason=str(reason_after or ""),
        waitable=waitable_after,
    )
    if request_id:
        if bool(getattr(panel, "_script_motion_active", False)):
            _ack(True, "pick_demo_started")
        else:
            _ack(False, str(reason_after or "pick_demo_no_iniciado"))

def _on_remote_pick_object_request(panel, source: str) -> None:
    src = (source or "unknown").strip()
    panel._emit_log(f"[PICK][REMOTE] trigger={src}")
    panel._run_pick_object()

def _on_remote_object_select_request(panel, name: str, source: str) -> None:
    src = (source or "unknown").strip()
    target = (name or "").strip()
    panel._emit_log(f"[PICK][REMOTE] trigger={src} name={target or 'clear'}")
    request_id = ""
    if src.startswith("service:") and "#" in src:
        request_id = src.rsplit("#", 1)[-1].strip()

    def _ack(success: bool, message: str) -> None:
        if not request_id:
            return
        panel._emit_log(
            f"[PICK][REMOTE][ACK] request_id={request_id} success={str(bool(success)).lower()} message={message or 'n/a'}"
        )
        if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
            try:
                panel.ros_worker.complete_object_select_request(request_id, success, message)
            except Exception:
                pass

    def _defer_select_until_on_table(reason: str) -> bool:
        if not request_id or not target:
            return False
        wait_sec = max(
            2.0,
            float(os.environ.get("PANEL_REMOTE_SELECT_ON_TABLE_WAIT_SEC", "8.0") or 8.0),
        )
        poll_sec = max(
            0.1,
            float(os.environ.get("PANEL_REMOTE_SELECT_ON_TABLE_POLL_SEC", "0.25") or 0.25),
        )
        panel._emit_log(
            "[PICK][REMOTE][ACK] "
            f"request_id={request_id} deferred=true waiting_select=true "
            f"reason={reason or 'n/a'} wait_sec={wait_sec:.1f}"
        )

        def _wait_and_select() -> None:
            deadline = time.time() + wait_sec
            last_xyz = None
            while time.time() < deadline:
                pose = None
                if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
                    try:
                        poses, _ts = panel.ros_worker.pose_snapshot()
                    except Exception:
                        poses = {}
                    pose = poses.get(target)
                if pose is None:
                    pose = get_object_position(target)
                if pose is not None and len(pose) >= 3:
                    try:
                        xyz = (float(pose[0]), float(pose[1]), float(pose[2]))
                    except Exception:
                        xyz = None
                    if xyz is not None:
                        last_xyz = xyz
                        if is_on_table(xyz):
                            break
                time.sleep(poll_sec)

            def _select_now() -> None:
                xyz = last_xyz
                if (xyz is None or not is_on_table(xyz)) and target:
                    pose = get_object_position(target)
                    if pose is not None and len(pose) >= 3:
                        try:
                            xyz = (float(pose[0]), float(pose[1]), float(pose[2]))
                        except Exception:
                            xyz = None
                if xyz is not None and is_on_table(xyz):
                    x, y, z = xyz
                    bulk_update_object_positions(
                        {target: (x, y, z)},
                        source="remote_pose_select_retry",
                        objects_stable=True,
                    )
                    recalc_object_states("remote_select_pose_retry_sync")
                    px, py = -1, -1
                    w = getattr(panel.camera_view, "_img_width", 0) if hasattr(panel, "camera_view") else 0
                    h = getattr(panel.camera_view, "_img_height", 0) if hasattr(panel, "camera_view") else 0
                    if panel._camera_stream_ok and panel._pose_info_ok and w > 0 and h > 0:
                        pix = world_xyz_to_pixel(x, y, z, w, h) or table_xy_to_pixel(x, y, w, h)
                        if pix:
                            px, py = int(pix[0]), int(pix[1])
                    panel._select_object(target, px, py, x, y, z, source="remote_pose_retry")
                if getattr(panel, "_selected_object", None) == target:
                    _ack(True, "selected")
                else:
                    _ack(False, f"selection_rejected:{target}")

            panel.signal_run_ui.emit(_select_now)

        panel._run_async(_wait_and_select, name="remote_select_wait")
        return True

    def _trigger_pick_demo_recover(reason: str) -> None:
        if target != PICK_DEMO_OBJECT_NAME:
            return
        if panel._pick_demo_recover_inflight:
            return
        state = get_object_state(PICK_DEMO_OBJECT_NAME)
        if state is not None and (state.owner != ObjectOwner.NONE or state.attached):
            return
        panel._emit_log(
            "[PICK_DEMO][RECOVER] "
            f"trigger=remote_select reason={reason or 'n/a'}"
        )
        panel._pick_demo_recover_inflight = True

        def _worker() -> None:
            try:
                panel._recover_pick_demo_to_table(reason or "remote_select")
            finally:
                panel._pick_demo_recover_inflight = False

        panel._run_async(_worker, name="pick_demo_recover_remote")

    if target.lower() in ("", "clear", "none", "null"):
        if panel._selected_object:
            prev_state = get_object_state(panel._selected_object)
            if prev_state and prev_state.logical_state == ObjectLogicalState.SELECTED:
                update_object_state(
                    panel._selected_object,
                    logical_state=ObjectLogicalState.ON_TABLE,
                    reason="remote_clear",
            )
        panel._selected_object = None
        panel._selected_px = None
        panel._selected_world = None
        panel._selected_base = None
        panel._selection_timestamp = 0.0
        panel._selection_last_user_name = ""
        panel._selection_last_user_ts = 0.0
        panel._last_cornell = None
        panel._last_cornell_reason = "Selección limpiada remotamente"
        panel._refresh_science_ui()
        panel._update_objects()
        panel._set_status("Selección remota limpiada", error=False)
        _ack(True, "selection_cleared")
        return
    if not bool(getattr(panel, "_objects_release_done", False)) and not bool(
        getattr(panel, "_detach_inflight", False)
    ):
        panel._emit_log("[PICK][REMOTE] release pendiente; lanzando Soltar objetos")
        panel._release_objects()
    live_pose = None
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            poses, _ts = panel.ros_worker.pose_snapshot()
        except Exception:
            poses = {}
        live_pose = poses.get(target)
    if live_pose is not None and len(live_pose) >= 3:
        x, y, z = float(live_pose[0]), float(live_pose[1]), float(live_pose[2])
        if not is_on_table((x, y, z)):
            _trigger_pick_demo_recover("remote_select_live_not_on_table")
            if _defer_select_until_on_table("remote_pose_not_on_table_yet"):
                return
        if is_on_table((x, y, z)):
            bulk_update_object_positions(
                {target: (x, y, z)},
                source="remote_pose_select",
                objects_stable=True,
            )
            recalc_object_states("remote_select_pose_sync")
        px, py = -1, -1
        w = getattr(panel.camera_view, "_img_width", 0) if hasattr(panel, "camera_view") else 0
        h = getattr(panel.camera_view, "_img_height", 0) if hasattr(panel, "camera_view") else 0
        if panel._camera_stream_ok and panel._pose_info_ok and w > 0 and h > 0:
            pix = world_xyz_to_pixel(x, y, z, w, h) or table_xy_to_pixel(x, y, w, h)
            if pix:
                px, py = int(pix[0]), int(pix[1])
        panel._select_object(target, px, py, x, y, z, source="remote_pose")
        if getattr(panel, "_selected_object", None) == target:
            _ack(True, "selected")
        else:
            if _defer_select_until_on_table("selection_rejected_transient"):
                return
            _ack(False, f"selection_rejected:{target}")
        return
    objects = get_object_positions() or {}
    obj_pose = objects.get(target)
    if obj_pose is not None and len(obj_pose) >= 3:
        x, y, z = float(obj_pose[0]), float(obj_pose[1]), float(obj_pose[2])
        if not is_on_table((x, y, z)):
            _trigger_pick_demo_recover("remote_select_store_not_on_table")
            if _defer_select_until_on_table("remote_store_not_on_table"):
                return
        px, py = -1, -1
        w = getattr(panel.camera_view, "_img_width", 0) if hasattr(panel, "camera_view") else 0
        h = getattr(panel.camera_view, "_img_height", 0) if hasattr(panel, "camera_view") else 0
        if panel._camera_stream_ok and panel._pose_info_ok and w > 0 and h > 0:
            pix = world_xyz_to_pixel(x, y, z, w, h) or table_xy_to_pixel(x, y, w, h)
            if pix:
                px, py = int(pix[0]), int(pix[1])
        panel._select_object(target, px, py, x, y, z, source="remote_store")
    else:
        # Sin pose conocida: diferir hasta que Gazebo publique posición sobre mesa
        if _defer_select_until_on_table("remote_no_pose"):
            return
        panel._on_object_clicked(target)
    if getattr(panel, "_selected_object", None) == target:
        _ack(True, "selected")
    else:
        _ack(False, f"selection_rejected:{target}")
