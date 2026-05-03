#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/wait_moveit_result.py
# Contenido: F3-step4a — extracción de _wait_moveit_result (253 LOC).
"""Wait MoveIt result extraído del closure run_pick_object.worker.

``wait_moveit_result`` espera el resultado del último goal MoveIt
publicado en el topic ``moveit_result_topic`` (default
``/ur5_moveit_bridge/last_result``), parseando el JSON entregado
por el bridge y devolviendo un dict normalizado.

Antes de F3-step4a vivía como ``def`` anidada de 253 LOC dentro del
closure ``run_pick_object.worker``, capturando 2 deps reales:
``panel`` (para _emit_log + _last_pose_status) y ``moveit_result_topic``
(string del topic configurable). 15 callsites en el closure.

Aquí se extrae con el patrón estándar dataclass + función pura +
wrapper delgado en el closure. Los 14 callsites legacy NO cambian.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any


@dataclass
class WaitMoveItResultContext:
    """Captura de dependencias closure de wait_moveit_result."""

    panel: Any
    moveit_result_topic: str


def wait_moveit_result(
    ctx: WaitMoveItResultContext,
    label: str,
    since_wall: float,
    *,
    timeout_total: float = 12.0,
) -> dict:
    """Espera el resultado del bridge MoveIt para `label` y devuelve dict.

    Devuelve ``{}`` si timeout o si el JSON no llegó/parseó.
    """
    timeout_total = max(0.5, float(timeout_total))
    started = time.time()
    deadline = started + timeout_total
    raw = ""
    ok = False
    last_diag = 0.0
    cursor_wall = since_wall
    cursor_seq = int(getattr(_wait_moveit_result, "_since_seq", -1) or -1)
    lost_pub_since = None
    wait_extended = False
    last_seen_request_id = -1
    last_seen_request_uuid = ""
    try:
        _grace_raw = _get_pick_object_params().moveit_active_request_grace_sec
    except Exception:
        _grace_raw = None
    active_request_grace_sec = _clamp_grace_window(_grace_raw, floor=10.0, default=90.0)
    try:
        _hb_raw = _get_pick_object_params().moveit_active_request_hb_sec
    except Exception:
        _hb_raw = None
    hb_recent_window_sec = _clamp_grace_window(_hb_raw, floor=1.2, default=2.5)
    # Contract matching: accept ONLY the expected request_id for this publish.
    expected_request_id = int(getattr(_wait_moveit_result, "_expected_request_id", -1) or -1)
    expected_stamp_ns = int(getattr(_wait_moveit_result, "_expected_stamp_ns", 0) or 0)
    expected_request_uuid = str(getattr(_wait_moveit_result, "_expected_request_uuid", "") or "")
    panel_request_id = int(getattr(_wait_moveit_result, "_panel_request_id", -1) or -1)
    ctx.panel._emit_log(
        _fmt_wait_state_log(
            state="enter",
            label=label,
            elapsed_sec=0.0,
            expected_id=expected_request_id,
            expected_uuid=expected_request_uuid,
            last_seen_id=last_seen_request_id,
            last_seen_uuid=last_seen_request_uuid,
        )
    )
    while time.time() < deadline:
        wait_chunk = _compute_wait_chunk_sec(deadline, time.time())
        ok, raw, _wall, _seq = ctx.panel.ros_worker.wait_for_moveit_result(
            since_wall=cursor_wall,
            since_seq=cursor_seq,
            timeout_sec=wait_chunk,
        )
        if ok:
            try:
                data = json.loads(raw)
            except Exception:
                cursor_wall = max(cursor_wall, _wall)
                cursor_seq = max(cursor_seq, int(_seq))
                continue
            req_id = int(data.get("request_id", -1) or -1)
            got_stamp_ns = int(data.get("target_stamp_ns", 0) or 0)
            got_uuid = str(data.get("request_uuid", "") or "")
            last_seen_request_id = req_id
            last_seen_request_uuid = got_uuid
            rcv_wall_us = int(time.time() * 1_000_000)
            pub_wall_us = int(getattr(_wait_moveit_result, "_pub_wall_us", 0) or 0)
            wall_us_delta = max(0, rcv_wall_us - pub_wall_us)
            ctx.panel._emit_log(
                f"[PICK_OBJ][RESULT_DIAGNOSTIC] label={label} got_request_id={req_id} "
                f"expected_request_id={expected_request_id} rcv_wall_us={rcv_wall_us} "
                f"pub_wall_us={pub_wall_us} delta_us={wall_us_delta} "
                f"got_uuid={got_uuid or 'n/a'} expected_uuid={expected_request_uuid or 'n/a'}"
            )
            ctx.panel._emit_log(
                f"[PANEL][RESULT_RX] label={label} got_request_id={req_id} "
                f"expected_request_id={expected_request_id} got_stamp={got_stamp_ns} "
                f"expected_stamp={expected_stamp_ns} got_uuid={got_uuid or 'n/a'} "
                f"expected_uuid={expected_request_uuid or 'n/a'}"
            )
            if expected_request_id >= 0 and req_id != expected_request_id:
                ctx.panel._emit_log(
                    f"[PICK_OBJ][RX_RESULT] ts={time.time():.6f} req_id={req_id} "
                    f"req_uuid={got_uuid or 'n/a'} success={str(bool(data.get('success', False))).lower()} "
                    "match=false accepted=false reason=request_id_mismatch"
                )
                ctx.panel._emit_log(
                    f"[PICK_OBJ][WAIT_RESULT] state=mismatch_id label={label} elapsed={time.time() - started:.1f}s "
                    f"expected_id={expected_request_id} expected_uuid={expected_request_uuid or 'n/a'} "
                    f"last_seen_id={req_id} last_seen_uuid={got_uuid or 'n/a'}"
                )
                ctx.panel._emit_log(
                    f"[PICK_OBJ][MOVEIT][RESULT] {label} ignore_unmatched request_id={req_id} "
                    f"expected_request_id={expected_request_id} target_stamp_ns={got_stamp_ns}"
                )
                cursor_wall = max(cursor_wall, _wall)
                cursor_seq = max(cursor_seq, int(_seq))
                continue
            if expected_request_uuid and got_uuid != expected_request_uuid:
                ctx.panel._emit_log(
                    f"[PICK_OBJ][RX_RESULT] ts={time.time():.6f} req_id={req_id} "
                    f"req_uuid={got_uuid or 'n/a'} success={str(bool(data.get('success', False))).lower()} "
                    "match=false accepted=false reason=request_uuid_mismatch"
                )
                ctx.panel._emit_log(
                    f"[PICK_OBJ][WAIT_RESULT] state=mismatch_uuid label={label} elapsed={time.time() - started:.1f}s "
                    f"expected_id={expected_request_id} expected_uuid={expected_request_uuid or 'n/a'} "
                    f"last_seen_id={req_id} last_seen_uuid={got_uuid or 'n/a'}"
                )
                ctx.panel._emit_log(
                    f"[PICK_OBJ][RESULT_STALE] {label} ignore_unmatched_uuid got_uuid={got_uuid or 'n/a'} "
                    f"expected_uuid={expected_request_uuid} request_id={req_id}"
                )
                cursor_wall = max(cursor_wall, _wall)
                cursor_seq = max(cursor_seq, int(_seq))
                continue
            raw = json.dumps(data, ensure_ascii=True)
            ctx.panel._emit_log(
                f"[PICK_OBJ][RX_RESULT] ts={time.time():.6f} req_id={req_id} "
                f"req_uuid={got_uuid or 'n/a'} success={str(bool(data.get('success', False))).lower()} "
                "match=true accepted=true reason=matched_expected"
            )
            break
        now = time.time()
        elapsed = now - started
        result_pubs = ctx.panel.ros_worker.topic_publisher_count(ctx.moveit_result_topic)
        result_subs = ctx.panel.ros_worker.topic_subscriber_count(ctx.moveit_result_topic)
        bridge_alive = bool(ctx.panel._proc_alive(getattr(ctx.panel, "moveit_bridge_proc", None)))
        try:
            bridge_detected = bool(ctx.panel._moveit_bridge_detected())
        except Exception:
            bridge_detected = False
        hb_age = ctx.panel.ros_worker.moveit_bridge_heartbeat_age()
        hb_recent = ctx.panel.ros_worker.has_recent_moveit_bridge_heartbeat(hb_recent_window_sec)
        bridge_present = bool(bridge_alive or bridge_detected or hb_recent)
        hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
        if (now - last_diag) >= 2.0:
            ctx.panel._emit_log(
                f"[PICK_OBJ][MOVEIT][EXEC] {label} still_waiting elapsed={elapsed:.1f}s "
                f"timeout={max(0.0, deadline - started):.1f}s result_pubs={result_pubs} result_subs={result_subs} "
                f"bridge_alive={str(bridge_alive).lower()} bridge_detected={str(bridge_detected).lower()} "
                f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
            )
            ctx.panel._emit_log(
                f"[PICK_OBJ][WAIT_RESULT] state=still_waiting label={label} elapsed={elapsed:.1f}s "
                f"expected_id={expected_request_id} expected_uuid={expected_request_uuid or 'n/a'} "
                f"last_seen_id={last_seen_request_id} last_seen_uuid={last_seen_request_uuid or 'n/a'}"
            )
            last_diag = now
            if result_pubs <= 0:
                if lost_pub_since is None:
                    lost_pub_since = now
                lost_age = now - float(lost_pub_since)
                # In stack launches where bridge is external to the ctx.panel process,
                # rely on node discovery/heartbeat as liveness signal as well.
                if (not bridge_present) and lost_age >= 1.5:
                    raise RuntimeError(
                        f"lost_result_publisher:{ctx.moveit_result_topic}:{label}:"
                        f"lost_age={lost_age:.1f}s bridge_alive={str(bridge_alive).lower()} "
                        f"bridge_detected={str(bridge_detected).lower()} "
                        f"hb_recent={str(bool(hb_recent)).lower()}"
                    )
            else:
                lost_pub_since = None
        if (
            not ok
            and not wait_extended
            and now >= deadline
            and result_pubs > 0
            and result_subs > 0
            and bridge_present
        ):
            old_timeout_total = max(0.0, deadline - started)
            deadline = now + active_request_grace_sec
            wait_extended = True
            ctx.panel._emit_log(
                f"[PICK_OBJ][MOVEIT][EXEC] {label} extend_wait old_timeout={old_timeout_total:.1f}s "
                f"new_timeout={max(0.0, deadline - started):.1f}s "
                f"grace={active_request_grace_sec:.1f}s bridge_alive={str(bridge_alive).lower()} "
                f"bridge_detected={str(bridge_detected).lower()} "
                f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt} "
                f"request_id={panel_request_id} expected_request_id={expected_request_id}"
            )
            continue
    if not ok:
        elapsed = time.time() - started
        result_pubs = ctx.panel.ros_worker.topic_publisher_count(ctx.moveit_result_topic)
        result_subs = ctx.panel.ros_worker.topic_subscriber_count(ctx.moveit_result_topic)
        bridge_alive = bool(ctx.panel._proc_alive(getattr(ctx.panel, "moveit_bridge_proc", None)))
        try:
            bridge_detected = bool(ctx.panel._moveit_bridge_detected())
        except Exception:
            bridge_detected = False
        hb_age = ctx.panel.ros_worker.moveit_bridge_heartbeat_age()
        hb_recent = ctx.panel.ros_worker.has_recent_moveit_bridge_heartbeat(hb_recent_window_sec)
        bridge_present = bool(bridge_alive or bridge_detected or hb_recent)
        hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
        lost_age_txt = (
            "n/a"
            if lost_pub_since is None
            else f"{max(0.0, time.time() - float(lost_pub_since)):.1f}s"
        )
        if result_pubs > 0 and result_subs > 0 and bridge_present:
            ctx.panel._emit_log(
                f"[PICK_OBJ][WAIT_RESULT] state=timeout_active_request label={label} elapsed={elapsed:.1f}s "
                f"expected_id={expected_request_id} expected_uuid={expected_request_uuid or 'n/a'} "
                f"last_seen_id={last_seen_request_id} last_seen_uuid={last_seen_request_uuid or 'n/a'}"
            )
            raise RuntimeError(
                f"result_timeout_active_request:{ctx.moveit_result_topic}:{label}:"
                f"elapsed={elapsed:.1f}s pubs={result_pubs} subs={result_subs} "
                f"bridge_alive={str(bridge_alive).lower()} bridge_detected={str(bridge_detected).lower()} "
                f"hb_recent={str(bool(hb_recent)).lower()} "
                f"hb_age={hb_age_txt} wait_extended={str(bool(wait_extended)).lower()} "
                f"since_seq={cursor_seq} panel_request_id={panel_request_id} "
                f"expected_request_id={expected_request_id} expected_stamp_ns={expected_stamp_ns} "
                f"expected_uuid={expected_request_uuid or 'n/a'}"
            )
        ctx.panel._emit_log(
            f"[PICK_OBJ][WAIT_RESULT] state=timeout_lost_publisher label={label} elapsed={elapsed:.1f}s "
            f"expected_id={expected_request_id} expected_uuid={expected_request_uuid or 'n/a'} "
            f"last_seen_id={last_seen_request_id} last_seen_uuid={last_seen_request_uuid or 'n/a'}"
        )
        raise RuntimeError(
            f"lost_result_publisher:{ctx.moveit_result_topic}:{label}:"
            f"elapsed={elapsed:.1f}s pubs={result_pubs} subs={result_subs} "
            f"lost_pub_age={lost_age_txt} since_seq={cursor_seq} "
            f"panel_request_id={panel_request_id} expected_request_id={expected_request_id} "
            f"expected_stamp_ns={expected_stamp_ns} expected_uuid={expected_request_uuid or 'n/a'}"
        )
    try:
        data = json.loads(raw)
    except Exception as exc:
        raise RuntimeError(f"resultado MoveIt inválido ({exc})") from exc
    plan_ok = bool(data.get("plan_ok", False))
    exec_ok = bool(data.get("exec_ok", False))
    success = bool(data.get("success", False))
    message = str(data.get("message") or "")
    ee_link_moveit = str(data.get("ee_link") or "")
    req_id = int(data.get("request_id", -1) or -1)
    stamp_ns = int(data.get("target_stamp_ns", 0) or 0)
    ctx.panel._emit_log(
        f"[PICK_OBJ][MOVEIT][PLAN] {label} plan_ok={str(plan_ok).lower()} "
        f"ee_link={ee_link_moveit or 'n/a'} msg={message or 'n/a'}"
    )
    ctx.panel._emit_log(
        f"[PICK_OBJ][MOVEIT][EXEC] {label} exec_ok={str(exec_ok).lower()} "
        f"success={str(success).lower()} result_topic={ctx.moveit_result_topic}"
    )
    ctx.panel._emit_log(
        f"[PICK_OBJ][MOVEIT][RESULT] {label} request_id={req_id} target_stamp_ns={stamp_ns} "
        f"success={str(success).lower()} plan_ok={str(plan_ok).lower()} "
        f"exec_ok={str(exec_ok).lower()} msg={message or 'n/a'}"
    )
    ctx.panel._emit_log(
        f"[PICK_OBJ][WAIT_RESULT] state=success label={label} elapsed={time.time() - started:.1f}s "
        f"expected_id={expected_request_id} expected_uuid={expected_request_uuid or 'n/a'} "
        f"last_seen_id={req_id} last_seen_uuid={got_uuid or 'n/a'}"
    )
    return data
