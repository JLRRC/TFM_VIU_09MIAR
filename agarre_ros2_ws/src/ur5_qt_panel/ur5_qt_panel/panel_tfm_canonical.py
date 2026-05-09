#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_canonical.py
# Contenido: Maquina de estados canonica TFM->MoveIt + completion de requests pendientes.
"""Maquina de estados canonica del flujo TFM->MoveIt.

Extraido de ``panel_tfm.py`` (lineas 317-465 originales). Cubre:

* ``tfm_canonical_use_pick_object`` — toggle env-controlled.
* Completion callbacks para los 3 tipos de request pendientes:
  ``complete_pending_tfm_infer_request``,
  ``complete_pending_tfm_execute_request``,
  ``complete_pending_pick_demo_request``.
* ``build_tfm_pick_object_override`` — construye el override que
  panel_pick_object consume cuando el flujo TFM dirige a MoveIt.
* ``tfm_canonical_state_reset`` / ``tfm_canonical_phase_update`` /
  ``tfm_canonical_finish`` — start/update/end del contexto canonico,
  con audit logs y JSON snapshots.
"""
from __future__ import annotations

import time
from datetime import datetime

from .panel_tfm_params import get_panel_tfm_params as _get_panel_tfm_params
from typing import Dict


def tfm_canonical_use_pick_object(panel) -> bool:
    return _get_panel_tfm_params().canonical_use_pick_object


def complete_pending_tfm_infer_request(panel, success: bool, message: str) -> None:
    request_id = str(getattr(panel, "_tfm_infer_pending_request_id", "") or "").strip()
    if not request_id:
        return
    panel._tfm_infer_pending_request_id = ""
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            panel.ros_worker.complete_tfm_infer_request(request_id, success, message)
        except Exception:
            pass


def complete_pending_tfm_execute_request(panel, success: bool, message: str) -> None:
    request_id = str(getattr(panel, "_tfm_execute_pending_request_id", "") or "").strip()
    if not request_id:
        return
    panel._tfm_execute_pending_request_id = ""
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            panel.ros_worker.complete_tfm_execute_request(request_id, success, message)
        except Exception:
            pass


def complete_pending_pick_demo_request(panel, success: bool, message: str) -> None:
    request_id = str(getattr(panel, "_pick_demo_pending_request_id", "") or "").strip()
    if not request_id:
        return
    panel._pick_demo_pending_request_id = ""
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            panel.ros_worker.complete_pick_demo_request(request_id, success, message)
        except Exception:
            pass


def build_tfm_pick_object_override(
    panel,
    *,
    grasp_base: Dict[str, float],
    selected_object: str,
    source: str,
) -> Dict[str, object]:
    return {
        "enabled": True,
        "mode": "tfm_moveit",
        "selected_object": str(selected_object or "").strip(),
        "frame": panel._business_base_frame(),
        "x": float(grasp_base.get("x", 0.0) or 0.0),
        "y": float(grasp_base.get("y", 0.0) or 0.0),
        "z": float(grasp_base.get("z", 0.0) or 0.0),
        "yaw_deg": float(grasp_base.get("yaw_deg", 0.0) or 0.0),
        "source": str(source or "unknown"),
        "created_ts": time.time(),
    }


def tfm_canonical_state_reset(
    panel,
    *,
    selected_object: str,
    grasp_base: Dict[str, float],
    source: str,
) -> None:
    session = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    panel._tfm_canonical_ctx = {
        "route": "TFM->MoveIt",
        "session": session,
        "selected_object": str(selected_object or "").strip(),
        "source": str(source or "unknown"),
        "grasp_base": dict(grasp_base),
        "state": "READY",
        "events": [],
        "start_ts": time.time(),
        "success": None,
        "message": "",
    }
    panel._audit_append(
        "logs/tfm_moveit_canonical.log",
        "[TFM][CANON] start "
        f"session={session} selected={selected_object or 'none'} "
        f"grasp=({float(grasp_base.get('x', 0.0) or 0.0):.3f},"
        f"{float(grasp_base.get('y', 0.0) or 0.0):.3f},"
        f"{float(grasp_base.get('z', 0.0) or 0.0):.3f}) "
        f"yaw={float(grasp_base.get('yaw_deg', 0.0) or 0.0):.1f} "
        f"source={source or 'unknown'}",
    )
    panel._audit_write_json(
        "artifacts/tfm_moveit_canonical_last.json",
        dict(panel._tfm_canonical_ctx),
    )


def tfm_canonical_phase_update(panel, state: str, *, detail: str = "") -> None:
    ctx = getattr(panel, "_tfm_canonical_ctx", None)
    if not isinstance(ctx, dict):
        return
    phase = str(state or "").strip() or "UNKNOWN"
    event = {
        "ts": time.time(),
        "state": phase,
        "detail": str(detail or ""),
    }
    events = ctx.setdefault("events", [])
    if isinstance(events, list):
        events.append(event)
    ctx["state"] = phase
    panel._audit_append(
        "logs/tfm_moveit_canonical.log",
        f"[TFM][CANON] state={phase} detail={detail or 'n/a'}",
    )
    panel._audit_write_json(
        "artifacts/tfm_moveit_canonical_last.json",
        dict(ctx),
    )


def tfm_canonical_finish(panel, success: bool, message: str, *, final_state: str) -> None:
    ctx = getattr(panel, "_tfm_canonical_ctx", None)
    end_ts = time.time()
    if isinstance(ctx, dict):
        ctx["success"] = bool(success)
        ctx["message"] = str(message or "")
        ctx["final_state"] = str(final_state or ("HOME_DONE" if success else "FAIL_TERMINAL"))
        ctx["state"] = ctx["final_state"]
        ctx["end_ts"] = end_ts
        ctx["duration_sec"] = max(0.0, end_ts - float(ctx.get("start_ts", end_ts) or end_ts))
        events = ctx.setdefault("events", [])
        if isinstance(events, list):
            events.append(
                {
                    "ts": end_ts,
                    "state": ctx["final_state"],
                    "detail": str(message or ""),
                }
            )
        panel._audit_append(
            "logs/tfm_moveit_canonical.log",
            "[TFM][CANON] finish "
            f"success={str(bool(success)).lower()} "
            f"final_state={ctx['final_state']} "
            f"duration={float(ctx.get('duration_sec', 0.0) or 0.0):.2f}s "
            f"message={message or 'n/a'}",
        )
        panel._audit_write_json(
            "artifacts/tfm_moveit_canonical_last.json",
            dict(ctx),
        )
    panel._pick_object_grasp_override = None
    panel._tfm_canonical_ctx = None
    panel._tfm_execute_inflight = False
    complete_pending_tfm_execute_request(panel, bool(success), str(message or ""))
