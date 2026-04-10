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
    JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD,
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
    get_object_positions,
    is_on_table,
    update_object_state,
    ObjectOwner,
    ObjectLogicalState,
)
from .panel_readiness import tf_ready_status
from .panel_utils import (
    angle_shortest_diff_rad,
    get_pose,
    transform_point_to_frame,
    world_to_base,
    world_xyz_to_pixel_float,
    table_xy_to_pixel_float,
)
from .ur5_kinematics import fk_ur5, ik_ur5

DIRECT_ROUTE_MODE = "direct_rg2_pinch_center"
DIRECT_SOURCE_FRAME = "rg2_pinch_center"
DIRECT_LEGACY_TCP_FRAME = "rg2_tcp"
DIRECT_EXECUTION_FRAME = "tool0"
DIRECT_EXECUTION_IK_MODE = "formal_rg2_pinch_center_source_to_tool0_numeric"
DIRECT_TOOL0_TO_RG2_TCP_Z_M = 0.175
DIRECT_GRASP_AUDIT_PREFIX = "[PICK][DIRECT_GRASP_AUDIT]"


def _effective_direct_grasp_z(source_frame: str, requested_offset_m: float) -> float:
    """Return the runtime grasp Z offset for the active operational grasp frame.

    `rg2_pinch_center` is already the functional contact frame in this project, so
    carrying over the old `rg2_tcp` vertical offset would shift DIRECTO upward and
    break physical contact.
    """
    if str(source_frame or "").strip() == "rg2_pinch_center":
        return 0.0
    return float(requested_offset_m)


def _direct_runtime_target_tol_m(label: str) -> float:
    label_name = str(label or "").strip().upper()
    if label_name == "GRASP_DOWN_JOINT":
        return max(
            0.005,
            float(os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_TCP_TOL_M", "0.020") or 0.020),
        )
    if label_name == "GRASP_ALIGN_IK":
        return max(
            0.005,
            float(os.environ.get("PANEL_PICK_DEMO_GRASP_ALIGN_TCP_TOL_M", "0.015") or 0.015),
        )
    return max(
        0.01,
        float(os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M", "0.040") or 0.040),
    )


def _pick_demo_tuple3(data):
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


def _pick_demo_fmt_vec(vec) -> str:
    vec3 = _pick_demo_tuple3(vec)
    if vec3 is None:
        return "none"
    return f"({vec3[0]:.3f},{vec3[1]:.3f},{vec3[2]:.3f})"


def _pick_demo_fmt_scalar(value, *, digits: int = 3) -> str:
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return "none"


def _pick_demo_env_float(name: str, default: float, *, minimum: float = 0.0) -> float:
    try:
        value = float(os.environ.get(name, str(default)) or default)
    except Exception:
        value = float(default)
    return max(float(minimum), float(value))


def _pick_demo_env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() not in {"", "0", "false", "no", "off"}


def _resolve_live_object_world(
    panel,
    object_name: str,
    *,
    trace_fn=None,
    now_fn=None,
    get_positions_fn=get_object_positions,
    get_state_fn=get_object_state,
):
    if trace_fn is None:
        trace_fn = lambda _line: None
    if now_fn is None:
        now_fn = time.time

    max_snapshot_age_sec = _pick_demo_env_float(
        "PANEL_PICK_DEMO_MAX_SNAPSHOT_AGE_SEC",
        0.10,
    )
    max_stable_cache_age_sec = _pick_demo_env_float(
        "PANEL_PICK_DEMO_MAX_STABLE_CACHE_AGE_SEC",
        0.20,
    )
    divergence_tol_m = _pick_demo_env_float(
        "PANEL_PICK_DEMO_OBJECT_SOURCE_DIVERGENCE_TOL_M",
        0.150,
        minimum=0.05,
    )
    allow_correlated_stable_fallback = _pick_demo_env_flag(
        "PANEL_PICK_DEMO_ALLOW_CORRELATED_STABLE_FALLBACK",
        True,
    )

    snapshot_world = None
    snapshot_age_sec = None
    snapshot_fresh = False
    snapshot_reason = "snapshot_unavailable"
    if (
        getattr(panel, "_ros_worker_started", False)
        and getattr(panel, "ros_worker", None) is not None
    ):
        try:
            ros_worker = panel.ros_worker
            if ros_worker.node_ready():
                pose_map, pose_ts = ros_worker.pose_snapshot()
                pose = (pose_map or {}).get(object_name)
                if pose is not None and len(pose) >= 3:
                    snapshot_world = _pick_demo_tuple3(pose)
                if pose_ts is not None:
                    snapshot_age_sec = max(0.0, float(now_fn()) - float(pose_ts))
        except Exception as exc:
            snapshot_reason = f"snapshot_error:{exc}"
        else:
            if snapshot_world is None:
                snapshot_reason = "snapshot_missing_object"
            elif snapshot_age_sec is None:
                snapshot_reason = "snapshot_age_unknown"
            elif snapshot_age_sec <= max_snapshot_age_sec:
                snapshot_fresh = True
                snapshot_reason = "snapshot_fresh"
            else:
                snapshot_reason = "snapshot_stale"

    trace_fn(
        "[LIVE_OBJ][SOURCE] "
        f"source=snapshot object={object_name} world={_pick_demo_fmt_vec(snapshot_world)} "
        f"age_sec={_pick_demo_fmt_scalar(snapshot_age_sec)} fresh={str(bool(snapshot_fresh)).lower()} "
        f"max_age_sec={_pick_demo_fmt_scalar(max_snapshot_age_sec)} reason={snapshot_reason}"
    )
    if snapshot_world is not None and not snapshot_fresh:
        trace_fn(
            "[LIVE_OBJ][AGE] "
            f"source=snapshot object={object_name} age_sec={_pick_demo_fmt_scalar(snapshot_age_sec)} "
            f"max_age_sec={_pick_demo_fmt_scalar(max_snapshot_age_sec)} reason={snapshot_reason}"
        )
        trace_fn(
            "[LIVE_OBJ][REJECT] "
            f"source=snapshot object={object_name} reason={snapshot_reason}"
        )

    state = None
    stable_world = None
    stable_age_sec = None
    stable_fresh = False
    stable_reason = "stable_unavailable"
    try:
        state = get_state_fn(object_name)
    except Exception as exc:
        stable_reason = f"stable_state_error:{exc}"
    if state is not None:
        stable_world = _pick_demo_tuple3(getattr(state, "position", None))
        last_update_ts = getattr(state, "last_update_ts", None)
        if last_update_ts is not None:
            try:
                stable_age_sec = max(0.0, float(now_fn()) - float(last_update_ts))
            except Exception:
                stable_age_sec = None
    if stable_world is None:
        try:
            stable_world = _pick_demo_tuple3((get_positions_fn() or {}).get(object_name))
        except Exception as exc:
            stable_reason = f"stable_positions_error:{exc}"

    if stable_world is None:
        stable_reason = "stable_missing_object"
    elif stable_age_sec is None:
        stable_reason = "stable_age_unknown"
    elif stable_age_sec <= max_stable_cache_age_sec:
        stable_fresh = True
        stable_reason = "stable_fresh"
    else:
        stable_reason = "stable_stale"

    trace_fn(
        "[LIVE_OBJ][SOURCE] "
        f"source=stable_cache object={object_name} world={_pick_demo_fmt_vec(stable_world)} "
        f"age_sec={_pick_demo_fmt_scalar(stable_age_sec)} fresh={str(bool(stable_fresh)).lower()} "
        f"max_age_sec={_pick_demo_fmt_scalar(max_stable_cache_age_sec)} reason={stable_reason}"
    )
    if stable_world is not None:
        trace_fn(
            "[LIVE_OBJ][CORRELATED] "
            f"source=stable_cache object={object_name} correlated_with_snapshot=true "
            f"allow_fallback={str(bool(allow_correlated_stable_fallback)).lower()}"
        )
    if stable_world is not None and not stable_fresh:
        trace_fn(
            "[LIVE_OBJ][AGE] "
            f"source=stable_cache object={object_name} age_sec={_pick_demo_fmt_scalar(stable_age_sec)} "
            f"max_age_sec={_pick_demo_fmt_scalar(max_stable_cache_age_sec)} reason={stable_reason}"
        )
        trace_fn(
            "[LIVE_OBJ][REJECT] "
            f"source=stable_cache object={object_name} reason={stable_reason}"
        )

    divergence_m = None
    if snapshot_fresh and stable_fresh and snapshot_world is not None and stable_world is not None:
        dx = float(snapshot_world[0]) - float(stable_world[0])
        dy = float(snapshot_world[1]) - float(stable_world[1])
        dz = float(snapshot_world[2]) - float(stable_world[2])
        divergence_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        if divergence_m > divergence_tol_m:
            trace_fn(
                "[LIVE_OBJ][REJECT] "
                f"source=snapshot object={object_name} reason=source_divergence "
                f"divergence_m={_pick_demo_fmt_scalar(divergence_m)} "
                f"tol_m={_pick_demo_fmt_scalar(divergence_tol_m)}"
            )
            if allow_correlated_stable_fallback:
                trace_fn(
                    "[LIVE_OBJ][FALLBACK] "
                    f"object={object_name} selected=stable_cache_correlated "
                    "reason=source_divergence_with_fresh_correlated_cache"
                )
                result = {
                    "world": stable_world,
                    "source": "stable_cache_correlated",
                    "reason": "source_divergence_with_correlated_fallback",
                    "snapshot_world": snapshot_world,
                    "snapshot_age_sec": snapshot_age_sec,
                    "stable_world": stable_world,
                    "stable_age_sec": stable_age_sec,
                    "divergence_m": divergence_m,
                }
                trace_fn(
                    "[LIVE_OBJ][FINAL] "
                    f"object={object_name} source={result['source']} world={_pick_demo_fmt_vec(result['world'])} "
                    f"reason={result['reason']}"
                )
                return result
            result = {
                "world": None,
                "source": "none",
                "reason": "source_divergence_correlated_fallback_disabled",
                "snapshot_world": snapshot_world,
                "snapshot_age_sec": snapshot_age_sec,
                "stable_world": stable_world,
                "stable_age_sec": stable_age_sec,
                "divergence_m": divergence_m,
            }
            trace_fn(
                "[LIVE_OBJ][FINAL] "
                f"object={object_name} source=none world=none reason={result['reason']}"
            )
            return result

    if snapshot_fresh and snapshot_world is not None:
        result = {
            "world": snapshot_world,
            "source": "snapshot_pose_info",
            "reason": "snapshot_fresh",
            "snapshot_world": snapshot_world,
            "snapshot_age_sec": snapshot_age_sec,
            "stable_world": stable_world,
            "stable_age_sec": stable_age_sec,
            "divergence_m": divergence_m,
        }
        trace_fn(
            "[LIVE_OBJ][FINAL] "
            f"object={object_name} source={result['source']} world={_pick_demo_fmt_vec(result['world'])} "
            f"reason={result['reason']}"
        )
        return result

    if stable_fresh and stable_world is not None:
        if allow_correlated_stable_fallback:
            trace_fn(
                "[LIVE_OBJ][FALLBACK] "
                f"object={object_name} selected=stable_cache_correlated reason=snapshot_not_usable"
            )
            result = {
                "world": stable_world,
                "source": "stable_cache_correlated",
                "reason": "snapshot_not_usable_correlated_fallback",
                "snapshot_world": snapshot_world,
                "snapshot_age_sec": snapshot_age_sec,
                "stable_world": stable_world,
                "stable_age_sec": stable_age_sec,
                "divergence_m": divergence_m,
            }
            trace_fn(
                "[LIVE_OBJ][FINAL] "
                f"object={object_name} source={result['source']} world={_pick_demo_fmt_vec(result['world'])} "
                f"reason={result['reason']}"
            )
            return result
        trace_fn(
            "[LIVE_OBJ][REJECT] "
            f"source=stable_cache object={object_name} reason=correlated_fallback_disabled"
        )
        result = {
            "world": None,
            "source": "none",
            "reason": "correlated_fallback_disabled",
            "snapshot_world": snapshot_world,
            "snapshot_age_sec": snapshot_age_sec,
            "stable_world": stable_world,
            "stable_age_sec": stable_age_sec,
            "divergence_m": divergence_m,
        }
        trace_fn(
            "[LIVE_OBJ][FINAL] "
            f"object={object_name} source=none world=none reason={result['reason']}"
        )
        return result

    result = {
        "world": None,
        "source": "none",
        "reason": "no_fresh_live_object_pose",
        "snapshot_world": snapshot_world,
        "snapshot_age_sec": snapshot_age_sec,
        "stable_world": stable_world,
        "stable_age_sec": stable_age_sec,
        "divergence_m": divergence_m,
    }
    trace_fn(
        "[LIVE_OBJ][FINAL] "
        f"object={object_name} source=none world=none reason={result['reason']}"
    )
    return result


def _resolve_live_object_base(
    panel,
    object_name: str,
    *,
    world_result=None,
    trace_fn=None,
    transform_fn=transform_point_to_frame,
    static_world_to_base_fn=world_to_base,
):
    if trace_fn is None:
        trace_fn = lambda _line: None
    if world_result is None:
        world_result = _resolve_live_object_world(
            panel,
            object_name,
            trace_fn=trace_fn,
        )
    world_pos = _pick_demo_tuple3((world_result or {}).get("world"))
    if world_pos is None:
        result = dict(world_result or {})
        result["base"] = None
        result["base_source"] = "none"
        result["base_reason"] = "world_pose_unavailable"
        trace_fn(
            "[LIVE_OBJ][FINAL] "
            f"object={object_name} base=none reason={result['base_reason']}"
        )
        return result

    world_frame = str(
        getattr(panel, "_world_frame_last_first", lambda fallback=None: WORLD_FRAME or "world")(
            WORLD_FRAME or "world"
        )
    ).strip() or "world"
    try:
        base_frame = str(panel._business_base_frame() or BASE_FRAME or "base_link")
    except Exception:
        base_frame = str(BASE_FRAME or "base_link")

    obj_base = None
    try:
        obj_base, _ = transform_fn(
            world_pos,
            base_frame,
            source_frame=world_frame,
        )
    except Exception as exc:
        trace_fn(
            "[LIVE_OBJ][REJECT] "
            f"source=tf_transform object={object_name} reason=tf_exception:{exc}"
        )
        obj_base = None

    if obj_base:
        base_pos = _pick_demo_tuple3(obj_base)
        result = dict(world_result or {})
        result["base"] = base_pos
        result["base_source"] = "tf_transform"
        result["base_reason"] = "tf_transform_ok"
        trace_fn(
            "[LIVE_OBJ][FINAL] "
            f"object={object_name} base_source={result['base_source']} base={_pick_demo_fmt_vec(base_pos)} "
            f"reason={result['base_reason']}"
        )
        return result

    allow_static_world_to_base_fallback = _pick_demo_env_flag(
        "PANEL_PICK_DEMO_ALLOW_STATIC_WORLD_TO_BASE_FALLBACK",
        False,
    )
    if allow_static_world_to_base_fallback:
        try:
            static_base = _pick_demo_tuple3(static_world_to_base_fn(*world_pos))
        except Exception as exc:
            static_base = None
            trace_fn(
                "[LIVE_OBJ][REJECT] "
                f"source=static_world_to_base object={object_name} reason=static_fallback_error:{exc}"
            )
        else:
            trace_fn(
                "[LIVE_OBJ][STATIC_FALLBACK] "
                f"object={object_name} enabled=true base={_pick_demo_fmt_vec(static_base)}"
            )
        result = dict(world_result or {})
        result["base"] = static_base
        result["base_source"] = "static_world_to_base" if static_base is not None else "none"
        result["base_reason"] = (
            "static_world_to_base_fallback"
            if static_base is not None
            else "static_world_to_base_fallback_failed"
        )
        trace_fn(
            "[LIVE_OBJ][FINAL] "
            f"object={object_name} base_source={result['base_source']} base={_pick_demo_fmt_vec(static_base)} "
            f"reason={result['base_reason']}"
        )
        return result

    trace_fn(
        "[LIVE_OBJ][STATIC_FALLBACK] "
        f"object={object_name} enabled=false reason=tf_transform_unavailable"
    )
    result = dict(world_result or {})
    result["base"] = None
    result["base_source"] = "none"
    result["base_reason"] = "tf_transform_unavailable_static_fallback_disabled"
    trace_fn(
        "[LIVE_OBJ][FINAL] "
        f"object={object_name} base_source=none base=none reason={result['base_reason']}"
    )
    return result


def _select_pick_demo_cycle_object_reference(
    panel,
    object_name: str,
    *,
    selected_base_anchor=None,
    trace_fn=None,
    resolve_world_fn=_resolve_live_object_world,
    resolve_base_fn=_resolve_live_object_base,
    get_state_fn=get_object_state,
    is_on_table_fn=is_on_table,
):
    if trace_fn is None:
        trace_fn = lambda _line: None

    selected_base = _pick_demo_tuple3(selected_base_anchor)
    max_promoted_stable_age_sec = _pick_demo_env_float(
        "PANEL_PICK_DEMO_MAX_PROMOTED_STABLE_AGE_SEC",
        2.00,
    )
    max_selected_stable_divergence_m = _pick_demo_env_float(
        "PANEL_PICK_DEMO_MAX_SELECTED_STABLE_DIVERGENCE_M",
        0.080,
        minimum=0.01,
    )
    require_object_on_table = _pick_demo_env_flag(
        "PANEL_PICK_DEMO_REQUIRE_OBJECT_ON_TABLE_FOR_PROMOTION",
        True,
    )

    def _dist3(a, b):
        aa = _pick_demo_tuple3(a)
        bb = _pick_demo_tuple3(b)
        if aa is None or bb is None:
            return None
        dx = float(aa[0]) - float(bb[0])
        dy = float(aa[1]) - float(bb[1])
        dz = float(aa[2]) - float(bb[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    trace_fn(
        "[PICK][DIRECT][CYCLE_REF][SELECT] tag=[DIRECT][CYCLE_REF][SELECT] "
        f"phase=BUTTON_PRESS selected_pose_base_link={_pick_demo_fmt_vec(selected_base)} "
        f"max_promoted_stable_age_sec={_pick_demo_fmt_scalar(max_promoted_stable_age_sec)} "
        f"max_selected_stable_divergence_m={_pick_demo_fmt_scalar(max_selected_stable_divergence_m)} "
        f"require_object_on_table={str(bool(require_object_on_table)).lower()}"
    )

    world_result = resolve_world_fn(panel, object_name, trace_fn=trace_fn)
    base_result = resolve_base_fn(panel, object_name, world_result=world_result, trace_fn=trace_fn)
    snapshot_world = _pick_demo_tuple3((world_result or {}).get("world"))
    snapshot_base = _pick_demo_tuple3((base_result or {}).get("base"))
    snapshot_source = str((world_result or {}).get("source") or "none")
    snapshot_reason = str((world_result or {}).get("reason") or "none")
    snapshot_age_sec = (world_result or {}).get("snapshot_age_sec")

    if snapshot_source == "snapshot_pose_info" and snapshot_world is not None and snapshot_base is not None:
        selected = {
            "ok": True,
            "world": snapshot_world,
            "base": snapshot_base,
            "source": "snapshot_pose_info",
            "reason": "snapshot_fresh_cycle_ref",
            "promoted_stable": False,
            "snapshot_age_sec": snapshot_age_sec,
            "stable_age_sec": (world_result or {}).get("stable_age_sec"),
            "selected_stable_divergence_m": None,
        }
        trace_fn(
            "[PICK][DIRECT][CYCLE_REF][SELECT] tag=[DIRECT][CYCLE_REF][SELECT] "
            f"phase=BUTTON_PRESS source={selected['source']} promoted_stable=false "
            f"snapshot_age_sec={_pick_demo_fmt_scalar(snapshot_age_sec)} "
            f"cycle_object_world={_pick_demo_fmt_vec(selected['world'])} "
            f"cycle_object_base={_pick_demo_fmt_vec(selected['base'])}"
        )
        return selected

    stable_world = _pick_demo_tuple3((world_result or {}).get("stable_world"))
    stable_age_sec = (world_result or {}).get("stable_age_sec")
    if stable_age_sec is None:
        try:
            panel_age = getattr(panel, "_last_trace_object_age_sec", None)
            if panel_age is not None:
                stable_age_sec = float(panel_age)
        except Exception:
            stable_age_sec = None
    stable_base = None
    stable_state = None
    stable_logical_state = "none"
    promoted_reject_reasons = []

    if stable_world is None:
        promoted_reject_reasons.append("stable_world_unavailable")
    if stable_age_sec is None:
        promoted_reject_reasons.append("stable_age_unknown")
    else:
        try:
            if float(stable_age_sec) > float(max_promoted_stable_age_sec):
                promoted_reject_reasons.append("stable_age_exceeded")
        except Exception:
            promoted_reject_reasons.append("stable_age_invalid")

    try:
        stable_state = get_state_fn(object_name)
    except Exception:
        stable_state = None
    if stable_state is not None:
        logical_state = getattr(stable_state, "logical_state", None)
        stable_logical_state = str(getattr(logical_state, "value", "none") or "none")
        if logical_state in (ObjectLogicalState.SPAWNED, ObjectLogicalState.RELEASED):
            promoted_reject_reasons.append("state_not_stable_for_promotion")

    on_table_ok = True
    if require_object_on_table:
        on_table_pose = stable_world if stable_world is not None else snapshot_world
        try:
            on_table_ok = bool(is_on_table_fn(on_table_pose)) if on_table_pose is not None else False
        except Exception:
            on_table_ok = False
        if not on_table_ok and stable_logical_state == "ON_TABLE":
            on_table_ok = True
        if not on_table_ok:
            promoted_reject_reasons.append("object_not_on_table")

    if stable_world is not None:
        stable_candidate_world_result = {
            "world": stable_world,
            "source": "stable_cache_promoted_candidate",
            "reason": "stable_candidate",
            "snapshot_world": (world_result or {}).get("snapshot_world"),
            "snapshot_age_sec": (world_result or {}).get("snapshot_age_sec"),
            "stable_world": stable_world,
            "stable_age_sec": stable_age_sec,
            "divergence_m": (world_result or {}).get("divergence_m"),
        }
        stable_base_result = resolve_base_fn(
            panel,
            object_name,
            world_result=stable_candidate_world_result,
            trace_fn=trace_fn,
        )
        stable_base = _pick_demo_tuple3((stable_base_result or {}).get("base"))
        if stable_base is None:
            promoted_reject_reasons.append("stable_base_transform_unavailable")

    selected_stable_divergence_m = _dist3(selected_base, stable_base)
    if (
        selected_stable_divergence_m is not None
        and selected_stable_divergence_m > max_selected_stable_divergence_m
    ):
        promoted_reject_reasons.append("selected_stable_divergence_exceeded")

    # Si el snapshot fresco no está disponible pero la referencia seleccionada y la
    # pose estable coinciden geométricamente sobre la mesa, permitimos seguir con
    # la referencia estable aunque su timestamp sea viejo. Esto evita abortos
    # espurios al reintentar DIRECTO con el demo quieto y ya seleccionado.
    allow_selected_stable_cycle_ref = bool(
        snapshot_base is None
        and stable_world is not None
        and stable_base is not None
        and on_table_ok
        and selected_stable_divergence_m is not None
        and selected_stable_divergence_m <= max_selected_stable_divergence_m
    )
    if allow_selected_stable_cycle_ref and "stable_age_exceeded" in promoted_reject_reasons:
        promoted_reject_reasons = [
            reason for reason in promoted_reject_reasons if reason != "stable_age_exceeded"
        ]
        trace_fn(
            "[PICK][DIRECT][CYCLE_REF][OVERRIDE] tag=[DIRECT][CYCLE_REF][OVERRIDE] "
            "phase=BUTTON_PRESS reason=stable_age_exceeded "
            f"selected_stable_divergence_m={_pick_demo_fmt_scalar(selected_stable_divergence_m)} "
            f"max_selected_stable_divergence_m={_pick_demo_fmt_scalar(max_selected_stable_divergence_m)} "
            f"object_on_table={str(bool(on_table_ok)).lower()} "
            "note=stable_cycle_ref_reused_when_selected_pose_matches_and_snapshot_missing"
        )

    if promoted_reject_reasons:
        for reject_reason in promoted_reject_reasons:
            trace_fn(
                "[PICK][DIRECT][CYCLE_REF][REJECT] tag=[DIRECT][CYCLE_REF][REJECT] "
                f"phase=BUTTON_PRESS source=stable_cache reject_reason={reject_reason} "
                f"stable_age_sec={_pick_demo_fmt_scalar(stable_age_sec)} "
                f"selected_stable_divergence_m={_pick_demo_fmt_scalar(selected_stable_divergence_m)} "
                f"max_promoted_stable_age_sec={_pick_demo_fmt_scalar(max_promoted_stable_age_sec)} "
                f"max_selected_stable_divergence_m={_pick_demo_fmt_scalar(max_selected_stable_divergence_m)} "
                f"object_on_table={str(bool(on_table_ok)).lower()} "
                f"object_logical_state={stable_logical_state} "
                f"stable_world={_pick_demo_fmt_vec(stable_world)} stable_base={_pick_demo_fmt_vec(stable_base)}"
            )
        if snapshot_base is None:
            trace_fn(
                "[PICK][DIRECT][CYCLE_REF][REJECT] tag=[DIRECT][CYCLE_REF][REJECT] "
                f"phase=BUTTON_PRESS source=snapshot reject_reason=snapshot_base_unavailable "
                f"snapshot_source={snapshot_source} snapshot_reason={snapshot_reason} "
                f"snapshot_world={_pick_demo_fmt_vec(snapshot_world)} snapshot_base={_pick_demo_fmt_vec(snapshot_base)}"
            )
        return {
            "ok": False,
            "world": None,
            "base": None,
            "source": "none",
            "reason": "cycle_reference_unavailable",
            "promoted_stable": False,
            "snapshot_age_sec": snapshot_age_sec,
            "stable_age_sec": stable_age_sec,
            "selected_stable_divergence_m": selected_stable_divergence_m,
            "reject_reasons": promoted_reject_reasons,
        }

    selected = {
        "ok": True,
        "world": stable_world,
        "base": stable_base,
        "source": "stable_cache_promoted_cycle",
        "reason": "stable_cache_promoted_cycle_reference",
        "promoted_stable": True,
        "snapshot_age_sec": snapshot_age_sec,
        "stable_age_sec": stable_age_sec,
        "selected_stable_divergence_m": selected_stable_divergence_m,
    }
    trace_fn(
        "[PICK][DIRECT][CYCLE_REF][PROMOTE_STABLE] tag=[DIRECT][CYCLE_REF][PROMOTE_STABLE] "
        "phase=BUTTON_PRESS source=stable_cache "
        f"stable_age_sec={_pick_demo_fmt_scalar(stable_age_sec)} "
        f"selected_stable_divergence_m={_pick_demo_fmt_scalar(selected_stable_divergence_m)} "
        f"object_on_table={str(bool(on_table_ok)).lower()} "
        f"object_logical_state={stable_logical_state} "
        f"cycle_object_world={_pick_demo_fmt_vec(selected['world'])} "
        f"cycle_object_base={_pick_demo_fmt_vec(selected['base'])}"
    )
    return selected


def _demo_object_in_basket(panel, timeout_sec: float = 4.0) -> bool:
    """Confirma por posicion que el objeto demo esta en la cesta."""
    start = time.monotonic()
    basket_world = tuple(float(v) for v in BASKET_DROP)
    release_reference_world = _pick_demo_tuple3(
        getattr(panel, "_pick_demo_release_reference_world", None)
    )
    world_frame = str(
        getattr(panel, "_world_frame_last_first", lambda fallback=None: WORLD_FRAME or "world")(
            WORLD_FRAME or "world"
        )
    ).strip() or "world"
    try:
        base_frame = str(panel._business_base_frame() or BASE_FRAME or "base_link")
    except Exception:
        base_frame = str(getattr(panel, "_base_frame_effective", "") or BASE_FRAME or "base_link")
    basket_base, _ = transform_point_to_frame(
        basket_world,
        base_frame,
        source_frame=world_frame,
    )
    release_reference_base = None
    if release_reference_world is not None:
        release_reference_base, _ = transform_point_to_frame(
            release_reference_world,
            base_frame,
            source_frame=world_frame,
        )

    def _fresh_demo_world() -> tuple[float, float, float] | None:
        try:
            if (
                getattr(panel, "_ros_worker_started", False)
                and getattr(panel, "ros_worker", None) is not None
            ):
                pose_map, _ = panel.ros_worker.pose_snapshot()
                live = (pose_map or {}).get(PICK_DEMO_OBJECT_NAME)
                if live is not None and len(live) >= 3:
                    return (float(live[0]), float(live[1]), float(live[2]))
        except Exception:
            pass
        return None

    xy_tol_world = 0.35
    z_tol_world = 0.35
    xy_tol_base = 0.30
    z_tol_base = 0.25
    release_xy_tol_world = 0.20
    release_xy_tol_base = 0.20
    release_z_headroom_world = 0.10
    release_z_headroom_base = 0.10
    while (time.monotonic() - start) <= timeout_sec:
        st = get_object_state(PICK_DEMO_OBJECT_NAME)
        if st is not None:
            physical_world = _pick_demo_tuple3(_fresh_demo_world())
            state_world = _pick_demo_tuple3(getattr(st, "position", None))
            obj_world = physical_world or state_world
            if obj_world is None:
                time.sleep(0.2)
                continue
            xw, yw, zw = (float(obj_world[0]), float(obj_world[1]), float(obj_world[2]))

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
            obj_base = None
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

            release_xy_ok_world = False
            release_xy_ok_base = False
            release_dxy_world = float("inf")
            release_dxy_base = float("inf")
            release_below_world = False
            release_below_base = False
            if release_reference_world is not None:
                release_dxw = xw - float(release_reference_world[0])
                release_dyw = yw - float(release_reference_world[1])
                release_dxy_world = (release_dxw * release_dxw + release_dyw * release_dyw) ** 0.5
                release_below_world = zw <= (float(release_reference_world[2]) + release_z_headroom_world)
                release_xy_ok_world = (
                    release_dxy_world <= release_xy_tol_world and release_below_world
                )
            if release_reference_base and obj_base:
                release_dxb = float(obj_base[0]) - float(release_reference_base[0])
                release_dyb = float(obj_base[1]) - float(release_reference_base[1])
                release_dxy_base = (release_dxb * release_dxb + release_dyb * release_dyb) ** 0.5
                release_below_base = float(obj_base[2]) <= (
                    float(release_reference_base[2]) + release_z_headroom_base
                )
                release_xy_ok_base = (
                    release_dxy_base <= release_xy_tol_base and release_below_base
                )

            detached_ok = (not bool(st.attached)) and (st.owner == ObjectOwner.NONE)
            release_ok = release_xy_ok_world or release_xy_ok_base
            if detached_ok and (world_ok or base_ok or release_ok):
                confirmation_source = "basket_reference"
                if release_ok and not (world_ok or base_ok):
                    confirmation_source = "release_reference"
                panel._emit_log(
                    "[PICK][DEMO] confirmacion cesta OK "
                    f"source={confirmation_source} "
                    f"world_obj=({xw:.3f},{yw:.3f},{zw:.3f}) "
                    f"world_basket=({basket_world[0]:.3f},{basket_world[1]:.3f},{basket_world[2]:.3f}) "
                    f"world_release={_pick_demo_fmt_vec(release_reference_world)} "
                    f"dxy_w={dxy_world:.3f} dz_w={dz_world:.3f} "
                    f"dxy_b={dxy_base:.3f} dz_b={dz_base:.3f} "
                    f"release_dxy_w={release_dxy_world:.3f} release_dxy_b={release_dxy_base:.3f}"
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
    requested_route_mode = str(
        os.environ.get("PANEL_PICK_DEMO_ROUTE_MODE", "manual_reference") or "manual_reference"
    ).strip().lower()
    route_candidates = "manual_reference,direct_ik_hybrid,joint_preset_fallback"
    route_gate_manual_reference = True
    route_gate_direct_ik = bool(tf_ok and bool(panel._ee_frame_effective) and ready)
    route_gate_joint_preset = True
    if requested_route_mode in {"direct_ik_hybrid", "direct_ik", "ik"} and route_gate_direct_ik:
        route_selected = "direct_ik_hybrid"
        route_reason = "requested_direct_ik_hybrid"
    elif requested_route_mode in {"manual_reference", "manual_ref", "reference"} and route_gate_manual_reference:
        route_selected = "manual_reference"
        route_reason = "requested_manual_reference"
    elif requested_route_mode in {"joint_preset_fallback", "joint_preset", "preset"}:
        route_selected = "joint_preset_fallback"
        route_reason = "requested_joint_preset_fallback"
    elif route_gate_direct_ik:
        route_selected = "direct_ik_hybrid"
        route_reason = (
            "run_pick_demo_uses_move_tcp_direct_for_APPROACH_COARSE_GRASP_DOWN_JOINT_and_GRASP_ALIGN_IK"
        )
    else:
        route_selected = "joint_preset_fallback"
        route_reason = "direct_ik_gate_false"
    panel._emit_log(
        "[PICK][DIRECT][ROUTE] "
        f"route_candidates={route_candidates} "
        f"route_gate_manual_reference={str(route_gate_manual_reference).lower()} "
        f"route_gate_direct_ik={str(route_gate_direct_ik).lower()} "
        f"route_gate_joint_preset_fallback={str(route_gate_joint_preset).lower()} "
        f"route_requested={requested_route_mode or 'auto'} "
        f"route_selected={route_selected} "
        f"route_reason={route_reason}"
    )
    panel._emit_log(
        f"[PICK][DIRECT] selection_ok=true target={PICK_DEMO_OBJECT_NAME} route={route_selected}"
    )
    panel._emit_log("[PICK] Secuencia manual iniciada (ruta directa con DIRECT IK trazable)")
    panel._emit_log("[PICK] target_source=selected_demo_object")
    panel._set_status("Pick demo: ejecutando secuencia manual…")
    panel._set_motion_lock(True)

    def worker():
        try:
            move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
            _move_sec_override = os.environ.get("PANEL_PICK_DEMO_MOVE_SEC")
            if _move_sec_override:
                try:
                    move_sec = max(1.0, float(_move_sec_override))
                except Exception:
                    pass
            home_pose = panel._get_home_joint_pose()
            selected_base_anchor_raw = getattr(panel, "_selected_base", None)
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
                return _pick_demo_tuple3(data)

            selected_base_anchor = _tuple3(selected_base_anchor_raw)
            _pick_demo_cycle_object_world = None
            _pick_demo_cycle_object_base = None
            _pick_demo_cycle_object_source = "none"
            _pick_demo_cycle_object_reason = "not_selected"
            _pick_demo_cycle_object_promoted_stable = False
            _pick_demo_cycle_object_snapshot_age_sec = None
            _pick_demo_cycle_object_stable_age_sec = None
            _pick_demo_cycle_object_selected_divergence_m = None

            def _clear_cycle_object_reference(*, reason: str) -> None:
                nonlocal _pick_demo_cycle_object_world
                nonlocal _pick_demo_cycle_object_base
                nonlocal _pick_demo_cycle_object_source
                nonlocal _pick_demo_cycle_object_reason
                nonlocal _pick_demo_cycle_object_promoted_stable
                nonlocal _pick_demo_cycle_object_snapshot_age_sec
                nonlocal _pick_demo_cycle_object_stable_age_sec
                nonlocal _pick_demo_cycle_object_selected_divergence_m

                if (
                    _pick_demo_cycle_object_world is None
                    and _pick_demo_cycle_object_base is None
                    and str(_pick_demo_cycle_object_source or "none") == "none"
                ):
                    return
                prev_world = _tuple3(_pick_demo_cycle_object_world)
                prev_base = _tuple3(_pick_demo_cycle_object_base)
                prev_source = str(_pick_demo_cycle_object_source or "none")
                _pick_demo_cycle_object_world = None
                _pick_demo_cycle_object_base = None
                _pick_demo_cycle_object_source = "none"
                _pick_demo_cycle_object_reason = str(reason or "cleared")
                _pick_demo_cycle_object_promoted_stable = False
                _pick_demo_cycle_object_snapshot_age_sec = None
                _pick_demo_cycle_object_stable_age_sec = None
                _pick_demo_cycle_object_selected_divergence_m = None
                _append_trace(
                    "[PICK][DIRECT][CYCLE_REF][CLEAR] tag=[DIRECT][CYCLE_REF][CLEAR] "
                    f"reason={_pick_demo_cycle_object_reason} prev_source={prev_source} "
                    f"prev_world={_fmt_vec(prev_world)} prev_base={_fmt_vec(prev_base)}"
                )

            # DIRECTO usa rg2_pinch_center como marco operacional de agarre.
            # rg2_pinch_center ES el punto de contacto — ya está al nivel de las yemas.
            # GRIPPER_TCP_Z_OFFSET=0.05 fue diseñado para rg2_tcp (base del gripper,
            # 5 cm sobre las yemas); aplicarlo a rg2_pinch_center eleva el agarre 5 cm
            # por encima del objeto, impidiendo el contacto físico.
            # Para rg2_pinch_center el offset correcto contra el centro del objeto es 0.
            _DIRECTO_GRASP_Z_RAW = float(
                os.environ.get("PANEL_PICK_DEMO_GRASP_TCP_Z_OFFSET_M", "0.0") or "0.0"
            )
            _DIRECTO_GRASP_Z = _effective_direct_grasp_z(
                DIRECT_SOURCE_FRAME,
                _DIRECTO_GRASP_Z_RAW,
            )
            grasp_z_for_source_frame = (
                0.0 if DIRECT_SOURCE_FRAME == "rg2_pinch_center" else _DIRECTO_GRASP_Z
            )
            if (
                DIRECT_SOURCE_FRAME == "rg2_pinch_center"
                and abs(_DIRECTO_GRASP_Z_RAW) > 1e-9
            ):
                panel._emit_log(
                    "[PICK][DIRECT][TARGET_Z] "
                    f"source_frame={DIRECT_SOURCE_FRAME} "
                    f"requested_offset_m={_DIRECTO_GRASP_Z_RAW:.3f} "
                    f"effective_offset_m={_DIRECTO_GRASP_Z:.3f} "
                    "note=legacy_vertical_offset_suppressed_for_pinch_center"
                )

            def _fmt_vec(vec) -> str:
                return _pick_demo_fmt_vec(vec)

            def _fmt_scalar(value, *, digits: int = 3) -> str:
                return _pick_demo_fmt_scalar(value, digits=digits)

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

            def _resolved_align_object_base() -> tuple[tuple[float, float, float] | None, str, dict]:
                obj_base_live = _tuple3(_live_object_base())
                anchor_delta = _vector_minus(selected_base_anchor, obj_base_live)
                anchor_norm = _vec_norm(anchor_delta)
                stale_tol_m = max(
                    0.01,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_SELECTED_BASE_STALE_TOL_M",
                            "0.080",
                        )
                        or 0.080
                    ),
                )
                extra = {
                    "selected_base_anchor": selected_base_anchor,
                    "live_object_base": obj_base_live,
                    "anchor_live_delta": anchor_delta,
                    "anchor_live_norm_m": anchor_norm,
                    "stale_tol_m": stale_tol_m,
                }
                if selected_base_anchor is not None and obj_base_live is not None:
                    if anchor_norm is not None and anchor_norm > stale_tol_m:
                        _append_trace(
                            "[PICK][DIRECT][ANCHOR] "
                            "selected_base_anchor_stale=true "
                            f"anchor={_fmt_vec(selected_base_anchor)} "
                            f"live={_fmt_vec(obj_base_live)} "
                            f"delta={_fmt_vec(anchor_delta)} "
                            f"norm_m={_fmt_scalar(anchor_norm)} "
                            f"stale_tol_m={_fmt_scalar(stale_tol_m)} "
                            "fallback=live_object_base"
                        )
                        return obj_base_live, "live_object_base_stale_fallback", extra
                    return selected_base_anchor, "selected_base_anchor", extra
                if obj_base_live is not None:
                    return obj_base_live, "live_object_base", extra
                if selected_base_anchor is not None:
                    return selected_base_anchor, "selected_base_anchor_no_live_confirm", extra
                return None, "target_unavailable", extra

            def _pose_position(target_frame: str, source_frame: str, *, timeout_sec: float = 0.20):
                pose, _pose_err = get_pose(target_frame, source_frame, timeout_sec=timeout_sec)
                if pose is None:
                    return None
                return _tuple3(pose.get("position"))

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
                def _local_joint_target_ok(local_tol_rad: float):
                    snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
                    if not snapshot:
                        return False, "no_local_joint_state"
                    parts = []
                    for idx, name in enumerate(UR5_JOINT_NAMES):
                        if idx >= len(joints):
                            break
                        curr = snapshot.get(name)
                        if curr is None:
                            parts.append(f"{name}=n/a")
                            return False, " ".join(parts)
                        diff = abs(angle_shortest_diff_rad(curr, joints[idx]))
                        parts.append(f"{name}={diff:.3f}")
                        if diff > float(local_tol_rad):
                            return False, " ".join(parts)
                    return True, " ".join(parts)

                panel._emit_log(f"[PICK] Paso joint: {label}")
                local_ok_before, local_diffs_before = _local_joint_target_ok(tol_rad)
                if local_ok_before:
                    panel._emit_log(
                        "[PICK][DIRECT][ROUTE] "
                        f"phase={label} joint_target_already_satisfied=true "
                        f"source=local_joint_state diffs={local_diffs_before}"
                    )
                    return
                ok, info = panel._publish_joint_trajectory(joints, move_sec)
                if not ok:
                    raise RuntimeError(f"{label} fallo: {info}")
                wait_timeout = move_sec + 2.0 if timeout_sec is None else timeout_sec
                try:
                    _step_extra = float(os.environ.get("PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC", "0") or "0")
                except Exception:
                    _step_extra = 0.0
                wait_timeout += _step_extra
                if panel._wait_for_joint_target(joints, wait_timeout, tol_rad=tol_rad):
                    return
                local_ok_after_wait, local_diffs_after_wait = _local_joint_target_ok(max(tol_rad, 0.02))
                if local_ok_after_wait:
                    panel._emit_log(
                        "[PICK][DIRECT][ROUTE] "
                        f"phase={label} joint_target_accept_after_wait_timeout=true "
                        f"source=local_joint_state diffs={local_diffs_after_wait}"
                    )
                    return
                if label in {"HOME", "MESA", "PICK_IMAGE", "PICK_PRE_CLOSE_REF", "HOME_WITH_OBJECT", "CESTA", "CESTA_RELEASE", "HOME_FINAL"}:
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
                    local_ok_after_retry, local_diffs_after_retry = _local_joint_target_ok(retry_tol)
                    if local_ok_after_retry:
                        panel._emit_log(
                            "[PICK][DIRECT][ROUTE] "
                            f"phase={label} joint_target_accept_after_retry_timeout=true "
                            f"source=local_joint_state diffs={local_diffs_after_retry}"
                        )
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
                if _pick_demo_cycle_object_world is not None:
                    _append_trace(
                        "[PICK][DIRECT][CYCLE_REF][USE] tag=[DIRECT][CYCLE_REF][USE] "
                        "phase=RUNTIME consumer=_live_object_world "
                        f"source={_pick_demo_cycle_object_source or 'none'} "
                        f"cycle_object_world={_fmt_vec(_pick_demo_cycle_object_world)} "
                        f"cycle_object_base={_fmt_vec(_pick_demo_cycle_object_base)}"
                    )
                    return _tuple3(_pick_demo_cycle_object_world)
                result = _resolve_live_object_world(
                    panel,
                    PICK_DEMO_OBJECT_NAME,
                    trace_fn=_append_trace,
                )
                return _tuple3(result.get("world"))

            def _live_object_base():
                if _pick_demo_cycle_object_base is not None:
                    _append_trace(
                        "[PICK][DIRECT][CYCLE_REF][USE] tag=[DIRECT][CYCLE_REF][USE] "
                        "phase=RUNTIME consumer=_live_object_base "
                        f"source={_pick_demo_cycle_object_source or 'none'} "
                        f"cycle_object_world={_fmt_vec(_pick_demo_cycle_object_world)} "
                        f"cycle_object_base={_fmt_vec(_pick_demo_cycle_object_base)}"
                    )
                    return _tuple3(_pick_demo_cycle_object_base)
                result = _resolve_live_object_base(
                    panel,
                    PICK_DEMO_OBJECT_NAME,
                    trace_fn=_append_trace,
                )
                return _tuple3(result.get("base"))

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

            def _fresh_gazebo_object_world():
                """Lee la posición del objeto DIRECTAMENTE de Gazebo (/world/.../pose/info).

                Ignora la referencia de ciclo congelada (_pick_demo_cycle_object_world) para
                que la validación de transporte siempre mida el desplazamiento REAL observable,
                no una estimación cacheada del panel.  Si el snapshot no está disponible cae
                de vuelta a _live_object_world() (conservador).

                Señal físicamente fiable: /world/{world}/pose/info ← fuente de verdad Gazebo.
                """
                try:
                    if (
                        getattr(panel, "_ros_worker_started", False)
                        and getattr(panel, "ros_worker", None) is not None
                    ):
                        pose_map, _ = panel.ros_worker.pose_snapshot()
                        live = (pose_map or {}).get(PICK_DEMO_OBJECT_NAME)
                        if live is not None and len(live) >= 3:
                            return (float(live[0]), float(live[1]), float(live[2]))
                except Exception:
                    pass
                return _live_object_world()

            def _fresh_gazebo_object_base():
                """Transforma la pose fresca de Gazebo a base_link para validaciones físicas."""
                obj_world = _tuple3(_fresh_gazebo_object_world())
                if obj_world is None:
                    return _live_object_base()
                world_frame = str(
                    getattr(panel, "_world_frame_last_first", lambda fallback=None: WORLD_FRAME or "world")(
                        WORLD_FRAME or "world"
                    )
                ).strip() or "world"
                base_frame = str(panel._business_base_frame() or BASE_FRAME or "base_link")
                try:
                    obj_base, _ = transform_point_to_frame(
                        obj_world,
                        base_frame,
                        source_frame=world_frame,
                    )
                    obj_base = _tuple3(obj_base)
                    if obj_base is not None:
                        return obj_base
                except Exception:
                    pass
                try:
                    return tuple(float(v) for v in world_to_base(*obj_world))
                except Exception:
                    return _live_object_base()

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

            def _camera_audit_meta() -> dict:
                topic = str(getattr(panel, "camera_topic", "") or "none").strip() or "none"
                image_ts = None
                frame_data = getattr(panel, "_last_camera_frame", None)
                if frame_data:
                    try:
                        image_ts = float(frame_data[3])
                    except Exception:
                        image_ts = None
                return {
                    "topic": topic,
                    "frame": "unknown",
                    "image_timestamp": image_ts,
                }

            def _audit_emit(
                stage: str,
                *,
                target_source: str,
                target_frame_original: str | None,
                target_pose_original=None,
                target_pose_world=None,
                target_pose_base_link=None,
                command_pose_sent=None,
                command_frame: str | None,
                command_joint_goal=None,
                extra: dict | None = None,
            ) -> None:
                world_frame = str(
                    getattr(panel, "_world_frame_last_first", lambda fallback=None: WORLD_FRAME or "world")(
                        WORLD_FRAME or "world"
                    )
                ).strip() or "world"
                try:
                    base_frame = str(panel._business_base_frame() or BASE_FRAME or "base_link")
                except Exception:
                    base_frame = str(BASE_FRAME or "base_link")
                object_world = _tuple3(_live_object_world())
                object_base = _tuple3(_live_object_base())
                tcp_world = _tuple3(_live_tcp_world())
                tcp_base = _tuple3(_live_tcp_base())
                tool0_world = _pose_position(world_frame, DIRECT_EXECUTION_FRAME, timeout_sec=0.12)
                tool0_base = _pose_position(base_frame, DIRECT_EXECUTION_FRAME, timeout_sec=0.12)
                pinch_world = _pose_position(world_frame, DIRECT_SOURCE_FRAME, timeout_sec=0.12)
                pinch_base = _pose_position(base_frame, DIRECT_SOURCE_FRAME, timeout_sec=0.12) or tcp_base
                rg2_tcp_world = _pose_position(world_frame, DIRECT_LEGACY_TCP_FRAME, timeout_sec=0.12)
                rg2_tcp_base = _pose_position(base_frame, DIRECT_LEGACY_TCP_FRAME, timeout_sec=0.12)
                camera_meta = _camera_audit_meta()
                selection_ts = float(getattr(panel, "_selection_timestamp", 0.0) or 0.0)
                selection_age = max(0.0, time.time() - selection_ts) if selection_ts > 0.0 else None
                selected_world = _tuple3(getattr(panel, "_selected_world", None))
                selected_base = _tuple3(getattr(panel, "_selected_base", None))
                panel_tcp_fk_base = _tuple3(getattr(panel, "_last_tcp_base", None))
                panel_tcp_fk_rpy_deg = _tuple3(getattr(panel, "_last_tcp_rpy_deg", None))
                panel_tcp_fk_age = max(
                    0.0,
                    time.monotonic() - float(getattr(panel, "_last_tcp_fk_ts", 0.0) or 0.0),
                ) if float(getattr(panel, "_last_tcp_fk_ts", 0.0) or 0.0) > 0.0 else None
                panel_trace_tcp_base = _tuple3(getattr(panel, "_last_trace_tcp_base", None))
                panel_trace_tcp_rpy_deg = _tuple3(getattr(panel, "_last_trace_tcp_rpy_deg", None))
                panel_trace_tcp_age = max(
                    0.0,
                    time.monotonic() - float(getattr(panel, "_last_trace_tcp_ts", 0.0) or 0.0),
                ) if float(getattr(panel, "_last_trace_tcp_ts", 0.0) or 0.0) > 0.0 else None
                panel_object_age = getattr(panel, "_last_trace_object_age_sec", None)
                delta_world = _vector_minus(tcp_world, object_world)
                delta_base = _vector_minus(tcp_base, object_base)
                delta_pinch_world = _vector_minus(pinch_world, object_world)
                delta_pinch_base = _vector_minus(pinch_base, object_base)
                delta_panel_live = _vector_minus(panel_tcp_fk_base, tcp_base)
                delta_panel_live_norm = _vec_norm(delta_panel_live)
                legacy_gap = _vector_minus(rg2_tcp_base, pinch_base)
                legacy_gap_norm = _vec_norm(legacy_gap)
                extra_payload = _json_safe(extra) or {}
                _append_trace(
                    f"{DIRECT_GRASP_AUDIT_PREFIX} "
                    f"stage={stage} "
                    f"timestamp={_iso_now()} "
                    f"request_id={run_id} "
                    f"grasp_mode=direct_object "
                    f"selected_object_name={selected_name or 'none'} "
                    f"selected_object_id=n/a "
                    f"user_selected_name={user_selected or 'none'} "
                    f"selection_age_sec={_fmt_scalar(selection_age)} "
                    f"target_source={target_source or 'none'} "
                    f"target_frame_original={target_frame_original or 'none'} "
                    f"target_pose_original={_fmt_vec(target_pose_original)} "
                    f"target_pose_world={_fmt_vec(target_pose_world)} "
                    f"target_pose_base_link={_fmt_vec(target_pose_base_link)} "
                    f"selected_pose_world={_fmt_vec(selected_world)} "
                    f"selected_pose_base_link={_fmt_vec(selected_base)} "
                    f"object_pose_world={_fmt_vec(object_world)} "
                    f"object_pose_base_link={_fmt_vec(object_base)} "
                    f"tcp_pose_world={_fmt_vec(tcp_world)} "
                    f"tcp_pose_base_link={_fmt_vec(tcp_base)} "
                    f"tool0_pose_world={_fmt_vec(tool0_world)} "
                    f"tool0_pose_base_link={_fmt_vec(tool0_base)} "
                    f"rg2_pinch_center_pose_world={_fmt_vec(pinch_world)} "
                    f"rg2_pinch_center_pose_base_link={_fmt_vec(pinch_base)} "
                    f"rg2_tcp_pose_world={_fmt_vec(rg2_tcp_world)} "
                    f"rg2_tcp_pose_base_link={_fmt_vec(rg2_tcp_base)} "
                    f"delta_object_tcp_world={_fmt_vec(delta_world)} "
                    f"delta_object_tcp_base={_fmt_vec(delta_base)} "
                    f"delta_object_pinch_center_world={_fmt_vec(delta_pinch_world)} "
                    f"delta_object_pinch_center_base={_fmt_vec(delta_pinch_base)} "
                    f"command_pose_sent={_fmt_vec(command_pose_sent)} "
                    f"command_frame={command_frame or 'none'} "
                    f"command_joint_goal={json.dumps(_json_safe(command_joint_goal), ensure_ascii=True)} "
                    f"camera_topic={camera_meta['topic']} "
                    f"camera_frame={camera_meta['frame']} "
                    f"image_timestamp={_fmt_scalar(camera_meta['image_timestamp'])} "
                    f"pose_from_image=false "
                    f"panel_tcp_fk_base={_fmt_vec(panel_tcp_fk_base)} "
                    f"panel_tcp_fk_rpy_deg={_fmt_vec(panel_tcp_fk_rpy_deg)} "
                    f"panel_trace_tcp_base={_fmt_vec(panel_trace_tcp_base)} "
                    f"panel_trace_tcp_rpy_deg={_fmt_vec(panel_trace_tcp_rpy_deg)} "
                    f"delta_panel_tcp_live={_fmt_vec(delta_panel_live)} "
                    f"delta_panel_tcp_live_norm_m={_fmt_scalar(delta_panel_live_norm)} "
                    f"panel_tcp_fk_age_sec={_fmt_scalar(panel_tcp_fk_age)} "
                    f"panel_trace_tcp_age_sec={_fmt_scalar(panel_trace_tcp_age)} "
                    f"panel_object_age_sec={_fmt_scalar(panel_object_age)} "
                    f"world_frame={world_frame} "
                    f"base_frame={base_frame} "
                    f"extra={json.dumps(extra_payload, ensure_ascii=True, sort_keys=True)}"
                )
                _append_trace(
                    "[PICK][DIRECT][BUTTON] "
                    f"stage={stage} request_id={run_id} grasp_mode=direct_object "
                    f"selected_object={selected_name or 'none'} user_selected={user_selected or 'none'} "
                    f"target_source={target_source or 'none'} target_frame_original={target_frame_original or 'none'} "
                    f"selection_age_sec={_fmt_scalar(selection_age)} pose_from_image=false"
                )
                _append_trace(
                    "[PICK][DIRECT][SELECT] "
                    f"stage={stage} selected_pose_world={_fmt_vec(selected_world)} "
                    f"selected_pose_base_link={_fmt_vec(selected_base)} "
                    f"target_pose_original={_fmt_vec(target_pose_original)} "
                    f"target_pose_world={_fmt_vec(target_pose_world)} "
                    f"target_pose_base_link={_fmt_vec(target_pose_base_link)} "
                    f"panel_object_age_sec={_fmt_scalar(panel_object_age)}"
                )
                _append_trace(
                    "[PICK][DIRECT][LIVE_OBJECT] "
                    f"stage={stage} object_pose_world={_fmt_vec(object_world)} "
                    f"object_pose_base_link={_fmt_vec(object_base)} world_frame={world_frame} base_frame={base_frame}"
                )
                _append_trace(
                    "[PICK][DIRECT][TCP_LIVE] "
                    f"stage={stage} tcp_pose_world={_fmt_vec(tcp_world)} "
                    f"tcp_pose_base_link={_fmt_vec(tcp_base)} "
                    f"tool0_pose_world={_fmt_vec(tool0_world)} tool0_pose_base_link={_fmt_vec(tool0_base)} "
                    f"rg2_pinch_center_pose_world={_fmt_vec(pinch_world)} rg2_pinch_center_pose_base_link={_fmt_vec(pinch_base)} "
                    f"rg2_tcp_pose_world={_fmt_vec(rg2_tcp_world)} rg2_tcp_pose_base_link={_fmt_vec(rg2_tcp_base)} "
                    f"panel_trace_tcp_base={_fmt_vec(panel_trace_tcp_base)} panel_trace_tcp_rpy_deg={_fmt_vec(panel_trace_tcp_rpy_deg)} "
                    f"panel_trace_tcp_age_sec={_fmt_scalar(panel_trace_tcp_age)}"
                )
                _append_trace(
                    "[RG2][AUDIT][CONTROL] "
                    f"stage={stage} reasoning_frame={DIRECT_SOURCE_FRAME} "
                    f"legacy_tcp_frame={DIRECT_LEGACY_TCP_FRAME} execution_frame={DIRECT_EXECUTION_FRAME} "
                    f"tool0_pose_base_link={_fmt_vec(tool0_base)} "
                    f"rg2_pinch_center_pose_base_link={_fmt_vec(pinch_base)} "
                    f"rg2_tcp_pose_base_link={_fmt_vec(rg2_tcp_base)}"
                )
                _append_trace(
                    "[RG2][AUDIT][COMPARE] "
                    f"stage={stage} object_pose_base_link={_fmt_vec(object_base)} "
                    f"tool0_pose_base_link={_fmt_vec(tool0_base)} "
                    f"rg2_pinch_center_pose_base_link={_fmt_vec(pinch_base)} "
                    f"rg2_tcp_pose_base_link={_fmt_vec(rg2_tcp_base)} "
                    f"delta_object_pinch_center_base={_fmt_vec(delta_pinch_base)} "
                    f"delta_object_tcp_base={_fmt_vec(delta_base)} "
                    f"legacy_tcp_vs_pinch_center_base={_fmt_vec(legacy_gap)} "
                    f"legacy_tcp_vs_pinch_center_dist_m={_fmt_scalar(legacy_gap_norm)}"
                )
                _append_trace(
                    "[PICK][DIRECT][PANEL_TRACE] "
                    f"stage={stage} panel_tcp_fk_base={_fmt_vec(panel_tcp_fk_base)} "
                    f"panel_tcp_fk_rpy_deg={_fmt_vec(panel_tcp_fk_rpy_deg)} "
                    f"tcp_live_base={_fmt_vec(tcp_base)} "
                    f"delta_panel_tcp_live={_fmt_vec(delta_panel_live)} "
                    f"delta_panel_tcp_live_norm_m={_fmt_scalar(delta_panel_live_norm)} "
                    f"panel_tcp_fk_age_sec={_fmt_scalar(panel_tcp_fk_age)}"
                )
                if delta_panel_live_norm is not None and delta_panel_live_norm > 0.02:
                    _append_trace(
                        "[PICK][DIRECT][DIVERGENCE] "
                        "kind=panel_fk_vs_live_tf "
                        f"stage={stage} delta_m={_fmt_scalar(delta_panel_live_norm)} "
                        f"panel_tcp_fk_base={_fmt_vec(panel_tcp_fk_base)} "
                        f"tcp_live_base={_fmt_vec(tcp_base)} "
                        f"delta={_fmt_vec(delta_panel_live)} "
                        "note=panel_fk_is_model_pose_not_runtime_rg2_tcp"
                    )

            _audit_emit(
                "BUTTON_PRESS",
                target_source="selected_demo_object",
                target_frame_original=str(getattr(panel, "_selected_base_frame", "") or "base_link"),
                target_pose_original=selected_base_anchor or _tuple3(getattr(panel, "_selected_world", None)),
                target_pose_world=_tuple3(_live_object_world()),
                target_pose_base_link=_tuple3(_live_object_base()),
                command_pose_sent=None,
                command_frame=None,
                extra={
                    "route_requested": requested_route_mode,
                    "route_selected": route_selected,
                    "selected_base_anchor": selected_base_anchor,
                },
            )

            cycle_ref = _select_pick_demo_cycle_object_reference(
                panel,
                PICK_DEMO_OBJECT_NAME,
                selected_base_anchor=selected_base_anchor,
                trace_fn=_append_trace,
            )
            if bool(cycle_ref.get("ok")):
                _pick_demo_cycle_object_world = _tuple3(cycle_ref.get("world"))
                _pick_demo_cycle_object_base = _tuple3(cycle_ref.get("base"))
                _pick_demo_cycle_object_source = str(cycle_ref.get("source") or "none")
                _pick_demo_cycle_object_reason = str(cycle_ref.get("reason") or "none")
                _pick_demo_cycle_object_promoted_stable = bool(cycle_ref.get("promoted_stable"))
                _pick_demo_cycle_object_snapshot_age_sec = cycle_ref.get("snapshot_age_sec")
                _pick_demo_cycle_object_stable_age_sec = cycle_ref.get("stable_age_sec")
                _pick_demo_cycle_object_selected_divergence_m = cycle_ref.get("selected_stable_divergence_m")
                _append_trace(
                    "[PICK][DIRECT][CYCLE_REF][USE] tag=[DIRECT][CYCLE_REF][USE] "
                    "phase=BUTTON_PRESS consumer=cycle_ref_init "
                    f"source={_pick_demo_cycle_object_source} promoted_stable={str(_pick_demo_cycle_object_promoted_stable).lower()} "
                    f"snapshot_age_sec={_fmt_scalar(_pick_demo_cycle_object_snapshot_age_sec)} "
                    f"stable_age_sec={_fmt_scalar(_pick_demo_cycle_object_stable_age_sec)} "
                    f"selected_stable_divergence_m={_fmt_scalar(_pick_demo_cycle_object_selected_divergence_m)} "
                    f"cycle_object_world={_fmt_vec(_pick_demo_cycle_object_world)} "
                    f"cycle_object_base={_fmt_vec(_pick_demo_cycle_object_base)}"
                )
                panel._emit_log(
                    "[PICK][DIRECT][SELECT] "
                    "cycle_tag=[DIRECT][CYCLE_REF][SELECT] "
                    f"cycle_source={_pick_demo_cycle_object_source} "
                    f"promoted_stable={str(_pick_demo_cycle_object_promoted_stable).lower()} "
                    f"cycle_object_base={_fmt_vec(_pick_demo_cycle_object_base)} "
                    f"cycle_object_world={_fmt_vec(_pick_demo_cycle_object_world)}"
                )
                if _pick_demo_cycle_object_promoted_stable:
                    panel._emit_log(
                        "[PICK][DIRECT][SELECT] "
                        "cycle_tag=[DIRECT][CYCLE_REF][PROMOTE_STABLE] "
                        f"stable_age_sec={_fmt_scalar(_pick_demo_cycle_object_stable_age_sec)} "
                        f"selected_stable_divergence_m={_fmt_scalar(_pick_demo_cycle_object_selected_divergence_m)}"
                    )
            else:
                _pick_demo_cycle_object_reason = str(cycle_ref.get("reason") or "cycle_reference_unavailable")
                reject_reasons = ",".join(cycle_ref.get("reject_reasons") or []) or "none"
                _append_trace(
                    "[PICK][DIRECT][CYCLE_REF][ABORT] tag=[DIRECT][CYCLE_REF][ABORT] "
                    "phase=BUTTON_PRESS reason=cycle_reference_unavailable "
                    f"cycle_reason={_pick_demo_cycle_object_reason} reject_reasons={reject_reasons}"
                )
                panel._emit_log(
                    "[PICK][DIRECT][ABORT] phase=BUTTON_PRESS reason=cycle_reference_unavailable"
                )
                panel._emit_log(
                    "[PICK][DIRECT][ABORT] "
                    "cycle_tag=[DIRECT][CYCLE_REF][ABORT] "
                    f"cycle_reason={_pick_demo_cycle_object_reason} reject_reasons={reject_reasons}"
                )
                raise RuntimeError("demo_cycle_reference_unavailable_before_approach_coarse")

            # ------------------------------------------------------------------
            # [COMPARE] — referencia estática Directo2 vs objeto live en BUTTON_PRESS
            # Emite FK del preset JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD contra
            # la posición actual del objeto para detectar si el preset es válido aquí.
            # ------------------------------------------------------------------
            try:
                _q_ref = list(JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD)
                _fk_ref = fk_ur5(_q_ref)
                # FK devuelve posición en base_link_inertia (modelo). Flip Rz(π): base_link = (-mx, -my, mz)
                _ref_tool0_base = (-float(_fk_ref[0][0]), -float(_fk_ref[0][1]), float(_fk_ref[0][2]))
                # rg2_tcp = tool0 + (0, 0, 0.175) en frame local del gripper.
                # Aproximación: desplazamiento en Z de base_link asumiendo gripper apuntando hacia abajo
                # (conservador, válido para comparación de magnitud).
                _ref_rg2_tcp_base = (
                    _ref_tool0_base[0],
                    _ref_tool0_base[1],
                    _ref_tool0_base[2] + DIRECT_TOOL0_TO_RG2_TCP_Z_M,
                )
                _obj_live_base = _live_object_base()
                _xy_dist_ref = None
                _dist3d_ref = None
                _dz_ref = None
                if _obj_live_base is not None:
                    _dx = _ref_rg2_tcp_base[0] - float(_obj_live_base[0])
                    _dy = _ref_rg2_tcp_base[1] - float(_obj_live_base[1])
                    _dz_ref = _ref_rg2_tcp_base[2] - float(_obj_live_base[2])
                    _xy_dist_ref = math.sqrt(_dx ** 2 + _dy ** 2)
                    _dist3d_ref = math.sqrt(_dx ** 2 + _dy ** 2 + _dz_ref ** 2)
                _s_xy = _fmt_scalar(_xy_dist_ref, digits=4)
                _s_d3 = _fmt_scalar(_dist3d_ref, digits=4)
                _s_dz = _fmt_scalar(_dz_ref, digits=4)
                panel._emit_log(
                    "[COMPARE][DIRECT2][GOOD_REF] "
                    f"preset=JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD "
                    f"tool0_base=({_ref_tool0_base[0]:.3f},{_ref_tool0_base[1]:.3f},{_ref_tool0_base[2]:.3f}) "
                    f"rg2_tcp_base=({_ref_rg2_tcp_base[0]:.3f},{_ref_rg2_tcp_base[1]:.3f},{_ref_rg2_tcp_base[2]:.3f}) "
                    f"object_base={_fmt_vec(_obj_live_base)} "
                    f"xy_dist_rg2tcp_obj={_s_xy} "
                    f"dist3d_rg2tcp_obj={_s_d3} "
                    f"dz_rg2tcp_obj={_s_dz} "
                    f"gripper_tcp_z_offset={float(GRIPPER_TCP_Z_OFFSET):.3f} "
                    "note=FK_preset_vs_live_object_at_button_press"
                )
                _verdict_ref = "PRESET_VALID_FOR_CURRENT_OBJECT_POSITION" if (
                    _xy_dist_ref is not None and _xy_dist_ref < 0.05
                ) else "PRESET_POSITION_MISMATCH"
                panel._emit_log(
                    f"[COMPARE][DIRECT2][GOOD_REF] verdict={_verdict_ref} "
                    f"xy_dist_threshold=0.050 "
                    f"xy_dist_actual={_s_xy}"
                )
            except Exception as _exc_compare:
                panel._emit_log(f"[COMPARE][DIRECT2][GOOD_REF] error={_exc_compare}")

            def _current_joint_seed(*, return_source: bool = False):
                # DIRECTO should prefer the live arm state as IK seed/orientation
                # reference.  A fixed HOME override is still kept as fallback when the
                # live snapshot is incomplete.
                seed = []
                try:
                    snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
                except Exception:
                    snapshot = {}
                for joint_name in UR5_JOINT_NAMES:
                    if joint_name not in snapshot:
                        seed = []
                        break
                    seed.append(float(snapshot[joint_name]))
                if len(seed) == len(UR5_JOINT_NAMES):
                    return (seed, "live_joint_state") if return_source else seed

                _seed_override_str = os.environ.get("PANEL_PICK_DEMO_IK_SEED_JOINTS", "").strip()
                if _seed_override_str:
                    try:
                        _override = [float(v.strip()) for v in _seed_override_str.split(",")]
                        if len(_override) == 6:
                            return (_override, "env_override") if return_source else _override
                    except Exception:
                        pass

                fallback = list(JOINT_GRASP_DOWN_POSE_RAD)
                return (fallback, "preset_fallback") if return_source else fallback

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
                            # _last_joint_wall uses monotonic time; keep the same base here.
                            joint_state_age_sec = max(0.0, time.monotonic() - float(ts))
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
                z_error = abs(z_gap - _DIRECTO_GRASP_Z)
                tcp_obj_dist = _dist(tcp_base, obj_base)
                xy_tol = max(
                    0.01,
                    float(os.environ.get("PANEL_PICK_DEMO_CLOSE_XY_TOL_M", "0.035") or 0.035),
                )
                z_tol = max(
                    0.01,
                    float(os.environ.get("PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M", "0.018") or 0.018),
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
                z_error = abs(z_gap - _DIRECTO_GRASP_Z)
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
                            os.environ.get("PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M", "0.015"),
                        )
                        or 0.015
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
                z_error = abs(float(dz) - _DIRECTO_GRASP_Z)
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
                step_gate_fn = getattr(panel, "_step_wait_for_phase", None)
                if callable(step_gate_fn):
                    step_gate_fn(
                        f"DIRECT.{phase}",
                        flow="DIRECT",
                        position=_tuple3(target_base) or _tuple3(last_target.get("target_base")),
                        decision=str(note or "").strip(),
                        object_position=_tuple3(_live_object_base()),
                    )
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

            def _move_tcp_direct(
                *,
                label: str,
                target_tcp_runtime,
                timeout_sec: float,
                audit_target_source: str = "runtime_target",
                target_pose_original=None,
                target_frame_original: str = "base_link",
                rot_weight: float = 0.35,
                ik_err_tol: float = 0.035,
                joint_weight: float = -1.0,
            ) -> dict:
                def _resolve_direct_execution_target(
                    tcp_target_base,
                    tool_rot,
                ) -> dict:
                    # DIRECT keeps rg2_pinch_center as the operational grasp semantics.
                    # Numeric UR5 IK still solves in tool0, so the only allowed
                    # conversion lives here as a fixed, traceable transform.
                    env_xyz = str(
                        os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_XYZ", "") or ""
                    ).strip()
                    env_value = os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M", "")
                    local_offset = None
                    offset_source = None
                    if env_xyz:
                        try:
                            parts = [float(v.strip()) for v in env_xyz.split(",")]
                            if len(parts) == 3:
                                local_offset = (parts[0], parts[1], parts[2])
                                offset_source = "env:PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_XYZ"
                        except Exception:
                            local_offset = None
                    if local_offset is None:
                        pose_tool0_tcp, _pose_err = get_pose(
                            DIRECT_EXECUTION_FRAME,
                            DIRECT_SOURCE_FRAME,
                            timeout_sec=0.25,
                        )
                        if pose_tool0_tcp:
                            try:
                                px, py, pz = pose_tool0_tcp["position"]
                                local_offset = (float(px), float(py), float(pz))
                                offset_source = f"tf:{DIRECT_EXECUTION_FRAME}<-{DIRECT_SOURCE_FRAME}"
                            except Exception:
                                local_offset = None
                    if local_offset is None:
                        default_offset_m = DIRECT_TOOL0_TO_RG2_TCP_Z_M
                        tcp_offset_m = max(
                            0.0,
                            float(env_value or default_offset_m),
                        )
                        local_offset = (0.0, 0.0, tcp_offset_m)
                        offset_source = (
                            "env:PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M"
                            if str(env_value).strip()
                            else "default:ur5.urdf.xacro/rg2_pinch_center_joint"
                        )
                    tcp_offset_m = float(
                        math.sqrt(
                            (float(local_offset[0]) ** 2)
                            + (float(local_offset[1]) ** 2)
                            + (float(local_offset[2]) ** 2)
                        )
                    )
                    # URDF chain uses base_link_inertia as kinematic root with a fixed
                    # base_link->base_link_inertia rotation Rz(pi); convert runtime target
                    # (base_link) into the FK/IK model frame before solving.
                    target_model = (
                        -float(tcp_target_base[0]),
                        -float(tcp_target_base[1]),
                        float(tcp_target_base[2]),
                    )
                    offset_vector = (
                        float(tool_rot[0, 0]) * float(local_offset[0])
                        + float(tool_rot[0, 1]) * float(local_offset[1])
                        + float(tool_rot[0, 2]) * float(local_offset[2]),
                        float(tool_rot[1, 0]) * float(local_offset[0])
                        + float(tool_rot[1, 1]) * float(local_offset[1])
                        + float(tool_rot[1, 2]) * float(local_offset[2]),
                        float(tool_rot[2, 0]) * float(local_offset[0])
                        + float(tool_rot[2, 1]) * float(local_offset[1])
                        + float(tool_rot[2, 2]) * float(local_offset[2]),
                    )
                    # operational grasp frame = tool0 + (R_tool0 * local_offset); solve tool0 target.
                    execution_target_tool0 = (
                        float(target_model[0]) - float(offset_vector[0]),
                        float(target_model[1]) - float(offset_vector[1]),
                        float(target_model[2]) - float(offset_vector[2]),
                    )
                    return {
                        "source_frame": DIRECT_SOURCE_FRAME,
                        "source_pose": _tuple3(tcp_target_base),
                        "target_model": _tuple3(target_model),
                        "execution_frame": DIRECT_EXECUTION_FRAME,
                        "execution_pose": _tuple3(execution_target_tool0),
                        "offset_local": _tuple3(local_offset),
                        "offset_vector": _tuple3(offset_vector),
                        "offset_m": float(tcp_offset_m),
                        "offset_source": str(offset_source or "unknown"),
                        "offset_mode": "fixed_subtract",
                        "model_frame_note": "base_link_to_base_link_inertia_rz_pi",
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                    }

                tcp_base = _live_tcp_base()
                if tcp_base is None:
                    raise RuntimeError(f"{label.lower()}_tcp_pose_unavailable")
                seed, seed_source = _current_joint_seed(return_source=True)
                _seed_pos, target_rot = fk_ur5(seed)
                panel._emit_log(
                    "[PICK][DIRECT][IK_SEED] "
                    f"label={label} source={seed_source} "
                    f"joints={json.dumps(_json_safe(seed), ensure_ascii=True)}"
                )
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
                target_model = execution_semantics["target_model"]
                offset_vector = execution_semantics["offset_vector"]
                tcp_offset_m = float(execution_semantics["offset_m"])
                panel._emit_log(
                    "[PICK][DIRECT][FRAME] "
                    f"label={label} route={DIRECT_ROUTE_MODE} "
                    f"source_frame={execution_semantics['source_frame']} "
                    f"execution_frame={execution_semantics['execution_frame']} "
                    f"ik_mode={execution_semantics['ik_mode']} "
                    f"source_pose={_fmt_vec(execution_semantics['source_pose'])} "
                    f"target_model={_fmt_vec(target_model)} "
                    f"offset_local={_fmt_vec(execution_semantics['offset_local'])} "
                    f"offset_vector={_fmt_vec(offset_vector)} "
                    f"offset_m={tcp_offset_m:.3f} "
                    f"offset_source={execution_semantics['offset_source']} "
                    f"offset_mode={execution_semantics['offset_mode']} "
                    f"execution_pose={_fmt_vec(target_ik)}"
                )
                panel._emit_log(
                    "[PICK][DIRECT][FRAME_AUDIT] "
                    f"label={label} source_frame={execution_semantics['source_frame']} "
                    f"execution_frame={execution_semantics['execution_frame']} "
                    f"target_tcp_runtime={_fmt_vec(target_tcp_runtime)} "
                    f"target_model={_fmt_vec(target_model)} execution_pose={_fmt_vec(target_ik)} "
                    f"ik_mode={DIRECT_EXECUTION_IK_MODE}"
                )
                panel._emit_log(
                    "[PICK][DIRECT][OFFSET] "
                    f"label={label} offset_source={execution_semantics['offset_source']} "
                    f"offset_local={_fmt_vec(execution_semantics['offset_local'])} "
                    f"offset_vector={_fmt_vec(offset_vector)} "
                    f"offset_m={tcp_offset_m:.4f} offset_mode={execution_semantics['offset_mode']}"
                )
                _effective_joint_weight = (
                    float(joint_weight)
                    if float(joint_weight) >= 0.0
                    else max(
                        0.0,
                        float(
                            os.environ.get(
                                "PANEL_PICK_DEMO_DIRECT_IK_SEED_WEIGHT",
                                "0.035",
                            )
                            or 0.035
                        ),
                    )
                )
                solved_q, err_norm, ik_ok = ik_ur5(
                    target_ik,
                    target_rot,
                    seed,
                    max_iter=240,
                    pos_weight=1.0,
                    rot_weight=float(rot_weight),
                    joint_weight=_effective_joint_weight,
                )
                # Compute pure position error via FK to validate independently of joint_weight cost
                import numpy as _np_ik_check
                import math as _math_ik_retry
                _fk_pos_solved, _ = fk_ur5(solved_q)
                pos_err_m = float(_np_ik_check.linalg.norm(
                    _np_ik_check.asarray(_fk_pos_solved, dtype=float)
                    - _np_ik_check.asarray(target_ik, dtype=float)
                ))
                # If the solution deviates too far from the seed (any joint > 90° off
                # after 2π-normalization), the solver landed in the wrong configuration
                # branch.  Retry with escalating seed_weights to force it closer to the
                # seed configuration.  Stop as soon as we get a valid solution that is
                # nearer to the seed OR once we've exhausted the retry schedule.
                _TWO_PI_R = 2.0 * _math_ik_retry.pi
                def _seed_max_dev(q_arr, s_arr):
                    devs = [
                        abs(float(q) + _TWO_PI_R * round((float(s) - float(q)) / _TWO_PI_R) - float(s))
                        for q, s in zip(q_arr, s_arr)
                    ]
                    return max(devs)
                _IK_DEV_THRESHOLD = _math_ik_retry.pi / 2  # 90° — flag a wrong-branch solution
                if ik_ok and pos_err_m <= float(ik_err_tol) and _seed_max_dev(solved_q, seed) > _IK_DEV_THRESHOLD:
                    _best_q, _best_err, _best_ok = solved_q, pos_err_m, ik_ok
                    # Build a list of (seed_variant, seed_weight) pairs to try.
                    # The first group uses the original seed with escalating weights.
                    # The second group uses diverse seeds derived from known good
                    # configurations (e.g. run-4 APPROACH_COARSE solution that had
                    # wrist_2≈-π/2, which lands in the wrist_3≈0° branch).
                    _PI_H = _math_ik_retry.pi / 2
                    _diverse_seeds = [
                        # wrist_2 flipped to -π/2 (mimics run-4 elbow configuration)
                        [seed[0], seed[1], seed[2], seed[3], -_PI_H, seed[5]],
                        # wrist_2 at -π/2, shoulder_pan slightly negative
                        [-0.26, seed[1], seed[2], seed[3], -_PI_H, 0.0],
                    ]
                    _retry_schedule = (
                        [(seed, sw) for sw in [0.08, 0.15, 0.25, 0.35]]
                        + [(_ds, 0.035) for _ds in _diverse_seeds]
                        + [(_ds, 0.08) for _ds in _diverse_seeds]
                    )
                    for _rseed, _retry_sw in _retry_schedule:
                        _rq, _re_norm, _rok = ik_ur5(
                            target_ik,
                            target_rot,
                            _rseed,
                            max_iter=360,
                            pos_weight=1.0,
                            rot_weight=float(rot_weight),
                            joint_weight=_retry_sw,
                        )
                        _rfk, _ = fk_ur5(_rq)
                        _rpos_err = float(_np_ik_check.linalg.norm(
                            _np_ik_check.asarray(_rfk, dtype=float)
                            - _np_ik_check.asarray(target_ik, dtype=float)
                        ))
                        panel._emit_log(
                            f"[PICK][DIRECT][IK_RETRY] label={label} retry_seed_weight={_retry_sw:.3f} "
                            f"seed={'diverse' if _rseed is not seed else 'home'} "
                            f"pos_err_m={_rpos_err:.4f} ok={str(bool(_rok)).lower()} "
                            f"max_dev_rad={_seed_max_dev(_rq, seed):.3f}"
                        )
                        if _rok and _rpos_err <= float(ik_err_tol):
                            # Accept this solution if it's closer to the seed
                            if _seed_max_dev(_rq, seed) < _seed_max_dev(_best_q, seed):
                                _best_q, _best_err, _best_ok = _rq, _rpos_err, _rok
                            if _seed_max_dev(_best_q, seed) <= _IK_DEV_THRESHOLD:
                                break  # Good enough — stop retrying
                    solved_q, pos_err_m, ik_ok = _best_q, _best_err, _best_ok
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
                    f"rot_weight={float(rot_weight):.3f} "
                    f"ik_err_tol={float(ik_err_tol):.3f} "
                    f"seed_weight={_effective_joint_weight:.3f} "
                    f"err_norm={float(err_norm):.4f} pos_err_m={pos_err_m:.4f} success={str(bool(ik_ok)).lower()}"
                )
                panel._emit_log(
                    "[PICK][DIRECT][IK_TARGET] "
                    f"label={label} target_source={audit_target_source} "
                    f"target_frame_original={target_frame_original} "
                    f"target_pose_original={_fmt_vec(target_pose_original)} "
                    f"target_pose_base_link={_fmt_vec(target_tcp_runtime)} "
                    f"command_pose_sent={_fmt_vec(target_ik)} command_frame={DIRECT_EXECUTION_FRAME}"
                )
                _audit_emit(
                    "BEFORE_EXECUTION",
                    target_source=audit_target_source,
                    target_frame_original=target_frame_original,
                    target_pose_original=target_pose_original,
                    target_pose_world=_target_world_from_base(target_tcp_runtime),
                    target_pose_base_link=target_tcp_runtime,
                    command_pose_sent=target_ik,
                    command_frame=DIRECT_EXECUTION_FRAME,
                    command_joint_goal=solved_q.tolist(),
                    extra={
                        "label": label,
                        "ik_mode": DIRECT_EXECUTION_IK_MODE,
                        "offset_source": execution_semantics.get("offset_source"),
                        "offset_local": execution_semantics.get("offset_local"),
                        "offset_vector": execution_semantics.get("offset_vector"),
                        "target_model": execution_semantics.get("target_model"),
                        "runtime_delta": delta_runtime,
                    },
                )
                if (not ik_ok) or pos_err_m > float(ik_err_tol):
                    raise RuntimeError(
                        f"{label.lower()}_ik_failed pos_err_m={pos_err_m:.4f}"
                    )
                solved_q_list = [float(v) for v in solved_q.tolist()]
                # Normalize each joint angle to the equivalent value closest to the
                # seed.  The IK solver is free to return any angle modulo 2π; without
                # this step a solution like wrist_3=-3.28 rad (equivalent to +0.1 rad)
                # makes the trajectory controller attempt a 187° rotation that it cannot
                # follow within the allotted time.
                import math as _math_norm
                _TWO_PI = 2.0 * _math_norm.pi
                solved_q_list = [
                    float(q) + _TWO_PI * round((float(s) - float(q)) / _TWO_PI)
                    for q, s in zip(solved_q_list, seed)
                ]
                panel._emit_log(
                    "[PICK][DIRECT][JOINT_GOAL] "
                    f"label={label} joints={json.dumps(_json_safe(solved_q_list), ensure_ascii=True)} "
                    f"timeout_sec={float(timeout_sec):.3f}"
                )
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
                runtime_target_tol_m = _direct_runtime_target_tol_m(label)
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
                            ee_frame=DIRECT_SOURCE_FRAME,
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
                panel._emit_log(
                    "[PICK][DIRECT][EXEC_RESULT] "
                    f"label={label} ik_ok={str(bool(ik_ok)).lower()} "
                    f"runtime_target_ok={json.dumps(_json_safe(runtime_target_ok), ensure_ascii=True)} "
                    f"runtime_target_dist={_fmt_scalar(runtime_target_dist)} "
                    f"runtime_target_pos={_fmt_vec(runtime_target_pos)} "
                    f"joint_goal={json.dumps(_json_safe(solved_q_list), ensure_ascii=True)}"
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

            def _joint_preset_fallback_ok(
                label: str,
                joints,
                *,
                target_base=None,
                obj_base=None,
            ) -> bool:
                try:
                    fk_pos, _fk_rot = fk_ur5(list(joints))
                    preset_base = (
                        -float(fk_pos[0]),
                        -float(fk_pos[1]),
                        float(fk_pos[2]),
                    )
                except Exception as exc:
                    panel._emit_log(
                        f"[PICK][DIRECT][FALLBACK_CHECK] label={label} ok=false reason=fk_error exc={exc}"
                    )
                    return False
                compare_target = _tuple3(target_base) or _tuple3(obj_base)
                if compare_target is None:
                    panel._emit_log(
                        f"[PICK][DIRECT][FALLBACK_CHECK] label={label} ok=false reason=target_unavailable preset_base={_fmt_vec(preset_base)}"
                    )
                    return False
                dx = float(preset_base[0]) - float(compare_target[0])
                dy = float(preset_base[1]) - float(compare_target[1])
                dz = float(preset_base[2]) - float(compare_target[2])
                xy_dist = math.hypot(dx, dy)
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                xy_tol = max(
                    0.01,
                    float(os.environ.get("PANEL_PICK_DEMO_FALLBACK_PRESET_MAX_XY_M", "0.05") or 0.05),
                )
                dist_tol = max(
                    xy_tol,
                    float(os.environ.get("PANEL_PICK_DEMO_FALLBACK_PRESET_MAX_DIST_M", "0.10") or 0.10),
                )
                ok = bool(xy_dist <= xy_tol and dist <= dist_tol)
                panel._emit_log(
                    "[PICK][DIRECT][FALLBACK_CHECK] "
                    f"label={label} ok={str(ok).lower()} preset_base={_fmt_vec(preset_base)} "
                    f"target_base={_fmt_vec(compare_target)} dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} "
                    f"xy_dist={xy_dist:.3f}/{xy_tol:.3f} dist={dist:.3f}/{dist_tol:.3f}"
                )
                return ok

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
                    obj_base_live = _live_object_base()
                    obj_base, align_target_source, _align_target_extra = _resolved_align_object_base()
                    if obj_base is None:
                        raise RuntimeError("demo_object_pose_unavailable_before_align")
                    target_z_expected = float(obj_base[2]) + _DIRECTO_GRASP_Z
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
                    already_aligned = bool(pre_metrics.get("ok"))
                    if already_aligned:
                        panel._emit_log(
                            "[PICK][DIRECT][ALIGN_TRACE] "
                            f"attempt={attempt}/{max_attempts} "
                            "align_target_frame=base_link "
                            f"align_target_pose={_fmt_vec(target_tcp_runtime_raw)} "
                            f"align_current_tcp={_fmt_vec(tcp_before)} "
                            f"align_object_pose={_fmt_vec(obj_base)} "
                            f"align_object_pose_live={_fmt_vec(obj_base_live)} "
                            "align_offset_vector=skip_move_already_aligned "
                            "align_reached_condition=true "
                            "align_timeout_reason=none "
                            f"align_target_source={align_target_source}"
                        )
                        return {
                            "label": "GRASP_ALIGN_IK",
                            "runtime_target_ok": True,
                            "runtime_target_dist": float(pre_metrics.get("tcp_obj_dist") or 0.0),
                            "target_tcp_runtime": _tuple3(target_tcp_runtime_raw),
                            "target_source_frame": DIRECT_SOURCE_FRAME,
                            "target_execution_frame": DIRECT_EXECUTION_FRAME,
                            "offset_vector": None,
                        }
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
                    # FIX-SYNTAX: the Rz(pi) frame fix belongs ONLY inside
                    # _resolve_direct_execution_target (called by _move_tcp_direct below).
                    # This block restores the debug metrics + IK_GEOM log that were
                    # corrupted when a duplicate of the Rz fix was incorrectly inserted here.
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
                    live_anchor_dx = (
                        float(obj_base_live[0]) - float(obj_base[0])
                        if obj_base_live is not None and obj_base is not None
                        else None
                    )
                    live_anchor_dy = (
                        float(obj_base_live[1]) - float(obj_base[1])
                        if obj_base_live is not None and obj_base is not None
                        else None
                    )
                    live_anchor_dz = (
                        float(obj_base_live[2]) - float(obj_base[2])
                        if obj_base_live is not None and obj_base is not None
                        else None
                    )
                    live_anchor_norm = (
                        math.sqrt(live_anchor_dx**2 + live_anchor_dy**2 + live_anchor_dz**2)
                        if live_anchor_dx is not None
                        else None
                    )
                    panel._emit_log(
                        "[PICK][DIRECT][IK_GEOM] "
                        f"attempt={attempt}/{max_attempts} "
                        f"tcp_before={_fmt_vec(tcp_before)} "
                        f"obj_base={_fmt_vec(obj_base)} "
                        f"obj_base_live={_fmt_vec(obj_base_live)} "
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
                        f"align_target_source={align_target_source} "
                        f"live_anchor_delta=({_fmt_scalar(live_anchor_dx)},{_fmt_scalar(live_anchor_dy)},{_fmt_scalar(live_anchor_dz)}) "
                        f"live_anchor_norm={_fmt_scalar(live_anchor_norm)} "
                        f"decision={decision}"
                    )
                    panel._emit_log(
                        "[PICK][DIRECT][ALIGN_TRACE] "
                        f"attempt={attempt}/{max_attempts} "
                        "align_target_frame=base_link "
                        f"align_target_pose={_fmt_vec(target_tcp_runtime)} "
                        f"align_current_tcp={_fmt_vec(tcp_before)} "
                        f"align_object_pose={_fmt_vec(obj_base)} "
                        f"align_object_pose_live={_fmt_vec(obj_base_live)} "
                        "align_offset_vector=deferred_to_direct_ik "
                        f"align_error_xyz={_fmt_vec(delta_used)} "
                        f"align_error_norm={_fmt_scalar(_dist(target_tcp_runtime, tcp_before) if tcp_before is not None else None)} "
                        "align_reached_condition=pre_move_false "
                        "align_timeout_reason=none "
                        f"align_target_source={align_target_source}"
                    )
                    # ----------------------------------------------------------
                    # [COMPARE] Directo live vs Directo2 preset — en cada intento de align
                    # ----------------------------------------------------------
                    try:
                        _cmp_obj = obj_base  # base_link
                        _cmp_tcp = tcp_before  # rg2_pinch_center en base_link (live)
                        _cmp_target = target_tcp_runtime  # target computado en base_link
                        _cmp_xy = None
                        _cmp_dz = None
                        _cmp_3d = None
                        if _cmp_tcp is not None and _cmp_obj is not None:
                            _dxx = float(_cmp_tcp[0]) - float(_cmp_obj[0])
                            _dyy = float(_cmp_tcp[1]) - float(_cmp_obj[1])
                            _dzz = float(_cmp_tcp[2]) - float(_cmp_obj[2])
                            _cmp_xy = math.sqrt(_dxx ** 2 + _dyy ** 2)
                            _cmp_dz = _dzz
                            _cmp_3d = math.sqrt(_dxx ** 2 + _dyy ** 2 + _dzz ** 2)
                        panel._emit_log(
                            "[COMPARE][OBJECT] "
                            f"attempt={attempt}/{max_attempts} "
                            f"obj_base_link={_fmt_vec(_cmp_obj)} "
                            f"obj_z_tgt={target_z_expected:.3f} "
                            f"gripper_tcp_z_offset={float(GRIPPER_TCP_Z_OFFSET):.3f}"
                        )
                        panel._emit_log(
                            "[COMPARE][PINCH] "
                            f"attempt={attempt}/{max_attempts} "
                            f"ee_frame={DIRECT_SOURCE_FRAME} "
                            f"tcp_base_link={_fmt_vec(_cmp_tcp)} "
                            f"xy_dist_to_obj={_cmp_xy:.4f if _cmp_xy is not None else 'N/A'} "
                            f"dz_to_obj={_cmp_dz:.4f if _cmp_dz is not None else 'N/A'} "
                            f"dist3d_to_obj={_cmp_3d:.4f if _cmp_3d is not None else 'N/A'}"
                        )
                        panel._emit_log(
                            "[COMPARE][TARGET] "
                            f"attempt={attempt}/{max_attempts} "
                            f"target_base_link={_fmt_vec(_cmp_target)} "
                            f"source_frame={DIRECT_SOURCE_FRAME} "
                            f"ik_frame={DIRECT_EXECUTION_FRAME} "
                            f"decision={decision}"
                        )
                        _div_xy = _cmp_xy
                        _div_dz = (
                            abs(_cmp_dz - _DIRECTO_GRASP_Z)
                            if _cmp_dz is not None else None
                        )
                        panel._emit_log(
                            "[COMPARE][DIVERGENCE] "
                            f"attempt={attempt}/{max_attempts} "
                            f"xy_dist_tcp_obj={_div_xy:.4f if _div_xy is not None else 'N/A'} "
                            f"dz_error_vs_offset={_div_dz:.4f if _div_dz is not None else 'N/A'} "
                            f"gripper_tcp_z_offset={float(GRIPPER_TCP_Z_OFFSET):.3f} "
                            "note=xy_should_be_lt_0.020_dz_error_should_be_lt_0.015"
                        )
                        _rc = "OK_WITHIN_TOLERANCE" if (
                            _div_xy is not None and _div_xy < 0.020
                        ) else "XY_MISALIGNED_PRESET_OR_IK_DRIFT"
                        panel._emit_log(
                            f"[COMPARE][ROOT_CAUSE] "
                            f"attempt={attempt}/{max_attempts} "
                            f"verdict={_rc} "
                            f"xy_tol=0.020 "
                            f"route={DIRECT_ROUTE_MODE} "
                            f"source_frame={DIRECT_SOURCE_FRAME}"
                        )
                    except Exception as _exc_cmp:
                        panel._emit_log(f"[COMPARE][DIVERGENCE] error={_exc_cmp}")
                    # ----------------------------------------------------------
                    try:
                        last_debug = _move_tcp_direct(
                            label="GRASP_ALIGN_IK",
                            target_tcp_runtime=target_tcp_runtime,
                            timeout_sec=move_sec + 8.0,
                            audit_target_source=align_target_source,
                            target_pose_original=obj_base,
                            target_frame_original="base_link",
                            rot_weight=max(
                                0.0,
                                float(
                                    os.environ.get(
                                        "PANEL_PICK_DEMO_ALIGN_ROT_WEIGHT",
                                        "0.10",
                                    )
                                    or 0.10
                                ),
                            ),
                            ik_err_tol=max(
                                0.035,
                                float(
                                    os.environ.get(
                                        "PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL",
                                        "0.10",
                                    )
                                    or 0.10
                                ),
                            ),
                            joint_weight=max(
                                0.0,
                                float(
                                    os.environ.get(
                                        "PANEL_PICK_DEMO_ALIGN_IK_SEED_WEIGHT",
                                        "0.50",
                                    )
                                    or 0.50
                                ),
                            ),
                        )
                    except Exception as exc:
                        tcp_fail = _live_tcp_base()
                        fail_err_xyz = None
                        fail_err_norm = None
                        if tcp_fail is not None:
                            fail_err_xyz = (
                                float(target_tcp_runtime[0]) - float(tcp_fail[0]),
                                float(target_tcp_runtime[1]) - float(tcp_fail[1]),
                                float(target_tcp_runtime[2]) - float(tcp_fail[2]),
                            )
                            fail_err_norm = _dist(target_tcp_runtime, tcp_fail)
                        panel._emit_log(
                            "[PICK][DIRECT][ALIGN_TRACE] "
                            f"attempt={attempt}/{max_attempts} "
                            "align_target_frame=base_link "
                            f"align_target_pose={_fmt_vec(target_tcp_runtime)} "
                            f"align_current_tcp={_fmt_vec(tcp_fail)} "
                            f"align_object_pose={_fmt_vec(obj_base)} "
                            f"align_object_pose_live={_fmt_vec(obj_base_live)} "
                            "align_offset_vector=direct_ik_logged_separately "
                            f"align_error_xyz={_fmt_vec(fail_err_xyz)} "
                            f"align_error_norm={_fmt_scalar(fail_err_norm)} "
                            "align_reached_condition=false "
                            f"align_timeout_reason={exc} "
                            f"align_target_source={align_target_source}"
                        )
                        # If PRE_CLOSE is already satisfied despite the joint timeout,
                        # keep the sequence moving instead of aborting a valid grasp.
                        _pre_fail_metrics = _pre_close_alignment_metrics()
                        if bool(_pre_fail_metrics.get("ok")):
                            panel._emit_log(
                                "[PICK][DIRECT][ALIGN_TRACE] "
                                f"attempt={attempt}/{max_attempts} "
                                "align_graceful_fallback=pre_close_ok_despite_joint_timeout "
                                f"align_target_source={align_target_source} "
                                f"xy_dist={_fmt_scalar(_pre_fail_metrics.get('xy_dist'))} "
                                f"z_error={_fmt_scalar(_pre_fail_metrics.get('z_error'))}"
                            )
                            last_debug = {}
                            last_metrics = _pre_fail_metrics
                            break
                        raise
                    last_metrics = _pre_close_alignment_metrics()
                    runtime_ok = bool((last_debug or {}).get("runtime_target_ok"))
                    tcp_after = _live_tcp_base()
                    obj_after = _live_object_base()
                    z_error_after = None
                    target_tcp_used_z = float(target_tcp_runtime[2])
                    tcp_final_z = None
                    z_expected_after = None
                    if obj_after is not None:
                        z_expected_after = float(obj_after[2]) + _DIRECTO_GRASP_Z
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
                    align_err_xyz = None
                    align_err_norm = None
                    if tcp_after is not None:
                        align_err_xyz = (
                            float(target_tcp_runtime[0]) - float(tcp_after[0]),
                            float(target_tcp_runtime[1]) - float(tcp_after[1]),
                            float(target_tcp_runtime[2]) - float(tcp_after[2]),
                        )
                        align_err_norm = _dist(target_tcp_runtime, tcp_after)
                    panel._emit_log(
                        "[PICK][DIRECT][ALIGN_TRACE] "
                        f"attempt={attempt}/{max_attempts} "
                        "align_target_frame=base_link "
                        f"align_target_pose={_fmt_vec(target_tcp_runtime)} "
                        f"align_current_tcp={_fmt_vec(tcp_after)} "
                        f"align_object_pose={_fmt_vec(obj_base)} "
                        f"align_object_pose_live={_fmt_vec(obj_after)} "
                        f"align_offset_vector={_fmt_vec((last_debug or {}).get('offset_vector'))} "
                        f"align_error_xyz={_fmt_vec(align_err_xyz)} "
                        f"align_error_norm={_fmt_scalar(align_err_norm)} "
                        f"align_reached_condition={str(convergence_ok).lower()} "
                        "align_timeout_reason=none "
                        f"align_target_source={align_target_source}"
                    )
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
                live_world_fn=None,
            ) -> None:
                # PHYSICAL_GATE: live_world_fn permite inyectar una fuente de posición
                # del objeto independiente de la referencia de ciclo congelada.
                # Si no se proporciona, usa _live_object_world() (comportamiento previo).
                _obj_world_source = live_world_fn if live_world_fn is not None else _live_object_world
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
                using_fresh = live_world_fn is not None
                panel._emit_log(
                    "[PICK][DIRECT][PHYSICS] "
                    f"phase={phase} start initial_obj_world=({initial_obj_world[0]:.3f},{initial_obj_world[1]:.3f},{initial_obj_world[2]:.3f}) "
                    f"min_obj_move={float(min_obj_move_m):.3f} min_lift_delta={float(min_lift_delta_m):.3f} "
                    f"max_tcp_dist={float(max_tcp_dist_m):.3f} "
                    f"expected_tcp_obj_z_gap={float(GRIPPER_TCP_Z_OFFSET):.3f} "
                    f"using_fresh_gazebo={str(using_fresh).lower()}"
                )
                while time.time() < deadline:
                    obj_world = _obj_world_source()
                    if live_world_fn is not None:
                        obj_base = _fresh_gazebo_object_base()
                    else:
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

            def _run_manual_like_transport(
                *,
                close_metrics: dict | None,
                close_confirmed: bool,
            ) -> None:
                nonlocal demo_logical_attached
                nonlocal demo_attach_published

                post_close_hold_sec = max(
                    0.0,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_POST_CLOSE_HOLD_SEC",
                            "0.45",
                        )
                        or 0.45
                    ),
                )
                if post_close_hold_sec > 1e-4:
                    panel._emit_log(
                        "[PICK][DIRECT][MANUAL_LIKE] "
                        f"post_close_hold wait_sec={post_close_hold_sec:.2f}"
                    )
                    time.sleep(post_close_hold_sec)

                tcp_base_grasp = _live_tcp_base()
                obj_base_grasp = _live_object_base()
                attach_manual_like_ok = False
                if tcp_base_grasp is not None and obj_base_grasp is not None:
                    try:
                        attach_manual_like_ok = bool(
                            panel._attempt_attach(
                                "demo_manual_like_transport",
                                selected_name=PICK_DEMO_OBJECT_NAME,
                                tcp_base=tcp_base_grasp,
                                object_base=obj_base_grasp,
                                base_frame=str(panel._business_base_frame() or BASE_FRAME or "base_link"),
                                xy_tol_m=max(
                                    0.03,
                                    float(
                                        os.environ.get(
                                            "PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_XY_TOL_M",
                                            "0.060",
                                        )
                                        or 0.060
                                    ),
                                ),
                                z_tol_m=max(
                                    0.03,
                                    float(
                                        os.environ.get(
                                            "PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_Z_TOL_M",
                                            "0.060",
                                        )
                                        or 0.060
                                    ),
                                ),
                                z_ref_mode="center",
                            )
                        )
                    except Exception as exc:
                        panel._emit_log(
                            "[PICK][DIRECT][MANUAL_LIKE] "
                            f"attach_exception={exc}"
                        )
                        attach_manual_like_ok = False
                demo_attach_published = bool(attach_manual_like_ok)
                if attach_manual_like_ok:
                    panel._emit_log(
                        "[PICK][DIRECT][MANUAL_LIKE] "
                        "attach_backend_publish=true reason=post_close_transport"
                    )
                    try:
                        _wait_demo_attach_follow(
                            timeout_sec=max(
                                0.6,
                                float(
                                    os.environ.get(
                                        "PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_WAIT_SEC",
                                        "0.9",
                                    )
                                    or 0.9
                                ),
                            ),
                            max_tcp_dist_m=max(
                                0.10,
                                float(
                                    os.environ.get(
                                        "PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_MAX_TCP_DIST_M",
                                        "0.14",
                                    )
                                    or 0.14
                                ),
                            ),
                            min_consecutive=2,
                        )
                    except Exception as exc:
                        panel._emit_log(
                            "[PICK][DIRECT][MANUAL_LIKE] "
                            f"attach_follow_warning={exc}"
                        )
                else:
                    panel._emit_log(
                        "[PICK][DIRECT][MANUAL_LIKE] "
                        "attach_backend_publish=false reason=geometry_or_backend"
                    )

                grasp_mark_ok = False
                attach_mark_ok = False
                try:
                    grasp_mark_ok = bool(
                        mark_object_grasped(
                            PICK_DEMO_OBJECT_NAME,
                            reason="demo_manual_like_close_confirmed",
                        )
                    )
                except Exception:
                    grasp_mark_ok = False
                try:
                    attach_mark_ok = bool(
                        mark_object_attached(
                            PICK_DEMO_OBJECT_NAME,
                            reason="demo_manual_like_transport",
                        )
                    )
                except Exception:
                    attach_mark_ok = False
                demo_logical_attached = bool(attach_mark_ok)
                panel._emit_log(
                    "[PICK][DIRECT][MANUAL_LIKE] "
                    f"close_confirmed={str(bool(close_confirmed)).lower()} "
                    f"close_geometry_ok={str(bool((close_metrics or {}).get('ok'))).lower()} "
                    f"mark_grasped={str(grasp_mark_ok).lower()} "
                    f"mark_attached={str(attach_mark_ok).lower()}"
                )

                _run_joint_step(
                    "MESA_WITH_OBJECT",
                    JOINT_TABLE_POSE_RAD,
                    timeout_sec=move_sec + 8.0,
                    tol_rad=0.10,
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

                panel._emit_log("[DEMO] Abriendo pinza en cesta (manual_like)")

                def _open_and_release_manual_like():
                    panel._command_gripper(False, log_action="DROP", force=True)
                    mark_object_released(PICK_DEMO_OBJECT_NAME, reason="demo_drop_manual_like")
                    _detach_demo_object("demo_drop_manual_like")

                _phase_begin(
                    "RELEASE",
                    frame_used="basket",
                    offsets={"drop_reason": "demo_drop_manual_like", "mode": "manual_like"},
                    note="open gripper and release after manual-like transport",
                )
                _final_phase_trace(
                    "RELEASE",
                    event="wait_start",
                    expected="basket_release_done",
                    received="pending",
                    timeout_sec="1.40",
                    reason="manual_like_open_wait",
                    request_state="basket_release_manual_like",
                )
                panel.signal_run_ui.emit(_open_and_release_manual_like)
                time.sleep(1.0)
                demo_logical_attached = False
                _phase_end("RELEASE", attach_state=_read_attach_state(), result="ok")
                _final_phase_trace(
                    "RELEASE",
                    event="wait_done",
                    expected="basket_release_done",
                    received="true",
                    timeout_sec="1.40",
                    reason="basket_release_completed_manual_like",
                    logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                )

                _phase_begin(
                    "HOME_FINAL",
                    frame_used="base_link",
                    note="return home after basket release (manual_like)",
                )
                _final_phase_trace(
                    "HOME_FINAL",
                    event="wait_start",
                    expected="home_joint_reached",
                    received="pending",
                    timeout_sec="auto(move_sec+2)",
                    reason="run_joint_step_HOME_FINAL_manual_like",
                    request_state="home_final_manual_like",
                )
                _run_joint_step("HOME_FINAL", home_pose)
                _phase_end("HOME_FINAL", attach_state=_read_attach_state(), result="ok")
                _final_phase_trace(
                    "HOME_FINAL",
                    event="wait_done",
                    expected="home_joint_reached",
                    received="true",
                    timeout_sec="auto(move_sec+2)",
                    reason="home_final_completed_manual_like",
                    logical_state=str((_read_attach_state() or {}).get("logical_state") or "none"),
                )
                panel._emit_log("[PICK][DIRECT] AVISO: TRAMO FINAL COMPLETADO route=manual_like")
                panel._emit_log("[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE route=manual_like")

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
            # Waypoint intermedio para evitar el salto grande j2: 0°→90° en un solo paso.
            # MoveIt falla silenciosamente cuando el salto de elbow (j2) supera ~80°,
            # llegando al TCP correcto pero en rama equivocada (j2≈0 en vez de j2=1.571).
            # Con este paso intermedio a j2=55° los incrementos quedan < 55° y MoveIt
            # ejecuta los joints correctamente en cada paso.
            _run_joint_step(
                "HOME_ELBOW",
                [0.0, math.radians(-90.0), math.radians(55.0),
                 math.radians(-90.0), 0.0, 0.0],
            )
            _run_joint_step("MESA", JOINT_TABLE_POSE_RAD)
            _monitor_alcance(trigger="DIRECT_PICK_START")

            panel._emit_log("[DEMO] Abriendo pinza en posición MESA")
            panel.signal_run_ui.emit(lambda: panel._command_gripper(False, log_action="PICK", force=True))
            time.sleep(0.6)

            def _joint_goal_operational_pose_base(joint_goal) -> tuple[float, float, float] | None:
                try:
                    q_goal = [float(v) for v in joint_goal]
                    fk_pos_model, fk_rot = fk_ur5(q_goal)
                    local_offset = None
                    pose_tool0_source, _pose_err = get_pose(
                        DIRECT_EXECUTION_FRAME,
                        DIRECT_SOURCE_FRAME,
                        timeout_sec=0.25,
                    )
                    if pose_tool0_source:
                        try:
                            px, py, pz = pose_tool0_source["position"]
                            local_offset = (float(px), float(py), float(pz))
                        except Exception:
                            local_offset = None
                    if local_offset is None:
                        local_offset = (0.0, 0.0, float(DIRECT_TOOL0_TO_RG2_TCP_Z_M))
                    source_model = (
                        float(fk_pos_model[0])
                        + float(fk_rot[0, 0]) * float(local_offset[0])
                        + float(fk_rot[0, 1]) * float(local_offset[1])
                        + float(fk_rot[0, 2]) * float(local_offset[2]),
                        float(fk_pos_model[1])
                        + float(fk_rot[1, 0]) * float(local_offset[0])
                        + float(fk_rot[1, 1]) * float(local_offset[1])
                        + float(fk_rot[1, 2]) * float(local_offset[2]),
                        float(fk_pos_model[2])
                        + float(fk_rot[2, 0]) * float(local_offset[0])
                        + float(fk_rot[2, 1]) * float(local_offset[1])
                        + float(fk_rot[2, 2]) * float(local_offset[2]),
                    )
                    return (
                        -float(source_model[0]),
                        -float(source_model[1]),
                        float(source_model[2]),
                    )
                except Exception:
                    return None

            def _dynamic_pre_close_reference_base(obj_base_reference) -> tuple[float, float, float] | None:
                obj_base_3 = _tuple3(obj_base_reference)
                if obj_base_3 is None:
                    return None
                dynamic_extra_z_m = max(
                    0.0,
                    float(
                        os.environ.get(
                            "PANEL_PICK_DEMO_MANUAL_REF_EXTRA_Z_M",
                            os.environ.get("PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M", "0.10"),
                        )
                        or 0.10
                    ),
                )
                return (
                    float(obj_base_3[0]),
                    float(obj_base_3[1]),
                    float(obj_base_3[2]) + float(_DIRECTO_GRASP_Z) + float(dynamic_extra_z_m),
                )

            manual_reference_ok = False
            if route_selected == "manual_reference":
                ref_target_base = None
                ref_target_world = None
                obj_base_reference = _live_object_base()
                if obj_base_reference is not None:
                    ref_target_base = (
                        float(obj_base_reference[0]),
                        float(obj_base_reference[1]),
                        float(obj_base_reference[2]) + _DIRECTO_GRASP_Z,
                    )
                    ref_target_world = _target_world_from_base(ref_target_base)
                ref_manual_base = _joint_goal_operational_pose_base(
                    JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD
                )
                ref_dynamic_base = _dynamic_pre_close_reference_base(obj_base_reference)
                manual_ref_stale = False
                manual_ref_reason = "manual_success_reference_20260405"
                manual_ref_xy_dist = None
                manual_ref_z_gap_vs_obj = None
                if ref_manual_base is not None and obj_base_reference is not None:
                    manual_ref_xy_dist = math.hypot(
                        float(ref_manual_base[0]) - float(obj_base_reference[0]),
                        float(ref_manual_base[1]) - float(obj_base_reference[1]),
                    )
                    manual_ref_z_gap_vs_obj = float(ref_manual_base[2]) - float(obj_base_reference[2])
                    stale_xy_tol_m = max(
                        0.01,
                        float(os.environ.get("PANEL_PICK_DEMO_MANUAL_REF_STALE_XY_TOL_M", "0.08") or 0.08),
                    )
                    stale_z_below_tol_m = max(
                        0.0,
                        float(os.environ.get("PANEL_PICK_DEMO_MANUAL_REF_STALE_Z_BELOW_TOL_M", "0.005") or 0.005),
                    )
                    manual_ref_stale = bool(
                        float(manual_ref_xy_dist) > float(stale_xy_tol_m)
                        or float(manual_ref_z_gap_vs_obj) < -float(stale_z_below_tol_m)
                    )
                    if manual_ref_stale:
                        manual_ref_reason = "dynamic_object_reference_replacing_stale_manual_preset"
                        panel._emit_log(
                            "[PICK][DIRECT][REF] "
                            "phase=PICK_PRE_CLOSE_REF stale_manual_reference=true "
                            f"manual_ref={_fmt_vec(ref_manual_base)} "
                            f"object_ref={_fmt_vec(obj_base_reference)} "
                            f"xy_dist={_fmt_scalar(manual_ref_xy_dist)} "
                            f"z_gap_vs_obj={_fmt_scalar(manual_ref_z_gap_vs_obj)} "
                            f"dynamic_ref={_fmt_vec(ref_dynamic_base)}"
                        )
                ref_display_base = ref_dynamic_base if manual_ref_stale and ref_dynamic_base is not None else (ref_manual_base or ref_target_base)
                ref_display_world = _target_world_from_base(ref_display_base) if ref_display_base is not None else ref_target_world
                _phase_begin(
                    "PICK_PRE_CLOSE_REF",
                    target_world=ref_display_world,
                    target_base=ref_display_base,
                    frame_used="base_link",
                    joint_goal=(None if manual_ref_stale else [float(v) for v in JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD]),
                    note=manual_ref_reason,
                )
                panel._emit_log(
                    "[PICK][DIRECT][REF] "
                    f"phase=PICK_PRE_CLOSE_REF source={manual_ref_reason}"
                )
                ref_debug = None
                if manual_ref_stale and ref_dynamic_base is not None:
                    try:
                        ref_debug = _move_tcp_direct(
                            label="PICK_PRE_CLOSE_REF",
                            target_tcp_runtime=ref_dynamic_base,
                            timeout_sec=max(move_sec + 2.5, 4.0),
                            audit_target_source="dynamic_object_reference",
                            target_pose_original=obj_base_reference,
                            target_frame_original="base_link",
                        )
                    except Exception as exc:
                        panel._emit_log(
                            "[PICK][DIRECT][REF] "
                            f"phase=PICK_PRE_CLOSE_REF dynamic_reference_failed={exc} fallback=direct_ik_hybrid"
                        )
                else:
                    _run_joint_step(
                        "PICK_PRE_CLOSE_REF",
                        JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD,
                        timeout_sec=move_sec + 6.0,
                        tol_rad=0.05,
                    )
                ref_metrics = _pre_close_alignment_metrics()
                panel._emit_log(
                    "[PICK][DIRECT][REF] "
                    f"phase=PICK_PRE_CLOSE_REF tcp={_fmt_vec(_live_tcp_base())} "
                    f"obj={_fmt_vec(_live_object_base())} "
                    f"xy_dist={_fmt_scalar(ref_metrics.get('xy_dist'))} "
                    f"z_error={_fmt_scalar(ref_metrics.get('z_error'))} "
                    f"ok={str(bool(ref_metrics.get('ok'))).lower()}"
                )
                manual_reference_ok = bool(ref_metrics.get("ok"))
                _phase_end(
                    "PICK_PRE_CLOSE_REF",
                    result="ok" if manual_reference_ok else "partial",
                    joint_goal=(None if manual_ref_stale else [float(v) for v in JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD]),
                    ik_solution=ref_debug,
                    note=json.dumps(_json_safe(ref_metrics), ensure_ascii=False, sort_keys=True),
                )
                if not manual_reference_ok:
                    panel._emit_log(
                        "[PICK][DIRECT][REF] "
                        "manual_reference_not_reached -> fallback=direct_ik_hybrid "
                        f"xy_dist={_fmt_scalar(ref_metrics.get('xy_dist'))} "
                        f"z_error={_fmt_scalar(ref_metrics.get('z_error'))}"
                    )
            if route_selected != "manual_reference" or not manual_reference_ok:
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
                tcp_before_coarse = _live_tcp_base()
                if obj_base_before_coarse is None:
                    _append_trace(
                        "[PICK][DIRECT][CYCLE_REF][ABORT] tag=[DIRECT][CYCLE_REF][ABORT] "
                        "phase=APPROACH_COARSE reason=cycle_object_base_unavailable"
                    )
                    panel._emit_log(
                        "[PICK][DIRECT][ABORT] phase=APPROACH_COARSE reason=live_object_base_unavailable"
                    )
                    raise RuntimeError("demo_live_object_pose_unavailable_before_approach_coarse")
                target_base_coarse = None
                target_world_coarse = None
                target_base_grasp_down = None
                target_world_grasp_down = None
                if obj_base_before_coarse is not None:
                    target_x_coarse = float(obj_base_before_coarse[0])
                    target_y_coarse = float(obj_base_before_coarse[1])
                    target_z_coarse = float(obj_base_before_coarse[2]) + _DIRECTO_GRASP_Z + float(coarse_extra_z_m)
                    coarse_target_mode = "object_xy_plus_object_z"
                    if tcp_before_coarse is not None:
                        keep_xy_tol = max(
                            0.01,
                            float(os.environ.get("PANEL_PICK_DEMO_APPROACH_COARSE_KEEP_XY_TOL_M", "0.06") or 0.06),
                        )
                        tcp_obj_xy = math.hypot(
                            float(tcp_before_coarse[0]) - float(obj_base_before_coarse[0]),
                            float(tcp_before_coarse[1]) - float(obj_base_before_coarse[1]),
                        )
                        if tcp_obj_xy <= keep_xy_tol:
                            target_x_coarse = float(tcp_before_coarse[0])
                            target_y_coarse = float(tcp_before_coarse[1])
                            coarse_target_mode = "keep_current_xy_plus_object_z"
                    target_base_coarse = (
                        float(target_x_coarse),
                        float(target_y_coarse),
                        float(target_z_coarse),
                    )
                    target_world_coarse = _target_world_from_base(target_base_coarse)
                    target_base_grasp_down = (
                        float(obj_base_before_coarse[0]),
                        float(obj_base_before_coarse[1]),
                        float(obj_base_before_coarse[2]) + _DIRECTO_GRASP_Z + float(grasp_down_extra_z_m),
                    )
                    target_world_grasp_down = _target_world_from_base(target_base_grasp_down)
                    panel._emit_log(
                        "[PICK][DIRECT][APPROACH_PLAN] "
                        f"mode={coarse_target_mode} tcp_before={_fmt_vec(tcp_before_coarse)} "
                        f"obj_before={_fmt_vec(obj_base_before_coarse)} target={_fmt_vec(target_base_coarse)}"
                    )
                _phase_begin(
                    "APPROACH_COARSE",
                    target_world=target_world_coarse,
                    target_base=target_base_coarse,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "coarse_extra_z_m": float(coarse_extra_z_m),
                        "relative_mode": "keep_current_xy_when_aligned",
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
                        "relative_mode": "keep_current_xy_when_aligned",
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
                    coarse_skip_xy_tol = max(
                        0.005,
                        float(os.environ.get("PANEL_PICK_DEMO_APPROACH_COARSE_SKIP_XY_TOL_M", "0.03") or 0.03),
                    )
                    coarse_skip_z_tol = max(
                        0.005,
                        float(os.environ.get("PANEL_PICK_DEMO_APPROACH_COARSE_SKIP_Z_TOL_M", "0.04") or 0.04),
                    )
                    if tcp_before_coarse is not None:
                        coarse_dx = float(tcp_before_coarse[0]) - float(target_base_coarse[0])
                        coarse_dy = float(tcp_before_coarse[1]) - float(target_base_coarse[1])
                        coarse_dz = float(tcp_before_coarse[2]) - float(target_base_coarse[2])
                        coarse_xy = math.hypot(coarse_dx, coarse_dy)
                        if coarse_xy <= coarse_skip_xy_tol and abs(coarse_dz) <= coarse_skip_z_tol:
                            approach_decision = "skip_already_at_coarse_target"
                            panel._emit_log(
                                "[PICK][DIRECT][APPROACH_SKIP] "
                                f"tcp_before={_fmt_vec(tcp_before_coarse)} target={_fmt_vec(target_base_coarse)} "
                                f"xy_dist={coarse_xy:.3f}/{coarse_skip_xy_tol:.3f} "
                                f"dz={coarse_dz:.3f}/{coarse_skip_z_tol:.3f}"
                            )
                            approach_debug = {
                                "ik_solution": _current_joint_seed(),
                                "note": approach_decision,
                            }
                    if approach_decision == "direct_ik_move":
                        try:
                            approach_debug = _move_tcp_direct(
                                label="APPROACH_COARSE",
                                target_tcp_runtime=target_base_coarse,
                                timeout_sec=max(move_sec + 2.5, 4.0),
                                audit_target_source="live_object_base",
                                target_pose_original=obj_base_before_coarse,
                                target_frame_original="base_link",
                            )
                        except Exception as exc:
                            if _joint_preset_fallback_ok(
                                "APPROACH_COARSE",
                                JOINT_GRASP_DOWN_POSE_RAD,
                                target_base=target_base_coarse,
                                obj_base=obj_base_before_coarse,
                            ):
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
                                approach_decision = f"abort_direct_ik_failed:{exc}"
                                panel._emit_log(
                                    "[PICK][DIRECT][ABORT] "
                                    f"phase=APPROACH_COARSE direct_ik_failed={exc} fallback=rejected"
                                )
                                raise
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

                # Refrescar target justo antes del descenso, pero conservando el XY ya
                # alineado en APPROACH_COARSE para que GRASP_DOWN_JOINT sea un descenso
                # corto relativo desde la pose buena alcanzada.
                obj_base_before_grasp_down = _live_object_base()
                if obj_base_before_grasp_down is None:
                    _append_trace(
                        "[PICK][DIRECT][CYCLE_REF][ABORT] tag=[DIRECT][CYCLE_REF][ABORT] "
                        "phase=GRASP_DOWN_JOINT reason=cycle_object_base_unavailable"
                    )
                    panel._emit_log(
                        "[PICK][DIRECT][ABORT] phase=GRASP_DOWN_JOINT reason=live_object_base_unavailable"
                    )
                    raise RuntimeError("demo_live_object_pose_unavailable_before_grasp_down")
                target_base_grasp_down = None
                target_world_grasp_down = None
                tcp_before_grasp_down = _live_tcp_base()
                tcp_before_grasp_down_source = "live_tcp"
                if tcp_before_grasp_down is None:
                    tcp_before_grasp_down = _tuple3(getattr(panel, "_last_trace_tcp_base", None))
                    if tcp_before_grasp_down is not None:
                        tcp_before_grasp_down_source = "panel_last_trace_tcp_base"
                if tcp_before_grasp_down is None:
                    tcp_before_grasp_down = _tuple3(getattr(panel, "_last_tcp_base", None))
                    if tcp_before_grasp_down is not None:
                        tcp_before_grasp_down_source = "panel_last_tcp_base"
                if obj_base_before_grasp_down is not None:
                    target_x = float(obj_base_before_grasp_down[0])
                    target_y = float(obj_base_before_grasp_down[1])
                    target_z = float(obj_base_before_grasp_down[2]) + grasp_z_for_source_frame + float(grasp_down_extra_z_m)
                    target_mode = "object_xy_plus_object_z"
                    if tcp_before_grasp_down is not None:
                        target_x = float(tcp_before_grasp_down[0])
                        target_y = float(tcp_before_grasp_down[1])
                        target_mode = "keep_current_xy_plus_object_z"
                    target_base_grasp_down = (
                        float(target_x),
                        float(target_y),
                        float(target_z),
                    )
                    target_world_grasp_down = _target_world_from_base(target_base_grasp_down)
                    panel._emit_log(
                        "[PICK][DIRECT][GRASP_DOWN_PLAN] "
                        f"mode={target_mode} "
                        f"tcp_source={tcp_before_grasp_down_source} "
                        f"tcp_before={_fmt_vec(tcp_before_grasp_down)} "
                        f"obj_before={_fmt_vec(obj_base_before_grasp_down)} "
                        f"target={_fmt_vec(target_base_grasp_down)}"
                    )
                _phase_begin(
                    "GRASP_DOWN_JOINT",
                    target_world=target_world_grasp_down,
                    target_base=target_base_grasp_down,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "grasp_down_extra_z_m": float(grasp_down_extra_z_m),
                        "relative_mode": "keep_current_xy_plus_object_z",
                        "tcp_source": str(tcp_before_grasp_down_source),
                        "mode": "direct_rg2_tcp",
                        "execution_mode": "hybrid_geometric_with_preset_fallback",
                    },
                    joint_goal=[float(v) for v in JOINT_GRASP_DOWN_POSE_RAD],
                    note="relative short descent from APPROACH_COARSE before GRASP_ALIGN_IK",
                )
                _trace_phase_pose(
                    phase="GRASP_DOWN_JOINT",
                    event="target_set",
                    target_base=target_base_grasp_down,
                    frame_used="base_link",
                    offsets={
                        "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                        "grasp_down_extra_z_m": float(grasp_down_extra_z_m),
                        "relative_mode": "keep_current_xy_plus_object_z",
                        "tcp_source": str(tcp_before_grasp_down_source),
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
                            audit_target_source=f"{tcp_before_grasp_down_source}+live_object_z",
                            target_pose_original=target_base_grasp_down,
                            target_frame_original="base_link",
                        )
                    except Exception as exc:
                        if _joint_preset_fallback_ok(
                            "GRASP_DOWN_JOINT",
                            JOINT_GRASP_DOWN_POSE_RAD,
                            target_base=target_base_grasp_down,
                            obj_base=obj_base_before_grasp_down,
                        ):
                            grasp_down_decision = f"fallback_joint_preset:{exc}"
                            panel._emit_log(
                                "[PICK][DIRECT][WARN] "
                                f"phase=GRASP_DOWN_JOINT direct_ik_failed={exc} fallback=joint_preset"
                            )
                            _run_joint_step(
                                "GRASP_DOWN_JOINT_FALLBACK",
                                JOINT_GRASP_DOWN_POSE_RAD,
                                timeout_sec=move_sec + 6.0,
                                tol_rad=0.08,
                            )
                        else:
                            grasp_down_decision = f"abort_direct_ik_failed:{exc}"
                            panel._emit_log(
                                "[PICK][DIRECT][ABORT] "
                                f"phase=GRASP_DOWN_JOINT direct_ik_failed={exc} fallback=rejected"
                            )
                            raise
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
                        "tcp_source": str(tcp_before_grasp_down_source),
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
            obj_base_align, obj_base_align_source, _align_base_extra = _resolved_align_object_base()
            target_base_align = None
            target_world_align = None
            if obj_base_align is not None:
                target_base_align = (
                    float(obj_base_align[0]),
                    float(obj_base_align[1]),
                    float(obj_base_align[2]) + grasp_z_for_source_frame,
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
                    note="fine alignment IK over stable selected object pose",
                )
                panel._emit_log(
                    "[PICK][DIRECT][ALIGN_TRACE] "
                    "attempt=phase_start "
                    "align_target_frame=base_link "
                    f"align_target_pose={_fmt_vec(target_base_align)} "
                    f"align_current_tcp={_fmt_vec(_live_tcp_base())} "
                    f"align_object_pose={_fmt_vec(obj_base_align)} "
                    f"align_object_pose_live={_fmt_vec(_live_object_base())} "
                    "align_offset_vector=deferred_to_direct_ik "
                    f"align_error_xyz={_fmt_vec(None)} "
                    f"align_error_norm={_fmt_scalar(None)} "
                    "align_reached_condition=phase_enter "
                    "align_timeout_reason=none "
                    f"align_target_source={obj_base_align_source}"
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
                panel._emit_log(
                    "[PICK][DIRECT][ROUTE] "
                    "moveit_extra_down=disabled "
                    f"route_selected={route_selected} "
                    "route_reason=optional_cartesian_extra_down_disabled"
                )
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
            # PHYSICAL_GATE: capturar posición inicial del objeto desde Gazebo real,
            # no desde la referencia de ciclo congelada.  Esto garantiza que el gate
            # de transporte mide desplazamiento físico observable, no un delta artificial.
            initial_obj_world = _fresh_gazebo_object_world()
            if initial_obj_world is None:
                _append_trace(
                    "[PICK][DIRECT][CYCLE_REF][ABORT] tag=[DIRECT][CYCLE_REF][ABORT] "
                    "phase=PRE_CLOSE reason=cycle_object_world_unavailable"
                )
                _emit_transition_decision(
                    from_phase="GRASP_ALIGN_IK",
                    to_phase="PRE_CLOSE",
                    decision="blocked",
                    reason="object_pose_unavailable_before_pre_close",
                    condition="initial_obj_world_available",
                    metrics=_pre_close_alignment_metrics(),
                )
                panel._emit_log(
                    "[PICK][DIRECT][ABORT] phase=PRE_CLOSE reason=live_object_world_unavailable"
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
                    float(obj_base_pre[2]) + grasp_z_for_source_frame,
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
            close_metrics = dict(pre_close_metrics) if bool(pre_close_ok) else _close_alignment_metrics()
            close_metrics["geometry_source"] = (
                "pre_close_reference" if bool(pre_close_ok) else "post_close_live"
            )
            close_metrics["gripper_closed_measured"] = bool(
                (close_wait_state or {}).get("measured_target_ok")
            )
            close_metrics["gripper_opening_sum"] = (close_wait_state or {}).get("opening_sum")
            close_metrics["gripper_max_abs_err"] = (close_wait_state or {}).get("max_abs_err")
            close_metrics["close_confirmed"] = bool(close_confirmed)
            close_metrics["close_wait_state"] = _json_safe(close_wait_state)
            # basket es el camino seguro por defecto para el demo: mantiene el
            # transporte hasta la cesta y evita que la ruta manual_like deje el
            # objeto restaurado en una pose vieja del backend.
            post_close_mode = str(
                os.environ.get("PANEL_PICK_DEMO_POST_CLOSE_MODE", "basket") or "basket"
            ).strip().lower()
            manual_like_mode = post_close_mode in {"manual_like", "manual", "simple"}
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
                if manual_like_mode:
                    panel._emit_log(
                        "[PICK][DIRECT][MANUAL_LIKE] "
                        "close_geometry_override=true "
                        f"route_selected={route_selected} "
                        f"xy_dist={_fmt_scalar(close_metrics.get('xy_dist'))} "
                        f"z_error={_fmt_scalar(close_metrics.get('z_error'))}"
                    )
                    _run_manual_like_transport(
                        close_metrics=close_metrics,
                        close_confirmed=close_confirmed,
                    )
                    return
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
            if manual_like_mode:
                panel._emit_log(
                    "[PICK][DIRECT][MANUAL_LIKE] "
                    "close_gate_ok -> using manual-like transport "
                    f"route_selected={route_selected}"
                )
                _run_manual_like_transport(
                    close_metrics=close_metrics,
                    close_confirmed=close_confirmed,
                )
                return
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
                float(obj_base_grasp[2]) + _DIRECTO_GRASP_Z,
            )
            target_world_attach = _target_world_from_base(target_base_attach)
            attach_xy_tol_m = max(
                0.02,
                float(os.environ.get("PANEL_PICK_DEMO_ATTACH_XY_TOL_M", "0.080") or 0.080),
            )
            attach_z_tol_m = max(
                0.02,
                float(os.environ.get("PANEL_PICK_DEMO_ATTACH_Z_TOL_M", "0.080") or 0.080),
            )
            attach_dx = float(obj_base_grasp[0]) - float(tcp_base_grasp[0])
            attach_dy = float(obj_base_grasp[1]) - float(tcp_base_grasp[1])
            attach_tcp_contact_z = float(tcp_base_grasp[2]) - float(GRIPPER_TCP_Z_OFFSET)
            attach_obj_ref_z = float(obj_base_grasp[2])
            attach_dz = attach_obj_ref_z - attach_tcp_contact_z
            attach_xy_dist = math.hypot(attach_dx, attach_dy)
            attach_tcp_obj_dist = _dist(tcp_base_grasp, obj_base_grasp)
            attach_geometry_ok = bool(
                abs(attach_dx) <= attach_xy_tol_m
                and abs(attach_dy) <= attach_xy_tol_m
                and abs(attach_dz) <= attach_z_tol_m
            )
            attach_reference_metrics = {
                "ok": attach_geometry_ok,
                "geometry_ok": attach_geometry_ok,
                "reason": "ok" if attach_geometry_ok else "alignment_out_of_tolerance",
                "geometry_source": "attach_contact_reference",
                "xy_dist": float(attach_xy_dist),
                "z_gap": float(float(tcp_base_grasp[2]) - float(obj_base_grasp[2])),
                "z_error": float(abs(attach_dz)),
                "tcp_obj_dist": float(attach_tcp_obj_dist),
                "xy_tol": float(attach_xy_tol_m),
                "z_tol": float(attach_z_tol_m),
                "tcp_contact_z": float(attach_tcp_contact_z),
                "object_ref_z": float(attach_obj_ref_z),
                "z_ref_mode": "center",
                "tcp_base": _tuple3(tcp_base_grasp),
                "object_base": _tuple3(obj_base_grasp),
            }
            _phase_begin(
                "ATTACH_GATE",
                target_world=target_world_attach,
                target_base=target_base_attach,
                frame_used="base_link",
                offsets={
                    "tcp_z_offset_m": float(GRIPPER_TCP_Z_OFFSET),
                    "attach_xy_tol_m": float(attach_xy_tol_m),
                    "attach_z_tol_m": float(attach_z_tol_m),
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
                xy_tol_m=attach_xy_tol_m,
                z_tol_m=attach_z_tol_m,
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
            _clear_cycle_object_reference(reason="attach_follow_confirmed")
            if not mark_object_grasped(PICK_DEMO_OBJECT_NAME, reason="demo_attach_follow_confirmed"):
                raise RuntimeError("demo_mark_grasped_after_attach_failed")
            if not mark_object_attached(PICK_DEMO_OBJECT_NAME, reason="demo_attach_follow_confirmed"):
                raise RuntimeError("demo_mark_attached_after_attach_failed")
            demo_logical_attached = True
            attach_metrics = dict(attach_reference_metrics)
            attach_metrics["follow_confirmed"] = True
            attach_metrics["attach_published"] = bool(demo_attach_published)
            attach_metrics["logical_attached"] = True
            attach_metrics["logical_state"] = "CARRIED"
            attach_metrics["owner"] = "ROBOT"
            attach_metrics["gripper_closed_measured"] = bool(
                (_read_gripper_state(expected_closed=True) or {}).get("measured_target_ok")
            )
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
                #
                # PHYSICAL_GATE (attach lógico vs físico):
                #   min_lift_delta_m subido de 0.025 → 0.060.
                #   Razón: el backend Gazebo demo_transport/follow_tcp puede elevar el
                #   objeto ~3-4 cm al crear el attach kinematic (respawn), INCLUSO cuando
                #   el gripper falla el contacto físico.  Con 2.5 cm de umbral ese artefacto
                #   bastaba para pasar el gate (falso positivo observado, delta ≈ 0.036 m).
                #   6 cm distingue un lift real (≥ 12 cm con TCP) de un respawn espurio.
                #
                #   live_world_fn=_fresh_gazebo_object_world: usa pose Gazebo directa,
                #   no la referencia de ciclo congelada del panel.  Sin esto, si la
                #   referencia estaba activa, obj_move = 0 siempre → gate siempre fallaba
                #   o era inconsistente con los datos físicos reales.
                _validate_demo_carry(
                    initial_obj_world=initial_obj_world,
                    phase="post_grasp_lift",
                    timeout_sec=1.6,
                    min_obj_move_m=0.030,
                    min_lift_delta_m=0.060,
                    max_tcp_dist_m=0.080,
                    live_world_fn=_fresh_gazebo_object_world,
                )
                # PHYSICAL_GATE: separación explícita attach lógico / attach físico.
                # Llegamos aquí SOLO si el objeto se elevó ≥6 cm medido desde Gazebo real.
                # Esto es evidencia física observable de transporte, no solo un attach lógico.
                panel._emit_log(
                    "[PICK][DIRECT][PHYSICS] "
                    "attach_fisico_confirmado=true "
                    "evidencia=carry_validation_gazebo_observable "
                    f"umbral_lift_m=0.060 object={PICK_DEMO_OBJECT_NAME} "
                    "nota=objeto_elevado_por_encima_del_respawn_cinetico"
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
            if not demo_logical_attached:
                if not mark_object_grasped(PICK_DEMO_OBJECT_NAME, reason="demo_physical_lift_ok"):
                    raise RuntimeError("demo_mark_grasped_failed")
                if not mark_object_attached(PICK_DEMO_OBJECT_NAME, reason="demo_physical_lift_ok"):
                    raise RuntimeError("demo_mark_attached_failed")
                demo_logical_attached = True
            short_release_mode = str(
                os.environ.get("PANEL_PICK_DEMO_SHORT_RELEASE_ONLY", "0") or "0"
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
                live_world_fn=_fresh_gazebo_object_world,
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
                _detach_demo_object("demo_drop")

            _phase_begin(
                "RELEASE",
                frame_used="basket",
                offsets={"drop_reason": "demo_drop"},
                note="open gripper and logical release in basket",
            )
            _final_phase_trace(
                "RELEASE",
                event="wait_start",
                expected="basket_release_done&&gripper_open_confirmed",
                received="pending",
                timeout_sec="1.80",
                reason="open_wait_detach_wait",
                request_state="basket_release",
            )
            panel._pick_demo_release_reference_world = _tuple3(_live_tcp_world())
            release_open_state_pre_cmd = _read_gripper_state(expected_closed=True)
            panel.signal_run_ui.emit(_open_and_release)
            time.sleep(0.1)
            release_open_timeout_sec = max(
                0.8,
                float(
                    os.environ.get(
                        "PANEL_PICK_DEMO_RELEASE_OPEN_CONFIRM_TIMEOUT_SEC",
                        "1.8",
                    )
                    or 1.8
                ),
            )
            release_open_confirmed, release_open_wait_state = _wait_for_gripper_target(
                False,
                timeout_sec=release_open_timeout_sec,
                opening_ref_sum=release_open_state_pre_cmd.get("opening_sum"),
            )
            panel._emit_log(
                "[PICK][DIRECT][RELEASE] "
                f"open_wait_done confirmed={bool(release_open_confirmed)} "
                f"mode={str((release_open_wait_state or {}).get('confirm_mode') or 'none')} "
                f"opening_sum={_fmt_scalar((release_open_wait_state or {}).get('opening_sum'))} "
                f"max_abs_err={_fmt_scalar((release_open_wait_state or {}).get('max_abs_err'))} "
                f"measured_ok={bool((release_open_wait_state or {}).get('measured_target_ok'))} "
                f"closed_flag={bool((release_open_wait_state or {}).get('closed_flag'))}"
            )
            _phase_end(
                "RELEASE",
                attach_state=_read_attach_state(),
                result="ok" if bool(release_open_confirmed) else "failed",
            )
            _final_phase_trace(
                "RELEASE",
                event="wait_done",
                expected="basket_release_done&&gripper_open_confirmed",
                received=str(bool(release_open_confirmed)).lower(),
                timeout_sec=f"{release_open_timeout_sec:.2f}",
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
