"""Helpers internos del pick demo: resolucion de objeto + validacion transporte.

Extraido de ``ur5_qt_panel/panel_pick_demo.py`` (lineas 140-1244 originales).

Funciones (todas reciben ``panel`` como primer argumento):

* Resolucion de objeto:
  - ``_live_joint_seed_or_none``
  - ``_resolve_live_object_world``
  - ``_resolve_live_object_base``
  - ``_select_pick_demo_cycle_object_reference``
* Validacion de transporte y waits:
  - ``_demo_object_in_basket``
  - ``_validate_demo_transport_follow``
  - ``_wait_for_demo_runtime_target_progress``

Sin estado propio. Reexportado por ``panel_pick_demo`` para mantener
compatibilidad con cualquier import legacy que dependiera de esos
nombres como atributos de modulo de ``panel_pick_demo``.
"""

from __future__ import annotations

import math
import time

try:
    from std_msgs.msg import Empty
except Exception:  # pragma: no cover - ROS not available in unit contexts
    Empty = None

try:
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
except Exception:  # pragma: no cover
    Point = None
    Marker = None
    MarkerArray = None

from ..panel_robot_presets import PICK_DEMO_OBJECT_NAME
from ..panel_config import (
    BASKET_DROP,
    WORLD_FRAME,
    BASE_FRAME,
    UR5_JOINT_NAMES,
)
from ..panel_objects import (
    get_object_state,
    get_object_positions,
    is_on_table,
    ObjectOwner,
    ObjectLogicalState,
)
from ..directo_geometry import (
    _pick_demo_tuple3,
    _pick_demo_fmt_scalar,
    _pick_demo_env_float,
    _pick_demo_env_flag,
)
from ..panel_utils import (
    fmt_vec3,
    transform_point_to_frame,
    world_to_base,
)
from ..directo_gate_evaluator import _coerce_ur5_joint_vector


# Las funciones extraidas se apenden aqui via sed.
def _live_joint_seed_or_none(panel) -> list[float] | None:
    try:
        joint_snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
    except Exception:
        return None
    live_seed = [
        float(joint_snapshot[name])
        for name in UR5_JOINT_NAMES
        if name in joint_snapshot
    ]
    return _coerce_ur5_joint_vector(live_seed)


# _normalize_joint_goal_near_seed → moved to directo_gate_evaluator.py
# _resolve_joint_goal_normalization_seed → moved to directo_gate_evaluator.py
# _normalize_joint_goal_for_execution → moved to directo_gate_evaluator.py


# _build_transport_seed_candidates → moved to directo_gate_evaluator.py


# _evaluate_transport_stage_preexec_model_guard → moved to directo_gate_evaluator.py
# _direct_pregrasp_gate_caps → moved to directo_gate_evaluator.py
# _should_transport_prep_failure_jump_to_replan → moved to directo_gate_evaluator.py
# _evaluate_transport_stage_postcheck → moved to directo_gate_evaluator.py


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
        f"source=snapshot object={object_name} world={fmt_vec3(snapshot_world)} "
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
        f"source=stable_cache object={object_name} world={fmt_vec3(stable_world)} "
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
                    f"object={object_name} source={result['source']} world={fmt_vec3(result['world'])} "
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
            f"object={object_name} source={result['source']} world={fmt_vec3(result['world'])} "
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
                f"object={object_name} source={result['source']} world={fmt_vec3(result['world'])} "
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
            f"object={object_name} base_source={result['base_source']} base={fmt_vec3(base_pos)} "
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
                f"object={object_name} enabled=true base={fmt_vec3(static_base)}"
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
            f"object={object_name} base_source={result['base_source']} base={fmt_vec3(static_base)} "
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


def _select_compute_stable_promotion_status(
    panel,
    object_name: str,
    *,
    world_result: dict | None,
    snapshot_world,
    stable_world,
    stable_age_sec,
    max_promoted_stable_age_sec: float,
    require_object_on_table: bool,
    resolve_base_fn,
    get_state_fn,
    is_on_table_fn,
    trace_fn,
):
    """F3-step23a: calcula promoted_reject_reasons + on_table_ok + stable_base
    + stable_logical_state para _select_pick_demo_cycle_object_reference (~70 LOC).
    """
    promoted_reject_reasons: list[str] = []
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

    stable_logical_state = "none"
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

    stable_base = None
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

    return promoted_reject_reasons, on_table_ok, stable_base, stable_logical_state


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
        f"phase=BUTTON_PRESS selected_pose_base_link={fmt_vec3(selected_base)} "
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
            f"cycle_object_world={fmt_vec3(selected['world'])} "
            f"cycle_object_base={fmt_vec3(selected['base'])}"
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
    promoted_reject_reasons, on_table_ok, stable_base, stable_logical_state = (
        _select_compute_stable_promotion_status(
            panel,
            object_name,
            world_result=world_result,
            snapshot_world=snapshot_world,
            stable_world=stable_world,
            stable_age_sec=stable_age_sec,
            max_promoted_stable_age_sec=max_promoted_stable_age_sec,
            require_object_on_table=require_object_on_table,
            resolve_base_fn=resolve_base_fn,
            get_state_fn=get_state_fn,
            is_on_table_fn=is_on_table_fn,
            trace_fn=trace_fn,
        )
    )

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
                f"stable_world={fmt_vec3(stable_world)} stable_base={fmt_vec3(stable_base)}"
            )
        if snapshot_base is None:
            trace_fn(
                "[PICK][DIRECT][CYCLE_REF][REJECT] tag=[DIRECT][CYCLE_REF][REJECT] "
                f"phase=BUTTON_PRESS source=snapshot reject_reason=snapshot_base_unavailable "
                f"snapshot_source={snapshot_source} snapshot_reason={snapshot_reason} "
                f"snapshot_world={fmt_vec3(snapshot_world)} snapshot_base={fmt_vec3(snapshot_base)}"
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
        f"cycle_object_world={fmt_vec3(selected['world'])} "
        f"cycle_object_base={fmt_vec3(selected['base'])}"
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
    release_only_reject_logged = False
    last_diag = None
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
            last_diag = {
                "world_obj": (xw, yw, zw),
                "dxy_world": dxy_world,
                "dz_world": dz_world,
                "dxy_base": dxy_base,
                "dz_base": dz_base,
                "release_dxy_world": release_dxy_world,
                "release_dxy_base": release_dxy_base,
            }
            basket_ok = bool(world_ok or base_ok)
            if detached_ok and basket_ok:
                confirmation_source = "basket_reference_world" if world_ok else "basket_reference_base"
                panel._emit_log(
                    "[PICK][DEMO] confirmacion cesta OK "
                    f"source={confirmation_source} "
                    f"world_obj=({xw:.3f},{yw:.3f},{zw:.3f}) "
                    f"world_basket=({basket_world[0]:.3f},{basket_world[1]:.3f},{basket_world[2]:.3f}) "
                    f"world_release={fmt_vec3(release_reference_world)} "
                    f"dxy_w={dxy_world:.3f} dz_w={dz_world:.3f} "
                    f"dxy_b={dxy_base:.3f} dz_b={dz_base:.3f} "
                    f"release_dxy_w={release_dxy_world:.3f} release_dxy_b={release_dxy_base:.3f}"
                )
                return True
            if detached_ok and release_ok and not basket_ok and not release_only_reject_logged:
                release_only_reject_logged = True
                panel._emit_log(
                    "[PICK][DEMO][REJECT] basket_confirmation_release_only "
                    f"world_obj=({xw:.3f},{yw:.3f},{zw:.3f}) "
                    f"world_basket=({basket_world[0]:.3f},{basket_world[1]:.3f},{basket_world[2]:.3f}) "
                    f"world_release={fmt_vec3(release_reference_world)} "
                    f"dxy_w={dxy_world:.3f} dz_w={dz_world:.3f} "
                    f"dxy_b={dxy_base:.3f} dz_b={dz_base:.3f} "
                    f"release_dxy_w={release_dxy_world:.3f} release_dxy_b={release_dxy_base:.3f}"
                )
        time.sleep(0.2)
    if last_diag is not None:
        world_obj = last_diag["world_obj"]
        panel._emit_log(
            "[PICK][DEMO][FAIL] basket_confirmation_timeout "
            f"world_obj=({world_obj[0]:.3f},{world_obj[1]:.3f},{world_obj[2]:.3f}) "
            f"world_basket=({basket_world[0]:.3f},{basket_world[1]:.3f},{basket_world[2]:.3f}) "
            f"world_release={fmt_vec3(release_reference_world)} "
            f"dxy_w={last_diag['dxy_world']:.3f} dz_w={last_diag['dz_world']:.3f} "
            f"dxy_b={last_diag['dxy_base']:.3f} dz_b={last_diag['dz_base']:.3f} "
            f"release_dxy_w={last_diag['release_dxy_world']:.3f} "
            f"release_dxy_b={last_diag['release_dxy_base']:.3f}"
        )
    panel._emit_log("[PICK][DEMO] confirmacion cesta NO alcanzada (timeout)")
    return False


def _validate_demo_transport_follow(
    panel,
    *,
    phase: str,
    timeout_sec: float,
    max_tcp_dist_m: float,
    min_obj_world_z: float,
    min_consecutive: int = 2,
    live_object_world_fn,
    live_object_base_fn,
    live_tcp_base_fn,
    clock_fn=None,
    sleep_fn=None,
) -> dict:
    """Fail fast if the carried object stops physically following the TCP."""
    log_fn = getattr(panel, "_emit_log", lambda _line: None)
    if clock_fn is None:
        clock_fn = time.monotonic
    if sleep_fn is None:
        sleep_fn = time.sleep

    deadline = float(clock_fn()) + max(0.3, float(timeout_sec))
    consecutive_ok = 0
    best_tcp_dist = float("inf")
    best_obj_world_z = float("-inf")
    last_obj_world = None
    last_obj_base = None
    last_tcp_base = None
    next_sample_log_ts = 0.0
    fail_reasons = {
        "pose_unavailable": 0,
        "tcp_dist_above_max": 0,
        "obj_world_z_below_min": 0,
    }

    log_fn(
        "[PICK][DIRECT][TRANSPORT] "
        f"phase={phase} start timeout_sec={float(timeout_sec):.2f} "
        f"max_tcp_dist_m={float(max_tcp_dist_m):.3f} "
        f"min_obj_world_z={float(min_obj_world_z):.3f}"
    )
    while float(clock_fn()) < deadline:
        obj_world = _pick_demo_tuple3(live_object_world_fn())
        obj_base = _pick_demo_tuple3(live_object_base_fn())
        tcp_base = _pick_demo_tuple3(live_tcp_base_fn())
        last_obj_world = obj_world
        last_obj_base = obj_base
        last_tcp_base = tcp_base

        if obj_world is None or obj_base is None or tcp_base is None:
            fail_reasons["pose_unavailable"] += 1
            consecutive_ok = 0
            sleep_fn(0.08)
            continue

        obj_world_z = float(obj_world[2])
        dx = float(obj_base[0]) - float(tcp_base[0])
        dy = float(obj_base[1]) - float(tcp_base[1])
        dz = float(obj_base[2]) - float(tcp_base[2])
        tcp_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        best_tcp_dist = min(best_tcp_dist, tcp_dist)
        best_obj_world_z = max(best_obj_world_z, obj_world_z)

        cond_tcp = tcp_dist <= float(max_tcp_dist_m)
        cond_z = obj_world_z >= float(min_obj_world_z)
        if not cond_tcp:
            fail_reasons["tcp_dist_above_max"] += 1
        if not cond_z:
            fail_reasons["obj_world_z_below_min"] += 1

        now_ts = float(clock_fn())
        if now_ts >= next_sample_log_ts:
            log_fn(
                "[PICK][DIRECT][TRANSPORT] "
                f"phase={phase} sample "
                f"obj_world={fmt_vec3(obj_world)} "
                f"obj_base={fmt_vec3(obj_base)} "
                f"tcp_base={fmt_vec3(tcp_base)} "
                f"tcp_dist={tcp_dist:.3f} obj_world_z={obj_world_z:.3f} "
                f"cond_tcp={str(cond_tcp).lower()} cond_z={str(cond_z).lower()}"
            )
            next_sample_log_ts = now_ts + 0.24

        if cond_tcp and cond_z:
            consecutive_ok += 1
            if consecutive_ok >= max(1, int(min_consecutive)):
                log_fn(
                    "[PICK][DIRECT][TRANSPORT] "
                    f"phase={phase} ok tcp_dist={tcp_dist:.3f} "
                    f"obj_world_z={obj_world_z:.3f} consecutive={consecutive_ok}"
                )
                return {
                    "ok": True,
                    "phase": phase,
                    "tcp_dist": float(tcp_dist),
                    "obj_world_z": float(obj_world_z),
                    "obj_world": obj_world,
                    "obj_base": obj_base,
                    "tcp_base": tcp_base,
                }
        else:
            consecutive_ok = 0
        sleep_fn(0.08)

    fail_reason_keys = [key for key, count in fail_reasons.items() if int(count) > 0] or ["unknown"]
    log_fn(
        "[PICK][DIRECT][TRANSPORT] "
        f"phase={phase} failed best_tcp_dist={_pick_demo_fmt_scalar(best_tcp_dist)} "
        f"best_obj_world_z={_pick_demo_fmt_scalar(best_obj_world_z)} "
        f"last_obj_world={fmt_vec3(last_obj_world)} "
        f"last_obj_base={fmt_vec3(last_obj_base)} "
        f"last_tcp_base={fmt_vec3(last_tcp_base)} "
        f"fail_reasons={','.join(fail_reason_keys)}"
    )
    raise RuntimeError(
        "demo_transport_follow_failed "
        f"phase={phase} best_tcp_dist={_pick_demo_fmt_scalar(best_tcp_dist)} "
        f"best_obj_world_z={_pick_demo_fmt_scalar(best_obj_world_z)} "
        f"last_obj_world={fmt_vec3(last_obj_world)} "
        f"last_obj_base={fmt_vec3(last_obj_base)} "
        f"last_tcp_base={fmt_vec3(last_tcp_base)} "
        f"fail_reasons={','.join(fail_reason_keys)}"
    )


# _compute_demo_basket_targets, _compute_demo_linear_stage_targets,
# _compute_demo_stage_count_for_distance, _compute_demo_transport_recovery_stage_targets,
# _compute_demo_transport_micro_recovery_target → moved to directo_geometry.py


# _compute_demo_joint_prep_waypoint, _compute_demo_joint_prep_waypoints,
# _compute_demo_transport_prep_joint_tol, _joint_step_wait_timeout
# → moved to directo_geometry.py


def _wait_for_demo_runtime_target_progress(
    panel,
    *,
    label: str,
    target_xyz,
    timeout_sec: float,
    tol_xyz_m: float,
    live_tcp_base_fn,
    fallback_wait_fn=None,
    ee_frame: str | None = None,
    clock_fn=None,
    sleep_fn=None,
) -> dict:
    target_xyz_3 = _pick_demo_tuple3(target_xyz)
    if target_xyz_3 is None:
        raise ValueError("runtime_target_unavailable")
    if clock_fn is None:
        clock_fn = time.monotonic
    if sleep_fn is None:
        sleep_fn = time.sleep

    safe_timeout_sec = max(0.1, float(timeout_sec))
    poll_sec = min(
        safe_timeout_sec,
        _pick_demo_env_float(
            "PANEL_PICK_DEMO_TRANSPORT_RUNTIME_PROGRESS_POLL_SEC",
            0.10,
            minimum=0.02,
        ),
    )
    arm_sec = min(
        safe_timeout_sec,
        _pick_demo_env_float(
            "PANEL_PICK_DEMO_TRANSPORT_RUNTIME_PROGRESS_ARM_SEC",
            1.5,
            minimum=0.0,
        ),
    )
    stall_timeout_sec = min(
        safe_timeout_sec,
        _pick_demo_env_float(
            "PANEL_PICK_DEMO_TRANSPORT_RUNTIME_STALL_TIMEOUT_SEC",
            8.0,
            minimum=poll_sec,
        ),
    )
    min_progress_m = _pick_demo_env_float(
        "PANEL_PICK_DEMO_TRANSPORT_RUNTIME_MIN_PROGRESS_M",
        0.008,
        minimum=0.001,
    )
    wrong_direction_tol_m = _pick_demo_env_float(
        "PANEL_PICK_DEMO_TRANSPORT_RUNTIME_WRONG_DIRECTION_TOL_M",
        0.020,
        minimum=0.001,
    )

    start_ts = float(clock_fn())
    deadline_ts = start_ts + safe_timeout_sec
    sample_count = 0
    start_dist_m = None
    best_dist_m = float("inf")
    best_pos = None
    last_pos = None
    last_dist_m = float("inf")
    last_progress_ts = start_ts

    while float(clock_fn()) <= deadline_ts:
        now_ts = float(clock_fn())
        curr_pos = _pick_demo_tuple3(live_tcp_base_fn())
        if curr_pos is not None:
            sample_count += 1
            last_pos = curr_pos
            last_dist_m = math.sqrt(
                (float(curr_pos[0]) - float(target_xyz_3[0])) ** 2
                + (float(curr_pos[1]) - float(target_xyz_3[1])) ** 2
                + (float(curr_pos[2]) - float(target_xyz_3[2])) ** 2
            )
            if start_dist_m is None:
                start_dist_m = float(last_dist_m)
                best_dist_m = float(last_dist_m)
                best_pos = curr_pos
                last_progress_ts = now_ts
            if float(last_dist_m) <= float(tol_xyz_m):
                return {
                    "ok": True,
                    "reason": "target_reached",
                    "pos": curr_pos,
                    "dist_m": float(last_dist_m),
                    "best_dist_m": float(best_dist_m),
                    "start_dist_m": float(start_dist_m),
                    "elapsed_sec": max(0.0, now_ts - start_ts),
                    "samples": int(sample_count),
                }
            if float(last_dist_m) < float(best_dist_m) - float(min_progress_m):
                best_dist_m = float(last_dist_m)
                best_pos = curr_pos
                last_progress_ts = now_ts

            elapsed_sec = max(0.0, now_ts - start_ts)
            if elapsed_sec >= float(arm_sec):
                if (
                    start_dist_m is not None
                    and float(best_dist_m)
                    < float(start_dist_m) - float(min_progress_m)
                    and float(last_dist_m) > float(best_dist_m) + float(wrong_direction_tol_m)
                ):
                    return {
                        "ok": False,
                        "reason": "wrong_direction_drift",
                        "pos": curr_pos,
                        "dist_m": float(last_dist_m),
                        "best_dist_m": float(best_dist_m),
                        "best_pos": _pick_demo_tuple3(best_pos),
                        "start_dist_m": float(start_dist_m),
                        "elapsed_sec": float(elapsed_sec),
                        "samples": int(sample_count),
                    }
                if (now_ts - last_progress_ts) >= float(stall_timeout_sec):
                    return {
                        "ok": False,
                        "reason": "no_progress",
                        "pos": curr_pos,
                        "dist_m": float(last_dist_m),
                        "best_dist_m": float(best_dist_m),
                        "best_pos": _pick_demo_tuple3(best_pos),
                        "start_dist_m": float(start_dist_m),
                        "elapsed_sec": float(elapsed_sec),
                        "samples": int(sample_count),
                    }
        sleep_fn(poll_sec)

    if sample_count == 0 and callable(fallback_wait_fn):
        wait_ok, wait_pos, wait_dist = fallback_wait_fn(
            target_xyz_3,
            timeout_sec=safe_timeout_sec,
            tol_xyz_m=float(tol_xyz_m),
            ee_frame=ee_frame,
        )
        return {
            "ok": bool(wait_ok),
            "reason": "fallback_wait",
            "pos": _pick_demo_tuple3(wait_pos),
            "dist_m": float(wait_dist),
            "best_dist_m": float(wait_dist),
            "start_dist_m": float(wait_dist),
            "elapsed_sec": float(safe_timeout_sec),
            "samples": 0,
        }

    return {
        "ok": False,
        "reason": "timeout" if sample_count > 0 else "pose_unavailable",
        "pos": _pick_demo_tuple3(last_pos),
        "dist_m": float(last_dist_m),
        "best_dist_m": float(best_dist_m if sample_count > 0 else float("inf")),
        "best_pos": _pick_demo_tuple3(best_pos),
        "start_dist_m": float(start_dist_m) if start_dist_m is not None else None,
        "elapsed_sec": float(max(0.0, float(clock_fn()) - start_ts)),
        "samples": int(sample_count),
    }


# _transport_prep_failure_policy → moved to directo_gate_evaluator.py


