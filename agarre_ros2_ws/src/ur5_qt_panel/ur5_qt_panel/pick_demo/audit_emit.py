#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/audit_emit.py
# Contenido: F3-step3 — extracción de _audit_emit (210 LOC) del closure run_pick_demo.worker.
"""``audit_emit`` — emisor pure-ish de los logs estructurados del Pick Demo.

Hasta F3-step3 (2026-05-03), ``_audit_emit`` vivía como ``def`` anidada
dentro del closure ``run_pick_demo.worker``, capturando 10 free vars
(``panel`` + 5 helpers de pose live + 4 vars de contexto). Aquí se
extrae a module-level con ``AuditEmitContext`` como **único** argumento
de contexto (todos los kwargs específicos del evento siguen viniendo
explícitamente como antes).

El cuerpo de la función conserva la estructura original 1:1 (mismas
``_append_trace`` calls, mismas claves del schema audit). El único
cambio es que las dependencias closure (``_append_trace``,
``_live_object_world``, ``_live_object_base``, ``_live_tcp_world``,
``_live_tcp_base``, ``run_id``, ``selected_name``, ``user_selected``,
``target_object_id``) ahora se accesan vía ``ctx.<campo>``.

En ``panel_pick_demo.run_pick_demo.worker`` queda un wrapper trivial
que construye ``AuditEmitContext`` y delega — esto preserva los 4
callsites originales sin modificar firmas.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional


# Re-export del prefix audit para que el módulo sea autocontenido.
_DIRECT_GRASP_AUDIT_PREFIX = "[PICK][DIRECT_GRASP_AUDIT]"


@dataclass
class AuditEmitContext:
    """Contexto que captura las dependencias closure de ``_audit_emit``.

    Atributos:
        panel: instancia ControlPanelV2 (para _emit_log + atributos
            ``_world_frame_last_first``, ``_business_base_frame``,
            ``_selection_timestamp``, ``_selected_world``, ``_selected_base``,
            ``_last_tcp_*``, ``_last_trace_*``).
        live_object_world: callable() -> tuple3|None. Pose live del objeto
            en world. Capturado del closure.
        live_object_base: idem en base_link.
        live_tcp_world: idem TCP world.
        live_tcp_base: idem TCP base_link.
        append_trace: callable(line: str) -> None. Helper de log a
            disco/audit.log del closure.
        run_id: identificador único de la sesión run_pick_demo.
        selected_name: nombre del objeto seleccionado (string vacío si
            ninguno).
        user_selected: nombre seleccionado por el usuario (puede diferir
            de ``selected_name`` si hubo override programático).
        target_object_id: ID derivado (e.g. ``name:box_red``).
        world_frame_default: fallback si ``panel._world_frame_last_first``
            no está disponible (default ``"world"``).
        base_frame_default: fallback si ``panel._business_base_frame``
            no responde (default ``"base_link"``).
        direct_execution_frame / direct_source_frame /
            direct_legacy_tcp_frame: constantes del módulo padre que el
            audit usa para resolver poses TF.
        get_pose_position: callable(target_frame, source_frame, *,
            timeout_sec) -> tuple3|None. Wrapper TF (typically the
            module-level ``_pose_position`` already promoted in step1.1).
        env_object_height_m_fn: callable() -> float. Resuelve la altura
            del objeto vía env (PANEL_PICK_DEMO_OBJECT_HEIGHT_M).
        camera_audit_meta_fn: callable(panel) -> dict. Helper module-level
            ``_camera_audit_meta`` (promovido en F3-step1.3).
        tuple3_fn: callable(value) -> tuple3|None.
        object_top_pose_fn: callable(pose) -> tuple3|None.
        fmt_vec_fn / fmt_scalar_fn: formatters.
        iso_now_fn: callable() -> str timestamp ISO.
        vector_minus_fn / vec_norm_fn / z_delta_fn: math helpers.
        json_safe_fn: serializer recursivo.
    """

    panel: Any
    live_object_world: Callable[[], Any]
    live_object_base: Callable[[], Any]
    live_tcp_world: Callable[[], Any]
    live_tcp_base: Callable[[], Any]
    append_trace: Callable[[str], None]
    run_id: str
    selected_name: str
    user_selected: str
    target_object_id: str
    direct_execution_frame: str
    direct_source_frame: str
    direct_legacy_tcp_frame: str
    get_pose_position: Callable[..., Any]
    env_object_height_m_fn: Callable[[], float]
    camera_audit_meta_fn: Callable[[Any], dict]
    tuple3_fn: Callable[[Any], Any]
    object_top_pose_fn: Callable[[Any], Any]
    fmt_vec_fn: Callable[[Any], str]
    fmt_scalar_fn: Callable[..., str]
    iso_now_fn: Callable[[], str]
    vector_minus_fn: Callable[[Any, Any], Any]
    vec_norm_fn: Callable[[Any], Any]
    z_delta_fn: Callable[[Any, Any], Any]
    json_safe_fn: Callable[[Any], Any]
    world_frame_default: str = "world"
    base_frame_default: str = "base_link"


def audit_emit(
    ctx: AuditEmitContext,
    stage: str,
    *,
    target_source: str,
    target_frame_original: Optional[str],
    target_pose_original: Any = None,
    target_pose_world: Any = None,
    target_pose_base_link: Any = None,
    command_pose_sent: Any = None,
    command_frame: Optional[str] = None,
    command_joint_goal: Any = None,
    extra: Optional[dict] = None,
) -> None:
    """Emite el bloque de logs structured del audit Pick Demo.

    Equivalente funcional 1:1 al ``_audit_emit`` legacy del closure.
    Conserva todas las claves del schema; cualquier cambio de schema
    debe sincronizarse con consumidores externos (test_quality_metrics,
    parsers de evidencia, etc.).
    """
    panel = ctx.panel
    world_frame = str(
        getattr(panel, "_world_frame_last_first", lambda fallback=None: ctx.world_frame_default)(
            ctx.world_frame_default
        )
    ).strip() or ctx.world_frame_default
    try:
        base_frame = str(panel._business_base_frame() or ctx.base_frame_default)
    except Exception:
        base_frame = str(ctx.base_frame_default)
    object_world = ctx.tuple3_fn(ctx.live_object_world())
    object_base = ctx.tuple3_fn(ctx.live_object_base())
    object_top_world = ctx.object_top_pose_fn(object_world)
    object_top_base = ctx.object_top_pose_fn(object_base)
    tcp_world = ctx.tuple3_fn(ctx.live_tcp_world())
    tcp_base = ctx.tuple3_fn(ctx.live_tcp_base())
    tool0_world = ctx.get_pose_position(world_frame, ctx.direct_execution_frame, timeout_sec=0.12)
    tool0_base = ctx.get_pose_position(base_frame, ctx.direct_execution_frame, timeout_sec=0.12)
    pinch_world = ctx.get_pose_position(world_frame, ctx.direct_source_frame, timeout_sec=0.12)
    pinch_base = ctx.get_pose_position(base_frame, ctx.direct_source_frame, timeout_sec=0.12) or tcp_base
    rg2_tcp_world = ctx.get_pose_position(world_frame, ctx.direct_legacy_tcp_frame, timeout_sec=0.12)
    rg2_tcp_base = ctx.get_pose_position(base_frame, ctx.direct_legacy_tcp_frame, timeout_sec=0.12)
    object_height_m = ctx.env_object_height_m_fn()
    camera_meta = ctx.camera_audit_meta_fn(panel)
    selection_ts = float(getattr(panel, "_selection_timestamp", 0.0) or 0.0)
    selection_age = max(0.0, time.time() - selection_ts) if selection_ts > 0.0 else None
    selected_world = ctx.tuple3_fn(getattr(panel, "_selected_world", None))
    selected_base = ctx.tuple3_fn(getattr(panel, "_selected_base", None))
    panel_tcp_fk_base = ctx.tuple3_fn(getattr(panel, "_last_tcp_base", None))
    panel_tcp_fk_rpy_deg = ctx.tuple3_fn(getattr(panel, "_last_tcp_rpy_deg", None))
    panel_tcp_fk_age = max(
        0.0,
        time.monotonic() - float(getattr(panel, "_last_tcp_fk_ts", 0.0) or 0.0),
    ) if float(getattr(panel, "_last_tcp_fk_ts", 0.0) or 0.0) > 0.0 else None
    panel_trace_tcp_base = ctx.tuple3_fn(getattr(panel, "_last_trace_tcp_base", None))
    panel_trace_tcp_rpy_deg = ctx.tuple3_fn(getattr(panel, "_last_trace_tcp_rpy_deg", None))
    panel_trace_tcp_age = max(
        0.0,
        time.monotonic() - float(getattr(panel, "_last_trace_tcp_ts", 0.0) or 0.0),
    ) if float(getattr(panel, "_last_trace_tcp_ts", 0.0) or 0.0) > 0.0 else None
    panel_object_age = getattr(panel, "_last_trace_object_age_sec", None)
    delta_world = ctx.vector_minus_fn(tcp_world, object_world)
    delta_base = ctx.vector_minus_fn(tcp_base, object_base)
    delta_pinch_world = ctx.vector_minus_fn(pinch_world, object_world)
    delta_pinch_base = ctx.vector_minus_fn(pinch_base, object_base)
    delta_panel_live = ctx.vector_minus_fn(panel_tcp_fk_base, tcp_base)
    delta_panel_live_norm = ctx.vec_norm_fn(delta_panel_live)
    legacy_gap = ctx.vector_minus_fn(rg2_tcp_base, pinch_base)
    legacy_gap_norm = ctx.vec_norm_fn(legacy_gap)
    tool0_top_dz = ctx.z_delta_fn(tool0_base, object_top_base)
    pinch_top_dz = ctx.z_delta_fn(pinch_base, object_top_base)
    rg2_tcp_top_dz = ctx.z_delta_fn(rg2_tcp_base, object_top_base)
    extra_payload = ctx.json_safe_fn(extra) or {}
    fmt_vec = ctx.fmt_vec_fn
    fmt_scalar = ctx.fmt_scalar_fn
    ctx.append_trace(
        f"{_DIRECT_GRASP_AUDIT_PREFIX} "
        f"stage={stage} "
        f"timestamp={ctx.iso_now_fn()} "
        f"request_id={ctx.run_id} "
        f"grasp_mode=direct_object "
        f"selected_object_name={ctx.selected_name or 'none'} "
        f"selected_object_id={ctx.target_object_id} "
        f"user_selected_name={ctx.user_selected or 'none'} "
        f"selection_age_sec={fmt_scalar(selection_age)} "
        f"target_source={target_source or 'none'} "
        f"target_frame_original={target_frame_original or 'none'} "
        f"target_pose_original={fmt_vec(target_pose_original)} "
        f"target_pose_world={fmt_vec(target_pose_world)} "
        f"target_pose_base_link={fmt_vec(target_pose_base_link)} "
        f"selected_pose_world={fmt_vec(selected_world)} "
        f"selected_pose_base_link={fmt_vec(selected_base)} "
        f"object_pose_world={fmt_vec(object_world)} "
        f"object_pose_base_link={fmt_vec(object_base)} "
        f"object_top_pose_world={fmt_vec(object_top_world)} "
        f"object_top_pose_base_link={fmt_vec(object_top_base)} "
        f"tcp_pose_world={fmt_vec(tcp_world)} "
        f"tcp_pose_base_link={fmt_vec(tcp_base)} "
        f"tool0_pose_world={fmt_vec(tool0_world)} "
        f"tool0_pose_base_link={fmt_vec(tool0_base)} "
        f"rg2_pinch_center_pose_world={fmt_vec(pinch_world)} "
        f"rg2_pinch_center_pose_base_link={fmt_vec(pinch_base)} "
        f"rg2_tcp_pose_world={fmt_vec(rg2_tcp_world)} "
        f"rg2_tcp_pose_base_link={fmt_vec(rg2_tcp_base)} "
        f"delta_object_tcp_world={fmt_vec(delta_world)} "
        f"delta_object_tcp_base={fmt_vec(delta_base)} "
        f"delta_object_pinch_center_world={fmt_vec(delta_pinch_world)} "
        f"delta_object_pinch_center_base={fmt_vec(delta_pinch_base)} "
        f"dz_tool0_vs_object_top_m={fmt_scalar(tool0_top_dz)} "
        f"dz_pinch_center_vs_object_top_m={fmt_scalar(pinch_top_dz)} "
        f"dz_rg2_tcp_vs_object_top_m={fmt_scalar(rg2_tcp_top_dz)} "
        f"command_pose_sent={fmt_vec(command_pose_sent)} "
        f"command_frame={command_frame or 'none'} "
        f"command_joint_goal={json.dumps(ctx.json_safe_fn(command_joint_goal), ensure_ascii=True)} "
        f"camera_topic={camera_meta['topic']} "
        f"camera_frame={camera_meta['frame']} "
        f"image_timestamp={fmt_scalar(camera_meta['image_timestamp'])} "
        f"pose_from_image=false "
        f"panel_tcp_fk_base={fmt_vec(panel_tcp_fk_base)} "
        f"panel_tcp_fk_rpy_deg={fmt_vec(panel_tcp_fk_rpy_deg)} "
        f"panel_trace_tcp_base={fmt_vec(panel_trace_tcp_base)} "
        f"panel_trace_tcp_rpy_deg={fmt_vec(panel_trace_tcp_rpy_deg)} "
        f"delta_panel_tcp_live={fmt_vec(delta_panel_live)} "
        f"delta_panel_tcp_live_norm_m={fmt_scalar(delta_panel_live_norm)} "
        f"panel_tcp_fk_age_sec={fmt_scalar(panel_tcp_fk_age)} "
        f"panel_trace_tcp_age_sec={fmt_scalar(panel_trace_tcp_age)} "
        f"panel_object_age_sec={fmt_scalar(panel_object_age)} "
        f"object_height_m={fmt_scalar(object_height_m)} "
        f"world_frame={world_frame} "
        f"base_frame={base_frame} "
        f"extra={json.dumps(extra_payload, ensure_ascii=True, sort_keys=True)}"
    )
    ctx.append_trace(
        "[PICK][DIRECT][BUTTON] "
        f"stage={stage} request_id={ctx.run_id} grasp_mode=direct_object "
        f"selected_object={ctx.selected_name or 'none'} user_selected={ctx.user_selected or 'none'} "
        f"target_source={target_source or 'none'} target_frame_original={target_frame_original or 'none'} "
        f"selection_age_sec={fmt_scalar(selection_age)} pose_from_image=false"
    )
    ctx.append_trace(
        "[PICK][DIRECT][SELECT] "
        f"stage={stage} selected_pose_world={fmt_vec(selected_world)} "
        f"selected_pose_base_link={fmt_vec(selected_base)} "
        f"target_pose_original={fmt_vec(target_pose_original)} "
        f"target_pose_world={fmt_vec(target_pose_world)} "
        f"target_pose_base_link={fmt_vec(target_pose_base_link)} "
        f"panel_object_age_sec={fmt_scalar(panel_object_age)}"
    )
    ctx.append_trace(
        "[PICK][DIRECT][LIVE_OBJECT] "
        f"stage={stage} object_pose_world={fmt_vec(object_world)} "
        f"object_pose_base_link={fmt_vec(object_base)} world_frame={world_frame} base_frame={base_frame}"
    )
    ctx.append_trace(
        "[PICK][DIRECT][TCP_LIVE] "
        f"stage={stage} tcp_pose_world={fmt_vec(tcp_world)} "
        f"tcp_pose_base_link={fmt_vec(tcp_base)} "
        f"tool0_pose_world={fmt_vec(tool0_world)} tool0_pose_base_link={fmt_vec(tool0_base)} "
        f"rg2_pinch_center_pose_world={fmt_vec(pinch_world)} rg2_pinch_center_pose_base_link={fmt_vec(pinch_base)} "
        f"rg2_tcp_pose_world={fmt_vec(rg2_tcp_world)} rg2_tcp_pose_base_link={fmt_vec(rg2_tcp_base)} "
        f"panel_trace_tcp_base={fmt_vec(panel_trace_tcp_base)} panel_trace_tcp_rpy_deg={fmt_vec(panel_trace_tcp_rpy_deg)} "
        f"panel_trace_tcp_age_sec={fmt_scalar(panel_trace_tcp_age)}"
    )
    ctx.append_trace(
        "[RG2][AUDIT][CONTROL] "
        f"stage={stage} reasoning_frame={ctx.direct_source_frame} "
        f"legacy_tcp_frame={ctx.direct_legacy_tcp_frame} execution_frame={ctx.direct_execution_frame} "
        f"tool0_pose_base_link={fmt_vec(tool0_base)} "
        f"rg2_pinch_center_pose_base_link={fmt_vec(pinch_base)} "
        f"rg2_tcp_pose_base_link={fmt_vec(rg2_tcp_base)}"
    )
    ctx.append_trace(
        "[RG2][AUDIT][COMPARE] "
        f"stage={stage} object_pose_base_link={fmt_vec(object_base)} "
        f"tool0_pose_base_link={fmt_vec(tool0_base)} "
        f"rg2_pinch_center_pose_base_link={fmt_vec(pinch_base)} "
        f"rg2_tcp_pose_base_link={fmt_vec(rg2_tcp_base)} "
        f"delta_object_pinch_center_base={fmt_vec(delta_pinch_base)} "
        f"delta_object_tcp_base={fmt_vec(delta_base)} "
        f"legacy_tcp_vs_pinch_center_base={fmt_vec(legacy_gap)} "
        f"legacy_tcp_vs_pinch_center_dist_m={fmt_scalar(legacy_gap_norm)}"
    )
    geom_audit_line = (
        "[RG2][AUDIT][GEOM] "
        f"stage={stage} "
        f"object_pose_base_link={fmt_vec(object_base)} "
        f"object_top_pose_base_link={fmt_vec(object_top_base)} "
        f"tool0_pose_base_link={fmt_vec(tool0_base)} "
        f"rg2_pinch_center_pose_base_link={fmt_vec(pinch_base)} "
        f"rg2_tcp_pose_base_link={fmt_vec(rg2_tcp_base)} "
        f"dz_tool0_vs_object_top_m={fmt_scalar(tool0_top_dz)} "
        f"dz_pinch_center_vs_object_top_m={fmt_scalar(pinch_top_dz)} "
        f"dz_rg2_tcp_vs_object_top_m={fmt_scalar(rg2_tcp_top_dz)} "
        f"object_height_m={fmt_scalar(object_height_m)}"
    )
    panel._emit_log(geom_audit_line)
    ctx.append_trace(geom_audit_line)
    ctx.append_trace(
        "[PICK][DIRECT][PANEL_TRACE] "
        f"stage={stage} panel_tcp_fk_base={fmt_vec(panel_tcp_fk_base)} "
        f"panel_tcp_fk_rpy_deg={fmt_vec(panel_tcp_fk_rpy_deg)} "
        f"tcp_live_base={fmt_vec(tcp_base)} "
        f"delta_panel_tcp_live={fmt_vec(delta_panel_live)} "
        f"delta_panel_tcp_live_norm_m={fmt_scalar(delta_panel_live_norm)} "
        f"panel_tcp_fk_age_sec={fmt_scalar(panel_tcp_fk_age)}"
    )
    if delta_panel_live_norm is not None and delta_panel_live_norm > 0.02:
        ctx.append_trace(
            "[PICK][DIRECT][DIVERGENCE] "
            "kind=panel_fk_vs_live_tf "
            f"stage={stage} delta_m={fmt_scalar(delta_panel_live_norm)} "
            f"panel_tcp_fk_base={fmt_vec(panel_tcp_fk_base)} "
            f"tcp_live_base={fmt_vec(tcp_base)} "
            f"delta={fmt_vec(delta_panel_live)} "
            "note=panel_fk_is_model_pose_not_runtime_rg2_tcp"
        )
