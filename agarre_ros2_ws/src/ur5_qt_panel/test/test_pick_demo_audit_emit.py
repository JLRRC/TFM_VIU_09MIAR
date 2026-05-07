"""F3-step3a: tests del audit_emit puro (sin Qt/ROS).

Cubre:
* Construcción de AuditEmitContext con stubs.
* audit_emit invoca append_trace para los 7 bloques esperados (audit
  master + BUTTON + SELECT + LIVE_OBJECT + TCP_LIVE + RG2 CONTROL +
  RG2 COMPARE + GEOM + PANEL_TRACE + DIVERGENCE opcional).
* panel._emit_log se invoca exactamente 1 vez (línea geom).
* Schema del audit master contiene las claves contractuales.
* DIVERGENCE solo se emite si delta_panel_live_norm > 0.02.
"""

from __future__ import annotations

from typing import List
from unittest.mock import MagicMock


from ur5_qt_panel.pick_demo.audit_emit import (
    AuditEmitContext,
    audit_emit,
)


# ---------------------------------------------------------------------------
# Stubs
# ---------------------------------------------------------------------------


def _identity_tuple3(value):
    """Stub _tuple3: pasa tuples 3 tal cual, None → None."""
    if value is None:
        return None
    if isinstance(value, (tuple, list)) and len(value) >= 3:
        return (float(value[0]), float(value[1]), float(value[2]))
    return None


def _stub_object_top_pose(pose):
    if pose is None:
        return None
    return (pose[0], pose[1], pose[2] + 0.05)


def _stub_pose_position(target_frame, source_frame, *, timeout_sec):
    return (0.1, 0.2, 0.3)


def _stub_camera_audit_meta(panel):
    return {"topic": "/cam/test", "frame": "frame_01", "image_timestamp": 12345.6}


def _stub_fmt_vec(v):
    if v is None:
        return "none"
    try:
        return f"({v[0]:.3f},{v[1]:.3f},{v[2]:.3f})"
    except Exception:
        return "none"


def _stub_fmt_scalar(value, *, digits=3):
    if value is None:
        return "none"
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return "none"


def _stub_iso_now():
    return "2026-05-03T12:00:00Z"


def _stub_vector_minus(a, b):
    if a is None or b is None:
        return None
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _stub_vec_norm(v):
    if v is None:
        return None
    return (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5


def _stub_z_delta(a, b):
    if a is None or b is None:
        return None
    return a[2] - b[2]


def _stub_json_safe(value):
    return value


def _make_panel(divergence=False):
    """Construye un mock panel con atributos esperados."""
    panel = MagicMock()
    panel._emit_log = MagicMock()
    # Si divergence=True, panel_tcp_fk_base muy lejos de tcp_base.
    if divergence:
        panel._last_tcp_base = (1.0, 1.0, 1.0)  # ⇒ delta grande con tcp 0.4,0.5,0.6
    else:
        panel._last_tcp_base = (0.4, 0.5, 0.6)
    panel._last_tcp_rpy_deg = (0.0, 0.0, 0.0)
    panel._last_tcp_fk_ts = 100.0
    panel._last_trace_tcp_base = (0.4, 0.5, 0.6)
    panel._last_trace_tcp_rpy_deg = (0.0, 0.0, 0.0)
    panel._last_trace_tcp_ts = 99.0
    panel._last_trace_object_age_sec = 0.5
    panel._selection_timestamp = 0.0
    panel._selected_world = (0.5, 0.0, 0.05)
    panel._selected_base = (0.5, 0.0, 0.05)
    panel._world_frame_last_first = lambda fb=None: "world"
    panel._business_base_frame = lambda: "base_link"
    return panel


def _make_ctx(panel=None, divergence=False) -> AuditEmitContext:
    if panel is None:
        panel = _make_panel(divergence=divergence)
    appended: List[str] = []
    return AuditEmitContext(
        panel=panel,
        live_object_world=lambda: (0.5, 0.0, 0.05),
        live_object_base=lambda: (0.5, 0.0, 0.05),
        live_tcp_world=lambda: (0.4, 0.5, 0.6),
        live_tcp_base=lambda: (0.4, 0.5, 0.6),
        append_trace=appended.append,
        run_id="run-test-001",
        selected_name="box_red",
        user_selected="box_red",
        target_object_id="name:box_red",
        direct_execution_frame="tool0",
        direct_source_frame="rg2_pinch_center",
        direct_legacy_tcp_frame="rg2_tcp",
        get_pose_position=_stub_pose_position,
        env_object_height_m_fn=lambda: 0.05,
        camera_audit_meta_fn=_stub_camera_audit_meta,
        tuple3_fn=_identity_tuple3,
        object_top_pose_fn=_stub_object_top_pose,
        fmt_vec_fn=_stub_fmt_vec,
        fmt_scalar_fn=_stub_fmt_scalar,
        iso_now_fn=_stub_iso_now,
        vector_minus_fn=_stub_vector_minus,
        vec_norm_fn=_stub_vec_norm,
        z_delta_fn=_stub_z_delta,
        json_safe_fn=_stub_json_safe,
        world_frame_default="world",
        base_frame_default="base_link",
    ), appended


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_audit_emit_calls_append_trace_at_least_8_times_when_no_divergence():
    """Sin divergence: append_trace se llama 8 veces (1 master + 6 secundarios + GEOM + PANEL_TRACE)."""
    ctx, appended = _make_ctx(divergence=False)
    audit_emit(
        ctx, "BUTTON_PRESS",
        target_source="selected_demo_object",
        target_frame_original="base_link",
    )
    # master + BUTTON + SELECT + LIVE_OBJECT + TCP_LIVE + CONTROL + COMPARE + GEOM + PANEL_TRACE = 9 (sin divergence)
    assert len(appended) == 9


def test_audit_emit_calls_append_trace_10_times_when_divergence():
    """Con panel_tcp_fk_base lejos de tcp_base: añade DIVERGENCE bloque."""
    ctx, appended = _make_ctx(divergence=True)
    audit_emit(
        ctx, "BUTTON_PRESS",
        target_source="x",
        target_frame_original="base_link",
    )
    # 9 normal + 1 DIVERGENCE = 10.
    assert len(appended) == 10
    assert any("[PICK][DIRECT][DIVERGENCE]" in line for line in appended)


def test_audit_emit_master_line_contains_contract_keys():
    """El master line debe contener todas las claves del schema contractual."""
    ctx, appended = _make_ctx()
    audit_emit(
        ctx, "BUTTON_PRESS",
        target_source="selected_demo_object",
        target_frame_original="base_link",
    )
    master = appended[0]
    contract_keys = [
        "stage=BUTTON_PRESS",
        "request_id=run-test-001",
        "grasp_mode=direct_object",
        "selected_object_name=box_red",
        "selected_object_id=name:box_red",
        "user_selected_name=box_red",
        "target_source=selected_demo_object",
        "target_frame_original=base_link",
        "object_pose_world=",
        "object_pose_base_link=",
        "tcp_pose_world=",
        "tcp_pose_base_link=",
        "tool0_pose_world=",
        "rg2_pinch_center_pose_world=",
        "rg2_tcp_pose_world=",
        "delta_object_tcp_world=",
        "world_frame=world",
        "base_frame=base_link",
        "extra=",
    ]
    for key in contract_keys:
        assert key in master, f"missing schema key {key!r} in master line"


def test_audit_emit_invokes_panel_emit_log_exactly_once():
    """Sólo la línea GEOM va al panel._emit_log."""
    ctx, _ = _make_ctx()
    audit_emit(
        ctx, "STAGE_X",
        target_source="x", target_frame_original=None,
    )
    assert ctx.panel._emit_log.call_count == 1
    log_arg = ctx.panel._emit_log.call_args[0][0]
    assert log_arg.startswith("[RG2][AUDIT][GEOM]")


def test_audit_emit_handles_none_for_optional_kwargs():
    """target_pose_*, command_pose_sent y extra pueden ser None."""
    ctx, appended = _make_ctx()
    audit_emit(
        ctx, "TEST",
        target_source="src",
        target_frame_original=None,
        target_pose_original=None,
        target_pose_world=None,
        target_pose_base_link=None,
        command_pose_sent=None,
        command_frame=None,
        command_joint_goal=None,
        extra=None,
    )
    master = appended[0]
    assert "target_frame_original=none" in master
    assert "command_frame=none" in master
    assert "command_joint_goal=null" in master  # json.dumps(None) = "null"


def test_audit_emit_uses_world_frame_default_when_callable_missing():
    """Si panel._world_frame_last_first no existe, se usa world_frame_default."""
    panel = _make_panel()
    # Simular missing attr.
    del panel._world_frame_last_first
    ctx, appended = _make_ctx(panel=panel)
    audit_emit(
        ctx, "TEST",
        target_source="x", target_frame_original=None,
    )
    master = appended[0]
    assert "world_frame=world" in master


def test_audit_emit_extra_payload_serialized_as_json():
    """El kwarg `extra` se serializa con json.dumps + sort_keys."""
    ctx, appended = _make_ctx()
    audit_emit(
        ctx, "TEST",
        target_source="x", target_frame_original=None,
        extra={"key_b": 2, "key_a": 1},
    )
    master = appended[0]
    # sort_keys=True en json.dumps ⇒ key_a antes de key_b.
    assert 'extra={"key_a": 1, "key_b": 2}' in master


def test_audit_emit_geom_line_includes_dz_and_object_height():
    """GEOM line es la única que va a _emit_log y debe llevar dz_*"""
    ctx, _ = _make_ctx()
    audit_emit(
        ctx, "TEST",
        target_source="x", target_frame_original=None,
    )
    log_call = ctx.panel._emit_log.call_args[0][0]
    assert "dz_tool0_vs_object_top_m=" in log_call
    assert "dz_pinch_center_vs_object_top_m=" in log_call
    assert "dz_rg2_tcp_vs_object_top_m=" in log_call
    assert "object_height_m=" in log_call
