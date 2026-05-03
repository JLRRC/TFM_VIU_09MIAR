"""F3-step3c: tests smoke del align_demo_grasp_direct extraído.

La función real ejecuta IK calls + audit logs por cada intento; no es
viable testearla end-to-end sin Qt/ROS. Aquí cubrimos:

  * Construcción de AlignGraspContext + AlignGraspState válidos.
  * AlignGraspState es mutable (z_alineada_alert_emitted modificable).
  * align_demo_grasp_direct lanza RuntimeError cuando obj_base no está
    disponible al inicio (validación temprana del contrato).
  * Tras "already_aligned" early return, devuelve dict con label
    GRASP_ALIGN_IK y runtime_target_ok=True.
"""

from __future__ import annotations

from dataclasses import dataclass
from unittest.mock import MagicMock

import pytest

from ur5_qt_panel.pick_demo.align_grasp import (
    AlignGraspContext,
    AlignGraspState,
    align_demo_grasp_direct,
)


def _stub_fmt_vec(v):
    return "(0,0,0)" if v is None else f"({v[0]:.3f},{v[1]:.3f},{v[2]:.3f})"


def _stub_fmt_scalar(value, *, digits=3):
    if value is None:
        return "none"
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return "none"


def _stub_tuple3(value):
    if value is None:
        return None
    if isinstance(value, (tuple, list)) and len(value) >= 3:
        return (float(value[0]), float(value[1]), float(value[2]))
    return None


def _make_pick_demo_params():
    """Mock params dataclass-like usado por la función pura."""
    p = MagicMock()
    p.direct_ik_runtime_attempts = 3
    p.align_z_residual_tol_m = 0.005
    p.align_z_improve_min_m = 0.002
    p.align_z_bias_gain = 0.5
    p.align_z_bias_cap_m = 0.02
    p.align_xy_lock_factor = 1.5
    p.align_exit_xy_tol_m = 0.005
    p.align_exit_z_tol_m = 0.008
    p.align_ik_err_tol = 0.05
    p.align_ik_seed_weight = 0.3
    p.align_rot_weight = 0.1
    p.align_no_effect_tol_m = 0.001
    return p


def _make_ctx(*, obj_base_value=None):
    """Construye un AlignGraspContext con stubs mínimos."""
    panel = MagicMock()
    panel._emit_log = MagicMock()
    return AlignGraspContext(
        panel=panel,
        live_object_base=lambda: obj_base_value,
        live_tcp_base=lambda: (0.4, 0.0, 0.10),
        move_tcp_direct=MagicMock(return_value={"runtime_target_ok": True}),
        append_trace=MagicMock(),
        resolved_align_object_base=lambda: (
            obj_base_value, "test_source", {},
        ),
        pre_close_alignment_metrics=lambda: {
            "xy_dist": 0.001,  # ya alineado en XY
            "z_error": 0.001,  # ya alineado en Z
            "xy_tol": 0.020,
            "z_tol": 0.020,
            "tcp_obj_dist": 0.05,
        },
        direct_debug_state={},
        move_sec=2.0,
        directo_grasp_z=0.10,
        grasp_contact_z_offset_m=0.0,
        get_pick_demo_params=_make_pick_demo_params,
        fmt_vec=_stub_fmt_vec,
        fmt_scalar=_stub_fmt_scalar,
        tuple3=_stub_tuple3,
        direct_source_frame="rg2_pinch_center",
        direct_execution_frame="tool0",
    )


# ---------------------------------------------------------------------------
# AlignGraspState
# ---------------------------------------------------------------------------


def test_align_grasp_state_default_is_false():
    state = AlignGraspState()
    assert state.z_alineada_alert_emitted is False


def test_align_grasp_state_is_mutable():
    state = AlignGraspState(z_alineada_alert_emitted=False)
    state.z_alineada_alert_emitted = True
    assert state.z_alineada_alert_emitted is True


# ---------------------------------------------------------------------------
# Contract: RuntimeError si obj_base no disponible al inicio
# ---------------------------------------------------------------------------


def test_align_demo_grasp_direct_raises_when_obj_base_unavailable():
    ctx = _make_ctx(obj_base_value=None)
    state = AlignGraspState()
    with pytest.raises(RuntimeError, match="demo_object_pose_unavailable_before_align"):
        align_demo_grasp_direct(ctx, state)


# ---------------------------------------------------------------------------
# Happy path: already_aligned ⇒ early return con label GRASP_ALIGN_IK
# ---------------------------------------------------------------------------


def test_align_demo_grasp_direct_returns_dict_when_already_aligned():
    """Si pre_metrics indican que ya está alineado, retorna dict sin IK."""
    ctx = _make_ctx(obj_base_value=(0.4, 0.0, 0.05))
    state = AlignGraspState()
    result = align_demo_grasp_direct(ctx, state)
    assert isinstance(result, dict)
    assert result["label"] == "GRASP_ALIGN_IK"
    assert result["runtime_target_ok"] is True
    # move_tcp_direct NO se invoca cuando ya está alineado.
    assert ctx.move_tcp_direct.call_count == 0
