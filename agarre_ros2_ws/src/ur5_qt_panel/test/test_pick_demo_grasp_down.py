"""F3-step3d: tests smoke del run_grasp_down_conservative extraído.

Como en align_grasp, no podemos testear E2E sin Qt/ROS. Cubrimos:

  * Construcción válida de GraspDownContext.
  * El módulo importa OK (smoke import).
  * Signature correcta (kwargs requeridos).
"""

from __future__ import annotations

from unittest.mock import MagicMock


from ur5_qt_panel.pick_demo.grasp_down import (
    GraspDownContext,
    run_grasp_down_conservative,
)


def _make_ctx():
    panel = MagicMock()
    panel._emit_log = MagicMock()
    return GraspDownContext(
        panel=panel,
        live_object_base=lambda: (0.4, 0.0, 0.05),
        live_tcp_base=lambda: (0.4, 0.0, 0.10),
        move_tcp_direct=MagicMock(),
        run_joint_step=MagicMock(),
        append_trace=MagicMock(),
        emit_pose_consistency=MagicMock(),
        pose_consistency_metrics=MagicMock(return_value={}),
        grasp_down_joint_quality=MagicMock(return_value={}),
        grasp_down_runtime_metrics=MagicMock(return_value={}),
        grasp_down_waypoints=MagicMock(return_value=[]),
        joint_preset_fallback_ok=MagicMock(return_value=False),
        current_joint_seed=MagicMock(return_value=[0.0] * 6),
        move_sec=2.0,
        get_pick_demo_params=MagicMock(),
        fmt_vec=lambda v: "(0,0,0)" if v is None else f"({v[0]:.3f},{v[1]:.3f},{v[2]:.3f})",
        fmt_scalar=lambda value, *, digits=3: "none" if value is None else f"{value:.{digits}f}",
        tuple3=lambda v: tuple(float(c) for c in v[:3]) if v is not None else None,
        iso_now=lambda: "2026-05-03T12:00:00Z",
    )


def test_grasp_down_context_constructs_with_all_required_fields():
    ctx = _make_ctx()
    assert ctx.panel is not None
    assert callable(ctx.live_object_base)
    assert callable(ctx.move_tcp_direct)
    assert callable(ctx.run_joint_step)


def test_run_grasp_down_conservative_is_callable():
    """Smoke: la función importa y es callable. No la invocamos por
    completo (requeriría stub de ~13 helpers funcionales)."""
    assert callable(run_grasp_down_conservative)


def test_run_grasp_down_conservative_requires_kwonly_args():
    """Verifica que la firma kw-only se preserva del legacy."""
    import inspect
    sig = inspect.signature(run_grasp_down_conservative)
    params = sig.parameters
    assert "ctx" in params
    assert "target_base" in params
    assert "obj_base" in params
    assert "timeout_sec" in params
    assert "audit_target_source" in params
    assert "phase_seed_joints" in params
    # Verificar que target_base / obj_base / etc. son KEYWORD_ONLY.
    for name in ("target_base", "obj_base", "timeout_sec", "audit_target_source", "phase_seed_joints"):
        assert params[name].kind == inspect.Parameter.KEYWORD_ONLY, \
            f"{name} debería ser KEYWORD_ONLY"
