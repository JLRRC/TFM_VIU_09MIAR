#!/usr/bin/env python3
"""Tests unitarios de los helpers de geometria canonica del RG2."""

from __future__ import annotations

from pathlib import Path

from ur5_tools.gripper_geometry import (
    TOOL0_FRAME,
    load_gripper_geometry,
    RG2_PINCH_CENTER_FRAME,
    RG2_TCP_FRAME,
    contact_z_correction_for_frame,
    evaluate_geometry_snapshot,
    evaluate_runtime_geometry,
    tool0_offset_for_frame,
    validate_pick_demo_anchor,
    vector_distance,
)


ROOT = Path(__file__).resolve().parents[3]


def test_load_gripper_geometry_matches_validated_tcp_fix() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    tcp_xyz = geometry.xyz_for_frame(RG2_TCP_FRAME)
    pinch_xyz = geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME)
    assert tcp_xyz[2] == pinch_xyz[2]
    assert tcp_xyz[2] == 0.0050885
    assert vector_distance(tcp_xyz, pinch_xyz) == 0.0


def test_model_anchor_matches_canonical_geometry() -> None:
    ok, reason = validate_pick_demo_anchor(
        str(ROOT / 'models' / 'ur5_rg2' / 'model.sdf'),
        geometry=load_gripper_geometry(str(ROOT)),
        tolerance_m=1e-6,
    )
    assert ok, reason


def test_contact_z_correction_is_urdf_driven() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    assert contact_z_correction_for_frame(RG2_PINCH_CENTER_FRAME, geometry=geometry) == 0.0
    assert contact_z_correction_for_frame(RG2_TCP_FRAME, geometry=geometry) == 0.0
    assert contact_z_correction_for_frame(TOOL0_FRAME, geometry=geometry) == 0.0050885


def test_tool0_offset_for_contact_frames_is_urdf_driven() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    assert tool0_offset_for_frame(RG2_PINCH_CENTER_FRAME, geometry=geometry) == (
        0.0,
        0.0,
        0.0050885,
    )
    assert tool0_offset_for_frame(RG2_TCP_FRAME, geometry=geometry) == (
        0.0,
        0.0,
        0.0050885,
    )


def test_evaluate_geometry_snapshot_accepts_matching_runtime_capture() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    ok, reason, snapshot = evaluate_geometry_snapshot(
        {
            RG2_TCP_FRAME: geometry.xyz_for_frame(RG2_TCP_FRAME),
            RG2_PINCH_CENTER_FRAME: geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME),
        },
        geometry=geometry,
    )
    assert ok, reason
    assert reason == "ok"
    assert snapshot["pair_error_m"] == 0.0
    assert snapshot["source_path"] == geometry.tcp.source_path


def test_evaluate_geometry_snapshot_rejects_runtime_mismatch() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    ok, reason, snapshot = evaluate_geometry_snapshot(
        {
            RG2_TCP_FRAME: (0.0, 0.0, 0.1050885),
            RG2_PINCH_CENTER_FRAME: geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME),
        },
        geometry=geometry,
        offset_tol_m=0.002,
    )
    assert not ok
    assert "tool0->rg2_tcp err_m=" in reason
    assert snapshot["frame_error_m"][RG2_TCP_FRAME] > 0.09


def test_evaluate_runtime_geometry_uses_single_runtime_lookup_callback() -> None:
    geometry = load_gripper_geometry(str(ROOT))

    def _lookup(tool_frame: str, frame_name: str):
        assert tool_frame == TOOL0_FRAME
        return geometry.xyz_for_frame(frame_name)

    ok, reason, snapshot = evaluate_runtime_geometry(
        _lookup,
        geometry=geometry,
        tool_frame=TOOL0_FRAME,
    )

    assert ok, reason
    assert reason == "ok"
    assert snapshot["ok"] is True
    assert snapshot["reason"] == "ok"
    assert snapshot["urdf_source"] == geometry.tcp.source_path


def test_evaluate_runtime_geometry_returns_partial_snapshot_on_lookup_failure() -> None:
    geometry = load_gripper_geometry(str(ROOT))

    def _lookup(tool_frame: str, frame_name: str):
        if frame_name == RG2_TCP_FRAME:
            return geometry.xyz_for_frame(frame_name)
        raise LookupError(f"{tool_frame}->{frame_name} missing")

    ok, reason, snapshot = evaluate_runtime_geometry(
        _lookup,
        geometry=geometry,
        tool_frame=TOOL0_FRAME,
    )

    assert not ok
    assert reason == "tool0->rg2_pinch_center missing"
    assert snapshot["ok"] is False
    assert snapshot["reason"] == "tool0->rg2_pinch_center missing"
    assert snapshot["actual_xyz"][RG2_TCP_FRAME] == list(geometry.xyz_for_frame(RG2_TCP_FRAME))
