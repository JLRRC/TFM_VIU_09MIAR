#!/usr/bin/env python3
"""Tests unitarios de los helpers de geometria canonica del RG2."""

from __future__ import annotations

from pathlib import Path

from ur5_tools.gripper_geometry import (
    load_gripper_geometry,
    RG2_PINCH_CENTER_FRAME,
    RG2_TCP_FRAME,
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
