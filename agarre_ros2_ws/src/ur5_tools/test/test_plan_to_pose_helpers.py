#!/usr/bin/env python3
"""F10 (auditoría 2026-05-10): tests offline de plan_to_pose_helpers."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from ur5_tools.plan_to_pose_helpers import (
    extract_ordered_joint_positions,
    parse_plan_to_pose_request,
    select_traj_duration_and_timeout,
)


# -------------------- extract_ordered_joint_positions --------------------


_UR5_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def test_extract_positions_happy_path():
    payload = {
        "name": list(_UR5_JOINTS),
        "position": [0.1, -1.5, 0.0, -1.5, 0.0, 0.0],
    }
    positions, missing = extract_ordered_joint_positions(payload, _UR5_JOINTS)
    assert missing is None
    assert positions == [0.1, -1.5, 0.0, -1.5, 0.0, 0.0]


def test_extract_positions_reorders_when_input_unordered():
    """El joint_state puede llegar con joints en orden distinto."""
    payload = {
        "name": ["wrist_3_joint", "shoulder_pan_joint", "elbow_joint",
                 "wrist_2_joint", "wrist_1_joint", "shoulder_lift_joint"],
        "position": [99.0, 0.1, 33.0, 55.0, 44.0, -1.5],
    }
    positions, missing = extract_ordered_joint_positions(payload, _UR5_JOINTS)
    assert missing is None
    # Esperado en orden canónico
    assert positions == [0.1, -1.5, 33.0, 44.0, 55.0, 99.0]


def test_extract_positions_returns_missing_joint_name():
    payload = {
        "name": ["shoulder_pan_joint", "elbow_joint", "wrist_1_joint",
                 "wrist_2_joint", "wrist_3_joint"],  # falta shoulder_lift
        "position": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
    positions, missing = extract_ordered_joint_positions(payload, _UR5_JOINTS)
    assert positions is None
    assert missing == "shoulder_lift_joint"


def test_extract_positions_returns_first_missing_in_order():
    """Si faltan varios, devuelve el primero según el orden canónico."""
    payload = {
        "name": ["elbow_joint"],
        "position": [0.0],
    }
    positions, missing = extract_ordered_joint_positions(payload, _UR5_JOINTS)
    assert positions is None
    assert missing == "shoulder_pan_joint"  # primer canónico ausente


def test_extract_positions_none_payload():
    positions, missing = extract_ordered_joint_positions(None, _UR5_JOINTS)
    assert positions is None
    assert missing == "no_payload"


def test_extract_positions_empty_payload():
    payload = {"name": [], "position": []}
    positions, missing = extract_ordered_joint_positions(payload, _UR5_JOINTS)
    assert positions is None
    assert missing == "no_payload"


def test_extract_positions_missing_position_key():
    payload = {"name": list(_UR5_JOINTS)}
    positions, missing = extract_ordered_joint_positions(payload, _UR5_JOINTS)
    assert positions is None
    assert missing == "no_payload"


# -------------------- select_traj_duration_and_timeout -------------------


def test_select_long_distance_uses_long_minima():
    """Distancia > 0.4m (default) usa duration=25s, timeout=120s mínimos."""
    duration, timeout, is_long = select_traj_duration_and_timeout(
        dist_m=0.5,
        default_duration_sec=10.0,
        default_timeout_sec=60.0,
    )
    assert is_long is True
    assert duration == 25.0  # 25 > 10
    assert timeout == 120.0  # 120 > 60


def test_select_short_distance_uses_short_minima():
    duration, timeout, is_long = select_traj_duration_and_timeout(
        dist_m=0.1,
        default_duration_sec=5.0,
        default_timeout_sec=20.0,
    )
    assert is_long is False
    assert duration == 8.0  # 8 > 5
    assert timeout == 30.0  # 30 > 20


def test_select_default_higher_than_minimum_wins():
    """Si default es mayor que el mínimo, se usa default."""
    duration, timeout, is_long = select_traj_duration_and_timeout(
        dist_m=0.5,
        default_duration_sec=40.0,  # > 25 long_min
        default_timeout_sec=200.0,  # > 120 long_min
    )
    assert duration == 40.0
    assert timeout == 200.0


def test_select_at_threshold_is_short():
    """dist == 0.4 (threshold) NO supera el límite → corta."""
    _, _, is_long = select_traj_duration_and_timeout(
        dist_m=0.4,
        default_duration_sec=0.0,
        default_timeout_sec=0.0,
    )
    assert is_long is False


def test_select_just_above_threshold_is_long():
    _, _, is_long = select_traj_duration_and_timeout(
        dist_m=0.401,
        default_duration_sec=0.0,
        default_timeout_sec=0.0,
    )
    assert is_long is True


def test_select_custom_threshold():
    """Se puede sobreescribir el threshold."""
    _, _, is_long = select_traj_duration_and_timeout(
        dist_m=0.5,
        default_duration_sec=0.0,
        default_timeout_sec=0.0,
        long_dist_threshold_m=1.0,
    )
    assert is_long is False


def test_select_custom_minima():
    duration, timeout, is_long = select_traj_duration_and_timeout(
        dist_m=2.0,
        default_duration_sec=10.0,
        default_timeout_sec=10.0,
        long_min_duration_sec=50.0,
        long_min_timeout_sec=300.0,
    )
    assert is_long is True
    assert duration == 50.0
    assert timeout == 300.0


# -------------------- parse_plan_to_pose_request -------------------------


def _make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


def test_parse_request_full_fields():
    req = SimpleNamespace(
        target_pose_base=_make_pose(0.5, -0.3, 0.05, qz=0.7, qw=0.7),
        ee_frame="rg2_pinch_center",
        cartesian=True,
        timeout_sec=5.5,
    )
    xyz, quat, ee, cart, tout = parse_plan_to_pose_request(req)
    assert xyz == (0.5, -0.3, 0.05)
    assert quat == (0.0, 0.0, 0.7, 0.7)
    assert ee == "rg2_pinch_center"
    assert cart is True
    assert tout == 5.5


def test_parse_request_empty_ee_frame_uses_default():
    req = SimpleNamespace(
        target_pose_base=_make_pose(0.0, 0.0, 0.0),
        ee_frame="",
        cartesian=False,
        timeout_sec=0.0,
    )
    _xyz, _quat, ee, _c, _t = parse_plan_to_pose_request(req)
    assert ee == "rg2_pinch_center"


def test_parse_request_custom_default_ee_frame():
    req = SimpleNamespace(
        target_pose_base=_make_pose(0.0, 0.0, 0.0),
    )
    _xyz, _quat, ee, _c, _t = parse_plan_to_pose_request(
        req, default_ee_frame="tool0"
    )
    assert ee == "tool0"


def test_parse_request_missing_optional_fields():
    """Sin ee_frame/cartesian/timeout_sec → usa defaults."""
    req = SimpleNamespace(
        target_pose_base=_make_pose(1.0, 2.0, 3.0),
    )
    xyz, quat, ee, cart, tout = parse_plan_to_pose_request(req)
    assert xyz == (1.0, 2.0, 3.0)
    assert quat == (0.0, 0.0, 0.0, 1.0)
    assert ee == "rg2_pinch_center"
    assert cart is False
    assert tout == 0.0


def test_parse_request_strips_ee_frame_whitespace():
    req = SimpleNamespace(
        target_pose_base=_make_pose(0.0, 0.0, 0.0),
        ee_frame="  rg2_tcp  ",
    )
    _xyz, _quat, ee, _c, _t = parse_plan_to_pose_request(req)
    assert ee == "rg2_tcp"
