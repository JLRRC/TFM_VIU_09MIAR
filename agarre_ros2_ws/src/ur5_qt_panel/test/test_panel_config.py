#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_config.py
# Contenido: Unit tests for panel_config constants — importable without ROS.
"""Sanity checks for panel_config module constants and types."""

import pytest

from ur5_qt_panel import panel_config as cfg


# ---------------------------------------------------------------------------
# Module-level import sanity
# ---------------------------------------------------------------------------

def test_panel_config_importable():
    assert cfg is not None


# ---------------------------------------------------------------------------
# Joint names
# ---------------------------------------------------------------------------

def test_ur5_joint_names_has_six_elements():
    assert len(cfg.UR5_JOINT_NAMES) == 6


def test_ur5_joint_names_are_strings():
    for name in cfg.UR5_JOINT_NAMES:
        assert isinstance(name, str) and name


def test_ur5_joint_names_contain_expected():
    names = list(cfg.UR5_JOINT_NAMES)
    expected = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"}
    assert expected <= set(names), f"Missing joints: {expected - set(names)}"


def test_gripper_joint_names_is_list():
    assert isinstance(cfg.GRIPPER_JOINT_NAMES, (list, tuple))


# ---------------------------------------------------------------------------
# Table geometry
# ---------------------------------------------------------------------------

def test_table_top_z_is_float():
    assert isinstance(cfg.TABLE_TOP_Z, (int, float))


def test_table_top_z_in_plausible_range():
    # Table top Z in Gazebo world should be between 0.5 m and 1.2 m
    assert 0.5 <= float(cfg.TABLE_TOP_Z) <= 1.2, f"TABLE_TOP_Z={cfg.TABLE_TOP_Z}"


def test_table_size_positive():
    assert float(cfg.TABLE_SIZE_X) > 0
    assert float(cfg.TABLE_SIZE_Y) > 0


def test_table_object_z_range_consistent():
    assert float(cfg.TABLE_OBJECT_Z_MIN) < float(cfg.TABLE_OBJECT_Z_MAX)


# ---------------------------------------------------------------------------
# Gripper parameters
# ---------------------------------------------------------------------------

def test_gripper_open_rad_is_float():
    assert isinstance(cfg.GRIPPER_OPEN_RAD, (int, float))


def test_gripper_closed_rad_is_float():
    assert isinstance(cfg.GRIPPER_CLOSED_RAD, (int, float))


def test_gripper_open_closed_ordered():
    # Open position must be >= closed position (prismatic: open=0.055, closed=0.0)
    assert float(cfg.GRIPPER_OPEN_RAD) >= float(cfg.GRIPPER_CLOSED_RAD)


def test_gripper_open_in_prismatic_range():
    # RG2 prismatic joint: [0, 0.055] m
    assert 0.0 <= float(cfg.GRIPPER_OPEN_RAD) <= 0.10, (
        f"GRIPPER_OPEN_RAD={cfg.GRIPPER_OPEN_RAD} outside [0, 0.10]"
    )


def test_gripper_closed_non_negative():
    assert float(cfg.GRIPPER_CLOSED_RAD) >= 0.0


def test_gripper_tcp_z_offset_is_float():
    assert isinstance(cfg.GRIPPER_TCP_Z_OFFSET, (int, float))


# ---------------------------------------------------------------------------
# Frame constants
# ---------------------------------------------------------------------------

def test_global_frame_effective_is_base_link():
    assert cfg.GLOBAL_FRAME_EFFECTIVE == "base_link"


def test_world_frame_is_string_or_none():
    # WORLD_FRAME comes from env and may be None when ROS env is absent
    assert cfg.WORLD_FRAME is None or (isinstance(cfg.WORLD_FRAME, str) and cfg.WORLD_FRAME)


# ---------------------------------------------------------------------------
# Timeout constants (must be positive)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("attr", [
    "MOVEIT_READY_TIMEOUT_SEC",
    "GZ_LAUNCH_TIMEOUT_SEC",
    "BRIDGE_LAUNCH_TIMEOUT_SEC",
    "CONTROLLER_READY_TIMEOUT_SEC",
    "PICK_TF_TIMEOUT_SEC",
    "CRITICAL_CLOCK_TIMEOUT_SEC",
    "CRITICAL_POSE_TIMEOUT_SEC",
    "SELECTION_TIMEOUT_SEC",
])
def test_timeout_is_positive(attr):
    val = getattr(cfg, attr)
    assert float(val) > 0, f"{attr}={val} must be positive"


# ---------------------------------------------------------------------------
# Pick demo geometry offsets (must be non-negative or plausible)
# ---------------------------------------------------------------------------

def test_pick_demo_pre_grasp_z_positive():
    assert float(cfg.PICK_DEMO_PRE_GRASP_Z_OFFSET) >= 0.0


def test_pick_demo_transport_z_positive():
    assert float(cfg.PICK_DEMO_TRANSPORT_Z_OFFSET) >= 0.0


# ---------------------------------------------------------------------------
# UR5 base position (must be numbers)
# ---------------------------------------------------------------------------

def test_ur5_base_position_are_floats():
    for attr in ("UR5_BASE_X", "UR5_BASE_Y", "UR5_BASE_Z"):
        val = getattr(cfg, attr)
        assert isinstance(val, (int, float)), f"{attr} must be float, got {type(val)}"


def test_ur5_reach_radius_positive():
    assert float(cfg.UR5_REACH_RADIUS) > 0.0


# ---------------------------------------------------------------------------
# Boolean flags are actual bools
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("attr", [
    "TABLE_IMAGE_SWAP_XY",
    "TABLE_IMAGE_FLIP_X",
    "TABLE_IMAGE_FLIP_Y",
    "USE_SIM_TIME",
    "PANEL_MANAGED",
])
def test_boolean_flag_type(attr):
    val = getattr(cfg, attr)
    assert isinstance(val, (bool, int)), f"{attr} should be bool-like, got {type(val)}"


# ---------------------------------------------------------------------------
# refresh_object_groups — unknown object branch (lines 117-121)
# ---------------------------------------------------------------------------

def test_refresh_object_groups_unknown_object_added_to_unknown_sets(monkeypatch):
    unknown_name = "__test_unknown_sentinel_xz9q__"
    monkeypatch.setitem(cfg.OBJECT_POSITIONS, unknown_name, (0.1, 0.2, 0.85))
    cfg._UNKNOWN_WARNED.discard(unknown_name)
    cfg.refresh_object_groups()
    assert unknown_name in cfg.UNKNOWN_OBJECTS
    assert unknown_name in cfg.UNKNOWN_NAME_SET
    assert unknown_name in cfg.UNKNOWN_OBJECT_NAMES
    cfg._UNKNOWN_WARNED.discard(unknown_name)
