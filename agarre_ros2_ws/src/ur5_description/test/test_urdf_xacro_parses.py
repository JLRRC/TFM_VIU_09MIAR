#!/usr/bin/env python3
"""F4 audit-v4 (2026-05-08): basic URDF/Xacro structural test.

Verifies:
  1. ur5.urdf.xacro file exists.
  2. Critical frames declared (link names) for the pick: world,
     base_link, tool0, rg2_pinch_center, rg2_tcp.
  3. Joints between key links exist.
  4. ur5_controllers.yaml is valid YAML with controller manager keys.

Without xacro processing (offline-friendly), uses lxml/regex on the
Xacro source. Comprehensive parse runs in T22 URDF↔SDF parity test.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

PKG_DIR = Path(__file__).resolve().parent.parent
XACRO_PATH = PKG_DIR / "urdf" / "ur5.urdf.xacro"
CONTROLLERS_YAML = PKG_DIR / "config" / "ur5_controllers.yaml"


def test_urdf_xacro_file_exists() -> None:
    assert XACRO_PATH.is_file(), f"ur5.urdf.xacro missing: {XACRO_PATH}"


def test_urdf_xacro_has_robot_root() -> None:
    """Source has the canonical xacro <robot> root tag."""
    txt = XACRO_PATH.read_text(encoding="utf-8")
    assert "<robot" in txt, "ur5.urdf.xacro missing <robot> root"
    assert 'name="' in txt, "robot root missing name attribute"


def _xacro_text() -> str:
    return XACRO_PATH.read_text(encoding="utf-8")


@pytest.mark.parametrize("link_name", [
    "world",
    "rg2_base_link",
    "rg2_pinch_center",
    "rg2_tcp",
    "rg2_finger_link1",
    "rg2_finger_link2",
])
def test_xacro_declares_critical_link(link_name: str) -> None:
    """Local links declared in this xacro (UR core links come from ur_macro)."""
    txt = _xacro_text()
    pattern = rf'<link\s+name="{re.escape(link_name)}"'
    assert re.search(pattern, txt), f"link {link_name!r} not declared in URDF"


@pytest.mark.parametrize("joint_name", [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "rg2_mount_joint",
    "rg2_finger_joint1",
    "rg2_finger_joint2",
    "rg2_tcp_joint",
    "rg2_pinch_center_joint",
])
def test_xacro_declares_joint(joint_name: str) -> None:
    """All UR5+RG2 joints (UR via ur_macro params, RG2 local)."""
    txt = _xacro_text()
    pattern = rf'<joint\s+name="{re.escape(joint_name)}"'
    assert re.search(pattern, txt), f"joint {joint_name!r} not declared in URDF"


def test_xacro_includes_ur_macro() -> None:
    """ur_description macro must be included to bring base_link/tool0/etc."""
    txt = _xacro_text()
    has_ur_macro = (
        "<xacro:ur_robot" in txt
        or "ur_description" in txt
        or "ur_macro" in txt
    )
    assert has_ur_macro, "ur_description macro not included in xacro"


def test_controllers_yaml_loads() -> None:
    import yaml
    assert CONTROLLERS_YAML.is_file(), f"controllers yaml missing: {CONTROLLERS_YAML}"
    with CONTROLLERS_YAML.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert isinstance(data, dict), "controllers yaml is not a mapping"
    assert data, "controllers yaml is empty"


def test_controllers_yaml_has_controller_manager() -> None:
    import yaml
    with CONTROLLERS_YAML.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert "controller_manager" in data, (
        "controller_manager root missing from ur5_controllers.yaml"
    )


@pytest.mark.parametrize("controller_name", [
    "joint_state_broadcaster",
    "joint_trajectory_controller",
])
def test_controllers_yaml_lists_canonical_controllers(controller_name: str) -> None:
    """Cada controller canónico debe aparecer al menos como sección."""
    txt = CONTROLLERS_YAML.read_text(encoding="utf-8")
    assert controller_name in txt, (
        f"controller {controller_name!r} not present in ur5_controllers.yaml"
    )
