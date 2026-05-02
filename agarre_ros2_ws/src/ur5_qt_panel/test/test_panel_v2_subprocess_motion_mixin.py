#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_subprocess_motion_mixin.py
# Contenido: F14-step10 (2026-05-01) — verifica el contrato del mixin subprocess+motion.
"""Tests offline F14-step10 — PanelV2SubprocessMotionMixin (64 wrappers)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_SRC_DIR = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

SAMPLE_METHODS = (
    "_nudge_drop_objects", "_start_release_service",
    "_start_world_tf_publisher", "_stop_world_tf_publisher",
    "_stop_gazebo", "_start_robot_state_publisher", "_start_bridge",
    "_stop_bridge", "_start_moveit", "_stop_moveit",
    "_wait_for_moveit_ready", "_kill_proc", "_proc_alive",
    "_rosbag_running", "_start_bag", "_stop_bag", "_refresh_status_sync",
    "_apply_status", "_moveit_topics_ready", "_moveit_action_ready",
    "_list_topic_names", "_topic_has_any_publishers",
    "_follow_joint_traj_ready", "_moveit_bridge_detected",
    "_move_group_ready", "_moveit_ready", "_update_system_stats",
    "_known_process_pids", "_list_stale_processes",
    "_detect_stale_processes", "_refresh_controls",
    "_maybe_auto_run_pick_demo", "_wait_for_state_change",
    "_apply_home_joint2_offset", "_get_home_joint_pose",
    "_set_test_failed", "_set_panel_flow_state",
    "_get_gripper_force", "_wait_for_joint_convergence",
    "_run_baseline_motion", "_go_home",
    "_compute_reach_overlay_points", "_send_joints",
    "_send_joints_retry",
)

LEGACY_NO_PROPAGATE = ("_stop_gazebo", "_go_home", "_send_joints")


def _read(name: str) -> str:
    return (_SRC_DIR / name).read_text(encoding="utf-8")


def test_mixin_module_exists():
    src = _read("panel_v2_subprocess_motion_mixin.py")
    assert "class PanelV2SubprocessMotionMixin" in src


def test_mixin_imports_both_aliases():
    src = _read("panel_v2_subprocess_motion_mixin.py")
    assert "import panel_status_mgmt as _sm" in src
    assert "import panel_motion_control as _mc" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_mixin_defines_method(method):
    src = _read("panel_v2_subprocess_motion_mixin.py")
    assert re.search(rf"def\s+{re.escape(method)}\s*\(", src)


def test_panel_v2_inherits_from_mixin():
    src = _read("panel_v2.py")
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "ControlPanelV2":
            base_names = []
            for b in node.bases:
                if isinstance(b, ast.Name):
                    base_names.append(b.id)
                elif isinstance(b, ast.Attribute):
                    base_names.append(b.attr)
            assert "PanelV2SubprocessMotionMixin" in base_names
            return
    pytest.fail("clase ControlPanelV2 no encontrada")


def test_panel_v2_imports_mixin():
    src = _read("panel_v2.py")
    assert "PanelV2SubprocessMotionMixin" in src
    assert "panel_v2_subprocess_motion_mixin" in src


@pytest.mark.parametrize("method", SAMPLE_METHODS)
def test_panel_v2_no_longer_defines_method_locally(method):
    src = _read("panel_v2.py")
    matches = re.findall(
        rf"^\s{{4}}def\s+{re.escape(method)}\s*\(", src, flags=re.MULTILINE
    )
    assert not matches, f"panel_v2.py todavía define '{method}'"


@pytest.mark.parametrize("method", LEGACY_NO_PROPAGATE)
def test_legacy_signature_preserved(method):
    src = _read("panel_v2_subprocess_motion_mixin.py")
    pat = rf"return\s+_(?:sm|mc)\.{re.escape(method)}\(self\)\s*$"
    assert re.search(pat, src, flags=re.MULTILINE), (
        f"{method} debe llamar al backend con (self) sin propagar"
    )


def test_panel_v2_under_1900_lines():
    src = _read("panel_v2.py")
    line_count = src.count("\n") + (0 if src.endswith("\n") else 1)
    assert line_count <= 1900, (
        f"panel_v2.py creció a {line_count} LOC; umbral F14-step10 <=1900"
    )
