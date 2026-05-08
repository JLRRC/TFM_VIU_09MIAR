#!/usr/bin/env python3
"""Audit-v4 (2026-05-08): snapshot del contrato de panel_ros.

Congela LOC + RosWorker public methods para que el split iter2 (deferred
v1.1, separar publishers/subscribers/clients) no rompa el panel.
"""
from __future__ import annotations

from pathlib import Path
from typing import Set

PKG = Path(__file__).resolve().parent.parent / "ur5_qt_panel"
MODULE_FILE = PKG / "panel_ros.py"


# Métodos públicos de RosWorker que panel_v2 / mixins usan.
EXPECTED_ROSWORKER_METHODS: Set[str] = {
    "start",
    "stop",
    "stop_and_join",
    "list_topic_names",
    "list_action_names",
    "topic_has_publishers",
    "topic_has_subscribers",
    "topic_subscriber_count",
    "topic_publisher_count",
    "node_name",
    "node_namespace",
    "publisher_nodes_by_topic",
    "topic_names_and_types",
    "service_names_and_types",
    "has_service",
    "image_frame_snapshot",
    "system_state_snapshot",
    "call_trigger_detail",
    "call_trigger",
    "get_latest_depth_frame",
    "subscribe_pose_info",
    "subscribe_moveit_result",
    "moveit_result_snapshot",
}


def test_file_exists() -> None:
    assert MODULE_FILE.is_file()


def test_loc_does_not_grow() -> None:
    BASELINE = 2158
    n = sum(1 for _ in MODULE_FILE.read_text(encoding="utf-8").splitlines())
    assert n <= BASELINE, f"panel_ros.py grew {BASELINE}→{n}"


def test_rosworker_class_present() -> None:
    text = MODULE_FILE.read_text(encoding="utf-8")
    assert "class RosWorker(QObject)" in text, "RosWorker class missing"


def test_expected_methods_present() -> None:
    text = MODULE_FILE.read_text(encoding="utf-8")
    import re
    methods: Set[str] = set()
    for m in re.finditer(r"^    def\s+(\w+)\s*\(", text, flags=re.MULTILINE):
        methods.add(m.group(1))
    missing = EXPECTED_ROSWORKER_METHODS - methods
    assert not missing, f"missing RosWorker methods: {missing}"
