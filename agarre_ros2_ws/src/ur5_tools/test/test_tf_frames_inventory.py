#!/usr/bin/env python3
"""F4 audit-v4 (2026-05-08): TF frames inventory test (live).

Verifica que los frames críticos del pick están publicados en TF cuando
el stack está corriendo. Skip cleanly si no hay ROS context (offline).

Frames críticos:
- world / base_link (root chain, post fix world_tf_publisher)
- tool0 (UR5 end of links chain)
- rg2_pinch_center (TCP semántico, post unification 0.0050885 m)
- rg2_tcp (TCP geométrico)
- rg2_finger_link1 / rg2_finger_link2 (gripper fingers)
- camera frames (post-arranque cámaras)
- pick_demo_anchor (drop anchor durante release)

Uso:
    # Con stack vivo (panel/ur5_stack lanzado):
    PICK_E2E_LIVE=1 pytest test_tf_frames_inventory.py -v

    # Offline (skip):
    pytest test_tf_frames_inventory.py -v
"""
from __future__ import annotations

import os
from typing import List, Optional, Set

import pytest

REQUIRED_FRAMES_AT_REST: List[str] = [
    "world",
    "base_link",
    "tool0",
    "rg2_pinch_center",
    "rg2_tcp",
    "rg2_finger_link1",
    "rg2_finger_link2",
    "rg2_base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
]

OPTIONAL_FRAMES = [
    # Aparecen post-pick:
    "pick_demo_anchor",
    # Aparecen si cámaras están conectadas:
    "camera_color_optical_frame",
    "camera_depth_optical_frame",
]


def _live_only() -> None:
    """Skip si no hay stack vivo."""
    if not os.environ.get("PICK_E2E_LIVE"):
        pytest.skip(
            "PICK_E2E_LIVE not set; this test requires the stack running. "
            "Run with: PICK_E2E_LIVE=1 pytest <file>"
        )
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pytest.skip("rclpy not available (test requires ROS 2 environment)")


def _list_tf_frames(timeout_sec: float = 5.0) -> Set[str]:
    """Devuelve el conjunto de frames publicados en TF.

    Crea un nodo temporal, suscribe a /tf_static + /tf, espera hasta
    timeout_sec, retorna nombres únicos de frame visto.
    """
    import rclpy
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage  # type: ignore[import-not-found]

    if not rclpy.ok():
        rclpy.init()

    frames: Set[str] = set()

    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )

    static_qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    class _Probe(Node):
        def __init__(self) -> None:
            super().__init__("tf_frames_inventory_probe")
            self.create_subscription(TFMessage, "/tf_static", self._cb, static_qos)
            self.create_subscription(TFMessage, "/tf", self._cb, 100)

        def _cb(self, msg: TFMessage) -> None:
            for t in msg.transforms:
                frames.add(str(t.header.frame_id))
                frames.add(str(t.child_frame_id))

    node = _Probe()
    try:
        import time as _time
        deadline = _time.monotonic() + timeout_sec
        while _time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
    return frames


@pytest.mark.parametrize("frame", REQUIRED_FRAMES_AT_REST)
def test_required_tf_frame_published_live(frame: str) -> None:
    """Cada frame crítico debe estar en /tf_static o /tf cuando el stack
    está vivo y a reposo (sin pick activo)."""
    _live_only()
    visible = _list_tf_frames(timeout_sec=5.0)
    assert frame in visible, (
        f"frame {frame!r} not seen in /tf or /tf_static after 5s. "
        f"Visible: {sorted(visible)[:20]}..."
    )


def test_world_to_base_link_published_live() -> None:
    """world → base_link es la raíz de la cadena cinemática."""
    _live_only()
    visible = _list_tf_frames(timeout_sec=5.0)
    assert "world" in visible and "base_link" in visible, (
        "world or base_link missing from TF tree. Check world_tf_publisher."
    )


def test_no_orphan_critical_frames_live() -> None:
    """Skip-friendly: ningún frame crítico aparece sin su parent."""
    _live_only()
    # Heuristic: si rg2_pinch_center está, también debe estar rg2_base_link.
    visible = _list_tf_frames(timeout_sec=5.0)
    if "rg2_pinch_center" in visible:
        assert "rg2_base_link" in visible, (
            "rg2_pinch_center sin rg2_base_link → cadena RG2 rota"
        )
