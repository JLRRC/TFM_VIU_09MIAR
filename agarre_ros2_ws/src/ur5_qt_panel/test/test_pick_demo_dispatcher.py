#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_demo_dispatcher.py
# Contenido: F6.4 final — tests offline del dispatcher pick demo.
"""Tests offline de ``ur5_qt_panel.pick_demo_dispatcher``.

NO requieren ROS — usan mocks de panel + legacy + monkey-patch para
simular las distintas ramas del dispatcher.
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock

import pytest


# --------------------------------------------------------------------------
# Stub legacy que se inyecta en cada test
# --------------------------------------------------------------------------


def _make_legacy_stub():
    """Devuelve un MagicMock que actúa como run_pick_demo(panel)."""
    return MagicMock(name="legacy_run_pick_demo")


# --------------------------------------------------------------------------
# Stub panel
# --------------------------------------------------------------------------


def _make_panel(*, with_node: bool = True, selected: str = "box_red",
                drop=(0.5, 0.0, 0.05)):
    panel = MagicMock(name="panel")
    panel._selected_object = selected
    panel._basket_drop_world = drop
    panel._emit_log = MagicMock()
    if with_node:
        node = MagicMock(name="node")
        node.get_logger = MagicMock(return_value=MagicMock())
        worker = MagicMock(name="ros_worker")
        worker._node = node
        panel.ros_worker = worker
        panel._moveit_node = None
        panel._node = None
    else:
        panel.ros_worker = MagicMock(spec=[])
        panel._moveit_node = None
        panel._node = None
    return panel


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------


def test_no_env_default_is_legacy_20260507(monkeypatch):
    """2026-05-07: default revertido a legacy (era orchestrator en F12).

    Razón: bug bridge MoveIt-controller (sim_time vs wall_time) bloquea el
    orchestrator en Gazebo Sim. El legacy run_pick_demo está validado live
    (objetivo-cumplido-pinzas-agarran-objeto-20260507).
    """
    monkeypatch.delenv("PANEL_PICK_DEMO_USE_ORCHESTRATOR", raising=False)
    monkeypatch.delenv("USE_LEGACY_PICK_DEMO", raising=False)

    from ur5_qt_panel.pick_demo_dispatcher import dispatch_pick_demo

    legacy = _make_legacy_stub()
    panel = _make_panel()
    mode = dispatch_pick_demo(panel, legacy=legacy)
    assert mode == "legacy"
    legacy.assert_called_once_with(panel)


def test_legacy_env_forces_legacy_f12():
    """F12: USE_LEGACY_PICK_DEMO=1 fuerza legacy pese al default orchestrator."""
    from ur5_qt_panel.pick_demo_dispatcher import dispatch_pick_demo
    legacy = _make_legacy_stub()
    panel = _make_panel()
    mode = dispatch_pick_demo(panel, legacy_env_value="1", legacy=legacy)
    assert mode == "legacy"
    legacy.assert_called_once_with(panel)


def test_flag_explicit_off_calls_legacy():
    """PANEL_PICK_DEMO_USE_ORCHESTRATOR=0 fuerza legacy."""
    from ur5_qt_panel.pick_demo_dispatcher import dispatch_pick_demo
    legacy = _make_legacy_stub()
    panel = _make_panel()
    mode = dispatch_pick_demo(panel, env_value="0", legacy=legacy)
    assert mode == "legacy"
    legacy.assert_called_once_with(panel)


@pytest.mark.parametrize("v", ["1", "true", "yes", "on"])
def test_flag_on_falls_back_to_legacy_when_no_rclpy(v, monkeypatch):
    """Sin rclpy disponible, el dispatcher cae al legacy con log."""
    # Simular rclpy_available() → False inyectando un módulo dummy.
    fake_client = types.ModuleType("ur5_qt_panel.pick_place_client")
    fake_client.rclpy_available = lambda: False
    fake_client.PickPlaceClient = MagicMock()
    monkeypatch.setitem(sys.modules, "ur5_qt_panel.pick_place_client", fake_client)

    from ur5_qt_panel.pick_demo_dispatcher import dispatch_pick_demo
    legacy = _make_legacy_stub()
    panel = _make_panel()
    mode = dispatch_pick_demo(panel, env_value=v, legacy=legacy)
    assert mode == "orchestrator_fallback_no_rclpy"
    legacy.assert_called_once_with(panel)
    panel._emit_log.assert_called()


def test_flag_on_no_node_falls_back_to_legacy(monkeypatch):
    fake_client = types.ModuleType("ur5_qt_panel.pick_place_client")
    fake_client.rclpy_available = lambda: True
    fake_client.PickPlaceClient = MagicMock()
    monkeypatch.setitem(sys.modules, "ur5_qt_panel.pick_place_client", fake_client)

    from ur5_qt_panel.pick_demo_dispatcher import dispatch_pick_demo
    legacy = _make_legacy_stub()
    panel = _make_panel(with_node=False)
    mode = dispatch_pick_demo(panel, env_value="1", legacy=legacy)
    assert mode == "orchestrator_fallback_no_node"
    legacy.assert_called_once_with(panel)


def test_flag_on_no_object_falls_back_to_legacy(monkeypatch):
    fake_client = types.ModuleType("ur5_qt_panel.pick_place_client")
    fake_client.rclpy_available = lambda: True
    fake_client.PickPlaceClient = MagicMock()
    monkeypatch.setitem(sys.modules, "ur5_qt_panel.pick_place_client", fake_client)

    from ur5_qt_panel.pick_demo_dispatcher import dispatch_pick_demo
    legacy = _make_legacy_stub()
    panel = _make_panel(selected="")  # sin objeto
    mode = dispatch_pick_demo(panel, env_value="1", legacy=legacy)
    assert mode == "orchestrator_fallback_no_object"
    legacy.assert_called_once_with(panel)


def test_flag_on_server_unavailable_falls_back(monkeypatch):
    fake_client_mod = types.ModuleType("ur5_qt_panel.pick_place_client")
    fake_client_mod.rclpy_available = lambda: True

    client_instance = MagicMock(name="PickPlaceClient_instance")
    client_instance.wait_for_server = MagicMock(return_value=False)
    fake_client_mod.PickPlaceClient = MagicMock(return_value=client_instance)
    monkeypatch.setitem(sys.modules, "ur5_qt_panel.pick_place_client", fake_client_mod)

    from ur5_qt_panel.pick_demo_dispatcher import dispatch_pick_demo
    legacy = _make_legacy_stub()
    panel = _make_panel()
    mode = dispatch_pick_demo(panel, env_value="1", legacy=legacy)
    assert mode == "orchestrator_fallback_no_server"
    legacy.assert_called_once_with(panel)


def test_flag_on_server_available_sends_goal(monkeypatch):
    fake_client_mod = types.ModuleType("ur5_qt_panel.pick_place_client")
    fake_client_mod.rclpy_available = lambda: True

    client_instance = MagicMock(name="PickPlaceClient_instance")
    client_instance.wait_for_server = MagicMock(return_value=True)
    client_instance.send_goal = MagicMock(return_value=True)
    fake_client_mod.PickPlaceClient = MagicMock(return_value=client_instance)
    monkeypatch.setitem(sys.modules, "ur5_qt_panel.pick_place_client", fake_client_mod)

    from ur5_qt_panel.pick_demo_dispatcher import dispatch_pick_demo
    legacy = _make_legacy_stub()
    panel = _make_panel()
    mode = dispatch_pick_demo(panel, env_value="1", legacy=legacy)
    assert mode == "orchestrator"
    # Legacy NO se llama cuando todo va bien.
    legacy.assert_not_called()
    # send_goal se llamó con el object_name + drop_xyz del panel.
    client_instance.send_goal.assert_called_once()
    args, kwargs = client_instance.send_goal.call_args
    assert args[0] == "box_red"
    assert args[1] == (0.5, 0.0, 0.05)
    assert "on_feedback" in kwargs
    assert "on_done" in kwargs


def test_resolve_panel_node_prefers_ros_worker_node():
    from ur5_qt_panel.pick_demo_dispatcher import _resolve_panel_node
    panel = _make_panel()
    n = _resolve_panel_node(panel)
    assert n is not None
    assert n is panel.ros_worker._node


def test_resolve_panel_node_returns_none_if_no_logger():
    from ur5_qt_panel.pick_demo_dispatcher import _resolve_panel_node
    panel = MagicMock()
    panel.ros_worker = MagicMock(spec=[])
    panel._moveit_node = None
    panel._node = None
    n = _resolve_panel_node(panel)
    assert n is None


def test_drop_xyz_from_panel_uses_basket_drop():
    from ur5_qt_panel.pick_demo_dispatcher import _drop_xyz_from_panel
    panel = MagicMock()
    panel._basket_drop_world = (1.0, 2.0, 3.0)
    assert _drop_xyz_from_panel(panel) == (1.0, 2.0, 3.0)


def test_drop_xyz_from_panel_default_when_missing():
    """2026-05-07: default cambiado de (0.5,0,0.05) a (-1.30,0,1.10) —
    pose alcanzable en world frame para que TRANSPORT no caiga fuera del
    workspace al convertir world→base_link."""
    from ur5_qt_panel.pick_demo_dispatcher import _drop_xyz_from_panel
    panel = MagicMock()
    panel._basket_drop_world = None
    xyz = _drop_xyz_from_panel(panel)
    assert xyz == (-1.30, 0.0, 1.10)


def test_drop_xyz_from_panel_default_when_invalid():
    """2026-05-07: ver test_drop_xyz_from_panel_default_when_missing."""
    from ur5_qt_panel.pick_demo_dispatcher import _drop_xyz_from_panel
    panel = MagicMock()
    panel._basket_drop_world = "garbage"
    xyz = _drop_xyz_from_panel(panel)
    assert xyz == (-1.30, 0.0, 1.10)


# ---------------------------------------------------------------------------
# F5-step2: _object_pose_world_from_panel
# ---------------------------------------------------------------------------


def test_object_pose_world_from_callable_attribute():
    from ur5_qt_panel.pick_demo_dispatcher import _object_pose_world_from_panel

    class _P:
        def get_object_pose_world(self, name):
            return (0.4, 0.1, 0.03) if name == "box_red" else None

    panel = _P()
    assert _object_pose_world_from_panel(panel, "box_red") == (0.4, 0.1, 0.03)
    assert _object_pose_world_from_panel(panel, "missing") is None


def test_object_pose_world_from_callable_returns_tuple7():
    from ur5_qt_panel.pick_demo_dispatcher import _object_pose_world_from_panel

    class _P:
        def get_object_pose_world(self, name):
            return (0.4, 0.1, 0.03, 0.0, 0.0, 0.7071, 0.7071)

    pose = _object_pose_world_from_panel(_P(), "box")
    assert pose == (0.4, 0.1, 0.03, 0.0, 0.0, 0.7071, 0.7071)


def test_object_pose_world_returns_none_for_empty_name():
    from ur5_qt_panel.pick_demo_dispatcher import _object_pose_world_from_panel

    panel = MagicMock()
    assert _object_pose_world_from_panel(panel, "") is None
    assert _object_pose_world_from_panel(panel, "   ") is None
    assert _object_pose_world_from_panel(panel, None) is None


def test_object_pose_world_from_dict_fallback():
    from ur5_qt_panel.pick_demo_dispatcher import _object_pose_world_from_panel

    class _P:
        # Sin get_object_pose_world callable; sólo dict.
        _object_positions = {"box_red": (0.5, -0.2, 0.05)}

    pose = _object_pose_world_from_panel(_P(), "box_red")
    assert pose == (0.5, -0.2, 0.05)


def test_object_pose_world_returns_none_for_invalid_dict_value():
    from ur5_qt_panel.pick_demo_dispatcher import _object_pose_world_from_panel

    class _P:
        _object_positions = {"box": "garbage"}

    pose = _object_pose_world_from_panel(_P(), "box")
    assert pose is None


def test_object_pose_world_callable_exception_falls_back_to_dict():
    from ur5_qt_panel.pick_demo_dispatcher import _object_pose_world_from_panel

    class _P:
        _object_positions = {"box": (0.6, 0.2, 0.04)}

        def get_object_pose_world(self, name):
            raise RuntimeError("simulated")

    pose = _object_pose_world_from_panel(_P(), "box")
    assert pose == (0.6, 0.2, 0.04)
