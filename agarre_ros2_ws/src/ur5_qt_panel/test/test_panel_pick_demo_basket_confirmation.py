#!/usr/bin/env python3
"""Pruebas de confirmación física de cesta para DIRECTO."""

from __future__ import annotations

from types import SimpleNamespace

from ur5_qt_panel.panel_objects import ObjectOwner
from ur5_qt_panel import panel_pick_demo as pick_demo


class _FakePanel:
    def __init__(self, *, release_reference_world=None):
        self._pick_demo_release_reference_world = release_reference_world
        self._logs = []
        self.ros_worker = None
        self._ros_worker_started = False

    def _emit_log(self, line: str) -> None:
        self._logs.append(line)

    def _world_frame_last_first(self, fallback=None):
        return fallback or "world"

    def _business_base_frame(self):
        return "base_link"


def _freeze_monotonic(monkeypatch, values):
    ticks = iter(values)
    monkeypatch.setattr(pick_demo.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(pick_demo.time, "sleep", lambda _sec: None)


def test_demo_object_in_basket_requires_real_basket_proximity(monkeypatch) -> None:
    panel = _FakePanel(release_reference_world=(-1.058, -0.072, 0.714))
    final_obj_world = (-1.055, -0.072, 0.025)
    state = SimpleNamespace(
        attached=False,
        owner=ObjectOwner.NONE,
        position=final_obj_world,
    )

    monkeypatch.setattr(pick_demo, "get_object_state", lambda _name: state)
    monkeypatch.setattr(
        pick_demo,
        "transform_point_to_frame",
        lambda point, *_args, **_kwargs: (tuple(float(v) for v in point), None),
    )
    _freeze_monotonic(monkeypatch, [0.0, 0.0, 0.5])

    assert pick_demo._demo_object_in_basket(panel, timeout_sec=0.1) is False
    assert not any("confirmacion cesta OK" in line for line in panel._logs)
    assert any("basket_confirmation_release_only" in line for line in panel._logs)
    assert any("basket_confirmation_timeout" in line for line in panel._logs)


def test_demo_object_in_basket_accepts_object_inside_basket(monkeypatch) -> None:
    panel = _FakePanel()
    basket_world = tuple(float(v) for v in pick_demo.BASKET_DROP)
    state = SimpleNamespace(
        attached=False,
        owner=ObjectOwner.NONE,
        position=basket_world,
    )

    monkeypatch.setattr(pick_demo, "get_object_state", lambda _name: state)
    monkeypatch.setattr(
        pick_demo,
        "transform_point_to_frame",
        lambda point, *_args, **_kwargs: (tuple(float(v) for v in point), None),
    )
    monkeypatch.setattr(pick_demo.time, "monotonic", lambda: 0.0)
    monkeypatch.setattr(pick_demo.time, "sleep", lambda _sec: None)

    assert pick_demo._demo_object_in_basket(panel, timeout_sec=0.1) is True
    assert any("confirmacion cesta OK" in line for line in panel._logs)
