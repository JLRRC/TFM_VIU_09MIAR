"""F4: cobertura de tfm_grasping.geometry.Grasp2D.

Tests puros (stdlib + numpy opcional). Valida:
  - Construcción y atributos.
  - Serialización ``to_dict`` round-trip.
  - Inmutabilidad (frozen dataclass).
  - Defaults para campos opcionales.
"""

from __future__ import annotations

import math

import pytest

from tfm_grasping.geometry import Grasp2D


def _g(**overrides):
    base = dict(
        center_x=160.0,
        center_y=120.0,
        angle_rad=math.radians(45.0),
        width_px=64.0,
        quality=0.85,
    )
    base.update(overrides)
    return Grasp2D(**base)


def test_grasp2d_required_attributes():
    g = _g()
    assert g.center_x == 160.0
    assert g.center_y == 120.0
    assert math.isclose(g.angle_rad, math.radians(45.0), abs_tol=1e-9)
    assert g.width_px == 64.0
    assert g.quality == 0.85


def test_grasp2d_optional_defaults():
    g = _g()
    assert g.height_px is None
    assert g.depth_m is None
    assert g.frame_id == ""


def test_grasp2d_optional_overrides():
    g = _g(height_px=32.0, depth_m=0.45, frame_id="camera_overhead")
    assert g.height_px == 32.0
    assert g.depth_m == 0.45
    assert g.frame_id == "camera_overhead"


def test_grasp2d_to_dict_contains_all_keys():
    g = _g(height_px=32.0, depth_m=0.45, frame_id="camera_overhead")
    d = g.to_dict()
    assert set(d.keys()) == {
        "center_x",
        "center_y",
        "angle_rad",
        "width_px",
        "height_px",
        "quality",
        "depth_m",
        "frame_id",
    }


def test_grasp2d_to_dict_roundtrip_values():
    g = _g(height_px=32.0, depth_m=0.45, frame_id="cam")
    d = g.to_dict()
    assert d["center_x"] == 160.0
    assert d["center_y"] == 120.0
    assert math.isclose(d["angle_rad"], math.radians(45.0), abs_tol=1e-9)
    assert d["width_px"] == 64.0
    assert d["height_px"] == 32.0
    assert d["quality"] == 0.85
    assert d["depth_m"] == 0.45
    assert d["frame_id"] == "cam"


def test_grasp2d_to_dict_handles_none_optionals():
    g = _g()
    d = g.to_dict()
    assert d["height_px"] is None
    assert d["depth_m"] is None


def test_grasp2d_is_frozen():
    g = _g()
    with pytest.raises((AttributeError, Exception)):
        g.center_x = 999.0


def test_grasp2d_equality_by_value():
    g1 = _g()
    g2 = _g()
    assert g1 == g2


def test_grasp2d_inequality_when_value_differs():
    g1 = _g()
    g2 = _g(quality=0.99)
    assert g1 != g2


def test_grasp2d_hashable():
    g = _g()
    s = {g, _g()}
    assert len(s) == 1
