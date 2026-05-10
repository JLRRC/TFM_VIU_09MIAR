#!/usr/bin/env python3
"""F8 (auditoría 2026-05-10): tests offline de release_objects_geometry."""
from __future__ import annotations

import math
import textwrap
from pathlib import Path

import pytest

from ur5_tools.release_objects_geometry import (
    is_pose_on_table,
    parse_table_geometry_from_sdf,
    pose_tuple_from_text,
    quat_from_rpy,
)


# ----------------------------- quat_from_rpy --------------------------------
def test_quat_from_rpy_identity():
    qx, qy, qz, qw = quat_from_rpy(0.0, 0.0, 0.0)
    assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quat_from_rpy_pi_x_axis():
    """180° around X: (1,0,0,0)."""
    qx, qy, qz, qw = quat_from_rpy(math.pi, 0.0, 0.0)
    assert qx == pytest.approx(1.0)
    assert qy == pytest.approx(0.0, abs=1e-9)
    assert qz == pytest.approx(0.0, abs=1e-9)
    assert qw == pytest.approx(0.0, abs=1e-9)


def test_quat_from_rpy_pi_z_axis():
    """180° around Z: (0,0,1,0)."""
    qx, qy, qz, qw = quat_from_rpy(0.0, 0.0, math.pi)
    assert qx == pytest.approx(0.0, abs=1e-9)
    assert qy == pytest.approx(0.0, abs=1e-9)
    assert qz == pytest.approx(1.0)
    assert qw == pytest.approx(0.0, abs=1e-9)


def test_quat_from_rpy_unit_norm():
    """El cuaternión debe ser unitario para cualquier RPY."""
    for r, p, y in [(0.5, -0.3, 0.7), (1.2, 2.1, -1.5)]:
        qx, qy, qz, qw = quat_from_rpy(r, p, y)
        norm2 = qx * qx + qy * qy + qz * qz + qw * qw
        assert norm2 == pytest.approx(1.0, abs=1e-9)


# ----------------------------- pose_tuple_from_text -------------------------
def test_pose_tuple_from_text_full():
    # yaw exacto π/2 (en lugar del aproximado 1.5708 del SDF típico).
    out = pose_tuple_from_text(f"0.5 -0.3 0.05 0 0 {math.pi / 2}")
    assert out[:3] == pytest.approx((0.5, -0.3, 0.05))
    # yaw=π/2 → qz=sin(π/4), qw=cos(π/4)
    assert out[5] == pytest.approx(math.sin(math.pi / 4))
    assert out[6] == pytest.approx(math.cos(math.pi / 4))


def test_pose_tuple_from_text_pads_missing():
    """Faltan rpy → quat identidad."""
    out = pose_tuple_from_text("1.0 2.0 3.0")
    assert out == pytest.approx((1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))


def test_pose_tuple_from_text_empty_returns_zeros():
    out = pose_tuple_from_text("")
    assert out == pytest.approx((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))


def test_pose_tuple_from_text_extra_ignored():
    out = pose_tuple_from_text("1 2 3 0 0 0 999 888")
    assert out == pytest.approx((1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))


# ----------------------------- is_pose_on_table -----------------------------
def test_is_pose_on_table_center():
    geom = (0.0, 0.0, 1.0, 1.0, 0.5)  # cuadrado 1×1 centrado en origen, top z=0.5
    assert is_pose_on_table((0.0, 0.0, 0.55), geom)


def test_is_pose_on_table_outside_xy():
    geom = (0.0, 0.0, 1.0, 1.0, 0.5)
    # x fuera por mucho
    assert not is_pose_on_table((5.0, 0.0, 0.55), geom)


def test_is_pose_on_table_within_margin():
    geom = (0.0, 0.0, 1.0, 1.0, 0.5)
    # x = half_x + margin_default (0.09) → en el límite
    assert is_pose_on_table((0.59, 0.0, 0.55), geom)


def test_is_pose_on_table_below_surface():
    geom = (0.0, 0.0, 1.0, 1.0, 0.5)
    # z por debajo de table_z → falso
    assert not is_pose_on_table((0.0, 0.0, 0.4), geom)


def test_is_pose_on_table_too_high():
    geom = (0.0, 0.0, 1.0, 1.0, 0.5)
    # z = table_z + 0.20 (max default 0.08) → falso
    assert not is_pose_on_table((0.0, 0.0, 0.7), geom)


# ----------------------------- parse_table_geometry_from_sdf ----------------
def test_parse_table_geometry_returns_none_if_path_missing():
    assert parse_table_geometry_from_sdf("/nonexistent/path.sdf") is None


def test_parse_table_geometry_full(tmp_path: Path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text(textwrap.dedent("""\
        <sdf version="1.10">
          <world name="testworld">
            <model name="mesa_pro">
              <pose>0.10 -0.05 0.20 0 0 0</pose>
              <link name="lnk">
                <collision name="tablero_collision">
                  <pose>0.01 0.02 0.03 0 0 0</pose>
                  <geometry>
                    <box>
                      <size>0.5 0.4 0.05</size>
                    </box>
                  </geometry>
                </collision>
              </link>
            </model>
          </world>
        </sdf>
    """), encoding="utf-8")
    geom = parse_table_geometry_from_sdf(str(sdf))
    assert geom is not None
    cx, cy, sx, sy, tz = geom
    # center = model_pose + coll_pose
    assert cx == pytest.approx(0.11)
    assert cy == pytest.approx(-0.03)
    assert sx == pytest.approx(0.5)
    assert sy == pytest.approx(0.4)
    # table_z = model_z + coll_z + size_z/2 = 0.20 + 0.03 + 0.025 = 0.255
    assert tz == pytest.approx(0.255)


def test_parse_table_geometry_no_table_returns_none(tmp_path: Path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text("<sdf version='1.10'><world name='w'/></sdf>", encoding="utf-8")
    assert parse_table_geometry_from_sdf(str(sdf)) is None


def test_parse_table_geometry_malformed_returns_none(tmp_path: Path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text("<<<not xml at all>>>", encoding="utf-8")
    assert parse_table_geometry_from_sdf(str(sdf)) is None
