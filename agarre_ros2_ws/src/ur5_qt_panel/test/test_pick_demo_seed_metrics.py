#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_demo_seed_metrics.py
"""F3-step40 (2026-05-08) — Tests offline para pick_demo.seed_metrics.

Cubre seed_devs / seed_max_dev / seed_sum_dev con casos de:
- Sin desviación (q=s).
- Wrap angular (q y s en ramas distintas, |q-s| > π).
- Múltiples joints.
"""

from __future__ import annotations

import math

import pytest

from ur5_qt_panel.pick_demo.seed_metrics import (
    seed_devs,
    seed_max_dev,
    seed_sum_dev,
)


# ---------------------------------------------------------------------------
# seed_devs
# ---------------------------------------------------------------------------


def test_seed_devs_zero_when_equal():
    """q == s ⇒ deviation = 0 en cada joint."""
    devs = seed_devs([0.0, 1.0, -2.5], [0.0, 1.0, -2.5])
    assert devs == [0.0, 0.0, 0.0]


def test_seed_devs_simple_difference():
    """|q - s| < π ⇒ no wrap, deviation = |q - s|."""
    devs = seed_devs([0.1, 0.5], [0.0, 1.0])
    assert devs[0] == pytest.approx(0.1)
    assert devs[1] == pytest.approx(0.5)


def test_seed_devs_wrap_around_2pi():
    """q en rama opuesta del seed ⇒ wrap por 2π eliminado."""
    # q=π, s=-π. Sin wrap: |π - (-π)| = 2π. Con wrap: 0.
    devs = seed_devs([math.pi], [-math.pi])
    assert devs[0] == pytest.approx(0.0, abs=1e-9)


def test_seed_devs_wrap_partial():
    """q=3π/2 (~4.71), s=-π/2 (~-1.57). Diff sin wrap = 2π. Con wrap = 0."""
    devs = seed_devs([3.0 * math.pi / 2.0], [-math.pi / 2.0])
    assert devs[0] == pytest.approx(0.0, abs=1e-9)


def test_seed_devs_multiple_joints():
    """6 joints UR5 estilo."""
    q = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
    s = [0.1, -math.pi / 2 + 0.05, 0.02, -math.pi / 2, 0.0, 0.5]
    devs = seed_devs(q, s)
    assert len(devs) == 6
    assert devs[0] == pytest.approx(0.1)
    assert devs[1] == pytest.approx(0.05)
    assert devs[2] == pytest.approx(0.02)
    assert devs[5] == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# seed_max_dev / seed_sum_dev
# ---------------------------------------------------------------------------


def test_seed_max_dev_returns_max():
    q = [0.0, 0.0, 0.0]
    s = [0.1, 0.5, 0.3]
    assert seed_max_dev(q, s) == pytest.approx(0.5)


def test_seed_sum_dev_returns_sum():
    q = [0.0, 0.0, 0.0]
    s = [0.1, 0.2, 0.3]
    assert seed_sum_dev(q, s) == pytest.approx(0.6)


def test_seed_max_dev_with_wrap():
    """seed_max_dev consume seed_devs corregido por wrap."""
    q = [math.pi, 0.0]
    s = [-math.pi, 0.5]
    # devs = [0.0, 0.5]
    assert seed_max_dev(q, s) == pytest.approx(0.5)
