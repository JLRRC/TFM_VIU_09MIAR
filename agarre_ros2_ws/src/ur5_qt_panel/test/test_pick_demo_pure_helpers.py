#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_demo_pure_helpers.py
# Contenido: Tests unitarios de ur5_qt_panel.pick_demo.pure_helpers.
"""Tests para las funciones puras extraídas de run_pick_demo (F3 step 1)."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from ur5_qt_panel.pick_demo.pure_helpers import (
    iso_now,
    json_safe,
    vec_norm,
    vector_minus,
    z_delta,
)


# ---------------------------------------------------------------------------
# iso_now
# ---------------------------------------------------------------------------


def test_iso_now_returns_string_with_ms():
    s = iso_now()
    assert isinstance(s, str)
    # Formato esperado: "2026-04-30T22:01:02.345+02:00" (con ms y offset)
    assert "T" in s
    assert "." in s  # ms separator


def test_iso_now_monotonic_calls_strict_increase_or_equal():
    a = iso_now()
    b = iso_now()
    # Comparación lexicográfica: a <= b en ISO 8601 timestamps consecutivos
    assert a <= b


# ---------------------------------------------------------------------------
# json_safe
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value",
    [None, "abc", 42, 3.14, True, False],
)
def test_json_safe_passthrough_primitives(value):
    assert json_safe(value) == value


def test_json_safe_path_to_str():
    p = Path("/tmp/foo")
    assert json_safe(p) == "/tmp/foo"


def test_json_safe_list_recurses():
    assert json_safe([1, "a", Path("/x"), None]) == [1, "a", "/x", None]


def test_json_safe_tuple_to_list():
    assert json_safe((1, 2, 3)) == [1, 2, 3]


def test_json_safe_dict_keys_become_str():
    out = json_safe({1: "a", "b": 2})
    assert out == {"1": "a", "b": 2}


def test_json_safe_nested():
    inp = {"outer": [Path("/x"), {"inner": (1.0, None)}]}
    assert json_safe(inp) == {"outer": ["/x", {"inner": [1.0, None]}]}


def test_json_safe_falls_back_to_float_or_str():
    class HasFloat:
        def __float__(self):
            return 2.5

    class NoFloat:
        def __str__(self):
            return "hello"

    assert json_safe(HasFloat()) == 2.5
    assert json_safe(NoFloat()) == "hello"


# ---------------------------------------------------------------------------
# vector_minus
# ---------------------------------------------------------------------------


def test_vector_minus_basic():
    assert vector_minus((1.0, 2.0, 3.0), (1.0, 1.0, 1.0)) == (0.0, 1.0, 2.0)


def test_vector_minus_none_inputs():
    assert vector_minus(None, (1, 2, 3)) is None
    assert vector_minus((1, 2, 3), None) is None
    assert vector_minus(None, None) is None


def test_vector_minus_negative_result():
    assert vector_minus((0, 0, 0), (1, 2, 3)) == (-1.0, -2.0, -3.0)


def test_vector_minus_invalid_returns_none():
    assert vector_minus(("x", 0, 0), (0, 0, 0)) is None


# ---------------------------------------------------------------------------
# vec_norm
# ---------------------------------------------------------------------------


def test_vec_norm_unit_axes():
    assert vec_norm((1, 0, 0)) == pytest.approx(1.0)
    assert vec_norm((0, 1, 0)) == pytest.approx(1.0)
    assert vec_norm((0, 0, 1)) == pytest.approx(1.0)


def test_vec_norm_345_triple():
    assert vec_norm((3, 4, 0)) == pytest.approx(5.0)


def test_vec_norm_zero():
    assert vec_norm((0, 0, 0)) == 0.0


def test_vec_norm_none():
    assert vec_norm(None) is None


def test_vec_norm_invalid_input():
    assert vec_norm(("a", "b", "c")) is None


# ---------------------------------------------------------------------------
# z_delta
# ---------------------------------------------------------------------------


def test_z_delta_basic():
    assert z_delta((1, 2, 5), (1, 2, 3)) == 2.0


def test_z_delta_negative():
    assert z_delta((0, 0, 1), (0, 0, 4)) == -3.0


def test_z_delta_none_inputs():
    assert z_delta(None, (1, 2, 3)) is None
    assert z_delta((1, 2, 3), None) is None


def test_z_delta_invalid():
    assert z_delta((1, 2, "z"), (0, 0, 0)) is None
