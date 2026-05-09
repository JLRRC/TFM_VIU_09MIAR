#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_param_utils.py
"""Unit tests for param_utils — pure duck-typed logic, no ROS node needed."""
from __future__ import annotations

import pytest

from ur5_tools.param_utils import (
    read_float_param,
    read_int_param,
    read_str_list_param,
    read_str_param,
)


# ---------------------------------------------------------------------------
# Fake node stubs
# ---------------------------------------------------------------------------

class _Param:
    def __init__(self, value):
        self.value = value


class _FakeNode:
    def __init__(self, params: dict):
        self._params = params
        self.warnings: list[str] = []

    def get_parameter(self, name: str) -> _Param:
        if name not in self._params:
            raise KeyError(name)
        return _Param(self._params[name])

    def get_logger(self):
        class _L:
            def __init__(self, storage):
                self._s = storage
            def warn(self, msg):
                self._s.append(msg)
        return _L(self.warnings)


class _BrokenNode:
    """Node whose get_parameter always raises."""
    def get_parameter(self, name):
        raise RuntimeError("no params")

    def get_logger(self):
        class _L:
            def warn(self, msg): pass
        return _L()


# ---------------------------------------------------------------------------
# read_str_param
# ---------------------------------------------------------------------------

def test_read_str_param_returns_value():
    node = _FakeNode({"my_str": "hello"})
    assert read_str_param(node, "my_str") == "hello"


def test_read_str_param_missing_returns_default():
    node = _FakeNode({})
    assert read_str_param(node, "missing", default="fallback") == "fallback"


def test_read_str_param_none_returns_default():
    node = _FakeNode({"key": None})
    assert read_str_param(node, "key", default="d") == "d"


def test_read_str_param_broken_node_returns_default():
    assert read_str_param(_BrokenNode(), "x", default="safe") == "safe"


def test_read_str_param_int_coerced_to_str():
    node = _FakeNode({"n": 42})
    assert read_str_param(node, "n") == "42"


# ---------------------------------------------------------------------------
# read_str_list_param
# ---------------------------------------------------------------------------

def test_read_str_list_param_list_value():
    node = _FakeNode({"names": ["a", "b", "c"]})
    assert read_str_list_param(node, "names") == ["a", "b", "c"]


def test_read_str_list_param_tuple_value():
    node = _FakeNode({"t": ("x", "y")})
    assert read_str_list_param(node, "t") == ["x", "y"]


def test_read_str_list_param_str_becomes_single_element():
    node = _FakeNode({"s": "solo"})
    assert read_str_list_param(node, "s") == ["solo"]


def test_read_str_list_param_missing_returns_default():
    node = _FakeNode({})
    assert read_str_list_param(node, "x", default=["d"]) == ["d"]


def test_read_str_list_param_none_returns_empty():
    node = _FakeNode({"k": None})
    assert read_str_list_param(node, "k") == []


def test_read_str_list_param_broken_node_returns_default():
    result = read_str_list_param(_BrokenNode(), "x", default=["safe"])
    assert result == ["safe"]


# ---------------------------------------------------------------------------
# read_float_param
# ---------------------------------------------------------------------------

def test_read_float_param_returns_value():
    node = _FakeNode({"f": 3.14})
    assert read_float_param(node, "f") == pytest.approx(3.14)


def test_read_float_param_int_cast_to_float():
    node = _FakeNode({"n": 5})
    result = read_float_param(node, "n")
    assert isinstance(result, float)
    assert result == pytest.approx(5.0)


def test_read_float_param_str_parsed():
    node = _FakeNode({"s": "2.5"})
    assert read_float_param(node, "s") == pytest.approx(2.5)


def test_read_float_param_invalid_returns_default():
    node = _FakeNode({"bad": "not_a_float"})
    assert read_float_param(node, "bad", default=9.9) == pytest.approx(9.9)


def test_read_float_param_clamped_to_min():
    node = _FakeNode({"v": -5.0})
    assert read_float_param(node, "v", default=0.0, min_value=0.0) == pytest.approx(0.0)


def test_read_float_param_clamped_to_max():
    node = _FakeNode({"v": 100.0})
    assert read_float_param(node, "v", default=0.0, max_value=1.0) == pytest.approx(1.0)


def test_read_float_param_within_range_not_clamped():
    node = _FakeNode({"v": 0.5})
    assert read_float_param(node, "v", default=0.0, min_value=0.0, max_value=1.0) == pytest.approx(0.5)


def test_read_float_param_missing_returns_default():
    node = _FakeNode({})
    assert read_float_param(node, "missing", default=7.7) == pytest.approx(7.7)


def test_read_float_param_broken_node_returns_default():
    assert read_float_param(_BrokenNode(), "x", default=1.5) == pytest.approx(1.5)


# ---------------------------------------------------------------------------
# read_int_param
# ---------------------------------------------------------------------------

def test_read_int_param_returns_value():
    node = _FakeNode({"i": 42})
    assert read_int_param(node, "i") == 42


def test_read_int_param_float_truncated():
    node = _FakeNode({"f": 3.9})
    assert read_int_param(node, "f") == 3


def test_read_int_param_invalid_returns_default():
    node = _FakeNode({"bad": "xyz"})
    assert read_int_param(node, "bad", default=0) == 0


def test_read_int_param_clamped_to_min():
    node = _FakeNode({"v": -10})
    assert read_int_param(node, "v", default=0, min_value=0) == 0


def test_read_int_param_above_min_not_clamped():
    node = _FakeNode({"v": 5})
    assert read_int_param(node, "v", default=0, min_value=0) == 5


def test_read_int_param_missing_returns_default():
    node = _FakeNode({})
    assert read_int_param(node, "gone", default=99) == 99


def test_read_int_param_broken_node_returns_default():
    assert read_int_param(_BrokenNode(), "x", default=7) == 7


# ---------------------------------------------------------------------------
# Additional coverage: uncovered error paths
# ---------------------------------------------------------------------------

class _NodeWithBrokenLogger:
    """Node that returns a non-list/tuple/str value AND has a logger that raises."""
    def get_parameter(self, name):
        return _Param(42)  # int → not list/tuple/str → triggers _warn_invalid

    def get_logger(self):
        raise RuntimeError("logger_hardware_fault")


class _NodeWithBadStrValue:
    """Node whose parameter value has a __str__ that raises."""
    class _BadValue:
        def __str__(self):
            raise RuntimeError("no_str_conversion")

    class _Logger:
        def warn(self, msg): pass

    def get_parameter(self, name):
        return _Param(_NodeWithBadStrValue._BadValue())

    def get_logger(self):
        return _NodeWithBadStrValue._Logger()


def test_warn_invalid_logger_raises_is_swallowed():
    # _warn_invalid called; node.get_logger() raises → except: pass (lines 16-17)
    # Triggered via read_str_list_param with int value + broken logger
    result = read_str_list_param(_NodeWithBrokenLogger(), "x", default=["safe"])
    assert result == ["safe"]


def test_read_str_param_str_conversion_raises_returns_default():
    # str(value) raises → except Exception: _warn_invalid, return default (lines 29-31)
    result = read_str_param(_NodeWithBadStrValue(), "x", default="fallback")
    assert result == "fallback"


def test_read_str_list_param_non_list_tuple_str_warns_and_returns_default():
    # value is int → not list/tuple/str → _warn_invalid + return list(default) (lines 50-51)
    node = _FakeNode({"n": 42})
    result = read_str_list_param(node, "n", default=["d"])
    assert result == ["d"]
    assert any("42" in w for w in node.warnings)
