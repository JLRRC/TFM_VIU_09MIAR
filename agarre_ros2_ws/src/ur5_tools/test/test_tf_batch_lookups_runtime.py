#!/usr/bin/env python3
"""F8 audit-v4: tests offline para tf_batch_lookups_runtime con mock Buffer."""
from __future__ import annotations

from typing import Any, Dict, List, Tuple

from ur5_tools.tf_batch_lookups import LookupRequest
from ur5_tools.tf_batch_lookups_runtime import (
    execute_batch,
    lookup_with_batch_cache,
)


class _FakeBuffer:
    """Mock de tf2_ros.Buffer.

    - Cuenta cuántas llamadas a ``lookup_transform`` recibe.
    - Devuelve un dict-like marker por (target,source).
    - Si target/source contiene 'fail' lanza para simular fallo.
    """

    def __init__(self) -> None:
        self.calls: List[Tuple[str, str]] = []

    def lookup_transform(
        self, target: str, source: str, time: Any, timeout: Any
    ) -> Dict[str, str]:
        self.calls.append((target, source))
        if "fail" in target or "fail" in source:
            raise RuntimeError("synthetic failure")
        return {"target": target, "source": source}


def test_execute_batch_dedupes_latest() -> None:
    buf = _FakeBuffer()
    reqs = [
        LookupRequest("base", "world"),
        LookupRequest("base", "world"),
        LookupRequest("base", "world"),
        LookupRequest("tcp", "base"),
    ]
    out = execute_batch(buf, reqs)
    assert len(buf.calls) == 2
    assert sorted(buf.calls) == [("base", "world"), ("tcp", "base")]
    assert ("base", "world", 0.0) in out
    assert ("tcp", "base", 0.0) in out


def test_execute_batch_handles_failure_per_entry() -> None:
    buf = _FakeBuffer()
    reqs = [
        LookupRequest("ok_target", "ok_source"),
        LookupRequest("fail_target", "any"),
    ]
    out = execute_batch(buf, reqs)
    assert out[("ok_target", "ok_source", 0.0)] is not None
    assert out[("fail_target", "any", 0.0)] is None


def test_execute_batch_logs_savings() -> None:
    buf = _FakeBuffer()
    captured: List[str] = []

    def _log(msg: str) -> None:
        captured.append(msg)

    reqs = [
        LookupRequest("base", "world"),
        LookupRequest("base", "world"),
        LookupRequest("base", "world"),
    ]
    execute_batch(buf, reqs, log_fn=_log)
    assert any("n_input=3" in m and "saved=2" in m for m in captured)


def test_execute_batch_empty_input() -> None:
    buf = _FakeBuffer()
    out = execute_batch(buf, [])
    assert out == {}
    assert buf.calls == []


def test_lookup_with_batch_cache_hit() -> None:
    buf = _FakeBuffer()
    reqs = [
        LookupRequest("base", "world"),
        LookupRequest("tcp", "base"),
    ]
    res = lookup_with_batch_cache(buf, reqs, "base", "world")
    assert res is not None
    assert res["target"] == "base"


def test_lookup_with_batch_cache_miss_returns_none() -> None:
    buf = _FakeBuffer()
    reqs = [LookupRequest("base", "world")]
    res = lookup_with_batch_cache(buf, reqs, "tcp", "world")
    assert res is None


def test_execute_batch_separates_timestamped_from_latest() -> None:
    """time_sec=0 (latest) y time_sec>0 deben dar entradas distintas."""
    buf = _FakeBuffer()
    reqs = [
        LookupRequest("base", "world", time_sec=0.0),
        LookupRequest("base", "world", time_sec=1234.5),
    ]
    out = execute_batch(buf, reqs)
    # 2 lookups distintos.
    assert len(buf.calls) == 2
    assert ("base", "world", 0.0) in out
    assert ("base", "world", 1234.5) in out
