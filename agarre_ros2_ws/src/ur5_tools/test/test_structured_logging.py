#!/usr/bin/env python3
"""F11 audit (2026-05-10): tests del helper structured_logging."""
from __future__ import annotations

from ur5_tools.structured_logging import KNOWN_STATUSES, LogCtx, fmt, parse


def test_fmt_basic_phase_none() -> None:
    ctx = LogCtx(run_id="r1", node="orch")
    assert fmt(ctx, "OK", "hola") == (
        "[RUN_ID=r1][NODE=orch][PHASE=-][STATUS=OK] hola"
    )


def test_fmt_with_phase_and_cycle() -> None:
    ctx = LogCtx(run_id="r2", node="orch", phase="GRASP", cycle=3)
    assert fmt(ctx, "STARTED", "go") == (
        "[RUN_ID=r2][NODE=orch][PHASE=GRASP][STATUS=STARTED][CYCLE=3] go"
    )


def test_fmt_with_kwargs() -> None:
    ctx = LogCtx(run_id="r3", node="grip")
    out = fmt(ctx, "OK", "closed", dt_ms=1234, attempts=2)
    assert "[dt_ms=1234,attempts=2]" in out
    assert out.endswith(" closed")


def test_fmt_renders_floats_compactly() -> None:
    ctx = LogCtx(run_id="r4", node="planner")
    out = fmt(ctx, "OK", "result", error=0.001234567)
    assert "[error=0.001235]" in out


def test_fmt_renders_bools_as_1_0() -> None:
    ctx = LogCtx(run_id="r5", node="x")
    out = fmt(ctx, "OK", "m", success=True, retry=False)
    assert "[success=1,retry=0]" in out


def test_fmt_includes_extra_from_ctx() -> None:
    ctx = LogCtx(run_id="r6", node="x", extra={"session": "abc"})
    out = fmt(ctx, "OK", "m")
    assert "[session=abc]" in out


def test_fmt_kw_overrides_extra() -> None:
    ctx = LogCtx(run_id="r7", node="x", extra={"foo": "ctx"})
    out = fmt(ctx, "OK", "m", foo="kw")
    assert "[foo=kw]" in out
    assert "foo=ctx" not in out


def test_known_statuses_contains_canonical() -> None:
    for s in ("STARTED", "OK", "FAIL", "RETRY", "SKIPPED", "PROGRESS"):
        assert s in KNOWN_STATUSES


def test_parse_round_trip() -> None:
    ctx = LogCtx(run_id="abc", node="orch", phase="LIFT", cycle=2)
    line = fmt(ctx, "OK", "lifted", dt_ms=900)
    parsed = parse(line)
    assert parsed["run_id"] == "abc"
    assert parsed["node"] == "orch"
    assert parsed["phase"] == "LIFT"
    assert parsed["status"] == "OK"
    assert parsed["cycle"] == 2
    assert parsed["extra"]["dt_ms"] == "900"
    assert parsed["msg"] == "lifted"


def test_parse_phase_dash_means_none() -> None:
    ctx = LogCtx(run_id="abc", node="orch")
    line = fmt(ctx, "INFO", "alive")
    parsed = parse(line)
    assert parsed["phase"] is None
    assert parsed["cycle"] is None


def test_parse_unstructured_line_returns_msg_only() -> None:
    parsed = parse("plain log line without prefix")
    assert parsed == {"msg": "plain log line without prefix"}
