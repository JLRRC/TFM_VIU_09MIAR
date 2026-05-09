#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_tf_batch_lookups.py
"""F8 / #20 (2026-05-08) — Tests offline para tf_batch_lookups."""

from __future__ import annotations

import pytest

from ur5_tools.tf_batch_lookups import (
    BatchPlan,
    LookupRequest,
    batch_lookup_requests,
    reduction_ratio,
)


def test_empty_input_returns_empty_plan():
    plan = batch_lookup_requests([])
    assert plan.n_input == 0
    assert plan.n_output == 0
    assert plan.saved_calls == 0
    assert plan.unique_requests == []


def test_single_request_passes_through():
    req = LookupRequest("tcp", "base", time_sec=0.0, timeout_sec=0.1)
    plan = batch_lookup_requests([req])
    assert plan.n_input == 1
    assert plan.n_output == 1
    assert plan.saved_calls == 0


def test_two_identical_latest_collapse():
    req = LookupRequest("tcp", "base", time_sec=0.0, timeout_sec=0.1)
    plan = batch_lookup_requests([req, req])
    assert plan.n_input == 2
    assert plan.n_output == 1
    assert plan.saved_calls == 1
    assert plan.duplicates_grouped[("tcp", "base")] == 2


def test_three_identical_latest_collapse_to_one():
    reqs = [
        LookupRequest("tcp", "base"),
        LookupRequest("tcp", "base"),
        LookupRequest("tcp", "base"),
    ]
    plan = batch_lookup_requests(reqs)
    assert plan.n_output == 1
    assert plan.saved_calls == 2


def test_different_pairs_kept_separate():
    reqs = [
        LookupRequest("tcp", "base"),
        LookupRequest("rg2", "base"),
        LookupRequest("tcp", "world"),
    ]
    plan = batch_lookup_requests(reqs)
    assert plan.n_output == 3
    assert plan.saved_calls == 0


def test_max_timeout_wins_in_dedup():
    """Cuando se deduplica, gana el timeout más permisivo."""
    reqs = [
        LookupRequest("tcp", "base", timeout_sec=0.05),
        LookupRequest("tcp", "base", timeout_sec=0.50),  # ← winner
        LookupRequest("tcp", "base", timeout_sec=0.10),
    ]
    plan = batch_lookup_requests(reqs)
    assert plan.n_output == 1
    assert plan.unique_requests[0].timeout_sec == pytest.approx(0.50)


def test_timestamped_lookups_separate_from_latest():
    """Time(0)=latest y time>0 son lookups distintos aunque mismo (target,source)."""
    reqs = [
        LookupRequest("tcp", "base", time_sec=0.0),
        LookupRequest("tcp", "base", time_sec=12345.6),
    ]
    plan = batch_lookup_requests(reqs)
    assert plan.n_output == 2


def test_timestamped_same_pair_dedup():
    """Mismo (target, source, time_sec) con time>0 sí se deduplica."""
    reqs = [
        LookupRequest("tcp", "base", time_sec=12345.6, timeout_sec=0.1),
        LookupRequest("tcp", "base", time_sec=12345.6, timeout_sec=0.2),
    ]
    plan = batch_lookup_requests(reqs)
    assert plan.n_output == 1
    assert plan.saved_calls == 1
    assert plan.unique_requests[0].timeout_sec == pytest.approx(0.2)


def test_timestamped_different_times_kept():
    """Mismo par pero distintos timestamps son distintos lookups."""
    reqs = [
        LookupRequest("tcp", "base", time_sec=12345.0),
        LookupRequest("tcp", "base", time_sec=12346.0),
    ]
    plan = batch_lookup_requests(reqs)
    assert plan.n_output == 2


def test_real_world_panel_pick_demo_workload():
    """Caso real estimado: 5 lookups latest del mismo par + 3 distintos pares + 2 timestamped."""
    reqs = (
        [LookupRequest("rg2_pinch_center", "base_link") for _ in range(5)]
        + [LookupRequest("tool0", "base_link")]
        + [LookupRequest("rg2_tcp", "base_link")]
        + [LookupRequest("base_link", "world")]
        + [LookupRequest("rg2_pinch_center", "base_link", time_sec=1000.0),
           LookupRequest("rg2_pinch_center", "base_link", time_sec=1001.0)]
    )
    plan = batch_lookup_requests(reqs)
    # Esperado: 4 únicos latest + 2 timestamped = 6
    assert plan.n_output == 6
    assert plan.n_input == 10
    assert plan.saved_calls == 4
    assert reduction_ratio(plan) == pytest.approx(0.4)


def test_reduction_ratio_zero_input():
    plan = BatchPlan(unique_requests=[], n_input=0, n_output=0,
                     saved_calls=0, duplicates_grouped={})
    assert reduction_ratio(plan) == 0.0


def test_reduction_ratio_no_savings():
    reqs = [LookupRequest("tcp", "base"), LookupRequest("rg2", "base")]
    plan = batch_lookup_requests(reqs)
    assert reduction_ratio(plan) == 0.0


def test_request_strips_whitespace():
    """Frames con whitespace se normalizan (mismo bucket)."""
    reqs = [
        LookupRequest(" tcp ", "base"),
        LookupRequest("tcp", " base "),
    ]
    plan = batch_lookup_requests(reqs)
    assert plan.n_output == 1
