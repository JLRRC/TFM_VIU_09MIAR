#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_preflight.py
# Contenido: F12-step2c.1 — tests de la lógica pura de INITIAL_SNAPSHOT y HOME_INITIAL.
"""Tests del módulo ``tfm_orchestrator.preflight`` (sin ROS).

Cubre:

* ``compose_initial_snapshot``: construcción + cálculo de deltas y dist3d.
* Validación de entradas malformadas (None, longitudes incorrectas, NaN).
* ``InitialSnapshot.is_complete`` / ``to_log_dict``.
* ``validate_home_target``: target reached, joints faltantes, fuera de
  tolerancia, casos límite (target con arity diferente, current vacío).
* Diferencia angular más corta (wrap a [-π, π]).
"""

from __future__ import annotations

import math

import pytest

from tfm_orchestrator.preflight import (
    UR5_HOME_JOINTS_RAD,
    UR5_JOINT_NAMES,
    compose_initial_snapshot,
    validate_home_target,
)


# ---------------------------------------------------------------------------
# compose_initial_snapshot
# ---------------------------------------------------------------------------


def test_compose_initial_snapshot_with_complete_data():
    snap = compose_initial_snapshot(
        request_id="req-001",
        tcp_base=(0.4, 0.0, 0.5),
        object_base=(0.5, 0.0, 0.5),
        joints={"shoulder_pan_joint": 0.0, "elbow_joint": 1.5},
        pose_source="tf_live",
        flow_name="DIRECT",
    )
    assert snap.request_id == "req-001"
    assert snap.tcp_base == (0.4, 0.0, 0.5)
    assert snap.object_base == (0.5, 0.0, 0.5)
    assert snap.dx == pytest.approx(0.1)
    assert snap.dy == pytest.approx(0.0)
    assert snap.dz == pytest.approx(0.0)
    assert snap.dist3d == pytest.approx(0.1)
    assert snap.joints["shoulder_pan_joint"] == 0.0
    assert snap.joints["elbow_joint"] == 1.5
    assert snap.pose_source == "tf_live"
    assert snap.flow_name == "DIRECT"
    assert snap.is_complete()


def test_compose_initial_snapshot_missing_object():
    snap = compose_initial_snapshot(
        request_id="r2",
        tcp_base=(0.4, 0.0, 0.5),
        object_base=None,
    )
    assert snap.tcp_base == (0.4, 0.0, 0.5)
    assert snap.object_base is None
    assert snap.dx is None
    assert snap.dy is None
    assert snap.dz is None
    assert snap.dist3d is None
    assert not snap.is_complete()


def test_compose_initial_snapshot_missing_tcp():
    snap = compose_initial_snapshot(
        tcp_base=None,
        object_base=(0.5, 0.0, 0.5),
    )
    assert snap.dist3d is None
    assert not snap.is_complete()


def test_compose_initial_snapshot_malformed_vec():
    """Listas de longitud 2 deben tratarse como None (no propagar error)."""
    snap = compose_initial_snapshot(
        tcp_base=[0.1, 0.2],  # missing z
        object_base=(0.5, 0.0, 0.5),
    )
    assert snap.tcp_base is None
    assert snap.dist3d is None


def test_compose_initial_snapshot_non_numeric_value():
    snap = compose_initial_snapshot(
        tcp_base=("a", 0.0, 0.5),
        object_base=(0.5, 0.0, 0.5),
    )
    assert snap.tcp_base is None


def test_compose_initial_snapshot_joints_filters_invalid_values():
    snap = compose_initial_snapshot(
        joints={"shoulder_pan_joint": 0.5, "bad": "abc", "wrist_3_joint": 1.0},
    )
    assert "shoulder_pan_joint" in snap.joints
    assert "wrist_3_joint" in snap.joints
    assert "bad" not in snap.joints


def test_compose_initial_snapshot_default_flow_name():
    snap = compose_initial_snapshot()
    assert snap.flow_name == "DIRECT"


def test_compose_initial_snapshot_to_log_dict_keys():
    snap = compose_initial_snapshot(
        tcp_base=(0.1, 0.0, 0.5),
        object_base=(0.4, 0.0, 0.5),
    )
    d = snap.to_log_dict()
    assert set(d.keys()) == {
        "request_id", "tcp_base", "object_base",
        "dx", "dy", "dz", "dist3d",
        "joints", "pose_source", "flow_name",
    }
    assert d["dist3d"] == pytest.approx(0.3)


def test_compose_initial_snapshot_is_immutable():
    snap = compose_initial_snapshot(tcp_base=(0.1, 0.0, 0.5))
    with pytest.raises(Exception):
        snap.tcp_base = (1.0, 1.0, 1.0)  # type: ignore[misc]


# ---------------------------------------------------------------------------
# validate_home_target
# ---------------------------------------------------------------------------


def _home_joints_dict() -> dict:
    return dict(zip(UR5_JOINT_NAMES, UR5_HOME_JOINTS_RAD))


def test_validate_home_target_at_home():
    chk = validate_home_target(_home_joints_dict())
    assert chk.target_reached
    assert chk.reason == "ok"
    assert chk.max_diff_rad == pytest.approx(0.0)
    assert set(chk.per_joint_diffs.keys()) == set(UR5_JOINT_NAMES)


def test_validate_home_target_within_tolerance():
    cur = _home_joints_dict()
    cur["shoulder_pan_joint"] = 0.01  # dentro de tol_rad=0.02
    chk = validate_home_target(cur, tol_rad=0.02)
    assert chk.target_reached
    assert chk.max_diff_rad == pytest.approx(0.01)


def test_validate_home_target_out_of_tolerance():
    cur = _home_joints_dict()
    cur["elbow_joint"] = 0.5  # muy lejos del home (target=0.0)
    chk = validate_home_target(cur, tol_rad=0.02)
    assert not chk.target_reached
    assert chk.reason.startswith("out_of_tol:elbow_joint=")


def test_validate_home_target_missing_joint():
    cur = _home_joints_dict()
    del cur["wrist_3_joint"]
    chk = validate_home_target(cur, tol_rad=0.02)
    assert not chk.target_reached
    assert "wrist_3_joint" in chk.missing_joints
    assert chk.reason == "missing:wrist_3_joint"


def test_validate_home_target_empty_joints():
    chk = validate_home_target({}, tol_rad=0.02)
    assert not chk.target_reached
    assert chk.reason == "no_joints"
    assert chk.missing_joints == list(UR5_JOINT_NAMES)


def test_validate_home_target_none_joints():
    chk = validate_home_target(None, tol_rad=0.02)
    assert not chk.target_reached
    assert chk.reason == "no_joints"


def test_validate_home_target_arity_mismatch():
    chk = validate_home_target(
        {"a": 0.0},
        target=(0.0, 0.0),
        joint_names=("a",),  # arity 1 vs target arity 2
        tol_rad=0.02,
    )
    assert not chk.target_reached
    assert chk.reason.startswith("target_arity_mismatch:")


def test_validate_home_target_angle_wraparound():
    """Diferencia angular toma el camino corto: 2π es lo mismo que 0."""
    cur = _home_joints_dict()
    cur["shoulder_pan_joint"] = 2.0 * math.pi  # equivalente a 0.0
    chk = validate_home_target(cur, tol_rad=0.001)
    assert chk.target_reached
    assert chk.max_diff_rad < 1e-6


def test_validate_home_target_custom_tolerance():
    cur = _home_joints_dict()
    cur["shoulder_pan_joint"] = 0.05
    # Tolerancia laxa: pasa.
    chk_lax = validate_home_target(cur, tol_rad=0.1)
    assert chk_lax.target_reached
    # Tolerancia estricta: falla.
    chk_strict = validate_home_target(cur, tol_rad=0.01)
    assert not chk_strict.target_reached


def test_validate_home_target_check_is_immutable():
    chk = validate_home_target(_home_joints_dict())
    with pytest.raises(Exception):
        chk.target_reached = False  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Constantes documentadas
# ---------------------------------------------------------------------------


def test_ur5_home_joints_canonical_shape():
    assert len(UR5_HOME_JOINTS_RAD) == 6
    assert len(UR5_JOINT_NAMES) == 6
    # shoulder_lift y wrist_1 son -π/2 en HOME canónico.
    names_to_value = dict(zip(UR5_JOINT_NAMES, UR5_HOME_JOINTS_RAD))
    assert names_to_value["shoulder_lift_joint"] == pytest.approx(-math.pi / 2.0)
    assert names_to_value["wrist_1_joint"] == pytest.approx(-math.pi / 2.0)
