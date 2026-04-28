#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_world_tf_publisher_score.py
# Contenido: F4 mínimo — regression test de world_tf_publisher._score_name.
"""Tests del scorer de world_tf_publisher.

Caso de regresión central:
  - 2026-04-27 el publisher aceptaba ``rg2_base_link`` como ``base_link``
    via ``endswith("base_link")`` y publicaba ``world->base_link`` con la
    pose del gripper rotada -90° en X. ``pos_err_m ≈ 0.487`` en
    panel_pick_demo, IK imposible.
  - Fix en commit 309a88e: separador ``"::"`` o ``"/"`` requerido.

Este test se ejecuta sin Gazebo / sin rclpy (no instancia ``Node``); solo
ejercita ``_score_name`` sobre un stub con los atributos relevantes.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Callable

import pytest

from ur5_tools.world_tf_publisher import WorldTfPublisher


def _make_scorer(
    model_name: str = "ur5_rg2",
    base_frame: str = "base_link",
) -> Callable[[str], int]:
    """Vincular ``_score_name`` a un stub mínimo sin levantar un Node."""
    ns = SimpleNamespace(_model_name=model_name, _base_frame=base_frame)
    return lambda name: WorldTfPublisher._score_name(ns, name)


def test_full_qualified_model_link_wins():
    score = _make_scorer()
    assert score("ur5_rg2::base_link") == 120


def test_bare_base_frame_high():
    score = _make_scorer()
    assert score("base_link") == 110


def test_other_model_base_link_endswith_double_colon():
    score = _make_scorer()
    # endswith("::base_link") sin coincidir con el modelo propio
    assert score("foreign_model::base_link") == 100


def test_other_model_base_frame_param_endswith():
    # Cuando el base_frame es distinto a "base_link", la rama dedicada se aplica.
    score = _make_scorer(model_name="ur5_rg2", base_frame="custom_base")
    assert score("foreign_model::custom_base") == 95


def test_slash_separator_accepts_real_base_link():
    score = _make_scorer()
    # Cualquier prefijo con "/" seguido del frame exacto al final.
    assert score("ur5_rg2/foo/base_link") == 85


def test_regression_rg2_base_link_must_not_match():
    """Regresión 2026-04-27: el publisher elegía rg2_base_link como base_link.

    Con base_frame='base_link', el nombre 'rg2_base_link' termina con
    'base_link' por sufijo simple. La función debe devolver 0 (rechazar).
    """
    score = _make_scorer()
    assert score("rg2_base_link") == 0


def test_regression_rg2_base_link_no_match_with_double_colon_prefix():
    """Igual que el caso anterior pero el nombre ya viene con '::'.

    'ur5_rg2::rg2_base_link' termina en '::rg2_base_link', NO en
    '::base_link'. Debe devolver 0.
    """
    score = _make_scorer()
    assert score("ur5_rg2::rg2_base_link") == 0


def test_random_name_returns_zero():
    score = _make_scorer()
    assert score("foo") == 0
    assert score("/world") == 0
    assert score("table") == 0


def test_model_only_fallback():
    score = _make_scorer()
    # Cuando solo hay pose a nivel modelo (sin link), 50 puntos como fallback.
    assert score("ur5_rg2") == 50


def test_empty_name_zero():
    score = _make_scorer()
    assert score("") == 0


def test_priority_ordering():
    """Los scores deben preservar la jerarquía esperada de preferencia."""
    score = _make_scorer()
    full_qual = score("ur5_rg2::base_link")
    bare = score("base_link")
    foreign_double_colon = score("foreign_model::base_link")
    slash = score("ur5_rg2/foo/base_link")
    model_only = score("ur5_rg2")
    rg2_base = score("rg2_base_link")

    # Jerarquía: full > bare > foreign(::) > slash > model_only > rg2_base(rechazado)
    assert full_qual > bare > foreign_double_colon > slash > model_only > rg2_base
    assert rg2_base == 0


@pytest.mark.parametrize(
    "name",
    [
        "rg2_base_link",
        "RG2_BASE_LINK",  # caso por curiosidad: mayúsculas no deben confundir
        "ur5_rg2::rg2_base_link",
        "unrelated_base_link",
    ],
)
def test_anti_endswith_suffix_trap(name):
    """Cualquier nombre con sufijo 'base_link' sin '::' o '/' debe ser 0.

    Esta familia de tests congela el comportamiento del fix 309a88e — si
    alguien vuelve a ``endswith(self._base_frame)`` plano, estos casos
    saltarán.
    """
    score = _make_scorer()
    assert score(name) == 0, (
        f"name={name!r} aceptado por sufijo plano: regresión del bug 2026-04-27"
    )
