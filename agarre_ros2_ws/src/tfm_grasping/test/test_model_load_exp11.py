#!/usr/bin/env python3
"""Regression tests: loader accepts EXP1 (SimpleCNN) and EXP1.1 (SimpleGrasp) checkpoints."""

import os
import pytest

CKPT_EXP11 = os.path.expanduser(
    "~/TFM/agarre_inteligente/experiments"
    "/EXP1.1_SIMPLEGRASP_RGB/seed_0/checkpoints/best.pth"
)
CKPT_EXP1 = os.path.expanduser(
    "~/TFM/agarre_inteligente/experiments"
    "/EXP1_SIMPLE_RGB/seed_0/checkpoints/best.pth"
)
CKPT_EXP3 = os.path.expanduser(
    "~/TFM/agarre_inteligente/experiments"
    "/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth"
)


def _requires_checkpoint(path):
    return pytest.mark.skipif(
        not os.path.isfile(path),
        reason=f"checkpoint no disponible: {path}",
    )


@_requires_checkpoint(CKPT_EXP11)
def test_exp11_loads_as_simple_grasp():
    from tfm_grasping.model import GraspModel

    m = GraspModel(model_path=CKPT_EXP11)
    assert m.load(), f"EXP1.1 no cargó: {m.last_error()}"
    assert m.info.loaded
    assert m.info.model_name == "simple_grasp", (
        f"se esperaba simple_grasp, se obtuvo {m.info.model_name!r}"
    )


@_requires_checkpoint(CKPT_EXP1)
def test_exp1_loads_as_simple_cnn():
    from tfm_grasping.model import GraspModel

    m = GraspModel(model_path=CKPT_EXP1)
    assert m.load(), f"EXP1 no cargó: {m.last_error()}"
    assert m.info.loaded
    assert m.info.model_name == "simple_cnn", (
        f"se esperaba simple_cnn, se obtuvo {m.info.model_name!r}"
    )


@_requires_checkpoint(CKPT_EXP3)
def test_exp3_loads_as_resnet18():
    from tfm_grasping.model import GraspModel

    m = GraspModel(model_path=CKPT_EXP3)
    assert m.load(), f"EXP3 no cargó: {m.last_error()}"
    assert m.info.loaded
    assert m.info.model_name == "resnet18", (
        f"se esperaba resnet18, se obtuvo {m.info.model_name!r}"
    )
