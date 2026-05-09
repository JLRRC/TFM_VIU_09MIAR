#!/usr/bin/env python3
"""Regression tests: loader accepts EXP1 (SimpleCNN) and EXP1.1 (SimpleGrasp) checkpoints.

Also verifies that simple_grasp inference clips diverged outputs to visible frame coordinates.
"""

import os
import numpy as np
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


@_requires_checkpoint(CKPT_EXP11)
def test_exp11_inference_stays_in_frame():
    """Diverged simple_grasp outputs must produce coordinates within a sane frame range."""
    from tfm_grasping.model import GraspModel
    from tfm_grasping.perception import InputFrame

    m = GraspModel(model_path=CKPT_EXP11, img_size=224)
    assert m.load()

    frame_w, frame_h = 320, 240
    image = np.zeros((frame_h, frame_w, 3), dtype=np.uint8)
    frame = InputFrame(image=image, width=frame_w, height=frame_h, timestamp=0.0)

    grasp = m.infer(frame)
    assert grasp is not None, "infer devolvió None para simple_grasp con frame válido"

    # Las coordenadas deben ser finitas y no astronómicas.
    margin = max(frame_w, frame_h) * 2  # tolerancia: hasta 2 veces el frame
    assert abs(grasp.center_x) < margin, f"center_x fuera de rango: {grasp.center_x}"
    assert abs(grasp.center_y) < margin, f"center_y fuera de rango: {grasp.center_y}"
    assert grasp.width_px < margin, f"width_px fuera de rango: {grasp.width_px}"
    assert grasp.height_px < margin, f"height_px fuera de rango: {grasp.height_px}"
    assert abs(grasp.angle_rad) < 2 * 3.14159, f"angle_rad fuera de rango: {grasp.angle_rad}"
