#!/usr/bin/env python3
"""Regression tests for runtime grasp output scaling."""

import math

import numpy as np

from tfm_grasping.model import GraspModel
from tfm_grasping.perception import InputFrame


FLOAT_TOL = 1e-5


def _frame(width=320, height=240):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    return InputFrame(image=image, width=width, height=height, timestamp=0.0)


def test_decode_prediction_scales_normalized_outputs_to_frame_pixels():
    model = GraspModel(img_size=224)
    frame = _frame()

    grasp = model._decode_prediction(
        np.array([0.5, 0.25, 0.4, 0.2, 0.5], dtype=np.float32),
        frame,
        roi_info=None,
    )

    assert grasp is not None
    assert grasp.center_x == 160.0
    assert grasp.center_y == 60.0
    assert math.isclose(grasp.width_px, 128.0, rel_tol=0.0, abs_tol=FLOAT_TOL)
    assert math.isclose(grasp.height_px, 48.0, rel_tol=0.0, abs_tol=FLOAT_TOL)
    assert math.isclose(grasp.angle_rad, math.radians(45.0), rel_tol=0.0, abs_tol=FLOAT_TOL)


def test_decode_prediction_preserves_pixel_outputs_in_roi_space():
    model = GraspModel(img_size=224)
    frame = _frame()

    grasp = model._decode_prediction(
        np.array([112.0, 56.0, 44.8, 22.4, 30.0], dtype=np.float32),
        frame,
        roi_info=(80, 40, 160, 160),
    )

    assert grasp is not None
    assert grasp.center_x == 160.0
    assert grasp.center_y == 80.0
    assert math.isclose(grasp.width_px, 32.0, rel_tol=0.0, abs_tol=FLOAT_TOL)
    assert math.isclose(grasp.height_px, 16.0, rel_tol=0.0, abs_tol=FLOAT_TOL)
    assert math.isclose(grasp.angle_rad, math.radians(30.0), rel_tol=0.0, abs_tol=FLOAT_TOL)


def test_decode_prediction_handles_mixed_pixel_centers_and_normalized_sizes():
    model = GraspModel(img_size=224)
    frame = _frame()

    grasp = model._decode_prediction(
        np.array([112.0, 56.0, 0.4, 0.2, 0.5], dtype=np.float32),
        frame,
        roi_info=(80, 40, 160, 160),
    )

    assert grasp is not None
    assert grasp.center_x == 160.0
    assert grasp.center_y == 80.0
    assert math.isclose(grasp.width_px, 64.0, rel_tol=0.0, abs_tol=FLOAT_TOL)
    assert math.isclose(grasp.height_px, 32.0, rel_tol=0.0, abs_tol=FLOAT_TOL)
    assert math.isclose(grasp.angle_rad, math.radians(45.0), rel_tol=0.0, abs_tol=FLOAT_TOL)