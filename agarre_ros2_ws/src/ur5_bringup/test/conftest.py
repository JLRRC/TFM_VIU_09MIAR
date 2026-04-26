#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/conftest.py
"""Add the launch/ directory to sys.path so launch_helpers is importable without install."""

import os
import sys

LAUNCH_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "launch")
)
if LAUNCH_DIR not in sys.path:
    sys.path.insert(0, LAUNCH_DIR)
