#!/usr/bin/env python3
"""Pytest config to import local tfm_grasping package without install."""

import os
import sys

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)