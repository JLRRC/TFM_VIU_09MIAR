"""Safety gate: workspace boundary checks and pre-flight checks for autopick."""
from __future__ import annotations

from typing import Dict, Optional, Tuple


_DEFAULTS = {
    "base_x_min": 0.20,
    "base_x_max": 0.70,
    "base_y_min": -0.30,
    "base_y_max": 0.30,
    "base_z_min": 0.00,
    "base_z_max": 0.30,
}


def check_workspace(
    point_base: Tuple[float, float, float],
    limits: Optional[Dict[str, float]] = None,
) -> Tuple[bool, str]:
    """Return (ok, reason).  ok=True means the point is inside the workspace."""
    lim = dict(_DEFAULTS)
    if limits:
        lim.update(limits)
    x, y, z = point_base
    if not (lim["base_x_min"] <= x <= lim["base_x_max"]):
        return False, f"x={x:.3f} outside [{lim['base_x_min']}, {lim['base_x_max']}]"
    if not (lim["base_y_min"] <= y <= lim["base_y_max"]):
        return False, f"y={y:.3f} outside [{lim['base_y_min']}, {lim['base_y_max']}]"
    if not (lim["base_z_min"] <= z <= lim["base_z_max"]):
        return False, f"z={z:.3f} outside [{lim['base_z_min']}, {lim['base_z_max']}]"
    return True, ""


def check_object_on_table(
    object_world_z: float,
    expected_z: float = 0.77,
    max_delta: float = 0.20,
) -> Tuple[bool, str]:
    """Validate that the object is on the table (within max_delta of expected_z)."""
    delta = abs(object_world_z - expected_z)
    if delta > max_delta:
        return False, (
            f"object world_z={object_world_z:.3f} deviates {delta:.3f} m "
            f"from expected={expected_z:.3f} (max={max_delta:.3f})"
        )
    return True, ""


def check_all_preflight(
    *,
    image_ok: bool,
    object_ok: bool,
    tf_ok: bool,
    bridge_ok: bool,
    one_shot_done: bool,
    stop_requested: bool,
) -> Tuple[bool, str]:
    """Return (ok, reason) for a full pre-flight check."""
    if stop_requested:
        return False, "STOP requested"
    if one_shot_done:
        return False, "one-shot already executed; press RESET to run again"
    if not image_ok:
        return False, "no valid camera image"
    if not object_ok:
        return False, "pick_demo not detected"
    if not tf_ok:
        return False, "TF not fresh"
    if not bridge_ok:
        return False, "MoveIt bridge not available"
    return True, ""
