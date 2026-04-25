"""TF2 utilities for visual_autopick: lookup_transform and transform_point."""
from __future__ import annotations

import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException


def lookup_transform(
    tf_buffer: Buffer,
    target_frame: str,
    source_frame: str,
    *,
    max_age_sec: float = 0.5,
    node: Optional[Node] = None,
):
    """Return (transform, age_sec, error_str).

    transform is a geometry_msgs/TransformStamped or None on failure.
    age_sec is float or None.
    error_str is '' on success.
    """
    try:
        t = tf_buffer.lookup_transform(target_frame, source_frame, Time())
    except (LookupException, ConnectivityException, ExtrapolationException) as exc:
        return None, None, str(exc)

    # Compute age
    now_ns = time.time_ns()
    stamp = t.header.stamp
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    age_sec = (now_ns - stamp_ns) / 1e9

    if age_sec > max_age_sec:
        return t, age_sec, f"STALE age_sec={age_sec:.3f} > max={max_age_sec}"

    return t, age_sec, ""


def transform_point(
    tf_buffer: Buffer,
    target_frame: str,
    source_frame: str,
    point: Tuple[float, float, float],
    *,
    max_age_sec: float = 0.5,
) -> Tuple[Optional[Tuple[float, float, float]], str]:
    """Transform a (x,y,z) point from source_frame to target_frame.

    Returns ((tx, ty, tz), '') on success or (None, error_str) on failure.
    """
    t, age_sec, err = lookup_transform(
        tf_buffer, target_frame, source_frame, max_age_sec=max_age_sec
    )
    if t is None or err:
        return None, err or "transform lookup failed"

    # Manual 3-D point transform using the TransformStamped
    tr = t.transform.translation
    rot = t.transform.rotation

    # Rotate point by quaternion then add translation
    qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
    px, py, pz = point

    # v' = q * v * q^-1  (using formula for rotating a vector by a unit quaternion)
    ix = qw * px + qy * pz - qz * py
    iy = qw * py + qz * px - qx * pz
    iz = qw * pz + qx * py - qy * px
    iw = -qx * px - qy * py - qz * pz

    rx = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy)
    ry = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz)
    rz = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx)

    return (rx + tr.x, ry + tr.y, rz + tr.z), ""
