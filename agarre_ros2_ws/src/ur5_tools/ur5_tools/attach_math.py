# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/attach_math.py
# Contenido: Helpers puros de quaternion + matriz extraidos de gripper_attach_backend.
"""Helpers puros de quaternion/rotacion/DH (refactor C.1).

Extraido de ``gripper_attach_backend.py`` (lineas 137-267 originales).
Funciones matematicas puras sin dependencias de ROS o de la clase
``GripperAttachBackend``. Reexportado desde gripper_attach_backend
para mantener compatibilidad si algo accede a ``module._quat_*``.

Funciones publicas:
- _quat_normalize, _quat_inverse, _quat_multiply_raw, _quat_multiply
- _rotate_vector
- _matmul3, _matvec3
- _dh_transform
- _quat_from_rot3
"""

from __future__ import annotations

import math
from typing import Tuple


def _quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_inverse(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = _quat_normalize(q)
    return (-x, -y, -z, w)


def _quat_multiply_raw(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_multiply(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    return _quat_normalize(_quat_multiply_raw(a, b))


def _rotate_vector(
    q: Tuple[float, float, float, float],
    v: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    qx, qy, qz, qw = _quat_normalize(q)
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


def _matmul3(
    a: Tuple[Tuple[float, float, float], ...],
    b: Tuple[Tuple[float, float, float], ...],
) -> Tuple[Tuple[float, float, float], ...]:
    return tuple(
        tuple(
            a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            for j in range(3)
        )
        for i in range(3)
    )


def _matvec3(
    a: Tuple[Tuple[float, float, float], ...],
    v: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    return (
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    )


def _dh_transform(
    a: float, d: float, alpha: float, theta: float
) -> Tuple[Tuple[Tuple[float, float, float], ...], Tuple[float, float, float]]:
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ct = math.cos(theta)
    st = math.sin(theta)
    rot = (
        (ct, -st * ca, st * sa),
        (st, ct * ca, -ct * sa),
        (0.0, sa, ca),
    )
    pos = (
        a * ct,
        a * st,
        d,
    )
    return rot, pos


def _quat_from_rot3(
    rot: Tuple[Tuple[float, float, float], ...],
) -> Tuple[float, float, float, float]:
    m00, m01, m02 = rot[0]
    m10, m11, m12 = rot[1]
    m20, m21, m22 = rot[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return _quat_normalize((qx, qy, qz, qw))
