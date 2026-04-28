#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pixel_geometry.py
# Contenido: Helpers de geometria pixel<->world<->table extraidos de panel_utils.
"""Helpers de geometria visual extraidos de panel_utils.py (refactor B.2).

Extraido de ``panel_utils.py`` (lineas 302-651 originales). Funciones puras
de mapeo entre pixeles de imagen, normalizado [-0.5, 0.5], coordenadas de
mesa y coordenadas world. Sin estado propio. Reexportadas por panel_utils
para mantener compatibilidad con todos los callers existentes
(panel_remote_callbacks, panel_motion_control, panel_calib_selection,
panel_draw_overlays, panel_tfm_science, panel_calib_actions, panel_tfm).

Funciones publicas:
- pixel_to_norm / norm_to_pixel
- compute_homography
- pixel_to_table_xy
- world_xyz_to_pixel / world_xyz_to_pixel_float
- table_xy_to_pixel / table_xy_to_pixel_float

Helpers privados:
- _pixel_to_norm / _norm_to_pixel
- _apply_homography / _invert_3x3 / _solve_linear_system
- _pixel_to_world_at_z
"""

from __future__ import annotations

from typing import List, Optional, Tuple

try:
    from .panel_config import (
        TABLE_CENTER_X,
        TABLE_CENTER_Y,
        TABLE_IMAGE_FLIP_X,
        TABLE_IMAGE_FLIP_Y,
        TABLE_IMAGE_SWAP_XY,
        TABLE_PIXEL_AFFINE,
        TABLE_PIXEL_HOMOGRAPHY,
        TABLE_PIXEL_RECT,
        TABLE_CAM_INFO,
        TABLE_SIZE_X,
        TABLE_SIZE_Y,
    )
except Exception:  # pragma: no cover
    from .panel_config import (  # type: ignore
        TABLE_CENTER_X,
        TABLE_CENTER_Y,
        TABLE_IMAGE_FLIP_X,
        TABLE_IMAGE_FLIP_Y,
        TABLE_IMAGE_SWAP_XY,
        TABLE_PIXEL_AFFINE,
        TABLE_PIXEL_HOMOGRAPHY,
        TABLE_PIXEL_RECT,
        TABLE_CAM_INFO,
        TABLE_SIZE_X,
        TABLE_SIZE_Y,
    )


# Las funciones extraidas se apenden aqui via sed.
# ------------------------------------------------------------------
# table calibration
# ------------------------------------------------------------------

def _pixel_to_norm(px: float, py: float, w: int, h: int) -> Tuple[float, float]:
    nx = (px / float(w)) - 0.5
    ny = (py / float(h)) - 0.5
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    return nx, ny


def _norm_to_pixel(nx: float, ny: float, w: int, h: int) -> Tuple[int, int]:
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    px = (nx + 0.5) * w
    py = (ny + 0.5) * h
    return int(max(0, min(w - 1, px))), int(max(0, min(h - 1, py)))


def pixel_to_norm(px: float, py: float, w: int, h: int) -> Tuple[float, float]:
    return _pixel_to_norm(px, py, w, h)


def norm_to_pixel(nx: float, ny: float, w: int, h: int) -> Tuple[int, int]:
    return _norm_to_pixel(nx, ny, w, h)


def _apply_homography(mat: List[List[float]], u: float, v: float) -> Optional[Tuple[float, float]]:
    denom = (mat[2][0] * u) + (mat[2][1] * v) + mat[2][2]
    if abs(denom) < 1e-8:
        return None
    x = ((mat[0][0] * u) + (mat[0][1] * v) + mat[0][2]) / denom
    y = ((mat[1][0] * u) + (mat[1][1] * v) + mat[1][2]) / denom
    return x, y


def _invert_3x3(mat: List[List[float]]) -> Optional[List[List[float]]]:
    a, b, c = mat[0]
    d, e, f = mat[1]
    g, h, i = mat[2]
    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(det) < 1e-10:
        return None
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def _solve_linear_system(a: List[List[float]], b: List[float]) -> Optional[List[float]]:
    n = len(b)
    mat = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = col
        max_val = abs(mat[col][col])
        for r in range(col + 1, n):
            val = abs(mat[r][col])
            if val > max_val:
                max_val = val
                pivot = r
        if max_val < 1e-10:
            return None
        if pivot != col:
            mat[col], mat[pivot] = mat[pivot], mat[col]
        pivot_val = mat[col][col]
        for c in range(col, n + 1):
            mat[col][c] /= pivot_val
        for r in range(n):
            if r == col:
                continue
            factor = mat[r][col]
            if abs(factor) < 1e-12:
                continue
            for c in range(col, n + 1):
                mat[r][c] -= factor * mat[col][c]
    return [mat[i][n] for i in range(n)]


def compute_homography(
    pixels: List[Tuple[float, float]],
    worlds: List[Tuple[float, float]],
) -> Optional[List[List[float]]]:
    if len(pixels) < 4 or len(worlds) < 4:
        return None
    pts = list(zip(pixels, worlds))[:4]
    a: List[List[float]] = []
    b: List[float] = []
    for (u, v), (x, y) in pts:
        a.append([u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v])
        b.append(x)
        a.append([0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v])
        b.append(y)
    sol = _solve_linear_system(a, b)
    if not sol:
        return None
    return [
        [sol[0], sol[1], sol[2]],
        [sol[3], sol[4], sol[5]],
        [sol[6], sol[7], 1.0],
    ]


def _pixel_to_world_at_z(px: float, py: float, w: int, h: int, z_target: float) -> Optional[Tuple[float, float]]:
    # See world_xyz_to_pixel_float: TABLE_CAM_INFO is updated at runtime in
    # panel_utils via `global`; read live to honour calibration JSON.
    try:
        from .panel_utils import TABLE_CAM_INFO as _LIVE_CAM_INFO
    except Exception:
        _LIVE_CAM_INFO = None
    cam = _LIVE_CAM_INFO if _LIVE_CAM_INFO else TABLE_CAM_INFO
    if not cam:
        return None
    pos = cam.get("position")
    rot = cam.get("rotation")
    fx = cam.get("fx")
    fy = cam.get("fy")
    cx = cam.get("cx")
    cy = cam.get("cy")
    cam_w = cam.get("width")
    cam_h = cam.get("height")
    if (
        not isinstance(pos, list) or len(pos) != 3
        or not isinstance(rot, list) or len(rot) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in rot)
    ):
        return None
    try:
        fx = float(fx)
        fy = float(fy)
        cx = float(cx)
        cy = float(cy)
    except (TypeError, ValueError):
        return None
    cam_w = int(cam_w) if isinstance(cam_w, (int, float)) else w
    cam_h = int(cam_h) if isinstance(cam_h, (int, float)) else h
    if cam_w > 0 and cam_h > 0 and (cam_w != w or cam_h != h):
        sx = w / float(cam_w)
        sy = h / float(cam_h)
        fx *= sx
        fy *= sy
        cx *= sx
        cy *= sy
    xcam = 1.0
    ycam = -(px - cx) / fx
    zcam = -(py - cy) / fy
    dir_world = (
        rot[0][0] * xcam + rot[0][1] * ycam + rot[0][2] * zcam,
        rot[1][0] * xcam + rot[1][1] * ycam + rot[1][2] * zcam,
        rot[2][0] * xcam + rot[2][1] * ycam + rot[2][2] * zcam,
    )
    try:
        ox, oy, oz = float(pos[0]), float(pos[1]), float(pos[2])
    except (TypeError, ValueError):
        return None
    dz = dir_world[2]
    if abs(dz) < 1e-8:
        return None
    t = (float(z_target) - oz) / dz
    if t <= 0.0:
        return None
    xw = ox + t * dir_world[0]
    yw = oy + t * dir_world[1]
    return xw, yw


def pixel_to_table_xy(px: int, py: int, w: int, h: int, z_target: Optional[float] = None) -> Tuple[float, float]:
    if w <= 0 or h <= 0:
        return 0.0, 0.0
    if z_target is not None:
        out = _pixel_to_world_at_z(float(px), float(py), w, h, float(z_target))
        if out:
            return out
    if TABLE_PIXEL_HOMOGRAPHY:
        nx, ny = _pixel_to_norm(float(px), float(py), w, h)
        out = _apply_homography(TABLE_PIXEL_HOMOGRAPHY, nx, ny)
        if out:
            return out
    if TABLE_PIXEL_AFFINE:
        a, b, c = TABLE_PIXEL_AFFINE[0]
        d, e, f = TABLE_PIXEL_AFFINE[1]
        x = (a * px) + (b * py) + c
        y = (d * px) + (e * py) + f
        return x, y
    if TABLE_PIXEL_RECT:
        px1, py1 = TABLE_PIXEL_RECT["p1"]
        px2, py2 = TABLE_PIXEL_RECT["p2"]
        x1, y1 = TABLE_PIXEL_RECT["w1"]
        x2, y2 = TABLE_PIXEL_RECT["w2"]
        nx, ny = _pixel_to_norm(px, py, w, h)
        npx1, npy1 = _pixel_to_norm(px1, py1, w, h)
        npx2, npy2 = _pixel_to_norm(px2, py2, w, h)
        sx = (x2 - x1) / max(1e-6, (npx2 - npx1))
        sy = (y2 - y1) / max(1e-6, (npy2 - npy1))
        x = x1 + (nx - npx1) * sx
        y = y1 + (ny - npy1) * sy
        return x, y
    nx = (px / float(w)) - 0.5
    ny = (py / float(h)) - 0.5
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    x = TABLE_CENTER_X + nx * TABLE_SIZE_X
    y = TABLE_CENTER_Y + ny * TABLE_SIZE_Y
    return x, y


def world_xyz_to_pixel(x: float, y: float, z: float, w: int, h: int) -> Optional[Tuple[int, int]]:
    out = world_xyz_to_pixel_float(x, y, z, w, h)
    if not out:
        return None
    u, v = out
    return int(max(0, min(w - 1, u))), int(max(0, min(h - 1, v)))


def world_xyz_to_pixel_float(x: float, y: float, z: float, w: int, h: int) -> Optional[Tuple[float, float]]:
    if w <= 0 or h <= 0:
        return None
    # load_table_calib() updates panel_utils.TABLE_CAM_INFO via `global`, but the
    # module-level `from .panel_config import TABLE_CAM_INFO` here was bound at
    # import time to None and never refreshed. Look up the live value from
    # panel_utils each call so the calibration JSON actually takes effect.
    try:
        from .panel_utils import TABLE_CAM_INFO as _LIVE_CAM_INFO
    except Exception:
        _LIVE_CAM_INFO = None
    cam = _LIVE_CAM_INFO if _LIVE_CAM_INFO else TABLE_CAM_INFO
    if not cam:
        return None
    pos = cam.get("position")
    rot = cam.get("rotation")
    fx = cam.get("fx")
    fy = cam.get("fy")
    cx = cam.get("cx")
    cy = cam.get("cy")
    cam_w = cam.get("width")
    cam_h = cam.get("height")
    if (
        not isinstance(pos, list) or len(pos) != 3
        or not isinstance(rot, list) or len(rot) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in rot)
    ):
        return None
    try:
        fx = float(fx)
        fy = float(fy)
        cx = float(cx)
        cy = float(cy)
    except (TypeError, ValueError):
        return None
    cam_w = int(cam_w) if isinstance(cam_w, (int, float)) else w
    cam_h = int(cam_h) if isinstance(cam_h, (int, float)) else h
    if cam_w > 0 and cam_h > 0 and (cam_w != w or cam_h != h):
        sx = w / float(cam_w)
        sy = h / float(cam_h)
        fx *= sx
        fy *= sy
        cx *= sx
        cy *= sy
    try:
        ox, oy, oz = float(pos[0]), float(pos[1]), float(pos[2])
    except (TypeError, ValueError):
        return None
    vx = x - ox
    vy = y - oy
    vz = z - oz
    xcam = rot[0][0] * vx + rot[1][0] * vy + rot[2][0] * vz
    ycam = rot[0][1] * vx + rot[1][1] * vy + rot[2][1] * vz
    zcam = rot[0][2] * vx + rot[1][2] * vy + rot[2][2] * vz
    if xcam <= 1e-6:
        return None
    u = cx - (ycam / xcam) * fx
    v = cy - (zcam / xcam) * fy
    return max(0.0, min(float(w - 1), float(u))), max(0.0, min(float(h - 1), float(v)))


def table_xy_to_pixel(x: float, y: float, w: int, h: int) -> Optional[Tuple[int, int]]:
    out = table_xy_to_pixel_float(x, y, w, h)
    if not out:
        return None
    px, py = out
    return int(max(0, min(w - 1, px))), int(max(0, min(h - 1, py)))


def table_xy_to_pixel_float(x: float, y: float, w: int, h: int) -> Optional[Tuple[float, float]]:
    if w <= 0 or h <= 0:
        return None
    if TABLE_PIXEL_HOMOGRAPHY:
        inv = _invert_3x3(TABLE_PIXEL_HOMOGRAPHY)
        if not inv:
            return None
        out = _apply_homography(inv, float(x), float(y))
        if out:
            nx, ny = out
            # Keep the inverse homography path consistent with _norm_to_pixel.
            # Without these flips, the calibration grid can appear vertically
            # or horizontally mirrored even if pixel->world is correct.
            if TABLE_IMAGE_FLIP_Y:
                ny = -ny
            if TABLE_IMAGE_FLIP_X:
                nx = -nx
            if TABLE_IMAGE_SWAP_XY:
                nx, ny = ny, nx
            px = (nx + 0.5) * w
            py = (ny + 0.5) * h
            return max(0.0, min(float(w - 1), float(px))), max(0.0, min(float(h - 1), float(py)))
        return None
    if TABLE_PIXEL_AFFINE:
        a, b, c = TABLE_PIXEL_AFFINE[0]
        d, e, f = TABLE_PIXEL_AFFINE[1]
        det = (a * e) - (b * d)
        if abs(det) < 1e-8:
            return None
        inv = [[e / det, -b / det], [-d / det, a / det]]
        tx = x - c
        ty = y - f
        px = (inv[0][0] * tx) + (inv[0][1] * ty)
        py = (inv[1][0] * tx) + (inv[1][1] * ty)
        return max(0.0, min(float(w - 1), float(px))), max(0.0, min(float(h - 1), float(py)))
    if TABLE_PIXEL_RECT:
        px1, py1 = TABLE_PIXEL_RECT["p1"]
        px2, py2 = TABLE_PIXEL_RECT["p2"]
        x1, y1 = TABLE_PIXEL_RECT["w1"]
        x2, y2 = TABLE_PIXEL_RECT["w2"]
        npx1, npy1 = _pixel_to_norm(px1, py1, w, h)
        npx2, npy2 = _pixel_to_norm(px2, py2, w, h)
        sx = (npx2 - npx1) / max(1e-6, (x2 - x1))
        sy = (npy2 - npy1) / max(1e-6, (y2 - y1))
        nx = npx1 + (x - x1) * sx
        ny = npy1 + (y - y1) * sy
        if TABLE_IMAGE_SWAP_XY:
            nx, ny = ny, nx
        px = (nx + 0.5) * w
        py = (ny + 0.5) * h
        return max(0.0, min(float(w - 1), float(px))), max(0.0, min(float(h - 1), float(py)))
    nx = (x - TABLE_CENTER_X) / max(1e-6, TABLE_SIZE_X)
    ny = (y - TABLE_CENTER_Y) / max(1e-6, TABLE_SIZE_Y)
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    px = (nx + 0.5) * w
    py = (ny + 0.5) * h
    return max(0.0, min(float(w - 1), float(px))), max(0.0, min(float(h - 1), float(py)))

