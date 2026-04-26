#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_draw_overlays.py
# Contenido: Camera overlay drawing methods extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Camera overlay drawing callbacks for ControlPanelV2."""
from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from .panel_process import ensure_dir
from .panel_utils import base_to_world, table_xy_to_pixel, table_xy_to_pixel_float, transform_point_to_frame, world_to_base

try:
    import numpy as np
except ImportError:
    np = None  # type: ignore

from PyQt5.QtGui import QImage

from .panel_config import (
    OVERLAY_ANTIALIAS,
    OVERLAY_CALIB,
    OVERLAY_REACH,
    OVERLAY_SELECTION,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    TABLE_SIZE_X,
    TABLE_SIZE_Y,
)
from .panel_utils import (
    world_xyz_to_pixel,
    world_xyz_to_pixel_float,
)
from .panel_config import TEST_CORNER_OVERLAY, TCP_POSE_TEXT_OVERLAY
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME
from .tf_pose_utils import get_tcp_in_base as tf_get_tcp_in_base


def _log_exception(context: str, exc: Exception) -> None:
    print(f"[DRAW][ERROR][{context}] {exc}")


def _compute_homography(pixel_points: List[Tuple[float, float]], world_points: List[Tuple[float, float]]):
    if len(pixel_points) < 4 or len(world_points) < 4:
        return None
    a_rows = []
    for (u, v), (x, y) in zip(pixel_points, world_points):
        a_rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, u * x, v * x, x])
        a_rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, u * y, v * y, y])
    mat = np.array(a_rows, dtype=float)
    _, _, v_t = np.linalg.svd(mat)
    h = v_t[-1].reshape((3, 3))
    if abs(h[2, 2]) > 1e-8:
        h = h / h[2, 2]
    return h.tolist()

def _draw_calib_overlay(panel, qimg: QImage, w: int, h: int) -> QImage:
    """Dibujar malla y puntos de calibración sobre la imagen."""
    from PyQt5.QtGui import QPainter, QPen, QColor
    from PyQt5.QtCore import Qt, QPointF

    # Copiar imagen para no modificar original
    img_copy = qimg.copy()
    painter = QPainter(img_copy)
    if OVERLAY_ANTIALIAS:
        painter.setRenderHint(QPainter.Antialiasing)

    try:
        if time.time() <= panel._calib_grid_until:
            # Dibujar malla de calibración (grid) - IGUAL A PANEL ONLY
            # Usar coordenadas de mundo (0.025m steps) + tabla_xy_to_pixel para convertir a píxeles
            pen = QPen(QColor(30, 64, 175, 180))
            pen.setWidth(2)
            painter.setPen(pen)

            step_x = 0.025  # metros
            step_y = 0.025  # metros
            x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
            x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
            y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
            y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)

            # Líneas verticales (X constante)
            x = x_min
            while x <= (x_max + 1e-6):
                p0 = table_xy_to_pixel(x, y_min, w, h)
                p1 = table_xy_to_pixel(x, y_max, w, h)
                if p0 and p1:
                    painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                x += step_x

            # Líneas horizontales (Y constante)
            y = y_min
            while y <= (y_max + 1e-6):
                p0 = table_xy_to_pixel(x_min, y, w, h)
                p1 = table_xy_to_pixel(x_max, y, w, h)
                if p0 and p1:
                    painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                y += step_y

            # Contorno de la mesa para mayor contraste
            p_tl = table_xy_to_pixel(x_min, y_max, w, h)
            p_tr = table_xy_to_pixel(x_max, y_max, w, h)
            p_br = table_xy_to_pixel(x_max, y_min, w, h)
            p_bl = table_xy_to_pixel(x_min, y_min, w, h)
            if p_tl and p_tr and p_br and p_bl:
                edge_pen = QPen(QColor(16, 185, 129, 200))
                edge_pen.setWidth(2)
                painter.setPen(edge_pen)
                painter.drawLine(QPointF(p_tl[0], p_tl[1]), QPointF(p_tr[0], p_tr[1]))
                painter.drawLine(QPointF(p_tr[0], p_tr[1]), QPointF(p_br[0], p_br[1]))
                painter.drawLine(QPointF(p_br[0], p_br[1]), QPointF(p_bl[0], p_bl[1]))
                painter.drawLine(QPointF(p_bl[0], p_bl[1]), QPointF(p_tl[0], p_tl[1]))

            # Etiquetas básicas de ejes para visibilidad (mismo estilo Panel Only)
            painter.setPen(QPen(QColor(30, 64, 175, 160)))
            for label_x in (TABLE_CENTER_X - 0.4, TABLE_CENTER_X, TABLE_CENTER_X + 0.4):
                p = table_xy_to_pixel(label_x, y_min, w, h)
                if p:
                    painter.drawText(p[0] + 3, p[1] + 12, f"x={label_x:.1f}")
            for label_y in (TABLE_CENTER_Y - 0.3, TABLE_CENTER_Y, TABLE_CENTER_Y + 0.3):
                p = table_xy_to_pixel(x_min, label_y, w, h)
                if p:
                    painter.drawText(p[0] + 3, p[1] - 3, f"y={label_y:.1f}")

        # Dibujar cada punto de calibración
        for i, (px, py) in enumerate(panel._calib_points):
            # Cruz roja
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            size = 10
            painter.drawLine(px - size, py, px + size, py)
            painter.drawLine(px, py - size, px, py + size)

            # Círculo exterior
            painter.setPen(QPen(QColor(255, 255, 0), 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(px, py), 8, 8)

            # Texto con número
            painter.setPen(QPen(QColor(0, 255, 0), 1))
            painter.drawText(QPointF(px + 12, py - 8), f"P{i+1}")

        def _draw_arrow(p0, p1, color: QColor, label: str):
            if not p0 or not p1:
                return
            painter.setPen(QPen(color, 2))
            painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
            vx = p1[0] - p0[0]
            vy = p1[1] - p0[1]
            norm = math.hypot(vx, vy)
            if norm > 1e-3:
                ux = vx / norm
                uy = vy / norm
                size = 8.0
                perp = (-uy, ux)
                a1 = (p1[0] - ux * size + perp[0] * (size / 2.0), p1[1] - uy * size + perp[1] * (size / 2.0))
                a2 = (p1[0] - ux * size - perp[0] * (size / 2.0), p1[1] - uy * size - perp[1] * (size / 2.0))
                painter.drawLine(QPointF(p1[0], p1[1]), QPointF(a1[0], a1[1]))
                painter.drawLine(QPointF(p1[0], p1[1]), QPointF(a2[0], a2[1]))
            painter.setPen(QPen(color, 1))
            painter.drawText(QPointF(p1[0] + 4.0, p1[1] - 4.0), label)

        # Dibujar ejes usando la proyeccion real de la calibracion.
        # El icono fijo en esquina solo queda como fallback cuando no hay
        # una proyeccion geometrica util.
        table_top = panel._resolve_table_top_z()
        axis_span = min(TABLE_SIZE_X, TABLE_SIZE_Y) * 0.18
        base = panel._world_to_pixel(TABLE_CENTER_X, TABLE_CENTER_Y, table_top, w, h)
        x_tip = panel._world_to_pixel(TABLE_CENTER_X + axis_span, TABLE_CENTER_Y, table_top, w, h)
        y_tip = panel._world_to_pixel(TABLE_CENTER_X, TABLE_CENTER_Y + axis_span, table_top, w, h)
        z_tip = world_xyz_to_pixel(TABLE_CENTER_X, TABLE_CENTER_Y, table_top + axis_span, w, h)

        projected_axes_ok = bool(
            base
            and x_tip
            and y_tip
            and math.hypot(float(x_tip[0]) - float(base[0]), float(x_tip[1]) - float(base[1])) > 8.0
            and math.hypot(float(y_tip[0]) - float(base[0]), float(y_tip[1]) - float(base[1])) > 8.0
        )

        if projected_axes_ok:
            _draw_arrow(base, x_tip, QColor(239, 68, 68), "X+")
            _draw_arrow(base, y_tip, QColor(34, 197, 94), "Y+")
            if z_tip and math.hypot(float(z_tip[0]) - float(base[0]), float(z_tip[1]) - float(base[1])) > 8.0:
                _draw_arrow(base, z_tip, QColor(59, 130, 246), "Z+")
        else:
            axis_len_px = max(26.0, min(w, h) * 0.07)
            margin = 16.0
            base = (margin, margin + axis_len_px * 1.2)
            x_tip = (base[0] + axis_len_px, base[1])
            y_tip = (base[0], base[1] - axis_len_px)
            z_tip = (base[0] - axis_len_px * 0.5, base[1] - axis_len_px * 0.7)
            _draw_arrow(base, x_tip, QColor(239, 68, 68), "X+")
            _draw_arrow(base, y_tip, QColor(34, 197, 94), "Y+")
            _draw_arrow(base, z_tip, QColor(59, 130, 246), "Z+")
    finally:
        painter.end()
    return img_copy

def _draw_selection_overlay(panel, qimg: QImage, w: int, h: int) -> QImage:
    """Dibujar selección actual sobre la imagen."""
    from PyQt5.QtGui import QPainter, QPen, QColor
    from PyQt5.QtCore import Qt, QPointF
    
    if not panel._selected_px:
        return qimg
    
    px, py = panel._selected_px
    img_copy = qimg.copy()
    painter = QPainter(img_copy)
    if OVERLAY_ANTIALIAS:
        painter.setRenderHint(QPainter.Antialiasing)
    
    color = QColor(34, 197, 94)
    painter.setPen(QPen(color, 1))
    painter.setBrush(Qt.NoBrush)
    painter.drawEllipse(QPointF(px, py), 9, 9)
    
    painter.drawLine(px - 10, py, px + 10, py)
    painter.drawLine(px, py - 10, px, py + 10)
    
    # Texto con coordenadas de negocio en base_link.
    if panel._selected_base:
        bx, by, _bz = panel._selected_base
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawText(px + 22, py, f"base({bx:.3f}, {by:.3f})")
    
    painter.end()
    panel._emit_log_throttled(
        "PICK:overlay",
        f"[PICK][OVERLAY] draw px=({int(px)},{int(py)}) base={'yes' if bool(panel._selected_base) else 'no'}",
    )
    return img_copy

def _draw_grasp_overlay(panel, qimg: QImage, w: int, h: int) -> QImage:
    """Dibujar overlay de grasp siguiendo la convención visual del TFM.

    Convención:
    - Predicción / grasp inferido → rojo continuo
    - GT / referencia             → verde discontinuo
    """
    from PyQt5.QtGui import QPainter, QPen, QColor
    from PyQt5.QtCore import Qt, QPointF

    if not panel._last_grasp_px:
        return qimg
    img_copy = qimg.copy()
    painter = QPainter(img_copy)
    if OVERLAY_ANTIALIAS:
        painter.setRenderHint(QPainter.Antialiasing)

    def _draw_rect_grasp(
        grasp: Dict[str, float],
        *,
        long_color: QColor,
        short_color: QColor,
        center_color: QColor,
        tag: str,
        pen_style=Qt.SolidLine,
    ) -> None:
        cx = float(grasp.get("cx", 0.0))
        cy = float(grasp.get("cy", 0.0))
        gw = float(grasp.get("w", 0.0))
        gh = float(grasp.get("h", 0.0))
        angle_deg = float(grasp.get("angle_deg", 0.0))
        if gw <= 0.0:
            return
        if gh <= 0.0:
            gh = max(4.0, gw * 0.25)

        hw = gw / 2.0
        hh = gh / 2.0
        ang = math.radians(angle_deg)
        cs = math.cos(ang)
        sn = math.sin(ang)

        def _rot(dx: float, dy: float) -> QPointF:
            return QPointF(cx + dx * cs - dy * sn, cy + dx * sn + dy * cs)

        p0 = _rot(-hw, -hh)
        p1 = _rot(hw, -hh)
        p2 = _rot(hw, hh)
        p3 = _rot(-hw, hh)

        painter.setPen(QPen(long_color, 2, pen_style))
        painter.drawLine(p0, p1)
        painter.drawLine(p3, p2)

        painter.setPen(QPen(short_color, 3, pen_style))
        painter.drawLine(p1, p2)
        painter.drawLine(p0, p3)

        painter.setPen(QPen(center_color, 2, pen_style))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), 3, 3)

        painter.setPen(QPen(center_color, 1, pen_style))
        painter.drawText(QPointF(cx + 6, cy - 6), f"{tag} {angle_deg:.0f}°")

    _draw_rect_grasp(
        panel._last_grasp_px,
        long_color=QColor(239, 68, 68),
        short_color=QColor(239, 68, 68),
        center_color=QColor(255, 255, 255),
        tag="P",
    )

    if panel._tfm_visual_compare_enabled:
        ref = panel._build_reference_grasp(w, h)
        if ref:
            _draw_rect_grasp(
                ref,
                long_color=QColor(34, 197, 94),
                short_color=QColor(34, 197, 94),
                center_color=QColor(34, 197, 94),
                tag="R",
                pen_style=Qt.DashLine,
            )

    painter.end()
    return img_copy

def _tfm_overlay_focus_enabled(panel) -> bool:
    return bool(getattr(panel, "_tfm_overlay_focus_active", False))

def _should_draw_reach_overlay(panel, overhead_only: bool) -> bool:
    return bool(
        OVERLAY_REACH
        and overhead_only
        and panel._reach_overlay_enabled
        and not panel._tfm_overlay_focus_enabled()
    )

def _should_draw_selection_overlay(panel, overhead_only: bool) -> bool:
    return bool(
        OVERLAY_SELECTION
        and overhead_only
        and panel._selected_px
        and not panel._tfm_overlay_focus_enabled()
    )

def _should_draw_test_corner_overlay(panel) -> bool:
    return bool(TEST_CORNER_OVERLAY and not panel._tfm_overlay_focus_enabled())

def _should_draw_tcp_pose_overlay(panel, overhead_only: bool) -> bool:
    return bool(overhead_only and not panel._tfm_overlay_focus_enabled())

def _base_to_world_coords(panel,
    coords_base: Tuple[float, float, float],
    *,
    world_frame: Optional[str] = None,
    timeout_sec: float = 0.35,
) -> Optional[Tuple[float, float, float]]:
    """
    Transform coordenadas de base_link a world.
    FASE 3: Timeout aumentado y mejor fallback logging.
    """
    base_frame = panel._business_base_frame()
    target_world = world_frame or panel._world_frame_last_first()
    # FASE 3: Timeout aumentado de 0.35 a 1.0s para reducir fallos TF
    coords, _ = transform_point_to_frame(
        (float(coords_base[0]), float(coords_base[1]), float(coords_base[2])),
        target_world,
        source_frame=base_frame,
        timeout_sec=max(timeout_sec, 1.0),
    )
    if coords:
        return (float(coords[0]), float(coords[1]), float(coords[2]))
    # Render fallback only con logging
    panel._emit_log(
        f"[TEST][FASE3] TF fallback usado para transform base->world (puede causar imprecisión)"
    )
    wx, wy, wz = base_to_world(
        float(coords_base[0]), float(coords_base[1]), float(coords_base[2])
    )
    return (float(wx), float(wy), float(wz))

def _compute_test_corner_base_points(panel,
    *,
    inset_m: Optional[float] = None,
    z_override: Optional[float] = None,
) -> List[Tuple[str, Tuple[float, float, float]]]:
    """Return two front corner target points on the table in base_link."""
    x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
    x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
    y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
    y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)
    table_z = float(z_override) if z_override is not None else float(panel._resolve_table_top_z())
    if inset_m is None:
        try:
            inset_m = float(os.environ.get("PANEL_TEST_CORNER_INSET_M", "0.06"))
        except Exception:
            inset_m = 0.06
    inset_m = max(0.0, float(inset_m))
    inset_x = min(max(0.02, inset_m), max(0.02, (x_max - x_min) * 0.24))
    inset_y = min(max(0.02, inset_m), max(0.02, (y_max - y_min) * 0.24))

    # Front edge = nearest edge to robot origin in base_link (no dependencia de get_pose(world,base)).
    edge_name = "x_min"
    world_frame = panel._world_frame_last_first()
    edge_centers_world = {
        "x_min": (x_min, TABLE_CENTER_Y),
        "x_max": (x_max, TABLE_CENTER_Y),
        "y_min": (TABLE_CENTER_X, y_min),
        "y_max": (TABLE_CENTER_X, y_max),
    }
    edge_centers_base: Dict[str, Tuple[float, float, float]] = {}
    for edge_key, (wx, wy) in edge_centers_world.items():
        # FASE 3: Timeout aumentado de 0.35 a 1.0s
        center_base = panel._ensure_base_coords((wx, wy, table_z), world_frame, timeout_sec=1.0)
        if center_base is None:
            try:
                center_base = world_to_base(float(wx), float(wy), float(table_z))
            except Exception:
                center_base = None
        if center_base is not None:
            edge_centers_base[edge_key] = (
                float(center_base[0]),
                float(center_base[1]),
                float(center_base[2]),
            )
    if edge_centers_base:
        edge_name = min(
            edge_centers_base.keys(),
            key=lambda name: math.hypot(
                float(edge_centers_base[name][0]),
                float(edge_centers_base[name][1]),
            ),
        )

    if edge_name == "x_min":
        raw_points = [
            (x_min + inset_x, y_min + inset_y, table_z),
            (x_min + inset_x, y_max - inset_y, table_z),
        ]
    elif edge_name == "x_max":
        raw_points = [
            (x_max - inset_x, y_min + inset_y, table_z),
            (x_max - inset_x, y_max - inset_y, table_z),
        ]
    elif edge_name == "y_min":
        raw_points = [
            (x_min + inset_x, y_min + inset_y, table_z),
            (x_max - inset_x, y_min + inset_y, table_z),
        ]
    else:
        raw_points = [
            (x_min + inset_x, y_max - inset_y, table_z),
            (x_max - inset_x, y_max - inset_y, table_z),
        ]

    base_points: List[Tuple[float, float, float]] = []
    for point in raw_points:
        # FASE 3: Timeout aumentado de 0.35 a 1.0s
        converted = panel._ensure_base_coords(point, world_frame, timeout_sec=1.0)
        if converted is None:
            try:
                converted = world_to_base(
                    float(point[0]), float(point[1]), float(point[2])
            )
            except Exception:
                converted = None
        if converted is not None:
            base_points.append(converted)
    if len(base_points) < 2:
        return []
    # Stable labels in base: larger base-Y is left.
    ordered = sorted(base_points, key=lambda p: p[1], reverse=True)
    return [
        ("FRONT_LEFT", (float(ordered[0][0]), float(ordered[0][1]), float(ordered[0][2]))),
        ("FRONT_RIGHT", (float(ordered[1][0]), float(ordered[1][1]), float(ordered[1][2]))),
    ]

def _compute_test_corner_world_points(panel,
    *,
    inset_m: Optional[float] = None,
    z_override: Optional[float] = None,
) -> List[Tuple[str, Tuple[float, float, float]]]:
    """Render helper: convert base_link test points to world for image overlay."""
    base_points = panel._compute_test_corner_base_points(
        inset_m=inset_m,
        z_override=z_override,
    )
    world_frame = panel._world_frame_last_first()
    out: List[Tuple[str, Tuple[float, float, float]]] = []
    for label, base_pt in base_points:
        world_pt = panel._base_to_world_coords(base_pt, world_frame=world_frame)
        if world_pt is not None:
            out.append((label, world_pt))
    return out

def _draw_test_corner_overlay(panel, qimg: QImage, w: int, h: int) -> QImage:
    """Draw two corner marks on the table for TEST ROBOT positioning."""
    from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
    from PyQt5.QtCore import Qt, QPointF

    points = panel._compute_test_corner_world_points()
    if not points:
        return qimg
    img_copy = qimg.copy()
    painter = QPainter(img_copy)
    if OVERLAY_ANTIALIAS:
        painter.setRenderHint(QPainter.Antialiasing)
    try:
        pen = QPen(QColor(245, 158, 11, 230), 2)
        brush = QBrush(QColor(245, 158, 11, 90))
        painter.setPen(pen)
        painter.setBrush(brush)
        line_pts: List[Tuple[int, int]] = []
        drawn = 0
        for idx, (label, (wx, wy, wz)) in enumerate(points, start=1):
            pix = world_xyz_to_pixel(wx, wy, wz, w, h)
            if not pix:
                pix = table_xy_to_pixel(wx, wy, w, h)
            if not pix:
                continue
            px, py = int(pix[0]), int(pix[1])
            line_pts.append((px, py))
            painter.drawEllipse(QPointF(px, py), 10.0, 10.0)
            painter.drawLine(px - 13, py, px + 13, py)
            painter.drawLine(px, py - 13, px, py + 13)
            painter.setPen(QPen(QColor(255, 255, 255, 240), 1))
            painter.drawText(QPointF(px + 12.0, py - 8.0), f"T{idx}:{label}")
            painter.setPen(pen)
            drawn += 1
        if len(line_pts) == 2:
            painter.setPen(QPen(QColor(245, 158, 11, 180), 2, Qt.DashLine))
            painter.drawLine(
                QPointF(float(line_pts[0][0]), float(line_pts[0][1])),
                QPointF(float(line_pts[1][0]), float(line_pts[1][1])),
            )
        if drawn <= 0:
            return qimg
    finally:
        painter.end()
    return img_copy

def _draw_tcp_pose_overlay(panel, qimg: QImage, w: int, h: int) -> QImage:
    """Draw a live TCP pose HUD over the camera view."""
    from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
    from PyQt5.QtCore import QRectF, QPointF

    img_copy = qimg.copy()
    painter = QPainter(img_copy)
    if OVERLAY_ANTIALIAS:
        painter.setRenderHint(QPainter.Antialiasing)
    try:
        tcp_base = None
        tcp_source = "none"
        base_frame = panel._business_base_frame()
        # Usar rg2_tcp para el dot de cámara: es el frame visual de la pinza (tip/TCP),
        # que corresponde mejor a lo que la cámara muestra visualmente.
        # El frame operacional (rg2_pinch_center) se sigue usando para IK y grasping —
        # este cambio solo afecta la posición del marcador en el overlay de cámara.
        ee_frame = "rg2_tcp"
        tcp_pose_base, rpy, tcp_reason = tf_get_tcp_in_base(
            base_frame=base_frame,
            ee_frame=ee_frame,
            timeout=0.03,
            logger=None,
        )
        if tcp_pose_base is not None:
            tcp_base = (
                float(tcp_pose_base.pose.position.x),
                float(tcp_pose_base.pose.position.y),
                float(tcp_pose_base.pose.position.z),
            )
            tcp_source = f"tf2:{ee_frame}@{base_frame}"
        else:
            tcp_source = f"tf2:{ee_frame}_unavailable:{tcp_reason}"
        line1 = "TCP(base): --"
        line2 = "TCP source: --"
        line3 = "RPY[deg]: --"
        line4 = "TCP↔NEXT(px): --"
        if tcp_base is not None:
            bx, by, bz = tcp_base
            line1 = f"TCP(base): x={bx:+.3f} y={by:+.3f} z={bz:+.3f}"
            line2 = f"TCP source: {tcp_source}"
        if rpy is not None and isinstance(rpy, (list, tuple)) and len(rpy) >= 3:
            rr = float(rpy[0])
            pp = float(rpy[1])
            yy = float(rpy[2])
            line3 = f"RPY[deg]:  r={rr:+.1f} p={pp:+.1f} y={yy:+.1f}"

        obj_name = str(
            panel._selected_object
            or getattr(panel, "_last_grasp_selection_name", "")
            or PICK_DEMO_OBJECT_NAME
            or ""
        ).strip()

        def _project_base_to_px_canonical(base_xyz: Optional[Tuple[float, float, float]]) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float, float]], str]:
            if base_xyz is None:
                return None, None, "none"
            world_frame = panel._world_frame_last_first()
            world_xyz_raw, _ = transform_point_to_frame(
                (float(base_xyz[0]), float(base_xyz[1]), float(base_xyz[2])),
                world_frame,
                source_frame=base_frame,
                timeout_sec=0.30,
            )
            world_xyz = None
            if world_xyz_raw:
                world_xyz = (
                    float(world_xyz_raw[0]),
                    float(world_xyz_raw[1]),
                    float(world_xyz_raw[2]),
                )
            if world_xyz is None:
                return None, None, "base_to_world_tf_fail"
            px = world_xyz_to_pixel_float(
                float(world_xyz[0]),
                float(world_xyz[1]),
                float(world_xyz[2]),
                w,
                h,
            )
            if px is not None:
                return (float(px[0]), float(px[1])), world_xyz, "world_xyz_3d"
            return None, world_xyz, "world_xyz_no_camera"

        def _project_base_to_px_ghost(base_xyz: Optional[Tuple[float, float, float]]) -> Tuple[Optional[Tuple[float, float]], str]:
            if base_xyz is None:
                return None, "none"
            world_xyz = panel._base_to_world_coords(base_xyz, timeout_sec=0.05)
            if world_xyz is None:
                return None, "legacy_base_to_world_fail"
            px = table_xy_to_pixel_float(
                float(world_xyz[0]),
                float(world_xyz[1]),
                w,
                h,
            )
            if px is not None:
                return (float(px[0]), float(px[1])), "legacy_table_xy"
            return None, "legacy_projection_fail"

        tcp_px, tcp_world, tcp_px_src = _project_base_to_px_canonical(tcp_base)
        def _fmt_vec_any(vec: Optional[Tuple[float, ...]]) -> str:
            if vec is None:
                return "none"
            try:
                if len(vec) >= 3:
                    return f"({float(vec[0]):.3f},{float(vec[1]):.3f},{float(vec[2]):.3f})"
                if len(vec) == 2:
                    return f"({float(vec[0]):.1f},{float(vec[1]):.1f})"
            except Exception:
                return "none"
            return "none"
        if tcp_px is not None:
            painter.setPen(QPen(QColor(34, 211, 238, 220), 2))
            painter.setBrush(QBrush(QColor(34, 211, 238, 210)))
            painter.drawEllipse(QPointF(float(tcp_px[0]), float(tcp_px[1])), 3.5, 3.5)
            painter.drawLine(
                QPointF(float(tcp_px[0]) - 8.0, float(tcp_px[1])),
                QPointF(float(tcp_px[0]) + 8.0, float(tcp_px[1])),
            )
            painter.drawLine(
                QPointF(float(tcp_px[0]), float(tcp_px[1]) - 8.0),
                QPointF(float(tcp_px[0]), float(tcp_px[1]) + 8.0),
            )
        panel._emit_log_throttled(
            "VISUAL:tcp_target_line",
            "[PICK][VISUAL][TCP_TARGET_LINE] "
            f"origin={ee_frame}@{base_frame} obj={obj_name or 'none'} "
            f"tcp_base={_fmt_vec_any(tcp_base)} tcp_world={_fmt_vec_any(tcp_world)} "
            f"tcp_px={_fmt_vec_any(tcp_px)} "
            f"src={tcp_px_src} "
            f"tcp_source={tcp_source} topic={panel.camera_topic or 'none'}",
        )

        if TCP_POSE_TEXT_OVERLAY:
            panel_x = 10.0
            panel_y = 10.0
            panel_w = min(float(w - 20), 460.0)
            panel_h = 86.0
            bg = QColor(15, 23, 42, 165)
            border = QColor(148, 163, 184, 210)
            painter.setPen(QPen(border, 1))
            painter.setBrush(QBrush(bg))
            painter.drawRoundedRect(QRectF(panel_x, panel_y, panel_w, panel_h), 8.0, 8.0)
            painter.setPen(QPen(QColor(226, 232, 240), 1))
            tx = panel_x + 10.0
            ty = panel_y + 20.0
            painter.drawText(QPointF(tx, ty), line1)
            painter.drawText(QPointF(tx, ty + 18.0), line2)
            painter.drawText(QPointF(tx, ty + 36.0), line3)
            painter.drawText(QPointF(tx, ty + 54.0), line4)
    finally:
        painter.end()
    return img_copy

def _save_overhead_frame_with_overlays(panel, out_path: str) -> bool:
    """Save current camera frame with the same overlays used in runtime display."""
    if not panel._last_camera_frame:
        return False
    qimg, w, h, _ts = panel._last_camera_frame
    display = qimg
    topic = str(panel.camera_topic or "").strip()
    overhead_only = panel._overhead_camera_active(topic)
    if OVERLAY_CALIB and (panel._calibrating or (time.time() <= panel._calib_grid_until)):
        display = panel._draw_calib_overlay(display, w, h)
    if panel._should_draw_reach_overlay(overhead_only):
        display = panel._draw_reach_overlay(display, w, h)
    if panel._should_draw_selection_overlay(overhead_only):
        display = panel._draw_selection_overlay(display, w, h)
    if overhead_only and panel._last_grasp_px:
        display = panel._draw_grasp_overlay(display, w, h)
    if panel._should_draw_test_corner_overlay():
        display = panel._draw_test_corner_overlay(display, w, h)
    # Persistir siempre la linea canonica en snapshots overhead.
    if panel._should_draw_tcp_pose_overlay(overhead_only):
        display = panel._draw_tcp_pose_overlay(display, w, h)
    out = Path(str(out_path)).expanduser()
    ensure_dir(str(out.parent))
    return bool(display.save(str(out)))

def _save_grasp_overlay(panel, filename: str = "overlay_last.png") -> str:
    if not panel._last_camera_frame or not panel._last_grasp_px:
        return ""
    qimg, w, h, _ts = panel._last_camera_frame
    overlay = panel._draw_grasp_overlay(qimg, w, h)
    safe_name = str(filename or "overlay_last.png").strip() or "overlay_last.png"
    out_path = panel._audit_root() / "figures" / safe_name
    ensure_dir(str(out_path.parent))
    if overlay.save(str(out_path)):
        return str(out_path)
    return ""

def _refresh_grasp_overlay_now(panel) -> bool:
    if not panel._last_camera_frame or not panel._last_grasp_px:
        return False
    qimg, w, h, ts = panel._last_camera_frame
    with panel._camera_frame_lock:
        panel._camera_pending_frame = (
            panel.camera_topic or panel._last_grasp_frame or "image",
            qimg,
            w,
            h,
            panel._camera_last_fps,
            ts,
        )
    panel._refresh_camera_display()
    return True

def _draw_reach_overlay(panel, qimg: QImage, w: int, h: int) -> QImage:
    """Dibujar alcance del robot como puntos sobre la imagen."""
    from PyQt5.QtGui import QPainter, QPen, QColor
    from PyQt5.QtCore import Qt, QPointF

    if not panel._reach_overlay_points or panel._reach_overlay_size != (w, h):
        panel._reach_overlay_points = panel._compute_reach_overlay_points(w, h)
        panel._reach_overlay_size = (w, h)
    if not panel._reach_overlay_points:
        return qimg
    img_copy = qimg.copy()
    painter = QPainter(img_copy)
    if OVERLAY_ANTIALIAS:
        painter.setRenderHint(QPainter.Antialiasing)
    painter.setPen(QPen(QColor(34, 197, 94, 200), 2, Qt.SolidLine))
    for px, py in panel._reach_overlay_points:
        painter.drawPoint(QPointF(px, py))
    painter.end()
    return img_copy

