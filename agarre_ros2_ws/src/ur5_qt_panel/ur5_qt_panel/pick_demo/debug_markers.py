#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/debug_markers.py
# Contenido: F3-step38 — publish_direct_debug_markers extraído del closure run_pick_demo.
"""Construcción y publicación de MarkerArray para visualización RViz del flujo Direct.

Función extraída del closure ``run_pick_demo._publish_direct_debug_markers``
(84 LOC). El wrapper en panel_pick_demo se reduce a ~16 LOC delegando aquí.

El caller resuelve: publisher, frame_id, stamp, live_pinch_world,
live_tool0_world (de ``_pose_position`` o equivalente). Esta función
sólo arma la MarkerArray con 8 markers (6 spheres + 2 arrows) y publica.
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

from .marker_helpers import make_sphere_marker, make_arrow_marker

Vec3 = Tuple[float, float, float]


def publish_direct_debug_markers(
    pub: Any,
    marker_array_cls: Any,
    *,
    frame_id: str,
    stamp: Any,
    live_pinch_world: Optional[Vec3],
    live_tool0_world: Optional[Vec3],
    object_frozen_world: Optional[Vec3] = None,
    object_fresh_world: Optional[Vec3] = None,
    target_semantic_world: Optional[Vec3] = None,
    target_exec_tool0_world: Optional[Vec3] = None,
) -> None:
    """Construye y publica MarkerArray de debug RViz para el pipeline Direct.

    Markers:
      - 6 spheres (object frozen/fresh, target semantic/exec_tool0,
        live rg2_pinch_center/tool0)
      - 2 arrows (live tool0 → live pinch, target_exec → target_semantic)

    No-op si pub o marker_array_cls son None. Markers con xyz=None se
    descartan (las helpers puras devuelven None en ese caso).
    """
    if pub is None or marker_array_cls is None:
        return
    marker_array = marker_array_cls()
    markers = [
        make_sphere_marker(
            ns="directo/object_frozen",
            marker_id=1,
            frame_id=frame_id,
            xyz=object_frozen_world,
            rgba=(1.0, 1.0, 0.0, 0.90),
            stamp=stamp,
        ),
        make_sphere_marker(
            ns="directo/object_fresh",
            marker_id=2,
            frame_id=frame_id,
            xyz=object_fresh_world,
            rgba=(0.0, 1.0, 1.0, 0.90),
            stamp=stamp,
        ),
        make_sphere_marker(
            ns="directo/target_semantic_rg2_pinch_center",
            marker_id=3,
            frame_id=frame_id,
            xyz=target_semantic_world,
            rgba=(1.0, 0.0, 1.0, 0.90),
            stamp=stamp,
        ),
        make_sphere_marker(
            ns="directo/target_exec_tool0",
            marker_id=4,
            frame_id=frame_id,
            xyz=target_exec_tool0_world,
            rgba=(0.35, 0.70, 1.0, 0.90),
            stamp=stamp,
        ),
        make_sphere_marker(
            ns="directo/live_rg2_pinch_center",
            marker_id=5,
            frame_id=frame_id,
            xyz=live_pinch_world,
            rgba=(0.0, 1.0, 0.0, 0.90),
            stamp=stamp,
        ),
        make_sphere_marker(
            ns="directo/live_tool0",
            marker_id=6,
            frame_id=frame_id,
            xyz=live_tool0_world,
            rgba=(1.0, 0.0, 0.0, 0.90),
            stamp=stamp,
        ),
        make_arrow_marker(
            ns="directo/live_tool0_to_pinch",
            marker_id=10,
            frame_id=frame_id,
            start_xyz=live_tool0_world,
            end_xyz=live_pinch_world,
            rgba=(1.0, 1.0, 1.0, 0.95),
            stamp=stamp,
        ),
        make_arrow_marker(
            ns="directo/target_exec_to_semantic",
            marker_id=11,
            frame_id=frame_id,
            start_xyz=target_exec_tool0_world,
            end_xyz=target_semantic_world,
            rgba=(1.0, 1.0, 1.0, 0.95),
            stamp=stamp,
        ),
    ]
    marker_array.markers = [m for m in markers if m is not None]
    if marker_array.markers:
        pub.publish(marker_array)
