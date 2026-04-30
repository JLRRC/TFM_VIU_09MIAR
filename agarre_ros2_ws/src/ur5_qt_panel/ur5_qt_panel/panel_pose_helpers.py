#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pose_helpers.py
# Contenido: Helpers de seleccion de frames + get_pose extraidos de panel_utils.
"""Helpers de seleccion de frames base/EE/world + get_pose (refactor B.7).

Extraido de ``panel_utils.py`` (lineas 543-792 originales). Funciones
helper para descubrir frames preferentes en el grafo TF a partir de
heuristicas (whitelist + caches + parent/child traversal).

Reexportado desde panel_utils para mantener compatibilidad con
``panel_tf_diagnose``, ``panel_watchdog``, ``panel_direct2``.

Constantes publicas:
- BASE_FRAME_CANDIDATES, EE_FRAME_PREFERENCE, WORLD_FRAME_CANDIDATES

Funciones publicas:
- get_pose (lookup TF + transform pose)
- effective_base_frame, effective_world_frame

Funciones privadas:
- _select_base_frame, _select_ee_frame
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Dict, List, Optional, Set, Tuple

try:
    from .panel_config import (
        BASE_FRAME,
        WORLD_FRAME,
    )
except Exception:  # pragma: no cover
    from .panel_config import (  # type: ignore
        BASE_FRAME,
        WORLD_FRAME,
    )

from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params

try:
    from geometry_msgs.msg import PoseStamped, TransformStamped
except Exception:  # pragma: no cover
    PoseStamped = None  # type: ignore
    TransformStamped = None  # type: ignore

from .panel_tf import get_tf_helper
from .panel_tf_discovery import (
    EE_FRAME_CANDIDATE_BASES,
    EE_FRAME_SUBSTRING_KEYWORDS,
    _can_transform_between,
    _collect_leaf_frames,
    _frame_depth,
)
from . import panel_tf_discovery as _tfd  # share _BASE_FRAME_CACHE state

if TYPE_CHECKING:
    from .panel_tf import TfHelper


def _log_exception(context: str, exc: Exception) -> None:
    """Inline simplificado para evitar circular import."""

    import os
    debug_enabled = _get_panel_ui_params().debug_exceptions
    if debug_enabled:
        import traceback
        traceback.print_exc()


# Las funciones extraidas se apenden aqui via sed.
def _select_base_frame(
    helper: TfHelper,
    frames: Set[str],
    children: Dict[str, List[str]],
    parent_map: Dict[str, str],
    world_frame: str,
    robot_keyword_frames: List[str],
    timeout_sec: float,
) -> Optional[str]:
    # _BASE_FRAME_CACHE compartido vive en panel_tf_discovery (acceso via _tfd)
    candidate_order: List[str] = []
    seen: Set[str] = set()

    def add_candidate(name: Optional[str]) -> None:
        if not name or name in seen:
            return
        seen.add(name)
        candidate_order.append(name)

    add_candidate(BASE_FRAME)
    for candidate in BASE_FRAME_CANDIDATES:
        add_candidate(candidate)
    add_candidate(_tfd._BASE_FRAME_CACHE)

    robot_frames = list(robot_keyword_frames)
    if not robot_frames:
        leaves = _collect_leaf_frames(frames, children)
        fallback_frames = leaves or sorted(frames)
        depth_cache: Dict[str, int] = {}
        robot_frames = sorted(
            fallback_frames,
            key=lambda f: _frame_depth(f, parent_map, depth_cache),
            reverse=True,
        )

    base_world_only: Optional[str] = None
    base_link_candidate: Optional[str] = None
    for candidate in candidate_order:
        if candidate not in frames:
            continue
        if not _can_transform_between(helper, candidate, world_frame, timeout_sec):
            continue
        if candidate == "base_link":
            base_link_candidate = candidate
        robot_connected = any(
            _can_transform_between(helper, candidate, robot_frame, timeout_sec)
            for robot_frame in robot_frames
        )
        if robot_connected:
            _tfd._BASE_FRAME_CACHE = candidate
            return candidate
        if candidate == "base":
            base_world_only = base_world_only or candidate

    if base_link_candidate:
        _tfd._BASE_FRAME_CACHE = base_link_candidate
        return base_link_candidate
    if base_world_only:
        _tfd._BASE_FRAME_CACHE = base_world_only
        return base_world_only
    return None


def _select_ee_frame(
    helper: TfHelper,
    frames: Set[str],
    children: Dict[str, List[str]],
    parent_map: Dict[str, str],
    base_frame: str,
    timeout_sec: float,
) -> Optional[str]:
    if not base_frame:
        return None
    seen: Set[str] = set()
    candidates: List[str] = []

    def add_candidate(name: str) -> None:
        if not name or name in seen or name not in frames:
            return
        seen.add(name)
        candidates.append(name)

    for preferred in EE_FRAME_PREFERENCE:
        add_candidate(preferred)

    # Align with tf_probe keepers priority.
    for keeper in ("rg2_pinch_center", "rg2_tcp", "tool0", "tcp", "ee_link", "flange", "wrist_3_link", "ft_frame", "rg2_hand"):
        add_candidate(keeper)

    prefixes = set()
    for frame in frames:
        if "/" in frame:
            prefixes.add(frame.split("/", 1)[0])
        if "_" in frame:
            prefixes.add(frame.split("_", 1)[0])
    prefixes.discard("")

    for prefix in sorted(prefixes):
        for name in EE_FRAME_CANDIDATE_BASES:
            add_candidate(f"{prefix}/{name}")
            add_candidate(f"{prefix}_{name}")

    for name in EE_FRAME_CANDIDATE_BASES:
        add_candidate(name)

    for frame in sorted(frames):
        if any(keyword in frame.lower() for keyword in EE_FRAME_SUBSTRING_KEYWORDS):
            add_candidate(frame)

    for candidate in candidates:
        if _can_transform_between(helper, base_frame, candidate, timeout_sec):
            return candidate

    leaves = _collect_leaf_frames(frames, children) or list(frames)
    preferred = [
        leaf
        for leaf in leaves
        if any(keyword in leaf.lower() for keyword in EE_FRAME_SUBSTRING_KEYWORDS)
    ]
    leaf_candidates = preferred or leaves
    depth_cache: Dict[str, int] = {}
    ordered_leaves = sorted(
        leaf_candidates,
        key=lambda f: _frame_depth(f, parent_map, depth_cache),
        reverse=True,
    )
    for leaf in ordered_leaves:
        if _can_transform_between(helper, base_frame, leaf, timeout_sec):
            return leaf
    return None


def get_pose(
    target_frame: str,
    source_frame: str,
    timeout_sec: float = 0.8,
) -> Tuple[Optional[Dict[str, object]], Optional[str]]:
    helper = get_tf_helper()
    if helper is None:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {source_frame}->{target_frame} timed out"
    stamp_ns = 0
    try:
        stamp = transform.header.stamp
        stamp_ns = (int(getattr(stamp, "sec", 0)) * 1_000_000_000) + int(
            getattr(stamp, "nanosec", 0)
        )
    except Exception:
        stamp_ns = 0
    pose = {
        "frame": target_frame,
        "position": (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ),
        "orientation": (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ),
        "stamp_ns": int(stamp_ns),
    }
    return pose, None


def _pose_from_transform(transform: "TransformStamped", frame_id: str) -> Optional["PoseStamped"]:
    if not transform or PoseStamped is None:
        return None
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = transform.header.stamp
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


def lookup_pose(
    frame_from: str,
    frame_to: str,
    timeout_sec: float = 0.8,
) -> Tuple[Optional["PoseStamped"], Optional[str]]:
    """Return PoseStamped of *frame_from* expressed in *frame_to*."""
    helper = get_tf_helper()
    if helper is None:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(frame_to, frame_from, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {frame_from}->{frame_to} timed out"
    pose = _pose_from_transform(transform, frame_to)
    if not pose:
        return None, "pose helper unavailable"
    return pose, None


def transform_pose(
    pose: Optional["PoseStamped"],
    target_frame: str,
    timeout_sec: float = 0.8,
) -> Optional["PoseStamped"]:
    helper = get_tf_helper()
    if helper is None or pose is None:
        return None
    return helper.transform_pose(pose, target_frame, timeout_sec)


BASE_FRAME_CANDIDATES = [
    "base_link",
]
EE_FRAME_PREFERENCE = [
    "rg2_pinch_center",
    "rg2_tcp",
    "tool0",
    "tcp",
    "ee_link",
    "ee",
    "tool_link",
    "end_effector_link",
    "flange",
    "wrist_3_link",
    "rg2_hand",
    "ft_frame",
]
WORLD_FRAME_CANDIDATES = ["world", "map", "odom"]


def effective_base_frame(panel, *, default: str = "base_link") -> str:
    """Resolve the effective base frame for a panel instance."""
    candidate = str(getattr(panel, "_base_frame_effective", "") or "").strip()
    if candidate and candidate != "base_link":
        return "base_link"
    setting = str(BASE_FRAME or "").strip()
    if setting and setting != "base_link":
        return "base_link"
    return "base_link"


def effective_world_frame(panel, *, default: str = "world") -> str:
    """Resolve the effective world frame for a panel instance."""
    last = getattr(panel, "_world_frame_last_first", None)
    if callable(last):
        candidate = last()
    else:
        candidate = None
    return candidate or WORLD_FRAME or default
