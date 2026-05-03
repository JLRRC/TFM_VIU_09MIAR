#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tf_discovery.py
# Contenido: Helpers de discovery y debug de frames TF extraidos de panel_utils.
"""Helpers de discovery y debug de frames TF extraidos de panel_utils.py (B.5).

Extraido de ``panel_utils.py`` (lineas 685-991 originales). Funciones que:

* dumpean transforms TF para debug.
* parsean salida YAML de ``ros2 run tf2_tools view_frames`` o equivalente.
* construyen el grafo de frames (parent/child).
* descubren frames base/mundo/EE candidatos.
* logguean diagnosticos solo una vez (caches).

Reexportado desde panel_utils para mantener compatibilidad con
``panel_helpers``, ``panel_trace_callbacks``, ``panel_object_mgmt``.

Constantes publicas:
- EE_FRAME_CANDIDATE_BASES
- ROBOT_FRAME_KEYWORDS
- EE_FRAME_SUBSTRING_KEYWORDS

Funciones publicas:
- debug_dump_tf
- _can_transform_between
- discover_robot_base_frame, discover_world_frame, discover_base_and_ee_frames
- _log_tf_frames_once, _log_tf_frame_summary, _log_tf_yaml_head_once,
  _log_ee_unavailable_once

Helpers privados (modulares pero accesibles via reexport):
- _parse_tf_yaml_records, _extract_frames_from_yaml, _load_tf_frame_records
- _build_frame_graph, _collect_leaf_frames, _frame_depth
- _preferred_base_frame
"""

from __future__ import annotations

import re
from typing import TYPE_CHECKING, Dict, List, Optional, Set, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

from .logging_utils import emit_log_line, timestamped_line
from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params
from .panel_tf import get_tf_helper  # noqa: F401

if TYPE_CHECKING:
    from .panel_tf import TfHelper


def yaw_from_quaternion(quat) -> float:  # late binding via panel_utils
    """Lazy wrapper para evitar circular import; delega a panel_utils."""

    from .panel_utils import yaw_from_quaternion as _impl
    return _impl(quat)


def _log_exception(context: str, exc: Exception) -> None:
    """Inline simplificado para evitar circular import."""

    import os
    debug_enabled = _get_panel_ui_params().debug_exceptions
    if debug_enabled:
        import traceback
        traceback.print_exc()


# Las constantes y funciones extraidas se apenden aqui via sed.
def debug_dump_tf(
    target_frame: str, source_frame: str = "world", timeout_sec: float = 1.0
) -> Tuple[Optional[Dict[str, object]], Optional[str]]:
    """Lookup and describe TF from *source_frame* to *target_frame*."""
    helper = get_tf_helper()
    if not helper:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {source_frame}->{target_frame} timed out"
    t = transform.transform.translation
    yaw = yaw_from_quaternion(transform.transform.rotation)
    return (
        {
            "translation": (t.x, t.y, t.z),
            "yaw": yaw,
            "frame_rel": source_frame,
        },
        None,
    )


_TF_FRAMES_LOGGED = False
_TF_FRAME_SUMMARY_LOGGED = False
_TF_YAML_HEAD_LOGGED = False
_EE_UNAVAILABLE_LOGGED = False
_TF_YAML_HEAD_LOGGED = False
EE_FRAME_CANDIDATE_BASES = (
    "rg2_pinch_center",
    "tool0",
    "rg2_tcp",
    "tcp",
    "ee_link",
    "gripper_link",
    "wrist_3_link",
    "flange",
    "ee",
    "tool_link",
    "end_effector_link",
    "rg2_hand",
    "ft_frame",
)

ROBOT_FRAME_KEYWORDS = ("wrist", "shoulder", "elbow", "tool", "tcp", "ee", "flange", "rg2", "hand")
EE_FRAME_SUBSTRING_KEYWORDS = ("tool", "tcp", "ee", "gripper", "flange", "wrist", "rg2", "hand", "ft")

_BASE_FRAME_CACHE: Optional[str] = None
_LAST_TRACE_FRAME_LOG: Optional[str] = None


def _parse_tf_yaml_records(yaml_text: str) -> List[Dict[str, object]]:
    if not yaml_text:
        return []
    parsed = None
    if yaml is not None:
        try:
            parsed = yaml.safe_load(yaml_text)
        except Exception:
            parsed = None
    records: List[Dict[str, object]] = []
    if isinstance(parsed, dict):
        frames = parsed.get("frames")
        if isinstance(frames, list):
            for entry in frames:
                if isinstance(entry, dict):
                    records.append(entry)
    if not records:
        for match in _FRAME_ID_PATTERN.finditer(yaml_text):
            records.append({"frame_id": match.group(1)})
    return records


_FRAME_ID_PATTERN = re.compile(r'frame_id:\s*"([^"]+)"')
_CHILD_FRAME_ID_PATTERN = re.compile(r'child_frame_id:\s*"([^"]+)"')
_FRAME_LINE_PATTERN = re.compile(r'Frame\s+["\']?([A-Za-z0-9_/:\.\-]+)["\']?', re.IGNORECASE)
_FRAME_KEY_PATTERN = re.compile(r'^\s*([A-Za-z0-9_/:\.\-]+)\s*:\s*(?:\{|\[|$)', re.MULTILINE)
_FRAME_LIST_ITEM_PATTERN = re.compile(r'^\s*-\s*([A-Za-z0-9_/:\.\-]+)\s*$', re.MULTILINE)
_FRAME_LIST_FRAME_PATTERN = re.compile(r'^\s*-\s*frame\s*:\s*([A-Za-z0-9_/:\.\-]+)\s*$', re.IGNORECASE | re.MULTILINE)


def _extract_frames_from_yaml(yaml_text: str) -> Set[str]:
    """Return frame names extracted from the TF YAML dump."""
    if not yaml_text:
        return set()
    found: Set[str] = set()
    for pattern in (
        _FRAME_ID_PATTERN,
        _CHILD_FRAME_ID_PATTERN,
        _FRAME_LINE_PATTERN,
        _FRAME_KEY_PATTERN,
        _FRAME_LIST_ITEM_PATTERN,
        _FRAME_LIST_FRAME_PATTERN,
    ):
        for match in pattern.finditer(yaml_text):
            name = match.group(1)
            if name:
                found.add(name)
    cleaned: Set[str] = set()
    ignore = {"frames", "transforms", "child_frames", "header", "data", "frame"}
    for frame in found:
        if frame.lower() in ignore:
            continue
        cleaned.add(frame)
    return cleaned


def _load_tf_frame_records(source) -> List[Dict[str, object]]:
    if source is None:
        return []
    if isinstance(source, str):
        return _parse_tf_yaml_records(source)
    try:
        yaml_text = source.all_frames_as_yaml()
    except Exception:
        return []
    return _parse_tf_yaml_records(yaml_text)


def _build_frame_graph(records: List[Dict[str, object]]) -> Tuple[Set[str], Dict[str, List[str]], Dict[str, str]]:
    frames: Set[str] = set()
    children: Dict[str, List[str]] = {}
    parent_map: Dict[str, str] = {}
    for entry in records:
        fid = entry.get("frame_id")
        if not fid:
            continue
        frames.add(fid)
        parent = entry.get("parent_frame_id")
        if parent and parent != fid:
            parent_map[fid] = parent
            children.setdefault(parent, []).append(fid)
        for child in entry.get("child_frames") or []:
            if isinstance(child, dict):
                child_id = child.get("frame_id")
                if child_id and child_id != fid:
                    frames.add(child_id)
                    parent_map[child_id] = fid
                    children.setdefault(fid, []).append(child_id)
    return frames, children, parent_map


def _collect_leaf_frames(frames: Set[str], children: Dict[str, List[str]]) -> List[str]:
    if not frames:
        return []
    parents = set(children.keys())
    return [frame for frame in frames if frame not in parents]


def _frame_depth(
    frame: str,
    parent_map: Dict[str, str],
    cache: Dict[str, int],
    visiting: Optional[Set[str]] = None,
) -> int:
    if frame in cache:
        return cache[frame]
    if visiting is None:
        visiting = set()
    if frame in visiting:
        cache[frame] = 0
        return 0
    visiting.add(frame)
    parent = parent_map.get(frame)
    if not parent or parent == frame:
        depth = 0
    else:
        depth = 1 + _frame_depth(parent, parent_map, cache, visiting)
    visiting.remove(frame)
    cache[frame] = depth
    return depth


def _log_tf_frames_once(frames: Set[str]) -> None:
    global _TF_FRAMES_LOGGED
    if _TF_FRAMES_LOGGED or not frames:
        return
    sample = ", ".join(sorted(frames)[:80])
    emit_log_line(
        timestamped_line(f"[TRACE] Available TF frames ({len(frames)}): {sample}"),
    )
    _TF_FRAMES_LOGGED = True

def _log_tf_frame_summary(all_frames: Set[str], robot_keyword_frames: List[str]) -> None:
    global _TF_FRAME_SUMMARY_LOGGED
    if _TF_FRAME_SUMMARY_LOGGED or not all_frames:
        return
    sample = ", ".join(sorted(all_frames)[:80])
    emit_log_line(
        timestamped_line(
            f"[TRACE] TF summary frames={len(all_frames)} robot_candidates={len(robot_keyword_frames)} sample={sample}"
        ),
    )
    _TF_FRAME_SUMMARY_LOGGED = True


def _log_tf_yaml_head_once(yaml_text: str) -> None:
    global _TF_YAML_HEAD_LOGGED
    if _TF_YAML_HEAD_LOGGED or not yaml_text:
        return
    lines = yaml_text.strip().splitlines()
    head = "\n".join(lines[:20])
    emit_log_line(
        timestamped_line(f"[TRACE][DIAG] TF YAML head:\n{head}"),
    )
    _TF_YAML_HEAD_LOGGED = True


def _log_ee_unavailable_once() -> None:
    global _EE_UNAVAILABLE_LOGGED
    if _EE_UNAVAILABLE_LOGGED:
        return
    emit_log_line(timestamped_line("[TRACE] EE unavailable (no valid EE frame)"))
    _EE_UNAVAILABLE_LOGGED = True


def _can_transform_between(helper: TfHelper, frame_a: str, frame_b: str, timeout_sec: float) -> bool:
    if not frame_a or not frame_b:
        return False
    if helper.can_transform(frame_a, frame_b, timeout_sec=timeout_sec):
        return True
    if helper.can_transform(frame_b, frame_a, timeout_sec=timeout_sec):
        return True
    return False


def _preferred_base_frame(helper: Optional[TfHelper], world_frame: str, timeout_sec: float = 0.2) -> Optional[str]:
    """Return the first base candidate that transforms to the world frame."""
    if helper is None:
        return None
    candidate = "base_link"
    if _can_transform_between(helper, candidate, world_frame, timeout_sec=timeout_sec):
        return candidate
    return None


def discover_robot_base_frame(world_frame: Optional[str] = None, timeout_sec: float = 0.2) -> Optional[str]:
    """Detect the most likely robot base frame given the TF tree."""
    helper = get_tf_helper()
    if helper is None:
        return None
    base_frame, _ = discover_base_and_ee_frames(world_frame, timeout_sec)
    return base_frame


def discover_world_frame(helper: TfHelper, base_frame: str, selection_frame: Optional[str] = None, timeout_sec: float = 0.2) -> Optional[str]:
    """Return the first world frame candidate that transforms to *base_frame*."""
    if helper is None or not base_frame:
        return selection_frame or WORLD_FRAME or "world"
    frames = helper.list_frames()
    candidates: List[str] = []
    if selection_frame:
        candidates.append(selection_frame)
    for candidate in (WORLD_FRAME, *WORLD_FRAME_CANDIDATES, "gz_world", "map", "odom"):
        if not candidate:
            continue
        if candidate not in candidates:
            candidates.append(candidate)
    for candidate in candidates:
        if candidate not in frames:
            continue
        if _can_transform_between(helper, candidate, base_frame, timeout_sec):
            return candidate
    return selection_frame or WORLD_FRAME or "world"


def discover_base_and_ee_frames(world_frame: Optional[str] = None, timeout_sec: float = 0.1) -> Tuple[Optional[str], Optional[str]]:
    """Return effective base and EE frames based on live TF contents."""
    global _BASE_FRAME_CACHE, _LAST_TRACE_FRAME_LOG, _EE_UNAVAILABLE_LOGGED
    helper = get_tf_helper()
    if helper is None:
        return None, None
    frames = helper.list_frames()
    if not frames:
        return None, None
    records = _load_tf_frame_records(helper.frames_yaml())
    graph_frames, children, parent_map = _build_frame_graph(records)
    all_frames = frames.union(graph_frames)
    if not all_frames:
        return None, None
    target_world = world_frame or WORLD_FRAME or "world"
    sorted_frames = sorted(all_frames)
    robot_keyword_frames = [
        frame for frame in sorted_frames if any(keyword in frame.lower() for keyword in ROBOT_FRAME_KEYWORDS)
    ]
    _log_tf_frame_summary(all_frames, robot_keyword_frames)
    base_frame: Optional[str] = None
    if "base_link" in all_frames and _can_transform_between(helper, "base_link", target_world, timeout_sec):
        base_frame = "base_link"
    fallback_base = "base_link"
    from .panel_pose_helpers import _select_ee_frame  # lazy import (circular)
    ee_frame = _select_ee_frame(helper, all_frames, children, parent_map, fallback_base, timeout_sec)
    effective_base = base_frame or fallback_base
    disallowed_ee = {effective_base}
    if target_world:
        disallowed_ee.add(target_world)
    if ee_frame and ee_frame in disallowed_ee:
        ee_frame = None
        _log_ee_unavailable_once()
    elif ee_frame:
        _EE_UNAVAILABLE_LOGGED = False
    log_msg = f"[TRACE] Using BASE_FRAME_EFFECTIVE={effective_base or 'n/a'} EE_FRAME_EFFECTIVE={ee_frame or 'n/a'}"
    if log_msg != _LAST_TRACE_FRAME_LOG:
        emit_log_line(timestamped_line(log_msg))
        _LAST_TRACE_FRAME_LOG = log_msg
    return effective_base, ee_frame
