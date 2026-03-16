#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_objects.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Object position helpers for the panel."""
from __future__ import annotations

import json
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple

from .panel_config import (
    ATTACHABLE_OBJECTS,
    PICK_DEMO_OBJECT_NAMES,
    DROP_OBJECT_NAMES,
    UNKNOWN_OBJECT_NAMES,
    OBJECT_POSITIONS,
    OBJECT_POS_PATH,
    GZ_WORLD,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    TABLE_OBJECT_XY_MARGIN,
    TABLE_SIZE_X,
    TABLE_SIZE_Y,
    TABLE_TOP_Z,
    WORLDS_DIR,
    refresh_object_groups,
)
from .logging_utils import timestamped_line

OBJECT_POS_LOCK = threading.Lock()
_DEBUG_EXCEPTIONS = os.environ.get("PANEL_DEBUG_EXCEPTIONS", "").strip() in ("1", "true", "True")
_LOADED_FROM_DISK = False
_LOADING_FROM_DISK = False
_STATE_LOG_LAST: Dict[str, float] = {}
_STATE_LOG_PERIOD_SEC = 1.5
_MOVE_EPS = 1e-4
_STABLE_MOVE_EPS = 0.005
_SPECIAL_PICK_OBJECT = "pick_demo"
_TEST_READ_ONLY = False
_TABLE_TOL = 0.08
_TABLE_GEOM_CACHE: Optional[Tuple[float, float, float, float, float, str]] = None
_TABLE_GEOM_WARNED = False
_TYPE_LOGGED = False


class ObjectLogicalState(Enum):
    SPAWNED = "SPAWNED"
    ON_TABLE = "ON_TABLE"
    SELECTED = "SELECTED"
    GRASPED = "GRASPED"
    CARRIED = "CARRIED"
    RELEASED = "RELEASED"


class ObjectOwner(Enum):
    NONE = "NONE"
    ROBOT = "ROBOT"


@dataclass
class ObjectState:
    name: str
    logical_state: ObjectLogicalState
    owner: ObjectOwner
    attached: bool
    position: Tuple[float, float, float]
    last_update_ts: float = 0.0


_OBJECT_STATES: Dict[str, ObjectState] = {}
_ALLOWED_TRANSITIONS = {
    ObjectLogicalState.SPAWNED: {
        ObjectLogicalState.SPAWNED,
        ObjectLogicalState.ON_TABLE,
        ObjectLogicalState.SELECTED,
    },
    ObjectLogicalState.ON_TABLE: {
        ObjectLogicalState.ON_TABLE,
        ObjectLogicalState.SELECTED,
        ObjectLogicalState.GRASPED,
    },
    ObjectLogicalState.SELECTED: {
        ObjectLogicalState.SELECTED,
        ObjectLogicalState.ON_TABLE,
        ObjectLogicalState.GRASPED,
    },
    ObjectLogicalState.GRASPED: {ObjectLogicalState.GRASPED, ObjectLogicalState.CARRIED},
    ObjectLogicalState.CARRIED: {ObjectLogicalState.CARRIED, ObjectLogicalState.RELEASED},
    ObjectLogicalState.RELEASED: {ObjectLogicalState.RELEASED, ObjectLogicalState.ON_TABLE},
}


def _log_exception(context: str, exc: Exception) -> None:
    if not _DEBUG_EXCEPTIONS:
        return
    print(timestamped_line(f"[OBJECTS][WARN] {context}: {exc}"), file=sys.stderr, flush=True)


def _log_state(message: str, *, key: Optional[str] = None) -> None:
    now = time.monotonic()
    log_key = key or message
    last = _STATE_LOG_LAST.get(log_key, 0.0)
    if (now - last) < _STATE_LOG_PERIOD_SEC:
        return
    _STATE_LOG_LAST[log_key] = now
    print(timestamped_line(message), file=sys.stderr, flush=True)


def _log_object_types_once() -> None:
    global _TYPE_LOGGED
    if _TYPE_LOGGED:
        return
    for name in sorted(PICK_DEMO_OBJECT_NAMES):
        _log_state(f"[OBJECTS][TYPE] {name} -> PICK_DEMO", key=f"type:pick_demo:{name}")
    for name in sorted(DROP_OBJECT_NAMES):
        _log_state(f"[OBJECTS][TYPE] {name} -> DROP", key=f"type:drop:{name}")
    for name in sorted(UNKNOWN_OBJECT_NAMES):
        _log_state(f"[OBJECTS][TYPE] {name} -> UNKNOWN", key=f"type:unknown:{name}")
    _TYPE_LOGGED = True


def _table_sdf_path() -> Optional[str]:
    world_name = (GZ_WORLD or "").strip()
    candidates = []
    if world_name:
        if world_name.endswith(".sdf"):
            candidates.append(os.path.join(WORLDS_DIR, world_name))
        else:
            candidates.append(os.path.join(WORLDS_DIR, f"{world_name}.sdf"))
    candidates.append(os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf"))
    for cand in candidates:
        if cand and os.path.isfile(cand):
            return cand
    return None


def _parse_pose(text: str) -> Tuple[float, float, float]:
    parts = [p for p in (text or "").split() if p]
    if len(parts) >= 3:
        try:
            return float(parts[0]), float(parts[1]), float(parts[2])
        except Exception as exc:
            _log_exception("parse pose", exc)
    return 0.0, 0.0, 0.0


def _load_table_geometry() -> Tuple[float, float, float, float, float, str]:
    global _TABLE_GEOM_CACHE, _TABLE_GEOM_WARNED
    if _TABLE_GEOM_CACHE is not None:
        return _TABLE_GEOM_CACHE
    sdf_path = _table_sdf_path()
    if sdf_path and os.path.isfile(sdf_path):
        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()
        except Exception as exc:
            _log_exception("parse table sdf", exc)
        else:
            for model in root.findall(".//model"):
                if (model.attrib.get("name") or "") != "mesa_pro":
                    continue
                model_pose = _parse_pose(model.findtext("pose") or "")
                link = model.find("link")
                if link is None:
                    continue
                link_pose = _parse_pose(link.findtext("pose") or "")
                collision = link.find("collision")
                if collision is None:
                    collision = link.find("visual")
                if collision is None:
                    continue
                geom = collision.find("geometry")
                if geom is None:
                    continue
                box = geom.find("box")
                if box is None:
                    continue
                size_text = box.findtext("size") or ""
                parts = [p for p in size_text.split() if p]
                if len(parts) != 3:
                    continue
                try:
                    size = tuple(float(p) for p in parts)
                except Exception as exc:
                    _log_exception("parse mesa_pro size", exc)
                    continue
                collision_pose = _parse_pose(collision.findtext("pose") or "")
                center_x = model_pose[0] + link_pose[0] + collision_pose[0]
                center_y = model_pose[1] + link_pose[1] + collision_pose[1]
                top_z = model_pose[2] + link_pose[2] + collision_pose[2] + (size[2] / 2.0)
                _TABLE_GEOM_CACHE = (
                    center_x,
                    center_y,
                    float(size[0]),
                    float(size[1]),
                    top_z,
                    "sdf",
                )
                return _TABLE_GEOM_CACHE
    if not _TABLE_GEOM_WARNED:
        _log_state("[OBJECTS][WARN] mesa_pro no disponible; usando geometria configurada", key="table_geom_cfg")
        _TABLE_GEOM_WARNED = True
    _TABLE_GEOM_CACHE = (TABLE_CENTER_X, TABLE_CENTER_Y, TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_TOP_Z, "config")
    return _TABLE_GEOM_CACHE


def _on_table_geometry(position: Tuple[float, float, float]) -> bool:
    x, y, z = position
    center_x, center_y, size_x, size_y, table_z, _src = _load_table_geometry()
    half_x = size_x / 2.0 + TABLE_OBJECT_XY_MARGIN
    half_y = size_y / 2.0 + TABLE_OBJECT_XY_MARGIN
    dx = x - center_x
    dy = y - center_y
    dz = z - table_z
    height_ok = 0.0 <= dz <= _TABLE_TOL
    x_ok = abs(dx) <= half_x
    y_ok = abs(dy) <= half_y
    return bool(height_ok and x_ok and y_ok)


def _table_geom_debug(position: Tuple[float, float, float]) -> str:
    x, y, z = position
    center_x, center_y, size_x, size_y, table_z, src = _load_table_geometry()
    half_x = size_x / 2.0 + TABLE_OBJECT_XY_MARGIN
    half_y = size_y / 2.0 + TABLE_OBJECT_XY_MARGIN
    dx = x - center_x
    dy = y - center_y
    dz = z - table_z
    return (
        f"pos=({x:.3f},{y:.3f},{z:.3f}) "
        f"dx={dx:.3f} dy={dy:.3f} "
        f"half=({half_x:.3f},{half_y:.3f}) "
        f"table_z={table_z:.3f} dz={dz:.3f} tol={_TABLE_TOL:.3f} src={src}"
    )


def table_geom_debug(position: Tuple[float, float, float]) -> str:
    return _table_geom_debug(position)


def _initial_logical_state(name: str, position: Tuple[float, float, float]) -> ObjectLogicalState:
    if name in ATTACHABLE_OBJECTS and _on_table_geometry(position):
        return ObjectLogicalState.ON_TABLE
    return ObjectLogicalState.SPAWNED


def _ensure_object_state(name: str, position: Tuple[float, float, float]) -> ObjectState:
    state = _OBJECT_STATES.get(name)
    if state is not None:
        return state
    logical_state = _initial_logical_state(name, position)
    state = ObjectState(
        name=name,
        logical_state=logical_state,
        owner=ObjectOwner.NONE,
        attached=False,
        position=position,
        last_update_ts=time.time(),
    )
    _OBJECT_STATES[name] = state
    return state


def _sync_object_states_locked() -> None:
    for name, pos in OBJECT_POSITIONS.items():
        state = _OBJECT_STATES.get(name)
        if state is None:
            _OBJECT_STATES[name] = ObjectState(
                name=name,
                logical_state=_initial_logical_state(name, pos),
                owner=ObjectOwner.NONE,
                attached=False,
                position=pos,
                last_update_ts=time.time(),
            )
        else:
            state.position = pos
    for name in list(_OBJECT_STATES.keys()):
        if name not in OBJECT_POSITIONS:
            _OBJECT_STATES.pop(name, None)


def get_object_state(name: str) -> Optional[ObjectState]:
    _ensure_loaded()
    with OBJECT_POS_LOCK:
        state = _OBJECT_STATES.get(name)
        return None if state is None else ObjectState(**state.__dict__)


def get_object_states() -> Dict[str, ObjectState]:
    _ensure_loaded()
    with OBJECT_POS_LOCK:
        return {k: ObjectState(**v.__dict__) for k, v in _OBJECT_STATES.items()}


def is_on_table(position: Tuple[float, float, float]) -> bool:
    return _on_table_geometry(position)


def set_test_read_only(enabled: bool, *, reason: str = "") -> None:
    global _TEST_READ_ONLY
    _TEST_READ_ONLY = bool(enabled)
    state = "on" if _TEST_READ_ONLY else "off"
    _log_state(f"[OBJECTS][TEST] read-only {state} ({reason or 'toggle'})", key="test_ro")
    if _TEST_READ_ONLY:
        with OBJECT_POS_LOCK:
            for name, obj in _OBJECT_STATES.items():
                if obj.owner != ObjectOwner.NONE or obj.attached:
                    _log_state(
                        f"[OBJECTS][INVARIANT] TEST requires owner NONE (found {name})",
                        key=f"test_owner:{name}",
                    )
                if obj.logical_state in (ObjectLogicalState.GRASPED, ObjectLogicalState.CARRIED):
                    _log_state(
                        f"[OBJECTS][INVARIANT] TEST requires non-grasped state ({name})",
                        key=f"test_state:{name}",
                    )


def recalc_object_states(reason: str = "recalc") -> None:
    _ensure_loaded()
    with OBJECT_POS_LOCK:
        for name, obj in _OBJECT_STATES.items():
            if obj.owner != ObjectOwner.NONE or obj.attached:
                continue
            prev_state = obj.logical_state
            on_table = _on_table_geometry(obj.position)
            if on_table:
                if prev_state != ObjectLogicalState.SELECTED:
                    obj.logical_state = ObjectLogicalState.ON_TABLE
            else:
                if prev_state == ObjectLogicalState.SELECTED:
                    _log_state(
                        f"[OBJECTS][INVARIANT] selected off-table ({name})",
                        key=f"selected_off:{name}",
                    )
                obj.logical_state = ObjectLogicalState.SPAWNED
            if obj.logical_state != prev_state:
                _log_state(
                    f"[OBJECTS] state {name} -> {obj.logical_state.value} reason={reason}",
                    key=f"recalc:{name}:{obj.logical_state.value}",
                )


def _can_move(state: ObjectState) -> bool:
    return state.owner == ObjectOwner.NONE or state.attached


def _position_changed(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> bool:
    return _position_changed_eps(a, b, _MOVE_EPS)


def _position_changed_eps(
    a: Tuple[float, float, float],
    b: Tuple[float, float, float],
    eps: float,
) -> bool:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5 > eps


def _transition_allowed(prev: ObjectLogicalState, new: ObjectLogicalState) -> bool:
    allowed = _ALLOWED_TRANSITIONS.get(prev, set())
    return new in allowed


def update_object_state(
    name: str,
    *,
    logical_state: Optional[ObjectLogicalState] = None,
    owner: Optional[ObjectOwner] = None,
    attached: Optional[bool] = None,
    reason: str = "",
    read_only: bool = False,
) -> bool:
    _ensure_loaded()
    if read_only or _TEST_READ_ONLY:
        _log_state(f"[OBJECTS][RO] state update blocked for {name} ({reason})", key=f"ro:{name}")
        return False
    with OBJECT_POS_LOCK:
        pos = OBJECT_POSITIONS.get(name, (0.0, 0.0, 0.0))
        state = _ensure_object_state(name, pos)
        next_state = logical_state or state.logical_state
        next_owner = owner or state.owner
        next_attached = attached if attached is not None else state.attached

        # REMOVED: Bloqueo de ownership que solo permitía pick_demo
        # Ahora se permite agarrar cualquier objeto (PICK Objeto)
        # El control de seguridad se realiza a nivel de UI/gating
        if next_attached and next_owner != ObjectOwner.ROBOT:
            _log_state(
                f"[OBJECTS][INVARIANT] attached requires owner=ROBOT ({name})",
                key=f"attach_owner:{name}",
            )
            return False
        if logical_state and not _transition_allowed(state.logical_state, logical_state):
            _log_state(
                f"[OBJECTS][INVARIANT] illegal transition {state.logical_state.value}->{logical_state.value} ({name})",
                key=f"transition:{name}",
            )
            return False
        if next_state == ObjectLogicalState.SELECTED and not _on_table_geometry(state.position):
            _log_state(
                f"[OBJECTS][INVARIANT] SELECTED requires on_table geometry ({name})",
                key=f"select_geom:{name}",
            )
            return False

        if next_owner == ObjectOwner.ROBOT and next_state not in (
            ObjectLogicalState.GRASPED,
            ObjectLogicalState.CARRIED,
        ):
            _log_state(
                f"[OBJECTS][INVARIANT] owner=ROBOT requires GRASPED/CARRIED ({name})",
                key=f"owner_state:{name}",
            )
            return False
        if next_owner == ObjectOwner.NONE and next_state == ObjectLogicalState.CARRIED:
            _log_state(
                f"[OBJECTS][INVARIANT] owner=NONE cannot be CARRIED ({name})",
                key=f"owner_none:{name}",
            )
            return False
        if _TEST_READ_ONLY and next_state in (ObjectLogicalState.GRASPED, ObjectLogicalState.CARRIED):
            _log_state(
                f"[OBJECTS][INVARIANT] TEST read-only blocks {next_state.value} ({name})",
                key=f"test_state:{name}",
            )
            return False

        changed = (
            state.logical_state != next_state
            or state.owner != next_owner
            or state.attached != next_attached
        )
        state.logical_state = next_state
        state.owner = next_owner
        state.attached = next_attached
        state.last_update_ts = time.time()
        if changed:
            _log_state(
                (
                    f"[OBJECTS] state {name} -> {state.logical_state.value} "
                    f"owner={state.owner.value} attached={str(state.attached).lower()} "
                    f"reason={reason or 'update'}"
                ),
                key=f"state:{name}:{state.logical_state.value}:{state.owner.value}:{state.attached}",
            )
    return True


def mark_object_grasped(name: str, *, reason: str = "", read_only: bool = False) -> bool:
    state = get_object_state(name)
    if state and state.logical_state not in (
        ObjectLogicalState.ON_TABLE,
        ObjectLogicalState.SELECTED,
    ):
        _log_state(
            f"[OBJECTS][INVARIANT] GRASPED requires ON_TABLE/SELECTED ({name})",
            key=f"grasp_gate:{name}",
        )
        return False
    if state and state.logical_state == ObjectLogicalState.RELEASED:
        update_object_state(
            name,
            logical_state=ObjectLogicalState.ON_TABLE,
            owner=ObjectOwner.NONE,
            attached=False,
            reason=reason or "reset_on_table",
            read_only=read_only,
        )
    return update_object_state(
        name,
        logical_state=ObjectLogicalState.GRASPED,
        owner=ObjectOwner.ROBOT,
        attached=False,
        reason=reason or "grasp",
        read_only=read_only,
    )


def mark_object_attached(name: str, *, reason: str = "", read_only: bool = False) -> bool:
    ok = update_object_state(
        name,
        logical_state=ObjectLogicalState.CARRIED,
        owner=ObjectOwner.ROBOT,
        attached=True,
        reason=reason or "attach",
        read_only=read_only,
    )
    return ok


def mark_object_released(name: str, *, reason: str = "", read_only: bool = False) -> bool:
    return update_object_state(
        name,
        logical_state=ObjectLogicalState.RELEASED,
        owner=ObjectOwner.NONE,
        attached=False,
        reason=reason or "release",
        read_only=read_only,
    )


def force_release_all_objects(*, reason: str = "force_release_all") -> int:
    """Force owner/attach cleanup and normalize logical state by geometry."""
    _ensure_loaded()
    released = 0
    with OBJECT_POS_LOCK:
        for name, pos in OBJECT_POSITIONS.items():
            state = _ensure_object_state(name, pos)
            prev_state = state.logical_state
            prev_owner = state.owner
            prev_attached = state.attached
            state.owner = ObjectOwner.NONE
            state.attached = False
            on_table = _on_table_geometry(state.position)
            state.logical_state = (
                ObjectLogicalState.ON_TABLE if on_table else ObjectLogicalState.SPAWNED
            )
            state.last_update_ts = time.time()
            if (
                state.logical_state != prev_state
                or state.owner != prev_owner
                or state.attached != prev_attached
            ):
                released += 1
                _log_state(
                    (
                        f"[OBJECTS] state {name} -> {state.logical_state.value} "
                        f"owner={state.owner.value} attached={str(state.attached).lower()} "
                        f"reason={reason}"
                    ),
                    key=f"force_release:{name}:{state.logical_state.value}",
                )
    return released


def get_object_positions() -> Dict[str, Tuple[float, float, float]]:
    _ensure_loaded()
    with OBJECT_POS_LOCK:
        return dict(OBJECT_POSITIONS)


def get_object_position(name: str) -> Optional[Tuple[float, float, float]]:
    _ensure_loaded()
    with OBJECT_POS_LOCK:
        return OBJECT_POSITIONS.get(name)


def set_object_position(name: str, position: Tuple[float, float, float]) -> None:
    _ensure_loaded()
    x, y, z = float(position[0]), float(position[1]), float(position[2])
    with OBJECT_POS_LOCK:
        OBJECT_POSITIONS[name] = (x, y, z)
        state = _ensure_object_state(name, (x, y, z))
        state.position = (x, y, z)
        if state.owner == ObjectOwner.NONE and not state.attached:
            on_table = _on_table_geometry(state.position)
            if on_table and state.logical_state != ObjectLogicalState.SELECTED:
                state.logical_state = ObjectLogicalState.ON_TABLE
            elif not on_table and state.logical_state == ObjectLogicalState.ON_TABLE:
                state.logical_state = ObjectLogicalState.SPAWNED
    refresh_object_groups()


def bulk_update_object_positions(
    updates: Dict[str, Tuple[float, float, float]],
    allow_new: bool = False,
    *,
    source: str = "unknown",
    read_only: bool = False,
    objects_stable: bool = False,
) -> int:
    """Update multiple object poses with locking. Skips unknown objects unless allow_new is True."""
    updated = 0
    _ensure_loaded()
    if read_only or _TEST_READ_ONLY:
        _log_state(f"[OBJECTS][RO] pose updates blocked ({source})", key="ro_updates")
        return 0
    with OBJECT_POS_LOCK:
        for name, pos in updates.items():
            if not allow_new and name not in OBJECT_POSITIONS:
                continue
            try:
                x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            except Exception as exc:
                _log_exception(f"invalid position for {name}", exc)
                continue
            current = OBJECT_POSITIONS.get(name, (x, y, z))
            state = _ensure_object_state(name, current)
            next_pos = (x, y, z)
            moved = _position_changed(state.position, next_pos)
            if moved and not _can_move(state):
                _log_state(
                    f"[OBJECTS][INVARIANT] move blocked {name} owner={state.owner.value} attached={state.attached} ({source})",
                    key=f"move:{name}",
                )
                continue
            stable_moved = _position_changed_eps(state.position, next_pos, _STABLE_MOVE_EPS)
            if stable_moved and objects_stable and state.owner == ObjectOwner.NONE and not state.attached:
                _log_state(
                    f"[OBJECTS][INVARIANT] move while stable {name} owner=NONE ({source})",
                    key=f"stable_move:{name}",
                )
            OBJECT_POSITIONS[name] = next_pos
            state.position = next_pos
            state.last_update_ts = time.time()
            if state.owner == ObjectOwner.NONE and not state.attached:
                on_table = _on_table_geometry(state.position) if objects_stable else False
                if objects_stable:
                    center_x, center_y, size_x, size_y, table_z, src = _load_table_geometry()
                    dz = state.position[2] - table_z
                    _log_state(
                        (
                            f"[TABLE] z={table_z:.3f} tol={_TABLE_TOL:.3f} "
                            f"center=({center_x:.3f},{center_y:.3f}) "
                            f"size=({size_x:.3f},{size_y:.3f}) src={src}"
                        ),
                        key="table_geom",
                    )
                    _log_state(
                        f"[OBJ] {name} z={state.position[2]:.3f} dz={dz:.3f} -> ON_TABLE={on_table}",
                        key=f"table_obj:{name}",
                    )
                    if _DEBUG_EXCEPTIONS and 0.0 <= dz <= _TABLE_TOL and not on_table:
                        raise RuntimeError(
                            f"[INVARIANT VIOLATION] {name}: "
                            f"Physically on table (dz={dz:.3f}) but on_table=False"
                        )
                prev_state = state.logical_state
                if on_table:
                    if state.logical_state != ObjectLogicalState.SELECTED:
                        state.logical_state = ObjectLogicalState.ON_TABLE
                else:
                    if state.logical_state == ObjectLogicalState.SELECTED:
                        _log_state(
                            f"[OBJECTS][INVARIANT] selected off-table ({name})",
                            key=f"selected_off:{name}",
                        )
                        state.logical_state = ObjectLogicalState.SPAWNED
                    elif state.logical_state != ObjectLogicalState.RELEASED:
                        state.logical_state = ObjectLogicalState.SPAWNED
                if state.logical_state != prev_state:
                    _log_state(
                        f"[OBJECTS] state {name} -> {state.logical_state.value} reason={source}",
                        key=f"pos_state:{name}:{state.logical_state.value}",
                    )
            updated += 1
    if updated:
        refresh_object_groups()
    return updated


def remove_object_position(name: str) -> None:
    _ensure_loaded()
    with OBJECT_POS_LOCK:
        OBJECT_POSITIONS.pop(name, None)
        _OBJECT_STATES.pop(name, None)
    refresh_object_groups()


def load_object_positions() -> None:
    if not os.path.isfile(OBJECT_POS_PATH):
        return
    try:
        with open(OBJECT_POS_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            with OBJECT_POS_LOCK:
                for name, vals in data.items():
                    if (
                        isinstance(vals, list)
                        and len(vals) == 3
                        and all(isinstance(v, (int, float)) for v in vals)
                    ):
                        OBJECT_POSITIONS[name] = (float(vals[0]), float(vals[1]), float(vals[2]))
                _sync_object_states_locked()
        refresh_object_groups()
    except Exception as exc:
        _log_exception("load_object_positions", exc)


def _ensure_loaded() -> None:
    global _LOADED_FROM_DISK, _LOADING_FROM_DISK
    if _LOADED_FROM_DISK:
        return
    with OBJECT_POS_LOCK:
        if _LOADED_FROM_DISK:
            return
        if _LOADING_FROM_DISK:
            loading = True
        else:
            _LOADING_FROM_DISK = True
            loading = False
    if loading:
        for _ in range(50):
            if _LOADED_FROM_DISK:
                return
            time.sleep(0.02)
        return
    load_object_positions()
    with OBJECT_POS_LOCK:
        _sync_object_states_locked()
        _LOADED_FROM_DISK = True
        _LOADING_FROM_DISK = False
    _log_object_types_once()


def save_object_positions() -> None:
    with OBJECT_POS_LOCK:
        data = {k: [float(v[0]), float(v[1]), float(v[2])] for k, v in OBJECT_POSITIONS.items()}
    try:
        with open(OBJECT_POS_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except Exception as exc:
        _log_exception("save_object_positions", exc)
