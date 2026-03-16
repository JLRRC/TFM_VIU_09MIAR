#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_physics.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Physics/settle routines extracted from panel_v2."""
from __future__ import annotations

import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional, Set, Tuple

from PyQt5.QtCore import QTimer

from .panel_config import (
    DROP_OBJECT_NAMES,
    PICK_DEMO_OBJECT_NAMES,
    PICK_DEMO_OBJECTS,
    DROP_OBJECTS,
    ATTACH_DIST_M,
    ATTACH_REL_EPS,
    ATTACH_HAND_MOVE_EPS,
    ATTACH_WINDOW_SEC,
    ATTACH_SNAP_EPS,
    FALL_TEST_DELAY_SEC,
    GZ_WORLD,
    POSE_INFO_MAX_AGE_SEC,
    POSE_INFO_POLL_SEC,
    POSE_INFO_LOG_PERIOD,
    ALLOW_UNSETTLED_ON_TIMEOUT,
)
from .panel_utils import (
    _log_exception,
    get_object_positions,
    read_world_name,
)

if TYPE_CHECKING:
    from .panel_v2 import ControlPanelV2

HAND_LINK_CANDIDATES = ("rg2_hand", "gripper", "tool0", "ee_link")
OBJECT_SETTLE_Z_EPS = 0.002
OBJECT_FALL_Z_EPS = 0.01
OBJECT_SETTLE_WINDOW_SEC = 1.0
OBJECT_SETTLE_POLL_SEC = 0.6
OBJECT_SETTLE_TIMEOUT_SEC = 10.0
OBJECT_SETTLE_XY_EPS = 0.002
SETTLE_PATTERNS = ("box_", "cyl_", "cross_")


def _parse_first_json_object(text: str) -> Optional[Dict[str, object]]:
    if not text:
        return None
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("{"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    return None


def _parse_sdf_gravity(world_path: str) -> Tuple[float, float, float]:
    try:
        if not world_path:
            return (0.0, 0.0, -9.81)
        text = Path(world_path).read_text(encoding="utf-8", errors="ignore")
        if "<gravity>" not in text:
            return (0.0, 0.0, -9.81)
        root = ET.fromstring(text)
        gravity = root.find(".//gravity")
        if gravity is None or not gravity.text:
            return (0.0, 0.0, -9.81)
        parts = gravity.text.strip().split()
        if len(parts) != 3:
            return (0.0, 0.0, -9.81)
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except Exception:
        return (0.0, 0.0, -9.81)


class PanelPhysics:
    def __init__(self, panel: "ControlPanelV2") -> None:
        self._panel = panel

    def _footprint_radius(self, name: str) -> Optional[float]:
        p = self._panel
        sdf = getattr(p, "_sdf_model_cache", {}).get(name)
        if not sdf:
            return None
        geom_type = sdf.get("type")
        if geom_type == "box":
            size = sdf.get("size")
            if size and len(size) >= 2:
                return max(float(size[0]), float(size[1])) / 2.0
        if geom_type in ("cylinder", "sphere"):
            radius = sdf.get("radius")
            if radius:
                return float(radius)
        return None

    def log_initial_overlap_warnings(
        self, positions: Dict[str, Tuple[float, float, float]], targets: Set[str]
    ) -> None:
        p = self._panel
        try:
            p._load_sdf_geometry_cache()
        except Exception:
            return
        radii: Dict[str, float] = {}
        for name in targets:
            radius = self._footprint_radius(name)
            if radius:
                radii[name] = radius
        names = sorted(radii.keys())
        for i, a in enumerate(names):
            pa = positions.get(a)
            if not pa:
                continue
            for b in names[i + 1 :]:
                pb = positions.get(b)
                if not pb:
                    continue
                dx = pb[0] - pa[0]
                dy = pb[1] - pa[1]
                dist = (dx * dx + dy * dy) ** 0.5
                min_dist = radii[a] + radii[b]
                if dist < (min_dist * 0.98):
                    p._emit_log(
                        f"[PHYSICS][OVERLAP] {a} vs {b} dist={dist:.3f} min={min_dist:.3f}"
                    )

    def start_objects_settle_watch(self) -> None:
        p = self._panel
        if p._closing:
            return
        if p._settle_worker_active:
            return
        if not p._pose_info_ok:
            now = time.monotonic()
            if (now - getattr(p, "_last_settle_gating_log", 0.0)) > 1.5:
                p._emit_log("[PHYSICS][SETTLE] gating: pose_info_ready=false, settle NO iniciado")
                p._last_settle_gating_log = now
            p._update_pose_info_status()
            return
        p._ensure_pose_subscription()
        p._objects_settled = False
        p._objects_seen_fall = False
        spawn = dict(get_object_positions())
        for name, pos in PICK_DEMO_OBJECTS.items():
            spawn[name] = pos
        for name, pos in DROP_OBJECTS.items():
            spawn.setdefault(name, pos)
        p._spawn_positions_snapshot = spawn
        p._settle_log_snapshot_active = (
            p._settle_log_snapshot_next or not p._settle_log_once_done
        )
        p._settle_log_snapshot_next = False
        p._settle_worker_active = True
        if p._settle_thread:
            try:
                if p._settle_thread.isRunning():
                    return
            except RuntimeError:
                # Thread QObject already deleted; clear stale handle.
                p._settle_thread = None
        p._settle_thread = p._run_async(self.objects_settle_worker, name="objects_settle")

    def invalidate_settle(self, reason: str, *, restart: bool = True) -> None:
        p = self._panel
        if p._closing:
            return
        p._objects_settled = False
        p._objects_seen_fall = False
        p._calibration_ready = False
        p._spawn_positions_snapshot = get_object_positions()
        p._emit_log(f"[PHYSICS] Revalidar settle: {reason}")
        if restart and p._gz_running:
            p.signal_start_objects_settle_watch.emit()

    def run_fall_test_async(self) -> None:
        p = self._panel
        if p._fall_test_active or not p._gz_running or p._closing:
            return
        if not p._pose_info_ok:
            now = time.monotonic()
            if (now - p._fall_test_last_log) >= POSE_INFO_LOG_PERIOD:
                p._emit_log("[PHYSICS][FALL_TEST] waiting pose/info data...")
                p._fall_test_last_log = now
            return
        p._fall_test_active = True
        world_path = p.world_combo.currentText().strip()
        world_name = p._gz_world_name or (read_world_name(world_path) if world_path else GZ_WORLD)

        def worker():
            try:
                poses_t0 = p._read_world_pose_info(world_name)
                if not poses_t0:
                    p._emit_log("[PHYSICS][FALL_TEST] no pose data (bridge?)")
                    return
                t0_map: Dict[str, float] = {}
                for pose in poses_t0:
                    name = pose.get("name")
                    if name in PICK_DEMO_OBJECT_NAMES:
                        pos = pose.get("position") or {}
                        t0_map[name] = float(pos.get("z") or 0.0)
                time.sleep(max(0.1, FALL_TEST_DELAY_SEC))
                poses_t1 = p._read_world_pose_info(world_name)
                if not poses_t1:
                    p._emit_log("[PHYSICS][FALL_TEST] no pose data at t1")
                    return
                t1_map: Dict[str, float] = {}
                for pose in poses_t1:
                    name = pose.get("name")
                    if name in PICK_DEMO_OBJECT_NAMES:
                        pos = pose.get("position") or {}
                        t1_map[name] = float(pos.get("z") or 0.0)
                for name in PICK_DEMO_OBJECT_NAMES:
                    if name not in t0_map or name not in t1_map:
                        continue
                    dz = t0_map[name] - t1_map[name]
                    ok = dz >= OBJECT_FALL_Z_EPS
                    p._emit_log(
                        f"[PHYSICS][FALL_TEST] model={name} dz={dz:.3f} ok={str(ok).lower()}"
                    )
            finally:
                p._fall_test_active = False

        p._run_async(worker)

    def objects_settle_worker(self) -> None:
        p = self._panel
        settled = self.wait_for_objects_to_settle(
            timeout=OBJECT_SETTLE_TIMEOUT_SEC,
            log_snapshot=p._settle_log_snapshot_active,
        )
        if not settled:
            self.log_settle_snapshot("fail/timeout")
            p._emit_log("[PHYSICS][SETTLE][DIAG] Snapshot automático por fallo de settle.")
        if not settled and ALLOW_UNSETTLED_ON_TIMEOUT:
            p._emit_log("[PHYSICS][SETTLE] Continuando sin estabilizar (override).")
            settled = True
        p._objects_settled = settled
        if settled:
            p._emit_log("[PHYSICS][SETTLE] OK")
            p.signal_handle_objects_settled.emit()
            p.signal_status.emit("Objetos estabilizados", False)
        else:
            p._emit_log("[PHYSICS][SETTLE] Timeout: objetos no estabilizados.")
            p.signal_status.emit("Timeout: objetos no estabilizados", True)
        p._settle_worker_active = False
        p.signal_refresh_controls.emit()

    def log_settle_snapshot(self, reason: str) -> None:
        p = self._panel
        world_path = p.world_combo.currentText().strip()
        world_name = p._gz_world_name or (read_world_name(world_path) if world_path else GZ_WORLD)
        targets = p._settle_targets()
        if not targets:
            return
        poses = p._read_world_pose_info(world_name)
        if not poses:
            return
        p._emit_log(f"[PHYSICS][SETTLE] snapshot({reason})")
        for pose in poses:
            name = pose.get("name")
            if name not in targets:
                continue
            position = pose.get("position") or {}
            try:
                z = float(position.get("z") or 0.0)
            except Exception as exc:
                _log_exception(f"settle snapshot parse {name}", exc)
                continue
            p._emit_log(f"[PHYSICS][SETTLE] model={name} z={z:.3f}")

    def request_settle_snapshot(self, reason: str) -> None:
        p = self._panel
        if not p._gz_running:
            return
        p._settle_log_snapshot_next = True
        self.log_settle_snapshot(reason)

    def wait_for_objects_to_settle(
        self,
        timeout: float = OBJECT_SETTLE_TIMEOUT_SEC,
        *,
        log_snapshot: bool = False,
    ) -> bool:
        p = self._panel
        p._log_calib_blocked("esperando caída/estabilidad de objetos")
        world_path = p.world_combo.currentText().strip()
        world_name = p._gz_world_name or (read_world_name(world_path) if world_path else GZ_WORLD)
        targets = p._settle_targets()
        if not targets:
            p._log("[PHYSICS][SETTLE] No hay objetos dinámicos configurados para monitorizar.")
            return False
        start: Optional[float] = None
        pose_wait_start = time.time()
        stable_since: Dict[str, float] = {}
        stable_ok: Dict[str, bool] = {name: False for name in targets}
        has_fallen: Dict[str, bool] = {name: False for name in targets}
        attached_since: Dict[str, float] = {}
        attached_confirmed: Set[str] = set()
        attach_rel: Dict[str, Tuple[float, float, float]] = {}
        last_hand_pos: Optional[Tuple[float, float, float]] = None
        poses_seen = False
        spawn_positions = dict(p._spawn_positions_snapshot or {})
        last_positions: Dict[str, Tuple[float, float, float]] = {}
        last_block_log = 0.0
        last_pose_log = 0.0
        last_state_log: Dict[str, float] = {}
        state_log_period = 2.0
        if log_snapshot:
            self.log_settle_snapshot("inicio")
            p._settle_log_once_done = True
        self.log_initial_overlap_warnings(spawn_positions, targets)
        while p._gz_running and not p._closing and ((time.time() - start) < timeout if start else True):
            poses = p._read_world_pose_info(world_name)
            if not poses:
                if (time.time() - pose_wait_start) >= timeout:
                    p._emit_log("[PHYSICS][SETTLE] Timeout: sin pose/info.")
                    return False
                if (time.time() - last_pose_log) >= 2.0:
                    p._emit_log("[PHYSICS][SETTLE] waiting pose/info data...")
                    last_pose_log = time.time()
                if p._ros_worker_started:
                    p.ros_worker.wait_for_pose_info(OBJECT_SETTLE_POLL_SEC)
                else:
                    time.sleep(OBJECT_SETTLE_POLL_SEC)
                continue
            poses_seen = True
            if start is None:
                start = time.time()
            now = time.time()
            round_ok = True
            pose_map: Dict[str, Tuple[float, float, float]] = {}
            for pose in poses:
                name = pose.get("name")
                if not name:
                    continue
                position = pose.get("position") or {}
                pose_map[name] = (
                    float(position.get("x") or 0.0),
                    float(position.get("y") or 0.0),
                    float(position.get("z") or 0.0),
                )
            hand_pos = None
            for key, pos in pose_map.items():
                if any(key.endswith(f"::{cand}") or key == cand for cand in HAND_LINK_CANDIDATES):
                    hand_pos = pos
                    break
            hand_moved = False
            if hand_pos and last_hand_pos:
                dxh = hand_pos[0] - last_hand_pos[0]
                dyh = hand_pos[1] - last_hand_pos[1]
                dzh = hand_pos[2] - last_hand_pos[2]
                hand_moved = (dxh * dxh + dyh * dyh + dzh * dzh) ** 0.5 > ATTACH_HAND_MOVE_EPS
            if hand_pos:
                last_hand_pos = hand_pos
            for pose in poses:
                name = pose.get("name")
                if name not in targets:
                    continue
                position = pose.get("position") or {}
                x = float(position.get("x") or 0.0)
                y = float(position.get("y") or 0.0)
                z = float(position.get("z") or 0.0)
                prev = last_positions.get(name)
                dz = z - prev[2] if prev else 0.0
                dx = x - prev[0] if prev else 0.0
                dy = y - prev[1] if prev else 0.0
                spawn_z = spawn_positions.get(name, (x, y, z))[2]
                if (spawn_z - z) > OBJECT_FALL_Z_EPS:
                    has_fallen[name] = True
                require_fall = any(name.startswith(pat) for pat in SETTLE_PATTERNS)
                if not require_fall:
                    has_fallen[name] = True
                if hand_pos and name in DROP_OBJECT_NAMES:
                    dxh = x - hand_pos[0]
                    dyh = y - hand_pos[1]
                    dzh = z - hand_pos[2]
                    dist = (dxh * dxh + dyh * dyh + dzh * dzh) ** 0.5
                    if dist <= ATTACH_DIST_M:
                        prev_rel = attach_rel.get(name)
                        rel = (dxh, dyh, dzh)
                        rel_ok = True
                        if prev_rel:
                            dr = (
                                (rel[0] - prev_rel[0]) ** 2
                                + (rel[1] - prev_rel[1]) ** 2
                                + (rel[2] - prev_rel[2]) ** 2
                            ) ** 0.5
                            rel_ok = dr <= ATTACH_REL_EPS
                        attach_rel[name] = rel
                        if rel_ok and (hand_moved or dist <= ATTACH_SNAP_EPS):
                            attached_since.setdefault(name, now)
                            if (now - attached_since[name]) >= ATTACH_WINDOW_SEC:
                                attached_confirmed.add(name)
                        else:
                            attached_since.pop(name, None)
                    else:
                        attached_since.pop(name, None)
                stable = bool(prev) and abs(dz) <= OBJECT_SETTLE_Z_EPS and abs(dx) <= OBJECT_SETTLE_XY_EPS and abs(dy) <= OBJECT_SETTLE_XY_EPS
                if name in attached_confirmed:
                    has_fallen[name] = False
                    stable_ok[name] = False
                    stable_since.pop(name, None)
                elif not has_fallen.get(name, False):
                    stable_ok[name] = False
                    stable_since.pop(name, None)
                elif stable:
                    if name not in stable_since:
                        stable_since[name] = now
                    elif (now - stable_since[name]) >= OBJECT_SETTLE_WINDOW_SEC:
                        stable_ok[name] = True
                else:
                    stable_ok[name] = False
                    stable_since.pop(name, None)
                last_positions[name] = (x, y, z)
                last_log = last_state_log.get(name, 0.0)
                if (now - last_log) >= state_log_period:
                    dz_from_spawn = spawn_z - z
                    stable_flag = bool(has_fallen.get(name, False) and stable_ok.get(name, False))
                    p._emit_log(
                        f"[PHYSICS][SETTLE] model={name} z={z:.3f} dz={dz_from_spawn:.3f} "
                        f"has_fallen={has_fallen.get(name, False)} stable={stable_flag}"
                    )
                    last_state_log[name] = now
                if not (has_fallen.get(name, False) and stable_ok.get(name, False)):
                    round_ok = False
            if round_ok and targets:
                p._objects_seen_fall = True
                return True
            if (now - last_block_log) >= 1.8:
                p._log_calib_blocked("esperando caída/estabilidad de objetos")
                last_block_log = now
            if p._ros_worker_started:
                p.ros_worker.wait_for_pose_info(OBJECT_SETTLE_POLL_SEC)
            else:
                time.sleep(OBJECT_SETTLE_POLL_SEC)
        if poses_seen:
            for name in sorted(attached_confirmed):
                p._emit_log(f"[PHYSICS][ATTACH] model={name} attached=true")
            for name in sorted(targets):
                require_fall = any(name.startswith(pat) for pat in SETTLE_PATTERNS)
                if require_fall and not has_fallen.get(name, False):
                    p._emit_log(
                        f"[PHYSICS][SETTLE] model={name} has_fallen=false -> revisar SDF: static/kinematic/gravity/inertial/joint"
                    )
                else:
                    p._emit_log(f"[PHYSICS][SETTLE] model={name} has_fallen=true stable={stable_ok.get(name, False)}")
        return False

    def detect_world_name(self) -> Optional[str]:
        p = self._panel
        world_path = p.world_combo.currentText().strip()
        return read_world_name(world_path) if world_path else None

    def probe_pose_motion(self, world_name: str, targets: Set[str]) -> Tuple[Optional[bool], float, Optional[str]]:
        p = self._panel
        if not targets:
            return None, 0.0, None
        poses_a = p._read_world_pose_info(world_name)
        if not poses_a:
            return None, 0.0, None
        dt = 0.4
        time.sleep(dt)
        poses_b = p._read_world_pose_info(world_name)
        if not poses_b:
            return None, dt, None
        def extract(poses: List[Dict[str, object]]) -> Dict[str, Tuple[float, float, float]]:
            data: Dict[str, Tuple[float, float, float]] = {}
            for pose in poses:
                name = pose.get("name")
                if name not in targets:
                    continue
                position = pose.get("position") or {}
                try:
                    data[name] = (
                        float(position.get("x") or 0.0),
                        float(position.get("y") or 0.0),
                        float(position.get("z") or 0.0),
                    )
                except Exception as exc:
                    _log_exception(f"probe pose motion parse {name}", exc)
                    continue
            return data
        a = extract(poses_a)
        b = extract(poses_b)
        if not a or not b:
            return None, dt, None
        for name, pa in a.items():
            pb = b.get(name)
            if not pb:
                continue
            dx = pb[0] - pa[0]
            dy = pb[1] - pa[1]
            dz = pb[2] - pa[2]
            if abs(dx) > 1e-4 or abs(dy) > 1e-4 or abs(dz) > 1e-4:
                return True, dt, name
        return False, dt, None

    def check_physics_runtime(self) -> None:
        p = self._panel
        world_path = p.world_combo.currentText().strip()
        world_name = self.detect_world_name() or (read_world_name(world_path) if world_path else GZ_WORLD)
        p._gz_world_name = world_name
        p._ensure_pose_subscription()
        if not p._pose_info_ok:
            return
        stepping, dt, sample = self.probe_pose_motion(world_name, p._settle_targets())
        if stepping is None:
            for _ in range(3):
                if p._ros_worker_started:
                    p.ros_worker.wait_for_pose_info(0.25)
                else:
                    time.sleep(0.25)
                stepping, dt, sample = self.probe_pose_motion(world_name, p._settle_targets())
                if stepping is not None:
                    break
        gravity = _parse_sdf_gravity(world_path)
        step_txt = "unknown" if stepping is None else str(stepping)
        inferred_by = "pose_delta" if stepping is not None else "none"
        dt_txt = f"{dt:.2f}s" if stepping is not None else "n/a"
        sample_txt = sample or "n/a"
        p._emit_log(
            f"[PHYSICS] world={world_name} gravity={gravity} stepping={step_txt} "
            f"inferred_by={inferred_by} dt={dt_txt} sample_model={sample_txt}"
        )

    def schedule_physics_runtime_check(self) -> None:
        p = self._panel
        if p._physics_runtime_check_scheduled or p._closing:
            return
        p._physics_runtime_check_scheduled = True

        def _run():
            p._physics_runtime_check_scheduled = False
            if p._closing or not p._gz_running:
                return
            count, age = (0, float("inf"))
            if p._ros_worker_started and p.ros_worker.node_ready():
                count, age = p.ros_worker.pose_info_status()
            if count > 0 and age < POSE_INFO_MAX_AGE_SEC:
                p._run_async(self.check_physics_runtime)
                return
            self.schedule_physics_runtime_check()

        QTimer.singleShot(int(POSE_INFO_POLL_SEC * 1000), _run)
