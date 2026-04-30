#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_gz_objects.py
# Contenido: Gazebo object management callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Gazebo object management callbacks for ControlPanelV2."""
from __future__ import annotations

import copy
import math
import os
import shlex
import shutil
import subprocess
import time
from typing import Optional, Tuple

from .panel_config import (
    ATTACH_DIST_M,
    BASE_FRAME,
    DEFAULT_WORLD_CANDIDATES,
    DROP_ANCHOR_PREFIX,
    GRIPPER_ATTACH_PREFIX,
    GZ_WORLD,
    NUDGE_DROP_DZ,
    NUDGE_DROP_OBJECTS,
    NUDGE_DROP_Z_MIN,
    ROS_AVAILABLE,
    WORLDS_DIR,
)
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME
from .panel_objects import (
    ObjectLogicalState,
    ObjectOwner,
    bulk_update_object_positions,
    force_release_all_objects,
    get_object_position,
    get_object_positions,
    get_object_state,
    is_on_table,
    mark_object_released,
    recalc_object_states,
)
from .panel_process import build_gz_env, resolve_gz_partition
from .panel_ros_params import get_panel_ros_params as _get_panel_ros_params
from .panel_utils import read_world_name, world_to_base
import xml.etree.ElementTree as ET
from PyQt5.QtCore import QTimer
from .panel_config import (
    DROP_OBJECT_NAMES,
    contact_z_correction_for_frame,
    rclpy,
)

try:
    # FIX: Empty se usa como std_msgs/msg/Empty en pub.publish(Empty()).
    # Antes importabamos std_srvs/srv/Empty (clase de servicio que NO se
    # puede instanciar) y eso lanzaba NotImplementedError dentro del
    # worker async de _release_objects, dejando _objects_release_done
    # eternamente en False y bloqueando validate_pick_3_cycles.sh.
    from std_msgs.msg import Empty
except Exception:
    Empty = None  # type: ignore

try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity as GzEntity
except Exception:
    SetEntityPose = None  # type: ignore
    GzEntity = None  # type: ignore


def _log_exception(context: str, exc: Exception) -> None:
    print(f"[GZ_OBJ][ERROR][{context}] {exc}")


def _drop_detach_supported(panel) -> bool:
    if panel._detach_feature_available:
        return True
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    if panel.ros_worker and panel.ros_worker.node_ready():
        try:
            topics = panel.ros_worker.list_topic_names()
        except Exception as exc:
            _log_exception("drop_detach list topics", exc)
            topics = []
        prefix = f"{DROP_ANCHOR_PREFIX}/"
        for topic in topics:
            if topic.startswith(prefix) and topic.endswith("/detach"):
                panel._detach_feature_available = True
                return True
    return False

def _release_objects(panel):
    """Publicar detach a todos los objetos en Gazebo (emergencia)."""
    if panel._detach_inflight:
        return
    if panel._detach_auto_disabled and time.time() < panel._detach_backoff_until:
        return
    # Permitir relanzar la liberación global si el servicio tarda en estar listo.
    panel._objects_release_done = False
    panel._drop_hold_enabled = False
    detach_supported = panel._drop_detach_supported()
    if not detach_supported and not panel._detach_feature_logged:
        panel._emit_log("[PHYSICS][DETACH] canales drop_anchor detach no disponibles; usando release_objects service")
        panel._detach_feature_logged = True
    panel._detach_inflight = True
    panel._detach_attempted = True
    panel._log_button("Soltar objetos")
    panel._ui_set_status("Soltando objetos…")

    def worker():
        try:
            if not ROS_AVAILABLE or not panel.ros_worker or not panel.ros_worker.node_ready():
                panel._log_error("ROS no disponible; no se pudo soltar objetos")
                panel._ui_set_status("Servicio objetos no accesible", error=True)
                if panel._auto_release_drop_objects:
                    panel._schedule_release_retry("ros_not_ready")
                return
            release_ok = False
            release_msg = ""
            detach_sent = 0
            used_release_service = False
            if detach_supported:
                for name in DROP_OBJECT_NAMES:
                    topic = f"{DROP_ANCHOR_PREFIX}/{name}/detach"
                    pub = panel._get_attach_publisher(topic)
                    if pub is None:
                        continue
                    if panel.ros_worker.topic_subscriber_count(topic) <= 0:
                        continue
                    pub.publish(Empty())
                    detach_sent += 1
                if detach_sent > 0:
                    release_ok = True
                    release_msg = f"drop_anchor detach publicado count={detach_sent}"
                    panel._drop_anchor_attached = False
                    panel._emit_log(
                        f"[PHYSICS][DETACH] drop_anchor detach publicado count={detach_sent}"
                    )
            if not release_ok:
                if panel.ros_worker.has_service("/release_objects"):
                    used_release_service = True
                    release_ok, release_msg = panel.ros_worker.call_trigger_detail(
                        "/release_objects", timeout_sec=20.0
                    )
                    release_msg = (release_msg or "").strip()
                    panel._emit_log(
                        "[PHYSICS][DROP] release_objects service "
                        f"success={str(bool(release_ok)).lower()} "
                        f"message='{release_msg or 'sin mensaje'}'"
                    )
                else:
                    release_msg = "release_objects service no disponible"
                    panel._emit_log(f"[PHYSICS][DROP] {release_msg}")

            if not release_ok:
                if used_release_service:
                    time.sleep(0.6)
                    try:
                        panel._refresh_objects_from_gz()
                    except Exception as exc:
                        panel._emit_log(f"[PHYSICS][DROP] refresh_after_failed_release err={exc}")
                    if panel._sync_external_release_state():
                        release_ok = True
                        release_msg = (
                            f"{release_msg}; recovered_via_pose_sync"
                            if release_msg
                            else "recovered_via_pose_sync"
                        )
                        panel._emit_log(
                            "[PHYSICS][DROP] release_objects recovered via pose/info reconciliation"
                        )
            if not release_ok:
                panel._objects_release_done = False
                panel._objects_settled = False
                panel.signal_refresh_controls.emit()
                hint = release_msg or "revisa [PHYSICS][DROP] para delete/spawn/drop_anchor"
                panel._emit_log(
                    f"[PHYSICS][DROP] release_objects failed: {hint}"
            )
                panel._ui_set_status(f"❌ Soltar objetos falló: {hint}", error=True)
                if panel._auto_release_drop_objects:
                    panel._schedule_release_retry(f"release_failed:{hint}")
                return

            if release_ok and used_release_service:
                # Give Gazebo a short window to respawn dynamic models before detach.
                time.sleep(0.4)
                try:
                    panel._refresh_objects_from_gz()
                except Exception as exc:
                    panel._emit_log(f"[PHYSICS][DROP] refresh_after_release err={exc}")

            if release_ok and used_release_service:
                panel._drop_anchor_attached = False
                panel._emit_log(
                    "[PHYSICS][DETACH] detach skipped: joint not present (global reset already applied)"
                )

            panel._objects_release_done = True
            panel._objects_settled = False
            panel._release_retry_count = 0
            state = get_object_state(PICK_DEMO_OBJECT_NAME)
            if state and state.logical_state in (
                ObjectLogicalState.GRASPED,
                ObjectLogicalState.CARRIED,
            ):
                mark_object_released(
                    PICK_DEMO_OBJECT_NAME, reason="release_objects_grasped"
            )
            else:
                panel._emit_log(
                    "[OBJECTS] release grasp skipped (not grasped/carried); applying global reset"
            )
            released_count = force_release_all_objects(
                reason="release_objects_global_reset"
            )
            recalc_object_states(reason="release_objects_post_reset")
            panel._emit_log(
                f"[OBJECTS] global reset applied owners=NONE count={released_count}"
            )
            panel.signal_refresh_controls.emit()
            panel._ui_set_status("✅ Objetos soltados")
            if used_release_service and panel._sync_external_release_state():
                panel._emit_log(
                    "[PHYSICS][DROP] panel settle skipped: service release already reconciled"
                )
            else:
                panel._invalidate_settle("objetos liberados", restart=True)
            panel._maybe_nudge_drop_objects("release_success")
        except Exception as e:
            import traceback
            tb = traceback.format_exc()
            panel._log_error(f"Soltar objetos error: {e}")
            panel._emit_log(f"[PHYSICS][DROP][TRACEBACK] {tb}")
            panel._ui_set_status(f"Error soltando objetos: {e}", error=True)
        finally:
            panel._detach_inflight = False

    panel._run_async(worker)

def _schedule_release_retry(panel, reason: str) -> None:
    panel._release_retry_count = int(panel._release_retry_count or 0) + 1
    attempt = panel._release_retry_count
    if attempt <= 8:
        delay_ms = min(4000, 500 * attempt)
        panel._emit_log(
            f"[PHYSICS][DETACH] retry reason={reason} attempt={attempt}/8 delay_ms={delay_ms}"
        )
        panel.signal_run_ui_delayed.emit(panel._release_objects, delay_ms)
        return
    panel._detach_auto_disabled = True
    panel._detach_backoff_until = time.time() + 10.0
    panel._emit_log(
        "[PHYSICS][DETACH] retry agotado; auto-release pausado (10s) para evitar bucle"
    )

def _attach_drop_objects(panel, reason: str) -> None:
    if panel._drop_anchor_attached:
        return
    if panel._objects_release_done:
        return
    if not ROS_AVAILABLE or not panel.ros_worker or not panel.ros_worker.node_ready():
        return
    for name in DROP_OBJECT_NAMES:
        topic = f"{DROP_ANCHOR_PREFIX}/{name}/attach"
        pub = panel._get_attach_publisher(topic)
        if pub is not None:
            pub.publish(Empty())
    panel._drop_anchor_attached = True
    panel._emit_log(f"[PHYSICS][HOLD] drop_anchor attach publicado ({reason})")

def _maybe_nudge_drop_objects(panel, reason: str) -> None:
    if not NUDGE_DROP_OBJECTS:
        return
    if panel._drop_nudge_done:
        return
    panel._drop_nudge_done = True
    def worker():
        time.sleep(0.4)
        panel._nudge_drop_objects(reason)
    panel._run_async(worker)

def _resolve_set_pose_service(panel, world_name: str) -> Optional[str]:
    if not panel.ros_worker or not panel.ros_worker.node_ready():
        return None
    expected = f"/world/{world_name}/set_entity_pose"
    services = panel.ros_worker.service_names_and_types()
    if not services:
        return None
    type_hint = "SetEntityPose"
    candidates = []
    for name, types in services:
        if not name:
            continue
        if any(type_hint in t for t in (types or [])):
            candidates.append(name)
    if expected in candidates:
        return expected
    for suffix in ("/set_entity_pose", "/set_pose"):
        for name in candidates:
            if name.endswith(suffix) and f"/world/{world_name}/" in name:
                return name
    for name in candidates:
        if name.endswith("/set_entity_pose"):
            return name
    return None

def _resolve_gz_cli(panel) -> Optional[str]:
    cached = str(getattr(panel, "_pick_demo_recover_gz_cli", "") or "").strip()
    if cached and os.path.isfile(cached):
        return cached
    candidates = [
        shutil.which("gz"),
        "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
        "/opt/ros/humble/opt/gz_tools_vendor/bin/gz",
    ]
    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            panel._pick_demo_recover_gz_cli = candidate
            return candidate
    return None

def _resolve_world_sdf_path(panel) -> str:
    world_path = panel.world_combo.currentText().strip()
    if world_path and os.path.isfile(world_path):
        return world_path
    if world_path:
        cand = os.path.join(WORLDS_DIR, world_path)
        if os.path.isfile(cand):
            return cand
        if not cand.endswith(".sdf") and os.path.isfile(cand + ".sdf"):
            return cand + ".sdf"
    for cand in DEFAULT_WORLD_CANDIDATES:
        if os.path.isfile(cand):
            return cand
    return ""

def _load_pick_demo_recover_sdf(panel) -> Optional[str]:
    world_sdf = panel._resolve_world_sdf_path()
    cached_world = str(getattr(panel, "_pick_demo_recover_world_sdf", "") or "").strip()
    cached_sdf = str(getattr(panel, "_pick_demo_recover_sdf", "") or "")
    if world_sdf and cached_sdf and world_sdf == cached_world:
        return cached_sdf
    if not world_sdf or not os.path.exists(world_sdf):
        panel._emit_log("[PICK_DEMO][RECOVER] world_sdf_missing")
        return None
    try:
        tree = ET.parse(world_sdf)
        root = tree.getroot()
        world_elem = root.find("world")
        if world_elem is None:
            panel._emit_log("[PICK_DEMO][RECOVER] world_tag_missing")
            return None
        model_elem = None
        for model in world_elem.findall("model"):
            if (model.get("name") or "").strip() == PICK_DEMO_OBJECT_NAME:
                model_elem = model
                break
        if model_elem is None:
            panel._emit_log("[PICK_DEMO][RECOVER] model_pick_demo_missing_in_world")
            return None
        model_copy = copy.deepcopy(model_elem)
        pose_elem = model_copy.find("pose")
        if pose_elem is not None:
            model_copy.remove(pose_elem)
        static_elem = model_copy.find("static")
        if static_elem is None:
            static_elem = ET.SubElement(model_copy, "static")
        static_elem.text = "false"
        for link in model_copy.findall("link"):
            gravity = link.find("gravity")
            if gravity is None:
                gravity = ET.SubElement(link, "gravity")
            gravity.text = "true"
            kinematic = link.find("kinematic")
            if kinematic is None:
                kinematic = ET.SubElement(link, "kinematic")
            kinematic.text = "false"
        sdf_root = ET.Element("sdf", {"version": root.get("version") or "1.10"})
        sdf_root.append(model_copy)
        sdf_text = ET.tostring(sdf_root, encoding="unicode")
    except Exception as exc:
        panel._emit_log(f"[PICK_DEMO][RECOVER] world_parse_error={exc}")
        return None
    panel._pick_demo_recover_world_sdf = world_sdf
    panel._pick_demo_recover_sdf = sdf_text
    return sdf_text

def _run_gz_service_cli(panel,
    service_name: str,
    req_type: str,
    rep_type: str,
    req_text: str,
    *,
    timeout_ms: int,
    cmd_timeout_sec: float,
) -> Tuple[bool, str]:
    gz_cli = panel._resolve_gz_cli()
    if not gz_cli:
        return False, "gz_cli_missing"
    env_prefix = build_gz_env(resolve_gz_partition(panel.gz_partition))
    cmd = (
        f"{env_prefix}{shlex.quote(gz_cli)} service -s {service_name} "
        f"--reqtype {req_type} --reptype {rep_type} "
        f"--timeout {int(timeout_ms)} --req '{req_text}'"
    )
    try:
        result = subprocess.run(
            ["bash", "-lc", cmd],
            text=True,
            capture_output=True,
            timeout=max(0.8, float(cmd_timeout_sec)),
        )
    except Exception as exc:
        return False, f"service_exception:{exc}"
    stdout = str(result.stdout or "").strip()
    stderr = str(result.stderr or "").strip()
    detail = stderr or stdout or f"rc={result.returncode}"
    if result.returncode != 0:
        return False, detail
    return ("data: true" in stdout), detail

def _recover_pick_demo_to_table_gz(panel, reason: str, world_name: str) -> bool:
    sdf_text = panel._load_pick_demo_recover_sdf()
    if not sdf_text:
        panel._emit_log("[PICK_DEMO][RECOVER] gz_respawn_missing_sdf")
        return False
    delete_services = [
        f"/world/{world_name}/remove/blocking",
        f"/world/{world_name}/remove",
        f"/world/{world_name}/remove_entity/blocking",
        f"/world/{world_name}/remove_entity",
        f"/world/{world_name}/delete",
    ]
    spawn_services = [
        f"/world/{world_name}/create/blocking",
        f"/world/{world_name}/create",
        f"/world/{world_name}/spawn/blocking",
        f"/world/{world_name}/spawn",
    ]
    delete_req = f'name: "{PICK_DEMO_OBJECT_NAME}" type: MODEL'
    tx, ty, tz = panel._pick_demo_spawn_pose
    sdf_escaped = sdf_text.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "")
    spawn_req = (
        f'name: "{PICK_DEMO_OBJECT_NAME}" '
        "allow_renaming: false "
        f'sdf: "{sdf_escaped}" '
        "pose {"
        f"position {{x: {float(tx)} y: {float(ty)} z: {float(tz)}}} "
        "orientation {w: 1}"
        "}"
    )
    delete_detail = "delete_not_attempted"
    for service_name in delete_services:
        ok, detail = panel._run_gz_service_cli(
            service_name,
            "gz.msgs.Entity",
            "gz.msgs.Boolean",
            delete_req,
            timeout_ms=700,
            cmd_timeout_sec=1.6,
        )
        delete_detail = f"{service_name}:{detail}"
        if ok or "false" in str(detail).lower() or "not found" in str(detail).lower():
            break
    time.sleep(0.08)
    spawn_detail = "spawn_not_attempted"
    spawn_ok = False
    for service_name in spawn_services:
        ok, detail = panel._run_gz_service_cli(
            service_name,
            "gz.msgs.EntityFactory",
            "gz.msgs.Boolean",
            spawn_req,
            timeout_ms=1200,
            cmd_timeout_sec=2.8,
        )
        spawn_detail = f"{service_name}:{detail}"
        if ok:
            spawn_ok = True
            break
    if not spawn_ok:
        panel._emit_log(
            "[PICK_DEMO][RECOVER] "
            f"gz_respawn_failed reason={reason} delete={delete_detail} spawn={spawn_detail}"
        )
        return False
    time.sleep(0.2)
    panel._emit_log(
        "[PICK_DEMO][RECOVER] "
        f"gz_respawn_ok reason={reason} delete={delete_detail} spawn={spawn_detail}"
    )
    return True

def _maybe_hold_drop_objects(panel, reason: str) -> None:
    if not panel._drop_hold_enabled:
        return
    if panel._objects_release_done:
        return
    if not DROP_OBJECT_NAMES:
        return
    if panel._drop_hold_inflight:
        return
    now = time.monotonic()
    if (now - panel._drop_hold_last_ts) < 0.9:
        return
    panel._drop_hold_last_ts = now
    panel._drop_hold_inflight = True

    def worker():
        try:
            panel._hold_drop_objects(reason)
        finally:
            panel._drop_hold_inflight = False

    panel._run_async(worker)

def _maybe_recover_pick_demo(panel, reason: str) -> None:
    if panel._pick_demo_recover_inflight:
        return
    if getattr(panel, "_pick_target_lock_active", False) or getattr(panel, "_script_motion_active", False):
        return
    pos = get_object_position(PICK_DEMO_OBJECT_NAME)
    if pos is None or is_on_table(pos):
        return
    state = get_object_state(PICK_DEMO_OBJECT_NAME)
    allow_rearm_after_success = bool(getattr(panel, "_pick_demo_executed", False))
    if state is not None:
        if state.owner != ObjectOwner.NONE or state.attached:
            return
        if state.logical_state in (
            ObjectLogicalState.GRASPED,
            ObjectLogicalState.CARRIED,
        ):
            return
        if state.logical_state == ObjectLogicalState.RELEASED and not allow_rearm_after_success:
            return
    now = time.monotonic()
    if (now - panel._pick_demo_recover_last_ts) < 1.5:
        return
    panel._pick_demo_recover_last_ts = now
    panel._pick_demo_recover_inflight = True

    def worker() -> None:
        try:
            panel._recover_pick_demo_to_table(reason)
        finally:
            panel._pick_demo_recover_inflight = False

    panel._run_async(worker, name="pick_demo_recover")

def _recover_pick_demo_to_table(panel, reason: str) -> None:
    world_name = panel._gz_world_name or read_world_name(panel.world_combo.currentText().strip()) or GZ_WORLD
    tx, ty, tz = panel._pick_demo_spawn_pose
    pose_reset_ok = False
    if ROS_AVAILABLE and SetEntityPose is not None:
        panel._ensure_moveit_node()
        if panel._moveit_node is not None:
            service_name = panel._set_pose_service_name or panel._resolve_set_pose_service(world_name)
            if service_name:
                panel._set_pose_service_name = service_name
                client = panel._moveit_node.create_client(SetEntityPose, service_name)
                if client.wait_for_service(timeout_sec=0.4):
                    req = SetEntityPose.Request()
                    req.entity.name = PICK_DEMO_OBJECT_NAME
                    req.entity.type = GzEntity.MODEL if GzEntity else 2
                    req.pose.position.x = float(tx)
                    req.pose.position.y = float(ty)
                    req.pose.position.z = float(tz)
                    req.pose.orientation.w = 1.0
                    future = client.call_async(req)
                    rclpy.spin_until_future_complete(panel._moveit_node, future, timeout_sec=0.8)
                    result = future.result() if future.done() else None
                    pose_reset_ok = bool(result and bool(getattr(result, "success", False)))
    if not pose_reset_ok:
        pose_reset_ok = panel._recover_pick_demo_to_table_gz(reason, world_name)
    if not pose_reset_ok:
        panel._emit_log("[PICK_DEMO][RECOVER] pose_reset_failed")
        return
    bulk_update_object_positions(
        {PICK_DEMO_OBJECT_NAME: (float(tx), float(ty), float(tz))},
        source=f"pick_demo_recover:{reason}",
        objects_stable=True,
    )
    recalc_object_states("pick_demo_recover")
    panel._emit_log(
        "[PICK_DEMO][RECOVER] "
        f"pose_reset_ok reason={reason} target=({float(tx):.3f},{float(ty):.3f},{float(tz):.3f})"
    )
    if getattr(panel, "_pick_demo_executed", False):
        def _rearm_demo_button() -> None:
            panel._pick_demo_executed = False
            if getattr(panel, "btn_pick_demo", None) is not None:
                panel.btn_pick_demo.setToolTip("Demo (secuencia joints, sin MoveIt)")
            panel._emit_log("[PICK_DEMO][RECOVER] rearmed=true")
            panel._refresh_controls()

        panel.signal_run_ui.emit(_rearm_demo_button)
    panel.signal_update_objects.emit()
    panel.signal_refresh_controls.emit()

def _hold_drop_objects(panel, reason: str) -> None:
    if not ROS_AVAILABLE or SetEntityPose is None:
        panel._hold_drop_objects_gz(reason)
        return
    panel._ensure_moveit_node()
    if panel._moveit_node is None:
        panel._hold_drop_objects_gz(reason, fallback_note="[PHYSICS][HOLD] nodo ROS no listo")
        return
    world_name = panel._gz_world_name or read_world_name(panel.world_combo.currentText().strip()) or GZ_WORLD
    service_name = panel._set_pose_service_name or panel._resolve_set_pose_service(world_name)
    if not service_name:
        panel._hold_drop_objects_gz(reason, fallback_note="[PHYSICS][HOLD] servicio SetEntityPose no encontrado")
        return
    panel._set_pose_service_name = service_name
    client = panel._moveit_node.create_client(SetEntityPose, service_name)
    if not client.wait_for_service(timeout_sec=0.4):
        panel._hold_drop_objects_gz(reason, fallback_note=f"[PHYSICS][HOLD] servicio no disponible: {service_name}")
        return
    panel._drop_hold_warned = False
    positions = get_object_positions()
    held = 0
    for name in DROP_OBJECT_NAMES:
        target = panel._drop_spawn_positions.get(name)
        if not target:
            continue
        current = positions.get(name)
        if current and current[2] >= (target[2] - 0.05):
            continue
        tx, ty, tz = target
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = GzEntity.MODEL if GzEntity else 2
        req.pose.position.x = float(tx)
        req.pose.position.y = float(ty)
        req.pose.position.z = float(tz)
        req.pose.orientation.w = 1.0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(panel._moveit_node, future, timeout_sec=0.6)
        if future.done() and future.result() and future.result().success:
            held += 1
    now = time.monotonic()
    if held and (now - panel._drop_hold_log_ts) > 2.0:
        panel._emit_log(f"[PHYSICS][HOLD] held={held} reason={reason}")
        panel._drop_hold_log_ts = now

def _hold_drop_objects_gz(panel, reason: str, fallback_note: str = "") -> None:
    if panel._drop_hold_gz_available is None:
        panel._drop_hold_gz_available = shutil.which("gz") is not None
    if not panel._drop_hold_gz_available:
        if not panel._drop_hold_gz_warned:
            note = " (gz CLI no disponible)" if not fallback_note else f" ({fallback_note})"
            panel._emit_log(f"[PHYSICS][HOLD] fallback deshabilitado{note}")
            panel._drop_hold_gz_warned = True
        return
    if fallback_note and not panel._drop_hold_warned:
        panel._emit_log(fallback_note)
        panel._drop_hold_warned = True
    world_name = panel._gz_world_name or read_world_name(panel.world_combo.currentText().strip()) or GZ_WORLD
    now = time.monotonic()
    if (now - panel._drop_hold_gz_last_check) >= panel._drop_hold_gz_check_interval:
        panel._drop_hold_gz_last_check = now
        env_prefix = build_gz_env(resolve_gz_partition(panel.gz_partition))
        try:
            result = subprocess.run(
                ["bash", "-lc", f"{env_prefix}gz service -l"],
                text=True,
                capture_output=True,
                timeout=2.5,
            )
            candidates = [line.strip() for line in (result.stdout or "").splitlines() if line.strip()]
            wanted = [
                f"/world/{world_name}/set_entity_pose",
                f"/world/{world_name}/set_pose",
            ]
            for name in wanted:
                if name in candidates:
                    panel._drop_hold_gz_service_name = name
                    break
            if not panel._drop_hold_gz_service_name:
                for name in candidates:
                    if name.endswith("/set_entity_pose") or name.endswith("/set_pose"):
                        if f"/world/{world_name}/" in name:
                            panel._drop_hold_gz_service_name = name
                            break
            if panel._drop_hold_gz_service_name:
                panel._emit_log(f"[PHYSICS][HOLD] gz_service={panel._drop_hold_gz_service_name}")
            elif not panel._drop_hold_gz_warned:
                panel._emit_log("[PHYSICS][HOLD] gz service no encontrado (set_entity_pose/set_pose)")
                panel._drop_hold_gz_warned = True
        except Exception as exc:
            if not panel._drop_hold_gz_warned:
                panel._emit_log(f"[PHYSICS][HOLD] gz service list error: {exc}")
                panel._drop_hold_gz_warned = True
    service_name = panel._drop_hold_gz_service_name
    positions = {}
    poses = panel._read_world_pose_info(world_name)
    if poses:
        for pose in poses:
            name = pose.get("name")
            pos = pose.get("position") or {}
            if not isinstance(name, str):
                continue
            try:
                positions[name] = (
                    float(pos.get("x") or 0.0),
                    float(pos.get("y") or 0.0),
                    float(pos.get("z") or 0.0),
            )
            except Exception:
                continue
    held = 0
    service_candidates = [
        service_name,
        f"/world/{world_name}/set_entity_pose",
        f"/world/{world_name}/set_pose",
    ]
    service_candidates = [s for s in service_candidates if s]
    for name in DROP_OBJECT_NAMES:
        target = panel._drop_spawn_positions.get(name)
        if not target:
            continue
        current = positions.get(name)
        if current and current[2] >= (target[2] - 0.05):
            continue
        tx, ty, tz = target
        req = (
            f'name: "{name}" '
            f"position {{x: {float(tx)} y: {float(ty)} z: {float(tz)}}} "
            "orientation {w: 1}"
        )
        env_prefix = build_gz_env(resolve_gz_partition(panel.gz_partition))
        ok = False
        for svc in service_candidates:
            cmd = (
                f"{env_prefix}gz service -s {svc} "
                "--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                f"--timeout 800 --req '{req}'"
            )
            try:
                res = subprocess.run(
                    ["bash", "-lc", cmd],
                    text=True,
                    capture_output=True,
                    timeout=1.6,
            )
            except Exception as exc:
                if not panel._drop_hold_gz_warned:
                    panel._emit_log(f"[PHYSICS][HOLD] gz service error: {exc}")
                    panel._drop_hold_gz_warned = True
                break
            if res.returncode == 0:
                ok = True
                if panel._drop_hold_gz_service_name != svc:
                    panel._drop_hold_gz_service_name = svc
                    panel._emit_log(f"[PHYSICS][HOLD] gz_service={svc}")
                break
        if not ok and not panel._drop_hold_gz_warned:
            msg = (res.stderr or res.stdout or "").strip() if "res" in locals() else ""
            panel._emit_log(f"[PHYSICS][HOLD] gz service rc!=0 {msg}")
            panel._drop_hold_gz_warned = True
        if ok:
            held += 1
    now = time.monotonic()
    if held and (now - panel._drop_hold_log_ts) > 2.0:
        panel._emit_log(f"[PHYSICS][HOLD] held={held} reason={reason} via=gz")
        panel._drop_hold_log_ts = now

def _drop_hold_tick(panel) -> None:
    if panel._closing:
        return
    if not panel._gz_running:
        return
    if not panel._drop_hold_enabled:
        return
    if panel._objects_release_done:
        return
    panel._maybe_hold_drop_objects("timer")

def _nudge_drop_objects(panel, reason: str) -> None:
    if not ROS_AVAILABLE or SetEntityPose is None:
        panel._emit_log("[PHYSICS][NUDGE] no disponible (SetEntityPose)")
        return
        panel._ensure_moveit_node()
    if panel._moveit_node is None:
        panel._emit_log("[PHYSICS][NUDGE] nodo ROS no listo")
        return
    world_name = panel._gz_world_name or read_world_name(panel.world_combo.currentText().strip()) or GZ_WORLD
    service_name = panel._set_pose_service_name or panel._resolve_set_pose_service(world_name)
    if not service_name:
        panel._emit_log("[PHYSICS][NUDGE] servicio SetEntityPose no encontrado")
        return
    panel._set_pose_service_name = service_name
    client = panel._moveit_node.create_client(SetEntityPose, service_name)
    if not client.wait_for_service(timeout_sec=0.6):
        panel._emit_log(f"[PHYSICS][NUDGE] servicio no disponible: {service_name}")
        return
    positions = get_object_positions()
    nudged = 0
    for name in DROP_OBJECT_NAMES:
        pos = positions.get(name)
        if not pos:
            continue
        x, y, z = pos
        if z < NUDGE_DROP_Z_MIN:
            continue
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = GzEntity.MODEL if GzEntity else 2
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z - NUDGE_DROP_DZ)
        req.pose.orientation.w = 1.0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(panel._moveit_node, future, timeout_sec=0.6)
        if future.done() and future.result() and future.result().success:
            nudged += 1
    panel._emit_log(f"[PHYSICS][NUDGE] nudged={nudged} reason={reason}")
def _find_attach_candidate(panel,
    preferred_name: Optional[str] = None,
) -> Optional[str]:
    tcp_world = panel._last_tcp_world
    if not tcp_world:
        return None
    positions = get_object_positions()
    if not positions:
        return None
    tcp_x, tcp_y, tcp_z = tcp_world
    selected_name = (preferred_name or panel._selected_object or "").strip()
    if selected_name:
        pos = positions.get(selected_name)
        if pos is not None:
            dx = pos[0] - tcp_x
            dy = pos[1] - tcp_y
            dz = pos[2] - tcp_z
            if (dx * dx + dy * dy + dz * dz) ** 0.5 <= ATTACH_DIST_M:
                return panel._normalize_attach_name(selected_name)
    best_name = None
    best_dist = 1e9
    for name, pos in positions.items():
        dx = pos[0] - tcp_x
        dy = pos[1] - tcp_y
        dz = pos[2] - tcp_z
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        if dist < best_dist:
            best_dist = dist
            best_name = name
    if best_name and best_dist <= ATTACH_DIST_M:
        return panel._normalize_attach_name(best_name)
    return None

def _attempt_attach(panel,
    reason: str,
    *,
    selected_name: Optional[str] = None,
    tcp_base: Optional[Tuple[float, float, float]] = None,
    object_base: Optional[Tuple[float, float, float]] = None,
    object_height: Optional[float] = None,
    base_frame: Optional[str] = None,
    xy_tol_m: float = 0.02,
    z_tol_m: float = 0.04,
    z_ref_mode: Optional[str] = None,
    z_clearance_m: float = 0.0,
    tcp_stamp_ns: Optional[int] = None,
    obj_stamp_ns: Optional[int] = None,
    tcp_z_correction_m: Optional[float] = None,
) -> bool:
    """Intenta vincular el objeto seleccionado al gripper en Gazebo."""
    positions = get_object_positions() or {}
    selected = (selected_name or panel._selected_object or "").strip()
    lock_active = bool(getattr(panel, "_pick_target_lock_active", False))
    lock_name = str(getattr(panel, "_pick_target_lock_name", "") or "")
    lock_id = str(getattr(panel, "_pick_target_lock_id", "") or "n/a")
    if lock_active:
        panel._emit_log(
            f"[ATTACH][LOCK] active=true lock_id={lock_id} lock_name={lock_name or 'none'} "
            f"selected_arg={selected or 'none'} reason={reason}"
        )
        if selected and lock_name and selected != lock_name:
            panel._log_error(
                "[ATTACH] target_mismatch_with_lock "
                f"lock_id={lock_id} lock_name={lock_name} selected={selected} reason={reason}"
            )
            return False
    attach_name = None
    resolved_entity = None
    if selected:
        if selected not in positions:
            panel._log_error(
                f"[ATTACH] selected_missing_in_positions selected={selected} reason={reason}"
            )
            return False
        resolved_entity = selected
        attach_name = panel._normalize_attach_name(selected)
    if attach_name is None:
        attach_name = panel._find_attach_candidate(preferred_name=selected)
        resolved_entity = attach_name
    panel._emit_log(
        f"[PICK][ATTACH] selected={selected or 'none'} attach_name={attach_name or 'none'} "
        f"resolved_entity={resolved_entity or 'none'} reason={reason}"
    )
    if not attach_name:
        panel._log_warning(f"[PICK] Sin objeto para vincular: selected={selected or 'none'} reason={reason}")
        return False

    attach_entity = resolved_entity or attach_name
    obj_world = positions.get(attach_entity)
    frame_name = (base_frame or panel._base_frame_effective or BASE_FRAME or "base_link").strip()
    tcp_base_pos = tcp_base
    if tcp_base_pos is None and panel._last_tcp_world is not None:
        tcp_base_pos = world_to_base(*panel._last_tcp_world)
    obj_base_pos = object_base
    if obj_base_pos is None and obj_world is not None:
        obj_base_pos = world_to_base(*obj_world)

    if tcp_base_pos is not None and obj_base_pos is not None:
        obj_height = max(0.0, float(object_height or 0.0))
        obj_center_z = float(obj_base_pos[2])
        obj_top_z = obj_center_z + (obj_height * 0.5 if obj_height > 0.0 else 0.0)
        mode = str(z_ref_mode or _get_panel_ros_params().attach_z_ref_mode).strip().lower()
        if mode not in ("center", "top"):
            mode = "top"
        z_clearance = float(z_clearance_m or 0.0)
        obj_ref_z = obj_center_z if mode == "center" else obj_top_z
        obj_ref_z += z_clearance
        # Si no se especifica correccion explícita, derivarla del frame operativo
        # para no duplicar offsets legacy fuera del URDF canónico.
        _tz_corr = (
            float(tcp_z_correction_m)
            if tcp_z_correction_m is not None
            else float(
                contact_z_correction_for_frame(
                    getattr(panel, "_ee_frame_effective", "") or panel._required_ee_frame or "rg2_pinch_center"
                )
            )
        )
        tcp_contact_z = float(tcp_base_pos[2]) - _tz_corr
        dx = float(obj_base_pos[0]) - float(tcp_base_pos[0])
        dy = float(obj_base_pos[1]) - float(tcp_base_pos[1])
        dz = obj_ref_z - tcp_contact_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        tcp_stamp_txt = "n/a" if not tcp_stamp_ns else str(int(tcp_stamp_ns))
        obj_stamp_txt = "n/a" if not obj_stamp_ns else str(int(obj_stamp_ns))
        panel._emit_log(
            "[PICK][ATTACH][GEOM] "
            f"frame={frame_name} tcp=({float(tcp_base_pos[0]):.3f},{float(tcp_base_pos[1]):.3f},{float(tcp_base_pos[2]):.3f}) "
            f"tcp_contact_z={tcp_contact_z:.3f} tcp_z_correction={_tz_corr:.3f} "
            f"obj_center=({float(obj_base_pos[0]):.3f},{float(obj_base_pos[1]):.3f},{obj_center_z:.3f}) "
            f"obj_top_z={obj_top_z:.3f} obj_height={obj_height:.3f} "
            f"z_ref_mode={mode} z_clearance={z_clearance:.3f} obj_ref_z={obj_ref_z:.3f} "
            f"tcp_stamp_ns={tcp_stamp_txt} obj_stamp_ns={obj_stamp_txt}"
        )
        panel._emit_log(
            "[PICK][ATTACH] "
            f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} dist={dist:.3f} "
            f"thresh=(xy={xy_tol_m:.3f},z={z_tol_m:.3f}) frame={frame_name} "
            f"z_ref_mode={mode}"
        )
        if abs(dx) > float(xy_tol_m) or abs(dy) > float(xy_tol_m) or abs(dz) > float(z_tol_m):
            panel._emit_log(
                "[PICK][ATTACH] range_reject "
                f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} "
                f"(xy_tol={xy_tol_m:.3f}, z_tol={z_tol_m:.3f})"
            )
            return False
    else:
        panel._emit_log(
            "[PICK][ATTACH] geometry_unavailable "
            f"tcp_base={'ok' if tcp_base_pos is not None else 'none'} "
            f"obj_base={'ok' if obj_base_pos is not None else 'none'}"
        )

    attach_topic = f"{GRIPPER_ATTACH_PREFIX}/{attach_name}/attach"
    try:
        pub = panel._get_attach_publisher(attach_topic)
        if pub is not None:
            pub_count = -1
            sub_count = -1
            if panel._ros_worker_started and panel.ros_worker.node_ready():
                pub_count = int(panel.ros_worker.topic_publisher_count(attach_topic))
                sub_count = int(panel.ros_worker.topic_subscriber_count(attach_topic))
            panel._emit_log(
                f"[PICK][ATTACH] pub_count={pub_count} sub_count={sub_count} topic={attach_topic}"
            )
            if sub_count <= 0:
                wait_deadline = time.time() + 1.2
                while time.time() < wait_deadline:
                    time.sleep(0.2)
                    if panel._ros_worker_started and panel.ros_worker.node_ready():
                        sub_count = int(panel.ros_worker.topic_subscriber_count(attach_topic))
                        if sub_count > 0:
                            break
                if sub_count == 0:
                    panel._log_error(
                        f"[PICK][ATTACH] attach_backend_missing topic={attach_topic} "
                        "reason=no_subscribers"
                    )
                    return False
            pub.publish(Empty())
            panel._emit_log(
                f"[PICK][ATTACH] publish attach_name={attach_name} topic={attach_topic}"
            )
            return True
    except Exception as exc:
        panel._log_error(f"[PICK] Error al vincular: {exc}")
    return False

def _schedule_attach_attempt(panel, reason: str, delay_ms: int = 150) -> None:
    """Programa un intento de attach después de cerrar el gripper."""
    def attempt():
        panel._attempt_attach(reason)
    QTimer.singleShot(delay_ms, attempt)
