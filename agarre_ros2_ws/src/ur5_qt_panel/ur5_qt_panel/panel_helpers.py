#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_helpers.py
# Contenido: Helper method callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Helper method callbacks (readiness, logging, UI controls, step mode) for ControlPanelV2."""
from __future__ import annotations

import hashlib
import math
import os
import time
import threading
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

try:
    import psutil
except ImportError:
    psutil = None

from .panel_config import (
    CAMERA_READY_MAX_AGE_SEC,
    DROP_OBJECT_NAMES,
    ROS_AVAILABLE,
    STATUS_TOPIC_CACHE_SEC,
    WS_DIR,
)
from .panel_state import MoveItState, SystemState, EXTERNAL_STATE_MAP
from .panel_external_state import (
    external_state_active,
    resolve_external_state,
    apply_external_system_state,
)
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME


def _log_exception(context: str, exc: Exception) -> None:
    print(f"[HELPERS][ERROR][{context}] {exc}")


def _moveit_not_ready_reason(panel) -> str:
    return moveit_not_ready_reason(panel)

def _set_moveit_wait_status(panel, label: str, reason: Optional[str] = None) -> str:
    return set_moveit_wait_status(panel, label, reason)

def _controllers_not_ready_reason(panel) -> str:
    return controllers_not_ready_reason(panel)

@staticmethod
def _ros_node_not_ready_reason() -> str:
    return ros_node_not_ready_reason()

@staticmethod
def _controller_manager_not_ready_reason() -> str:
    return controller_manager_not_ready_reason()

@staticmethod
def _list_controllers_not_ready_reason(kind: str) -> str:
    return list_controllers_not_ready_reason(kind)

def _camera_not_ready_reason(panel) -> str:
    now = _runtime_time()
    _ready, fault, source_down, age, in_grace = panel._camera_runtime_flags(now)
    depth_required, depth_topic = panel._camera_depth_expectation()
    depth_age = now - panel._last_camera_depth_frame_ts if panel._last_camera_depth_frame_ts else float("inf")
    rgb_topic = panel.camera_topic_combo.currentText().strip() if hasattr(panel, "camera_topic_combo") else panel.camera_topic
    rgb_pub_count = panel.ros_worker.topic_publisher_count(rgb_topic) if panel._ros_worker_started and panel.ros_worker.node_ready() and rgb_topic else 0
    depth_pub_count = (
        panel.ros_worker.topic_publisher_count(depth_topic)
        if depth_required and panel._ros_worker_started and panel.ros_worker.node_ready() and depth_topic
        else 0
    )
    if source_down:
        return f"camera_source_down ({panel._gazebo_state()})"
    if panel._camera_frame_count <= 0 and in_grace:
        return "camera_warmup"
    if panel._camera_frame_count <= 0:
        return (
            "camera_no_frames "
            f"topic={rgb_topic or 'n/a'} pubs={rgb_pub_count} "
            f"last_age={'inf' if math.isinf(age) else f'{age:.1f}s'}"
        )
    if depth_required and panel._camera_depth_frame_count <= 0:
        return (
            "camera_depth_warmup "
            f"topic={depth_topic or 'n/a'} pubs={depth_pub_count} "
            f"last_age={'inf' if math.isinf(depth_age) else f'{depth_age:.1f}s'}"
        )
    if depth_required and depth_age >= CAMERA_READY_MAX_AGE_SEC:
        return (
            "camera_depth_stale "
            f"topic={depth_topic or 'n/a'} age={'inf' if math.isinf(depth_age) else f'{depth_age:.1f}s'}"
        )
    if fault:
        return f"camera_fault age={age:.1f}s"
    return camera_not_ready_reason(None)

def _tfm_experiment_ready_status(panel) -> Tuple[bool, str]:
    if not panel.tfm_module:
        return False, "modelo no disponible"
    if not bool(getattr(panel, "_tfm_experiment_applied", False)):
        return False, "aplica un experimento primero"
    return True, ""

def _tfm_infer_ready_status(panel) -> Tuple[bool, str]:
    if panel._tfm_infer_inflight:
        return False, "inferencia en curso"
    if not panel.tfm_module:
        return False, "modelo no disponible"
    if not bool(getattr(panel, "_camera_subscribed", False)):
        return False, "cámara no conectada"
    frame_snapshot = panel._latest_camera_frame_snapshot()
    if not frame_snapshot:
        return False, "sin frame de cámara"
    _qimg, w, h, frame_ts = frame_snapshot
    if w <= 0 or h <= 0:
        return False, "frame inválido"
    now = _runtime_time()
    camera_ready, _fault, _source_down, age, _in_grace = panel._camera_runtime_flags(now)
    if not camera_ready:
        camera_reason = panel._camera_not_ready_reason()
        if not (
            camera_reason.startswith("camera_depth_warmup")
            or camera_reason.startswith("camera_depth_stale")
        ):
            return False, camera_reason
    if frame_ts:
        frame_age = max(0.0, now - float(frame_ts))
        infer_frame_max_age_sec = max(
            max(0.2, float(CAMERA_READY_MAX_AGE_SEC)),
            float(os.environ.get("PANEL_TFM_INFER_FRAME_MAX_AGE_SEC", "4.0") or 4.0),
        )
        if frame_age >= infer_frame_max_age_sec:
            return False, f"frame stale age={frame_age:.2f}s"
    depth_required, depth_topic = panel._camera_depth_expectation()
    if depth_required:
        ros_worker = getattr(panel, "ros_worker", None)
        depth_frame = None
        if ros_worker is not None:
            try:
                depth_frame = ros_worker.get_latest_depth_frame(depth_topic)
            except Exception:
                depth_frame = None
            if not depth_frame:
                try:
                    depth_snapshot = ros_worker.image_frame_snapshot(depth_topic)
                except Exception:
                    depth_snapshot = None
                try:
                    depth_types = next(
                        (types for name, types in ros_worker.topic_names_and_types() if name == depth_topic),
                        [],
                    )
                except Exception:
                    depth_types = []
                try:
                    depth_subs = int(ros_worker.topic_subscriber_count(depth_topic))
                except Exception:
                    depth_subs = -1
                try:
                    depth_pubs = int(ros_worker.topic_publisher_count(depth_topic))
                except Exception:
                    depth_pubs = -1
                panel._emit_log_throttled(
                    f"tfm_depth_diag:{depth_topic}",
                    (
                        f"[TFM][DEPTH] warmup topic={depth_topic or 'n/a'} pubs={depth_pubs} "
                        f"subs={depth_subs} types={','.join(depth_types) or 'n/a'} "
                        f"snapshot={'yes' if depth_snapshot else 'no'} "
                        f"panel_frames={int(getattr(panel, '_camera_depth_frame_count', 0) or 0)}"
                    ),
                    min_interval=2.0,
                )
                camera_ctrl = getattr(panel, "_camera_ctrl", None)
                resubscribe_ts = float(getattr(panel, "_last_tfm_depth_resubscribe_ts", 0.0) or 0.0)
                if camera_ctrl is not None and (now - resubscribe_ts) >= 2.0:
                    panel._last_tfm_depth_resubscribe_ts = now
                    try:
                        camera_ctrl._ensure_depth_subscription()
                    except Exception:
                        pass
                    try:
                        camera_ctrl._sync_from_worker_snapshot(now=now)
                    except Exception:
                        pass
                    try:
                        depth_frame = ros_worker.get_latest_depth_frame(depth_topic)
                    except Exception:
                        depth_frame = None
        if not depth_frame:
            return False, f"camera_depth_warmup topic={depth_topic} last_age=inf"
        _depth_img, depth_ts = depth_frame
        depth_age = max(0.0, now - float(depth_ts))
        if depth_age >= CAMERA_READY_MAX_AGE_SEC:
            return False, f"camera_depth_stale topic={depth_topic} age={depth_age:.1f}s"
    return True, ""

@staticmethod
def _tfm_infer_waitable_reason(reason: str) -> bool:
    text = str(reason or "").strip().lower()
    if not text:
        return False
    if text in {
        "cámara no conectada",
        "cámara no lista",
        "sin frame de cámara",
    }:
        return True
    return text.startswith("frame stale age=") or text.startswith("camera_")

def _current_grasp_status(panel) -> Tuple[bool, str]:
    if panel._tfm_infer_inflight:
        return False, "inferencia en curso"
    if panel._tfm_execute_inflight:
        return False, "ejecución en curso"
    if not panel._last_grasp_px:
        return False, "sin grasp"
    grasp_ts = float(getattr(panel, "_last_grasp_update_ts", 0.0) or 0.0)
    max_age_sec = max(
        5.0,
        float(os.environ.get("PANEL_TFM_GRASP_MAX_AGE_SEC", "60.0") or 60.0),
    )
    if grasp_ts > 0.0:
        age = max(0.0, _runtime_time() - grasp_ts)
        if age > max_age_sec:
            return False, f"grasp expirado age={age:.1f}s max_age={max_age_sec:.1f}s"
    current_selection = str(getattr(panel, "_selected_object", "") or "").strip()
    grasp_selection = str(getattr(panel, "_last_grasp_selection_name", "") or "").strip()
    if current_selection and grasp_selection and current_selection != grasp_selection:
        return False, f"grasp no corresponde a la selección actual ({grasp_selection} -> {current_selection})"
    if not panel._last_grasp_base and not panel._last_grasp_world:
        return False, "grasp base_link no disponible"
    return True, ""

def _restore_execute_selection_context(panel) -> None:
    snapshot = getattr(panel, "_last_infer_selection_snapshot", None)
    if isinstance(snapshot, dict) and snapshot:
        panel._restore_infer_selection_snapshot(snapshot)
    grasp_selection = str(getattr(panel, "_last_grasp_selection_name", "") or "").strip()
    if not grasp_selection:
        return
    if not str(getattr(panel, "_selected_object", "") or "").strip():
        panel._selected_object = grasp_selection
    if not str(getattr(panel, "_selection_last_user_name", "") or "").strip():
        panel._selection_last_user_name = grasp_selection
    if float(getattr(panel, "_selection_last_user_ts", 0.0) or 0.0) <= 0.0:
        panel._selection_last_user_ts = float(time.time())
    panel._ensure_selected_object_in_store(
        grasp_selection,
        reason="execute_restore_from_grasp",
    )

def _calibration_action_status(panel) -> Tuple[bool, str]:
    if getattr(panel, "_pick_target_lock_active", False):
        return False, "PICK activo"
    ok, reason = panel._basic_ready_status()
    if not ok:
        return False, reason
    if not panel._tf_ready_state:
        return False, panel._tf_not_ready_reason()
    if not panel._calibration_topic_allowed():
        return False, f"solo disponible en {CALIBRATION_CAMERA_TOPIC}"
    if not panel._objects_settled:
        return False, "objetos no estabilizados"
    if not panel._pose_info_ok:
        return False, "pose/info no disponible"
    if panel._camera_required and not panel._camera_stream_ok:
        return False, "cámara no publica"
    return True, ""

@staticmethod
def _pose_info_not_ready_reason() -> str:
    return pose_info_not_ready_reason(None)

def _tf_not_ready_reason(panel) -> str:
    return tf_not_ready_reason(panel)

def _moveit_control_status(panel) -> Tuple[bool, str]:
    return moveit_control_status(panel)

def _manual_control_status(panel) -> Tuple[bool, str]:
    return manual_control_status(panel)

def _external_publishers_for_topic(panel, topic: str) -> List[str]:
    if not topic or not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return []
    pubs = panel.ros_worker.publisher_nodes_by_topic(topic)
    if not pubs:
        return []
    self_name = panel.ros_worker.node_name()
    self_ns = panel.ros_worker.node_namespace()
    self_full = f"{self_ns}/{self_name}".replace("//", "/") if self_name else ""
    bridge_name = "ur5_moveit_bridge"
    bridge_ns = panel.ros_worker.node_namespace()
    bridge_full = f"{bridge_ns}/{bridge_name}".replace("//", "/") if bridge_name else ""
    bridge_active = panel._proc_alive(panel.moveit_bridge_proc) or panel._moveit_bridge_detected()
    moveit_name = ""
    moveit_ns = ""
    moveit_full = ""
    if panel._moveit_node is not None:
        try:
            moveit_name = panel._moveit_node.get_name()
            moveit_ns = panel._moveit_node.get_namespace()
            moveit_full = f"{moveit_ns}/{moveit_name}".replace("//", "/") if moveit_name else ""
        except Exception:
            moveit_name = ""
            moveit_ns = ""
            moveit_full = ""
    externals: List[str] = []
    for pub in pubs:
        if not pub:
            continue
        # Some DDS graph entries expose anonymous placeholders
        # ("_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_").
        # Treat them as introspection noise and do not block TEST ROBOT.
        if "_NODE_NAME_UNKNOWN_" in pub or "_NODE_NAMESPACE_UNKNOWN_" in pub:
            panel._emit_log_throttled(
                f"TEST:ignore_unknown_pub:{topic}",
                f"[SAFETY] TEST ROBOT: ignorando publisher desconocido en {topic}: {pub}",
            )
            continue
        if self_name and (pub == self_name or pub.endswith(f"/{self_name}")):
            continue
        if self_full and pub == self_full:
            continue
        if not bridge_active:
            if bridge_name and (pub == bridge_name or pub.endswith(f"/{bridge_name}")):
                continue
            if bridge_full and pub == bridge_full:
                continue
        if moveit_name and (pub == moveit_name or pub.endswith(f"/{moveit_name}")):
            continue
        if moveit_full and pub == moveit_full:
            continue
        externals.append(pub)
    return sorted(set(externals))

def _bridge_publishers_only(panel, externals: List[str]) -> bool:
    if not externals:
        return False
    names = {"ur5_moveit_bridge", "/ur5_moveit_bridge"}
    ns = panel.ros_worker.node_namespace()
    if ns:
        names.add(f"{ns}/ur5_moveit_bridge".replace("//", "/"))
    return all(pub in names for pub in externals)

def _set_robot_test_blocked(panel, reason: Optional[str]) -> None:
    if reason == panel._robot_test_block_reason:
        return
    panel._robot_test_block_reason = reason
    if not reason:
        panel._robot_test_cleanup_pending = False
        panel._robot_test_cleanup_retries = 0
    if reason:
        panel._set_status(f"TEST ROBOT bloqueado: {reason}", error=True)

def _await_external_publishers_clear(panel,
    topic: str,
    deadline_ts: float,
    on_clear,
    on_timeout,
) -> None:
    if panel._closing:
        return
    if not topic:
        on_timeout()
        return
    externals = panel._external_publishers_for_topic(topic)
    if externals and panel._bridge_publishers_only(externals):
        on_clear()
        return
    if not externals:
        on_clear()
        return
    if time.time() >= deadline_ts:
        on_timeout()
        return
    QTimer.singleShot(100, lambda: panel._await_external_publishers_clear(topic, deadline_ts, on_clear, on_timeout))

def _schedule_robot_test_cleanup_check(panel, topic: str, *, delay_ms: int = 500, max_retries: int = 6
) -> None:
    if not topic:
        return
    panel._robot_test_cleanup_topic = topic
    if panel._robot_test_cleanup_pending:
        return
    panel._robot_test_cleanup_pending = True
    panel._robot_test_cleanup_retries = max_retries

    def _check():
        if not panel._robot_test_cleanup_pending:
            return
        if panel._robot_test_cleanup_retries <= 0:
            panel._robot_test_cleanup_pending = False
            return
        panel._robot_test_cleanup_retries -= 1
        externals = panel._external_publishers_for_topic(panel._robot_test_cleanup_topic)
        if not externals:
            panel._robot_test_cleanup_pending = False
            panel._set_robot_test_blocked(None)
            panel._set_status("TEST ROBOT desbloqueado: limpieza OK", error=False)
            return
        panel._set_status("TEST ROBOT bloqueado: esperando limpieza de publishers...", error=False)
        QTimer.singleShot(delay_ms, _check)

    QTimer.singleShot(delay_ms, _check)

def _update_camera_topics_async(panel, topics: object) -> None:
    panel._camera_ctrl.update_topics_async(topics)

def _emit_log(panel, msg: str, *, flush: bool = True):
    """Print a timestamped log line."""
    if not panel._should_emit_log(msg):
        return
    print(timestamped_line(msg), flush=flush)

def _metric_mark(panel, label: str) -> None:
    if not panel._metrics_enabled:
        return
    if label in panel._perf_marks:
        return
    elapsed = time.monotonic() - panel._perf_start_monotonic
    panel._perf_marks[label] = elapsed
    panel._emit_log(f"[METRICS] {label}={elapsed:.2f}s")

def _audit_root(panel) -> Path:
    return Path(WS_DIR).parent / "auditoria" / "panel_audit"

def _audit_append(panel, rel_path: str, msg: str) -> None:
    try:
        out_path = panel._audit_root() / rel_path
        ensure_dir(str(out_path.parent))
        with out_path.open("a", encoding="utf-8") as f:
            f.write(f"{timestamped_line(msg)}\n")
    except Exception:
        pass

def _audit_write_json(panel, rel_path: str, payload: Dict[str, object]) -> None:
    try:
        out_path = panel._audit_root() / rel_path
        ensure_dir(str(out_path.parent))
        out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    except Exception:
        pass

def _sha256_file(panel, path: str) -> str:
    try:
        import hashlib
    except Exception:
        return ""
    h = hashlib.sha256()
    try:
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)
    except Exception:
        return ""
    return h.hexdigest()

def _should_emit_log(panel, msg: str) -> bool:
    if getattr(panel, "_debug_logs_enabled", False):
        return True
    if msg.startswith("[STARTUP]"):
        return True
    if msg.startswith("[SHUTDOWN]"):
        return True
    if msg.startswith("[TFM]"):
        return True
    if msg.startswith("[BTN]"):
        return True
    if msg.startswith("[ERROR]"):
        return True
    if msg.startswith("[PICK]"):
        return True
    if msg.startswith("[PICK_OBJ]"):
        return True
    if msg.startswith("[PICK_OBJ_DEBUG]"):
        return True
    if msg.startswith("[MOVEIT2]"):
        return True
    if msg.startswith("[GRASP_Z_FIX]"):
        return True
    if msg.startswith("[PHYSICS]"):
        return True
    if msg.startswith("[CALIB]"):
        return True
    return False

def _set_motion_lock(panel, active: bool):
    """Habilitar/deshabilitar control manual mientras corre un flujo predefinido."""
    panel._script_motion_active = active
    QTimer.singleShot(0, panel._refresh_controls)

def _set_btn_state(panel, btn: QPushButton, enabled: bool, tooltip: str = "") -> None:
    btn.setEnabled(enabled)
    if enabled:
        if tooltip:
            btn.setToolTip(tooltip)
        return
    if tooltip:
        btn.setToolTip(tooltip)

def _set_launching_style(panel, btn: QPushButton, active: bool) -> None:
    if active:
        btn.setStyleSheet("background:#f59e0b; color:#0f172a; font-weight:600;")
    else:
        btn.setStyleSheet("")

def _clear_launching_if_timeout(panel, label: str, start_ts: float, timeout_sec: float) -> bool:
    if not start_ts:
        return False
    if (time.time() - start_ts) < timeout_sec:
        return False
    panel._emit_log(f"[WARN] {label} timeout; re-habilitando botón")
    return True

def _controller_drop_grace_active(panel) -> bool:
    if panel._controller_spawn_inflight:
        return True
    if not panel._controller_spawn_last_start:
        return False
    return (time.time() - panel._controller_spawn_last_start) < CONTROLLER_DROP_GRACE_SEC

def _require_ready_basic(panel, action: str) -> bool:
    return panel._safety.require_ready_basic(action)

def _basic_ready_status(panel) -> Tuple[bool, str]:
    if panel._state_ready_basic():
        return True, ""
    reason = panel._system_state_reason or panel._system_state.value
    return False, reason

def _pick_demo_remote_ready_status(panel) -> Tuple[bool, str, bool]:
    if getattr(panel, "_script_motion_active", False) or getattr(panel, "_manual_inflight", False):
        return False, "ejecución en curso", False
    if not bool(getattr(panel, "_objects_release_done", False)):
        return False, "release de objetos pendiente", True
    if not bool(getattr(panel, "_objects_settled", False)):
        return False, "objetos no estabilizados", True
    basic_ok, basic_reason = panel._basic_ready_status()
    if not basic_ok:
        reason_txt = str(basic_reason or "basic_not_ready")
        basic_waitable = (
            panel._system_state != SystemState.ERROR_FATAL
            and not reason_txt.lower().startswith("startup failed:")
        )
        return False, reason_txt, basic_waitable
    ok, reason = pick_ui_status(panel)
    if not ok:
        reason_txt = str(reason or "pick_ui_no_listo")
        if reason_txt != "camera_warmup" or not bool(
            getattr(panel, "btn_pick_demo", None) and panel.btn_pick_demo.isEnabled()
        ):
            return False, reason_txt, True
        panel._emit_log_throttled(
            "PICK_REMOTE:camera_warmup_bypass",
            "[PICK][REMOTE][DIRECT] camera_warmup tolerado: btn_pick_demo ya habilitado",
            min_interval=2.0,
        )
    controllers_ok, controllers_reason = panel._controllers_ready()
    if not controllers_ok:
        return False, str(controllers_reason or "controladores no listos"), True
    if not bool(panel._ee_frame_effective):
        return False, panel._tf_not_ready_reason(), True
    if not bool(getattr(panel, "_objects_settled", False)):
        return False, "objetos no estabilizados", True
    selected_name = str(getattr(panel, "_selected_object", "") or "").strip()
    user_selected = str(getattr(panel, "_selection_last_user_name", "") or "").strip()
    if selected_name != PICK_DEMO_OBJECT_NAME or user_selected != PICK_DEMO_OBJECT_NAME:
        reason = (
            "selección inválida "
            f"(selected={selected_name or 'none'} user_selected={user_selected or 'none'} "
            f"required={PICK_DEMO_OBJECT_NAME})"
        )
        return False, reason, False
    return True, "", False

def _auto_release_drop_objects_when_ready(panel) -> None:
    if not panel._auto_release_drop_objects or panel._closing or not panel._gz_running:
        return
    if panel._objects_release_done or panel._detach_inflight:
        return
    if panel._pose_info_ok and panel._sync_external_release_state():
        panel._emit_log("[PHYSICS] Auto-release DROP omitido: escena ya reconciliada en mesa.")
        return
    now = time.monotonic()
    bridge_age = (now - panel._bridge_start_ts) if panel._bridge_start_ts > 0.0 else 0.0
    pose_ready_live = False
    try:
        pose_ready_live = bool(panel._pose_info_ready())
    except Exception:
        pose_ready_live = False
    if pose_ready_live and not panel._pose_info_ok:
        try:
            panel._update_pose_info_status()
        except Exception:
            pass
        if panel._pose_info_ok and panel._sync_external_release_state():
            panel._emit_log("[PHYSICS] Auto-release DROP omitido: escena reconciliada tras pose/info.")
            return
    if not panel._pose_info_ok and bridge_age < max(2.0, float(TF_INIT_GRACE_SEC)):
        QTimer.singleShot(400, panel._auto_release_drop_objects_when_ready)
        return
    panel._emit_log("[PHYSICS] Auto-release DROP: ejecutando release_objects tras espera inicial.")
    panel._release_objects()

def _require_ready_vision(panel, action: str) -> bool:
    return panel._safety.require_ready_vision(action)

def _require_manual_ready(panel, action: str) -> bool:
    return panel._safety.require_manual_ready(action)

def _log(panel, msg: str):
    if panel._debug_logs_enabled:
        panel._emit_log(msg)

def _emit_log_throttled(panel, key: str, msg: str, min_interval: float = 1.0) -> None:
    panel._safety.emit_log_throttled(key, msg, min_interval=min_interval)

def _block_if_managed(panel, action: str) -> bool:
    if not panel._managed_mode:
        return False
    panel._emit_log(f"[WARN] {action} bloqueado: PANEL_MANAGED=1")
    return True

def _log_error(panel, msg: str):
    """SIEMPRE loguear errores, incluso si debug no está activo."""
    panel._emit_log(f"[ERROR] {msg}")

def _log_warning(panel, msg: str):
    """Loguear warnings cuando debug está activo."""
    if panel._debug_logs_enabled:
        panel._emit_log(f"[WARN] {msg}")

def _on_async_error(panel, msg: str) -> None:
    panel._log_warning(f"[ASYNC] {msg}")

def _run_ui_callable(panel, fn) -> None:
    try:
        fn()
    except Exception as exc:
        panel._log_warning(f"[UI] {exc}")

def _run_ui_delayed(panel, fn, delay_ms: int) -> None:
    QTimer.singleShot(int(delay_ms), fn)

def _set_debug_motion_button_waiting(panel, waiting: bool, reason: str = "") -> None:
    btn = getattr(panel, "btn_debug_motion", None)
    if btn is None:
        return
    btn.setEnabled(True)
    if waiting:
        btn.setText("DEBUG MOVIMIENTO: CONTINUAR")
        btn.setStyleSheet("background:#f59e0b; color:#0f172a; font-weight:700;")
        btn.setToolTip(
            f"Pausa activa ({reason or 'debug_manual'}). Pulsa para continuar la secuencia."
        )
    else:
        btn.setText("DEBUG MOVIMIENTO")
        btn.setStyleSheet("")
        btn.setToolTip(
            "Pausa manual de depuración: cuando haya una pausa activa, pulsa para continuar."
        )

def _on_step_mode_combo_changed(panel, text: str) -> None:
    if panel._step_mode_combo_syncing:
        return
    panel._set_step_mode(text, emit_log=True)

def _set_step_mode(panel, mode: str, *, emit_log: bool = True) -> None:
    requested = str(mode or "").strip().upper()
    normalized = "STEP_BY_STEP" if requested == "STEP_BY_STEP" else "AUTO"
    previous = panel._step_mode
    panel._step_mode = normalized

    combo = getattr(panel, "step_mode_combo", None)
    if combo is not None and combo.currentText() != normalized:
        panel._step_mode_combo_syncing = True
        try:
            combo.setCurrentText(normalized)
        finally:
            panel._step_mode_combo_syncing = False

    if normalized == "STEP_BY_STEP":
        panel.signal_run_ui.emit(panel._step_window_set_waiting)
        if emit_log and previous != normalized:
            panel._emit_log("[STEP] mode=STEP_BY_STEP")
        return

    # AUTO always releases any gate currently waiting.
    panel._step_wait_event.set()
    panel._direct_clear_waiting_for_approach_confirmation(reason="step_mode_auto")
    panel.signal_run_ui.emit(panel._step_window_hide)
    if emit_log and previous != normalized:
        panel._emit_log("[STEP] mode=AUTO")

def _ensure_step_window(panel) -> None:
    if panel._step_window is not None:
        return
    build_step_window(panel)

def _step_cart_debug_trace_path(panel) -> Path:
    out_dir = Path(panel.ws_dir) / "historico" / "step_cartesian_debug"
    ensure_dir(str(out_dir))
    return out_dir / "manual_moves.jsonl"

def _step_cart_debug_log_event(panel, event: str, **payload) -> None:
    data = {
        "ts": datetime.now().isoformat(timespec="milliseconds"),
        "event": str(event or "unknown"),
        "step_mode": str(panel._step_mode or ""),
        "frame": "base_link",
        **payload,
    }
    line = json.dumps(data, ensure_ascii=True, sort_keys=True)
    panel._emit_log(f"[STEP][CART_DEBUG] {line}")
    try:
        trace_path = panel._step_cart_debug_trace_path()
        with trace_path.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except Exception as exc:
        panel._emit_log(f"[STEP][CART_DEBUG] trace_write_error={exc}")

def _step_cart_debug_sample(panel) -> Dict[str, object]:
    base_frame = panel._business_base_frame()
    pinch = panel._step_fetch_live_pose("rg2_pinch_center")
    tool0 = panel._step_fetch_live_pose("tool0")
    now_wall = time.time()
    panel._step_cart_debug_last_sample_wall = now_wall
    return {
        "base_frame": base_frame,
        "pinch": pinch,
        "tool0": tool0,
        "wall": now_wall,
    }

def _step_cart_debug_set_status(panel, text: str, error: bool = False) -> None:
    if panel._step_cart_debug_status_label is None:
        return
    panel._step_cart_debug_status_label.setText(str(text or "--"))
    color = "#ef4444" if error else "#0f766e"
    panel._step_cart_debug_status_label.setStyleSheet(f"color:{color}; font-weight:600;")

def _step_cart_debug_refresh(panel) -> None:
    if panel._step_cart_debug_window is None:
        return
    if not panel._step_cart_debug_window.isVisible():
        return
    sample = panel._step_cart_debug_sample()
    pinch = sample.get("pinch")
    tool0 = sample.get("tool0")
    base_frame = str(sample.get("base_frame") or "base_link")
    wall = float(sample.get("wall") or time.time())
    age_s = max(0.0, time.time() - wall)
    if panel._step_cart_debug_pose_label is not None:
        panel._step_cart_debug_pose_label.setText(
            f"rg2_pinch_center@{base_frame}: {panel._step_format_inline_xyz(pinch)}"
        )
    if panel._step_cart_debug_tool0_label is not None:
        panel._step_cart_debug_tool0_label.setText(
            f"tool0@{base_frame}: {panel._step_format_inline_xyz(tool0)}"
        )
    if panel._step_cart_debug_frame_label is not None:
        panel._step_cart_debug_frame_label.setText("Frame operativo: rg2_pinch_center -> base_link")
    if panel._step_cart_debug_time_label is not None:
        panel._step_cart_debug_time_label.setText(
            f"timestamp: {datetime.fromtimestamp(wall).strftime('%H:%M:%S.%f')[:-3]} | age: {age_s:.3f}s"
        )

def _step_cart_debug_step_m(panel) -> float:
    combo = panel._step_cart_debug_step_combo
    if combo is None:
        return 0.002
    data = combo.currentData()
    if isinstance(data, (int, float)):
        return max(0.001, float(data))
    txt = str(combo.currentText() or "2 mm").strip().lower().replace("mm", "").strip()
    try:
        return max(0.001, float(txt) / 1000.0)
    except Exception:
        return 0.002

def _direct_clear_waiting_for_approach_confirmation(panel, *args, **kwargs):
    _sc._direct_clear_waiting_for_approach_confirmation(panel, *args, **kwargs)

def _direct_enter_waiting_for_approach_confirmation(panel, *args, **kwargs):
    _sc._direct_enter_waiting_for_approach_confirmation(panel, *args, **kwargs)

def _direct_release_waiting_for_approach_confirmation(panel, *args, **kwargs):
    return _sc._direct_release_waiting_for_approach_confirmation(panel, *args, **kwargs)

def _direct_waiting_for_approach_confirmation(panel, *args, **kwargs):
    return _sc._direct_waiting_for_approach_confirmation(panel, *args, **kwargs)

def _ensure_step_cart_debug_window(panel, *args, **kwargs):
    _sc._ensure_step_cart_debug_window(panel, *args, **kwargs)

def _on_step_continue_clicked(panel, *args, **kwargs):
    _sc._on_step_continue_clicked(panel, *args, **kwargs)

def _on_step_phase_start_clicked(panel, *args, **kwargs):
    _sc._on_step_phase_start_clicked(panel, *args, **kwargs)

def _on_step_window_finished(panel, *args, **kwargs):
    _sc._on_step_window_finished(panel, *args, **kwargs)

def _read_gripper_feedback_state(panel, *args, **kwargs):
    return _sc._read_gripper_feedback_state(panel, *args, **kwargs)

def _show_step_cart_debug_window(panel, *args, **kwargs):
    _sc._show_step_cart_debug_window(panel, *args, **kwargs)

def _step_assess_target_reached(panel, *args, **kwargs):
    return _sc._step_assess_target_reached(panel, *args, **kwargs)

def _step_capture_start_pose(panel, *args, **kwargs):
    _sc._step_capture_start_pose(panel, *args, **kwargs)

def _step_cart_debug_handle_axis(panel, *args, **kwargs):
    _sc._step_cart_debug_handle_axis(panel, *args, **kwargs)

def _step_cart_debug_move_delta(panel, *args, **kwargs):
    _sc._step_cart_debug_move_delta(panel, *args, **kwargs)

def _step_cart_debug_run_validation_xyz(panel, *args, **kwargs):
    _sc._step_cart_debug_run_validation_xyz(panel, *args, **kwargs)

def _step_cartesian_move_runtime_target(panel, *args, **kwargs):
    return _sc._step_cartesian_move_runtime_target(panel, *args, **kwargs)

def _step_display_position(panel, *args, **kwargs):
    return _sc._step_display_position(panel, *args, **kwargs)

def _step_effective_flow(panel, *args, **kwargs):
    return _sc._step_effective_flow(panel, *args, **kwargs)

def _step_fetch_live_pose(panel, *args, **kwargs):
    return _sc._step_fetch_live_pose(panel, *args, **kwargs)

def _step_fetch_object_world(panel, *args, **kwargs):
    return _sc._step_fetch_object_world(panel, *args, **kwargs)

def _step_find_history_row(panel, *args, **kwargs):
    return _sc._step_find_history_row(panel, *args, **kwargs)

def _step_format_inline_rpy(panel, *args, **kwargs):
    return _sc._step_format_inline_rpy(panel, *args, **kwargs)

def _step_format_inline_xyz(panel, *args, **kwargs):
    return _sc._step_format_inline_xyz(panel, *args, **kwargs)

def _step_format_xyz(panel, *args, **kwargs):
    return _sc._step_format_xyz(panel, *args, **kwargs)

def _step_live_gripper_state(panel, *args, **kwargs):
    return _sc._step_live_gripper_state(panel, *args, **kwargs)

def _step_live_pose_text(panel, *args, **kwargs):
    return _sc._step_live_pose_text(panel, *args, **kwargs)

def _step_operational_frame_name(panel, *args, **kwargs):
    return _sc._step_operational_frame_name(panel, *args, **kwargs)

def _step_phase_action_text(panel, *args, **kwargs):
    return _sc._step_phase_action_text(panel, *args, **kwargs)

def _step_phase_completed(panel, *args, **kwargs):
    return _sc._step_phase_completed(panel, *args, **kwargs)

def _step_phase_gate_already_owned(panel, *args, **kwargs):
    return _sc._step_phase_gate_already_owned(panel, *args, **kwargs)

def _step_phase_gripper_state(panel, *args, **kwargs):
    return _sc._step_phase_gripper_state(panel, *args, **kwargs)

def _step_phase_intent(panel, *args, **kwargs):
    return _sc._step_phase_intent(panel, *args, **kwargs)

def _step_phase_sequence(panel, *args, **kwargs):
    return _sc._step_phase_sequence(panel, *args, **kwargs)

def _step_pipeline_phase_state(panel, *args, **kwargs):
    return _sc._step_pipeline_phase_state(panel, *args, **kwargs)

def _step_pipeline_rebuild(panel, *args, **kwargs):
    _sc._step_pipeline_rebuild(panel, *args, **kwargs)

def _step_pre_insert_inicio_row(panel, *args, **kwargs):
    _sc._step_pre_insert_inicio_row(panel, *args, **kwargs)

def _step_predict_next_phase(panel, *args, **kwargs):
    return _sc._step_predict_next_phase(panel, *args, **kwargs)

def _step_prepare_pipeline_view(panel, *args, **kwargs):
    _sc._step_prepare_pipeline_view(panel, *args, **kwargs)

def _step_present_flow_name(panel, *args, **kwargs):
    return _sc._step_present_flow_name(panel, *args, **kwargs)

def _step_record_current_phase_actual(panel, *args, **kwargs):
    _sc._step_record_current_phase_actual(panel, *args, **kwargs)

def _step_record_direct_event_snapshot(panel, *args, **kwargs):
    _sc._step_record_direct_event_snapshot(panel, *args, **kwargs)

def _step_record_direct_home_initial(panel, *args, **kwargs):
    _sc._step_record_direct_home_initial(panel, *args, **kwargs)

def _step_record_direct_initial_snapshot(panel, *args, **kwargs):
    _sc._step_record_direct_initial_snapshot(panel, *args, **kwargs)

def _step_record_direct_mesa_ready(panel, *args, **kwargs):
    _sc._step_record_direct_mesa_ready(panel, *args, **kwargs)

def _step_record_history(panel, *args, **kwargs):
    _sc._step_record_history(panel, *args, **kwargs)

def _step_refresh_pipeline_table(panel, *args, **kwargs):
    _sc._step_refresh_pipeline_table(panel, *args, **kwargs)

def _step_reset_sequence_view(panel, *args, **kwargs):
    _sc._step_reset_sequence_view(panel, *args, **kwargs)

def _step_runtime_refresh(panel, *args, **kwargs):
    _sc._step_runtime_refresh(panel, *args, **kwargs)

def _step_selected_object_name(panel, *args, **kwargs):
    return _sc._step_selected_object_name(panel, *args, **kwargs)

def _step_set_exec_target(panel, *args, **kwargs):
    _sc._step_set_exec_target(panel, *args, **kwargs)

def _step_status_item(panel, *args, **kwargs):
    return _sc._step_status_item(panel, *args, **kwargs)

def _step_update_phase_result(panel, *args, **kwargs):
    _sc._step_update_phase_result(panel, *args, **kwargs)

def _step_update_row_object_metrics(panel, *args, **kwargs):
    _sc._step_update_row_object_metrics(panel, *args, **kwargs)

def _step_upsert_history_row_ordered(panel, *args, **kwargs):
    _sc._step_upsert_history_row_ordered(panel, *args, **kwargs)

def _step_wait_for_phase(panel, *args, **kwargs):
    _sc._step_wait_for_phase(panel, *args, **kwargs)

def _step_window_hide(panel, *args, **kwargs):
    _sc._step_window_hide(panel, *args, **kwargs)

def _step_window_maybe_refresh(panel, *args, **kwargs):
    _sc._step_window_maybe_refresh(panel, *args, **kwargs)

def _step_window_refresh(panel, *args, **kwargs):
    _sc._step_window_refresh(panel, *args, **kwargs)

def _step_window_set_waiting(panel, *args, **kwargs):
    _sc._step_window_set_waiting(panel, *args, **kwargs)


def _on_debug_motion_button(panel) -> None:
    with panel._debug_motion_lock:
        waiting = bool(panel._debug_motion_wait_active)
        reason = str(panel._debug_motion_wait_reason or "")
        panel._debug_motion_continue_event.set()
    if waiting:
        panel._emit_log(
            "[DEBUG][MOTION] continue_button=pressed "
            f"reason={reason or 'debug_manual_pause'}"
        )
        panel._set_status("DEBUG MOVIMIENTO: reanudando secuencia", error=False)
    else:
        panel._emit_log("[DEBUG][MOTION] continue_button=pressed waiting=false")

def _debug_motion_wait_for_continue(panel, *, reason: str, timeout_sec: Optional[float] = None) -> bool:
    btn = getattr(panel, "btn_debug_motion", None)
    if btn is None or not btn.isVisible():
        return True
    if not bool(getattr(panel, "_debug_motion_pause_alcance_enabled", False)):
        return True
    wait_timeout = (
        float(panel._debug_motion_pause_timeout_sec)
        if timeout_sec is None
        else max(0.0, float(timeout_sec))
    )
    with panel._debug_motion_lock:
        panel._debug_motion_continue_event.clear()
        panel._debug_motion_wait_active = True
        panel._debug_motion_wait_reason = str(reason or "debug_manual_pause")
    panel.signal_run_ui.emit(
        lambda: panel._set_debug_motion_button_waiting(True, str(reason or "debug_manual_pause"))
    )
    panel._emit_log(
        "[DEBUG][MOTION] pause_enter "
        f"reason={reason or 'debug_manual_pause'} "
        f"timeout={'inf' if wait_timeout <= 0.0 else f'{wait_timeout:.1f}s'}"
    )
    if wait_timeout <= 0.0:
        resumed = bool(panel._debug_motion_continue_event.wait())
    else:
        resumed = bool(panel._debug_motion_continue_event.wait(wait_timeout))
    with panel._debug_motion_lock:
        panel._debug_motion_wait_active = False
        panel._debug_motion_wait_reason = ""
        panel._debug_motion_continue_event.clear()
    panel.signal_run_ui.emit(lambda: panel._set_debug_motion_button_waiting(False, ""))
    if resumed:
        panel._emit_log(
            "[DEBUG][MOTION] pause_exit "
            f"reason={reason or 'debug_manual_pause'} resume=button"
        )
        return True
    panel._emit_log(
        "[DEBUG][MOTION] pause_timeout "
        f"reason={reason or 'debug_manual_pause'} timeout={wait_timeout:.1f}s"
    )
    return False

def _run_async(panel, fn, *, name: str = "", on_done=None) -> QThread:
    thread = _FnThread(fn, name=name)
    thread.error.connect(panel._on_async_error)
    if on_done:
        thread.finished.connect(on_done)

    def _cleanup() -> None:
        try:
            panel._async_threads.remove(thread)
        except ValueError:
            pass

    thread.finished.connect(_cleanup)
    thread.finished.connect(thread.deleteLater)
    panel._async_threads.append(thread)
    thread.start()
    return thread

def _log_ros_message(panel, msg: str):
    """Mostrar siempre los mensajes provenientes del RosWorker, pero solo cuando el bridge esté activo."""
    # Los errores de inicialización ROS son siempre visibles (no esperar al bridge).
    if "[ROS] ERROR rclpy" in msg:
        panel._emit_log(msg)
        return
    if not panel._bridge_running:
        return
    panel._emit_log(msg)

def _on_system_state_update(panel, state: str, reason: str) -> None:
    state = (state or "").strip().upper()
    if not state:
        return
    prev = panel._external_state
    panel._external_state = state
    panel._external_state_reason = reason or ""
    panel._external_state_last = time.time()
    # Trigger a state resolution pass so _system_state reflects the
    # external state immediately (without waiting for the next
    # _refresh_controls call, which may only fire on demand).
    if prev != state or panel._system_state in (SystemState.BOOT, SystemState.WAITING_GAZEBO):
        try:
            panel.signal_refresh_controls.emit()
        except RuntimeError:
            pass

def _external_state_active(panel) -> bool:
    return external_state_active(panel)

def _resolve_external_state(panel) -> Tuple[Optional[SystemState], str]:
    return resolve_external_state(panel)

def _apply_external_system_state(panel) -> None:
    apply_external_system_state(panel)

def _log_camera_diagnostics(panel, reason: str):
    """Emitir detalles adicionales para debugging cuando hay fallos de cámara."""
    if not panel._camera_required:
        return
    if not panel._debug_logs_enabled:
        return
    node_ready = panel.ros_worker.node_ready()
    ctrl_ok = panel._ros2_control_available()
    clock_ok, clock_age = panel._clock_status()
    bridge_ok = panel._bridge_running
    last_age = "n/a"
    if panel._last_camera_frame_ts:
        last_age = f"{_runtime_time() - panel._last_camera_frame_ts:.1f}s"
    topics = []
    if node_ready:
        try:
            topics = panel.ros_worker.list_topic_names()
        except Exception as exc:
            panel._log(f"[CAMERA-DIAG] fallo listando topics: {exc}")
    camera_topics = [t for t in topics if t.startswith(CAMERA_TOPIC_PREFIX)]
    diag = (
        f"[CAMERA-DIAG] {reason} node_ready={node_ready} ros2_ctrl={ctrl_ok} "
        f"clock={clock_ok}:{clock_age} bridge={bridge_ok} "
        f"last_frame_age={last_age} camera_topics={len(camera_topics)}/{len(topics)}"
    )
    panel._log(diag)
    if camera_topics:
        preview = ", ".join(camera_topics[:4])
        suffix = "..." if len(camera_topics) > 4 else ""
        panel._log(f"[CAMERA-DIAG] camera topics sample: {preview}{suffix}")

def _sync_moveit_from_system_state(panel) -> None:
    if not panel._moveit_required:
        moveit_detected = bool(
            panel._moveit_running or panel._moveit_bridge_detected() or panel._moveit_status_ready()
        )
        if moveit_detected:
            panel._moveit_required = True
            if panel._moveit_state != MoveItState.READY:
                panel._moveit_state = MoveItState.READY
                panel._moveit_state_reason = "move_group detectado"
            panel._emit_log("[MOVEIT] move_group detectado; saliendo de modo manual automáticamente")
            return
        panel._moveit_state = MoveItState.OFF
        panel._moveit_state_reason = "manual"
        return
    if not panel._external_state_active():
        return
    if panel._system_state == SystemState.READY_MOVEIT:
        if panel._moveit_state != MoveItState.READY:
            panel._moveit_state = MoveItState.READY
            panel._moveit_state_reason = "move_group listo (externo)"
        return
    if panel._system_state == SystemState.READY_VISION:
        panel._moveit_state = MoveItState.WAITING_MOVEIT_READY
        panel._moveit_state_reason = panel._system_state_reason or "move_group no listo"

def _clock_status(panel) -> Tuple[bool, str]:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False, "node_off"
    ok, age = panel.ros_worker.clock_alive()
    if ok:
        return True, f"age={age:.2f}s"
    return False, f"age={age:.2f}s"

def _joint_states_status(panel) -> Tuple[bool, str]:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False, "node_off"
    topic = (panel._joint_current_topic or panel.joint_topic or "/joint_states").strip() or "/joint_states"
    has_pub = panel.ros_worker.topic_has_publishers(topic)
    if not has_pub:
        return False, f"{topic}:no_publishers"
    payload, ts = panel.ros_worker.get_last_joint_state()
    if payload is None:
        return False, f"{topic}:no_msgs"
    names = payload.get("name", []) or []
    if len(names) == 0:
        return False, f"{topic}:empty"
    strict_identity = str(os.environ.get("PANEL_STRICT_JOINT_IDENTITY", "1")).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    if strict_identity:
        normalized = {_normalize_joint_name(str(n)) for n in names if str(n).strip()}
        missing = [jn for jn in UR5_JOINT_NAMES if jn not in normalized]
        if missing:
            sample = ", ".join(sorted(list(normalized))[:8])
            return False, f"{topic}:joint_identity_mismatch missing={','.join(missing)} sample={sample}"
    age = float("inf")
    if ts:
        age = max(0.0, _runtime_time() - ts)
    if age > 2.0:
        return False, f"{topic}:stale age={age:.2f}s"
    return True, f"topic={topic} age={age:.2f}s names={len(names)}"

def _bridge_transport_detected(panel) -> bool:
    # A pose-only bridge (gz_pose_bridge) is not enough for DIRECTO; require
    # joint_states as well before treating the full ROS bridge as active.
    if panel._proc_alive(panel.bridge_proc):
        return True
    if not panel._pose_info_active():
        return False
    js_ok, _js_reason = panel._joint_states_status()
    return bool(js_ok)

def _bridge_ready_status(panel) -> Tuple[bool, str]:
    if not panel._bridge_running:
        return False, "bridge_process_off"
    clock_ok, clock_reason = panel._clock_status()
    if not clock_ok:
        return False, f"clock_not_ready ({clock_reason})"
    if not panel._pose_info_active():
        return False, "pose_info_no_publishers"
    js_ok, js_reason = panel._joint_states_status()
    if not js_ok:
        return False, js_reason
    helper = get_tf_helper()
    if helper is None:
        return False, "tf_helper_off"
    tf_stats = helper.tf_listener_stats()
    if tf_stats[0] <= 0:
        return False, f"tf_msgs={tf_stats[0]}"
    return True, f"clock+pose+joint_states+tf ok (tf_msgs={tf_stats[0]})"

def _tf_chain_ready_status(panel) -> Tuple[bool, str]:
    helper = get_tf_helper()
    if helper is None:
        return False, "tf_helper_off"
    base_frame = panel._business_base_frame()
    ee_frame = str(getattr(panel, "_required_ee_frame", "") or "rg2_pinch_center").strip() or "rg2_pinch_center"
    if not _can_transform_between(helper, base_frame, ee_frame, timeout_sec=0.2):
        return False, f"{base_frame}<->{ee_frame} missing"
    return True, f"{base_frame}->{ee_frame}"

def _camera_depth_expectation(panel) -> Tuple[bool, str]:
    topic = str(panel.camera_topic or "").strip()
    if topic.endswith("/depth_image"):
        return False, topic
    if topic.endswith("/image"):
        depth_topic = topic[: -len("/image")] + "/depth_image"
    elif topic.endswith("/rgb"):
        depth_topic = topic[: -len("/rgb")] + "/depth_image"
    else:
        depth_topic = "/camera_overhead/depth_image"
    # The runtime camera gate must follow the active TFM model modality.
    # Recent RGB checkpoints were being blocked by stale depth even though
    # preprocessing only consumes depth when in_channels == 4.
    depth_required = bool(panel._camera_depth_required_env)
    if not depth_required:
        in_channels = 0
        try:
            model_info = panel.tfm_module.model_info() if panel.tfm_module else {}
            in_channels = int((model_info or {}).get("in_channels", 0) or 0)
        except Exception:
            in_channels = 0
        if in_channels <= 0:
            try:
                modality = str(getattr(panel, "_exp_info", {}).get("modality", "") or "").strip().lower()
            except Exception:
                modality = ""
            if modality in ("rgbd", "rgb-d"):
                in_channels = 4
            elif modality == "rgb":
                in_channels = 3
        if in_channels > 0:
            depth_required = in_channels >= 4
        elif hasattr(panel, "chk_tfm_use_depth") and panel.chk_tfm_use_depth is not None:
            try:
                depth_required = bool(panel.chk_tfm_use_depth.isChecked())
            except Exception:
                pass
    return depth_required, depth_topic

def _camera_runtime_flags(panel, now: Optional[float] = None) -> Tuple[bool, bool, bool, float, bool]:
    """Return camera_ready, camera_fault, camera_source_down, frame_age, warmup_grace."""
    if now is None:
        now = _runtime_time()
    age = now - panel._last_camera_frame_ts if panel._last_camera_frame_ts else float("inf")
    gz_state = panel._gazebo_state()
    gazebo_ready = gz_state == "GAZEBO_READY"
    bridge_ready = bool(panel._bridge_running)
    source_down = (not gazebo_ready) or (not bridge_ready)
    has_frames = panel._camera_frame_count > 0
    depth_required, _depth_topic = panel._camera_depth_expectation()
    depth_age = (
        now - panel._last_camera_depth_frame_ts
        if panel._last_camera_depth_frame_ts
        else float("inf")
    )
    depth_ready = (
        (not depth_required)
        or (
            panel._camera_depth_frame_count > 0
            and depth_age < CAMERA_READY_MAX_AGE_SEC
            and gazebo_ready
            and bridge_ready
        )
    )
    grace_anchor = 0.0
    if panel._camera_subscribe_ts > 0.0:
        grace_anchor = panel._camera_subscribe_ts
    elif panel._camera_init_start > 0.0:
        grace_anchor = panel._camera_init_start
    elif panel._bridge_start_ts > 0.0:
        grace_anchor = panel._bridge_start_ts
    in_grace = grace_anchor > 0.0 and (now - grace_anchor) < panel._camera_warmup_grace_sec
    camera_ready = has_frames and age < CAMERA_READY_MAX_AGE_SEC and depth_ready and gazebo_ready and bridge_ready
    camera_fault = has_frames and age > panel._camera_fault_age_sec and gazebo_ready and bridge_ready and (not in_grace)
    if depth_required:
        camera_fault = camera_fault or (
            panel._camera_depth_frame_count > 0
            and depth_age > panel._camera_fault_age_sec
            and gazebo_ready
            and bridge_ready
            and (not in_grace)
        )
    return camera_ready, camera_fault, source_down, age, in_grace

def _sync_external_release_state(panel) -> bool:
    if panel._objects_release_done and panel._objects_settled:
        return True
    if not panel._pose_info_ok:
        return False
    positions = get_object_positions() or {}
    live_positions = {}
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            pose_map, _pose_ts = panel.ros_worker.pose_snapshot()
        except Exception:
            pose_map = {}
        for name in DROP_OBJECT_NAMES:
            pos = pose_map.get(name)
            if pos is None or len(pos) < 3:
                continue
            try:
                live_positions[name] = (float(pos[0]), float(pos[1]), float(pos[2]))
            except Exception:
                continue
        if live_positions:
            bulk_update_object_positions(
                live_positions,
                source="external_release_reconciled_live",
                objects_stable=True,
            )
            recalc_object_states(reason="external_release_reconciled_live")
            positions = get_object_positions() or {}
    if not DROP_OBJECT_NAMES:
        return False
    for name in DROP_OBJECT_NAMES:
        pos = positions.get(name)
        if pos is None or len(pos) < 3:
            return False
        try:
            xyz = (float(pos[0]), float(pos[1]), float(pos[2]))
        except Exception:
            return False
        if not is_on_table(xyz):
            return False
    if not panel._objects_release_done or not panel._objects_settled:
        panel._objects_release_done = True
        panel._objects_settled = True
        panel._emit_log(
            "[PHYSICS][DROP] external release reconciled scene=on_table_all "
            f"count={len(DROP_OBJECT_NAMES)}"
        )
        recalc_object_states(reason="external_release_reconciled")
        panel.signal_refresh_controls.emit()
    return True

def _pose_info_topic(panel) -> str:
    world_name = panel._gz_world_name or panel._detect_world_name() or GZ_WORLD
    return f"/world/{world_name}/pose/info"

def _pose_info_active(panel) -> bool:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False
    now = time.monotonic()
    if (
        STATUS_TOPIC_CACHE_SEC > 0.0
        and (now - panel._pose_info_active_ts) < STATUS_TOPIC_CACHE_SEC
    ):
        return panel._pose_info_active_cache
    active = panel.ros_worker.topic_has_publishers(panel._pose_info_topic())
    panel._pose_info_active_cache = bool(active)
    panel._pose_info_active_ts = now
    return active

def _pose_info_ready(panel) -> bool:
    if not panel._ros_worker_started or not panel.ros_worker.node_ready():
        return False
    if not panel._pose_info_active():
        return False
    poses, ts = panel.ros_worker.pose_snapshot()
    if not poses:
        return False
    if ts:
        age = _runtime_time() - ts
        if age > POSE_INFO_MAX_AGE_SEC:
            return False
    return True

def _gazebo_process_signal(panel) -> Tuple[bool, str]:
    """Signal S1: determine if Gazebo process is alive using PID/PGID + fallback scan."""
    if panel._proc_alive(panel.gz_proc):
        pid = int(getattr(panel.gz_proc, "pid", 0) or 0)
        if pid > 0:
            panel._gz_real_pid = pid
            if panel._gz_root_pid <= 0:
                panel._gz_root_pid = pid
            if panel._gz_pgid <= 0:
                try:
                    panel._gz_pgid = int(os.getpgid(pid))
                except Exception:
                    panel._gz_pgid = 0
        return True, "popen"
    if panel._gz_pgid > 0 and psutil is not None:
        try:
            for proc in psutil.process_iter(attrs=["pid", "cmdline", "status"]):
                info = proc.info
                pid = int(info.get("pid") or 0)
                if pid <= 0:
                    continue
                if info.get("status") == psutil.STATUS_ZOMBIE:
                    continue
                try:
                    if int(os.getpgid(pid)) != panel._gz_pgid:
                        continue
                except Exception:
                    continue
                cmdline = info.get("cmdline") or []
                if not cmdline:
                    continue
                joined = " ".join(cmdline).lower()
                if any(token in joined for token in ("gz sim", "gz-sim", "gzserver", "ign gazebo")):
                    panel._gz_real_pid = pid
                    return True, "pgid"
        except Exception as exc:
            _log_exception("gazebo process signal pgid", exc)
    proc_ok, proc_reason = gz_sim_status()
    if proc_ok:
        return True, f"fallback_{proc_reason}"
    return False, proc_reason
