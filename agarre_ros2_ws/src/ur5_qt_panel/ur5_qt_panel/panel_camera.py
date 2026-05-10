#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_camera.py
# Contenido: Widgets y controlador de cámara extraídos de panel_v2.py.
# Uso breve: Se importa desde panel_v2.py; no es un entry point independiente.
"""Camera view widget and controller extracted from panel_v2.py."""
from __future__ import annotations

import math
import time

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QLabel, QSizePolicy

from .panel_config import (
    CAMERA_FAST_SCALE,
    CAMERA_INFO_INTERVAL_SEC,
    CAMERA_INIT_GRACE_SEC,
    CAMERA_PREPROCESS_TFM,
    CAMERA_READY_MAX_AGE_SEC,
    CAMERA_SKIP_TFM_INPUT,
    CAMERA_TRACK_FPS,
    CAMERA_UI_SKIP_HIDDEN,
    OVERLAY_CALIB,
)
from .panel_camera_helpers import is_camera_topic
from .panel_tfm import _store_preprocessed_cache, build_tfm_preprocessed_input


def _runtime_time() -> float:
    """Steady local timestamp for runtime freshness and watchdog logic."""
    return time.monotonic()


class CameraView(QLabel):
    """Large camera view that auto-scales to the widget size."""

    clicked = pyqtSignal(int, int)  # Emite (x, y) en píxeles de la imagen original

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._qimg = None
        self._img_width = 0
        self._img_height = 0
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(480, 360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setText(title)
        self.setStyleSheet("background:#0b0f14; color:#94a3b8; border:1px solid #1f2937;")

    def set_frame(self, qimg, width=0, height=0):
        if qimg is None or qimg.isNull():
            return
        src_w = int(width or qimg.width() or 0)
        src_h = int(height or qimg.height() or 0)
        # Ignore clearly invalid placeholder frames and keep the last good image.
        if src_w <= 2 or src_h <= 2 or qimg.width() <= 2 or qimg.height() <= 2:
            return
        self._qimg = qimg
        self._img_width = src_w
        self._img_height = src_h
        self._update_pixmap()

    def mousePressEvent(self, event):
        """Convertir click en widget a coordenadas de imagen."""
        if event.button() == Qt.LeftButton and self._qimg is not None:
            # Obtener coordenadas del click en el widget
            widget_x = event.x()
            widget_y = event.y()

            # Convertir a coordenadas de imagen
            if self._img_width > 0 and self._img_height > 0:
                pixmap = self.pixmap()
                if pixmap:
                    # Calcular offset del pixmap centrado
                    px_w = pixmap.width()
                    px_h = pixmap.height()
                    offset_x = (self.width() - px_w) // 2
                    offset_y = (self.height() - px_h) // 2

                    # Coordenadas relativas al pixmap
                    rel_x = widget_x - offset_x
                    rel_y = widget_y - offset_y

                    # Escalar a imagen original
                    if 0 <= rel_x < px_w and 0 <= rel_y < px_h:
                        img_x = int(rel_x * self._img_width / px_w)
                        img_y = int(rel_y * self._img_height / px_h)
                        img_x = max(0, min(self._img_width - 1, img_x))
                        img_y = max(0, min(self._img_height - 1, img_y))
                        self.clicked.emit(img_x, img_y)
        super().mousePressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()

    def _update_pixmap(self):
        if self._qimg is None or self._qimg.isNull():
            return
        target_w = int(self.width())
        target_h = int(self.height())
        # Preserve the last valid pixmap while layouts are still settling.
        if target_w <= 8 or target_h <= 8:
            return
        pix = QPixmap.fromImage(self._qimg)
        if pix.isNull() or pix.width() <= 1 or pix.height() <= 1:
            return
        transform = Qt.FastTransformation if CAMERA_FAST_SCALE else Qt.SmoothTransformation
        pix = pix.scaled(target_w, target_h, Qt.KeepAspectRatio, transform)
        if pix.isNull() or pix.width() <= 1 or pix.height() <= 1:
            return
        self.setPixmap(pix)


class CameraController:
    """Camera IO and health checks separated from the main panel logic."""

    def __init__(self, panel: "ControlPanelV2") -> None:
        self._panel = panel

    def _sync_from_worker_snapshot(self, *, now=None) -> bool:
        p = self._panel
        if not p._camera_required or not getattr(p, "_camera_subscribed", False):
            return False
        if not getattr(p, "_ros_worker_started", False) or not p.ros_worker:
            return False
        if now is None:
            now = _runtime_time()
        max_age = max(
            2.0,
            float(CAMERA_READY_MAX_AGE_SEC),
            float(getattr(p, "_camera_warmup_grace_sec", 2.0) or 2.0),
        )
        synced = False
        topic_specs = [
            (str(getattr(p, "camera_topic", "") or "").strip(), "rgb"),
            (str(getattr(p, "_camera_depth_topic", "") or "").strip(), "depth"),
        ]
        for topic, kind in topic_specs:
            if not topic:
                continue
            try:
                snapshot = p.ros_worker.image_frame_snapshot(topic)
            except Exception:
                snapshot = None
            if not snapshot:
                continue
            qimg, w, h, fps, wall, count = snapshot
            age = max(0.0, now - float(wall))
            if int(count) <= 0 or age > max_age:
                continue
            if kind == "rgb":
                needs_sync = (p._camera_frame_count <= 0) or (float(wall) > float(p._last_camera_frame_ts) + 1e-6)
            else:
                needs_sync = (p._camera_depth_frame_count <= 0) or (
                    float(wall) > float(p._last_camera_depth_frame_ts) + 1e-6
                )
            if not needs_sync:
                continue
            self.on_image(topic, qimg, int(w), int(h), float(fps))
            p._emit_log_throttled(
                f"camera_snapshot_sync:{kind}",
                (
                    f"[CAMERA][RECOVER] snapshot_sync kind={kind} topic={topic} "
                    f"age={age:.2f}s count={int(count)} size={int(w)}x{int(h)}"
                ),
                min_interval=1.0,
            )
            synced = True
        return synced

    def schedule_reconnect(self, reason: str, delay_ms=None) -> None:
        p = self._panel
        if not p._camera_required or not p._bridge_running:
            return
        if p._camera_stream_ok or p._camera_reconnect_scheduled:
            return
        if delay_ms is None:
            exponent = min(max(0, int(p._camera_reconnect_attempts)), 3)
            delay_ms = min(
                p._camera_reconnect_max_delay_ms,
                p._camera_reconnect_base_delay_ms * (2 ** exponent),
            )
        delay_ms = max(250, int(delay_ms))
        reason = (reason or "camera_reconnect").strip() or "camera_reconnect"
        p._camera_reconnect_scheduled = True
        p._camera_reconnect_last_reason = reason
        p._emit_log_throttled(
            "camera_reconnect_schedule",
            (
                f"[CAMERA][RECOVER] programada reconexion en {delay_ms}ms "
                f"attempt={p._camera_reconnect_attempts + 1} reason={reason}"
            ),
            min_interval=1.0,
        )

        def _run() -> None:
            p._camera_reconnect_scheduled = False
            if p._camera_stream_ok or not p._camera_required or not p._bridge_running:
                return
            p._camera_reconnect_attempts += 1
            p._emit_log(
                f"[CAMERA][RECOVER] reconectando attempt={p._camera_reconnect_attempts} "
                f"reason={p._camera_reconnect_last_reason or reason}"
            )
            self.auto_connect()

        QTimer.singleShot(delay_ms, _run)

    def health_check(self) -> None:
        p = self._panel
        if not p._camera_required:
            return
        now = _runtime_time()
        self._sync_from_worker_snapshot(now=now)
        camera_ready, camera_fault, camera_source_down, age, in_grace = p._camera_runtime_flags(now)
        depth_required, depth_topic = p._camera_depth_expectation()
        depth_age = now - p._last_camera_depth_frame_ts if p._last_camera_depth_frame_ts > 0.0 else float("inf")
        if camera_source_down:
            p._camera_stream_ok = False
            p._camera_fault_since = 0.0
            p._camera_fault_active = False
            p._camera_fault_reason = ""
            if (now - getattr(p, "_last_camera_diag_log", 0.0)) > 2.5:
                p._emit_log(f"[CAMERA][DIAG] source_down gazebo={p._gazebo_state()} bridge={str(p._bridge_running).lower()}")
                p._last_camera_diag_log = now
            p.camera_info.setText("Cámara: source down")
            p.camera_info.setStyleSheet("color: #f59e0b; font-weight: bold;")
            return
        if p._camera_frame_count <= 0 and in_grace:
            p._camera_stream_ok = False
            warmup_txt = "Cámara: warmup"
            if depth_required:
                warmup_txt += " (RGB-D)"
            p.camera_info.setText(warmup_txt)
            p.camera_info.setStyleSheet("color: #f59e0b; font-weight: bold;")
            return
        if p._camera_subscribed and p._camera_frame_count <= 0:
            p._camera_stream_ok = False
            p.camera_info.setText("Sin imágenes")
            p.camera_info.setStyleSheet("color: #f43f5e; font-weight: bold;")
            self.schedule_reconnect("camera_no_frames", delay_ms=1500)
            return
        if depth_required and p._camera_depth_frame_count <= 0 and in_grace:
            p._camera_stream_ok = False
            p.camera_info.setText("Cámara: warmup depth")
            p.camera_info.setStyleSheet("color: #f59e0b; font-weight: bold;")
            return
        if depth_required and p._camera_subscribed and p._camera_depth_frame_count <= 0:
            p._camera_stream_ok = False
            p.camera_info.setText("Sin depth")
            p.camera_info.setStyleSheet("color: #f43f5e; font-weight: bold;")
            self.schedule_reconnect("camera_depth_no_frames", delay_ms=1500)
            return
        if p._camera_subscribed and (camera_fault or (p._camera_frame_count > 0 and age > p._camera_fault_age_sec)):
            if (now - getattr(p, "_last_camera_diag_log", 0.0)) > 2.5:
                p._emit_log(
                    f"[CAMERA][DIAG] No llegan imágenes desde hace {age:.1f}s en {p.camera_topic or 'N/A'}"
                )
                if depth_required:
                    p._emit_log(
                        f"[CAMERA][DIAG] depth_required=true depth_topic={depth_topic or 'N/A'} "
                        f"depth_age={'inf' if math.isinf(depth_age) else f'{depth_age:.1f}s'}"
                    )
                p._last_camera_diag_log = now
            p.camera_info.setText(f"Sin imágenes ({age:.1f}s)")
            p.camera_info.setStyleSheet("color: #f43f5e; font-weight: bold;")
            self.schedule_reconnect("camera_fault", delay_ms=1500)
        elif p._camera_subscribed and camera_ready:
            p.camera_info.setStyleSheet("")

    def update_topics_async(self, topics: object) -> None:
        self.update_topics(list(topics) if topics else [])

    def refresh_topics(self) -> None:
        p = self._panel
        p._log_button("Refresh topics")
        p._set_status("Detectando tópicos de imagen…")

        def worker():
            if not p._ros_worker_started:
                p._ensure_ros_worker_started()
            if not p.ros_worker.node_ready():
                p.signal_status.emit("Nodo ROS no listo para tópicos", True)
                return
            topics = p.ros_worker.topic_names_and_types()
            candidates = [name for name, _types in topics if is_camera_topic(name)]
            rgb_candidates = [name for name in candidates if not name.endswith("/depth_image")]
            depth_candidates = [name for name in candidates if name.endswith("/depth_image")]
            rgb_candidates.sort(key=lambda t: (0 if t == "/camera_overhead/image" else 1, t))
            depth_candidates.sort(key=lambda t: (0 if t == "/camera_overhead/depth_image" else 1, t))
            candidates = rgb_candidates + depth_candidates
            if candidates:
                if p._debug_logs_enabled:
                    p._log(f"[CAMERA] Candidate topics: {', '.join(candidates)}")
                p.signal_update_camera_topics.emit(candidates)
                p.signal_status.emit(f"Detectados {len(candidates)} tópicos de cámara", False)
            else:
                if p._camera_stream_ok or p._camera_frame_count > 0 or p._camera_subscribed:
                    if p._debug_logs_enabled:
                        p._log("[CAMERA] Tópicos aún no visibles en discovery (reintentando)")
                    p.signal_status.emit("Discovery cámara pendiente (reintentando)", False)
                    p.signal_schedule_camera_health_check.emit(2000)
                else:
                    if p._debug_logs_enabled:
                        p._log("[CAMERA] No se encontraron tópicos compatibles")
                    p.signal_status.emit("No se detectaron tópicos de cámara", False)

        p._run_async(worker)

    def schedule_health_check(self, delay_ms: int = 1800) -> None:
        p = self._panel
        if p._camera_health_retry_scheduled or not p._bridge_running:
            return
        p._camera_health_retry_scheduled = True

        def _run():
            p._camera_health_retry_scheduled = False
            self.check_topic_health()

        QTimer.singleShot(delay_ms, _run)

    def check_topic_health(self) -> None:
        p = self._panel
        if not p._camera_required:
            return
        now = _runtime_time()
        _camera_ready, _camera_fault, camera_source_down, _age, _in_grace = p._camera_runtime_flags(now)
        if camera_source_down:
            p._camera_stream_ok = False
            p._camera_fault_since = 0.0
            p._camera_fault_active = False
            p._camera_fault_reason = ""
            return
        if p._camera_topic_check_inflight or not p._bridge_running:
            return
        topic = p.camera_topic_combo.currentText().strip() or p.camera_topic
        if not topic:
            return
        p._camera_topic_check_inflight = True

        def worker():
            try:
                now = _runtime_time()
                _camera_ready, camera_fault, _camera_source_down, _age, in_grace = p._camera_runtime_flags(now)
                last_age = now - p._last_camera_frame_ts if p._last_camera_frame_ts else float("inf")
                depth_required, depth_topic = p._camera_depth_expectation()
                depth_last_age = (
                    now - p._last_camera_depth_frame_ts
                    if p._last_camera_depth_frame_ts
                    else float("inf")
                )
                depth_ready = (not depth_required) or (
                    p._camera_depth_frame_count >= 1 and depth_last_age < CAMERA_READY_MAX_AGE_SEC
                )
                ready = p._camera_frame_count >= 1 and last_age < CAMERA_READY_MAX_AGE_SEC and depth_ready
                p._camera_stream_ok = bool(ready)
                elapsed = max(1e-3, now - max(0.0, p._camera_subscribe_ts))
                p._camera_topic_hz = float(p._camera_frame_count) / elapsed if p._camera_frame_count > 0 else 0.0
                depth_hz = (
                    float(p._camera_depth_frame_count) / elapsed
                    if p._camera_depth_frame_count > 0
                    else 0.0
                )
                rgb_pub_count = p.ros_worker.topic_publisher_count(topic) if p.ros_worker else 0
                depth_pub_count = (
                    p.ros_worker.topic_publisher_count(depth_topic)
                    if depth_required and depth_topic and p.ros_worker
                    else 0
                )
                p._emit_log_throttled(
                    "camera_health_stats",
                    (
                        f"[CAMERA][HEALTH] topic={topic} pubs={rgb_pub_count} "
                        f"frames={p._camera_frame_count} hz={p._camera_topic_hz:.2f} "
                        f"last_age={'inf' if math.isinf(last_age) else f'{last_age:.2f}s'} "
                        f"depth_required={str(depth_required).lower()} "
                        f"depth_topic={depth_topic or 'n/a'} depth_pubs={depth_pub_count} "
                        f"depth_frames={p._camera_depth_frame_count} depth_hz={depth_hz:.2f} "
                        f"depth_last_age={'inf' if math.isinf(depth_last_age) else f'{depth_last_age:.2f}s'} "
                        f"ready={str(p._camera_stream_ok).lower()}"
                    ),
                    min_interval=1.5,
                )
                if p._debug_logs_enabled:
                    p._emit_log(
                        f"[BRIDGE] camera_topic={topic} frames={p._camera_frame_count} "
                        f"ready={p._camera_stream_ok} last_age={last_age:.2f}s"
                    )
                    if p._camera_stream_ok:
                        p._emit_log(f"[CAMERA] ready=True age={last_age:.2f}s")
                if not p._camera_stream_ok:
                    in_init_grace = in_grace
                    if in_init_grace or (p._camera_frame_count == 0 and last_age < 2.0):
                        p.signal_status.emit("Cámara esperando frames…", False)
                    else:
                        if depth_required and not depth_ready:
                            p.signal_status.emit(
                                f"Cámara no lista; depth pendiente ({depth_topic or 'n/a'})",
                                False,
                            )
                        if camera_fault and p._camera_fault_since <= 0.0:
                            p._camera_fault_since = now
                        if camera_fault and p._camera_fault_since > 0.0:
                            elapsed = now - p._camera_fault_since
                            if elapsed >= p._camera_fault_persist_sec:
                                p._camera_fault_active = True
                                p._camera_fault_reason = (
                                    f"camera_fault persistente age={last_age:.1f}s "
                                    f"persist={elapsed:.1f}s topic={topic}"
                                )
                                p.signal_status.emit("Cámara degradada (diagnóstico)", True)
                                p._emit_log(f"[CAMERA][DIAG] {p._camera_fault_reason}")
                            else:
                                p.signal_status.emit("Cámara no lista; esperando", False)
                        else:
                            p._camera_fault_since = 0.0
                            p.signal_status.emit("Cámara no lista; esperando", False)
                    p.signal_schedule_camera_health_check.emit(2000)
                elif p._objects_settled and not p._calibration_ready:
                    p._camera_fault_since = 0.0
                    p._camera_fault_active = False
                    p._camera_fault_reason = ""
                    p.signal_calibration_check.emit()
            finally:
                p._camera_topic_check_inflight = False
                p.signal_refresh_controls.emit()

        p._run_async(worker)

    def update_topics(self, topics) -> None:
        p = self._panel
        current = p.camera_topic_combo.currentText()
        p.camera_topic_combo.clear()
        for topic in topics:
            p.camera_topic_combo.addItem(topic)
        idx = p.camera_topic_combo.findText(current)
        if idx >= 0:
            p.camera_topic_combo.setCurrentIndex(idx)

    def _subscription_stale(self) -> bool:
        p = self._panel
        if not p._camera_subscribed:
            return False
        if not p.camera_topic:
            return True
        if not p._camera_stream_ok or p._camera_frame_count <= 0:
            return True
        if not p._last_camera_frame_ts:
            return True
        age = _runtime_time() - p._last_camera_frame_ts
        return age >= max(0.2, float(CAMERA_READY_MAX_AGE_SEC))

    def connect(self) -> None:
        p = self._panel
        p._log_button("Conectar cámara")
        if bool(getattr(p, "_script_motion_active", False)) and not bool(
            getattr(p, "_allow_camera_while_script_motion", False)
        ):
            p._emit_log("[CAMERA] connect bloqueado: robot en movimiento")
            p._set_status("Cámara bloqueada: robot en movimiento", error=False)
            return
        if not p._bridge_running:
            p._log_warning("Cámara en espera: bridge no activo")
            p._set_status("Cámara en espera: bridge no activo", error=False)
            return
        topic = p.camera_topic_combo.currentText().strip()
        if p._debug_logs_enabled:
            p._log(f"[CAMERA] Intentando conectar a: {topic}")
        if not topic:
            p._log_error("Tópico de cámara vacío")
            p._set_status("Tópico de cámara vacío", error=True)
            return
        if p._camera_subscribed:
            if topic == p.camera_topic:
                if not self._subscription_stale():
                    if p._debug_logs_enabled:
                        p._log(f"[CAMERA] Ya conectado a: {topic}")
                    return
                if p._debug_logs_enabled:
                    p._log(f"[CAMERA] Reintentando suscripción a: {topic}")
            if p._debug_logs_enabled:
                p._log(f"[CAMERA] Desuscribiendo de: {p.camera_topic}")
            self.unsubscribe()
        p.camera_topic = topic
        p.camera_info.setText("Conectando…")
        p.camera_view.setText("Conectando…")
        p._camera_status_connected = False
        self.subscribe(topic)

    def subscribe(self, topic: str) -> bool:
        p = self._panel
        msg_type = self.resolve_msg_type(topic)
        if p._debug_logs_enabled:
            p._log(f"[CAMERA] Suscribiendo a {topic} (tipo={msg_type})")
        self.clear_frame(reset_info=False)
        p._camera_initializing = True
        p._camera_init_start = _runtime_time()
        p._camera_subscribe_ts = p._camera_init_start
        p._critical_camera_deadline = time.monotonic() + max(0.1, CAMERA_INIT_GRACE_SEC)
        p._camera_frame_count = 0
        p._camera_depth_frame_count = 0
        p._last_camera_depth_frame_ts = 0.0
        p._camera_stream_ok = False
        p._camera_fault_since = 0.0
        p._camera_fault_active = False
        p._camera_fault_reason = ""
        try:
            subscribed = p.ros_worker.subscribe_image(topic, msg_type=msg_type)
        except Exception as exc:
            subscribed = False
            p._log_error(f"Error suscribiendo cámara: {exc}")
        p._camera_msg_type = msg_type
        if not subscribed:
            p._log_camera_diagnostics("suscripción a cámara fallida")
            p._set_status("Cámara: nodo ROS no listo (reintentando)", error=False)
            p._camera_subscribed = False
            return False
        p._camera_subscribed = True
        p._set_status(f"Suscrito a {topic}", error=False)
        if p._debug_logs_enabled:
            p._log(f"[CAMERA] Suscripción OK a {topic}")
        self._ensure_depth_subscription()
        QTimer.singleShot(350, self.health_check)
        self.start_health_check(1200)
        return True

    def start_health_check(self, delay_ms: int = 1200) -> None:
        p = self._panel
        p._camera_health_timer.setInterval(delay_ms)
        if not p._camera_health_timer.isActive():
            p._camera_health_timer.start()

    def unsubscribe(self) -> None:
        p = self._panel
        if not p._camera_subscribed or not p.camera_topic:
            return
        try:
            p.ros_worker.unsubscribe_image(p.camera_topic)
        except Exception as exc:
            p._log_warning(f"Error desuscribiendo: {exc}")
        depth_topic = p._camera_depth_topic
        if depth_topic:
            try:
                p.ros_worker.unsubscribe_image(depth_topic)
            except Exception:
                pass
        p._camera_subscribed = False
        self.clear_frame()

    def clear_frame(self, *, reset_info: bool = True) -> None:
        p = self._panel
        with p._camera_frame_lock:
            p._camera_pending_frame = None
        p.camera_view.setPixmap(QPixmap())
        if reset_info:
            p.camera_view.setText("Sin conexión")
            p.camera_info.setText("Sin conexión")
        p._camera_status_connected = False
        p._camera_msg_type = "image"
        p._last_camera_frame_ts = 0.0
        p._camera_frame_count = 0
        p._last_camera_depth_frame_ts = 0.0
        p._camera_depth_frame_count = 0
        p._camera_stream_ok = False

    def _ensure_depth_subscription(self) -> None:
        p = self._panel
        depth_required, depth_topic = p._camera_depth_expectation()
        p._camera_depth_topic = depth_topic
        if not depth_required or not depth_topic:
            return
        if not p._ros_worker_started or not p.ros_worker.node_ready():
            return
        try:
            subscribed = p.ros_worker.subscribe_image(depth_topic, msg_type="image")
        except Exception:
            subscribed = False
        if subscribed:
            p._emit_log(
                f"[CAMERA] depth subscription OK topic={depth_topic} required=true"
            )
        else:
            p._emit_log(
                f"[CAMERA][WARN] depth subscription failed topic={depth_topic} required=true"
            )

    def resolve_msg_type(self, topic: str) -> str:
        p = self._panel
        normalized = topic.lower()
        for name, types in p.ros_worker.topic_names_and_types():
            if name != topic:
                continue
            for t in types:
                lower = t.lower()
                if "compressedimage" in lower:
                    return "compressed"
                if "image" in lower:
                    return "image"
        if "compressed" in normalized:
            return "compressed"
        return "image"

    def auto_connect(self) -> None:
        p = self._panel
        if p._closing:
            p._emit_log("[CAMERA] auto_connect cancelado: panel cerrando")
            return
        if not p._camera_required:
            p._emit_log("[CAMERA] auto_connect cancelado: camera_required=false")
            return
        if bool(getattr(p, "_script_motion_active", False)):
            p._emit_log("[CAMERA] auto_connect diferido: script_motion_active")
            self.schedule_reconnect("script_motion_active", delay_ms=2000)
            return
        if not p._bridge_running:
            if p._debug_logs_enabled:
                p._log("[CAMERA] Bridge aún no activo, reintentando auto-conexión en 1s")
            QTimer.singleShot(1000, self.auto_connect)
            return
        if p._camera_subscribed and not self._subscription_stale():
            return
        if p._camera_subscribed and self._subscription_stale():
            if p._debug_logs_enabled:
                p._log("[CAMERA] Suscripción estancada; forzando reconexión")
            self.unsubscribe()
        if not p.ros_worker.node_ready():
            p._log_camera_diagnostics("auto-connect esperando nodo ROS")
            if p._debug_logs_enabled:
                p._log("[CAMERA] Nodo ROS aún no listo, reintentando conexión en 1s")
            QTimer.singleShot(1000, self.auto_connect)
            return
        if p.camera_topic_combo.count() == 0:
            self.refresh_topics()
            QTimer.singleShot(1500, self.auto_connect)
            return
        if p.camera_topic_combo.currentText().strip().endswith("/depth_image"):
            rgb_idx = p.camera_topic_combo.findText("/camera_overhead/image")
            if rgb_idx >= 0:
                p.camera_topic_combo.setCurrentIndex(rgb_idx)
        if p._debug_logs_enabled:
            p._log("[CAMERA] Auto-conectando cámara...")
        p.signal_connect_camera.emit()
        if not p._camera_stream_ok:
            self.schedule_reconnect("auto_connect_wait_frames", delay_ms=1500)

    def on_image(self, topic: str, qimg, w: int, h: int, fps: float) -> None:
        p = self._panel
        now = _runtime_time()
        if topic == p._camera_depth_topic and topic != p.camera_topic:
            p._last_camera_depth_frame_ts = now
            p._camera_depth_frame_count += 1
            p._tfm_preprocessed_cache = None
            return
        if topic != p.camera_topic:
            return
        with p._camera_frame_lock:
            p._camera_pending_frame = (topic, qimg, w, h, fps, now)
        if p.tfm_module and not CAMERA_SKIP_TFM_INPUT:
            info = p.tfm_module.model_info() if p.tfm_module else {}
            in_channels = int(info.get("in_channels", 3) or 3)
            if CAMERA_PREPROCESS_TFM:
                try:
                    pre = build_tfm_preprocessed_input(p, qimg, w, h, now)
                    if pre is not None:
                        p.tfm_module.set_input_image(pre, preprocessed=True)
                        _store_preprocessed_cache(
                            p,
                            frame_ts=now,
                            preprocessed=pre,
                            in_channels=in_channels,
                        )
                    elif in_channels == 3:
                        p.tfm_module.set_input_image(qimg, width=w, height=h)
                except Exception:
                    if in_channels == 3:
                        p.tfm_module.set_input_image(qimg, width=w, height=h)
            else:
                p.tfm_module.set_input_image(qimg, width=w, height=h)
        p._camera_last_fps = fps
        if CAMERA_TRACK_FPS:
            p._update_fps_stats(fps)
        p._last_camera_frame_ts = now
        p._camera_frame_count += 1
        if p._camera_frame_count >= 1:
            p._camera_stream_ok = True
            p._camera_ever_ok = True
            p._camera_fault_since = 0.0
            p._camera_fault_active = False
            p._camera_fault_reason = ""
            if p._metrics_enabled and p._camera_frame_count == 1:
                p._metric_mark("camera_ready")
        p._reset_camera_retry_backoff()
        if p._camera_initializing:
            p._camera_initializing = False
            p._camera_init_start = 0.0

    def refresh_display(self) -> None:
        p = self._panel
        frame = None
        with p._camera_frame_lock:
            frame = p._camera_pending_frame
            p._camera_pending_frame = None
        if not frame:
            return
        topic, qimg, w, h, fps, ts = frame
        if int(w) <= 2 or int(h) <= 2:
            p._emit_log_throttled(
                "camera:tiny_source_frame",
                f"[CAMERA][WARN] tiny_source_frame dropped topic={topic} size={int(w)}x{int(h)}",
                min_interval=1.0,
            )
            return
        # Keep a fresh frame cache for TFM/runtime capture even in offscreen runs.
        p._last_camera_frame = (qimg, w, h, ts)
        if CAMERA_UI_SKIP_HIDDEN and (not p.isVisible() or not p.camera_view.isVisible()):
            return
        display = qimg
        if w > 0 and h > 0:
            if OVERLAY_CALIB and (p._calibrating or (time.time() <= p._calib_grid_until)):
                display = p._draw_calib_overlay(display, w, h)
            overhead_only = p._overhead_camera_active(topic)
            if p._should_draw_reach_overlay(overhead_only):
                display = p._draw_reach_overlay(display, w, h)
            if p._should_draw_selection_overlay(overhead_only):
                display = p._draw_selection_overlay(display, w, h)
            if overhead_only and p._last_grasp_px:
                display = p._draw_grasp_overlay(display, w, h)
            if p._should_draw_test_corner_overlay():
                display = p._draw_test_corner_overlay(display, w, h)
            # Mantener permanente la linea canonica TCP↔OBJ en overhead.
            if p._should_draw_tcp_pose_overlay(overhead_only):
                display = p._draw_tcp_pose_overlay(display, w, h)
        p.camera_view.set_frame(display, w, h)
        now = _runtime_time()
        if (now - p._camera_info_last_ts) >= max(0.05, float(CAMERA_INFO_INTERVAL_SEC)):
            p.camera_info.setText(f"Conectado · {w}x{h} · fps {fps:.1f}")
            # TFM/MoveIt grasp execution uses _motion_in_progress rather than
            # the manual/script flags, so include it in the panel state label.
            _moving = bool(
                p._manual_inflight
                or p._script_motion_active
                or getattr(p, "_motion_in_progress", False)
            )
            p.motion_lbl.setText("moving" if _moving else "idle")
            p.motion_lbl.setStyleSheet(
                "color:#f59e0b; font-weight:bold; font-size:13px;" if _moving
                else "color:#22c55e; font-weight:bold; font-size:13px;"
            )
            p._camera_info_last_ts = now
        if not p._camera_status_connected:
            p._set_status(f"Cámara: {topic} conectada", error=False)
            p._camera_status_connected = True
        p._last_camera_frame_ts = ts
