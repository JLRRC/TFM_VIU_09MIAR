"""PyQt5 panel for visual autopick — shows camera, status, and control buttons."""
from __future__ import annotations

import threading
from typing import Dict, Optional

try:
    import cv2
    from cv_bridge import CvBridge
    _CV_OK = True
except ImportError:
    _CV_OK = False

try:
    from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
    from PyQt5.QtGui import QImage, QPixmap, QFont, QColor
    from PyQt5.QtWidgets import (
        QWidget,
        QVBoxLayout,
        QHBoxLayout,
        QLabel,
        QPushButton,
        QTextEdit,
        QGroupBox,
        QGridLayout,
        QSizePolicy,
    )
    _QT_OK = True
except ImportError:
    _QT_OK = False


def _color_label(text: str, ok: bool) -> str:
    """Return text with HTML color tag for status indicators."""
    color = "#00cc44" if ok else "#cc2200"
    return f'<span style="color:{color}; font-weight:bold;">{text}</span>'


class _Signals(QObject if _QT_OK else object):
    status_updated = pyqtSignal(dict) if _QT_OK else None  # type: ignore
    log_appended = pyqtSignal(str) if _QT_OK else None  # type: ignore
    image_updated = pyqtSignal(object) if _QT_OK else None  # type: ignore


class VisualAutopickPanel(QWidget if _QT_OK else object):  # type: ignore
    """Main PyQt5 panel window."""

    WINDOW_TITLE = "UR5 Visual Autopick"
    IMG_W = 480
    IMG_H = 360

    def __init__(self, node) -> None:
        if not _QT_OK:
            raise ImportError("PyQt5 is not available")
        super().__init__()
        self._node = node
        self._bridge = CvBridge() if _CV_OK else None
        self._signals = _Signals()
        self._signals.status_updated.connect(self._on_status_updated)
        self._signals.log_appended.connect(self._on_log_appended)
        self._signals.image_updated.connect(self._on_image_updated)

        self._build_ui()
        self.setWindowTitle(self.WINDOW_TITLE)
        self.resize(900, 720)

        # Register callback to receive status from node
        node.register_status_callback(self._status_from_node)

        # Poll for camera image
        self._img_timer = QTimer(self)
        self._img_timer.timeout.connect(self._poll_image)
        self._img_timer.start(100)  # 10 Hz

    # ------------------------------------------------------------------ UI --
    def _build_ui(self) -> None:
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # Left: camera feed
        left = QVBoxLayout()
        cam_group = QGroupBox("Cámara /camera_overhead/image")
        cam_layout = QVBoxLayout()
        self._img_label = QLabel()
        self._img_label.setFixedSize(self.IMG_W, self.IMG_H)
        self._img_label.setAlignment(Qt.AlignCenter)
        self._img_label.setStyleSheet("background-color: #111; color: #aaa;")
        self._img_label.setText("Esperando imagen...")
        cam_layout.addWidget(self._img_label)
        self._cam_status_label = QLabel("Cámara: --")
        cam_layout.addWidget(self._cam_status_label)
        cam_group.setLayout(cam_layout)
        left.addWidget(cam_group)
        left.addStretch()
        main_layout.addLayout(left)

        # Right: status + buttons + log
        right = QVBoxLayout()

        # Status grid
        status_group = QGroupBox("Estado del sistema")
        status_grid = QGridLayout()
        self._status_labels: Dict[str, QLabel] = {}

        rows = [
            ("phase", "Fase"),
            ("camera", "Cámara"),
            ("object", "pick_demo"),
            ("tf", "TF"),
            ("bridge", "MoveIt/bridge"),
            ("gripper", "Gripper"),
        ]
        for i, (key, label_text) in enumerate(rows):
            lbl_key = QLabel(f"{label_text}:")
            lbl_key.setFont(QFont("Monospace", 9, QFont.Bold))
            lbl_val = QLabel("--")
            lbl_val.setTextFormat(Qt.RichText)
            lbl_val.setMinimumWidth(220)
            status_grid.addWidget(lbl_key, i, 0)
            status_grid.addWidget(lbl_val, i, 1)
            self._status_labels[key] = lbl_val

        status_group.setLayout(status_grid)
        right.addWidget(status_group)

        # Buttons
        btn_group = QGroupBox("Control")
        btn_layout = QHBoxLayout()

        self._btn_arm = QPushButton("ARM AUTO PICK")
        self._btn_arm.setMinimumHeight(48)
        self._btn_arm.setStyleSheet(
            "background-color: #005500; color: white; font-weight: bold; font-size: 14px;"
        )
        self._btn_arm.clicked.connect(self._on_arm_clicked)
        btn_layout.addWidget(self._btn_arm)

        self._btn_stop = QPushButton("STOP")
        self._btn_stop.setMinimumHeight(48)
        self._btn_stop.setStyleSheet(
            "background-color: #550000; color: white; font-weight: bold; font-size: 14px;"
        )
        self._btn_stop.clicked.connect(self._on_stop_clicked)
        btn_layout.addWidget(self._btn_stop)

        self._btn_reset = QPushButton("RESET")
        self._btn_reset.setMinimumHeight(48)
        self._btn_reset.setStyleSheet(
            "background-color: #003366; color: white; font-weight: bold; font-size: 12px;"
        )
        self._btn_reset.clicked.connect(self._on_reset_clicked)
        btn_layout.addWidget(self._btn_reset)

        btn_group.setLayout(btn_layout)
        right.addWidget(btn_group)

        # Log
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setFont(QFont("Monospace", 8))
        self._log_text.setMinimumHeight(200)
        log_layout.addWidget(self._log_text)
        log_group.setLayout(log_layout)
        right.addWidget(log_group)

        main_layout.addLayout(right)

    # ---------------------------------------------------------------- slots --
    def _on_arm_clicked(self) -> None:
        self._append_log("[PANEL] ARM AUTO PICK pressed")
        ok, reason = self._node.arm_autopick()
        if ok:
            self._append_log(f"[PANEL] Armed: {reason}")
        else:
            self._append_log(f"[PANEL] Cannot arm: {reason}")

    def _on_stop_clicked(self) -> None:
        self._append_log("[PANEL] STOP pressed")
        self._node.stop()

    def _on_reset_clicked(self) -> None:
        self._append_log("[PANEL] RESET pressed")
        self._node.reset()

    def _on_status_updated(self, status: Dict) -> None:
        phase = status.get("phase", "?")
        self._status_labels["phase"].setText(
            f'<b><span style="color:#ffcc00;">{phase}</span></b>'
        )

        img = status.get("image", {})
        img_ok = img.get("received", False) and (img.get("age_sec") or 999) < 1.0
        img_txt = (
            f"OK frames={img.get('frame_count',0)} age={img.get('age_sec',0.0):.2f}s"
            if img_ok else "Sin imagen"
        )
        self._status_labels["camera"].setText(_color_label(img_txt, img_ok))

        obj_ok = status.get("object_detected", False)
        obj_pos = status.get("object_pos_world")
        obj_txt = (
            f"OK world=({obj_pos[0]:.3f},{obj_pos[1]:.3f},{obj_pos[2]:.3f})"
            if (obj_ok and obj_pos) else "No detectado"
        )
        self._status_labels["object"].setText(_color_label(obj_txt, obj_ok))

        tf_ok = status.get("tf_ok", False)
        tf_age = status.get("tf_age_sec")
        tf_txt = (
            f"OK age={tf_age:.3f}s" if (tf_ok and tf_age is not None) else
            f"FAIL {status.get('tf_error','')}"
        )
        self._status_labels["tf"].setText(_color_label(tf_txt, tf_ok))

        bridge_ok = status.get("bridge_alive", False)
        self._status_labels["bridge"].setText(
            _color_label("ALIVE" if bridge_ok else "OFFLINE", bridge_ok)
        )

        # Gripper: derive from phase
        gripper_txt = "OK" if phase in ("CLOSE_GRIPPER", "LIFT", "DONE") else "--"
        gripper_ok = phase in ("LIFT", "DONE")
        self._status_labels["gripper"].setText(_color_label(gripper_txt, gripper_ok))

        # Log last_log if new
        last_log = status.get("last_log", "")
        if last_log:
            self._append_log(last_log)

    def _on_log_appended(self, text: str) -> None:
        self._log_text.append(text)
        self._log_text.ensureCursorVisible()

    def _on_image_updated(self, pixmap) -> None:
        if pixmap is not None:
            self._img_label.setPixmap(
                pixmap.scaled(self.IMG_W, self.IMG_H, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

    # ---------------------------------------------------------------- polling --
    def _poll_image(self) -> None:
        if not _CV_OK or self._bridge is None:
            return
        msg = self._node._image_monitor.get_latest_image()
        if msg is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            px = QPixmap.fromImage(qt_img)
            self._signals.image_updated.emit(px)
        except Exception:
            pass

    def _status_from_node(self, status: Dict) -> None:
        """Called from ROS timer thread — re-emit via signal to Qt main thread."""
        self._signals.status_updated.emit(status)

    def _append_log(self, text: str) -> None:
        self._signals.log_appended.emit(text)
