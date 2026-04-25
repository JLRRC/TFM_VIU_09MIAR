#!/usr/bin/env python3
"""Step-by-step Qt panel: camera + phase buttons + mode selector."""
from __future__ import annotations

import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

try:
    from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
    from PyQt5.QtGui import QFont
    from PyQt5.QtWidgets import (
        QApplication,
        QComboBox,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QPushButton,
        QSizePolicy,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
    _QT_OK = True
except ImportError:
    _QT_OK = False

from .camera_widget import CameraWidget
from .step_pick_client import StepPickClient

_PHASE_ORDER = ["HOME", "APPROACH", "DOWN", "CLOSE", "LIFT"]
_PHASE_DELAYS = {"HOME": 5.0, "APPROACH": 4.5, "DOWN": 3.0, "CLOSE": 2.0, "LIFT": 3.5}

_BTN_STYLE = (
    "QPushButton{{background:{bg};color:white;font-size:13px;font-weight:bold;"
    "border-radius:5px;padding:6px;}}"
    "QPushButton:disabled{{background:#555;}}"
)


class _Signals(QObject if _QT_OK else object):  # type: ignore
    status_updated = pyqtSignal(str) if _QT_OK else None  # type: ignore
    log_appended   = pyqtSignal(str) if _QT_OK else None  # type: ignore


class StepPanelNode(Node):
    def __init__(self):
        super().__init__("step_panel")
        self._signals = _Signals()
        self._client = StepPickClient(self, on_status=self._on_status)

    def _on_status(self, msg: String) -> None:
        if self._signals.status_updated:
            self._signals.status_updated.emit(msg.data)
        if self._signals.log_appended:
            self._signals.log_appended.emit(msg.data)


class StepPanelWindow(QWidget if _QT_OK else object):  # type: ignore
    TITLE = "UR5 Visual Autopick — Panel Paso a Paso"

    def __init__(self, node: StepPanelNode):
        if not _QT_OK:
            return
        super().__init__()
        self._node = node
        self._client = node._client
        self._busy = False
        self._next_phase_idx = 0  # index into _PHASE_ORDER

        self.setWindowTitle(self.TITLE)
        self._build_ui()

        node._signals.status_updated.connect(self._on_status_update)
        node._signals.log_appended.connect(self._append_log)

    def _build_ui(self) -> None:
        main = QVBoxLayout(self)

        # Camera
        cam_box = QGroupBox("Cámara overhead")
        cam_lay = QVBoxLayout(cam_box)
        self._cam = CameraWidget(self._node, topic="/camera_overhead/image")
        cam_lay.addWidget(self._cam)
        main.addWidget(cam_box)

        # Status bar
        status_box = QGroupBox("Estado del sistema")
        status_lay = QVBoxLayout(status_box)
        self._status_label = QLabel("IDLE")
        self._status_label.setFont(QFont("Monospace", 11))
        self._status_label.setAlignment(Qt.AlignCenter)
        status_lay.addWidget(self._status_label)
        main.addWidget(status_box)

        # Mode + Object selectors
        sel_row = QHBoxLayout()

        obj_box = QGroupBox("Objeto")
        obj_lay = QVBoxLayout(obj_box)
        self._obj_combo = QComboBox()
        self._obj_combo.addItem("pick_demo")
        obj_lay.addWidget(self._obj_combo)
        sel_row.addWidget(obj_box)

        mode_box = QGroupBox("Modo")
        mode_lay = QVBoxLayout(mode_box)
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["DIRECTO", "MOVEIT"])
        mode_lay.addWidget(self._mode_combo)
        sel_row.addWidget(mode_box)

        main.addLayout(sel_row)

        # Phase buttons
        phase_box = QGroupBox("Fases")
        phase_lay = QHBoxLayout(phase_box)
        self._phase_btns: dict[str, QPushButton] = {}
        colors = {
            "HOME": "#4a4a8a", "APPROACH": "#5a7a2a", "DOWN": "#7a5a2a",
            "CLOSE": "#8a2a2a", "LIFT": "#2a6a8a", "RESET": "#6a4a2a",
        }
        for label in list(_PHASE_ORDER) + ["RESET"]:
            btn = QPushButton(label)
            btn.setMinimumHeight(45)
            btn.setStyleSheet(_BTN_STYLE.format(bg=colors.get(label, "#444")))
            btn.clicked.connect(lambda checked, p=label: self._send_phase(p))
            phase_lay.addWidget(btn)
            self._phase_btns[label] = btn
        main.addWidget(phase_box)

        # Action buttons
        action_box = QGroupBox("Acciones")
        action_lay = QHBoxLayout(action_box)

        self._btn_next = QPushButton("▶ Siguiente paso")
        self._btn_next.setMinimumHeight(50)
        self._btn_next.setStyleSheet(_BTN_STYLE.format(bg="#2a6a3a"))
        self._btn_next.clicked.connect(self._send_next)
        action_lay.addWidget(self._btn_next)

        self._btn_full = QPushButton("▶▶ Secuencia completa")
        self._btn_full.setMinimumHeight(50)
        self._btn_full.setStyleSheet(_BTN_STYLE.format(bg="#1a3a6e"))
        self._btn_full.clicked.connect(self._run_full_sequence)
        action_lay.addWidget(self._btn_full)

        self._btn_abort = QPushButton("■ Parar / Abortar")
        self._btn_abort.setMinimumHeight(50)
        self._btn_abort.setStyleSheet(_BTN_STYLE.format(bg="#8a1a1a"))
        self._btn_abort.clicked.connect(self._abort)
        action_lay.addWidget(self._btn_abort)

        main.addWidget(action_box)

        # Log
        log_box = QGroupBox("Log")
        log_lay = QVBoxLayout(log_box)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(120)
        self._log.setFont(QFont("Monospace", 9))
        log_lay.addWidget(self._log)
        main.addWidget(log_box)

    def _current_mode(self) -> str:
        return self._mode_combo.currentText()

    def _send_phase(self, phase: str) -> None:
        mode = self._current_mode()
        self._client.send_command(mode, phase)
        if phase in _PHASE_ORDER:
            idx = _PHASE_ORDER.index(phase)
            self._next_phase_idx = idx + 1

    def _send_next(self) -> None:
        if self._busy:
            return
        if self._next_phase_idx >= len(_PHASE_ORDER):
            self._append_log("Secuencia completa. Usa RESET para reiniciar.")
            return
        phase = _PHASE_ORDER[self._next_phase_idx]
        self._send_phase(phase)

    def _run_full_sequence(self) -> None:
        if self._busy:
            return
        self._next_phase_idx = 0
        thread = threading.Thread(target=self._sequence_worker, daemon=True)
        thread.start()

    def _sequence_worker(self) -> None:
        self._set_busy(True)
        mode = self._current_mode()
        for phase in _PHASE_ORDER:
            self._client.send_command(mode, phase)
            time.sleep(_PHASE_DELAYS.get(phase, 3.0))
            self._next_phase_idx += 1
        self._set_busy(False)

    def _abort(self) -> None:
        mode = self._current_mode()
        self._client.abort(mode)
        self._next_phase_idx = 0
        self._set_busy(False)

    def _on_status_update(self, text: str) -> None:
        self._status_label.setText(text)

    def _append_log(self, text: str) -> None:
        self._log.append(text)
        cursor = self._log.textCursor()
        cursor.movePosition(cursor.End)
        self._log.setTextCursor(cursor)
        doc = self._log.document()
        while doc.lineCount() > 200:
            cursor = self._log.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _set_busy(self, busy: bool) -> None:
        self._busy = busy
        self._btn_full.setEnabled(not busy)
        self._btn_next.setEnabled(not busy)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StepPanelNode()

    if not _QT_OK:
        node.get_logger().error("PyQt5 not available. Cannot open panel.")
        rclpy.spin(node)
        return

    app = QApplication(sys.argv)
    win = StepPanelWindow(node)
    win.show()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
