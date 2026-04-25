#!/usr/bin/env python3
"""Simple Qt panel: camera + DIRECTO and MOVEIT buttons."""
from __future__ import annotations

import sys
import threading

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

_FULL_SEQUENCE = ["HOME", "APPROACH", "DOWN", "CLOSE", "LIFT"]


class _Signals(QObject if _QT_OK else object):  # type: ignore
    log_appended = pyqtSignal(str) if _QT_OK else None  # type: ignore


class SimplePanelNode(Node):
    def __init__(self):
        super().__init__("simple_panel")
        self._client = StepPickClient(self, on_status=self._on_status)
        self._signals = _Signals()

    def _on_status(self, msg: String) -> None:
        if self._signals.log_appended:
            self._signals.log_appended.emit(msg.data)


class SimplePanelWindow(QWidget if _QT_OK else object):  # type: ignore
    TITLE = "UR5 Visual Autopick — Panel Simple"

    def __init__(self, node: SimplePanelNode):
        if not _QT_OK:
            return
        super().__init__()
        self._node = node
        self._client = node._client
        self._busy = False

        self.setWindowTitle(self.TITLE)
        self._build_ui()
        node._signals.log_appended.connect(self._append_log)

    def _build_ui(self) -> None:
        main = QVBoxLayout(self)

        # Camera
        cam_box = QGroupBox("Cámara overhead")
        cam_lay = QVBoxLayout(cam_box)
        self._cam = CameraWidget(self._node, topic="/camera_overhead/image")
        cam_lay.addWidget(self._cam)
        main.addWidget(cam_box)

        # Controls row
        ctrl = QHBoxLayout()

        obj_box = QGroupBox("Objeto")
        obj_lay = QVBoxLayout(obj_box)
        self._obj_combo = QComboBox()
        self._obj_combo.addItem("pick_demo")
        obj_lay.addWidget(self._obj_combo)
        ctrl.addWidget(obj_box)

        btn_box = QGroupBox("Acción")
        btn_lay = QHBoxLayout(btn_box)

        self._btn_directo = QPushButton("DIRECTO")
        self._btn_directo.setFixedHeight(60)
        self._btn_directo.setStyleSheet(
            "QPushButton{background:#1a6e2a;color:white;font-size:16px;font-weight:bold;border-radius:6px;}"
            "QPushButton:disabled{background:#555;}"
        )
        self._btn_directo.clicked.connect(lambda: self._run_sequence("DIRECTO"))
        btn_lay.addWidget(self._btn_directo)

        self._btn_moveit = QPushButton("MOVEIT")
        self._btn_moveit.setFixedHeight(60)
        self._btn_moveit.setStyleSheet(
            "QPushButton{background:#1a3a6e;color:white;font-size:16px;font-weight:bold;border-radius:6px;}"
            "QPushButton:disabled{background:#555;}"
        )
        self._btn_moveit.clicked.connect(lambda: self._run_sequence("MOVEIT"))
        btn_lay.addWidget(self._btn_moveit)

        ctrl.addWidget(btn_box)
        main.addLayout(ctrl)

        # Log
        log_box = QGroupBox("Estado / Log")
        log_lay = QVBoxLayout(log_box)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(120)
        self._log.setFont(QFont("Monospace", 9))
        log_lay.addWidget(self._log)
        main.addWidget(log_box)

    def _append_log(self, text: str) -> None:
        self._log.append(text)
        cursor = self._log.textCursor()
        cursor.movePosition(cursor.End)
        self._log.setTextCursor(cursor)
        # Trim to 200 lines
        doc = self._log.document()
        while doc.lineCount() > 200:
            cursor = self._log.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _run_sequence(self, mode: str) -> None:
        if self._busy:
            return
        self._set_busy(True)
        thread = threading.Thread(
            target=self._sequence_worker, args=(mode,), daemon=True
        )
        thread.start()

    def _sequence_worker(self, mode: str) -> None:
        import time
        delays = {"HOME": 5.0, "APPROACH": 4.5, "DOWN": 3.0, "CLOSE": 2.0, "LIFT": 3.5}
        for phase in _FULL_SEQUENCE:
            self._client.send_command(mode, phase)
            time.sleep(delays.get(phase, 3.0))
        self._set_busy(False)

    def _set_busy(self, busy: bool) -> None:
        self._busy = busy
        self._btn_directo.setEnabled(not busy)
        self._btn_moveit.setEnabled(not busy)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimplePanelNode()

    if not _QT_OK:
        node.get_logger().error("PyQt5 not available. Cannot open panel.")
        rclpy.spin(node)
        return

    app = QApplication(sys.argv)
    win = SimplePanelWindow(node)
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
