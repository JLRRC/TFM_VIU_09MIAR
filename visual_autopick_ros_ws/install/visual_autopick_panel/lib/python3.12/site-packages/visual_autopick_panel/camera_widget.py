"""Reusable Qt camera viewer widget for ROS 2 Image topics."""
from __future__ import annotations

import threading
from typing import Optional

try:
    import cv2
    from cv_bridge import CvBridge
    _CV_OK = True
except ImportError:
    _CV_OK = False

try:
    from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
    from PyQt5.QtGui import QImage, QPixmap
    from PyQt5.QtWidgets import QLabel, QSizePolicy
    _QT_OK = True
except ImportError:
    _QT_OK = False

from sensor_msgs.msg import Image


class CameraWidget(QLabel if _QT_OK else object):  # type: ignore
    """QLabel that subscribes to a ROS Image topic and displays live frames."""

    IMG_W = 480
    IMG_H = 360

    def __init__(self, ros_node, topic: str = "/camera_overhead/image", parent=None):
        if not _QT_OK:
            return
        super().__init__(parent)
        self.setMinimumSize(self.IMG_W, self.IMG_H)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.setAlignment(Qt.AlignCenter)
        self._set_no_signal()

        self._bridge = CvBridge() if _CV_OK else None
        self._lock = threading.Lock()
        self._pending_frame: Optional[QImage] = None

        self._sub = ros_node.create_subscription(
            Image, topic, self._on_image, 10
        )

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)  # 10 Hz UI refresh

    def _set_no_signal(self) -> None:
        self.setText("<span style='color:#888;font-size:14px;'>Sin señal</span>")

    def _on_image(self, msg: Image) -> None:
        if not _CV_OK or self._bridge is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            cv_resized = cv2.resize(cv_img, (self.IMG_W, self.IMG_H))
            h, w, ch = cv_resized.shape
            qi = QImage(cv_resized.data, w, h, w * ch, QImage.Format_RGB888)
            with self._lock:
                self._pending_frame = qi.copy()
        except Exception:
            pass

    def _refresh(self) -> None:
        with self._lock:
            frame = self._pending_frame
            self._pending_frame = None
        if frame is not None:
            self.setPixmap(QPixmap.fromImage(frame))
