#!/usr/bin/env python3
"""
autopick_panel.py
=================
Panel unificado UR5 Visual Autopick — visual_autopick_ros_ws.

Modos
-----
  AUTO        — Lanza la secuencia completa de forma automática (DIRECTO o MOVEIT).
  STEP_BY_STEP — Control paso a paso con diagnóstico detallado, idéntico en
                  estructura al panel_v2.py del workspace agarre_ros2_ws.

Flujos: DIRECTO | MOVEIT
Fases:  HOME | APPROACH | DOWN | CLOSE | LIFT

Topics
------
  /visual_autopick/step_command  (std_msgs/String, pub) → "MODE:PHASE"
  /visual_autopick/status        (std_msgs/String, sub) → "MODE:PHASE:STATE"
  /joint_states                  (sensor_msgs/JointState, sub)
  /world/visual_autopick_world/pose/info  (tf2_msgs/TFMessage, sub)
  /camera_overhead/image         (sensor_msgs/Image, sub) — via CameraWidget
"""
from __future__ import annotations

import math
import sys
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from PyQt5.QtCore import (
        QObject, Qt, QTimer, pyqtSignal,
    )
    from PyQt5.QtWidgets import (
        QAbstractScrollArea, QAbstractScrollArea as _QSA,
        QApplication, QCheckBox, QComboBox, QDialog, QGridLayout,
        QGroupBox, QHBoxLayout, QHeaderView, QLabel, QMainWindow,
        QPushButton, QScrollArea, QSizePolicy, QTableWidget,
        QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget,
    )
    _QT_OK = True
except ImportError:
    _QT_OK = False

from .camera_widget import CameraWidget
from .autopick_step_helpers import (
    step_phase_sequence,
    step_predict_next_phase,
    step_phase_intent,
    step_phase_gripper_state,
    step_phase_action_text,
    step_present_flow_name,
    step_known_tcp_world,
    step_phase_delay,
    ARM_JOINTS,
    JOINT_HOME_RAD,
    JOINT_MESA_RAD,
    JOINT_CESTA_RAD,
)

# Utility functions and StepByStepDialog live in step_by_step_panel.py
from .step_by_step_panel import StepByStepDialog, _fmt_xyz


# ──────────────────────────────────────────────────────────────────────────────
# Signals (for cross-thread UI updates)
# ──────────────────────────────────────────────────────────────────────────────

class _Signals(QObject if _QT_OK else object):  # type: ignore
    status_updated = pyqtSignal(str)  if _QT_OK else None  # type: ignore
    joints_updated = pyqtSignal()     if _QT_OK else None  # type: ignore
    log_appended   = pyqtSignal(str)  if _QT_OK else None  # type: ignore


# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 node
# ──────────────────────────────────────────────────────────────────────────────

class AutopickNode(Node):
    def __init__(self) -> None:
        super().__init__("autopick_panel")
        self.signals = _Signals()

        self._cmd_pub = self.create_publisher(String, "/visual_autopick/step_command", 10)
        self._arm_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self.create_subscription(String, "/visual_autopick/status", self._on_status, 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, qos_profile_sensor_data
        )
        self.create_subscription(
            TFMessage,
            "/world/visual_autopick_world/pose/info",
            self._on_gz_pose,
            qos_profile_sensor_data,
        )
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._last_status = ""
        self._last_joints: Dict[str, float] = {}
        self._gz_poses: Dict[str, Tuple[float, float, float]] = {}

    # ── Commands ─────────────────────────────────────────────────────────────

    def publish_joint_trajectory(self, angles: List[float], duration_sec: float = 4.0) -> None:
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = list(angles)
        pt.velocities = [0.0] * len(angles)
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec % 1.0) * 1e9)
        traj.points = [pt]
        self._arm_pub.publish(traj)

    def publish_gripper(self, open_gripper: bool) -> None:
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0] if open_gripper else [0.4, 0.4]
        self._gripper_pub.publish(msg)
        state = "OPEN" if open_gripper else "CLOSE"
        self.get_logger().info(f"[GRIPPER] {state}")
        if self.signals.log_appended:
            self.signals.log_appended.emit(f"[GRIPPER] {state}")

    def send_command(self, mode: str, phase: str) -> None:
        msg = String()
        msg.data = f"{mode.upper()}:{phase.upper()}"
        self._cmd_pub.publish(msg)
        self.get_logger().info(f"[CMD] {msg.data}")
        if self.signals.log_appended:
            self.signals.log_appended.emit(f"[CMD] {msg.data}")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_status(self, msg: String) -> None:
        self._last_status = msg.data
        if self.signals.status_updated:
            self.signals.status_updated.emit(msg.data)

    def _on_joints(self, msg: JointState) -> None:
        self._last_joints = dict(zip(msg.name, msg.position))
        if self.signals.joints_updated:
            self.signals.joints_updated.emit()

    def _on_gz_pose(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            p = tf.transform.translation
            self._gz_poses[tf.child_frame_id] = (p.x, p.y, p.z)

    # ── Queries ──────────────────────────────────────────────────────────────

    def get_tcp_world(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self._tf_buf.lookup_transform(
                "world", "rg2_pinch_center", Time(),
                timeout=Duration(seconds=0.1),
            )
            p = t.transform.translation
            return (p.x, p.y, p.z)
        except Exception:
            return None

    def get_tool0_world(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self._tf_buf.lookup_transform(
                "world", "tool0", Time(), timeout=Duration(seconds=0.1)
            )
            p = t.transform.translation
            return (p.x, p.y, p.z)
        except Exception:
            return None

    def get_tcp_base(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self._tf_buf.lookup_transform(
                "base_link", "rg2_pinch_center", Time(),
                timeout=Duration(seconds=0.1),
            )
            p = t.transform.translation
            return (p.x, p.y, p.z)
        except Exception:
            return None

    def get_object_world(self) -> Optional[Tuple[float, float, float]]:
        for name in ["pick_demo", "visual_autopick_world::pick_demo"]:
            pos = self._gz_poses.get(name)
            if pos is not None:
                return pos
        return None

    def get_base_link_world(self) -> Optional[Tuple[float, float, float]]:
        for name in ["base_link", "ur5_rg2::base_link"]:
            pos = self._gz_poses.get(name)
            if pos is not None:
                return pos
        return None

    def get_gripper_str(self) -> str:
        j1 = self._last_joints.get("rg2_finger_joint1")
        if j1 is None:
            return "--"
        state = "cerrado" if j1 < 0.005 else ("abierto" if j1 > 0.038 else "parcial")
        return f"{j1:.4f} m ({state})"

    def get_arm_joints_str(self) -> str:
        names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ]
        parts = []
        for n in names:
            v = self._last_joints.get(n)
            short = n.split("_")[0]
            parts.append(f"{short}={v:+.3f}" if v is not None else f"{short}=--")
        return "  ".join(parts)

    def get_all_joints_str(self) -> str:
        names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
            "rg2_finger_joint1", "rg2_finger_joint2",
        ]
        lines = []
        for n in names:
            v = self._last_joints.get(n)
            lines.append(f"{n}: {v:+.4f} rad/m" if v is not None else f"{n}: --")
        return "\n".join(lines)




# ──────────────────────────────────────────────────────────────────────────────
# Main window
# ──────────────────────────────────────────────────────────────────────────────

class AutopickMainWindow(QMainWindow if _QT_OK else object):  # type: ignore
    TITLE = "UR5 Visual Autopick"

    def __init__(self, node: AutopickNode) -> None:
        if not _QT_OK:
            return
        super().__init__()
        self._node = node
        self._flow = "DIRECTO"
        self._step_dlg: Optional[StepByStepDialog] = None
        self._auto_busy = False
        self._gripper_open = True  # track current gripper state

        self.setWindowTitle(self.TITLE)
        self.resize(560, 680)
        self._build_ui()

        node.signals.status_updated.connect(self._on_status)
        node.signals.log_appended.connect(self._append_log)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(1000)
        self._refresh_timer.timeout.connect(self._refresh_status_bar)
        self._refresh_timer.start()

    # ── UI ───────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(8)
        layout.setContentsMargins(10, 10, 10, 10)

        # ── Top bar: mode + flow combos ───────────────────────────────────────
        top = QHBoxLayout()
        top.addWidget(QLabel("Modo:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["AUTO", "STEP_BY_STEP"])
        self._mode_combo.currentTextChanged.connect(self._on_mode_changed)
        top.addWidget(self._mode_combo)
        top.addSpacing(12)
        top.addWidget(QLabel("Flujo:"))
        self._flow_combo = QComboBox()
        self._flow_combo.addItems(["DIRECTO", "MOVEIT"])
        self._flow_combo.currentTextChanged.connect(self._on_flow_changed)
        top.addWidget(self._flow_combo)
        top.addSpacing(8)
        self._btn_iniciar = QPushButton("Iniciar")
        self._btn_iniciar.setStyleSheet(
            "QPushButton{background:#16a34a;color:white;font-size:13px;font-weight:700;"
            "border-radius:5px;padding:5px 18px;}"
            "QPushButton:disabled{background:#94a3b8;}"
        )
        self._btn_iniciar.clicked.connect(lambda: self._run_auto(self._flow_combo.currentText()))
        top.addWidget(self._btn_iniciar)
        top.addSpacing(8)
        self._btn_gripper = QPushButton("Pinza: Abrir")
        self._btn_gripper.setStyleSheet(
            "QPushButton{background:#0369a1;color:white;font-size:13px;font-weight:700;"
            "border-radius:5px;padding:5px 14px;}"
            "QPushButton:disabled{background:#94a3b8;}"
        )
        self._btn_gripper.clicked.connect(self._toggle_gripper)
        top.addWidget(self._btn_gripper)
        top.addStretch()
        layout.addLayout(top)

        # ── Camera ───────────────────────────────────────────────────────────
        cam_box = QGroupBox("Cámara overhead")
        cam_lay = QVBoxLayout(cam_box)
        self._cam = CameraWidget(self._node, topic="/camera_overhead/image")
        cam_lay.addWidget(self._cam, alignment=Qt.AlignCenter)
        layout.addWidget(cam_box)

        # ── Status label ─────────────────────────────────────────────────────
        self._lbl_status = QLabel("Estado: --")
        self._lbl_status.setStyleSheet("font-size:13px;")
        layout.addWidget(self._lbl_status)

        # ── Robot poses (siempre visibles) ────────────────────────────────────
        pose_box = QGroupBox("Robot")
        pose_row = QHBoxLayout(pose_box)
        pose_row.setSpacing(6)
        _btn_style = (
            "QPushButton{{background:{bg};color:white;font-size:13px;font-weight:700;"
            "border-radius:5px;padding:7px 14px;}}"
            "QPushButton:disabled{{background:#94a3b8;}}"
        )
        self._btn_home_pose = QPushButton("UR5 → HOME")
        self._btn_home_pose.setStyleSheet(_btn_style.format(bg="#0369a1"))
        self._btn_home_pose.clicked.connect(lambda: self._go_pose("HOME"))

        self._btn_mesa_pose = QPushButton("UR5 → Mesa")
        self._btn_mesa_pose.setStyleSheet(_btn_style.format(bg="#047857"))
        self._btn_mesa_pose.clicked.connect(lambda: self._go_pose("MESA"))

        self._btn_cesta_pose = QPushButton("UR5 → Cesta")
        self._btn_cesta_pose.setStyleSheet(_btn_style.format(bg="#b45309"))
        self._btn_cesta_pose.clicked.connect(lambda: self._go_pose("CESTA"))

        pose_row.addWidget(self._btn_home_pose)
        pose_row.addWidget(self._btn_mesa_pose)
        pose_row.addWidget(self._btn_cesta_pose)
        layout.addWidget(pose_box)

        self._baseline_busy = False

        # ── AUTO controls (visible in AUTO mode) ──────────────────────────────
        self._auto_group = QGroupBox("Control AUTO — secuencia completa")
        auto_lay = QVBoxLayout(self._auto_group)

        btn_row = QHBoxLayout()
        self._btn_directo = QPushButton("DIRECTO")
        self._btn_directo.setStyleSheet(
            "QPushButton{background:#0284c7;color:white;font-size:14px;font-weight:700;"
            "border-radius:6px;padding:10px;}"
            "QPushButton:disabled{background:#94a3b8;}"
        )
        self._btn_directo.clicked.connect(lambda: self._run_auto("DIRECTO"))

        self._btn_moveit = QPushButton("MOVEIT")
        self._btn_moveit.setStyleSheet(
            "QPushButton{background:#7c3aed;color:white;font-size:14px;font-weight:700;"
            "border-radius:6px;padding:10px;}"
            "QPushButton:disabled{background:#94a3b8;}"
        )
        self._btn_moveit.clicked.connect(lambda: self._run_auto("MOVEIT"))

        self._btn_abort = QPushButton("ABORTAR")
        self._btn_abort.setStyleSheet(
            "QPushButton{background:#dc2626;color:white;font-size:13px;font-weight:700;"
            "border-radius:6px;padding:8px;}"
        )
        self._btn_abort.clicked.connect(self._abort)

        btn_row.addWidget(self._btn_directo)
        btn_row.addWidget(self._btn_moveit)
        btn_row.addWidget(self._btn_abort)
        auto_lay.addLayout(btn_row)
        layout.addWidget(self._auto_group)

        # ── STEP controls (hidden initially) ─────────────────────────────────
        self._step_group = QGroupBox("Control STEP by STEP")
        step_lay = QVBoxLayout(self._step_group)
        btn_open = QPushButton("Abrir ventana STEP by STEP")
        btn_open.setStyleSheet(
            "QPushButton{background:#0f766e;color:white;font-size:13px;font-weight:700;"
            "border-radius:6px;padding:10px;}"
        )
        btn_open.clicked.connect(self._open_step_dialog)
        step_lay.addWidget(btn_open)
        self._step_group.hide()
        layout.addWidget(self._step_group)

        # ── Log ──────────────────────────────────────────────────────────────
        log_box = QGroupBox("Log")
        log_lay = QVBoxLayout(log_box)
        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumHeight(160)
        log_lay.addWidget(self._log_text)
        layout.addWidget(log_box)

    # ── Mode / flow ───────────────────────────────────────────────────────────

    def _on_mode_changed(self, mode: str) -> None:
        if mode == "AUTO":
            self._auto_group.show()
            self._step_group.hide()
        else:
            self._auto_group.hide()
            self._step_group.show()

    def _on_flow_changed(self, flow: str) -> None:
        self._flow = flow

    def _toggle_gripper(self) -> None:
        self._gripper_open = not self._gripper_open
        self._node.publish_gripper(self._gripper_open)
        label = "Pinza: Abrir" if self._gripper_open else "Pinza: Cerrar"
        self._btn_gripper.setText(label)

    # ── AUTO ─────────────────────────────────────────────────────────────────

    def _run_auto(self, flow: str) -> None:
        if self._auto_busy:
            self._append_log("[AUTO] Secuencia en curso. Aborta primero.")
            return
        self._auto_busy = True
        self._btn_directo.setEnabled(False)
        self._btn_moveit.setEnabled(False)
        self._btn_iniciar.setEnabled(False)
        self._append_log(f"[AUTO] Iniciando secuencia {flow}…")

        def worker() -> None:
            try:
                phases = ["HOME", "APPROACH", "DOWN", "CLOSE", "LIFT"]
                for phase in phases:
                    self._node.send_command(flow, phase)
                    time.sleep(step_phase_delay(phase))
            finally:
                QTimer.singleShot(0, self._auto_done)

        threading.Thread(target=worker, daemon=True).start()

    def _auto_done(self) -> None:
        self._auto_busy = False
        self._btn_directo.setEnabled(True)
        self._btn_moveit.setEnabled(True)
        self._btn_iniciar.setEnabled(True)
        self._append_log("[AUTO] Secuencia completada.")

    def _abort(self) -> None:
        self._node.send_command(self._flow, "ABORT")
        self._auto_busy = False
        self._btn_directo.setEnabled(True)
        self._btn_moveit.setEnabled(True)
        self._btn_iniciar.setEnabled(True)
        self._append_log(f"[AUTO] ABORT enviado ({self._flow})")

    # ── Robot pose buttons ────────────────────────────────────────────────────

    def _go_pose(self, name: str) -> None:
        if self._baseline_busy:
            self._append_log(f"[POSE] Robot ocupado — espera a que termine el movimiento actual")
            return
        poses = {"HOME": JOINT_HOME_RAD, "MESA": JOINT_MESA_RAD, "CESTA": JOINT_CESTA_RAD}
        angles = poses.get(name)
        if angles is None:
            return
        self._baseline_busy = True
        self._btn_home_pose.setEnabled(False)
        self._btn_mesa_pose.setEnabled(False)
        self._btn_cesta_pose.setEnabled(False)
        self._append_log(f"[POSE] Moviendo a {name}…")

        def _done_gui() -> None:
            self._append_log(f"[POSE] {name} ejecutado")
            self._pose_done()

        def worker() -> None:
            try:
                self._node.publish_joint_trajectory(angles, duration_sec=4.0)
                time.sleep(5.0)
            finally:
                QTimer.singleShot(0, _done_gui)

        threading.Thread(target=worker, daemon=True).start()

    def _pose_done(self) -> None:
        self._baseline_busy = False
        self._btn_home_pose.setEnabled(True)
        self._btn_mesa_pose.setEnabled(True)
        self._btn_cesta_pose.setEnabled(True)

    # ── STEP window ───────────────────────────────────────────────────────────

    def _open_step_dialog(self) -> None:
        flow = self._flow_combo.currentText()
        if self._step_dlg is None or not self._step_dlg.isVisible():
            self._step_dlg = StepByStepDialog(self._node, flow, self)
            self._step_dlg.show()
        else:
            self._step_dlg.raise_()

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_status(self, status: str) -> None:
        self._lbl_status.setText(f"Estado: {status}")

    def _append_log(self, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self._log_text.append(f"[{ts}] {text}")
        sb = self._log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _refresh_status_bar(self) -> None:
        tcp = self._node.get_tcp_world()
        obj = self._node.get_object_world()
        gripper = self._node.get_gripper_str()
        parts = []
        if tcp:
            parts.append(f"TCP {_fmt_xyz(tcp, prec=3)}")
        if obj:
            parts.append(f"Obj {_fmt_xyz(obj, prec=3)}")
        if gripper != "--":
            parts.append(f"Gripper {gripper}")
        if parts:
            self.statusBar().showMessage("  |  ".join(parts))


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main() -> None:
    if not _QT_OK:
        print("[autopick_panel] PyQt5 no disponible — saliendo.", file=sys.stderr)
        sys.exit(1)

    # QApplication MUST exist before any QObject (including ROS nodes with pyqtSignal).
    # Pass only argv[0] so Qt doesn't try to parse --ros-args from launch.
    app = QApplication(sys.argv[:1])
    app.setApplicationName("UR5 Visual Autopick")

    rclpy.init(args=sys.argv)
    node = AutopickNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    win = AutopickMainWindow(node)
    win.show()

    try:
        ret = app.exec_()
    except ExternalShutdownException:
        ret = 0
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(ret)


if __name__ == "__main__":
    main()
