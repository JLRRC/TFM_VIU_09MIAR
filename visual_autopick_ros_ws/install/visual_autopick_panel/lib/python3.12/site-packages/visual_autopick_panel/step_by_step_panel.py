#!/usr/bin/env python3
"""Panel STEP by STEP — standalone o embebido en autopick_panel.

Standalone:
  ros2 run visual_autopick_panel step_by_step_panel

Embebido:
  from visual_autopick_panel.step_by_step_panel import StepByStepDialog
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
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
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

# ──────────────────────────────────────────────────────────────────────────────
# Utility functions (shared — autopick_panel.py imports _fmt_xyz from here)
# ──────────────────────────────────────────────────────────────────────────────

def _dist3(
    a: Optional[Tuple[float, float, float]],
    b: Optional[Tuple[float, float, float]],
) -> str:
    if a is None or b is None:
        return "--"
    d = math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))
    return f"{d:.4f}"


def _fmt_xyz(p: Optional[Tuple[float, float, float]], prec: int = 4) -> str:
    if p is None:
        return "--"
    fmt = f"{{:+.{prec}f}}"
    return f"({fmt.format(p[0])}, {fmt.format(p[1])}, {fmt.format(p[2])})"


def _fmt_col(v: Optional[float]) -> str:
    return "--" if v is None else f"{v:+.4f}"


def _runtime_status_style(s: str) -> str:
    c = {"OK": "#16a34a", "WARN": "#ca8a04", "ERROR": "#dc2626"}.get(
        s.upper(), "#64748b"
    )
    return f"font-weight:700; color:{c}; font-size:12px;"


_BOX_STYLE = (
    "QGroupBox {{font-weight:700; color:#0f172a;"
    "border:1px solid #cbd5e1; border-radius:6px;"
    "margin-top:6px; padding:6px 6px 4px 6px; background:{bg};}}"
    "QGroupBox::title {{subcontrol-origin: margin; left:8px; padding:0 4px;}}"
)


def _build_runtime_block(
    key: str,
    title: str,
    parent_layout: "QVBoxLayout",
    blocks: Dict,
) -> None:
    block_box = QGroupBox(title)
    block_layout = QVBoxLayout(block_box)
    block_layout.setContentsMargins(4, 6, 4, 4)
    block_layout.setSpacing(4)

    status_lbl = QLabel("STALE")
    status_lbl.setStyleSheet(_runtime_status_style("STALE"))
    summary_lbl = QLabel("sin dato")
    summary_lbl.setWordWrap(True)
    summary_lbl.setStyleSheet("color:#475569; font-size:11px;")

    planned_lbl = QLabel("sin dato")
    planned_lbl.setWordWrap(True)
    planned_lbl.setAlignment(Qt.AlignTop | Qt.AlignLeft)
    planned_lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
    planned_lbl.setStyleSheet("color:#0f172a; font-size:11px;")

    runtime_lbl = QLabel("sin dato")
    runtime_lbl.setWordWrap(True)
    runtime_lbl.setAlignment(Qt.AlignTop | Qt.AlignLeft)
    runtime_lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
    runtime_lbl.setStyleSheet("color:#0f172a; font-size:11px;")

    meta_row = QHBoxLayout()
    meta_row.setSpacing(6)
    meta_row.addWidget(status_lbl, 0, Qt.AlignTop)
    meta_row.addWidget(summary_lbl, 1)
    block_layout.addLayout(meta_row)

    cols = QGridLayout()
    cols.setHorizontalSpacing(8)

    plan_box = QGroupBox("PLANIFICADO / PANEL")
    plan_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    plan_box.setStyleSheet(_BOX_STYLE.format(bg="#f8fafc"))
    pl = QVBoxLayout(plan_box)
    pl.setContentsMargins(6, 10, 6, 4)
    pl.addWidget(planned_lbl)

    rt_box = QGroupBox("MEDIDO / RUNTIME")
    rt_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    rt_box.setStyleSheet(_BOX_STYLE.format(bg="#ffffff"))
    rl = QVBoxLayout(rt_box)
    rl.setContentsMargins(6, 10, 6, 4)
    rl.addWidget(runtime_lbl)

    cols.addWidget(plan_box, 0, 0)
    cols.addWidget(rt_box, 0, 1)
    cols.setColumnStretch(0, 1)
    cols.setColumnStretch(1, 1)
    block_layout.addLayout(cols)

    parent_layout.addWidget(block_box)
    blocks[key] = {
        "status":  status_lbl,
        "summary": summary_lbl,
        "planned": planned_lbl,
        "runtime": runtime_lbl,
    }


# ──────────────────────────────────────────────────────────────────────────────
# Signals + ROS node (used when launched standalone)
# ──────────────────────────────────────────────────────────────────────────────

class _Signals(QObject if _QT_OK else object):  # type: ignore
    status_updated = pyqtSignal(str) if _QT_OK else None  # type: ignore
    joints_updated = pyqtSignal()    if _QT_OK else None  # type: ignore
    log_appended   = pyqtSignal(str) if _QT_OK else None  # type: ignore


def _make_arm_traj(angles: List[float], duration_sec: float) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = list(angles)
    pt.velocities = [0.0] * len(angles)
    pt.time_from_start.sec = int(duration_sec)
    pt.time_from_start.nanosec = int((duration_sec % 1.0) * 1e9)
    traj.points = [pt]
    return traj


class _StepNode(Node):
    """ROS 2 node for the standalone step_by_step_panel."""

    def __init__(self) -> None:
        super().__init__("step_by_step_panel")
        self.signals = _Signals()

        self._cmd_pub = self.create_publisher(String, "/visual_autopick/step_command", 10)
        self._arm_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
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

    def publish_joint_trajectory(self, angles: List[float], duration_sec: float = 4.0) -> None:
        self._arm_pub.publish(_make_arm_traj(angles, duration_sec))

    def send_command(self, mode: str, phase: str) -> None:
        msg = String()
        msg.data = f"{mode.upper()}:{phase.upper()}"
        self._cmd_pub.publish(msg)
        self.get_logger().info(f"[CMD] {msg.data}")
        if self.signals.log_appended:
            self.signals.log_appended.emit(f"[CMD] {msg.data}")

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

    def get_tcp_world(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self._tf_buf.lookup_transform(
                "world", "rg2_pinch_center", Time(), timeout=Duration(seconds=0.1)
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
                "base_link", "rg2_pinch_center", Time(), timeout=Duration(seconds=0.1)
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


# ──────────────────────────────────────────────────────────────────────────────
# StepByStepDialog — idéntico en estructura al panel_v2.py de agarre_ros2_ws
# ──────────────────────────────────────────────────────────────────────────────

class StepByStepDialog(QDialog if _QT_OK else object):  # type: ignore
    """Non-modal dialog idéntico en estructura al STEP de panel_v2.py.

    Acepta cualquier nodo que exponga la interfaz de _StepNode / AutopickNode.
    """

    def __init__(self, node, flow: str, parent=None) -> None:
        if not _QT_OK:
            return
        super().__init__(parent)
        self.setWindowTitle("STEP by STEP")
        self.setModal(False)
        self.setAttribute(Qt.WA_DeleteOnClose, False)
        self.resize(980, 760)

        self._node = node
        self._flow = flow.upper()
        self._seq = step_phase_sequence(self._flow)
        self._cur_idx = 0
        self._selected_phase: Optional[str] = None
        self._phase_buttons: List[QPushButton] = []
        self._history_rows: List[Dict] = []
        self._phase_start_tcp: Optional[Tuple[float, float, float]] = None

        self._build_ui()
        self._refresh_pipeline_table()
        self._refresh_info_labels()

        node.signals.status_updated.connect(self._on_status)
        node.signals.log_appended.connect(self._append_log)

        self._live_timer = QTimer(self)
        self._live_timer.setInterval(500)
        self._live_timer.timeout.connect(self._refresh_live)
        self._live_timer.start()

    # ── UI build ─────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(_QSA.NoFrame)
        content = QWidget()
        root.addWidget(scroll)
        scroll.setWidget(content)

        layout = QVBoxLayout(content)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        lbl_title = QLabel("Control paso a paso")
        lbl_title.setStyleSheet("font-weight:700;")
        layout.addWidget(lbl_title)

        # ── 13 info labels in 2-column grid ──────────────────────────────────
        self._lbl_mode              = QLabel("Flujo cargado: --")
        self._lbl_phase             = QLabel("Fase lista para iniciar: --")
        self._lbl_current           = QLabel("Fase en ejecución: --")
        self._lbl_next              = QLabel("Próxima fase bloqueada: --")
        self._lbl_intent            = QLabel("Objetivo de la fase: --")
        self._lbl_decision          = QLabel("Acción exacta al pulsar Iniciar: --")
        self._lbl_target            = QLabel("XYZ objetivo de la fase (world): --")
        self._lbl_live_operational  = QLabel("XYZ actual del TCP (world): --")
        self._lbl_live_visual       = QLabel("XYZ de referencia visual (world): --")
        self._lbl_gripper_expected  = QLabel("Pinza esperada en la fase seleccionada: --")
        self._lbl_gripper_live      = QLabel("Pinza live: --")
        self._lbl_object            = QLabel("Objeto activo (world): --")
        self._lbl_start_pose        = QLabel("Pose inicial del robot al lanzar la secuencia: --")

        all_info = [
            self._lbl_mode, self._lbl_phase, self._lbl_current,
            self._lbl_next, self._lbl_intent, self._lbl_decision,
            self._lbl_target, self._lbl_live_operational,
            self._lbl_live_visual, self._lbl_gripper_expected,
            self._lbl_gripper_live, self._lbl_object, self._lbl_start_pose,
        ]
        for lbl in all_info:
            lbl.setWordWrap(True)

        info_grid = QGridLayout()
        info_grid.setHorizontalSpacing(20)
        info_grid.setVerticalSpacing(6)
        left_lbls = [
            self._lbl_mode, self._lbl_phase, self._lbl_current,
            self._lbl_next, self._lbl_intent, self._lbl_decision,
            self._lbl_target,
        ]
        right_lbls = [
            self._lbl_live_operational, self._lbl_live_visual,
            self._lbl_gripper_expected, self._lbl_gripper_live,
            self._lbl_object, self._lbl_start_pose,
        ]
        for row, lbl in enumerate(left_lbls):
            info_grid.addWidget(lbl, row, 0)
        for row, lbl in enumerate(right_lbls):
            info_grid.addWidget(lbl, row, 1)
        info_grid.setColumnStretch(0, 1)
        info_grid.setColumnStretch(1, 1)
        layout.addLayout(info_grid)

        # ── Runtime verification — 3 blocks ──────────────────────────────────
        lbl_rt_help = QLabel(
            "Izquierda: PLANIFICADO / PANEL | Derecha: MEDIDO / RUNTIME. "
            "Fuentes: [PANEL] [TF] [JOINTS] [GAZEBO] [URDF/SDF]."
        )
        lbl_rt_help.setWordWrap(True)
        lbl_rt_help.setStyleSheet("color:#475569; font-size:12px;")

        self._runtime_blocks: Dict[str, Dict[str, QLabel]] = {}
        rt_group = QGroupBox("Verificacion runtime Gazebo/TF")
        rt_group.setStyleSheet("QGroupBox { font-weight: 700; }")
        rt_layout = QVBoxLayout(rt_group)
        rt_layout.setContentsMargins(4, 4, 4, 4)
        rt_layout.setSpacing(4)
        rt_layout.addWidget(lbl_rt_help)

        _build_runtime_block("dh_tf",       "BLOQUE 1 - DH / TF",         rt_layout, self._runtime_blocks)
        _build_runtime_block("joints_ctrl", "BLOQUE 2 - JOINTS / CONTROL", rt_layout, self._runtime_blocks)
        _build_runtime_block("sdf_gazebo",  "BLOQUE 3 - SDF / GAZEBO",     rt_layout, self._runtime_blocks)
        layout.addWidget(rt_group)

        # ── Pipeline table (5 cols) ───────────────────────────────────────────
        lbl_pipe_title = QLabel("Pipeline completo")
        lbl_pipe_title.setStyleSheet("font-weight:700;")
        layout.addWidget(lbl_pipe_title)

        self._pipe_table = QTableWidget(0, 5)
        self._pipe_table.setHorizontalHeaderLabels([
            "Iniciar", "Fase", "Qué hará al iniciar", "Pinza esperada", "Estado",
        ])
        self._pipe_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._pipe_table.setSelectionMode(QTableWidget.NoSelection)
        self._pipe_table.setSizeAdjustPolicy(_QSA.AdjustToContents)
        ph = self._pipe_table.horizontalHeader()
        ph.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        ph.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        ph.setSectionResizeMode(2, QHeaderView.Stretch)
        ph.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        ph.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self._pipe_table.verticalHeader().setVisible(False)
        layout.addWidget(self._pipe_table)

        lbl_pipe_help = QLabel(
            "Solo se habilita el botón Iniciar de la fase que toca ejecutar; "
            "las demás permanecen bloqueadas."
        )
        lbl_pipe_help.setWordWrap(True)
        lbl_pipe_help.setStyleSheet("color:#475569; font-size:12px;")
        layout.addWidget(lbl_pipe_help)

        # ── History table (23 cols) ───────────────────────────────────────────
        lbl_hist_title = QLabel("Histórico ejecutado")
        lbl_hist_title.setStyleSheet("font-weight:700;")
        layout.addWidget(lbl_hist_title)

        lbl_hist_help = QLabel(
            "Tabla STEP: Org=pose al abrir la fase | TCP-TF=TCP real por TF al cerrar | "
            "Obj World=pose objeto en world | Target=destino planificado | Exec=target enviado | "
            "D TCP-Obj / D Target-Obj / Err TCP-Exec en metros | Tipo Target explicita la semantica."
        )
        lbl_hist_help.setWordWrap(True)
        lbl_hist_help.setStyleSheet("color:#475569; font-size:12px;")
        layout.addWidget(lbl_hist_help)

        self._hist_table = QTableWidget(0, 23)
        self._hist_table.setHorizontalHeaderLabels([
            "Fase", "Pinza",
            "Xw Org",    "Yw Org",    "Zw Org",
            "Xw TCP-TF", "Yw TCP-TF", "Zw TCP-TF",
            "Xw Obj",    "Yw Obj",    "Zw Obj",
            "Xw Target", "Yw Target", "Zw Target",
            "Xw Exec",   "Yw Exec",   "Zw Exec",
            "D TCP-Obj", "D Target-Obj", "Err TCP-Exec",
            "Tipo Target", "Razón", "Estado",
        ])
        self._hist_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._hist_table.setSelectionMode(QTableWidget.NoSelection)
        self._hist_table.setSizeAdjustPolicy(_QSA.AdjustToContents)
        hh = self._hist_table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.Stretch)
        hh.setSectionResizeMode(21, QHeaderView.Stretch)
        for col in range(1, 23):
            if col not in (0, 21):
                hh.setSectionResizeMode(col, QHeaderView.ResizeToContents)
        self._hist_table.verticalHeader().setVisible(False)
        layout.addWidget(self._hist_table, 1)

        # ── Log ──────────────────────────────────────────────────────────────
        lbl_log = QLabel("Log")
        lbl_log.setStyleSheet("font-weight:700;")
        layout.addWidget(lbl_log)
        self._log_box = QTextEdit()
        self._log_box.setReadOnly(True)
        self._log_box.setMaximumHeight(120)
        layout.addWidget(self._log_box)

        # ── Cartesian debugger (stub) ─────────────────────────────────────────
        bottom = QHBoxLayout()
        self._chk_cart = QCheckBox("Depuración cartesiana")
        self._chk_cart.setChecked(False)
        self._btn_cart = QPushButton("Depurador cartesiano")
        self._btn_cart.setEnabled(False)
        self._btn_cart.setToolTip(
            "Abrir ventana auxiliar para mover rg2_pinch_center en XYZ. "
            "(No disponible en este workspace simplificado.)"
        )
        self._chk_cart.toggled.connect(self._btn_cart.setEnabled)
        bottom.addWidget(self._chk_cart)
        bottom.addWidget(self._btn_cart)
        layout.addLayout(bottom)

    # ── Pipeline table rendering ──────────────────────────────────────────────

    def _refresh_pipeline_table(self) -> None:
        self._pipe_table.setRowCount(0)
        self._phase_buttons.clear()

        for i, phase in enumerate(self._seq):
            row = self._pipe_table.rowCount()
            self._pipe_table.insertRow(row)

            is_current = (i == self._cur_idx)
            is_done    = (i < self._cur_idx)

            btn = QPushButton("Iniciar")
            btn.setEnabled(is_current)
            btn.setStyleSheet(
                "QPushButton{background:#16a34a;color:white;font-weight:700;border-radius:4px;}"
                "QPushButton:disabled{background:#94a3b8;color:#e2e8f0;}"
            )
            captured_phase = phase
            btn.clicked.connect(lambda _=False, p=captured_phase: self._execute_phase(p))
            self._pipe_table.setCellWidget(row, 0, btn)
            self._phase_buttons.append(btn)

            self._pipe_table.setItem(row, 1, QTableWidgetItem(phase))
            self._pipe_table.setItem(
                row, 2, QTableWidgetItem(step_phase_action_text(self._flow, phase))
            )
            self._pipe_table.setItem(
                row, 3, QTableWidgetItem(step_phase_gripper_state(self._flow, phase))
            )

            if is_done:
                state_text, state_color = "OK", "#16a34a"
            elif is_current:
                state_text, state_color = "LISTO", "#0284c7"
            else:
                state_text, state_color = "BLOQUEADO", "#94a3b8"
            item = QTableWidgetItem(state_text)
            item.setForeground(__import__("PyQt5.QtGui", fromlist=["QColor"]).QColor(state_color))
            self._pipe_table.setItem(row, 4, item)

    # ── Info labels refresh ───────────────────────────────────────────────────

    def _refresh_info_labels(self) -> None:
        self._lbl_mode.setText(f"Flujo cargado: {step_present_flow_name(self._flow)}")
        if 0 <= self._cur_idx < len(self._seq):
            phase = self._seq[self._cur_idx]
            self._lbl_phase.setText(f"Fase lista para iniciar: {phase}")
            self._lbl_next.setText(
                f"Próxima fase bloqueada: {step_predict_next_phase(self._flow, phase)}"
            )
            self._lbl_intent.setText(f"Objetivo de la fase: {step_phase_intent(self._flow, phase)}")
            self._lbl_decision.setText(
                f"Acción exacta al pulsar Iniciar: {step_phase_action_text(self._flow, phase)}"
            )
            target = step_known_tcp_world(phase)
            self._lbl_target.setText(f"XYZ objetivo de la fase (world): {_fmt_xyz(target)}")
            self._lbl_gripper_expected.setText(
                f"Pinza esperada en la fase seleccionada: "
                f"{step_phase_gripper_state(self._flow, phase)}"
            )
        elif self._cur_idx >= len(self._seq):
            self._lbl_phase.setText("Fase lista para iniciar: -- (SECUENCIA COMPLETA)")
        else:
            self._lbl_phase.setText("Fase lista para iniciar: --")

    # ── Live data refresh (500 ms timer) ─────────────────────────────────────

    def _refresh_live(self) -> None:
        tcp = self._node.get_tcp_world()
        self._lbl_live_operational.setText(f"XYZ actual del TCP (world): {_fmt_xyz(tcp)}")

        tool0 = self._node.get_tool0_world()
        self._lbl_live_visual.setText(f"XYZ de referencia visual (world): {_fmt_xyz(tool0)}")

        self._lbl_gripper_live.setText(f"Pinza live: {self._node.get_gripper_str()}")

        obj = self._node.get_object_world()
        self._lbl_object.setText(f"Objeto activo (world): pick_demo @ {_fmt_xyz(obj)}")

        self._lbl_start_pose.setText(
            f"Pose inicial del robot al lanzar la secuencia: "
            f"{_fmt_xyz(self._phase_start_tcp) if self._phase_start_tcp else '--'}"
        )

        self._refresh_runtime_blocks(tcp, tool0, obj)

    def _refresh_runtime_blocks(
        self,
        tcp: Optional[Tuple],
        tool0: Optional[Tuple],
        obj: Optional[Tuple],
    ) -> None:
        # BLOQUE 1 — DH / TF
        blk = self._runtime_blocks["dh_tf"]
        tcp_bl = self._node.get_tcp_base()
        if tcp is not None and tcp_bl is not None:
            blk["status"].setText("OK")
            blk["status"].setStyleSheet(_runtime_status_style("OK"))
            blk["summary"].setText(
                f"rg2_pinch_center@world={_fmt_xyz(tcp)}  |  @base_link={_fmt_xyz(tcp_bl)}"
            )
        else:
            blk["status"].setText("STALE")
            blk["status"].setStyleSheet(_runtime_status_style("STALE"))
            blk["summary"].setText("TF no disponible — ¿robot_state_publisher activo?")

        if 0 <= self._cur_idx < len(self._seq):
            phase = self._seq[self._cur_idx]
            target = step_known_tcp_world(phase)
            blk["planned"].setText(
                f"Fase: {phase}\n"
                f"TCP target (world): {_fmt_xyz(target)}\n"
                f"Offset canon. tool0→rg2_pinch_center: (0, 0, +0.175)"
            )
        else:
            blk["planned"].setText("Secuencia completada o no iniciada.")

        blk["runtime"].setText(
            f"TCP world:      {_fmt_xyz(tcp)}\n"
            f"tool0 world:    {_fmt_xyz(tool0)}\n"
            f"TCP base_link:  {_fmt_xyz(tcp_bl)}"
        )

        # BLOQUE 2 — JOINTS / CONTROL
        blk = self._runtime_blocks["joints_ctrl"]
        arm_str = self._node.get_arm_joints_str()
        gripper_str = self._node.get_gripper_str()
        if "--" not in arm_str or any(j in arm_str for j in ("pan", "lift", "elbow")):
            blk["status"].setText("OK")
            blk["status"].setStyleSheet(_runtime_status_style("OK"))
            blk["summary"].setText("6 joints arm + 2 gripper recibidos")
        else:
            blk["status"].setText("STALE")
            blk["status"].setStyleSheet(_runtime_status_style("STALE"))
            blk["summary"].setText("Sin /joint_states — ¿joint_state_broadcaster activo?")

        if 0 <= self._cur_idx < len(self._seq):
            phase = self._seq[self._cur_idx]
            gs = step_phase_gripper_state(self._flow, phase)
            blk["planned"].setText(
                f"Fase: {phase}\n"
                f"Pinza esperada: {gs}\n"
                f"Controladores: joint_trajectory_controller, gripper_controller"
            )
        else:
            blk["planned"].setText("Secuencia completada o no iniciada.")

        blk["runtime"].setText(f"Arm: {arm_str}\nGripper: {gripper_str}")

        # BLOQUE 3 — SDF / GAZEBO
        blk = self._runtime_blocks["sdf_gazebo"]
        base_pos = self._node.get_base_link_world()
        if obj is not None or base_pos is not None:
            blk["status"].setText("OK")
            blk["status"].setStyleSheet(_runtime_status_style("OK"))
            blk["summary"].setText("Gazebo pose/info recibido correctamente.")
        else:
            blk["status"].setText("STALE")
            blk["status"].setStyleSheet(_runtime_status_style("STALE"))
            blk["summary"].setText(
                "Sin datos Gazebo — ¿gz_bridge activo con /world/visual_autopick_world/pose/info?"
            )

        blk["planned"].setText(
            "base_link esperado en world: (-0.85, 0.0, 0.850)\n"
            "pick_demo esperado en world:  (-0.42, 0.0, 0.876)\n"
            "TCP_Z_OFFSET canónico (URDF): 0.175 m"
        )
        blk["runtime"].setText(
            f"base_link@world:   {_fmt_xyz(base_pos)}\n"
            f"pick_demo@world:   {_fmt_xyz(obj)}\n"
            f"D TCP→objeto:      {_dist3(tcp, obj)}"
        )

    # ── Phase execution ───────────────────────────────────────────────────────

    def _execute_phase(self, phase: str) -> None:
        self._phase_start_tcp = self._node.get_tcp_world() or self._phase_start_tcp

        row_data: Dict = {
            "phase":    phase,
            "gripper":  step_phase_gripper_state(self._flow, phase),
            "org_tcp":  self._node.get_tcp_world(),
            "obj":      self._node.get_object_world(),
            "target":   step_known_tcp_world(phase),
            "exec_tcp": None,
            "reason":   step_phase_intent(self._flow, phase),
            "state":    "EXECUTING",
        }

        self._node.send_command(self._flow, phase)
        self._lbl_current.setText(f"Fase en ejecución: {phase}")
        self._append_log(f"[STEP] Iniciando fase {self._flow}:{phase}")

        self._history_rows.append(row_data)
        self._add_history_table_row(row_data, provisional=True)

        delay = step_phase_delay(phase)

        def _advance():
            time.sleep(delay)
            row_data["exec_tcp"] = self._node.get_tcp_world()
            row_data["state"] = "OK"
            QTimer.singleShot(0, lambda: self._on_phase_done(phase, row_data))

        threading.Thread(target=_advance, daemon=True).start()

    def _on_phase_done(self, phase: str, row_data: Dict) -> None:
        self._update_last_history_row(row_data)
        if self._cur_idx < len(self._seq):
            self._cur_idx += 1
        self._refresh_pipeline_table()
        self._refresh_info_labels()
        self._lbl_current.setText(f"Fase en ejecución: DONE ({phase})")
        self._append_log(f"[STEP] Fase {self._flow}:{phase} completada")

    # ── History table ─────────────────────────────────────────────────────────

    def _add_history_table_row(self, d: Dict, provisional: bool = False) -> None:
        row = self._hist_table.rowCount()
        self._hist_table.insertRow(row)
        self._fill_history_row(row, d, provisional)

    def _update_last_history_row(self, d: Dict) -> None:
        row = self._hist_table.rowCount() - 1
        if row >= 0:
            self._fill_history_row(row, d, provisional=False)

    def _fill_history_row(self, row: int, d: Dict, provisional: bool) -> None:
        org = d.get("org_tcp")
        end = d.get("exec_tcp")
        obj = d.get("obj")
        tgt = d.get("target")

        cells = [
            d.get("phase", "--"),
            d.get("gripper", "--"),
            _fmt_col(org[0] if org else None),
            _fmt_col(org[1] if org else None),
            _fmt_col(org[2] if org else None),
            _fmt_col(end[0] if end else None),
            _fmt_col(end[1] if end else None),
            _fmt_col(end[2] if end else None),
            _fmt_col(obj[0] if obj else None),
            _fmt_col(obj[1] if obj else None),
            _fmt_col(obj[2] if obj else None),
            _fmt_col(tgt[0] if tgt else None),
            _fmt_col(tgt[1] if tgt else None),
            _fmt_col(tgt[2] if tgt else None),
            _fmt_col(tgt[0] if tgt else None),
            _fmt_col(tgt[1] if tgt else None),
            _fmt_col(tgt[2] if tgt else None),
            _dist3(end, obj),
            _dist3(tgt, obj),
            _dist3(end, tgt),
            "JOINT_PRESET",
            d.get("reason", "--"),
            "EJECUTANDO" if provisional else d.get("state", "--"),
        ]
        for col, text in enumerate(cells):
            item = QTableWidgetItem(str(text))
            if provisional and col == 22:
                from PyQt5.QtGui import QColor
                item.setForeground(QColor("#ca8a04"))
            elif not provisional and col == 22 and d.get("state") == "OK":
                from PyQt5.QtGui import QColor
                item.setForeground(QColor("#16a34a"))
            self._hist_table.setItem(row, col, item)

    # ── Status / log ─────────────────────────────────────────────────────────

    def _on_status(self, status: str) -> None:
        self._lbl_current.setText(f"Fase en ejecución: {status}")

    def _append_log(self, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self._log_box.append(f"[{ts}] {text}")
        sb = self._log_box.verticalScrollBar()
        sb.setValue(sb.maximum())


# ──────────────────────────────────────────────────────────────────────────────
# Standalone window wrapper
# ──────────────────────────────────────────────────────────────────────────────

class _StepMainWindow(QMainWindow if _QT_OK else object):  # type: ignore
    """Main window for standalone step_by_step_panel launch."""

    def __init__(self, node: _StepNode) -> None:
        if not _QT_OK:
            return
        super().__init__()
        self._node = node
        self._flow = "DIRECTO"
        self._baseline_busy = False
        self.setWindowTitle("UR5 — STEP by STEP")
        self.resize(1100, 860)
        self._build_ui()
        node.signals.status_updated.connect(self._on_status)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        main_lay = QHBoxLayout(central)
        main_lay.setContentsMargins(8, 8, 8, 8)
        main_lay.setSpacing(8)

        # ── Left sidebar: camera + flow combo + robot pose buttons ────────────
        left = QVBoxLayout()
        left.setSpacing(6)

        cam_box = QGroupBox("Cámara overhead")
        cam_inner = QVBoxLayout(cam_box)
        self._cam = CameraWidget(self._node, topic="/camera_overhead/image")
        cam_inner.addWidget(self._cam, alignment=Qt.AlignCenter)
        left.addWidget(cam_box)

        # Flow selector
        flow_row = QHBoxLayout()
        flow_row.addWidget(QLabel("Flujo:"))
        self._flow_combo = QComboBox()
        self._flow_combo.addItems(["DIRECTO", "MOVEIT"])
        self._flow_combo.currentTextChanged.connect(self._on_flow_changed)
        flow_row.addWidget(self._flow_combo)
        flow_row.addStretch()
        left.addLayout(flow_row)

        # Status
        self._lbl_status = QLabel("Estado: --")
        self._lbl_status.setStyleSheet("font-size:13px;")
        left.addWidget(self._lbl_status)

        # Robot pose buttons
        _btn_style = (
            "QPushButton{{background:{bg};color:white;font-size:13px;font-weight:700;"
            "border-radius:5px;padding:7px 14px;}}"
            "QPushButton:disabled{{background:#94a3b8;}}"
        )
        pose_box = QGroupBox("Posiciones robot")
        pose_lay = QVBoxLayout(pose_box)
        self._btn_home = QPushButton("UR5 → HOME")
        self._btn_home.setStyleSheet(_btn_style.format(bg="#0369a1"))
        self._btn_home.clicked.connect(lambda: self._go_pose("HOME"))

        self._btn_mesa = QPushButton("UR5 → Mesa")
        self._btn_mesa.setStyleSheet(_btn_style.format(bg="#047857"))
        self._btn_mesa.clicked.connect(lambda: self._go_pose("MESA"))

        self._btn_cesta = QPushButton("UR5 → Cesta")
        self._btn_cesta.setStyleSheet(_btn_style.format(bg="#b45309"))
        self._btn_cesta.clicked.connect(lambda: self._go_pose("CESTA"))

        for btn in (self._btn_home, self._btn_mesa, self._btn_cesta):
            pose_lay.addWidget(btn)
        left.addWidget(pose_box)
        left.addStretch()

        main_lay.addLayout(left, 1)

        # ── Right: step panel dialog embedded as widget ───────────────────────
        self._step_dlg = StepByStepDialog(self._node, self._flow, self)
        main_lay.addWidget(self._step_dlg, 3)

    def _on_flow_changed(self, flow: str) -> None:
        self._flow = flow

    def _on_status(self, status: str) -> None:
        self._lbl_status.setText(f"Estado: {status}")

    def _go_pose(self, name: str) -> None:
        if self._baseline_busy:
            return
        poses = {"HOME": JOINT_HOME_RAD, "MESA": JOINT_MESA_RAD, "CESTA": JOINT_CESTA_RAD}
        angles = poses.get(name)
        if angles is None:
            return
        self._baseline_busy = True
        self._btn_home.setEnabled(False)
        self._btn_mesa.setEnabled(False)
        self._btn_cesta.setEnabled(False)

        def worker() -> None:
            try:
                self._node.publish_joint_trajectory(angles, duration_sec=4.0)
                time.sleep(5.0)
            finally:
                QTimer.singleShot(0, self._pose_done)

        threading.Thread(target=worker, daemon=True).start()

    def _pose_done(self) -> None:
        self._baseline_busy = False
        self._btn_home.setEnabled(True)
        self._btn_mesa.setEnabled(True)
        self._btn_cesta.setEnabled(True)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point — standalone launch
# ──────────────────────────────────────────────────────────────────────────────

def main() -> None:
    if not _QT_OK:
        print("[step_by_step_panel] PyQt5 no disponible — saliendo.", file=sys.stderr)
        sys.exit(1)

    app = QApplication(sys.argv[:1])
    app.setApplicationName("UR5 STEP by STEP")

    rclpy.init(args=sys.argv)
    node = _StepNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    win = _StepMainWindow(node)
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
