#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_trace_ui.py
# Contenido: Constructor Qt del grupo trace/diagnóstico del panel.
# Uso breve: Importado por panel_v2.py; la función recibe el panel como argumento.
"""Builder functions for trace/diagnostic and science QGroupBoxes."""
from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractScrollArea,
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QVBoxLayout,
    QWidget,
)


def build_trace_group(panel) -> QGroupBox:
    """Build the trace/diagnostic QGroupBox and attach widget refs as panel attributes."""
    trace_group = QGroupBox("")
    trace_group.setStyleSheet(
        "QGroupBox {"
        "font-size:11px; font-weight:700;"
        "background:#f8fafc;"
        "border:1px solid #cbd5f5;"
        "border-radius:8px;"
        "margin-top:6px;"
        "padding:8px;"
        "}"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; top: -4px; padding:0 4px; }"
        "QLabel { font-size:10px; color:#0f172a; }"
    )
    trace_layout = QVBoxLayout()
    trace_layout.setContentsMargins(6, 4, 6, 4)
    trace_layout.setSpacing(2)

    panel.lbl_trace_frames = QLabel("Frames: world=..., base=..., ee=...")
    panel.lbl_trace_frames.setStyleSheet("font-size: 11px; font-weight: 600;")
    trace_layout.addWidget(panel.lbl_trace_frames)

    panel.trace_table = QTableWidget(2, 8)
    panel.trace_table.setHorizontalHeaderLabels(["frame", "x", "y", "z", "qx", "qy", "qz", "qw"])
    panel.trace_table.setVerticalHeaderLabels(["Object", "TCP"])
    panel.trace_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    panel.trace_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
    panel.trace_table.setEditTriggers(QTableWidget.NoEditTriggers)
    panel.trace_table.setSelectionMode(QTableWidget.NoSelection)
    panel.trace_table.setStyleSheet(
        "font-size:10px;"
        "background:#ffffff;"
        "border:none;"
        "gridline-color:#e2e8f0;"
    )
    panel.trace_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
    panel.trace_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    panel.trace_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    panel.trace_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    panel.trace_table.setFixedHeight(120)
    panel.trace_table.setShowGrid(True)
    panel.trace_table.setFrameStyle(QTableWidget.Box | QTableWidget.Plain)
    trace_layout.addWidget(panel.trace_table)

    error_block = QWidget()
    error_layout = QGridLayout()
    error_layout.setContentsMargins(0, 0, 0, 0)
    error_layout.addWidget(QLabel("<b>Base</b> (m):"), 0, 0)
    panel.lbl_trace_error_base = QLabel("--")
    error_layout.addWidget(panel.lbl_trace_error_base, 0, 1)
    error_layout.addWidget(QLabel("<b>World</b> (m):"), 1, 0)
    panel.lbl_trace_error_world = QLabel("--")
    error_layout.addWidget(panel.lbl_trace_error_world, 1, 1)
    error_block.setLayout(error_layout)
    trace_layout.addWidget(error_block)

    tf_block = QWidget()
    tf_layout = QGridLayout()
    tf_layout.setContentsMargins(0, 0, 0, 0)
    tf_layout.addWidget(QLabel("translation:"), 0, 0)
    panel.lbl_trace_tf_translation = QLabel("--")
    tf_layout.addWidget(panel.lbl_trace_tf_translation, 0, 1)
    tf_layout.addWidget(QLabel("yaw:"), 1, 0)
    panel.lbl_trace_tf_yaw = QLabel("--")
    tf_layout.addWidget(panel.lbl_trace_tf_yaw, 1, 1)
    tf_block.setLayout(tf_layout)
    trace_layout.addWidget(tf_block)

    control_row = QHBoxLayout()
    panel.chk_trace_freeze = QCheckBox("Freeze")
    control_row.addWidget(panel.chk_trace_freeze)
    control_row.addStretch(1)
    panel.btn_trace_diag = QPushButton("TRACE DIAG ONCE")
    panel.btn_trace_diag.clicked.connect(panel._run_trace_diag_once)
    panel.btn_trace_diag.setStyleSheet("font-size:9px; padding:2px 8px;")
    control_row.addWidget(panel.btn_trace_diag)
    panel.btn_self_check = QPushButton("SELF CHECK")
    panel.btn_self_check.clicked.connect(panel._run_self_check_once)
    panel.btn_self_check.setStyleSheet("font-size:9px; padding:2px 8px;")
    control_row.addWidget(panel.btn_self_check)
    panel.btn_copy_trace = QPushButton("Copy trace")
    panel.btn_copy_trace.clicked.connect(panel._copy_trace_text)
    panel.btn_copy_trace.setStyleSheet("font-size:9px; padding:2px 8px;")
    control_row.addWidget(panel.btn_copy_trace)
    trace_layout.addLayout(control_row)

    trace_group.setLayout(trace_layout)
    return trace_group

def build_science_group(panel) -> QGroupBox:
    science_group = QGroupBox("")
    science_group.setFlat(True)
    science_group.setStyleSheet("")
    science_layout = QVBoxLayout()
    science_layout.setContentsMargins(6, 6, 6, 6)
    science_layout.setSpacing(4)

    block_css = (
        "QGroupBox {"
        "font-size:11px; font-weight:700;"
        "background:#f8fafc;"
        "border:1px solid #cbd5f5;"
        "border-radius:8px;"
        "margin-top:6px;"
        "padding:8px;"
        "}"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; top: -4px; padding:0 4px; }"
        "QLabel { font-size:10px; }"
    )

    cornell_group = QGroupBox("Evaluación Cornell (online - simulación)")
    cornell_group.setFlat(True)
    cornell_group.setStyleSheet(block_css)
    cornell_layout = QGridLayout()
    cornell_layout.setContentsMargins(2, 2, 2, 2)
    cornell_layout.setHorizontalSpacing(6)
    cornell_layout.setVerticalSpacing(3)
    cornell_layout.addWidget(QLabel("IoU"), 0, 0)
    panel.lbl_cornell_iou = QLabel("--")
    cornell_layout.addWidget(panel.lbl_cornell_iou, 0, 1)
    cornell_layout.addWidget(QLabel("dTheta"), 0, 2)
    panel.lbl_cornell_theta = QLabel("--")
    cornell_layout.addWidget(panel.lbl_cornell_theta, 0, 3)
    cornell_layout.addWidget(QLabel("Success"), 1, 0)
    panel.lbl_cornell_success = QLabel("--")
    cornell_layout.addWidget(panel.lbl_cornell_success, 1, 1)
    panel.lbl_cornell_note = QLabel("Evaluación geométrica en simulación. No validación física.")
    panel.lbl_cornell_note.setStyleSheet("color:#64748b; font-size:9px;")
    cornell_layout.addWidget(panel.lbl_cornell_note, 2, 0, 1, 4)
    cornell_group.setLayout(cornell_layout)
    science_layout.addWidget(cornell_group)

    exp_group = QGroupBox("Configuración experimental activa")
    exp_group.setFlat(True)
    exp_group.setStyleSheet(block_css)
    exp_layout = QGridLayout()
    exp_layout.setContentsMargins(2, 2, 2, 2)
    exp_layout.setHorizontalSpacing(6)
    exp_layout.setVerticalSpacing(3)
    exp_layout.addWidget(QLabel("Modelo"), 0, 0)
    panel.lbl_exp_model = QLabel("--")
    exp_layout.addWidget(panel.lbl_exp_model, 0, 1)
    exp_layout.addWidget(QLabel("Modalidad"), 0, 2)
    panel.lbl_exp_modality = QLabel("--")
    exp_layout.addWidget(panel.lbl_exp_modality, 0, 3)
    exp_layout.addWidget(QLabel("Experimento"), 1, 0)
    panel.lbl_exp_name = QLabel("--")
    exp_layout.addWidget(panel.lbl_exp_name, 1, 1)
    exp_layout.addWidget(QLabel("Semilla"), 1, 2)
    panel.lbl_exp_seed = QLabel("--")
    exp_layout.addWidget(panel.lbl_exp_seed, 1, 3)
    exp_layout.addWidget(QLabel("Epoch"), 2, 0)
    panel.lbl_exp_epoch = QLabel("--")
    exp_layout.addWidget(panel.lbl_exp_epoch, 2, 1)
    exp_layout.addWidget(QLabel("Acierto"), 2, 2)
    panel.lbl_exp_success = QLabel("--")
    exp_layout.addWidget(panel.lbl_exp_success, 2, 3)
    exp_layout.addWidget(QLabel("IoU val"), 3, 0)
    panel.lbl_exp_iou = QLabel("--")
    exp_layout.addWidget(panel.lbl_exp_iou, 3, 1)
    exp_layout.addWidget(QLabel("Pesos"), 3, 2)
    panel.lbl_exp_weights = QLabel("--")
    panel.lbl_exp_weights.setWordWrap(True)
    exp_layout.addWidget(panel.lbl_exp_weights, 3, 3)
    exp_group.setLayout(exp_layout)
    science_layout.addWidget(exp_group)

    perf_group = QGroupBox("Rendimiento temporal")
    perf_group.setFlat(True)
    perf_group.setStyleSheet(block_css)
    perf_layout = QGridLayout()
    perf_layout.setContentsMargins(2, 2, 2, 2)
    perf_layout.setHorizontalSpacing(6)
    perf_layout.setVerticalSpacing(3)
    perf_layout.addWidget(QLabel("Latencia inferencia"), 0, 0)
    panel.lbl_perf_infer = QLabel("--")
    perf_layout.addWidget(panel.lbl_perf_infer, 0, 1)
    perf_layout.addWidget(QLabel("Latencia percepción"), 1, 0)
    panel.lbl_perf_total = QLabel("--")
    perf_layout.addWidget(panel.lbl_perf_total, 1, 1)
    perf_layout.addWidget(QLabel("FPS efectivo"), 2, 0)
    panel.lbl_perf_fps = QLabel("--")
    perf_layout.addWidget(panel.lbl_perf_fps, 2, 1)
    perf_group.setLayout(perf_layout)
    science_layout.addWidget(perf_group)

    grasp_group = QGroupBox("Parámetros del grasp")
    grasp_group.setFlat(True)
    grasp_group.setStyleSheet(block_css)
    grasp_layout = QGridLayout()
    grasp_layout.setContentsMargins(2, 2, 2, 2)
    grasp_layout.setHorizontalSpacing(6)
    grasp_layout.setVerticalSpacing(3)
    grasp_layout.addWidget(QLabel("Imagen (px)"), 0, 0)
    panel.lbl_grasp_img = QLabel("--")
    panel.lbl_grasp_img.setWordWrap(True)
    grasp_layout.addWidget(panel.lbl_grasp_img, 0, 1)
    grasp_layout.addWidget(QLabel("Mundo (m)"), 1, 0)
    panel.lbl_grasp_world = QLabel("--")
    panel.lbl_grasp_world.setWordWrap(True)
    grasp_layout.addWidget(panel.lbl_grasp_world, 1, 1)
    grasp_layout.addWidget(QLabel("Frame"), 2, 0)
    panel.lbl_grasp_frame = QLabel("--")
    grasp_layout.addWidget(panel.lbl_grasp_frame, 2, 1)
    grasp_group.setLayout(grasp_layout)
    science_layout.addWidget(grasp_group)

    panel.btn_save_episode = QPushButton("Guardar episodio experimental")
    panel.btn_save_episode.clicked.connect(panel._save_episode)
    science_layout.addWidget(panel.btn_save_episode)

    science_group.setLayout(science_layout)
    return science_group

