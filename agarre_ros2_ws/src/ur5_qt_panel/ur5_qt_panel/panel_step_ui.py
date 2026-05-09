#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_step_ui.py
# Contenido: Constructores Qt del diálogo STEP by STEP y del depurador cartesiano.
# Uso breve: Importado por panel_v2.py; las funciones reciben el panel como argumento.
"""Builder functions for STEP by STEP UI dialogs."""
from __future__ import annotations

from typing import Dict

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QAbstractScrollArea,
    QCheckBox,
    QComboBox,
    QDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTableWidget,
    QVBoxLayout,
    QWidget,
)

from .panel_runtime_pose_auditor import runtime_status_style


def _build_step_runtime_section(panel):
    """F3-step10a: runtime section (Gazebo/TF) extraída de build_step_window.

    Crea: lbl_runtime_title + lbl_runtime_help + runtime_group con 3 sub-bloques
    (BLOQUE 1 DH/TF + BLOQUE 2 JOINTS/CONTROL + BLOQUE 3 SDF/GAZEBO). Cada
    sub-bloque tiene status_lbl + summary_lbl + planned_lbl + runtime_lbl en
    columnas PLANIFICADO/PANEL vs MEDIDO/RUNTIME.

    Devuelve (lbl_runtime_title, lbl_runtime_help, runtime_group, runtime_blocks).
    """
    lbl_runtime_title = QLabel("Verificacion runtime Gazebo/TF")
    lbl_runtime_title.setStyleSheet("font-weight:700;")
    lbl_runtime_help = QLabel(
        "Izquierda: PLANIFICADO / PANEL | Derecha: MEDIDO / RUNTIME. "
        "Fuentes: [PANEL] [TF] [JOINTS] [GAZEBO] [URDF/SDF]."
    )
    lbl_runtime_help.setWordWrap(True)
    lbl_runtime_help.setStyleSheet("color:#475569; font-size:12px;")
    runtime_group = QGroupBox("Verificacion runtime Gazebo/TF")
    runtime_group.setStyleSheet("QGroupBox { font-weight: 700; }")
    runtime_group_layout = QVBoxLayout(runtime_group)
    runtime_group_layout.setContentsMargins(4, 4, 4, 4)
    runtime_group_layout.setSpacing(4)
    runtime_blocks: Dict[str, Dict[str, QLabel]] = {}

    def _add_runtime_block(block_key: str, title: str) -> None:
        block_box = QGroupBox(title)
        block_layout = QVBoxLayout(block_box)
        block_layout.setContentsMargins(4, 6, 4, 4)
        block_layout.setSpacing(4)
        status_lbl = QLabel("STALE")
        status_lbl.setStyleSheet(runtime_status_style("STALE"))
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

        columns_grid = QGridLayout()
        columns_grid.setHorizontalSpacing(8)
        columns_grid.setVerticalSpacing(0)

        plan_box = QGroupBox("PLANIFICADO / PANEL")
        plan_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        plan_box.setStyleSheet(
            "QGroupBox {"
            "font-weight:700; color:#0f172a;"
            "border:1px solid #cbd5e1; border-radius:6px;"
            "margin-top:6px; padding:6px 6px 4px 6px;"
            "background:#f8fafc;"
            "}"
            "QGroupBox::title {"
            "subcontrol-origin: margin; left:8px; padding:0 4px;"
            "}"
        )
        plan_layout = QVBoxLayout(plan_box)
        plan_layout.setContentsMargins(6, 10, 6, 4)
        plan_layout.setSpacing(2)
        plan_layout.addWidget(planned_lbl)

        runtime_box = QGroupBox("MEDIDO / RUNTIME")
        runtime_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        runtime_box.setStyleSheet(
            "QGroupBox {"
            "font-weight:700; color:#0f172a;"
            "border:1px solid #cbd5e1; border-radius:6px;"
            "margin-top:6px; padding:6px 6px 4px 6px;"
            "background:#ffffff;"
            "}"
            "QGroupBox::title {"
            "subcontrol-origin: margin; left:8px; padding:0 4px;"
            "}"
        )
        runtime_layout = QVBoxLayout(runtime_box)
        runtime_layout.setContentsMargins(6, 10, 6, 4)
        runtime_layout.setSpacing(2)
        runtime_layout.addWidget(runtime_lbl)

        columns_grid.addWidget(plan_box, 0, 0)
        columns_grid.addWidget(runtime_box, 0, 1)
        columns_grid.setColumnStretch(0, 1)
        columns_grid.setColumnStretch(1, 1)
        block_layout.addLayout(columns_grid)

        runtime_group_layout.addWidget(block_box)
        runtime_blocks[block_key] = {
            "status": status_lbl,
            "summary": summary_lbl,
            "planned": planned_lbl,
            "runtime": runtime_lbl,
        }

    _add_runtime_block("dh_tf", "BLOQUE 1 - DH / TF")
    _add_runtime_block("joints_control", "BLOQUE 2 - JOINTS / CONTROL")
    _add_runtime_block("sdf_gazebo", "BLOQUE 3 - SDF / GAZEBO")

    return lbl_runtime_title, lbl_runtime_help, runtime_group, runtime_blocks


def _build_step_pipeline_history_widgets(panel):
    """F3-step10b: pipeline_table + history_table + cart_debug widgets.

    Crea: lbl_pipeline_title + pipeline_table (5 cols) + lbl_pipeline_help +
    lbl_history_title + lbl_history_frame_help + history_table (23 cols) +
    btn_cart_debug + chk_cart_debug. Configura header resize policies +
    no-edit + no-selection + AdjustToContents.

    Devuelve dict con todos los widgets (caller los usa para layout y attrs).
    """
    lbl_pipeline_title = QLabel("Pipeline completo")
    lbl_pipeline_title.setStyleSheet("font-weight:700;")
    pipeline_table = QTableWidget(0, 5)
    pipeline_table.setHorizontalHeaderLabels([
        "Iniciar",
        "Fase",
        "Qué hará al iniciar",
        "Pinza esperada",
        "Estado",
    ])
    pipeline_table.setEditTriggers(QTableWidget.NoEditTriggers)
    pipeline_table.setSelectionMode(QTableWidget.NoSelection)
    pipeline_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
    pipeline_header = pipeline_table.horizontalHeader()
    if pipeline_header is not None:
        pipeline_header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        pipeline_header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        pipeline_header.setSectionResizeMode(2, QHeaderView.Stretch)
        pipeline_header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        pipeline_header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
    pipeline_table.verticalHeader().setVisible(False)
    lbl_pipeline_help = QLabel(
        "Aquí se muestra el pipeline completo del flujo cargado. Solo se habilita el botón "
        "Iniciar de la fase que toca ejecutar; las demás permanecen bloqueadas."
    )
    lbl_pipeline_help.setWordWrap(True)
    lbl_pipeline_help.setStyleSheet("color:#475569; font-size:12px;")
    lbl_history_title = QLabel("Histórico ejecutado")
    lbl_history_title.setStyleSheet("font-weight:700;")
    lbl_history_frame_help = QLabel(
        "Tabla STEP: Org=pose al abrir la fase | TCP-TF=TCP real por TF al cerrar | "
        "Obj World=pose objeto en world | Target=destino planificado | Exec=target enviado | "
        "D TCP-Obj / D Target-Obj / Err TCP-Exec en metros | Tipo Target explicita la semantica."
    )
    lbl_history_frame_help.setWordWrap(True)
    lbl_history_frame_help.setStyleSheet("color:#475569; font-size:12px;")
    history_table = QTableWidget(0, 23)
    history_table.setHorizontalHeaderLabels([
        "Fase", "Pinza",
        "Xw Org", "Yw Org", "Zw Org",
        "Xw TCP-TF", "Yw TCP-TF", "Zw TCP-TF",
        "Xw Obj", "Yw Obj", "Zw Obj",
        "Xw Target", "Yw Target", "Zw Target",
        "Xw Exec", "Yw Exec", "Zw Exec",
        "D TCP-Obj",
        "D Target-Obj",
        "Err TCP-Exec",
        "Tipo Target",
        "Razón",
        "Estado",
    ])
    history_table.setEditTriggers(QTableWidget.NoEditTriggers)
    history_table.setSelectionMode(QTableWidget.NoSelection)
    history_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
    history_header = history_table.horizontalHeader()
    if history_header is not None:
        history_header.setSectionResizeMode(0, QHeaderView.Stretch)
        history_header.setSectionResizeMode(21, QHeaderView.Stretch)
        for _col in range(1, 23):
            if _col not in (0, 21):
                history_header.setSectionResizeMode(_col, QHeaderView.ResizeToContents)
    history_table.verticalHeader().setVisible(False)
    btn_cart_debug = QPushButton("Depurador cartesiano")
    btn_cart_debug.setToolTip("Abrir ventana auxiliar para mover rg2_pinch_center en XYZ (base_link).")
    btn_cart_debug.clicked.connect(panel._show_step_cart_debug_window)
    btn_cart_debug.setEnabled(False)
    chk_cart_debug = QCheckBox("Depuración cartesiana")
    chk_cart_debug.setChecked(False)
    chk_cart_debug.setToolTip("Activa para habilitar el depurador cartesiano manual.")
    chk_cart_debug.toggled.connect(btn_cart_debug.setEnabled)
    return {
        "lbl_pipeline_title": lbl_pipeline_title,
        "pipeline_table": pipeline_table,
        "lbl_pipeline_help": lbl_pipeline_help,
        "lbl_history_title": lbl_history_title,
        "lbl_history_frame_help": lbl_history_frame_help,
        "history_table": history_table,
        "btn_cart_debug": btn_cart_debug,
        "chk_cart_debug": chk_cart_debug,
    }


def build_step_window(panel) -> None:
    """Build the STEP by STEP QDialog and attach all widgets as panel attributes."""
    dlg = QDialog(panel)
    dlg.setWindowTitle("STEP by STEP")
    dlg.setModal(False)
    dlg.setAttribute(Qt.WA_DeleteOnClose, False)
    dlg.resize(980, 760)

    root_layout = QVBoxLayout(dlg)
    root_layout.setContentsMargins(0, 0, 0, 0)
    root_layout.setSpacing(0)

    scroll = QScrollArea(dlg)
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QAbstractScrollArea.NoFrame)
    content = QWidget()
    root_layout.addWidget(scroll)
    scroll.setWidget(content)

    layout = QVBoxLayout(content)
    layout.setContentsMargins(12, 12, 12, 12)
    layout.setSpacing(8)

    lbl_title = QLabel("Control paso a paso")
    lbl_title.setStyleSheet("font-weight:700;")
    lbl_mode = QLabel("Flujo cargado: --")
    lbl_phase = QLabel("Fase lista para iniciar: --")
    lbl_current = QLabel("Fase en ejecución: --")
    lbl_next = QLabel("Próxima fase bloqueada: --")
    lbl_intent = QLabel("Objetivo de la fase: --")
    lbl_decision = QLabel("Acción exacta al pulsar Iniciar: --")
    lbl_target = QLabel("XYZ objetivo de la fase (world): --")
    lbl_live_operational = QLabel("XYZ actual del TCP (world): --")
    lbl_live_visual = QLabel("XYZ de referencia visual (world): --")
    lbl_gripper_expected = QLabel("Pinza esperada en la fase seleccionada: --")
    lbl_gripper_live = QLabel("Pinza live: --")
    lbl_object = QLabel("Objeto activo (world): --")
    lbl_start_pose = QLabel("Pose inicial del robot al lanzar la secuencia: --")
    info_labels = [
        lbl_mode,
        lbl_phase,
        lbl_current,
        lbl_next,
        lbl_intent,
        lbl_decision,
        lbl_target,
        lbl_live_operational,
        lbl_live_visual,
        lbl_gripper_expected,
        lbl_gripper_live,
        lbl_object,
        lbl_start_pose,
    ]
    for info_label in info_labels:
        info_label.setWordWrap(True)

    info_grid = QGridLayout()
    info_grid.setHorizontalSpacing(20)
    info_grid.setVerticalSpacing(6)
    left_info_labels = [
        lbl_mode,
        lbl_phase,
        lbl_current,
        lbl_next,
        lbl_intent,
        lbl_decision,
        lbl_target,
    ]
    right_info_labels = [
        lbl_live_operational,
        lbl_live_visual,
        lbl_gripper_expected,
        lbl_gripper_live,
        lbl_object,
        lbl_start_pose,
    ]
    for row, info_label in enumerate(left_info_labels):
        info_grid.addWidget(info_label, row, 0)
    for row, info_label in enumerate(right_info_labels):
        info_grid.addWidget(info_label, row, 1)
    info_grid.setColumnStretch(0, 1)
    info_grid.setColumnStretch(1, 1)

    lbl_runtime_title, lbl_runtime_help, runtime_group, runtime_blocks = (
        _build_step_runtime_section(panel)
    )
    pipeline_widgets = _build_step_pipeline_history_widgets(panel)
    lbl_pipeline_title = pipeline_widgets["lbl_pipeline_title"]
    pipeline_table = pipeline_widgets["pipeline_table"]
    lbl_pipeline_help = pipeline_widgets["lbl_pipeline_help"]
    lbl_history_title = pipeline_widgets["lbl_history_title"]
    lbl_history_frame_help = pipeline_widgets["lbl_history_frame_help"]
    history_table = pipeline_widgets["history_table"]
    btn_cart_debug = pipeline_widgets["btn_cart_debug"]
    chk_cart_debug = pipeline_widgets["chk_cart_debug"]

    layout.addWidget(lbl_title)
    layout.addLayout(info_grid)

    # ── Secciones colapsables: 3 bloques con botón ▶/▼ que muestran/ocultan
    # su contenido. Por defecto todos arrancan cerrados; el usuario expande
    # solo el bloque que quiere consultar.
    def _make_collapsible(title: str, widgets: list, start_open: bool = False):
        toggle_btn = QPushButton(("▼ " if start_open else "▶ ") + title)
        toggle_btn.setCheckable(True)
        toggle_btn.setChecked(start_open)
        toggle_btn.setStyleSheet(
            "QPushButton {"
            " text-align: left; font-weight: 700; padding: 6px 8px;"
            " border: 1px solid #cbd5e1; border-radius: 4px;"
            " background: #f1f5f9;"
            "}"
            "QPushButton:checked { background: #e0e7ff; }"
        )
        for w in widgets:
            w.setVisible(start_open)

        def _on_toggle(checked, btn=toggle_btn, ws=widgets, t=title):
            btn.setText(("▼ " if checked else "▶ ") + t)
            for _w in ws:
                _w.setVisible(checked)
        toggle_btn.toggled.connect(_on_toggle)
        return toggle_btn

    btn_runtime = _make_collapsible(
        "Verificación runtime Gazebo/TF",
        [runtime_group],
    )
    btn_pipeline = _make_collapsible(
        "Pipeline completo",
        [lbl_pipeline_title, pipeline_table, lbl_pipeline_help],
    )
    btn_history = _make_collapsible(
        "Histórico ejecutado",
        [lbl_history_title, lbl_history_frame_help, history_table],
    )

    layout.addWidget(btn_runtime)
    layout.addWidget(runtime_group)
    layout.addWidget(btn_pipeline)
    layout.addWidget(lbl_pipeline_title)
    layout.addWidget(pipeline_table)
    layout.addWidget(lbl_pipeline_help)
    layout.addWidget(btn_history)
    layout.addWidget(lbl_history_title)
    layout.addWidget(lbl_history_frame_help)
    layout.addWidget(history_table, 1)
    bottom_row = QHBoxLayout()
    bottom_row.setContentsMargins(0, 0, 0, 0)
    bottom_row.addWidget(chk_cart_debug)
    bottom_row.addWidget(btn_cart_debug)
    layout.addLayout(bottom_row)

    dlg.finished.connect(panel._on_step_window_finished)

    panel._step_window = dlg
    panel._step_pipeline_table = pipeline_table
    panel._step_pipeline_help_label = lbl_pipeline_help
    panel._step_mode_label = lbl_mode
    panel._step_phase_label = lbl_phase
    panel._step_current_label = lbl_current
    panel._step_next_label = lbl_next
    panel._step_intent_label = lbl_intent
    panel._step_decision_label = lbl_decision
    panel._step_target_label = lbl_target
    panel._step_live_operational_label = lbl_live_operational
    panel._step_live_visual_label = lbl_live_visual
    panel._step_gripper_expected_label = lbl_gripper_expected
    panel._step_live_gripper_label = lbl_gripper_live
    panel._step_object_label = lbl_object
    panel._step_start_pose_label = lbl_start_pose
    panel._step_runtime_section = runtime_group
    panel._step_runtime_help_label = lbl_runtime_help
    panel._step_runtime_block_labels = runtime_blocks
    panel._step_history_frame_help_label = lbl_history_frame_help
    panel._step_history_table = history_table

    # Timer de refresco live para los labels de pose del step window.
    # Solo actúa si hay un gate activo (_step_wait_active) y modo STEP_BY_STEP.
    # Frecuencia: 500ms — suficiente para que el usuario vea la pose "en vivo"
    # sin saturar el bus TF con lookups continuos.
    _step_live_timer = QTimer(dlg)
    _step_live_timer.setInterval(500)
    _step_live_timer.timeout.connect(panel._step_window_maybe_refresh)
    _step_live_timer.start()
    panel._step_live_timer = _step_live_timer


def build_step_cart_debug_window(panel) -> None:
    """Build the cartesian debug QDialog and attach all widgets as panel attributes."""
    parent = panel._step_window if panel._step_window is not None else panel
    dlg = QDialog(parent)
    dlg.setWindowTitle("STEP Depuracion Cartesiana")
    dlg.setModal(False)
    dlg.setAttribute(Qt.WA_DeleteOnClose, False)
    dlg.resize(420, 340)

    root = QVBoxLayout(dlg)
    root.setContentsMargins(10, 10, 10, 10)
    root.setSpacing(8)

    lbl_title = QLabel("Depuracion manual XYZ (base_link)")
    lbl_title.setStyleSheet("font-weight:700;")
    lbl_pose = QLabel("rg2_pinch_center@base_link: --")
    lbl_tool0 = QLabel("tool0@base_link: --")
    lbl_frame = QLabel("Frame operativo: rg2_pinch_center -> base_link")
    lbl_time = QLabel("timestamp: --")

    step_row = QHBoxLayout()
    step_row.addWidget(QLabel("Paso:"))
    combo = QComboBox()
    combo.addItem("1 mm", 0.001)
    combo.addItem("2 mm", 0.002)
    combo.addItem("5 mm", 0.005)
    combo.addItem("10 mm", 0.010)
    combo.setCurrentIndex(1)
    step_row.addWidget(combo)
    step_row.addStretch(1)

    grid = QGridLayout()
    grid.setHorizontalSpacing(6)
    grid.setVerticalSpacing(6)
    btn_x_neg = QPushButton("X-")
    btn_x_pos = QPushButton("X+")
    btn_y_neg = QPushButton("Y-")
    btn_y_pos = QPushButton("Y+")
    btn_z_neg = QPushButton("Z-")
    btn_z_pos = QPushButton("Z+")
    btn_refresh = QPushButton("Actualizar")
    btn_validate = QPushButton("Validar XYZ 5mm")

    btn_x_neg.clicked.connect(lambda: panel._step_cart_debug_handle_axis("X", -1))
    btn_x_pos.clicked.connect(lambda: panel._step_cart_debug_handle_axis("X", +1))
    btn_y_neg.clicked.connect(lambda: panel._step_cart_debug_handle_axis("Y", -1))
    btn_y_pos.clicked.connect(lambda: panel._step_cart_debug_handle_axis("Y", +1))
    btn_z_neg.clicked.connect(lambda: panel._step_cart_debug_handle_axis("Z", -1))
    btn_z_pos.clicked.connect(lambda: panel._step_cart_debug_handle_axis("Z", +1))
    btn_refresh.clicked.connect(panel._step_cart_debug_refresh)
    btn_validate.clicked.connect(panel._step_cart_debug_run_validation_xyz)

    grid.addWidget(btn_x_neg, 0, 0)
    grid.addWidget(btn_x_pos, 0, 1)
    grid.addWidget(btn_y_neg, 1, 0)
    grid.addWidget(btn_y_pos, 1, 1)
    grid.addWidget(btn_z_neg, 2, 0)
    grid.addWidget(btn_z_pos, 2, 1)
    grid.addWidget(btn_refresh, 3, 0, 1, 2)
    grid.addWidget(btn_validate, 4, 0, 1, 2)

    # Fila de descensos rápidos fijos
    lbl_bajar = QLabel("Bajar rápido:")
    lbl_bajar.setStyleSheet("font-weight:600;")
    drop_row = QHBoxLayout()
    drop_row.setSpacing(6)
    for _mm in (25, 50, 75, 100):
        _dist_m = _mm / 1000.0
        _btn = QPushButton(f"↓{_mm}mm")
        _btn.setToolTip(f"Bajar {_mm} mm en Z (base_link)")
        _btn.clicked.connect(
            lambda checked=False, d=_dist_m: panel._step_cart_debug_move_delta(0.0, 0.0, -d)
        )
        drop_row.addWidget(_btn)

    lbl_status = QLabel("Estado: listo")
    lbl_status.setStyleSheet("color:#0f766e; font-weight:600;")

    root.addWidget(lbl_title)
    root.addWidget(lbl_pose)
    root.addWidget(lbl_tool0)
    root.addWidget(lbl_frame)
    root.addWidget(lbl_time)
    root.addLayout(step_row)
    root.addLayout(grid)
    root.addWidget(lbl_bajar)
    root.addLayout(drop_row)
    root.addWidget(lbl_status)

    timer = QTimer(dlg)
    timer.setInterval(250)
    timer.timeout.connect(panel._step_cart_debug_refresh)
    timer.start()

    panel._step_cart_debug_window = dlg
    panel._step_cart_debug_pose_label = lbl_pose
    panel._step_cart_debug_tool0_label = lbl_tool0
    panel._step_cart_debug_frame_label = lbl_frame
    panel._step_cart_debug_time_label = lbl_time
    panel._step_cart_debug_status_label = lbl_status
    panel._step_cart_debug_step_combo = combo
    panel._step_cart_debug_timer = timer
