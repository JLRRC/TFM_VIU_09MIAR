#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_main_ui.py
# Contenido: Constructor Qt de la ventana principal del panel UR5.
# Uso breve: Importado por panel_v2.py; la función recibe el panel como argumento.
"""Builder function for the main ControlPanelV2 UI."""
from __future__ import annotations

import math
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from .panel_config import (
    BRIDGE_BASE_YAML,
    CAMERA_DISPLAY_INTERVAL_MS,
    DEFAULT_JOINT_MOVE_SEC,
    JOINT_SLIDER_DEG_MAX,
    JOINT_SLIDER_DEG_MIN,
    JOINT_SLIDER_SCALE,
)
from .panel_utils import load_home_pose

def build_main_ui(panel) -> None:
    root = QWidget()
    main = QVBoxLayout()
    main.setContentsMargins(8, 8, 8, 8)
    main.setSpacing(6)

    # --- Fila superior de botones y estado ---
    top = QHBoxLayout()
    top.setSpacing(6)

    panel.btn_kill_hard = QPushButton("STOP")
    panel.btn_kill_hard.setToolTip("Uso manual de emergencia (kill_all.sh)")
    panel.btn_star = QPushButton("START")
    panel.btn_star.setToolTip("Arranque automático: Gazebo, bridge, MoveIt y bridge MoveIt")
    panel.btn_close_terminal = QPushButton("EXIT")
    panel.btn_debug_joints = QPushButton("Debug articulaciones/poses")
    panel.btn_debug_joints.setCheckable(True)
    panel.btn_debug_logs = QPushButton("Debug logs")
    panel.btn_debug_logs.setCheckable(True)
    panel.btn_debug_logs.setChecked(panel._debug_logs_enabled)
    panel.step_mode_combo = QComboBox()
    panel.step_mode_combo.addItems(["AUTO", "STEP_BY_STEP"])
    panel.step_mode_combo.setCurrentText(panel._step_mode)
    panel.step_mode_combo.currentTextChanged.connect(panel._on_step_mode_combo_changed)
    panel._apply_debug_button_style(panel.btn_debug_joints, panel._debug_joints_to_stdout)
    panel._apply_debug_button_style(panel.btn_debug_logs, panel._debug_logs_enabled)

    panel.btn_kill_hard.clicked.connect(lambda: panel._debounced_btn_action(panel.btn_kill_hard, panel._stop_all))
    panel.btn_star.clicked.connect(lambda: panel._debounced_btn_action(panel.btn_star, panel._start_all))
    panel.btn_close_terminal.clicked.connect(panel._close_terminal)
    panel.btn_debug_logs.clicked.connect(lambda: panel._toggle_debug("DEBUG_LOGS_TO_STDOUT"))

    panel.status_lbl = QLabel("Listo · Bridge ROS ↔ Gazebo activo (0.0s)")
    panel.status_lbl.setAlignment(Qt.AlignCenter)
    panel.status_lbl.setStyleSheet(
        "background:#0f172a; color:white; padding:8px 14px; border-radius:12px; font-weight:600;"
    )

    # --- Combo de mundos y modos ---
    panel.world_combo = QComboBox()
    panel.world_combo.setEditable(True)
    panel._fill_worlds()
    panel.btn_world_browse = QPushButton("Mundo…")
    panel.btn_world_browse.clicked.connect(panel._choose_world)
    panel.mode_combo = QComboBox()
    panel.mode_combo.addItems(["Auto", "Headless", "GUI"])
    panel.btn_gz_start = QPushButton("Lanzar Gazebo")
    panel.btn_gz_stop = QPushButton("Detener Gazebo")
    panel.btn_gz_start.clicked.connect(panel._start_gazebo)
    panel.btn_gz_stop.clicked.connect(panel._stop_gazebo)
    panel.btn_debug_joints.clicked.connect(panel._toggle_debug_poses)

    # --- Bridge YAML ---
    panel.bridge_presets = QComboBox()
    panel._fill_bridge_presets()
    panel.bridge_presets.currentTextChanged.connect(panel._apply_bridge_preset)
    panel.bridge_edit = QLineEdit(BRIDGE_BASE_YAML)
    panel.btn_bridge_browse = QPushButton("YAML…")
    panel.btn_bridge_browse.clicked.connect(panel._choose_yaml)
    panel.btn_bridge_start = QPushButton("Lanzar bridge")
    panel.btn_bridge_stop = QPushButton("Detener bridge")
    panel.btn_bridge_start.clicked.connect(lambda: panel._debounced_btn_action(panel.btn_bridge_start, panel._start_bridge))
    panel.btn_bridge_stop.clicked.connect(lambda: panel._debounced_btn_action(panel.btn_bridge_stop, panel._stop_bridge))

    # --- Bag ---
    panel.bag_name = QLineEdit(f"demo_{int(time.time())}")
    panel.bag_topics = QLineEdit(
        "/camera_overhead/image /camera_north/image /camera_lateral/image /camera_east/image /camera_west/image"
    )
    panel.btn_bag_start = QPushButton("Start bag")
    panel.btn_bag_stop = QPushButton("Stop bag")
    panel.btn_bag_start.clicked.connect(lambda: panel._debounced_btn_action(panel.btn_bag_start, panel._start_bag))
    panel.btn_bag_stop.clicked.connect(lambda: panel._debounced_btn_action(panel.btn_bag_stop, panel._stop_bag))

    # --- MoveIt ---
    panel.btn_moveit_start = QPushButton("Lanzar MoveIt")
    panel.btn_moveit_stop = QPushButton("Detener MoveIt")
    panel.btn_moveit_bridge_start = QPushButton("Lanzar MoveIt bridge")
    panel.btn_moveit_bridge_stop = QPushButton("Detener MoveIt bridge")
    panel.btn_moveit_start.clicked.connect(
        lambda: panel._debounced_btn_action(panel.btn_moveit_start, panel._start_moveit)
    )
    panel.btn_moveit_stop.clicked.connect(
        lambda: panel._debounced_btn_action(panel.btn_moveit_stop, panel._stop_moveit)
    )
    panel.btn_moveit_bridge_start.clicked.connect(
        lambda: panel._debounced_btn_action(panel.btn_moveit_bridge_start, panel._start_moveit_bridge)
    )
    panel.btn_moveit_bridge_stop.clicked.connect(
        lambda: panel._debounced_btn_action(panel.btn_moveit_bridge_stop, panel._stop_moveit_bridge)
    )

    # --- LEDs de estado ---
    panel.led_gz = QLabel()
    panel.led_bridge = QLabel()
    panel.led_clock = QLabel()
    panel.led_bag = QLabel()
    panel.led_ros2 = QLabel()
    panel.led_ur5 = QLabel()
    panel.led_moveit = QLabel()
    panel.led_moveit_bridge = QLabel()
    for led in (
        panel.led_gz,
        panel.led_bridge,
        panel.led_clock,
        panel.led_bag,
        panel.led_ros2,
        panel.led_ur5,
        panel.led_moveit,
        panel.led_moveit_bridge,
    ):
        set_led(led, "off")

    # --- Sistema: labels para CPU/RAM/Load ---
    panel.sys_cpu_lbl = QLabel("CPU  --")
    panel.sys_ram_lbl = QLabel("RAM  --")
    panel.sys_load_lbl = QLabel("Load  --")
    panel.sys_health_lbl = QLabel("Todo OK")
    for lbl in (panel.sys_cpu_lbl, panel.sys_ram_lbl, panel.sys_load_lbl, panel.sys_health_lbl):
        lbl.setTextInteractionFlags(Qt.NoTextInteraction)

    panel.status_timer = QTimer(self)
    panel.status_timer.timeout.connect(panel._refresh_status_async)
    panel.status_updated.connect(panel._apply_status)

    for btn in (
        panel.btn_kill_hard,
        panel.btn_star,
        panel.btn_close_terminal,
        panel.btn_debug_joints,
        panel.btn_debug_logs,
    ):
        btn.setMinimumHeight(32)

    top.addWidget(panel.btn_star)
    top.addWidget(panel.btn_kill_hard)
    top.addWidget(panel.btn_close_terminal)
    top.addWidget(panel.btn_debug_joints)
    top.addWidget(panel.btn_debug_logs)
    top.addWidget(QLabel("Ejecución:"))
    top.addWidget(panel.step_mode_combo)
    top.addWidget(panel.status_lbl, 1)

    main.addLayout(top)

    controls_status_row = QHBoxLayout()
    controls_status_row.setSpacing(6)
    controls_col = QVBoxLayout()
    controls_col.setSpacing(4)

    gz_row = QHBoxLayout()
    gz_row.setSpacing(6)
    gz_row.addWidget(QLabel("Mundo:"))
    gz_row.addWidget(panel.world_combo, 1)
    gz_row.addWidget(panel.btn_world_browse)
    gz_row.addWidget(QLabel("Modo:"))
    gz_row.addWidget(panel.mode_combo)
    gz_row.addWidget(panel.btn_gz_start)
    gz_row.addWidget(panel.btn_gz_stop)
    controls_col.addLayout(gz_row)

    br_row = QHBoxLayout()
    br_row.setSpacing(6)
    br_row.addWidget(QLabel("Bridge YAML:"))
    br_row.addWidget(panel.bridge_presets)
    br_row.addWidget(panel.bridge_edit, 1)
    br_row.addWidget(panel.btn_bridge_browse)
    br_row.addWidget(panel.btn_bridge_start)
    br_row.addWidget(panel.btn_bridge_stop)
    controls_col.addLayout(br_row)

    bag_row = QHBoxLayout()
    bag_row.setSpacing(6)
    bag_row.addWidget(QLabel("Bag nombre:"))
    bag_row.addWidget(panel.bag_name, 1)
    bag_row.addWidget(QLabel("Tópicos:"))
    bag_row.addWidget(panel.bag_topics, 2)
    bag_row.addWidget(panel.btn_bag_start)
    bag_row.addWidget(panel.btn_bag_stop)
    controls_col.addLayout(bag_row)

    moveit_row = QHBoxLayout()
    moveit_row.setSpacing(6)
    moveit_row.addWidget(QLabel("MoveIt:"))
    moveit_row.addWidget(panel.btn_moveit_start)
    moveit_row.addWidget(panel.btn_moveit_stop)
    moveit_row.addSpacing(8)
    moveit_row.addWidget(QLabel("Bridge:"))
    moveit_row.addWidget(panel.btn_moveit_bridge_start)
    moveit_row.addWidget(panel.btn_moveit_bridge_stop)
    moveit_row.addStretch(1)
    controls_col.addLayout(moveit_row)

    controls_status_row.addLayout(controls_col, 3)

    status_group = QGroupBox("")
    status_group.setFlat(True)
    status_grid = QGridLayout()
    status_grid.setSpacing(4)
    status_grid.addWidget(QLabel("Gazebo"), 0, 0)
    status_grid.addWidget(panel.led_gz, 0, 1)
    status_grid.addWidget(QLabel("Gazebo bridge"), 0, 2)
    status_grid.addWidget(panel.led_bridge, 0, 3)
    status_grid.addWidget(QLabel("/clock"), 1, 0)
    status_grid.addWidget(panel.led_clock, 1, 1)
    status_grid.addWidget(QLabel("Rosbag"), 1, 2)
    status_grid.addWidget(panel.led_bag, 1, 3)
    status_grid.addWidget(QLabel("ros2_control"), 2, 0)
    status_grid.addWidget(panel.led_ros2, 2, 1)
    status_grid.addWidget(QLabel("UR5 (sim)"), 2, 2)
    status_grid.addWidget(panel.led_ur5, 2, 3)
    panel.lbl_moveit_status = QLabel("MoveIt")
    status_grid.addWidget(panel.lbl_moveit_status, 3, 0)
    status_grid.addWidget(panel.led_moveit, 3, 1)
    panel.lbl_moveit_bridge_status = QLabel("MoveIt bridge")
    status_grid.addWidget(panel.lbl_moveit_bridge_status, 3, 2)
    status_grid.addWidget(panel.led_moveit_bridge, 3, 3)
    status_group.setLayout(status_grid)
    controls_status_row.addWidget(status_group, 1)

    sys_group = QGroupBox("")
    sys_group.setFlat(True)
    sys_group.setStyleSheet("")
    sys_layout = QVBoxLayout()
    sys_layout.setContentsMargins(6, 6, 6, 6)
    sys_layout.setSpacing(2)
    sys_layout.addWidget(panel.sys_cpu_lbl)
    sys_layout.addWidget(panel.sys_ram_lbl)
    sys_layout.addWidget(panel.sys_load_lbl)
    sys_layout.addWidget(panel.sys_health_lbl)
    sys_group.setLayout(sys_layout)
    controls_status_row.addWidget(sys_group, 0)

    main.addLayout(controls_status_row)

    cam_group = QGroupBox("")
    cam_group.setFlat(True)
    cam_group.setStyleSheet("QGroupBox { margin:0; padding:0; }")
    cam_layout = QVBoxLayout()
    cam_layout.setContentsMargins(0, 6, 6, 6)
    cam_layout.setSpacing(2)

    cam_top = QHBoxLayout()
    cam_top.setSpacing(4)
    panel.camera_topic_combo = QComboBox()
    panel.camera_topic_combo.setEditable(True)
    panel.camera_topic_combo.addItem(panel.camera_topic)
    panel.btn_camera_refresh = QPushButton("↻")
    panel.btn_camera_refresh.setToolTip("Detectar tópicos")
    panel.btn_camera_refresh.setMaximumWidth(32)
    panel.btn_camera_refresh.clicked.connect(panel._camera_ctrl.refresh_topics)
    panel.btn_camera_connect = QPushButton("Conectar")
    panel.btn_camera_connect.clicked.connect(panel._camera_ctrl.connect)
    panel.btn_camera_far_front = QPushButton("Vista frontal")
    panel.btn_camera_far_front.setToolTip(
        "Cambia a una cámara frontal lejana de la mesa (prioriza /camera_west/image)."
    )
    panel.btn_camera_far_front.clicked.connect(panel._set_far_front_camera_view)
    panel.btn_camera_top = QPushButton("Vista cenital")
    panel.btn_camera_top.setToolTip(
        "Cambia a la cámara cenital (/camera_debug_top/image) — vista superior de la mesa y el robot."
    )
    panel.btn_camera_top.clicked.connect(panel._set_top_camera_view)
    panel.btn_camera_wrist = QPushButton("Vista muñeca")
    panel.btn_camera_wrist.setToolTip(
        "Cambia a la cámara de muñeca (/camera_wrist/image) — ve lo que ve la pinza."
    )
    panel.btn_camera_wrist.clicked.connect(panel._set_wrist_camera_view)
    panel.btn_release_objects = QPushButton("Soltar objetos")
    panel.btn_release_objects.setMinimumHeight(32)
    panel.btn_release_objects.setToolTip("Libera todos los objetos del drop anchor en Gazebo")
    panel.btn_release_objects.clicked.connect(panel._release_objects)
    panel.btn_calibrate = QPushButton("Calibrar")
    panel.btn_calibrate.setMinimumHeight(32)
    panel.btn_calibrate.setToolTip("Inicia o detiene la calibración de la mesa")
    panel.btn_calibrate.clicked.connect(panel._start_calibration)
    panel.btn_recover = QPushButton("Recover")
    panel.btn_recover.setToolTip("Diagnóstico: reintenta Gazebo/bridge/cámara sin stop_all")
    panel.btn_recover.clicked.connect(panel._recover_runtime)
    cam_top.addWidget(QLabel("Tópico:"))
    cam_top.addWidget(panel.camera_topic_combo, 1)
    cam_top.addWidget(panel.btn_camera_refresh)
    cam_top.addWidget(panel.btn_camera_connect)
    cam_top.addWidget(panel.btn_camera_far_front)
    cam_top.addWidget(panel.btn_camera_top)
    cam_top.addWidget(panel.btn_camera_wrist)
    cam_top.addWidget(panel.btn_release_objects)
    cam_top.addWidget(panel.btn_calibrate)
    cam_top.addWidget(panel.btn_recover)
    cam_layout.addLayout(cam_top)

    panel.camera_view = CameraView("Esperando imagen...")
    panel.camera_view.clicked.connect(panel._on_camera_click)
    panel.camera_info = QLabel("Sin conexión")
    panel.camera_info.setStyleSheet("color:#64748b; font-size:13px;")
    panel.motion_lbl = QLabel("idle")
    panel.motion_lbl.setStyleSheet("color:#22c55e; font-weight:bold; font-size:13px;")
    cam_info_row = QHBoxLayout()
    cam_info_row.setContentsMargins(0, 0, 0, 0)
    cam_info_row.addWidget(panel.camera_info)
    cam_info_row.addWidget(panel.motion_lbl)
    cam_info_row.addStretch(1)
    cam_layout.addWidget(panel.camera_view, 1)
    cam_layout.addLayout(cam_info_row)
    cam_group.setLayout(cam_layout)
    panel._last_camera_frame_ts = 0.0
    panel._camera_frame_lock = threading.Lock()
    panel._camera_pending_frame = None
    panel._camera_initializing = False
    panel._camera_init_start = 0.0
    panel._camera_display_timer = QTimer(self)
    panel._camera_display_timer.setInterval(int(CAMERA_DISPLAY_INTERVAL_MS))
    panel._camera_display_timer.timeout.connect(panel._camera_ctrl.refresh_display)
    panel._camera_display_timer.start()
    panel._camera_health_timer = QTimer(self)
    panel._camera_health_timer.setInterval(1200)
    panel._camera_health_timer.timeout.connect(panel._camera_ctrl.health_check)
    panel._camera_msg_type = "image"
    panel._camera_status_connected = False
    panel._camera_info_last_ts = 0.0

    panel.obj_panel = ObjectListPanel()
    panel.obj_panel.selected.connect(panel._on_object_clicked)
    panel.obj_panel.setFixedWidth(52)

    g_manual = QGroupBox("")
    g_manual.setFlat(True)
    g_manual.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
    g_manual.setMaximumHeight(380)
    manual_layout = QVBoxLayout()
    manual_layout.setContentsMargins(6, 6, 6, 6)
    manual_layout.setSpacing(3)

    manual_top = QHBoxLayout()
    manual_top.setSpacing(3)
    panel.btn_send_joints = QPushButton("Mover articulaciones")
    panel.btn_send_joints.setMinimumWidth(140)
    panel.btn_send_joints.clicked.connect(panel._send_joints)
    panel.joint_time = QDoubleSpinBox()
    panel.joint_time.setDecimals(2)
    panel.joint_time.setRange(0.5, 8.0)
    panel.joint_time.setSingleStep(0.25)
    panel.joint_time.setValue(DEFAULT_JOINT_MOVE_SEC)
    panel.joint_time.setSuffix(" s")
    panel.joint_time.setMaximumWidth(70)
    panel.chk_auto_joints = QCheckBox("Auto")
    panel.chk_auto_joints.setChecked(True)
    panel.btn_gripper = QPushButton("Cerrar gripper")
    panel.btn_gripper.setCheckable(True)
    panel.btn_gripper.setMinimumHeight(28)
    panel.btn_gripper.clicked.connect(panel._toggle_gripper_button)
    manual_top.addWidget(panel.btn_send_joints)
    manual_top.addWidget(QLabel("t"))
    manual_top.addWidget(panel.joint_time)
    manual_top.addWidget(panel.chk_auto_joints)
    manual_top.addWidget(panel.btn_gripper)
    manual_top.addStretch(1)
    manual_layout.addLayout(manual_top)

    slider_grid = QGridLayout()
    slider_grid.setHorizontalSpacing(3)
    slider_grid.setVerticalSpacing(2)
    slider_grid.setColumnStretch(2, 1)
    panel.joint_sliders = []
    panel.joint_value_labels = []
    panel.joint_step_buttons: list = []   # pares (btn_minus, btn_plus) por joint
    slider_min = int(JOINT_SLIDER_DEG_MIN * JOINT_SLIDER_SCALE)
    slider_max = int(JOINT_SLIDER_DEG_MAX * JOINT_SLIDER_SCALE)
    home_pose = load_home_pose()
    for idx, joint in enumerate(UR5_JOINT_NAMES):
        jlabel = QLabel(f"J{idx + 1}")
        jlabel.setToolTip(joint)
        jlabel.setStyleSheet("font-size:9px;")
        btn_minus = QPushButton("-")
        btn_plus = QPushButton("+")
        for b in (btn_minus, btn_plus):
            b.setFixedWidth(18)
            b.setFixedHeight(18)
            b.setStyleSheet("font-size:9px; padding:0;")
        btn_minus.clicked.connect(lambda _=False, i=idx: panel._step_joint(i, -1))
        btn_plus.clicked.connect(lambda _=False, i=idx: panel._step_joint(i, 1))
        panel.joint_step_buttons.append((btn_minus, btn_plus))
        slider = QSlider(Qt.Horizontal)
        slider.setRange(slider_min, slider_max)
        slider.setSingleStep(1)
        slider.setPageStep(10)
        slider.setFixedHeight(18)
        slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        value_lbl = QLabel("0.0 deg / 0.00 rad")
        value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        value_lbl.setFixedWidth(120)
        value_lbl.setStyleSheet("font-size:9px;")
        slider.valueChanged.connect(lambda v, i=idx: panel._on_slider_change(i, v))
        slider.sliderReleased.connect(panel._maybe_send_auto)
        panel.joint_sliders.append(slider)
        panel.joint_value_labels.append(value_lbl)
        if idx < len(home_pose):
            slider.setValue(int(round(math.degrees(home_pose[idx]) * JOINT_SLIDER_SCALE)))
        else:
            slider.setValue(0)
        slider_grid.addWidget(jlabel, idx, 0)
        slider_grid.addWidget(btn_minus, idx, 1)
        slider_grid.addWidget(slider, idx, 2)
        slider_grid.addWidget(btn_plus, idx, 3)
        slider_grid.addWidget(value_lbl, idx, 4)
    manual_layout.addLayout(slider_grid)

    info_grid = QGridLayout()
    info_grid.setHorizontalSpacing(6)
    info_grid.setVerticalSpacing(3)
    info_font_css = (
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
    info_grid.setColumnStretch(0, 1)
    info_grid.setColumnStretch(1, 1)
    info_grid.setColumnStretch(2, 1)

    panel.dof_pos_labels: Dict[str, QLabel] = {}
    panel.dof_vel_labels: Dict[str, QLabel] = {}
    panel.gripper_labels: Dict[str, QLabel] = {}
    panel.gripper_total_lbl: Optional[QLabel] = None
    panel.tcp_xyz_lbl: Optional[QLabel] = None
    panel.tcp_rpy_lbl: Optional[QLabel] = None
    panel.tcp_live_xyz_lbl: Optional[QLabel] = None
    panel.tcp_live_rpy_lbl: Optional[QLabel] = None
    panel.vel_norm_lbl: Optional[QLabel] = None
    panel.vel_max_lbl: Optional[QLabel] = None
    panel.eff_max_lbl: Optional[QLabel] = None

    dof_group = QGroupBox("DOF UR5 (pos/vel)")
    dof_group.setFlat(True)
    dof_group.setStyleSheet(info_font_css)
    dof_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    dof_layout = QGridLayout()
    dof_layout.setContentsMargins(2, 2, 2, 2)
    dof_layout.setSpacing(1)
    dof_headers = ["Joint", "Posición", "Velocidad"]
    dof_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    for col, header in enumerate(dof_headers):
        dof_layout.addWidget(QLabel(f"<b>{header}</b>"), 0, col)
    for row, joint in enumerate(dof_joints, 1):
        name_lbl = QLabel(joint.replace("_", " "))
        pos_lbl = QLabel("--")
        vel_lbl = QLabel("--")
        panel.dof_pos_labels[joint] = pos_lbl
        panel.dof_vel_labels[joint] = vel_lbl
        dof_layout.addWidget(name_lbl, row, 0)
        dof_layout.addWidget(pos_lbl, row, 1)
        dof_layout.addWidget(vel_lbl, row, 2)
    dof_group.setLayout(dof_layout)
    info_grid.addWidget(dof_group, 0, 0, 2, 1)

    gripper_group = QGroupBox("Pinza RG2 (pos)")
    gripper_group.setFlat(True)
    gripper_group.setStyleSheet(info_font_css)
    gripper_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    gripper_layout = QGridLayout()
    gripper_layout.setContentsMargins(2, 2, 2, 2)
    gripper_layout.setSpacing(1)
    gripper_layout.addWidget(QLabel("<b>Joint</b>"), 0, 0)
    gripper_layout.addWidget(QLabel("<b>Apertura</b>"), 0, 1)
    for row, joint in enumerate(["rg2_finger_joint1", "rg2_finger_joint2"], start=1):
        name_lbl = QLabel(joint)
        pos_lbl = QLabel("--")
        panel.gripper_labels[joint] = pos_lbl
        gripper_layout.addWidget(name_lbl, row, 0)
        gripper_layout.addWidget(pos_lbl, row, 1)
    panel.gripper_total_lbl = QLabel("--")
    gripper_layout.addWidget(QLabel("Apertura total"), 3, 0)
    gripper_layout.addWidget(panel.gripper_total_lbl, 3, 1)
    gripper_group.setLayout(gripper_layout)
    info_grid.addWidget(gripper_group, 0, 1, 2, 1)

    kin_group = QGroupBox("TCP Panel")
    kin_group.setFlat(True)
    kin_group.setStyleSheet(info_font_css)
    kin_layout = QGridLayout()
    kin_layout.setContentsMargins(2, 2, 2, 2)
    kin_layout.setSpacing(1)
    panel.tcp_xyz_lbl = QLabel("--")
    panel.tcp_rpy_lbl = QLabel("--")
    panel.tcp_live_xyz_lbl = QLabel("--")
    panel.tcp_live_rpy_lbl = QLabel("--")
    kin_layout.addWidget(QLabel("<b>FK modelo xyz [m]</b>"), 0, 0)
    kin_layout.addWidget(panel.tcp_xyz_lbl, 0, 1)
    kin_layout.addWidget(QLabel("<b>FK modelo RPY [deg]</b>"), 1, 0)
    kin_layout.addWidget(panel.tcp_rpy_lbl, 1, 1)
    kin_layout.addWidget(QLabel("<b>TF vivo ee operativo xyz [m]</b>"), 2, 0)
    kin_layout.addWidget(panel.tcp_live_xyz_lbl, 2, 1)
    kin_layout.addWidget(QLabel("<b>TF vivo ee operativo RPY [deg]</b>"), 3, 0)
    kin_layout.addWidget(panel.tcp_live_rpy_lbl, 3, 1)
    kin_group.setLayout(kin_layout)
    info_grid.addWidget(kin_group, 0, 2)

    dyn_group = QGroupBox("Dinámica (vel/eff)")
    dyn_group.setFlat(True)
    dyn_group.setStyleSheet(info_font_css)
    dyn_layout = QGridLayout()
    dyn_layout.setContentsMargins(2, 2, 2, 2)
    dyn_layout.setSpacing(1)
    panel.vel_max_lbl = QLabel("--")
    panel.eff_max_lbl = QLabel("--")
    panel.vel_norm_lbl = QLabel("--")
    dyn_layout.addWidget(QLabel("<b>||qdot||</b>"), 0, 0)
    dyn_layout.addWidget(panel.vel_norm_lbl, 0, 1)
    dyn_layout.addWidget(QLabel("<b>max |qdot|</b>"), 1, 0)
    dyn_layout.addWidget(panel.vel_max_lbl, 1, 1)
    dyn_layout.addWidget(QLabel("<b>max |eff|</b>"), 2, 0)
    dyn_layout.addWidget(panel.eff_max_lbl, 2, 1)
    dyn_group.setLayout(dyn_layout)
    info_grid.addWidget(dyn_group, 1, 2)

    manual_layout.addLayout(info_grid)

    panel.lbl_joint_states = QLabel("Joint states: esperando /joint_states ...")
    panel.lbl_joint_states.setStyleSheet("color:#64748b; font-size:10px;")
    manual_layout.addWidget(panel.lbl_joint_states)
    manual_layout.addStretch(1)
    g_manual.setLayout(manual_layout)
    panel.trace_group = panel._build_trace_group()
    panel.science_group = panel._build_science_group()

    g_robot = QGroupBox("")
    g_robot.setFlat(True)
    g_robot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
    robot_layout = QGridLayout()
    robot_layout.setContentsMargins(6, 8, 6, 6)
    robot_layout.setHorizontalSpacing(12)
    robot_layout.setVerticalSpacing(6)

    panel.btn_debug_motion = QPushButton("DEBUG MOVIMIENTO")
    panel.btn_home = QPushButton("UR5 → HOME")
    panel.btn_table = QPushButton("UR5 → Mesa")
    panel.btn_basket = QPushButton("UR5 → Cesta")
    for b in (
        panel.btn_debug_motion,
        panel.btn_home,
        panel.btn_table,
        panel.btn_basket,
    ):
        b.setMinimumHeight(32)
    panel.btn_debug_motion.setVisible(False)
    panel.btn_debug_motion.setEnabled(False)
    panel.btn_debug_motion.setToolTip(
        "Pausa manual de depuración: cuando haya una pausa activa, pulsa para continuar."
    )
    panel.btn_debug_motion.clicked.connect(panel._on_debug_motion_button)
    panel.btn_home.clicked.connect(panel._go_home)
    panel.btn_table.clicked.connect(panel._go_table)
    panel.btn_basket.clicked.connect(panel._go_basket)
    panel._set_debug_motion_button_waiting(False)

    panel.btn_pick_demo = QPushButton("Agarre Objeto (Directo)")
    panel.btn_pick_demo.setMinimumHeight(32)
    panel.btn_pick_demo.setToolTip(
        "Demo pick & place con objeto de posicion conocida (fuera del TFM)"
    )
    panel.btn_pick_demo.clicked.connect(panel._run_pick_demo)
    panel.btn_pick_demo_approach = QPushButton("Iniciar APPROACH_COARSE")
    panel.btn_pick_demo_approach.setMinimumHeight(32)
    panel.btn_pick_demo_approach.setToolTip(
        "Segundo paso del flujo Directo: continuar desde MESA hacia APPROACH_COARSE"
    )
    panel.btn_pick_demo_approach.clicked.connect(panel._run_pick_demo)
    panel.btn_pick_demo_approach.setVisible(False)
    panel.btn_pick_demo_approach.setEnabled(False)
    panel.btn_pick_object = QPushButton("Agarre Objeto (MoveIT)")
    panel.btn_pick_object.setMinimumHeight(32)
    panel.btn_pick_object.setToolTip("Coger objeto seleccionado y llevarlo a la cesta")
    panel.btn_pick_object.clicked.connect(panel._run_pick_object)
    panel.combo_tfm_experiment = QComboBox()
    panel.combo_tfm_experiment.setMinimumHeight(30)
    panel.chk_tfm_use_depth = QCheckBox("Usar RGB-D (depth)")
    panel.chk_tfm_use_depth.setToolTip("Se muestran EXP1-EXP4 con su mejor checkpoint")
    panel.chk_tfm_use_depth.setChecked(True)
    panel.chk_tfm_use_depth.setEnabled(False)
    panel.chk_tfm_use_depth.setVisible(False)
    panel.chk_tfm_repro_mode = QCheckBox("Modo reproducción TFM")
    panel.chk_tfm_repro_mode.setToolTip(
        "Fija el caso principal documentado: EXP3_RESNET18_RGB_AUGMENT / seed_0"
    )
    panel.chk_tfm_repro_mode.setChecked(panel._tfm_repro_profile_env() == "exp3_seed0")
    panel.chk_tfm_repro_mode.setVisible(False)
    panel.chk_tfm_raw_output = QCheckBox("Predicción raw (sin ajustes panel)")
    panel.chk_tfm_raw_output.setToolTip(
        "Desactiva los ajustes heurísticos posteriores de ángulo, centro y tamaño"
    )
    panel.chk_tfm_raw_output.setChecked(panel._tfm_raw_output_env_enabled())
    panel.chk_tfm_raw_output.setVisible(False)
    panel.btn_tfm_memoria_case = QPushButton("Caso TFM Memoria")
    panel.btn_tfm_memoria_case.setToolTip(
        "Activa de una vez EXP3_RESNET18_RGB_AUGMENT / seed_0 + salida raw y aplica el experimento"
    )
    panel.btn_tfm_apply = QPushButton("Aplicar experimento")
    panel.btn_tfm_infer = QPushButton("Inferir agarre")
    panel.btn_tfm_visualize = QPushButton("Comparar grasp/ref")
    panel.btn_tfm_visualize.setToolTip(
        "Alterna comparación entre grasp inferido (P) y referencia Cornell del objeto seleccionado (R)"
    )
    panel.btn_tfm_reset = QPushButton("Reset TFM")
    panel.btn_tfm_grasp_object = QPushButton("Agarre objeto")
    panel.btn_tfm_grasp_object.setToolTip(
        "Ejecutar agarre MoveIt usando el objeto señalado por la inferencia del experimento aplicado\n"
        "mode=moveit  source=infer_model  tcp=rg2_pinch_center  frame=base_link"
    )
    for b in (
        panel.btn_tfm_memoria_case,
        panel.btn_tfm_apply,
        panel.btn_tfm_infer,
        panel.btn_tfm_visualize,
        panel.btn_tfm_reset,
        panel.btn_tfm_grasp_object,
    ):
        b.setMinimumHeight(32)
    panel.btn_tfm_apply.clicked.connect(panel._tfm_apply_experiment)
    panel.btn_tfm_memoria_case.clicked.connect(panel._tfm_apply_memoria_case)
    panel.btn_tfm_infer.clicked.connect(panel._tfm_infer_grasp)
    panel.btn_tfm_visualize.clicked.connect(panel._tfm_visualize_grasp)
    panel.btn_tfm_reset.clicked.connect(panel._tfm_reset_grasp)
    panel.btn_tfm_grasp_object.clicked.connect(panel._on_tfm_grasp_object_clicked)
    panel.combo_tfm_experiment.currentIndexChanged.connect(lambda _idx: panel._on_tfm_checkpoint_selection_changed())
    panel.chk_tfm_use_depth.stateChanged.connect(lambda _state: panel._refresh_tfm_checkpoint_options())
    panel.chk_tfm_repro_mode.stateChanged.connect(lambda _state: panel._on_tfm_repro_mode_changed())
    panel.chk_tfm_raw_output.stateChanged.connect(lambda _state: panel._on_tfm_postprocess_mode_changed())
    panel._refresh_tfm_checkpoint_options()
    if not panel.tfm_module:
        panel.combo_tfm_experiment.setEnabled(False)
        panel.chk_tfm_use_depth.setEnabled(False)
        panel.chk_tfm_repro_mode.setEnabled(False)
        panel.chk_tfm_raw_output.setEnabled(False)
        panel.btn_tfm_memoria_case.setEnabled(False)
        panel.btn_tfm_apply.setEnabled(False)
        panel.btn_tfm_infer.setEnabled(False)
        panel.btn_tfm_visualize.setEnabled(False)
        panel.btn_tfm_reset.setEnabled(False)
        panel.btn_tfm_grasp_object.setEnabled(False)
    # "Agarre objeto" solo se habilita tras inferencia OK (independientemente del módulo)
    panel.btn_tfm_grasp_object.setEnabled(False)

    baseline_title = QLabel("Baseline (UR5 / Demo)")
    baseline_title.setStyleSheet("font-weight:700;")
    tfm_title = QLabel("TFM - Agarre Inteligente")
    tfm_title.setStyleSheet("font-weight:700;")
    baseline_info = QLabel("Pose objeto: Gazebo (/world/.../pose/info)")
    baseline_info.setWordWrap(True)
    baseline_info.setStyleSheet("color:#666; font-size:11px;")
    tfm_info = QLabel("Pose objeto: camara + homografia/TFM")
    tfm_info.setWordWrap(True)
    tfm_info.setStyleSheet("color:#666; font-size:11px;")

    baseline_col = QVBoxLayout()
    baseline_col.setSpacing(6)
    baseline_col.addWidget(panel.btn_home)
    baseline_col.addWidget(panel.btn_table)
    baseline_col.addWidget(panel.btn_basket)
    baseline_col.addWidget(panel.btn_pick_demo)
    baseline_col.addWidget(panel.btn_pick_demo_approach)
    baseline_col.addWidget(panel.btn_pick_object)
    baseline_col.addWidget(baseline_info)
    baseline_col.addStretch(1)

    tfm_col = QVBoxLayout()
    tfm_col.setSpacing(6)
    tfm_col.addWidget(panel.combo_tfm_experiment)
    tfm_col.addWidget(panel.chk_tfm_use_depth)
    tfm_col.addWidget(panel.chk_tfm_repro_mode)
    tfm_col.addWidget(panel.chk_tfm_raw_output)
    _tfm_grid_top = QHBoxLayout()
    _tfm_grid_top.setSpacing(6)
    _tfm_grid_top.addWidget(panel.btn_tfm_memoria_case)
    _tfm_grid_top.addWidget(panel.btn_tfm_apply)
    _tfm_grid_bot = QHBoxLayout()
    _tfm_grid_bot.setSpacing(6)
    _tfm_grid_bot.addWidget(panel.btn_tfm_infer)
    _tfm_grid_bot.addWidget(panel.btn_tfm_visualize)
    tfm_col.addLayout(_tfm_grid_top)
    tfm_col.addLayout(_tfm_grid_bot)
    tfm_col.addWidget(panel.btn_tfm_grasp_object)
    tfm_col.addWidget(panel.btn_tfm_reset)
    tfm_col.addWidget(tfm_info)
    tfm_col.addStretch(1)

    robot_layout.addWidget(baseline_title, 0, 0)
    robot_layout.addWidget(tfm_title, 0, 1)
    robot_layout.addLayout(baseline_col, 1, 0)
    robot_layout.addLayout(tfm_col, 1, 1)
    g_robot.setLayout(robot_layout)

    left_col = QVBoxLayout()
    left_col.setSpacing(4)
    cam_row = QHBoxLayout()
    cam_row.setContentsMargins(0, 0, 0, 0)
    cam_row.setSpacing(0)
    cam_row.addWidget(panel.obj_panel, 0, Qt.AlignTop)
    cam_row.addWidget(cam_group, 1)
    left_col.addLayout(cam_row, 2)
    left_col.addWidget(g_robot, 1)
    left_col.addStretch(1)

    right_col = QVBoxLayout()
    right_col.setSpacing(6)
    right_col.addWidget(g_manual, 2)
    object_trace_layout = QHBoxLayout()
    object_trace_layout.setSpacing(6)
    if panel.trace_group:
        object_trace_layout.addWidget(panel.trace_group, 1)
    if panel.science_group:
        object_trace_layout.addWidget(panel.science_group, 1)
    right_col.addLayout(object_trace_layout, 1)
    right_col.addStretch(1)

    top_row = QHBoxLayout()
    top_row.setSpacing(8)
    top_row.addLayout(left_col, 1)
    top_row.addLayout(right_col, 1)

    main.addLayout(top_row)

    # Al final del método, tras crear todos los widgets:
    root.setLayout(main)
    panel.setCentralWidget(root)
    panel._start_trace_timer()
    panel._apply_status(False, False, False, False, False, False, False)

