#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_object_mgmt.py
# Contenido: Object management callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""Object management and table/calibration callbacks for ControlPanelV2."""
from __future__ import annotations

import os
import time
from typing import Dict, List, Optional, Set, Tuple

from .panel_config import (
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    TABLE_TOP_Z,
    WORLDS_DIR,
)
from .panel_robot_presets import JOINT_TABLE_POSE_RAD
from .panel_objects import (
    ObjectLogicalState,
    get_object_positions,
    get_object_state,
    recalc_object_states,
    save_object_positions,
)


def _log_exception(context: str, exc: Exception) -> None:
    print(f"[OBJ_MGMT][ERROR][{context}] {exc}")


def _resolve_table_top_z(panel) -> float:
    if panel._table_top_z is not None:
        return panel._table_top_z
    panel._load_sdf_geometry_cache()
    if panel._table_top_z is None:
        panel._table_top_z = TABLE_TOP_Z
    return panel._table_top_z

def _resolve_safety_surface_z(panel, table_top: float) -> float:
    basket_top = float(BASKET_DROP[2]) if BASKET_DROP else table_top
    helper = get_tf_helper()
    if helper is None:
        return table_top
    world_frame = WORLD_FRAME or "world"
    ee_frame = panel._ee_frame_effective or "tool0"
    if not _can_transform_between(helper, world_frame, ee_frame, timeout_sec=0.2):
        return table_top
    tf = helper.lookup_transform(world_frame, ee_frame, timeout_sec=0.4)
    if not tf:
        return table_top
    x = float(tf.transform.translation.x)
    y = float(tf.transform.translation.y)
    table_x, table_y = TABLE_CENTER_X, TABLE_CENTER_Y
    basket_x, basket_y = float(BASKET_DROP[0]), float(BASKET_DROP[1])
    vec_x = basket_x - table_x
    vec_y = basket_y - table_y
    denom = (vec_x * vec_x) + (vec_y * vec_y)
    if denom <= 1e-6:
        return table_top
    t = ((x - table_x) * vec_x + (y - table_y) * vec_y) / denom
    return basket_top if t >= 0.5 else table_top

def _check_robot_above_table(panel,
    table_top: float,
    label: str = "",
    *,
    xy_margin: float = 0.00,
    z_eps: float = 0.01,
    monitor_mode: str = "list",
    monitor_frames: Optional[List[str]] = None,
) -> Tuple[bool, str]:
    helper = get_tf_helper()
    if helper is None:
        return False, "tf_helper=no"
    base_frame = panel._base_frame_effective or BASE_FRAME or "base_link"
    world_frame = WORLD_FRAME or "world"
    if not _can_transform_between(helper, world_frame, base_frame, timeout_sec=0.2):
        return False, f"tf_missing {world_frame}<->{base_frame}"
    surface_top = panel._resolve_safety_surface_z(table_top)
    frames = helper.list_frames()
    if not frames:
        return False, "tf_frames=empty"
    ee_frame = panel._ee_frame_effective or "tool0"
    default_monitor = [
        ee_frame,
        "rg2_tcp",
        "tool0",
        "ee_link",
        "flange",
        "gripper_link",
        "wrist_3_link",
    ]
    mode = str(monitor_mode or "list").strip().lower()
    candidates: List[str] = []
    if mode == "all":
        candidates = [
            name
            for name in sorted(frames)
            if any(keyword in name.lower() for keyword in ROBOT_FRAME_KEYWORDS)
        ]
    else:
        source = [str(f).strip() for f in (monitor_frames or []) if str(f).strip()]
        if not source:
            source = default_monitor
        for frame in source:
            if frame in frames and frame not in candidates:
                candidates.append(frame)
        if not candidates:
            for frame in default_monitor:
                if frame in frames and frame not in candidates:
                    candidates.append(frame)
        if not candidates:
            candidates = [
                name
                for name in sorted(frames)
                if any(keyword in name.lower() for keyword in ROBOT_FRAME_KEYWORDS)
            ]
    if not candidates:
        return False, "robot_frames=empty"
    x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
    x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
    y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
    y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)
    margin = max(0.0, float(xy_margin))
    x_min_chk = x_min - margin
    x_max_chk = x_max + margin
    y_min_chk = y_min - margin
    y_max_chk = y_max + margin
    z_eps = max(0.0, float(z_eps))
    min_z = None
    min_frame = ""
    violations: List[Tuple[str, float, float, float]] = []
    checked_count = 0
    over_table_count = 0
    for frame in candidates:
        tf = helper.lookup_transform(world_frame, frame, timeout_sec=0.6)
        if not tf:
            continue
        checked_count += 1
        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        z = float(tf.transform.translation.z)
        if min_z is None or z < min_z:
            min_z = z
            min_frame = frame
        over_table = (x_min_chk <= x <= x_max_chk) and (y_min_chk <= y <= y_max_chk)
        if not over_table:
            continue
        over_table_count += 1
        if z < (surface_top - z_eps):
            violations.append((frame, z, x, y))
    if min_z is None:
        return False, "tf_lookup=empty"
    ee_world = helper.lookup_transform(world_frame, ee_frame, timeout_sec=0.2)
    base_world = helper.lookup_transform(world_frame, base_frame, timeout_sec=0.2)
    basket_x, basket_y, basket_z = BASKET_DROP
    tcp_txt = "n/a"
    if ee_world is not None:
        t = ee_world.transform.translation
        tcp_txt = f"({float(t.x):.3f},{float(t.y):.3f},{float(t.z):.3f})"
    base_txt = "n/a"
    if base_world is not None:
        t = base_world.transform.translation
        base_txt = f"({float(t.x):.3f},{float(t.y):.3f},{float(t.z):.3f})"
    panel._emit_log(
        "[SAFETY][TABLE] "
        f"label={label or 'n/a'} frame={world_frame} base_frame={base_frame} ee_frame={ee_frame} "
        f"tcp_world={tcp_txt} tf_world_to_base_t={base_txt} "
        f"mode={mode} checked={checked_count} over_table={over_table_count}/{len(candidates)} "
        f"xy_margin={margin:.3f} z_eps={z_eps:.3f} "
        f"table_center=({TABLE_CENTER_X:.3f},{TABLE_CENTER_Y:.3f}) "
        f"table_bounds=([{x_min:.3f},{x_max:.3f}],[{y_min:.3f},{y_max:.3f}]) "
        f"table_top={table_top:.3f} surface_top={surface_top:.3f} "
        f"basket=({basket_x:.3f},{basket_y:.3f},{basket_z:.3f})"
    )
    if violations:
        worst = min(violations, key=lambda item: item[1])
        return (
            False,
            (
                f"frame={worst[0]} xyz=({worst[2]:.3f},{worst[3]:.3f},{worst[1]:.3f}) "
                f"surface_z={surface_top:.3f} margin={margin:.3f} z_eps={z_eps:.3f}"
            ),
        )
    return (
        True,
        (
            f"checked={checked_count} over_table={over_table_count} "
            f"min_frame={min_frame} min_z={min_z:.3f} surface_z={surface_top:.3f}"
        ),
    )

def _load_sdf_geometry_cache(panel) -> None:
    if panel._sdf_model_cache:
        return
    world_path = panel.world_combo.currentText().strip()
    if not world_path or not os.path.isfile(world_path):
        world_path = os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf")
    if not os.path.isfile(world_path):
        return
    try:
        tree = ET.parse(world_path)
        root = tree.getroot()
    except Exception as exc:
        _log_exception("parse SDF geometry cache", exc)
        return

    def _parse_pose(pose_text: str) -> Tuple[float, float, float]:
        parts = [p for p in (pose_text or "").split() if p]
        if len(parts) >= 3:
            try:
                return float(parts[0]), float(parts[1]), float(parts[2])
            except Exception as exc:
                _log_exception("parse model pose", exc)
        return 0.0, 0.0, 0.0

    for model in root.findall(".//model"):
        name = model.attrib.get("name") or ""
        if not name:
            continue
        geom = None
        geom_type = None
        size = None
        length = None
        radius = None
        model_pose = _parse_pose(model.findtext("pose") or "")
        link = model.find("link")
        collision_pose = (0.0, 0.0, 0.0)
        if link is not None:
            collision = link.find("collision")
            if collision is None:
                collision = link.find("visual")
            if collision is not None:
                geom = collision.find("geometry")
                collision_pose = _parse_pose(collision.findtext("pose") or "")
        if geom is None:
            continue
        box = geom.find("box")
        cyl = geom.find("cylinder")
        sph = geom.find("sphere")
        if box is not None:
            geom_type = "box"
            size_text = box.findtext("size") or ""
            parts = [p for p in size_text.split() if p]
            if len(parts) == 3:
                try:
                    size = tuple(float(p) for p in parts)
                except Exception as exc:
                    _log_exception("parse box size", exc)
                    size = None
        elif cyl is not None:
            geom_type = "cylinder"
            try:
                radius = float(cyl.findtext("radius") or 0.0)
                length = float(cyl.findtext("length") or 0.0)
            except Exception as exc:
                _log_exception("parse cylinder size", exc)
                radius = None
                length = None
        elif sph is not None:
            geom_type = "sphere"
            try:
                radius = float(sph.findtext("radius") or 0.0)
            except Exception as exc:
                _log_exception("parse sphere radius", exc)
                radius = None
        data = {
            "type": geom_type,
            "size": size,
            "radius": radius,
            "length": length,
            "pose": model_pose,
        }
        panel._sdf_model_cache[name] = data
        if name == "mesa_pro" and size and len(size) == 3:
            link_pose = _parse_pose(link.findtext("pose") or "")
            top = (
                model_pose[2]
                + link_pose[2]
                + collision_pose[2]
                + (size[2] / 2.0)
            )
            panel._table_top_z = top

def _load_joint_limits(panel) -> None:
    path = UR5_JOINT_LIMITS_YAML or ""
    if not path or not os.path.isfile(path):
        panel._joint_limits_ok = False
        panel._joint_limits_err = f"joint_limits.yaml no encontrado: {path or 'no especificado'}"
        panel._emit_log(f"[TFM] WARN: límites articulares no disponibles ({panel._joint_limits_err})")
        return
    if yaml is None:
        panel._joint_limits_ok = False
        panel._joint_limits_err = "PyYAML no disponible"
        panel._emit_log("[TFM] WARN: límites articulares no disponibles (PyYAML no disponible)")
        return
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:
        panel._joint_limits_ok = False
        panel._joint_limits_err = f"error leyendo {path}: {exc}"
        panel._emit_log(f"[TFM] WARN: límites articulares no disponibles ({exc})")
        return
    joint_limits = data.get("joint_limits") if isinstance(data, dict) else None
    if not isinstance(joint_limits, dict):
        panel._joint_limits_ok = False
        panel._joint_limits_err = f"formato inválido en {path}"
        panel._emit_log(f"[TFM] WARN: límites articulares inválidos ({path})")
        return
    missing = [name for name in UR5_JOINT_NAMES if name not in joint_limits]
    if missing:
        panel._joint_limits_ok = False
        panel._joint_limits_err = f"faltan joints: {', '.join(missing)}"
        panel._emit_log(f"[TFM] WARN: límites articulares incompletos ({', '.join(missing)})")
        return
    panel._joint_limits_ok = True
    panel._joint_limits_err = ""
    panel._emit_log(f"[TFM] Limites articulares OK ({Path(path).name})")

def _post_calibration_pipeline(panel) -> None:
    recalc_object_states("calibration")
    objects = panel._build_object_report()
    if objects:
        panel._log_object_report(objects)
        panel._pickable_map_cache = {obj["id"]: obj["pickable"] for obj in objects}
    else:
        panel._pickable_map_cache = None
    panel.signal_update_objects.emit()

def _build_object_report(panel) -> List[Dict[str, object]]:
    from .panel_utils import get_object_positions

    panel._load_sdf_geometry_cache()
    table_top = panel._resolve_table_top_z()
    objects = []
    positions = get_object_positions()
    if not positions:
        return objects
    for name, (x, y, z) in sorted(positions.items()):
        state = get_object_state(name)
        if not state or state.logical_state not in (
            ObjectLogicalState.ON_TABLE,
            ObjectLogicalState.SELECTED,
        ):
            continue
        sdf = panel._sdf_model_cache.get(name, {})
        geom_type = sdf.get("type") or "desconocido"
        size = sdf.get("size")
        radius = sdf.get("radius")
        length = sdf.get("length")
        height = None
        if geom_type == "box" and size:
            height = float(size[2])
        elif geom_type == "cylinder" and length:
            height = float(length)
        elif geom_type == "sphere" and radius:
            height = float(radius) * 2.0
        if height:
            z = table_top + (height / 2.0)
        # NOTE: yaw not needed for pickable; avoid ROS spin in worker threads.
        yaw = 0.0

        pickable, reason = panel._is_pickable(name, x, y, z, table_top, positions)
        tipo = "cubo" if geom_type == "box" else "cilindro" if geom_type == "cylinder" else "prisma"
        objects.append(
            {
                "id": name,
                "tipo": tipo,
                "pose": [round(x, 3), round(y, 3), round(z, 3), round(yaw, 3)],
                "pickable": pickable,
                "motivo": reason,
            }
        )
    return objects

def _is_pickable(panel,
    name: str,
    x: float,
    y: float,
    z: float,
    table_top: float,
    positions: Dict[str, Tuple[float, float, float]],
) -> Tuple[bool, Optional[str]]:
    dx = x - UR5_BASE_X
    dy = y - UR5_BASE_Y
    dist = (dx * dx + dy * dy) ** 0.5
    if dist > (UR5_REACH_RADIUS - 0.02):
        return False, "fuera_de_alcance"
    pre_grasp_z = z + PICKABLE_PRE_GRASP_Z
    if pre_grasp_z <= (table_top + 0.05):
        return False, "pregrasp_bajo"
    for other, (ox, oy, _oz) in positions.items():
        if other == name:
            continue
        if (ox - x) ** 2 + (oy - y) ** 2 < (PICKABLE_MIN_CLEARANCE ** 2):
            return False, "colision_con_objetos"
    return True, None

def _log_object_report(panel, objects: List[Dict[str, object]]) -> None:
    panel._emit_log("[PICK][REPORT] Objetos detectados:")
    for obj in objects:
        motivo = obj.get("motivo")
        motivo_txt = f" motivo={motivo}" if motivo else ""
        panel._emit_log(
            f"- id={obj['id']} tipo={obj['tipo']} pose={obj['pose']} pickable={obj['pickable']}{motivo_txt}"
        )

def _go_table(panel):
    panel._log_button("Go Mesa")
    panel._run_baseline_motion("MESA", lambda: list(JOINT_TABLE_POSE_RAD))

def _go_basket(panel):
    panel._log_button("Go Cesta")
    panel._run_baseline_motion("CESTA", lambda: list(JOINT_BASKET_POSE_RAD))

def _toggle_gripper_button(panel, checked: bool):
    if not panel._command_gripper(checked, log_action="Gripper"):
        return
    if checked:
        panel._schedule_attach_attempt("gripper_close")

def _on_camera_click(panel, px: int, py: int):
    """Manejar click en la imagen de cámara."""
    if hasattr(panel, "camera_view") and not panel.camera_view.isEnabled():
        return
    panel._emit_log(f"[PICK][CLICK] px=({px},{py})")
    ok, reason = pick_ui_status(panel)
    if not ok and "pose/info" not in str(reason):
        panel._emit_log(f"[PICK] Bloqueado: {reason}")
        return
    if not ok:
        # Permitir selección visual aunque pose/info no esté listo.
        panel._emit_log(f"[PICK] Selección permitida sin pose/info: {reason}")
    # Prioridad 1: Si está calibrando, manejar calibración
    if panel._calibrating:
        panel._handle_calibration_click(px, py)
        return
    
    # Prioridad 2: Si hay calibración válida, seleccionar objeto
    panel._handle_object_selection_click(px, py)

def _load_table_calibration(panel):
    """Cargar calibración de tabla desde archivo (IGUAL A PANEL ONLY)."""
    from .panel_utils import load_table_calib, TABLE_CALIB_PATH
    
    panel._emit_log("[CALIB] Intentando cargar calibración...")
    try:
        calib = load_table_calib()
        if calib:
            # Determinar tipo de calibración
            if isinstance(calib, dict):
                msg = f"[CALIB] Calibración RECT cargada desde {TABLE_CALIB_PATH}"
            elif isinstance(calib, list):
                if len(calib) == 3 and all(len(row) == 3 for row in calib):
                    msg = f"[CALIB] Calibración HOMOGRAFÍA cargada desde {TABLE_CALIB_PATH}"
                else:
                    msg = f"[CALIB] Calibración AFINE cargada desde {TABLE_CALIB_PATH}"
            panel._emit_log(msg)
            panel._log(msg)
            panel._set_status("✅ Calibración cargada", error=False)
        else:
            if os.path.isfile(TABLE_CALIB_PATH):
                msg = f"[CALIB] ⚠️ Archivo calibración inválido: {TABLE_CALIB_PATH}"
            else:
                msg = "[CALIB] ℹ️ No hay calibración guardada. Se intentará calibración automática al iniciar cámara."
            panel._emit_log(msg)
            panel._log(msg)
            panel._set_status("Sin calibración guardada", error=False)
    except Exception as e:
        msg = f"[CALIB] ERROR cargando calibración: {e}"
        panel._emit_log(msg)
        panel._log(msg)
    recalc_object_states("calibration_load")

def _refresh_objects_from_gz_async(panel):
    panel._run_async(panel._refresh_objects_from_gz)

def _resolve_pose_object_name(panel,
    raw_name: object,
    known: Dict[str, Tuple[float, float, float]],
) -> Optional[str]:
    """Map a pose/info entity name to a known object id."""
    if not isinstance(raw_name, str):
        return None
    name = raw_name.strip()
    if not name:
        return None
    if name in known:
        return name
    parts = [p for p in name.split("::") if p]
    for token in reversed(parts):
        token = token.strip()
        if token in known:
            return token
    tail = name.split("/")[-1].strip()
    if tail in known:
        return tail
    return None

def _extract_pose_updates(panel,
    poses: List[Dict[str, object]],
    known: Dict[str, Tuple[float, float, float]],
) -> Tuple[Dict[str, Tuple[float, float, float]], Dict[str, str]]:
    """Build object updates from pose/info with robust name mapping."""
    updates: Dict[str, Tuple[float, float, float]] = {}
    sources: Dict[str, str] = {}
    score_map: Dict[str, Tuple[int, int, float, float]] = {}
    for pose in poses:
        if not isinstance(pose, dict):
            continue
        raw_name = pose.get("name")
        key_name = panel._resolve_pose_object_name(raw_name, known)
        pos = pose.get("position") or {}
        if not key_name or not isinstance(pos, dict):
            continue
        try:
            x = float(pos.get("x"))
            y = float(pos.get("y"))
            z = float(pos.get("z"))
        except (TypeError, ValueError):
            continue
        raw_name_str = str(raw_name or "").strip()
        exact_match = 1 if raw_name_str == key_name else 0
        hierarchy_depth = raw_name_str.count("::") + raw_name_str.count("/")
        prev_known = known.get(key_name)
        continuity_score = float("-inf")
        if prev_known is not None:
            dx = float(x) - float(prev_known[0])
            dy = float(y) - float(prev_known[1])
            dz = float(z) - float(prev_known[2])
            continuity_score = -((dx * dx + dy * dy + dz * dz) ** 0.5)
        candidate_score = (
            exact_match,
            -hierarchy_depth,
            continuity_score,
            float(z),
        )
        prev_score = score_map.get(key_name)
        if prev_score is None or candidate_score > prev_score:
            score_map[key_name] = candidate_score
            updates[key_name] = (x, y, z)
            sources[key_name] = raw_name_str
    return updates, sources

def _refresh_objects_from_gz(panel):
    """Sincronizar poses de objetos desde Gazebo (igual a Panel Only)."""
    if not gz_sim_status()[0]:
        return
    panel._ensure_pose_subscription()
    objects_read_only = False

    world_path = panel.world_combo.currentText().strip()
    sdf_path = ""
    if world_path and os.path.isfile(world_path):
        sdf_path = world_path
    else:
        cand = os.path.join(WORLDS_DIR, world_path) if world_path else ""
        if cand and os.path.isfile(cand):
            sdf_path = cand
        elif cand and not cand.endswith(".sdf") and os.path.isfile(cand + ".sdf"):
            sdf_path = cand + ".sdf"
        if not sdf_path:
            for c in DEFAULT_WORLD_CANDIDATES:
                if os.path.isfile(c):
                    sdf_path = c
                    break

    world_name = read_world_name(sdf_path) if sdf_path else GZ_WORLD
    poses = panel._read_world_pose_info(world_name)
    if not poses:
        if not POSE_CLI_ENABLED:
            now = time.monotonic()
            if (now - panel._pose_cli_warn_ts) > 3.0:
                panel._pose_cli_warn_ts = now
                panel._emit_log("[PICK] Objetos: pose/info vacío; gz CLI deshabilitado")
            return
        now = time.monotonic()
        if (now - panel._pose_cli_last_ts) < max(0.1, POSE_CLI_MIN_INTERVAL_SEC):
            return
        panel._pose_cli_last_ts = now
        env_base = (
            "export GZ_SIM_RESOURCE_PATH='{}:{}:${{GZ_SIM_RESOURCE_PATH:-}}' ; "
            "export GZ_LOG_LEVEL=error; export IGN_LOGGER_LEVEL=error; "
            "export GZ_TRANSPORT_IP='{}' ; "
        ).format(
            MODELS_DIR,
            WORLDS_DIR,
            get_gz_transport_ip(),
        )
        partitions = []
        part = resolve_gz_partition(panel.gz_partition)
        if part:
            partitions.append(part)
        partitions.append("")
        out = ""
        for _ in range(4):
            for p in partitions:
                env = env_base + (f"export GZ_PARTITION='{p}' ; " if p else "")
                cmd = (
                    bash_preamble(panel.ws_dir)
                    + env
                    + f"gz topic -e -n 1 -t '/world/{world_name}/pose/info' --json-output"
            )
                res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
                poses = _parse_pose_json(res.stdout or "")
                if poses:
                    out = res.stdout
                    break
            if out:
                break
            panel._wait_for_state_change(0.6)
        if not out:
            panel._log("[PICK] Objetos: no se pudo leer poses desde Gazebo")
            return
        poses = _parse_pose_json(out)
        if not poses:
            return
    targets = panel._settle_targets()
    if panel._objects_settled and targets:
        seen_targets = {pose.get("name") for pose in poses if pose.get("name") in targets}
        if seen_targets != targets:
            panel._invalidate_settle("cambio de modelos en Gazebo", restart=True)
    known = get_object_positions()
    updates, pose_sources = panel._extract_pose_updates(poses, known)
    if getattr(panel, "_pick_target_lock_active", False):
        lock_name = str(getattr(panel, "_pick_target_lock_name", "") or "")
        lock_id = str(getattr(panel, "_pick_target_lock_id", "") or "n/a")
        if lock_name:
            lock_pose = updates.get(lock_name)
            if lock_pose is not None:
                src = pose_sources.get(lock_name, lock_name)
                lock_base = panel._ensure_base_coords(
                    (float(lock_pose[0]), float(lock_pose[1]), float(lock_pose[2])),
                    panel._world_frame_last_first(),
                    timeout_sec=0.35,
            )
                base_txt = "base=(n/a)"
                if lock_base is not None:
                    base_txt = (
                        f"base=({float(lock_base[0]):.3f},{float(lock_base[1]):.3f},{float(lock_base[2]):.3f})"
                    )
                panel._emit_log(
                    "[PICK_OBJ][TARGET_LOCK] pose_sync_during_pick "
                    f"lock_id={lock_id} lock_name={lock_name} "
                    f"{base_txt} "
                    f"source={src}"
            )
            else:
                panel._emit_log_throttled(
                    f"PICK:target_lock_pose_missing:{lock_name}",
                    "[PICK_OBJ][TARGET_LOCK] pose_sync_during_pick_missing "
                    f"lock_id={lock_id} lock_name={lock_name}",
            )

    updated = bulk_update_object_positions(
        updates,
        source="pose/info",
        read_only=objects_read_only,
        objects_stable=panel._objects_settled,
    )
    if updated:
        if SAVE_POSE_INFO_POSITIONS:
            save_object_positions()
        elif not panel._pose_info_save_logged:
            panel._emit_log(
                "[OBJECTS][INFO] pose/info no actualiza object_positions.json "
                "(set PANEL_SAVE_POSE_INFO_POSITIONS=1 para habilitar)"
            )
            panel._pose_info_save_logged = True
        if panel._debug_logs_enabled:
            sample = []
            for name in sorted(updates.keys()):
                x, y, z = updates[name]
                source_name = pose_sources.get(name, name)
                base_pose = panel._ensure_base_coords(
                    (float(x), float(y), float(z)),
                    panel._world_frame_last_first(),
                    timeout_sec=0.25,
            )
                if base_pose is not None:
                    sample.append(
                        f"{name}=({float(base_pose[0]):.3f},{float(base_pose[1]):.3f},{float(base_pose[2]):.3f})@{source_name}"
                    )
                else:
                    sample.append(f"{name}=base(n/a)@{source_name}")
            if sample:
                panel._emit_log(
                    "[OBJECTS][POSE_SRC] frame_id=base_link source=pose_info "
                    + " ".join(sample[:8])
            )
        panel._post_calibration_pipeline()
        panel.signal_update_objects.emit()
        panel._log(f"[PICK] Objetos sincronizados desde Gazebo ({updated}).")
        panel._maybe_recover_pick_demo("pose_sync")
    elif objects_read_only:
        panel._log("[PICK] Objetos: TEST activo, sync bloqueado (read-only).")
    else:
        panel._log("[PICK] Objetos: sin cambios desde Gazebo.")
        panel._maybe_recover_pick_demo("pose_sync_nochange")

def _settle_targets(panel) -> Set[str]:
    targets = set()
    for name in get_object_positions().keys():
        if name in SETTLE_MANUAL:
            targets.add(name)
    return targets

def _read_world_pose_info(panel, world_name: str) -> Optional[List[Dict[str, object]]]:
    if not panel._ros_worker_started:
        return None
    poses, ts = panel.ros_worker.pose_snapshot()
    if not poses:
        # Fallback: use cached object positions synced from Gazebo.
        fallback = get_object_positions()
        if not fallback:
            return None
        out = []
        for name, (x, y, z) in fallback.items():
            out.append({"name": name, "position": {"x": x, "y": y, "z": z}})
        return out
    age = _runtime_time() - ts if ts else float("inf")
    if age > POSE_INFO_MAX_AGE_SEC and not POSE_INFO_ALLOW_STALE:
        return None
    out: List[Dict[str, object]] = []
    for name, (x, y, z) in poses.items():
        out.append(
            {
                "name": name,
                "position": {"x": x, "y": y, "z": z},
            }
        )
    return out
