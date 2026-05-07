#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_science.py
# Contenido: TFM checkpoint, experiment, and science UI callbacks extracted from ControlPanelV2.
# Uso breve: Importado por panel_v2.py; cada función recibe panel como primer argumento.
"""TFM checkpoint discovery, experiment loading, and science UI callbacks."""
from __future__ import annotations

import csv
from datetime import datetime
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from .panel_objects import get_object_position
from .panel_tfm_params import get_panel_tfm_params as _get_panel_tfm_params
from .panel_utils import base_to_world, pixel_to_table_xy, table_xy_to_pixel, table_xy_to_pixel_float
try:
    import yaml
except ImportError:
    yaml = None  # type: ignore

from .panel_config import (
    INFER_CKPT,
    VISION_EXP_DIR,
)
from .panel_utils import (
    world_xyz_to_pixel,
    world_xyz_to_pixel_float,
)
from .panel_camera import _runtime_time
from .logging_utils import emit_log_line


def _log_exception(context: str, exc: Exception) -> None:
    emit_log_line(f"[TFM_SCI][ERROR][{context}] {exc}")


def _discover_tfm_checkpoints(panel, allow_rgbd: Optional[bool] = None) -> List[str]:
    del allow_rgbd  # Selector fijo EXP1..EXP4 + EXP1.1/EXP1.2, independiente del checkbox depth.
    repro_ckpt = panel._tfm_repro_checkpoint()
    if repro_ckpt:
        panel._tfm_ckpt_meta = {repro_ckpt: panel._tfm_repro_checkpoint_meta()}
        panel._audit_write_json(
            "artifacts/checkpoints_index.json",
            {
                "root": str(Path(VISION_EXP_DIR).expanduser()),
                "count": 1,
                "repro_mode": panel._tfm_repro_profile(),
                "entries": [
                    {
                        "path": repro_ckpt,
                        **panel._tfm_ckpt_meta[repro_ckpt],
                    }
                ],
            },
        )
        return [repro_ckpt]
    exp_names = (
        "EXP1_SIMPLE_RGB",
        "EXP2_SIMPLE_RGBD",
        "EXP3_RESNET18_RGB_AUGMENT",
        "EXP4_RESNET18_RGBD",
        "EXP1.1_SIMPLEGRASP_RGB",
        "EXP1.2_SIMPLEGRASP_RGBD",
    )
    exp_root = Path(VISION_EXP_DIR).expanduser()
    ckpts: List[str] = []
    ckpt_meta: Dict[str, Dict[str, object]] = {}

    for exp_name in exp_names:
        exp_dir = exp_root / exp_name
        if not exp_dir.exists():
            continue

        best_seed: Optional[int] = None
        best_success: Optional[float] = None
        best_iou: Optional[float] = None
        best_loss: Optional[float] = None
        selection_basis = "val_success"
        summary = exp_dir / "best_epoch_summary.csv"
        if summary.exists():
            best_seed, best_success, best_iou, best_loss, selection_basis = panel._tfm_select_seed_from_summary(exp_name, summary)

        candidates: List[Path] = []
        if best_seed is not None:
            candidates.append(exp_dir / f"seed_{best_seed}" / "checkpoints" / "best.pth")
        candidates.extend(sorted(exp_dir.glob("seed_*/checkpoints/best.pth")))
        chosen = next((p for p in candidates if p.exists()), None)
        if not chosen:
            continue

        ckpt = str(chosen)
        ckpts.append(ckpt)
        ckpt_meta[ckpt] = {
            "experiment": exp_name,
            "seed": best_seed,
            "val_success": best_success,
            "val_iou": best_iou,
            "val_loss": best_loss,
            "selection_basis": selection_basis,
            "role": "auxiliar 4.6.2" if panel._tfm_is_aux_experiment(exp_name) else "oficial",
        }

    panel._tfm_ckpt_meta = ckpt_meta
    panel._audit_write_json(
        "artifacts/checkpoints_index.json",
        {
            "root": str(exp_root),
            "count": len(ckpts),
            "entries": [
                {
                    "path": ckpt,
                    "experiment": ckpt_meta.get(ckpt, {}).get("experiment"),
                    "seed": ckpt_meta.get(ckpt, {}).get("seed"),
                    "val_success": ckpt_meta.get(ckpt, {}).get("val_success"),
                    "val_iou": ckpt_meta.get(ckpt, {}).get("val_iou"),
                    "val_loss": ckpt_meta.get(ckpt, {}).get("val_loss"),
                    "selection_basis": ckpt_meta.get(ckpt, {}).get("selection_basis"),
                    "role": ckpt_meta.get(ckpt, {}).get("role"),
                }
                for ckpt in ckpts
            ],
        },
    )
    return ckpts

def _pick_default_tfm_checkpoint(panel, preferred: str = "") -> str:
    preferred_path = str(Path(preferred).expanduser()) if preferred else ""
    if preferred_path and Path(preferred_path).is_file():
        return preferred_path

    best_ckpt = ""
    best_success = float("-inf")
    for ckpt in panel._tfm_ckpt_options:
        meta = getattr(panel, "_tfm_ckpt_meta", {}).get(ckpt, {})
        try:
            success = float(meta.get("val_success", "nan"))
        except Exception:
            success = float("nan")
        if math.isfinite(success) and success > best_success and Path(ckpt).is_file():
            best_success = success
            best_ckpt = ckpt

    if best_ckpt:
        return best_ckpt

    for ckpt in panel._tfm_ckpt_options:
        if Path(ckpt).is_file():
            return ckpt
    return ""

def _refresh_tfm_checkpoint_options(panel) -> None:
    panel._tfm_ckpt_options = panel._discover_tfm_checkpoints(allow_rgbd=True)
    if panel._tfm_ckpt_options:
        selected_valid = bool(panel._tfm_ckpt_selected and panel._tfm_ckpt_selected in panel._tfm_ckpt_options)
        if not selected_valid:
            panel._tfm_ckpt_selected = panel._pick_default_tfm_checkpoint(preferred=INFER_CKPT)
    if not hasattr(panel, "combo_tfm_experiment"):
        panel._load_experiment_info()
        panel._refresh_science_ui()
        return
    panel._tfm_refreshing_ckpt_combo = True
    panel.combo_tfm_experiment.clear()
    try:
        if panel._tfm_ckpt_options:
            for ckpt in panel._tfm_ckpt_options:
                panel.combo_tfm_experiment.addItem(panel._format_ckpt_label(ckpt), ckpt)
            if panel._tfm_ckpt_selected:
                idx = panel.combo_tfm_experiment.findData(panel._tfm_ckpt_selected)
                if idx >= 0:
                    panel.combo_tfm_experiment.setCurrentIndex(idx)
                    panel._tfm_ckpt_selected = str(panel.combo_tfm_experiment.itemData(idx) or panel._tfm_ckpt_selected)
        else:
            panel.combo_tfm_experiment.addItem("Sin checkpoints encontrados", "")
    finally:
        panel._tfm_refreshing_ckpt_combo = False
    panel._load_experiment_info()
    panel._refresh_science_ui()

def _format_ckpt_label(panel, ckpt_path: str) -> str:
    meta = getattr(panel, "_tfm_ckpt_meta", {}).get(ckpt_path, {}) if hasattr(panel, "_tfm_ckpt_meta") else {}
    exp_name = str(meta.get("experiment") or "").strip()
    if exp_name:
        seed = meta.get("seed")
        success = meta.get("val_success")
        val_loss = meta.get("val_loss")
        selection_basis = str(meta.get("selection_basis") or "val_success").strip()
        success_txt = "--"
        try:
            if success is not None and math.isfinite(float(success)):
                success_txt = f"{float(success) * 100.0:.1f}%"
        except Exception:
            success_txt = "--"
        loss_txt = "--"
        try:
            if val_loss is not None and math.isfinite(float(val_loss)):
                loss_txt = f"{float(val_loss):.3f}"
        except Exception:
            loss_txt = "--"
        if isinstance(seed, (int, float)) and math.isfinite(float(seed)):
            seed_txt = f"seed_{int(float(seed))}"
        else:
            seed_txt = "seed_?"
        metric_txt = f"acierto {success_txt}"
        if selection_basis == "val_loss":
            metric_txt = f"val_loss {loss_txt}"
        return f"{exp_name} | mejor {seed_txt} | {metric_txt}"
    path = Path(ckpt_path)
    if path.parent.name == "checkpoints" and path.parent.parent.name:
        return f"{path.parent.parent.name}/{path.name}"
    return path.name

def _tfm_get_ckpt_path(panel) -> str:
    if hasattr(panel, "combo_tfm_experiment"):
        data = panel.combo_tfm_experiment.currentData()  # type: ignore[attr-defined]
        if isinstance(data, str) and data:
            return data
    return panel._tfm_ckpt_selected or ""

def _tfm_apply_experiment(panel) -> None:
    panel._log_button("TFM Aplicar experimento")
    ckpt_path = panel._tfm_get_ckpt_path()
    if not ckpt_path:
        panel._set_status("TFM: sin checkpoint seleccionado", error=True)
        return
    panel._tfm_ckpt_selected = ckpt_path
    panel._tfm_experiment_applied = True
    panel._tfm_overlay_focus_active = True
    panel._load_experiment_info()
    panel._refresh_science_ui()
    panel._refresh_controls()
    ckpt_info = {
        "path": ckpt_path,
        "exists": False,
        "size_bytes": None,
        "sha256": "",
    }
    try:
        p = Path(ckpt_path)
        if p.is_file():
            ckpt_info["exists"] = True
            ckpt_info["size_bytes"] = p.stat().st_size
            ckpt_info["sha256"] = panel._sha256_file(str(p))
    except Exception:
        pass
    if panel.tfm_module:
        panel.tfm_module.load_model(ckpt_path)
        err = panel.tfm_module.last_error()
        if err:
            panel._refresh_camera_display()
            panel._set_status(f"TFM: checkpoint aplicado (sin carga de modelo: {err})", error=False)
            panel._audit_append(
                "logs/apply_experiment.log",
                f"[TFM] apply_experiment FAIL ckpt={ckpt_path} err={err}",
            )
            panel._tfm_preprocessed_cache = None
            return
    model_info = panel.tfm_module.model_info() if panel.tfm_module else {}
    panel._load_experiment_info()
    panel._audit_write_json(
        "artifacts/tfm_session_last.json",
        {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "status": "OK",
            "selection_policy": panel._exp_info.get("selection_policy", ""),
            "postprocess_policy": panel._exp_info.get("postprocess_policy", ""),
            "experiment": panel._exp_info.get("experiment", "--"),
            "experiment_base": panel._exp_info.get("experiment_base", "--"),
            "seed": panel._exp_info.get("seed", "--"),
            "model": panel._exp_info.get("model", "--"),
            "modality": panel._exp_info.get("modality", "--"),
            "epoch": panel._exp_info.get("epoch", "--"),
            "config_path": panel._exp_info.get("config_path", ""),
            "weights_path": panel._exp_info.get("weights_path", ckpt_path),
            "val_success_pct": panel._exp_info.get("val_success_pct", "--"),
            "val_iou": panel._exp_info.get("val_iou", "--"),
            "checkpoint": ckpt_info,
            "model_info": model_info,
        },
    )
    panel._refresh_science_ui()
    try:
        camera_ctrl = getattr(panel, "_camera_ctrl", None)
        if camera_ctrl is not None:
            camera_ctrl._ensure_depth_subscription()
            camera_ctrl._sync_from_worker_snapshot(now=_runtime_time())
    except Exception:
        pass
    panel._refresh_camera_display()
    panel._set_status("TFM: experimento aplicado", error=False)
    panel._audit_append(
        "logs/apply_experiment.log",
        "[TFM] apply_experiment OK "
        f"ckpt={ckpt_path} size={ckpt_info['size_bytes']} sha256={ckpt_info['sha256']} "
        f"selection_policy={panel._exp_info.get('selection_policy', '')} "
        f"postprocess_policy={panel._exp_info.get('postprocess_policy', '')} "
        f"model={model_info}",
    )

def _tfm_reset_grasp(panel) -> None:
    panel._log_button("TFM Reset")
    panel._tfm_experiment_applied = False
    panel._tfm_overlay_focus_active = False
    panel._last_grasp_px = None
    panel._last_grasp_world = None
    panel._last_grasp_base = None
    panel._last_grasp_update_ts = 0.0
    panel._last_grasp_selection_name = ""
    panel._last_cornell_ref = None
    panel._last_cornell_reason = "Inferir y seleccionar un objeto"
    panel._tfm_visual_compare_enabled = False
    panel._last_grasp_frame = ""
    panel._last_grasp_source = ""
    panel._last_infer_image_path = ""
    panel._last_infer_output_path = ""
    panel._last_cornell = None
    panel._last_tfm_postprocess_note = ""
    if panel.tfm_module:
        panel.tfm_module.reset()
    if hasattr(panel, "btn_tfm_grasp_object"):
        panel.btn_tfm_grasp_object.setEnabled(False)
    panel._refresh_science_ui()
    panel._refresh_controls()
    panel._refresh_camera_display()
    panel._set_status("TFM: reset", error=False)
    panel._audit_append("logs/reset.log", "[TFM] reset OK")

def _load_experiment_info(panel) -> None:
    info: Dict[str, object] = {
        "model": "--",
        "modality": "--",
        "experiment": "--",
        "experiment_base": "--",
        "seed": "--",
        "epoch": "--",
        "val_success_pct": "--",
        "val_iou": "--",
        "weights": "--",
        "weights_path": "",
        "config_path": "",
        "selection_policy": "",
        "postprocess_policy": panel._tfm_postprocess_policy_label(),
    }
    ckpt_value = panel._tfm_ckpt_selected or INFER_CKPT
    ckpt_path = Path(ckpt_value).expanduser() if ckpt_value else None
    if ckpt_path:
        info["weights"] = ckpt_path.name
        info["weights_path"] = str(ckpt_path)
    if panel._tfm_repro_profile() == "exp3_seed0":
        info["selection_policy"] = "modo reproducción TFM (EXP3 seed_0)"
    else:
        info["selection_policy"] = "mejor checkpoint automático"
    seed_dir = None
    exp_dir = None
    if ckpt_path and ckpt_path.exists():
        if ckpt_path.parent.name == "checkpoints":
            seed_dir = ckpt_path.parent.parent
        else:
            seed_dir = ckpt_path.parent
    if seed_dir:
        seed_match = re.match(r"seed_(\d+)", seed_dir.name)
        if seed_match:
            info["seed"] = seed_match.group(1)
            exp_dir = seed_dir.parent
        else:
            exp_dir = seed_dir
    if exp_dir and exp_dir.name:
        info["experiment"] = exp_dir.name
        info["experiment_base"] = exp_dir.name
    cfg_dir = seed_dir or exp_dir
    if cfg_dir:
        cfg_path = cfg_dir / "config_snapshot.yaml"
        if not cfg_path.exists():
            cfg_path = cfg_dir / "config_used.yaml"
        info["config_path"] = str(cfg_path)
        if yaml is not None and cfg_path.exists():
            try:
                data = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
            except Exception:
                data = {}
            if isinstance(data, dict):
                model_cfg = data.get("model", {}) if isinstance(data.get("model", {}), dict) else {}
                data_cfg = data.get("data", {}) if isinstance(data.get("data", {}), dict) else {}
                model_name = str(model_cfg.get("name", "")).strip()
                if model_name:
                    info["model"] = model_name
                modality = str(data_cfg.get("modality", "")).strip().lower()
                if modality:
                    info["modality"] = "RGB-D" if modality == "rgbd" else modality.upper()
                elif data_cfg.get("use_depth"):
                    info["modality"] = "RGB-D"
    meta = getattr(panel, "_tfm_ckpt_meta", {}).get(str(ckpt_path), {}) if ckpt_path else {}
    if isinstance(meta, dict):
        success = meta.get("val_success")
        iou = meta.get("val_iou")
        val_loss = meta.get("val_loss")
        selection_basis = str(meta.get("selection_basis") or "").strip()
        try:
            if success is not None and math.isfinite(float(success)):
                info["val_success_pct"] = f"{float(success) * 100.0:.1f}%"
        except Exception:
            pass
        try:
            if iou is not None and math.isfinite(float(iou)):
                info["val_iou"] = f"{float(iou):.3f}"
        except Exception:
            pass
        if selection_basis == "val_loss":
            try:
                if val_loss is not None and math.isfinite(float(val_loss)):
                    info["selection_policy"] = f"{info['selection_policy']} | semilla elegida por val_loss={float(val_loss):.3f}"
            except Exception:
                pass
    if ckpt_path and ckpt_path.exists():
        try:
            import torch  # type: ignore
        except Exception:
            torch = None
        if torch is not None:
            try:
                ckpt = torch.load(str(ckpt_path), map_location="cpu")
                if isinstance(ckpt, dict):
                    epoch = ckpt.get("epoch")
                    if epoch is None and isinstance(ckpt.get("metrics"), dict):
                        epoch = ckpt.get("metrics", {}).get("epoch")
                    if epoch is not None:
                        info["epoch"] = str(int(epoch))
            except Exception:
                pass
    panel._exp_info = info

def _format_value(panel, value: Optional[float], fmt: str, suffix: str = "") -> str:
    if value is None or not math.isfinite(float(value)):
        return "--"
    return f"{fmt.format(value)}{suffix}"

def _refresh_science_ui(panel) -> None:
    if panel.lbl_cornell_iou:
        iou = panel._last_cornell.get("iou") if panel._last_cornell else None
        panel.lbl_cornell_iou.setText(panel._format_value(iou, "{:.3f}"))
    if panel.lbl_cornell_theta:
        dtheta = panel._last_cornell.get("dtheta") if panel._last_cornell else None
        panel.lbl_cornell_theta.setText(panel._format_value(dtheta, "{:.1f}", "°"))
    if panel.lbl_cornell_success:
        success = panel._last_cornell.get("success") if panel._last_cornell else None
        if success is None:
            panel.lbl_cornell_success.setText("--")
        else:
            panel.lbl_cornell_success.setText("SUCCESS" if success else "FAIL")
    if panel.lbl_cornell_note:
        if panel._cornell_metrics:
            base_note = "Evaluación geométrica en simulación. No validación física."
            detail = str(panel._last_cornell_reason or "").strip()
            postprocess_note = str(getattr(panel, "_last_tfm_postprocess_note", "") or "").strip()
            detail_parts = [part for part in (detail, postprocess_note) if part]
            detail_txt = " ".join(detail_parts)
            panel.lbl_cornell_note.setText(f"{base_note} {detail_txt}" if detail_txt else base_note)
        else:
            detail = panel._cornell_metrics_err or "dependencia no disponible"
            panel.lbl_cornell_note.setText(f"Cornell offline: {detail}")
    if panel.lbl_exp_model:
        panel.lbl_exp_model.setText(str(panel._exp_info.get("model", "--")))
    if panel.lbl_exp_modality:
        panel.lbl_exp_modality.setText(str(panel._exp_info.get("modality", "--")))
    if panel.lbl_exp_name:
        panel.lbl_exp_name.setText(str(panel._exp_info.get("experiment", "--")))
    if panel.lbl_exp_seed:
        panel.lbl_exp_seed.setText(str(panel._exp_info.get("seed", "--")))
    if panel.lbl_exp_epoch:
        panel.lbl_exp_epoch.setText(str(panel._exp_info.get("epoch", "--")))
    if panel.lbl_exp_success:
        panel.lbl_exp_success.setText(str(panel._exp_info.get("val_success_pct", "--")))
    if panel.lbl_exp_iou:
        panel.lbl_exp_iou.setText(str(panel._exp_info.get("val_iou", "--")))
    if panel.lbl_exp_weights:
        weights_text = str(panel._exp_info.get("weights", "--"))
        details: List[str] = []
        selection_policy = str(panel._exp_info.get("selection_policy", "") or "").strip()
        postprocess_policy = str(panel._exp_info.get("postprocess_policy", "") or "").strip()
        if selection_policy:
            details.append(selection_policy)
        if postprocess_policy:
            details.append(postprocess_policy)
        if details:
            weights_text = f"{weights_text} ({' | '.join(details)})"
        panel.lbl_exp_weights.setText(weights_text)
    if panel.lbl_perf_infer:
        avg = panel._mean_history(panel._perf_infer_hist)
        inst = panel._perf_infer_ms
        if inst is None:
            panel.lbl_perf_infer.setText("--")
        else:
            avg_txt = f"{avg:.1f} ms" if avg is not None else "--"
            panel.lbl_perf_infer.setText(f"{inst:.1f} ms (avg {avg_txt})")
    if panel.lbl_perf_total:
        avg = panel._mean_history(panel._perf_total_hist)
        inst = panel._perf_total_ms
        if inst is None:
            panel.lbl_perf_total.setText("--")
        else:
            avg_txt = f"{avg:.1f} ms" if avg is not None else "--"
            panel.lbl_perf_total.setText(f"{inst:.1f} ms (avg {avg_txt})")
    if panel.lbl_perf_fps:
        inst = panel._camera_last_fps
        avg = panel._perf_fps_avg if panel._perf_fps_hist else None
        if inst <= 0.0:
            panel.lbl_perf_fps.setText("--")
        else:
            avg_txt = f"{avg:.1f}" if avg is not None else "--"
            panel.lbl_perf_fps.setText(f"{inst:.1f} (avg {avg_txt})")
    if panel.lbl_grasp_img:
        if panel._last_grasp_px:
            g = panel._last_grasp_px
            panel.lbl_grasp_img.setText(
                f"cx={g.get('cx', 0.0):.1f}, cy={g.get('cy', 0.0):.1f}, "
                f"w={g.get('w', 0.0):.1f}, h={g.get('h', 0.0):.1f}, "
                f"theta={g.get('angle_deg', 0.0):.1f}°"
            )
        else:
            panel.lbl_grasp_img.setText("--")
    if panel.lbl_grasp_world:
        if panel._last_grasp_base:
            g = panel._last_grasp_base
            yaw = g.get("yaw_deg")
            yaw_txt = f"{yaw:.1f}°" if yaw is not None else "--"
            panel.lbl_grasp_world.setText(
                f"x={g.get('x', 0.0):.3f}, y={g.get('y', 0.0):.3f}, "
                f"z={g.get('z', 0.0):.3f}, yaw={yaw_txt}"
            )
        else:
            panel.lbl_grasp_world.setText("--")
    if panel.lbl_grasp_frame:
        image_frame = panel.camera_topic or "image"
        base_frame = panel._business_base_frame()
        panel.lbl_grasp_frame.setText(f"image={image_frame} | base={base_frame}")

def _world_to_pixel(panel, x: float, y: float, z: float, w: int, h: int) -> Optional[Tuple[int, int]]:
    pix = world_xyz_to_pixel(x, y, z, w, h)
    if not pix:
        pix = table_xy_to_pixel(x, y, w, h)
    return pix

def _world_to_pixel_diag(panel,
    x: float,
    y: float,
    z: float,
    w: int,
    h: int,
) -> Tuple[Optional[Tuple[int, int]], Optional[Tuple[float, float]], str]:
    pix_float = world_xyz_to_pixel_float(x, y, z, w, h)
    if pix_float is not None:
        pix_int = world_xyz_to_pixel(x, y, z, w, h)
        return pix_int, pix_float, "world_xyz"
    pix_float = table_xy_to_pixel_float(x, y, w, h)
    if pix_float is not None:
        pix_int = table_xy_to_pixel(x, y, w, h)
        return pix_int, pix_float, "table_xy"
    return None, None, "none"

def _build_reference_grasp(panel, frame_w: int, frame_h: int) -> Optional[Dict[str, float]]:
    if not panel._selected_object:
        return None
    if frame_w <= 0 or frame_h <= 0:
        return None
    panel._load_sdf_geometry_cache()
    sdf = panel._sdf_model_cache.get(panel._selected_object, {})
    geom_type = sdf.get("type")
    size = sdf.get("size")
    radius = sdf.get("radius")
    width_m = None
    height_m = None
    if geom_type == "box" and size and len(size) == 3:
        width_m = float(size[0])
        height_m = float(size[1])
    elif geom_type in ("cylinder", "sphere") and radius:
        width_m = float(radius) * 2.0
        height_m = float(radius) * 2.0
    if not width_m or not height_m:
        return None
    pose_source = "selection"
    live_pose = None
    if getattr(panel, "_ros_worker_started", False) and getattr(panel, "ros_worker", None) is not None:
        try:
            poses, _pose_ts = panel.ros_worker.pose_snapshot()
        except Exception:
            poses = {}
        live_pose = poses.get(panel._selected_object)
    if live_pose is not None and len(live_pose) >= 3:
        wx, wy, wz = float(live_pose[0]), float(live_pose[1]), float(live_pose[2])
        panel._selected_world = (wx, wy, wz)
        selected_base = panel._ensure_base_coords(
            (wx, wy, wz),
            panel._world_frame_last_first(),
            timeout_sec=0.35,
        )
        if selected_base is not None:
            panel._selected_base = selected_base
            panel._selected_base_frame = panel._business_base_frame()
        pose_source = "pose_snapshot"
    elif panel._selected_world:
        wx, wy, wz = panel._selected_world
    elif panel._selected_base:
        wx, wy, wz = base_to_world(
            float(panel._selected_base[0]),
            float(panel._selected_base[1]),
            float(panel._selected_base[2]),
        )
        pose_source = "selected_base"
    else:
        return None
    z = wz if wz and wz > 0 else panel._resolve_table_top_z()
    if not panel._selected_px or panel._selected_px[0] < 0 or panel._selected_px[1] < 0:
        selected_px, _selected_px_float, _selected_px_src = panel._world_to_pixel_diag(wx, wy, z, frame_w, frame_h)
        if selected_px:
            panel._selected_px = (int(selected_px[0]), int(selected_px[1]))
    center_px, center_px_float, center_src = panel._world_to_pixel_diag(wx, wy, z, frame_w, frame_h)
    if not center_px or not center_px_float:
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_ref_diag "
            f"selected={panel._selected_object or 'none'} pose_source={pose_source} "
            f"geom={geom_type or 'unknown'} size_m=({float(width_m or 0.0):.4f},{float(height_m or 0.0):.4f}) "
            f"world=({float(wx):.4f},{float(wy):.4f},{float(z):.4f}) center_src={center_src}",
        )
        return None
    left_px, left_px_float, left_src = panel._world_to_pixel_diag(wx - width_m / 2.0, wy, z, frame_w, frame_h)
    right_px, right_px_float, right_src = panel._world_to_pixel_diag(wx + width_m / 2.0, wy, z, frame_w, frame_h)
    down_px, down_px_float, down_src = panel._world_to_pixel_diag(wx, wy - height_m / 2.0, z, frame_w, frame_h)
    up_px, up_px_float, up_src = panel._world_to_pixel_diag(wx, wy + height_m / 2.0, z, frame_w, frame_h)
    if (
        not left_px or not right_px or not down_px or not up_px
        or not left_px_float or not right_px_float or not down_px_float or not up_px_float
    ):
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_ref_diag "
            f"selected={panel._selected_object or 'none'} pose_source={pose_source} "
            f"geom={geom_type or 'unknown'} size_m=({float(width_m or 0.0):.4f},{float(height_m or 0.0):.4f}) "
            f"world=({float(wx):.4f},{float(wy):.4f},{float(z):.4f}) "
            f"center_src={center_src} left_src={left_src} right_src={right_src} down_src={down_src} up_src={up_src} "
            f"center={center_px_float} left={left_px_float} right={right_px_float} down={down_px_float} up={up_px_float}",
        )
        return None
    w_px = int(round(math.hypot(right_px[0] - left_px[0], right_px[1] - left_px[1])))
    h_px = int(round(math.hypot(up_px[0] - down_px[0], up_px[1] - down_px[1])))
    w_px_float = math.hypot(
        float(right_px_float[0]) - float(left_px_float[0]),
        float(right_px_float[1]) - float(left_px_float[1]),
    )
    h_px_float = math.hypot(
        float(up_px_float[0]) - float(down_px_float[0]),
        float(up_px_float[1]) - float(down_px_float[1]),
    )
    if w_px <= 1 or h_px <= 1 or w_px_float <= 2.0 or h_px_float <= 2.0:
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_ref_diag "
            f"selected={panel._selected_object or 'none'} pose_source={pose_source} "
            f"geom={geom_type or 'unknown'} size_m=({float(width_m or 0.0):.4f},{float(height_m or 0.0):.4f}) "
            f"world=({float(wx):.4f},{float(wy):.4f},{float(z):.4f}) "
            f"srcs=center:{center_src},left:{left_src},right:{right_src},down:{down_src},up:{up_src} "
            f"center_f=({center_px_float[0]:.2f},{center_px_float[1]:.2f}) "
            f"left_f=({left_px_float[0]:.2f},{left_px_float[1]:.2f}) right_f=({right_px_float[0]:.2f},{right_px_float[1]:.2f}) "
            f"down_f=({down_px_float[0]:.2f},{down_px_float[1]:.2f}) up_f=({up_px_float[0]:.2f},{up_px_float[1]:.2f}) "
            f"span_float=({w_px_float:.2f},{h_px_float:.2f}) "
            f"center_i={center_px} left_i={left_px} right_i={right_px} down_i={down_px} up_i={up_px} "
            f"span_int=({w_px},{h_px})",
        )
    return {
        "cx": float(center_px_float[0]),
        "cy": float(center_px_float[1]),
        "w": float(max(1.0, w_px_float)),
        "h": float(max(1.0, h_px_float)),
        "angle_deg": 0.0,
    }

def _grasp_projection_z_target(panel) -> Tuple[float, str]:
    table_top = float(panel._resolve_table_top_z())
    if panel._selected_object:
        obj_pos = get_object_position(panel._selected_object)
        if obj_pos and len(obj_pos) >= 3:
            try:
                obj_z = float(obj_pos[2])
            except Exception:
                obj_z = table_top
            if math.isfinite(obj_z) and obj_z > 0.0:
                return obj_z, f"selected_object:{panel._selected_object}"
    if panel._selected_world and len(panel._selected_world) >= 3:
        try:
            world_z = float(panel._selected_world[2])
        except Exception:
            world_z = table_top
        if math.isfinite(world_z) and world_z > 0.0:
            return world_z, "selected_world"
    return table_top, "table_top"

def _compute_world_grasp(panel, frame_w: int, frame_h: int) -> Optional[Dict[str, float]]:
    if not panel._last_grasp_px:
        return None
    if frame_w <= 0 or frame_h <= 0:
        return None
    cx = panel._last_grasp_px.get("cx", 0.0)
    cy = panel._last_grasp_px.get("cy", 0.0)
    angle_deg = panel._last_grasp_px.get("angle_deg", 0.0)
    table_top = panel._resolve_table_top_z()
    proj_z_target, proj_z_source = panel._grasp_projection_z_target()
    px = int(round(cx))
    py = int(round(cy))
    wx, wy = pixel_to_table_xy(px, py, frame_w, frame_h, z_target=proj_z_target)
    step = 10.0
    dx = math.cos(math.radians(angle_deg)) * step
    dy = math.sin(math.radians(angle_deg)) * step
    wx2, wy2 = pixel_to_table_xy(
        int(round(cx + dx)),
        int(round(cy + dy)),
        frame_w,
        frame_h,
        z_target=proj_z_target,
    )
    yaw_deg = None
    if wx2 is not None and wy2 is not None:
        yaw_deg = math.degrees(math.atan2(wy2 - wy, wx2 - wx))
    return {
        "x": float(wx),
        "y": float(wy),
        "z": float(proj_z_target),
        "yaw_deg": yaw_deg,
        "proj_z_target": float(proj_z_target),
        "proj_z_source": proj_z_source,
        "table_top_z": float(table_top),
        "grasp_semantics": "projection_surface",
    }

def _world_grasp_to_base(panel, world_grasp: Optional[Dict[str, float]]) -> Optional[Dict[str, float]]:
    if not world_grasp:
        return None
    base_coords = panel._ensure_base_coords(
        (
            float(world_grasp.get("x", 0.0)),
            float(world_grasp.get("y", 0.0)),
            float(world_grasp.get("z", 0.0)),
        ),
        panel._world_frame_config_first(),
        timeout_sec=0.35,
    )
    if base_coords is None:
        return None
    return {
        "x": float(base_coords[0]),
        "y": float(base_coords[1]),
        "z": float(base_coords[2]),
        "yaw_deg": float(world_grasp.get("yaw_deg", 0.0) or 0.0),
        "proj_z_target": float(world_grasp.get("proj_z_target", world_grasp.get("z", 0.0)) or 0.0),
        "proj_z_source": str(world_grasp.get("proj_z_source", "unknown") or "unknown"),
        "table_top_z_world": float(world_grasp.get("table_top_z", 0.0) or 0.0),
        "grasp_semantics": str(world_grasp.get("grasp_semantics", "projection_surface") or "projection_surface"),
    }

def _update_cornell_metrics(panel, pred: Dict[str, float], ref: Dict[str, float]) -> None:
    if not panel._cornell_metrics:
        panel._last_cornell = None
        return
    try:
        angle_diff_deg = panel._cornell_metrics["angle_diff_deg"]
        compute_grasp_success = panel._cornell_metrics["compute_grasp_success"]
        grasp_iou = panel._cornell_metrics["grasp_iou"]
        pred_params = [pred["cx"], pred["cy"], pred["w"], pred["h"], pred["angle_deg"]]
        ref_params = [ref["cx"], ref["cy"], ref["w"], ref["h"], ref["angle_deg"]]
        iou = grasp_iou(pred_params, ref_params)
        dtheta = angle_diff_deg(float(pred_params[4]), float(ref_params[4]))
        success = bool(compute_grasp_success(pred_params, ref_params, iou_thresh=0.25, angle_thresh=30.0))
        panel._last_cornell = {"iou": float(iou), "dtheta": float(dtheta), "success": success}
    except Exception:
        panel._last_cornell = None

def _refresh_cornell_metrics(panel, frame_w: int, frame_h: int) -> None:
    panel._last_cornell = None
    panel._last_cornell_ref = None
    if not panel._cornell_metrics:
        panel._last_cornell_reason = f"Cornell offline: {panel._cornell_metrics_err or 'dependencia no disponible'}"
        return
    if not panel._last_grasp_px:
        panel._last_cornell_reason = "Inferir agarre primero"
        return
    if not panel._selected_object:
        panel._last_cornell_reason = "Selecciona un objeto para comparar con referencia Cornell"
        return
    if frame_w <= 0 or frame_h <= 0:
        panel._last_cornell_reason = "Sin frame válido para proyectar referencia"
        return
    ref = panel._build_reference_grasp(frame_w, frame_h)
    if not ref:
        panel._last_cornell_reason = f"No se pudo proyectar referencia para {panel._selected_object}"
        return
    panel._last_cornell_ref = ref
    panel._update_cornell_metrics(panel._last_grasp_px, ref)
    if panel._last_cornell:
        pred = panel._last_grasp_px
        panel._last_cornell_reason = (
            f"Ref {panel._selected_object}: pred {pred['w']:.1f}x{pred['h']:.1f}px vs ref {ref['w']:.1f}x{ref['h']:.1f}px"
        )
        panel._audit_append(
            "logs/infer.log",
            "[TFM] infer_ref "
            f"selected={panel._selected_object or 'none'} "
            f"pred={{'w': {pred['w']:.2f}, 'h': {pred['h']:.2f}, 'angle_deg': {pred['angle_deg']:.2f}}} "
            f"ref={{'cx': {ref['cx']:.2f}, 'cy': {ref['cy']:.2f}, 'w': {ref['w']:.2f}, 'h': {ref['h']:.2f}, 'angle_deg': {ref['angle_deg']:.2f}}}",
        )
    else:
        panel._last_cornell_reason = f"No se pudieron calcular métricas para {panel._selected_object}"

# --- TFM mode selector methods (extracted from panel_v2.py) ---
def _tfm_repro_profile_env(panel) -> str:
    raw = _get_panel_tfm_params().repro_mode
    if raw in ("1", "true", "yes", "on", "exp3_seed0", "pdf_main_case", "tfm_pdf_main_case"):
        return "exp3_seed0"
    return ""

def _tfm_repro_profile(panel) -> str:
    if hasattr(panel, "chk_tfm_repro_mode") and panel.chk_tfm_repro_mode is not None:
        try:
            if bool(panel.chk_tfm_repro_mode.isChecked()):
                return "exp3_seed0"
            return ""
        except Exception:
            pass
    return panel._tfm_repro_profile_env()

def _tfm_raw_output_env_enabled(panel) -> bool:
    raw = _get_panel_tfm_params().raw_output
    return raw in ("1", "true", "yes", "on", "raw", "disable_postprocess")

def _tfm_postprocess_enabled(panel) -> bool:
    if hasattr(panel, "chk_tfm_raw_output") and panel.chk_tfm_raw_output is not None:
        try:
            return not bool(panel.chk_tfm_raw_output.isChecked())
        except Exception:
            pass
    return not panel._tfm_raw_output_env_enabled()

def _tfm_postprocess_policy_label(panel) -> str:
    return "ajustes panel habilitados" if panel._tfm_postprocess_enabled() else "raw sin ajustes panel"

def _on_tfm_repro_mode_changed(panel) -> None:
    mode = panel._tfm_repro_profile() or "best_checkpoint_auto"
    panel._tfm_experiment_applied = False
    panel._refresh_tfm_checkpoint_options()
    panel._set_status("TFM: modo de selección actualizado; reaplica experimento", error=False)
    panel._audit_append(
        "logs/apply_experiment.log",
        f"[TFM] selector_mode_changed mode={mode}",
    )
    panel._refresh_controls()

def _on_tfm_postprocess_mode_changed(panel) -> None:
    policy = panel._tfm_postprocess_policy_label()
    panel._tfm_experiment_applied = False
    panel._set_status(f"TFM: salida configurada en {policy}; reaplica experimento", error=False)
    panel._audit_append(
        "logs/infer.log",
        f"[TFM] postprocess_mode_changed policy={policy}",
    )
    panel._load_experiment_info()
    panel._refresh_science_ui()
    panel._refresh_controls()

def _on_tfm_checkpoint_selection_changed(panel) -> None:
    if bool(getattr(panel, "_tfm_refreshing_ckpt_combo", False)):
        return
    ckpt_path = panel._tfm_get_ckpt_path()
    panel._tfm_ckpt_selected = ckpt_path
    panel._tfm_experiment_applied = False
    panel._load_experiment_info()
    panel._refresh_science_ui()
    panel._refresh_controls()
    label = Path(ckpt_path).name if ckpt_path else "sin checkpoint"
    panel._set_status(f"TFM: checkpoint seleccionado ({label}); reaplica experimento", error=False)
    panel._audit_append(
        "logs/apply_experiment.log",
        f"[TFM] checkpoint_selection_changed ckpt={ckpt_path}",
    )

def _tfm_apply_memoria_case(panel) -> None:
    panel._log_button("TFM Caso Memoria")
    changed = False
    if hasattr(panel, "chk_tfm_repro_mode") and panel.chk_tfm_repro_mode is not None:
        try:
            if not bool(panel.chk_tfm_repro_mode.isChecked()):
                panel.chk_tfm_repro_mode.setChecked(True)
                changed = True
        except Exception:
            pass
    if hasattr(panel, "chk_tfm_raw_output") and panel.chk_tfm_raw_output is not None:
        try:
            if not bool(panel.chk_tfm_raw_output.isChecked()):
                panel.chk_tfm_raw_output.setChecked(True)
                changed = True
        except Exception:
            pass
    panel._audit_append(
        "logs/apply_experiment.log",
        "[TFM] memoria_case preset=EXP3_RESNET18_RGB_AUGMENT/seed_0 postprocess=raw "
        f"changed={str(bool(changed)).lower()}",
    )
    panel._set_status("TFM: preset memoria activado; aplicando experimento", error=False)
    panel._tfm_apply_experiment()

def _tfm_repro_checkpoint(panel) -> str:
    if panel._tfm_repro_profile() != "exp3_seed0":
        return ""
    ckpt = (
        Path(VISION_EXP_DIR).expanduser()
        / "EXP3_RESNET18_RGB_AUGMENT"
        / "seed_0"
        / "checkpoints"
        / "best.pth"
    )
    return str(ckpt) if ckpt.is_file() else ""

def _tfm_repro_checkpoint_meta(panel) -> Dict[str, object]:
    meta: Dict[str, object] = {
        "experiment": "EXP3_RESNET18_RGB_AUGMENT",
        "seed": 0,
        "val_success": None,
        "val_iou": None,
        "val_loss": None,
        "selection_basis": "val_success",
        "role": "oficial",
    }
    summary = (
        Path(VISION_EXP_DIR).expanduser()
        / "EXP3_RESNET18_RGB_AUGMENT"
        / "best_epoch_summary.csv"
    )
    if not summary.exists():
        return meta
    try:
        with summary.open("r", encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                try:
                    seed_val = int(float(row.get("seed", "nan")))
                except Exception:
                    continue
                if seed_val != 0:
                    continue
                try:
                    success = float(row.get("val_success", "nan"))
                    meta["val_success"] = success if math.isfinite(success) else None
                except Exception:
                    pass
                try:
                    iou = float(row.get("val_iou", "nan"))
                    meta["val_iou"] = iou if math.isfinite(iou) else None
                except Exception:
                    pass
                break
    except Exception:
        pass
    return meta

def _tfm_is_aux_experiment(panel, exp_name: str) -> bool:
    return exp_name.startswith("EXP1.1_") or exp_name.startswith("EXP1.2_")

def _tfm_select_seed_from_summary(panel, exp_name: str, summary_path: Path) -> Tuple[Optional[int], Optional[float], Optional[float], Optional[float], str]:
    best_seed: Optional[int] = None
    best_success: Optional[float] = None
    best_iou: Optional[float] = None
    best_loss: Optional[float] = None
    selection_basis = "val_success"
    rows: List[Dict[str, float]] = []

    try:
        with summary_path.open("r", encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                try:
                    seed_val = float(row.get("seed", "nan"))
                except Exception:
                    continue
                if not math.isfinite(seed_val):
                    continue
                parsed: Dict[str, float] = {"seed": float(int(seed_val))}
                for key in ("val_success", "val_iou", "val_loss"):
                    try:
                        value = float(row.get(key, "nan"))
                    except Exception:
                        value = float("nan")
                    parsed[key] = value
                rows.append(parsed)
    except Exception:
        return best_seed, best_success, best_iou, best_loss, selection_basis

    if not rows:
        return best_seed, best_success, best_iou, best_loss, selection_basis

    rows_with_success = [row for row in rows if math.isfinite(row["val_success"])]
    positive_success = [row for row in rows_with_success if row["val_success"] > 0.0]

    chosen: Optional[Dict[str, float]] = None
    if positive_success:
        chosen = sorted(
            positive_success,
            key=lambda row: (
                row["val_success"],
                row["val_iou"] if math.isfinite(row["val_iou"]) else float("-inf"),
                -(row["val_loss"] if math.isfinite(row["val_loss"]) else float("inf")),
            ),
            reverse=True,
        )[0]
    elif panel._tfm_is_aux_experiment(exp_name):
        rows_with_loss = [row for row in rows if math.isfinite(row["val_loss"])]
        if rows_with_loss:
            chosen = min(rows_with_loss, key=lambda row: row["val_loss"])
            selection_basis = "val_loss"
    elif rows_with_success:
        chosen = max(rows_with_success, key=lambda row: row["val_success"])

    if chosen is None:
        chosen = rows[0]

    best_seed = int(chosen["seed"])
    best_success = chosen["val_success"] if math.isfinite(chosen["val_success"]) else None
    best_iou = chosen["val_iou"] if math.isfinite(chosen["val_iou"]) else None
    best_loss = chosen["val_loss"] if math.isfinite(chosen["val_loss"]) else None
    return best_seed, best_success, best_iou, best_loss, selection_basis


def _save_episode(panel) -> None:
    pass  # placeholder — episode saving not yet implemented

