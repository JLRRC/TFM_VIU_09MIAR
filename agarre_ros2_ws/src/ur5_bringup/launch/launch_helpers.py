#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/launch_helpers.py
# Contenido: Pure Python helpers for ur5_stack.launch.py — no launch framework deps.
# Uso breve: Imported by ur5_stack.launch.py; all functions are unit-testable.
"""Pure launch helpers: world-name extraction, YAML patching, model copy, env defaults."""

from __future__ import annotations

import os
import re
import shutil
import time
from typing import Any


# ---------------------------------------------------------------------------
# World / Gazebo setup helpers
# ---------------------------------------------------------------------------

def resolve_world_name(world_file: str, default: str = "ur5_mesa_objetos") -> str:
    """Extract <world name="..."> from *world_file*, returning *default* on failure."""
    try:
        with open(world_file, "r", encoding="utf-8") as f:
            data = f.read()
        match = re.search(r'<world name="([^"]+)">', data)
        if match:
            return match.group(1)
    except Exception:
        pass
    return default


def resolve_gz_partition(log_dir: str) -> str:
    """Return GZ_PARTITION env var, or generate one and persist it to *log_dir*."""
    gz_partition = os.environ.get("GZ_PARTITION", "").strip()
    if not gz_partition:
        gz_partition = f"ur5pro_{int(time.time())}"
    try:
        with open(os.path.join(log_dir, "gz_partition.txt"), "w", encoding="utf-8") as f:
            f.write(gz_partition)
    except Exception:
        pass
    return gz_partition


def patch_bridge_yaml(base_yaml: str, runtime_yaml: str, world_name: str) -> str:
    """Copy *base_yaml* to *runtime_yaml*, replacing the placeholder world name.

    Returns the path actually used (runtime_yaml on success, base_yaml on error).
    """
    try:
        with open(base_yaml, "r", encoding="utf-8") as f:
            text = f.read()
        if world_name:
            text = text.replace("/world/ur5_mesa_objetos/", f"/world/{world_name}/")
        with open(runtime_yaml, "w", encoding="utf-8") as f:
            f.write(text)
        return runtime_yaml
    except Exception:
        return base_yaml


def copy_runtime_model(src: str, dst: str) -> None:
    """Copy model directory *src* → *dst* (dirs_exist_ok). Silently ignores errors."""
    try:
        if os.path.isdir(src):
            shutil.copytree(src, dst, dirs_exist_ok=True)
    except Exception:
        pass


def build_gz_resource_path(runtime_models_root: str, ws_dir: str) -> str:
    """Return a colon-joined GZ_SIM_RESOURCE_PATH including existing env value."""
    base = f"{runtime_models_root}:{ws_dir}/models:{ws_dir}/worlds:{ws_dir}/install"
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    return f"{base}:{existing}" if existing else base


def build_gz_plugin_path() -> str:
    """Return a colon-joined GZ_SIM_SYSTEM_PLUGIN_PATH including existing env value."""
    base = "/opt/ros/jazzy/lib"
    existing = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    return f"{base}:{existing}" if existing else base


def prepare_runtime_world_sdf(
    world_file: str,
    log_dir: str,
    runtime_ur5_model: str,
    *,
    headless: bool,
    keep_cameras: bool,
) -> str:
    """Patch *world_file* (strip cameras in headless, inject runtime model URI).

    Returns path to the written runtime world file, or *world_file* on error.
    """
    _CAMERA_NAMES = (
        "camera_overhead", "camera_debug_top",
        "camera_north", "camera_south", "camera_east", "camera_west",
    )
    try:
        if not os.path.isfile(world_file):
            return world_file
        with open(world_file, "r", encoding="utf-8") as f:
            text = f.read()
        if headless and not keep_cameras:
            for cam in _CAMERA_NAMES:
                text = re.sub(
                    rf'<model\s+name="{cam}">.*?</model>', "", text, flags=re.DOTALL
                )
        text = text.replace(
            "<uri>model://ur5_rg2</uri>",
            f"<uri>file://{runtime_ur5_model}</uri>",
        )
        runtime_world = os.path.join(log_dir, "world_runtime.sdf")
        with open(runtime_world, "w", encoding="utf-8") as f:
            f.write(text)
        return runtime_world
    except Exception:
        return world_file


# ---------------------------------------------------------------------------
# Declarative env-var passthrough table
# ---------------------------------------------------------------------------
# Each entry: (ENV_VAR_NAME, default_value).
# In ur5_stack.launch.py these become:
#   SetEnvironmentVariable(name, os.environ.get(name, default))
# Override any of these at launch time by setting the env var before launch.

def load_runtime_defaults(yaml_path: str | None = None) -> dict[str, str]:
    """Load runtime defaults YAML as a flat ``{ENV_VAR_NAME: str_value}`` dict.

    Source of truth for tunables (F2 refactor TFM). Falls back to ``{}`` if
    yaml is unavailable or the file does not exist; callers must keep their
    own literal defaults as last resort.

    Resolution priority for any tunable (handled by the caller):
        1. ``os.environ[ENV_NAME]`` if set (operator override).
        2. This YAML.
        3. Built-in literal default.
    """
    if yaml_path is None:
        ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
        yaml_path = os.path.join(ws_dir, "src", "ur5_bringup", "config", "runtime_defaults.yaml")
    try:
        import yaml  # type: ignore
    except Exception:
        return {}
    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}
    return {str(k): "" if v is None else str(v) for k, v in data.items()}


PANEL_ENV_DEFAULTS: list[tuple[str, str]] = [
    # Debug / audit root
    ("PANEL_DIRECT_DEBUG_ROOT", "/home/laboratorio/TFM/historico"),
    # GRASP_DOWN segmented IK descent
    ("PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M", "0.005"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN", "0"),  # Fix 2026-05-04: cartesian devuelve fraction=0 en este setup
    ("PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT", "0.035"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL", "0.200"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M", "0.005"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_FORCE_INHERIT_XY", "1"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK", "1"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_MAX_DEV_RAD", "0.28"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_SUM_DEV_RAD", "0.60"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M", "0.015"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M", "0.008"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M", "0.012"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_MAX_ATTEMPTS", "4"),
    ("PANEL_PICK_DEMO_GRASP_DOWN_UTIL_XY_TOL_M", "0.015"),
    # Transport runtime no_progress tuning (Fix 2026-05-04 bug CESTA_STAGE_1):
    # con stall=8s y min_progress=8mm, el detector cortaba al estancarse el
    # FollowJointTrajectory en los últimos ~28mm a la cesta. 15s + 3mm permite
    # que el controller termine el goal dentro de la tolerancia 60mm cesta.
    ("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_STALL_TIMEOUT_SEC", "15.0"),
    ("PANEL_PICK_DEMO_TRANSPORT_RUNTIME_MIN_PROGRESS_M", "0.003"),
    # ATTACH gate tolerances
    ("PANEL_PICK_DEMO_ATTACH_XY_TOL_M", "0.020"),
    ("PANEL_PICK_DEMO_ATTACH_Z_TOL_M", "0.010"),
    ("PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M", "0.040"),
    ("PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M", "0.012"),
    ("PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC", "0.35"),
    ("PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES", "5"),
    ("PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M", "0.020"),
    # Gripper close confirmation
    ("PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M", "0.020"),
    # Fix 2026-05-04 (bug "el objeto se mueve solo"): subido de 3.0→8.0s y
    # delta 0.01→0.060. Con 3s+10mm el panel daba por "cerrado" cuando el
    # gripper sólo había cerrado 19mm (de los 85mm necesarios para tocar un
    # objeto de 50mm); luego el attach lógico arrastraba el objeto sin
    # contacto físico real. Con 8s+60mm sólo se confirma cierre cuando los
    # dedos hayan reducido 60mm (objeto 50mm + 10mm de holgura).
    ("PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC", "8.0"),
    ("PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM", "0.060"),
    ("PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M", "0.12"),
    # TF stability gate for GRASP_DOWN post-motion check
    ("PANEL_PICK_OBJECT_GRASP_TF_STABLE_TOL_M", "0.045"),
    ("PANEL_PICK_OBJECT_GRASP_TF_STABLE_SAMPLES", "5"),
    ("PANEL_PICK_OBJECT_GRASP_TF_STABLE_MIN_OK", "4"),
    # Approach coarse gate
    ("PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M", "0.035"),
    ("PANEL_PICK_DEMO_APPROACH_COARSE_GATE_XY_TOL_M", "0.012"),
    ("PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M", "0.012"),
    # PRE_CLOSE tolerances
    ("PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M", "0.020"),
    ("PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M", "0.010"),
    ("PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES", "2"),
    # CLOSE tolerances
    ("PANEL_PICK_DEMO_CLOSE_XY_TOL_M", "0.008"),
    ("PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M", "0.008"),
    # GRASP_ALIGN_IK convergence
    ("PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL", "0.200"),
    ("PANEL_PICK_DEMO_ALIGN_IK_SEED_WEIGHT", "0.80"),
    ("PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M", "0.020"),
    ("PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M", "0.010"),
    ("PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M", "0.008"),
    # Pose source freshness
    ("PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC", "0.400"),
    ("PANEL_PICK_DEMO_POSE_SOURCE_TOL_M", "0.006"),
    ("PANEL_PICK_DEMO_PHASE_JUMP_TOL_M", "0.010"),
    # Direct IK runtime settle
    ("PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC", "2.5"),
    ("PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M", "0.003"),
    ("PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SAMPLES", "3"),
    ("PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_POLL_SEC", "0.10"),
]


def env_passthrough_actions(
    defaults: list[tuple[str, str]],
) -> list[Any]:
    """Build SetEnvironmentVariable actions for each (name, default) pair.

    Returns a list of dicts ``{"name": ..., "value": ...}`` so this module stays
    free of launch framework imports. The caller creates the actual actions.
    """
    return [
        {"name": name, "value": os.environ.get(name, default)}
        for name, default in defaults
    ]
