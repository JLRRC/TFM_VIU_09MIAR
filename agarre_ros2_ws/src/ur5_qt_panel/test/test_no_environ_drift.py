#!/usr/bin/env python3
"""Gating: no growth in os.environ.get/os.getenv outside allowed baseline.

The baseline is the snapshot taken on 2026-05-08 (audit-v4-baseline-fixed).
Each entry maps a file relative to the package dir to its allowed read
count. The test FAILS if:
  * A new file appears with env reads (not in baseline).
  * A baselined file grows above its allowed count.

It's allowed for any baselined file to DECREASE (we want migration to
panel_env helpers / *_params.py). When you reduce a file's reads, you
must also lower the baseline number here.

Authoritative locations for env reads (no need to track):
  * src/ur5_qt_panel/ur5_qt_panel/panel_env.py — central API.
  * Any */launch/*.py and */launch_helpers.py — launch glue.
  * Any *_params.py — frozen dataclasses.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Tuple

PKG_ROOT = Path(__file__).resolve().parent.parent / "ur5_qt_panel"
WS_SRC = Path(__file__).resolve().parents[3] / "src"

ENV_PATTERN = re.compile(r"\b(?:os\.environ\.get|os\.getenv)\s*\(")

EXCLUDE_DIRS = {"__pycache__", "build", "install", "log"}

ALLOWED_BASE_FILES = {
    "ur5_qt_panel/panel_env.py",
}

ALLOWED_PATTERNS = (
    "_params.py",
    "/launch/",
    "launch_helpers.py",
)


BASELINE: Dict[str, int] = {
    # 2026-05-09: panel_pick_object.py + panel_tfm_execute.py +
    # moveit_bridge/* eliminados con MoveIt-classic.
    # moveit_bridge_utils.py reducido a 4 helpers env (mirror panel_env).
    "ur5_tools/moveit_bridge_utils.py": 4,
    # F2 audit (2026-05-10): workspace_paths centraliza WS_DIR/GZ_*
    # como single source of truth — 6 reads documentados.
    "ur5_tools/workspace_paths.py": 6,
    "ur5_tools/release_objects_service.py": 3,
    "ur5_qt_panel/panel_settings.py": 3,
    "ur5_qt_panel/directo_geometry.py": 3,
    "ur5_qt_panel/panel_motion_helpers.py": 3,  # 2026-05-09: defaults inlined post-borrar moveit_bridge.
    "ur5_tools/gripper_geometry.py": 2,
    "ur5_tools/attach_set_pose.py": 2,
    "ur5_qt_panel/pick_demo_dispatcher.py": 2,
    "ur5_qt_panel/panel_v2_helpers.py": 2,
    "ur5_qt_panel/panel_utils.py": 2,
    "ur5_qt_panel/panel_ros.py": 2,
    "ur5_qt_panel/panel_launchers.py": 2,
    "ur5_qt_panel/panel_gz_startup.py": 2,
    "ur5_qt_panel/panel_config.py": 2,
    "tfm_grasping/model.py": 2,
    "ur5_tools/world_tf_publisher.py": 1,
    "ur5_tools/system_state_manager.py": 1,
    "ur5_tools/planning_scene_sync.py": 1,
    "ur5_tools/gripper_attach_backend.py": 1,
    "ur5_tools/evidence_logger.py": 1,
    "ur5_tools/cycle_logger.py": 1,
    "ur5_qt_panel/panel_system_status.py": 1,
    "ur5_qt_panel/panel_startup.py": 1,
    "ur5_qt_panel/directo_gate_evaluator.py": 1,
}


def _file_is_allowed_unconditionally(rel: str) -> bool:
    if rel in ALLOWED_BASE_FILES:
        return True
    for pat in ALLOWED_PATTERNS:
        if pat in rel:
            return True
    return False


def _scan() -> Dict[str, int]:
    found: Dict[str, int] = {}
    for pkg_dir in WS_SRC.iterdir():
        if not pkg_dir.is_dir():
            continue
        pkg_src = pkg_dir / pkg_dir.name
        if not pkg_src.exists():
            continue
        for py in pkg_src.rglob("*.py"):
            if set(py.parts) & EXCLUDE_DIRS:
                continue
            if py.name.startswith("test_"):
                continue
            try:
                txt = py.read_text(encoding="utf-8")
            except (OSError, UnicodeDecodeError):
                continue
            count = len(ENV_PATTERN.findall(txt))
            if count == 0:
                continue
            rel = f"{pkg_dir.name}/{py.relative_to(pkg_src).as_posix()}"
            if _file_is_allowed_unconditionally(rel):
                continue
            found[rel] = count
    return found


def test_no_environ_drift_outside_baseline() -> None:
    found = _scan()
    new_files: List[str] = []
    growths: List[Tuple[str, int, int]] = []
    for rel, count in sorted(found.items()):
        baseline = BASELINE.get(rel)
        if baseline is None:
            new_files.append(f"  {rel}: {count}")
        elif count > baseline:
            growths.append((rel, baseline, count))
    msgs: List[str] = []
    if new_files:
        msgs.append(
            "New files with env reads (must use panel_env helpers or "
            "add to BASELINE if intentional):\n" + "\n".join(new_files)
        )
    if growths:
        msgs.append(
            "Files exceeded baseline (must decrease, not grow):\n"
            + "\n".join(f"  {f}: baseline={b}, found={c}" for f, b, c in growths)
        )
    assert not msgs, "\n\n".join(msgs)
