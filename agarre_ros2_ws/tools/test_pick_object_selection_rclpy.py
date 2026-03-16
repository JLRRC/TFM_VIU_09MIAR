#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_pick_object_selection_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Validate PICK_OBJ target selection consistency from runtime logs."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


RE_CLICK = re.compile(r"\[BTN\] PICK Objeto")
RE_SELECTED = re.compile(r"\[OBJECTS\] state (\S+) -> SELECTED reason=select")
RE_SNAPSHOT = re.compile(r"\[PICK_OBJ\]\[TARGET\] snapshot=(\S+) store_selected=(\S+) ui_selected=(\S+)")
RE_USING = re.compile(r"\[PICK_OBJ\]\[OBJECT\] using=(\S+) for planning\+attach")


def _pick_last_log(path_arg: str) -> Path | None:
    if path_arg:
        p = Path(path_arg)
        return p if p.exists() else None
    candidates = [
        Path("log/ros2_launch.log"),
        Path("log/panel.log"),
    ]
    existing = [p for p in candidates if p.exists()]
    if not existing:
        return None
    return max(existing, key=lambda p: p.stat().st_mtime)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", default="")
    parser.add_argument("--expected", default="")
    args = parser.parse_args()

    log_path = _pick_last_log(args.log)
    if log_path is None:
        print("[FAIL] log file not found")
        return 1

    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    click_idx = -1
    for i, line in enumerate(lines):
        if RE_CLICK.search(line):
            click_idx = i
    if click_idx < 0:
        print(f"[FAIL] no PICK click found in {log_path}")
        return 1

    selected_before = "none"
    for line in reversed(lines[:click_idx]):
        m = RE_SELECTED.search(line)
        if m:
            selected_before = m.group(1)
            break

    snapshot = None
    using = None
    for line in lines[click_idx:]:
        if snapshot is None:
            m = RE_SNAPSHOT.search(line)
            if m:
                snapshot = {
                    "snapshot": m.group(1),
                    "store": m.group(2),
                    "ui": m.group(3),
                }
                continue
        if using is None:
            m = RE_USING.search(line)
            if m:
                using = m.group(1)
                break

    if snapshot is None:
        print("[FAIL] missing [PICK_OBJ][TARGET] snapshot log")
        return 1
    if using is None:
        print("[FAIL] missing [PICK_OBJ][OBJECT] using log")
        return 1

    expected = (args.expected or selected_before).strip()
    print(f"[INFO] log={log_path}")
    print(f"[INFO] selected_before={selected_before}")
    print(
        "[INFO] snapshot="
        f"{snapshot['snapshot']} store_selected={snapshot['store']} ui_selected={snapshot['ui']}"
    )
    print(f"[INFO] using={using}")
    print(f"[INFO] expected={expected or 'none'}")

    if expected and using != expected:
        print(f"[FAIL] selection mismatch using={using} expected={expected}")
        return 1
    if using == "pick_demo" and expected and expected != "pick_demo":
        print("[FAIL] stale target fallback to pick_demo")
        return 1

    print("[PASS] PICK_OBJ uses selected snapshot target")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
