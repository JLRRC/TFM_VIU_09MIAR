#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/validate_pick_object_log.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Validate the latest PICK Objeto click flow from panel logs."""
from __future__ import annotations

import argparse
import pathlib
import re
import sys
from dataclasses import dataclass
from typing import Optional


RE_SELECT_STORE = re.compile(
    r"\[PICK\]\[SELECT_STORE\].*name=(?P<name>[^\s]+).*update_ok=(?P<ok>[^\s]+)"
)
RE_SELECTED_STATE = re.compile(
    r"\[OBJECTS\] state (?P<name>[^\s]+) -> SELECTED .*reason=select"
)
RE_PICK_BTN = re.compile(r"\[BTN\] PICK Objeto")
RE_PICK_SNAPSHOT_A = re.compile(
    r"\[PICK_OBJ\]\[TARGET\].*snapshot=(?P<snap>[^\s]+).*ui_selected=(?P<ui>[^\s]+).*store_selected=(?P<store>[^\s]+)"
)
RE_PICK_SNAPSHOT_B = re.compile(
    r"\[PICK_OBJ\]\[TARGET\].*snapshot=(?P<snap>[^\s]+).*store_selected=(?P<store>[^\s]+).*ui_selected=(?P<ui>[^\s]+)"
)
RE_PICK_TARGET = re.compile(r"\[PICK_OBJ\]\[TARGET\] name=(?P<name>[^\s]+)")
RE_PICK_TOPICS = re.compile(
    r"\[PICK_OBJ\]\[MOVEIT\]\[PLAN\] topics pose_subs=(?P<pose>\d+) result_pubs=(?P<pubs>\d+)"
)


@dataclass
class Check:
    name: str
    passed: bool
    detail: str


def _find_last_index(lines: list[str], pattern: re.Pattern[str]) -> int:
    for idx in range(len(lines) - 1, -1, -1):
        if pattern.search(lines[idx]):
            return idx
    return -1


def _find_last_before(lines: list[str], start_idx: int, pattern: re.Pattern[str]) -> Optional[re.Match[str]]:
    for idx in range(start_idx - 1, -1, -1):
        match = pattern.search(lines[idx])
        if match:
            return match
    return None


def _find_first_after(
    lines: list[str],
    start_idx: int,
    pattern: re.Pattern[str],
    *,
    max_lines: int = 300,
) -> Optional[re.Match[str]]:
    end = min(len(lines), start_idx + max_lines)
    for idx in range(start_idx, end):
        match = pattern.search(lines[idx])
        if match:
            return match
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate latest PICK Objeto click from log")
    parser.add_argument(
        "--log",
        default="log/ros2_launch.log",
        help="Path to panel launch log (default: log/ros2_launch.log)",
    )
    args = parser.parse_args()

    log_path = pathlib.Path(args.log)
    if not log_path.is_file():
        print(f"[FAIL] log file not found: {log_path}")
        return 2

    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    if not lines:
        print(f"[FAIL] empty log: {log_path}")
        return 2

    pick_idx = _find_last_index(lines, RE_PICK_BTN)
    if pick_idx < 0:
        print("[FAIL] no '[BTN] PICK Objeto' found")
        return 2

    select_store = _find_last_before(lines, pick_idx, RE_SELECT_STORE)
    selected_state = _find_last_before(lines, pick_idx, RE_SELECTED_STATE)
    snapshot = _find_first_after(lines, pick_idx, RE_PICK_SNAPSHOT_A)
    if snapshot is None:
        snapshot = _find_first_after(lines, pick_idx, RE_PICK_SNAPSHOT_B)
    target = _find_first_after(lines, pick_idx, RE_PICK_TARGET)
    topics = _find_first_after(lines, pick_idx, RE_PICK_TOPICS)

    expected_name = None
    if select_store is not None and select_store.group("ok").lower() == "true":
        expected_name = select_store.group("name")
    elif selected_state is not None:
        expected_name = selected_state.group("name")

    checks: list[Check] = []
    checks.append(
        Check(
            "selection_event",
            expected_name is not None,
            f"expected_selected={expected_name or 'none'}",
        )
    )

    if snapshot is not None:
        ui_selected = snapshot.group("ui")
        checks.append(
            Check(
                "snapshot_matches_selection",
                expected_name is not None and ui_selected == expected_name,
                f"ui_selected={ui_selected} expected={expected_name or 'none'}",
            )
        )
    else:
        checks.append(Check("snapshot_present", False, "missing [PICK_OBJ][TARGET] snapshot"))

    if target is not None:
        target_name = target.group("name")
        checks.append(
            Check(
                "target_matches_selection",
                expected_name is not None and target_name == expected_name,
                f"pick_target={target_name} expected={expected_name or 'none'}",
            )
        )
    else:
        checks.append(Check("target_present", False, "missing [PICK_OBJ][TARGET] name=..."))

    if topics is not None:
        pose_subs = int(topics.group("pose"))
        result_pubs = int(topics.group("pubs"))
        checks.append(
            Check(
                "moveit_bridge_connected",
                pose_subs > 0 and result_pubs > 0,
                f"pose_subs={pose_subs} result_pubs={result_pubs}",
            )
        )
    else:
        checks.append(Check("topics_present", False, "missing MOVEIT topics line"))

    print(f"[INFO] log={log_path}")
    print(f"[INFO] last_pick_line={pick_idx + 1}")
    for item in checks:
        status = "PASS" if item.passed else "FAIL"
        print(f"[{status}] {item.name}: {item.detail}")

    if all(item.passed for item in checks):
        print("[RESULT] PASS")
        return 0

    print("[RESULT] FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
