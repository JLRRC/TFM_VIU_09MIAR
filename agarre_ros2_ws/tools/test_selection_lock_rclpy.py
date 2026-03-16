#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_selection_lock_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Validate PICK_OBJ target lock uses the latest selected object.

Two modes:
- live (default): subscribe to /rosout and validate latest PICK cycle.
- log mode (--log): parse a log file with the same checks.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import time
from dataclasses import dataclass
from typing import Optional


RE_SELECT_STORE = re.compile(r"\[PICK\]\[SELECT_STORE\].*name=(?P<name>[^\s]+).*update_ok=(?P<ok>[^\s]+)")
RE_SELECT_STATE = re.compile(r"\[PICK\]\[SELECT_STATE\].*name=(?P<name>[^\s]+).*")
RE_BTN = re.compile(r"\[BTN\] PICK Objeto")
RE_EFFECTIVE = re.compile(
    r"\[PICK\]\[SELECT_EFFECTIVE\].*ui_selected=(?P<ui>[^\s]+).*store_selected=(?P<store>[^\s]+).*user_selected=(?P<user>[^\s]+)"
)
RE_TARGET = re.compile(r"\[PICK_OBJ\]\[TARGET\].*snapshot=(?P<snap>[^\s]+).*")
RE_LOCK = re.compile(r"\[PICK_OBJ\]\[TARGET_LOCK\].*name=(?P<name>[^\s]+).*")


@dataclass
class Evidence:
    expected: str = ""
    ui: str = ""
    store: str = ""
    user: str = ""
    target: str = ""
    lock_name: str = ""


def _update_from_line(ev: Evidence, line: str) -> None:
    m = RE_SELECT_STORE.search(line)
    if m and m.group("ok").lower() == "true":
        ev.expected = m.group("name")
    m = RE_SELECT_STATE.search(line)
    if m and not ev.expected:
        ev.expected = m.group("name")
    m = RE_EFFECTIVE.search(line)
    if m:
        ev.ui = m.group("ui")
        ev.store = m.group("store")
        ev.user = m.group("user")
    m = RE_TARGET.search(line)
    if m:
        ev.target = m.group("snap")
    m = RE_LOCK.search(line)
    if m:
        ev.lock_name = m.group("name")


def _eval(ev: Evidence, expected_override: str = "") -> tuple[bool, str]:
    expected = (expected_override or ev.expected).strip()
    if not expected:
        return False, "missing expected selection (no SELECT_STORE/SELECT_STATE)"
    if not ev.target:
        return False, "missing [PICK_OBJ][TARGET] snapshot"
    if not ev.lock_name:
        return False, "missing [PICK_OBJ][TARGET_LOCK]"
    if ev.ui and ev.ui != expected:
        return False, f"ui_selected mismatch ui={ev.ui} expected={expected}"
    if ev.store and ev.store != expected:
        return False, f"store_selected mismatch store={ev.store} expected={expected}"
    if ev.user and ev.user != expected:
        return False, f"user_selected mismatch user={ev.user} expected={expected}"
    if ev.target != expected:
        return False, f"target mismatch snapshot={ev.target} expected={expected}"
    if ev.lock_name != expected:
        return False, f"target_lock mismatch lock={ev.lock_name} expected={expected}"
    return True, f"target={ev.target} lock={ev.lock_name} expected={expected}"


def _run_log_mode(path: pathlib.Path, expected: str) -> int:
    if not path.exists():
        print(f"[FAIL] log not found: {path}")
        return 1
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    last_pick_idx = -1
    for idx, line in enumerate(lines):
        if RE_BTN.search(line):
            last_pick_idx = idx
    if last_pick_idx < 0:
        print("[FAIL] no '[BTN] PICK Objeto' in log")
        return 1
    ev = Evidence()
    for line in lines[max(0, last_pick_idx - 400): last_pick_idx + 400]:
        _update_from_line(ev, line)
    ok, detail = _eval(ev, expected)
    print(f"[INFO] mode=log file={path}")
    print(f"[INFO] expected={expected or ev.expected or 'none'} ui={ev.ui or 'none'} store={ev.store or 'none'} user={ev.user or 'none'}")
    print(f"[INFO] target={ev.target or 'none'} lock={ev.lock_name or 'none'}")
    if ok:
        print(f"[PASS] {detail}")
        return 0
    print(f"[FAIL] {detail}")
    return 1


def _run_live_mode(timeout: float, expected: str) -> int:
    import rclpy
    from rcl_interfaces.msg import Log

    rclpy.init()
    node = rclpy.create_node("test_selection_lock_rclpy")
    ev = Evidence()
    active_pick = False

    def _cb(msg: Log) -> None:
        nonlocal active_pick
        text = msg.msg or ""
        if RE_BTN.search(text):
            active_pick = True
        if active_pick:
            _update_from_line(ev, text)

    node.create_subscription(Log, "/rosout", _cb, 100)
    start = time.time()
    try:
        while rclpy.ok() and (time.time() - start) < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            if ev.target and ev.lock_name:
                ok, detail = _eval(ev, expected)
                if ok:
                    print(f"[PASS] {detail}")
                    return 0
        ok, detail = _eval(ev, expected)
        if ok:
            print(f"[PASS] {detail}")
            return 0
        print(f"[FAIL] {detail}")
        print(
            f"[INFO] expected={expected or ev.expected or 'none'} ui={ev.ui or 'none'} "
            f"store={ev.store or 'none'} user={ev.user or 'none'} target={ev.target or 'none'} lock={ev.lock_name or 'none'}"
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", default="", help="Parse this log file instead of live /rosout")
    parser.add_argument("--expected", default="", help="Expected target name")
    parser.add_argument("--timeout", type=float, default=20.0, help="Live mode timeout seconds")
    args = parser.parse_args()

    if args.log:
        return _run_log_mode(pathlib.Path(args.log), args.expected)
    return _run_live_mode(args.timeout, args.expected)


if __name__ == "__main__":
    raise SystemExit(main())
