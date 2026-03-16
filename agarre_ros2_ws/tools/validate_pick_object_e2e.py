#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/validate_pick_object_e2e.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Validate latest PICK OBJETO run from logs with explicit PASS/FAIL reason."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Iterable, Optional


RE_PICK_CLICK = re.compile(r"\[BTN\] PICK Objeto")
RE_PICK_REMOTE = re.compile(r"\[PICK\]\[REMOTE\] trigger=(?:service|topic):/panel/pick_object")
RE_SELECTED = re.compile(r"\[OBJECTS\] state (\S+) -> SELECTED reason=select")
RE_TARGET = re.compile(r"\[PICK_OBJ\]\[TARGET\] name=(\S+)")
RE_USING = re.compile(r"\[PICK_OBJ\]\[OBJECT\] using=(\S+) for planning\+attach")
RE_RESULT = re.compile(
    r"\[PICK_OBJ\]\[MOVEIT\]\[RESULT\] PRE_GRASP .*request_id=(\d+).*success=(true|false).*exec_ok=(true|false)"
)
RE_DIST = re.compile(r"\[PICK_OBJ\]\[TF\]\[CHECK\] PRE_GRASP .* dist=([0-9.]+) tol=([0-9.]+) .* ok=(true|false)")
RE_ABORT_REASON = re.compile(r"\[PICK_OBJ\]\[ABORT\] (.*)")
RE_ATTACH = re.compile(r"\[ATTACH\] selected=(\S+) attach_name=(\S+) resolved_entity=(\S+)")
RE_ATTACH_COUNTS = re.compile(r"\[ATTACH\] pub_count=([-0-9]+) sub_count=([-0-9]+) topic=(\S+)")
RE_STATE_CARRIED = re.compile(r"\[OBJECTS\] state (\S+) -> CARRIED")
RE_STATE_RELEASED = re.compile(r"\[OBJECTS\] state (\S+) -> RELEASED")
RE_SEQUENCE_OK = re.compile(r"\[PICK_OBJ\] === SECUENCIA COMPLETADA EXITOSAMENTE ===")
RE_BRIDGE_RECOVERED = re.compile(r"\[PICK_OBJ\]\[MOVEIT\]\[PLAN\] .*bridge_recovered=true")


def _candidate_logs() -> list[Path]:
    home = Path.home()
    candidates = [
        Path("log/ros2_launch.log"),
        Path("log/panel.log"),
        home / ".ros" / "log" / "latest" / "launch.log",
    ]
    ros_dir = home / ".ros" / "log"
    if ros_dir.is_dir():
        candidates.extend(sorted(ros_dir.glob("*/*.log")))
    # Keep existing only and deduplicate while preserving order.
    out: list[Path] = []
    seen: set[str] = set()
    for p in candidates:
        if not p.exists():
            continue
        key = str(p.resolve())
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out


def _pick_latest_log(paths: Iterable[Path], *, require_pick: bool) -> Optional[Path]:
    latest: Optional[Path] = None
    latest_mtime = -1.0
    for p in paths:
        if require_pick:
            try:
                txt = p.read_text(encoding="utf-8", errors="replace")
            except Exception:
                continue
            if "[BTN] PICK Objeto" not in txt:
                continue
        try:
            mtime = p.stat().st_mtime
        except Exception:
            continue
        if mtime > latest_mtime:
            latest_mtime = mtime
            latest = p
    return latest


def _last_pick_segment(lines: list[str]) -> tuple[int, list[str]]:
    idx = -1
    for i, line in enumerate(lines):
        if RE_PICK_CLICK.search(line) or RE_PICK_REMOTE.search(line):
            idx = i
    if idx < 0:
        return -1, []
    return idx, lines[idx:]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", default="", help="Log file path")
    parser.add_argument(
        "--last",
        action="store_true",
        help="Auto-pick latest known log (ignores --log when provided empty)",
    )
    parser.add_argument("--tol", type=float, default=0.03, help="PRE_GRASP tolerance meters")
    args = parser.parse_args()

    path: Optional[Path]
    if args.last or not args.log:
        path = _pick_latest_log(_candidate_logs(), require_pick=True)
        if path is None:
            path = _pick_latest_log(_candidate_logs(), require_pick=False)
    else:
        path = Path(args.log)
    if path is None or not path.exists():
        print("[FAIL] no log file found (use --log or --last)")
        return 1

    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    click_idx, seg = _last_pick_segment(lines)
    if click_idx < 0:
        print(f"[FAIL] no PICK OBJETO trigger found in {path}")
        return 1

    # Selection before click.
    selected_before = "n/a"
    for line in reversed(lines[:click_idx]):
        m = RE_SELECTED.search(line)
        if m:
            selected_before = m.group(1)
            break

    target = "n/a"
    for line in seg:
        m = RE_TARGET.search(line)
        if m:
            target = m.group(1)
            break
    using = "n/a"
    for line in seg:
        m = RE_USING.search(line)
        if m:
            using = m.group(1)
            break

    result_match = None
    for line in seg:
        m = RE_RESULT.search(line)
        if m:
            result_match = (int(m.group(1)), m.group(2) == "true", m.group(3) == "true", line.strip())
            break

    dist = None
    dist_tol = None
    dist_ok = False
    for line in seg:
        m = RE_DIST.search(line)
        if m:
            dist = float(m.group(1))
            dist_tol = float(m.group(2))
            dist_ok = (m.group(3) == "true")
            break

    has_no_result = any("sin resultado" in line for line in seg)
    has_pose_subs_zero = any("pose_subs=0" in line for line in seg)
    bridge_recovered = any(RE_BRIDGE_RECOVERED.search(line) for line in seg)
    has_attach_fallback = any("fallback=logical_attach" in line for line in seg)
    has_attach_backend_missing = any("attach_backend_missing" in line for line in seg)
    sequence_completed = any(RE_SEQUENCE_OK.search(line) for line in seg)
    abort_reason = ""
    for line in seg:
        m = RE_ABORT_REASON.search(line)
        if m:
            abort_reason = m.group(1)
            break
    attach_selected = "n/a"
    attach_name = "n/a"
    attach_entity = "n/a"
    attach_pub_count = -1
    attach_sub_count = -1
    attach_topic = "n/a"
    for line in seg:
        m = RE_ATTACH.search(line)
        if m:
            attach_selected, attach_name, attach_entity = m.group(1), m.group(2), m.group(3)
            break
    for line in seg:
        m = RE_ATTACH_COUNTS.search(line)
        if m:
            attach_pub_count = int(m.group(1))
            attach_sub_count = int(m.group(2))
            attach_topic = m.group(3)
            break
    carried_target = False
    released_target = False
    for line in seg:
        m = RE_STATE_CARRIED.search(line)
        if m and target != "n/a" and m.group(1) == target:
            carried_target = True
        m = RE_STATE_RELEASED.search(line)
        if m and target != "n/a" and m.group(1) == target:
            released_target = True

    print(f"[INFO] log={path}")
    print(f"[INFO] selected_before={selected_before}")
    print(f"[INFO] pick_target={target}")
    print(f"[INFO] using_target={using}")
    print(f"[INFO] has_no_result={str(has_no_result).lower()}")
    print(f"[INFO] has_pose_subs_zero={str(has_pose_subs_zero).lower()}")
    print(f"[INFO] bridge_recovered={str(bridge_recovered).lower()}")
    print(f"[INFO] has_attach_fallback={str(has_attach_fallback).lower()}")
    print(f"[INFO] has_attach_backend_missing={str(has_attach_backend_missing).lower()}")
    print(
        "[INFO] attach="
        f"selected={attach_selected} attach_name={attach_name} entity={attach_entity} "
        f"pub_count={attach_pub_count} sub_count={attach_sub_count} topic={attach_topic}"
    )
    print(f"[INFO] state_carried_target={str(carried_target).lower()}")
    print(f"[INFO] state_released_target={str(released_target).lower()}")
    print(f"[INFO] sequence_completed={str(sequence_completed).lower()}")
    if result_match is not None:
        req_id, success, exec_ok, raw_line = result_match
        print(f"[INFO] pregrasp_result request_id={req_id} success={success} exec_ok={exec_ok}")
        print(f"[INFO] pregrasp_result_line={raw_line}")
    else:
        req_id, success, exec_ok = (-1, False, False)
        print("[INFO] pregrasp_result=missing")
    effective_tol = dist_tol if dist_tol is not None else float(args.tol)
    if dist is not None:
        print(f"[INFO] pregrasp_dist={dist:.3f} tf_ok={str(dist_ok).lower()} tol={effective_tol:.3f}")
    else:
        print("[INFO] pregrasp_dist=n/a")
    if abort_reason:
        print(f"[INFO] abort_reason={abort_reason}")

    terminal_success = sequence_completed and carried_target and released_target

    fail_reason = ""
    if has_no_result and not terminal_success:
        fail_reason = "no_result"
    elif has_pose_subs_zero and not bridge_recovered and not terminal_success:
        fail_reason = "bridge_disconnected"
    elif target == "n/a":
        fail_reason = "no_target"
    elif using == "n/a":
        fail_reason = "no_using_target"
    elif target != using:
        fail_reason = "target_using_mismatch"
    elif selected_before != "n/a" and target != selected_before:
        fail_reason = "selection_mismatch"
    elif has_attach_backend_missing and not terminal_success:
        fail_reason = "attach_backend_missing"
    elif has_attach_fallback and not terminal_success:
        fail_reason = "attach_logical_fallback"
    elif attach_name == "n/a" and not terminal_success:
        fail_reason = "attach_not_attempted"
    elif target != "n/a" and attach_name != target and not terminal_success:
        fail_reason = "attach_target_mismatch"
    elif attach_sub_count < 1 and not terminal_success:
        fail_reason = "attach_no_subscriber"
    elif result_match is None:
        fail_reason = "missing_pregrasp_result"
    elif not success or not exec_ok:
        fail_reason = "pregrasp_exec_failed"
    elif dist is None:
        fail_reason = "missing_tf_check"
    elif (dist >= effective_tol) or (not dist_ok):
        # Dist can be larger transiently; we only fail if final check remains bad.
        if "tf_stale_timeout" in abort_reason:
            fail_reason = "tf_stale_timeout"
        else:
            fail_reason = "tf_mismatch"
    elif not carried_target:
        fail_reason = "target_not_carried"
    elif not released_target:
        fail_reason = "target_not_released"

    if fail_reason:
        print(f"[FAIL] reason={fail_reason}")
        return 1

    if terminal_success:
        print("[PASS] PICK_OBJ E2E OK (bridge recovered and full pick sequence completed)")
    else:
        print("[PASS] PICK_OBJ E2E OK (selection + result + PRE_GRASP dist<tolerance)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
