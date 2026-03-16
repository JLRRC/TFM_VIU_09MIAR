#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/bench_infer_log.py
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
"""Parse infer.log and report latency stats for recent TFM inferences."""
from __future__ import annotations

import argparse
import os
import re
import statistics as stats
from typing import List, Tuple


LINE_RE = re.compile(
    r"infer_end(?:\\s+session=(?P<session>\\S+))?\\s+status=(?P<status>OK|FAIL)\\s+"
    r"infer_ms=(?P<infer>[0-9.]+)\\s+total_ms=(?P<total>[0-9.]+)"
)
START_RE = re.compile(
    r"infer_start\\s+session=(?P<session>\\S+)\\s+mode=(?P<mode>\\S+)"
)


def _last_session(lines: List[str]) -> str:
    for line in reversed(lines):
        m = LINE_RE.search(line)
        if m and m.group("session"):
            return m.group("session")
    return ""


def load_metrics(path: str, last_n: int, session: str):
    if not os.path.isfile(path):
        raise FileNotFoundError(f"infer.log no existe: {path}")
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    if session:
        session = session.strip()
    elif session == "":
        session = _last_session(lines)

    per_mode = {}
    cur_mode = {}
    all_infer: List[float] = []
    all_total: List[float] = []

    for line in lines:
        ms = START_RE.search(line)
        if ms:
            cur_mode[ms.group("session")] = ms.group("mode")
            continue
        me = LINE_RE.search(line)
        if not me:
            continue
        sess = me.group("session") or ""
        if session and sess != session:
            continue
        mode = cur_mode.get(sess, "unknown")
        try:
            infer_v = float(me.group("infer"))
            total_v = float(me.group("total"))
        except Exception:
            continue
        all_infer.append(infer_v)
        all_total.append(total_v)
        per_mode.setdefault(mode, {"infer": [], "total": []})
        per_mode[mode]["infer"].append(infer_v)
        per_mode[mode]["total"].append(total_v)
        if last_n and len(all_infer) >= last_n:
            break
    return all_infer, all_total, per_mode, session


def summarize(vals: List[float]) -> str:
    if not vals:
        return "n=0"
    p50 = stats.median(vals)
    p90 = stats.quantiles(vals, n=10)[8] if len(vals) >= 10 else max(vals)
    mean = stats.mean(vals)
    return f"n={len(vals)} mean={mean:.2f}ms p50={p50:.2f}ms p90={p90:.2f}ms"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="", help="Ruta a logs/infer.log")
    ap.add_argument("--last", type=int, default=30, help="Cuántas inferencias recientes")
    ap.add_argument("--session", default="", help="Session ID (si no se indica, usa la última)")
    args = ap.parse_args()

    log_path = args.log
    if not log_path:
        ws_dir = os.path.expanduser(os.environ.get("WS_DIR", "~/TFM/agarre_ros2_ws"))
        log_path = os.path.join(ws_dir, "logs", "infer.log")

    infer_vals, total_vals, per_mode, sess = load_metrics(log_path, args.last, args.session)
    if sess:
        print(f"[session]  {sess}")
    print(f"[infer_ms]  {summarize(infer_vals)}")
    print(f"[total_ms]  {summarize(total_vals)}")
    for mode, vals in sorted(per_mode.items()):
        print(f"[mode={mode}] infer_ms {summarize(vals['infer'])}")
        print(f"[mode={mode}] total_ms {summarize(vals['total'])}")


if __name__ == "__main__":
    main()
