#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/panel_perf_compare.py
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
"""Run before/after measurements and print a comparison summary."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from typing import Dict, Optional


def _run_measure(cmd: str) -> Optional[Dict[str, object]]:
    try:
        res = subprocess.run(["bash", "-lc", cmd], check=False, text=True, capture_output=True)
    except Exception:
        return None
    out = (res.stdout or "").strip()
    if not out:
        return None
    try:
        return json.loads(out)
    except Exception:
        return None


def _summary(data: Dict[str, object]) -> Dict[str, float]:
    summary = data.get("summary") if isinstance(data.get("summary"), dict) else {}
    def _get(name: str) -> float:
        try:
            return float(summary.get(name, 0.0))
        except Exception:
            return 0.0
    return {
        "cpu_avg": _get("cpu_avg"),
        "cpu_p95": _get("cpu_p95"),
        "cpu_max": _get("cpu_max"),
        "rss_avg": _get("rss_avg"),
        "rss_max": _get("rss_max"),
        "camera_hz": float(data.get("camera_hz", 0.0)) if "camera_hz" in data else 0.0,
    }


def _format_delta(after: float, before: float) -> str:
    if before == 0.0:
        return "n/a"
    diff = after - before
    pct = (diff / before) * 100.0
    return f"{diff:+.2f} ({pct:+.1f}%)"


def main() -> int:
    parser = argparse.ArgumentParser(description="Before/after perf comparison for panel_v2.")
    parser.add_argument("--match", default="panel_v2")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--camera-topic", default="")
    parser.add_argument("--before-env", default="", help="Env exports for BEFORE run")
    parser.add_argument("--after-env", default="", help="Env exports for AFTER run")
    args = parser.parse_args()

    base_cmd = (
        f"python3 /home/laboratorio/TFM/agarre_ros2_ws/scripts/panel_perf_measure.py "
        f"--match {args.match} --duration {args.duration} --interval {args.interval}"
    )
    if args.camera_topic:
        base_cmd += f" --camera-topic {args.camera_topic}"

    before_cmd = f"{args.before_env} {base_cmd}".strip()
    after_cmd = f"{args.after_env} {base_cmd}".strip()

    before = _run_measure(before_cmd)
    after = _run_measure(after_cmd)
    if not before or not after:
        print("No se pudo ejecutar medicion before/after.", file=sys.stderr)
        return 2

    b = _summary(before)
    a = _summary(after)
    comparison = {
        "cpu_avg": _format_delta(a["cpu_avg"], b["cpu_avg"]),
        "cpu_p95": _format_delta(a["cpu_p95"], b["cpu_p95"]),
        "cpu_max": _format_delta(a["cpu_max"], b["cpu_max"]),
        "rss_avg": _format_delta(a["rss_avg"], b["rss_avg"]),
        "rss_max": _format_delta(a["rss_max"], b["rss_max"]),
        "camera_hz": _format_delta(a["camera_hz"], b["camera_hz"]),
    }
    result = {"before": b, "after": a, "delta": comparison}
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
