#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/panel_perf_measure.py
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
"""Measure panel CPU/RSS and optional camera topic rate."""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import psutil  # type: ignore
except Exception:  # pragma: no cover
    psutil = None


@dataclass
class ProcSample:
    ts: float
    cpu_percent: float
    rss_mb: float


def _find_pid(match: str) -> Optional[int]:
    if psutil is not None:
        for proc in psutil.process_iter(attrs=["pid", "cmdline"]):
            cmdline = proc.info.get("cmdline") or []
            if not cmdline:
                continue
            if any(match in part for part in cmdline):
                return int(proc.info["pid"])
        return None
    try:
        res = subprocess.run(
            ["bash", "-lc", f"pgrep -af {match!r} | head -n 1"],
            check=False,
            text=True,
            capture_output=True,
        )
        line = (res.stdout or "").strip()
        if not line:
            return None
        return int(line.split()[0])
    except Exception:
        return None


def _proc_sample(pid: int, last_cpu: Optional[float]) -> Tuple[Optional[ProcSample], Optional[float]]:
    if psutil is not None:
        try:
            proc = psutil.Process(pid)
            if last_cpu is None:
                proc.cpu_percent(interval=None)
                return None, 0.0
            cpu = proc.cpu_percent(interval=None)
            rss = float(proc.memory_info().rss) / (1024.0 * 1024.0)
            return ProcSample(ts=time.time(), cpu_percent=float(cpu), rss_mb=rss), cpu
        except Exception:
            return None, last_cpu
    # /proc fallback
    try:
        with open(f"/proc/{pid}/stat", "r", encoding="utf-8") as f:
            fields = f.read().split()
        utime = float(fields[13])
        stime = float(fields[14])
        rss_pages = float(fields[23])
        hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        page_size = os.sysconf("SC_PAGE_SIZE")
        cpu_time = (utime + stime) / hz
        rss = (rss_pages * page_size) / (1024.0 * 1024.0)
        if last_cpu is None:
            return None, cpu_time
        now = time.time()
        cpu_percent = max(0.0, (cpu_time - last_cpu) * 100.0)
        return ProcSample(ts=now, cpu_percent=cpu_percent, rss_mb=rss), cpu_time
    except Exception:
        return None, last_cpu


class CameraRate:
    def __init__(self, topic: str):
        self._topic = topic
        self._count = 0
        self._start = 0.0
        self._running = False
        self._node = None
        self._executor = None
        self._sub = None

    def start(self) -> bool:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
        except Exception:
            return False
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            return False
        try:
            self._node = rclpy.create_node("panel_perf_measure")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._sub = self._node.create_subscription(
                Image, self._topic, self._on_msg, qos_profile_sensor_data
            )
            self._count = 0
            self._start = time.time()
            self._running = True
            return True
        except Exception:
            self.stop()
            return False

    def spin_once(self) -> None:
        if not self._running or self._executor is None:
            return
        try:
            self._executor.spin_once(timeout_sec=0.05)
        except Exception:
            pass

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        try:
            if self._node and self._sub:
                self._node.destroy_subscription(self._sub)
        except Exception:
            pass
        try:
            if self._executor and self._node:
                self._executor.remove_node(self._node)
        except Exception:
            pass
        try:
            if self._node:
                self._node.destroy_node()
        except Exception:
            pass
        self._node = None
        self._executor = None
        self._sub = None

    def _on_msg(self, _msg) -> None:
        if not self._running:
            return
        self._count += 1

    def rate_hz(self) -> float:
        if not self._start:
            return 0.0
        dt = max(1e-6, time.time() - self._start)
        return float(self._count) / dt


def _summarize(samples: List[ProcSample]) -> Dict[str, float]:
    if not samples:
        return {"cpu_avg": 0.0, "cpu_p95": 0.0, "cpu_max": 0.0, "rss_avg": 0.0, "rss_max": 0.0}
    cpu = sorted(s.cpu_percent for s in samples)
    rss = [s.rss_mb for s in samples]
    cpu_avg = sum(cpu) / len(cpu)
    rss_avg = sum(rss) / len(rss)
    idx = int(0.95 * (len(cpu) - 1))
    return {
        "cpu_avg": cpu_avg,
        "cpu_p95": cpu[idx],
        "cpu_max": max(cpu),
        "rss_avg": rss_avg,
        "rss_max": max(rss),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Measure panel performance (CPU/RSS + optional camera rate).")
    parser.add_argument("--pid", type=int, default=0, help="Panel PID (auto-detect if 0)")
    parser.add_argument("--match", default="panel_v2", help="Process match string for auto-detect")
    parser.add_argument("--duration", type=float, default=10.0, help="Measurement duration in seconds")
    parser.add_argument("--interval", type=float, default=0.5, help="Sampling interval in seconds")
    parser.add_argument("--camera-topic", default="", help="Optional camera topic to measure rate (ROS 2)")
    parser.add_argument("--out", default="", help="Optional JSON output path")
    args = parser.parse_args()

    pid = args.pid or 0
    if pid <= 0:
        pid = _find_pid(args.match) or 0
    if pid <= 0:
        print("PID no encontrado. Usa --pid o --match.", file=sys.stderr)
        return 2

    samples: List[ProcSample] = []
    last_cpu = None
    cam = CameraRate(args.camera_topic) if args.camera_topic else None
    cam_ok = cam.start() if cam else False

    t_end = time.time() + max(0.2, args.duration)
    while time.time() < t_end:
        sample, last_cpu = _proc_sample(pid, last_cpu)
        if sample:
            samples.append(sample)
        if cam_ok and cam:
            cam.spin_once()
        time.sleep(max(0.05, args.interval))

    if cam:
        cam.stop()
    summary = _summarize(samples)
    out: Dict[str, object] = {
        "pid": pid,
        "duration_sec": args.duration,
        "interval_sec": args.interval,
        "samples": len(samples),
        "summary": summary,
    }
    if cam:
        out["camera_topic"] = args.camera_topic
        out["camera_hz"] = cam.rate_hz() if cam_ok else 0.0
        out["camera_ok"] = cam_ok

    print(json.dumps(out, indent=2))
    if args.out:
        try:
            with open(args.out, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=2)
        except Exception as exc:
            print(f"No se pudo escribir {args.out}: {exc}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
