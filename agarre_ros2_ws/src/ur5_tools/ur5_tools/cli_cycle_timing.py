#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/cli_cycle_timing.py
"""F8-step1 (2026-05-08) — CLI sobre cycle_timing_analyzer.

Uso:
    cycle_timing log/ros2_launch.log
    cycle_timing log/ros2_launch.log --json
    cycle_timing log/ros2_launch.log --bottleneck

Sin ROS. Sin live. Pure analizador. Útil para comparar runs y subir
los resultados a un report markdown.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List

from .cycle_timing_analyzer import (
    CycleTiming,
    analyze_cycle,
    format_cycle_summary,
)


def cycle_to_json(cycle: CycleTiming) -> str:
    """Serializa el CycleTiming a JSON formateado."""
    return json.dumps(
        {
            "success": cycle.success,
            "n_phases": cycle.n_phases,
            "total_duration_sec": cycle.total_duration_sec,
            "phases": [
                {
                    "phase": p.phase,
                    "start_sec": p.start_sec,
                    "end_sec": p.end_sec,
                    "duration_sec": p.duration_sec,
                    "success": p.success,
                }
                for p in cycle.phases
            ],
        },
        indent=2,
        sort_keys=True,
    )


def find_bottleneck(cycle: CycleTiming) -> str:
    """Devuelve resumen del bottleneck (fase con max duration)."""
    if cycle.n_phases == 0:
        return "no phases detected"
    completed = [p for p in cycle.phases if p.duration_sec is not None]
    if not completed:
        return "no completed phases"
    longest = max(completed, key=lambda p: p.duration_sec or 0.0)
    if cycle.total_duration_sec and cycle.total_duration_sec > 0:
        pct = 100.0 * (longest.duration_sec or 0.0) / cycle.total_duration_sec
        return (
            f"BOTTLENECK: {longest.phase} = "
            f"{longest.duration_sec:.1f}s ({pct:.1f}% del cycle)"
        )
    return f"BOTTLENECK: {longest.phase} = {longest.duration_sec:.1f}s"


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Analiza un ros2_launch.log y reporta timing del cycle pick.",
    )
    parser.add_argument(
        "logfile",
        type=Path,
        help="Path al log file (típicamente log/ros2_launch.log)",
    )
    parser.add_argument(
        "--json", action="store_true",
        help="Output JSON (en lugar de resumen humano).",
    )
    parser.add_argument(
        "--bottleneck", action="store_true",
        help="Imprime sólo el bottleneck (max duration phase).",
    )
    args = parser.parse_args(argv)

    if not args.logfile.exists():
        print(f"ERROR: log file no existe: {args.logfile}", file=sys.stderr)
        return 1
    try:
        with args.logfile.open("r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
    except Exception as exc:
        print(f"ERROR leyendo {args.logfile}: {exc}", file=sys.stderr)
        return 2

    cycle = analyze_cycle(lines)

    if args.bottleneck:
        print(find_bottleneck(cycle))
    elif args.json:
        print(cycle_to_json(cycle))
    else:
        print(format_cycle_summary(cycle))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
