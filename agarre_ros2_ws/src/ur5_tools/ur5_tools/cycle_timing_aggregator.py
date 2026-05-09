#!/usr/bin/env python3
"""F8 audit-v4 (2026-05-08): aggregator multi-cycle con percentiles p50/p95/p99.

Parsea N log files (cada uno puede contener 1+ cycles), extrae las duraciones
por fase y calcula percentiles agregados via ``perf_helpers.compute_percentiles``.

Output: tabla markdown lista para baseline doc.

Uso:
    cycle_timing_aggregator log1.log log2.log log3.log
    cycle_timing_aggregator --json log1.log log2.log
    cycle_timing_aggregator --baseline-md log1.log log2.log > F8_BASELINE.md
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence

from .cycle_timing_analyzer import CycleTiming, analyze_cycle
from .perf_helpers import compute_percentiles


def _read_log(path: Path) -> List[str]:
    return path.read_text(encoding="utf-8", errors="ignore").splitlines()


def aggregate_phase_samples(cycles: Sequence[CycleTiming]) -> Dict[str, List[float]]:
    """Para cada fase, recolecta la duración en segundos (skip None)."""
    samples: Dict[str, List[float]] = {}
    for c in cycles:
        for p in c.phases:
            if p.duration_sec is None or not p.success:
                continue
            samples.setdefault(p.phase, []).append(float(p.duration_sec))
    return samples


def aggregate_cycle_totals(cycles: Sequence[CycleTiming]) -> List[float]:
    return [
        float(c.total_duration_sec)
        for c in cycles
        if c.total_duration_sec is not None and c.success
    ]


def render_baseline_markdown(
    cycles: Sequence[CycleTiming],
    *,
    title: str = "F8 baseline percentiles",
) -> str:
    samples = aggregate_phase_samples(cycles)
    totals = aggregate_cycle_totals(cycles)
    n_total = len(cycles)
    n_success = sum(1 for c in cycles if c.success)
    out: List[str] = []
    out.append(f"# {title}")
    out.append("")
    out.append(f"- Cycles parseados: **{n_total}**")
    out.append(f"- Cycles success: **{n_success}**")
    out.append("")
    out.append("## Cycle total duration (s)")
    out.append("")
    if totals:
        pct = compute_percentiles(totals) or {}
        out.append("| n | mean | min | p50 | p95 | p99 | max |")
        out.append("|---|---|---|---|---|---|---|")
        out.append(
            "| {n} | {mean:.2f} | {mn:.2f} | {p50:.2f} | {p95:.2f} | "
            "{p99:.2f} | {mx:.2f} |".format(
                n=len(totals),
                mean=sum(totals) / max(1, len(totals)),
                mn=min(totals),
                mx=max(totals),
                p50=pct.get("p50", 0.0),
                p95=pct.get("p95", 0.0),
                p99=pct.get("p99", 0.0),
            )
        )
    else:
        out.append("(no successful cycles)")
    out.append("")
    out.append("## Per-phase duration (s)")
    out.append("")
    if samples:
        out.append("| phase | n | mean | min | p50 | p95 | p99 | max |")
        out.append("|---|---|---|---|---|---|---|---|")
        for phase in sorted(samples):
            vals = samples[phase]
            pct = compute_percentiles(vals) or {}
            out.append(
                "| {phase} | {n} | {mean:.2f} | {mn:.2f} | {p50:.2f} | "
                "{p95:.2f} | {p99:.2f} | {mx:.2f} |".format(
                    phase=phase,
                    n=len(vals),
                    mean=sum(vals) / max(1, len(vals)),
                    mn=min(vals),
                    mx=max(vals),
                    p50=pct.get("p50", 0.0),
                    p95=pct.get("p95", 0.0),
                    p99=pct.get("p99", 0.0),
                )
            )
    else:
        out.append("(no phase samples)")
    return "\n".join(out) + "\n"


def render_json(cycles: Sequence[CycleTiming]) -> str:
    samples = aggregate_phase_samples(cycles)
    totals = aggregate_cycle_totals(cycles)
    payload: Dict[str, Any] = {
        "n_cycles": len(cycles),
        "n_success": sum(1 for c in cycles if c.success),
        "cycle_total_sec": {
            "n": len(totals),
            "samples": totals,
            "percentiles": (compute_percentiles(totals) or {}) if totals else {},
        },
        "per_phase": {
            phase: {
                "n": len(vals),
                "samples": vals,
                "percentiles": compute_percentiles(vals) or {},
            }
            for phase, vals in sorted(samples.items())
        },
    }
    return json.dumps(payload, indent=2)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="cycle_timing_aggregator",
        description=__doc__,
    )
    p.add_argument("logs", nargs="+", type=Path, help="Log files con phase events")
    p.add_argument(
        "--json",
        action="store_true",
        help="Output JSON instead of markdown",
    )
    p.add_argument(
        "--title",
        default="F8 baseline percentiles",
        help="Título del reporte markdown",
    )
    return p


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    cycles: List[CycleTiming] = []
    for log_path in args.logs:
        if not log_path.is_file():
            print(f"WARN: log no existe: {log_path}", file=sys.stderr)
            continue
        lines = _read_log(log_path)
        cycle = analyze_cycle(lines)
        cycles.append(cycle)
    if not cycles:
        print("ERROR: ningún cycle parseado", file=sys.stderr)
        return 1
    if args.json:
        print(render_json(cycles))
    else:
        print(render_baseline_markdown(cycles, title=args.title))
    return 0


if __name__ == "__main__":
    sys.exit(main())
