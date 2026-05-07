#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/generate_latency_table.py
# Contenido: F8 — script standalone para generar tabla de latencias en MD/LaTeX.
"""Genera tabla de latencias por fase desde snapshots PhaseTimings.

Lee uno o más archivos JSONL (p.ej. el ``events.jsonl`` del
evidence_logger) y agrupa los eventos ``kind=phase_timings`` (o un
JSON con campo ``data`` que contenga un snapshot de PhaseTimings)
para producir una tabla agregada con samples / mean / min / max /
success_rate por fase.

Uso CLI:

    python3 -m ur5_tools.generate_latency_table \
        --input historico/2026-05-01/events.jsonl \
        --format markdown \
        --output report/ARTICULO/tablas/latencies.md

    # múltiples sesiones:
    python3 -m ur5_tools.generate_latency_table \
        --input historico/2026-05-01/*.jsonl \
        --format latex

Formats: ``markdown``, ``latex``, ``json``. Default: markdown.

Cero dependencia ROS — sólo stdlib.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional

from ur5_tools.evidence_helpers import aggregate_phase_timings


def load_snapshots_from_jsonl(path: Path) -> List[Dict]:
    """Lee eventos JSONL y extrae snapshots de PhaseTimings.

    Acepta dos formatos por evento:
      * Evento del evidence_logger: ``{"kind": "phase_timings", "data": <snapshot>}``
      * Snapshot directo: ``{"phases": ..., "phase_order": ...}``

    Líneas no-JSON se ignoran silenciosamente.
    """
    snapshots: List[Dict] = []
    if not path.exists():
        return snapshots
    with open(path, "r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except (json.JSONDecodeError, ValueError):
                continue
            if not isinstance(obj, dict):
                continue
            # Caso 1: evento envuelto.
            if obj.get("kind") == "phase_timings" and isinstance(obj.get("data"), dict):
                snapshots.append(obj["data"])
                continue
            # Caso 2: snapshot directo (tiene "phases" como dict).
            if "phases" in obj and isinstance(obj["phases"], dict):
                snapshots.append(obj)
    return snapshots


def format_seconds(value: Optional[float]) -> str:
    if value is None:
        return "—"
    return f"{value:.3f}"


def format_rate(value: Optional[float]) -> str:
    if value is None:
        return "—"
    return f"{value * 100:.1f}%"


def render_markdown(aggregated: Dict, *, title: Optional[str] = None) -> str:
    """Renderiza la agregación como tabla markdown."""
    lines: List[str] = []
    if title:
        lines.append(f"## {title}\n")
    lines.append(
        f"_Sessions: {aggregated['total_sessions']} "
        f"(successful: {aggregated['successful_sessions']}) | "
        f"Avg total duration: {format_seconds(aggregated['avg_total_duration_sec'])}s_\n"
    )
    lines.append("| Phase | Samples | Mean (s) | Min (s) | Max (s) | Success rate |")
    lines.append("|-------|--------:|---------:|--------:|--------:|-------------:|")
    for phase, stats in aggregated["per_phase"].items():
        lines.append(
            f"| {phase} | {stats['samples']} | "
            f"{format_seconds(stats['mean_sec'])} | "
            f"{format_seconds(stats['min_sec'])} | "
            f"{format_seconds(stats['max_sec'])} | "
            f"{format_rate(stats['success_rate'])} |"
        )
    return "\n".join(lines)


def render_latex(aggregated: Dict, *, title: Optional[str] = None) -> str:
    """Renderiza la agregación como tabla LaTeX (booktabs)."""
    lines: List[str] = []
    lines.append("\\begin{table}[h]")
    lines.append("\\centering")
    if title:
        lines.append(f"\\caption{{{title}}}")
    lines.append("\\begin{tabular}{lrrrrr}")
    lines.append("\\toprule")
    lines.append(
        "Fase & Muestras & Media (s) & Min (s) & Max (s) & Tasa éxito \\\\"
    )
    lines.append("\\midrule")
    for phase, stats in aggregated["per_phase"].items():
        rate = format_rate(stats["success_rate"]).replace("%", "\\%")
        lines.append(
            f"{phase.replace('_', '\\_')} & {stats['samples']} & "
            f"{format_seconds(stats['mean_sec'])} & "
            f"{format_seconds(stats['min_sec'])} & "
            f"{format_seconds(stats['max_sec'])} & {rate} \\\\"
        )
    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}")
    return "\n".join(lines)


def render_json(aggregated: Dict) -> str:
    return json.dumps(aggregated, indent=2)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="generate_latency_table",
        description="Genera tabla de latencias PhaseTimings desde JSONL.",
    )
    p.add_argument(
        "--input",
        nargs="+",
        required=True,
        type=Path,
        help="Uno o más JSONL del evidence_logger (o snapshots directos).",
    )
    p.add_argument(
        "--format",
        choices=("markdown", "latex", "json"),
        default="markdown",
        help="Formato de salida (default: markdown).",
    )
    p.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Archivo de salida; si se omite, escribe a stdout.",
    )
    p.add_argument(
        "--title",
        default="Latencias del orchestrator (PickPlace.action)",
        help="Título para markdown/latex.",
    )
    return p


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    all_snapshots: List[Dict] = []
    for path in args.input:
        all_snapshots.extend(load_snapshots_from_jsonl(path))

    if not all_snapshots:
        print("[generate_latency_table] no snapshots found", file=sys.stderr)
        return 1

    aggregated = aggregate_phase_timings(all_snapshots)

    if args.format == "markdown":
        rendered = render_markdown(aggregated, title=args.title)
    elif args.format == "latex":
        rendered = render_latex(aggregated, title=args.title)
    else:
        rendered = render_json(aggregated)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
        print(f"[generate_latency_table] wrote {args.output}", file=sys.stderr)
    else:
        print(rendered)

    return 0


if __name__ == "__main__":
    sys.exit(main())
