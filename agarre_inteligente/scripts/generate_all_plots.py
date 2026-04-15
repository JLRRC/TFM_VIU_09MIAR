#!/usr/bin/env python3
"""Compat wrapper: genera plots cuantitativos y exporta resumen."""

from __future__ import annotations

import subprocess
import sys


def run(cmd):
    print("[RUN]", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main() -> int:
    run([sys.executable, "scripts/summarize_results.py", "--experiments-root", "experiments", "--output", "reports/tables/summary_results.csv"])
    run([sys.executable, "scripts/validate_official_scope.py", "--summary", "reports/tables/summary_results.csv", "--results-by-seed", "reports/tables/results_by_seed.csv"])
    run([sys.executable, "scripts/generate_figures.py", "--experiments-root", "experiments", "--summary", "reports/tables/summary_results.csv", "--out-dir", "reports/figures"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
