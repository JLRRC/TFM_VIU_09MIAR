#!/usr/bin/env python3
"""Compat wrapper: genera figuras de resultados desde artefactos experimentales."""

from __future__ import annotations

import subprocess
import sys


def main() -> int:
    cmd = [
        sys.executable,
        "scripts/generate_figures.py",
        "--experiments-root",
        "experiments",
        "--summary",
        "reports/tables/summary_results.csv",
        "--out-dir",
        "reports/figures",
    ]
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
