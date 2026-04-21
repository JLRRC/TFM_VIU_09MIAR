#!/usr/bin/env python3
"""Resume corridas batch de DIRECTO con conocimiento del smoke visual."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
from typing import Dict, List


SUCCESS_VISUAL_KEYS = {"pre_grasp", "grasp_confirmed", "lift_with_object", "basket_drop"}
SUCCESS_LOG_PATTERNS = (
    re.compile(r"SECUENCIA COMPLETADA EXITOSAMENTE route=basket"),
    re.compile(r"\[PICK\]\[DEMO\] confirmacion cesta OK"),
)
FAIL_LOG_PATTERNS = (
    re.compile(r"ERROR_FATAL:"),
    re.compile(r"system_state no disponible"),
    re.compile(r"demo_carry_validation_failed"),
)


def _parse_summary(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    if not path.is_file():
        return data
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def _load_visual_manifest(run_dir: Path) -> Dict[str, dict]:
    manifest_path = run_dir / "visual_smoke" / "visual_capture_manifest.json"
    if not manifest_path.is_file():
        return {}
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _load_helper_text(run_dir: Path) -> str:
    helper_path = run_dir / "helper.log"
    if not helper_path.is_file():
        return ""
    return helper_path.read_text(encoding="utf-8", errors="replace")


def _as_int(raw: str, default: int = 999) -> int:
    try:
        return int(str(raw).strip())
    except Exception:
        return default


def _summarize_run(run_dir: Path) -> Dict[str, object]:
    summary = _parse_summary(run_dir / "orchestrator_summary.txt")
    visual_manifest = _load_visual_manifest(run_dir)
    helper_text = _load_helper_text(run_dir)
    visual_keys = sorted(key for key in visual_manifest.keys() if key in SUCCESS_VISUAL_KEYS)
    helper_rc = _as_int(summary.get("helper_final_rc", "999"))
    benchmark_rc = _as_int(summary.get("benchmark_rc", "999"))
    visual_ok = set(visual_keys) == SUCCESS_VISUAL_KEYS
    success_hits = [pattern.pattern for pattern in SUCCESS_LOG_PATTERNS if pattern.search(helper_text)]
    fail_hits = [pattern.pattern for pattern in FAIL_LOG_PATTERNS if pattern.search(helper_text)]
    sequence_ok = len(success_hits) == len(SUCCESS_LOG_PATTERNS)
    log_ok = not fail_hits
    passed = helper_rc == 0 and benchmark_rc == 0 and visual_ok and sequence_ok and log_ok
    return {
        "run_dir": str(run_dir),
        "helper_rc": helper_rc,
        "benchmark_rc": benchmark_rc,
        "visual_ok": visual_ok,
        "visual_keys": visual_keys,
        "sequence_ok": sequence_ok,
        "success_hits": success_hits,
        "log_ok": log_ok,
        "fail_hits": fail_hits,
        "passed": passed,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Resume corridas batch de DIRECTO")
    parser.add_argument("batch_dir", help="Directorio que contiene corridas directo_validation_*")
    args = parser.parse_args()

    batch_dir = Path(args.batch_dir).expanduser().resolve()
    run_dirs: List[Path] = sorted(
        path
        for path in batch_dir.iterdir()
        if path.is_dir() and path.name.startswith("directo_validation_")
    )
    results = [_summarize_run(run_dir) for run_dir in run_dirs]
    passed = sum(1 for item in results if item["passed"])
    payload = {
        "batch_dir": str(batch_dir),
        "runs": len(results),
        "passed": passed,
        "failed": len(results) - passed,
        "results": results,
    }
    out_path = batch_dir / "batch_summary.json"
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
