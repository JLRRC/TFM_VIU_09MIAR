#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/cycle_logger.py
# Contenido: JSON per-cycle logger for pick & place — no ROS dependencies.
# Uso breve: from ur5_tools.cycle_logger import CycleLogger
"""Structured JSON logger that records one file per pick cycle.

Each cycle produces a self-contained JSON document:
  {
    "cycle_id":    "20260426_142315_001",
    "start_iso":   "2026-04-26T14:23:15.123456",
    "end_iso":     "2026-04-26T14:23:42.987654",
    "duration_sec": 27.864,
    "outcome":     "SUCCESS" | "FAILED" | "ABORTED" | "UNKNOWN",
    "backend":     "DIRECTO" | "MOVEIT" | "unknown",
    "phases": [
      {"name": "APPROACH_COARSE", "start_sec": 0.0, "end_sec": 3.21, "ok": true, "note": ""},
      ...
    ],
    "metrics": {
      "phase_count": 9,
      "failed_phases": 0,
      "total_duration_sec": 27.864
    },
    "errors": []
  }

Usage::

    logger = CycleLogger(log_root="/path/to/log/cycles")
    logger.begin_cycle(backend="DIRECTO")
    logger.begin_phase("APPROACH_COARSE")
    # ... robot moves ...
    logger.end_phase(ok=True)
    logger.end_cycle(outcome="SUCCESS")
    # file written to log_root/20260426_142315_001.json
"""

from __future__ import annotations

import datetime
import json
import os
import threading
import time
from typing import Any

from .moveit_bridge_utils import bridge_env_str

_DEFAULT_LOG_ROOT = os.path.join(
    bridge_env_str("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws")),
    "log",
    "cycles",
)
_COUNTER_LOCK = threading.Lock()
_GLOBAL_COUNTER = 0


def _next_cycle_id() -> str:
    global _GLOBAL_COUNTER
    with _COUNTER_LOCK:
        _GLOBAL_COUNTER += 1
        count = _GLOBAL_COUNTER
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{ts}_{count:03d}"


class CycleLogger:
    """Thread-safe per-cycle structured logger.

    Safe to instantiate once at startup and reuse across multiple cycles.
    ``begin_cycle`` / ``end_cycle`` delimit each pick attempt.
    """

    def __init__(self, log_root: str | None = None) -> None:
        self._log_root = log_root or _DEFAULT_LOG_ROOT
        self._lock = threading.Lock()
        self._reset()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def begin_cycle(self, *, backend: str = "unknown") -> str:
        """Start a new cycle. Returns the cycle_id."""
        with self._lock:
            self._reset()
            self._cycle_id = _next_cycle_id()
            self._start_mono = time.monotonic()
            self._start_iso = datetime.datetime.now().isoformat()
            self._backend = str(backend)
        return self._cycle_id

    def begin_phase(self, name: str) -> None:
        """Mark the start of a named phase."""
        with self._lock:
            if self._current_phase is not None:
                self._flush_current_phase(ok=None, note="implicitly_ended")
            self._current_phase = {
                "name": str(name),
                "_start_mono": time.monotonic() - self._start_mono,
                "ok": None,
                "note": "",
            }

    def end_phase(self, *, ok: bool = True, note: str = "") -> None:
        """Mark the end of the current phase."""
        with self._lock:
            self._flush_current_phase(ok=ok, note=note)

    def add_error(self, message: str) -> None:
        """Record a freeform error string for the current cycle."""
        with self._lock:
            self._errors.append(str(message))

    def add_metric(self, key: str, value: Any) -> None:
        """Record a named metric (overrides if key exists)."""
        with self._lock:
            self._extra_metrics[str(key)] = value

    def end_cycle(
        self,
        *,
        outcome: str = "UNKNOWN",
        note: str = "",
    ) -> str | None:
        """Finalise the cycle and write the JSON file.

        Returns the path of the written file, or None on error.
        """
        with self._lock:
            if self._current_phase is not None:
                self._flush_current_phase(ok=None, note="cycle_ended_during_phase")
            end_mono = time.monotonic()
            duration = max(0.0, end_mono - self._start_mono)
            end_iso = datetime.datetime.now().isoformat()
            failed = sum(1 for p in self._phases if p.get("ok") is False)
            doc = {
                "cycle_id": self._cycle_id,
                "start_iso": self._start_iso,
                "end_iso": end_iso,
                "duration_sec": round(duration, 3),
                "outcome": str(outcome).upper(),
                "backend": self._backend,
                "note": str(note),
                "phases": list(self._phases),
                "metrics": {
                    "phase_count": len(self._phases),
                    "failed_phases": failed,
                    "total_duration_sec": round(duration, 3),
                    **self._extra_metrics,
                },
                "errors": list(self._errors),
            }
            return self._write(doc)

    @property
    def cycle_id(self) -> str:
        with self._lock:
            return self._cycle_id

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._cycle_id)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _reset(self) -> None:
        self._cycle_id = ""
        self._start_mono = 0.0
        self._start_iso = ""
        self._backend = "unknown"
        self._phases: list[dict[str, Any]] = []
        self._current_phase: dict[str, Any] | None = None
        self._errors: list[str] = []
        self._extra_metrics: dict[str, Any] = {}

    def _flush_current_phase(self, *, ok: bool | None, note: str | None) -> None:
        if self._current_phase is None:
            return
        p = self._current_phase
        elapsed = time.monotonic() - self._start_mono
        self._phases.append({
            "name": p["name"],
            "start_sec": round(float(p["_start_mono"]), 3),
            "end_sec": round(elapsed, 3),
            "duration_sec": round(elapsed - float(p["_start_mono"]), 3),
            "ok": ok,
            "note": str(note or p.get("note", "")),
        })
        self._current_phase = None

    def _write(self, doc: dict[str, Any]) -> str | None:
        try:
            os.makedirs(self._log_root, exist_ok=True)
            path = os.path.join(self._log_root, f"{doc['cycle_id']}.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(doc, f, indent=2, ensure_ascii=False)
            return path
        except Exception:
            return None
