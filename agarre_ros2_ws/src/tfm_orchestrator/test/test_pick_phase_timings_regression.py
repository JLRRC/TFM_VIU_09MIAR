#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_pick_phase_timings_regression.py
# Contenido: F4 T17 — pick phase timings regression (skeleton + opt-in live).
"""F4 T17 — Pick phase timings regression.

Compara los tiempos de cada fase del pick con un baseline JSON
(``historico/F9_PERFORMANCE_BASELINE.json``). El test:

  * **Skip si no hay baseline** — comportamiento por defecto en CI offline.
  * **Compara timings live** cuando el evidence_logger ha producido
    `phase_timings` events recientes en `report/runs/<latest>/events.jsonl`.
  * **Tolera ±20%** de variación frente al baseline.

Uso:

  Para generar baseline (one-time, manual tras run live exitoso):
    cp report/runs/<run>/metrics.json historico/F9_PERFORMANCE_BASELINE.json

  Para correr el test offline:
    pytest src/tfm_orchestrator/test/test_pick_phase_timings_regression.py

Si el baseline existe y los timings actuales se desvían ±20%, el test
falla — eso bloquea regresiones de performance no documentadas.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional

import pytest


WS_ROOT = Path(__file__).resolve().parents[3]
REPO_ROOT = WS_ROOT.parent  # /home/laboratorio/TFM
BASELINE_PATH = REPO_ROOT / "historico" / "F9_PERFORMANCE_BASELINE.json"
TIMINGS_DRIFT_RATIO = 0.20  # ±20%


def _find_latest_metrics() -> Optional[Path]:
    """Devuelve el `metrics.json` más reciente en `report/runs/`, o None."""
    runs_dir = REPO_ROOT / "report" / "runs"
    if not runs_dir.exists():
        return None
    metrics = list(runs_dir.rglob("metrics.json"))
    if not metrics:
        return None
    metrics.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return metrics[0]


def _load_phase_timings(metrics_path: Path) -> Dict[str, float]:
    """Extrae `phase → mean_sec` de un metrics.json."""
    try:
        data = json.loads(metrics_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return {}
    out: Dict[str, float] = {}
    per_phase = data.get("per_phase", {})
    if not isinstance(per_phase, dict):
        return out
    for phase, stats in per_phase.items():
        if isinstance(stats, dict) and "mean_sec" in stats:
            try:
                out[str(phase)] = float(stats["mean_sec"])
            except (TypeError, ValueError):
                continue
    return out


def test_t17_phase_timings_regression() -> None:
    """T17 — phase timings actuales dentro de ±20% del baseline."""
    if not BASELINE_PATH.exists():
        pytest.skip(
            f"Baseline ausente: {BASELINE_PATH}. "
            "Genera con: cp report/runs/<latest>/metrics.json "
            f"{BASELINE_PATH.relative_to(REPO_ROOT)}"
        )

    baseline = _load_phase_timings(BASELINE_PATH)
    if not baseline:
        pytest.skip(f"Baseline {BASELINE_PATH} sin per_phase válido")

    latest = _find_latest_metrics()
    if latest is None:
        pytest.skip("No hay metrics.json reciente en report/runs/")

    current = _load_phase_timings(latest)
    if not current:
        pytest.skip(f"Metrics actual {latest} sin per_phase válido")

    deviations: List[str] = []
    for phase, baseline_sec in baseline.items():
        if phase not in current:
            deviations.append(f"phase={phase}: presente en baseline pero ausente en {latest}")
            continue
        current_sec = current[phase]
        if baseline_sec <= 0:
            continue
        ratio = abs(current_sec - baseline_sec) / baseline_sec
        if ratio > TIMINGS_DRIFT_RATIO:
            deviations.append(
                f"phase={phase}: baseline={baseline_sec:.3f}s "
                f"current={current_sec:.3f}s drift={ratio*100:.1f}% "
                f"> {TIMINGS_DRIFT_RATIO*100:.0f}%"
            )

    new_phases = set(current.keys()) - set(baseline.keys())
    for phase in new_phases:
        # Nueva fase no en baseline — no fail, sólo informar.
        # (Si fuera fail, cualquier nuevo helper rompería CI antes de
        # regenerar baseline.)
        pass

    assert not deviations, "Phase timings drift > ±20%:\n  " + "\n  ".join(deviations)


def test_t17_baseline_format_when_present() -> None:
    """T17-format — si el baseline existe, su shape es válido."""
    if not BASELINE_PATH.exists():
        pytest.skip(f"Baseline ausente: {BASELINE_PATH}")
    try:
        data = json.loads(BASELINE_PATH.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        pytest.fail(f"Baseline JSON inválido: {e}")
    assert isinstance(data, dict), "Baseline debe ser dict top-level"
    if "per_phase" in data:
        assert isinstance(data["per_phase"], dict), "per_phase debe ser dict"
        for phase, stats in data["per_phase"].items():
            assert isinstance(phase, str), f"phase key no es str: {phase!r}"
            assert isinstance(stats, dict), (
                f"stats no es dict para {phase}: {type(stats).__name__}"
            )
