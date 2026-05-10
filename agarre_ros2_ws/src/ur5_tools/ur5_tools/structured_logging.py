#!/usr/bin/env python3
"""F11 audit (2026-05-10): formato uniforme de logs estructurados.

Convención global:

    [RUN_ID=<id>][NODE=<node>][PHASE=<phase>][STATUS=<STARTED|OK|FAIL|...>] <msg>

Permite filtrar/correlacionar trazas en el log central
(`evidence_logger`) sin parsear strings ad-hoc. Cualquier nodo crítico
del pipeline pick&place debería usar :func:`fmt` en lugar de
strings concatenados a mano.

API:

    >>> ctx = LogCtx(run_id="r-123", node="motion_planner", phase="APPROACH", cycle=1)
    >>> fmt(ctx, "STARTED", "plan to xy=0.4,0.0 z=0.20")
    '[RUN_ID=r-123][NODE=motion_planner][PHASE=APPROACH][STATUS=STARTED][CYCLE=1] plan to xy=0.4,0.0 z=0.20'

    >>> fmt(ctx, "OK", "executed", dt_ms=1234, attempts=2)
    '[RUN_ID=r-123][NODE=motion_planner][PHASE=APPROACH][STATUS=OK][CYCLE=1][dt_ms=1234][attempts=2] executed'

Para `STATUS`, los valores recomendados son:

    STARTED   — fase/operación iniciada
    OK        — completada con éxito
    FAIL      — completada con fallo (incluir reason en kw `reason=...`)
    RETRY     — reintento con back-off
    SKIPPED   — saltada por gate o config
    PROGRESS  — feedback intermedio (incluir kw `progress=0.42`)
    ABORTED   — cancelada por usuario o supervisor

`PHASE` puede ser ``None`` (fuera de fases del pick).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

# Conjunto canónico de status. No se enforce, sólo es referencia.
KNOWN_STATUSES = frozenset({
    "STARTED",
    "OK",
    "FAIL",
    "RETRY",
    "SKIPPED",
    "PROGRESS",
    "ABORTED",
    "INFO",
    "WARN",
    "ERROR",
})


@dataclass(frozen=True)
class LogCtx:
    """Contexto inmutable para una traza estructurada.

    Args:
        run_id: identificador del ciclo pick (e.g. "pick-2026-05-10-001").
        node: nombre del nodo emisor (e.g. "pick_orchestrator").
        phase: fase actual del pipeline o ``None``.
        cycle: número de ciclo dentro del run (1-indexed) o ``None``.
        extra: campos adicionales que se concatenan en cada llamada a
            :func:`fmt` (no obligatorio).
    """

    run_id: str
    node: str
    phase: Optional[str] = None
    cycle: Optional[int] = None
    extra: Dict[str, Any] = field(default_factory=dict)


def fmt(ctx: LogCtx, status: str, msg: str, **kw: Any) -> str:
    """Devuelve una traza con prefijo estructurado."""
    parts = [
        f"[RUN_ID={ctx.run_id}]",
        f"[NODE={ctx.node}]",
        f"[PHASE={ctx.phase or '-'}]",
        f"[STATUS={status}]",
    ]
    if ctx.cycle is not None:
        parts.append(f"[CYCLE={ctx.cycle}]")
    merged: Dict[str, Any] = dict(ctx.extra)
    merged.update(kw)
    if merged:
        kv = ",".join(f"{k}={_render(v)}" for k, v in merged.items())
        parts.append(f"[{kv}]")
    parts.append(" ")
    parts.append(msg)
    return "".join(parts)


def _render(v: Any) -> str:
    """Render compacto para un valor."""
    if isinstance(v, float):
        return f"{v:.4g}"
    if isinstance(v, bool):
        return "1" if v else "0"
    return str(v)


def parse(line: str) -> Dict[str, Any]:
    """Parser tolerante: extrae los campos estructurados de una traza.

    Devuelve un dict con claves ``run_id``, ``node``, ``phase``,
    ``status``, ``cycle`` (None si ausente), ``extra`` (dict) y ``msg``.
    Si la línea no es estructurada, devuelve sólo ``msg`` con la línea
    completa.
    """
    import re

    pat = re.compile(
        r"^\[RUN_ID=(?P<run_id>[^\]]+)\]"
        r"\[NODE=(?P<node>[^\]]+)\]"
        r"\[PHASE=(?P<phase>[^\]]+)\]"
        r"\[STATUS=(?P<status>[^\]]+)\]"
        r"(?:\[CYCLE=(?P<cycle>\d+)\])?"
        r"(?:\[(?P<extra>[^\]]*)\])?"
        r"\s+(?P<msg>.*)$"
    )
    m = pat.match(line)
    if not m:
        return {"msg": line}
    out: Dict[str, Any] = {
        "run_id": m.group("run_id"),
        "node": m.group("node"),
        "phase": None if m.group("phase") == "-" else m.group("phase"),
        "status": m.group("status"),
        "cycle": int(m.group("cycle")) if m.group("cycle") else None,
        "extra": _parse_kv(m.group("extra")),
        "msg": m.group("msg"),
    }
    return out


def _parse_kv(s: Optional[str]) -> Dict[str, str]:
    if not s:
        return {}
    out: Dict[str, str] = {}
    for tok in s.split(","):
        tok = tok.strip()
        if "=" not in tok:
            continue
        k, _, v = tok.partition("=")
        out[k.strip()] = v.strip()
    return out
