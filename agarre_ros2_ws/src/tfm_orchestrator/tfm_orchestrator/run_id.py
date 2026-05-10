#!/usr/bin/env python3
"""F5 (auditoría 2026-05-10): generación + formato del PICK_RUN_ID.

Contexto:
    Hasta esta fase, los logs del pipeline pick & place llevaban tags
    como ``[PICK_DEMO][ORCH][DONE]`` pero NO había un identificador
    estable propagado entre nodos: correlacionar un evento del
    orchestrator con uno del attach backend o del evidence_logger
    requería emparejar timestamps a mano.

    Este módulo introduce el ``PICK_RUN_ID``: 8 chars hex generados al
    inicio de cada ejecución del action ``/pick_place``. El orchestrator
    es el dueño y lo publica en ``/pick/run_id`` (latched). Cualquier
    consumidor (evidence_logger, paneles, tests) puede suscribirse y
    etiquetar sus líneas con el ID activo.

Diseño:
    Helper PURO — cero dependencia ROS. Permite tests unitarios y reuso
    desde cualquier proceso (no sólo nodos rclpy). El nodo ROS lo importa
    y lo combina con el publisher correspondiente.

Formato canónico de línea de log:

    [PICK_RUN_ID=<id8>][PHASE=<name>][NODE=<short>][STATUS=<S/F/W/I>] msg key=val ...

Donde STATUS = STARTED / FINISHED / FAILED / WARN / INFO (1 letra cada uno).
Idioma de los key=val es libre (sin espacios en val).
"""
from __future__ import annotations

import re
import uuid
from typing import Iterable, Mapping, Optional

#: Longitud canónica del ID de pick (8 chars hex).
RUN_ID_LENGTH: int = 8

#: Status válidos para format_log_line. Cada uno se serializa al primer
#: char (S/F/W/I/D) en la línea formateada — útil para grep rápido.
VALID_STATUS = frozenset({"STARTED", "FINISHED", "FAILED", "WARN", "INFO", "DEBUG"})

# Caracteres prohibidos en valores key=val (rompen el parser).
_FORBIDDEN_VALUE_CHARS = re.compile(r"[\s\[\]=]")


def generate_run_id() -> str:
    """Genera un PICK_RUN_ID nuevo.

    Returns:
        8 caracteres hex en minúsculas (4 bytes de entropía). Ejemplos
        válidos: ``"a3b8c9d1"``, ``"00ff00ff"``.

    Implementación:
        ``uuid.uuid4().hex[:8]`` — 32 bits de entropía es suficiente
        para distinguir ejecuciones humanas en una misma sesión sin
        colisiones prácticas (~65k IDs antes de colisión esperada).
    """
    return uuid.uuid4().hex[:RUN_ID_LENGTH]


def is_valid_run_id(value: object) -> bool:
    """Comprueba si ``value`` parece un PICK_RUN_ID válido.

    Reglas: string de longitud RUN_ID_LENGTH compuesto sólo por
    caracteres hex en minúsculas. Útil para validar IDs leídos de
    logs / topics antes de propagarlos.
    """
    if not isinstance(value, str) or len(value) != RUN_ID_LENGTH:
        return False
    return all(c in "0123456789abcdef" for c in value)


def _format_kv(extra: Optional[Mapping[str, object]]) -> str:
    if not extra:
        return ""
    parts = []
    for key, val in extra.items():
        sval = str(val)
        # Si el valor contiene espacios o brackets, lo entrecomillamos
        # para preservar el parsing posterior.
        if _FORBIDDEN_VALUE_CHARS.search(sval):
            sval = '"' + sval.replace('"', "'") + '"'
        parts.append(f"{key}={sval}")
    return " ".join(parts)


def format_log_line(
    run_id: str,
    phase: str,
    node: str,
    status: str,
    msg: str,
    **extra: object,
) -> str:
    """Formatea una línea de log canónica con PICK_RUN_ID.

    Args:
        run_id: ID de la sesión (debería pasar ``is_valid_run_id``,
            pero NO se enforce aquí — un ID inválido se propaga tal
            cual para no perder información en producción).
        phase: nombre de la fase del FSM. Debería coincidir con
            ``PickPhase.name`` (HOME_INITIAL, APPROACH, GRASP, etc.).
        node: identificador corto del nodo emisor. Convención:
            ``orch``, ``attach``, ``plan``, ``evidence``, ``panel``.
        status: uno de VALID_STATUS. Se serializa al primer char.
            Si ``status`` no está en VALID_STATUS, se usa la primera
            letra en mayúscula (fallback no destructivo).
        msg: mensaje libre (sin ``\\n``, idealmente sin espacios
            iniciales/finales).
        **extra: pares clave=valor adicionales. Si el valor contiene
            espacios o brackets se entrecomilla automáticamente.

    Returns:
        Línea sin trailing newline.

    Examples:
        >>> format_log_line("a3b8c9d1", "APPROACH", "orch", "STARTED", "dispatch")
        '[PICK_RUN_ID=a3b8c9d1][PHASE=APPROACH][NODE=orch][STATUS=S] dispatch'

        >>> format_log_line("a3b8c9d1", "GRASP", "attach", "WARN",
        ...                 "gate_failed", tcp_obj_dist=0.847, gate_max=1.0)
        '[PICK_RUN_ID=a3b8c9d1][PHASE=GRASP][NODE=attach][STATUS=W] gate_failed tcp_obj_dist=0.847 gate_max=1.0'
    """
    status_char = status[:1].upper() if status else "?"
    base = (
        f"[PICK_RUN_ID={run_id}]"
        f"[PHASE={phase}]"
        f"[NODE={node}]"
        f"[STATUS={status_char}]"
        f" {msg.rstrip()}"
    )
    kv = _format_kv(extra)
    if kv:
        return f"{base} {kv}"
    return base


def parse_log_line(line: str) -> Optional[dict]:
    """Inverso parcial de ``format_log_line`` — útil en tests / análisis.

    Devuelve ``None`` si la línea no encaja con el formato canónico.
    Sólo extrae los 4 brackets canónicos y el mensaje libre — los
    pares ``key=val`` quedan dentro del campo ``msg`` para evitar
    parsing ambiguo cuando hay valores entrecomillados.
    """
    pattern = re.compile(
        r"^\[PICK_RUN_ID=(?P<run_id>[0-9a-f]+)\]"
        r"\[PHASE=(?P<phase>[^\]]+)\]"
        r"\[NODE=(?P<node>[^\]]+)\]"
        r"\[STATUS=(?P<status>[^\]]+)\]\s*(?P<msg>.*)$"
    )
    m = pattern.match(line)
    if m is None:
        return None
    return dict(m.groupdict())


def short_status(status: str) -> str:
    """Devuelve el char de 1 letra usado en STATUS=. Útil para tests."""
    return status[:1].upper() if status else "?"


__all__: Iterable[str] = (
    "RUN_ID_LENGTH",
    "VALID_STATUS",
    "generate_run_id",
    "is_valid_run_id",
    "format_log_line",
    "parse_log_line",
    "short_status",
)
