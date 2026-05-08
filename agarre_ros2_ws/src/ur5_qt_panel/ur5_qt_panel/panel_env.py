#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_env.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Environment diagnostics and validation for the UR5 panel."""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional


# ---------------------------------------------------------------------------
# F2-step5 (2026-05-08): helpers tipo-genéricos para env reads.
# Centralizan el patrón os.environ.get + cast + fallback que estaba
# duplicado en panel_settings y otros módulos. Cualquier nuevo read de
# env en el panel debe usar estas funciones (y no `os.environ.get`).
# ---------------------------------------------------------------------------


def env_str(name: str, default: str) -> str:
    """Lee `name` como str. None → default."""
    raw = os.environ.get(name)
    if raw is None:
        return str(default)
    return str(raw)


def env_int(name: str, default: int) -> int:
    """Lee `name` como int. None / inválido → default."""
    raw = os.environ.get(name)
    if raw is None:
        return int(default)
    try:
        return int(raw)
    except (TypeError, ValueError):
        return int(default)


def env_float(name: str, default: float) -> float:
    """Lee `name` como float. None / inválido → default."""
    raw = os.environ.get(name)
    if raw is None:
        return float(default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float(default)


def env_bool(name: str, default: bool) -> bool:
    """Lee `name` como bool con parseo permisivo (1/true/yes/on)."""
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def env_optional_str(name: str) -> Optional[str]:
    """Lee `name` o None. Sin coerción a str si está presente."""
    raw = os.environ.get(name)
    if raw is None:
        return None
    return str(raw)


def env_optional_int(name: str) -> Optional[int]:
    """Lee `name` como int o None. Inválido → None."""
    raw = os.environ.get(name)
    if raw is None:
        return None
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def env_optional_float(name: str) -> Optional[float]:
    """Lee `name` como float o None. Inválido → None."""
    raw = os.environ.get(name)
    if raw is None:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def env_optional_bool(name: str) -> Optional[bool]:
    """Lee `name` como bool o None (no presencia)."""
    raw = os.environ.get(name)
    if raw is None:
        return None
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class EnvDiagnostics:
    ssh_session: bool
    display_available: bool
    use_sim_time: bool
    gz_transport_ip: str
    rmw_implementation: str


def is_ssh_session() -> bool:
    return bool(os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_TTY"))


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY"))


def get_gz_transport_ip(default: str = "127.0.0.1") -> str:
    return env_str("GZ_TRANSPORT_IP", default)


# F2-step3 (audit-v4 2026-05-08): consolidación de env reads de panel_process.
def get_gz_partition(default: str = "") -> str:
    """Lee `GZ_PARTITION` (string vacío si no está)."""
    return env_str("GZ_PARTITION", default).strip()


def get_gz_ip(default: str = "") -> str:
    """Lee `GZ_IP` (string vacío si no está)."""
    return env_str("GZ_IP", default).strip()


def get_panel_ros_timeout(default: float = 1.5) -> float:
    """Lee `PANEL_ROS_TIMEOUT` con fallback default. Vacío → default."""
    raw = env_optional_str("PANEL_ROS_TIMEOUT")
    if not raw:
        return float(default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float(default)


# F2-step4 (audit-v4 2026-05-08): consolidación de UI flags.
def get_panel_max_fps(default: float = 12.0) -> float:
    """Lee `PANEL_MAX_FPS` (default 12). Cap inferior a >0."""
    raw = env_optional_str("PANEL_MAX_FPS")
    if not raw:
        return float(default)
    try:
        v = float(raw)
        return v if v > 0 else float(default)
    except (TypeError, ValueError):
        return float(default)


def is_panel_ros2_only() -> bool:
    """Lee `PANEL_ROS2_ONLY=1` (true si "1")."""
    return env_str("PANEL_ROS2_ONLY", "0") == "1"


def is_panel_single_cam(default_when_ros2_only: bool = True) -> bool:
    """Lee `PANEL_SINGLE_CAM`. Default depende de PANEL_ROS2_ONLY."""
    default = "1" if is_panel_ros2_only() and default_when_ros2_only else "0"
    return env_str("PANEL_SINGLE_CAM", default) == "1"


def is_strict_physics_mode() -> bool:
    """Lee `PANEL_STRICT_PHYSICS_MODE` con parseo permisivo (1/true/yes/on)."""
    return env_bool("PANEL_STRICT_PHYSICS_MODE", default=False)


def effective_mode(mode: str) -> str:
    normalized = (mode or "").strip().lower()
    if normalized.startswith("gui"):
        return "gui"
    if normalized.startswith("auto"):
        return "gui" if has_display() and not is_ssh_session() else "headless"
    return "headless"


def collect_env_diagnostics(*, use_sim_time: bool) -> EnvDiagnostics:
    return EnvDiagnostics(
        ssh_session=is_ssh_session(),
        display_available=has_display(),
        use_sim_time=bool(use_sim_time),
        gz_transport_ip=get_gz_transport_ip(),
        rmw_implementation=env_str("RMW_IMPLEMENTATION", "auto"),
    )


def format_env_diagnostics(diag: EnvDiagnostics) -> str:
    display = "on" if diag.display_available else "off"
    ssh = "yes" if diag.ssh_session else "no"
    sim = "on" if diag.use_sim_time else "off"
    return (
        f"ssh={ssh} display={display} use_sim_time={sim} "
        f"rmw={diag.rmw_implementation} gz_ip={diag.gz_transport_ip}"
    )


def validate_env(mode: str, diag: EnvDiagnostics) -> List[str]:
    warnings: List[str] = []
    if mode == "gui":
        if diag.ssh_session:
            warnings.append("GUI solicitado con sesion SSH activa; forzando headless es recomendado.")
        if not diag.display_available:
            warnings.append("GUI solicitado pero DISPLAY no esta disponible.")
    return warnings
