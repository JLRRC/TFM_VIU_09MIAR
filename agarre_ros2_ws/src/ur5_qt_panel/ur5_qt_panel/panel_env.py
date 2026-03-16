#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_env.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Environment diagnostics and validation for the UR5 panel."""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List


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
    return os.environ.get("GZ_TRANSPORT_IP", default)


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
        rmw_implementation=os.environ.get("RMW_IMPLEMENTATION", "auto"),
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
