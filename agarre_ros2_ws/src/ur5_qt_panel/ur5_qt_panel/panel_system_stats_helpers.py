#!/usr/bin/env python3
"""F7 (auditoría 2026-05-10): system stats + stale processes helpers.

Funciones que actualizan los labels de CPU/RAM/Load en la UI y detectan
procesos zombi del proyecto que sobreviven a sesiones anteriores del
panel. Cada función recibe ``panel`` como primer argumento.

Extraídas de ``panel_status_mgmt.py`` líneas ~514–657. ``panel_status_mgmt``
re-exporta cada símbolo para preservar la API que usan los mixins.
"""
from __future__ import annotations

import os
from typing import List, Set, Tuple

from .logging_utils import emit_log_line
from .panel_config import STALE_PROCESS_GRACE_SEC

try:
    import psutil
except ImportError:
    psutil = None  # type: ignore[assignment]

from PyQt5.QtWidgets import QLabel


def _log_exception(context: str, exc: Exception) -> None:
    """Copia local para evitar dependencia circular con panel_status_mgmt."""
    emit_log_line(f"[STATUS_MGMT][ERROR][{context}] {exc}")


def _update_system_stats(panel):
    """Actualizar labels de CPU/RAM/Load, tolerando ausencia de psutil."""
    cpu_txt = "CPU  --"
    ram_txt = "RAM  --"
    load_txt = "Load  --"
    cpu_alert = False
    ram_alert = False
    load_alert = False
    stale_count = 0
    cores = max(1, os.cpu_count() or 1)
    try:
        if psutil:
            cpu = psutil.cpu_percent(interval=None)
            vm = psutil.virtual_memory()
            used_gb = vm.used / (1024 ** 3)
            total_gb = vm.total / (1024 ** 3)
            ram_txt = f"RAM  {used_gb:.1f}/{total_gb:.1f} GB ({vm.percent:.0f}%)"
            cpu_txt = f"CPU  {cpu:.0f}%"
            cpu_alert = cpu >= 85
            ram_alert = vm.percent >= 90
        else:
            # Fallback simple usando loadavg
            load1, load5, load15 = os.getloadavg()
            cores = max(1, os.cpu_count() or 1)
            cpu_txt = f"CPU  {load1 / cores * 100:.0f}%"
    except Exception as exc:
        _log_exception("update_system_stats cpu/ram", exc)
    try:
        load1, load5, load15 = os.getloadavg()
        load_txt = f"Load  {load1:.2f} {load5:.2f} {load15:.2f}"
        if not psutil:
            cores = max(1, os.cpu_count() or 1)
        load_alert = load1 >= max(4.0, cores * 1.5)
    except Exception as exc:
        _log_exception("update_system_stats loadavg", exc)
    try:
        stale_count, _stale_hint = panel._detect_stale_processes()
    except Exception as exc:
        _log_exception("update_system_stats stale procs", exc)
        stale_count = 0
    health_alert = stale_count > 0
    health_txt = "Proc.Zombis activos" if health_alert else "Todo OK"
    panel._set_stat_label(panel.sys_cpu_lbl, cpu_txt, cpu_alert)
    panel._set_stat_label(panel.sys_ram_lbl, ram_txt, ram_alert)
    panel._set_stat_label(panel.sys_load_lbl, load_txt, load_alert)
    panel._set_stat_label(panel.sys_health_lbl, health_txt, health_alert)


def _set_stat_label(panel, label: QLabel, text: str, alert: bool):
    color = "#dc2626" if alert else "#0f172a"
    label.setStyleSheet(f"font-size:11px; color:{color};")
    label.setText(text)


def _known_process_pids(panel) -> Set[int]:
    pids = {os.getpid()}
    if psutil:
        try:
            for parent in psutil.Process(os.getpid()).parents():
                pids.add(parent.pid)
        except Exception as exc:
            _log_exception("list parent processes", exc)
    for proc in (
        panel.gz_proc,
        panel.gz_gui_proc,
        panel.bridge_proc,
        panel.bag_proc,
        panel.moveit_proc,
        panel.moveit_bridge_proc,
        panel.release_service_proc,
        panel.rsp_proc,
    ):
        if proc is None:
            continue
        try:
            if proc.pid:
                pids.add(proc.pid)
        except Exception as exc:
            _log_exception("read proc pid", exc)
            continue
    return pids


def _list_stale_processes(panel) -> List[Tuple[int, str, str]]:
    """Listar procesos del proyecto que no pertenecen al panel actual."""
    if not psutil:
        return []
    ws_dir = os.path.realpath(panel.ws_dir)
    grace_sec = max(0.0, STALE_PROCESS_GRACE_SEC)
    cutoff = panel._panel_start_ts - grace_sec
    ignore_pids = panel._known_process_pids()
    patterns = (
        "ur5_qt_panel",
        "ur5_tools",
        "ur5_moveit_bridge",
        "release_objects_service",
        "ur5_moveit_config",
        "gz-transport-topic",
        "ros_gz_bridge",
        "parameter_bridge",
        "gz sim",
        "gzserver",
        "gzclient",
        "ign gazebo",
        "robot_state_publisher",
        "controller_manager",
        "spawner",
        "move_group",
    )
    stale = []
    for proc in psutil.process_iter(["pid", "cmdline", "name", "status"]):
        try:
            pid = proc.info["pid"]
            if pid in ignore_pids:
                continue
            try:
                if proc.create_time() >= cutoff:
                    continue
            except Exception as exc:
                _log_exception("read proc create_time", exc)
            cmdline = proc.info.get("cmdline") or []
            cmd = " ".join(cmdline) if cmdline else (proc.info.get("name") or "")
            cmd = cmd.strip()
            if not cmd:
                continue
            cmd_lower = cmd.lower()
            if ws_dir in cmd:
                stale.append((pid, cmd, proc.info.get("status") or ""))
                continue
            if any(pat in cmd_lower for pat in patterns):
                stale.append((pid, cmd, proc.info.get("status") or ""))
                continue
            if proc.info.get("status") == psutil.STATUS_ZOMBIE and "ros2" in cmd_lower:
                stale.append((pid, cmd, proc.info.get("status") or ""))
        except Exception as exc:
            _log_exception("scan stale process", exc)
            continue
    return stale


def _detect_stale_processes(panel) -> Tuple[int, str]:
    """Detecta procesos del proyecto que no pertenecen al panel actual."""
    stale = panel._list_stale_processes()
    if not stale:
        return 0, ""
    sample_cmd = stale[0][1]
    hint = sample_cmd.split()[0] if sample_cmd else ""
    return len(stale), hint


__all__ = [
    "_update_system_stats",
    "_set_stat_label",
    "_known_process_pids",
    "_list_stale_processes",
    "_detect_stale_processes",
]
