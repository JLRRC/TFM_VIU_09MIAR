#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_workers.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Worker helpers for running shell commands from the panel."""
from __future__ import annotations

import os
import subprocess
from typing import List, Optional

from PyQt5.QtCore import QObject, pyqtSignal, QThread

from .panel_process import run_cmd

TELEPORT_BLOCK_PATTERNS = ("set_pose", "set_entity_pose", "teleport")


def _contains_teleport_keywords(cmd: str) -> bool:
    lowered = cmd.lower()
    return any(kw in lowered for kw in TELEPORT_BLOCK_PATTERNS)


class CmdRunner(QObject):
    line = pyqtSignal(str)
    started = pyqtSignal(str)
    finished = pyqtSignal(str, int)

    def __init__(self) -> None:
        super().__init__()
        self._threads: List[QThread] = []

    def run_cmd(self, cmd: str, timeout: Optional[float] = None, env: Optional[dict] = None):
        """Run a blocking shell command; return (rc, stdout, stderr)."""
        if _contains_teleport_keywords(cmd):
            snippet = cmd.splitlines()[0][:120]
            msg = f"[SAFETY] Teleport detectado: bloqueado ({snippet})"
            self.line.emit(msg)
            return 1, "", msg
        res = run_cmd(cmd, timeout=timeout, capture_output=True, env=env)
        stdout = res.stdout if res.stdout is not None else ""
        stderr = res.stderr if res.stderr is not None else ""
        return res.returncode, stdout, stderr

    def run_stream(self, tag: str, cmd: str, proc_slot: Optional[dict] = None, key: Optional[str] = None):
        thread = QThread()
        worker = _CmdWorker(self, tag, cmd, proc_slot, key)
        worker.moveToThread(thread)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)

        def _cleanup() -> None:
            try:
                self._threads.remove(thread)
            except ValueError:
                pass

        thread.finished.connect(_cleanup)
        thread.started.connect(worker.run)
        self._threads.append(thread)
        thread.start()


class _CmdWorker(QObject):
    finished = pyqtSignal()

    def __init__(self, runner: CmdRunner, tag: str, cmd: str, proc_slot: Optional[dict], key: Optional[str]):
        super().__init__()
        self._runner = runner
        self._tag = tag
        self._cmd = cmd
        self._proc_slot = proc_slot
        self._key = key

    def run(self) -> None:
        def emit(msg: str) -> bool:
            try:
                self._runner.line.emit(msg)
                return True
            except RuntimeError:
                return False

        if _contains_teleport_keywords(self._cmd):
            snippet = self._cmd.splitlines()[0][:120]
            emit(f"[SAFETY] Teleport detectado: bloqueado ({snippet})")
            self.finished.emit()
            return
        if not emit(f"[{self._tag}] $ {self._cmd}"):
            self.finished.emit()
            return
        try:
            self._runner.started.emit(self._tag)
            process = subprocess.Popen(
                ["bash", "-lc", self._cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
            )
            if self._proc_slot is not None and self._key is not None:
                self._proc_slot[self._key] = process
            if process.stdout:
                for line in process.stdout:
                    if not emit(f"[{self._tag}] {line.rstrip()}"):
                        self.finished.emit()
                        return
            rc = process.wait()
            emit(f"[{self._tag}] [EXIT] rc={rc}")
            self._runner.finished.emit(self._tag, rc)
        except Exception as exc:
            emit(f"[{self._tag}] [ERROR] {exc}")
            self._runner.finished.emit(self._tag, -1)
        finally:
            self.finished.emit()
