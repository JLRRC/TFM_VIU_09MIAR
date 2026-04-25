#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_process.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Process and shell helpers for the UR5 panel."""
from __future__ import annotations

import os
import shlex
import shutil
import signal
import subprocess
import time
from typing import List, Optional, Tuple, Union

from .panel_config import FASTRTPS_PROFILES, GZ_PARTITION_FILE
from .logging_utils import timestamped_line

ROS_CMD_TIMEOUT = float(os.environ.get("PANEL_ROS_TIMEOUT", "1.5"))
STDBUF_PREFIX = "stdbuf -oL -eL " if shutil.which("stdbuf") else ""
GZ_LOG_FILTERS = [
    r"libEGL warning: egl: failed to create dri2 screen",
    r"libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device\\.",
]

_DEBUG_EXCEPTIONS = os.environ.get("PANEL_DEBUG_EXCEPTIONS", "").strip() in ("1", "true", "True")


def _log_exception(context: str, exc: Exception) -> None:
    if not _DEBUG_EXCEPTIONS:
        return
    print(timestamped_line(f"[PROCESS][WARN] {context}: {exc}"), flush=True)


def run_cmd(
    cmd: Union[str, List[str]],
    timeout: Optional[float] = None,
    capture_output: bool = True,
    check: bool = False,
    env: Optional[dict] = None,
) -> subprocess.CompletedProcess:
    """Run a shell command via bash -lc with optional timeout and output capture."""
    try:
        if isinstance(cmd, (list, tuple)):
            if timeout is not None:
                res = subprocess.run(
                    cmd,
                    text=True,
                    capture_output=capture_output,
                    timeout=timeout,
                    env=env,
                )
            else:
                res = subprocess.run(
                    cmd,
                    text=True,
                    capture_output=capture_output,
                    env=env,
                )
        else:
            if timeout is not None:
                res = subprocess.run(
                    ["bash", "-lc", cmd],
                    text=True,
                    capture_output=capture_output,
                    timeout=timeout,
                    env=env,
                )
            else:
                res = subprocess.run(
                    ["bash", "-lc", cmd],
                    text=True,
                    capture_output=capture_output,
                    env=env,
                )
        if check and res.returncode != 0:
            raise subprocess.CalledProcessError(res.returncode, cmd, output=res.stdout, stderr=res.stderr)
        return res
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(args=cmd, returncode=124, stdout="", stderr=str(exc))
    except Exception as exc:
        _log_exception("run_cmd", exc)
        return subprocess.CompletedProcess(args=cmd, returncode=1, stdout="", stderr=str(exc))


def run_cmd_output(cmd: str, timeout_sec: float = ROS_CMD_TIMEOUT) -> Tuple[str, Optional[str]]:
    """Run a shell command and capture stdout/stderr with timeout."""
    res = run_cmd(cmd, timeout=timeout_sec, capture_output=True)
    if res.returncode == 124:
        return "", f"timeout {timeout_sec}s"
    if res.stderr:
        return res.stdout or "", res.stderr
    return res.stdout or "", None


def ensure_dir(path: str) -> None:
    """Create directory if absent."""
    os.makedirs(path, exist_ok=True)


def rotate_log(path: str) -> None:
    """Rotate a log file before starting fresh."""
    try:
        if os.path.isfile(path):
            backup = f"{path}.{time.strftime('%Y%m%d_%H%M%S')}.bak"
            shutil.copy2(path, backup)
        with open(path, "w", encoding="utf-8") as f:
            f.write("")
    except Exception as exc:
        _log_exception("rotate_log", exc)


def log_to_file(cmd: str, log_path: str, filter_cmd: Optional[str]) -> str:
    """Build the pipeline that filters and logs a command."""
    ensure_dir(os.path.dirname(log_path))
    redir = f">> '{log_path}' 2>&1"
    if filter_cmd:
        return f"{cmd} 2>&1 | {filter_cmd} {redir}"
    return f"{cmd} {redir}"


def build_log_filter_cmd(
    filters: List[str],
    unbuffered: bool = False,
    include_regex: Optional[str] = None,
) -> str:
    if not filters and not include_regex:
        return ""
    parts: List[str] = []
    if include_regex:
        parts.append(f"grep -E {shlex.quote(include_regex)}")
    if filters:
        expr = "|".join(filters)
        quoted = shlex.quote(expr)
        parts.append(f"grep -Ev {quoted}")
    cmd = " | ".join(parts)
    if unbuffered and STDBUF_PREFIX:
        return f"{STDBUF_PREFIX}{cmd}"
    return cmd


def with_line_buffer(cmd: str) -> str:
    if STDBUF_PREFIX:
        return f"{STDBUF_PREFIX}{cmd}"
    return cmd


def now_tag() -> str:
    """Timestamp used for filenames."""
    return time.strftime("%Y%m%d_%H%M%S")


def safe_topic_name(topic: str) -> str:
    """Sanitize ROS topic names for filenames."""
    return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in topic.strip("/"))


def kill_process_group(proc: subprocess.Popen, label: str, log_fn) -> None:
    """Ensure a child process group is terminated, logging the attempt."""
    if proc is None:
        return
    try:
        proc.terminate()
    except Exception as exc:
        _log_exception("terminate process", exc)
    try:
        pgid = os.getpgid(proc.pid)
        if pgid:
            os.killpg(pgid, signal.SIGTERM)
    except Exception as exc:
        _log_exception("kill process group", exc)
    finally:
        log_fn(f"[{label}] Proceso detenido (pid={getattr(proc, 'pid', 'n/a')}).")


def bash_preamble(ws_dir: str) -> str:
    return (
        "set +u; "
        "export AMENT_TRACE_SETUP_FILES=\"${AMENT_TRACE_SETUP_FILES:-}\"; "
        f"export FASTRTPS_DEFAULT_PROFILES_FILE='{FASTRTPS_PROFILES}'; "
        "export RMW_FASTRTPS_USE_SHM=0; "
        "if [ -z \"${RMW_IMPLEMENTATION:-}\" ]; then "
        "  if [ -f /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ] "
        "|| [ -f /usr/lib/librmw_cyclonedds_cpp.so ] "
        "|| [ -f /usr/lib/x86_64-linux-gnu/librmw_cyclonedds_cpp.so ]; then "
        "    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
        "  fi; "
        "fi; "
        "source /opt/ros/jazzy/setup.bash; "
        f"if [ -f '{ws_dir}/install/setup.bash' ]; then source '{ws_dir}/install/setup.bash'; fi; "
        "set -u; "
        f"cd '{ws_dir}'; "
    )


def read_gz_partition_file() -> str:
    try:
        with open(GZ_PARTITION_FILE, "r", encoding="utf-8") as f:
            return f.read().strip()
    except Exception:
        return ""


def resolve_gz_partition(preferred: str = "") -> str:
    if preferred:
        return preferred
    env_part = os.environ.get("GZ_PARTITION", "").strip()
    if env_part:
        return env_part
    return read_gz_partition_file()


def build_gz_env(partition: str = "") -> str:
    gz_ip = os.environ.get("GZ_IP", "").strip()
    gz_transport_ip = os.environ.get("GZ_TRANSPORT_IP", "").strip()
    env = ""
    if gz_ip:
        env += f"export GZ_IP='{gz_ip}' ; "
    if gz_transport_ip:
        env += f"export GZ_TRANSPORT_IP='{gz_transport_ip}' ; "
    if partition:
        env += f"export GZ_PARTITION='{partition}' ; "
    return env
