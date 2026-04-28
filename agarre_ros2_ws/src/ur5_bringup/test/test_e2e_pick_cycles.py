#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_e2e_pick_cycles.py
# Contenido: E2E pytest equivalente a validate_pick_3_cycles.sh.
"""E2E test del ciclo pick_demo (sustituye validate_pick_3_cycles.sh).

Cada test:

1. (Re)lanza el stack con ``start_panel_v2.sh --bg`` y env vars
   estandarizados (panel offscreen + auto release).
2. Espera a que ``/panel/select_object`` y ``/panel/pick_demo``
   esten disponibles.
3. Espera a que el panel logue ``moveit_state=READY`` (timeout
   tolerante; si no aparece se continua porque el flujo direct
   tampoco depende de MoveIt).
4. Llama ``/panel/select_object {name: pick_demo}``.
5. Llama ``/panel/pick_demo``.
6. Vigila el log de ``ros2_launch.log`` esperando uno de:

   * ``SECUENCIA COMPLETADA EXITOSAMENTE`` (PASS)
   * ``carry_coherence_failed | [PICK_OBJ][ABORT] |
     [PICK][DIRECT][ABORT] | Error en pick objeto |
     [PICK_OBJ][FAIL_CLASS]`` (FAIL)
   * timeout (`PICK_VALIDATE_WAIT_LOOPS` segundos)

7. Para el panel y assertea PASS.

**Skip behaviour**: el test se salta si ``PICK_E2E_LIVE`` no esta
seteado a ``1``. Esto evita que se dispare en CI o en `colcon test`
por defecto. Para correrlo localmente:

```bash
PICK_E2E_LIVE=1 python3 -m pytest \
    src/ur5_bringup/test/test_e2e_pick_cycles.py -q -s
```

Variables de entorno usables:

* ``PICK_E2E_LIVE`` — *requerido*; cualquier valor distinto de 1 salta.
* ``PICK_VALIDATE_CYCLES`` — numero de ciclos (default 3).
* ``PICK_VALIDATE_WAIT_LOOPS`` — segundos de espera por ciclo (default 600).
* ``PICK_VALIDATE_REQUIRE_PASS`` — si se setea a 0, solo emite warning
  si fallan ciclos en lugar de fallar el test (debug mode).
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Tuple

import pytest

# ---------------------------------------------------------------------------
# Ubicaciones reproducibles (independientes del cwd del runner).
# ---------------------------------------------------------------------------

WS_DIR = Path(__file__).resolve().parents[3]
START_SCRIPT = WS_DIR / "scripts" / "start_panel_v2.sh"
STOP_SCRIPT = WS_DIR / "scripts" / "stop_panel_v2.sh"
ROS2_LAUNCH_LOG = WS_DIR / "log" / "ros2_launch.log"
SUMMARY_DIR = WS_DIR / "auditoria"

PASS_PATTERN = re.compile(r"SECUENCIA COMPLETADA EXITOSAMENTE")
FAIL_PATTERN = re.compile(
    r"carry_coherence_failed"
    r"|\[PICK_OBJ\]\[ABORT\]"
    r"|\[PICK\]\[DIRECT\]\[ABORT\]"
    r"|Error en pick objeto"
    r"|\[PICK_OBJ\]\[FAIL_CLASS\]"
)
MOVEIT_READY_PATTERN = re.compile(r"\[PICK\]\[MOVEIT\]\[INIT\].*moveit_state=READY")
ERROR_FATAL_PATTERN = re.compile(r"ERROR_FATAL")

DEFAULT_PANEL_ENV = {
    "PANEL_COLD_BOOT": "1",
    "PANEL_FORCE_OFFSCREEN": "1",
    "PANEL_START_STACK": "1",
    "PANEL_LAUNCH_MOVEIT": "1",
    "MOVEIT_MODE": "move_group",
    "PANEL_AUTO_BRIDGE": "0",
    "PANEL_AUTO_RELEASE_DROP_OBJECTS": "1",
}


# ---------------------------------------------------------------------------
# Skip control + helpers de skip.
# ---------------------------------------------------------------------------


pytestmark = pytest.mark.skipif(
    os.environ.get("PICK_E2E_LIVE", "0").strip() != "1",
    reason="set PICK_E2E_LIVE=1 to enable the E2E pick_demo cycle test",
)


def _bool_env(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off", "")


def _int_env(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        return int(raw.strip())
    except ValueError:
        return default


# ---------------------------------------------------------------------------
# Stack lifecycle.
# ---------------------------------------------------------------------------


def _stop_panel(*, timeout_sec: float = 30.0) -> None:
    if not STOP_SCRIPT.exists():
        return
    subprocess.run(
        ["bash", str(STOP_SCRIPT)],
        cwd=str(WS_DIR.parent),
        timeout=timeout_sec,
        check=False,
        capture_output=True,
    )
    # Pequena gracia para que los hijos terminen.
    time.sleep(2.0)


def _start_panel() -> None:
    assert START_SCRIPT.exists(), f"start_panel_v2.sh missing at {START_SCRIPT}"
    env = os.environ.copy()
    env.update(DEFAULT_PANEL_ENV)
    subprocess.run(
        ["bash", str(START_SCRIPT), "--bg"],
        cwd=str(WS_DIR.parent),
        env=env,
        timeout=120.0,
        check=False,
        capture_output=True,
    )


def _wait_for_services(timeout_sec: float = 120.0) -> bool:
    """Bloquea hasta que /panel/pick_demo y /panel/select_object existan."""
    deadline = time.monotonic() + timeout_sec
    needed = {"/panel/pick_demo", "/panel/select_object"}
    while time.monotonic() < deadline:
        listed = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True,
            text=True,
            timeout=10.0,
        )
        if listed.returncode == 0:
            available = {ln.strip() for ln in listed.stdout.splitlines() if ln.strip()}
            if needed.issubset(available):
                return True
        time.sleep(2.0)
    return False


def _watch_log(
    pattern_pass: re.Pattern,
    patterns_fail: re.Pattern,
    *,
    start_offset: int,
    deadline: float,
) -> Tuple[str, str]:
    """Devuelve (resultado, snippet). resultado in {PASS, FAIL, TIMEOUT}.

    ``start_offset`` es el byte offset desde el cual empezar a leer
    el log: asi ignoramos lineas de runs anteriores.
    """
    last_excerpt: List[str] = []
    while time.monotonic() < deadline:
        if not ROS2_LAUNCH_LOG.exists():
            time.sleep(1.0)
            continue
        with ROS2_LAUNCH_LOG.open("rb") as fh:
            fh.seek(start_offset)
            tail = fh.read().decode("utf-8", errors="replace")
        if pattern_pass.search(tail):
            last_excerpt = [
                line for line in tail.splitlines() if pattern_pass.search(line)
            ][-5:]
            return "PASS", "\n".join(last_excerpt)
        if patterns_fail.search(tail):
            last_excerpt = [
                line for line in tail.splitlines() if patterns_fail.search(line)
            ][-5:]
            return "FAIL", "\n".join(last_excerpt)
        last_excerpt = tail.splitlines()[-20:]
        time.sleep(1.0)
    return "TIMEOUT", "\n".join(last_excerpt)


def _wait_moveit_ready(start_offset: int, timeout_sec: float = 120.0) -> None:
    """Espera (best-effort) a que el panel logue moveit_state=READY.

    No falla si no aparece: el flujo direct no depende estrictamente de
    MoveIt y la espera es solo para reducir flakiness al disparar el pick.
    """
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if not ROS2_LAUNCH_LOG.exists():
            time.sleep(1.0)
            continue
        with ROS2_LAUNCH_LOG.open("rb") as fh:
            fh.seek(start_offset)
            tail = fh.read().decode("utf-8", errors="replace")
        if MOVEIT_READY_PATTERN.search(tail):
            if not ERROR_FATAL_PATTERN.search(tail):
                time.sleep(3.0)
                return
        time.sleep(1.0)


def _call_select_object(name: str) -> Tuple[bool, str]:
    res = subprocess.run(
        [
            "ros2", "service", "call", "/panel/select_object",
            "ur5_panel_interfaces/srv/SelectObject", f"{{name: {name}}}",
        ],
        capture_output=True,
        text=True,
        timeout=15.0,
    )
    out = (res.stdout or "") + (res.stderr or "")
    success = bool(re.search(r"success:\s*true|success\s*=\s*True", out, re.I))
    return success, out


def _call_pick_demo() -> Tuple[bool, str]:
    res = subprocess.run(
        [
            "ros2", "service", "call", "/panel/pick_demo",
            "std_srvs/srv/Trigger", "{}",
        ],
        capture_output=True,
        text=True,
        timeout=15.0,
    )
    out = (res.stdout or "") + (res.stderr or "")
    success = bool(re.search(r"success:\s*true|success\s*=\s*True", out, re.I))
    return success, out


def _log_offset() -> int:
    if not ROS2_LAUNCH_LOG.exists():
        return 0
    return ROS2_LAUNCH_LOG.stat().st_size


# ---------------------------------------------------------------------------
# Test cases.
# ---------------------------------------------------------------------------


def _run_one_cycle(cycle_idx: int) -> Tuple[str, str]:
    """Ejecuta un ciclo completo y devuelve (resultado, snippet del log)."""
    _stop_panel()
    _start_panel()
    if not _wait_for_services(timeout_sec=120.0):
        return "TIMEOUT", "services /panel/pick_demo o /panel/select_object no aparecen"

    start_offset = _log_offset()
    _wait_moveit_ready(start_offset, timeout_sec=120.0)

    sel_ok, sel_out = _call_select_object("pick_demo")
    if not sel_ok:
        return "FAIL_SELECT", sel_out
    time.sleep(1.0)
    pick_ok, pick_out = _call_pick_demo()
    if not pick_ok:
        return "FAIL_PICK", pick_out

    wait_loops = _int_env("PICK_VALIDATE_WAIT_LOOPS", 600)
    deadline = time.monotonic() + max(60, wait_loops)
    return _watch_log(
        PASS_PATTERN,
        FAIL_PATTERN,
        start_offset=start_offset,
        deadline=deadline,
    )


@pytest.fixture(scope="module", autouse=True)
def _ensure_summary_dir():
    SUMMARY_DIR.mkdir(parents=True, exist_ok=True)
    yield
    _stop_panel()


def test_pick_demo_cycles():
    cycles = max(1, _int_env("PICK_VALIDATE_CYCLES", 3))
    require_pass = _bool_env("PICK_VALIDATE_REQUIRE_PASS", True)

    summary_path = (
        SUMMARY_DIR
        / f"pick_{cycles}_cycles_pytest_{time.strftime('%Y%m%d_%H%M%S')}.log"
    )
    results: List[Tuple[int, str]] = []
    pass_count = 0
    with summary_path.open("w", encoding="utf-8") as summary:
        for cycle in range(1, cycles + 1):
            summary.write(f"=== CYCLE {cycle} START ===\n")
            print(f"\n=== CYCLE {cycle} START ===", flush=True)
            t0 = time.monotonic()
            result, excerpt = _run_one_cycle(cycle)
            elapsed = time.monotonic() - t0
            results.append((cycle, result))
            summary.write(f"CYCLE_{cycle}={result} elapsed_sec={elapsed:.1f}\n")
            summary.write(f"{excerpt}\n=== CYCLE {cycle} END ===\n\n")
            print(
                f"CYCLE_{cycle}={result} elapsed={elapsed:.1f}s",
                flush=True,
            )
            print(excerpt, flush=True)
            print(f"=== CYCLE {cycle} END ===\n", flush=True)
            if result == "PASS":
                pass_count += 1

        summary.write(f"SUMMARY PASS={pass_count}/{cycles}\n")
        summary.write(f"SUMMARY_FILE={summary_path}\n")

    print(f"SUMMARY PASS={pass_count}/{cycles} (file={summary_path})", flush=True)
    failed = [str(c) for c, r in results if r != "PASS"]
    if failed and require_pass:
        pytest.fail(
            f"{len(failed)} ciclos no PASS: {','.join(failed)}; "
            f"detalles en {summary_path}",
            pytrace=False,
        )
    elif failed:
        sys.stderr.write(
            f"WARN: {len(failed)} ciclos no PASS pero PICK_VALIDATE_REQUIRE_PASS=0; "
            f"detalles en {summary_path}\n"
        )
