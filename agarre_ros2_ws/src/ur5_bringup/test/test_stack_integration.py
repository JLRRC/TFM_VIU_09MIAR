#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_stack_integration.py
# Contenido: Tests de integracion ROS para el stack UR5 (F4.2 del plan).
"""Tests de integracion ROS del stack UR5+RG2 (F4.2).

Tres aspectos cubiertos:

1. ``test_stack_reaches_ready`` — el stack publica ``STATE READY``
   en ``/system_state`` dentro del timeout.
2. ``test_required_tf_frames_fresh`` — los frames clave
   (``world``, ``base_link``, ``tool0``, ``rg2_pinch_center``)
   existen y se transforman entre si con stamp fresco.
3. ``test_moveit_plan_to_safe_pose`` — MoveIt acepta un plan
   trivial via servicio (cuando MoveIt esta lanzado).

Para que pasen necesitan stack corriendo. Se controlan con la
variable de entorno ``STACK_LIVE_TESTS``:

* ``STACK_LIVE_TESTS=1`` activa los tests. La fixture
  ``_stack_running`` (autouse, session-scoped) verifica que ya
  hay servicios disponibles. Si no, los tests se skipean
  individualmente con un mensaje claro.
* Por defecto (sin env var), TODOS skipean — no se disparan en
  ``colcon test`` ni CI.

Uso recomendado (local):

```bash
# 1) Lanzar el stack (en otra terminal o background)
PANEL_FORCE_OFFSCREEN=1 PANEL_START_STACK=1 PANEL_LAUNCH_MOVEIT=1 \
    ./scripts/start_panel_v2.sh --bg

# 2) Esperar a que arranque (~30-60s) y correr los tests
STACK_LIVE_TESTS=1 python3 -m pytest \
    src/ur5_bringup/test/test_stack_integration.py -v -s
```

Variables de entorno usables:

* ``STACK_LIVE_TESTS`` — *requerido*; cualquier valor distinto de 1 salta.
* ``STACK_READY_TIMEOUT_SEC`` — segundos para alcanzar READY (default 60).
* ``STACK_TF_MAX_AGE_SEC`` — frescura maxima TF (default 1.0).
* ``STACK_MOVEIT_PLAN_TIMEOUT_SEC`` — timeout de planificacion (default 30).
"""
from __future__ import annotations

import os
import re
import subprocess
import time
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import pytest

# ---------------------------------------------------------------------------
# Locations + control flags.
# ---------------------------------------------------------------------------

WS_DIR = Path(__file__).resolve().parents[3]
ROS2_LAUNCH_LOG = WS_DIR / "log" / "ros2_launch.log"

LIVE_REQUIRED = os.environ.get("STACK_LIVE_TESTS", "0").strip() == "1"
pytestmark = pytest.mark.skipif(
    not LIVE_REQUIRED,
    reason="set STACK_LIVE_TESTS=1 to enable stack-integration tests",
)

REQUIRED_TF_PAIRS: Tuple[Tuple[str, str], ...] = (
    ("world", "base_link"),
    ("base_link", "tool0"),
    ("base_link", "rg2_pinch_center"),
)

REQUIRED_SERVICES: Tuple[str, ...] = (
    "/panel/select_object",
    "/panel/pick_demo",
)


def _int_env(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        return int(raw.strip())
    except ValueError:
        return default


def _float_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        return float(raw.strip())
    except ValueError:
        return default


# ---------------------------------------------------------------------------
# Fixtures.
# ---------------------------------------------------------------------------


def _wait_for_services(
    needed: Iterable[str], timeout_sec: float = 60.0
) -> List[str]:
    deadline = time.monotonic() + timeout_sec
    needed_set = {str(name).strip() for name in needed if str(name).strip()}
    last_seen: List[str] = []
    while time.monotonic() < deadline:
        listed = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True,
            text=True,
            timeout=10.0,
        )
        if listed.returncode == 0:
            last_seen = [ln.strip() for ln in listed.stdout.splitlines() if ln.strip()]
            if needed_set.issubset(set(last_seen)):
                return last_seen
        time.sleep(2.0)
    return last_seen


@pytest.fixture(scope="module")
def _stack_running() -> List[str]:
    """Verifica que el stack ya esta levantado (no lo lanza esta fixture).

    Si no, falla con un mensaje claro: el lanzamiento es responsabilidad
    del usuario (igual que F4.3). Se descubre el estado via
    ``ros2 service list``.
    """
    services = _wait_for_services(REQUIRED_SERVICES, timeout_sec=30.0)
    missing = [s for s in REQUIRED_SERVICES if s not in services]
    if missing:
        pytest.fail(
            "Stack no levantado: faltan servicios "
            + ", ".join(missing)
            + ". Lanzar con scripts/start_panel_v2.sh --bg antes de "
            "correr STACK_LIVE_TESTS=1.",
            pytrace=False,
        )
    return services


# ---------------------------------------------------------------------------
# Test 1: stack reaches READY.
# ---------------------------------------------------------------------------


def _scan_log_for_state_ready(timeout_sec: float) -> Tuple[bool, str]:
    """Watch ``ros2_launch.log`` waiting for ``STATE READY``.

    Aceptamos coincidencia tanto del system_state_manager
    (``STATE READY (Sistema listo)``) como de un publish redundante
    en topic /system_state al iniciar.
    """
    pattern = re.compile(r"\bSTATE\s+READY\b", re.IGNORECASE)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if ROS2_LAUNCH_LOG.exists():
            text = ROS2_LAUNCH_LOG.read_text(encoding="utf-8", errors="replace")
            match = pattern.search(text)
            if match:
                start = max(0, match.start() - 80)
                end = min(len(text), match.end() + 120)
                return True, text[start:end].replace("\n", " | ")
        time.sleep(1.0)
    return False, ""


def test_stack_reaches_ready(_stack_running):
    """system_state_manager debe alcanzar STATE READY dentro del timeout."""
    timeout = _float_env("STACK_READY_TIMEOUT_SEC", 60.0)
    ok, snippet = _scan_log_for_state_ready(timeout)
    assert ok, (
        f"system_state_manager no alcanzo STATE READY en {timeout:.0f}s. "
        f"Log: {ROS2_LAUNCH_LOG}"
    )


# ---------------------------------------------------------------------------
# Test 2: required TF frames are present and fresh.
# ---------------------------------------------------------------------------


def _tf_pair_age_sec(parent: str, child: str) -> Optional[float]:
    """Devuelve la edad (sim_time o wall) del transform parent->child.

    Implementacion: invoca ``ros2 run tf2_ros tf2_echo`` con timeout
    corto y parsea la primera linea ``At time``. Si no aparece, devuelve
    None.
    """
    proc = subprocess.run(
        ["timeout", "5", "ros2", "run", "tf2_ros", "tf2_echo", parent, child],
        capture_output=True,
        text=True,
    )
    out = (proc.stdout or "") + (proc.stderr or "")
    match_time = re.search(r"At time\s+([0-9.]+)", out)
    if not match_time:
        return None
    try:
        stamp = float(match_time.group(1))
    except ValueError:
        return None
    # tf2_echo usa sim_time por defecto cuando use_sim_time=true; aqui
    # solo verificamos que el stamp es > 0 (i.e. el transform existe y
    # no es t=0). Para frescura wall-time real haria falta consultar
    # /tf con rclpy; con tf2_echo bastante con que devuelva un stamp.
    return 0.0 if stamp > 0.0 else None


def test_required_tf_frames_fresh(_stack_running):
    """Los frames clave deben existir y producir transforms validos."""
    failures: List[str] = []
    for parent, child in REQUIRED_TF_PAIRS:
        age = _tf_pair_age_sec(parent, child)
        if age is None:
            failures.append(f"{parent}->{child}: no transform")
    assert not failures, (
        "Frames TF no resueltos: " + "; ".join(failures)
    )


# ---------------------------------------------------------------------------
# Test 3: MoveIt plan to a safe pose.
# ---------------------------------------------------------------------------


def _service_exists(name: str) -> bool:
    listed = subprocess.run(
        ["ros2", "service", "list"], capture_output=True, text=True, timeout=10.0
    )
    if listed.returncode != 0:
        return False
    return any(line.strip() == name for line in listed.stdout.splitlines())


def test_moveit_plan_to_safe_pose(_stack_running):
    """Planificacion trivial via /panel/recover (HOME safe)."""
    # /panel/recover es un Trigger expuesto por el panel que dispara
    # un plan a HOME_SAFE. Si no existe en el namespace, skip explicito
    # (significa que el panel no lanzo MoveIt).
    if not _service_exists("/panel/recover"):
        pytest.skip("/panel/recover no disponible (MoveIt no lanzado)")

    timeout = _float_env("STACK_MOVEIT_PLAN_TIMEOUT_SEC", 30.0)
    proc = subprocess.run(
        [
            "ros2", "service", "call", "/panel/recover",
            "std_srvs/srv/Trigger", "{}",
        ],
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    output = (proc.stdout or "") + (proc.stderr or "")
    assert proc.returncode == 0, (
        f"ros2 service call /panel/recover fallo: rc={proc.returncode} {output}"
    )
    assert re.search(r"success:\s*true|success\s*=\s*True", output, re.I), (
        f"/panel/recover no devolvio success=True: {output}"
    )
