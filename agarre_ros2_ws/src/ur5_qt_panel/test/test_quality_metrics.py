#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_quality_metrics.py
# Contenido: F4 meta-tests de calidad — baseline tracking de LOC, prints,
#            ciclos de imports y env vars indocumentadas.
# Uso breve:
#   PYTHONPATH=. pytest test/test_quality_metrics.py -v
#   - Pure stdlib + AST. No requiere ROS, Qt, Gazebo ni rclpy.
#   - Cada test usa una lista de exenciones (LEGACY_*) para no bloquear
#     el refactor en curso. Cualquier violación FUERA de esas listas
#     hace fallar el test.
"""F4 — Meta-tests de calidad del workspace.

Estos tests miden invariantes globales (LOC, prints, ciclos imports, env
vars indocumentadas) y comparan contra una baseline embebida. La baseline
es la "deuda técnica visible": cuando una fase reduce un valor, basta con
actualizar la lista para apretar el cinturón.

Tests incluidos:
* T13 :: test_no_print_in_production_outside_legacy_list
* T14 :: test_max_loc_per_file_outside_legacy_list
* T15 :: test_max_loc_per_function
* T16 :: test_no_circular_imports
* T5  :: test_env_vars_have_documentation
"""
from __future__ import annotations

import ast
import re
from pathlib import Path
from typing import Dict, List, Set, Tuple

import pytest

# ---------------------------------------------------------------------------
# Resolución de rutas (test vive en agarre_ros2_ws/src/ur5_qt_panel/test/).
# ---------------------------------------------------------------------------
_TEST_FILE = Path(__file__).resolve()
SRC_ROOT = _TEST_FILE.parent.parent.parent  # → agarre_ros2_ws/src
assert SRC_ROOT.name == "src", f"unexpected SRC_ROOT={SRC_ROOT}"

EXCLUDE_DIR_NAMES = {"test", "tests", "build", "install", "log", "__pycache__"}


def _iter_production_py_files() -> List[Path]:
    """Devolver archivos .py de código productivo (sin tests / build)."""
    out: List[Path] = []
    for path in SRC_ROOT.rglob("*.py"):
        parts = set(path.parts)
        if parts & EXCLUDE_DIR_NAMES:
            continue
        out.append(path)
    return sorted(out)


def _rel(path: Path) -> str:
    return str(path.relative_to(SRC_ROOT))


# ---------------------------------------------------------------------------
# T13 — No print() en producción
# ---------------------------------------------------------------------------
# F4 baseline (2026-05-02): archivos legacy con print() pendientes de migrar
# a emit_log_line / self.get_logger(). Cualquier print() en un archivo NO
# listado aquí hace fallar el test.
LEGACY_PRINT_FILES: Set[str] = {
    # F1 cierre (2026-05-03): 19 archivos panel_* migrados a emit_log_line.
    # Quedan sólo:
    #  - tfm_grasping/grasp_inference.py: warning fatal pre-rclpy.init.
    #  - ur5_tools/generate_latency_table.py: CLI standalone, no nodo.
    "tfm_grasping/tfm_grasping/grasp_inference.py",
    "ur5_tools/ur5_tools/generate_latency_table.py",
}

# Líneas con `print(` que son comentarios o docstrings — los ignoramos
# detectando llamadas reales con AST.


def _file_has_print_call(path: Path) -> bool:
    """True si hay una llamada ``print(...)`` real (AST), no comentario."""
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except SyntaxError:
        return False
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name) and func.id == "print":
                return True
    return False


def test_no_print_in_production_outside_legacy_list() -> None:
    """T13 — solo los archivos LEGACY_PRINT_FILES pueden tener ``print(``.

    Cualquier ``print()`` en un archivo nuevo hace fallar el test. Los
    archivos legacy se irán retirando de la lista a medida que se migren
    a ``emit_log_line`` (FASE 1 patrón).
    """
    offenders: List[str] = []
    legacy_clean: List[str] = []
    for path in _iter_production_py_files():
        rel = _rel(path)
        has_print = _file_has_print_call(path)
        if has_print and rel not in LEGACY_PRINT_FILES:
            offenders.append(rel)
        if not has_print and rel in LEGACY_PRINT_FILES:
            legacy_clean.append(rel)

    msg_parts: List[str] = []
    if offenders:
        msg_parts.append(
            "Archivos con print() fuera de LEGACY_PRINT_FILES "
            f"(añade emit_log_line / get_logger):\n  " + "\n  ".join(offenders)
        )
    if legacy_clean:
        msg_parts.append(
            "Archivos LEGACY_PRINT_FILES ya limpios (remover de la lista):\n  "
            + "\n  ".join(legacy_clean)
        )
    assert not msg_parts, "\n\n".join(msg_parts)


# ---------------------------------------------------------------------------
# T14 — LOC por archivo
# ---------------------------------------------------------------------------
# Umbral global (objetivo F3 step1): ningún .py productivo > 1500 LOC.
# Mientras dure el refactor, los archivos de la baseline pueden superar
# el umbral pero NO crecer > +50 LOC desde su valor actual.
MAX_LOC_PER_FILE_GLOBAL = 1500
# Sólo los archivos que SUPERAN el umbral global aparecen aquí. Los que
# están entre 800-1500 ya cumplen el criterio sin necesidad de exención.
LEGACY_OVERSIZE_FILES_LOC: Dict[str, int] = {
    # F3-step1.4 (2026-05-03): +23 LOC por cadena _seed_devs/max/sum
    # (constante _TWO_PI_R + 3 helpers IK) y _close_only(panel)
    # promovidos a module-level. run_pick_demo y worker bajaron -7 LOC.
    "ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py":   8985,   # F3-step3e: -259 LOC (_run_joint_step body fuera) — bajo 9k

    "ur5_qt_panel/ur5_qt_panel/panel_pick_object.py":  4614,
    "ur5_qt_panel/ur5_qt_panel/panel_ros.py":          2149,
    "ur5_tools/ur5_tools/ur5_moveit_bridge.py":        1763,
}
# Margen de crecimiento permitido para legacy oversize (drift cap).
LEGACY_FILE_GROWTH_MARGIN_LOC = 50


def _count_loc(path: Path) -> int:
    return sum(1 for _ in path.read_text(encoding="utf-8").splitlines())


def test_max_loc_per_file_outside_legacy_list() -> None:
    """T14 — ningún .py > 1500 LOC, salvo legacy con margen +50."""
    new_oversize: List[Tuple[str, int]] = []
    grew_too_much: List[Tuple[str, int, int]] = []
    legacy_now_clean: List[str] = []

    for path in _iter_production_py_files():
        rel = _rel(path)
        loc = _count_loc(path)
        baseline = LEGACY_OVERSIZE_FILES_LOC.get(rel)
        if baseline is None:
            if loc > MAX_LOC_PER_FILE_GLOBAL:
                new_oversize.append((rel, loc))
        else:
            allowed = baseline + LEGACY_FILE_GROWTH_MARGIN_LOC
            if loc > allowed:
                grew_too_much.append((rel, loc, allowed))
            elif loc <= MAX_LOC_PER_FILE_GLOBAL:
                legacy_now_clean.append(rel)

    msg_parts: List[str] = []
    if new_oversize:
        msg_parts.append(
            f"Archivos NUEVOS > {MAX_LOC_PER_FILE_GLOBAL} LOC "
            "(divide en módulos antes de mergear):\n  "
            + "\n  ".join(f"{r}: {n} LOC" for r, n in new_oversize)
        )
    if grew_too_much:
        msg_parts.append(
            "Archivos legacy oversize crecieron más allá del margen "
            f"+{LEGACY_FILE_GROWTH_MARGIN_LOC}:\n  "
            + "\n  ".join(
                f"{r}: {n} > {a} (baseline)" for r, n, a in grew_too_much
            )
        )
    if legacy_now_clean:
        msg_parts.append(
            f"Archivos legacy ya bajo {MAX_LOC_PER_FILE_GLOBAL} LOC "
            "(quita de LEGACY_OVERSIZE_FILES_LOC):\n  "
            + "\n  ".join(legacy_now_clean)
        )
    assert not msg_parts, "\n\n".join(msg_parts)


# ---------------------------------------------------------------------------
# T15 — LOC por función
# ---------------------------------------------------------------------------
MAX_LOC_PER_FUNCTION_GLOBAL = 200
# Funciones legacy super-largas conocidas. Clave: "rel/path.py::qualname".
# Baseline registrada por AST (function end_lineno - lineno + 1). Listado
# completo por orden de prioridad de refactor (las top-3 son F3-step1).
LEGACY_OVERSIZE_FUNCTIONS_LOC: Dict[str, int] = {
    # F3 step1 — el monolito que decide el TFM (drift hacia abajo).
    "ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py::run_pick_demo": 8410,   # F3-step3e: -263 LOC (_run_joint_step fuera)
    "ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py::run_pick_demo.worker": 8137,  # F3-step3e: -263 LOC
    # F3-step3e (2026-05-03): run_joint_step extracted to pick_demo/joint_step.py.
    # 296 LOC = cuerpo legacy 1:1 con 4 nested defs internos
    # (_local_joint_target_ok / _runtime_target_ok / _strict_refine_runtime_status /
    # _emit_strict_refine_runtime_log). Reducción < 200 requiere extraer las 4
    # nested como sub-helpers del módulo (F3-step3e-bis pendiente).
    "ur5_qt_panel/ur5_qt_panel/pick_demo/joint_step.py::run_joint_step": 296,
    # F3-step3d (2026-05-03): run_grasp_down_conservative extracted to
    # pick_demo/grasp_down.py. 534 LOC = cuerpo legacy 1:1 con descenso
    # segmentado en waypoints intermedios + IK estricto + fallback preset.
    # Reducción estructural < 200 requiere extraer cada waypoint loop
    # iteration en sub-helper (F3-step3d-bis pendiente).
    "ur5_qt_panel/ur5_qt_panel/pick_demo/grasp_down.py::run_grasp_down_conservative": 534,
    # F3-step3c (2026-05-03): align_demo_grasp_direct extracted to
    # pick_demo/align_grasp.py. 550 LOC = cuerpo legacy 1:1 + sub-bloques
    # GRASP_ALIGN_IK con audit detallado por intento. Reducción estructural
    # < 200 requiere extraer los 4 sub-bloques internos en sub-helpers
    # (F3-step3c-bis pendiente).
    "ur5_qt_panel/ur5_qt_panel/pick_demo/align_grasp.py::align_demo_grasp_direct": 550,
    # F3-step3a (2026-05-03): nueva fn pura ``audit_emit`` en
    # pick_demo/audit_emit.py contiene el cuerpo legacy 1:1 (217 LOC).
    # La rebajada estructural a < 200 LOC requiere extraer las múltiples
    # llamadas a ``_append_trace`` en sub-helpers (F3-step3a-bis pendiente).
    "ur5_qt_panel/ur5_qt_panel/pick_demo/audit_emit.py::audit_emit": 217,
    # F5-step5 (2026-05-03): el factory de runtime nodes creció +20 LOC
    # al añadir Node object_pose_resolver y su parámetro de gating.
    "ur5_bringup/launch/runtime_nodes_factory.py::build_runtime_node_actions": 219,
    "ur5_qt_panel/ur5_qt_panel/panel_pick_object.py::run_pick_object": 4518,
    "ur5_qt_panel/ur5_qt_panel/panel_pick_object.py::run_pick_object.worker": 3512,
    # Otros gigantes históricos.
    "ur5_tools/ur5_tools/moveit_bridge/executor.py::ExecutorMixin._execute_joint_trajectory_action": 1343,
    "ur5_qt_panel/ur5_qt_panel/panel_main_ui.py::build_main_ui": 733,
    "ur5_qt_panel/ur5_qt_panel/panel_v2.py::ControlPanelV2.__init__": 678,
    "ur5_tools/ur5_tools/moveit_bridge/moveit_py_planner.py::MoveItPyPlannerMixin._plan_with_moveit_py": 557,
    "ur5_qt_panel/ur5_qt_panel/panel_tfm_execute.py::execute_tfm_world_grasp": 462,
    "ur5_qt_panel/ur5_qt_panel/panel_trace_callbacks.py::_refresh_trace_data": 438,
    "ur5_tools/ur5_tools/ur5_moveit_bridge.py::UR5MoveItBridge.__init__": 416,
    "ur5_qt_panel/ur5_qt_panel/panel_ui_state.py::apply_ui_state": 375,
    "ur5_bringup/launch/ur5_stack.launch.py::generate_launch_description": 350,
    "ur5_qt_panel/ur5_qt_panel/panel_step_ui.py::build_step_window": 346,
    "ur5_tools/ur5_tools/moveit_bridge/geometry.py::GeometryMixin._compute_approach_ik_seeded": 341,
    "ur5_tools/ur5_tools/moveit_bridge/trajectory_prep.py::TrajectoryPrepMixin._prepare_joint_trajectory_for_controller": 339,
    "ur5_qt_panel/ur5_qt_panel/panel_tfm.py::on_tfm_grasp_object_clicked": 318,
    "ur5_tools/ur5_tools/gripper_attach_backend.py::GripperAttachBackend.__init__": 309,
    "ur5_qt_panel/ur5_qt_panel/panel_launchers.py::start_moveit_bridge": 287,
    "ur5_qt_panel/ur5_qt_panel/panel_tfm_execute.py::execute_tfm_world_grasp.worker": 286,
    "ur5_qt_panel/ur5_qt_panel/panel_tfm_inference.py::handle_infer_result": 283,
    "ur5_qt_panel/ur5_qt_panel/panel_ros.py::RosWorker._thread_main": 279,
    "ur5_qt_panel/ur5_qt_panel/pick_demo/internal_helpers.py::_resolve_live_object_world": 261,
    "ur5_qt_panel/ur5_qt_panel/panel_pick_object.py::run_pick_object.worker._wait_moveit_result": 253,
    "ur5_bringup/launch/ur5_stack.launch.py::_prepare_runtime": 250,
    "ur5_qt_panel/ur5_qt_panel/step_window_refresh.py::_step_window_refresh": 249,
    "ur5_qt_panel/ur5_qt_panel/panel_launchers.py::start_gazebo": 237,
    "ur5_qt_panel/ur5_qt_panel/pick_demo/internal_helpers.py::_select_pick_demo_cycle_object_reference": 231,
    "ur5_qt_panel/ur5_qt_panel/panel_tfm_inference.py::tfm_infer": 219,
    "ur5_qt_panel/ur5_qt_panel/panel_runtime_pose_auditor.py::build_runtime_audit_snapshot": 213,
    "ur5_tools/ur5_tools/ur5_moveit_bridge.py::UR5MoveItBridge._pose_callback": 210,
    "ur5_tools/ur5_tools/ur5_moveit_bridge.py::UR5MoveItBridge._plan_worker": 207,
    "ur5_qt_panel/ur5_qt_panel/panel_remote_callbacks.py::_on_remote_object_select_request": 206,
}
LEGACY_FUNC_GROWTH_MARGIN_LOC = 30


def _function_loc(node: ast.AST) -> int:
    end = getattr(node, "end_lineno", None)
    start = getattr(node, "lineno", None)
    if end is None or start is None:
        return 0
    return int(end) - int(start) + 1


def _walk_functions(tree: ast.AST, parents: Tuple[str, ...] = ()) -> List[Tuple[str, int]]:
    """Yield (qualname, loc) for every FunctionDef/AsyncFunctionDef."""
    out: List[Tuple[str, int]] = []
    for child in ast.iter_child_nodes(tree):
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
            qualname = ".".join(parents + (child.name,))
            out.append((qualname, _function_loc(child)))
            out.extend(_walk_functions(child, parents + (child.name,)))
        elif isinstance(child, ast.ClassDef):
            out.extend(_walk_functions(child, parents + (child.name,)))
    return out


def test_max_loc_per_function() -> None:
    """T15 — ninguna función > 200 LOC salvo legacy con margen +30."""
    new_oversize: List[Tuple[str, int]] = []
    grew_too_much: List[Tuple[str, int, int]] = []
    legacy_now_clean: List[str] = []

    for path in _iter_production_py_files():
        rel = _rel(path)
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError:
            continue
        for qualname, loc in _walk_functions(tree):
            key = f"{rel}::{qualname}"
            baseline = LEGACY_OVERSIZE_FUNCTIONS_LOC.get(key)
            if baseline is None:
                if loc > MAX_LOC_PER_FUNCTION_GLOBAL:
                    new_oversize.append((key, loc))
            else:
                allowed = baseline + LEGACY_FUNC_GROWTH_MARGIN_LOC
                if loc > allowed:
                    grew_too_much.append((key, loc, allowed))
                elif loc <= MAX_LOC_PER_FUNCTION_GLOBAL:
                    legacy_now_clean.append(key)

    msg_parts: List[str] = []
    if new_oversize:
        msg_parts.append(
            f"Funciones NUEVAS > {MAX_LOC_PER_FUNCTION_GLOBAL} LOC "
            "(extrae helpers puros antes de mergear):\n  "
            + "\n  ".join(f"{q}: {n} LOC" for q, n in new_oversize)
        )
    if grew_too_much:
        msg_parts.append(
            "Funciones legacy crecieron más allá del margen "
            f"+{LEGACY_FUNC_GROWTH_MARGIN_LOC}:\n  "
            + "\n  ".join(
                f"{q}: {n} > {a} (baseline)" for q, n, a in grew_too_much
            )
        )
    if legacy_now_clean:
        msg_parts.append(
            f"Funciones legacy ya bajo {MAX_LOC_PER_FUNCTION_GLOBAL} "
            "(quita de LEGACY_OVERSIZE_FUNCTIONS_LOC):\n  "
            + "\n  ".join(legacy_now_clean)
        )
    assert not msg_parts, "\n\n".join(msg_parts)


# ---------------------------------------------------------------------------
# T16 — Sin ciclos en imports intra-paquete
# ---------------------------------------------------------------------------
# Construimos un grafo de imports relativos (`from .foo import ...` o
# `from .foo.bar import ...`) dentro de cada paquete y comprobamos que
# es un DAG via búsqueda de ciclos (Tarjan's SCC simplificado).


def _module_name_of(path: Path) -> str:
    """Convertir agarre_ros2_ws/src/PKG/PKG/foo.py → 'PKG.foo'."""
    rel = path.relative_to(SRC_ROOT)
    parts = list(rel.with_suffix("").parts)
    # parts es como ['ur5_qt_panel', 'ur5_qt_panel', 'foo'] → ['ur5_qt_panel', 'foo']
    if len(parts) >= 2 and parts[0] == parts[1]:
        parts = [parts[0]] + parts[2:]
    return ".".join(parts)


def _imports_of(path: Path, current_pkg: str) -> Set[str]:
    """Devolver módulos importados via ``from . / .foo import ...`` (intra-paquete)."""
    out: Set[str] = set()
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except SyntaxError:
        return out
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.level and node.level >= 1:
            # `from .foo import bar` ⇒ módulo = current_pkg + ".foo"
            if node.module:
                target = f"{current_pkg}.{node.module}"
            else:
                target = current_pkg
            out.add(target)
    return out


def _find_cycle(graph: Dict[str, Set[str]]) -> List[str]:
    """Búsqueda DFS clásica; devuelve un ciclo (lista de nodos) o []."""
    visited: Set[str] = set()
    stack: List[str] = []
    on_stack: Set[str] = set()

    def dfs(node: str) -> List[str]:
        visited.add(node)
        stack.append(node)
        on_stack.add(node)
        for nxt in graph.get(node, ()):
            if nxt in on_stack:
                idx = stack.index(nxt)
                return stack[idx:] + [nxt]
            if nxt not in visited:
                cyc = dfs(nxt)
                if cyc:
                    return cyc
        stack.pop()
        on_stack.discard(node)
        return []

    for n in list(graph.keys()):
        if n not in visited:
            cyc = dfs(n)
            if cyc:
                return cyc
    return []


# Ciclos legacy conocidos (modelados como conjunto de nodos del SCC). Cada
# entrada es un frozenset con los módulos involucrados — el orden no
# importa, lo que importa es el conjunto. Cualquier ciclo detectado cuyo
# conjunto NO esté aquí hace fallar el test.
LEGACY_IMPORT_CYCLES: Set[frozenset] = {
    # ur5_qt_panel — núcleo panel/utils/state interconectado.
    frozenset({"ur5_qt_panel.panel_camera", "ur5_qt_panel.panel_tfm",
               "ur5_qt_panel.panel_tfm_inference"}),
    frozenset({"ur5_qt_panel.panel_controllers_query",
               "ur5_qt_panel.panel_utils",
               "ur5_qt_panel.panel_system_status"}),
    frozenset({"ur5_qt_panel.panel_controllers_query",
               "ur5_qt_panel.panel_utils"}),
    frozenset({"ur5_qt_panel.panel_utils",
               "ur5_qt_panel.panel_table_objects",
               "ur5_qt_panel.panel_system_status",
               "ur5_qt_panel.panel_controllers_query"}),
    frozenset({"ur5_qt_panel.panel_physics", "ur5_qt_panel.panel_v2"}),
    frozenset({"ur5_qt_panel.panel_pixel_geometry",
               "ur5_qt_panel.panel_utils"}),
    frozenset({"ur5_qt_panel.panel_pose_helpers",
               "ur5_qt_panel.panel_tf_discovery"}),
    frozenset({"ur5_qt_panel.panel_pose_helpers",
               "ur5_qt_panel.panel_tf_discovery",
               "ur5_qt_panel.panel_utils"}),
    frozenset({"ur5_qt_panel.panel_tf_discovery",
               "ur5_qt_panel.panel_utils"}),
    frozenset({"ur5_qt_panel.panel_tf_monitor", "ur5_qt_panel.panel_v2"}),
    # ur5_tools — gripper_attach_backend y sus 5 helpers se importan mutuamente.
    frozenset({"ur5_tools.attach_demo_transport",
               "ur5_tools.gripper_attach_backend"}),
    frozenset({"ur5_tools.attach_gz_cli",
               "ur5_tools.gripper_attach_backend"}),
    frozenset({"ur5_tools.attach_pose_lookup",
               "ur5_tools.gripper_attach_backend"}),
    frozenset({"ur5_tools.attach_pose_sub",
               "ur5_tools.gripper_attach_backend"}),
    frozenset({"ur5_tools.attach_set_pose",
               "ur5_tools.gripper_attach_backend"}),
}


def _all_cycles(graph: Dict[str, Set[str]]) -> List[List[str]]:
    """Devolver todos los ciclos simples (DFS exhaustivo, no Tarjan)."""
    cycles: List[List[str]] = []
    seen_sets: Set[frozenset] = set()

    def dfs(node: str, start: str, path: List[str], visited: Set[str]) -> None:
        for nxt in graph.get(node, ()):
            if nxt == start and len(path) >= 1:
                cyc = path + [start]
                key = frozenset(cyc[:-1])
                if key not in seen_sets:
                    seen_sets.add(key)
                    cycles.append(cyc)
            elif nxt not in visited and nxt > start:
                # nxt > start: evita reportar el mismo ciclo desde N nodos.
                visited.add(nxt)
                dfs(nxt, start, path + [nxt], visited)
                visited.discard(nxt)

    for n in sorted(graph.keys()):
        dfs(n, n, [n], {n})
    return cycles


def test_no_circular_imports() -> None:
    """T16 — los imports relativos intra-paquete forman un DAG salvo
    los ciclos en ``LEGACY_IMPORT_CYCLES``.
    """
    graph: Dict[str, Set[str]] = {}
    for path in _iter_production_py_files():
        rel = path.relative_to(SRC_ROOT)
        parts = rel.parts
        if len(parts) < 3:
            continue  # no es un módulo dentro de un paquete Python
        pkg = parts[0]
        if pkg != parts[1]:
            continue  # no es ament_python (PKG/PKG/...) — saltamos
        mod = _module_name_of(path)
        graph.setdefault(mod, set()).update(_imports_of(path, pkg))

    new_cycles: List[List[str]] = []
    legacy_resolved: List[frozenset] = []
    found_sets: Set[frozenset] = set()

    for cyc in _all_cycles(graph):
        nodes = frozenset(cyc[:-1])
        found_sets.add(nodes)
        if nodes not in LEGACY_IMPORT_CYCLES:
            new_cycles.append(cyc)
    for legacy in LEGACY_IMPORT_CYCLES:
        if legacy not in found_sets:
            legacy_resolved.append(legacy)

    msg_parts: List[str] = []
    if new_cycles:
        msg_parts.append(
            "Ciclos de imports NUEVOS:\n  "
            + "\n  ".join(" → ".join(c) for c in new_cycles)
            + "\n\nExtrae símbolos compartidos a un módulo tercero o "
            "usa imports diferidos."
        )
    if legacy_resolved:
        msg_parts.append(
            "Ciclos LEGACY_IMPORT_CYCLES ya resueltos (remover):\n  "
            + "\n  ".join(", ".join(sorted(s)) for s in legacy_resolved)
        )
    assert not msg_parts, "\n\n".join(msg_parts)


# ---------------------------------------------------------------------------
# T5 — Env vars con prefijo PANEL_*/UR5_* documentadas en algún *_params.py
# ---------------------------------------------------------------------------
# F2-step1 dejó deliberadamente fuera unas pocas vars (mutación runtime,
# kill-switches, cross-cutting). Las listamos explícitamente para que el
# test sea informativo: cualquier env var nueva sin documentar falla.
DOCUMENTED_ENV_PREFIXES = ("PANEL_", "UR5_", "ATTACH_BACKEND_")
LEGACY_UNDOC_ENV_VARS: Set[str] = {
    # F2-step1 — kill-switches (valor obligatorio "")
    "PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_XYZ",
    "PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M",
    # F2-step1 — diagnostic-only / overrides puntuales
    "PANEL_PICK_DEMO_DIRECT_IK_ERR_TOL",
    "PANEL_PICK_DEMO_MANUAL_REF_EXTRA_Z_M",
    "PANEL_PICK_DEMO_ON_TOP_XY_TOL_M",
    "PANEL_PICK_DEMO_USE_ORCHESTRATOR",
    # F2-step1 — cross-cutting global
    "PANEL_STRICT_PHYSICS_MODE",
    # F2-step1 — defaults dinámicos basados en moveit_exclusive
    "PANEL_PICK_OBJECT_TRANSPORT_JOINT_FALLBACK",
    "PANEL_PICK_OBJECT_APPROACH_JOINT_FALLBACK",
    "PANEL_PICK_OBJECT_PRE_GRASP_JOINT_FALLBACK",
    "PANEL_PICK_OBJECT_RETURN_TO_MESA",
    "PANEL_PICK_OBJECT_HOME_BEFORE_CESTA",
    "PANEL_PICK_OBJECT_TRANSPORT_JOINT_ONLY",
    "PANEL_PICK_OBJECT_BASKET_JOINT_ONLY",
    "PANEL_PICK_OBJECT_MOVEIT_WAIT_SEC",
    "PANEL_PICK_OBJECT_MOVEIT_WAIT_RECOVERED_SEC",
    # F2-step1 — labels dinámicos por fase (LIFT/TRANSPORT/APPROACH cartesian)
    "PANEL_PICK_OBJECT_LIFT_CARTESIAN",
    "PANEL_PICK_OBJECT_TRANSPORT_CARTESIAN",
    "PANEL_PICK_OBJECT_APPROACH_CARTESIAN",
    # F2-step2 pendiente — panel_settings.py canaliza con helpers _env_*
    # propios; estas vars se leen una sola vez y no encajan en un dataclass
    # frozen separado. Documentadas en panel_settings.PanelSettings.
    "PANEL_BASE_FRAME",
    "PANEL_WORLD_FRAME",
    "PANEL_GRIPPER_TCP_Z_OFFSET",
    # F2-step3 pendiente — utilidades sueltas (panel_config, panel_process,
    # panel_utils, panel_startup, cameras_tab, gripper_geometry).
    "PANEL_CTRL_LIST_RETRY_WINDOW_SEC",
    "PANEL_CTRL_LAST_OK_GRACE_SEC",
    "PANEL_ROS_TIMEOUT",
    "PANEL_STATIC_TF",
    "PANEL_MOVEIT_STARTUP_TIMEOUT_SEC",
    "PANEL_MAX_FPS",
    "PANEL_ROS2_ONLY",
    "PANEL_SINGLE_CAM",
    "UR5_GEOMETRY_URDF_XACRO",
}

ENV_GET_RE = re.compile(
    r"""os\.(?:environ\.get|getenv)\(\s*["']([A-Z_][A-Z0-9_]*)["']""",
    re.MULTILINE,
)


def _collect_documented_env_vars() -> Set[str]:
    """Lee los ENV_VAR_BY_FIELD de todos los *_params.py."""
    documented: Set[str] = set()
    for params_file in SRC_ROOT.rglob("*_params.py"):
        if "test" in params_file.parts:
            continue
        try:
            text = params_file.read_text(encoding="utf-8")
        except OSError:
            continue
        for m in re.finditer(r'"\s*([A-Z_][A-Z0-9_]*)\s*"', text):
            name = m.group(1)
            if name.startswith(DOCUMENTED_ENV_PREFIXES):
                documented.add(name)
    return documented


def _collect_used_env_vars() -> Dict[str, List[str]]:
    """Devuelve {var: [archivos]} para vars con prefijo PANEL_/UR5_/ATTACH_BACKEND_."""
    used: Dict[str, List[str]] = {}
    for path in _iter_production_py_files():
        if path.name.endswith("_params.py"):
            continue  # los params files leen sus propias vars; no contar
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            continue
        for m in ENV_GET_RE.finditer(text):
            name = m.group(1)
            if not name.startswith(DOCUMENTED_ENV_PREFIXES):
                continue
            used.setdefault(name, []).append(_rel(path))
    return used


def test_env_vars_have_documentation() -> None:
    """T5 — toda env var con prefijo PANEL_/UR5_ debe estar en algún *_params.py."""
    documented = _collect_documented_env_vars()
    used = _collect_used_env_vars()

    undocumented: Dict[str, List[str]] = {
        name: files for name, files in used.items()
        if name not in documented and name not in LEGACY_UNDOC_ENV_VARS
    }
    legacy_now_doc = sorted(LEGACY_UNDOC_ENV_VARS & documented)

    msg_parts: List[str] = []
    if undocumented:
        lines = [
            f"  {name}  (en {', '.join(sorted(set(files)))})"
            for name, files in sorted(undocumented.items())
        ]
        msg_parts.append(
            "Env vars usadas SIN entrada en ningún *_params.py "
            "(añade al dataclass + ENV_VAR_BY_FIELD):\n" + "\n".join(lines)
        )
    if legacy_now_doc:
        msg_parts.append(
            "Env vars LEGACY_UNDOC_ENV_VARS ya documentadas (remover de la lista):\n  "
            + "\n  ".join(legacy_now_doc)
        )
    assert not msg_parts, "\n\n".join(msg_parts)


# ---------------------------------------------------------------------------
# Sanity-check del propio test infrastructure
# ---------------------------------------------------------------------------


def test_meta_test_finds_production_files() -> None:
    """Sanity: el discovery debe encontrar al menos algunos archivos
    productivos. Si esto falla es que las rutas se rompieron."""
    files = _iter_production_py_files()
    assert len(files) > 50, f"Solo {len(files)} archivos productivos detectados"
    assert any("panel_v2.py" in str(p) for p in files)
    assert any("ur5_moveit_bridge.py" in str(p) for p in files)
