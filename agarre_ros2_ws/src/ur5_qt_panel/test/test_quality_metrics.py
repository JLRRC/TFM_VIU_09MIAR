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
    # F1 audit-v4 (2026-05-07): grasp_inference.py migrado a logging.getLogger.
    # Quedan sólo:
    #  - ur5_tools/generate_latency_table.py: CLI standalone, prints a stderr
    #    son parte del contrato del tool (no es un nodo ROS).
    "ur5_tools/ur5_tools/generate_latency_table.py",
    # F8-step1 (2026-05-08): cli_cycle_timing es CLI tool — prints a stdout
    # son su contrato (consumido por scripts shell).
    "ur5_tools/ur5_tools/cli_cycle_timing.py",
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
    # F5-legacy-removed (2026-05-08): panel_pick_demo.py: 8623 → 536 LOC.
    # Ya bajo umbral global 1500 → REMOVIDO de LEGACY_OVERSIZE_FILES_LOC.
    # Si crece de nuevo, el test general (>1500) lo detectará.

    "ur5_qt_panel/ur5_qt_panel/panel_pick_object.py":  3823,   # F3-step5bis-c: -84 más (moveit_bridge_path body fuera con callables-as-args)
    "ur5_qt_panel/ur5_qt_panel/panel_ros.py":          2149,
    "ur5_tools/ur5_tools/ur5_moveit_bridge.py":        1838,  # F3-step21a/b: +75 LOC docstrings + helpers
    # F3-step6a..f (2026-05-03): _execute_joint_trajectory_action 1.343→1.027 LOC
    # (-316 LOC, -23.5%) extracción de 6 helpers. File creció 1394→1551 por
    # dataclass _FjtPollingThresholds + 5 metodos helper. Trade-off aceptado:
    # función monstruo trozeada a costa de +157 LOC totales del archivo.
    "ur5_tools/ur5_tools/moveit_bridge/executor.py":   1551,
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
    # F5-legacy-removed (2026-05-08): run_pick_demo + closure worker BORRADOS.
    # No hay baselines ya que las funciones no existen. Si vuelven, este test
    # las marcará como new_oversize automáticamente (deseado).
    # F3-step3e (2026-05-03): run_joint_step extracted to pick_demo/joint_step.py.
    # 296 LOC = cuerpo legacy 1:1 con 4 nested defs internos
    # (_local_joint_target_ok / _runtime_target_ok / _strict_refine_runtime_status /
    # _emit_strict_refine_runtime_log). Reducción < 200 requiere extraer las 4
    # nested como sub-helpers del módulo (F3-step3e-bis pendiente).
    "ur5_qt_panel/ur5_qt_panel/pick_demo/joint_step.py::run_joint_step": 259,  # F3-step28a/b: -50 LOC con 2 helpers (retry + grace)
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
    # F3-step18b/c (2026-05-03): audit_emit bajó de 217 → 171 LOC con 2 helpers
    # (_audit_emit_compute_pose_data 99 + _audit_emit_geom_and_panel_traces 40).
    # Removido de baseline (T15 cumplido).
    # F5-step5 (2026-05-03): el factory de runtime nodes creció +20 LOC
    # al añadir Node object_pose_resolver y su parámetro de gating.
    # F3-step29 (2026-05-03): build_runtime_node_actions bajó 238→173 con 2
    # helpers (_build_gripper_attach_backend_node 36 + _build_orchestrator_
    # service_nodes 50). Removido de baseline.
    "ur5_qt_panel/ur5_qt_panel/panel_pick_object.py::run_pick_object": 3702,   # F3-step5bis-c: -88 más (moveit_bridge_path)
    "ur5_qt_panel/ur5_qt_panel/panel_pick_object.py::run_pick_object.worker": 2696,  # F3-step5bis-c: -88 más
    # F3-step4a/b/c (2026-05-03): 3 funciones grandes de panel_pick_object
    # extraídas a pick_object/<modulo>.py con dataclass + función pura.
    # F3-step25a/b (2026-05-03): wait_moveit_result bajó de 263 → 193 LOC con
    # 2 sub-helpers (_wait_moveit_emit_diag_and_extend 79 + _wait_moveit_raise_
    # timeout 65). Removido de baseline (T15 cumplido).
    # Otros gigantes históricos.
    "ur5_tools/ur5_tools/moveit_bridge/executor.py::ExecutorMixin._execute_joint_trajectory_action": 1343,
    # F1.7 audit-v4 (2026-05-08): _execute_moveit_direct creció 183 → 201 LOC
    # al añadir first-attempt-timeout + cancel-on-hang (Bug B fix).
    # Refactor estructural en F3-step6 cuando se parte el método en validate/
    # send/poll/handle helpers (ver audit v4 §F3-step6).
    # F1.22 + F1.23 LIVE (2026-05-08): TF check post-FIRST_ATTEMPT_TIMEOUT
    # + controller restart + final TF check post-retry. Trade-off aceptado:
    # mitigación BUG_CONTROLLER_FEEDBACK_HANG documentada.
    "ur5_tools/ur5_tools/plan_to_pose_server.py::PlanToPoseServer._execute_moveit_direct": 343,
    # F1.24 H9+H10+H11 LIVE (2026-05-08): bypass MoveIt vía FJT directo. La
    # función ejecuta el camino canónico que cierra BUG_CONTROLLER_FEEDBACK_HANG
    # y deja T35 × 3 cycles consecutivos verde. F1.24 H14 (2026-05-08):
    # +8 LOC al cablear path_tolerance_rad (anti-flakiness T35 × 5 stress).
    # Refactor estructural a sub-helpers (build_ik_request / fjt_send_and_wait /
    # fallback_to_moveit) está planificado pero requiere validación live para
    # no romper el camino verde.
    "ur5_tools/ur5_tools/plan_to_pose_server.py::PlanToPoseServer._execute_fjt_direct": 261,
    # F1.24-refactor T15 (2026-05-08): __init__ partido en 3 helpers
    # (_init_declare_and_read_params 124 + _init_setup_tf_and_clients 46 +
    # _init_setup_action_server_and_bridge 55), todos < 200. __init__ queda
    # en 5 LOC. Removido del baseline T15.
    # F3-step9 + step9bis/ter/quater/quintus (2026-05-03): build_main_ui bajó
    # de 733 → 44 LOC con 6 sub-helpers (_build_main_ui_topbar_and_leds 122 +
    # _controls_status_row 130 + _camera_and_objects 98 + _manual_joints_and_
    # info 98 + _info_grid 124 + _robot_baseline_and_tfm 175). Removido de
    # baseline (T15 cumplido para esta función).
    # F3-step6 + step6-bis/ter/quater/v (2026-05-03): ControlPanelV2.__init__
    # bajó de 678 → 177 LOC con 6 sub-helpers _init_*. Removido de la
    # baseline (T15 cumplido para esta función).
    "ur5_tools/ur5_tools/moveit_bridge/moveit_py_planner.py::MoveItPyPlannerMixin._plan_with_moveit_py": 557,
    "ur5_qt_panel/ur5_qt_panel/panel_tfm_execute.py::execute_tfm_world_grasp": 408,  # F3-step33: -54 LOC con _execute_tfm_canonical_pick_object_route (71 LOC)
    # F3-step30 (2026-05-03): _refresh_trace_data bajó 438→116 con 3 helpers
    # (_refresh_trace_data_resolve_object_pose 109 + _resolve_tcp_pose 154 +
    # _emit_audits 117). Removido de baseline.
    # F3-step8 (2026-05-03): UR5MoveItBridge.__init__ bajó de 416 → 158 LOC
    # con 2 sub-helpers (_init_declare_parameters 48 + _init_parse_parameters_
    # and_validate 224). Removido de baseline T15.
    # F3-step8: NUEVA fn pura > 200 (cuerpo legacy de parsing 1:1).
    # F3-step22 (2026-05-03): _init_parse_parameters_and_validate bajó de
    # 224 → 183 LOC con _init_parse_sim_time_and_compatibility (50 LOC).
    # Removido de baseline (T15 cumplido).
    # F3-step11 (a..e) (2026-05-03): apply_ui_state bajó de 375 → 193 LOC con
    # 5 sub-helpers (_apply_ui_state_launchers 83 + _camera_and_release 41 +
    # _pick_buttons 56 + _tfm_buttons 56 + _nav_and_finalize 72). Removido de
    # baseline (T15 cumplido).
    # F3-step26 (2026-05-03): generate_launch_description bajó 369→158 con
    # 3 helpers (_build_bridge_params 54 + _build_system_state_and_attach_extras 17 +
    # _build_launch_arguments 62). Removido de baseline.
    # F3-step10a/b (2026-05-03): build_step_window bajó de 346 → 186 LOC con
    # 2 sub-helpers (_build_step_runtime_section 111 + _build_step_pipeline_
    # history_widgets 90). Removido de baseline (T15 cumplido).
    "ur5_tools/ur5_tools/moveit_bridge/geometry.py::GeometryMixin._compute_approach_ik_seeded": 229,  # F3-step31a: -112 LOC con _eval_ik_seed_candidate (143 LOC)
    "ur5_tools/ur5_tools/moveit_bridge/trajectory_prep.py::TrajectoryPrepMixin._prepare_joint_trajectory_for_controller": 283,  # F3-step32a/b: -56 LOC con 2 helpers
    # F3-step12 (a/b/c) (2026-05-03): on_tfm_grasp_object_clicked bajó de
    # 318 → 199 LOC con 3 sub-helpers (_tfm_grasp_compute_yaw_from_minor_axis 59
    # + _tfm_grasp_compute_width_and_preopen 44 + _tfm_grasp_run_pre_checks 98).
    # Removido de baseline (T15 cumplido).
    # F3-step27 (2026-05-03): GripperAttachBackend.__init__ bajó 342→7 con
    # 3 sub-helpers (_init_declare_parameters 51 + _init_parse_parameters 117 +
    # _init_state_pubs_subs_timers 162). Removido de baseline.
    # F3-step14a/b (2026-05-03): start_moveit_bridge bajó de 287 → 100 LOC
    # con 2 sub-helpers (_build_moveit_bridge_ros_args 101 + _verify_moveit_
    # bridge_ready_async 36). Removido de baseline (T15 cumplido).
    "ur5_qt_panel/ur5_qt_panel/panel_tfm_execute.py::execute_tfm_world_grasp.worker": 286,
    # F3-step15a/b/c (2026-05-03): handle_infer_result bajó de 283 → 154 LOC
    # con 3 sub-helpers (_handle_infer_compute_alignment_2d 41 +
    # _handle_infer_write_audit 79 + _handle_infer_log_postprocess_adjustments 76).
    # Removido de baseline (T15 cumplido).
    # F3-step7 (2026-05-03): RosWorker._thread_main bajó de 279 → 120 LOC
    # con sub-helper _thread_main_create_optional_subs_and_services (167 LOC).
    # Removido de baseline (T15 cumplido).
    # F3-step24a/b (2026-05-03): _resolve_live_object_world bajó de 261 →
    # 179 LOC con 2 sub-helpers (_resolve_live_object_world_snapshot 60 +
    # _resolve_live_object_world_stable 71). Removido de baseline.
    # F3-step4a (2026-05-03): _wait_moveit_result extraído ⇒ wrapper
    # < 200 LOC. Removido de la baseline.
    # F3-step20 (2026-05-03): _prepare_runtime bajó de 250 → 192 LOC con
    # _build_runtime_environment_actions (97 LOC, sólo el ensamblado de
    # SetEnvironmentVariable + SetLaunchConfiguration). Removido de baseline.
    # F3-step19a/b (2026-05-03): _step_window_refresh bajó de 249 → 132 LOC
    # con 2 sub-helpers (_step_window_refresh_history_table 75 + _mesh_align 48).
    # Removido de baseline (T15 cumplido).
    # F3-step13a/b (2026-05-03): start_gazebo bajó de 237 → 120 LOC con
    # 2 sub-helpers (_spawn_gz_gui_client 57 + _prepare_gz_runtime_assets 80).
    # Removido de baseline (T15 cumplido).
    # F3-step23 (2026-05-03): _select_pick_demo_cycle_object_reference bajó
    # de 231 → 189 LOC con _select_compute_stable_promotion_status (76 LOC).
    # Removido de baseline (T15 cumplido).
    # F3-step18a (2026-05-03): tfm_infer bajó de 219 → 118 LOC con
    # _tfm_infer_run_script_mode (127 LOC, branch sin tfm_module). Removido
    # de baseline (T15 cumplido).
    # F3-step17 (a/b/c) (2026-05-03): build_runtime_audit_snapshot bajó de
    # 213 → 114 LOC con 3 sub-helpers (_build_dh_tf_block 57 + _build_joints_
    # control_block 56 + _build_sdf_gazebo_block 73). Removido de baseline.
    # F3-step21a/b (2026-05-03): _pose_callback bajó 210→186 con _pose_
    # callback_publish_rejection (50 LOC); _plan_worker bajó 207→180 con
    # _plan_worker_finalize_result (55 LOC). Removidos de baseline.
    # F3-step16 (2026-05-03): _on_remote_object_select_request bajó de 206 →
    # 139 LOC con _remote_select_defer_until_on_table (86 LOC, module-level).
    # Removido de baseline (T15 cumplido).
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
# T31 — F2 baseline: env reads SCATTERED (excluyendo *_params.py)
# ---------------------------------------------------------------------------
# F2 audit-v4 (2026-05-08): el objetivo F2 es centralizar env reads en
# `*_params.py` tipados (con dataclass frozen + ENV_VAR_BY_FIELD). Las
# reads en `*_params.py` y `panel_settings.py` son el DESTINO esperado;
# las que quedan FUERA son las "scattered" que se deben migrar.
#
# Este test:
#   1. Asegura que las scattered NO suben (regresión).
#   2. Detecta cuando bajan → forzar update del baseline en el mismo commit
#      (documenta progreso F2).
ENV_READS_BASELINE_SCATTERED = 66  # F5-legacy-removed (2026-05-08): 82→66 tras borrar run_pick_demo.
ENV_READS_DRIFT_MARGIN = 0  # no se permite incremento.

# Archivos que SON el destino legítimo de env reads (no contar como scatter).
_F2_ENV_READ_DESTINATIONS = (
    "_params.py",       # *_params.py (panel_pick_demo_params, etc.)
    "panel_settings.py",  # helper centralizado
    "panel_env.py",      # helpers centralizados de env
    "launch_helpers.py", # helpers de launch — env válido aquí
)


def _is_env_read_destination(rel_path: str) -> bool:
    return any(rel_path.endswith(suffix) for suffix in _F2_ENV_READ_DESTINATIONS)


def test_env_reads_total_count_under_baseline() -> None:
    """T31 — env reads SCATTERED (fuera de *_params.py) <= baseline."""
    scattered = 0
    per_file: Dict[str, int] = {}
    for path in _iter_production_py_files():
        rel = _rel(path)
        if _is_env_read_destination(rel):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            continue
        n = text.count("os.environ.get(") + text.count("os.environ[")
        if n:
            per_file[rel] = n
            scattered += n

    upper = ENV_READS_BASELINE_SCATTERED + ENV_READS_DRIFT_MARGIN
    if scattered > upper:
        top = sorted(per_file.items(), key=lambda kv: -kv[1])[:5]
        top_str = "\n  ".join(f"{k}: {v}" for k, v in top)
        raise AssertionError(
            f"Env reads scattered regresion: {scattered} > baseline={upper}.\n"
            f"Top files (mover a *_params.py):\n  {top_str}"
        )
    if scattered < ENV_READS_BASELINE_SCATTERED:
        raise AssertionError(
            f"Env reads scattered bajaron: {scattered} < baseline={ENV_READS_BASELINE_SCATTERED}. "
            f"ACTUALIZA ENV_READS_BASELINE_SCATTERED = {scattered} en el mismo commit "
            "(documenta progreso F2)."
        )


# ---------------------------------------------------------------------------
# T19 — Hash compat de interfaces IDL (.srv / .action)
# ---------------------------------------------------------------------------


# Baseline computado con sha256 sobre cada archivo IDL completo.
# CUALQUIER cambio en una .srv o .action hace fallar este test, forzando
# al desarrollador a actualizar este diccionario en el mismo commit (lo
# que documenta el bump y obliga a revisar consumidores). Si se añade
# una interfaz nueva, también se debe añadir aquí.
INTERFACE_IDL_SHA256: Dict[str, str] = {
    "srv/Attach.srv":                 "9af36c77cbb7e7615cb3f629dc51d68812077a483c25428e81f8545bbbf9a5d2",
    "srv/Close.srv":                  "c6513eddb488591c09025d19676c8eba731a54a20b07450257142d87455e11d1",
    "srv/ComputeApproachPose.srv":    "ffcd64aa0909f923bb70504c8a463070768694b99e9f115fcede9b0e5f1650ac",
    "srv/Detach.srv":                 "19234a211e0624103fd2a5b5c715f210ff334327889aad0e69d760e15c3e7f62",
    "srv/Open.srv":                   "abe79298978b674dc48dbf94169bb3d8dcbf4c7d4c309f326219efc4300a6434",
    "srv/ResolveObjectPoseWorld.srv": "50fc276c79cc4fa74b1a76428c67e5f0155456d7a57abc5e830addbc4319e888",
    "srv/SelectObject.srv":           "fec5f91a17660e04d9ea07ac5d392705353243dfe208234d109d63e3799d0186",
    "srv/SetWidth.srv":               "9243cbd6711949a1feffc036015257e730946b95b649660885a142dfd773f7b4",
    "srv/WorldToBase.srv":            "d0685c8fe466c1a7abe67f15065f1b6e11279ac9c9a302524e277697ca8769f4",
    "action/PickPlace.action":        "ddbfe63ccef2357e5a498d23d2d6c207858ecefd357cced9bca8b8fd57edd166",
    "action/PlanToPose.action":       "db63bbbe4b36c64ad26c061548a9a7349403268526105c2b5ad388b9d704e6d1",
}


def test_interface_idl_hash_compat() -> None:
    """T19 — Detecta drift no documentado en .srv / .action.

    Cualquier cambio (añadir campo, cambiar tipo, renombrar) altera el
    sha256. El test fuerza al dev a actualizar INTERFACE_IDL_SHA256 en el
    mismo PR que toca la interfaz, lo que documenta el bump y previene
    cambios silenciosos que rompen consumidores en otros paquetes.
    """
    import hashlib

    interfaces_root = SRC_ROOT / "ur5_panel_interfaces"
    assert interfaces_root.is_dir(), f"falta {interfaces_root}"

    actual_hashes: Dict[str, str] = {}
    for rel in INTERFACE_IDL_SHA256:
        path = interfaces_root / rel
        assert path.is_file(), f"falta interfaz documentada: {rel}"
        actual_hashes[rel] = hashlib.sha256(path.read_bytes()).hexdigest()

    on_disk: Set[str] = set()
    for sub in ("srv", "action"):
        for p in (interfaces_root / sub).glob(f"*.{sub}"):
            on_disk.add(f"{sub}/{p.name}")
    new_undocumented = sorted(on_disk - set(INTERFACE_IDL_SHA256))
    removed = sorted(set(INTERFACE_IDL_SHA256) - on_disk)
    drift = {
        rel: (INTERFACE_IDL_SHA256[rel], actual_hashes[rel])
        for rel in INTERFACE_IDL_SHA256
        if INTERFACE_IDL_SHA256[rel] != actual_hashes[rel]
    }

    msg_parts: List[str] = []
    if drift:
        lines = [
            f"  {rel}\n    expected={exp}\n    actual  ={act}"
            for rel, (exp, act) in drift.items()
        ]
        msg_parts.append(
            "Interfaces IDL han cambiado SIN actualizar baseline "
            "(documenta el bump, revisa consumidores; luego actualiza "
            "INTERFACE_IDL_SHA256 con los nuevos hashes):\n" + "\n".join(lines)
        )
    if new_undocumented:
        msg_parts.append(
            "Interfaces nuevas SIN entrada en INTERFACE_IDL_SHA256 "
            "(añádelas con su sha256):\n  " + "\n  ".join(new_undocumented)
        )
    if removed:
        msg_parts.append(
            "Interfaces en INTERFACE_IDL_SHA256 que ya no existen en disco "
            "(borra la entrada o restaura el archivo):\n  " + "\n  ".join(removed)
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
