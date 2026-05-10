#!/usr/bin/env python3
"""F11 (auditoría 2026-05-10): verifica que los subpaquetes namespace
re-exportan correctamente los símbolos del top-level.

Garantiza que ``from ur5_tools.{domain} import X`` funciona igual que
``from ur5_tools.X_module import X`` para los símbolos documentados
en ``ARCHITECTURE.md``.
"""
from __future__ import annotations


def test_geometry_subpackage_reexports():
    from ur5_tools import geometry as geom
    from ur5_tools.geometry_constants import (
        BASE_LINK_IN_WORLD as direct_BLW,
        BASKET_DROP_WORLD as direct_BDW,
        base_to_world as direct_b2w,
        world_to_base as direct_w2b,
    )
    assert geom.BASE_LINK_IN_WORLD == direct_BLW
    assert geom.BASKET_DROP_WORLD == direct_BDW
    assert geom.base_to_world is direct_b2w
    assert geom.world_to_base is direct_w2b


def test_planning_subpackage_reexports():
    from ur5_tools import planning as pl
    from ur5_tools.plan_to_pose_helpers import (
        extract_ordered_joint_positions as direct_extract,
        parse_plan_to_pose_request as direct_parse,
        select_traj_duration_and_timeout as direct_select,
    )
    from ur5_tools.plan_to_pose_logic import (
        PlanToPoseGoal as direct_Goal,
        PlanToPoseResult as direct_Result,
    )
    assert pl.extract_ordered_joint_positions is direct_extract
    assert pl.parse_plan_to_pose_request is direct_parse
    assert pl.select_traj_duration_and_timeout is direct_select
    assert pl.PlanToPoseGoal is direct_Goal
    assert pl.PlanToPoseResult is direct_Result


def test_gripper_subpackage_reexports_geometry_helpers():
    from ur5_tools import gripper as gr
    from ur5_tools.release_objects_geometry import (
        is_pose_on_table as direct_iset,
        parse_table_geometry_from_sdf as direct_ptg,
        pose_tuple_from_text as direct_ptft,
        quat_from_rpy as direct_qfr,
    )
    assert gr.is_pose_on_table is direct_iset
    assert gr.parse_table_geometry_from_sdf is direct_ptg
    assert gr.pose_tuple_from_text is direct_ptft
    assert gr.quat_from_rpy is direct_qfr


def test_gripper_subpackage_reexports_logic():
    from ur5_tools import gripper as gr
    from ur5_tools.release_objects_logic import (
        compute_missing_required as direct_cmr,
        drop_anchor_cleanup_targets as direct_dact,
        find_drop_anchor_duplicates as direct_fdad,
        parse_world_name_from_sdf as direct_pwns,
        pick_gz_service as direct_pgs,
    )
    assert gr.compute_missing_required is direct_cmr
    assert gr.drop_anchor_cleanup_targets is direct_dact
    assert gr.find_drop_anchor_duplicates is direct_fdad
    assert gr.parse_world_name_from_sdf is direct_pwns
    assert gr.pick_gz_service is direct_pgs


def test_state_subpackage_reexports_evidence_helpers():
    from ur5_tools import state as st
    from ur5_tools.evidence_helpers import (
        compute_session_metrics as direct_csm,
        now_iso as direct_now,
        parse_grasp_result as direct_pgr,
        safe_unique_dir as direct_sud,
    )
    assert st.compute_session_metrics is direct_csm
    assert st.now_iso is direct_now
    assert st.parse_grasp_result is direct_pgr
    assert st.safe_unique_dir is direct_sud


def test_utils_subpackage_reexports_param_utils():
    from ur5_tools import utils as ut
    from ur5_tools.param_utils import (
        read_float_param as direct_rfp,
        read_int_param as direct_rip,
        read_str_list_param as direct_rslp,
        read_str_param as direct_rsp,
    )
    assert ut.read_float_param is direct_rfp
    assert ut.read_int_param is direct_rip
    assert ut.read_str_list_param is direct_rslp
    assert ut.read_str_param is direct_rsp


def test_diagnostics_subpackage_imports_clean():
    """Sin assert sobre símbolos — solo que el subpackage importa."""
    from ur5_tools import diagnostics  # noqa: F401


def test_bridges_subpackage_imports_clean():
    """Sin assert sobre símbolos — bridges/__init__ es minimal."""
    from ur5_tools import bridges  # noqa: F401


def test_all_subpackages_present():
    """Sanity: los 7 subpaquetes existen como atributos del paquete."""
    import ur5_tools
    expected = {
        "geometry",
        "planning",
        "gripper",
        "state",
        "diagnostics",
        "bridges",
        "utils",
    }
    for name in expected:
        # Al hacer 'import ur5_tools.geometry' se debe poder
        # encontrar el subpaquete por __path__/import.
        mod = __import__(f"ur5_tools.{name}", fromlist=[name])
        assert mod is not None, f"subpackage ur5_tools.{name} no importable"
