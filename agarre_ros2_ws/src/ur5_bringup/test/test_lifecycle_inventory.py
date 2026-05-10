#!/usr/bin/env python3
"""F10 audit (2026-05-10): inventario locked de Node vs LifecycleNode.

Si alguien convierte un Node en LC (o vice versa), este test fallará
y obligará a actualizar el inventario en
``docs/LIFECYCLE_ARCHITECTURE.md`` para que el cambio sea explícito y
rastreable.
"""
from __future__ import annotations

import re
from pathlib import Path

WS = Path(__file__).resolve().parents[3]

# Inventario locked: actualizar este dict si se migra un nodo.
LIFECYCLE_NODES = {
    "src/tfm_orchestrator/tfm_orchestrator/pick_orchestrator_lifecycle_node.py",
    "src/ur5_tools/ur5_tools/gripper_attach_backend.py",
    "src/ur5_tools/ur5_tools/system_state_manager.py",
    "src/ur5_tools/ur5_tools/world_tf_publisher.py",
    "src/ur5_tools/ur5_tools/release_objects_service.py",
    "src/ur5_tools/ur5_tools/tf_geometry_service.py",
    "src/ur5_tools/ur5_tools/object_pose_resolver_service.py",
    "src/ur5_tools/ur5_tools/gz_pose_bridge.py",
}

PLAIN_NODES = {
    # Pendientes de migrar (riesgo + live test budget):
    "src/ur5_tools/ur5_tools/evidence_logger.py",
    "src/ur5_tools/ur5_tools/planning_scene_sync.py",
    "src/ur5_tools/ur5_tools/plan_to_pose_server.py",
    # No migran por diseño (one-shots o probes):
    "src/ur5_tools/ur5_tools/controller_bootstrap.py",
    "src/ur5_tools/ur5_tools/gz_ros_control_guard.py",
    "src/ur5_tools/ur5_tools/clock_probe.py",
    "src/ur5_tools/ur5_tools/tf_probe.py",
    "src/ur5_tools/ur5_tools/jt_smoke_test.py",
}

LC_RE = re.compile(r"\bLifecycleNode\b")
NODE_RE = re.compile(r"^\s*class\s+\w+\s*\(\s*Node\s*\)", re.MULTILINE)


def _has_lifecycle(path: Path) -> bool:
    txt = path.read_text(encoding="utf-8")
    return bool(LC_RE.search(txt) and "LifecycleNode):" in txt or "    LifecycleNode," in txt)


def _is_plain_node(path: Path) -> bool:
    txt = path.read_text(encoding="utf-8")
    return bool(NODE_RE.search(txt))


def test_lifecycle_inventory_locked() -> None:
    for rel in LIFECYCLE_NODES:
        path = WS / rel
        assert path.is_file(), f"Lifecycle node ausente: {rel}"
        assert _has_lifecycle(path), (
            f"{rel} debería ser LifecycleNode (en LIFECYCLE_NODES) pero no lo es. "
            "Si la migración revertió, actualiza docs/LIFECYCLE_ARCHITECTURE.md."
        )


def test_plain_node_inventory_locked() -> None:
    for rel in PLAIN_NODES:
        path = WS / rel
        assert path.is_file(), f"Plain Node ausente: {rel}"
        # Algún _node.py declara Node y otro hereda mixin → buscamos
        # tanto la sintaxis simple como la compuesta.
        txt = path.read_text(encoding="utf-8")
        is_lc = "LifecycleNode" in txt and ("LifecycleNode):" in txt or "LifecycleNode," in txt)
        if is_lc:
            raise AssertionError(
                f"{rel} ahora es LifecycleNode — actualiza el inventario "
                "(LIFECYCLE_NODES vs PLAIN_NODES) en este test y "
                "docs/LIFECYCLE_ARCHITECTURE.md."
            )


def test_total_count_matches_doc() -> None:
    """Total de nodos productivos ≥ 11 (8 LC + 3 pendientes), las 3
    probe utilities no cuentan como pipeline."""
    pipeline_lc_count = len(LIFECYCLE_NODES)
    pipeline_pending_count = len([
        p for p in PLAIN_NODES if any(
            kw in p for kw in ("evidence_logger", "planning_scene_sync", "plan_to_pose_server")
        )
    ])
    assert pipeline_lc_count == 8, (
        f"Esperaba 8 LifecycleNodes en pipeline, hay {pipeline_lc_count}"
    )
    assert pipeline_pending_count == 3, (
        f"Esperaba 3 nodos productivos pendientes de LC, hay {pipeline_pending_count}"
    )
