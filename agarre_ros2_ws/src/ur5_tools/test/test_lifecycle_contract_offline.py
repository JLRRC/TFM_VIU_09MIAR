"""Contract test offline para los 8 LifecycleNodes del workspace.

Audit 2026-05-10 (Action 9). Sin lanzar el stack (no requiere rclpy
con DDS activo). Verifica vía AST + import que cada clase:

  1. Existe en su módulo canónico.
  2. Hereda (directa o indirectamente vía mixins) de LifecycleNode.
  3. Implementa los 5 callbacks canónicos del lifecycle:
     on_configure, on_activate, on_deactivate, on_cleanup, on_shutdown.

La validación runtime de transiciones reales (configure→activate→
deactivate→cleanup) requiere `launch_testing` con el stack vivo y
queda fuera del scope offline (T06 de la matriz de auditoría).
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

WS_SRC = Path(__file__).resolve().parents[2]
assert WS_SRC.name == "src", f"unexpected layout: {WS_SRC}"

# (módulo path, nombre clase esperada)
LIFECYCLE_NODES: list[tuple[Path, str]] = [
    (WS_SRC / "ur5_tools/ur5_tools/tf_geometry_service.py", "TfGeometryService"),
    (
        WS_SRC / "ur5_tools/ur5_tools/object_pose_resolver_service.py",
        "ObjectPoseResolverService",
    ),
    (WS_SRC / "ur5_tools/ur5_tools/gz_pose_bridge.py", "GzPoseBridge"),
    (
        WS_SRC / "ur5_tools/ur5_tools/release_objects_service.py",
        "ReleaseObjectsService",
    ),
    (WS_SRC / "ur5_tools/ur5_tools/system_state_manager.py", "SystemStateManager"),
    (WS_SRC / "ur5_tools/ur5_tools/world_tf_publisher.py", "WorldTfPublisher"),
    (
        WS_SRC / "ur5_tools/ur5_tools/gripper_attach_backend.py",
        "GripperAttachBackend",
    ),
    (
        WS_SRC
        / "tfm_orchestrator/tfm_orchestrator/pick_orchestrator_lifecycle_node.py",
        "PickOrchestratorLifecycleNode",
    ),
]

REQUIRED_CALLBACKS = (
    "on_configure",
    "on_activate",
    "on_deactivate",
    "on_cleanup",
    "on_shutdown",
)


def _find_class_def(tree: ast.Module, name: str) -> ast.ClassDef | None:
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == name:
            return node
    return None


def _bases_string(class_node: ast.ClassDef) -> str:
    """String join of base names for easy substring check (handles attr access)."""
    parts: list[str] = []
    for base in class_node.bases:
        try:
            parts.append(ast.unparse(base))
        except AttributeError:  # pragma: no cover (py<3.9)
            parts.append(getattr(base, "id", ""))
    return " ".join(parts)


def _method_names(class_node: ast.ClassDef) -> set[str]:
    out: set[str] = set()
    for node in class_node.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            out.add(node.name)
    return out


@pytest.mark.parametrize(
    "module_path,class_name",
    LIFECYCLE_NODES,
    ids=[c for _, c in LIFECYCLE_NODES],
)
def test_lifecycle_node_module_exists(module_path: Path, class_name: str) -> None:
    assert module_path.is_file(), f"missing module: {module_path}"


@pytest.mark.parametrize(
    "module_path,class_name",
    LIFECYCLE_NODES,
    ids=[c for _, c in LIFECYCLE_NODES],
)
def test_lifecycle_node_class_inherits(module_path: Path, class_name: str) -> None:
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    cls = _find_class_def(tree, class_name)
    assert cls is not None, (
        f"{module_path.name}: clase {class_name!r} no encontrada"
    )
    bases = _bases_string(cls)
    assert "LifecycleNode" in bases, (
        f"{class_name} en {module_path.name} no hereda de LifecycleNode "
        f"(bases: {bases})"
    )


@pytest.mark.parametrize(
    "module_path,class_name",
    LIFECYCLE_NODES,
    ids=[c for _, c in LIFECYCLE_NODES],
)
def test_lifecycle_node_implements_callbacks(
    module_path: Path, class_name: str
) -> None:
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    cls = _find_class_def(tree, class_name)
    assert cls is not None
    methods = _method_names(cls)
    missing = [name for name in REQUIRED_CALLBACKS if name not in methods]
    assert not missing, (
        f"{class_name} en {module_path.name} no implementa lifecycle "
        f"callbacks: {missing}. Implementados: {sorted(methods & set(REQUIRED_CALLBACKS))}"
    )
