# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_release_integrity.py
# Contenido: T30 — integridad del repo a nivel release (no de runtime).
"""T30 — release integrity guard.

Valida que los archivos clave del repo a nivel release existen y tienen
estructura mínima coherente. Detecta:

- README.md borrado o vacío.
- CHANGELOG.md borrado o sin tag actual.
- LICENSE borrado.
- docs/ canónico descomprimido (architecture, BUG_*, OPERATION, AUDIT).

Si alguien hace un refactor y borra un doc canónico por accidente, este
test lo detecta. CI rápido (offline).
"""
from __future__ import annotations

from pathlib import Path

# Repo root: ws_dir/../.. (ws_dir es agarre_ros2_ws/).
REPO_ROOT = Path(__file__).resolve().parents[4]
WS_DIR = Path(__file__).resolve().parents[3]


def _read(path: Path) -> str:
    assert path.is_file(), f"archivo no encontrado: {path}"
    return path.read_text(encoding="utf-8", errors="replace")


# ---------------------------------------------------------------------------
# Repo root
# ---------------------------------------------------------------------------


def test_readme_exists_and_has_objetivo_cumplido():
    """README.md debe existir y mencionar el objetivo cumplido."""
    text = _read(REPO_ROOT / "README.md")
    assert "OBJETIVO CUMPLIDO" in text, (
        "README.md debe declarar el estado del objetivo. "
        "Si quitaste el header de cierre, restauralo."
    )
    assert "objetivo-cumplido-pinzas-agarran-objeto-20260507" in text, (
        "README.md debe referenciar el tag de cierre."
    )


def test_changelog_exists_and_has_release_tags():
    """CHANGELOG.md debe existir con los tags de release principales."""
    text = _read(REPO_ROOT / "CHANGELOG.md")
    expected_tags = ["2026-05-07", "OBJETIVO CUMPLIDO"]
    for tag in expected_tags:
        assert tag in text, f"CHANGELOG.md debe mencionar: {tag!r}"


def test_license_exists():
    """LICENSE debe existir (MIT)."""
    text = _read(REPO_ROOT / "LICENSE")
    assert "MIT License" in text, "LICENSE debe ser MIT"


def test_gitignore_excludes_volatile_dirs():
    """gitignore debe excluir directorios volátiles."""
    text = _read(REPO_ROOT / ".gitignore")
    expected = ["BORRAR/", "historico/", "auditoria/"]
    for entry in expected:
        assert entry in text, (
            f".gitignore debe excluir {entry} (evitar commits accidentales)"
        )


# ---------------------------------------------------------------------------
# docs canónicos
# ---------------------------------------------------------------------------


def test_docs_canonicos_existen():
    """Los docs canónicos del proyecto deben existir en docs/."""
    docs_dir = WS_DIR / "docs"
    assert docs_dir.is_dir(), "agarre_ros2_ws/docs/ debe existir"
    expected_docs = [
        "architecture.md",
        "OPERATION.md",
        "AUDIT_20260506.md",
        "CONFIG_HIERARCHY.md",
    ]
    for doc in expected_docs:
        path = docs_dir / doc
        assert path.is_file(), f"falta doc canónico: {path}"


def test_bug_docs_existen():
    """Bugs documentados deben tener su .md."""
    bug_dir = REPO_ROOT / "docs"
    assert bug_dir.is_dir(), "/docs/ del repo root debe existir"
    bugs = ["BUG_BRIDGE_PATH_TOLERANCE.md", "BUG_GRASP_DOWN_TCP_TRUNCATION.md"]
    for bug in bugs:
        assert (bug_dir / bug).is_file(), (
            f"falta doc de bug: {bug_dir / bug}. "
            "Estos docs son evidencia de bugs investigados."
        )


def test_architecture_has_mermaid_diagram():
    """architecture.md debe tener el diagrama mermaid (added v1.2)."""
    text = _read(WS_DIR / "docs" / "architecture.md")
    assert "```mermaid" in text, (
        "architecture.md debe tener el diagrama mermaid del sistema. "
        "Si lo quitaste, restauralo desde el commit v1.2."
    )


# ---------------------------------------------------------------------------
# Workspace structure
# ---------------------------------------------------------------------------


def test_workspace_packages_existen():
    """Los 8 paquetes ROS 2 del workspace deben existir."""
    expected_pkgs = [
        "tfm_grasping",
        "tfm_orchestrator",
        "ur5_bringup",
        "ur5_description",
        "ur5_moveit_config",
        "ur5_panel_interfaces",
        "ur5_qt_panel",
        "ur5_tools",
    ]
    src = WS_DIR / "src"
    for pkg in expected_pkgs:
        assert (src / pkg).is_dir(), f"falta paquete: src/{pkg}"


def test_critical_models_exist():
    """Los modelos URDF + SDF del UR5+RG2 deben existir."""
    assert (WS_DIR / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro").is_file()
    assert (WS_DIR / "models" / "ur5_rg2" / "model.sdf").is_file()


# ---------------------------------------------------------------------------
# Scripts canónicos
# ---------------------------------------------------------------------------


def test_lanzar_panelv2_script_exists():
    """Script de arranque del panel debe existir y ser ejecutable."""
    script = REPO_ROOT / "lanzar_panelv2.sh"
    assert script.is_file(), f"falta script canónico: {script}"
    # Verificar shebang básico
    text = _read(script)
    assert text.startswith("#!"), "lanzar_panelv2.sh debe tener shebang"


def test_lanzar_panelc2_script_exists():
    """Script smart launcher canónico."""
    script = REPO_ROOT / "lanzar_panelc2.sh"
    assert script.is_file()
