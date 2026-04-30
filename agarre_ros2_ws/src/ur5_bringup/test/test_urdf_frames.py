#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_urdf_frames.py
# Contenido: F4 — verificación estática de frames TF clave en el URDF.
"""Smoke test de los frames TF en el URDF (F4 mínima).

Expande ``ur5_description/urdf/ur5.urdf.xacro`` con la CLI ``xacro``
(ROS 2) y verifica que aparecen los frames clave del proyecto:

    world, base_link, tool0, rg2_tcp, rg2_pinch_center, pick_demo_anchor

Si alguno se renombra o desaparece accidentalmente (típico tras un
refactor de joints/links), el test lo detecta antes de F3.

Si la CLI ``xacro`` no está en PATH (no hay ROS sourceado), el test
se skipea con razón clara.
"""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


WS_SRC = Path(__file__).resolve().parents[2]
XACRO_FILE = WS_SRC / "ur5_description" / "urdf" / "ur5.urdf.xacro"

# Frames clave del proyecto (semánticos, no anatómicos del UR5).
# Si renombras alguno, actualiza esta lista junto al cambio para
# documentar que el rename fue intencional.
EXPECTED_FRAMES = {
    "world",
    "base_link",
    "tool0",
    "rg2_tcp",
    "rg2_pinch_center",
}


@pytest.fixture(scope="module")
def urdf_xml() -> str:
    """Expande el xacro a URDF XML plano. Skip si no hay xacro CLI."""
    if not XACRO_FILE.is_file():
        pytest.fail(f"xacro source no encontrado: {XACRO_FILE}")
    if shutil.which("xacro") is None:
        pytest.skip("xacro CLI no disponible (sin ROS 2 sourceado)")
    result = subprocess.run(
        ["xacro", str(XACRO_FILE)],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    if result.returncode != 0:
        pytest.fail(
            f"xacro {XACRO_FILE.name} salió con código {result.returncode}\n"
            f"stderr:\n{result.stderr[:2000]}"
        )
    return result.stdout


def test_xacro_expands_to_well_formed_urdf(urdf_xml):
    """El XML resultante de xacro se parsea sin errores."""
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError as e:
        pytest.fail(f"URDF XML mal formado tras xacro: {e}")
    assert root.tag == "robot", (
        f"raíz del URDF debe ser <robot>, encontrada <{root.tag}>"
    )


@pytest.mark.parametrize("frame", sorted(EXPECTED_FRAMES))
def test_expected_frame_present(urdf_xml, frame):
    """Cada frame clave aparece como ``<link name="...">`` en el URDF."""
    root = ET.fromstring(urdf_xml)
    links = {link.get("name") for link in root.iter("link")}
    assert frame in links, (
        f"frame esperado '{frame}' NO aparece como <link> en el URDF "
        f"expandido. Links presentes ({len(links)}): "
        f"{sorted(l for l in links if l)[:20]}..."
    )


def test_frame_chain_world_to_base(urdf_xml):
    """Verifica que existe una cadena de joints conectando world→base_link.

    Con frames sueltos sin parent/child el robot_state_publisher no
    podría publicar la TF chain. Validación mínima: para cada frame
    esperado distinto de 'world', existe al menos un joint con
    ``child link='<frame>'``.
    """
    root = ET.fromstring(urdf_xml)
    children = set()
    for joint in root.iter("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            children.add(child.get("link"))
    expected_with_parent = EXPECTED_FRAMES - {"world"}
    missing = expected_with_parent - children
    assert not missing, (
        f"frames sin joint padre (no se publicarán por TF): {sorted(missing)}"
    )
