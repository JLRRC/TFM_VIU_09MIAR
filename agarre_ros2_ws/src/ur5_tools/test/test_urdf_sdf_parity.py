# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_urdf_sdf_parity.py
# Contenido: T22 — paridad geometrica URDF (xacro) <-> SDF de Gazebo.
# Uso breve: pytest src/ur5_tools/test/test_urdf_sdf_parity.py
"""T22 — Paridad URDF <-> SDF para los joints fijos de la cadena RG2.

Bug raiz cubierto (2026-05-06):
    El SDF tenia ``end_effector_frame_fixed_joint`` con traslacion 0.275 m en Y
    desde wrist_3_link y compensaba con ``rg2_mount_joint`` a -0.1927 m en Z.
    Eso desplazaba el frame ``tool0`` 192.7 mm respecto a su posicion estandar
    ROS-Industrial (= origen de wrist_3_link), y desincronizaba los TF
    publicados por el RSP (URDF) con la posicion fisica de los dedos en Gazebo
    (SDF). Resultado: el panel "agarraba" geometricamente pero los dedos no
    tocaban el objeto.

Este test parsea ambos modelos y comprueba que los offsets canonicos del RG2
permanecen alineados.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, Tuple

ROOT = Path(__file__).resolve().parents[3]
URDF_PATH = ROOT / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro"
SDF_PATH = ROOT / "models" / "ur5_rg2" / "model.sdf"

PARITY_TOL_M = 1e-6


def _parse_xacro_properties(text: str) -> Dict[str, str]:
    """Resuelve ``<xacro:property name="..." value="..."/>`` simples del URDF."""
    props: Dict[str, str] = {}
    for match in re.finditer(
        r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"\s*/>',
        text,
    ):
        props[match.group(1)] = match.group(2)
    return props


def _resolve_xacro_xyz(raw: str, properties: Dict[str, str]) -> str:
    """Sustituye ``${name}`` por el valor de la property correspondiente."""
    def replace(m: "re.Match[str]") -> str:
        return properties.get(m.group(1), m.group(0))

    return re.sub(r"\$\{([^}]+)\}", replace, raw)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _parse_sdf_pose(text: str, joint_name: str) -> Tuple[float, float, float]:
    """Devuelve la traslacion XYZ del primer ``<pose>`` dentro del joint dado."""
    match = re.search(
        rf'<joint name="{re.escape(joint_name)}"[^>]*>(.*?)</joint>',
        text,
        flags=re.DOTALL,
    )
    assert match, f"joint {joint_name!r} no encontrado en SDF"
    body = match.group(1)
    pose = re.search(r"<pose[^>]*>([^<]+)</pose>", body)
    assert pose, f"<pose> no encontrado dentro de {joint_name!r}"
    tokens = [tok for tok in pose.group(1).strip().split() if tok]
    assert len(tokens) >= 3, f"pose de {joint_name!r} tiene <3 valores: {pose.group(1)!r}"
    return (float(tokens[0]), float(tokens[1]), float(tokens[2]))


def _parse_urdf_origin_xyz(text: str, joint_name: str) -> Tuple[float, float, float]:
    """Devuelve la traslacion XYZ de ``<origin xyz="..."/>``, resolviendo xacro properties."""
    match = re.search(
        rf'<joint name="{re.escape(joint_name)}"[^>]*>(.*?)</joint>',
        text,
        flags=re.DOTALL,
    )
    assert match, f"joint {joint_name!r} no encontrado en URDF"
    body = match.group(1)
    origin = re.search(r'<origin\s+xyz="([^"]+)"', body)
    assert origin, f"<origin xyz=...> no encontrado dentro de {joint_name!r}"
    raw = origin.group(1).strip()
    properties = _parse_xacro_properties(text)
    resolved = _resolve_xacro_xyz(raw, properties)
    tokens = [tok for tok in resolved.strip().split() if tok]
    assert len(tokens) >= 3, (
        f"origin de {joint_name!r} tiene <3 valores tras resolver xacro: raw={raw!r}"
    )
    return (float(tokens[0]), float(tokens[1]), float(tokens[2]))


def _max_abs_delta(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))


def test_rg2_mount_joint_translation_is_zero_in_both_models() -> None:
    """rg2_mount_joint debe estar en tool0 (origen 0,0,0) en URDF y SDF."""
    urdf_xyz = _parse_urdf_origin_xyz(_read(URDF_PATH), "rg2_mount_joint")
    sdf_xyz = _parse_sdf_pose(_read(SDF_PATH), "rg2_mount_joint")
    assert _max_abs_delta(urdf_xyz, (0.0, 0.0, 0.0)) <= PARITY_TOL_M, (
        f"URDF rg2_mount_joint origin debe ser (0,0,0); actual={urdf_xyz}"
    )
    assert _max_abs_delta(sdf_xyz, (0.0, 0.0, 0.0)) <= PARITY_TOL_M, (
        f"SDF rg2_mount_joint pose debe ser (0,0,0); actual={sdf_xyz}. "
        "Si has reintroducido el offset -0.1927m, revisa el bug URDF<->SDF de "
        "2026-05-06: estaba compensando un end_effector_frame_fixed_joint "
        "incorrecto, ya corregido."
    )


def test_end_effector_frame_fixed_joint_translation_is_zero_in_sdf() -> None:
    """tool0 SDF debe estar en el origen de wrist_3_link (estandar ROS-Industrial UR)."""
    sdf_xyz = _parse_sdf_pose(_read(SDF_PATH), "end_effector_frame_fixed_joint")
    assert _max_abs_delta(sdf_xyz, (0.0, 0.0, 0.0)) <= PARITY_TOL_M, (
        f"SDF end_effector_frame_fixed_joint translation debe ser (0,0,0); "
        f"actual={sdf_xyz}. El estandar UR ROS-Industrial pone tool0 en el "
        "origen del wrist_3_link, solo rotado. Si has reintroducido un offset, "
        "revisa el bug 2026-05-06."
    )


def test_pinch_center_offset_matches_between_urdf_and_anchor_sdf() -> None:
    """rg2_pinch_center_joint URDF debe coincidir con pick_demo_anchor_joint SDF."""
    urdf_xyz = _parse_urdf_origin_xyz(_read(URDF_PATH), "rg2_pinch_center_joint")
    sdf_xyz = _parse_sdf_pose(_read(SDF_PATH), "pick_demo_anchor_joint")
    delta = _max_abs_delta(urdf_xyz, sdf_xyz)
    assert delta <= PARITY_TOL_M, (
        f"URDF rg2_pinch_center vs SDF pick_demo_anchor diverge {delta:.6f} m. "
        f"URDF={urdf_xyz} SDF={sdf_xyz}. "
        "Estos dos joints deben describir el mismo punto fisico (centro de los "
        "dedos del RG2). Si divergen, el attach logico no coincidira con el "
        "punto donde los dedos cierran."
    )


def test_finger_joint_offsets_match_between_urdf_and_sdf() -> None:
    """rg2_finger_joint1/2 deben tener identico offset en URDF y SDF."""
    urdf_text = _read(URDF_PATH)
    sdf_text = _read(SDF_PATH)
    for joint, expected in (
        ("rg2_finger_joint1", (0.0, 0.020, 0.0)),
        ("rg2_finger_joint2", (0.0, -0.020, 0.0)),
    ):
        urdf_xyz = _parse_urdf_origin_xyz(urdf_text, joint)
        sdf_xyz = _parse_sdf_pose(sdf_text, joint)
        assert _max_abs_delta(urdf_xyz, expected) <= PARITY_TOL_M, (
            f"URDF {joint} origin debe ser {expected}; actual={urdf_xyz}"
        )
        assert _max_abs_delta(sdf_xyz, expected) <= PARITY_TOL_M, (
            f"SDF {joint} pose debe ser {expected}; actual={sdf_xyz}"
        )


def test_runtime_gz_models_copy_does_not_reintroduce_bug() -> None:
    """Si existe la copia runtime, no debe haber reintroducido los offsets bug."""
    runtime_sdf = ROOT / "log" / "gz_models" / "ur5_rg2" / "model.sdf"
    if not runtime_sdf.is_file():
        # La copia se genera al lanzar el panel; ausente en CI offline.
        return
    text = _read(runtime_sdf)
    mount_xyz = _parse_sdf_pose(text, "rg2_mount_joint")
    eef_xyz = _parse_sdf_pose(text, "end_effector_frame_fixed_joint")
    assert _max_abs_delta(mount_xyz, (0.0, 0.0, 0.0)) <= PARITY_TOL_M, (
        f"Runtime SDF rg2_mount_joint regreso a offset bug: {mount_xyz}. "
        "Borra log/gz_models/ y relanza el panel para regenerar desde fuente."
    )
    assert _max_abs_delta(eef_xyz, (0.0, 0.0, 0.0)) <= PARITY_TOL_M, (
        f"Runtime SDF end_effector_frame_fixed_joint regreso a offset bug: {eef_xyz}. "
        "Borra log/gz_models/ y relanza el panel para regenerar desde fuente."
    )
